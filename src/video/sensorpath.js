/* ============================================================
   SENSOR ATTITUDE PATH — device orientation recorded alongside a clip.

   A phone reports attitude from GRAVITY (pitch/roll, absolute and
   drift-free) plus the compass (azimuth, smooth but biased — field-measured
   14–66° wrong near metal). The visual stabilizer is the opposite: precise
   and absolutely anchored where it can see structure, but an incremental
   chain that DRIFTS (measured ~1.4° over a 22 s clip) and freezes outright
   on frames with nothing to track.

   So they are complementary, and this module keeps each honest about what
   it is good for:
     · the sensor log supplies MOTION (deltas) — never absolute pointing,
       because the compass bias would poison a calibrated placement;
     · the visual solve supplies the ABSOLUTE frame and the precision.

   Pure functions, no DOM — asserted in scripts/mathcheck.js.
   ============================================================ */

const angD = (a, b) => ((a - b + 540) % 360) - 180;
const norm360 = (a) => ((a % 360) + 360) % 360;

/* interpolate an attitude log at time t (az wrap-aware). `log` is
   [{t, az, el, roll}] sorted by t. */
export function sensorAt(log, t) {
  if (!Array.isArray(log) || !log.length) return null;
  if (t <= log[0].t) return log[0];
  if (t >= log[log.length - 1].t) return log[log.length - 1];
  let lo = 0, hi = log.length - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (log[m].t <= t) lo = m; else hi = m; }
  const a = log[lo], b = log[hi], u = (t - a.t) / Math.max(1e-9, b.t - a.t);
  return {
    t,
    az: norm360(a.az + angD(b.az, a.az) * u),
    el: a.el + (b.el - a.el) * u,
    roll: (a.roll || 0) + ((b.roll || 0) - (a.roll || 0)) * u,
  };
}

/* CLOCK SYNC. The attitude log runs on performance.now(); the video runs on
   its own media timeline, and the gap between "start recording" and the first
   encoded frame is neither zero nor constant across devices. Recover the
   offset the same way two witnesses' clips are synced: search the shift that
   minimises disagreement — but compare only the SHAPE of the motion, with each
   axis's mean removed, so a compass bias (or a hand-placed photo alignment)
   can't drag the answer. Returns {offset, rms, conf}; conf is low when the
   motion is too gentle to localise a minimum.
   CONVENTION: `offset` is what you ADD to a VIDEO time to index the log —
   sensorAt(log, videoT + offset). */
export function syncSensor(log, visual, opts = {}) {
  if (!Array.isArray(log) || log.length < 4 || !Array.isArray(visual) || visual.length < 4) return null;
  const range = opts.range == null ? 2.0 : opts.range;      // ± seconds to search
  const step = opts.step == null ? 0.02 : opts.step;
  const score = (off) => {
    const dAz = [], dEl = [], dRoll = [];
    for (const v of visual) {
      const s = sensorAt(log, v.t + off);
      if (!s) continue;
      dAz.push(angD(s.az, v.az)); dEl.push(s.el - v.el); dRoll.push((s.roll || 0) - (v.roll || 0));
    }
    if (dAz.length < 4) return null;
    const dev = (a) => { const m = a.reduce((x, y) => x + y, 0) / a.length; return Math.sqrt(a.reduce((x, y) => x + (y - m) * (y - m), 0) / a.length); };
    /* bias-free disagreement: only the VARIATION counts */
    return Math.sqrt(dev(dAz) ** 2 + dev(dEl) ** 2 + dev(dRoll) ** 2);
  };
  let best = null, worst = 0, n = 0, sum = 0;
  for (let off = -range; off <= range + 1e-9; off += step) {
    const s = score(off);
    if (s == null) continue;
    n++; sum += s; worst = Math.max(worst, s);
    if (!best || s < best.rms) best = { offset: +off.toFixed(3), rms: s };
  }
  if (!best || !n) return null;
  /* a real lock is a deep minimum; a slow pan matches almost everywhere */
  const mean = sum / n;
  const conf = mean > 1e-6 ? Math.max(0, Math.min(1, 1 - best.rms / mean)) : 0;
  return { offset: best.offset, rms: +best.rms.toFixed(4), conf: +conf.toFixed(3) };
}

/* FUSION. Walk the visual path in time. A frame the vision SOLVED well is
   taken as-is and becomes an anchor. A weak or held frame is carried forward
   from the last anchor by the sensor's own DELTA over that span — real motion
   instead of a frozen repeat. When vision recovers, the sensor-carried run is
   smeared linearly onto the recovered pose so the two meet in the middle
   rather than snapping (same treatment the tracker's own re-anchors get).

   opts.minN     — inliers at or above which a visual sample is trusted (6)
   opts.maxCarry — seconds of sensor-only carry before it is left frozen and
                   flagged, since attitude alone can't be trusted forever (2.5)
   Returns a NEW path; each entry gains `src`: "v" solved, "s" sensor-carried,
   "b" sensor-carried then smeared onto a recovered solve, "h" left held. */
export function fuseSensorVisual(visual, log, opts = {}) {
  if (!Array.isArray(visual) || !visual.length) return visual;
  if (!Array.isArray(log) || log.length < 2) return visual.map((p) => ({ ...p, src: "v" }));
  const minN = opts.minN == null ? 6 : opts.minN;
  const maxCarry = opts.maxCarry == null ? 2.5 : opts.maxCarry;
  const off = opts.offset || 0;
  /* the stabilize walk stores the held flag as `h` (1/0); accept both */
  const strong = (p) => !(p.held || p.h) && (p.n == null || p.n >= minN);

  const out = visual.map((p) => ({ ...p }));
  let anchor = null;                       // last trusted visual pose
  let run = [];                            // indices carried since that anchor
  const carry = (i, from) => {
    const s0 = sensorAt(log, from.t + off), s1 = sensorAt(log, out[i].t + off);
    if (!s0 || !s1) return false;
    out[i].az = norm360(from.az + angD(s1.az, s0.az));
    out[i].el = from.el + (s1.el - s0.el);
    out[i].roll = (from.roll || 0) + ((s1.roll || 0) - (s0.roll || 0));
    return true;
  };
  for (let i = 0; i < out.length; i++) {
    const p = out[i];
    if (strong(p)) {
      if (run.length && anchor) {
        /* the carried run ran from `anchor` to this recovered solve: smear the
           end-to-end disagreement back across it, linear in time */
        const pre = { az: p.az, el: p.el, roll: p.roll || 0 };
        if (carry(i, anchor)) {
          const eAz = angD(pre.az, out[i].az), eEl = pre.el - out[i].el, eRoll = (pre.roll || 0) - (out[i].roll || 0);
          const t0 = anchor.t, t1 = p.t, span = Math.max(1e-9, t1 - t0);
          for (const k of run) {
            const u = (out[k].t - t0) / span;
            out[k].az = norm360(out[k].az + eAz * u);
            out[k].el += eEl * u;
            out[k].roll = (out[k].roll || 0) + eRoll * u;
            out[k].src = "b";
          }
        }
        out[i].az = pre.az; out[i].el = pre.el; out[i].roll = pre.roll;   // the solve itself is untouched
      }
      p.src = "v";
      anchor = { t: p.t, az: p.az, el: p.el, roll: p.roll || 0 };
      run = [];
      continue;
    }
    /* weak frame: carry it on the sensor, unless we've been carrying too long */
    if (anchor && p.t - anchor.t <= maxCarry && carry(i, anchor)) { p.src = "s"; run.push(i); }
    else { p.src = "h"; }
  }
  return out;
}


/* MOTION DISAGREEMENT. Inlier count is NOT a truth signal: a tracker that
   loses the scene and re-acquires on whatever drifted into frame reports a
   confident, high-inlier solve that barely moves. Field case: the phone swept
   95° of azimuth (gravity confirms 27° of elevation with it) while the visual
   solve reported 11.5° and 34–46 inliers throughout — so every frame passed as
   "strong" and the sensors were never consulted.

   Gravity cannot be wrong about that, so compare the PATH LENGTH each source
   travelled. Returns {vis, sen, ratio} in degrees; a ratio far below 1 means
   the vision missed real motion. */
export function motionDisagreement(visual, log, offset = 0) {
  if (!Array.isArray(visual) || visual.length < 3 || !Array.isArray(log) || log.length < 3) return null;
  let vis = 0, sen = 0;
  for (let i = 1; i < visual.length; i++) {
    const a = visual[i - 1], b = visual[i];
    vis += Math.hypot(angD(b.az, a.az) * Math.cos(((a.el + b.el) / 2) * Math.PI / 180), b.el - a.el);
    const s0 = sensorAt(log, a.t + offset), s1 = sensorAt(log, b.t + offset);
    if (s0 && s1) sen += Math.hypot(angD(s1.az, s0.az) * Math.cos(((s0.el + s1.el) / 2) * Math.PI / 180), s1.el - s0.el);
  }
  return { vis: +vis.toFixed(2), sen: +sen.toFixed(2), ratio: sen > 1e-6 ? +(vis / sen).toFixed(3) : 1 };
}

/* SENSOR-ONLY PATH. When the vision demonstrably missed the motion, the log is
   the better witness — but ONLY for motion. The absolute frame still comes from
   the user's alignment: the path is built so that at `anchor.t` it equals the
   placement pose exactly, and every other frame is that pose plus the sensor's
   own delta. So the compass bias cancels out entirely (it is common to both
   ends of the difference) and a star/terrain-calibrated placement is preserved.

   FOV is carried from the anchor unchanged: this mode exists for in-app
   recordings, which have no optical zoom, so a constant field of view is not an
   assumption — it is the truth about the capture. */
export function sensorOnlyPath(log, times, anchor, opts = {}) {
  if (!Array.isArray(log) || log.length < 2 || !Array.isArray(times) || !times.length || !anchor) return null;
  const off = opts.offset || 0;
  const s0 = sensorAt(log, anchor.t + off);
  if (!s0) return null;
  return times.map((t) => {
    const s = sensorAt(log, t + off) || s0;
    return {
      t: +t.toFixed(3),
      az: +norm360(anchor.az + angD(s.az, s0.az)).toFixed(3),
      el: +(anchor.el + (s.el - s0.el)).toFixed(3),
      roll: +((anchor.roll || 0) + ((s.roll || 0) - (s0.roll || 0))).toFixed(3),
      fov: anchor.fov, k: anchor.k || 0,
      n: 0, src: "s",
    };
  });
}

/* how much of a fused path came from where — for honest UI/report wording */
export function fuseStats(path) {
  const c = { v: 0, s: 0, b: 0, h: 0 };
  for (const p of path || []) c[p.src || "v"] = (c[p.src || "v"] || 0) + 1;
  return c;
}
