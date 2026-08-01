/* ============================================================
   TRAJECTORY ANALYSIS
   Track points (pixels on video frames, or direct az/el) become
   time-stamped sight directions. Two observers → interpolate to
   common times, triangulate each sample → 3D path → velocity,
   acceleration, felt g-load, turn rate. One observer → angular
   kinematics that scale with an assumed distance.
   ============================================================ */

import { D2R, R2D, clampN, dot, sub, add, scl, mag, unit, enuFromGeo, dirFromAzEl, dirFromAzElAt, dirToAzEl } from "./geodesy.js";
import { isNum } from "./format.js";
import { pixelDirFromAnchor, angSizeFromPoints, lensK } from "./projection.js";
import { intersectLines } from "./triangulate.js";

/* ─── MULTI-MOMENT TRACK ────────────────────────────────────────────────
   One observer can hold several timestamped photos ("moments") of the same
   object — the primary shot plus `s.moments[]`. Each PLACED shot yields one
   sight direction (its A.az/A.el) at its capture time (whenMs). Two or more
   placed shots therefore describe an angular trajectory exactly like manually
   drawn track points do. `sourceTrack` returns that unified point list:
     • ≥2 placed shots  → one {t,az,el,ang} point per shot, t in seconds from
       the earliest shot, sorted by time (the multi-photo path);
     • otherwise        → the source's own manual `track` (the single-photo
       hand-drawn path — unchanged, so existing sightings behave identically).
   Pure + deterministic so the whole trajectory pipeline stays testable. */
export function sourceTrack(s) {
  if (!s) return [];
  const placedShot = (m) => isNum(m?.A?.az) && isNum(m?.A?.el) && isNum(m?.whenMs);
  const momentPt = (m, t0) => {
    const ang = angSizeFromPoints(m.A?.p1, m.A?.p2, m.natW, m.natH, +m.fovH, lensK(m));
    const pt = { t: (+m.whenMs - t0) / 1000, az: +m.A.az, el: +m.A.el };
    if (isNum(ang) && +ang > 0) pt.ang = +ang;
    else if (isNum(m.A?.angManual) && +m.A.angManual > 0) pt.ang = +m.A.angManual;
    return pt;
  };
  const manual = s.track || [];
  const extras = (s.moments || []).filter(placedShot);        // moments beyond the primary
  const shots = [s, ...extras].filter(placedShot);            // primary + placed moments

  // Pure multi-photo path: ≥2 placed shots and no hand-drawn track — each shot
  // contributes one direction at its capture time.
  if (shots.length >= 2 && manual.length === 0) {
    const t0 = Math.min(...shots.map((m) => +m.whenMs));
    return shots.map((m) => momentPt(m, t0)).sort((a, b) => a.t - b.t);
  }

  // Hybrid path: a hand-drawn track AND ≥1 extra placed moment. The drawn points
  // live in the primary photo's frame, anchored at the primary's capture time
  // (their p.t is seconds from that first mark); the extra moments drop onto the
  // same absolute timeline at their own times. Sorted so points before/between/
  // after the photos interleave correctly. `s.whenMs` is the primary's clock.
  if (manual.length && extras.length && isNum(s.whenMs)) {
    const t0 = Math.min(+s.whenMs, ...extras.map((m) => +m.whenMs));
    const base = (+s.whenMs - t0) / 1000;
    const drawn = manual.map((p) => ({ ...p, t: base + (+p.t || 0) }));
    const mom = extras.map((m) => momentPt(m, t0));
    return [...drawn, ...mom].sort((a, b) => a.t - b.t);
  }

  // Single photo (drawn track, or nothing) — unchanged from the original model.
  return manual;
}

/* ─── VISIBILITY SEGMENTS ───────────────────────────────────────────────
   A witness only measured the object where they could SEE it — a real track
   has holes (behind trees, too small, out of frame). Interpolating a
   direction across a hole fabricates a path nobody observed, and a
   triangulation that consumes it is silently wrong wherever the other
   witness's real data falls inside the hole (field case: a drone visible
   only in sections of each of two videos). So: adjacent samples further
   apart than gapS form a BREAK; interpolation is legal only inside a
   segment, and stereo triangulation only at instants inside EVERY witness's
   segments. gapS adapts to the track's own cadence (waypoints ~0.5 s, dense
   tracker paths ~0.25 s), floored so a deliberately sparse but continuous
   track isn't shredded. */
export function trackSegments(ts, opts = {}) {
  if (!ts || !ts.length) return [];
  const dts = [];
  for (let i = 1; i < ts.length; i++) dts.push(ts[i] - ts[i - 1]);
  const med = dts.slice().sort((a, b) => a - b)[dts.length >> 1] || 0;
  const gapS = opts.gapS != null ? +opts.gapS : Math.max(1.6, med * 4);
  const segs = [];
  let s0 = ts[0];
  for (let i = 1; i < ts.length; i++) {
    if (ts[i] - ts[i - 1] > gapS) { segs.push([s0, ts[i - 1]]); s0 = ts[i]; }
  }
  segs.push([s0, ts[ts.length - 1]]);
  return segs;
}
export const inSegments = (segs, t) => segs.some(([a, b]) => t >= a - 1e-9 && t <= b + 1e-9);
/* intersection of two segment lists — the span BOTH witnesses actually saw */
export function interSegments(A, B) {
  const out = [];
  for (const a of A) for (const b of B) {
    const lo = Math.max(a[0], b[0]), hi = Math.min(a[1], b[1]);
    if (hi > lo) out.push([lo, hi]);
  }
  return out.sort((x, y) => x[0] - y[0]);
}
export const segsDur = (segs) => segs.reduce((s, [a, b]) => s + (b - a), 0);

/* spherical linear interpolation between unit vectors */
function slerp(a, b, f) {
  const c = clampN(dot(a, b), -1, 1);
  const th = Math.acos(c);
  if (th < 1e-6) return a;
  const sA = Math.sin((1 - f) * th) / Math.sin(th), sB = Math.sin(f * th) / Math.sin(th);
  return [a[0] * sA + b[0] * sB, a[1] * sA + b[1] * sB, a[2] * sA + b[2] * sB];
}

/* Corner rounding: a real object flies an arc, not a vertex — piecewise-
   linear witness paths imply infinite instantaneous turn. Each interior
   point may carry r (0..0.49): the fraction of each adjacent segment (in
   time and angle) consumed by a quadratic arc through the corner, sampled
   as `virt` points on the sphere. r=0 (or absent) keeps the hard corner —
   which is then a deliberate claim, not a drawing artifact. */
function roundCorners(pts) {
  if (pts.length < 3 || !pts.some((p, i) => i > 0 && i < pts.length - 1 && p.r > 0)) return pts;
  const out = [pts[0]];
  for (let i = 1; i < pts.length - 1; i++) {
    const f = clampN(+pts[i].r || 0, 0, 0.49);
    if (!(f > 0)) { out.push(pts[i]); continue; }
    const prev = pts[i - 1], v = pts[i], next = pts[i + 1];
    const dtP = v.ct - prev.ct, dtN = next.ct - v.ct;
    if (dtP <= 0 || dtN <= 0) { out.push(pts[i]); continue; }
    const P0 = slerp(v.d, prev.d, f), P1 = slerp(v.d, next.d, f);
    const te = v.ct - f * dtP, tx = v.ct + f * dtN;
    const uin = unit(sub(v.d, P0)), wout = unit(sub(P1, v.d));
    let mAvg = add(uin, wout); const mm = mag(mAvg);
    if (v.anchor && mm > 1e-6) {
      /* the ANCHOR is a MEASURED direction (snapped to the placed object): the
         arc must pass exactly THROUGH it AND stay smooth. A plain drawn vertex
         cuts inside (below) — but cutting the anchor would leave the object off
         its measured spot, and the earlier reflected-control quad passed through
         it but KINKED at the leg joins. So build TWO cubic-Hermite halves,
         P0→v and v→P1, sharing an averaged tangent at v: each is tangent to its
         leg at P0/P1 (smooth join to the straight legs) and meets its sibling at
         v with the same tangent (smooth through the object). */
      mAvg = scl(mAvg, 1 / mm);
      const l1 = mag(sub(v.d, P0)) / 3, l2 = mag(sub(P1, v.d)) / 3;
      const c1 = add(P0, scl(uin, l1)), c2 = sub(v.d, scl(mAvg, l1));
      const c3 = add(v.d, scl(mAvg, l2)), c4 = sub(P1, scl(wout, l2));
      const cub = (a, b, c, e, t) => { const s = 1 - t; return add(add(scl(a, s * s * s), scl(b, 3 * s * s * t)), add(scl(c, 3 * s * t * t), scl(e, t * t * t))); };
      /* sample finely: the curvature is highest right at v (the arc bulges out to
         reach it), so a coarse step there reads as a false kink in the sampled
         path even though the curve is C1-smooth (12 → the dome path is smoother
         than the plain quadratic corner). */
      const M = 12;
      for (let half = 0; half < 2; half++) {
        const t0 = half === 0 ? te : v.ct, t1 = half === 0 ? v.ct : tx;
        for (let k = half === 0 ? 0 : 1; k <= M; k++) {
          const t = k / M;
          const d = unit(half === 0 ? cub(P0, c1, c2, v.d, t) : cub(v.d, c3, c4, P1, t));
          const ae = dirToAzEl(d);
          out.push({ ct: t0 + (t1 - t0) * t, d, az: ae.az, el: ae.el, virt: (half === 0 && k === 0) ? undefined : true });
        }
      }
      out[out.length - 1 - 2 * M].virt = undefined; out[out.length - 1 - 2 * M].orig = i;
    } else {
      /* plain drawn vertex: a GUESS where two legs met — the object flies INSIDE
         it, so the vertex is the quadratic control and the curve cuts the corner. */
      const N = 6;
      for (let k = 0; k <= N; k++) {
        const t = k / N;
        const d = unit(slerp(slerp(P0, v.d, t), slerp(v.d, P1, t), t));
        const ae = dirToAzEl(d);
        out.push({ ct: te + (tx - te) * t, d, az: ae.az, el: ae.el, virt: k !== 0 || undefined });
      }
      /* mark the arc's start as the display anchor for point i */
      out[out.length - 1 - N].virt = undefined; out[out.length - 1 - N].orig = i;
    }
  }
  out.push(pts[pts.length - 1]);
  return out;
}

/* Convert a source's track into [{ct, d}] — clock time + unit direction.
   Pixel points are anchored to Moment A: the FIRST pixel track point is
   defined to lie along A's az/el; later points are pixel offsets from it.

   POSE NOTE (video roadmap): `s.mediaAim` is read here as the pose for
   every point, which assumes the camera never moved. When per-frame poses
   land, a point's own `p.pose` should win over the source-level one. */
export function trackDirections(s) {
  const track = sourceTrack(s);
  if (!track || track.length < 2) return null;
  const pts = [...track].sort((a, b) => a.t - b.t);
  const fov = isNum(s.fovH) ? +s.fovH : null;
  /* the ANCHOR pixel point sits at A.az/el; every other pixel point is an offset
     from it. Normally that's the first tapped point, but a point flagged
     `anchor` (SNAPPED to the measured 3D object — which may be anywhere ALONG
     the path, not just the start) wins, so the recalled path is pinned to the
     object's real measured direction at the right point. */
  const p0 = pts.find((p) => p.x != null && p.anchor) || pts.find((p) => p.x != null);
  const anchored = isNum(s.A?.az) && isNum(s.A?.el);
  const useAim = s.mediaAim && isNum(s.mediaAim.az) && s.natW && fov;
  const out = [];
  for (const p of pts) {
    let d;
    if (p.az != null && p.el != null) d = dirFromAzEl(+p.az, +p.el);
    else if (s.natW && fov && p0 && (useAim || anchored)) {
      d = pixelDirFromAnchor(p.x, p.y, p0.x, p0.y,
        anchored ? +s.A.az : 0, anchored ? +s.A.el : 0,
        s.natW, s.natH, fov, useAim ? s.mediaAim : null);
    } else continue;
    const ct = (isNum(s.A?.t) && s.A?.videoTime != null) ? (+s.A.t + (p.t - s.A.videoTime)) : p.t;
    const ae = dirToAzEl(d);
    out.push({ ct, d, az: ae.az, el: ae.el, r: p.r, anchor: p.anchor });
  }
  return out.length >= 2 ? roundCorners(out) : null;
}

/* 3D kinematics from a time-stamped position series.
   Velocities per segment; 3-point smoothing before differentiating for
   acceleration (raw finite differences amplify point jitter badly).
   Felt load = |a − g⃗| / 9.81, so steady level motion reads 1 g. */
export function kinematics(times, pos) {
  const segs = [];
  for (let i = 0; i < pos.length - 1; i++) {
    const dt = times[i + 1] - times[i];
    if (dt <= 0) continue;
    segs.push({ t: (times[i] + times[i + 1]) / 2, v: scl(sub(pos[i + 1], pos[i]), 1 / dt) });
  }
  if (!segs.length) return null;
  const vs = segs.map((s, i) => (segs.length < 3 || i === 0 || i === segs.length - 1)
    ? s.v : scl(add(add(segs[i - 1].v, s.v), segs[i + 1].v), 1 / 3));
  const speeds = vs.map(mag);
  const GV = [0, 0, -9.81];
  const acc = [];
  for (let i = 0; i < vs.length - 1; i++) {
    const dt = segs[i + 1].t - segs[i].t;
    if (dt <= 0) continue;
    const a = scl(sub(vs[i + 1], vs[i]), 1 / dt);
    const load = mag(sub(a, GV)) / 9.81;
    const sp = Math.min(speeds[i], speeds[i + 1]);
    const turn = sp > 0.01 ? (Math.acos(clampN(dot(unit(vs[i]), unit(vs[i + 1])), -1, 1)) * R2D) / dt : 0;
    acc.push({ t: (segs[i].t + segs[i + 1].t) / 2, a: mag(a), load, turn });
  }
  let path = 0;
  for (let i = 0; i < pos.length - 1; i++) path += mag(sub(pos[i + 1], pos[i]));
  const dur = times[times.length - 1] - times[0];
  const peak = (arr, fn) => (arr.length ? arr.reduce((m, x) => Math.max(m, fn(x)), 0) : null);
  return {
    n: pos.length, dur, path,
    segs: segs.map((s, i) => ({ t: s.t, speed: speeds[i] })),
    acc,
    peakSpeed: Math.max(...speeds),
    avgSpeed: dur > 0 ? path / dur : 0,
    peakA: peak(acc, (x) => x.a),
    peakLoad: peak(acc, (x) => x.load),
    peakTurn: peak(acc, (x) => x.turn),
  };
}

/* Single-observer 3D path from per-point angular SIZE. One witness can't know
   absolute distance, but the RATIO of angular sizes fixes the relative range
   at each moment (range ∝ 1/tan(halfAngle)), which recovers radial (toward/
   away) motion — the piece a pure angular path misses. Positions are in units
   of the reference point's range (ρ_ref = 1); multiply by an assumed reference
   distance for absolute metres. Speeds/accelerations then include the radial
   component, not just the transverse one. */
export function soloTrack(s) {
  const raw = sourceTrack(s).filter((p) => isNum(p.az) && isNum(p.el)).sort((a, b) => a.t - b.t);
  if (raw.length < 3) return null;
  const ang = raw.map((p) => (isNum(p.ang) && +p.ang > 0 ? +p.ang : null));
  if (!ang.some((a) => a != null)) return null;               // no size info → no radial reconstruction
  /* per-point projection factor: how foreshortened the silhouette is at that
     orientation (1 = broadside, <1 = foreshortened). Precomputed in the UI so
     the math stays decoupled from the shape system. Dividing it out means a
     pure rotation is NOT misread as the object flying closer/farther. */
  const projF = raw.map((p) => (isNum(p.projF) && +p.projF > 0 ? +p.projF : 1));
  const iRef = ang.findIndex((a) => a != null);
  const tanRef = Math.tan((ang[iRef] * D2R) / 2), fRef = projF[iRef];
  /* range ∝ projF / tan(½·apparentSize) ; ρ normalized to the reference point */
  const rho = ang.map((a, i) => (a != null ? (projF[i] * tanRef) / (fRef * Math.tan((a * D2R) / 2)) : null));
  const times = raw.map((p) => +p.t);
  /* fill any point that was never sized: linear in time, nearest at the ends */
  for (let i = 0; i < rho.length; i++) {
    if (rho[i] != null) continue;
    let a = i - 1; while (a >= 0 && rho[a] == null) a--;
    let b = i + 1; while (b < rho.length && rho[b] == null) b++;
    if (a < 0 && b >= rho.length) rho[i] = 1;
    else if (a < 0) rho[i] = rho[b];
    else if (b >= rho.length) rho[i] = rho[a];
    else rho[i] = rho[a] + (rho[b] - rho[a]) * ((times[i] - times[a]) / ((times[b] - times[a]) || 1));
  }
  const pos = raw.map((p, i) => scl(dirFromAzEl(+p.az, +p.el), rho[i]));
  const k3d = kinematics(times, pos);
  if (!k3d) return null;
  let iNear = 0, iFar = 0;
  rho.forEach((r, i) => { if (r < rho[iNear]) iNear = i; if (r > rho[iFar]) iFar = i; });
  const oriented = projF.some((f) => Math.abs(f - 1) > 1e-3);   // any point's attitude recorded
  return { k3d, rho, times, pos, iRef, iNear, iFar, rangeRatio: rho[iFar] / rho[iNear], nAng: ang.filter((a) => a != null).length, oriented };
}

export function analyzeTracks(sources) {
  /* solo angular kinematics on the unit sphere (scale by distance later);
     `rad` adds the 3D reconstruction when the points carry angular sizes */
  const solo = sources.map((s) => {
    const dirs = trackDirections(s);
    if (!dirs || dirs.length < 3) return null;
    const k = kinematics(dirs.map((d) => d.ct), dirs.map((d) => d.d));
    return k ? { name: s.name, k, rad: soloTrack(s) } : null;
  }).filter(Boolean);

  /* stereo: interpolate each observer's direction to common sample times */
  const withT = sources
    .filter((s) => isNum(s.lat) && isNum(s.lon))
    .map((s) => ({ s, dirs: trackDirections(s) }))
    .filter((o) => o.dirs);
  let stereo = null;
  if (withT.length >= 2) {
    const ref = { lat: +withT[0].s.lat, lon: +withT[0].s.lon, alt: isNum(withT[0].s.alt) ? +withT[0].s.alt : 0 };
    /* ABSOLUTE CLOCKS: A.t anchors a track to a common clock when the user
       set one; otherwise, when EVERY witness carries a capture time and none
       used A.t, each track is anchored at whenMs + video t. Without this,
       two clips' tracks align at their recording STARTS — silently wrong by
       the difference in start times (field case: 39 s). */
    /* A.t is a deliberate common-clock anchor only when it DIFFERS from the
       source's own video reference: ct = A.t + (p.t − videoTime), so an A.t
       equal to videoTime (or 0 on a photo/plain track) is an identity
       placeholder that anchors nothing — field data carried both patterns
       ("0.00", and A.t == videoTime written by a wizard flow). */
    const anchored = withT.some((o) => {
      if (!isNum(o.s.A?.t)) return false;
      const vref = isNum(o.s.A?.videoTime) ? +o.s.A.videoTime : 0;
      return Math.abs(+o.s.A.t - vref) > 0.5;
    });
    const useWhen = !anchored && withT.every((o) => isNum(o.s.whenMs));
    let obs = withT.map((o) => {
      const dirs = useWhen ? o.dirs.map((d) => ({ ...d, ct: d.ct + (+o.s.whenMs / 1000) })) : o.dirs;
      return { ...o, dirs, P: enuFromGeo(+o.s.lat, +o.s.lon, isNum(o.s.alt) ? +o.s.alt : 0, ref) };
    });
    /* GEOMETRIC CLOCK SYNC (two witnesses): capture clocks lie — field case:
       one video's captured time was ~20 min wrong, and even hand-corrected
       to the minute it sat ~40 s off (proven against the drone's own flight
       log). The object's motion is the shared signal: search the relative
       offset that minimises mean ray-miss over shared-visibility instants,
       and ADOPT it only when the minimum is DECISIVE — a hovering object
       fits every offset equally, and a flat minimum must never invent a
       shift. A ±45 s window covers clock drift and start-vs-end stamps;
       when the tracks don't overlap AT ALL, a wide coarse RESCUE sweep
       (±30 min) hunts for where they geometrically meet — the 20-minute
       class of error. */
    let sync = null;
    if (obs.length === 2 && useWhen) {
      const T = obs.map((o) => o.dirs.map((d) => d.ct));
      const D = obs.map((o) => o.dirs.map((d) => d.d));
      const dirAtL = (ts, ds, t) => {
        if (t <= ts[0]) return ds[0];
        if (t >= ts[ts.length - 1]) return ds[ds.length - 1];
        let i = 0; while (i < ts.length - 2 && ts[i + 1] < t) i++;
        const a = ds[i], b = ds[i + 1], u = (t - ts[i]) / Math.max(1e-9, ts[i + 1] - ts[i]);
        return unit([a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u]);
      };
      const triAt = (delta, cap = 80) => {
        const T1 = T[1].map((x) => x + delta);
        const w0 = Math.max(T[0][0], T1[0]), w1 = Math.min(T[0][T[0].length - 1], T1[T1.length - 1]);
        if (!(w1 > w0)) return null;
        let sh = interSegments(trackSegments(T[0]), trackSegments(T1))
          .map(([a, b]) => [Math.max(a, w0), Math.min(b, w1)]).filter(([a, b]) => b > a);
        if (!sh.length) return null;
        let ts = T[0].filter((x) => inSegments(sh, x));
        if (ts.length > cap) { const st = Math.ceil(ts.length / cap); ts = ts.filter((_, i) => i % st === 0); }
        if (ts.length < 5) return null;
        let s = 0, n = 0;
        for (const t of ts) {
          const sol = intersectLines([{ P: obs[0].P, d: dirAtL(T[0], D[0], t) }, { P: obs[1].P, d: dirAtL(T1, D[1], t) }]);
          if (!sol || sol.ts.some((x) => x <= 0)) continue;
          s += sol.rmsMiss; n++;
        }
        return n >= 5 ? { miss: s / n, n } : null;
      };
      const scan = (lo, hi, step) => {
        let bst = null;
        for (let dl = lo; dl <= hi + 1e-9; dl += step) {
          const r = triAt(dl);
          if (r && (!bst || r.miss < bst.miss)) bst = { delta: dl, ...r };
        }
        return bst;
      };
      const base = triAt(0);
      let best = scan(-45, 45, 1);
      if (!best && !base) {
        const spanMin = Math.min(T[0][T[0].length - 1] - T[0][0], T[1][T[1].length - 1] - T[1][0]);
        best = scan(-1800, 1800, Math.max(2, spanMin / 6));
        if (best) best.rescued = true;
      }
      if (best) {
        const fine = scan(best.delta - 1.6, best.delta + 1.6, 0.1);
        if (fine && fine.miss < best.miss) best = { ...best, delta: fine.delta, miss: fine.miss, n: fine.n };
        const off = [triAt(best.delta - 3), triAt(best.delta + 3)].filter(Boolean);
        const rise = off.length ? Math.min(...off.map((r) => r.miss)) - best.miss : 0;
        const sharp = rise > Math.max(0.6, best.miss * 0.4);
        const beats = !base || best.miss < base.miss * 0.55;
        if (Math.abs(best.delta) > 1.2 && sharp && beats) {
          obs = obs.map((o, i) => (i === 0 ? o : { ...o, dirs: o.dirs.map((d) => ({ ...d, ct: d.ct + best.delta })) }));
          sync = { applied: true, delta: +best.delta.toFixed(2), rescued: !!best.rescued, rise: +rise.toFixed(2) };
        } else {
          sync = { applied: false, delta: +best.delta.toFixed(2), flat: !sharp };
        }
      } else if (!base) {
        sync = { applied: false, searchedWide: true };
      }
    }
    obs = obs.map((o) => ({ ...o, segs: trackSegments(o.dirs.map((d) => d.ct)) }));
    const t0 = Math.max(...obs.map((o) => o.dirs[0].ct));
    const t1 = Math.min(...obs.map((o) => o.dirs[o.dirs.length - 1].ct));
    /* the span every witness ACTUALLY saw — triangulating inside another
       witness's visibility hole consumes an interpolated (fabricated) ray */
    let shared = obs[0].segs;
    for (let i = 1; i < obs.length; i++) shared = interSegments(shared, obs[i].segs);
    shared = shared.map(([a, b]) => [Math.max(a, t0), Math.min(b, t1)]).filter(([a, b]) => b > a);
    const sharedDur = segsDur(shared);
    if (!(t1 > t0)) stereo = { overlapErr: true, sync };
    else if (!(sharedDur > 0)) stereo = { overlapErr: true, noShared: true, sync };
    else {
      let ts = [...new Set(obs.flatMap((o) => o.dirs.map((d) => +d.ct.toFixed(3))))]
        .filter((t) => t >= t0 && t <= t1 && inSegments(shared, t)).sort((a, b) => a - b);
      if (ts.length > 140) { const step = Math.ceil(ts.length / 140); ts = ts.filter((_, i) => i % step === 0); }
      const lerpDir = (dirs, t) => {
        let i = 0;
        while (i < dirs.length - 2 && dirs[i + 1].ct < t) i++;
        const a = dirs[i], b = dirs[Math.min(i + 1, dirs.length - 1)];
        const fr = b.ct > a.ct ? clampN((t - a.ct) / (b.ct - a.ct), 0, 1) : 0;
        return unit([a.d[0] + (b.d[0] - a.d[0]) * fr, a.d[1] + (b.d[1] - a.d[1]) * fr, a.d[2] + (b.d[2] - a.d[2]) * fr]);
      };
      const times = [], pos = []; let missSum = 0;
      for (const t of ts) {
        const sol = intersectLines(obs.map((o) => ({ P: o.P, d: lerpDir(o.dirs, t) })));
        if (!sol || sol.ts.some((x) => x <= 0)) continue;
        times.push(t); pos.push(sol.X); missSum += sol.rmsMiss;
      }
      if (pos.length >= 3) {
        const k = kinematics(times, pos);
        if (k) stereo = { k, times, pos, ref, avgMiss: missSum / pos.length, window: [t0, t1], nObs: obs.length, sharedDur, cutDur: Math.max(0, (t1 - t0) - sharedDur), sharedSegs: shared.length, sync };
      }
      if (!stereo) stereo = pos.length ? { sparse: true, sync } : { overlapErr: true, sync };
    }
  }
  return { stereo, solo };
}

/* TWO-VIDEO (or more) STEREO from dense object tracks. Each stabilized +
   object-tracked clip yields objPath = the object's WORLD direction every
   frame (camera motion already removed by stabilization), so each sample is
   directly a sight-line at a clock time — no per-frame pose conversion needed.
   Given ≥2 observers with positions we:
     1. put every clip on an ABSOLUTE clock (EXIF whenMs + video t, plus any
        per-source syncOffset the user nudged),
     2. AUTO-SYNC the clips — device clocks drift by seconds and EXIF video
        timestamps are coarse, so we search the relative time offset that
        MINIMISES the mean ray-miss of the per-instant intersections. The
        object's own motion is the shared signal that pins the true offset; a
        far/slow object makes the minimum shallow, which we report as low sync
        confidence rather than a false-precise number,
     3. triangulate each common instant with MEDIAN/MAD outlier rejection (a
        blurred frame, a momentary mistrack, or a compass glitch shows up as a
        fat ray-miss and is dropped),
     4. run kinematics on the resulting dense 3D path.
   Returns the fix, dense trajectory, recovered offset + confidence, and
   residuals for honest grading. Robust by construction to the small
   imperfections inherent in hand-held video. Pure. */
export function stereoVideo(sources, opts = {}) {
  /* q below minQ = the tracker HELD or rode the guide there — a prediction,
     not an observation (the object was invisible: blur, occlusion, out of
     frame). Dropping them turns invisible stretches into real GAPS, which
     the segment masking below then excludes from triangulation. A brief
     blur still bridges: the gap threshold tolerates short dropouts. */
  const minQ = opts.minQ != null ? +opts.minQ : 0.3;
  let qDropped = 0;
  const obs = (sources || [])
    .filter((s) => isNum(s.lat) && isNum(s.lon) && Array.isArray(s.objPath) && s.objPath.length >= 3)
    .map((s) => {
      const all = s.objPath.filter((p) => isNum(p.t) && isNum(p.az) && isNum(p.el));
      const samp = all.filter((p) => !(isNum(p.q) && +p.q < minQ)).sort((a, b) => a.t - b.t);
      qDropped += all.length - samp.length;
      return {
        s,
        base: (isNum(s.whenMs) ? +s.whenMs / 1000 : 0) + (isNum(s.syncOffset) ? +s.syncOffset : 0),
        samp,
      };
    })
    .filter((o) => o.samp.length >= 3);
  if (obs.length < 2) return null;
  const ref = { lat: +obs[0].s.lat, lon: +obs[0].s.lon, alt: isNum(obs[0].s.alt) ? +obs[0].s.alt : 0 };
  obs.forEach((o) => {
    o.P = enuFromGeo(+o.s.lat, +o.s.lon, isNum(o.s.alt) ? +o.s.alt : 0, ref);
    o.t = o.samp.map((p) => o.base + (+p.t));   // absolute clock (pre-offset)
    /* each clip's az/el is in its OWN observer's local frame — rotate into
       the shared reference frame (identity for the reference observer) */
    o.d = o.samp.map((p) => dirFromAzElAt(+p.az, +p.el, +o.s.lat, +o.s.lon, ref));
    o.q = o.samp.map((p) => (p.q == null ? 1 : +p.q));
  });
  const dirAt = (ts, ds, t) => {
    if (t <= ts[0]) return ds[0];
    if (t >= ts[ts.length - 1]) return ds[ds.length - 1];
    let i = 0; while (i < ts.length - 2 && ts[i + 1] < t) i++;
    const a = ds[i], b = ds[i + 1], u = (t - ts[i]) / Math.max(1e-9, ts[i + 1] - ts[i]);
    return unit([a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u]);
  };
  /* triangulate the overlap with observers[1..] shifted by `delta`. cap caps
     the number of instants; reject enables MAD outlier rejection. */
  const triAt = (delta, cap, reject) => {
    const T = obs.map((o, i) => (i === 0 ? o.t : o.t.map((x) => x + delta)));
    const t0 = Math.max(...T.map((tt) => tt[0]));
    const t1 = Math.min(...T.map((tt) => tt[tt.length - 1]));
    if (!(t1 > t0 + 1e-6)) return null;
    /* only instants inside EVERY clip's visibility segments — a hole in one
       clip (object invisible) must not consume the other's real data
       against an interpolated ray */
    let shared = trackSegments(T[0]);
    for (let i = 1; i < T.length; i++) shared = interSegments(shared, trackSegments(T[i]));
    shared = shared.map(([a, b]) => [Math.max(a, t0), Math.min(b, t1)]).filter(([a, b]) => b > a);
    if (!shared.length) return null;
    let times = obs[0].t.filter((x) => x >= t0 && x <= t1 && inSegments(shared, x));
    if (times.length > cap) { const step = Math.ceil(times.length / cap); times = times.filter((_, i) => i % step === 0); }
    if (times.length < 3) return null;
    const rows = [];
    for (const t of times) {
      const lines = obs.map((o, i) => ({ P: o.P, d: dirAt(T[i], o.d, t) }));
      const sol = intersectLines(lines);
      if (!sol || sol.ts.some((x) => x <= 0)) continue;  // object behind an observer ⇒ reject
      rows.push({ t, X: sol.X, miss: sol.rmsMiss });
    }
    if (rows.length < 3) return null;
    let kept = rows;
    if (reject) {
      const ms = rows.map((r) => r.miss).slice().sort((a, b) => a - b);
      const med = ms[ms.length >> 1];
      const mad = ms.map((m) => Math.abs(m - med)).sort((a, b) => a - b)[ms.length >> 1] || 1e-6;
      const thr = med + 4 * mad + 0.5;   // +0.5 m floor so a tight fix isn't over-trimmed
      kept = rows.filter((r) => r.miss <= thr);
      if (kept.length < 3) kept = rows;
    }
    const meanMiss = kept.reduce((s, r) => s + r.miss, 0) / kept.length;
    return { rows: kept, meanMiss, dropped: rows.length - kept.length, t0, t1, sharedDur: segsDur(shared) };
  };
  /* AUTO-SYNC: coarse sweep then refine, minimising mean ray-miss. Window is
     tight when both clips carry EXIF time (only clock drift to absorb), wide
     when they don't (the clips could start at any relative time). */
  const bothWhen = obs.every((o) => isNum(o.s.whenMs));
  const durs = obs.map((o) => o.t[o.t.length - 1] - o.t[0]);
  const W = opts.window != null ? opts.window : (bothWhen ? 6 : Math.max(...durs) + 2);
  let best = null;
  for (let dl = -W; dl <= W + 1e-9; dl += 0.1) {
    const r = triAt(dl, 120, false);
    if (r && (!best || r.meanMiss < best.meanMiss)) best = { delta: dl, meanMiss: r.meanMiss };
  }
  if (!best) return null;
  for (let dl = best.delta - 0.14; dl <= best.delta + 0.14 + 1e-9; dl += 0.02) {
    const r = triAt(dl, 160, false);
    if (r && r.meanMiss < best.meanMiss) best = { delta: dl, meanMiss: r.meanMiss };
  }
  const offset = obs[0].base != null ? best.delta : best.delta;  // relative to obs[1..] clock
  const final = triAt(best.delta, 400, true);
  if (!final) return null;
  const times = final.rows.map((r) => r.t), pos = final.rows.map((r) => r.X);
  const k = kinematics(times, pos);
  /* sync CONFIDENCE: how sharply the miss rises when we mistune the offset by
     ±0.5 s. A deep, narrow minimum ⇒ well-constrained; a flat one (far/slow
     object) ⇒ the sync — and thus absolute speed — is soft. */
  const off1 = triAt(best.delta - 0.5, 120, false), off2 = triAt(best.delta + 0.5, 120, false);
  const rise = Math.min(off1 ? off1.meanMiss : Infinity, off2 ? off2.meanMiss : Infinity) - best.meanMiss;
  const syncConf = !isFinite(rise) ? 0 : clampN(rise / (best.meanMiss + 0.5), 0, 1);
  /* per-observer mean range to the fixed path */
  const perObs = obs.map((o) => {
    let s = 0; for (const X of pos) s += mag(sub(X, o.P));
    return { name: o.s.name, meanRange: s / pos.length, P: o.P };
  });
  /* baseline + convergence for grading (max pair separation; mean angle
     between rays at the fixed points) */
  let baseline = 0;
  for (let i = 0; i < obs.length; i++) for (let j = i + 1; j < obs.length; j++) baseline = Math.max(baseline, mag(sub(obs[i].P, obs[j].P)));
  let convSum = 0, convN = 0;
  for (const X of pos) {
    for (let i = 0; i < obs.length; i++) for (let j = i + 1; j < obs.length; j++) {
      const di = unit(sub(X, obs[i].P)), dj = unit(sub(X, obs[j].P));
      convSum += Math.acos(clampN(dot(di, dj), -1, 1)) * R2D; convN++;
    }
  }
  const conv = convN ? convSum / convN : 0;
  return {
    ok: true, ref, offset, meanMiss: final.meanMiss, dropped: final.dropped,
    n: times.length, times, pos, k, window: [final.t0, final.t1],
    nObs: obs.length, perObs, syncConf, bothWhen, baseline, conv,
    names: obs.map((o) => o.s.name),
    sharedDur: final.sharedDur, cutDur: Math.max(0, (final.t1 - final.t0) - final.sharedDur), qDropped,
  };
}

/* MIXED VIDEO + STILL — pull the MOST out of a dense video clip paired with a
   single still photo of the same object. The still gives ONE sight-line at ONE
   instant; the video gives a dense WORLD-frame angular track (objPath) plus,
   where the object was sized across frames, a range-ratio profile. We:
     1. ANCHOR: find the instant in the clip where the video's object direction
        ray comes CLOSEST to the still's sight-line ray (min ray-miss over the
        span) — this is when the object was where the still saw it, and it
        auto-corrects a wrong clip clock (a still is one sample, so a motion
        cross-correlation isn't possible, but the geometry pins it),
     2. triangulate those two rays → the object's absolute 3D position and
        RANGE at that instant,
     3. propagate that absolute range across every frame with the video's own
        size profile (range ∝ 1/tan(½·apparent size); constant range if the
        object was never sized) → a full absolute 3D trajectory,
     4. kinematics on that path → real speed / acceleration / g-load, and true
        size = angular size × range at every frame.
   One still + one video therefore yields an absolute trajectory a lone video
   can't. Returns the anchor fix + dense path + residuals. Pure. */
export function mixedStereo(sources) {
  const hasTrack = (s) => Array.isArray(s.objPath) && s.objPath.length >= 3;
  const vids = (sources || []).filter((s) => isNum(s.lat) && isNum(s.lon) && hasTrack(s));
  const stills = (sources || []).filter((s) => isNum(s.lat) && isNum(s.lon) && isNum(s.A?.az) && isNum(s.A?.el) && !hasTrack(s));
  if (!vids.length || !stills.length) return null;
  const vid = vids[0], still = stills[0];
  const ref = { lat: +vid.lat, lon: +vid.lon, alt: isNum(vid.alt) ? +vid.alt : 0 };
  const Pv = enuFromGeo(+vid.lat, +vid.lon, isNum(vid.alt) ? +vid.alt : 0, ref);
  const Ps = enuFromGeo(+still.lat, +still.lon, isNum(still.alt) ? +still.alt : 0, ref);
  const op = vid.objPath.filter((p) => isNum(p.t) && isNum(p.az) && isNum(p.el)).sort((a, b) => a.t - b.t);
  const vt = op.map((p) => +p.t);
  const dirs = op.map((p) => dirFromAzElAt(+p.az, +p.el, +vid.lat, +vid.lon, ref));
  const dirAtV = (t) => {
    if (t <= vt[0]) return dirs[0];
    if (t >= vt[vt.length - 1]) return dirs[dirs.length - 1];
    let i = 0; while (i < vt.length - 2 && vt[i + 1] < t) i++;
    const a = dirs[i], b = dirs[i + 1], u = (t - vt[i]) / Math.max(1e-9, vt[i + 1] - vt[i]);
    return unit([a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u, a[2] + (b[2] - a[2]) * u]);
  };
  const ds = dirFromAzElAt(+still.A.az, +still.A.el, +still.lat, +still.lon, ref);
  /* ANCHOR SEARCH: the clip time whose object ray best meets the still ray */
  let best = null;
  const scan = (lo, hi, step) => {
    for (let t = lo; t <= hi + 1e-9; t += step) {
      const sol = intersectLines([{ P: Pv, d: dirAtV(t) }, { P: Ps, d: ds }]);
      if (!sol || sol.ts.some((x) => x <= 0)) continue;   // object behind an observer
      if (!best || sol.rmsMiss < best.miss) best = { t, miss: sol.rmsMiss, X: sol.X };
    }
  };
  const span = vt[vt.length - 1] - vt[0];
  scan(vt[0], vt[vt.length - 1], Math.max(span / 200, 0.02));
  if (!best) return null;
  scan(best.t - span / 100, best.t + span / 100, Math.max(span / 2000, 0.005));
  const distAnchor = mag(sub(best.X, Pv));
  if (!(distAnchor > 0)) return null;
  /* size profile → absolute range at every frame */
  const sized = (vid.track || []).filter((p) => isNum(p.t) && isNum(p.ang) && +p.ang > 0).map((p) => ({ t: +p.t, ang: +p.ang })).sort((a, b) => a.t - b.t);
  const angAt = sized.length ? (t) => {
    if (t <= sized[0].t) return sized[0].ang;
    if (t >= sized[sized.length - 1].t) return sized[sized.length - 1].ang;
    let i = 0; while (i < sized.length - 1 && sized[i + 1].t < t) i++;
    const a = sized[i], b = sized[i + 1]; return a.ang + (b.ang - a.ang) * ((t - a.t) / Math.max(1e-9, b.t - a.t));
  } : null;
  const angAnchor = angAt ? angAt(best.t) : null;
  const rangeAt = (t) => (angAt && angAnchor > 0) ? distAnchor * Math.tan((angAnchor * D2R) / 2) / Math.tan((angAt(t) * D2R) / 2) : distAnchor;
  const pos = op.map((p, i) => add(Pv, scl(dirs[i], rangeAt(+p.t))));
  const k = kinematics(vt, pos);
  const sizeSeries = angAt ? op.map((p) => 2 * rangeAt(+p.t) * Math.tan((angAt(+p.t) * D2R) / 2)) : null;
  const baseline = mag(sub(Pv, Ps));
  const dvA = dirAtV(best.t), conv = Math.acos(clampN(dot(dvA, ds), -1, 1)) * R2D;
  return {
    ok: true, ref, Pv, Ps, baseline, conv,
    anchor: { X: best.X, dist: distAnchor, rmsMiss: best.miss, vt: best.t, ang: angAnchor, sizeM: angAnchor != null ? 2 * distAnchor * Math.tan((angAnchor * D2R) / 2) : null },
    n: pos.length, times: vt, pos, k, sized: !!angAt,
    sizeMin: sizeSeries ? Math.min(...sizeSeries) : null,
    sizeMax: sizeSeries ? Math.max(...sizeSeries) : null,
    names: [vid.name, still.name],
  };
}

/* DENSE per-frame object kinematics from a stabilized video's objPath
   ([{t,az,el,q}] — the tracked object's WORLD direction each sampled frame,
   camera motion already removed by stabilization). This is the ANGULAR
   trajectory, measured with NO distance assumption:
     • ω(t) = dθ/dt, the angular rate in deg/s between samples (itself a strong
       discriminator — a satellite tracks at a near-constant rate, an aircraft's
       varies, a hovering object ≈ 0),
     • the total angular sweep and duration.
   If the measure-step track carries per-frame angular SIZES, they are
   interpolated onto the objPath times, giving the size profile (min/max +
   range-ratio) — so linear size and speed at ANY assumed distance follow via
   the returned helpers. `q` (match confidence) rides along so the report can
   flag stretches held on the guide rather than pixel-locked. Pure. */
export function videoKinematics(source) {
  const op = (source?.objPath || []).filter((p) => isNum(p.t) && isNum(p.az) && isNum(p.el)).sort((a, b) => a.t - b.t);
  if (op.length < 3) return null;
  const dirs = op.map((p) => dirFromAzEl(+p.az, +p.el));
  const t = op.map((p) => +p.t);
  const rate = []; let sweep = 0;
  for (let i = 0; i < op.length - 1; i++) {
    const dt = t[i + 1] - t[i]; if (!(dt > 1e-6)) continue;
    const dth = Math.acos(clampN(dot(dirs[i], dirs[i + 1]), -1, 1)) * R2D;
    sweep += dth;
    rate.push({ t: (t[i] + t[i + 1]) / 2, omega: dth / dt });
  }
  if (!rate.length) return null;
  const dur = t[t.length - 1] - t[0];
  const peakOmega = Math.max(...rate.map((r) => r.omega));
  const avgOmega = dur > 0 ? sweep / dur : 0;
  /* per-frame angular SIZE: interpolate the sized measure-step points onto the
     objPath times (nearest-held at the ends). `ang` is degrees of full width. */
  const sized = (source?.track || []).filter((p) => isNum(p.t) && isNum(p.ang) && +p.ang > 0)
    .map((p) => ({ t: +p.t, ang: +p.ang })).sort((a, b) => a.t - b.t);
  let ang = null;
  if (sized.length) {
    ang = t.map((tt) => {
      if (tt <= sized[0].t) return sized[0].ang;
      if (tt >= sized[sized.length - 1].t) return sized[sized.length - 1].ang;
      let i = 0; while (i < sized.length - 1 && sized[i + 1].t < tt) i++;
      const a = sized[i], b = sized[i + 1], u = (tt - a.t) / Math.max(1e-9, b.t - a.t);
      return a.ang + (b.ang - a.ang) * u;
    });
  }
  const angMin = ang ? Math.min(...ang) : null;
  const angMax = ang ? Math.max(...ang) : null;
  /* range ∝ 1/tan(½·angularSize): the range ratio (how much closer/farther the
     object got) is measurable from size alone, distance-free. */
  const rangeRatio = (angMin && angMax) ? Math.tan((angMax * D2R) / 2) / Math.tan((angMin * D2R) / 2) : null;
  return {
    n: op.length, dur, sweep, peakOmega, avgOmega,
    samples: op.map((p, i) => ({ t: t[i], az: +p.az, el: +p.el, q: p.q, ang: ang ? ang[i] : null })),
    rate, ang, angMin, angMax, rangeRatio,
    /* at an assumed reference distance D (metres) at the reference time, return
       the object's true size (m) and its mean TANGENTIAL speed (m/s). If sizes
       vary, range tracks size so the reported speed includes radial motion. */
    atDistance(D) {
      if (!(D > 0)) return null;
      // reference: the first sized frame (or the first frame if unsized)
      const iRef = ang ? 0 : 0;
      const angRef = ang ? ang[iRef] : null;
      const range = ang ? t.map((_, i) => D * Math.tan((angRef * D2R) / 2) / Math.tan((ang[i] * D2R) / 2)) : t.map(() => D);
      // 3D positions in metres, then finite-difference speed
      let pathLen = 0, peakSp = 0;
      for (let i = 0; i < op.length - 1; i++) {
        const dt = t[i + 1] - t[i]; if (!(dt > 1e-6)) continue;
        const p0 = scl(dirs[i], range[i]), p1 = scl(dirs[i + 1], range[i + 1]);
        const seg = mag(sub(p1, p0));
        pathLen += seg;
        peakSp = Math.max(peakSp, seg / dt);
      }
      const avgSp = dur > 0 ? pathLen / dur : 0;
      const sizeRef = ang ? 2 * D * Math.tan((angRef * D2R) / 2) : null;
      return { range0: D, sizeM: sizeRef, avgSpeed: avgSp, peakSpeed: peakSp, path: pathLen };
    },
  };
}
