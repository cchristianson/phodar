/* ============================================================
   TRAJECTORY ANALYSIS
   Track points (pixels on video frames, or direct az/el) become
   time-stamped sight directions. Two observers → interpolate to
   common times, triangulate each sample → 3D path → velocity,
   acceleration, felt g-load, turn rate. One observer → angular
   kinematics that scale with an assumed distance.
   ============================================================ */

import { D2R, R2D, clampN, dot, sub, add, scl, mag, unit, enuFromGeo, dirFromAzEl, dirToAzEl } from "./geodesy.js";
import { isNum } from "./format.js";
import { pixelDirFromAnchor, angSizeFromPoints } from "./projection.js";
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
    const ang = angSizeFromPoints(m.A?.p1, m.A?.p2, m.natW, m.natH, +m.fovH);
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
  const p0 = pts.find((p) => p.x != null);
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
    out.push({ ct, d, az: ae.az, el: ae.el, r: p.r });
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
    const obs = withT.map((o) => ({ ...o, P: enuFromGeo(+o.s.lat, +o.s.lon, isNum(o.s.alt) ? +o.s.alt : 0, ref) }));
    const t0 = Math.max(...obs.map((o) => o.dirs[0].ct));
    const t1 = Math.min(...obs.map((o) => o.dirs[o.dirs.length - 1].ct));
    if (!(t1 > t0)) stereo = { overlapErr: true };
    else {
      let ts = [...new Set(obs.flatMap((o) => o.dirs.map((d) => +d.ct.toFixed(3))))]
        .filter((t) => t >= t0 && t <= t1).sort((a, b) => a - b);
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
        if (k) stereo = { k, times, pos, ref, avgMiss: missSum / pos.length, window: [t0, t1], nObs: obs.length };
      }
      if (!stereo) stereo = pos.length ? { sparse: true } : { overlapErr: true };
    }
  }
  return { stereo, solo };
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
