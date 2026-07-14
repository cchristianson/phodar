/* ============================================================
   TRAJECTORY ANALYSIS
   Track points (pixels on video frames, or direct az/el) become
   time-stamped sight directions. Two observers → interpolate to
   common times, triangulate each sample → 3D path → velocity,
   acceleration, felt g-load, turn rate. One observer → angular
   kinematics that scale with an assumed distance.
   ============================================================ */

import { R2D, clampN, dot, sub, add, scl, mag, unit, enuFromGeo, dirFromAzEl, dirToAzEl } from "./geodesy.js";
import { isNum } from "./format.js";
import { pixelDirFromAnchor } from "./projection.js";
import { intersectLines } from "./triangulate.js";

/* Convert a source's track into [{ct, d}] — clock time + unit direction.
   Pixel points are anchored to Moment A: the FIRST pixel track point is
   defined to lie along A's az/el; later points are pixel offsets from it.

   POSE NOTE (video roadmap): `s.mediaAim` is read here as the pose for
   every point, which assumes the camera never moved. When per-frame poses
   land, a point's own `p.pose` should win over the source-level one. */
export function trackDirections(s) {
  if (!s.track || s.track.length < 2) return null;
  const pts = [...s.track].sort((a, b) => a.t - b.t);
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
    out.push({ ct, d, az: ae.az, el: ae.el });
  }
  return out.length >= 2 ? out : null;
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

export function analyzeTracks(sources) {
  /* solo angular kinematics on the unit sphere (scale by distance later) */
  const solo = sources.map((s) => {
    const dirs = trackDirections(s);
    if (!dirs || dirs.length < 3) return null;
    const k = kinematics(dirs.map((d) => d.ct), dirs.map((d) => d.d));
    return k ? { name: s.name, k } : null;
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
