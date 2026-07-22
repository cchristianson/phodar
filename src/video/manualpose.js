/* ============================================================
   MANUAL POSE FROM HAND-MARKED REFERENCES
   A fallback for clips the automatic stabilizer can't solve —
   low-contrast night skies, drifting-but-usable clouds, near-black
   stretches where no feature detector or NCC tracker can hold.

   The idea: the user hand-marks a few WORLD-FIXED features (a cloud
   notch, a star, a ground light, the horizon) on the alignment frame,
   then re-marks the SAME features on other frames. A human supplies the
   correspondence that NCC can't on soft self-similar texture; the math
   here is identical to the automatic path — every mark is just an
   anchor {px, py, g}, solved by the same kind of coordinate descent.

   world direction g of each feature comes from its mark on the ALIGNMENT
   frame, whose pose is known (the placement). Re-marking that feature on
   another frame gives another anchor with the SAME g (distant-scene /
   pure-rotation assumption the whole pipeline already makes). Each frame
   with ≥1 mark solves; sparse keyframes interpolate downstream via
   posePathAt — so the output is an ordinary posePath and everything
   (playback, object track, export, report) consumes it unchanged.

   This module is self-contained: it imports only the shared projection/
   geodesy primitives and is used by nothing except the manual-refs UI, so
   it can be removed wholesale without touching the automatic pipeline.
   ============================================================ */

import { R2D, dot, clampN, dirFromAzEl } from "../math/geodesy.js";
import { isNum } from "../math/format.js";
import { pixToDirK } from "../math/projection.js";

/* Solve one frame's pose from anchors [{px,py,g}] by coordinate descent with
   step-halving on summed squared angular error. UNLIKE solvePoseAnchors (which
   keeps the photo CENTER fixed for star plate-solves), az/el are ALWAYS free —
   the whole point here is that the camera panned, so the center MUST move to put
   each known direction where it now appears. More marks unlock more DOF:
     1 mark  → az, el              (roll + fov held from the seed)
     2 marks → + roll
     3 marks → + fov               (unless seed.lockFov)
     4 marks → + k (lens distortion)
   Seeded (and bounded ±45°) from the previous frame's pose, so a chain that
   steps ≤45° between marked keyframes tracks arbitrarily large total pans. */
export function solvePose(anchors, natW, natH, seed) {
  const n = anchors.length;
  const P = { az: seed.az, el: seed.el, roll: seed.roll || 0, fov: seed.fov, k: seed.k || 0 };
  const params = ["az", "el",
    ...(n >= 2 ? ["roll"] : []),
    ...(n >= 3 && !seed.lockFov ? ["fov"] : []),
    ...(n >= 4 && !seed.lockK ? ["k"] : [])];
  const lim = {
    az: [P.az - 45, P.az + 45], el: [clampN(P.el - 45, -89, 89), clampN(P.el + 45, -89, 89)],
    roll: [-180, 180], fov: [8, 150], k: [-0.6, 0.6],
  };
  const step = { az: 2, el: 2, roll: 2, fov: 2, k: 0.02 };
  const cl = (pn, v) => Math.min(lim[pn][1], Math.max(lim[pn][0], v));
  const sse = (q) => {
    let s = 0;
    for (const a of anchors) {
      const c = clampN(dot(pixToDirK(a.px, a.py, natW, natH, q.az, q.el, q.roll, q.fov, q.k), a.g), -1, 1);
      s += Math.acos(c) ** 2;
    }
    return s;
  };
  for (let iter = 0; iter < 500; iter++) {
    let any = false;
    for (const pn of params) {
      const base = sse(P), v = P[pn], st = step[pn];
      P[pn] = cl(pn, v + st); const up = sse(P);
      P[pn] = cl(pn, v - st); const dn = sse(P);
      if (up < base && up <= dn) { P[pn] = cl(pn, v + st); any = true; }
      else if (dn < base) { P[pn] = cl(pn, v - st); any = true; }
      else P[pn] = v;
    }
    if (!any) { for (const pn of params) step[pn] *= 0.5; if (step.az < 1e-4) break; }
  }
  return { az: P.az, el: P.el, roll: P.roll, fov: P.fov, k: P.k, rms: Math.sqrt(sse(P) / Math.max(1, n)) * R2D, n };
}

/* Build a posePath from hand-marked reference features.
     camRefs : [{ marks: [{t,x,y}], celestial?: {az,el} }, ...]
     refPose : { t, az, el, roll, fov, k } — the ALIGNMENT frame's known pose
     dims    : { natW, natH }
     opts    : { lockFov }
   Returns a sorted posePath [{t,az,el,roll,fov,k,n,rms}] or null.

   Each feature's world direction g is fixed once: from its mark nearest the
   alignment time (relative), or from an attached celestial az/el (absolute —
   a tagged Moon/star, more accurate and drift-proof). Keyframes are solved
   OUTWARD from the alignment time so each warm-starts from its neighbour. */
export function solveManualPoses(camRefs, refPose, dims, opts = {}) {
  const { natW, natH } = dims || {};
  if (!(natW > 0 && natH > 0) || !refPose || !isNum(refPose.az)) return null;
  const refs = (camRefs || []).filter((r) => Array.isArray(r?.marks) && r.marks.some((m) => isNum(m?.x)));
  if (!refs.length) return null;

  const rp = { az: +refPose.az, el: +refPose.el, roll: +refPose.roll || 0, fov: +refPose.fov, k: +refPose.k || 0 };
  const refT = isNum(refPose.t) ? +refPose.t : 0;

  // fixed world direction of each feature
  const G = refs.map((r) => {
    if (r.celestial && isNum(r.celestial.az) && isNum(r.celestial.el)) {
      // absolute: tagged Moon/star — direction straight from the ephemeris
      return dirFromAzEl(+r.celestial.az, +r.celestial.el);
    }
    const near = r.marks.filter((m) => isNum(m.x)).reduce((a, m) => (Math.abs(m.t - refT) < Math.abs(a.t - refT) ? m : a));
    return pixToDirK(near.x, near.y, natW, natH, rp.az, rp.el, rp.roll, rp.fov, rp.k);
  });

  // union of marked keyframe times
  const times = [...new Set(refs.flatMap((r) => r.marks.filter((m) => isNum(m.x)).map((m) => +(+m.t).toFixed(3))))].sort((a, b) => a - b);
  if (!times.length) return null;

  const gather = (t) => {
    const out = [];
    refs.forEach((r, i) => {
      const m = r.marks.find((mm) => isNum(mm.x) && Math.abs(mm.t - t) < 1e-3);
      if (m) out.push({ px: m.x, py: m.y, g: G[i] });
    });
    return out;
  };
  const chain = (ts, startSeed) => {
    let seed = { ...startSeed, lockFov: opts.lockFov };
    const out = [];
    for (const t of ts) {
      const anchors = gather(t);
      if (!anchors.length) continue;
      const sol = solvePose(anchors, natW, natH, seed);
      out.push({ t, az: sol.az, el: sol.el, roll: sol.roll, fov: sol.fov, k: sol.k, n: sol.n, rms: sol.rms });
      seed = { az: sol.az, el: sol.el, roll: sol.roll, fov: sol.fov, k: sol.k, lockFov: opts.lockFov };
    }
    return out;
  };

  const start = { az: rp.az, el: rp.el, roll: rp.roll, fov: rp.fov, k: rp.k };
  const after = times.filter((t) => t >= refT);
  const before = times.filter((t) => t < refT).reverse();
  const path = [...chain(before, start), ...chain(after, start)].sort((a, b) => a.t - b.t);
  return path.length ? path : null;
}
