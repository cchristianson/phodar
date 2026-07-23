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
import { despikePath, smoothPath, posePathAt } from "./postrack.js";

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

  /* world direction of each feature. A feature is anchored when its g is known:
       • a celestial tag (Moon/star) → known upfront from the ephemeris;
       • a mark on the ALIGN frame → g from the known placement pose;
       • otherwise g is learned LAZILY (bootstrapping): once a nearby keyframe is
         solved, a feature first appearing there gets its g from THAT solved pose.
     This is what makes references HAND OFF — as the camera pans and one point
     leaves the frame, a fresh one that overlapped it on ≥1 frame carries the
     pose onward, so no single feature has to stay in view the whole clip. */
  const G = new Array(refs.length).fill(null);
  const ALIGN_TOL = 0.06;
  const markAt = (r, t) => r.marks.find((m) => isNum(m.x) && Math.abs(m.t - t) < 1e-3);
  refs.forEach((r, i) => {
    if (r.celestial && isNum(r.celestial.az) && isNum(r.celestial.el)) { G[i] = dirFromAzEl(+r.celestial.az, +r.celestial.el); return; }
    const m = r.marks.find((mm) => isNum(mm.x) && Math.abs(mm.t - refT) < ALIGN_TOL);   // marked on the align frame
    if (m) G[i] = pixToDirK(m.x, m.y, natW, natH, rp.az, rp.el, rp.roll, rp.fov, rp.k);
  });
  if (!G.some(Boolean)) return null;   // nothing anchored — need ≥1 reference on the align frame (or a celestial tag)

  const times = [...new Set(refs.flatMap((r) => r.marks.filter((m) => isNum(m.x)).map((m) => +(+m.t).toFixed(3))))].sort((a, b) => a - b);
  if (!times.length) return null;

  const byT = {};
  const chain = (ts, startSeed) => {
    let seed = { ...startSeed, lockFov: opts.lockFov };
    for (const t of ts) {
      const anchors = [];
      refs.forEach((r, i) => { if (G[i]) { const m = markAt(r, t); if (m) anchors.push({ px: m.x, py: m.y, g: G[i] }); } });
      if (!anchors.length) continue;   // no anchored feature here yet — hold (posePathAt bridges/interpolates)
      const sol = solvePose(anchors, natW, natH, seed);
      byT[t] = { t, az: sol.az, el: sol.el, roll: sol.roll, fov: sol.fov, k: sol.k, n: sol.n, rms: sol.rms };
      seed = { az: sol.az, el: sol.el, roll: sol.roll, fov: sol.fov, k: sol.k, lockFov: opts.lockFov };
      // anchor any NEW feature marked on this now-solved frame → carries the pose forward
      refs.forEach((r, i) => { if (!G[i]) { const m = markAt(r, t); if (m) G[i] = pixToDirK(m.x, m.y, natW, natH, sol.az, sol.el, sol.roll, sol.fov, sol.k); } });
    }
  };

  const start = { az: rp.az, el: rp.el, roll: rp.roll, fov: rp.fov, k: rp.k };
  chain(times.filter((t) => t >= refT), start);            // forward from the align frame
  chain(times.filter((t) => t < refT).reverse(), start);   // then backward
  const path = Object.values(byT).sort((a, b) => a.t - b.t);
  /* AVERAGE OUT imperfect placement: each keyframe's pose carries the noise of
     that frame's taps. Run the same evidence-weighted despike + smooth the auto
     walk uses (weighted by each keyframe's anchor count `n`) so a single sloppy
     mark is pulled back toward its neighbours and per-frame jitter is damped,
     while a real sustained pan/turn passes through. */
  if (path.length >= 3 && opts.smooth !== false) {
    despikePath(path, { passes: 1 });
    smoothPath(path, { passes: opts.smoothPasses == null ? 1 : opts.smoothPasses });
  }
  return path.length ? path : null;
}

/* HYBRID: fuse the automatic stabilization with the hand-marked pose. The
   automatic walk gives dense, smooth motion but can be CONFIDENTLY WRONG
   (cloud-locked, or frozen on a held frame); the manual keyframes are ground
   truth but sparse. So decide span-by-span between consecutive manual
   keyframes:
     • auto AGREES with the marks at both ends (center within `gate`, roll too,
       and the frame wasn't held) → keep auto's dense shape, but smear a linear
       correction across the span so it passes exactly through both marks (the
       same drift-correction the auto walk does at re-anchors);
     • auto DISAGREES at either end (garbage / held) → drop auto there and use
       the manual interpolation.
   Ground truth is pinned at every manual keyframe either way. If auto is all
   bad it degrades to (dense) manual; if auto is all good it's auto with tiny
   corrections. Returns a dense posePath tagged per-sample with `src`. */
export function hybridPath(autoPath, manualPath, opts = {}) {
  if (!Array.isArray(autoPath) || !autoPath.length) return manualPath || null;
  if (!Array.isArray(manualPath) || !manualPath.length) return autoPath;
  const gate = opts.gate == null ? 2.5 : opts.gate;        // deg the auto MOTION may differ from the marks across a span
  const angD = (a, b) => ((a - b + 540) % 360) - 180;
  const nAt = (t) => { let bd = Infinity, bn = 99; for (const p of autoPath) { const d = Math.abs(p.t - t); if (d < bd) { bd = d; bn = p.n == null ? 99 : p.n; } } return bn; };

  // residual (mark − auto) at each manual keyframe
  const anch = manualPath.map((m) => {
    const a = posePathAt(autoPath, m.t);
    return { t: m.t, held: nAt(m.t) < 6, res: { dAz: angD(m.az, a.az), dEl: m.el - (a.el || 0), dRoll: (m.roll || 0) - (a.roll || 0), dFov: m.fov - a.fov } };
  });
  /* Trust auto in a span only when its MOTION matches the marks — i.e. the
     correction is roughly CONSTANT across the span (a steady offset/drift, which
     the smear removes), NOT merely that auto happens to match at the endpoints.
     A span where auto barely moved but the marks swept has a big correction
     CHANGE → distrust → use the marks. (This was the bug: cloud-locked auto that
     matched the sparse marks by luck but undershot the real pan between them,
     leaving jitter instead of the big movement.) */
  const consistent = (A, B) => !A.held && !B.held
    && Math.abs(angD(A.res.dAz, B.res.dAz)) <= gate
    && Math.abs(A.res.dEl - B.res.dEl) <= gate
    && Math.abs(A.res.dRoll - B.res.dRoll) <= gate * 1.5;

  const times = [...new Set([...autoPath.map((p) => +p.t), ...manualPath.map((p) => +p.t)])].sort((a, b) => a - b);
  const out = [];
  for (const t of times) {
    let lo = null, hi = null;
    for (const an of anch) { if (an.t <= t + 1e-9) lo = an; if (an.t >= t - 1e-9 && !hi) hi = an; }
    const useAuto = !!(lo && hi && consistent(lo, hi));   // interior span AND auto's motion agrees with the marks
    if (useAuto) {
      const a = posePathAt(autoPath, t);
      const u = hi.t > lo.t ? clampN((t - lo.t) / (hi.t - lo.t), 0, 1) : 0;
      const lin = (k) => lo.res[k] + (hi.res[k] - lo.res[k]) * u;
      out.push({ t, az: (((a.az + lin("dAz")) % 360) + 360) % 360, el: a.el + lin("dEl"), roll: (a.roll || 0) + lin("dRoll"), fov: a.fov + lin("dFov"), k: a.k || 0, n: a.n, src: "auto" });
    } else {
      const m = posePathAt(manualPath, t);
      out.push({ t, az: m.az, el: m.el, roll: m.roll || 0, fov: m.fov, k: m.k || 0, n: m.n, src: "manual" });
    }
  }
  return out;
}
