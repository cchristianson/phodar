/* ============================================================
   PROJECTION / CAMERA MATH
   A pinhole photo IS a gnomonic projection, so every operation here
   is a projective one: pixels ↔ directions given a pose, and the
   photo-quad ↔ sky-view homography.

   POSE NOTE (video roadmap): the `pose` argument below — {az, el, roll}
   — is deliberately a *sample*, not a per-source constant. A still has
   one; a video has one per frame. Nothing in this module may assume
   the camera never moved.
   ============================================================ */

import { D2R, R2D, RAD, dot, unit, dirFromAzEl } from "./geodesy.js";
import { isNum } from "./format.js";

/* focal length in pixels for a given horizontal FOV */
export const focalPx = (natW, fovH) => (natW / 2) / Math.tan((fovH * D2R) / 2);

/* camera basis for the photo's pose (az, el, roll) */
export function photoBasis(az, el, roll) {
  const f = dirFromAzEl(az, el);
  let r0 = [f[1], -f[0], 0];
  const rl = Math.hypot(r0[0], r0[1]) || 1; r0 = [r0[0] / rl, r0[1] / rl, 0];
  const u0 = [r0[1] * f[2] - r0[2] * f[1], r0[2] * f[0] - r0[0] * f[2], r0[0] * f[1] - r0[1] * f[0]];
  const cr = Math.cos(roll * RAD), sr = Math.sin(roll * RAD);
  const r = [r0[0] * cr + u0[0] * sr, r0[1] * cr + u0[1] * sr, r0[2] * cr + u0[2] * sr];
  const u = [-r0[0] * sr + u0[0] * cr, -r0[1] * sr + u0[1] * cr, -r0[2] * sr + u0[2] * cr];
  return { f, r, u };
}

/* Two-tap star align: given the world direction the photo currently shows at
   the tap (vS = unproject(tap)), the object's true direction g, the photo's
   pose basis {f,r,u} and its current fov/roll (deg), solve the roll + FOV that
   land g exactly where the object sits in the photo — keeping the photo CENTER
   (f) fixed, so a terrain match at the center survives. Closed form: the object
   is at photo-plane coords (Xc,Yc); the target at (Gr,Gu); a rotation by dRoll
   plus a radial scale s maps one to the other (s → fov, angle → roll).
   Returns null if either direction is behind the photo center or the object is
   at the dead center (no radial leverage). */
export function solveRollFov(vS, g, basis, fovDeg, rollDeg) {
  const { f, r, u } = basis;
  const vf = dot(vS, f), gf = dot(g, f);
  if (vf <= 1e-3 || gf <= 1e-3) return null;
  const Xc = dot(vS, r) / vf, Yc = dot(vS, u) / vf;
  const Gr = dot(g, r) / gf, Gu = dot(g, u) / gf;
  const rc = Math.hypot(Xc, Yc), rt = Math.hypot(Gr, Gu);
  if (rc < 1e-3) return null;
  const s = rt / rc;
  const dRoll = (Math.atan2(Gu, Gr) - Math.atan2(Yc, Xc)) * R2D;
  const fov = 2 * Math.atan(s * Math.tan((fovDeg * RAD) / 2)) * R2D;
  return { fov, roll: rollDeg + dRoll, dRoll };
}

/* --- radial-distortion-aware pixel↔direction, and a multi-star pose fit ---
   A single gnomonic FOV is one radial scale; a real lens's scale drifts with
   radius (barrel/pincushion), so two stars at different radii can't both land
   under FOV alone. Adding one radial term k (tan-space: s = 1 + k·ρ²) gives the
   extra DOF, fitted from ≥2 anchors while the photo center stays fixed. */
export function pixToDirK(px, py, natW, natH, az, el, roll, fov, k) {
  const b = photoBasis(az, el, roll);
  const fpx = (natW / 2) / Math.tan((fov * RAD) / 2);
  const x = (px - natW / 2) / fpx, y = (natH / 2 - py) / fpx;
  const s = 1 + (k || 0) * (x * x + y * y);
  return unit([b.f[0] + b.r[0] * x * s + b.u[0] * y * s, b.f[1] + b.r[1] * x * s + b.u[1] * y * s, b.f[2] + b.r[2] * x * s + b.u[2] * y * s]);
}
/* inverse: where does true direction g fall as a pixel, under (pose, k)?
   Newton-inverts the radial term. Used to capture an anchor's fixed pixel. */
export function dirToPixK(g, natW, natH, az, el, roll, fov, k) {
  const b = photoBasis(az, el, roll);
  const fpx = (natW / 2) / Math.tan((fov * RAD) / 2);
  const gf = dot(g, b.f); if (gf <= 1e-6) return null;
  const Xc = dot(g, b.r) / gf, Yc = dot(g, b.u) / gf;   // = x·s, y·s (distorted)
  const rd = Math.hypot(Xc, Yc);
  let rho = rd;
  for (let i = 0; i < 10 && (k || 0) !== 0; i++) { const fr = rho * (1 + k * rho * rho) - rd, dfr = 1 + 3 * k * rho * rho; rho -= fr / dfr; }
  const sc = rd > 1e-9 ? rho / rd : 1;
  return { px: natW / 2 + Xc * sc * fpx, py: natH / 2 - Yc * sc * fpx };
}
/* Fit the photo pose to star anchors [{px,py,g}] by coordinate descent with
   step-halving on summed squared angular error. The active parameter set grows
   with the anchor count (each anchor = 2 constraints), so more stars lock in
   more of the pose:
     1 anchor  → roll, fov            (center kept — one star can't move it)
     2 anchors → roll, fov, k         (+ lens distortion; center still kept, so
                                        the terrain match survives)
     3+ anchors→ az, el, roll, fov, k (FULL plate solve — the stars drive the
                                        pointing too, matching the whole sky)
   (az,el) seed is the incoming center; it's only moved with ≥3 anchors, and
   bounded to ±8° of the seed so a mis-aim can't fling the pose. Returns
   {az,el,roll,fov,k,rms}.
   `seed.lockFov`/`seed.lockK` pin FOV/distortion to the seed instead of fitting
   them — used by per-frame video tracking, where the lens doesn't change between
   frames so a sparse frame shouldn't be free to drift its FOV. */
export function solvePoseAnchors(anchors, natW, natH, az0, el0, seed) {
  const n = anchors.length;
  const fitCenter = n >= 3, fitK = n >= 2 && !seed.lockK;
  const P = { az: az0, el: el0, roll: seed.roll, fov: seed.fov, k: seed.k || 0 };
  const lim = { az: [az0 - 8, az0 + 8], el: [el0 - 8, el0 + 8], roll: [-90, 90], fov: [8, 150], k: [-0.6, 0.6] };
  const sse = (q) => {
    let s = 0;
    for (const a of anchors) { const c = Math.min(1, Math.max(-1, dot(pixToDirK(a.px, a.py, natW, natH, q.az, q.el, q.roll, q.fov, q.k), a.g))); s += Math.acos(c) ** 2; }
    return s;
  };
  const params = [...(fitCenter ? ["az", "el"] : []), "roll", ...(seed.lockFov ? [] : ["fov"]), ...(fitK ? ["k"] : [])];
  const step = { az: 1.0, el: 1.0, roll: 1.0, fov: 2.0, k: 0.02 };
  const cl = (pn, v) => Math.min(lim[pn][1], Math.max(lim[pn][0], v));
  for (let iter = 0; iter < 400; iter++) {
    let any = false;
    for (const pn of params) {
      const base = sse(P), v = P[pn], st = step[pn];
      P[pn] = cl(pn, v + st); const up = sse(P);
      P[pn] = cl(pn, v - st); const dn = sse(P);
      if (up < base && up <= dn) { P[pn] = cl(pn, v + st); any = true; }
      else if (dn < base) { P[pn] = cl(pn, v - st); any = true; }
      else P[pn] = v;
    }
    if (!any) { for (const pn of params) step[pn] *= 0.5; if (step.fov < 1e-4) break; }
  }
  return { az: P.az, el: P.el, roll: P.roll, fov: P.fov, k: P.k, rms: Math.sqrt(sse(P) / Math.max(1, n)) * R2D };
}

export function angSizeFromPoints(p1, p2, natW, natH, fovH) {
  if (!p1 || !p2 || !natW || !fovH) return null;
  const f = focalPx(natW, fovH);
  const cx = natW / 2, cy = natH / 2;
  const ray = (p) => unit([p.x - cx, p.y - cy, f]);
  const c = Math.min(1, Math.max(-1, dot(ray(p1), ray(p2))));
  return Math.acos(c) * R2D;
}


/* Direction of a pixel through the best available camera model.
   With a placed photo (pose) the pose is exact. Without one, we build a
   zero-roll frame anchored at a reference pixel whose direction is known —
   which correctly handles azimuth convergence at elevation (the naive
   dAz = atan(dx/f) is off by 1/cos(el): 15% at 30° up, 41% at 45°). */
export function pixelDirFromAnchor(px, py, refPx, refPy, refAz, refEl, natW, natH, fovH, pose) {
  const fpx = focalPx(natW, fovH);
  if (pose && isNum(pose.az) && isNum(pose.el)) {
    const b = photoBasis(+pose.az, +pose.el, +pose.roll || 0);
    const x = (px - natW / 2) / fpx, y = (natH / 2 - py) / fpx;
    return unit([b.f[0] + b.r[0] * x + b.u[0] * y, b.f[1] + b.r[1] * x + b.u[1] * y, b.f[2] + b.r[2] * x + b.u[2] * y]);
  }
  const b = photoBasis(refAz, refEl, 0);
  const x = (px - refPx) / fpx, y = (refPy - py) / fpx;
  return unit([b.f[0] + b.r[0] * x + b.u[0] * y, b.f[1] + b.r[1] * x + b.u[1] * y, b.f[2] + b.r[2] * x + b.u[2] * y]);
}

/* --- homography: map the photo's flat quad onto the sky view exactly.
   A pinhole photo IS a gnomonic projection, so placing it in another
   gnomonic view is a projective transform. --- */
export function solveN(A, b) {
  const n = b.length, M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-9) return null;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((row, i) => row[n] / row[i]);
}

export function homography(sp, dp) {
  const A = [], b = [];
  for (let i = 0; i < 4; i++) {
    const [x, y] = sp[i], [X, Y] = dp[i];
    A.push([x, y, 1, 0, 0, 0, -X * x, -X * y]); b.push(X);
    A.push([0, 0, 0, x, y, 1, -Y * x, -Y * y]); b.push(Y);
  }
  const h = solveN(A, b);
  return h ? [[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1]] : null;
}

/* NOTE: user images are NEVER rendered through this — iOS Safari composes
   its hidden EXIF-orientation transform with author matrices unpredictably.
   See the canvas triangle-mesh warp in SkyAimer. Kept for the on-axis
   equivalence proof and for non-image overlays. */
export const matrix3dFromH = (H) =>
  `matrix3d(${H[0][0]},${H[1][0]},0,${H[2][0]},${H[0][1]},${H[1][1]},0,${H[2][1]},0,0,1,0,${H[0][2]},${H[1][2]},0,${H[2][2]})`;
