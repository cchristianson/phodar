/* ============================================================
   AUTO STAR-ALIGN — a LOCAL plate solve. Not blind astrometry:
   we already have (a) the catalog star directions at the sighting
   time/place, (b) a rough manual placement as a seed, and (c) the
   solvePoseAnchors fitter. This module supplies the two missing
   pieces — detect the bright star blobs in the photo, and match
   them to the catalog — then ICP-refines the pose to a precise
   az/el/roll/FOV/distortion. Mirrors the "Snap to ridges" flow,
   but against stars instead of the DEM skyline.

   Pure + tested (scripts/mathcheck.js drives a synthetic star field
   at a known pose and asserts recovery).
   ============================================================ */

import { R2D, dot, clampN } from "../math/geodesy.js";
import { dirToPixK, pixToDirK, solvePoseAnchors } from "../math/projection.js";

/* Detect compact bright blobs (stars) in an RGBA buffer. Clouds are large
   and diffuse → they form one over-size connected component and are dropped;
   stars are tiny bright peaks. Returns [{x,y,bright,area}] brightest-first, in
   the buffer's own pixel coordinates (scale to native outside). */
export function detectStars(data, w, h, opts = {}) {
  const N = w * h;
  const lum = new Float32Array(N);
  let sum = 0;
  for (let i = 0; i < N; i++) {
    const l = 0.299 * data[i * 4] + 0.587 * data[i * 4 + 1] + 0.114 * data[i * 4 + 2];
    lum[i] = l; sum += l;
  }
  const mean = sum / N;
  let vs = 0;
  for (let i = 0; i < N; i++) { const d = lum[i] - mean; vs += d * d; }
  const std = Math.sqrt(vs / N) || 1;
  const thr = Math.max(mean + (opts.kSigma || 4.5) * std, mean + 10);
  const maxArea = opts.maxArea || Math.min(160, Math.max(24, Math.round(w * h * 0.00006))); // bigger ⇒ cloud, dropped
  const minArea = opts.minArea || 1;
  const seen = new Uint8Array(N);
  const stars = [];
  const stack = [];
  for (let p0 = 0; p0 < N; p0++) {
    if (seen[p0] || lum[p0] < thr) continue;
    let area = 0, sx = 0, sy = 0, sw = 0, peak = 0, big = false;
    stack.length = 0; stack.push(p0); seen[p0] = 1;
    while (stack.length) {
      const p = stack.pop();
      const x = p % w, y = (p - x) / w, l = lum[p];
      area++; sw += l; sx += x * l; sy += y * l; if (l > peak) peak = l;
      if (area > maxArea) big = true;
      const x0 = x > 0 ? -1 : 0, x1 = x < w - 1 ? 1 : 0, y0 = y > 0 ? -1 : 0, y1 = y < h - 1 ? 1 : 0;
      for (let dy = y0; dy <= y1; dy++) for (let dx = x0; dx <= x1; dx++) {
        if (!dx && !dy) continue;
        const np = (y + dy) * w + (x + dx);
        if (!seen[np] && lum[np] >= thr) { seen[np] = 1; stack.push(np); }
      }
    }
    if (big || area < minArea) continue;
    stars.push({ x: sx / sw, y: sy / sw, bright: peak, area });
  }
  stars.sort((a, b) => b.bright - a.bright);
  return stars.slice(0, opts.maxN || 60);
}

/* catalog world-dirs → in-frame pixels under a pose */
function inFrameCatalog(cat, natW, natH, pose) {
  const out = [];
  for (const c of cat) {
    const p = dirToPixK(c.g, natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k || 0);
    if (!p) continue;
    if (p.px < -0.05 * natW || p.px > 1.05 * natW || p.py < -0.05 * natH || p.py > 1.05 * natH) continue;
    out.push({ px: p.px, py: p.py, g: c.g });
  }
  return out;
}

/* unique nearest-neighbour matches (catalog ↔ detected) within tol px */
function greedyMatch(catPix, det, tol) {
  const pairs = [];
  for (let i = 0; i < catPix.length; i++) {
    for (let j = 0; j < det.length; j++) {
      const d = Math.hypot(catPix[i].px - det[j].x, catPix[i].py - det[j].y);
      if (d <= tol) pairs.push({ i, j, d });
    }
  }
  pairs.sort((a, b) => a.d - b.d);
  const uc = new Set(), ud = new Set(), out = [];
  for (const p of pairs) {
    if (uc.has(p.i) || ud.has(p.j)) continue;
    uc.add(p.i); ud.add(p.j); out.push({ cat: catPix[p.i], det: det[p.j] });
  }
  return out;
}

/* Solve the photo pose from detected stars.
   det: [{x,y}] star pixels in NATIVE coords. cat: [{g:[x,y,z]}] catalog dirs
   (unit). pose0: current manual placement seed. Coarse-searches small
   roll/az/el/FOV offsets for the most star matches, then ICP-refines with
   shrinking tolerance via solvePoseAnchors. Returns {az,el,roll,fov,k,rms,n}
   or null when it can't lock (too few matches or poor fit). */
export function autoStarAlign(det, cat, natW, natH, pose0, opts = {}) {
  if (!det || det.length < 4 || !cat || cat.length < 4) return null;
  const el0 = pose0.el, tol0 = (opts.tol0 || 0.06) * natW;
  let best = null;
  for (let dr = -20; dr <= 20; dr += 5)
    for (let da = -6; da <= 6; da += 3)
      for (let de = -6; de <= 6; de += 3)
        for (const ff of [0.8, 0.9, 1.0, 1.15, 1.3]) {
          const pose = { az: pose0.az + da, el: clampN(el0 + de, -20, 89.5), roll: pose0.roll + dr, fov: clampN(pose0.fov * ff, 8, 140), k: pose0.k || 0 };
          const cp = inFrameCatalog(cat, natW, natH, pose);
          if (cp.length < 4) continue;
          const m = greedyMatch(cp, det, tol0);
          const score = m.length - 1e-4 * m.reduce((s, x) => s + Math.hypot(x.cat.px - x.det.x, x.cat.py - x.det.y), 0);
          if (!best || score > best.score) best = { score, pose, n: m.length };
        }
  if (!best || best.n < 4) return null;

  const anchorsOf = (m) => m.map((p) => ({ px: p.det.x, py: p.det.y, g: p.cat.g }));
  const solveOn = (m, seed) => solvePoseAnchors(anchorsOf(m), natW, natH, seed.az, seed.el, { roll: seed.roll, fov: seed.fov, k: seed.k });
  const residDeg = (m, pose) => m.map((p) => {
    const d = pixToDirK(p.det.x, p.det.y, natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k);
    return Math.acos(Math.min(1, Math.max(-1, dot(d, p.cat.g)))) * R2D;
  });
  const median = (a) => { const s = [...a].sort((x, y) => x - y); return s.length ? s[s.length >> 1] : 0; };
  const inlierFloor = opts.inlierDeg || 0.4;
  /* keep only correspondences whose residual is near the consensus — a UFO,
     satellite, plane, or a cloud-hidden star mis-matched to a nearby blob sits
     far off and is dropped, so it can't bias the least-squares pose */
  const trim = (m, pose) => {
    if (m.length < 5) return m;
    const res = residDeg(m, pose), thr = Math.max(2.5 * median(res), inlierFloor);
    const keep = m.filter((_, i) => res[i] <= thr);
    return keep.length >= 4 ? keep : m;
  };

  let pose = best.pose, inliers = [];
  const iters = opts.iters || 10;
  for (let it = 0; it < iters; it++) {
    const tol = (0.06 - (0.06 - 0.016) * (it / (iters - 1))) * natW;
    let m = greedyMatch(inFrameCatalog(cat, natW, natH, pose), det, tol);
    if (m.length < 4) return null;
    let sol = solveOn(m, pose);
    const keep = trim(m, sol);                     // robust: reject outlier matches
    if (keep.length < m.length) { sol = solveOn(keep, sol); m = keep; }
    pose = { az: sol.az, el: sol.el, roll: sol.roll, fov: sol.fov, k: sol.k };
    inliers = m;
  }

  /* final gate on INLIERS only — clouds hiding some stars just means fewer
     inliers, which is fine as long as enough survive */
  let m = greedyMatch(inFrameCatalog(cat, natW, natH, pose), det, (opts.finalTol || 0.02) * natW);
  m = trim(m, pose);
  if (m.length < (opts.minMatch || 5)) return null;
  const rms = Math.sqrt(residDeg(m, pose).reduce((s, r) => s + r * r, 0) / m.length);
  if (rms > (opts.maxRms || 1.2)) return null;
  return { az: pose.az, el: pose.el, roll: pose.roll, fov: pose.fov, k: pose.k, rms, n: m.length };
}
