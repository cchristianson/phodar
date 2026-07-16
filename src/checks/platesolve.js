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

import { R2D, D2R, dot, unit, clampN, dirToAzEl } from "../math/geodesy.js";
import { photoBasis, dirToPixK, pixToDirK, solvePoseAnchors } from "../math/projection.js";

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

/* ---- SEEDLESS solve (asterism matching) — no manual placement needed ----
   Angular distances between stars are invariant to how the camera points/rolls,
   so we match the PATTERN of bright dots to the catalog directly. With the FOV
   known (EXIF) and the visible catalog known (location+time), a two-star
   correspondence pins a full 3-DOF rotation; we verify each hypothesis by how
   many other catalog stars then land on detected blobs (RANSAC-style), and hand
   the winning pose to autoStarAlign to polish. Real star-tracker "lost-in-space"
   method — the human never has to get it close. */
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const angBetween = (a, b) => Math.acos(Math.min(1, Math.max(-1, dot(a, b))));
const camVec = (px, py, natW, natH, fpx) => unit([(px - natW / 2) / fpx, (natH / 2 - py) / fpx, 1]);
/* orthonormal frame from two directions (first axis = v1) */
function frame2(v1, v2) {
  const E1 = unit(v1);
  const p = dot(v2, E1);
  const E2 = unit([v2[0] - p * E1[0], v2[1] - p * E1[1], v2[2] - p * E1[2]]);
  return [E1, E2, cross(E1, E2)];
}
/* rotation rows (world→camera) mapping world w1,w2 onto camera u1,u2.
   Row i is the WORLD direction that becomes camera axis i (x=right,y=up,z=fwd). */
function rot2(u1, u2, w1, w2) {
  const E = frame2(u1, u2), F = frame2(w1, w2);
  const row = (i) => [
    E[0][i] * F[0][0] + E[1][i] * F[1][0] + E[2][i] * F[2][0],
    E[0][i] * F[0][1] + E[1][i] * F[1][1] + E[2][i] * F[2][1],
    E[0][i] * F[0][2] + E[1][i] * F[1][2] + E[2][i] * F[2][2],
  ];
  return [row(0), row(1), row(2)];
}
export function blindStarAlign(det, cat, natW, natH, fovGuess, opts = {}) {
  if (!det || det.length < 5 || !cat || cat.length < 5 || !(fovGuess > 0)) return null;
  const cx = natW / 2, cy = natH / 2;
  const dts = det.slice(0, opts.nd || 14);                                        // brightest detected blobs
  const cts = cat.slice().sort((a, b) => (a.mag ?? 9) - (b.mag ?? 9)).slice(0, opts.nc || 80); // brightest catalog
  const angTol = (opts.angTol || 0.7) * D2R;                                      // loose: tolerates lens distortion at the seed stage
  const tolPx = (opts.matchTol || 0.022) * natW;
  const minInl = opts.minInl || 6;
  /* catalog pairwise angular distances (pose-invariant), sorted for a range scan */
  const catPairs = [];
  for (let a = 0; a < cts.length; a++) for (let b = a + 1; b < cts.length; b++) catPairs.push({ a, b, d: angBetween(cts[a].g, cts[b].g) });
  catPairs.sort((p, q) => p.d - q.d);
  const cpD = catPairs.map((p) => p.d);
  const lb = (x) => { let lo = 0, hi = cpD.length; while (lo < hi) { const m = (lo + hi) >> 1; if (cpD[m] < x) lo = m + 1; else hi = m; } return lo; };

  /* inlier count for a full pose (basis once, then project every catalog star) */
  const countInliers = (az, el, roll, fov) => {
    const b = photoBasis(az, el, roll);
    const fpx = (natW / 2) / Math.tan((fov * D2R) / 2);
    let inl = 0;
    for (const c of cts) {
      const gf = c.g[0] * b.f[0] + c.g[1] * b.f[1] + c.g[2] * b.f[2];
      if (gf <= 0.03) continue;
      const px = cx + (c.g[0] * b.r[0] + c.g[1] * b.r[1] + c.g[2] * b.r[2]) / gf * fpx;
      const py = cy - (c.g[0] * b.u[0] + c.g[1] * b.u[1] + c.g[2] * b.u[2]) / gf * fpx;
      if (px < 0 || px > natW || py < 0 || py > natH) continue;
      for (const s of dts) { if (Math.abs(px - s.x) < tolPx && Math.abs(py - s.y) < tolPx) { inl++; break; } }
    }
    return inl;
  };

  const nPair = Math.min(opts.nPairSeed || 7, dts.length);
  let best = null;
  for (const ff of (opts.fovFactors || [0.8, 0.9, 1.0, 1.12, 1.28, 1.45])) {
    const fov = fovGuess * ff;
    const fpx = (natW / 2) / Math.tan((fov * D2R) / 2);
    const bvec = dts.map((s) => camVec(s.x, s.y, natW, natH, fpx));
    const detR = bvec.map((v) => Math.acos(Math.min(1, Math.max(-1, v[2]))));     // each blob's angular radius from centre (roll-invariant)
    for (let i = 0; i < nPair; i++) for (let j = i + 1; j < nPair; j++) {
      const dij = angBetween(bvec[i], bvec[j]);
      for (let idx = lb(dij - angTol); idx < catPairs.length && cpD[idx] <= dij + angTol; idx++) {
        const cp = catPairs[idx];
        for (const asg of [[cp.a, cp.b], [cp.b, cp.a]]) {
          /* a 2-star match fixes the CENTRE reliably; roll is ill-constrained
             (esp. for stars near the centre) so we don't trust it — sweep it */
          const center = rot2(bvec[i], bvec[j], cts[asg[0]].g, cts[asg[1]].g)[2];
          const ae = dirToAzEl(center);
          if (!(ae.el > -30 && ae.el < 91)) continue;
          /* roll-invariant pre-score: how many catalog stars sit at a radius
             from the centre that some blob also has? prunes junk centres cheaply */
          let pre = 0;
          for (const c of cts) { const cr = angBetween(center, c.g); for (const dr of detR) if (Math.abs(cr - dr) < angTol) { pre++; break; } }
          if (pre < minInl) continue;
          for (let roll = -180; roll < 180; roll += 8) {              // coarse roll sweep + verify
            const inl = countInliers(ae.az, ae.el, roll, fov);
            if (!best || inl > best.inl) best = { inl, az: ae.az, el: ae.el, roll, fov };
          }
        }
      }
    }
  }
  if (!best || best.inl < minInl) return null;
  for (let roll = best.roll - 8; roll <= best.roll + 8; roll += 1) {              // fine roll
    const inl = countInliers(best.az, best.el, roll, best.fov);
    if (inl >= best.inl) { best.inl = inl; best.roll = roll; }
  }
  /* hand the seedless hypothesis to the robust ICP solver to polish + fit k */
  return autoStarAlign(det, cat, natW, natH, { az: best.az, el: clampN(best.el, -20, 89.5), roll: best.roll, fov: best.fov, k: 0 }, opts);
}
