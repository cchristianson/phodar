/* ============================================================
   VIDEO POSE TRACKING — per-frame camera pose from background features.
   The foundation of world-locked ("stabilized") video: solve each frame's
   camera pose (az/el/roll, FOV pinned) so the sky/terrain/stars stay fixed on
   the dome and only the object moves along its true angular path.

   The insight that makes this cheap: once the REFERENCE frame is aligned to the
   dome (its pose is known), every background pixel becomes a FIXED world
   direction via `pixToDirK`. So each later frame is exactly the star-align
   problem — {pixel ↔ known world dir} correspondences into `solvePoseAnchors`
   with the moving object (and drifting clouds) rejected as outliers, the same
   median-residual trim `autoStarAlign` uses. No new pose math; this module only
   adds the feature detection + template tracking that feed it.

   Pure + frame-agnostic (the DOM only ever supplies pixel buffers). Tested in
   scripts/mathcheck.js against synthesized rotating frames.
   ============================================================ */

import { D2R, R2D, dot, clampN, dirToAzEl } from "../math/geodesy.js";
import { pixToDirK, dirToPixK, solvePoseAnchors } from "../math/projection.js";
import { detectStars } from "../checks/platesolve.js";

/* ---------- feature detection (day AND night) ---------- */

const gray = (data, w, h) => {
  const g = new Float32Array(w * h);
  for (let i = 0; i < w * h; i++) g[i] = 0.299 * data[i * 4] + 0.587 * data[i * 4 + 1] + 0.114 * data[i * 4 + 2];
  return g;
};

/* daytime corners — per-cell peak of gradient energy (gx²+gy²), min-separated.
   Lands features on ridgelines, tree edges and rooftops — the static structure a
   daytime clip has instead of stars. Returns [{x,y,score}] strongest-first. */
function dayCorners(data, w, h, o) {
  const g = gray(data, w, h);
  const en = new Float32Array(w * h);
  for (let y = 1; y < h - 1; y++) for (let x = 1; x < w - 1; x++) {
    const i = y * w + x, gx = g[i + 1] - g[i - 1], gy = g[i + w] - g[i - w];
    en[i] = gx * gx + gy * gy;
  }
  const cell = o.cell || 32, cols = Math.ceil(w / cell), rows = Math.ceil(h / cell);
  const cand = [];
  for (let cy = 0; cy < rows; cy++) for (let cx = 0; cx < cols; cx++) {
    const x0 = cx * cell, y0 = cy * cell, x1 = Math.min(w - 1, x0 + cell), y1 = Math.min(h - 1, y0 + cell);
    let bx = -1, by = -1, be = 0;
    for (let y = y0; y < y1; y++) for (let x = x0; x < x1; x++) {
      const e = en[y * w + x];
      if (e > be && !(o.inExcl && o.inExcl(x, y))) { be = e; bx = x; by = y; }
    }
    if (bx >= 0 && be > (o.minEnergy || 4)) cand.push({ x: bx, y: by, score: be });
  }
  cand.sort((a, b) => b.score - a.score);
  const out = [], sep = o.minSep || 8;
  for (const c of cand) {
    if (out.some((p) => Math.hypot(p.x - c.x, p.y - c.y) < sep)) continue;
    out.push(c); if (out.length >= (o.maxN || 60)) break;
  }
  return out;
}

/* Pick trackable static background features from a frame buffer (RGBA).
   'night' → bright star blobs (detectStars); 'day' → gradient corners;
   'auto' → BOTH combined (stars first, corners fill the remaining slots) — a
   dusk clip with a few stars still gets its tree line / ridge / rooftops as
   references, and a day clip with specular blobs loses nothing. `excludeRect`
   {x0,y0,x1,y1} (buffer px) drops features inside the object ROI. */
export function detectBgFeatures(data, w, h, opts = {}) {
  const mode = opts.mode || "auto", maxN = opts.maxN || 60, excl = opts.excludeRect;
  const inExcl = excl ? (x, y) => x >= excl.x0 && x <= excl.x1 && y >= excl.y0 && y <= excl.y1 : null;
  const stars = mode !== "day"
    ? detectStars(data, w, h, { maxN: maxN * 2 }).filter((s) => !(inExcl && inExcl(s.x, s.y))).slice(0, maxN).map((s) => ({ x: s.x, y: s.y, score: s.bright }))
    : [];
  if (mode === "night") return stars;
  const out = mode === "auto" ? [...stars] : [];
  const sep = opts.minSep || 8;
  for (const c of dayCorners(data, w, h, { ...opts, maxN, inExcl })) {
    if (out.length >= maxN) break;
    if (out.some((p) => Math.hypot(p.x - c.x, p.y - c.y) < sep)) continue;
    out.push(c);
  }
  return out;
}

/* ---------- template tracking (normalized cross-correlation) ---------- */

/* zero-mean NCC of a (2hp+1)² patch: template centered at (tx,ty) in `gt`
   vs a candidate patch centered at (cx,cy) in `gc`. Returns -2 out of bounds. */
const bilin = (g, w, h, x, y) => {
  const x0 = Math.floor(x), y0 = Math.floor(y);
  if (x0 < 0 || y0 < 0 || x0 >= w - 1 || y0 >= h - 1) return null;
  const fx = x - x0, fy = y - y0, i = y0 * w + x0;
  return g[i] * (1 - fx) * (1 - fy) + g[i + 1] * fx * (1 - fy) + g[i + w] * (1 - fx) * fy + g[i + w + 1] * fx * fy;
};

/* Track features from `prevData` into `nextData`. Each feature carries a
   template center (tx,ty) in prevData and a predicted search center (px,py) in
   nextData. Returns [{px,py,ncc,ok}] — refined (sub-pixel) positions in
   nextData; ok = ncc ≥ minNcc. A flat/low-texture template returns ok:false.
   `opts.tScale` (zoom support): how much BIGGER the scene appears in nextData
   than in prevData — the template is bilinear-resampled from prev at step
   1/tScale so it matches the next frame's scale before correlating. */
export function trackFeatures(prevData, nextData, w, h, feats, opts = {}) {
  const P = opts.patch || 15, S = opts.search || 12, minNcc = opts.minNcc == null ? 0.6 : opts.minNcc;
  const tScale = opts.tScale || 1;
  const hp = P >> 1, n = (2 * hp + 1) * (2 * hp + 1);
  const gt = gray(prevData, w, h), gc = gray(nextData, w, h);
  const out = [];
  for (const f of feats) {
    const cx = Math.round(f.px), cy = Math.round(f.py);
    // extract the template (scale-resampled) and its stats; reject flat patches
    const T = new Float32Array(n);
    let ok = true, tMean = 0, k2 = 0;
    outer: for (let dy = -hp; dy <= hp; dy++) for (let dx = -hp; dx <= hp; dx++) {
      const v = bilin(gt, w, h, f.tx + dx / tScale, f.ty + dy / tScale);
      if (v == null) { ok = false; break outer; }
      T[k2++] = v; tMean += v;
    }
    let tInv = 0;
    if (ok) {
      tMean /= n;
      let tVar = 0; for (let i = 0; i < n; i++) { const d = T[i] - tMean; tVar += d * d; }
      if (tVar < (opts.minVar || 25)) ok = false; else tInv = 1 / Math.sqrt(tVar);
    }
    if (!ok) { out.push({ px: f.px, py: f.py, ncc: 0, ok: false }); continue; }
    const nccC = (px0, py0) => { // zero-mean NCC of T vs the next-frame patch centred at an integer pixel
      if (px0 - hp < 0 || py0 - hp < 0 || px0 + hp >= w || py0 + hp >= h) return -2;
      let cMean = 0;
      for (let dy = -hp; dy <= hp; dy++) for (let dx = -hp; dx <= hp; dx++) cMean += gc[(py0 + dy) * w + (px0 + dx)];
      cMean /= n;
      let num = 0, cVar = 0, ti = 0;
      for (let dy = -hp; dy <= hp; dy++) for (let dx = -hp; dx <= hp; dx++) {
        const c = gc[(py0 + dy) * w + (px0 + dx)] - cMean, t = T[ti++] - tMean;
        num += t * c; cVar += c * c;
      }
      return cVar > 1e-6 ? num * tInv / Math.sqrt(cVar) : 0;
    };
    let best = -2, bx = cx, by = cy;
    for (let dy = -S; dy <= S; dy++) for (let dx = -S; dx <= S; dx++) {
      const v = nccC(cx + dx, cy + dy);
      if (v > best) { best = v; bx = cx + dx; by = cy + dy; }
    }
    // parabolic sub-pixel refine on the NCC peak (x and y independently)
    let sx = bx, sy = by;
    const nx0 = nccC(bx - 1, by), nx1 = nccC(bx + 1, by), ny0 = nccC(bx, by - 1), ny1 = nccC(bx, by + 1);
    if (nx0 > -2 && nx1 > -2) { const d = nx0 - 2 * best + nx1; if (Math.abs(d) > 1e-6) sx = bx + clampN(0.5 * (nx0 - nx1) / d, -1, 1); }
    if (ny0 > -2 && ny1 > -2) { const d = ny0 - 2 * best + ny1; if (Math.abs(d) > 1e-6) sy = by + clampN(0.5 * (ny0 - ny1) / d, -1, 1); }
    out.push({ px: sx, py: sy, ncc: best, ok: best >= minNcc });
  }
  return out;
}

/* ---------- robust pose from tracked correspondences ---------- */

const median = (a) => { const s = [...a].sort((x, y) => x - y); return s.length ? s[s.length >> 1] : 0; };
const residDeg = (anchors, natW, natH, p) => anchors.map((a) =>
  Math.acos(clampN(dot(pixToDirK(a.px, a.py, natW, natH, p.az, p.el, p.roll, p.fov, p.k || 0), a.g), -1, 1)) * R2D);

/* Solve one frame's pose from background correspondences [{px,py,g}] (native
   px, g = fixed world dir from the reference frame). Mirrors autoStarAlign's
   median-residual trim so the moving object / drifting clouds are rejected. FOV
   and k are pinned to the seed by default (fixed-zoom assumption). Returns
   {az,el,roll,fov,k,rms,n} or null when it can't hold a lock. */
export function poseFromTracks(anchors, natW, natH, seedPose, opts = {}) {
  const minMatch = opts.minMatch || 6;
  if (!anchors || anchors.length < minMatch) return null;
  const seed = { roll: seedPose.roll || 0, fov: seedPose.fov, k: seedPose.k || 0, lockFov: opts.lockFov !== false, lockK: opts.lockK !== false };
  let keep = anchors;
  let sol = solvePoseAnchors(keep, natW, natH, seedPose.az, seedPose.el, seed);
  for (let pass = 0; pass < 2; pass++) {
    const res = residDeg(keep, natW, natH, sol);
    const thr = Math.max(2.5 * median(res), opts.inlierDeg || 0.4);
    const nk = keep.filter((_, i) => res[i] <= thr);
    if (nk.length >= minMatch && nk.length < keep.length) { keep = nk; sol = solvePoseAnchors(keep, natW, natH, seedPose.az, seedPose.el, seed); }
    else break;
  }
  const res = residDeg(keep, natW, natH, sol);
  const rms = Math.sqrt(res.reduce((s, r) => s + r * r, 0) / keep.length);
  if (keep.length < minMatch || rms > (opts.maxRms || 1.5)) return null;
  return { az: sol.az, el: sol.el, roll: sol.roll, fov: sol.fov, k: sol.k, rms, n: keep.length };
}

/* ---------- global registration against the reference frame ----------
   Differential tracking cannot survive self-similar scenes under zoom: every
   feature finds a lookalike patch near its prediction, the scale change is
   masked, and the pose goes self-consistently wrong (field-observed
   repeatedly). The robust PRIMARY is global: match the whole downsampled
   frame against the reference across an explicit scale ladder — whole-frame
   structure cannot alias the way local patches do, and the answer is
   ABSOLUTE (reference-anchored), so drift is impossible by construction. */

export function grayDown(data, w, h, W2) {  // RGBA → box-downsampled gray W2×H2
  const H2 = Math.max(8, Math.round(h * W2 / w));
  const g = new Float32Array(W2 * H2);
  for (let y = 0; y < H2; y++) for (let x = 0; x < W2; x++) {
    const x0 = Math.floor(x * w / W2), x1 = Math.max(x0 + 1, Math.floor((x + 1) * w / W2));
    const y0 = Math.floor(y * h / H2), y1 = Math.max(y0 + 1, Math.floor((y + 1) * h / H2));
    let s = 0, n = 0;
    for (let yy = y0; yy < y1; yy++) for (let xx = x0; xx < x1; xx++) { const i = (yy * w + xx) * 4; s += 0.299 * data[i] + 0.587 * data[i + 1] + 0.114 * data[i + 2]; n++; }
    g[x + y * W2] = s / n;
  }
  return { g, w: W2, h: H2 };
}

/* zero-mean NCC of a small template swept over an image; returns best {x,y,ncc} */
function nccSweep(im, tp) {
  const { g, w, h } = im, { g: t, w: tw, h: th } = tp;
  const n = tw * th;
  let tMean = 0; for (let i = 0; i < n; i++) tMean += t[i];
  tMean /= n;
  let tVar = 0; for (let i = 0; i < n; i++) { const d = t[i] - tMean; tVar += d * d; }
  if (tVar < 1e-3) return { x: 0, y: 0, ncc: -1 };
  const tInv = 1 / Math.sqrt(tVar);
  let best = -2, bx = 0, by = 0;
  for (let oy = 0; oy <= h - th; oy++) for (let ox = 0; ox <= w - tw; ox++) {
    let cMean = 0;
    for (let y = 0; y < th; y++) for (let x = 0; x < tw; x++) cMean += g[(oy + y) * w + (ox + x)];
    cMean /= n;
    let num = 0, cVar = 0;
    for (let y = 0; y < th; y++) for (let x = 0; x < tw; x++) {
      const c = g[(oy + y) * w + (ox + x)] - cMean, d = t[y * tw + x] - tMean;
      num += d * c; cVar += c * c;
    }
    const v = cVar > 1e-3 ? num * tInv / Math.sqrt(cVar) : 0;
    if (v > best) { best = v; bx = ox; by = oy; }
  }
  return { x: bx, y: by, ncc: best };
}

/* bilinear shrink of a gray image by factor k (k>1 ⇒ output is k× smaller) */
function shrinkGray(im, k) {
  const tw = Math.max(6, Math.round(im.w / k)), th = Math.max(6, Math.round(im.h / k));
  const g = new Float32Array(tw * th);
  for (let y = 0; y < th; y++) for (let x = 0; x < tw; x++) {
    const sx = clampN((x + 0.5) * k - 0.5, 0, im.w - 1.001), sy = clampN((y + 0.5) * k - 0.5, 0, im.h - 1.001);
    const x0 = Math.floor(sx), y0 = Math.floor(sy), fx = sx - x0, fy = sy - y0, i = y0 * im.w + x0;
    g[y * tw + x] = im.g[i] * (1 - fx) * (1 - fy) + im.g[i + 1] * fx * (1 - fy) + im.g[i + im.w] * (1 - fx) * fy + im.g[i + im.w + 1] * fx * fy;
  }
  return { g, w: tw, h: th };
}

/* Register a frame against the reference across a scale ladder. `tracker`
   carries refG (coarse gray of the reference) + ref.pose. `curData` is the
   frame at tracking resolution. Returns {s, fov, az, el, score} — the frame's
   ABSOLUTE center pose and zoom (roll assumed ≈ reference; the sparse refine
   recovers it) — or null when the frame no longer overlaps the reference well
   enough (pan-away → caller falls back to the differential chain). */
export function registerToRef(tracker, curG, opts = {}) {
  const refG = tracker.refG; if (!refG) return null;
  const { natW, natH } = tracker;
  const rp = tracker.ref.pose;
  const minScore = opts.minScore == null ? 0.5 : opts.minScore;
  /* physical FOV cap: the camera can never see WIDER than the lens's widest
     (digital zoom only narrows). Without it the s<1 rungs — whose templates
     are the SMALLEST and therefore decorrelate least under handheld
     roll/parallax mismatch — win at the zoom-out landing and report an
     impossible 110–127° FOV (field-observed: the s=0.7 rung beating truth
     s≈1 exactly when the clip returns to full wide). */
  const fovCap = opts.fovMax || (tracker.opts && tracker.opts.fovMax) || 150;
  const sMin = Math.tan((rp.fov * D2R) / 2) / Math.tan((Math.min(150, fovCap) * D2R) / 2);
  /* the template is a CENTRAL CROP (80%) of the moving frame, so every rung —
     including s≈1 — has translation freedom. Without the crop, the s=1
     template is the whole frame (zero slide room) and a handheld PAN makes a
     slightly-shrunk rung score better, biasing the scale (field-observed as a
     steady ~5% FOV error while un-zoomed). */
  const crop = (im, f) => {
    const tw = Math.round(im.w * f), th = Math.round(im.h * f);
    const x0 = (im.w - tw) >> 1, y0 = (im.h - th) >> 1;
    const g = new Float32Array(tw * th);
    for (let y = 0; y < th; y++) for (let x = 0; x < tw; x++) g[y * tw + x] = im.g[(y0 + y) * im.w + (x0 + x)];
    return { g, w: tw, h: th };
  };
  /* global registration is only meaningful on AREA-TEXTURED content (terrain,
     foliage, clouds, buildings). A sparse POINT field (stars) at 96 px is a
     handful of sub-pixel dots: the honest scale's correlation dies of
     sub-pixel misalignment while an aliased scale can land dots-on-dots and
     win confidently. Point content is precisely what the differential feature
     chain is best at — hand it over. Gate: the fraction of coarse pixels
     carrying signal must look like area texture, not isolated dots. */
  const areaFrac = (im) => {
    let m = 0; for (let i = 0; i < im.g.length; i++) m += im.g[i];
    m /= im.g.length;
    let act = 0;
    for (let i = 0; i < im.g.length; i++) if (Math.abs(im.g[i] - m) > 6) act++;
    return act / im.g.length;
  };
  if (areaFrac(refG) < 0.12 || areaFrac(curG) < 0.12) return null;
  /* ROLL COMPENSATION: the sweep assumes the frame sits at the reference's
     roll — a handheld tilt of ~5°+ decorrelates the whole-frame NCC (field-
     observed: the clip's rolled tail lost every global lock, or worse, a
     tilted frame's best correlation was a FALSE deep-zoom match). When the
     chain's roll estimate differs from the reference, also try the frame
     de-rotated by that hint and keep whichever scores better. The matched
     CENTER is rotation-invariant (de-rotation is about the center), so the
     az/el/fov mapping below is unchanged; roll itself stays owned by the
     sparse solve. */
  const rollHint = opts.rollHint || 0;
  const rotG = (im, deg) => {
    const c = Math.cos(deg * D2R), s = Math.sin(deg * D2R);
    const cx = (im.w - 1) / 2, cy = (im.h - 1) / 2;
    const g = new Float32Array(im.w * im.h);
    for (let y = 0; y < im.h; y++) for (let x = 0; x < im.w; x++) {
      const dx = x - cx, dy = y - cy;
      const sx = clampN(cx + dx * c - dy * s, 0, im.w - 1.001), sy = clampN(cy + dx * s + dy * c, 0, im.h - 1.001);
      const x0 = Math.floor(sx), y0 = Math.floor(sy), fx = sx - x0, fy = sy - y0, i = y0 * im.w + x0;
      g[y * im.w + x] = im.g[i] * (1 - fx) * (1 - fy) + im.g[i + 1] * fx * (1 - fy) + im.g[i + im.w] * (1 - fx) * fy + im.g[i + im.w + 1] * fx * fy;
    }
    return { g, w: im.w, h: im.h };
  };
  const curGs = Math.abs(rollHint) > 1.5 ? [curG, rotG(curG, rollHint)] : [curG];
  let out = null;
  for (const curGi of curGs) {
    const r = sweepLadder(curGi);
    if (r && (!out || r.score > out.score)) out = r;
  }
  return out;
  function sweepLadder(curG) {
  const curC = crop(curG, 0.8), refC = crop(refG, 0.8);
  const rungs = [];
  for (let s = 0.7; s <= 5.05; s *= 1.115) if (s >= sMin - 1e-9) rungs.push(+s.toFixed(4));
  if (!rungs.length) return null;
  const tryS = (s) => {
    if (s >= 1) {              // zoomed IN vs ref: cur = magnified sub-region of ref
      const tp = shrinkGray(curC, s);
      if (tp.w >= refG.w || tp.h >= refG.h) return null;
      const m = nccSweep(refG, tp);
      if (m.ncc <= -1) return null;
      return { s, ncc: m.ncc, cx: m.x + tp.w / 2, cy: m.y + tp.h / 2 };
    }
    // zoomed OUT vs ref: ref content is a sub-region of cur
    const tp = shrinkGray(refC, 1 / s);
    if (tp.w >= curG.w || tp.h >= curG.h) return null;
    const m = nccSweep(curG, tp);
    if (m.ncc <= -1) return null;
    // ref-crop center found at (m.x,m.y) in cur → map cur's center into ref coords
    return { s, ncc: m.ncc, cx: refG.w / 2 + (curG.w / 2 - (m.x + tp.w / 2)) / s, cy: refG.h / 2 + (curG.h / 2 - (m.y + tp.h / 2)) / s };
  };
  const res = rungs.map(tryS);
  let bi = -1;
  for (let i = 0; i < res.length; i++) if (res[i] && (bi < 0 || res[i].ncc > res[bi].ncc)) bi = i;
  if (bi < 0 || res[bi].ncc < minScore) return null;
  let best = res[bi];
  /* sub-rung scale: parabolic fit over log-s using the neighbouring rungs */
  const lo = res[bi - 1], hi = res[bi + 1];
  let sFine = best.s;
  if (lo && hi) {
    const d = lo.ncc - 2 * best.ncc + hi.ncc;
    if (Math.abs(d) > 1e-9) {
      const off = clampN(0.5 * (lo.ncc - hi.ncc) / d, -1, 1);
      sFine = best.s * Math.pow(1.115, off);
    }
  }
  /* matched center (refG coords) → native ref pixel → absolute world dir */
  const px = (best.cx / refG.w) * natW, py = (best.cy / refG.h) * natH;
  const gDir = pixToDirK(px, py, natW, natH, rp.az, rp.el, rp.roll, rp.fov, rp.k || 0);
  const ae = dirToAzEl(gDir);
  const fov = clampN(2 * Math.atan(Math.tan((rp.fov * D2R) / 2) / sFine) * R2D, 5, fovCap);
  return { s: sFine, fov, az: ae.az, el: ae.el, score: best.ncc };
  }
}

/* ---------- orchestration: seed on the reference frame, step per frame ---------- */

/* Seed the tracker from the aligned reference frame. `refData` is the frame
   buffer at tracking resolution (w×h); (natW,natH) the native photo size;
   `refPose` {az,el,roll,fov,k} the frame's solved pose. Each detected feature's
   world direction is FROZEN from refPose — that's the per-frame "catalog". */
export function initTracker(refData, w, h, natW, natH, refPose, opts = {}) {
  const sc = natW / w;                               // tracking-buffer px → native px
  const seeds = detectBgFeatures(refData, w, h, opts);
  const features = seeds.map((f, i) => ({
    id: i, prime: true,                              // prime = seeded on the ALIGNED reference frame (uncontaminated g)
    g: pixToDirK(f.x * sc, f.y * sc, natW, natH, refPose.az, refPose.el, refPose.roll, refPose.fov, refPose.k || 0),
    tx: f.x, ty: f.y, px: f.x, py: f.y,
  }));
  return {
    w, h, natW, natH, sc, prevData: refData, lastPose: { ...refPose }, features, nextId: features.length, opts,
    /* the reference frame is kept forever: every frame globally registers
       against it (zoom + drift proof), and near its scale frames re-anchor
       feature-precisely against it too (see stepTracker) */
    ref: { data: refData, pose: { ...refPose }, feats: features.map((f) => ({ g: f.g, tx: f.tx, ty: f.ty })) },
    refG: grayDown(refData, w, h, 96),
  };
}

/* Advance the tracker onto `nextData` (adjacent frame). Predicts each feature's
   pixel from the last pose, tracks it, solves the new pose (outliers trimmed),
   then re-acquires features that dropped out (their world dir re-projected under
   the new pose) and tops up when too few survive — so the sky keeps tracking
   through a pan. Mutates the tracker. Returns {pose, nInliers, features}. */
export function stepTracker(tracker, nextData, opts = {}) {
  const { w, h, natW, natH, sc } = tracker;
  const o = { ...tracker.opts, ...opts }, p0 = tracker.lastPose;
  const minMatch = o.minMatch || 6, maxN = o.maxN || 60, sep = o.minSep || 8;
  const fovCap = o.fovMax || 150;   // physical lens-widest cap (see registerToRef)
  // 0. GLOBAL registration against the reference — the zoom-proof, drift-proof
  //    coarse pose. When it locks, it seeds the predictions (so the sparse
  //    layer starts at the right scale and place); when the frame has panned
  //    off the reference coverage it returns null and the differential chain
  //    below carries the step alone.
  let glob = null;
  if (tracker.refG && o.global !== false) {
    glob = registerToRef(tracker, grayDown(nextData, w, h, 96), { rollHint: (p0.roll || 0) - (tracker.ref.pose.roll || 0) });
    /* CONTINUITY GATE: a handheld camera can't teleport between adjacent
       samples (≤¼ s). A global fix far from the chain is a wrong-placement
       match — at deep zoom the template is tiny and self-similar content
       offers lookalike placements (field-observed: a 30° az excursion held
       for 1.5 s mid-zoom, and an 11° one at a zoom-out landing). Reject it;
       the chain carries the frame and later good fixes/anchors absorb any
       small drift. The costs are asymmetric — a rejected TRUE fix only
       delays re-anchoring, an accepted FALSE one poisons samples — so the
       gate is tight. */
    if (glob) {
      const dAzG = Math.abs(((glob.az - p0.az + 540) % 360) - 180);
      if (dAzG > 8 || Math.abs(glob.el - p0.el) > 10) glob = null;
    }
    if (glob) glob.pose = { az: glob.az, el: glob.el, roll: p0.roll, fov: glob.fov, k: p0.k || 0 };
  }
  const basePose = glob ? glob.pose : p0;
  const tScale0 = glob ? clampN(Math.tan((p0.fov * D2R) / 2) / Math.tan((glob.fov * D2R) / 2), 0.25, 4) : 1; // appearance scale prev→next per the global fix
  // 1. predict search centers from the base pose (drop features now off-frame)
  const feats = [];
  for (const ft of tracker.features) {
    const p = dirToPixK(ft.g, natW, natH, basePose.az, basePose.el, basePose.roll, basePose.fov, basePose.k || 0);
    if (!p) continue;
    const bx = p.px / sc, by = p.py / sc;
    if (bx < -8 || bx > w + 8 || by < -8 || by > h + 8) continue;
    feats.push({ ...ft, px: bx, py: by });
  }
  // 2. track templates from prevData into nextData (scale-matched when zoomed)
  const tracked = trackFeatures(tracker.prevData, nextData, w, h, feats, { ...o, tScale: tScale0 });
  /* shared scale machinery: probeAt scores a feature subset under one zoom
     hypothesis (templates resampled + positions re-predicted under its FOV);
     adoptScale re-tracks EVERYTHING under a factor and adopts the result when
     it beats what plain tracking found. */
  /* two probe subsets for two jobs: the per-step scale probe wants EDGE-BIASED
     features (zoom displacement grows with radius — that's the signal); the
     collapse rescue wants RADIUS-DIVERSITY, because under a violent zoom-in
     only the central features remain in-frame under the candidate FOVs */
  const byR = [...feats].sort((a, b) =>
    Math.hypot(b.px - w / 2, b.py - h / 2) - Math.hypot(a.px - w / 2, a.py - h / 2));
  const subEdge = byR.slice(0, 12);
  const subMix = [];
  for (let i = 0, j = byR.length - 1; subMix.length < Math.min(12, byR.length) && i <= j; i++, j--) {
    subMix.push(byR[i]); if (i < j && subMix.length < 12) subMix.push(byR[j]);
  }
  const probeAt = (probeSub, sh, search) => {
    const fovS = clampN(2 * Math.atan(Math.tan((p0.fov * D2R) / 2) / sh) * R2D, 5, fovCap);
    const f2 = [];
    for (const ft of probeSub) {
      const p = dirToPixK(ft.g, natW, natH, p0.az, p0.el, p0.roll, fovS, p0.k || 0);
      if (!p) continue;
      const bx = p.px / sc, by = p.py / sc;
      if (bx < 0 || bx > w || by < 0 || by > h) continue;
      f2.push({ ...ft, px: bx, py: by });
    }
    if (f2.length < 3) return { score: -1, okn: 0, pts: [], scatter: 1e9 };
    const tr = trackFeatures(tracker.prevData, nextData, w, h, f2, { ...o, search, tScale: sh });
    let score = 0, okn = 0; const pts = [];
    for (let i = 0; i < f2.length; i++) if (tr[i].ok) { score += tr[i].ncc; okn++; pts.push([f2[i], tr[i]]); }
    /* radial-fit COHERENCE: a true scale mismatch shows as a LINEAR radial
       residual (slope = s_true/sh − 1, fitted out); false lookalike matches
       land wherever the nearest neighbour sits — random scatter around any
       line. The rms about the fit separates them where raw residual size
       can't (rung quantization looks like noise otherwise). */
    let scatter = 1e9;
    if (okn >= 4) {
      const rs = [], es = [];
      for (const [fi, ti] of pts) {
        const rx = fi.px - w / 2, ry = fi.py - h / 2, rr = Math.hypot(rx, ry) || 1;
        rs.push(rr); es.push(((ti.px - fi.px) * rx + (ti.py - fi.py) * ry) / rr);
      }
      const n2 = rs.length;
      let sr = 0, se = 0, srr = 0, sre = 0;
      for (let i = 0; i < n2; i++) { sr += rs[i]; se += es[i]; srr += rs[i] * rs[i]; sre += rs[i] * es[i]; }
      const den = n2 * srr - sr * sr;
      const b = Math.abs(den) > 1e-6 ? (n2 * sre - sr * se) / den : 0;
      const a = (se - b * sr) / n2;
      let v = 0; for (let i = 0; i < n2; i++) { const d = es[i] - (a + b * rs[i]); v += d * d; }
      scatter = Math.sqrt(v / n2);
    }
    return { score, okn, pts, scatter };
  };
  const pairRefine = (pts, sh) => {           // matched pairs' true distance ratio beats the ladder's coarse factor
    const rat = [];
    for (let i = 0; i < pts.length; i++) for (let j = i + 1; j < pts.length; j++) {
      const d0 = Math.hypot(pts[i][0].tx - pts[j][0].tx, pts[i][0].ty - pts[j][0].ty);
      if (d0 < 20) continue;
      rat.push(Math.hypot(pts[i][1].px - pts[j][1].px, pts[i][1].py - pts[j][1].py) / d0);
    }
    if (rat.length < 3) return sh;
    rat.sort((x, y) => x - y); return rat[rat.length >> 1];
  };
  const adoptScale = (sR) => {
    const fovR = clampN(2 * Math.atan(Math.tan((p0.fov * D2R) / 2) / sR) * R2D, 5, fovCap);
    const fAll = [], iAll = [];
    for (let i = 0; i < feats.length; i++) {
      const p = dirToPixK(feats[i].g, natW, natH, p0.az, p0.el, p0.roll, fovR, p0.k || 0);
      if (!p) continue;
      const bx = p.px / sc, by = p.py / sc;
      if (bx < -8 || bx > w + 8 || by < -8 || by > h + 8) continue;
      fAll.push({ ...feats[i], px: bx, py: by }); iAll.push(i);
    }
    if (fAll.length < 3) return;
    const trAll = trackFeatures(tracker.prevData, nextData, w, h, fAll, { ...o, search: 10, tScale: sR });
    let okn2 = 0; for (const t2 of trAll) if (t2.ok) okn2++;
    let okNow = 0; for (const t2 of tracked) if (t2.ok) okNow++;
    /* "not clearly fewer" — the caller already established the scale is right;
       false in-place matches can inflate okNow, so demanding strictly more
       would wrongly reject the truth */
    if (okn2 + 2 >= okNow && okn2 >= 3) {
      for (let i = 0; i < feats.length; i++) tracked[i] = { px: feats[i].px, py: feats[i].py, ncc: 0, ok: false };
      for (let k3 = 0; k3 < iAll.length; k3++) tracked[iAll[k3]] = trAll[k3];
    }
  };
  // 2a-i. SCALE PROBE (every step) — self-similar texture (foliage, clouds)
  //     FALSELY matches "in place" during a zoom: each feature finds a
  //     lookalike patch near its old spot, so the zoom is masked (s reads ≈1
  //     and the pose goes self-consistently wrong at the old FOV, field-
  //     observed). Appearance + COHERENCE decide instead: under the TRUE scale
  //     hypothesis the matches land tightly on their predictions; false
  //     lookalike matches land wherever the nearest neighbour sits — a large,
  //     incoherent median residual. A hypothesis that's markedly more coherent
  //     than s=1 is zoom evidence — refine it from the pairs and re-track all.
  if (!glob && feats.length >= 4) {   // differential fallback only — the global fix already owns scale
    const s1 = probeAt(subEdge, 1, 10);
    /* only probe when s=1 still HAS a population (the masking scenario);
       an outright collapse belongs to the wide-ladder rescue below */
    if (s1.okn >= 4) {
      let bestP = null;
      for (const sh of [0.78, 0.86, 0.93, 1.08, 1.16, 1.26]) {
        const rp = probeAt(subEdge, sh, 10);
        if (rp.okn >= 4 && (!bestP || rp.scatter < bestP.scatter)) bestP = { sh, ...rp };
      }
      if (bestP && bestP.scatter + 1.2 < s1.scatter) adoptScale(pairRefine(bestP.pts, bestP.sh));
    }
  }
  // 2a-ii. FAST-ZOOM RESCUE — when tracking has COLLAPSED outright (a violent
  //     zoom: predictions overshoot the search AND templates rescale so NCC
  //     finds nothing anywhere), sweep a wide factor ladder and adopt the one
  //     that makes the background reappear.
  let okIdx0 = 0; for (const t of tracked) if (t.ok) okIdx0++;
  if (!glob && okIdx0 < Math.max(minMatch, Math.ceil(feats.length * 0.3)) && feats.length >= 4) {
    const LADDER = [0.35, 0.45, 0.55, 0.67, 0.8, 1.25, 1.5, 1.8, 2.15, 2.6];
    let best = null;
    for (const sh of LADDER) {
      const rp = probeAt(subMix, sh, 9);
      if (rp.okn >= 3 && (!best || rp.score > best.score)) best = { sh, ...rp };
    }
    if (best) adoptScale(pairRefine(best.pts, best.sh));
  }
  // 2b. ZOOM detection — pairwise-distance scale between the tracked features'
  //     previous and current pixels. Rotation/pan preserve those distances;
  //     zoom SCALES them, so a median ratio s≠1 means the lens zoomed. The
  //     predictions above used the old FOV, so during a zoom the edge features
  //     overshoot the NCC search window and fail — re-predict the failures
  //     under the scale-corrected FOV and give them a second try.
  const okIdx = [];
  for (let i = 0; i < feats.length; i++) if (tracked[i].ok) okIdx.push(i);
  let s = 1;
  if (okIdx.length >= 3) {
    const rat = [], n = Math.min(okIdx.length, 16);
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) {
      const a = feats[okIdx[i]], b = feats[okIdx[j]];
      const d0 = Math.hypot(a.tx - b.tx, a.ty - b.ty);
      if (d0 < 20) continue; // tiny baselines amplify pixel noise
      rat.push(Math.hypot(tracked[okIdx[i]].px - tracked[okIdx[j]].px, tracked[okIdx[i]].py - tracked[okIdx[j]].py) / d0);
    }
    if (rat.length >= 3) { rat.sort((x, y) => x - y); s = rat[rat.length >> 1]; }
  }
  const zoom = Math.abs(s - 1) > 0.015;
  const fovZ = clampN(2 * Math.atan(Math.tan((p0.fov * D2R) / 2) / s) * R2D, 5, fovCap); // spread apart (s>1) = zoomed in = narrower FOV
  if (zoom) {
    const missI = [], missF = [];
    for (let i = 0; i < feats.length; i++) if (!tracked[i].ok) {
      const p = dirToPixK(feats[i].g, natW, natH, basePose.az, basePose.el, basePose.roll, fovZ, basePose.k || 0);
      if (!p) continue;
      const bx = p.px / sc, by = p.py / sc;
      if (bx < -8 || bx > w + 8 || by < -8 || by > h + 8) continue;
      missI.push(i); missF.push({ ...feats[i], px: bx, py: by });
    }
    if (missF.length) {
      const tr2 = trackFeatures(tracker.prevData, nextData, w, h, missF, { ...o, tScale: s });
      for (let k2 = 0; k2 < missI.length; k2++) if (tr2[k2].ok) tracked[missI[k2]] = tr2[k2];
    }
  }
  // 3. inlier correspondences → native px anchors (g fixed)
  const anchors = [];
  for (let i = 0; i < feats.length; i++) if (tracked[i].ok) anchors.push({ px: tracked[i].px * sc, py: tracked[i].py * sc, g: feats[i].g });
  // 4. solve (keep the previous pose if we can't lock). Under zoom evidence the
  //    FOV seed comes from the scale estimate; with plenty of anchors the FOV is
  //    freed for the solver to polish, else it locks at the estimated value —
  //    a sparse frame is never allowed to wander FOV on its own.
  let pose = basePose, nInliers = anchors.length, solved = false;
  if (anchors.length >= minMatch) {
    const seed2 = zoom ? { ...basePose, fov: fovZ } : basePose;
    /* FOV is freed for the polish whenever the anchors give it real leverage
       (plentiful + radially spread) — the global fix and re-anchoring guard
       against drift, and the coarse global scale is rung-quantized, so the
       sparse solve is what restores sub-degree FOV. A sparse or centre-only
       frame still keeps FOV locked to the seed. */
    let rMaxA = 0;
    for (const a of anchors) rMaxA = Math.max(rMaxA, Math.hypot(a.px - natW / 2, a.py - natH / 2));
    const fovFree = (anchors.length >= 10 && rMaxA > 0.35 * Math.min(natW, natH)) || (zoom && anchors.length >= 8);
    const sol = poseFromTracks(anchors, natW, natH, seed2, { ...o, lockFov: !fovFree });
    /* physical sanity: no handheld step (≤¼ s) rolls the camera ~10°+. A wild
       roll on thin evidence is a garbage solve from a blurred frame — reject
       it (field-observed as a single frame snapping 25° off mid-zoom-out). */
    if (sol && !(Math.abs(sol.roll - basePose.roll) > 10 && sol.n < 12)) {
      pose = { az: sol.az, el: sol.el, roll: sol.roll, fov: Math.min(sol.fov, fovCap), k: sol.k }; nInliers = sol.n; solved = true;
    }
  }
  // 4b. ABSOLUTE RE-ANCHOR — incremental tracking drifts through feature
  //     TURNOVER: replacements get their world dir from the current pose
  //     estimate, baking in its error (a zoom episode churns many features).
  //     Whenever the current zoom is near enough the reference frame's scale
  //     that its templates are recognizable, match the PRISTINE reference
  //     features directly into this frame and re-solve absolutely — zeroing
  //     the accumulated drift instead of carrying it forward.
  let anchored = false, drift = null;
  const ref = tracker.ref;
  if (ref && (anchors.length >= minMatch || glob) && o.reanchor !== false) {
    const tS = Math.tan((ref.pose.fov * D2R) / 2) / Math.tan((pose.fov * D2R) / 2); // scene scale: current vs reference
    if (tS > 0.65 && tS < 1.55) {
      const fR = [];
      for (const rf of ref.feats.slice(0, 20)) {
        const p = dirToPixK(rf.g, natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k || 0);
        if (!p) continue;
        const bx = p.px / sc, by = p.py / sc;
        if (bx < 0 || bx > w || by < 0 || by > h) continue;
        fR.push({ g: rf.g, tx: rf.tx, ty: rf.ty, px: bx, py: by });
      }
      if (fR.length >= minMatch) {
        const trR = trackFeatures(ref.data, nextData, w, h, fR, { ...o, search: 12, tScale: tS });
        const ancR = [];
        for (let i = 0; i < fR.length; i++) if (trR[i].ok) ancR.push({ px: trR[i].px * sc, py: trR[i].py * sc, g: fR[i].g });
        if (ancR.length >= Math.max(minMatch, 8)) {
          /* the anchor owns ANGULAR drift — not zoom. FOV is only freed when
             the matches have real radial leverage (mid-zoom the outer refs are
             off-frame, and a central-only cluster lets a free-FOV solve slide
             FOV back toward the reference on weak evidence, flattening the
             tracked zoom step by step). Otherwise FOV stays locked to the
             incremental estimate — the scale detector owns the zoom. */
          let rMax = 0;
          for (const a of ancR) rMax = Math.max(rMax, Math.hypot(a.px / sc - w / 2, a.py / sc - h / 2));
          /* FOV from the anchor only near native template scale — heavy
             resampling (deep zoom) biases the matches by ~a degree, worse
             than the sparse polish it would override */
          const fovFree = rMax > 0.35 * Math.min(w, h) && ancR.length >= 12 && tS > 0.77 && tS < 1.3;
          const solR = poseFromTracks(ancR, natW, natH, pose, { ...o, lockFov: !fovFree, maxRms: 0.5 });
          if (solR) {
            const dA = ((solR.az - pose.az + 540) % 360) - 180, dE = solR.el - pose.el, dR = solR.roll - pose.roll, dF = solR.fov - pose.fov;
            /* re-anchoring fixes SMALL drift; a big disagreement means the
               anchor itself mis-matched (self-similar texture) — reject it and
               keep the chain rather than snapping to a wrong absolute */
            if (Math.abs(dA) <= 3 && Math.abs(dE) <= 3 && Math.abs(dR) <= 4 && Math.abs(dF) <= Math.max(2.5, pose.fov * 0.1)) {
              drift = { dAz: dA, dEl: dE, dRoll: dR, dFov: dF };
              pose = { az: solR.az, el: solR.el, roll: solR.roll, fov: Math.min(solR.fov, fovCap), k: solR.k };
              nInliers = solR.n; anchored = true;
            }
          }
        }
      }
    }
  }
  // 5. update templates (successful → new pixel), re-acquire the rest under the new pose
  const kept = [];
  for (let i = 0; i < feats.length; i++) {
    const ft = feats[i];
    if (tracked[i].ok) { kept.push({ ...ft, tx: tracked[i].px, ty: tracked[i].py, px: tracked[i].px, py: tracked[i].py }); continue; }
    const p = dirToPixK(ft.g, natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k || 0);
    if (p) { const bx = p.px / sc, by = p.py / sc; if (bx >= 0 && bx < w && by >= 0 && by < h) kept.push({ ...ft, tx: bx, ty: by, px: bx, py: by }); }
  }
  // 6. top up when the herd thins (features left the frame during a pan).
  //    ONLY when this step actually SOLVED: topping up while the pose is held
  //    bakes the held pose's error into every new feature's world dir — an
  //    unresolved zoom then rebuilds the whole population at the wrong FOV and
  //    the tracker goes self-consistently blind to it (field-observed).
  if ((solved || glob) && kept.length < minMatch + 4) {
    for (const f of detectBgFeatures(nextData, w, h, { ...o, maxN })) {
      if (kept.some((k) => Math.hypot(k.tx - f.x, k.ty - f.y) < sep)) continue;
      kept.push({ id: tracker.nextId++, prime: false, g: pixToDirK(f.x * sc, f.y * sc, natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k || 0), tx: f.x, ty: f.y, px: f.x, py: f.y });
      if (kept.length >= maxN) break;
    }
  }
  // a successful anchor purges contamination: non-prime features (whose g was
  // assigned from a possibly-drifted estimate) get their world dir re-derived
  // under the corrected pose
  if (anchored) for (const f of kept) if (!f.prime) f.g = pixToDirK(f.tx * sc, f.ty * sc, natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k || 0);
  tracker.prevData = nextData; tracker.lastPose = pose; tracker.features = kept;
  /* held = neither the sparse solve nor the global register placed this frame
     — the pose is the PREVIOUS frame's, frozen. Known-wrong whenever the
     camera kept moving; callers can bridge such frames by interpolation. */
  return { pose, nInliers, features: kept, scale: s, anchored, drift, global: glob ? +glob.score.toFixed(2) : null, held: !solved && !glob };
}

/* Interpolate a pose path at any time t — az wrap-aware, everything else
   linear. Clamps to the ends. Used by stabilized playback/export to give
   every video frame a pose even between solved samples. */
export function posePathAt(path, t) {
  if (!path || !path.length) return null;
  if (t <= path[0].t) return { ...path[0], t };
  const last = path[path.length - 1];
  if (t >= last.t) return { ...last, t };
  let lo = 0, hi = path.length - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (path[m].t <= t) lo = m; else hi = m; }
  const a = path[lo], b = path[hi], u = (t - a.t) / Math.max(1e-9, b.t - a.t);
  const dAz = ((b.az - a.az + 540) % 360) - 180;
  return {
    t,
    az: (((a.az + dAz * u) % 360) + 360) % 360,
    el: a.el + (b.el - a.el) * u,
    roll: (a.roll || 0) + ((b.roll || 0) - (a.roll || 0)) * u,
    fov: a.fov + (b.fov - a.fov) * u,
    k: (a.k || 0) + ((b.k || 0) - (a.k || 0)) * u,
    n: Math.min(a.n == null ? 99 : a.n, b.n == null ? 99 : b.n),
  };
}

/* DESPIKE a solved pose path: a sample that deviates sharply from its
   neighbours' time-interpolation — while the neighbours agree with each
   other, or far beyond their own gap — is a garbage solve from one blurred
   frame, not camera motion (a real whip/zoom is a RAMP across samples, which
   this preserves: ramp neighbours disagree, so the gates stay closed).
   Low-evidence entries (n < 6) are despiked at half the threshold. Mutates
   path in place; returns the number of corrected samples. */
export function despikePath(path, opts = {}) {
  const angD = (a, b) => ((a - b + 540) % 360) - 180;
  let fixed = 0;
  for (let pass = 0; pass < (opts.passes || 2); pass++) {
    for (let i = 1; i < path.length - 1; i++) {
      const a = path[i - 1], b = path[i], c = path[i + 1];
      const span = c.t - a.t; if (!(span > 1e-6)) continue;
      const u = (b.t - a.t) / span;
      const iAz = a.az + angD(c.az, a.az) * u;
      const iEl = a.el + (c.el - a.el) * u;
      const iRoll = a.roll + (c.roll - a.roll) * u;
      const iFov = a.fov + (c.fov - a.fov) * u;
      const nAz = Math.abs(angD(c.az, a.az)), nEl = Math.abs(c.el - a.el), nRoll = Math.abs(c.roll - a.roll);
      const nFovR = Math.abs(c.fov - a.fov) / Math.max(c.fov, a.fov);
      const k = (b.n || 0) < 6 ? 0.5 : 1;               // weak frames yield sooner
      const devAz = Math.abs(angD(b.az, iAz)), devEl = Math.abs(b.el - iEl), devRoll = Math.abs(b.roll - iRoll);
      const devFovR = Math.abs(b.fov - iFov) / Math.max(b.fov, iFov);
      let did = false;
      if (devAz > Math.max(1.2 * k, 1.6 * nAz)) { b.az = +(((iAz % 360) + 360) % 360).toFixed(3); did = true; }
      if (devEl > Math.max(1.0 * k, 1.6 * nEl)) { b.el = +iEl.toFixed(3); did = true; }
      if (devRoll > Math.max(1.5 * k, 1.6 * nRoll)) { b.roll = +iRoll.toFixed(3); did = true; }
      if (devFovR > Math.max(0.06 * k, 1.6 * nFovR)) { b.fov = +iFov.toFixed(2); did = true; }
      if (did) fixed++;
    }
  }
  return fixed;
}

/* Distribute a re-anchor's drift correction back across the un-anchored span
   ("meet in the middle"): entries between the last anchor (at time ancT) and
   the anchored entry at kIdx get the correction scaled by their time fraction,
   so the incremental chain bends smoothly onto the absolute fix instead of
   snapping. Mutates path[segFrom..kIdx-1]. */
export function smearDrift(path, segFrom, kIdx, ancT, dr) {
  const span = Math.abs(path[kIdx].t - ancT);
  if (!(span > 1e-6)) return;
  for (let i = segFrom; i < kIdx; i++) {
    const f = clampN(Math.abs(path[i].t - ancT) / span, 0, 1);
    path[i].az = +((((path[i].az + f * dr.dAz) % 360) + 360) % 360).toFixed(3);
    path[i].el = +(path[i].el + f * dr.dEl).toFixed(3);
    path[i].roll = +(path[i].roll + f * dr.dRoll).toFixed(3);
    path[i].fov = +clampN(path[i].fov + f * dr.dFov, 5, 150).toFixed(2);
  }
}
