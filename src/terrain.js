/* ============================================================
   TERRAIN SKYLINE — the strongest calibration source in hills,
   where auto-horizon fails: compute the predicted ridge line from
   a DEM and draw it in the sky view; drag the photo until its
   ridges sit on the line and az + pitch + roll are calibrated
   simultaneously — day or night, no compass.

   Data: AWS Open Data Terrain Tiles (Terrarium PNG — free, no key,
   CORS-open (probed 2026-07-14), 3DEP/NED 10 m in the US).
     https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png
     h = R*256 + G + B/256 − 32768   (meters)
   Attribution required — surfaced in the sky-view chip title and
   report (Mapzen/Nextzen terrain tiles; USGS 3DEP, SRTM, ETOPO1 et al.)

   Geometry: for az 0→360° (0.4° step) ray-march log-spaced ground
   distances; elevation angle of the ground at distance d is
     atan2(h(d) − eye − d²(1−k)/2R, d)     k ≈ 0.13 refraction
   and the skyline is the running max. Grids: z13 3×3 (~±7 km,
   ~19 m/px) nested in z11 5×5 (~±36 km) — ~34 tiles ≈ 2–4 MB once,
   cached per observer position.
   ============================================================ */

import { D2R, R2D, RE } from "./math/geodesy.js";

export const TERRAIN_ATTRIB = "Terrain: Mapzen/AWS terrain tiles (USGS 3DEP, SRTM, ETOPO1 et al.)";

const K_REFR = 0.13, EYE_M = 1.6;
export const AZ_STEP = 0.4, N_AZ = Math.round(360 / AZ_STEP);

/* ---------- pure core (node-testable) ----------
   sampleEN(eastM, northM) → terrain height (m, MSL) or null off-grid.
   h0 = observer ground height (m, MSL).
   Returns { els: Float32Array(N_AZ) degrees, dists: Float32Array(N_AZ) m } */
export function skylineFromSampler(sampleEN, h0, maxDistM = 35000) {
  const dists = [];
  for (let d = 40; d < maxDistM; d *= 1.06) dists.push(d);
  const eye = h0 + EYE_M;
  const els = new Float32Array(N_AZ), ridgeD = new Float32Array(N_AZ);
  for (let i = 0; i < N_AZ; i++) {
    const a = i * AZ_STEP * D2R, sa = Math.sin(a), ca = Math.cos(a);
    let best = -89, bd = 0;
    for (const d of dists) {
      const h = sampleEN(sa * d, ca * d);
      if (h == null) continue;
      const el = Math.atan2(h - eye - (d * d * (1 - K_REFR)) / (2 * RE), d) * R2D;
      if (el > best) { best = el; bd = d; }
    }
    els[i] = best; ridgeD[i] = bd;
  }
  return { els, dists: ridgeD };
}

/* interpolate a skyline elevation at an arbitrary azimuth */
export function skylineElAt(els, azDeg) {
  const x = (((azDeg % 360) + 360) % 360) / AZ_STEP;
  const i = Math.floor(x), f = x - i;
  return els[i % N_AZ] * (1 - f) + els[(i + 1) % N_AZ] * f;
}

/* ---------- slippy-tile math ---------- */
const tx = (lon, z) => ((lon + 180) / 360) * (1 << z);
const ty = (lat, z) => {
  const r = lat * D2R;
  return ((1 - Math.log(Math.tan(r) + 1 / Math.cos(r)) / Math.PI) / 2) * (1 << z);
};

/* ---------- browser loader: tiles → Float32 heightfield grids ---------- */
async function loadGrid(lat, lon, z, half) {
  const cx = Math.floor(tx(lon, z)), cy = Math.floor(ty(lat, z));
  const n = 2 * half + 1, size = n * 256;
  const cv = document.createElement("canvas");
  cv.width = size; cv.height = size;
  const ctx = cv.getContext("2d", { willReadFrequently: true });
  const jobs = [];
  for (let dy = -half; dy <= half; dy++)
    for (let dx = -half; dx <= half; dx++) {
      const url = `https://s3.amazonaws.com/elevation-tiles-prod/terrarium/${z}/${cx + dx}/${cy + dy}.png`;
      jobs.push(new Promise((res) => {
        const im = new Image();
        im.crossOrigin = "anonymous";
        im.onload = () => { ctx.drawImage(im, (dx + half) * 256, (dy + half) * 256); res(true); };
        im.onerror = () => res(false);
        im.src = url;
      }));
    }
  const got = await Promise.all(jobs);
  if (!got.some(Boolean)) throw new Error("no terrain tiles reachable");
  const px = ctx.getImageData(0, 0, size, size).data;
  const h = new Float32Array(size * size);
  for (let i = 0, j = 0; i < h.length; i++, j += 4) h[i] = px[j] * 256 + px[j + 1] + px[j + 2] / 256 - 32768;
  return { h, size, z, x0: cx - half, y0: cy - half };
}

function gridSample(g, lat, lon) {
  const px = (tx(lon, g.z) - g.x0) * 256, py = (ty(lat, g.z) - g.y0) * 256;
  if (px < 0.5 || py < 0.5 || px >= g.size - 1.5 || py >= g.size - 1.5) return null;
  const x = Math.floor(px), y = Math.floor(py), fx = px - x, fy = py - y;
  const i = y * g.size + x;
  return g.h[i] * (1 - fx) * (1 - fy) + g.h[i + 1] * fx * (1 - fy) +
    g.h[i + g.size] * (1 - fx) * fy + g.h[i + g.size + 1] * fx * fy;
}

/* ---------- public API (cached per ~100 m of observer position) ---------- */
const skyCache = new Map();
export async function predictedSkyline(lat, lon) {
  const key = `${lat.toFixed(3)},${lon.toFixed(3)}`;
  if (skyCache.has(key)) return skyCache.get(key);
  const p = (async () => {
    const [fine, coarse] = await Promise.all([loadGrid(lat, lon, 13, 1), loadGrid(lat, lon, 11, 2)]);
    const mLat = 111320, mLon = 111320 * Math.max(0.2, Math.cos(lat * D2R));
    const sampleEN = (e, n) => {
      const la = lat + n / mLat, lo = lon + e / mLon;
      const f = gridSample(fine, la, lo);
      return f != null ? f : gridSample(coarse, la, lo);
    };
    const h0 = sampleEN(0, 0);
    if (h0 == null) throw new Error("observer off the DEM grid");
    const sk = skylineFromSampler(sampleEN, h0);
    return { ...sk, h0 };
  })();
  skyCache.set(key, p);
  p.catch(() => skyCache.delete(key));
  return p;
}

/* ============================================================
   SKYLINE SNAP (v2) — one-tap absolute pose from the ridges.
   detectSkyline finds the photo's sky/ground silhouette as a polyline;
   matchSkyline cross-correlates it against the DEM skyline over an
   azimuth scan, solving pitch (intercept) and roll (slope vs horizontal
   image offset) by least squares at each candidate. This is also the
   video plan's keyframe anchor.
   ============================================================ */

/* photo skyline from ImageData: for each of ~72 columns, the strongest
   sky→ground transition of a "skyness" score (bright + blue-ish).
   Returns [{x, y}] in the SOURCE pixel space of the ImageData, outlier-
   rejected by a windowed median. Null if too few clean columns. */
export function detectSkyline(im, W, H) {
  const px = im.data ? im.data : im;
  const sky = (x, y) => {
    const i = (y * W + x) * 4;
    const r = px[i], g = px[i + 1], b = px[i + 2];
    return 0.5 * (0.299 * r + 0.587 * g + 0.114 * b) + 0.9 * (b - r);
  };
  const NC = 72, WIN = 3;
  const pts = [];
  for (let ci = 0; ci < NC; ci++) {
    const x = Math.round((0.04 + (0.92 * ci) / (NC - 1)) * (W - 1));
    let bestY = -1, bestG = 0;
    for (let y = Math.round(H * 0.04) + WIN; y < Math.round(H * 0.92) - WIN; y++) {
      let above = 0, below = 0;
      for (let k = 1; k <= WIN; k++) { above += sky(x, y - k); below += sky(x, y + k); }
      const gdt = above - below;
      if (gdt > bestG) { bestG = gdt; bestY = y; }
    }
    if (bestY > 0 && bestG > 90) pts.push({ x, y: bestY });
  }
  if (pts.length < 20) return null;
  /* windowed-median outlier rejection */
  const keep = pts.filter((p, i) => {
    const win = pts.slice(Math.max(0, i - 3), i + 4).map((q) => q.y).sort((a, b) => a - b);
    const med = win[Math.floor(win.length / 2)];
    return Math.abs(p.y - med) < Math.max(6, 0.04 * H);
  });
  return keep.length >= 20 ? keep : null;
}

/* Cross-correlate photo skyline directions against the DEM skyline.
   samples: [{az, el, thx}] — az/el of each detected skyline point under
   the CURRENT pose; thx = horizontal offset from image center (radians).
   elAt(az) — DEM skyline elevation. Returns the pose correction
   {dAz, dEl, dRollDeg, rms, n} minimizing the residual after fitting
   el-offset (intercept) and roll (slope in thx) at each azimuth shift. */
export function matchSkyline(samples, elAt) {
  if (!samples || samples.length < 12) return null;
  const fitAt = (dAz) => {
    /* least squares r_i = a + b·thx_i over residuals r_i = elAt(az+dAz) − el */
    let sw = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (const s of samples) {
      const r = elAt(s.az + dAz) - s.el;
      sw++; sx += s.thx; sy += r; sxx += s.thx * s.thx; sxy += s.thx * r;
    }
    const den = sw * sxx - sx * sx;
    const b = Math.abs(den) > 1e-9 ? (sw * sxy - sx * sy) / den : 0;
    const a = (sy - b * sx) / sw;
    let r2 = 0;
    for (const s of samples) {
      const e = (elAt(s.az + dAz) - s.el) - (a + b * s.thx);
      r2 += e * e;
    }
    return { a, b, rms: Math.sqrt(r2 / sw) };
  };
  let best = null;
  for (let dAz = -180; dAz < 180; dAz += 0.5) {
    const f = fitAt(dAz);
    if (!best || f.rms < best.rms) best = { dAz, ...f };
  }
  for (let dAz = best.dAz - 0.6; dAz <= best.dAz + 0.6; dAz += 0.05) {
    const f = fitAt(dAz);
    if (f.rms < best.rms) best = { dAz, ...f };
  }
  /* roll tilts the horizon by roll_deg·thx(rad) degrees of elevation:
     the residual slope b (deg/rad) = −roll_deg */
  return { dAz: best.dAz, dEl: best.a, dRollDeg: -best.b, rms: best.rms, n: samples.length };
}

/* single-point DEM elevation (one z13 tile — for "use terrain elevation") */
const tileCache = new Map();
export async function demElevation(lat, lon) {
  const cx = Math.floor(tx(lon, 13)), cy = Math.floor(ty(lat, 13));
  const key = `13/${cx}/${cy}`;
  if (!tileCache.has(key)) {
    tileCache.set(key, (async () => {
      const cv = document.createElement("canvas");
      cv.width = 256; cv.height = 256;
      const ctx = cv.getContext("2d", { willReadFrequently: true });
      await new Promise((res, rej) => {
        const im = new Image();
        im.crossOrigin = "anonymous";
        im.onload = () => { ctx.drawImage(im, 0, 0); res(); };
        im.onerror = () => rej(new Error("terrain tile unreachable"));
        im.src = `https://s3.amazonaws.com/elevation-tiles-prod/terrarium/13/${cx}/${cy}.png`;
      });
      const px = ctx.getImageData(0, 0, 256, 256).data;
      const h = new Float32Array(65536);
      for (let i = 0, j = 0; i < h.length; i++, j += 4) h[i] = px[j] * 256 + px[j + 1] + px[j + 2] / 256 - 32768;
      return { h, size: 256, z: 13, x0: cx, y0: cy };
    })());
    tileCache.get(key).catch(() => tileCache.delete(key));
  }
  const g = await tileCache.get(key);
  const v = gridSample(g, lat, lon);
  if (v == null) throw new Error("point off tile");
  return v;
}
