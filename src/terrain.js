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
   Returns { els: Float32Array(N_AZ) degrees, dists: Float32Array(N_AZ) m,
             ridges: [{pts: [[azDeg, elDeg], …], dist: medianM}] } — ridges
   are the VISIBLE interior crest lines below the silhouette: a crest is
   emitted only where nearer terrain hasn't already covered it AND the
   ground behind it drops ≥ RIDGE_PROM below its sight-line before higher
   terrain rises past it, i.e. exactly the depth edges a photo shows.
   Occluded stretches emit nothing, so layering comes out for free. */
const RIDGE_PROM = 0.25; // deg a crest must stand above what's behind it
export function skylineFromSampler(sampleEN, h0, maxDistM = 35000, minDistM = 200) {
  /* Start the march past the immediate FOREGROUND. The DEM tile is ~19 m/px
     (z13) / ~76 m/px (z11), so samples within a couple hundred metres are
     resolution noise, not the horizon this feature calibrates against. That
     noise only ever changes the skyline when it spikes ABOVE eye level at
     close range — e.g. a coastal observer whose own beach reads 3 m higher
     than their pixel, producing a fake 2–3° "berm" over open ocean (field
     report). For normal terrain the far horizon already wins, so this is a
     targeted fix, not a general change. 200 m ≈ a few coarse-grid pixels.
     BUILDINGS override this: an OSM footprint is a real, sharp object metres
     away — the house across the street IS the silhouette — so the urban
     composite passes a small minDistM to march the near field the DEM skip
     was hiding. On flat ground near samples read below the horizon and never
     raise the skyline, so lowering the floor is safe off the coast. */
  const dists = [];
  for (let d = minDistM; d < maxDistM; d *= 1.06) dists.push(d);
  /* clamp to sea level: DEM tiles carry BATHYMETRY (negative sea-floor depth)
     over oceans, but what you SEE is the water surface at 0 m. Using the raw
     depths made the running-max skyline pick the shallowest-relative sea floor
     per azimuth — a bumpy, too-low fake ridge over open water. Below-sea-level
     land (rare, inland) reads as 0 too; an acceptable trade for correct coasts. */
  const eye = Math.max(0, h0) + EYE_M;
  const els = new Float32Array(N_AZ), ridgeD = new Float32Array(N_AZ);
  const cols = new Array(N_AZ);
  for (let i = 0; i < N_AZ; i++) {
    const a = i * AZ_STEP * D2R, sa = Math.sin(a), ca = Math.cos(a);
    let best = -89, bd = 0;
    let pend = null, dipMin = Infinity; // last visible crest; deepest dip since
    const crests = [];
    for (const d of dists) {
      const h = sampleEN(sa * d, ca * d);
      if (h == null) continue;
      const hs = h > 0 ? h : 0; // water / below sea level → visible surface is sea level
      const el = Math.atan2(hs - eye - (d * d * (1 - K_REFR)) / (2 * RE), d) * R2D;
      if (el > best) {
        if (pend && pend.el - dipMin >= RIDGE_PROM) crests.push(pend);
        best = el; bd = d;
        pend = { el, d }; dipMin = Infinity;
      } else if (el < dipMin) dipMin = el;
    }
    els[i] = best; ridgeD[i] = bd;
    cols[i] = crests; // interior only — the final pend IS the skyline
  }
  return { els, dists: ridgeD, ridges: linkRidges(cols) };
}

/* stitch per-azimuth crest lists into ridge polylines: a crest joins the
   segment from the previous column when its depth is coherent (<1.5×
   jump) and its elevation continuous; otherwise it starts a new segment.
   Segments meeting across az 360→0 are merged; specks (<2°) dropped. */
function linkRidges(cols) {
  const segs = [];
  let open = []; // segments touching the previous column
  for (let i = 0; i < N_AZ; i++) {
    const nx = [], used = new Set();
    for (const c of cols[i]) {
      let bj = -1, bs = Infinity;
      for (let j = 0; j < open.length; j++) {
        if (used.has(j)) continue;
        const dr = Math.abs(Math.log(c.d / open[j].d)), de = Math.abs(c.el - open[j].el);
        if (dr > 0.41 || de > 1.2) continue; // 0.41 ≈ ln 1.5
        if (de + dr < bs) { bs = de + dr; bj = j; }
      }
      let seg;
      if (bj >= 0) { used.add(bj); seg = open[bj].seg; seg.pts.push([i * AZ_STEP, c.el]); seg.ds.push(c.d); }
      else { seg = { pts: [[i * AZ_STEP, c.el]], ds: [c.d], i0: i }; segs.push(seg); }
      nx.push({ seg, el: c.el, d: c.d });
    }
    open = nx;
  }
  /* merge across north: a segment ending at the last column continues one
     starting at column 0 when the endpoints line up */
  for (const e of segs) {
    if (e.merged || Math.round(e.pts[e.pts.length - 1][0] / AZ_STEP) !== N_AZ - 1) continue;
    for (const s of segs) {
      if (s === e || s.merged || s.i0 !== 0) continue;
      const de = Math.abs(e.pts[e.pts.length - 1][1] - s.pts[0][1]);
      const dr = Math.abs(Math.log(e.ds[e.ds.length - 1] / s.ds[0]));
      if (de <= 1.2 && dr <= 0.41) { e.pts = e.pts.concat(s.pts); e.ds = e.ds.concat(s.ds); s.merged = true; break; }
    }
  }
  return segs.filter((s) => !s.merged && s.pts.length >= 5).map((s) => {
    const ds = s.ds.slice().sort((a, b) => a - b);
    return { pts: s.pts, dist: ds[ds.length >> 1] };
  });
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

/* DEM ground field for an observer: resolves to
     { sampleEN(eastM, northM) → surface height (m, MSL) | null off-grid,
       h0 → observer ground height, mLat, mLon }
   Exported (and cached) so other calibration layers — e.g. buildings.js —
   can composite added heights ON TOP of the ground and reuse
   skylineFromSampler unchanged, in the SAME ENU frame. The mLat/mLon
   equirectangular scales are handed out so those layers place their
   footprints on exactly this field. */
const demCache = new Map();
export function demSampler(lat, lon) {
  const key = `${lat.toFixed(3)},${lon.toFixed(3)}`;
  if (demCache.has(key)) return demCache.get(key);
  const p = (async () => {
    // z11 7×7 (~±49 km ground at mid-latitudes) so DISTANT prominent summits
    // are captured — the Cascades sit 35–45 km from Bend and fell just outside
    // the old 5×5 (~±35 km) grid, so the skyline never rose to them.
    const [fine, coarse] = await Promise.all([loadGrid(lat, lon, 13, 1), loadGrid(lat, lon, 11, 3)]);
    const mLat = 111320, mLon = 111320 * Math.max(0.2, Math.cos(lat * D2R));
    const sampleEN = (e, n) => {
      const la = lat + n / mLat, lo = lon + e / mLon;
      const f = gridSample(fine, la, lo);
      return f != null ? f : gridSample(coarse, la, lo);
    };
    const h0 = sampleEN(0, 0);
    if (h0 == null) throw new Error("observer off the DEM grid");
    return { sampleEN, h0, mLat, mLon };
  })();
  demCache.set(key, p);
  p.catch(() => demCache.delete(key));
  return p;
}

const skyCache = new Map();
export async function predictedSkyline(lat, lon) {
  const key = `${lat.toFixed(3)},${lon.toFixed(3)}`;
  if (skyCache.has(key)) return skyCache.get(key);
  const p = (async () => {
    const { sampleEN, h0 } = await demSampler(lat, lon);
    const sk = skylineFromSampler(sampleEN, h0, 48000); // march far enough to reach 35–45 km summits
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

/* photo skyline from ImageData: for each of ~72 columns, the sky→ground
   silhouette using a "skyness" score (bright + blue-ish).
   Returns [{x, y}] in the SOURCE pixel space of the ImageData, outlier-
   rejected by a windowed median. Null if too few clean columns.

   Not "strongest edge" — that loses to high-contrast FOREGROUND (a tree
   canopy against bright sky out-gradients a hazy distant ridge, so the
   detector locked onto branches and the snap couldn't match any azimuth).
   Instead: the LOWEST sky-above/ground-below boundary that has sustained
   ground beneath it. Everything below the true horizon is terrain, so
   foliage hanging in the sky (which has sky BELOW it) is skipped, while a
   column fully occluded by a near tree yields a too-high point that the
   windowed median rejects as an outlier. The threshold is per-column
   adaptive (from that column's own bright/dark span) so it survives
   varied lighting rather than a fixed contrast floor. */
export function detectSkyline(im, W, H) {
  const px = im.data ? im.data : im;
  const sky = (x, y) => {
    const i = (y * W + x) * 4;
    const r = px[i], g = px[i + 1], b = px[i + 2];
    return 0.5 * (0.299 * r + 0.587 * g + 0.114 * b) + 0.9 * (b - r);
  };
  const NC = 72, WIN = 3, BAND = Math.max(4, Math.round(H * 0.05));
  const yT = Math.round(H * 0.04) + WIN, yB = Math.round(H * 0.92) - WIN;
  const pts = [];
  for (let ci = 0; ci < NC; ci++) {
    const x = Math.round((0.04 + (0.92 * ci) / (NC - 1)) * (W - 1));
    /* column's own sky/ground range → adaptive threshold */
    let smax = -1e9, smin = 1e9;
    for (let y = yT; y <= yB; y++) { const s = sky(x, y); if (s > smax) smax = s; if (s < smin) smin = s; }
    if (smax - smin < 40) continue; // no real sky/ground contrast here
    const thr = smin + 0.45 * (smax - smin);
    const meanAbove = (y) => { let a = 0; for (let k = 1; k <= WIN; k++) a += sky(x, y - k); return a / WIN; };
    const meanBelow = (y) => { let a = 0; for (let k = 1; k <= WIN; k++) a += sky(x, y + k); return a / WIN; };
    let found = -1;
    for (let y = yB; y >= yT; y--) {
      if (meanAbove(y) < thr || meanBelow(y) >= thr) continue; // need sky over ground
      let gnd = 0, tot = 0;
      for (let k = 1; k <= BAND && y + k < H; k++) { tot++; if (sky(x, y + k) < thr) gnd++; }
      if (tot > 0 && gnd / tot >= 0.8) { found = y; break; } // sustained ground below
    }
    if (found > 0) pts.push({ x, y: found });
  }
  if (pts.length < 20) return null;
  /* windowed-median outlier rejection — also culls fully-occluded columns */
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
