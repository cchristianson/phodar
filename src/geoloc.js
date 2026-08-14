/* ============================================================
   Skyline geolocation ("📍 Find my spot") — pure core.

   Locates a photo/video with stripped metadata inside a user-chosen
   search area by matching the DEM terrain skyline against every frame
   at once, optionally anchored by user-pinned STRUCTURES (water tanks,
   masts) matched to OSM twins.

   Proven on a real 91 s Himalaya clip before productizing, and two
   findings from those four harness rounds are load-bearing here:

   1. Silhouette shape alone is often NOT decisive — four sweeps over a
      28 km box separated best from median by only ~1.1×. sweepVerdict
      exists so the UI says "terrain can't decide alone" instead of
      inventing a pin on the map. What DID discriminate was a pinned
      landmark's bearing consistency (2° for the true-area candidate vs
      60–160° for rivals), which is why pins are first-class here.
   2. Atmospheric haze makes a DISTANT ridge literally sky-blue —
      terrain.js's detectSkyline (blueness-keyed, built for snap-to-
      ridges on near terrain) latches a near tree line and calls the far
      wall sky (measured: sky lum ~185 b−r ~20 vs mountain lum ~107
      b−r ~59). farSkyline below keys on LUMINANCE DEFICIT from the
      top-of-frame sky reference instead: haze-blued terrain is still
      darker than sky. It also gates on the top band actually BEING
      bright sky, so frames without clean sky are dropped rather than
      contributing a garbage curve that flattens the joint score.

   All functions are pure (DEM injected as sampleEN closures, pixels as
   RGBA arrays) except the two browser helpers at the bottom (tile
   region loader, landmark fetch), which are never called by tests.
   ============================================================ */

import { pixToDirK } from "./math/projection.js";
import { dirToAzEl } from "./math/geodesy.js";
import { skylineFromSampler, skylineElAt, loadGrid, gridSample } from "./terrain.js";

export const SWEEP_FOVS = [15, 20, 27, 36, 48]; // candidate horizontal FOVs when the lens is unknown (zoom clips vary per frame)
export const AZ_STEP = 2;                       // sweep curve resolution (deg) — the coherence window is ±40°, 2° bins are plenty
export const WIN_DEG = 40;                      // one camera pans, it doesn't teleport: all frames must point within ±this of one center

/* ---------- far-skyline detector (haze-proof) ----------
   px: RGBA array (or ImageData), W×H. Returns [{x,y}] or null.
   Scans each column TOP-DOWN for the first sustained drop below a
   threshold set from the top-band sky reference — the FAR silhouette,
   not the strongest edge. */
export function farSkyline(px, W, H, opts = {}) {
  const d = px.data ? px.data : px;
  const skyMin = opts.skyMin ?? 150;   // top band must be genuinely bright sky…
  const dropMin = opts.dropMin ?? 60;  // …and the column must contain something meaningfully darker
  const NC = opts.cols ?? 72, BAND = Math.max(6, Math.round(H * 0.04));
  const pts = [];
  for (let ci = 0; ci < NC; ci++) {
    const x = Math.round((0.04 + (0.92 * ci) / (NC - 1)) * (W - 1));
    const lum = (y) => { const i = (y * W + x) * 4; return 0.299 * d[i] + 0.587 * d[i + 1] + 0.114 * d[i + 2]; };
    const y0 = Math.round(H * 0.02), y1 = Math.round(H * 0.10);
    let skyRef = 0, n = 0;
    for (let y = y0; y <= y1; y++) { skyRef += lum(y); n++; }
    skyRef /= n;
    let mn = 1e9;
    for (let y = y0; y < H * 0.95; y++) { const l = lum(y); if (l < mn) mn = l; }
    if (skyRef < skyMin || skyRef - mn < dropMin) continue;
    const thr = skyRef - 0.35 * (skyRef - mn);
    let found = -1;
    for (let y = y1; y < H * 0.95; y++) {
      if (lum(y) >= thr) continue;
      let below = 0, tot = 0;
      for (let k = 1; k <= BAND && y + k < H; k++) { tot++; if (lum(y + k) < thr) below++; }
      if (tot && below / tot >= 0.85) { found = y; break; }
    }
    if (found > 0) pts.push({ x, y: found });
  }
  if (pts.length < 20) return null;
  const keep = pts.filter((p, i) => {
    const win = pts.slice(Math.max(0, i - 3), i + 4).map((q) => q.y).sort((a, b) => a - b);
    const med = win[Math.floor(win.length / 2)];
    return Math.abs(p.y - med) < Math.max(6, 0.05 * H);
  });
  return keep.length >= 20 ? keep : null;
}

/* skyline pixels → az/el samples under a nominal pose, one set per
   candidate FOV, each with its elevation spread (the normalizer that
   stops a narrow FOV from "winning" by having nothing to explain) */
export function skySampleSets(pts, W, H, fovs = SWEEP_FOVS) {
  return fovs.map((fov) => {
    const fpx = (W / 2) / Math.tan((fov * Math.PI) / 360);
    const ss = pts.map((p) => {
      const dd = pixToDirK(p.x, p.y, W, H, 0, 0, 0, fov, 0);
      const ae = dirToAzEl(dd);
      return { az: ae.az > 180 ? ae.az - 360 : ae.az, el: ae.el, thx: Math.atan2(p.x - W / 2, fpx) };
    });
    const mean = ss.reduce((a, s) => a + s.el, 0) / ss.length;
    const std = Math.sqrt(ss.reduce((a, s) => a + (s.el - mean) ** 2, 0) / ss.length);
    return { fov, ss, std: Math.max(0.15, std) };
  });
}

/* rms of one azimuth shift under the matchSkyline model (free elevation
   intercept + free roll slope — pitch and roll are absorbed, azimuth
   and shape are what score) */
function rmsAt(ss, elAt, dAz) {
  let sw = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (const s of ss) {
    const r = elAt(s.az + dAz) - s.el;
    sw++; sx += s.thx; sy += r; sxx += s.thx * s.thx; sxy += s.thx * r;
  }
  const den = sw * sxx - sx * sx;
  const b = Math.abs(den) > 1e-9 ? (sw * sxy - sx * sy) / den : 0;
  const a = (sy - b * sx) / sw;
  let r2 = 0;
  for (const s of ss) {
    const e = (elAt(s.az + dAz) - s.el) - (a + b * s.thx);
    r2 += e * e;
  }
  return Math.sqrt(r2 / sw);
}

/* one frame's normalized-rms-vs-pointing-azimuth curve (best FOV per bin) */
export function azCurve(sets, elAt, stepDeg = AZ_STEP) {
  const NB = Math.round(360 / stepDeg);
  const c = new Float32Array(NB).fill(1e9);
  const fi = new Uint8Array(NB);
  for (let vi = 0; vi < sets.length; vi++) {
    const v = sets[vi];
    for (let b = 0; b < NB; b++) {
      const nr = rmsAt(v.ss, elAt, b * stepDeg - 180) / v.std;
      if (nr < c[b]) { c[b] = nr; fi[b] = vi; }
    }
  }
  return { c, fi, stepDeg };
}

/* joint score over all frames: the best single pointing WINDOW that
   explains them at once. Trimmed mean (keep the best 75%) so one bad
   frame can't sink a true candidate. Returns per-frame pointing within
   the chosen window — that's what a pinned frame's bearing test uses. */
export function coherentWindow(curves, winDeg = WIN_DEG, keepFrac = 0.75) {
  if (!curves.length) return null;
  const stepDeg = curves[0].stepDeg, NB = curves[0].c.length;
  const win = Math.round(winDeg / stepDeg);
  const keepN = Math.max(Math.min(3, curves.length), Math.ceil(curves.length * keepFrac));
  let best = null;
  for (let A = 0; A < NB; A++) {
    const ms = curves.map(({ c }) => {
      let m = 1e9;
      for (let o = -win; o <= win; o++) {
        const v = c[(A + o + NB) % NB];
        if (v < m) m = v;
      }
      return m;
    }).sort((a, b) => a - b).slice(0, keepN);
    const sc = ms.reduce((a, b) => a + b, 0) / ms.length;
    if (!best || sc < best.score) best = { score: sc, A };
  }
  const az = ((best.A * stepDeg - 180) % 360 + 360) % 360;
  const frameAz = curves.map(({ c, fi }) => {
    let m = 1e9, bi = best.A;
    for (let o = -win; o <= win; o++) {
      const b = (best.A + o + NB) % NB;
      if (c[b] < m) { m = c[b]; bi = b; }
    }
    return { az: ((bi * stepDeg - 180) % 360 + 360) % 360, nrms: m, fovIdx: fi[bi] };
  });
  return { score: best.score, az, frameAz };
}

/* score one candidate position: DEM skyline there vs every frame */
export function scoreCandidate(frameSets, sampleEN, h0, opts = {}) {
  const sk = skylineFromSampler(sampleEN, h0);
  const elAt = (a) => skylineElAt(sk.els, a);
  const curves = frameSets.map((f) => azCurve(f.sets, elAt, opts.stepDeg ?? AZ_STEP));
  const w = coherentWindow(curves, opts.winDeg ?? WIN_DEG, opts.keepFrac ?? 0.75);
  return w && { ...w, h0 };
}

/* ---------- candidate generation ---------- */
export function gridCandidates(lat, lon, radKm, stepKm) {
  const mLat = 111.32, mLon = 111.32 * Math.max(0.2, Math.cos((lat * Math.PI) / 180));
  const out = [];
  for (let dy = -radKm; dy <= radKm + 1e-9; dy += stepKm)
    for (let dx = -radKm; dx <= radKm + 1e-9; dx += stepKm)
      out.push({ lat: lat + dy / mLat, lon: lon + dx / mLon });
  return out;
}

/* camera candidates anchored to a landmark twin: the twin itself plus a
   ring (the camera stood NEAR the structure it filmed, not on it) */
export function ringCandidates(lat, lon, ringM = 300, n = 8) {
  const mLat = 111320, mLon = 111320 * Math.max(0.2, Math.cos((lat * Math.PI) / 180));
  const out = [{ lat, lon, ringBrg: null }];
  for (let i = 0; i < n; i++) {
    const b = (360 / n) * i, r = b * Math.PI / 180;
    out.push({ lat: lat + (ringM * Math.cos(r)) / mLat, lon: lon + (ringM * Math.sin(r)) / mLon, ringBrg: b });
  }
  return out;
}

/* ---------- pin geometry ---------- */
export function bearingDeg(fromLat, fromLon, toLat, toLon) {
  const mLon = Math.max(0.2, Math.cos((fromLat * Math.PI) / 180));
  const dN = (toLat - fromLat), dE = (toLon - fromLon) * mLon;
  return ((Math.atan2(dE, dN) * 180) / Math.PI + 360) % 360;
}
export function angDiff(a, b) {
  let d = Math.abs(a - b) % 360;
  return d > 180 ? 360 - d : d;
}
/* horizontal offset of a pinned pixel from the frame center, in degrees */
export function pinAzOffsetDeg(x, W, fovDeg) {
  const fpx = (W / 2) / Math.tan((fovDeg * Math.PI) / 360);
  return (Math.atan2(x - W / 2, fpx) * 180) / Math.PI;
}
/* deviation between "where the frame says the structure is" and "where
   the twin actually is from this candidate". Small = consistent. */
export function pinDeviation(cand, twin, frameAzDeg, pinOffDeg) {
  return angDiff(bearingDeg(cand.lat, cand.lon, twin.lat, twin.lon), (frameAzDeg + pinOffDeg + 360) % 360);
}

/* How far away a pinned structure of each kind can plausibly BE from the
   camera and still be prominent enough to pin. This range gate is what
   keeps the consistency test meaningful: without it, any candidate in a
   town with 31 mapped water towers finds SOME tower within a lax angular
   gate and every cell "passes". A peak is the exception — pinned off the
   skyline, tens of km out, where bearings move slowly but honestly. */
export const PIN_RANGE_KM = { water: 1.5, mast: 3, chimney: 3, pylon: 1.5, wind: 3, lighthouse: 5, peak: 40 };
const kindMatch = (p, t) => p === t || (p === "mast" && t === "tower");

/* joint consistency of ALL pins at one candidate: each pin's observed
   azimuth (frame pointing + pixel offset) vs the best matching twin
   within that kind's range. Worst pin governs (a spot must explain
   every structure, same as worst-witness-governs). A pin whose kind has
   no twin in range is SKIPPED — an unmapped mast is absence of map
   data, not evidence against the spot. Returns null when nothing was
   testable. */
export function pinsDeviation(cand, pinObs, twins) {
  const mLon = 111.32 * Math.max(0.2, Math.cos((cand.lat * Math.PI) / 180));
  let worst = null;
  for (const po of pinObs) {
    const rangeKm = PIN_RANGE_KM[po.kind] ?? 2;
    let best = null;
    for (const tw of twins) {
      if (!kindMatch(po.kind, tw.kind)) continue;
      const dKm = Math.hypot((tw.lat - cand.lat) * 111.32, (tw.lon - cand.lon) * mLon);
      if (dKm > rangeKm || dKm < 0.02) continue;
      const dev = angDiff(bearingDeg(cand.lat, cand.lon, tw.lat, tw.lon), po.azDeg);
      if (best == null || dev < best) best = dev;
    }
    if (best == null) continue;
    if (worst == null || best > worst) worst = best;
  }
  return worst;
}

/* ---------- honesty gate ----------
   From the field rounds: a NON-decisive sweep runs best/median ≈ 0.87–0.9;
   a synthetic true-position recovery runs well under 0.6. The gap is wide,
   so the gate sits between them rather than being tuned to either. */
export function sweepVerdict(scores) {
  if (!scores || scores.length < 8) return { decisive: false, ratio: 1 };
  const s = scores.slice().sort((a, b) => a - b);
  const med = s[Math.floor(s.length / 2)];
  const ratio = med > 1e-9 ? s[0] / med : 1;
  return { decisive: ratio < 0.72, ratio: +ratio.toFixed(3) };
}

/* ============================================================
   Browser helpers (network/DOM — never called by tests)
   ============================================================ */

/* ONE shared coarse heightfield pair for the whole search area:
   z11 (~70 m/px) covering the candidates + their 35 km skyline march,
   z8 far fallback for big distant peaks. ~26 MB total regardless of
   candidate count — never demSampler (15 MB PER candidate). */
export async function loadRegion(lat, lon, radKm) {
  const spanM = (z) => (40075000 * Math.max(0.2, Math.cos((lat * Math.PI) / 180))) / (1 << z);
  const needM = radKm * 1000 + 36000;
  const half11 = Math.max(1, Math.ceil(needM / spanM(11) / 256) );
  const [g11, g8] = await Promise.all([
    loadGrid(lat, lon, 11, Math.min(3, half11)),
    loadGrid(lat, lon, 8, 2),
  ]);
  return { grids: [g11, g8], lat0: lat, lon0: lon };
}

/* per-candidate sampler over the shared region (pure given the grids) */
export function regionSampler(region, lat, lon) {
  const mLat = 111320, mLon = 111320 * Math.max(0.2, Math.cos((lat * Math.PI) / 180));
  const sampleEN = (e, n) => {
    const la = lat + n / mLat, lo = lon + e / mLon;
    for (const g of region.grids) {
      const v = gridSample(g, la, lo);
      if (v != null) return v;
    }
    return null;
  };
  const h0 = sampleEN(0, 0);
  if (h0 == null) throw new Error("candidate off the DEM region");
  return { sampleEN, h0 };
}

/* typed landmark twins near the search area (server Overpass proxy) */
export async function fetchLandmarks(lat, lon, radKm, kinds) {
  const r = await fetch(`/api/landmarks?lat=${lat.toFixed(5)}&lon=${lon.toFixed(5)}&r=${Math.round(radKm * 1000)}&kinds=${encodeURIComponent(kinds.join(","))}`, { signal: AbortSignal.timeout(25000) });
  if (!r.ok) throw new Error(`landmarks HTTP ${r.status}`);
  const j = await r.json();
  const out = [];
  for (const e of j.elements || []) {
    const la = e.lat ?? e.center?.lat, lo = e.lon ?? e.center?.lon;
    if (!isFinite(la) || !isFinite(lo)) continue;
    const t = e.tags || {};
    const kind = t.man_made === "water_tower" ? "water"
      : (t.man_made === "mast" || t.man_made === "communications_tower") ? "mast"
        : t.man_made === "chimney" ? "chimney"
          : t.man_made === "lighthouse" ? "lighthouse"
            : t.power === "tower" ? "pylon"
              : (t.power === "generator" && t["generator:source"] === "wind") ? "wind"
                : t.natural === "peak" ? "peak"
                  : t.man_made === "tower" ? "tower" : null;
    if (!kind) continue;
    out.push({ lat: la, lon: lo, kind, name: t.name || "", height: t.height ? parseFloat(t.height) : null });
  }
  return out;
}
