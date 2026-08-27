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

/* ---------- near-ridge depth layer ----------
   A pan from one spot has NO parallax (pure rotation), but ridge
   LAYERING encodes depth anyway: where the nearer, darker crest sits
   against the far wall changes fast with observer position, while the
   far wall barely moves. This detects the SECOND boundary below the far
   skyline — the next crest line, markedly darker than the haze-lit wall
   behind it — for scoring against the DEM's interior visible crests. */
export function nearSkyline(px, W, H, farPts, opts = {}) {
  const d = px.data ? px.data : px;
  const BAND = Math.max(5, Math.round(H * 0.03));
  const drop = opts.drop ?? 28;   // how much darker the next layer must be than the wall
  const out = [];
  for (const fp of farPts) {
    const x = fp.x;
    const lum = (y) => { const i = (y * W + x) * 4; return 0.299 * d[i] + 0.587 * d[i + 1] + 0.114 * d[i + 2]; };
    /* the wall's own brightness, sampled just below the far boundary */
    let wall = 0, n = 0;
    for (let y = fp.y + 4; y < Math.min(H, fp.y + 16); y++) { wall += lum(y); n++; }
    if (!n) continue;
    wall /= n;
    let found = -1;
    for (let y = fp.y + 10; y < H * 0.97; y++) {
      if (lum(y) >= wall - drop) continue;
      let below = 0, tot = 0;
      for (let k = 1; k <= BAND && y + k < H; k++) { tot++; if (lum(y + k) < wall - drop) below++; }
      if (tot && below / tot >= 0.85) { found = y; break; }
    }
    if (found > 0) out.push({ x, y: found });
  }
  if (out.length < 16) return null;
  const keep = out.filter((p, i) => {
    const win = out.slice(Math.max(0, i - 3), i + 4).map((q) => q.y).sort((a, b) => a - b);
    const med = win[Math.floor(win.length / 2)];
    return Math.abs(p.y - med) < Math.max(6, 0.06 * H);
  });
  return keep.length >= 16 ? keep : null;
}

/* candidate-side twin of the second layer: the highest INTERIOR visible
   crest per 1° azimuth bin, from skylineFromSampler's ridges (which
   already encode occlusion — a hidden crest emits nothing). NaN = no
   interior crest at that azimuth. */
export function ridgeProfileOf(sk) {
  const prof = new Float32Array(360).fill(NaN);
  for (const rg of sk.ridges || []) {
    for (const [az, el] of rg.pts) {
      const b = ((Math.round(az) % 360) + 360) % 360;
      if (!(prof[b] >= el)) prof[b] = el;
    }
  }
  return prof;
}
export function ridgeProfAt(prof, azDeg) {
  const b = ((Math.round(azDeg) % 360) + 360) % 360;
  return prof[b];
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

/* the fitted pitch intercept + roll slope at one shift (same model rmsAt
   minimizes — needed so the near layer can be scored under the FAR
   layer's pose, with no free parameters left to absorb the depth signal) */
function fitParams(ss, elAt, dAz) {
  let sw = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (const s of ss) {
    const r = elAt(s.az + dAz) - s.el;
    sw++; sx += s.thx; sy += r; sxx += s.thx * s.thx; sxy += s.thx * r;
  }
  const den = sw * sxx - sx * sx;
  const b = Math.abs(den) > 1e-9 ? (sw * sxy - sx * sy) / den : 0;
  return { a: (sy - b * sx) / sw, b };
}

/* joint far+near curve: at every pointing bin the far layer fits
   pitch/roll, then the near layer's residual vs the interior crests is
   scored UNDER that fixed pose (no free parameters left, so the depth
   signal fully counts) and folded into the bin. This matters because a
   smooth far wall leaves the pointing ambiguous — evaluating the near
   layer only at the far layer's chosen azimuth let the WRONG azimuth
   win first and then punished the truth for it (measured). A candidate
   must find one pointing that explains BOTH layers at once. */
function azCurveJoint(sets, nearSets, elAt, prof, stepDeg = AZ_STEP, nearW = 0.7) {
  const NB = Math.round(360 / stepDeg);
  const c = new Float32Array(NB).fill(1e9);
  const fi = new Uint8Array(NB);
  for (let vi = 0; vi < sets.length; vi++) {
    const v = sets[vi], nv = nearSets[vi];
    for (let b = 0; b < NB; b++) {
      const dAz = b * stepDeg - 180;
      const fit = fitParams(v.ss, elAt, dAz);
      let s2 = 0;
      for (const s of v.ss) {
        const e = (elAt(s.az + dAz) - s.el) - (fit.a + fit.b * s.thx);
        s2 += e * e;
      }
      let nr = Math.sqrt(s2 / v.ss.length);
      if (nv) {
        let n2 = 0, q2 = 0;
        for (const s of nv.ss) {
          const val = ridgeProfAt(prof, s.az + dAz);
          if (!isFinite(val)) continue;
          const e = (val - s.el) - (fit.a + fit.b * s.thx);
          q2 += e * e; n2++;
        }
        /* too few overlapping crest bins → the DEM says there's nothing
           NEAR at this pointing while the photo clearly shows a second
           layer: that mismatch is itself evidence against the pointing */
        nr += nearW * (n2 >= 12 ? Math.sqrt(q2 / n2) : 3);
      }
      nr /= v.std;
      if (nr < c[b]) { c[b] = nr; fi[b] = vi; }
    }
  }
  return { c, fi, stepDeg };
}

/* score one candidate position: DEM skyline there vs every frame.
   Frames may carry `nearSets` (the detected SECOND ridge layer, scored
   jointly against the DEM's interior visible crests — the depth signal
   a pan cannot get from parallax). */
export function scoreCandidate(frameSets, sampleEN, h0, opts = {}) {
  const sk = skylineFromSampler(sampleEN, h0);
  const elAt = (a) => skylineElAt(sk.els, a);
  const anyNear = frameSets.some((f) => f.nearSets);
  const prof = anyNear ? ridgeProfileOf(sk) : null;
  const step = opts.stepDeg ?? AZ_STEP;
  const curves = frameSets.map((f) => f.nearSets && prof
    ? azCurveJoint(f.sets, f.nearSets, elAt, prof, step, opts.nearW ?? 0.7)
    : azCurve(f.sets, elAt, step));
  const w = coherentWindow(curves, opts.winDeg ?? WIN_DEG, opts.keepFrac ?? 0.75);
  if (!w) return null;
  /* report how well the near layer agreed at the solved pointing */
  let nearSum = 0, nearN = 0;
  for (let i = 0; i < frameSets.length; i++) {
    const f = frameSets[i];
    if (!f.nearSets || !prof) continue;
    const fa = w.frameAz[i];
    const far = f.sets[fa.fovIdx], near = f.nearSets[fa.fovIdx];
    if (!far || !near) continue;
    const fit = fitParams(far.ss, elAt, fa.az);
    let s2 = 0, n2 = 0;
    for (const s of near.ss) {
      const v = ridgeProfAt(prof, s.az + fa.az);
      if (!isFinite(v)) continue;
      const e = (v - s.el) - (fit.a + fit.b * s.thx);
      s2 += e * e; n2++;
    }
    if (n2 >= 12) { nearSum += Math.sqrt(s2 / n2) / far.std; nearN++; }
  }
  return { ...w, nearScore: nearN ? +(nearSum / nearN).toFixed(3) : null, h0 };
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
export const PIN_RANGE_KM = { water: 1.5, mast: 3, chimney: 3, pylon: 1.5, wind: 3, lighthouse: 5, bridge: 5, peak: 40 };
const kindMatch = (p, t) => p === t || (p === "mast" && t === "tower");

/* ---------- EXTENDED (linear) twins ----------
   Every other pinnable structure is a POINT: a water tower, a mast, a
   chimney is one bearing from one spot. A BRIDGE is not — it is a line
   that can run a kilometre, and OSM's centre for it can sit far from the
   span you actually photographed. Matching a bridge by its centre would
   punish the true spot for standing near one end (and reward a wrong one
   that happens to line up with the middle), so an extended twin carries
   `pts` — its own OSM geometry — and the test asks the honest question
   instead: does the pinned bearing hit the structure ANYWHERE along its
   length? Deviation is the angular miss to the nearest point of the span
   (zero while the ray crosses it) and the range gate uses the NEAREST
   point, not the centre. Point twins have no `pts` and take the same
   path they always did. */
const lonKm = (lat) => 111.32 * Math.max(0.2, Math.cos((lat * Math.PI) / 180));
/* nearest/farthest distance (km) from the candidate and the smallest
   bearing miss over the WHOLE polyline. Solved exactly per segment —
   the ray either crosses the span (miss 0) or its miss is to one of the
   segment's ends — so a long bridge can't slip between sample points. */
export function spanParts(pts) {
  return Array.isArray(pts?.[0]) ? pts : [pts];   // one polyline, or several
}
export function spanMetrics(cand, pts, azDeg) {
  let near = Infinity, far = 0, dev = null;
  const brg = (n, e) => ((Math.atan2(e, n) * 180) / Math.PI + 360) % 360;
  const vertex = (n, e) => {
    const d = Math.hypot(n, e);
    if (d < near) near = d;
    if (d > far) far = d;
    if (d < 0.02) return;                       // a bearing from 20 m away is noise
    const dv = angDiff(brg(n, e), azDeg);
    if (dev == null || dv < dev) dev = dv;
  };
  const uN = Math.cos((azDeg * Math.PI) / 180), uE = Math.sin((azDeg * Math.PI) / 180);
  const mLon = lonKm(cand.lat);
  for (const part of spanParts(pts)) {
    /* km north / km east of the candidate — the same local plane the rest
       of the file works in */
    const P = part.map((q) => [(q.lat - cand.lat) * 111.32, (q.lon - cand.lon) * mLon]);
    for (const [n, e] of P) vertex(n, e);
    for (let i = 0; i + 1 < P.length; i++) {
      const [n0, e0] = P[i], [n1, e1] = P[i + 1];
      const dN = n1 - n0, dE = e1 - e0, L = Math.hypot(dN, dE);
      if (L < 1e-9) continue;
      /* nearest approach of the segment to the candidate (range gate) */
      const s = Math.max(0, Math.min(1, -(n0 * dN + e0 * dE) / (L * L)));
      near = Math.min(near, Math.hypot(n0 + dN * s, e0 + dE * s));
      /* does the sight-line cross this segment? P0 + s·d = t·u, t > 0 */
      const den = dN * uE - dE * uN;
      if (Math.abs(den) < 1e-12) continue;      // parallel to the ray
      const ss = (n0 * uE - e0 * uN) / -den;
      const t = Math.abs(uN) > Math.abs(uE) ? (n0 + dN * ss) / uN : (e0 + dE * ss) / uE;
      if (ss >= 0 && ss <= 1 && t > 0.02) dev = 0;
    }
  }
  return { near, far, dev };
}
/* where a camera could plausibly have STOOD to photograph this twin —
   the point itself, or for a span its two most distant points plus the
   middle between them. A bridge is photographed from a bank, which can
   be half a kilometre from the point OSM calls its centre; and its OSM
   geometry is often a CLOSED outline, so "first and last vertex" would
   be the same spot. Diameter by the usual two-pass walk (farthest from
   the centroid, then farthest from that), so it works for a closed
   outline and a chain of separate ways alike. */
export function twinAnchors(tw) {
  if (!tw.pts) return [{ lat: tw.lat, lon: tw.lon }];
  const all = spanParts(tw.pts).flat();
  if (all.length < 2) return [{ lat: tw.lat, lon: tw.lon }];
  const mLon = Math.max(0.2, Math.cos((all[0].lat * Math.PI) / 180));
  const d2 = (a, b) => (a.lat - b.lat) ** 2 + ((a.lon - b.lon) * mLon) ** 2;
  const cen = { lat: all.reduce((s, q) => s + q.lat, 0) / all.length, lon: all.reduce((s, q) => s + q.lon, 0) / all.length };
  const far = (from) => all.reduce((b, q) => (d2(q, from) > d2(b, from) ? q : b), all[0]);
  const A = far(cen), B = far(A);
  /* the third anchor is the point ON the structure closest to the middle
     of that diameter — projected onto the EDGES, not snapped to a
     vertex: a bridge mapped as a closed outline has no vertex near its
     middle, and snapping would put the third anchor back on a corner */
  const mid = { lat: (A.lat + B.lat) / 2, lon: (A.lon + B.lon) / 2 };
  let C = A, cBest = Infinity;
  for (const part of spanParts(tw.pts)) for (let i = 0; i + 1 < part.length; i++) {
    const p0 = part[i], p1 = part[i + 1];
    const dLat = p1.lat - p0.lat, dLon = (p1.lon - p0.lon) * mLon, L2 = dLat * dLat + dLon * dLon;
    if (L2 < 1e-18) continue;
    const u = Math.max(0, Math.min(1, ((mid.lat - p0.lat) * dLat + (mid.lon - p0.lon) * mLon * dLon) / L2));
    const q = { lat: p0.lat + (p1.lat - p0.lat) * u, lon: p0.lon + (p1.lon - p0.lon) * u };
    const d = d2(q, mid);
    if (d < cBest) { cBest = d; C = q; }
  }
  const out = [];
  for (const q of [A, C, B]) if (!out.some((o) => d2(o, q) < (0.05 / 111.32) ** 2)) out.push({ lat: q.lat, lon: q.lon });
  return out;
}

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
      let near, far, dev;
      /* ANY twin with geometry takes the span path — a one-way bridge is
         `pts: [oneLine]`, whose length is 1, so testing pts.length here
         would silently send the commonest case back to centre-matching */
      if (tw.pts) {
        ({ near, far, dev } = spanMetrics(cand, tw.pts, po.azDeg));
      } else {
        near = far = Math.hypot((tw.lat - cand.lat) * 111.32, (tw.lon - cand.lon) * mLon);
        dev = near < 0.02 ? null : angDiff(bearingDeg(cand.lat, cand.lon, tw.lat, tw.lon), po.azDeg);
      }
      /* nearest point governs the range gate; the whole structure being
         inside 20 m is the degenerate case (you are standing ON it) */
      if (near > rangeKm || far < 0.02 || dev == null) continue;
      if (best == null || dev < best) best = dev;
    }
    if (best == null) continue;
    if (worst == null || best > worst) worst = best;
  }
  return worst;
}

/* ---------- setting-context filters ----------
   The witness usually KNOWS the setting — "I was in town", "the view is
   open country" — and that context kills whole swaths of candidates
   before any skyline math runs. Places are OSM place nodes; each type
   carries a built-up radius since the node marks the CENTER. */
export const PLACE_RADIUS_KM = { city: 6, town: 2.5, village: 1.2, hamlet: 0.6 };
export function nearestPlaceEdge(cand, places) {
  const mLon = 111.32 * Math.max(0.2, Math.cos((cand.lat * Math.PI) / 180));
  let best = null;
  for (const p of places) {
    const dKm = Math.hypot((p.lat - cand.lat) * 111.32, (p.lon - cand.lon) * mLon);
    const edgeKm = dKm - (PLACE_RADIUS_KM[p.ptype] ?? 1);
    if (!best || edgeKm < best.edgeKm) best = { edgeKm, dKm, place: p };
  }
  return best;
}
/* built-up LAND-USE bounding boxes are the primary signal — a place node
   marks a town's center, and its type radius passed a bare field 1 km
   out as "in a town" (field report). bbox: [s, n, w, e] degrees. */
export function urbanDistKm(cand, urbans) {
  const mLon = 111.32 * Math.max(0.2, Math.cos((cand.lat * Math.PI) / 180));
  let best = null;
  for (const u of urbans) {
    const [s, n, w, e] = u.bbox;
    const dLat = cand.lat < s ? s - cand.lat : cand.lat > n ? cand.lat - n : 0;
    const dLon = cand.lon < w ? w - cand.lon : cand.lon > e ? cand.lon - e : 0;
    const d = Math.hypot(dLat * 111.32, dLon * mLon);
    if (best == null || d < best) best = d;
  }
  return best; // 0 = inside one; null = no data
}
/* does the view ray enter a built-up box (other than one you stand in)
   within maxKm? Slab test in local km coordinates. */
export function rayHitsUrban(cand, azDeg, urbans, maxKm = 8) {
  const mLon = 111.32 * Math.max(0.2, Math.cos((cand.lat * Math.PI) / 180));
  const dx = Math.sin((azDeg * Math.PI) / 180), dy = Math.cos((azDeg * Math.PI) / 180);
  for (const u of urbans) {
    const [s, n, w, e] = u.bbox;
    const x0 = (w - cand.lon) * mLon, x1 = (e - cand.lon) * mLon;
    const y0 = (s - cand.lat) * 111.32, y1 = (n - cand.lat) * 111.32;
    if (x0 <= 0 && x1 >= 0 && y0 <= 0 && y1 >= 0) continue; // standing in it
    let tmin = -1e9, tmax = 1e9;
    if (Math.abs(dx) < 1e-9) { if (x0 > 0 || x1 < 0) continue; }
    else { const a = x0 / dx, b = x1 / dx; tmin = Math.max(tmin, Math.min(a, b)); tmax = Math.min(tmax, Math.max(a, b)); }
    if (Math.abs(dy) < 1e-9) { if (y0 > 0 || y1 < 0) continue; }
    else { const a = y0 / dy, b = y1 / dy; tmin = Math.max(tmin, Math.min(a, b)); tmax = Math.min(tmax, Math.max(a, b)); }
    if (tmax >= Math.max(tmin, 0) && Math.max(tmin, 0) <= maxKm) return true;
  }
  return false;
}
/* Is the land-use mapping locally TRUSTWORTHY? Measured on Dehradun
   (~700k metro): 187 scattered polygons covering 1.35% of a 14 km
   search circle, with even the city core 1.4 km from the nearest one —
   filtering on that killed the true neighborhood. A land-use layer is
   only primary when it covers a meaningful fraction of the search
   area; below the gate the caller should fall back to place nodes. */
export function urbanCoverage(urbans, centerLat, radKm) {
  const mLon = 111.32 * Math.max(0.2, Math.cos((centerLat * Math.PI) / 180));
  let area = 0;
  for (const u of urbans) {
    const [s, n, w, e] = u.bbox;
    area += (n - s) * 111.32 * (e - w) * mLon;
  }
  const frac = area / (Math.PI * radKm * radKm);
  return { frac, ok: frac >= 0.04 };
}

/* stood: "town" = on built-up land; "out" = clearly off it. Land-use
   boxes when mapped AND dense enough to trust, place-node radii as
   fallback, no data → no filter. */
export function settingOk(cand, ctx, stood) {
  if (!stood) return true;
  const urbans = ctx.urbans || [], places = ctx.places || [];
  if (urbans.length) {
    const d = urbanDistKm(cand, urbans);
    return stood === "town" ? d <= 0.25 : d >= 0.5;
  }
  if (!places.length) return true; // no data → never filter on a guess
  const np = nearestPlaceEdge(cand, places);
  if (stood === "town") return !!np && np.edgeKm <= 0.3;
  if (stood === "out") return !np || np.edgeKm >= 1;
  return true;
}
/* look: does the solved pointing cross OTHER built-up land (not what you
   stand in)? "town" = yes; "open" = no. The exclusion is what makes
   "in city aimed out of city" work. */
export function lookOk(cand, azDeg, ctx, look) {
  if (!look) return true;
  const urbans = ctx.urbans || [], places = ctx.places || [];
  if (urbans.length) {
    const hit = rayHitsUrban(cand, azDeg, urbans, 8);
    return look === "town" ? hit : !hit;
  }
  if (!places.length) return true;
  const mLon = 111.32 * Math.max(0.2, Math.cos((cand.lat * Math.PI) / 180));
  const hit = places.some((p) => {
    const rad = PLACE_RADIUS_KM[p.ptype] ?? 1;
    const dKm = Math.hypot((p.lat - cand.lat) * 111.32, (p.lon - cand.lon) * mLon);
    if (dKm <= rad || dKm > rad + 8) return false; // standing inside it, or too far to matter
    return angDiff(bearingDeg(cand.lat, cand.lon, p.lat, p.lon), azDeg) < 25;
  });
  return look === "town" ? hit : !hit;
}

/* ---------- stabilized-pan lock ----------
   A solved posePath knows each frame's pointing RELATIVE to the others
   (and its FOV, even mid-zoom). Baking those relative az/el offsets into
   the samples merges the whole pan into ONE rigid set scored over a
   single global rotation — the per-frame ±window freedom (and the FOV
   sweep) collapse away, which is a far tighter joint constraint. */
export function lockedFrameSet(perFrame) {
  const ss = [];
  for (const f of perFrame) {
    const sets = skySampleSets(f.pts, f.W, f.H, [f.fov]);
    for (const s of sets[0].ss) ss.push({ az: s.az + f.relAz, el: s.el + f.relEl, thx: s.thx });
  }
  if (ss.length < 20) return null;
  const mean = ss.reduce((a, s) => a + s.el, 0) / ss.length;
  const std = Math.sqrt(ss.reduce((a, s) => a + (s.el - mean) ** 2, 0) / ss.length);
  return { fov: perFrame[0].fov, ss, std: Math.max(0.15, std) };
}

/* ---------- weather cross-check ----------
   The clip SHOWS its sky; the claimed date + a candidate area imply one
   from the reanalysis archive. A mismatch (clip overcast, archive says
   clear) indicts the date or the area. Two honesty notes are structural:
   cloud cover is ~25 km coarse, so this is an AREA-level check that can
   never separate candidates inside one search — and a hazy in-between
   sky reads as "mixed", which yields no verdict rather than a guess. */
export function skyStats(px, W, H) {
  const d = px.data ? px.data : px;
  let lum = 0, br = 0, n = 0;
  const y0 = Math.round(H * 0.02), y1 = Math.round(H * 0.10);
  for (let ci = 0; ci < 36; ci++) {
    const x = Math.round((0.04 + (0.92 * ci) / 35) * (W - 1));
    for (let y = y0; y <= y1; y += 2) {
      const i = (y * W + x) * 4;
      lum += 0.299 * d[i] + 0.587 * d[i + 1] + 0.114 * d[i + 2];
      br += d[i + 2] - d[i];
      n++;
    }
  }
  return n ? { lum: lum / n, br: br / n } : null;
}
export function skyCondition(stats) {
  const st = (stats || []).filter(Boolean);
  if (!st.length) return null;
  const lum = st.reduce((a, s) => a + s.lum, 0) / st.length;
  const br = st.reduce((a, s) => a + s.br, 0) / st.length;
  if (lum < 90) return null;      // dark sky — no daytime cloud read
  if (br >= 30) return "clear";   // saturated blue
  if (br <= 16) return "overcast"; // white/gray dome
  return "mixed";                  // hazy in-between — honest no-call
}
export function cloudMatch(cond, cloudPct) {
  if (!cond || cond === "mixed" || !isFinite(cloudPct)) return { verdict: "weak" };
  if (cond === "overcast") return { verdict: cloudPct >= 65 ? "match" : cloudPct <= 25 ? "mismatch" : "weak" };
  return { verdict: cloudPct <= 30 ? "match" : cloudPct >= 80 ? "mismatch" : "weak" };
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
  const out = [], bridgeWays = [];
  for (const e of j.elements || []) {
    const t = e.tags || {};
    /* built-up land-use ways come back as bounding boxes (out bb) */
    if (t.landuse && e.bounds && isFinite(e.bounds.minlat)) {
      out.push({ kind: "urban", bbox: [e.bounds.minlat, e.bounds.maxlat, e.bounds.minlon, e.bounds.maxlon] });
      continue;
    }
    /* bridges come back as WAYS WITH GEOMETRY (out geom) — they are the
       one pinnable structure with real extent, so the span is kept and
       matched along its whole length (spanMetrics) */
    const geom = Array.isArray(e.geometry) ? e.geometry.filter((g) => isFinite(g?.lat) && isFinite(g?.lon)).map((g) => ({ lat: g.lat, lon: g.lon })) : null;
    if (t.man_made === "bridge" || (t.bridge && t.bridge !== "no")) {
      if (geom && geom.length) bridgeWays.push({ name: t.name || "", id: e.id, geom });
      continue;
    }
    const la = e.lat ?? e.center?.lat, lo = e.lon ?? e.center?.lon;
    if (!isFinite(la) || !isFinite(lo)) continue;
    const kind = t.man_made === "water_tower" ? "water"
      : (t.man_made === "mast" || t.man_made === "communications_tower") ? "mast"
        : t.man_made === "chimney" ? "chimney"
          : t.man_made === "lighthouse" ? "lighthouse"
            : t.power === "tower" ? "pylon"
              : (t.power === "generator" && t["generator:source"] === "wind") ? "wind"
                : t.natural === "peak" ? "peak"
                  : /^(city|town|village|hamlet)$/.test(t.place || "") ? "place"
                    : t.man_made === "tower" ? "tower" : null;
    if (!kind) continue;
    out.push({ lat: la, lon: lo, kind, ptype: t.place || null, name: t.name || "", height: t.height ? parseFloat(t.height) : null });
  }
  /* ONE bridge is many OSM ways — the Golden Gate came back as 41 (deck
     segments, each sidewalk, the structure outline). Left apart they
     would flood the nearest-80 twin cap and spawn ring candidates around
     every segment, so ways sharing a NAME become one multi-part twin;
     unnamed ways stay on their own. Parts are kept SEPARATE inside it
     rather than concatenated — joining two disjoint segments would
     invent a span between them that no ray should ever be able to hit. */
  const byName = new Map();
  for (const b of bridgeWays) {
    const key = b.name || `#${b.id}`;
    if (!byName.has(key)) byName.set(key, []);
    byName.get(key).push(b.geom);
  }
  for (const [key, parts] of byName) {
    const longest = parts.reduce((a, b) => (b.length > a.length ? b : a), parts[0]);
    const c = longest[Math.floor(longest.length / 2)];
    out.push({ lat: c.lat, lon: c.lon, pts: parts, kind: "bridge", name: key.startsWith("#") ? "" : key });
  }
  return out;
}
