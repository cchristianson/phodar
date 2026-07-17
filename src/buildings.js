/* ============================================================
   URBAN BUILDING SILHOUETTE — the city analog of the DEM ridge line.
   For a sighting shot in town there is no mountain skyline to calibrate
   against, but rooftops make an equally sharp, surveyed silhouette. OSM
   building FOOTPRINTS + HEIGHTS (Overpass) are rasterized as an added-
   height field ON TOP OF the DEM ground and fed through the SAME
   `skylineFromSampler` ray-march terrain.js uses, so the predicted
   skyline, the layered-ridge draw and "Snap to ridges" all work over
   rooftops with no changes downstream.

   Honest coverage is the whole caveat: footprints are ubiquitous, but
   HEIGHTS are not. Only buildings tagged `height=` (or `building:levels=`,
   ×3 m/storey as an estimate) can be placed on the silhouette; buildings
   with no height at all are DROPPED, never guessed as a flat wall, and the
   count of dropped/estimated buildings is surfaced so a thin-coverage city
   reads as thin, not as a confident wrong answer.

   Data: OSM via Overpass, proxied through /api/buildings — CORS on the
   Overpass mirrors is unreliable, the same reason /api/peaks is proxied.
   Parsing + the height field are pure and unit-tested in mathcheck.
   ============================================================ */

import { D2R, R2D, RE } from "./math/geodesy.js";

export const BUILDINGS_ATTRIB = "Buildings: OpenStreetMap contributors (ODbL)";
const M_PER_LEVEL = 3.0; // metres per storey when only building:levels is known

/* an OSM `height` / `building:levels` tag set → metres, or null if unusable.
   Accepts "25", "25 m", "25.5m", and imperial "82'" / "82 ft" (→ metres).
   Falls back to levels × 3 m. Returns { m, est } — est=true when derived from
   levels rather than an explicit height. */
export function heightMeters(tags) {
  if (!tags) return null;
  var raw = tags.height != null ? String(tags.height)
    : tags["building:height"] != null ? String(tags["building:height"]) : null;
  if (raw != null) {
    var v = parseFloat(raw.replace(/[^0-9.\-]/g, ""));
    if (isFinite(v) && v > 0) return { m: /'|ft|feet/i.test(raw) ? v * 0.3048 : v, est: false };
  }
  var lv = tags["building:levels"];
  if (lv != null) {
    var n = parseFloat(String(lv).replace(/[^0-9.\-]/g, ""));
    if (isFinite(n) && n > 0) return { m: n * M_PER_LEVEL, est: true };
  }
  return null;
}

/* Overpass `out geom` JSON → buildings in the observer's local ENU frame:
     { buildings: [{ ring: [[e,n],…] metres, h: metres, est, assumed,
                     bbox: [e0,n0,e1,n1] }], known, est, assumed, dropped }
   With `opts.assumeM` (metres) set, a footprint with NO usable height tag is
   still placed at that assumed height (`assumed: true`) instead of dropped —
   OSM height coverage is thin (a lot of warehouses/houses have only a
   footprint), and an approximate rooftop is far more useful for VISUAL
   alignment than nothing. Without assumeM, untagged footprints are dropped.
   `known` = explicit-height, `est` = floor-count, `assumed` = defaulted.
   ENU uses the SAME equirectangular constants terrain.js's demSampler does. */
export function parseOverpassBuildings(json, obsLat, obsLon, opts) {
  var assumeM = (opts && opts.assumeM) || 0;
  var mLat = 111320, mLon = 111320 * Math.max(0.2, Math.cos(obsLat * D2R));
  var out = [], dropped = 0, est = 0, assumed = 0, known = 0;
  var els = (json && json.elements) || [];
  for (var k = 0; k < els.length; k++) {
    var el = els[k];
    if (el.type !== "way" || !el.tags || el.tags.building == null) continue;
    var geom = el.geometry;
    if (!Array.isArray(geom) || geom.length < 3) continue;
    var hm = heightMeters(el.tags);
    if (hm == null) {
      if (assumeM > 0) hm = { m: assumeM, est: false, assumed: true };
      else { dropped++; continue; }
    }
    var e0 = Infinity, n0 = Infinity, e1 = -Infinity, n1 = -Infinity, ring = [];
    for (var g = 0; g < geom.length; g++) {
      var pt = geom[g];
      if (pt.lat == null || pt.lon == null) continue;
      var e = (pt.lon - obsLon) * mLon, n = (pt.lat - obsLat) * mLat;
      ring.push([e, n]);
      if (e < e0) e0 = e; if (n < n0) n0 = n; if (e > e1) e1 = e; if (n > n1) n1 = n;
    }
    if (ring.length < 3) continue;
    if (hm.assumed) assumed++; else if (hm.est) est++; else known++;
    out.push({ ring: ring, h: hm.m, est: !!hm.est, assumed: !!hm.assumed, bbox: [e0, n0, e1, n1] });
  }
  return { buildings: out, dropped: dropped, est: est, assumed: assumed, known: known };
}

/* ---- hidden-line removal for the box wireframes (draw as if solid) --------
   Each box's occluding silhouette is the convex hull of its 8 projected
   corners; every edge is clipped against the hulls of NEARER boxes so a
   foremost building keeps its full wireframe while ones behind show only the
   parts poking above/between it. Pure 2D screen coords — unit-tested. */
export function convexHull2(pts) {
  var P = pts.slice().sort(function (a, b) { return a[0] - b[0] || a[1] - b[1]; });
  if (P.length < 3) return P.slice();
  var cr = function (o, a, b) { return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]); };
  var lo = [], i; for (i = 0; i < P.length; i++) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], P[i]) <= 0) lo.pop(); lo.push(P[i]); }
  var up = []; for (i = P.length - 1; i >= 0; i--) { while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], P[i]) <= 0) up.pop(); up.push(P[i]); }
  lo.pop(); up.pop(); return lo.concat(up);
}
/* interval [t0,t1] ⊂ [0,1] of segment p→q that lies INSIDE convex hull H (else null) */
export function segInsideHull(p, q, H) {
  if (H.length < 3) return null;
  var cx = 0, cy = 0, i; for (i = 0; i < H.length; i++) { cx += H[i][0]; cy += H[i][1]; } cx /= H.length; cy /= H.length;
  var dx = q[0] - p[0], dy = q[1] - p[1], tLo = 0, tHi = 1;
  for (i = 0; i < H.length; i++) {
    var A = H[i], B = H[(i + 1) % H.length];
    var nx = -(B[1] - A[1]), ny = (B[0] - A[0]);
    var s = ((cx - A[0]) * nx + (cy - A[1]) * ny) >= 0 ? 1 : -1; // orient half-plane so inside ≥ 0
    var a = (dx * nx + dy * ny) * s, b = ((p[0] - A[0]) * nx + (p[1] - A[1]) * ny) * s;
    if (Math.abs(a) < 1e-9) { if (b < 0) return null; continue; }
    var t = -b / a;
    if (a > 0) { if (t > tLo) tLo = t; } else { if (t < tHi) tHi = t; }
    if (tLo > tHi) return null;
  }
  return tLo < tHi ? [tLo, tHi] : null;
}
/* visible sub-segments of p→q after subtracting everything inside any hull */
export function visibleSegs(p, q, hulls) {
  var occ = [], i;
  for (i = 0; i < hulls.length; i++) { var iv = segInsideHull(p, q, hulls[i]); if (iv) occ.push(iv); }
  if (!occ.length) return [[0, 1]];
  occ.sort(function (a, b) { return a[0] - b[0]; });
  var out = [], cur = 0;
  for (i = 0; i < occ.length; i++) { var s = occ[i][0], e = occ[i][1]; if (s > cur + 1e-3) out.push([cur, s]); if (e > cur) cur = e; if (cur >= 1) break; }
  if (cur < 1 - 1e-3) out.push([cur, 1]);
  return out;
}
export const bboxHit = (a, b) => a[0] <= b[2] && a[2] >= b[0] && a[1] <= b[3] && a[3] >= b[1];

/* point-in-polygon (even-odd ray cast) with a bbox fast-reject */
function inRing(ring, bbox, e, n) {
  if (e < bbox[0] || e > bbox[2] || n < bbox[1] || n > bbox[3]) return false;
  var inside = false;
  for (var i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    var ei = ring[i][0], ni = ring[i][1], ej = ring[j][0], nj = ring[j][1];
    if ((ni > n) !== (nj > n) && e < ((ej - ei) * (n - ni)) / (nj - ni) + ei) inside = !inside;
  }
  return inside;
}

/* nearest distance² from the observer (ENU origin) to a bbox */
function bboxDist2(b) {
  var e = b[0] > 0 ? b[0] : (b[2] < 0 ? b[2] : 0);
  var n = b[1] > 0 ? b[1] : (b[3] < 0 ? b[3] : 0);
  return e * e + n * n;
}

/* added-height sampler: (e,n) → tallest covering rooftop height above the
   ground it stands on, or 0 where no footprint covers the point. Pruned to the
   `capN` NEAREST footprints within `maxM` — city silhouettes are near, far
   buildings are sub-degree, and a fixed cap bounds the per-sample cost so a
   dense downtown (thousands of footprints) can't stall the ray-march. */
export function buildingHeightSampler(buildings, maxM = 4000, capN = 600) {
  var lim2 = maxM * maxM;
  var near = buildings
    .map(function (b) { return { b: b, d2: bboxDist2(b.bbox) }; })
    .filter(function (x) { return x.d2 <= lim2; })
    .sort(function (a, b) { return a.d2 - b.d2; })
    .slice(0, capN)
    .map(function (x) { return x.b; });
  return function (e, n) {
    var hi = 0;
    for (var i = 0; i < near.length; i++) {
      var b = near[i];
      if (b.h > hi && inRing(b.ring, b.bbox, e, n)) hi = b.h;
    }
    return hi;
  };
}

/* browser fetch → parsed buildings in the observer's ENU frame.
   opts.assumeM places untagged footprints at that assumed height. */
export async function fetchBuildings(lat, lon, radiusM = 2500, opts) {
  var r = await fetch(`/api/buildings?lat=${lat.toFixed(5)}&lon=${lon.toFixed(5)}&r=${Math.round(radiusM)}`, { signal: AbortSignal.timeout(50000) });
  if (!r.ok) throw new Error(`buildings HTTP ${r.status}`);
  var j = await r.json();
  if (j.error) throw new Error(j.error);
  return parseOverpassBuildings(j, lat, lon, opts);
}

/* Nearest-N building footprints as INDIVIDUAL boxes for the sky view — the
   union silhouette merged everything into one jagged line; drawing each
   footprint as its own projected wireframe (roof outline + vertical corner
   edges) lets you match distinct buildings to the photo. Pure geometry — the
   dome does the projection so it needs no DEM (building base is taken at the
   observer's ground plane; flat-city assumption, fine for a visual aid).

   Excludes the footprint the observer STANDS IN/ON (origin inside it) — that's
   the building you're shooting from, not a silhouette in front of you (it was
   the source of the "too tall" near-spike from a window shot) — and anything
   closer than `minM`. Returns nearest `capN`, sorted near→far. */
export function buildingBoxes(parsed, opts) {
  var maxM = (opts && opts.maxM) || 1200, capN = (opts && opts.capN) || 160, minM = (opts && opts.minM) || 12;
  var bs = parsed.buildings || parsed, out = [];
  for (var i = 0; i < bs.length; i++) {
    var b = bs[i];
    if (inRing(b.ring, b.bbox, 0, 0)) continue;          // the building you're in/on
    var d2 = bboxDist2(b.bbox);
    if (d2 < minM * minM || d2 > maxM * maxM) continue;
    out.push({ ring: b.ring, h: b.h, dist: Math.sqrt(d2), est: !!b.est, assumed: !!b.assumed, bbox: b.bbox });
  }
  out.sort(function (a, b) { return a.dist - b.dist; });
  return out.slice(0, capN);
}

/* the tallest rooftop's elevation + azimuth (flat ground, eye 1.6 m) — a UI
   diagnostic so a glance says whether the silhouette actually computed. */
export function boxesPeak(boxes) {
  var peak = { el: -999, az: 0 };
  for (var i = 0; i < boxes.length; i++) {
    var ring = boxes[i].ring, h = boxes[i].h;
    for (var j = 0; j < ring.length; j++) {
      var e = ring[j][0], n = ring[j][1], dist = Math.hypot(e, n);
      if (dist < 1) continue;
      var el = Math.atan2(h - 1.6 - (dist * dist * 0.87) / (2 * RE), dist) * R2D;
      if (el > peak.el) { peak.el = el; peak.az = ((Math.atan2(e, n) * R2D) + 360) % 360; }
    }
  }
  return peak;
}

/* browser: nearest building boxes for the observer + a coverage summary.
   Cached per ~100 m of observer position + settings. */
const bldgCache = new Map();
export const BLDG_RADIUS_M = 1200; // fetch/draw radius — smaller than terrain: a
// 2.5 km `out geom` in a city is many MB and rate-limits the public Overpass
// mirrors (especially after a server redeploy clears the cache); near rooftops
// are what matter for alignment anyway.
export function predictedBuildingBoxes(lat, lon, radiusM = BLDG_RADIUS_M, opts) {
  var assumeM = opts && opts.assumeM != null ? opts.assumeM : 6; // untagged footprints → ~2 storeys
  var capN = opts && opts.capN != null ? opts.capN : 160;
  var key = `${lat.toFixed(3)},${lon.toFixed(3)},${Math.round(radiusM)},${assumeM},${capN}`;
  if (bldgCache.has(key)) return bldgCache.get(key);
  var p = (async () => {
    var bl = await fetchBuildings(lat, lon, radiusM, { assumeM: assumeM });
    var boxes = buildingBoxes(bl, { maxM: radiusM, capN: capN, minM: 12 });
    return {
      boxes: boxes, peak: boxesPeak(boxes),
      buildings: { n: bl.buildings.length, shown: boxes.length, known: bl.known, est: bl.est, assumed: bl.assumed, dropped: bl.dropped },
    };
  })();
  bldgCache.set(key, p);
  // Don't keep an EMPTY result cached: 0 buildings is often a busy-Overpass
  // hiccup (mirrors answer 200 [] under load), not genuine absence — so a
  // toggle-off/on retries the fetch instead of being stuck on "none found".
  p.then((r) => { if (!r || r.buildings.n === 0) bldgCache.delete(key); }, () => bldgCache.delete(key));
  return p;
}
