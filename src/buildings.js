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

import { D2R } from "./math/geodesy.js";
import { skylineFromSampler, demSampler } from "./terrain.js";

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
     { buildings: [{ ring: [[e,n],…] metres, h: metres, est: bool,
                     bbox: [e0,n0,e1,n1] }], dropped, est }
   `dropped` counts building ways with a footprint but no usable height.
   ENU uses the SAME equirectangular constants terrain.js's demSampler does,
   so the height field composites exactly onto the DEM ground. */
export function parseOverpassBuildings(json, obsLat, obsLon) {
  var mLat = 111320, mLon = 111320 * Math.max(0.2, Math.cos(obsLat * D2R));
  var out = [], dropped = 0, est = 0;
  var els = (json && json.elements) || [];
  for (var k = 0; k < els.length; k++) {
    var el = els[k];
    if (el.type !== "way" || !el.tags || el.tags.building == null) continue;
    var geom = el.geometry;
    if (!Array.isArray(geom) || geom.length < 3) continue;
    var hm = heightMeters(el.tags);
    if (hm == null) { dropped++; continue; }
    var e0 = Infinity, n0 = Infinity, e1 = -Infinity, n1 = -Infinity, ring = [];
    for (var g = 0; g < geom.length; g++) {
      var pt = geom[g];
      if (pt.lat == null || pt.lon == null) continue;
      var e = (pt.lon - obsLon) * mLon, n = (pt.lat - obsLat) * mLat;
      ring.push([e, n]);
      if (e < e0) e0 = e; if (n < n0) n0 = n; if (e > e1) e1 = e; if (n > n1) n1 = n;
    }
    if (ring.length < 3) continue;
    if (hm.est) est++;
    out.push({ ring: ring, h: hm.m, est: hm.est, bbox: [e0, n0, e1, n1] });
  }
  return { buildings: out, dropped: dropped, est: est };
}

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
   ground it stands on, or 0 where no footprint covers the point. Buildings
   whose nearest edge is beyond `maxM` are pruned up front (city silhouettes
   are near, and it keeps the per-sample loop cheap during the ray-march). */
export function buildingHeightSampler(buildings, maxM = 4000) {
  var lim2 = maxM * maxM;
  var near = buildings.filter(function (b) { return bboxDist2(b.bbox) <= lim2; });
  return function (e, n) {
    var hi = 0;
    for (var i = 0; i < near.length; i++) {
      var b = near[i];
      if (b.h > hi && inRing(b.ring, b.bbox, e, n)) hi = b.h;
    }
    return hi;
  };
}

/* browser fetch → parsed buildings in the observer's ENU frame */
export async function fetchBuildings(lat, lon, radiusM = 2500) {
  var r = await fetch(`/api/buildings?lat=${lat.toFixed(5)}&lon=${lon.toFixed(5)}&r=${Math.round(radiusM)}`, { signal: AbortSignal.timeout(30000) });
  if (!r.ok) throw new Error(`buildings HTTP ${r.status}`);
  var j = await r.json();
  if (j.error) throw new Error(j.error);
  return parseOverpassBuildings(j, lat, lon);
}

/* Predicted skyline INCLUDING urban rooftops: DEM ground + OSM building
   heights through the shared ray-march. Same return shape as terrain's
   predictedSkyline ({ els, dists, ridges, h0 }) PLUS a `buildings` coverage
   summary { n, dropped, est }, so the sky view draws and snaps to it with no
   changes. Cached per ~100 m of observer position + radius. */
const urbanCache = new Map();
export function predictedUrbanSkyline(lat, lon, radiusM = 2500) {
  var key = `${lat.toFixed(3)},${lon.toFixed(3)},${Math.round(radiusM)}`;
  if (urbanCache.has(key)) return urbanCache.get(key);
  var p = (async () => {
    var pair = await Promise.all([demSampler(lat, lon), fetchBuildings(lat, lon, radiusM)]);
    var ground = pair[0].sampleEN, h0 = pair[0].h0, bl = pair[1];
    var bh = buildingHeightSampler(bl.buildings, radiusM);
    var composite = function (e, n) { var gnd = ground(e, n); return gnd == null ? null : gnd + bh(e, n); };
    var sk = skylineFromSampler(composite, h0);
    return Object.assign({}, sk, { h0: h0, buildings: { n: bl.buildings.length, dropped: bl.dropped, est: bl.est } });
  })();
  urbanCache.set(key, p);
  p.catch(() => urbanCache.delete(key));
  return p;
}
