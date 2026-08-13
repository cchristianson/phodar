/* ============================================================
   ROAD OVERLAY — near-field OSM road centerlines in true perspective.
   A road is one of the best azimuth anchors a daytime photo has (field
   ask: a highway shot straight down the centerline), and unlike the DEM
   skyline it works on FLAT terrain where there is no ridge to snap to.
   Centerlines within ~2.5 km are projected from the observer's eye —
   each vertex gets its ground elevation from the SAME cached DEM tiles
   the terrain skyline uses, with the same curvature+refraction constant —
   so the perspective is real, not a flat-map guess.

   Occlusion is approximate BY DESIGN and computed once at derive time:
   each vertex is ray-marched against the DEM (a dozen samples) and
   dropped when nearer terrain rises above its sight-line. That hides a
   road behind a hill, but a re-nudged camera height won't re-run it —
   at ±few metres of eye the error is sub-line-width.

   Data: OSM via Overpass, proxied through /api/roads (CORS on the
   mirrors is unreliable — the same reason /api/peaks and /api/buildings
   are proxied). Parsing + projection are pure and mathcheck-asserted.
   ============================================================ */

import { D2R, R2D, RE } from "./math/geodesy.js";
import { demSampler } from "./terrain.js";

export const ROADS_ATTRIB = "Roads: OpenStreetMap contributors (ODbL)";
const MAJOR = /^(motorway|trunk|primary|secondary)/;
const K_REFR = 0.13; // curvature+refraction, same constant as skylineFromSampler

/* Overpass `out geom` JSON → road polylines in the observer's ENU frame:
     { roads: [{ pts: [[e,n],…] metres, major, name }], n, shown, vtx }
   Ways are CLIPPED to maxM (a way running out of range splits into the
   in-range runs), simplified to ~simplifyM vertex spacing (sub-line-width
   at dome scale; endpoints always kept), and the total vertex budget is
   spent nearest-first so a dense town can't flood the renderer. */
export function parseOverpassRoads(json, obsLat, obsLon, opts) {
  var maxM = (opts && opts.maxM) || 2600, simplifyM = (opts && opts.simplifyM) || 12, capV = (opts && opts.capV) || 2400;
  var mLat = 111320, mLon = 111320 * Math.max(0.2, Math.cos(obsLat * D2R));
  var all = [], n = 0;
  var els = (json && json.elements) || [];
  for (var k = 0; k < els.length; k++) {
    var el = els[k];
    if (el.type !== "way" || !el.tags || el.tags.highway == null) continue;
    var geom = el.geometry;
    if (!Array.isArray(geom) || geom.length < 2) continue;
    n++;
    var major = MAJOR.test(String(el.tags.highway));
    var run = [], last = null, dMin = Infinity;
    var flush = function () {
      if (run.length >= 2) all.push({ pts: run, major: major, name: el.tags.name || null, dMin: dMin });
      run = []; last = null; dMin = Infinity;
    };
    for (var g = 0; g < geom.length; g++) {
      var pt = geom[g];
      if (pt.lat == null || pt.lon == null) { flush(); continue; }
      var e = (pt.lon - obsLon) * mLon, nn = (pt.lat - obsLat) * mLat;
      var d = Math.hypot(e, nn);
      if (d > maxM) { flush(); continue; }
      if (d < dMin) dMin = d;
      if (last && Math.hypot(e - last[0], nn - last[1]) < simplifyM && g < geom.length - 1) continue;
      run.push([e, nn]); last = [e, nn];
    }
    flush();
  }
  /* nearest roads first; spend the vertex budget there */
  all.sort(function (a, b) { return a.dMin - b.dMin; });
  var out = [], vtx = 0;
  for (var i = 0; i < all.length; i++) {
    if (vtx + all[i].pts.length > capV) break;
    vtx += all[i].pts.length;
    out.push(all[i]);
  }
  return { roads: out, n: n, shown: out.length, vtx: vtx };
}

/* elevation angle of a stored vertex from an ABSOLUTE eye elevation (m).
   Split out so the renderers can honour a live camera-height nudge without
   re-deriving; gz is clamped ≥0 like the skyline's sea-level rule. */
export function roadElOf(p, eyeAbs) {
  return Math.atan2(Math.max(0, p.gz) - eyeAbs - (p.d * p.d * (1 - K_REFR)) / (2 * RE), p.d) * R2D;
}

/* ENU polylines → sight-line polylines: each vertex gains {az, d, gz}, and
   with a DEM sampler present, vertices whose ray is blocked by nearer
   terrain are dropped (splitting the polyline there — an occluded stretch
   emits nothing, so hills genuinely hide roads). sample(e,n)→m or null;
   h0 = observer ground elevation (m). */
export function roadSightlines(parsed, sample, h0, opts) {
  var eyeM = (opts && opts.eyeM) || 1.6;
  var eyeAbs = Math.max(0, h0) + eyeM;
  var occluded = function (azR, d, elDeg) {
    if (!sample) return false;
    var sa = Math.sin(azR), ca = Math.cos(azR);
    for (var m = Math.max(60, d * 0.08); m < d * 0.93; m *= 1.3) {
      var h = sample(sa * m, ca * m);
      if (h == null) continue;
      var elT = Math.atan2(Math.max(0, h) - eyeAbs - (m * m * (1 - K_REFR)) / (2 * RE), m) * R2D;
      if (elT > elDeg + 0.05) return true;
    }
    return false;
  };
  var polys = [];
  var roads = (parsed && parsed.roads) || [];
  for (var i = 0; i < roads.length; i++) {
    var rd = roads[i], v = [];
    var flush = function () { if (v.length >= 2) polys.push({ v: v, major: rd.major, name: rd.name, dMin: rd.dMin }); v = []; };
    for (var j = 0; j < rd.pts.length; j++) {
      var e = rd.pts[j][0], n = rd.pts[j][1];
      var d = Math.hypot(e, n);
      if (d < 3) { flush(); continue; }              // under the camera — azimuth undefined
      var gz = sample ? sample(e, n) : null;
      if (gz == null) gz = h0;
      var azR = Math.atan2(e, n);
      var el = Math.atan2(Math.max(0, gz) - eyeAbs - (d * d * (1 - K_REFR)) / (2 * RE), d) * R2D;
      if (occluded(azR, d, el)) { flush(); continue; }
      v.push({ az: ((azR * R2D) + 360) % 360, d: d, gz: gz });
    }
    flush();
  }
  return polys;
}

/* browser fetch → derived sight-line polylines, cached one promise per
   ~110 m cell (the same granularity as predictedSkyline / building boxes).
   The DEM sampler is the same cached tile promise the skyline uses — when
   it can't load (offline, Terrarium down) roads fall back to a flat-ground
   assumption at the observer's elevation, honestly flagged (`flat: true`). */
const _roadCache = new Map();
export function predictedRoadDirs(lat, lon, opts) {
  var key = lat.toFixed(3) + "," + lon.toFixed(3);
  var hit = _roadCache.get(key);
  if (hit) return hit;
  var p = (async function () {
    var r = await fetch(`/api/roads?lat=${lat.toFixed(5)}&lon=${lon.toFixed(5)}&r=2600`, { signal: AbortSignal.timeout(50000) });
    if (!r.ok) throw new Error(`roads HTTP ${r.status}`);
    var j = await r.json();
    if (j.error) throw new Error(j.error);
    var parsed = parseOverpassRoads(j, lat, lon, opts);
    var dem = await demSampler(lat, lon).catch(function () { return null; });
    var sample = dem ? dem.sampleEN : null;
    var h0 = dem ? dem.h0 : 0;
    return { polys: roadSightlines(parsed, sample, h0, opts), h0: h0, shown: parsed.shown, n: parsed.n, flat: !dem };
  })();
  p.catch(function () { _roadCache.delete(key); }); // errors retryable
  _roadCache.set(key, p);
  return p;
}
