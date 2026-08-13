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

/* real roadway width in metres — an explicit OSM width tag wins, then
   lanes × 3.4 m + shoulders, then a per-class default. This is what turns
   the centerline into a RIBBON whose edges converge like the road in the
   photo (field ask: "the road you are on should look big"). */
export function roadWidthM(tags) {
  var w = tags && tags.width != null ? parseFloat(String(tags.width).replace(/[^0-9.]/g, "")) : NaN;
  if (isFinite(w) && w > 1 && w < 60) return w;
  var lanes = tags && tags.lanes != null ? parseFloat(String(tags.lanes)) : NaN;
  if (isFinite(lanes) && lanes >= 1 && lanes <= 12) return lanes * 3.4 + 1.2;
  var h = String((tags && tags.highway) || "");
  if (/^motorway/.test(h)) return 19;
  if (/^trunk/.test(h)) return 12;
  if (/^primary/.test(h)) return 10;
  if (/^secondary/.test(h)) return 9;
  if (/^tertiary/.test(h)) return 8;
  if (/^(service|track)/.test(h)) return 4;
  return 7; // residential / unclassified / living_street
}

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
    var w = roadWidthM(el.tags);
    var run = [], last = null, dMin = Infinity;
    var flush = function () {
      if (run.length >= 2) all.push({ pts: run, major: major, w: w, name: el.tags.name || null, dMin: dMin });
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

/* ENU polylines → sight-line RIBBONS. Each road becomes a center polyline
   plus LEFT/RIGHT edge polylines offset half the real roadway width
   perpendicular to the local direction — that is what makes the road you
   stand on fill the frame and converge to its true vanishing point instead
   of reading as a thin thread. Every vertex gains {az, d, gz}; segments
   near the camera are DENSIFIED (perspective magnifies close range — a
   40 m Overpass segment is a visible chord at 30 m out), and with a DEM
   sampler present, vertices whose ray is blocked by nearer terrain are
   dropped (splitting the ribbon there — hills genuinely hide roads).
   sample(e,n)→m or null; h0 = observer ground elevation (m). */
export function roadSightlines(parsed, sample, h0, opts) {
  var eyeM = (opts && opts.eyeM) || 1.6;
  var eyeAbs = Math.max(0, h0) + eyeM;
  var occluded = function (azR, d, elDeg) {
    if (!sample) return false;
    var sa = Math.sin(azR), ca = Math.cos(azR);
    for (var m = Math.max(60, d * 0.08); m < d * 0.93; m *= 1.3) {
      var h = sample(sa * m, ca * m);
      if (h == null) continue;
      /* Ground-lock the march but only ever DOWNWARD: a near-field DEM
         wobble above eye level would otherwise wall off everything behind
         it (the terrain.js foreground-berm lesson) — while RAISING samples
         to h0 would fabricate an apron that erases a genuinely visible
         valley road seen from a ridge. min() kills berms, keeps valleys. */
      var h0m = Math.max(0, h);
      var hB = Math.min(h0m, gzBlend(h0m, m));
      var elT = Math.atan2(hB - eyeAbs - (m * m * (1 - K_REFR)) / (2 * RE), m) * R2D;
      if (elT > elDeg + 0.05) return true;
    }
    return false;
  };
  /* near-field subdivision: split long segments while either end is close.
     Perspective magnifies close range hard — the nearest vertex sets where
     the drawn ribbon STOPS at the bottom of the frame, so the near road is
     cut to ~6 m steps and allowed to run in to a couple of metres. */
  var densify = function (pts) {
    var out = [];
    for (var i = 0; i < pts.length; i++) {
      out.push(pts[i]);
      var a = pts[i], b = pts[i + 1];
      if (!b) break;
      var dNear = Math.min(Math.hypot(a[0], a[1]), Math.hypot(b[0], b[1]));
      var segL = Math.hypot(b[0] - a[0], b[1] - a[1]);
      if (dNear < 400 && segL > 12) {
        var steps = Math.min(24, Math.floor(segL / 6));
        for (var s = 1; s < steps; s++) out.push([a[0] + ((b[0] - a[0]) * s) / steps, a[1] + ((b[1] - a[1]) * s) / steps]);
      }
    }
    return out;
  };
  /* NEAR-FIELD GROUND LOCK: the DEM is a ~20 m grid — at 50 m out a one-cell
     wobble is metres of height, which visibly floats or sinks the very road
     the observer is STANDING ON. But that road IS the observer's ground, so
     the near field is locked to h0 and blended into the real DEM with
     distance (full DEM beyond ~380 m, where a metre is sub-line-width). */
  var lockA = (opts && opts.nearLockM) || 120, lockB = (opts && opts.farLockM) || 380;
  var gzBlend = function (gz, d) {
    var t = (d - lockA) / (lockB - lockA);
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    return h0 * (1 - t) + gz * t;
  };
  var sight = function (e, n, gz) {
    var d = Math.hypot(e, n);
    var azR = Math.atan2(e, n);
    return { az: ((azR * R2D) + 360) % 360, d: d, gz: gz };
  };
  var polys = [];
  var roads = (parsed && parsed.roads) || [];
  for (var i = 0; i < roads.length; i++) {
    var rd = roads[i], run = [];
    var wHalf = ((rd.w || 7) / 2);
    var flush = function () {
      if (run.length < 2) { run = []; return; }
      /* per-vertex left normal = average of adjacent segment directions */
      var L = [], R = [], C = [];
      for (var q = 0; q < run.length; q++) {
        var p = run[q];
        var pa = run[Math.max(0, q - 1)], pb = run[Math.min(run.length - 1, q + 1)];
        var dx = pb.e - pa.e, dy = pb.n - pa.n;
        var len = Math.hypot(dx, dy) || 1;
        var nx = -dy / len, ny = dx / len;
        C.push(sight(p.e, p.n, p.gz));
        L.push(sight(p.e + nx * wHalf, p.n + ny * wHalf, p.gz));
        R.push(sight(p.e - nx * wHalf, p.n - ny * wHalf, p.gz));
      }
      polys.push({ v: C, L: L, R: R, w: rd.w || 7, major: rd.major, name: rd.name, dMin: rd.dMin });
      run = [];
    };
    var pts = densify(rd.pts);
    for (var j = 0; j < pts.length; j++) {
      var e = pts[j][0], n = pts[j][1];
      var d = Math.hypot(e, n);
      if (d < 1.5) { flush(); continue; }            // under the camera — azimuth undefined
      var gz = sample ? sample(e, n) : null;
      if (gz == null) gz = h0;
      gz = gzBlend(gz, d);
      var azR = Math.atan2(e, n);
      var el = Math.atan2(Math.max(0, gz) - eyeAbs - (d * d * (1 - K_REFR)) / (2 * RE), d) * R2D;
      if (occluded(azR, d, el)) { flush(); continue; }
      run.push({ e: e, n: n, gz: gz });
    }
    flush();
  }
  return polys;
}

/* Where does a sight-line cross the mapped roads? Each crossing carries the
   road's name/class, its ground distance, and the ROAD POINT's own elevation
   angle — a low night light on that bearing has a mundane candidate first: a
   vehicle on that road (headlights cresting a rise read as a hovering or
   moving light). Pure: walks the derived center polylines for an azimuth
   sign change around the sight-line, interpolates the crossing, and keeps
   the nearest crossing per road. */
export function roadCrossings(polys, azDeg, eyeAbs) {
  var out = [];
  var wrap = function (x) { return ((x % 360) + 540) % 360 - 180; };
  for (var i = 0; i < (polys || []).length; i++) {
    var poly = polys[i], v = poly.v || [], best = null;
    for (var q = 0; q + 1 < v.length; q++) {
      var a = wrap(v[q].az - azDeg), b = wrap(v[q + 1].az - azDeg);
      if ((a >= 0) === (b >= 0) || Math.abs(a) + Math.abs(b) > 30) continue; // no crossing / wrap artifact
      var t = Math.abs(a) / (Math.abs(a) + Math.abs(b));
      var d = v[q].d + (v[q + 1].d - v[q].d) * t;
      var gz = v[q].gz + (v[q + 1].gz - v[q].gz) * t;
      var el = roadElOf({ d: d, gz: gz }, eyeAbs);
      if (!best || d < best.d) best = { d: d, el: el };
    }
    if (best) out.push({ name: poly.name || null, major: !!poly.major, d: best.d, el: best.el });
  }
  out.sort(function (x, y) { return x.d - y.d; });
  return out;
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
