// Exercises the REAL math core (src/math/*) — not a copy. A regression in
// triangulation, geodesy, or angular sizing fails `npm test` here.
import { D2R, R2D, RE, enuFromGeo, geoFromEnu, dirFromAzEl, sub, mag } from "../src/math/geodesy.js";
import { intersectLines, aspectSpan, covEllipse } from "../src/math/triangulate.js";
import { sunPos, moonFrac } from "../src/math/astro.js";
import { nearestLevel, balloonVerdict } from "../src/checks/winds.js";
import { rankCandidates, spanForAircraft } from "../src/checks/adsb.js";
import { trackDirections, sourceTrack, videoKinematics, stereoVideo, mixedStereo } from "../src/math/kinematics.js";
import { skylineFromSampler, skylineElAt, AZ_STEP, matchSkyline, detectSkyline } from "../src/terrain.js";
import { raDecToAzEl } from "../src/math/astro.js";
import { declination } from "../src/math/geomag.js";
import { parseMediaMeta } from "../src/exif.js";
import { planetPositions } from "../src/math/planets.js";
import { STARS } from "../src/math/starcat.js";
import { photoBasis, solveRollFov, pixToDirK, dirToPixK, solvePoseAnchors } from "../src/math/projection.js";
import { solveManualPoses, solvePose, hybridPath } from "../src/video/manualpose.js";
import { unit, dot, dirToAzEl } from "../src/math/geodesy.js";
import { parseLaunches, haversineKm } from "../src/checks/launches.js";
import { parseFireballs } from "../src/checks/fireballs.js";
import { parsePeaks, bearingDeg, distM } from "../src/checks/peaks.js";
import { heightMeters, parseOverpassBuildings, buildingHeightSampler, buildingBoxes, boxesPeak, convexHull2, segInsideHull, visibleSegs } from "../src/buildings.js";
import { detectStars, autoStarAlign, blindStarAlign, gridStarAlign } from "../src/checks/platesolve.js";
import { detectBgFeatures, trackFeatures, poseFromTracks, initTracker, stepTracker, stepObject, snapToObject, smearDrift, despikePath, smoothPath, smoothObjPath, posePathAt, registerToRef, grayDown } from "../src/video/postrack.js";
import { rotZ3, rotY3, mul3, I3, quatFromMat3, mat3FromQuat, slerp3, sampleShapeAt } from "../src/shapes.js";
import { muxMp4 } from "../src/video/mp4mux.js";
import { cloudBaseAGL, cloudRangeBound } from "../src/checks/weather.js";
import { activeShowers } from "../src/checks/meteorshowers.js";
import { aperture, relMag, colorDesc } from "../src/checks/photometry.js";
import { parseAirports } from "../src/checks/airports.js";

let fails = 0;
const approx = (got, want, tol, msg) => {
  const ok = Math.abs(got - want) <= tol;
  if (!ok) { fails++; console.error(`  FAIL ${msg}: got ${got}, want ${want} ±${tol}`); }
  else console.log(`  ok   ${msg}: ${got.toFixed ? got.toFixed(2) : got}`);
  return ok;
};

// --- ENU round-trip: 2000 m due-east baseline ---
const ref = { lat: 42.16380, lon: -123.64800, alt: 0 };
const P1 = enuFromGeo(42.16380, -123.64800, 0, ref);
const P2 = enuFromGeo(42.16380, -123.62374, 0, ref);
approx(P2[0], 2000, 1.0, "east baseline");
approx(P2[1], 0, 0.1, "north component ~0");

// --- Fix A: two crossing bearings recover a known point ---
const A = intersectLines([
  { P: P1, d: dirFromAzEl(18.43, 32.31) },
  { P: P2, d: dirFromAzEl(341.57, 32.31) },
]);
approx(A.rmsMiss, 0, 0.5, "Fix A rms miss");
approx(A.ts[0], 3742, 3, "Fix A range from obs1");
approx(A.ts[1], 3742, 3, "Fix A range from obs2");

// --- angular size → linear span at that range ---
const ang = 0.612 * D2R;
approx(2 * A.ts[0] * Math.tan(ang / 2), 40, 1, "size from angle @obs1");

// --- Fix B and displacement magnitude (Moment-B velocity primitive) ---
const B = intersectLines([
  { P: P1, d: dirFromAzEl(23.43, 31.45) },
  { P: P2, d: dirFromAzEl(346.87, 33.00) },
]);
approx(B.rmsMiss, 0, 1, "Fix B rms miss");
approx(mag(sub(B.X, A.X)), 300, 2, "A→B displacement");

// --- ADS-B ranking: an aircraft planted ON the sight-line must rank ~0° sep ---
{
  const az = 18.43, el = 32.31, rng = 6000;
  const d = dirFromAzEl(az, el);
  // curvature correction in acAzElRange subtracts d²(1-k)/2R from z — pre-add it
  const dg = Math.hypot(d[0] * rng, d[1] * rng);
  const P = [d[0] * rng, d[1] * rng, d[2] * rng + (dg * dg * (1 - 0.13)) / (2 * RE)];
  const geo = geoFromEnu(P, { lat: 42.16380, lon: -123.64800, alt: 0 });
  const src = { name: "obs", lat: 42.16380, lon: -123.64800, alt: 0, A: { az, el } };
  const onLine = { hex: "aaaaaa", flight: "TEST1", t: "B738", lat: geo.lat, lon: geo.lon, altM: geo.alt };
  const offLine = { hex: "bbbbbb", flight: "TEST2", t: "A320", lat: geo.lat + 0.3, lon: geo.lon, altM: geo.alt };
  const ranked = rankCandidates([src], [offLine, onLine]);
  approx(ranked[0].sepMax, 0, 0.05, "on-line aircraft sep ~0°");
  if (ranked[0].hex !== "aaaaaa") { fails++; console.error("  FAIL ranking order: on-line aircraft should rank first"); }
  else console.log("  ok   ranking order: on-line aircraft first");
  approx(ranked[0].predAng, 2 * Math.atan(35.8 / 2 / rng) * R2D, 0.02, "predicted angular size (B738 @6km)");
  approx(spanForAircraft("B738").span, 35.8, 0, "wingspan lookup B738");
  approx(spanForAircraft(null, "A5").span, 62, 0, "category fallback A5");
}

// --- terrain skyline: synthetic cone mountain at a known bearing ---
{
  // flat plain at 100 m MSL + a cone at 10 km due EAST: peak 1100 m, radius 3 km
  const sample = (e, n) => {
    const d = Math.hypot(e - 10000, n);
    return 100 + (d < 3000 ? (1 - d / 3000) * 1000 : 0);
  };
  const { els } = skylineFromSampler(sample, 100);
  // expected peak elevation: atan((1100 − 101.6 − curvature) / 10000)
  const drop = (10000 * 10000 * 0.87) / (2 * 6371000);
  const expEl = Math.atan2(1100 - 101.6 - drop, 10000) * 180 / Math.PI;
  approx(skylineElAt(els, 90), expEl, 0.35, "cone peak el @az 90");
  // due west: flat ground → slightly negative horizon (curvature)
  const west = skylineElAt(els, 270);
  if (west < 0.05 && west > -1.5) console.log(`  ok   flat-ground horizon slightly negative: ${west.toFixed(2)}°`);
  else { fails++; console.error(`  FAIL flat horizon west: got ${west}`); }
  // ridge should span roughly the cone's angular width (±~16°), gone by ±25°
  if (skylineElAt(els, 90 + 25) < 0.3) console.log("  ok   cone absent 25° off-bearing");
  else { fails++; console.error("  FAIL cone leaked far off-bearing"); }
}

// --- ocean: DEM bathymetry (negative sea-floor depth, varying by azimuth &
//     distance) must render as the FLAT sea horizon, not a bumpy low ridge.
//     (Field report: coastal observer looking out to sea saw fake terrain.) ---
{
  // sea floor (always below sea level) deepening with distance + an azimuthal
  // ripple — raw, the running max makes a bumpy, too-low skyline; clamped to
  // 0 m (the water surface) it's flat
  const sample = (e, n) => { const d = Math.hypot(e, n); return -(5 + d * 0.03 + 15 * (1 + Math.sin(Math.atan2(e, n) * 3))); };
  const { els } = skylineFromSampler(sample, 2);
  let mn = 99, mx = -99, sum = 0;
  for (let i = 0; i < els.length; i++) { mn = Math.min(mn, els[i]); mx = Math.max(mx, els[i]); sum += els[i]; }
  approx(sum / els.length, -0.057, 0.05, "ocean skyline ≈ geometric sea-horizon dip");
  if (mx - mn < 0.02) console.log(`  ok   ocean skyline flat across azimuth (spread ${(mx - mn).toFixed(4)}°)`);
  else { fails++; console.error(`  FAIL ocean skyline bumpy: spread ${(mx - mn).toFixed(4)}°`); }
}

// --- near-shore foreground noise: a spurious berm ~3 m above eye a few dozen
//     metres away (coarse-DEM shoreline artifact) must NOT become a 2-3° fake
//     ridge over open water. The march skips the resolution-limited foreground. ---
{
  const eye = 4 + 1.6; // observer 4 m; berm 7 m within 150 m is DEM noise, sea beyond
  const sample = (e, n) => { const d = Math.hypot(e, n); return d < 150 ? 7 : 0; };
  const { els } = skylineFromSampler(sample, 4);
  let mx = -99; for (let i = 0; i < els.length; i++) mx = Math.max(mx, els[i]);
  if (mx < 0.3) console.log(`  ok   near-shore berm ignored, sea horizon flat (max ${mx.toFixed(2)}°)`);
  else { fails++; console.error(`  FAIL near-field berm leaked a ${mx.toFixed(2)}° fake ridge`); }
}

// --- layered ridges: a near crest in front of a tall far wall must survive
//     as an interior ridge line; a cone fully hidden behind the near crest
//     must emit NOTHING (visibility/occlusion is the whole feature) ---
{
  const cone = (e0, n0, r, h) => (e, n) => { const d = Math.hypot(e - e0, n - n0); return d < r ? (1 - d / r) * h : 0; };
  const near = cone(2000, 0, 800, 150);   // peak 250 m @ 2 km due east
  const hidden = cone(4000, 0, 800, 150); // peak 250 m @ 4 km — below the near sight-line
  const far = cone(12000, 0, 3000, 2000); // peak 2100 m @ 12 km — the skyline
  const sample = (e, n) => 100 + Math.max(near(e, n), hidden(e, n), far(e, n));
  const { els, ridges } = skylineFromSampler(sample, 100);
  const curv = (d) => (d * d * 0.87) / (2 * 6371000);
  const farEl = Math.atan2(2100 - 101.6 - curv(12000), 12000) * 180 / Math.PI;
  approx(skylineElAt(els, 90), farEl, 0.6, "far wall is the skyline @az 90"); // tol covers where log-spaced samples land on the apex
  const nearEl = Math.atan2(250 - 101.6 - curv(2000), 2000) * 180 / Math.PI;
  const at90 = ridges.filter((r) => r.pts.some(([a, el]) => Math.abs(a - 90) < 1 && Math.abs(el - nearEl) < 0.5));
  if (at90.length) console.log(`  ok   near ridge visible under the far wall (${at90[0].pts.length} pts)`);
  else { fails++; console.error("  FAIL near ridge missing from interior layers"); }
  if (at90.length && Math.abs(at90[0].dist - 2000) < 500) console.log(`  ok   near ridge depth ~2 km: ${Math.round(at90[0].dist)} m`);
  else { fails++; console.error(`  FAIL near ridge depth: ${at90.length ? Math.round(at90[0].dist) : "n/a"} m`); }
  const span = at90.length ? at90[0].pts[at90[0].pts.length - 1][0] - at90[0].pts[0][0] : 0;
  if (span >= 8) console.log(`  ok   near ridge spans ${span.toFixed(1)}° of azimuth`);
  else { fails++; console.error(`  FAIL near ridge span too short: ${span.toFixed(1)}°`); }
  const leak = ridges.some((r) => r.dist > 3000 && r.dist < 6000 && r.pts.some(([a]) => Math.abs(a - 90) < 15));
  if (!leak) console.log("  ok   fully-occluded middle cone emits no ridge");
  else { fails++; console.error("  FAIL occluded cone leaked a ridge line"); }
  const phantom = ridges.some((r) => r.pts.some(([a]) => Math.abs(a - 270) < 20));
  if (!phantom) console.log("  ok   flat ground west has no ridge layers");
  else { fails++; console.error("  FAIL phantom ridge on flat ground"); }
}

// --- urban building silhouette: OSM footprints + heights composited onto the
//     DEM ground make the SAME skyline ray-march work over rooftops in town ---
{
  // height-tag parsing: metric, imperial, "N m", floor-count fallback, junk
  approx(heightMeters({ height: "45" }).m, 45, 1e-9, "height '45' → 45 m");
  approx(heightMeters({ height: "20 m" }).m, 20, 1e-9, "height '20 m' → 20 m");
  approx(heightMeters({ height: "82'" }).m, 82 * 0.3048, 1e-6, "height 82' → metres");
  const lv = heightMeters({ "building:levels": "3" });
  if (lv && Math.abs(lv.m - 9) < 1e-9 && lv.est) console.log("  ok   3 levels → 9 m (est)");
  else { fails++; console.error(`  FAIL levels→height: ${JSON.stringify(lv)}`); }
  if (heightMeters({ amenity: "cafe" }) == null) console.log("  ok   no height tag → null (dropped)");
  else { fails++; console.error("  FAIL untagged building got a height"); }

  // a synthetic Overpass response near an observer at 45°N, 122°W
  const oLat = 45, oLon = -122;
  const dN = (m) => oLat + m / 111320, dE = (m) => oLon + m / (111320 * Math.cos(oLat * D2R));
  // a 20 m square building centred 300 m due NORTH, 45 m tall
  const box = (n, e, half, tags, id) => ({
    type: "way", id, tags,
    geometry: [[e - half, n - half], [e + half, n - half], [e + half, n + half], [e - half, n + half], [e - half, n - half]]
      .map(([ee, nn]) => ({ lat: dN(nn), lon: dE(ee) })),
  });
  const oj = {
    elements: [
      box(300, 0, 10, { building: "yes", height: "45" }, 1),      // the silhouette
      box(200, -400, 10, { building: "house", "building:levels": "2" }, 2), // est, off to the west
      box(250, 300, 10, { building: "yes" }, 3),                    // NO height → dropped
      { type: "node", id: 9, lat: dN(100), lon: dE(0), tags: { building: "yes", height: "9" } }, // not a way → ignored
    ],
  };
  const parsed = parseOverpassBuildings(oj, oLat, oLon);
  if (parsed.buildings.length === 2 && parsed.dropped === 1 && parsed.est === 1)
    console.log(`  ok   parse: 2 placed, 1 dropped (no height), 1 estimated`);
  else { fails++; console.error(`  FAIL parse counts: ${JSON.stringify({ n: parsed.buildings.length, dropped: parsed.dropped, est: parsed.est })}`); }
  // the north building lands at ENU (~0 east, ~300 north)
  const b0 = parsed.buildings.find((b) => Math.abs(b.h - 45) < 1e-6);
  const cN = (b0.bbox[1] + b0.bbox[3]) / 2, cE = (b0.bbox[0] + b0.bbox[2]) / 2;
  if (Math.abs(cN - 300) < 2 && Math.abs(cE) < 2) console.log(`  ok   north building at ENU (${cE.toFixed(1)}, ${cN.toFixed(1)})`);
  else { fails++; console.error(`  FAIL building ENU: (${cE.toFixed(1)}, ${cN.toFixed(1)})`); }

  // composite the rooftop heights onto FLAT ground at 0 m and ray-march
  const bh = buildingHeightSampler(parsed.buildings);
  const composite = (e, n) => 0 + bh(e, n);
  const { els } = skylineFromSampler(composite, 0);
  const curv = (d) => (d * d * 0.87) / (2 * 6371000);
  const expN = Math.atan2(45 - 1.6 - curv(300), 300) * 180 / Math.PI; // eye 1.6 m
  approx(skylineElAt(els, 0), expN, 0.4, "rooftop silhouette el @az 0 (due N)");
  // off the building the skyline is the flat horizon (nothing to the east/south)
  if (Math.abs(skylineElAt(els, 90)) < 0.2 && Math.abs(skylineElAt(els, 180)) < 0.2)
    console.log("  ok   flat horizon where no building stands (E/S)");
  else { fails++; console.error(`  FAIL phantom rooftop off-bearing: E ${skylineElAt(els, 90).toFixed(2)}° S ${skylineElAt(els, 180).toFixed(2)}°`); }
  // the far pruning: a building beyond maxM contributes nothing to the field
  const far = buildingHeightSampler(parsed.buildings, 100); // 300 m building pruned out
  if (far(0, 300) === 0) console.log("  ok   building beyond maxM pruned from the height field");
  else { fails++; console.error("  FAIL distant building not pruned"); }

  // NEAR building (the house across the street) — must render when the urban
  // march starts close, and is SKIPPED by the DEM's 200 m foreground floor.
  // This is the exact field bug: "found 62 buildings but none rendered."
  const njson = { elements: [box(60, 0, 8, { building: "house", height: "10" }, 1)] }; // 10 m tall, 60 m due N
  const near = parseOverpassBuildings(njson, oLat, oLon);
  const nbh = buildingHeightSampler(near.buildings);
  const ncomp = (e, n) => 0 + nbh(e, n);
  const expNear = Math.atan2(10 - 1.6 - curv(60), 60) * 180 / Math.PI; // ≈ 8° at centre
  const fromClose = skylineElAt(skylineFromSampler(ncomp, 0, 35000, 8).els, 0);
  const fromFloor = skylineElAt(skylineFromSampler(ncomp, 0).els, 0); // default 200 m floor
  approx(fromClose, expNear, 1.2, "near house renders when marched from 8 m"); // tol spans the 16 m footprint depth
  if (fromFloor < 0.2) console.log(`  ok   same house invisible under the 200 m DEM floor (${fromFloor.toFixed(2)}°) — the bug`);
  else { fails++; console.error(`  FAIL near house leaked into the 200 m-floor march: ${fromFloor.toFixed(2)}°`); }

  // assumed heights: an UNTAGGED footprint (the warehouse next door) is dropped
  // by default but placed at the assumed height when asked — the coverage fix.
  const untagged = { elements: [box(80, 0, 12, { building: "warehouse" }, 1)] }; // no height/levels
  const noAssume = parseOverpassBuildings(untagged, oLat, oLon);
  const withAssume = parseOverpassBuildings(untagged, oLat, oLon, { assumeM: 6 });
  if (noAssume.buildings.length === 0 && noAssume.dropped === 1) console.log("  ok   untagged footprint dropped without assumeM");
  else { fails++; console.error(`  FAIL untagged drop: ${JSON.stringify({ n: noAssume.buildings.length, d: noAssume.dropped })}`); }
  if (withAssume.buildings.length === 1 && withAssume.assumed === 1 && withAssume.buildings[0].h === 6 && withAssume.buildings[0].assumed)
    console.log("  ok   untagged footprint placed at assumed 6 m with assumeM");
  else { fails++; console.error(`  FAIL assumeM place: ${JSON.stringify({ n: withAssume.buildings.length, a: withAssume.assumed, h: withAssume.buildings[0] && withAssume.buildings[0].h })}`); }

  // capN keeps the NEAREST footprints so a dense city can't stall the march
  const many = { elements: [box(50, 0, 5, { building: "yes", height: "10" }, 1), box(400, 0, 5, { building: "yes", height: "10" }, 2)] };
  const two = parseOverpassBuildings(many, oLat, oLon);
  const cap1 = buildingHeightSampler(two.buildings, 2500, 1); // keep nearest 1 only
  if (cap1(0, 50) === 10 && cap1(0, 400) === 0) console.log("  ok   capN keeps the nearest footprint, drops the farther");
  else { fails++; console.error(`  FAIL capN: near ${cap1(0, 50)}, far ${cap1(0, 400)}`); }

  // individual boxes: exclude the footprint the observer stands in (the "too
  // tall" near-spike from a window shot), skip < minM, sort near→far, cap.
  const scene = parseOverpassBuildings({ elements: [
    box(0, 0, 30, { building: "yes", height: "12" }, 1),     // observer INSIDE this one (origin covered)
    box(6, 0, 3, { building: "yes", height: "8" }, 2),       // 6 m away → inside minM, skipped
    box(300, 0, 10, { building: "yes", height: "20" }, 3),   // far, tall
    box(80, 0, 10, { building: "yes", height: "9" }, 4),     // near, short
  ] }, oLat, oLon);
  const boxes = buildingBoxes(scene, { maxM: 2500, capN: 160, minM: 12 });
  if (boxes.length === 2) console.log("  ok   boxes exclude observer's own footprint + the < 12 m neighbour");
  else { fails++; console.error(`  FAIL box count ${boxes.length} (want 2), dists ${boxes.map((b) => b.dist.toFixed(0))}`); }
  if (boxes[0] && Math.abs(boxes[0].dist - 80) < 15 && Math.abs(boxes[1].dist - 300) < 15)
    console.log("  ok   boxes sorted nearest → farthest");
  else { fails++; console.error(`  FAIL box order: ${boxes.map((b) => b.dist.toFixed(0))}`); }
  const cap = buildingBoxes(scene, { maxM: 2500, capN: 1, minM: 12 });
  if (cap.length === 1 && Math.abs(cap[0].dist - 80) < 15) console.log("  ok   buildingBoxes capN keeps only the nearest");
  else { fails++; console.error(`  FAIL box capN: ${cap.map((b) => b.dist.toFixed(0))}`); }
  // boxesPeak → the tallest rooftop in ANGLE: the NEAR 9 m building at ~71 m
  // (5.98°) out-angles the 20 m building at ~290 m (3.6°); the peak lands on
  // that footprint's nearest corner, ~8° off due north.
  const bpk = boxesPeak(boxes);
  const dCorner = Math.hypot(10, 70);
  const expPk = Math.atan2(9 - 1.6 - (dCorner * dCorner * 0.87) / (2 * 6371000), dCorner) * 180 / Math.PI;
  approx(bpk.el, expPk, 0.4, "boxesPeak tallest rooftop elevation (near building wins by angle)");
  if (Math.abs(((bpk.az + 180) % 360) - 180) < 12) console.log(`  ok   boxesPeak azimuth near due N (${bpk.az.toFixed(1)}°)`);
  else { fails++; console.error(`  FAIL boxesPeak az ${bpk.az}`); }
}

// --- building hidden-line removal: solid-then-wireframe occlusion ---
{
  // a unit square occluder [0,1]×[0,1]
  const sq = convexHull2([[0, 0], [1, 0], [1, 1], [0, 1], [0.5, 0.5]]); // interior pt dropped
  if (sq.length === 4) console.log("  ok   convexHull2 drops interior points (square → 4 corners)");
  else { fails++; console.error(`  FAIL hull size ${sq.length}: ${JSON.stringify(sq)}`); }

  // a horizontal segment crossing the square from x=-1 to x=2 at y=0.5:
  // inside interval is t where x∈[0,1] → t∈[1/3, 2/3]
  const iv = segInsideHull([-1, 0.5], [2, 0.5], sq);
  if (iv && Math.abs(iv[0] - 1 / 3) < 1e-6 && Math.abs(iv[1] - 2 / 3) < 1e-6) console.log("  ok   segInsideHull finds the occluded interval");
  else { fails++; console.error(`  FAIL segInsideHull ${JSON.stringify(iv)}`); }

  // a segment entirely above the square is never occluded
  if (segInsideHull([-1, 2], [2, 2], sq) == null) console.log("  ok   segment clear of the hull is not occluded");
  else { fails++; console.error("  FAIL clear segment reported occluded"); }

  // visibleSegs subtracts the occluded middle, leaving the two ends visible
  const vis = visibleSegs([-1, 0.5], [2, 0.5], [sq]);
  if (vis.length === 2 && Math.abs(vis[0][1] - 1 / 3) < 1e-6 && Math.abs(vis[1][0] - 2 / 3) < 1e-6)
    console.log("  ok   visibleSegs keeps the two ends, hides the covered middle");
  else { fails++; console.error(`  FAIL visibleSegs ${JSON.stringify(vis)}`); }

  // a segment fully inside the occluder disappears entirely (fully hidden edge)
  const gone = visibleSegs([0.2, 0.5], [0.8, 0.5], [sq]);
  if (gone.length === 0) console.log("  ok   fully-covered edge is removed");
  else { fails++; console.error(`  FAIL fully-covered edge survived ${JSON.stringify(gone)}`); }

  // an edge with no occluders stays whole (foremost building keeps full wireframe)
  const whole = visibleSegs([-1, 0.5], [2, 0.5], []);
  if (whole.length === 1 && whole[0][0] === 0 && whole[0][1] === 1) console.log("  ok   edge with no occluder stays whole");
  else { fails++; console.error(`  FAIL unoccluded edge changed ${JSON.stringify(whole)}`); }
}

// --- detectSkyline: the true horizon must win over foreground foliage.
//     (The reported failure: a tree canopy out-gradients a hazy distant
//     ridge, so the old strongest-edge detector locked onto branches and
//     the snap matched no azimuth. The fix finds the lowest sky/ground
//     boundary with sustained ground below, seeing THROUGH the foliage.) ---
{
  const W = 144, H = 100;
  const SKY = [150, 180, 220], RIDGE = [70, 90, 80], FIELD = [180, 165, 120], TREE = [25, 35, 20];
  const yh = (x) => Math.round(60 - 8 * Math.sin((Math.PI * x) / (W - 1))); // gentle bump ~52..60
  const build = (foliage) => {
    const px = new Uint8ClampedArray(W * H * 4);
    for (let x = 0; x < W; x++) {
      const h = yh(x);
      for (let y = 0; y < H; y++) {
        let c = y < h ? SKY : (y < h + 6 ? RIDGE : FIELD);
        // dark canopy in the side columns, ABOVE the horizon with sky showing
        // through beneath it — the case that fooled the old detector
        if (foliage && (x < 40 || x > 103) && y < 42) c = TREE;
        const i = (y * W + x) * 4; px[i] = c[0]; px[i + 1] = c[1]; px[i + 2] = c[2]; px[i + 3] = 255;
      }
    }
    return { data: px };
  };
  const rms = (pts) => { let s = 0; for (const p of pts) { const d = p.y - yh(p.x); s += d * d; } return Math.sqrt(s / pts.length); };
  const clean = detectSkyline(build(false), W, H);
  if (clean && clean.length >= 20 && rms(clean) < 3) console.log(`  ok   clean horizon detected (rms ${rms(clean).toFixed(2)} px, ${clean.length} pts)`);
  else { fails++; console.error(`  FAIL clean horizon: ${clean ? rms(clean).toFixed(2) + " px, " + clean.length + " pts" : "null"}`); }
  const fol = detectSkyline(build(true), W, H);
  if (fol && fol.length >= 20 && rms(fol) < 4) console.log(`  ok   ridge found under foreground foliage (rms ${rms(fol).toFixed(2)} px, ${fol.length} pts)`);
  else { fails++; console.error(`  FAIL foliage case: ${fol ? rms(fol).toFixed(2) + " px, " + fol.length + " pts" : "null"} — detector captured by the canopy`); }
}

// --- trajectory corner rounding: an arc beats a hard 90° corner ---
{
  const mk = (r) => ({ track: [{ t: 0, az: 0, el: 40 }, { t: 5, az: 45, el: 40, r }, { t: 10, az: 45, el: 5 }], A: {}, fovH: null });
  const hard = trackDirections(mk(0)), round = trackDirections(mk(0.4));
  if (hard.length === 3) console.log("  ok   r=0 keeps the hard corner (3 samples)");
  else { fails++; console.error("  FAIL hard corner sample count:", hard.length); }
  if (round.length > 6) console.log(`  ok   r=0.4 inserts arc samples (${round.length})`);
  else { fails++; console.error("  FAIL arc samples:", round.length); }
  if (round.every((p, i) => i === 0 || p.ct > round[i - 1].ct)) console.log("  ok   arc times strictly increasing");
  else { fails++; console.error("  FAIL arc times not monotonic"); }
  // geometric smoothing: the max single-step bend of the path must shrink.
  // (Kinematics on the COARSE hard corner under-reads the turn — the
  // instantaneous corner hides between samples; that's why arcs exist.)
  const maxBend = (dirs) => {
    let m = 0;
    for (let i = 1; i < dirs.length - 1; i++) {
      const u = sub(dirs[i].d, dirs[i - 1].d), v = sub(dirs[i + 1].d, dirs[i].d);
      const du = Math.hypot(...u), dv = Math.hypot(...v);
      if (du < 1e-9 || dv < 1e-9) continue;
      const c = (u[0] * v[0] + u[1] * v[1] + u[2] * v[2]) / (du * dv);
      m = Math.max(m, Math.acos(Math.min(1, Math.max(-1, c))) * 180 / Math.PI);
    }
    return m;
  };
  const bH = maxBend(hard), bR = maxBend(round);
  if (bR < bH * 0.45) console.log(`  ok   arc max bend ${bR.toFixed(1)}° ≪ hard corner ${bH.toFixed(1)}°`);
  else { fails++; console.error(`  FAIL rounding didn't soften the corner bend: ${bR} vs ${bH}`); }
}

// --- SNAP anchor: a pixel track point flagged `anchor` sits at A.az/el even
//     mid-path (the object was photographed anywhere along the recalled path) ---
{
  const base = { natW: 1000, natH: 1000, fovH: 60, A: { az: 100, el: 20 } };
  // no anchor → the FIRST point anchors at A.az/el (legacy behaviour)
  const d0 = trackDirections({ ...base, track: [{ t: 0, x: 400, y: 500 }, { t: 1, x: 500, y: 500 }, { t: 2, x: 600, y: 500 }] });
  if (d0 && Math.abs(d0[0].az - 100) < 1e-6 && Math.abs(d0[0].el - 20) < 1e-6) console.log("  ok   no anchor flag ⇒ first point sits at A.az/el");
  else { fails++; console.error("  FAIL default anchor:", d0 && [d0[0].az, d0[0].el]); }
  // MIDDLE point flagged anchor (at the image centre) ⇒ IT sits at A.az/el
  const d1 = trackDirections({ ...base, track: [{ t: 0, x: 400, y: 500 }, { t: 1, x: 500, y: 500, anchor: true }, { t: 2, x: 600, y: 500 }] });
  if (d1 && Math.abs(d1[1].az - 100) < 1e-6 && Math.abs(d1[1].el - 20) < 1e-6) console.log("  ok   mid-path `anchor` point snaps to the object's measured A.az/el");
  else { fails++; console.error("  FAIL mid-path anchor:", d1 && [d1[1].az, d1[1].el]); }
  // and the neighbours are OFFSET to opposite sides of it (monotet in az)
  if (d1 && d1[0].az < d1[1].az && d1[1].az < d1[2].az) console.log("  ok   neighbours offset either side of the anchored point");
  else { fails++; console.error("  FAIL anchor neighbours:", d1 && d1.map((p) => +p.az.toFixed(2))); }
}

// --- TURN RADIUS AT THE ANCHOR: rounding a corner whose vertex is the measured
//     object (anchor) must keep the arc ON that direction — a plain corner cuts
//     inside, the anchor must not (else the object "falls off" its placed spot) ---
{
  const vDir = dirFromAzEl(45, 40);                       // the corner vertex direction
  const minGap = (dirs) => {                              // closest arc approach to the vertex, deg
    let m = Infinity;
    for (const p of dirs) m = Math.min(m, Math.acos(Math.min(1, Math.max(-1, dot(unit(p.d), vDir)))) * R2D);
    return m;
  };
  const bend = (dirs) => {                                // max single-step bend along the path, deg
    let m = 0;
    for (let i = 1; i < dirs.length - 1; i++) {
      const u = sub(dirs[i].d, dirs[i - 1].d), w = sub(dirs[i + 1].d, dirs[i].d);
      const du = Math.hypot(...u), dw = Math.hypot(...w);
      if (du < 1e-9 || dw < 1e-9) continue;
      m = Math.max(m, Math.acos(Math.min(1, Math.max(-1, (u[0] * w[0] + u[1] * w[1] + u[2] * w[2]) / (du * dw)))) * R2D);
    }
    return m;
  };
  const trk = (anchor) => trackDirections({ A: {}, fovH: null,
    track: [{ t: 0, az: 0, el: 40 }, { t: 5, az: 45, el: 40, r: 0.4, anchor }, { t: 10, az: 45, el: 5 }] });
  const dPlain = trk(undefined), dAnchor = trk(true);
  const gapPlain = minGap(dPlain), gapAnchor = minGap(dAnchor);
  if (gapPlain > 1) console.log(`  ok   plain corner cuts inside the vertex (${gapPlain.toFixed(2)}° gap)`);
  else { fails++; console.error("  FAIL plain corner didn't cut:", gapPlain); }
  if (gapAnchor < 0.05) console.log(`  ok   anchored corner: wide-turn arc still passes THROUGH the object (${gapAnchor.toFixed(3)}° gap)`);
  else { fails++; console.error("  FAIL anchored turn falls off the object:", gapAnchor); }
  // ...AND smoothly: no kink at the leg joins or the anchor. The earlier
  // reflected-control quad passed through the object but bent ~20° at the joins;
  // the two-cubic path must be no less smooth than the plain rounded corner.
  const bAnchor = bend(dAnchor), bPlain = bend(dPlain);
  if (bAnchor <= bPlain + 1) console.log(`  ok   anchored corner stays smooth through the object (max bend ${bAnchor.toFixed(1)}° ≤ plain ${bPlain.toFixed(1)}°)`);
  else { fails++; console.error(`  FAIL anchored corner kinks: ${bAnchor.toFixed(1)}° vs plain ${bPlain.toFixed(1)}°`); }
}

// --- multi-moment track: placed primary + moments become a time-ordered
//     angular trajectory; a single placed shot falls back to the manual track ---
{
  // one observer, one manual track point drawn on a single photo: no moments →
  // sourceTrack must return the hand-drawn track untouched.
  const single = { whenMs: 1000, A: { az: 10, el: 20 }, moments: [],
    track: [{ t: 0, az: 10, el: 20 }, { t: 3, az: 14, el: 22 }] };
  const st1 = sourceTrack(single);
  if (st1.length === 2 && st1[0].az === 10 && st1[1].t === 3) console.log("  ok   single-photo track passes through unchanged");
  else { fails++; console.error("  FAIL single-photo passthrough:", JSON.stringify(st1)); }

  // primary + two moments, placed at 0/4/10 s (out of order in the array) →
  // three points, sorted by time, t rebased to the earliest shot.
  const multi = {
    whenMs: 40_000, A: { az: 50, el: 30 }, natW: 4000, natH: 3000, fovH: 65,
    track: [{ t: 0, az: 999, el: 999 }], // must be IGNORED once ≥2 shots exist
    moments: [
      { whenMs: 30_000, A: { az: 40, el: 25 } },
      { whenMs: 60_000, A: { az: 66, el: 41 } },
    ],
  };
  const st = sourceTrack(multi);
  if (st.length === 3 && st[0].t === 0 && st[1].t === 10 && st[2].t === 30
      && st[0].az === 40 && st[2].az === 66) console.log("  ok   primary+moments → 3 time-sorted points, t rebased");
  else { fails++; console.error("  FAIL multi-moment assembly:", JSON.stringify(st)); }

  // and the assembled track drives the real direction pipeline (≥2 dirs).
  const dirs = trackDirections(multi);
  if (dirs && dirs.length >= 3 && Math.abs(dirs[0].az - 40) < 1e-6) console.log("  ok   moment track feeds trackDirections");
  else { fails++; console.error("  FAIL moment trackDirections:", dirs && dirs.length); }

  // hybrid: a hand-drawn point BETWEEN two placed photos interleaves in time.
  // primary @40s (t0), one extra moment @60s (→20s); a drawn point at +8s from
  // the primary mark must land at t=8, between the primary (0) and moment (20).
  const hybrid = {
    whenMs: 40_000, A: { az: 50, el: 30 }, natW: 4000, natH: 3000, fovH: 65,
    track: [
      { t: 0, az: 50, el: 30 },     // the primary-photo mark (Moment 1)
      { t: 8, az: 58, el: 34 },     // a filled-in point 8 s later, no photo
    ],
    moments: [{ whenMs: 60_000, A: { az: 66, el: 41 } }],
  };
  const h = sourceTrack(hybrid);
  if (h.length === 3 && h[0].t === 0 && h[1].t === 8 && h[2].t === 20
      && h[2].az === 66) console.log("  ok   hybrid: drawn point interleaves between two placed photos");
  else { fails++; console.error("  FAIL hybrid interleave:", JSON.stringify(h)); }

  // hybrid must NOT trigger for a plain drawn track with no extra moments —
  // that stays the untouched single-photo path.
  const plain = { whenMs: 40_000, A: { az: 50, el: 30 }, moments: [],
    track: [{ t: 0, az: 50, el: 30 }, { t: 2, az: 51, el: 31 }] };
  const pl = sourceTrack(plain);
  if (pl.length === 2 && pl[1].t === 2) console.log("  ok   drawn track with no moments stays unchanged");
  else { fails++; console.error("  FAIL drawn passthrough with primary placed:", JSON.stringify(pl)); }
}

// --- skyline snap: recover a known pose offset from a synthetic ridge ---
{
  // "DEM": a wavy ridge line; "photo": samples of it seen from a camera whose
  // pose is off by known dAz/dEl/roll, plus deterministic noise
  const elAt = (az) => 4 + 3 * Math.sin(az * 3 * Math.PI / 180) + 1.2 * Math.sin(az * 11 * Math.PI / 180);
  const TRUE = { dAz: 7.3, dEl: -2.1, rollRad: 1.5 * Math.PI / 180 };
  const samples = [];
  for (let i = 0; i < 60; i++) {
    const thx = ((i / 59) - 0.5) * (60 * Math.PI / 180); // ±30° horizontal FOV
    const azPhoto = 200 + thx * 180 / Math.PI;           // camera thinks it points here
    const azTrue = azPhoto + TRUE.dAz;                   // world truth
    const noise = 0.05 * Math.sin(i * 12.9898);          // deterministic ±0.05°
    // what the camera MEASURES for that column: true ridge el, shifted by its
    // el error and tilted by its roll error
    const elPhoto = elAt(azTrue) - TRUE.dEl + TRUE.rollRad * thx * 180 / Math.PI + noise;
    samples.push({ az: azPhoto, el: elPhoto, thx });
  }
  const m = matchSkyline(samples, elAt);
  approx(m.dAz, TRUE.dAz, 0.15, "skyline snap dAz recovery");
  approx(m.dEl, TRUE.dEl, 0.1, "skyline snap dEl recovery");
  approx(m.dRollDeg, TRUE.rollRad * 180 / Math.PI, 0.3, "skyline snap roll recovery (deg)");
  if (m.rms < 0.12) console.log(`  ok   snap fit rms ${m.rms.toFixed(3)}° (noise floor)`);
  else { fails++; console.error("  FAIL snap rms too high:", m.rms); }
}

// --- WMM2025 declination vs official NOAA test vectors ---
{
  // rows from WMM2025_TEST_VALUES.txt (decimal year, alt km, lat, lon → dec°)
  const d2025 = new Date(Date.UTC(2025, 0));
  approx(declination(89, -121, 28000, d2025), -99.77, 0.02, "WMM decl 89N 121W");
  approx(declination(-33, 109, 51000, d2025), -5.49, 0.02, "WMM decl 33S 109E");
  approx(declination(-66, -5, 37000, new Date(Date.UTC(2027, 0))), -17.22, 0.02, "WMM decl 2027.0 66S 5W");
}

// --- EXIF parser: hand-built JPEG with known GPS/bearing/focal must parse.
//     (Guards the src/exif.js module boundary: a missing import there fails
//     silently through parseMediaMeta's try/catch — exactly the bug that
//     killed GPS autofill once.) ---
{
  const tiff = new Uint8Array(190);
  const dv = new DataView(tiff.buffer);
  const w16 = (o, v) => dv.setUint16(o, v, true), w32 = (o, v) => dv.setUint32(o, v, true);
  tiff[0] = 0x49; tiff[1] = 0x49; w16(2, 42); w32(4, 8);          // II, magic, IFD0@8
  w16(8, 2);                                                       // IFD0: 2 entries
  w16(10, 0x8769); w16(12, 4); w32(14, 1); w32(18, 38);            // → Exif IFD @38
  w16(22, 0x8825); w16(24, 4); w32(26, 1); w32(30, 56);            // → GPS IFD @56
  w32(34, 0);
  w16(38, 1);                                                      // Exif IFD: 1 entry
  w16(40, 0xA405); w16(42, 3); w32(44, 1); w16(48, 26);            // f35 = 26 mm
  w32(52, 0);
  w16(56, 6);                                                      // GPS IFD: 6 entries
  const gpsE = (i, tag, type, cnt, val, inline) => {
    const o = 58 + i * 12;
    w16(o, tag); w16(o + 2, type); w32(o + 4, cnt);
    if (inline != null) { tiff[o + 8] = inline.charCodeAt(0); tiff[o + 9] = 0; }
    else w32(o + 8, val);
  };
  gpsE(0, 1, 2, 2, 0, "N");
  gpsE(1, 2, 5, 3, 134);
  gpsE(2, 3, 2, 2, 0, "W");
  gpsE(3, 4, 5, 3, 158);
  gpsE(4, 16, 2, 2, 0, "M");
  gpsE(5, 17, 5, 1, 182);
  w32(130, 0);
  const rat = (o, n, d) => { w32(o, n); w32(o + 4, d); };
  rat(134, 42, 1); rat(142, 9, 1); rat(150, 4968, 100);            // 42°9'49.68" = 42.1638
  rat(158, 123, 1); rat(166, 38, 1); rat(174, 5280, 100);          // 123°38'52.8" = 123.648
  rat(182, 1234, 10);                                              // bearing 123.4°M
  const jpeg = new Uint8Array(4 + 2 + 6 + 190 + 2);
  jpeg.set([0xFF, 0xD8, 0xFF, 0xE1, 0x00, 198 & 0xFF], 0);
  jpeg[4] = 198 >> 8; jpeg[5] = 198 & 0xFF;
  jpeg.set([0x45, 0x78, 0x69, 0x66, 0, 0], 6);                     // "Exif\0\0"
  jpeg.set(tiff, 12);
  const m = parseMediaMeta(jpeg.buffer, false);
  if (!m) { fails++; console.error("  FAIL EXIF: parseMediaMeta returned null (module boundary broken?)"); }
  else {
    approx(m.lat, 42.1638, 0.0001, "EXIF GPS lat");
    approx(m.lon, -123.648, 0.0001, "EXIF GPS lon");
    approx(m.az, 123.4, 0.01, "EXIF bearing");
    if (m.azRef === "magnetic") console.log("  ok   EXIF bearing flagged magnetic");
    else { fails++; console.error("  FAIL EXIF azRef:", m.azRef); }
    approx(m.fovH, 2 * Math.atan(18 / 26) * R2D, 0.2, "EXIF f35→FOV (the line that broke)");
  }
}

// --- planets vs JPL Horizons ground truth (fetched 2026-07-14) ---
{
  const ms = Date.UTC(2026, 6, 14, 18, 0, 0);
  const ps = Object.fromEntries(planetPositions(ms, 0, 0).map((p) => [p.name, p]));
  approx(ps.Venus.ra, 157.819, 0.05, "Venus RA vs Horizons");
  approx(ps.Venus.dec, 10.582, 0.05, "Venus Dec vs Horizons");
  approx(ps.Saturn.ra, 14.413, 0.06, "Saturn RA vs Horizons (perturbed)");
  approx(ps.Saturn.dec, 3.508, 0.05, "Saturn Dec vs Horizons");
}

// --- star transit geometry: Sirius culminates at alt = 90 − lat + dec, az 180 ---
{
  const sirius = STARS.find((s) => s[3] === "Sirius");
  approx(sirius[0], 101.287, 0.01, "Sirius catalog RA");
  const lat = 40, lng = 0;
  let best = { alt: -99 };
  const day0 = Date.UTC(2026, 6, 14);
  for (let m = 0; m < 1440; m++) {
    const p = raDecToAzEl(sirius[0], sirius[1], day0 + m * 60000, lat, lng);
    if (p.alt > best.alt) best = p;
  }
  approx(best.alt, 90 - lat + sirius[1], 0.15, "Sirius transit altitude");
  approx(Math.abs(((best.az - 180 + 540) % 360) - 180), 0, 1.5, "Sirius transit azimuth ~180");
}

// --- two-tap star align (solveRollFov): recover roll + FOV from one anchor ---
// A photo is placed with a WRONG roll/FOV; the user taps a known star where it
// really sits in the photo. The solve must recover the true roll+FOV exactly,
// keeping the photo center fixed (so a terrain match at center survives).
{
  const natW = 4032, natH = 3024;
  const pixDir = (px, py, pose) => {
    const { f, r, u } = photoBasis(pose.az, pose.el, pose.roll);
    const fpx = (natW / 2) / Math.tan((pose.fov * D2R) / 2);
    const x = (px - natW / 2) / fpx, y = (natH / 2 - py) / fpx;
    return unit([f[0] + r[0] * x + u[0] * y, f[1] + r[1] * x + u[1] * y, f[2] + r[2] * x + u[2] * y]);
  };
  const dirToPix = (g, pose) => {
    const { f, r, u } = photoBasis(pose.az, pose.el, pose.roll);
    const fpx = (natW / 2) / Math.tan((pose.fov * D2R) / 2);
    const gf = dot(g, f); const X = dot(g, r) / gf, Y = dot(g, u) / gf;
    return { px: natW / 2 + X * fpx, py: natH / 2 - Y * fpx };
  };
  const cases = [
    { truth: { az: 265, el: 20, roll: 1.5, fov: 78 }, corr: { roll: 0, fov: 83 }, star: { az: 258, el: 33 } },
    { truth: { az: 90, el: 40, roll: 3, fov: 100 }, corr: { roll: -1, fov: 92 }, star: { az: 80, el: 55 } },
    { truth: { az: 300, el: 10, roll: 0.5, fov: 50 }, corr: { roll: 0, fov: 50 }, star: { az: 296, el: 18 } },
  ];
  cases.forEach((c, i) => {
    const g = dirFromAzEl(c.star.az, c.star.el);
    const truePose = { ...c.truth };
    const pix = dirToPix(g, truePose);                                  // star's fixed pixel
    const cur = { az: c.truth.az, el: c.truth.el, roll: c.corr.roll, fov: c.corr.fov };
    const vS = pixDir(pix.px, pix.py, cur);                             // world dir it shows now
    const sol = solveRollFov(vS, g, photoBasis(cur.az, cur.el, cur.roll), cur.fov, cur.roll);
    approx(sol.fov, c.truth.fov, 0.05, `star-align case ${i} FOV`);
    approx(sol.roll, c.truth.roll, 0.05, `star-align case ${i} roll`);
    const landed = dirToPix(g, { az: cur.az, el: cur.el, roll: sol.roll, fov: sol.fov });
    approx(Math.hypot(landed.px - pix.px, landed.py - pix.py), 0, 2, `star-align case ${i} lands on pixel`);
  });
}

// --- event correlators: launch (LL2) + fireball (CNEOS) parse/rank ---
{
  const t0 = Date.UTC(2026, 6, 15, 3, 30); // sighting time
  const launchBody = {
    results: [
      { name: "Falcon 9 | Starlink Group 10-1", net: "2026-07-15T03:00:00Z",
        rocket: { configuration: { name: "Falcon 9" } }, mission: { name: "Starlink Group 10-1" },
        pad: { latitude: "28.5618", longitude: "-80.577", location: { name: "Cape Canaveral, FL" } } },
      { name: "Electron | Some Sat", net: "2026-07-10T12:00:00Z",
        rocket: { configuration: { name: "Electron" } }, mission: { name: "Some Sat" },
        pad: { latitude: "-39.26", longitude: "177.86", location: { name: "Mahia, NZ" } } },
      { name: "bad", net: "not-a-date", pad: {} },
    ],
  };
  const L = parseLaunches(launchBody, 34.05, -118.25, t0);   // observer ~ Los Angeles
  approx(L.length, 2, 0, "launch: drops undated rows");
  approx(L[0].dtHours, -0.5, 0.01, "launch: nearest-in-time first (Starlink −0.5 h)");
  approx(L[0].starlink ? 1 : 0, 1, 0, "launch: Starlink mission flagged");
  approx(L[0].distKm, 3540, 120, "launch: pad distance (LA→Cape Canaveral ~3540 km)");
  approx(L[1].starlink ? 1 : 0, 0, 0, "launch: non-Starlink not flagged");

  const fbBody = {
    fields: ["date", "energy", "impact-e", "lat", "lat-dir", "lon", "lon-dir", "alt", "vel"],
    data: [
      ["2026-07-15 03:31:00", "2.5", "0.08", "34.1", "N", "118.0", "W", "42.0", "18.3"],
      ["2026-07-14 22:00:00", "0.4", "0.01", "10.0", "S", "50.0", "E", "35", "20"],
    ],
  };
  const F = parseFireballs(fbBody, 34.05, -118.25, t0);
  approx(F.length, 2, 0, "fireball: parsed rows");
  approx(F[0].dtHours, 60 / 3600 * 1, 0.02, "fireball: nearest first (+1 min)");
  approx(F[0].lon, -118.0, 0.001, "fireball: W hemisphere → negative lon");
  approx(F[1].lat, -10.0, 0.001, "fireball: S hemisphere → negative lat");
  approx(F[0].distKm, 24, 3, "fireball: distance to observer (~24 km)");
  approx(haversineKm(0, 0, 0, 1), 111.2, 0.5, "haversine: 1° lon at equator ≈ 111 km");
}

// --- named peaks (OSM Overpass): bearing / distance / elevation ---
{
  // due-north and due-east reference points (~11.1 km) from an observer
  const oLat = 42.0, oLon = -123.0;
  approx(bearingDeg(oLat, oLon, oLat + 0.1, oLon), 0, 0.01, "peaks: bearing due north = 0");
  approx(bearingDeg(oLat, oLon, oLat, oLon + 0.1), 90, 0.2, "peaks: bearing due east ≈ 90");
  approx(distM(oLat, oLon, oLat + 0.1, oLon) / 1000, 11.12, 0.05, "peaks: 0.1° lat ≈ 11.1 km");

  // a 2000 m summit ~10 km due east of a 400 m observer: elevation angle with
  // curvature drop (d²(1−k)/2R ≈ 6.8 m over 10 km) → atan2(2000−401.6−6.8, 10000)
  const body = {
    elements: [
      { lat: oLat, lon: oLon + 0.1207, tags: { name: "Test Peak", ele: "2000" } }, // ~10 km east
      { lat: oLat + 0.0005, lon: oLon, tags: { name: "Too Close" } },              // < 200 m: dropped
      { lat: oLat + 1.0, lon: oLon, tags: { name: "Too Far", ele: "3000" } },      // > 40 km: dropped
      { lat: oLat, lon: oLon - 0.05, tags: { ele: "1500" } },                       // no name: dropped
    ],
  };
  const P = parsePeaks(body, oLat, oLon, 400, 40);
  approx(P.length, 1, 0, "peaks: filters unnamed / too-close / too-far");
  approx(P[0].az, 90, 0.3, "peaks: summit due east ≈ 90° az");
  approx(P[0].distKm, 10.0, 0.2, "peaks: summit distance ≈ 10 km");
  const eye = 400 + 1.6, d = P[0].distKm * 1000;
  const wantEl = Math.atan2(2000 - eye - (d * d * (1 - 0.13)) / (2 * RE), d) * R2D;
  approx(P[0].el, wantEl, 0.02, "peaks: curvature-corrected elevation");
  approx(P[0].el > 8 && P[0].el < 10 ? 1 : 0, 1, 0, "peaks: elevation in plausible band (~9°)");
}

// --- multi-star pose fit with radial distortion (solvePoseAnchors) ---
// A photo has a true pose (roll,fov) AND lens distortion k. Two stars are
// captured (their fixed pixels via the TRUE model). A distortion-free fit
// (roll+fov only) can't land both; fitting k too must recover the truth and
// drive the residual to ~0.
{
  const natW = 4032, natH = 3024, az = 250, el = 12;
  const truth = { roll: 1.2, fov: 62, k: 0.08 };
  // two stars at different radii from center
  const stars = [dirFromAzEl(az - 9, el + 14), dirFromAzEl(az + 11, el + 3)];
  const anchors = stars.map((g) => {
    const px = dirToPixK(g, natW, natH, az, el, truth.roll, truth.fov, truth.k); // fixed pixel under TRUTH
    return { px: px.px, py: px.py, g };
  });
  // distortion-free fit (k forced 0 by using 1 anchor's closed form twice) leaves residual;
  // the joint fit with k must nail both:
  const seed = { roll: 0, fov: 68, k: 0 };
  const sol = solvePoseAnchors(anchors, natW, natH, az, el, seed);
  approx(sol.fov, truth.fov, 1.5, "distortion fit: recovered FOV");
  approx(sol.roll, truth.roll, 0.5, "distortion fit: recovered roll");
  approx(sol.k, truth.k, 0.02, "distortion fit: recovered k");
  approx(sol.rms, 0, 0.05, "distortion fit: both stars land (rms→0°)");
  // and a single anchor still solves roll+fov (k stays 0), rms→0
  const one = solvePoseAnchors([anchors[0]], natW, natH, az, el, seed);
  approx(one.rms, 0, 0.05, "single-anchor fit: lands (rms→0°)");
  approx(one.k, 0, 1e-9, "single-anchor fit: k stays 0");

  // 3+ anchors → FULL plate solve: recover a WRONG-CENTER pose too. Build 3
  // stars' fixed pixels under the TRUE pose (incl. a shifted az/el), seed the
  // solve from an OFF center, and confirm it recovers az/el/roll/fov/k.
  const T = { az: 252, el: 15, roll: 1.2, fov: 62, k: 0.08 };
  const gs = [dirFromAzEl(T.az - 10, T.el + 13), dirFromAzEl(T.az + 12, T.el + 2), dirFromAzEl(T.az + 4, T.el + 20)];
  const anc3 = gs.map((g) => { const p = dirToPixK(g, natW, natH, T.az, T.el, T.roll, T.fov, T.k); return { px: p.px, py: p.py, g }; });
  const sol3 = solvePoseAnchors(anc3, natW, natH, T.az - 3, T.el + 2, { roll: 0, fov: 68, k: 0 }); // seeded 3° off in az
  approx(sol3.az, T.az, 0.3, "3-anchor plate solve: recovered az (center freed)");
  approx(sol3.el, T.el, 0.3, "3-anchor plate solve: recovered el");
  approx(sol3.fov, T.fov, 1.0, "3-anchor plate solve: recovered FOV");
  approx(sol3.k, T.k, 0.03, "3-anchor plate solve: recovered k");
  approx(sol3.rms, 0, 0.1, "3-anchor plate solve: all stars land (rms→0°)");
}

// --- video pose tracking (src/video/postrack.js) ---
// Per-frame camera pose from tracked background features. Synthesize frames of a
// bright-blob "sky" rotating by a known per-frame pose, plus a moving object, and
// assert the tracker recovers the rotation and rejects the object.
{
  const natW = 4032, natH = 3024, TW = 400, TH = 300, sc = natW / TW;
  const P0 = { az: 250, el: 12, roll: 0, fov: 60, k: 0 };
  const bg = []; // 35 background dirs spread across the frame under P0
  for (let u = -3; u <= 3; u++) for (let v = -2; v <= 2; v++) bg.push(dirFromAzEl(P0.az + u * 7, P0.el + v * 7));
  const drawBlob = (data, bx, by, val) => {
    for (let dy = -3; dy <= 3; dy++) for (let dx = -3; dx <= 3; dx++) {
      const x = Math.round(bx) + dx, y = Math.round(by) + dy;
      if (x < 0 || y < 0 || x >= TW || y >= TH) continue;
      const g = val * Math.exp(-(dx * dx + dy * dy) / 4), i = (y * TW + x) * 4;
      data[i] = Math.min(255, data[i] + g); data[i + 1] = Math.min(255, data[i + 1] + g); data[i + 2] = Math.min(255, data[i + 2] + g);
    }
  };
  const renderFrame = (pose, objDir, objPose) => {
    const data = new Uint8ClampedArray(TW * TH * 4);
    for (let i = 0; i < TW * TH; i++) data[i * 4 + 3] = 255;
    for (const g of bg) { const p = dirToPixK(g, natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k); if (p) drawBlob(data, p.px / sc, p.py / sc, 235); }
    if (objDir) { const p = dirToPixK(objDir, natW, natH, objPose.az, objPose.el, objPose.roll, objPose.fov, objPose.k); if (p) drawBlob(data, p.px / sc, p.py / sc, 235); }
    return data;
  };

  // 1. poseFromTracks recovers a known rotation and trims a moving-object outlier
  for (const Pn of [{ az: 251, el: 12, roll: 0 }, { az: 253, el: 13, roll: 1.5 }, { az: 248, el: 11.4, roll: -1 }]) {
    const anchors = bg.map((g) => { const p = dirToPixK(g, natW, natH, Pn.az, Pn.el, Pn.roll, P0.fov, P0.k); return p ? { px: p.px, py: p.py, g } : null; }).filter(Boolean);
    anchors.push({ px: natW * 0.5, py: natH * 0.5, g: dirFromAzEl(P0.az + 3, P0.el + 25) }); // object: pixel↔g mismatch
    const sol = poseFromTracks(anchors, natW, natH, P0, { lockFov: true, lockK: true });
    approx(sol.az, Pn.az, 0.15, "postrack: recovered az");
    approx(sol.el, Pn.el, 0.15, "postrack: recovered el");
    approx(sol.roll, Pn.roll, 0.25, "postrack: recovered roll");
    approx(sol.n, bg.length, 0, "postrack: object outlier trimmed");
    approx(sol.rms, 0, 0.03, "postrack: bg features land (rms→0°)");
  }

  // 2. trackFeatures matches the projected prediction; a flat patch is rejected
  {
    const P1 = { az: 251, el: 12.4, roll: 0.6, fov: 60, k: 0 };
    const f0 = renderFrame(P0), f1 = renderFrame(P1);
    const feats = bg.slice(0, 10).map((g) => { const p = dirToPixK(g, natW, natH, P0.az, P0.el, P0.roll, P0.fov, P0.k); return { g, tx: p.px / sc, ty: p.py / sc, px: p.px / sc, py: p.py / sc }; });
    const tr = trackFeatures(f0, f1, TW, TH, feats, { patch: 11, search: 14 });
    let okCount = 0, maxErr = 0;
    for (let i = 0; i < feats.length; i++) { const q = dirToPixK(feats[i].g, natW, natH, P1.az, P1.el, P1.roll, P1.fov, P1.k); if (tr[i].ok) { okCount++; maxErr = Math.max(maxErr, Math.hypot(tr[i].px - q.px / sc, tr[i].py - q.py / sc)); } }
    approx(okCount >= 8 ? 1 : 0, 1, 0, "trackFeatures: most features tracked");
    approx(maxErr < 1.2 ? 1 : 0, 1, 0, "trackFeatures: within ~1px of prediction");
    const flatBuf = new Uint8ClampedArray(TW * TH * 4); for (let i = 0; i < TW * TH; i++) flatBuf[i * 4 + 3] = 255;
    const flat = trackFeatures(flatBuf, flatBuf, TW, TH, [{ tx: 100, ty: 100, px: 100, py: 100 }], {})[0];
    approx(flat.ok ? 0 : 1, 1, 0, "trackFeatures: flat/low-texture patch rejected");
  }

  // 2b. stepObject: the OBJECT tracker follows a mover through camera motion —
  // the camera pans while the object flies its own path; the recovered angular
  // track must match the OBJECT's truth, not freeze at the marked spot.
  {
    const poses = [P0,
      { az: 250.6, el: 12.2, roll: 0.3, fov: 60, k: 0 },
      { az: 251.2, el: 12.4, roll: 0.5, fov: 60, k: 0 },
      { az: 251.8, el: 12.5, roll: 0.4, fov: 60, k: 0 }];
    const objAzEl = (i) => ({ az: 249 + i * 0.9, el: 15.5 + i * 0.35 });
    const frames = poses.map((p, i) => { const o = objAzEl(i); return renderFrame(p, dirFromAzEl(o.az, o.el), p); });
    const o0 = objAzEl(0);
    const p0px = dirToPixK(dirFromAzEl(o0.az, o0.el), natW, natH, P0.az, P0.el, P0.roll, P0.fov, P0.k);
    let st = { tx: p0px.px / sc, ty: p0px.py / sc, g: dirFromAzEl(o0.az, o0.el) };
    let maxErr = 0, okAll = 1;
    for (let i = 1; i < poses.length; i++) {
      const o = stepObject(frames[i - 1], frames[i], TW, TH, st, poses[i], { natW, natH, patch: 11, search: 18 });
      if (!o.ok) okAll = 0;
      st = { tx: o.tx, ty: o.ty, g: o.g, gPrev: o.gPrev };
      const ae = dirToAzEl(o.g), tru = objAzEl(i);
      maxErr = Math.max(maxErr, Math.abs(ae.az - tru.az), Math.abs(ae.el - tru.el));
    }
    approx(okAll, 1, 0, "stepObject: the mover is matched on every frame");
    approx(maxErr < 0.25 ? 1 : 0, 1, 0, "stepObject: recovered angular path ≈ truth (<0.25°)");

    // FAST mover: its per-frame displacement escapes the near search ring
    // entirely (ground-truth failure on the real-texture synthetic: the near
    // ring latched onto leftover background and the template poisoned) — the
    // wide search must still catch it. Sparse field: the identical-twin
    // ambiguity of the dense star grid is untestable for ANY appearance
    // tracker, so the twins stay out of the search window here; the
    // real-texture regression lives in the offline clip harness.
    {
      const fPoses = [P0, { az: 250.4, el: 12.1, roll: 0.2, fov: 60, k: 0 }, { az: 250.8, el: 12.2, roll: 0.3, fov: 60, k: 0 }];
      const fObj = (i) => ({ az: 248.5 + i * 2.1, el: 15.6 + i * 0.8 });
      const mkSparse = (pose, o) => {
        const d = new Uint8ClampedArray(TW * TH * 4);
        for (let i = 0; i < TW * TH; i++) d[i * 4 + 3] = 255;
        for (const g of [dirFromAzEl(238, 4), dirFromAzEl(262, 4), dirFromAzEl(238, 24)]) {  // far stars only
          const p = dirToPixK(g, natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k);
          if (p) drawBlob(d, p.px / sc, p.py / sc, 235);
        }
        const p = dirToPixK(dirFromAzEl(o.az, o.el), natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k);
        if (p) drawBlob(d, p.px / sc, p.py / sc, 235);
        return d;
      };
      const fFrames = fPoses.map((p, i) => mkSparse(p, fObj(i)));
      const fo0 = fObj(0);
      const fp = dirToPixK(dirFromAzEl(fo0.az, fo0.el), natW, natH, P0.az, P0.el, P0.roll, P0.fov, P0.k);
      let fst = { tx: fp.px / sc, ty: fp.py / sc, g: dirFromAzEl(fo0.az, fo0.el) };
      let fErr = 0, fOk = 1;
      for (let i = 1; i < fPoses.length; i++) {
        const o = stepObject(fFrames[i - 1], fFrames[i], TW, TH, fst, fPoses[i], { natW, natH, patch: 11, search: 26 });
        if (!o.ok) fOk = 0;
        fst = { tx: o.tx, ty: o.ty, g: o.g, gPrev: o.gPrev };
        const ae = dirToAzEl(o.g), tru = fObj(i);
        fErr = Math.max(fErr, Math.abs(ae.az - tru.az), Math.abs(ae.el - tru.el));
      }
      approx(fOk, 1, 0, "stepObject: a FAST mover (outside the near ring) is still caught");
      approx(fErr < 0.3 ? 1 : 0, 1, 0, "stepObject: fast mover recovered (<0.3°)");
    }

    // DRIFT ANCHOR (opts.seed): the frame-to-frame template accumulates drift
    // over a long run; the pristine seed template can't. Over 14 frames the
    // object (a DISTINCT bright blob) sweeps past a dimmer background grid — a
    // patch that slowly absorbs a passing background blob biases the centroid
    // and walks off. The seed anchor (clean object appearance) stays locked.
    {
      const N = 14;
      const dPoses = Array.from({ length: N }, (_, i) => ({ az: 250 + i * 0.16, el: 12 + i * 0.04, roll: 0, fov: 60, k: 0 }));
      const dObj = (i) => ({ az: 248.9 + i * 0.5, el: 15.0 + i * 0.16 }); // grazes the bg grid over the run
      // custom render: dim background grid (val 150) + a BRIGHT, distinct object
      // (val 255) so the object template is unique (no identical-twin latch).
      const mkFrame = (pose, o) => {
        const d = new Uint8ClampedArray(TW * TH * 4);
        for (let i = 0; i < TW * TH; i++) d[i * 4 + 3] = 255;
        for (const g of bg) { const p = dirToPixK(g, natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k); if (p) drawBlob(d, p.px / sc, p.py / sc, 150); }
        const p = dirToPixK(dirFromAzEl(o.az, o.el), natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k);
        if (p) drawBlob(d, p.px / sc, p.py / sc, 255);
        return d;
      };
      const dFrames = dPoses.map((p, i) => mkFrame(p, dObj(i)));
      const d0 = dObj(0);
      const dp0 = dirToPixK(dirFromAzEl(d0.az, d0.el), natW, natH, dPoses[0].az, dPoses[0].el, dPoses[0].roll, dPoses[0].fov, dPoses[0].k);
      const seed = { data: dFrames[0], tx: dp0.px / sc, ty: dp0.py / sc };
      const runTrack = (useSeed) => {
        let st = { tx: dp0.px / sc, ty: dp0.py / sc, g: dirFromAzEl(d0.az, d0.el) };
        let mErr = 0;
        for (let i = 1; i < N; i++) {
          const o = stepObject(dFrames[i - 1], dFrames[i], TW, TH, st, dPoses[i], { natW, natH, patch: 11, search: 18, ...(useSeed ? { seed } : {}) });
          st = { tx: o.tx, ty: o.ty, g: o.g, gPrev: o.gPrev };
          const ae = dirToAzEl(o.g), tru = dObj(i);
          mErr = Math.max(mErr, Math.abs(ae.az - tru.az), Math.abs(ae.el - tru.el));
        }
        return mErr;
      };
      const errAnchored = runTrack(true);
      approx(errAnchored < 0.35 ? 1 : 0, 1, 0, "stepObject+seed: a 14-frame track stays locked to the object (<0.35°)");
      // the anchor's win over frame-to-frame shows on NOISY field clips (JPEG/
      // scale wobble seeds cumulative drift the clean synthetic can't reproduce);
      // here we assert the anchor path itself produces an accurate long track.
    }

    // HYBRID GUIDE: with a manual trajectory dir supplied, the prediction is
    // the guide, and a confident lookalike match far off the guide is
    // REJECTED — the human's path outranks the pixels; the hold rides the
    // guide (q reflects it in the app layer).
    {
      const gPoses = [P0, { az: 250.4, el: 12.1, roll: 0.2, fov: 60, k: 0 }];
      const gObj0 = { az: 248.5, el: 15.6 }, gObjT = { az: 250.6, el: 16.4 };
      // frame 1 contains ONLY a decoy blob 3° from where the guide says the object is
      const mkG = (pose, o, decoy) => {
        const d = new Uint8ClampedArray(TW * TH * 4);
        for (let i = 0; i < TW * TH; i++) d[i * 4 + 3] = 255;
        for (const gg of [o, decoy].filter(Boolean)) {
          const p = dirToPixK(dirFromAzEl(gg.az, gg.el), natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k);
          if (p) drawBlob(d, p.px / sc, p.py / sc, 235);
        }
        return d;
      };
      const fr0 = mkG(gPoses[0], gObj0, null);
      const fr1 = mkG(gPoses[1], { az: gObjT.az + 3, el: gObjT.el }, null);   // only the decoy exists
      const gp0 = dirToPixK(dirFromAzEl(gObj0.az, gObj0.el), natW, natH, P0.az, P0.el, P0.roll, P0.fov, P0.k);
      const gSt = { tx: gp0.px / sc, ty: gp0.py / sc, g: dirFromAzEl(gObj0.az, gObj0.el) };
      const og = stepObject(fr0, fr1, TW, TH, gSt, gPoses[1], { natW, natH, patch: 11, search: 26, guide: dirFromAzEl(gObjT.az, gObjT.el) });
      approx(og.ok ? 0 : 1, 1, 0, "guide: a lookalike 3° off the manual path is rejected");
      const aeG = dirToAzEl(og.g);
      approx(aeG.az, gObjT.az, 0.05, "guide: the hold rides the manual trajectory");
    }

    // snapToObject: an off-centre seed (thumb precision / generous shape)
    // must snap onto the object — a template cut a few px off a small object
    // is half background and gets lost immediately (e2e ground truth).
    {
      const fr = renderFrame(P0, dirFromAzEl(P0.az + 2, P0.el + 4), P0);
      const tp = dirToPixK(dirFromAzEl(P0.az + 2, P0.el + 4), natW, natH, P0.az, P0.el, P0.roll, P0.fov, P0.k);
      const sn = snapToObject(fr, TW, TH, tp.px / sc + 5, tp.py / sc - 4, 10);
      approx(Math.hypot(sn.x - tp.px / sc, sn.y - tp.py / sc) < 2 ? 1 : 0, 1, 0, "snapToObject: off-centre seed lands on the blob (<2px)");
    }
  }

  // 3. end-to-end initTracker/stepTracker recovers a 4-frame rotation path
  {
    const poses = [P0, { az: 250.8, el: 12.3, roll: 0.5, fov: 60, k: 0 }, { az: 251.7, el: 12.7, roll: 1.1, fov: 60, k: 0 }, { az: 252.4, el: 13.2, roll: 1.6, fov: 60, k: 0 }];
    const frames = poses.map((p) => renderFrame(p));
    const tracker = initTracker(frames[0], TW, TH, natW, natH, P0, { mode: "night", minMatch: 8, maxN: 40, patch: 11, search: 14 });
    approx(tracker.features.length >= 20 ? 1 : 0, 1, 0, "initTracker: seeded background features");
    let maxAzErr = 0, maxRollErr = 0, minInliers = 999;
    for (let i = 1; i < frames.length; i++) {
      const r = stepTracker(tracker, frames[i]);
      maxAzErr = Math.max(maxAzErr, Math.abs(r.pose.az - poses[i].az));
      maxRollErr = Math.max(maxRollErr, Math.abs(r.pose.roll - poses[i].roll));
      minInliers = Math.min(minInliers, r.nInliers);
    }
    approx(maxAzErr < 0.4 ? 1 : 0, 1, 0, "stepTracker: az path recovered (<0.4°)");
    approx(maxRollErr < 0.5 ? 1 : 0, 1, 0, "stepTracker: roll path recovered (<0.5°)");
    approx(minInliers >= 12 ? 1 : 0, 1, 0, "stepTracker: held enough inliers each frame");
  }

  // 4. ZOOM: the FOV narrows 60→46° while panning — the pairwise-distance scale
  // estimate must catch it (predictions re-tried under the corrected FOV) and
  // the solve must track both the zoom and the rotation
  {
    const zposes = [P0,
      { az: 250.3, el: 12.1, roll: 0.2, fov: 56, k: 0 },
      { az: 250.6, el: 12.2, roll: 0.4, fov: 52.5, k: 0 },
      { az: 250.9, el: 12.3, roll: 0.5, fov: 49, k: 0 },
      { az: 251.1, el: 12.4, roll: 0.6, fov: 46, k: 0 }];
    const zframes = zposes.map((p) => renderFrame(p));
    const tk = initTracker(zframes[0], TW, TH, natW, natH, P0, { mode: "night", minMatch: 6, maxN: 40, patch: 11, search: 16 });
    let maxFovErr = 0, maxAzErr = 0;
    for (let i = 1; i < zframes.length; i++) {
      const r = stepTracker(tk, zframes[i]);
      maxFovErr = Math.max(maxFovErr, Math.abs(r.pose.fov - zposes[i].fov));
      maxAzErr = Math.max(maxAzErr, Math.abs(r.pose.az - zposes[i].az));
    }
    approx(maxFovErr < 1.2 ? 1 : 0, 1, 0, "stepTracker: ZOOM tracked (fov within 1.2°)");
    approx(maxAzErr < 0.4 ? 1 : 0, 1, 0, "stepTracker: az stays locked through the zoom");
  }

  // 4b. FAST zoom: a single step jumps 60→40° (scale ≈1.6× — predictions
  // overshoot the search window AND templates rescale). The multi-scale rescue
  // sweep must recover it in one step.
  {
    const Pz = { az: 250.4, el: 12.2, roll: 0.3, fov: 40, k: 0 };
    const f0 = renderFrame(P0), f1 = renderFrame(Pz);
    const tk = initTracker(f0, TW, TH, natW, natH, P0, { mode: "night", minMatch: 6, maxN: 40, patch: 11, search: 14 });
    const r = stepTracker(tk, f1);
    approx(Math.abs(r.pose.fov - Pz.fov) < 1.5 ? 1 : 0, 1, 0, "stepTracker: FAST zoom rescued (fov 60→40 in one step)");
    approx(Math.abs(r.pose.az - Pz.az) < 0.4 ? 1 : 0, 1, 0, "stepTracker: az recovered through the fast zoom");
    approx(r.nInliers >= 8 ? 1 : 0, 1, 0, "stepTracker: fast zoom re-locked enough references");
  }

  // 4e. FULL ZOOM CYCLE with re-anchoring live (the field regression: anchors
  // must fix drift WITHOUT flattening the tracked zoom — FOV is only freed for
  // an anchor solve when the matches have real radial leverage)
  {
    const cf = [60, 56, 52, 48, 45, 42, 42, 45, 48, 52, 56, 60];
    const cposes = cf.map((f, i) => ({ az: 250 + i * 0.15, el: 12 + i * 0.05, roll: 0, fov: f, k: 0 }));
    const cframes = cposes.map((p) => renderFrame(p));
    const tk = initTracker(cframes[0], TW, TH, natW, natH, P0, { mode: "night", minMatch: 6, maxN: 40, patch: 11, search: 14 });
    let maxFovErr = 0, maxAzErr = 0, ancs = 0;
    for (let i = 1; i < cframes.length; i++) {
      const r = stepTracker(tk, cframes[i]);
      maxFovErr = Math.max(maxFovErr, Math.abs(r.pose.fov - cposes[i].fov));
      maxAzErr = Math.max(maxAzErr, Math.abs(r.pose.az - cposes[i].az));
      if (r.anchored) ancs++;
    }
    approx(maxFovErr < 0.8 ? 1 : 0, 1, 0, "zoom cycle + anchors: FOV tracks the whole 60→42→60 sweep");
    approx(maxAzErr < 0.3 ? 1 : 0, 1, 0, "zoom cycle + anchors: az stays locked throughout");
    approx(ancs >= 4 ? 1 : 0, 1, 0, "zoom cycle + anchors: re-anchoring stayed active");
  }

  // 4f. SELF-SIMILAR scene (foliage): the field regression. A jittered field of
  // similar-but-distinct blobs offers a lookalike near every prediction, so a
  // zoom can be MASKED by false in-place matches (s reads ≈1, pose goes
  // self-consistently wrong at the old FOV). The per-step scale probe's
  // radial-fit coherence must see through it across a full zoom cycle.
  {
    const bg2 = [];
    let q = 0;
    for (let u = -9; u <= 9; u++) for (let v = -6; v <= 6; v++) {
      q++;
      const ju = (((q * 37) % 13) / 13 - 0.5) * 1.4, jv = (((q * 53) % 11) / 11 - 0.5) * 1.4;
      bg2.push({ g: dirFromAzEl(P0.az + u * 2.2 + ju, P0.el + v * 2.2 + jv), sig: 1.4 + ((q * 7) % 5) * 0.55, amp: 150 + ((q * 13) % 7) * 14 });
    }
    const drawSoft = (data, bx, by, sig, amp) => {
      const R = Math.ceil(sig * 2.6);
      for (let dy = -R; dy <= R; dy++) for (let dx = -R; dx <= R; dx++) {
        const x = Math.round(bx) + dx, y = Math.round(by) + dy;
        if (x < 0 || y < 0 || x >= TW || y >= TH) continue;
        const g = amp * Math.exp(-(dx * dx + dy * dy) / (2 * sig * sig)), i = (y * TW + x) * 4;
        data[i] = Math.min(255, data[i] + g); data[i + 1] = Math.min(255, data[i + 1] + g); data[i + 2] = Math.min(255, data[i + 2] + g);
      }
    };
    const renderFoliage = (pose) => {
      const data = new Uint8ClampedArray(TW * TH * 4);
      for (let i = 0; i < TW * TH; i++) data[i * 4 + 3] = 255;
      for (const b of bg2) { const p = dirToPixK(b.g, natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k); if (p && p.px / sc > -6 && p.px / sc < TW + 6 && p.py / sc > -6 && p.py / sc < TH + 6) drawSoft(data, p.px / sc, p.py / sc, b.sig, b.amp); }
      return data;
    };
    const ff = [60, 55, 50, 45, 41, 41, 45, 50, 55, 60];
    const fposes = ff.map((f, i) => ({ az: 250 + i * 0.1, el: 12 + i * 0.04, roll: 0, fov: f, k: 0 }));
    const fframes = fposes.map(renderFoliage);
    const tk = initTracker(fframes[0], TW, TH, natW, natH, P0, { mode: "auto", minMatch: 6, maxN: 40, patch: 11, search: 14 });
    let maxFovErr = 0, maxAzErr = 0;
    for (let i = 1; i < fframes.length; i++) {
      const r = stepTracker(tk, fframes[i]);
      maxFovErr = Math.max(maxFovErr, Math.abs(r.pose.fov - fposes[i].fov));
      maxAzErr = Math.max(maxAzErr, Math.abs(r.pose.az - fposes[i].az));
    }
    approx(maxFovErr < 1.0 ? 1 : 0, 1, 0, "foliage zoom cycle: masked-zoom seen through (fov tracks <1°)");
    approx(maxAzErr < 0.3 ? 1 : 0, 1, 0, "foliage zoom cycle: az stays locked");
  }

  // 4g. AREA-TEXTURED world (the real-clip case, derived from a field video
  // that beat every feature-only approach): GLOBAL registration must engage
  // and carry a deep zoom cycle end-to-end. The texture mixes incommensurate
  // scales incl. a long-wavelength term — real scenes are not periodic, and a
  // purely periodic synthetic aliases sideways by one period.
  {
    const texAt = (az, el) => 120 + 48 * Math.sin(az * 0.23 + 1.1) + 40 * Math.sin(az * 0.9) * Math.cos(el * 1.3) + 30 * Math.sin(az * 2.7 + el * 1.9) + 22 * Math.cos(az * 5.1 - el * 3.7) + 14 * Math.sin(az * 11 + el * 7);
    const renderTex = (pose) => {
      const data = new Uint8ClampedArray(TW * TH * 4);
      const b = photoBasis(pose.az, pose.el, pose.roll);
      const fpx = (TW / 2) / Math.tan((pose.fov * D2R) / 2);
      for (let y = 0; y < TH; y++) for (let x = 0; x < TW; x++) {
        const xx = (x - TW / 2) / fpx, yy = (TH / 2 - y) / fpx;
        const d = unit([b.f[0] + b.r[0] * xx + b.u[0] * yy, b.f[1] + b.r[1] * xx + b.u[1] * yy, b.f[2] + b.r[2] * xx + b.u[2] * yy]);
        const ae = dirToAzEl(d);
        const v = Math.max(0, Math.min(255, texAt(ae.az, ae.el)));
        const i = (y * TW + x) * 4;
        data[i] = data[i + 1] = data[i + 2] = v; data[i + 3] = 255;
      }
      return data;
    };
    const tf = [60, 52, 44, 36, 30, 30, 36, 44, 52, 60];
    const tposes = tf.map((f, i) => ({ az: 250 + i * 0.15, el: 12 + i * 0.05, roll: 0, fov: f, k: 0 }));
    const tframes = tposes.map(renderTex);
    const tk = initTracker(tframes[0], TW, TH, natW, natH, P0, { mode: "day", minMatch: 6, maxN: 40, patch: 11, search: 14 });
    let maxF = 0, maxA = 0, globN = 0;
    for (let i = 1; i < tframes.length; i++) {
      const r = stepTracker(tk, tframes[i]);
      maxF = Math.max(maxF, Math.abs(r.pose.fov - tposes[i].fov));
      maxA = Math.max(maxA, Math.abs(r.pose.az - tposes[i].az));
      if (r.global != null) globN++;
    }
    approx(maxF < 1.5 ? 1 : 0, 1, 0, "textured world: 2x zoom cycle tracked via global registration (<1.5°)");
    approx(maxA < 0.3 ? 1 : 0, 1, 0, "textured world: az locked through the zoom");
    approx(globN >= 7 ? 1 : 0, 1, 0, "textured world: global registration engaged");

    // PHYSICAL FOV CAP (field bug: at a zoom-out landing the smallest-template
    // ladder rungs — which decorrelate least under handheld mismatch — won with
    // an impossible 110–127° FOV). With fovMax set, no path may ever report a
    // frame WIDER than the lens's widest; without it, a genuinely wider frame
    // is still found honestly.
    {
      const wide = renderTex({ az: 250, el: 12, roll: 0, fov: 70, k: 0 });
      const tk2 = initTracker(tframes[0], TW, TH, natW, natH, P0, { mode: "day", minMatch: 6, maxN: 40, patch: 11, search: 14 });
      const gFree = registerToRef(tk2, grayDown(wide, TW, TH, 96));
      approx(gFree && Math.abs(gFree.fov - 70) < 4 ? 1 : 0, 1, 0, "fov cap: uncapped register finds the wider frame (~70°)");
      const gCap = registerToRef(tk2, grayDown(wide, TW, TH, 96), { fovMax: 63 });
      approx(gCap == null || gCap.fov <= 63.01 ? 1 : 0, 1, 0, "fov cap: capped register never exceeds the lens's widest");
      const r2 = stepTracker(tk2, wide, { fovMax: 63 });
      approx(r2.pose.fov <= 63.01 ? 1 : 0, 1, 0, "fov cap: stepTracker pose respects the cap");
    }

    // ROLL-HINTED registration (field bug: a clip's rolled tail — horizon
    // tilted ~12° — decorrelated the whole-frame NCC, so every global lock
    // failed and the pose froze while the camera kept moving). De-rotating
    // the frame by the chain's roll estimate must restore a strong lock.
    {
      const rolled = renderTex({ az: 250.3, el: 12.1, roll: 9, fov: 60, k: 0 });
      const tk3 = initTracker(tframes[0], TW, TH, natW, natH, P0, { mode: "day", minMatch: 6, maxN: 40, patch: 11, search: 14 });
      const gRaw = registerToRef(tk3, grayDown(rolled, TW, TH, 96));
      const gHint = registerToRef(tk3, grayDown(rolled, TW, TH, 96), { rollHint: 9 });
      approx(gHint ? 1 : 0, 1, 0, "roll hint: rolled frame still locks globally");
      approx(gHint && (!gRaw || gHint.score >= gRaw.score) ? 1 : 0, 1, 0, "roll hint: de-rotated match scores at least the raw one");
      approx(gHint && Math.abs(gHint.az - 250.3) < 1.2 ? 1 : 0, 1, 0, "roll hint: center az recovered through the roll");
    }
  }

  // 4c. ABSOLUTE RE-ANCHOR: simulate accumulated drift (feature turnover baked
  // a +0.4° az error into every g and the pose) — the incremental solve alone
  // would confirm the drift, but matching the pristine REFERENCE features must
  // recover the truth and report the correction.
  {
    const Pt = { az: 250.5, el: 12.2, roll: 0.3, fov: 60, k: 0 }; // truth at the next frame
    const f0 = renderFrame(P0), f1 = renderFrame(Pt);
    const tk = initTracker(f0, TW, TH, natW, natH, P0, { mode: "night", minMatch: 6, maxN: 40, patch: 11, search: 14 });
    // contaminate: shift every WORKING feature's g and the pose by +0.4° az (turnover drift)
    tk.features = tk.features.map((f) => { const ae = dirToAzEl(f.g); return { ...f, prime: false, g: dirFromAzEl(ae.az + 0.4, ae.el) }; });
    tk.lastPose = { ...P0, az: P0.az + 0.4 };
    const r = stepTracker(tk, f1);
    approx(r.anchored ? 1 : 0, 1, 0, "re-anchor: locked directly to the reference frame");
    approx(r.pose.az, Pt.az, 0.1, "re-anchor: drift zeroed (az back on truth)");
    approx(r.drift ? Math.abs(r.drift.dAz + 0.4) < 0.15 ? 1 : 0 : 0, 1, 0, "re-anchor: measured the ~0.4° drift it removed");
  }

  // 4h. despikePath: a garbage single-frame solve (blurred frame mid-zoom)
  // must be pulled back to its neighbours' interpolation, while a REAL zoom
  // ramp and a real fast pan are left untouched
  {
    const mk = (t, az, el, roll, fov, n) => ({ t, az, el, roll, fov, n });
    const pth = [
      mk(0.00, 280, 13, 0, 90, 20),
      mk(0.25, 280.2, 13, 0, 88, 20),
      mk(0.50, 280.4, 13, -25, 60, 0),   // SPIKE: wild roll+fov on an n=0 frame
      mk(0.75, 280.6, 13, 0, 40, 18),    // real zoom ramp continues
      mk(1.00, 280.8, 13, 0, 30, 18),
      mk(1.25, 281.0, 13, 0, 24, 16),
      mk(1.50, 285.0, 13, 0, 24, 16),    // real fast pan (4° step, neighbours disagree)
      mk(1.75, 289.0, 13, 0, 24, 16),
    ];
    const fixedN = despikePath(pth);
    approx(fixedN >= 1 ? 1 : 0, 1, 0, "despike: the garbage frame was caught");
    approx(pth[2].roll, 0, 0.5, "despike: wild roll pulled to neighbours");
    approx(Math.abs(pth[2].fov - 64) < 8 ? 1 : 0, 1, 0, "despike: fov spike pulled onto the zoom ramp");
    approx(pth[6].az, 285.0, 0.01, "despike: a real fast pan is untouched");
    approx(pth[4].fov, 30, 0.01, "despike: the real zoom ramp is untouched");
    approx(pth[0].az, 280, 0, "despike: endpoints untouched");
  }

  // 4h-b. smoothPath: sub-degree solve noise (alternating wiggle) is damped —
  // hard on weak frames, gently on strong ones — while a real linear pan
  // (each sample ON its neighbours' interpolation) passes through unchanged.
  {
    const mk = (t, az, n) => ({ t, az, el: 10, roll: 0, fov: 60, n });
    const wig = (n) => Array.from({ length: 9 }, (_, i) => mk(i * 0.25, 250 + (i % 2 ? 0.4 : -0.4), n));
    const strong = wig(20); smoothPath(strong);
    const weak = wig(8); smoothPath(weak);
    const maxDev = (p) => Math.max(...p.slice(1, -1).map((q) => Math.abs(q.az - 250)));
    approx(maxDev(strong) < 0.25 ? 1 : 0, 1, 0, "smooth: strong-frame wiggle damped (0.4° → <0.25°)");
    approx(maxDev(weak) < 0.1 ? 1 : 0, 1, 0, "smooth: weak-frame wiggle nearly killed (<0.1°)");
    const ramp = Array.from({ length: 9 }, (_, i) => mk(i * 0.25, 250 + i * 0.5, 20)); smoothPath(ramp);
    approx(Math.max(...ramp.map((q, i) => Math.abs(q.az - (250 + i * 0.5)))) < 1e-6 ? 1 : 0, 1, 0, "smooth: a real linear pan is untouched");
  }

  // 4h-d. smoothObjPath: the OBJECT track (t,az,el,q) — a background-lookalike
  // LATCH (a lone big jump on a low-q frame) is despiked back onto the path,
  // per-frame jitter is damped, and a real smooth object sweep passes through.
  {
    const mko = (t, az, el, q) => ({ t, az, el, q });
    // a clean diagonal sweep + one latch spike at i=4 (low q) + small jitter
    const truthAz = (i) => 100 + i * 2, truthEl = (i) => 20 + i * 1;
    const op = Array.from({ length: 9 }, (_, i) => {
      if (i === 4) return mko(i * 0.25, 130, 40, 0.1);           // LATCH: ~22° az / ~16° el off, low confidence
      const j = i % 2 ? 0.3 : -0.3;                              // sub-degree matcher jitter
      return mko(i * 0.25, truthAz(i) + j, truthEl(i) + j, 0.9); // strong pixel locks
    });
    const spiked = smoothObjPath(op);
    approx(spiked >= 1 ? 1 : 0, 1, 0, "smoothObj: the background latch was caught");
    approx(Math.abs(op[4].az - truthAz(4)) < 3 ? 1 : 0, 1, 0, "smoothObj: latch az pulled back onto the sweep");
    approx(Math.abs(op[4].el - truthEl(4)) < 3 ? 1 : 0, 1, 0, "smoothObj: latch el pulled back onto the sweep");
    const jit = Math.max(...op.slice(1, -1).filter((_, i) => i + 1 !== 4).map((p, i0) => { const i = op.indexOf(p); return Math.abs(p.az - truthAz(i)); }));
    approx(jit < 0.3 ? 1 : 0, 1, 0, "smoothObj: per-frame jitter damped (<0.3°)");
    // a perfectly smooth sweep (all strong) barely moves — real motion survives
    const clean = Array.from({ length: 9 }, (_, i) => mko(i * 0.25, truthAz(i), truthEl(i), 0.9));
    smoothObjPath(clean);
    approx(Math.max(...clean.map((p, i) => Math.abs(p.az - truthAz(i)))) < 1e-6 ? 1 : 0, 1, 0, "smoothObj: a clean linear sweep is untouched");
  }

  // 4h-e. videoKinematics: dense per-frame ANGULAR kinematics from objPath.
  // An object crossing the sky at a constant angular rate near the horizon
  // (az sweeps, el ~0 so dθ ≈ dAz) must recover ω ≈ that rate, sweep ≈ ω·dur;
  // with a per-frame angular SIZE that halves, the range doubles and speed at
  // an assumed distance includes the radial (approach) component.
  {
    const RATE = 4;         // deg/s of true angular motion
    const dt = 0.2, N = 21; // 4 s clip
    const objPath = Array.from({ length: N }, (_, i) => ({ t: +(i * dt).toFixed(3), az: (100 + RATE * i * dt) % 360, el: 0, q: 0.9 }));
    const vk = videoKinematics({ objPath });
    approx(vk != null ? 1 : 0, 1, 0, "videoKin: returns a result for a ≥3-sample track");
    approx(vk.avgOmega, RATE, 0.05, "videoKin: average angular rate ≈ truth (deg/s)");
    approx(Math.abs(vk.peakOmega - RATE) < 0.1 ? 1 : 0, 1, 0, "videoKin: peak angular rate ≈ truth (constant-rate clip)");
    approx(vk.sweep, RATE * (N - 1) * dt, 0.1, "videoKin: total angular sweep = rate × duration");
    // no sizes → atDistance is pure tangential at a fixed range
    const flat = vk.atDistance(1000);
    approx(flat.avgSpeed, 1000 * (RATE * D2R), 1.5, "videoKin: tangential speed = range · ω(rad/s) when size is constant");
    approx(flat.sizeM == null ? 1 : 0, 1, 0, "videoKin: no size info ⇒ no linear size");
    // add sizes that HALVE across the clip → range doubles, rangeRatio ≈ 2
    const track = [{ t: 0, ang: 2.0 }, { t: (N - 1) * dt, ang: 1.0 }];
    const vk2 = videoKinematics({ objPath, track });
    approx(vk2.rangeRatio, 2, 0.06, "videoKin: range ratio from a halving angular size ≈ 2×");
    const d2 = vk2.atDistance(1000);
    approx(d2.sizeM > 0 && d2.peakSpeed > flat.peakSpeed ? 1 : 0, 1, 0, "videoKin: receding object's speed exceeds the pure-tangential case");
  }

  // 4h-f. stereoVideo: two observers' DENSE object tracks triangulate into a 3D
  // trajectory, auto-syncing a wrong clock and shrugging off angular noise + a
  // few mistracked outlier frames. Two clips shot simultaneously; observer B's
  // EXIF clock is 0.7 s fast, so the solver must recover offset ≈ −0.7 s.
  {
    const o0 = { lat: 42.30, lon: -122.90, alt: 400 };
    const o1 = { lat: 42.3016, lon: -122.8972, alt: 402 };  // ~250 m NE, small alt diff
    const refL = { lat: o0.lat, lon: o0.lon, alt: o0.alt };
    const P0 = enuFromGeo(o0.lat, o0.lon, o0.alt, refL);
    const P1 = enuFromGeo(o1.lat, o1.lon, o1.alt, refL);
    const objAt = (t) => [-150 + 90 * t, 650 + 30 * t, 480 + 22 * t]; // ENU metres, a moving craft ~800 m off
    const azelTo = (P, X) => dirToAzEl(unit(sub(X, P)));
    const fps = 0.1, N = 34;
    // deterministic ±0.012° angular noise (device/track jitter)
    const noise = (i, s) => (((Math.sin(i * 12.9898 + s * 78.233) * 43758.5453) % 1 + 1) % 1 - 0.5) * 0.024;
    const mkObj = (P, s) => Array.from({ length: N }, (_, i) => {
      const t = +(i * fps).toFixed(3), ae = azelTo(P, objAt(t));
      return { t, az: ae.az + noise(i, s), el: ae.el + noise(i, s + 5), q: 0.9 };
    });
    const objA = mkObj(P0, 1), objB = mkObj(P1, 2);
    // inject 2 outlier mistracked frames in B (a blurred-frame garbage direction)
    objB[10] = { ...objB[10], az: objB[10].az + 6, el: objB[10].el - 4 };
    objB[22] = { ...objB[22], az: objB[22].az - 5, el: objB[22].el + 5 };
    const T0 = 1_700_000_000; // arbitrary base epoch (s)
    const OFF = 0.7;          // B's clock runs 0.7 s fast
    const sA = { name: "A", lat: o0.lat, lon: o0.lon, alt: o0.alt, whenMs: T0 * 1000, objPath: objA };
    const sB = { name: "B", lat: o1.lat, lon: o1.lon, alt: o1.alt, whenMs: (T0 + OFF) * 1000, objPath: objB };
    const r = stereoVideo([sA, sB]);
    approx(r && r.ok ? 1 : 0, 1, 0, "stereoVideo: returns a fix from two dense tracks");
    approx(Math.abs(r.offset - (-OFF)) < 0.15 ? 1 : 0, 1, 0, "stereoVideo: recovers the wrong-clock offset (≈ −0.7 s)");
    approx(r.dropped >= 2 ? 1 : 0, 1, 0, "stereoVideo: the 2 mistracked frames are rejected as outliers");
    // trajectory accuracy at the recovered instants (compare to truth at each t
    // relative to A's clock, since A's base = T0 and samples are t = frameTime)
    let maxPosErr = 0;
    for (let i = 0; i < r.times.length; i++) {
      const tt = r.times[i] - T0;               // seconds into the clip
      const tru = objAt(tt);
      maxPosErr = Math.max(maxPosErr, mag(sub(r.pos[i], tru)));
    }
    approx(maxPosErr < 20 ? 1 : 0, 1, 0, "stereoVideo: dense 3D trajectory within 20 m of truth (noisy rays)");
    approx(r.syncConf > 0.05 ? 1 : 0, 1, 0, "stereoVideo: a moving object gives a usable sync confidence");
    approx(r.k && r.k.peakSpeed > 0 ? 1 : 0, 1, 0, "stereoVideo: kinematics (speed) recovered from the fixed path");

    // 4h-f2. mixedStereo: one VIDEO (dense track) + one STILL (single sight-line)
    // → absolute trajectory. The still saw the object mid-clip from a second
    // spot; the anchor search must find that instant, triangulate the true
    // distance, and (with a size profile) recover the whole path + true size.
    {
      const Tmid = 1.7;                                   // the still's capture time (s into the clip)
      const oMid = objAt(Tmid);                           // ENU truth at that instant
      const aeS = dirToAzEl(unit(sub(oMid, P1)));         // still observer B's sight-line
      const stillSrc = { name: "Photo", lat: o1.lat, lon: o1.lon, alt: o1.alt, whenMs: (T0 + Tmid) * 1000,
        A: { az: aeS.az, el: aeS.el }, fovH: 60 };
      // size the video object at two frames so range varies (and true size resolves)
      const angOf = (t) => { const P = P0, X = objAt(t); const rng = mag(sub(X, P)); return 2 * Math.atan((6 / 2) / (rng)) * R2D; }; // 6 m object
      const vidSized = { ...sA, track: [{ t: 0.2, ang: angOf(0.2) }, { t: 3.0, ang: angOf(3.0) }] };
      const mx = mixedStereo([vidSized, stillSrc]);
      approx(mx && mx.ok ? 1 : 0, 1, 0, "mixedStereo: returns a fix from a video + a still");
      approx(Math.abs(mx.anchor.vt - Tmid) < 0.25 ? 1 : 0, 1, 0, "mixedStereo: anchor search lands on the still's instant");
      const truthDist = mag(sub(oMid, P0));
      approx(Math.abs(mx.anchor.dist - truthDist) / truthDist < 0.05 ? 1 : 0, 1, 0, "mixedStereo: anchor range ≈ truth (<5%)");
      approx(mx.anchor.sizeM != null && Math.abs(mx.anchor.sizeM - 6) < 1.5 ? 1 : 0, 1, 0, "mixedStereo: true size ≈ 6 m (angular size × triangulated range)");
      // the propagated absolute path tracks truth at the sampled frames
      let mErr = 0; for (let i = 0; i < mx.times.length; i++) mErr = Math.max(mErr, mag(sub(mx.pos[i], objAt(mx.times[i]))));
      approx(mErr < 60 ? 1 : 0, 1, 0, "mixedStereo: absolute trajectory tracks truth across the clip");
      approx(mx.k && mx.k.peakSpeed > 0 ? 1 : 0, 1, 0, "mixedStereo: absolute kinematics recovered");
    }
  }

  // 4h-g. sampleShapeAt: keyframed SIZE + ROTATION interpolate smoothly along
  // the track. Quaternion SLERP (not matrix lerp) tumbles the attitude; size
  // ramps linearly; both clamp past the ends and fall back to the fitted shape.
  {
    // quaternion round-trip + SLERP midpoint
    const rz30 = rotZ3(30), rt = mat3FromQuat(quatFromMat3(rz30));
    approx(Math.max(...rz30.map((v, i) => Math.abs(v - rt[i]))) < 1e-9 ? 1 : 0, 1, 0, "quat: mat3→quat→mat3 round-trips");
    const mid = slerp3(rotZ3(0), rotZ3(60), 0.5), r30 = rotZ3(30);
    approx(Math.max(...r30.map((v, i) => Math.abs(v - mid[i]))) < 1e-6 ? 1 : 0, 1, 0, "slerp3: halfway rotZ 0→60 = rotZ 30");
    // a matrix LERP would shrink the rotation — confirm SLERP stays orthonormal
    const det = (m) => m[0] * (m[4] * m[8] - m[5] * m[7]) - m[1] * (m[3] * m[8] - m[5] * m[6]) + m[2] * (m[3] * m[7] - m[4] * m[6]);
    approx(Math.abs(det(slerp3(rotZ3(0), mul3(rotY3(80), rotZ3(70)), 0.37)) - 1) < 1e-9 ? 1 : 0, 1, 0, "slerp3: interpolated rotation stays a proper rotation (det=1)");
    // sampleShapeAt: 2 size marks ramp; rotation marks slerp; fallbacks
    const sf = { sizeNat: 100, rotM: I3 };
    const track = [
      { t: 0.0, wpx: 100, rotM: rotZ3(0) },
      { t: 2.0, wpx: 300, rotM: rotZ3(60) },
    ];
    const s1 = sampleShapeAt(track, sf, 1.0);
    approx(s1.wpx, 200, 1e-6, "sampleShapeAt: size ramps between two marks (midpoint 200 px)");
    approx(Math.max(...rotZ3(30).map((v, i) => Math.abs(v - s1.rotM[i]))) < 1e-6 ? 1 : 0, 1, 0, "sampleShapeAt: attitude slerps between two marks (midpoint = 30°)");
    const s0 = sampleShapeAt(track, sf, -5), sE = sampleShapeAt(track, sf, 99);
    approx(s0.wpx, 100, 1e-6, "sampleShapeAt: clamps size before the first mark");
    approx(sE.wpx, 300, 1e-6, "sampleShapeAt: clamps size after the last mark");
    const none = sampleShapeAt([{ t: 0, x: 1, y: 1 }], sf, 1);
    approx(none.wpx == null ? 1 : 0, 1, 0, "sampleShapeAt: no size marks ⇒ wpx null (caller uses fitted size)");
    approx(Math.max(...I3.map((v, i) => Math.abs(v - none.rotM[i]))) < 1e-9 ? 1 : 0, 1, 0, "sampleShapeAt: no attitude marks ⇒ fitted rotM");
    // baseline injection: the FIT is an implicit keyframe at markT (wFit,
    // shapeFit.rotM) so a SINGLE adjustment RAMPS from the fit instead of
    // going constant — "changes transition from changes, not from un-adjusted
    // points". Fit width 50 px @ markT=0; the only sized point is 100 px @ t=4.
    {
      const one = [{ t: 4.0, wpx: 100 }];
      const optB = { markT: 0, wFit: 50 };
      // WITHOUT the baseline it would be constant 100 everywhere:
      approx(sampleShapeAt(one, sf, 2.0).wpx, 100, 1e-6, "sampleShapeAt: single size mark alone ⇒ constant (no baseline)");
      // WITH the baseline it ramps 50→100 over t∈[0,4]:
      approx(sampleShapeAt(one, sf, 0.0, optB).wpx, 50, 1e-6, "sampleShapeAt: baseline anchors the fit width at markT");
      approx(sampleShapeAt(one, sf, 2.0, optB).wpx, 75, 1e-6, "sampleShapeAt: single adjustment RAMPS from the fit baseline (midpoint 75 px)");
      approx(sampleShapeAt(one, sf, 4.0, optB).wpx, 100, 1e-6, "sampleShapeAt: reaches the adjusted value at its keyframe");
      approx(sampleShapeAt(one, sf, 9.0, optB).wpx, 100, 1e-6, "sampleShapeAt: clamps past the adjusted keyframe");
      // a real keyframe ON markT is NOT duplicated (baseline skipped):
      approx(sampleShapeAt([{ t: 0, wpx: 120 }, { t: 4, wpx: 100 }], sf, 0.0, optB).wpx, 120, 1e-6, "sampleShapeAt: real keyframe on markT wins over the baseline");
      // no baseline supplied ⇒ un-adjusted case still returns fit (wpx null):
      approx(sampleShapeAt([{ t: 0, x: 1, y: 1 }], sf, 1, optB).wpx == null ? 1 : 0, 1, 0, "sampleShapeAt: baseline needs ≥1 real mark (no marks ⇒ still null)");
      // rotation baseline: one attitude mark @ t=4 (rotZ 60) ramps from fit (I3) at markT=0
      const oneR = [{ t: 4.0, rotM: rotZ3(60) }];
      const rMid = sampleShapeAt(oneR, sf, 2.0, { markT: 0, wFit: 50 }).rotM;
      approx(Math.max(...rotZ3(30).map((v, i) => Math.abs(v - rMid[i]))) < 1e-6 ? 1 : 0, 1, 0, "sampleShapeAt: single attitude mark slerps from the fit baseline (midpoint 30°)");
    }
  }

  // 4h-c. mp4 muxer (WebCodecs export container): structural integrity —
  // box layout, chunk offset landing on the mdat payload, sample table
  // consistency. (Full decode validation was done against a real x264
  // elementary stream; this guards the byte-packing against regressions.)
  {
    const avcC = new Uint8Array([1, 0x64, 0, 0x28, 0xff, 0xe1, 0, 2, 0x67, 0x64, 1, 0, 1, 0x68, 0xee]);
    const samples = [
      { data: new Uint8Array([0, 0, 0, 3, 1, 2, 3]), key: true },
      { data: new Uint8Array([0, 0, 0, 2, 9, 9]), key: false },
      { data: new Uint8Array([0, 0, 0, 1, 5]), key: true },
    ];
    const f = muxMp4({ width: 320, height: 180, fps: 30, avcC, samples });
    const dv = new DataView(f.buffer);
    const tag = (o) => String.fromCharCode(f[o], f[o + 1], f[o + 2], f[o + 3]);
    approx(tag(4) === "ftyp" ? 1 : 0, 1, 0, "mp4: starts with ftyp");
    const ftypLen = dv.getUint32(0), moovLen = dv.getUint32(ftypLen);
    approx(tag(ftypLen + 4) === "moov" ? 1 : 0, 1, 0, "mp4: moov follows ftyp");
    const mdatOff = ftypLen + moovLen;
    approx(tag(mdatOff + 4) === "mdat" ? 1 : 0, 1, 0, "mp4: mdat follows moov");
    approx(dv.getUint32(mdatOff), 8 + 7 + 6 + 5, 0, "mp4: mdat size = header + samples");
    let stco = -1, stsz = -1, stss = -1;
    for (let i = ftypLen; i < mdatOff; i++) {
      const t = tag(i);
      if (t === "stco") stco = i; else if (t === "stsz") stsz = i; else if (t === "stss") stss = i;
    }
    approx(dv.getUint32(stco + 12), mdatOff + 8, 0, "mp4: stco entry points at the mdat payload");
    approx(dv.getUint32(stsz + 12), 3, 0, "mp4: stsz sample count");
    approx(dv.getUint32(stsz + 16), 7, 0, "mp4: first sample size");
    approx(dv.getUint32(stss + 8), 2, 0, "mp4: two sync samples");
    approx(dv.getUint32(stss + 12), 1, 0, "mp4: first sync sample is #1");
    approx(f[mdatOff + 8 + 4], 1, 0, "mp4: first sample payload lands at the chunk offset");
  }

  // 4i. posePathAt: wrap-aware pose interpolation between path samples
  {
    const pp = [
      { t: 0, az: 358, el: 10, roll: 0, fov: 60, k: 0, n: 20 },
      { t: 1, az: 2, el: 12, roll: 1, fov: 50, k: 0, n: 10 },
    ];
    const mid = posePathAt(pp, 0.5);
    approx(mid.az, 0, 0.01, "posePathAt: az wraps through north (358→2 mid = 0)");
    approx(mid.el, 11, 0.01, "posePathAt: el lerps");
    approx(mid.fov, 55, 0.01, "posePathAt: fov lerps");
    approx(posePathAt(pp, -5).az, 358, 0.01, "posePathAt: clamps before the start");
    approx(posePathAt(pp, 9).fov, 50, 0.01, "posePathAt: clamps past the end");
  }

  // 4d. smearDrift: the correction distributes linearly in time across the span
  {
    const pth = [
      { t: 0, az: 100, el: 10, roll: 0, fov: 60 },   // last anchor
      { t: 1, az: 100, el: 10, roll: 0, fov: 60 },
      { t: 2, az: 100, el: 10, roll: 0, fov: 60 },
      { t: 3, az: 100, el: 10, roll: 0, fov: 60 },
      { t: 4, az: 101, el: 10.8, roll: 0, fov: 60 }, // anchored entry (already absolute)
    ];
    smearDrift(pth, 1, 4, 0, { dAz: 1, dEl: 0.8, dRoll: 0, dFov: 0 });
    approx(pth[2].az, 100.5, 0.01, "smearDrift: midpoint gets half the correction");
    approx(pth[3].el, 10.6, 0.01, "smearDrift: 3/4 point gets 3/4");
    approx(pth[0].az, 100, 0, "smearDrift: the anchor itself is untouched");
    approx(pth[4].az, 101, 0, "smearDrift: the anchored entry is untouched");
  }

  // 5. combined day+night references: star blobs AND a textured foreground
  // (tree) both contribute in auto mode — neither excludes the other
  {
    const data = new Uint8ClampedArray(TW * TH * 4);
    for (let i = 0; i < TW * TH; i++) data[i * 4 + 3] = 255;
    for (let i = 0; i < 12; i++) drawBlob(data, 30 + (i % 4) * 90 + (i * 7) % 23, 30 + Math.floor(i / 4) * 60, 235); // "stars" in the upper sky
    for (let y = 210; y < 280; y++) for (let x = 40; x < 110; x++) { const v = ((x * 31 + y * 17) % 7) * 30; const p = (y * TW + x) * 4; data[p] = v; data[p + 1] = v; data[p + 2] = v; } // "tree": textured block, strong gradients
    const fs = detectBgFeatures(data, TW, TH, { maxN: 60 });
    const nearTree = fs.filter((f) => f.x >= 35 && f.x <= 115 && f.y >= 205 && f.y <= 285).length;
    approx(fs.length >= 12 ? 1 : 0, 1, 0, "detectBgFeatures: stars + structure both found");
    approx(nearTree >= 1 ? 1 : 0, 1, 0, "detectBgFeatures: textured foreground (tree) contributes references");
  }
}

// --- aspectSpan: two-view mirror ambiguity (same-span mirror must be reported) ---
{
  const D = Math.PI / 180;
  const mk = (b1, b2, S, psi) => ({
    ok: true,
    perSource: [{ size: S * Math.abs(Math.sin((psi - b1) * D)) }, { size: S * Math.abs(Math.sin((psi - b2) * D)) }],
    obs: [{ s: { A: { az: b1 } } }, { s: { A: { az: b2 } } }],
  });
  // bearings 90° apart → the second exact-fit axis has the SAME span (regression:
  // the old gate required the span to differ and silently dropped this mirror).
  const r = aspectSpan(mk(20, 110, 30, 70));
  approx(r.length, 2, 0, "aspectSpan: same-span mirror reported (2 candidates)");
  approx(r[0].S, 30, 0.6, "aspectSpan: primary span ≈ 30");
  approx(r[1].S, 30, 0.6, "aspectSpan: mirror span ≈ 30 (same span)");
  const axes = [r[0].psi, r[1].psi].sort((a, b) => a - b);
  approx(axes[0], 70, 1.0, "aspectSpan: axis A ≈ 70°");
  approx(axes[1], 150, 1.0, "aspectSpan: axis B ≈ 150°");
  // a genuine DIFFERENT-span mirror must ALSO still be reported
  const r2 = aspectSpan(mk(10, 95, 25, 40));
  approx(r2.length, 2, 0, "aspectSpan: different-span mirror still reported");
}

// --- astro azimuth-origin convention + moon fraction (coverage gap) ---
{
  const t = Date.UTC(2026, 5, 21, 6, 0);           // arbitrary instant
  const lat = 42, lon = -123;
  // a star AT the north celestial pole (Dec=+90) is due NORTH at el = latitude,
  // at any time — the definitive azimuth-origin check (north, not south).
  const pole = raDecToAzEl(0, 90, t, lat, lon);
  approx(pole.alt, lat, 0.05, "astro: pole star elevation = latitude");
  approx(Math.min(pole.az, 360 - pole.az), 0, 0.2, "astro: pole star azimuth = north (0°)");
  // moon illumination fraction stays a physical [0,1] across a synodic month
  let lo = 1, hi = 0;
  for (let k = 0; k < 30; k++) { const f = moonFrac(t + k * 86400000); lo = Math.min(lo, f); hi = Math.max(hi, f); }
  approx(lo >= 0 && lo < 0.05 ? 1 : 0, 1, 0, "astro: moonFrac reaches ~new (≥0)");
  approx(hi <= 1 && hi > 0.9 ? 1 : 0, 1, 0, "astro: moonFrac reaches ~full (≤1)");
  // the Sun is up by day and its azimuth is a real bearing
  const s = sunPos(t, lat, lon);
  approx(s.az >= 0 && s.az < 360 ? 1 : 0, 1, 0, "astro: sun azimuth in [0,360)");
}

// --- winds check (coverage gap): pressure-level altitude + balloon verdict ---
{
  approx(nearestLevel(5600)[0], 500, 0, "winds: ~5.6 km → 500 hPa level");
  approx(nearestLevel(11800)[0], 200, 0, "winds: ~11.8 km → 200 hPa level");
  // object moving WITH the wind (same heading, similar speed) = balloon-consistent
  const wind = { speedMs: 10, fromDeg: 270, driftDeg: 90, hPa: 500, levelM: 5570 };
  const bc = balloonVerdict(11, 92, wind);
  approx(bc.verdict === "balloon-consistent" ? 1 : 0, 1, 0, "winds: with-wind motion → balloon-consistent");
  // object crossing the wind at 5× its speed = not wind-borne
  const nb = balloonVerdict(50, 0, wind);
  approx(nb.verdict === "not wind-borne" ? 1 : 0, 1, 0, "winds: cross-wind fast motion → not wind-borne");
}

// --- auto star-align (plate solve): detect blobs, and recover a known pose ---
{
  // detectStars: bright dots survive, a diffuse cloud is rejected as over-size
  const w = 80, h = 60, img = new Uint8ClampedArray(w * h * 4);
  for (let i = 0; i < w * h; i++) { img[i * 4] = img[i * 4 + 1] = img[i * 4 + 2] = 6; img[i * 4 + 3] = 255; }
  const put = (x, y, v) => { const p = (y * w + x) * 4; img[p] = img[p + 1] = img[p + 2] = v; };
  // a big dim cloud (12×12 ≈ 144 px, above noise but diffuse)
  for (let y = 40; y < 52; y++) for (let x = 8; x < 20; x++) put(x, y, 46);
  // four crisp stars (2×2 bright)
  const dots = [[15, 10], [60, 12], [30, 45], [68, 50]];
  for (const [x, y] of dots) for (let dy = 0; dy < 2; dy++) for (let dx = 0; dx < 2; dx++) put(x + dx, y + dy, 235);
  const found = detectStars(img, w, h, {});
  approx(found.length >= 4 && found.length <= 8 ? 1 : 0, 1, 0, "platesolve: 4 stars found, cloud rejected");
  const nearDot = found.filter((s) => dots.some(([x, y]) => Math.hypot(s.x - (x + 0.5), s.y - (y + 0.5)) < 2)).length;
  approx(nearDot, 4, 0, "platesolve: all four star centroids located");

  // background subtraction: stars survive a strong Milky-Way-like GRADIENT +
  // broad glow, and the smooth brightness ramp yields no false detections
  const gw = 120, gh = 90, gimg = new Uint8ClampedArray(gw * gh * 4);
  for (let y = 0; y < gh; y++) for (let x = 0; x < gw; x++) {
    const ramp = 12 + 70 * (x / gw) + 40 * Math.exp(-(((x - 80) ** 2 + (y - 45) ** 2) / 600)); // gradient + a soft glow blob
    const p = (y * gw + x) * 4; gimg[p] = gimg[p + 1] = gimg[p + 2] = ramp; gimg[p + 3] = 255;
  }
  const gdots = [[20, 20], [95, 30], [40, 65], [100, 70], [60, 40]];
  for (const [x, y] of gdots) for (let dy = 0; dy < 2; dy++) for (let dx = 0; dx < 2; dx++) { const p = ((y + dy) * gw + (x + dx)) * 4; gimg[p] = gimg[p + 1] = gimg[p + 2] = 250; }
  const gfound = detectStars(gimg, gw, gh, {});
  const gnear = gfound.filter((s) => gdots.some(([x, y]) => Math.hypot(s.x - (x + 0.5), s.y - (y + 0.5)) < 2.5)).length;
  approx(gnear, 5, 0, "platesolve: stars found through a Milky-Way gradient+glow");
  approx(gfound.length <= 7 ? 1 : 0, 1, 0, "platesolve: gradient itself makes no false stars");

  // autoStarAlign: recover a known pose from a perturbed seed, ROBUST to
  // (a) clouds hiding some catalog stars, (b) a UFO-like bright blob that isn't
  // a catalog star, and (c) faint non-catalog clutter
  const natW = 4000, natH = 3000;
  const truth = { az: 230, el: 85, roll: -30, fov: 45, k: 0.02 };
  let seed = 12345; const rnd = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };
  const cat = [], catPx = [], det = [];
  for (let i = 0; i < 30; i++) {
    const px = (0.12 + 0.76 * rnd()) * natW, py = (0.12 + 0.76 * rnd()) * natH;
    cat.push({ g: pixToDirK(px, py, natW, natH, truth.az, truth.el, truth.roll, truth.fov, truth.k) });
    catPx.push({ px, py });
  }
  const covered = new Set([5, 17]); // two stars hidden behind cloud → no detection
  for (let i = 0; i < 30; i++) {
    if (covered.has(i)) continue;
    const p = dirToPixK(cat[i].g, natW, natH, truth.az, truth.el, truth.roll, truth.fov, truth.k);
    det.push({ x: p.px + (rnd() - 0.5) * 2, y: p.py + (rnd() - 0.5) * 2 }); // ±1 px noise
  }
  det.push({ x: catPx[5].px + 120, y: catPx[5].py - 90 }); // a "UFO": bright blob, matches no star — must be rejected
  for (let i = 0; i < 8; i++) det.push({ x: rnd() * natW, y: rnd() * natH }); // faint non-catalog clutter
  const sol = autoStarAlign(det, cat, natW, natH, { az: truth.az + 7, el: truth.el - 4, roll: truth.roll + 12, fov: truth.fov * 1.1, k: 0 });
  approx(sol ? 1 : 0, 1, 0, "platesolve: locked despite clouds + a UFO");
  if (sol) {
    approx(sol.az, truth.az, 0.6, "platesolve: az recovered (robust)");
    approx(sol.el, truth.el, 0.6, "platesolve: el recovered (robust)");
    approx(sol.roll, truth.roll, 1.2, "platesolve: roll recovered (robust)");
    approx(sol.fov, truth.fov, 1.5, "platesolve: FOV recovered (robust)");
    approx(sol.n >= 26 && sol.n <= 28 ? 1 : 0, 1, 0, "platesolve: matched visible stars, excluded UFO/clutter");
    approx(sol.rms < 0.6 ? 1 : 0, 1, 0, "platesolve: sub-degree fit rms");
  }

  // blindStarAlign: recover the SAME pose with NO seed at all (asterism match),
  // and with the FOV guess 10% wrong — the human never has to get it close
  const cat2 = [], det2 = [];
  for (let i = 0; i < 26; i++) {
    const px = (0.1 + 0.8 * rnd()) * natW, py = (0.1 + 0.8 * rnd()) * natH;
    cat2.push({ g: pixToDirK(px, py, natW, natH, truth.az, truth.el, truth.roll, truth.fov, truth.k), mag: 1 + i * 0.09 });
    const p = dirToPixK(cat2[i].g, natW, natH, truth.az, truth.el, truth.roll, truth.fov, truth.k);
    det2.push({ x: p.px + (rnd() - 0.5) * 2, y: p.py + (rnd() - 0.5) * 2 });
  }
  for (let i = 0; i < 6; i++) det2.push({ x: rnd() * natW, y: rnd() * natH }); // clutter
  const blind = blindStarAlign(det2, cat2, natW, natH, truth.fov * 0.9); // NO pose seed; FOV guess off by 10%
  approx(blind ? 1 : 0, 1, 0, "platesolve: seedless (blind) lock");
  if (blind) {
    approx(blind.az, truth.az, 0.8, "platesolve: blind az recovered");
    approx(blind.el, truth.el, 0.8, "platesolve: blind el recovered");
    approx(blind.roll, truth.roll, 1.5, "platesolve: blind roll recovered");
    approx(blind.fov, truth.fov, 2.0, "platesolve: blind FOV recovered");
  }

  // gridStarAlign: the "straight up, don't know the rotation" case — FOV known
  // (EXIF) + an elevation prior, DENSE catalog, recover the rotation from a
  // wrong roll/az with the el prior a couple degrees off
  const zt = { az: 250, el: 87, roll: 40, fov: 70, k: 0.01 };
  const gcat = [], gdet = [];
  let gs = 4321; const grnd = () => { gs = (gs * 1103515245 + 12345) & 0x7fffffff; return gs / 0x7fffffff; };
  for (let i = 0; i < 320; i++) {              // a dense (deep-catalog-like) sky
    const az = grnd() * 360, el = grnd() * 88 + 1;
    const g = dirFromAzEl(az, el);
    gcat.push({ g, mag: 1 + grnd() * 4, alt: el });
    const p = dirToPixK(g, natW, natH, zt.az, zt.el, zt.roll, zt.fov, zt.k);
    if (p && p.px > 0 && p.px < natW && p.py > 0 && p.py < natH) gdet.push({ x: p.px + (grnd() - 0.5) * 2, y: p.py + (grnd() - 0.5) * 2 });
  }
  for (let i = 0; i < 15; i++) gdet.push({ x: grnd() * natW, y: grnd() * natH }); // clutter
  const grid = gridStarAlign(gdet, gcat, natW, natH, { fov: zt.fov, elPrior: 89, elBand: 8, minGrid: 10, minMatch: 12, maxRms: 0.6 });
  approx(grid ? 1 : 0, 1, 0, "platesolve: grid (FOV+el-prior) lock, rotation unknown");
  if (grid) {
    approx(grid.az, zt.az, 0.6, "platesolve: grid az recovered");
    approx(grid.el, zt.el, 0.6, "platesolve: grid el recovered");
    approx(grid.roll, zt.roll, 1.2, "platesolve: grid roll recovered");
  }
}

// --- weather: cloud base (Espy LCL) + single-witness range/size cap ---
{
  approx(cloudBaseAGL(20, 12), 1000, 1e-6, "weather: Espy cloud base 125·(T−Td)");
  approx(cloudBaseAGL(15, 15), 0, 1e-6, "weather: saturated air → base at ground");
  const cb = cloudRangeBound(1000, 30, 2); // base 1000 m, 30° up, 2° wide
  approx(cb.maxRange, 2000, 1e-6, "weather: below-cloud range cap = base/sin(el)");
  approx(cb.maxSize, 2 * 2000 * Math.tan(1 * D2R), 1e-6, "weather: below-cloud size cap");
  approx(cloudRangeBound(1000, 0, 2) == null ? 1 : 0, 1, 0, "weather: no cap at/below horizon");
}

// --- meteor showers: active-window membership incl. year wrap ---
{
  const perseidsAug12 = activeShowers(Date.UTC(2026, 7, 12, 6));
  approx(perseidsAug12[0]?.name === "Perseids" && perseidsAug12[0].daysFromPeak === 0 ? 1 : 0, 1, 0, "meteors: Perseids active & at peak on Aug 12");
  const quadJan2 = activeShowers(Date.UTC(2026, 0, 2, 6)); // year-wrap window (Dec 28 → Jan 12)
  approx(quadJan2.some((s) => s.name === "Quadrantids") ? 1 : 0, 1, 0, "meteors: Quadrantids active Jan 2 (wrapped window)");
  approx(activeShowers(Date.UTC(2026, 2, 15, 6)).length, 0, 0, "meteors: none active mid-March");
}

// --- photometry: aperture flux, background subtraction, relative magnitude ---
{
  const w = 60, h = 60, D = new Uint8ClampedArray(w * h * 4);
  const put = (x, y, v) => { const i = (y * w + x) * 4; D[i] = D[i + 1] = D[i + 2] = v; D[i + 3] = 255; };
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) put(x, y, 20); // sky background = 20
  // bright source A at (20,30): a 3x3 block at 220 over bg
  for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) put(20 + dx, 30 + dy, 220);
  // fainter source B at (40,30): a 3x3 block at 120
  for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) put(40 + dx, 30 + dy, 120);
  const A = aperture(D, w, h, 20, 30, 4), B = aperture(D, w, h, 40, 30, 4);
  approx(A && A.bg > 15 && A.bg < 25 ? 1 : 0, 1, 0, "photometry: background recovered (~20)");
  approx(A.flux > B.flux ? 1 : 0, 1, 0, "photometry: brighter source has more net flux");
  // net flux ratio should reflect the (220-20) vs (120-20) contrast over equal areas
  approx(A.flux / B.flux, 200 / 100, 0.15, "photometry: flux ratio tracks contrast");
  // if A is the object and B (mag 1.0) the reference, A must come out brighter (smaller mag)
  const mA = relMag(A.flux, B.flux, 1.0);
  approx(mA < 1.0 ? 1 : 0, 1, 0, "photometry: brighter object → smaller magnitude");
  approx(relMag(0, 100, 1) == null ? 1 : 0, 1, 0, "photometry: zero flux → no magnitude");
  approx(colorDesc(255, 60, 40) === "red / orange" ? 1 : 0, 1, 0, "photometry: red colour classified");
  approx(colorDesc(240, 240, 240) === "white (saturated core)" ? 1 : 0, 1, 0, "photometry: saturated white classified");
}

// --- covariance ellipse: elongated along East → major bearing ≈ 90° ---
{
  const pts = [];
  for (let i = 0; i < 40; i++) { const t = (i / 39 - 0.5); pts.push([t * 100, t * 6 + (i % 2 ? 1 : -1)]); } // long in x (E), thin in y (N)
  const e = covEllipse(pts);
  approx(e.major > e.minor ? 1 : 0, 1, 0, "ellipse: major axis longer");
  approx(e.bearing, 90, 6, "ellipse: East-elongated cloud → major bears ~90°");
  approx(covEllipse([[0, 0], [1, 1]]) == null ? 1 : 0, 1, 0, "ellipse: needs ≥3 points");
}

// --- airports: parse, sort by distance, bearing ---
{
  const els = [
    { lat: 42.30, lon: -122.87, tags: { aeroway: "aerodrome", name: "Far Field", icao: "KXXX" } },     // north, ~15 km
    { center: { lat: 42.15, lon: -122.66 }, tags: { aeroway: "aerodrome", name: "Near Field", iata: "MFR", aerodrome: "international" } }, // ~2 km
    { lat: 42.16, lon: -123.66, tags: { building: "yes" } }, // not an aerodrome → dropped
  ];
  const aps = parseAirports(els, 42.155, -122.661); // observer beside "Near Field"
  approx(aps.length, 2, 0, "airports: non-aerodrome dropped");
  approx(aps[0].name === "Near Field" ? 1 : 0, 1, 0, "airports: nearest first");
  approx(aps[0].bearing >= 0 && aps[0].bearing < 360 ? 1 : 0, 1, 0, "airports: bearing in range");
}

// --- MANUAL POSE FROM HAND-MARKED REFERENCES: recover a pan+roll+zoom
//     sequence from synthetic marks of world-fixed features (the fallback for
//     clips the auto stabilizer can't solve). ---
{
  const natW = 1920, natH = 1080;
  const refPose = { t: 1.0, az: 100, el: 20, roll: 2, fov: 60, k: 0 };
  // 4 world-fixed features, defined by their pixels in the ALIGN frame → g
  const alignPix = [{ x: 400, y: 300 }, { x: 1500, y: 350 }, { x: 500, y: 800 }, { x: 1400, y: 750 }];
  const G = alignPix.map((p) => pixToDirK(p.x, p.y, natW, natH, refPose.az, refPose.el, refPose.roll, refPose.fov, refPose.k));
  // a truth pose per keyframe: the camera pans, rolls, and zooms
  const truth = [
    { t: 0.5, az: 95, el: 18, roll: 0, fov: 62 },
    { t: 1.0, az: 100, el: 20, roll: 2, fov: 60 },
    { t: 1.5, az: 106, el: 23, roll: 4, fov: 56 },
    { t: 2.0, az: 111, el: 25, roll: 5, fov: 52 },
  ];
  // build camRefs: each feature marked on every keyframe (project g → pixel)
  const camRefs = G.map((g) => ({
    marks: truth.map((T) => { const p = dirToPixK(g, natW, natH, T.az, T.el, T.roll, T.fov, 0); return { t: T.t, x: p.px, y: p.py }; }),
  }));
  const path = solveManualPoses(camRefs, refPose, { natW, natH });
  approx(path && path.length === 4 ? 1 : 0, 1, 0, "manualPose: a keyframe solved for every marked frame");
  if (path) {
    let maxAz = 0, maxEl = 0, maxRoll = 0, maxFov = 0;
    for (const T of truth) {
      const s = path.find((p) => Math.abs(p.t - T.t) < 1e-3);
      maxAz = Math.max(maxAz, Math.abs(((s.az - T.az + 540) % 360) - 180));
      maxEl = Math.max(maxEl, Math.abs(s.el - T.el));
      maxRoll = Math.max(maxRoll, Math.abs(s.roll - T.roll));
      maxFov = Math.max(maxFov, Math.abs(s.fov - T.fov));
    }
    approx(maxAz < 0.3 ? 1 : 0, 1, 0, `manualPose: az recovered across the pan (max err ${maxAz.toFixed(2)}°)`);
    approx(maxEl < 0.3 ? 1 : 0, 1, 0, `manualPose: el recovered (max err ${maxEl.toFixed(2)}°)`);
    approx(maxRoll < 0.4 ? 1 : 0, 1, 0, `manualPose: roll recovered (max err ${maxRoll.toFixed(2)}°)`);
    approx(maxFov < 1.0 ? 1 : 0, 1, 0, `manualPose: FOV/zoom recovered (max err ${maxFov.toFixed(2)}°)`);
  }
  // a SINGLE mark still fixes az/el (roll+fov held from the seed) — the common
  // "only the Moon is visible" case.
  const oneRef = [{ marks: [{ t: 1.0, x: alignPix[0].x, y: alignPix[0].y }, (() => { const p = dirToPixK(G[0], natW, natH, 108, 24, 2, 60, 0); return { t: 2.0, x: p.px, y: p.py }; })()] }];
  const p1 = solveManualPoses(oneRef, refPose, { natW, natH }, { lockFov: true });
  const s2 = p1 && p1.find((p) => Math.abs(p.t - 2.0) < 1e-3);
  approx(s2 && Math.abs(((s2.az - 108 + 540) % 360) - 180) < 0.5 && Math.abs(s2.el - 24) < 0.5 ? 1 : 0, 1, 0, "manualPose: a lone reference still fixes az/el");

  // HAND-OFF: feature A (on the align frame) leaves; fresh features B,C that
  // OVERLAP A for a frame carry the pose after A is gone — a mid-clip point
  // going off-frame must not break the solve.
  {
    const truthH = [
      { t: 1.0, az: 100, el: 20, roll: 2, fov: 60 },   // align
      { t: 1.4, az: 104, el: 21, roll: 2, fov: 60 },   // A + B + C
      { t: 1.8, az: 108, el: 22, roll: 3, fov: 60 },   // B + C (A gone)
      { t: 2.2, az: 112, el: 23, roll: 4, fov: 60 },   // B + C
    ];
    const gA = pixToDirK(500, 400, natW, natH, 100, 20, 2, 60, 0); // A anchored on align only
    // B,C directions chosen so they land in-frame across 1.4–2.2 (offset from center at t=1.8)
    const gB = pixToDirK(900, 500, natW, natH, 108, 22, 3, 60, 0);
    const gC = pixToDirK(700, 800, natW, natH, 108, 22, 3, 60, 0);
    const pxAt = (g, T) => dirToPixK(g, natW, natH, T.az, T.el, T.roll, T.fov, 0);
    const A = { marks: [1.0, 1.4].map((t) => { const T = truthH.find((x) => x.t === t); const p = pxAt(gA, T); return { t, x: p.px, y: p.py }; }) };
    const mk = (g) => ({ marks: [1.4, 1.8, 2.2].map((t) => { const T = truthH.find((x) => x.t === t); const p = pxAt(g, T); return { t, x: p.px, y: p.py }; }) });
    const pH = solveManualPoses([A, mk(gB), mk(gC)], { t: 1.0, az: 100, el: 20, roll: 2, fov: 60, k: 0 }, { natW, natH });
    const gone = pH && pH.find((p) => Math.abs(p.t - 2.2) < 1e-3);   // solved only from B,C (A long gone)
    approx(gone && Math.abs(((gone.az - 112 + 540) % 360) - 180) < 0.4 && Math.abs(gone.el - 23) < 0.4 ? 1 : 0, 1, 0, "manualPose: pose handed off to fresh references after the first left frame");
  }

  // SMOOTHING: on a linear pan, one keyframe whose marks are jittered off should
  // be pulled back toward its neighbours' interpolation (imperfect placement).
  {
    const alignPix = [{ x: 400, y: 300 }, { x: 1500, y: 350 }, { x: 500, y: 800 }, { x: 1400, y: 750 }];
    const Gs = alignPix.map((p) => pixToDirK(p.x, p.y, natW, natH, 100, 20, 0, 60, 0));
    const rp = { t: 1.0, az: 100, el: 20, roll: 0, fov: 60, k: 0 };
    const lin = [0.6, 0.8, 1.0, 1.2, 1.4].map((t) => ({ t, az: 100 + (t - 1.0) * 10, el: 20, roll: 0, fov: 60 })); // smooth pan
    const jitter = (t) => (Math.abs(t - 1.2) < 1e-3 ? 14 : 0);   // ONE non-align keyframe's marks pushed ~14px off
    const camRefsS = Gs.map((g) => ({ marks: lin.map((T) => { const p = dirToPixK(g, natW, natH, T.az, T.el, T.roll, T.fov, 0); return { t: T.t, x: p.px + jitter(T.t), y: p.py }; }) }));
    const raw = solveManualPoses(camRefsS, rp, { natW, natH }, { smooth: false });
    const sm = solveManualPoses(camRefsS, rp, { natW, natH });   // smoothed (default)
    const mid = (path) => path.find((p) => Math.abs(p.t - 1.2) < 1e-3);
    const truthMid = 102; // linear pan → az at t=1.2 is exactly 102
    const devRaw = Math.abs(mid(raw).az - truthMid), devSm = Math.abs(mid(sm).az - truthMid);
    approx(devRaw > 0.3 ? 1 : 0, 1, 0, `manualPose: the jittered keyframe is off before smoothing (${devRaw.toFixed(2)}°)`);
    approx(devSm < devRaw * 0.7 ? 1 : 0, 1, 0, `manualPose: smoothing pulls it back toward the pan (${devSm.toFixed(2)}° < ${devRaw.toFixed(2)}°)`);
  }

  // HYBRID: auto is right in the first half, garbage (cloud-locked) in the
  // second. Manual keyframes are ground truth. hybridPath must KEEP auto where
  // it agrees and REPLACE it with manual where it's wrong — pinned to the marks.
  {
    // dense auto path: correct for t<3, then drifts +8° az wrong for t>=3
    const auto = [];
    for (let i = 0; i <= 12; i++) { const t = i * 0.5; const bad = t >= 3 ? 8 : 0; auto.push({ t, az: 100 + t * 2 + bad, el: 20, roll: 0, fov: 60, k: 0, n: 20 }); }
    // manual keyframes on the TRUE motion (no bad offset), sparse
    const man = [0, 1.5, 3, 4.5, 6].map((t) => ({ t, az: 100 + t * 2, el: 20, roll: 0, fov: 60, n: 4 }));
    const hyb = hybridPath(auto, man, { gate: 4 });
    const at = (t) => hyb.find((p) => Math.abs(p.t - t) < 1e-6);
    approx(at(3) && Math.abs(((at(3).az - 106 + 540) % 360) - 180) < 0.2 ? 1 : 0, 1, 0, "hybrid: truth pinned at the mark where auto goes bad (t=3)");
    approx(at(5) && Math.abs(((at(5).az - 110 + 540) % 360) - 180) < 0.6 ? 1 : 0, 1, 0, "hybrid: garbage auto span replaced by the marks (t=5)");
    // in the GOOD half a sample between marks should come from auto (dense shape kept)
    const g = at(1.0);
    approx(g && g.src === "auto" && Math.abs(((g.az - 102 + 540) % 360) - 180) < 0.3 ? 1 : 0, 1, 0, "hybrid: auto kept where it agrees with the marks (t=1)");
    approx(at(5) && at(5).src === "manual" ? 1 : 0, 1, 0, "hybrid: bad span tagged as manual");
  }
}

if (fails) { console.error(`\nmathcheck: ${fails} assertion(s) failed`); process.exit(1); }
console.log("mathcheck: all assertions passed");
