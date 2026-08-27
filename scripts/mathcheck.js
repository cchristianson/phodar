// Exercises the REAL math core (src/math/*) — not a copy. A regression in
// triangulation, geodesy, or angular sizing fails `npm test` here.
import fs from "node:fs";
import { D2R, R2D, RE, enuFromGeo, geoFromEnu, dirFromAzEl, sub, mag } from "../src/math/geodesy.js";
import { intersectLines, analyze, aspectSpan, covEllipse } from "../src/math/triangulate.js";
import { sunPos, moonFrac } from "../src/math/astro.js";
import { nearestLevel, balloonVerdict } from "../src/checks/winds.js";
import { rankCandidates, spanForAircraft } from "../src/checks/adsb.js";
import { trackQuality, trackAngAt, trackDirections, sourceTrack, videoKinematics, stereoVideo, mixedStereo, analyzeTracks, trackSegments, interSegments, inSegments, segsDur, kinematics } from "../src/math/kinematics.js";
import { skylineFromSampler, skylineElAt, AZ_STEP, matchSkyline, detectSkyline } from "../src/terrain.js";
import { raDecToAzEl, starAzEl, precessFromJ2000, refractionDeg, moonPos } from "../src/math/astro.js";
import { declination } from "../src/math/geomag.js";
import { parseMediaMeta } from "../src/exif.js";
import { sensorAt, syncSensor, fuseSensorVisual, fuseStats, motionDisagreement, sensorOnlyPath } from "../src/video/sensorpath.js";
import { planetPositions } from "../src/math/planets.js";
import { STARS } from "../src/math/starcat.js";
import { photoBasis, solveRollFov, pixToDirK, dirToPixK, solvePoseAnchors, reanchorDir, reanchorAzEl, reanchorPose } from "../src/math/projection.js";
import { solveManualPoses, solvePose } from "../src/video/manualpose.js";
import { rayToGround, pixelToGround, groundSpanM, groundKinematics, haversineM, bearingDeg as bearingDegGeo, groundHomography, pixelToGroundH, groundSpanH } from "../src/math/geolocate.js";
import { poseFromGravity, poseFromOrientation, upFromOrientation, gravitySign, poseQuality } from "../src/capture/pose.js";
import { unit, dot, dirToAzEl } from "../src/math/geodesy.js";
import { parseLaunches, haversineKm } from "../src/checks/launches.js";
import { parseFireballs } from "../src/checks/fireballs.js";
import { parsePeaks, bearingDeg, distM } from "../src/checks/peaks.js";
import { heightMeters, parseOverpassBuildings, buildingHeightSampler, buildingBoxes, boxesPeak, convexHull2, segInsideHull, visibleSegs } from "../src/buildings.js";
import { detectStars, autoStarAlign, blindStarAlign, gridStarAlign } from "../src/checks/platesolve.js";
import { detectBgFeatures, trackFeatures, poseFromTracks, initTracker, stepTracker, stepObject, snapToObject, pinFind, pinStep, smearDrift, despikePath, smoothPath, smoothObjPath, smoothPathAt, smoothObjPathAt, posePathAt, registerToRef, grayDown, applyPoseFixes, applyDirFixes, snapDirsToAnchors } from "../src/video/postrack.js";
import { rotZ3, rotY3, mul3, I3, quatFromMat3, mat3FromQuat, slerp3, sampleShapeAt, shapeWire, SHAPES, SHAPE_R0, shapeProjNat, pinApparentCenter, normSizedTrack } from "../src/shapes.js";
import { muxMp4 } from "../src/video/mp4mux.js";
import { cloudBaseAGL, cloudRangeBound } from "../src/checks/weather.js";
import { activeShowers } from "../src/checks/meteorshowers.js";
import { aperture, relMag, colorDesc } from "../src/checks/photometry.js";
import { parseAirports } from "../src/checks/airports.js";
import { parseFlightLog, parseWhen, thinLog, logStateAt, droneAltM, droneAzElRange, logMomentPer, syncLogTime, calibrationSummary, gradeCalibration, witnessClockCheck, witnessStatedMs, DRONE_PRESETS } from "../src/checks/flightlog.js";
import { analyzeSession } from "../src/analyze/engine.js";

let fails = 0;
const approx = (got, want, tol, msg) => {
  const ok = Math.abs(got - want) <= tol;
  if (!ok) { fails++; console.error(`  FAIL ${msg}: got ${got}, want ${want} ±${tol}`); }
  else console.log(`  ok   ${msg}: ${got.toFixed ? got.toFixed(2) : got}`);
  return ok;
};
/* plain boolean assertion + a section label — the file grew a lot of
   hand-rolled `if (…) …; else { fails++; console.error(…) }`; these are the
   same thing, said once */
const ok = (cond, msg) => {
  if (!cond) { fails++; console.error(`  FAIL ${msg}`); }
  else console.log(`  ok   ${msg}`);
  return !!cond;
};
const head = (title) => console.log(`\n— ${title} —`);

// --- ENU round-trip: a due-east baseline on the WGS84 ellipsoid ---
// These expectations used to be 2000 ±1 m and 0 ±0.1 m, which encoded the OLD
// equirectangular model (one mean radius, flat plane). The true values, both
// confirmed against independent algorithms:
//   east  2004.7965 m — matches Vincenty's inverse geodesic to 0.0000 m; the
//         old model was 4.80 m short (the +0.264% east-scale error at lat 42).
//   north 0.2849 m — two points at the SAME latitude are joined by a chord that
//         cuts inside the parallel, so it has a small poleward component in the
//         local frame. Closed form d²·tan(lat)/(2N) gives the same 0.2849 m.
const ref = { lat: 42.16380, lon: -123.64800, alt: 0 };
const P1 = enuFromGeo(42.16380, -123.64800, 0, ref);
const P2 = enuFromGeo(42.16380, -123.62374, 0, ref);
approx(P2[0], 2004.7965, 0.001, "east baseline (ellipsoidal, = Vincenty)");
approx(P2[1], 0.2849, 0.001, "north component of a same-latitude chord");

// --- Fix A: two crossing bearings recover a known point ---
const A = intersectLines([
  { P: P1, d: dirFromAzEl(18.43, 32.31) },
  { P: P2, d: dirFromAzEl(341.57, 32.31) },
]);
approx(A.rmsMiss, 0, 0.5, "Fix A rms miss");
// the ranges scale with the baseline: 3742 × (2004.7965/2000) = 3750.97, and
// the fix lands at 3751.5 — the same +0.24% correction, propagated.
approx(A.ts[0], 3751.5, 3, "Fix A range from obs1");
approx(A.ts[1], 3751.5, 3, "Fix A range from obs2");

// --- GEODESY IS EXACT: drive analyze() with truth built independently in ECEF.
// Each observer's az/el comes from its OWN true local vertical, i.e. the angles
// a perfect instrument would read on site. Before the ellipsoidal ENU + the
// per-observer frame rotation these missed by 28 m (5 km baseline) and 150 m
// (20 km), while still reporting a 0.000 m ray-miss and grading "excellent".
{
  const WA = 6378137.0, WF = 1 / 298.257223563, WE2 = WF * (2 - WF);
  const ec = (lat, lon, h) => {
    const p = lat * D2R, l = lon * D2R, sn = Math.sin(p), N = WA / Math.sqrt(1 - WE2 * sn * sn);
    return [(N + h) * Math.cos(p) * Math.cos(l), (N + h) * Math.cos(p) * Math.sin(l), (N * (1 - WE2) + h) * sn];
  };
  const bs = (lat, lon) => { const p = lat * D2R, l = lon * D2R;
    return { E: [-Math.sin(l), Math.cos(l), 0], N: [-Math.sin(p) * Math.cos(l), -Math.sin(p) * Math.sin(l), Math.cos(p)],
      U: [Math.cos(p) * Math.cos(l), Math.cos(p) * Math.sin(l), Math.sin(p)] }; };
  const dt3 = (u, v) => u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
  const azelTrue = (o, t) => {
    const O = ec(o.lat, o.lon, o.alt), P = ec(t.lat, t.lon, t.alt), b = bs(o.lat, o.lon);
    const v = [P[0] - O[0], P[1] - O[1], P[2] - O[2]];
    const e = dt3(v, b.E), n = dt3(v, b.N), u = dt3(v, b.U);
    return { az: ((Math.atan2(e, n) * R2D) + 360) % 360, el: Math.atan2(u, Math.hypot(e, n)) * R2D, range: Math.hypot(e, n, u) };
  };
  const off = (r, east, north) => {
    const p = r.lat * D2R, sn = Math.sin(p);
    const N = WA / Math.sqrt(1 - WE2 * sn * sn), M = WA * (1 - WE2) / Math.pow(1 - WE2 * sn * sn, 1.5);
    return { lat: r.lat + (north / M) * R2D, lon: r.lon + (east / (N * Math.cos(p))) * R2D, alt: r.alt };
  };
  const d3 = (u, v) => { const p = ec(u.lat, u.lon, u.alt), q = ec(v.lat, v.lon, v.alt); return Math.hypot(q[0] - p[0], q[1] - p[1], q[2] - p[2]); };
  for (const [nm, base, rng, dalt, lat, tol] of [
    ["120 m", 60, 120, 12, 42.3, 0.02], ["5 km baseline", 5000, 12000, 3000, 42.3, 0.05],
    ["20 km baseline", 20000, 40000, 10000, 42.3, 0.20], ["equator", 5000, 12000, 3000, 0.5, 0.05]]) {
    const o1 = { lat, lon: -122.90, alt: 400 }, o2 = off(o1, base, 0);
    const obj = { ...off(off(o1, base / 2, 0), 0, rng), alt: 400 + dalt };
    const t1 = azelTrue(o1, obj), t2 = azelTrue(o2, obj);
    /* a real camera records APPARENT angles: the atmosphere bends the ray, so
       the recorded elevation is higher than the geometric one by k·d/(2R). Add
       that here — otherwise this feeds vacuum angles to a pipeline that now
       (correctly) removes refraction, and the fix lands low by 18 m at 20 km. */
    const bend = (o, t) => {
      const dG = Math.cos(t.el * D2R) * t.range;
      return t.el + (0.13 * dG) / (2 * 6371000) * R2D;
    };
    t1.el = bend(o1, t1); t2.el = bend(o2, t2);
    const TS = 4.0, mkA = (r) => 2 * Math.atan(TS / 2 / r) * R2D;
    const fx = analyze([
      { name: "A", lat: o1.lat, lon: o1.lon, alt: o1.alt, A: { az: t1.az, el: t1.el, angManual: mkA(t1.range) }, B: {} },
      { name: "B", lat: o2.lat, lon: o2.lon, alt: o2.alt, A: { az: t2.az, el: t2.el, angManual: mkA(t2.range) }, B: {} },
    ]);
    approx(fx.ok ? 1 : 0, 1, 0, `exact-truth fix solves (${nm})`);
    approx(d3({ lat: fx.geoA.lat, lon: fx.geoA.lon, alt: fx.geoA.alt }, obj), 0, tol, `exact-truth 3-D position (${nm})`);
    approx(fx.geoA.alt - obj.alt, 0, tol, `exact-truth altitude (${nm})`);
    approx(fx.sizeAvg - TS, 0, 0.002, `exact-truth true size (${nm})`);
  }
}

// --- ASTRONOMICAL FRAME (audit findings 3-6). Everything used to be computed
// in a J2000-mean frame and read as if it were of-date: a ~0.37° error that a
// plate solve absorbs into the pose while still reporting ~0.04° rms.
{
  const T26 = Date.UTC(2026, 6, 30, 6, 0, 0), LA = 42.30, LO = -122.90;
  // precession carries J2000 coordinates to the equinox of date
  const sir = precessFromJ2000(101.287, -16.716, T26);   // Sirius
  approx(sir.ra, 101.5839, 0.002, "precession: Sirius RA J2000 -> of date");
  approx(sir.dec, -16.7453, 0.002, "precession: Sirius Dec J2000 -> of date");
  // the star path must USE it — starAzEl and raw raDecToAzEl differ by ~0.29°
  const dirOf = (az, el) => [Math.sin(az * D2R) * Math.cos(el * D2R), Math.cos(az * D2R) * Math.cos(el * D2R), Math.sin(el * D2R)];
  const sepOf = (p, q) => { const a = dirOf(p.az, p.alt), b = dirOf(q.az, q.alt);
    return Math.acos(Math.max(-1, Math.min(1, a[0] * b[0] + a[1] * b[1] + a[2] * b[2]))) * R2D; };
  approx(sepOf(starAzEl(101.287, -16.716, T26, LA, LO), raDecToAzEl(101.287, -16.716, T26, LA, LO)),
    0.2859, 0.01, "starAzEl precesses (J2000 vs of-date separation)");
  // starAzEl == precess-then-convert, exactly
  approx(sepOf(starAzEl(101.287, -16.716, T26, LA, LO), raDecToAzEl(sir.ra, sir.dec, T26, LA, LO)),
    0, 1e-9, "starAzEl = precessFromJ2000 + raDecToAzEl");
  // the Sun is of-date: at the published March-2026 equinox its declination is
  // ~0, so its altitude at the north pole is just refraction
  approx(sunPos(Date.UTC(2026, 2, 20, 14, 46), 90, 0).alt, refractionDeg(0), 0.06,
    "Sun is of-date (declination ~0 at the published equinox)");
  // refraction is applied to EVERY body, not just the Moon
  approx(refractionDeg(0) > 0.4 && refractionDeg(45) < 0.02 ? 1 : 0, 1, 0, "refraction model shape");
  const lowStar = starAzEl(sir.ra, sir.dec, T26, LA, LO);
  approx(lowStar.alt > -90 ? 1 : 0, 1, 0, "starAzEl returns a usable altitude");
  // the Moon must carry the principal periodic terms, not just the equation of
  // the centre. Compare the SHIPPED moonPos against an independent truncated
  // ELP over a lunation; with only the equation of the centre this was a mean
  // 0.82° and worst 1.19° out — wider than the Moon's own 0.52° disc.
  {
    const S = (x) => Math.sin(x * D2R), C = (x) => Math.cos(x * D2R);
    const toD = (ms) => ms / 86400000 - 0.5 + 2440588 - 2451545;
    const refMoon = (ms) => {                 // ecliptic lon/lat -> RA/Dec of date
      const d = toD(ms), Tc = d / 36525, e = (23.439291 - 0.0130042 * Tc) * D2R;
      const Lp = 218.3164477 + 13.176396474 * d, Dm = 297.8501921 + 12.190749117 * d;
      const M = 357.5291092 + 0.98560028 * d, Mp = 134.9633964 + 13.064992953 * d;
      const Fa = 93.2720950 + 13.229350449 * d;
      const lon = Lp + 6.288774 * S(Mp) + 1.274027 * S(2 * Dm - Mp) + 0.658314 * S(2 * Dm)
        + 0.213618 * S(2 * Mp) - 0.185116 * S(M) - 0.114332 * S(2 * Fa)
        + 0.058793 * S(2 * Dm - 2 * Mp) + 0.057066 * S(2 * Dm - M - Mp)
        + 0.053322 * S(2 * Dm + Mp) + 0.045758 * S(2 * Dm - M);
      const lat = 5.128122 * S(Fa) + 0.280602 * S(Mp + Fa) + 0.277693 * S(Mp - Fa)
        + 0.173237 * S(2 * Dm - Fa) + 0.055413 * S(2 * Dm - Mp + Fa) + 0.046271 * S(2 * Dm - Mp - Fa);
      const l = lon * D2R, b = lat * D2R;
      return { ra: Math.atan2(Math.sin(l) * Math.cos(e) - Math.tan(b) * Math.sin(e), Math.cos(l)) * R2D,
        dec: Math.asin(Math.sin(b) * Math.cos(e) + Math.cos(b) * Math.sin(e) * Math.sin(l)) * R2D };
    };
    let worst = 0, n = 0;
    for (let day = 0; day < 30; day++) {
      const ms = Date.UTC(2026, 6, 1 + day, 6), m = moonPos(ms, LA, LO);
      if (m.alt < 5) continue;
      const r = refMoon(ms), rp = raDecToAzEl(r.ra, r.dec, ms, LA, LO);
      worst = Math.max(worst, sepOf(m, rp)); n++;
    }
    approx(n > 8 ? 1 : 0, 1, 0, "moon: enough nights above 5° to test");
    approx(worst, 0, 0.15, "moon tracks a truncated ELP over a lunation (was 1.19deg out)");
  }
}

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

  // close-subject tells: SubjectDistance + Flash + SubjectDistanceRange parse
  // and surface as authenticity findings (the camera's own record of a near
  // subject vs a "distant craft" claim)
  {
    const t2 = new Uint8Array(76);
    const d2 = new DataView(t2.buffer);
    const W16 = (o, v) => d2.setUint16(o, v, true), W32 = (o, v) => d2.setUint32(o, v, true);
    t2[0] = 0x49; t2[1] = 0x49; W16(2, 42); W32(4, 8);            // II, magic, IFD0@8
    W16(8, 1); W16(10, 0x8769); W16(12, 4); W32(14, 1); W32(18, 26); W32(22, 0); // IFD0 → Exif@26
    W16(26, 3);                                                    // Exif IFD: 3 entries
    W16(28, 0x9206); W16(30, 5); W32(32, 1); W32(36, 68);          // SubjectDistance → rat@68
    W16(40, 0x9209); W16(42, 3); W32(44, 1); W16(48, 7);           // Flash 0b111: fired + return detected
    W16(52, 0xA40C); W16(54, 3); W32(56, 1); W16(60, 2);           // SubjectDistanceRange = close
    W32(64, 0);
    W32(68, 14); W32(72, 10);                                      // 1.4 m
    const jp = new Uint8Array(12 + 76);
    jp.set([0xFF, 0xD8, 0xFF, 0xE1, 0x00, 84, 0x45, 0x78, 0x69, 0x66, 0, 0], 0);
    jp.set(t2, 12);
    const m2 = parseMediaMeta(jp.buffer, false);
    ok(m2 && Math.abs(m2.subjDist - 1.4) < 1e-9 && m2.flash === 7 && m2.subjRange === 2, "EXIF close-subject tags parse (1.4 m, flash 0b111, range=close)");
    const { authDerived } = await import("../src/checks/authenticity.js");
    const finds = authDerived({ meta: m2 });
    ok(finds.some((x) => x.id === "subj-close" && x.level === "warn"), "auth: camera focused 1.4 m away → warn");
    ok(finds.some((x) => x.id === "flash-return" && x.level === "warn"), "auth: flash return light detected → warn");
    ok(finds.some((x) => x.id === "subj-range" && x.level === "warn"), "auth: camera-reported close range → warn");
    ok(authDerived({ meta: { subjDist: 240 } }).some((x) => x.id === "subj-dist" && x.level === "note"), "auth: a 240 m subject distance is a note, not a warn");
    ok(authDerived({ meta: { flash: 0x20 } }).length === 0, "auth: 'no flash function' (0x20) trips nothing");
    ok(authDerived({ meta: { flash: 1 } }).some((x) => x.id === "flash-fired" && x.level === "note"), "auth: flash fired without return → note");
  }
}

// --- QuickTime metadata at the END of the file. An iPhone writes `moov` last
//     (no faststart), so a head-only scan silently lost GPS + timestamp on
//     EVERY clip bigger than the window — measured on a 27 MB field recording
//     whose location string sat in the last 200 bytes. ---
{
  const mkMov = (size, atEnd) => {
    const u8 = new Uint8Array(size);
    for (let i = 0; i < size; i++) u8[i] = 0x20;                    // filler ('mdat' payload)
    const put = (off, s) => { for (let i = 0; i < s.length; i++) u8[off + i] = s.charCodeAt(i); };
    const moov = atEnd ? size - 400 : 100;
    put(moov, "moov");
    put(moov + 4, "mvhd");
    u8[moov + 8] = 0;                                               // version 0 → u32 creation time
    const sec = 2082844800 + 1700000000;                            // 1904 epoch + a real unix time
    u8[moov + 12] = (sec >>> 24) & 255; u8[moov + 13] = (sec >>> 16) & 255;
    u8[moov + 14] = (sec >>> 8) & 255; u8[moov + 15] = sec & 255;
    put(moov + 200, "©xyz");
    put(moov + 210, "+42.1569-123.5579+486.714/");                  // ISO-6709, as an iPhone writes it
    return u8;
  };
  const head = parseMediaMeta(mkMov(6000000, false).buffer, true);
  approx(head && head.lat, 42.1569, 1e-4, "QuickTime: faststart (moov at head) still parses");
  const tail = parseMediaMeta(mkMov(27000000, true).buffer, true);
  if (!tail) { fails++; console.error("  FAIL QuickTime: moov at END of a 27 MB file returned null"); }
  else {
    approx(tail.lat, 42.1569, 1e-4, "QuickTime GPS lat from moov at END of file");
    approx(tail.lon, -123.5579, 1e-4, "QuickTime GPS lon from moov at END of file");
    approx(tail.alt, 486.7, 0.05, "QuickTime altitude from moov at END of file");
    approx(tail.timeMs, 1700000000 * 1000, 1000, "QuickTime creation time from moov at END of file");
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

    // COAST LIMIT — the object is LOST (blank frames it cannot match) and never
    // comes back. A miss returns the extrapolated direction as the new state,
    // so velocity survives; without a cap the track keeps flying in a straight
    // line for the rest of the clip. Field case: a 5× zoom lost the object at
    // 11 s and the "track" then swept 358° of azimuth and dived 28° of
    // elevation — every sample fabricated, q=0, and the reported angular rate
    // measured off pure invention. It must COAST briefly, then FREEZE.
    {
      const blank = () => { const d = new Uint8ClampedArray(TW * TH * 4); for (let i = 0; i < TW * TH; i++) { d[i * 4] = d[i * 4 + 1] = d[i * 4 + 2] = 8; d[i * 4 + 3] = 255; } return d; };
      const lostPose = { az: 250.6, el: 12.2, roll: 0.3, fov: 60, k: 0 };
      /* seed with a real velocity so there IS something to run away with */
      let sl = { tx: TW / 2, ty: TH / 2, g: dirFromAzEl(249, 15.5), gPrev: dirFromAzEl(248.1, 15.15) };
      const seen = [];
      for (let i = 0; i < 30; i++) {
        const o = stepObject(blank(), blank(), TW, TH, sl, lostPose, { natW, natH, patch: 11, search: 18 });
        sl = { tx: o.tx, ty: o.ty, g: o.g, gPrev: o.gPrev, miss: o.miss };
        seen.push(dirToAzEl(o.g));
      }
      const start = dirToAzEl(dirFromAzEl(249, 15.5));
      const drift = Math.max(...seen.map((a) => Math.abs(((a.az - start.az + 540) % 360) - 180)));
      approx(seen.every((a) => !a.ok), true === true, 0, "stepObject: a lost object yields misses");
      approx(drift < 6 ? 1 : 0, 1, 0, `stepObject: a LOST object coasts then freezes (drifted ${drift.toFixed(1)}°, uncapped it ran 28.4°)`);
      /* and it must really be frozen, not creeping: the last ten samples equal */
      const tail = seen.slice(-10);
      const spread = Math.max(...tail.map((a) => Math.abs(a.az - tail[0].az))) + Math.max(...tail.map((a) => Math.abs(a.el - tail[0].el)));
      approx(spread, 0, 1e-9, "stepObject: the frozen tail does not creep");
    }

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

    // SCENE CUT (field case: a compilation clip hard-cut from a dusk city
    // segment to unrelated daytime sky at 23.6 s — the walk solved the splice
    // as camera motion and every later pose was fiction). stepTracker must
    // flag a textured-but-uncorrelated next frame and a palette flip, while
    // featureless and ordinary same-scene frames stay plain holds/solves.
    {
      /* scene B is genuinely DIFFERENT content (sharp hash-noise) — two
         smooth sinusoid fields are too self-similar to separate (measured:
         15/36 templates cross-match and the solve "succeeds"), and real
         splices swap content character, palette, or both. The
         smooth-vs-smooth splice is a stated limitation the chroma tell
         usually covers. */
      const texNoise = () => {
        const data = new Uint8ClampedArray(TW * TH * 4);
        for (let y = 0; y < TH; y++) for (let x = 0; x < TW; x++) {
          const s = Math.sin(x * 12.9898 + y * 78.233) * 43758.5453;
          const v = 50 + 170 * (s - Math.floor(s));
          const i = (y * TW + x) * 4;
          data[i] = data[i + 1] = data[i + 2] = v; data[i + 3] = 255;
        }
        return data;
      };
      const mkTk = () => initTracker(tframes[0], TW, TH, natW, natH, P0, { mode: "day", minMatch: 6, maxN: 40, patch: 11, search: 14 });
      const rCut = stepTracker(mkTk(), texNoise());
      approx(rCut.cut === 1 ? 1 : 0, 1, 0, "scene cut: textured-but-uncorrelated frame is flagged");
      const flat = new Uint8ClampedArray(TW * TH * 4);
      for (let i = 0; i < TW * TH; i++) { flat[i * 4] = flat[i * 4 + 1] = flat[i * 4 + 2] = 124; flat[i * 4 + 3] = 255; }
      const rFlat = stepTracker(mkTk(), flat);
      approx(!rFlat.cut && rFlat.held ? 1 : 0, 1, 0, "scene cut: a featureless same-palette frame stays a plain hold");
      /* the WB-swing lesson (field correction: "it's not a cut, it's a hard
         zoom"): a zoom into open sky re-exposes and re-white-balances — a
         full palette flip inside one continuous shot — so color must NEVER
         be cut evidence. A featureless blue frame after a dusk scene is an
         honest hold, and a same-scene frame under a strong WB/gain swing
         must still PLACE (gray NCC is invariant to per-channel affine). */
      const blue = new Uint8ClampedArray(TW * TH * 4);
      for (let i = 0; i < TW * TH; i++) { blue[i * 4] = 70; blue[i * 4 + 1] = 120; blue[i * 4 + 2] = 205; blue[i * 4 + 3] = 255; }
      const rBlue = stepTracker(mkTk(), blue);
      approx(!rBlue.cut && rBlue.held ? 1 : 0, 1, 0, "scene cut: a palette flip alone is NOT a cut — featureless blue after dusk stays a hold (WB-swing lesson)");
      const wb = new Uint8ClampedArray(tframes[1]);
      for (let i = 0; i < TW * TH; i++) {
        const v = wb[i * 4];
        wb[i * 4] = Math.min(255, v * 0.5 + 18); wb[i * 4 + 1] = Math.min(255, v * 0.55 + 26); wb[i * 4 + 2] = Math.min(255, v * 0.7 + 72);
      }
      const rWb = stepTracker(mkTk(), wb);
      approx(!rWb.cut && !rWb.held ? 1 : 0, 1, 0, "scene cut: a hard white-balance/exposure swing on the SAME scene still places the frame");
      const rSame = stepTracker(mkTk(), tframes[1]);
      approx(!rSame.cut ? 1 : 0, 1, 0, "scene cut: an ordinary same-scene step is never flagged");
    }
  }

  // 4i. SOFT SKY (clouds only — field ask: "most UFO videos have nothing to
  // reference except clouds"). A cloud edge ramps brightness over 10-20 px, so
  // its per-pixel gradient sits BELOW the strict corner gate and the detector
  // starves. The soft-sky fallback (3× downsampled detection + larger patch,
  // relaxed variance) must recover references and hold a pan on clouds alone.
  {
    // ~14 soft gaussian blobs at fixed world dirs — amplitude/σ tuned so the
    // per-pixel central-diff gradient energy stays under the strict gate (4)
    const clouds = [];
    let cq = 0;
    for (let u = -3; u <= 3; u++) for (let v = -2; v <= 2; v++) {
      cq++;
      if (cq % 2) continue; // sparse, irregular
      const ju = (((cq * 41) % 9) / 9 - 0.5) * 3, jv = (((cq * 29) % 7) / 7 - 0.5) * 3;
      clouds.push({ g: dirFromAzEl(P0.az + u * 8 + ju, P0.el + v * 8 + jv), sig: 13 + ((cq * 5) % 4) * 2, amp: 9 + ((cq * 3) % 3) * 2 });
    }
    const renderClouds = (pose) => {
      const data = new Uint8ClampedArray(TW * TH * 4);
      for (let i = 0; i < TW * TH; i++) { data[i * 4] = data[i * 4 + 1] = data[i * 4 + 2] = 140; data[i * 4 + 3] = 255; }
      for (const b of clouds) {
        const p = dirToPixK(b.g, natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k);
        if (!p) continue;
        const bx = p.px / sc, by = p.py / sc, R = Math.ceil(b.sig * 2.8);
        if (bx < -R || bx > TW + R || by < -R || by > TH + R) continue;
        for (let dy = -R; dy <= R; dy++) for (let dx = -R; dx <= R; dx++) {
          const x = Math.round(bx) + dx, y = Math.round(by) + dy;
          if (x < 0 || y < 0 || x >= TW || y >= TH) continue;
          const g = b.amp * Math.exp(-(dx * dx + dy * dy) / (2 * b.sig * b.sig)), i = (y * TW + x) * 4;
          data[i] = Math.min(255, data[i] + g); data[i + 1] = Math.min(255, data[i + 1] + g); data[i + 2] = Math.min(255, data[i + 2] + g);
        }
      }
      return data;
    };
    const c0 = renderClouds(P0);
    const strict = detectBgFeatures(c0, TW, TH, { mode: "day", maxN: 60, softMin: 0 });
    approx(strict.length < 8 ? 1 : 0, 1, 0, `soft sky: the strict corner pass STARVES on clouds (${strict.length} found)`);
    const withSoft = detectBgFeatures(c0, TW, TH, { mode: "day", maxN: 60 });
    const nSoft = withSoft.filter((f) => f.soft).length;
    approx(withSoft.length >= 16 ? 1 : 0, 1, 0, `soft sky: fallback recovers references (${withSoft.length} total)`);
    approx(nSoft >= 10 ? 1 : 0, 1, 0, `soft sky: soft-flagged cloud features present (${nSoft})`);
    // byte-identical guarantee: a frame with real corners must never take the
    // soft path — the star-grid frame from block 3 has plenty of hard features
    {
      const hard = renderFrame(P0);
      const a = detectBgFeatures(hard, TW, TH, { mode: "auto", maxN: 60 });
      const b = detectBgFeatures(hard, TW, TH, { mode: "auto", maxN: 60, softMin: 0 });
      approx(a.length, b.length, 0, "soft sky: a well-featured frame is untouched by the fallback");
      approx(a.some((f) => f.soft) ? 1 : 0, 0, 0, "soft sky: no soft features on a well-featured frame");
    }
    // the walk: a 10-frame pan with slight el/roll drift, clouds the ONLY
    // reference. Soft features localize coarser than hard corners (broad NCC
    // peak + 3×-grid placement), so the bar is looser than the star-grid walk —
    // but it must hold the pan instead of losing the world.
    {
      const wposes = Array.from({ length: 10 }, (_, i) => ({ az: 250 + i * 0.3, el: 12 + i * 0.06, roll: i * 0.08, fov: 60, k: 0 }));
      const wframes = wposes.map(renderClouds);
      const tk = initTracker(wframes[0], TW, TH, natW, natH, P0, { mode: "auto", minMatch: 6, maxN: 40, patch: 11, search: 14 });
      approx(tk.features.length >= 16 ? 1 : 0, 1, 0, `soft sky: tracker seeded on clouds (${tk.features.length} refs)`);
      let maxAzE = 0, maxElE = 0, minInl = 999;
      for (let i = 1; i < wframes.length; i++) {
        const r = stepTracker(tk, wframes[i]);
        maxAzE = Math.max(maxAzE, Math.abs(r.pose.az - wposes[i].az));
        maxElE = Math.max(maxElE, Math.abs(r.pose.el - wposes[i].el));
        minInl = Math.min(minInl, r.nInliers || 0);
      }
      approx(maxAzE < 0.6 ? 1 : 0, 1, 0, `soft sky: pan held on clouds alone (az err ${maxAzE.toFixed(2)}° < 0.6°)`);
      approx(maxElE < 0.6 ? 1 : 0, 1, 0, `soft sky: el held (${maxElE.toFixed(2)}° < 0.6°)`);
      approx(minInl >= 8 ? 1 : 0, 1, 0, `soft sky: healthy inlier count throughout (min ${minInl})`);
    }

    // 4j. CHAIN REGISTER — the Germany-clip failure: a sustained fast tilt on a
    // cloud-only sky, panning OFF the reference frame's coverage. The primary
    // global register dies (no overlap), and the sparse layer alone LATCHES:
    // every soft patch finds a lookalike near its stale prediction (per-step
    // motion exceeds the search window), so the solve confirms near-zero
    // motion with a confident inlier count while the camera sweeps tens of
    // degrees (field-measured: 42° frozen vs a tilt to ~62°). The chain
    // register (whole-frame vs the PREVIOUS frame) is the motion floor: it
    // seeds the predictions so the sparse layer matches truth again.
    {
      // taller cloud field so the whole pan stays populated
      const cl2 = [];
      let cq2 = 0;
      for (let u = -3; u <= 3; u++) for (let v = -2; v <= 9; v++) {
        cq2++;
        if (cq2 % 2) continue;
        const ju = (((cq2 * 41) % 9) / 9 - 0.5) * 3, jv = (((cq2 * 29) % 7) / 7 - 0.5) * 3;
        cl2.push({ g: dirFromAzEl(P0.az + u * 8 + ju, P0.el + v * 8 + jv), sig: 13 + ((cq2 * 5) % 4) * 2, amp: 9 + ((cq2 * 3) % 3) * 2 });
      }
      const renderC2 = (pose) => {
        const data = new Uint8ClampedArray(TW * TH * 4);
        for (let i = 0; i < TW * TH; i++) { data[i * 4] = data[i * 4 + 1] = data[i * 4 + 2] = 140; data[i * 4 + 3] = 255; }
        for (const b of cl2) {
          const p = dirToPixK(b.g, natW, natH, pose.az, pose.el, pose.roll, pose.fov, pose.k);
          if (!p) continue;
          const bx = p.px / sc, by = p.py / sc, R = Math.ceil(b.sig * 2.8);
          if (bx < -R || bx > TW + R || by < -R || by > TH + R) continue;
          for (let dy = -R; dy <= R; dy++) for (let dx = -R; dx <= R; dx++) {
            const x = Math.round(bx) + dx, y = Math.round(by) + dy;
            if (x < 0 || y < 0 || x >= TW || y >= TH) continue;
            const g = b.amp * Math.exp(-(dx * dx + dy * dy) / (2 * b.sig * b.sig)), i = (y * TW + x) * 4;
            data[i] = Math.min(255, data[i] + g); data[i + 1] = Math.min(255, data[i + 1] + g); data[i + 2] = Math.min(255, data[i + 2] + g);
          }
        }
        return data;
      };
      // 2.4°/step tilt (beyond the sparse search window) climbing 43° — well
      // off the reference's coverage by the end
      const tposes = Array.from({ length: 19 }, (_, i) => ({ az: 250 + i * 0.1, el: 12 + i * 2.4, roll: 0, fov: 60, k: 0 }));
      const tframes = tposes.map(renderC2);
      const runTilt = (chainOn) => {
        const tk = initTracker(tframes[0], TW, TH, natW, natH, P0, { mode: "day", minMatch: 6, maxN: 40, patch: 11, search: 14, ...(chainOn ? {} : { chain: false }) });
        let elEnd = P0.el, maxElErr = 0, chained = 0, heldN = 0;
        for (let i = 1; i < tframes.length; i++) {
          const r = stepTracker(tk, tframes[i]);
          elEnd = r.pose.el;
          maxElErr = Math.max(maxElErr, Math.abs(r.pose.el - tposes[i].el));
          if (r.chained != null) chained++;
          if (r.held) heldN++;
        }
        return { elEnd, maxElErr, chained, heldN };
      };
      const off = runTilt(false), on = runTilt(true);
      approx(off.maxElErr > 8 ? 1 : 0, 1, 0, `chain register: WITHOUT it the tilt is lost (el err ${off.maxElErr.toFixed(1)}° — the latch/freeze)`);
      approx(on.maxElErr < 1.5 ? 1 : 0, 1, 0, `chain register: tilt tracked to the top (el err ${on.maxElErr.toFixed(2)}° < 1.5°)`);
      approx(on.chained >= 5 ? 1 : 0, 1, 0, `chain register: engaged once off the reference (${on.chained} chained steps)`);
      approx(on.heldN, 0, 0, "chain register: no frozen frames on the sweep");
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

    /* ROBUST RATE (field case: a balloon-smooth object reported 9°/s peaks
       and hundreds-of-m/s ladder speeds). Three failure modes, each asserted:
       tracker noise concentrated in a pan/zoom window must NOT become a rate
       peak; a real SUSTAINED maneuver must survive the smoothing; and two
       size keyframes a fraction of a second apart (a sizing repeat) must not
       read as radial speed. */
    {
      let seed = 424242;
      const rnd = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff - 0.5; };
      // A: balloon drift 1.5°/s, noise σ0.03° — except σ0.35° + a lone 0.9°
      // excursion inside the camera's pan+zoom window t∈[5,8]
      const mkA = () => Array.from({ length: 85 }, (_, i) => {
        const tt = i * 0.25;
        const zoomy = tt >= 5 && tt <= 8;
        const n = (zoomy ? 0.35 : 0.03) * rnd() * 2 + (Math.abs(tt - 6) < 0.13 ? 0.9 : 0);
        return { t: +tt.toFixed(3), az: (350 + 1.5 * tt) % 360, el: 30 + n, q: 0.9 };
      });
      const ppZoom = [
        { t: 0, az: 350, el: 30, fov: 46 }, { t: 5, az: 350, el: 30, fov: 46 },
        { t: 6.5, az: 20, el: 32, fov: 20 }, { t: 8, az: 20, el: 32, fov: 10 }, { t: 21, az: 20, el: 32, fov: 10 },
      ];
      const vkA = videoKinematics({ objPath: mkA(), posePath: ppZoom });
      approx(vkA.peakOmega < 2.8 ? 1 : 0, 1, 0, `videoKin robust: zoom-window noise + a 0.9° excursion does NOT spike the peak (${vkA.peakOmega.toFixed(2)}°/s vs truth 1.5)`);
      const trueA = 1.5 * Math.cos(30 * Math.PI / 180);   // az rate × cos(el) = true sky rate
      approx(vkA.avgOmega, trueA, 0.2, "videoKin robust: average rate stays ≈ truth through the noise");
      approx(Math.abs(vkA.sweep - trueA * 21) / (trueA * 21) < 0.12 ? 1 : 0, 1, 0, "videoKin robust: sweep no longer random-walks upward with jitter");
      approx(vkA.noiseOmega > 0.01 ? 1 : 0, 1, 0, "videoKin robust: a noise floor is estimated and reported");
      // B: real sustained maneuver — 0.5°/s then 4°/s after t=10 (flat pose)
      const mkB = () => Array.from({ length: 85 }, (_, i) => {
        const tt = i * 0.25;
        const az = tt < 10 ? 350 + 0.5 * tt : 350 + 5 + 4 * (tt - 10);
        return { t: +tt.toFixed(3), az: az % 360, el: 30 + 0.05 * rnd() * 2, q: 0.9 };
      });
      const vkB = videoKinematics({ objPath: mkB() });
      const trueB = 4 * Math.cos(30 * Math.PI / 180);      // 3.46°/s of sky at el 30
      approx(vkB.peakOmega > trueB * 0.92 ? 1 : 0, 1, 0, `videoKin robust: a sustained maneuver survives the smoothing (peak ${vkB.peakOmega.toFixed(2)} vs truth ${trueB.toFixed(2)}°/s)`);
      const at15 = vkB.rate.reduce((b, r) => (Math.abs(r.t - 15) < Math.abs(b.t - 15) ? r : b), vkB.rate[0]);
      approx(at15.omega, trueB, 0.25, "videoKin robust: the fitted rate reaches the true post-maneuver value");
      // C: the field file's sizing repeat — 0.957° and 0.714° just 0.11 s apart
      const mkC = () => Array.from({ length: 85 }, (_, i) => ({ t: +(i * 0.25).toFixed(3), az: (350 + 1 * i * 0.25) % 360, el: 30, q: 0.9 }));
      const trkC = [{ t: 2, ang: 1.0 }, { t: 6.64, ang: 0.957 }, { t: 6.75, ang: 0.714 }, { t: 18, ang: 0.8 }];
      const dC = videoKinematics({ objPath: mkC(), track: trkC }).atDistance(120);
      approx(dC.peakSpeed < 15 ? 1 : 0, 1, 0, `videoKin robust: a 0.11 s sizing repeat no longer reads as radial speed (peak ${dC.peakSpeed.toFixed(1)} m/s, was ~300)`);
      /* D — ROTATION-STARVED ZOOM (the Germany field case): the object glides
         at 1°/s, but during a 3× zoom the solver held the camera's pointing
         on 6-8 anchors while the operator tilted to re-center — a phantom
         sustained ~4.4°/s ramp rides the track for ~1 s. Smoothing can't
         remove a sustained bias; the reliability mask must exclude it from
         the REPORTED peak while returning the excursion + its span honestly. */
      const mkD = () => Array.from({ length: 85 }, (_, i) => {
        const tt = i * 0.25;
        const phantom = tt >= 15 && tt <= 16.25 ? -4.4 * Math.min(tt - 15, 1.25) : (tt > 16.25 ? -5.5 : 0);
        return { t: +tt.toFixed(3), az: (350 + 1 * tt) % 360, el: 40 + phantom, q: 0.9 };
      });
      const ppD = Array.from({ length: 85 }, (_, i) => {
        const tt = i * 0.25;
        const fov = tt < 15 ? 16.6 : tt < 16 ? 16.6 - (16.6 - 5) * (tt - 15) : 5;
        const n = tt >= 15 && tt <= 16.1 ? 7 : 25;
        return { t: +tt.toFixed(3), az: 356.7, el: 41.3, roll: 0, fov, k: 0, n };
      });
      const vkD = videoKinematics({ objPath: mkD(), posePath: ppD });
      approx(vkD.zoomSpans.length >= 1 ? 1 : 0, 1, 0, "videoKin mask: the rotation-starved zoom span is detected");
      approx(vkD.peakOmegaAll > 3 ? 1 : 0, 1, 0, `videoKin mask: the phantom excursion is still visible in peakOmegaAll (${vkD.peakOmegaAll.toFixed(1)}°/s)`);
      approx(vkD.peakOmega < 1.6 ? 1 : 0, 1, 0, `videoKin mask: the REPORTED peak excludes the zoom span (${vkD.peakOmega.toFixed(2)}°/s ≈ the 1°/s glide)`);
      approx(videoKinematics({ objPath: mkD(), posePath: ppD }).atDistance(120).peakSpeed < 8 ? 1 : 0, 1, 0, "videoKin mask: implied peak speed also refuses the zoom-span excursion");
      // a REAL maneuver away from any zoom must still be reported at value
      const vkB2 = videoKinematics({ objPath: mkB(), posePath: ppD.map((p) => ({ ...p, fov: 16, n: 25 })) });
      approx(vkB2.peakOmega > 4 * Math.cos(30 * Math.PI / 180) * 0.9 ? 1 : 0, 1, 0, "videoKin mask: a sustained maneuver on a clean solve keeps its true peak");
      /* trackQuality — the camera-motion risk RATING the UI hint renders */
      const tqD = trackQuality({ objPath: mkD(), posePath: ppD });
      approx(["good", "fair", "poor"].includes(tqD.grade) ? 1 : 0, 1, 0, `trackQuality: a zoom-contaminated clip is rated below excellent (${tqD.grade})`);
      approx(tqD.reasons.length >= 1 ? 1 : 0, 1, 0, "trackQuality: the rating names its reasons");
      const tqClean = trackQuality({ objPath: mkB(), posePath: ppD.map((p) => ({ ...p, fov: 16, n: 25 })) });
      approx(tqClean.grade === "excellent" ? 1 : 0, 1, 0, `trackQuality: a steady-camera clean solve rates excellent (${tqClean.grade})`);
      const tqNoisy = trackQuality({ objPath: Array.from({ length: 85 }, (_, i) => ({ t: i * 0.25, az: 350 + 0.05 * rnd() * 6, el: 30 + 0.05 * rnd() * 6, q: 0.9 })) });
      approx(tqNoisy.grade === "fair" || tqNoisy.grade === "poor" ? 1 : 0, 1, 0, `trackQuality: a near-hover where noise rivals motion is flagged (${tqNoisy.grade})`);
      /* CAMERA-WORK rating (the Germany case): a smooth glide tracked through a
         camera that swept ~30°, zoomed 4× and ran half its frames on the
         frame-to-frame lock must be flagged camHeavy with the g caveat — the
         object's own motion being gentle is exactly why the caveat matters */
      const ppCam = Array.from({ length: 85 }, (_, i) => {
        const tt = i * 0.25;
        return { t: +tt.toFixed(3), az: 356, el: 30 + tt * 1.5, roll: 0, fov: 41.6 - tt * 1.5, k: 0, n: 20, ...(tt > 9 ? { c: 1 } : {}) };
      });
      const glide = Array.from({ length: 85 }, (_, i) => ({ t: i * 0.25, az: 356, el: 33 + i * 0.25 * 1.4, q: 0.9 }));
      const tqCam = trackQuality({ objPath: glide, posePath: ppCam });
      approx(tqCam.camHeavy ? 1 : 0, 1, 0, "trackQuality: heavy camera work (sweep + zoom + chain) is detected");
      approx(tqCam.grade === "fair" || tqCam.grade === "poor" ? 1 : 0, 1, 0, `trackQuality: camHeavy caps the grade (${tqCam.grade})`);
      approx(tqCam.reasons.some((r) => r.includes("upper bound")) ? 1 : 0, 1, 0, "trackQuality: the g upper-bound caveat is in the reasons");
      /* camReason is the SAME element (identity) — the results panel filters it
         out of the banner when the at-figure caveat renders below */
      approx(tqCam.camReason && tqCam.reasons.includes(tqCam.camReason) ? 1 : 0, 1, 0, "trackQuality: camReason rides in reasons by identity (banner dedupe)");
      approx(tqCam.chainPct > 0.4 ? 1 : 0, 1, 0, `trackQuality: chain fraction measured (${(tqCam.chainPct * 100).toFixed(0)}%)`);
      /* and a steady tripod solve must NOT trip the camera-work flag */
      approx(tqClean.camHeavy ? 1 : 0, 0, 0, "trackQuality: a steady camera is not flagged camHeavy");
    }
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

    /* pinApparentCenter: the track-point ghost anchors by APPARENT centre
       (bbox of the drawn curves) — the silhouette-chord midpoint sits
       off-centre for asymmetric shapes (a balloon's string hangs below its
       envelope), so midpoint pinning let the outline slide off the point as
       it scaled. Assert: the pinned bbox centre lands exactly on the target,
       stays there when the size doubles (scaling is ABOUT the point), and a
       symmetric orb needs no correction. */
    {
      const bbox = (sfx) => {
        const P = shapeProjNat(sfx).curves.flat();
        const xs = P.map((p) => p.x), ys = P.map((p) => p.y);
        return { cx: (Math.min(...xs) + Math.max(...xs)) / 2, cy: (Math.min(...ys) + Math.max(...ys)) / 2, w: Math.max(...xs) - Math.min(...xs) };
      };
      const bal = { kind: "balloon", cx: 500, cy: 400, sizeNat: 120, rotM: SHAPE_R0().balloon, cord: 1.6 };
      approx(Math.abs(bbox(bal).cy - 400) > 1 ? 1 : 0, 1, 0, "pinApparentCenter: origin-anchored balloon centre really sits off the origin (the bug's precondition)");
      const b1 = bbox(pinApparentCenter(bal, 500, 400));
      approx(b1.cx, 500, 1e-9, "pinApparentCenter: apparent centre lands on the point (x)");
      approx(b1.cy, 400, 1e-9, "pinApparentCenter: apparent centre lands on the point (y)");
      const b2 = bbox(pinApparentCenter({ ...bal, sizeNat: 240 }, 500, 400));
      approx(b2.cx, 500, 1e-9, "pinApparentCenter: centre stays on the point when the size doubles (x)");
      approx(b2.cy, 400, 1e-9, "pinApparentCenter: centre stays on the point when the size doubles (y)");
      approx(b2.w / b1.w, 2, 1e-6, "pinApparentCenter: doubling sizeNat doubles the extent about the pinned centre");
      const po = pinApparentCenter({ kind: "orb", cx: 100, cy: 100, sizeNat: 50, rotM: I3 }, 100, 100);
      approx(po.cx, 100, 1e-9, "pinApparentCenter: a symmetric orb needs no correction (cx)");
      approx(po.cy, 100, 1e-9, "pinApparentCenter: a symmetric orb needs no correction (cy)");
    }

    /* normSizedTrack: size keyframes are wpx AT THEIR OWN FRAME'S ZOOM — a
       real field clip zoomed 46°→5° (9.25×), and interpolating raw px (or
       projecting one frame's px through another frame's lens) ballooned the
       world-view wireframe ~8× at zoomed keyframes. Normalizing through
       eq = wpx·tan(fovOwn/2)/tan(fovTarget/2) makes equal ANGULAR sizes come
       out as equal px at the target scale. */
    {
      const D2Rl = Math.PI / 180;
      const fpx = (fov, natW = 720) => (natW / 2) / Math.tan((fov * D2Rl) / 2);
      const wpxOf = (angDeg, fov) => 2 * fpx(fov) * Math.tan((angDeg * D2Rl) / 2);
      const pp = [{ t: 0, fov: 41.6 }, { t: 5, fov: 20 }, { t: 10, fov: 5 }];
      // two keyframes, SAME 0.8° angular size, captured at 41.6° and 5° zoom
      const trk = [
        { t: 0, x: 1, y: 1, wpx: wpxOf(0.8, 41.6) },
        { t: 10, x: 1, y: 1, wpx: wpxOf(0.8, 5) },
      ];
      approx(trk[1].wpx / trk[0].wpx, Math.tan(20.8 * D2Rl) / Math.tan(2.5 * D2Rl), 1e-9,
        "normSizedTrack precondition: equal angles are ~8.7× different raw px across the zoom");
      const n = normSizedTrack(trk, pp, 41.6);
      approx(n[0].wpx, trk[0].wpx, 1e-9, "normSizedTrack: a keyframe already at the target scale is untouched");
      approx(n[1].wpx, trk[0].wpx, 1e-6, "normSizedTrack: equal angular sizes normalize to equal px (9× zoom collapsed)");
      // real-clip regression: 51.4 px at fov 5 ≈ 0.357° ⇒ ~5.93 px at fov 41.6
      const g = normSizedTrack([{ t: 10, wpx: 51.4 }], pp, 41.6)[0].wpx;
      approx(2 * Math.atan((g / 2) / fpx(41.6)) / D2Rl, 0.357, 0.002, "normSizedTrack: Germany-clip keyframe keeps its true angular size at the target scale");
      // identity paths: flat fov, no posePath, unsized points
      approx(normSizedTrack(trk, [{ t: 0, fov: 41.6 }, { t: 10, fov: 41.6 }], 41.6) === trk ? 1 : 0, 1, 0, "normSizedTrack: flat-FOV clip returns the track unchanged (same reference)");
      approx(normSizedTrack(trk, null, 41.6) === trk ? 1 : 0, 1, 0, "normSizedTrack: no posePath ⇒ unchanged");
      approx(normSizedTrack([{ t: 3, x: 1, y: 1 }], pp, 41.6)[0].wpx === undefined ? 1 : 0, 1, 0, "normSizedTrack: unsized points pass through untouched");
      // interpolated fov between samples: t=2.5 sits halfway 41.6→20
      const mid = normSizedTrack([{ t: 2.5, wpx: 100 }], pp, 41.6)[0].wpx;
      approx(mid, 100 * Math.tan(((41.6 + 20) / 2) * D2Rl / 2) / Math.tan(20.8 * D2Rl), 1e-9, "normSizedTrack: FOV interpolates linearly between pose samples");

      /* trackAngAt + the range math: the UI stamps `ang` through the BASE
         FOV, refreshed only on placement commit — the Germany field file
         carried one stale stamp (50.3 px on a ~15°-FOV frame stored as
         3.04°, true ≈1.05°) that inflated the report's range ratio ~3×. The
         math core now re-derives ang from wpx through the solved per-frame
         FOV at the mouth of every consumer. */
      const src5 = {
        natW: 720, fovH: 41.6,
        posePath: [{ t: 0, az: 0, el: 30, fov: 41.6 }, { t: 10, az: 0, el: 30, fov: 15 }],
        track: [
          { t: 0, x: 1, y: 1, wpx: wpxOf(0.8, 41.6), ang: 0.8 },        // fresh stamp, base frame
          { t: 10, x: 1, y: 1, wpx: 50.3, ang: 3.04123 },               // STALE: stamped at base FOV
        ],
      };
      const a10 = trackAngAt(src5, src5.track[1]);
      approx(a10, 2 * Math.atan((50.3 / 2) / fpx(15)) / D2Rl, 1e-9, "trackAngAt: stale stamp re-derived through the frame's solved FOV");
      approx(a10 < 1.2 && a10 > 1.0 ? 1 : 0, 1, 0, `trackAngAt: Germany-style 3.04° stamp corrects to ~1.1° (${a10.toFixed(3)})`);
      approx(trackAngAt({ natW: 720, fovH: 41.6 }, { t: 1, wpx: 10, ang: 0.5 }), 0.5, 1e-9, "trackAngAt: no posePath ⇒ the stored stamp stands");
      // flows through videoKinematics: rangeRatio uses corrected angs
      const op5 = Array.from({ length: 21 }, (_, i) => ({ t: i * 0.5, az: 0, el: 30 + i * 0.1, q: 0.9 }));
      const vk5 = videoKinematics({ ...src5, objPath: op5 });
      approx(vk5.rangeRatio, Math.tan((a10 * D2Rl) / 2) / Math.tan((0.8 * D2Rl) / 2), 1e-6, "videoKinematics: range ratio built from re-derived angs, not stale stamps (1.32×, not the stale 3.8×)");
    }

    /* CUBE ↔ DIAMOND (squash). The solid must actually BE a cube at 0 and a
       square bipyramid at 1, not merely look like one: check the corner set,
       the extents, and that the cap tapers monotonically in between. */
    const pts = (q) => shapeWire("cube", null, { squash: q }).flat();
    const ext = (P, i) => Math.max(...P.map((p) => p[i])) - Math.min(...P.map((p) => p[i]));
    const c0 = pts(0);
    approx(Math.abs(ext(c0, 0) - 1) < 1e-9 && Math.abs(ext(c0, 1) - 1) < 1e-9 && Math.abs(ext(c0, 2) - 1) < 1e-9 ? 1 : 0, 1, 0,
      "cube: squash 0 is a unit cube (1×1×1 extents)");
    // exactly 8 distinct corners, all at |x|=|y|=|z|=0.5
    const uniq = (P) => [...new Set(P.map((p) => p.map((v) => v.toFixed(4)).join(",")))];
    const corners0 = uniq(c0.filter((p) => Math.abs(Math.abs(p[2]) - 0.5) < 1e-9));
    approx(corners0.length, 8, 0, "cube: squash 0 has exactly 8 corners");
    approx(c0.every((p) => Math.abs(Math.abs(p[0]) - 0.5) < 1e-9 && Math.abs(Math.abs(p[1]) - 0.5) < 1e-9) ? 1 : 0, 1, 0,
      "cube: squash 0 has no waist ring (every point is on a cube corner column)");
    const c1 = pts(1);
    approx(Math.abs(ext(c1, 2) - 1) < 1e-9 ? 1 : 0, 1, 0, "diamond: squash 1 keeps the full height");
    // the caps have collapsed onto the axis → the only points at |z|=0.5 are the apexes
    const apex = uniq(c1.filter((p) => Math.abs(Math.abs(p[2]) - 0.5) < 1e-9));
    approx(apex.length, 2, 0, "diamond: squash 1 collapses the top and bottom faces to single apexes");
    approx(c1.filter((p) => Math.abs(p[2]) < 1e-9).length > 0 && uniq(c1.filter((p) => Math.abs(p[2]) < 1e-9)).length === 4 ? 1 : 0, 1, 0,
      "diamond: squash 1 has a 4-corner waist (square bipyramid = the diamond silhouette)");
    // monotone taper: the cap half-width shrinks as squash grows, and the waist never moves
    const capW = (q) => { const P = pts(q).filter((p) => Math.abs(p[2] - 0.5) < 1e-9); return Math.max(...P.map((p) => Math.abs(p[0]))); };
    const caps = [0, 0.25, 0.5, 0.75, 1].map(capW);
    approx(caps.every((v, i) => i === 0 || v < caps[i - 1] + 1e-9) && Math.abs(caps[0] - 0.5) < 1e-9 && caps[4] < 1e-9 ? 1 : 0, 1, 0,
      `cube→diamond: cap tapers monotonically 0.5→0 (${caps.map((v) => v.toFixed(3)).join(" ")})`);
    [0.25, 0.5, 0.75].forEach((q) => {
      const P = pts(q);
      approx(Math.abs(ext(P, 0) - 1) < 1e-9 && Math.abs(ext(P, 2) - 1) < 1e-9 ? 1 : 0, 1, 0, `cube→diamond: squash ${q} keeps the waist and height (extents 1×1)`);
    });
    // out-of-range and missing values must not produce a degenerate solid
    [undefined, NaN, -3, 7].forEach((q) => {
      const P = shapeWire("cube", null, q === undefined ? undefined : { squash: q }).flat();
      approx(P.length > 0 && Math.abs(ext(P, 2) - 1) < 1e-9 ? 1 : 0, 1, 0, `cube: squash ${String(q)} clamps to a valid solid`);
    });
    // registered everywhere a shape has to be
    approx(SHAPES.some((s) => s.k === "cube") ? 1 : 0, 1, 0, "cube: listed in the shape picker");
    approx(SHAPE_R0().cube && SHAPE_R0().cube.length === 9 ? 1 : 0, 1, 0, "cube: has a default 3/4 pose");

    /* STRETCH — height against a FIXED footprint, independent of squash so
       every combination is reachable: box, column, slab, tall gem, flat
       lozenge. */
    const extOf = (P, i) => Math.max(...P.map((p) => p[i])) - Math.min(...P.map((p) => p[i]));
    const cube = (o) => shapeWire("cube", null, o).flat();
    for (const [q, st, wantZ, what] of [
      [0, 1, 1, "cube"], [0, 2.4, 2.4, "tall box"], [0, 0.4, 0.4, "slab"],
      [1, 1, 1, "diamond"], [1, 2.5, 2.5, "tall diamond"], [1, 0.35, 0.35, "flat lozenge"],
      [0.5, 1.8, 1.8, "stretched gem"],
    ]) {
      const P = cube({ squash: q, stretch: st });
      approx(extOf(P, 2), wantZ, 1e-9, `cube stretch: ${what} is ${wantZ}× tall`);
      approx(extOf(P, 0), 1, 1e-9, `cube stretch: ${what} keeps its footprint`);
    }
    approx(extOf(cube({ stretch: 99 }), 2), 3, 1e-9, "cube stretch: clamped at 3×");
    approx(extOf(cube({ stretch: -5 }), 2), 0.25, 1e-9, "cube stretch: clamped at 0.25×");
    /* the squash corner cases must survive a stretch: caps still collapse */
    const uq = (P) => [...new Set(P.map((p) => p.map((v) => v.toFixed(4)).join(",")))];
    approx(uq(cube({ squash: 1, stretch: 2 }).filter((p) => Math.abs(Math.abs(p[2]) - 1) < 1e-9)).length, 2, 0,
      "cube stretch: a stretched diamond still has exactly 2 apexes");

    /* DEPTH — the footprint's second axis (y) against the fixed x width: the
       third independent proportion, added for the MONOLITH. It must scale y
       alone, compose with stretch and squash, and hit the classic 1:4:9. */
    const thin = cube({ depth: 0.25 });
    approx(extOf(thin, 1), 0.25, 1e-9, "cube depth: 0.25 thins the footprint's y to 0.25");
    approx(extOf(thin, 0), 1, 1e-9, "cube depth: the x width stays the fixed reference (1)");
    approx(extOf(thin, 2), 1, 1e-9, "cube depth: the height is untouched");
    const mono = cube({ depth: 0.25, stretch: 2.25 });
    approx(extOf(mono, 1), 0.25, 1e-9, "monolith: thickness 0.25 …");
    approx(extOf(mono, 0), 1, 1e-9, "monolith: … width 1 …");
    approx(extOf(mono, 2), 2.25, 1e-9, "monolith: … height 2.25 — the classic 1:4:9 slab");
    approx(extOf(cube({ depth: 99 }), 1), 3, 1e-9, "cube depth: clamped at 3×");
    approx(extOf(cube({ depth: -1 }), 1), 0.1, 1e-9, "cube depth: clamped at 0.1×");
    approx(extOf(cube({}), 1), 1, 1e-9, "cube depth: default 1 leaves the square footprint (old fits unchanged)");
    const thinDia = cube({ squash: 1, depth: 0.5 });
    approx(extOf(thinDia, 1), 0.5, 1e-9, "cube depth: composes with squash (thin diamond waist y = 0.5)");
    approx(uq(thinDia.filter((p) => Math.abs(Math.abs(p[2]) - 0.5) < 1e-9)).length, 2, 0,
      "cube depth: a thin diamond still collapses to exactly 2 apexes");

    /* PYRAMID — square base of fixed width, apex height = stretch */
    const pyr = (o) => shapeWire("pyr", null, o).flat();
    for (const [st, what] of [[1, "as tall as it is wide"], [2.2, "spire"], [0.3, "shallow cap"]]) {
      const P = pyr({ stretch: st });
      approx(extOf(P, 2), st, 1e-9, `pyramid: ${what} is ${st}× tall`);
      approx(extOf(P, 0), 1, 1e-9, `pyramid: ${what} keeps a unit base`);
      /* exactly one apex, on the axis, at the top */
      const top = uq(P.filter((p) => Math.abs(p[2] - st / 2) < 1e-9));
      approx(top.length, 1, 0, `pyramid: ${what} has a single apex`);
      approx(Math.hypot(...P.filter((p) => Math.abs(p[2] - st / 2) < 1e-9)[0].slice(0, 2)), 0, 1e-9,
        `pyramid: ${what} apex sits on the axis`);
      /* 4 base corners */
      approx(uq(P.filter((p) => Math.abs(p[2] + st / 2) < 1e-9)).length, 4, 0, `pyramid: ${what} has a 4-corner base`);
    }
    approx(SHAPES.some((x) => x.k === "pyr") ? 1 : 0, 1, 0, "pyramid: listed in the shape picker");
    approx(SHAPE_R0().pyr && SHAPE_R0().pyr.length === 9 ? 1 : 0, 1, 0, "pyramid: has a default 3/4 pose");

    /* V / DELTA — span is the measured dimension so it must stay 1 whatever
       the sweep or notch does; `notch` 0 must be a genuine solid delta (the
       trailing edge dead straight between the tips) and 1 two thin arms. */
    const vee = (o) => shapeWire("vee", null, o).flat();
    for (const [sw, what] of [[0.45, "default"], [0.12, "shallow arrowhead"], [0.9, "deep chevron"]]) {
      const P = vee({ sweep: sw });
      approx(extOf(P, 1), 1, 1e-9, `V: ${what} keeps a unit tip-to-tip span`);
      approx(extOf(P, 0), sw, 1e-9, `V: ${what} is ${sw} deep fore-aft`);
    }
    {
      /* the notch vertex is on the centreline; at notch 0 it sits exactly on
         the straight tip-to-tip trailing edge (x = −sweep/2 = the tips' x),
         and it marches forward toward the apex as the notch deepens */
      const sw = 0.5, nx = (nt) => { const P = vee({ sweep: sw, notch: nt }).filter((p) => Math.abs(p[1]) < 1e-9); return Math.min(...P.map((p) => p[0])); };
      approx(nx(0), -sw / 2, 1e-9, "V: notch 0 is a solid delta (trailing edge straight between the tips)");
      const xs = [0, 0.25, 0.5, 0.75, 1].map(nx);
      approx(xs.every((v, i) => i === 0 || v > xs[i - 1] + 1e-9) ? 1 : 0, 1, 0, `V: the notch marches forward monotonically (${xs.map((v) => v.toFixed(3)).join(" ")})`);
      approx(nx(1) < sw / 2 - 1e-6 ? 1 : 0, 1, 0, "V: even a full notch stops short of the apex (the arms stay joined)");
      /* the tips are the extremes of the span, at the rear */
      const P = vee({ sweep: sw, notch: 0.7 });
      const tip = P.filter((p) => Math.abs(Math.abs(p[1]) - 0.5) < 1e-9);
      approx(tip.every((p) => Math.abs(p[0] + sw / 2) < 1e-9) ? 1 : 0, 1, 0, "V: the tips sit at the rear of the planform");
    }
    [undefined, NaN, -3, 7].forEach((v) => {
      const P = vee(v === undefined ? undefined : { sweep: v, notch: v });
      approx(P.length > 0 && Math.abs(extOf(P, 1) - 1) < 1e-9 ? 1 : 0, 1, 0, `V: sweep/notch ${String(v)} clamps to a valid solid`);
    });
    approx(SHAPES.some((x) => x.k === "vee") ? 1 : 0, 1, 0, "V: listed in the shape picker");
    approx(SHAPE_R0().vee && SHAPE_R0().vee.length === 9 ? 1 : 0, 1, 0, "V: has a default 3/4 pose");

    /* STEALTH JET — wingspan is the reference dimension (as on every other
       aircraft here), nose forward, and it must be measurably NOT a plain
       delta: the trailing edge carries a centre notch and the tails rise
       above the wing. */
    {
      const P = shapeWire("stealth", null, null).flat();
      approx(extOf(P, 1), 1, 1e-9, "stealth: unit wingspan (the reference dimension)");
      approx(Math.max(...P.map((p) => p[0])), 0.62, 1e-9, "stealth: nose is the forward extreme");
      const tipX = P.filter((p) => Math.abs(Math.abs(p[1]) - 0.5) < 1e-9).map((p) => p[0]);
      const rearX = Math.min(...P.map((p) => p[0]));
      approx(tipX.every((x) => x > rearX + 1e-9) ? 1 : 0, 1, 0, "stealth: the tips are swept forward of the trailing edge (not a plain delta)");
      approx(Math.max(...P.map((p) => p[2])) > 0.15 ? 1 : 0, 1, 0, "stealth: canted tails rise clear of the wing");
      approx(Math.min(...P.map((p) => p[2])), -0.02, 1e-9, "stealth: flat belly at the widest line");
      approx(SHAPES.some((x) => x.k === "stealth") ? 1 : 0, 1, 0, "stealth: listed in the shape picker");
      approx(SHAPE_R0().stealth && SHAPE_R0().stealth.length === 9 ? 1 : 0, 1, 0, "stealth: has a default 3/4 pose");
    }

    /* BALLOON — envelope of revolution with the widest point in the UPPER
       third (that taper is what distinguishes it from an orb), plus a string
       whose length tracks `cord` and vanishes at 0. */
    {
      const bal = (o) => shapeWire("balloon", null, o).flat();
      const P = bal(null);
      approx(Math.abs(extOf(P, 0) - extOf(P, 1)) < 1e-9 ? 1 : 0, 1, 0, "balloon: envelope is a solid of revolution (square footprint)");
      const zTopM = Math.max(...P.map((p) => p[2]));
      /* widest ring: find the z of the point farthest from the axis */
      let wz = 0, wr = 0;
      for (const p of P) { const r = Math.hypot(p[0], p[1]); if (r > wr) { wr = r; wz = p[2]; } }
      const zEnvBot = zTopM - 0.6;
      approx((wz - zEnvBot) / 0.6 > 0.5 ? 1 : 0, 1, 0, `balloon: widest point sits in the upper half of the envelope (${(((wz - zEnvBot) / 0.6) * 100).toFixed(0)}%)`);
      /* the string: length scales with cord, and cord 0 removes it entirely */
      const drop = (c) => zTopM - Math.min(...bal({ cord: c }).map((p) => p[2]));
      const d0 = drop(0), d1 = drop(1), d2 = drop(2);
      approx(d1 - d0, 0.55, 1e-9, "balloon: string at cord 1 hangs 0.55 below the knot");
      approx(d2 - d0, 1.10, 1e-9, "balloon: string length is proportional to cord");
      approx(d0 < 0.68 ? 1 : 0, 1, 0, "balloon: cord 0 leaves a bare envelope (no string)");
      approx(Math.abs(extOf(bal(null), 2) - 1) < 0.2 ? 1 : 0, 1, 0, "balloon: the default configuration spans about one unit");
      [undefined, NaN, -3, 9].forEach((c) => {
        const Q = bal(c === undefined ? undefined : { cord: c });
        approx(Q.length > 0 && extOf(Q, 0) > 0.5 && extOf(Q, 2) > 0.5 ? 1 : 0, 1, 0, `balloon: cord ${String(c)} clamps to a valid solid`);
      });
      approx(SHAPES.some((x) => x.k === "balloon") ? 1 : 0, 1, 0, "balloon: listed in the shape picker");
      approx(SHAPE_R0().balloon && SHAPE_R0().balloon.length === 9 ? 1 : 0, 1, 0, "balloon: has a default 3/4 pose");
    }

    /* BIRD WING ANGLE — the wing swings about the shoulder instead of sliding
       fore/aft (which is not a thing a wing does). The root must stay welded
       to the body, the span must stay owned by `wing` alone, and an old
       `wingX` fit must carry over rather than snapping back to neutral. */
    {
      const bird = (o) => shapeWire("bird", null, o).flat();
      const tipX = (o) => { const P = bird(o); const m = Math.max(...P.map((p) => Math.abs(p[1]))); return P.find((p) => Math.abs(Math.abs(p[1]) - m) < 1e-9)[0]; };
      const xs = [-25, -12, 0, 20, 40, 55].map((a) => tipX({ wingA: a }));
      approx(xs.every((v, i) => i === 0 || v < xs[i - 1] - 1e-6) ? 1 : 0, 1, 0,
        `bird: the tip swings monotonically aft as the angle rises (${xs.map((v) => v.toFixed(3)).join(" ")})`);
      approx(xs[2], -0.05, 1e-9, "bird: 0° leaves the wing exactly where it was");
      /* the span belongs to `wing`, not to the angle — the two sliders answer
         separate questions and must not fight */
      for (const a of [-25, 0, 30, 55]) approx(extOf(bird({ wingA: a }), 1), 1, 1e-9, `bird: ${a}° keeps the unit span (angle never steals from wingspan)`);
      approx(extOf(bird({ wing: 1.6, wingA: 40 }), 1), 1.6, 1e-9, "bird: wingspan and angle compose");
      /* the root edge is welded to the body at every angle (the wings are the
         8-point curves — body planform/profile are 7, the tail fan 6) */
      const wings = (o) => shapeWire("bird", null, o).filter((c) => c.length === 8);
      for (const a of [-25, 0, 55]) {
        const W = wings({ wingA: a });
        approx(W.length, 2, 0, `bird: ${a}° draws two wings`);
        const root = W.flat().filter((p) => Math.abs(Math.abs(p[1]) - 0.03) < 1e-9);
        approx(root.length > 0 && root.every((p) => Math.abs(p[0] - 0.08) < 1e-9 || Math.abs(p[0] + 0.06) < 1e-9) ? 1 : 0, 1, 0, `bird: ${a}° leaves the wing root on the body`);
      }
      /* symmetric: left and right swing together */
      const P40 = bird({ wingA: 40 });
      approx(P40.every((p) => P40.some((q) => Math.abs(q[0] - p[0]) < 1e-9 && Math.abs(q[1] + p[1]) < 1e-9)) ? 1 : 0, 1, 0, "bird: both wings swing together (mirror-symmetric)");
      /* legacy carry-over: an old fore/aft SLIDE becomes the angle that puts
         the tip in the same place, so a stored bird keeps its silhouette */
      approx(tipX({ wingX: 0.15 }), -0.05 + 0.15, 1e-9, "bird: legacy wingX puts the tip exactly where it used to be");
      approx(tipX({ wingX: -0.15 }) < tipX({ wingX: 0.15 }) ? 1 : 0, 1, 0, "bird: the legacy carry-over keeps its direction");
      approx(tipX({ wingA: 20, wingX: 0.15 }), tipX({ wingA: 20 }), 1e-12, "bird: an explicit angle wins over the legacy field");
      /* out of range clamps rather than folding the wing through the body */
      for (const a of [NaN, -400, 400, 89]) {
        const P = bird({ wingA: a });
        approx(P.length > 0 && Math.abs(extOf(P, 1) - 1) < 1e-9 && extOf(P, 0) < 2 ? 1 : 0, 1, 0, `bird: wingA ${String(a)} clamps to a valid solid`);
      }
    }

    /* the four-place registration rule, enforced rather than remembered: every
       kind in the picker needs a default pose, real geometry, and a report
       3-view (the last one fails SILENTLY — an unregistered kind renders an
       empty figure in the report). */
    {
      const views = fs.readFileSync("src/phodar.jsx", "utf8");
      const blk = views.slice(views.indexOf("const SHAPE_VIEWS = {"), views.indexOf("const SHAPE_VIEWS = {") + 2000);
      for (const s of SHAPES) {
        approx(SHAPE_R0()[s.k] ? 1 : 0, 1, 0, `${s.k}: registered in SHAPE_R0`);
        approx(shapeWire(s.k, 3, null).length > 0 ? 1 : 0, 1, 0, `${s.k}: shapeWire returns geometry`);
        approx(new RegExp("\\n  " + s.k + ":").test(blk) ? 1 : 0, 1, 0, `${s.k}: registered in SHAPE_VIEWS (report 3-view)`);
      }
    }
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
  const path = solveManualPoses(camRefs, refPose, { natW, natH }, { densify: false });
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
  const p1 = solveManualPoses(oneRef, refPose, { natW, natH }, { lockFov: true, densify: false });
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
    const pH = solveManualPoses([A, mk(gB), mk(gC)], { t: 1.0, az: 100, el: 20, roll: 2, fov: 60, k: 0 }, { natW, natH }, { densify: false });
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
    const raw = solveManualPoses(camRefsS, rp, { natW, natH }, { smooth: false, densify: false });
    const sm = solveManualPoses(camRefsS, rp, { natW, natH }, { densify: false });   // smoothed (default amt)
    const mid = (path) => path.find((p) => Math.abs(p.t - 1.2) < 1e-3);
    const truthMid = 102; // linear pan → az at t=1.2 is exactly 102
    const devRaw = Math.abs(mid(raw).az - truthMid), devSm = Math.abs(mid(sm).az - truthMid);
    approx(devRaw > 0.3 ? 1 : 0, 1, 0, `manualPose: the jittered keyframe is off before smoothing (${devRaw.toFixed(2)}°)`);
    approx(devSm < devRaw * 0.7 ? 1 : 0, 1, 0, `manualPose: smoothing pulls it back toward the pan (${devSm.toFixed(2)}° < ${devRaw.toFixed(2)}°)`);
    // the smoothing-amount slider: more smoothing → closer to the neighbours' pan
    const devLo = Math.abs(mid(solveManualPoses(camRefsS, rp, { natW, natH }, { smoothAmt: 0.15, densify: false })).az - truthMid);
    const devHi = Math.abs(mid(solveManualPoses(camRefsS, rp, { natW, natH }, { smoothAmt: 0.85, densify: false })).az - truthMid);
    approx(devHi < devLo ? 1 : 0, 1, 0, `manualPose: more smoothing = smoother (${devHi.toFixed(2)}° < ${devLo.toFixed(2)}°)`);
    // DENSIFY (default): SPARSE marks (1 s apart) become a fine grid so in-app
    // playback glides instead of jumping keyframe-to-keyframe.
    const sparseRefs = Gs.map((g) => ({ marks: [0.6, 1.6, 2.6].map((t) => { const p = dirToPixK(g, natW, natH, 100 + (t - 0.6) * 5, 20, 0, 60, 0); return { t, x: p.px, y: p.py }; }) }));
    const dense = solveManualPoses(sparseRefs, { t: 0.6, az: 100, el: 20, roll: 0, fov: 60, k: 0 }, { natW, natH });
    let maxGap = 0; for (let i = 1; i < dense.length; i++) maxGap = Math.max(maxGap, dense[i].t - dense[i - 1].t);
    approx(dense.length > 6 && maxGap <= 0.21 ? 1 : 0, 1, 0, `manualPose: sparse marks densified for smooth playback (${dense.length} samples, max gap ${maxGap.toFixed(2)}s)`);
  }

}

// --- AERIAL GEOLOCATION (looking DOWN): a downward sensor's pixel sight-line
//     intersected with the ground geolocates + sizes a target from ONE platform.
{
  const platform = { lat: 40, lon: -100, alt: 1000 };  // 1000 m above the ground plane (groundAlt 0)
  const cam = { natW: 1920, natH: 1080, fov: 30, k: 0, roll: 0 };
  const centre = (el, az) => pixelToGround(cam.natW / 2, cam.natH / 2, { ...cam, az, el }, platform, 0);

  // NADIR (straight down): centre pixel lands directly under the platform, slant = height
  const nad = centre(-90, 0);
  approx(nad && haversineM(nad, { lat: 40, lon: -100 }) < 1 ? 1 : 0, 1, 0, "geolocate: nadir look lands under the platform");
  approx(nad ? nad.slant : 0, 1000, 1, "geolocate: nadir slant range = height (1000 m)");

  // OBLIQUE 30° depression, looking East: horizontal = H/tan30 = 1732 m, slant = H/sin30 = 2000 m
  const ob = centre(-30, 90);
  approx(ob ? ob.slant : 0, 2000, 5, "geolocate: 30° depression slant = 2000 m");
  approx(ob ? haversineM(ob, { lat: 40, lon: -100 }) : 0, 1732, 5, "geolocate: 30° depression ground offset = 1732 m");
  approx(ob ? bearingDegGeo({ lat: 40, lon: -100 }, ob) : 0, 90, 0.5, "geolocate: looking East puts the target due East");

  // a ray pointing UP (or level) can't hit the ground → null
  approx(rayToGround(platform, [0, 0.5, 0.866], 0) == null ? 1 : 0, 1, 0, "geolocate: an up-looking ray hits no ground");
  approx(rayToGround({ ...platform, alt: -5 }, [0, 0, -1], 0) == null ? 1 : 0, 1, 0, "geolocate: a platform below ground returns null");

  // SIZE: ground span between two pixels scales linearly with their separation
  const camO = { ...cam, az: 0, el: -60 };
  const span1 = groundSpanM({ x: cam.natW / 2 - 100, y: cam.natH / 2 }, { x: cam.natW / 2 + 100, y: cam.natH / 2 }, camO, platform, 0);
  const span2 = groundSpanM({ x: cam.natW / 2 - 200, y: cam.natH / 2 }, { x: cam.natW / 2 + 200, y: cam.natH / 2 }, camO, platform, 0);
  approx(span1 > 0 && Math.abs(span2 / span1 - 2) < 0.05 ? 1 : 0, 1, 0, `geolocate: object size (2× pixels ⇒ 2× ground span, ${span1.toFixed(0)}→${span2.toFixed(0)} m)`);

  // SPEED/HEADING: a target geolocated across frames → ground kinematics
  const N_PER_DEG = 111320; // ~m per degree latitude
  const trk = [
    { t: 0, lat: 40, lon: -100 },
    { t: 5, lat: 40 + 1000 / N_PER_DEG, lon: -100 },   // 1000 m North in 5 s
  ];
  const gk = groundKinematics(trk);
  approx(gk ? gk.avgSpeedMS : 0, 200, 2, "geolocate: ground speed = 200 m/s (1000 m in 5 s)");
  approx(gk ? gk.headingDeg : -1, 0, 0.5, "geolocate: heading due North (0°)");

  // GEOREFERENCE off GCPs (no telemetry): generate GCPs with the ray-cast method,
  // fit a homography to them, then confirm it geolocates a FRESH pixel to the
  // SAME spot — using no platform pose at all. Cross-validates the two methods.
  {
    const camG = { natW: 1920, natH: 1080, fov: 30, k: 0, roll: 3, az: 30, el: -40 };
    const gcpPix = [{ x: 300, y: 250 }, { x: 1600, y: 260 }, { x: 340, y: 830 }, { x: 1580, y: 840 }, { x: 960, y: 540 }];
    const gcps = gcpPix.map((p) => { const g = pixelToGround(p.x, p.y, camG, platform, 0); return { px: p.x, py: p.y, lat: g.lat, lon: g.lon }; });
    const geo = groundHomography(gcps);
    approx(geo && geo.rms < 0.5 ? 1 : 0, 1, 0, `georef: GCP homography fits (rms ${geo ? geo.rms.toFixed(2) : "—"} m)`);
    const tp = { x: 1200, y: 400 };   // a fresh pixel, not a GCP
    const gTrue = pixelToGround(tp.x, tp.y, camG, platform, 0);
    const gH = pixelToGroundH(tp.x, tp.y, geo);
    approx(gH ? haversineM(gTrue, gH) : 1e9, 0, 1.0, "georef: a fresh pixel geolocates to the same spot as the ray-cast (<1 m)");
    // size via GCP homography matches the ray-cast size
    const p1 = { x: 900, y: 500 }, p2 = { x: 1050, y: 560 };
    const sH = groundSpanH(p1, p2, geo), sM = groundSpanM(p1, p2, camG, platform, 0);
    approx(Math.abs(sH - sM) < 1 ? 1 : 0, 1, 0, `georef: object size matches the ray-cast (${sH.toFixed(1)} vs ${sM.toFixed(1)} m)`);
  }
}

/* ── user-tunable smoothing strength (the field slider) ─────────────────────
   One knob s∈[0,1]: 0 = despike only (hard corners preserved — a real
   anomalous maneuver is never averaged away), 0.25 = the historical fixed
   default, 1 = heavy (an airplane's jittery track reads as its clean curve). */
{
  const mkO = (t, az, q) => ({ t, az, el: 20, q });
  // (a) backward compatibility: s=0.25 reproduces the legacy built-in smoothing
  const jig = () => Array.from({ length: 30 }, (_, i) => mkO(i * 0.25, 100 + i * 0.3 + (i % 2 ? 0.25 : -0.25), 0.9));
  const legacy = jig(); smoothObjPath(legacy);
  const at25 = jig(); smoothObjPathAt(at25, 0.25);
  approx(Math.max(...legacy.map((p, i) => Math.abs(p.az - at25[i].az))), 0, 1e-9, "smooth slider: s=0.25 equals the legacy object smoothing");
  const jigP = () => Array.from({ length: 30 }, (_, i) => ({ t: i * 0.25, az: 250 + (i % 2 ? 0.2 : -0.2), el: 20, roll: 0, fov: 60, n: 12 }));
  const legP = jigP(); smoothPath(legP);
  const atP = jigP(); smoothPathAt(atP, 0.25);
  approx(Math.max(...legP.map((p, i) => Math.abs(p.az - atP[i].az))), 0, 1e-9, "smooth slider: s=0.25 equals the legacy camera smoothing");
  // (b) strength monotonically cleans a noisy straight track (airplane case)
  const rmsTo = (op) => Math.sqrt(op.slice(2, -2).reduce((s2, p, i) => { const truth = 100 + (i + 2) * 0.3; return s2 + (p.az - truth) ** 2; }, 0) / (op.length - 4));
  const s0 = jig(); smoothObjPathAt(s0, 0);
  const s25 = jig(); smoothObjPathAt(s25, 0.25);
  const s100 = jig(); smoothObjPathAt(s100, 1);
  if (!(rmsTo(s100) < rmsTo(s25) && rmsTo(s25) < rmsTo(s0))) { console.error("  FAIL smooth slider: strength must monotonically clean a noisy line", rmsTo(s0), rmsTo(s25), rmsTo(s100)); fails++; }
  else console.log(`  ok   smooth slider: noisy line rms ${rmsTo(s0).toFixed(3)}° → ${rmsTo(s25).toFixed(3)}° → ${rmsTo(s100).toFixed(3)}° as strength rises`);
  approx(rmsTo(s100), 0, 0.06, "smooth slider: full strength nearly recovers the clean line");
  // (c) a genuine hard corner survives s=0 exactly and is rounded at s=1 —
  // the trade the slider makes explicit (UAP corners vs airplane curves)
  const corner = () => Array.from({ length: 21 }, (_, i) => mkO(i * 0.25, i <= 10 ? 100 + i * 0.5 : 105 - (i - 10) * 0.5, 0.9));
  const c0 = corner(); smoothObjPathAt(c0, 0);
  approx(c0[10].az, 105, 1e-9, "smooth slider: s=0 preserves a hard corner exactly");
  const c1 = corner(); smoothObjPathAt(c1, 1);
  if (!(c1[10].az < 104.9)) { console.error("  FAIL smooth slider: s=1 should round the corner", c1[10].az); fails++; }
  else console.log(`  ok   smooth slider: s=1 rounds the corner (apex 105 → ${c1[10].az.toFixed(2)})`);
  // (regression lock) the strength wrappers must return the PATH ARRAY itself —
  // smoothObjPath returns a despike COUNT, and passing that through once
  // overwrote a source's objPath with a number (field bug: slider vanished)
  const retO = jig();
  if (smoothObjPathAt(retO, 0.5) !== retO) { console.error("  FAIL smooth slider: smoothObjPathAt must return the path array"); fails++; }
  else console.log("  ok   smooth slider: smoothObjPathAt returns the path array");
  const retP = jigP();
  if (smoothPathAt(retP, 0.5) !== retP) { console.error("  FAIL smooth slider: smoothPathAt must return the path array"); fails++; }
  else console.log("  ok   smooth slider: smoothPathAt returns the path array");
  // (d) a perfectly linear camera pan is untouched even at full strength
  const pan = Array.from({ length: 25 }, (_, i) => ({ t: i * 0.25, az: 240 + i * 0.8, el: 20 + i * 0.1, roll: 0, fov: 60, n: 20 }));
  smoothPathAt(pan, 1);
  approx(Math.max(...pan.map((p, i) => Math.abs(p.az - (240 + i * 0.8)))), 0, 0.002, "smooth slider: a linear pan passes through untouched at full strength");
}


/* ── sensor attitude path: sync + visual/sensor fusion ──────────────────────
   The phone's attitude log is drift-free in pitch/roll (gravity) and smooth
   but BIASED in azimuth (compass). So sync must ignore bias, and fusion must
   take only MOTION from the sensor while vision owns the absolute frame. */
{
  /* a synthetic truth camera: a pan with a wobble, so the sync has real shape */
  const truth = (t) => ({ t, az: 250 + 4 * t + 1.5 * Math.sin(t * 2.2), el: 12 + 0.8 * Math.sin(t * 1.3), roll: 0.5 * Math.sin(t * 0.9) });
  /* the sensor log: same motion, its own clock (+0.42 s), compass biased +37° */
  const OFF = 0.42, BIAS = 37;
  const log = [];
  for (let t = -1; t <= 24; t += 1 / 60) { const p = truth(t - OFF); log.push({ t, az: p.az + BIAS, el: p.el, roll: p.roll }); }
  const visual = [];
  for (let t = 0; t <= 22; t += 0.25) visual.push({ ...truth(t), fov: 32, n: 30, held: false });

  approx(sensorAt(log, 5).el, truth(5 - OFF).el, 0.01, "sensorAt: interpolates the log");
  const sync = syncSensor(log, visual, { range: 1.5, step: 0.02 });
  /* convention: offset is what you ADD to a video time to index the log */
  approx(sync.offset, OFF, 0.05, "syncSensor: recovers the clock offset despite a 37° compass bias");
  approx(sync.conf > 0.5 ? 1 : 0, 1, 0, "syncSensor: reports high confidence on a clip with real motion");

  /* FUSION: knock out a run of frames the way a real tracker fails (held), and
     check the sensor carries the true motion across it instead of freezing */
  const broken = visual.map((p, i) => (i >= 20 && i <= 27 ? { ...p, held: true, n: 2, az: visual[19].az, el: visual[19].el, roll: visual[19].roll } : { ...p }));
  const fused = fuseSensorVisual(broken, log, { offset: sync.offset, minN: 6 });
  let heldErr = 0, fusedErr = 0;
  for (let i = 20; i <= 27; i++) {
    const tr = truth(broken[i].t);
    heldErr = Math.max(heldErr, Math.abs(((broken[i].az - tr.az + 540) % 360) - 180));
    fusedErr = Math.max(fusedErr, Math.abs(((fused[i].az - tr.az + 540) % 360) - 180));
  }
  approx(heldErr > 1.5 ? 1 : 0, 1, 0, "fusion: the frozen run really was badly wrong (setup check)");
  approx(fusedErr < 0.25 ? 1 : 0, 1, 0, `fusion: sensor carries the held run to truth (${fusedErr.toFixed(2)}° vs ${heldErr.toFixed(2)}° frozen)`);

  /* frames vision solved well must come through UNTOUCHED — the compass bias
     must never leak into an absolutely-anchored pose */
  let maxTouch = 0;
  for (let i = 0; i < fused.length; i++) if (fused[i].src === "v") maxTouch = Math.max(maxTouch, Math.abs(((fused[i].az - broken[i].az + 540) % 360) - 180));
  approx(maxTouch, 0, 1e-9, "fusion: solved frames are left exactly alone (no compass bias leak)");

  /* the carried run must MEET the recovered solve, not snap to it */
  const jump = Math.abs(((fused[28].az - fused[27].az + 540) % 360) - 180);
  approx(jump < 1.2 ? 1 : 0, 1, 0, "fusion: carried run is smeared onto the recovered solve, no snap");

  /* a gap longer than maxCarry is left frozen and FLAGGED rather than invented */
  const longGap = visual.map((p, i) => (i >= 20 && i <= 60 ? { ...p, held: true, n: 1 } : { ...p }));
  const st = fuseStats(fuseSensorVisual(longGap, log, { offset: sync.offset, maxCarry: 2.5 }));
  approx(st.h > 0 ? 1 : 0, 1, 0, "fusion: a too-long sensor-only stretch is left held, not fabricated");
  /* FIELD CASE: the tracker lost the scene and froze at ~98° with 34-46
     inliers while the phone actually swept 95°. Inlier count called every
     frame "strong", so fusion never fired — the disagreement check is what
     catches it. */
  const frozen = visual.map((p, i) => ({ ...p, az: i < 8 ? p.az : visual[8].az, el: i < 8 ? p.el : visual[8].el, n: 40, held: false }));
  const dis = motionDisagreement(frozen, log, sync.offset);
  approx(dis.ratio < 0.3 ? 1 : 0, 1, 0, `disagreement: a frozen solve is caught (vision ${dis.vis}° vs sensors ${dis.sen}°)`);
  const agree = motionDisagreement(visual, log, sync.offset);
  approx(agree.ratio > 0.8 && agree.ratio < 1.25 ? 1 : 0, 1, 0, `disagreement: a good solve agrees with the log (ratio ${agree.ratio})`);

  /* sensor-only path: motion from the log, ABSOLUTE frame from the placement */
  const anchor = { t: 4, az: 250, el: 12, roll: 0, fov: 41.6, k: 0 };
  const only = sensorOnlyPath(log, visual.map((p) => p.t), anchor, { offset: sync.offset });
  const atA = only.find((p) => Math.abs(p.t - 4) < 1e-6);
  approx(atA.az, 250, 0.05, "sensorOnly: the anchor frame keeps the placement azimuth exactly");
  approx(atA.el, 12, 0.05, "sensorOnly: the anchor frame keeps the placement elevation exactly");
  /* the 37° compass bias must cancel: motion between frames matches truth */
  const p8 = only.find((p) => Math.abs(p.t - 8) < 1e-6);
  approx(p8.az - atA.az, truth(8).az - truth(4).az, 0.15, "sensorOnly: motion matches truth, compass bias cancels");
  approx(only.every((p) => p.fov === 41.6) ? 1 : 0, 1, 0, "sensorOnly: FOV carried from the anchor (in-app capture has no optical zoom)");

  /* no log at all → unchanged path */
  approx(fuseSensorVisual(visual, null).length, visual.length, 0, "fusion: no sensor log leaves the path untouched");
}


/* ── manual pose fixes (Fix frames mode) ────────────────────────────────────
   Absolute per-frame anchors → a delta field: exact at anchors, linear
   between them, HELD beyond the outermost (drift heals forward); a zero-
   delta anchor bounds the corrected region. Az wrap-aware. */
{
  const base = Array.from({ length: 9 }, (_, i) => ({ t: i, az: 100 + i, el: 20, roll: 0, fov: 60, n: 12 }));
  // one anchor at t=4, +3° az / +1° el off the solve → exact there, held both ways
  let out = applyPoseFixes(base, [{ t: 4, az: 107, el: 21, roll: 0 }]);
  approx(out[4].az, 107, 1e-6, "poseFix: anchor frame lands exactly on the anchor");
  approx(out[4].el, 21, 1e-6, "poseFix: anchor elevation exact");
  approx(out[0].az, 103, 1e-6, "poseFix: delta held before the first anchor (drift heals backward)");
  approx(out[8].az, 111, 1e-6, "poseFix: delta held after the last anchor (drift heals forward)");
  // a ZERO-delta anchor bounds the correction: interpolates 3°→0° between t=4 and t=8
  out = applyPoseFixes(base, [{ t: 4, az: 107, el: 20, roll: 0 }, { t: 8, az: 108, el: 20, roll: 0 }]);
  approx(out[6].az, 107.5, 1e-6, "poseFix: linear delta interpolation between anchors");
  approx(out[8].az, 108, 1e-6, "poseFix: bounding anchor restores the solve");
  // az wrap: fixing 359.5 → 0.5 must be a +1 delta, not −359
  const wrapBase = [{ t: 0, az: 359.5, el: 10, roll: 0, fov: 60 }, { t: 1, az: 359.5, el: 10, roll: 0, fov: 60 }];
  out = applyPoseFixes(wrapBase, [{ t: 0, az: 0.5, el: 10, roll: 0 }]);
  approx(out[1].az, 0.5, 1e-6, "poseFix: az delta is wrap-aware across north");
  // the object-direction series rides the same field
  const dirs = [{ t: 2, az: 200, el: 30, q: 0.9 }, { t: 6, az: 201, el: 31, q: 0.9 }];
  const d2 = applyDirFixes(dirs, base, [{ t: 4, az: 107, el: 21, roll: 0 }]);
  approx(d2[0].az, 203, 1e-6, "dirFix: object dirs shift by the pose delta at their time");
  approx(d2[1].el, 32, 1e-6, "dirFix: elevation delta applied");
  /* EXACT dir fix (with frame dims): a ROLL anchor rotates an off-center
     object about the frame center — a plain az/el delta shift misses that
     entirely (field report: a ~4° roll fix pulled the tracked object off its
     carefully-placed path). The object is a fixed PIXEL: converting that
     pixel through the corrected pose must land exactly. */
  const NW = 1920, NH = 1080;
  const rollBase = Array.from({ length: 5 }, (_, i) => ({ t: i, az: 100, el: 20, roll: 0, fov: 60, k: 0 }));
  const rollFix = [{ t: 2, az: 100, el: 20, roll: 4, fov: 60 }];
  // an object well off frame-center, converted under the base pose of t=2
  const objPix = { px: 1500, py: 300 };
  const g0 = dirToAzEl(pixToDirK(objPix.px, objPix.py, NW, NH, 100, 20, 0, 60, 0));
  const dExact = applyDirFixes([{ t: 2, az: g0.az, el: g0.el, q: 1 }], rollBase, rollFix, { natW: NW, natH: NH });
  const want = dirToAzEl(pixToDirK(objPix.px, objPix.py, NW, NH, 100, 20, 4, 60, 0));
  approx(dExact[0].az, want.az, 1e-3, "dirFix exact: roll anchor maps the object pixel through the corrected pose (az)");
  approx(dExact[0].el, want.el, 1e-3, "dirFix exact: roll anchor exact (el)");
  const dApprox = applyDirFixes([{ t: 2, az: g0.az, el: g0.el, q: 1 }], rollBase, rollFix); // no dims → old shift
  approx(Math.abs(dApprox[0].el - want.el) > 0.3 ? 1 : 0, 1, 0, "dirFix exact: the az/el-shift approximation demonstrably misses a roll fix (regression guard)");
  /* waypoint snap: the hand-tapped track points are ground truth — the final
     track must pass exactly through them, with the matcher's detail between
     taps preserved (an off-track drift is corrected by an interpolated delta
     field, held beyond the outermost tap). */
  const drift = Array.from({ length: 9 }, (_, i) => ({ t: i, az: 100 + i + 0.8, el: 20 + 0.4, q: 0.8 })); // matcher path with a constant +0.8/+0.4 error
  const snapAn = [{ t: 2, az: 102, el: 20 }, { t: 6, az: 106, el: 20 }];
  const snapped = snapDirsToAnchors(drift, snapAn);
  approx(snapped[2].az, 102, 1e-6, "waypoint snap: track passes exactly through a tap (az)");
  approx(snapped[2].el, 20, 1e-6, "waypoint snap: track passes exactly through a tap (el)");
  approx(snapped[4].az, 104, 1e-6, "waypoint snap: constant error fully corrected between taps");
  approx(snapped[0].az, 100, 1e-6, "waypoint snap: correction held before the first tap");
  approx(snapped[8].az, 108, 1e-6, "waypoint snap: correction held after the last tap");
  // a real wiggle BETWEEN taps survives: same anchors, matcher path with a bump at t=4
  const bump = drift.map((p) => (p.t === 4 ? { ...p, az: p.az + 1.5 } : p));
  const snapped2 = snapDirsToAnchors(bump, snapAn);
  approx(snapped2[4].az - snapped[4].az, 1.5, 1e-6, "waypoint snap: matcher detail between taps is preserved");
}

/* ── pinFind: faint-object pixel pin (close-up export) ──────────────────────
   Integral-image contrast sweep: must ACQUIRE a faint blob far from the seed
   (the field failure mode: prediction ~150+ px off a 12 px object), localise
   it to sub-pixel, and never fake a lock on empty sky. */
{
  const W2 = 360, H2 = 360;
  const mk = (bx, by, amp) => {
    const d = new Uint8ClampedArray(W2 * H2 * 4);
    for (let y = 0; y < H2; y++) for (let x = 0; x < W2; x++) {
      let v = 118 + 22 * (y / H2) + 6 * Math.sin(x * 0.05);
      const rr = Math.hypot(x - bx, y - by);
      if (amp) v -= amp * Math.exp(-(rr * rr) / (2 * 4.2 * 4.2));
      const i = (y * W2 + x) * 4; d[i] = d[i + 1] = d[i + 2] = v; d[i + 3] = 255;
    }
    return d;
  };
  let f = pinFind(mk(287.5, 96.5, 13), W2, H2, 180, 200, { objR: 5, reach: 160, step: 3 });
  approx(Math.hypot(f.x - 287.5, f.y - 96.5), 0, 1.0, "pinFind: acquires a faint blob ~150px off the seed (sub-px)");
  if (!(f.score >= 5)) { console.error("  FAIL pinFind: on-object score must clear the pin threshold", f.score); fails++; }
  else console.log(`  ok   pinFind: on-object score ${f.score.toFixed(1)} clears the pin threshold (5)`);
  f = pinFind(mk(0, 0, 0), W2, H2, 180, 180, { objR: 5, reach: 120, step: 3 });
  if (!(f.score < 5)) { console.error("  FAIL pinFind: empty sky must score below the pin threshold", f.score); fails++; }
  else console.log(`  ok   pinFind: empty gradient sky scores ${f.score.toFixed(2)} — no false lock`);
}

/* ── pinStep at DEEP ZOOM (field case: 10× zoom clip, object off frame for
   stretches of the close-up). At fov 5° one degree is ~144 px: between the
   solve's 0.25 s knots the interpolated pose can be ~1° wrong, so a LOCKED
   pin whose search window scales only with object size (±0.2°) missed its
   own prediction and the hold/glide frames wore the raw pose error. The
   locked window now scales with px-per-degree, and the detector radius
   follows the object's real pixel size (a 14 px cap put the contrast peak
   on a 200 px object's RIM). A locked pin must re-find the object through
   a 0.7° pose lie and aim so the object renders centered. */
{
  const natW = 720, natH = 1280;
  const oPose = { az: 0, el: 40, roll: 0, fov: 5, k: 0 };       // interpolated (wrong) pose
  const tPose = { az: 0.7, el: 40, roll: 0, fov: 5, k: 0 };     // the frame's TRUE pose
  const dTrue = pixToDirK(360, 640, natW, natH, tPose.az, tPose.el, tPose.roll, tPose.fov, tPose.k); // object at true frame center
  const objNat = dirToPixK(dTrue, natW, natH, tPose.az, tPose.el, tPose.roll, tPose.fov, tPose.k);   // its native pixel (360,640)
  const sample = (cx, cy, W2) => {
    const d = new Uint8ClampedArray(W2 * W2 * 4);
    for (let y = 0; y < W2; y++) for (let x = 0; x < W2; x++) {
      const nx = cx - W2 / 2 + x, ny = cy - W2 / 2 + y;
      const rr = Math.hypot(nx - objNat.px, ny - objNat.py);
      let v = 150 + 12 * (ny / natH);
      if (rr < 55) v -= 65 * Math.exp(-(rr * rr) / (2 * 34 * 34));
      const i = (y * W2 + x) * 4; d[i] = d[i + 1] = d[i + 2] = v; d[i + 3] = 255;
    }
    return d;
  };
  const st = { oAz: 0, oEl: 0, has: 1, missRun: 0, ok: 3 }; // LOCKED, dead-on last frame (zero offset from the track)
  const ae = dirToAzEl(dTrue);
  let r = pinStep(st, { az: ae.az, el: ae.el }, { natW, natH, pose: oPose, maxAng: 1.0 }, sample);
  if (r.mode !== "lock") { console.error("  FAIL pin deep zoom: locked pin must survive a 0.7° pose lie, got mode", r.mode); fails++; }
  else console.log("  ok   pin deep zoom: locked pin re-finds the object through a 0.7° pose-interpolation error");
  /* the EMA converges over a couple of frames while the pose error persists —
     what steady tracking inside a knot interval actually looks like */
  for (let i = 0; i < 2; i++) r = pinStep(st, { az: ae.az, el: ae.el }, { natW, natH, pose: oPose, maxAng: 1.0 }, sample);
  const aimPx = dirToPixK(dirFromAzEl(r.az, r.el), natW, natH, oPose.az, oPose.el, oPose.roll, oPose.fov, oPose.k);
  const missPx = Math.hypot(aimPx.px - objNat.px, aimPx.py - objNat.py);
  approx(missPx, 0, 9, "pin deep zoom: aim converges onto the object within 3 frames (px miss)");
  /* FRAME-EDGE HONESTY: at deep zoom the acquire window reaches the frame
     border, where the caller's black padding forms a huge artificial
     contrast edge — the pin locked onto THE FRAME EDGE and steered the
     close-up at it (field-measured). With no object in reach, an
     edge-in-window search must MISS, never lock on the border. */
  const sampleEdge = (cx, cy, W2) => {
    const d = new Uint8ClampedArray(W2 * W2 * 4);
    for (let y = 0; y < W2; y++) for (let x = 0; x < W2; x++) {
      const nx = cx - W2 / 2 + x;
      const v = nx < 0 || nx >= natW ? 0 : 150; // black padding left of the frame
      const i = (y * W2 + x) * 4; d[i] = d[i + 1] = d[i + 2] = v; d[i + 3] = 255;
    }
    return d;
  };
  const dEdge = pixToDirK(90, 640, natW, natH, oPose.az, oPose.el, oPose.roll, oPose.fov, oPose.k); // near the left frame edge
  const aeE = dirToAzEl(dEdge);
  const stE = { oAz: 0, oEl: 0, has: 0, missRun: 99, ok: 0 }; // acquiring (wide window)
  const rE = pinStep(stE, { az: aeE.az, el: aeE.el }, { natW, natH, pose: oPose, maxAng: 1.0 }, sampleEdge);
  if (rE.mode === "lock") { console.error("  FAIL pin edge: locked onto the black frame border"); fails++; }
  else console.log(`  ok   pin edge: black frame border never wins the contrast sweep (mode ${rE.mode})`);
  /* POLARITY (field case: from ~12 s a BRIGHT cloud lump out-contrasted the
     faint dark object and the polarity-blind sweep locked on the cloud).
     With pol −1 from the human's marks the dark object wins even against a
     stronger bright blob; polarity-blind, the bright blob wins — both
     asserted so the discrimination is proven, not assumed. */
  const objP = { x: 300, y: 640 }, cloudP = { x: 420, y: 640 };
  const sampleClutter = (cx, cy, W2) => {
    const d = new Uint8ClampedArray(W2 * W2 * 4);
    for (let y = 0; y < W2; y++) for (let x = 0; x < W2; x++) {
      const nx = cx - W2 / 2 + x, ny = cy - W2 / 2 + y;
      let v = 150;
      const rO = Math.hypot(nx - objP.x, ny - objP.y);
      if (rO < 40) v -= 22 * Math.exp(-(rO * rO) / (2 * 24 * 24));   // faint dark object
      const rC = Math.hypot(nx - cloudP.x, ny - cloudP.y);
      if (rC < 60) v += 55 * Math.exp(-(rC * rC) / (2 * 36 * 36));   // brighter cloud lump
      const i = (y * W2 + x) * 4; d[i] = d[i + 1] = d[i + 2] = v; d[i + 3] = 255;
    }
    return d;
  };
  const dPredC = pixToDirK(360, 640, natW, natH, oPose.az, oPose.el, oPose.roll, oPose.fov, oPose.k);
  const aeC = dirToAzEl(dPredC);
  const runClutter = (pol) => {
    const stC = { oAz: 0, oEl: 0, has: 0, missRun: 0, ok: 0 };
    const rC = pinStep(stC, { az: aeC.az, el: aeC.el }, { natW, natH, pose: oPose, maxAng: 1.0, pol }, sampleClutter);
    const pp = dirToPixK(dirFromAzEl(rC.az, rC.el), natW, natH, oPose.az, oPose.el, oPose.roll, oPose.fov, oPose.k);
    return { mode: rC.mode, dObj: Math.hypot(pp.px - objP.x, pp.py - objP.y), dCloud: Math.hypot(pp.px - cloudP.x, pp.py - cloudP.y) };
  };
  const cNeg = runClutter(-1), cBlind = runClutter(0);
  if (cNeg.mode === "lock" && cNeg.dObj < cNeg.dCloud && cNeg.dObj < 25) console.log(`  ok   pin polarity: dark object beats a brighter cloud lump when polarity is known (${cNeg.dObj.toFixed(0)}px off object)`);
  else { console.error("  FAIL pin polarity: dark object must win with pol −1", cNeg); fails++; }
  if (cBlind.dCloud < cBlind.dObj) console.log("  ok   pin polarity: polarity-blind sweep picks the cloud — the discrimination is real, not incidental");
  else { console.error("  FAIL pin polarity control: expected the blind sweep to pick the cloud", cBlind); fails++; }
}

/* ── re-anchoring solved paths across a placement change ────────────────────
   posePath/objPath are solved relative to the alignment frame's placement;
   when the placement moves, the stored world data must rotate by exactly the
   old→new placement rotation (R = B_new·B_oldᵀ). */
{
  const F = { az: 250.0, el: 8.0, roll: 1.5 };     // placement at stabilize time
  const T = { az: 243.2, el: 10.4, roll: -0.8 };   // re-aligned placement
  // identity: same from/to leaves a pose untouched
  let q = reanchorPose({ az: 100, el: 20, roll: 5 }, F, F);
  approx(q.az, 100, 1e-6, "reanchor: identity keeps az");
  approx(q.el, 20, 1e-6, "reanchor: identity keeps el");
  approx(q.roll, 5, 1e-6, "reanchor: identity keeps roll");
  // pure yaw delta: every pose yaws by the same amount, el/roll untouched
  q = reanchorPose({ az: 100, el: 20, roll: 5 }, { az: 10, el: 0, roll: 0 }, { az: 35, el: 0, roll: 0 });
  approx(q.az, 125, 1e-6, "reanchor: pure yaw shifts az by the delta");
  approx(q.el, 20, 1e-6, "reanchor: pure yaw keeps el");
  approx(q.roll, 5, 1e-6, "reanchor: pure yaw keeps roll");
  // consistency: a pixel through the re-anchored pose equals the rotated
  // direction of the same pixel through the original pose (fov/k carried)
  const P = { az: 231.0, el: 14.0, roll: 3.0, fov: 62, k: 0.05 };
  const P2 = { ...reanchorPose(P, F, T), fov: P.fov, k: P.k };
  for (const [px, py] of [[100, 80], [1800, 950], [960, 540]]) {
    const d1 = reanchorDir(pixToDirK(px, py, 1920, 1080, P.az, P.el, P.roll, P.fov, P.k), F, T);
    const d2 = pixToDirK(px, py, 1920, 1080, P2.az, P2.el, P2.roll, P2.fov, P2.k);
    const c = Math.min(1, Math.max(-1, d1[0] * d2[0] + d1[1] * d2[1] + d1[2] * d2[2]));
    approx(Math.acos(c) * R2D, 0, 1e-4, `reanchor: pixel (${px},${py}) maps identically through the re-anchored pose`);
  }
  // bare az/el direction rides the same rotation
  const ae = reanchorAzEl(200, 30, F, T);
  const dd = reanchorDir(dirFromAzEl(200, 30), F, T);
  const ae2 = dirToAzEl(dd);
  approx(ae.az, ae2.az, 1e-9, "reanchor: az/el helper matches the dir rotation");
  approx(ae.el, ae2.el, 1e-9, "reanchor: az/el helper matches the dir rotation (el)");
}

/* ── capture pose from phone gravity (web sensor capture) ──────────────────
   Device frame X=right, Y=top, Z=out-of-screen; back camera looks along −Z.
   Inputs are iOS accelerationIncludingGravity (points ALONG the pull, so
   up_device = −accel, fixed). Azimuth model is FIELD-CALIBRATED:
   portrait ⇒ webkitCompassHeading is already the camera heading (iOS
   tilt-compensates — verified aiming down 11° AND up 20°); landscape ⇒ it's
   the portrait top edge, ±90° off, recovered by the horizontal-plane
   rotation. opts.orient (screen angle) selects the regime. */
{
  const g = 9.81;
  // upright portrait, camera at the horizon: gravity down = −Y → el 0, roll 0,
  // and the heading passes through untouched (iOS already reports the camera)
  let p = poseFromGravity({ x: 0, y: -g, z: 0 }, 137);
  approx(p.el, 0, 0.01, "capture pose: horizon shot → elevation 0°");
  approx(p.roll, 0, 0.01, "capture pose: level shot → roll 0°");
  approx(p.az, 137, 0.01, "capture pose: portrait → heading is the camera az, untouched");
  // camera straight up (screen faces the ground, gravity +Z): el +90
  p = poseFromGravity({ x: 0, y: 0, z: g }, 0);
  approx(p.el, 90, 0.01, "capture pose: straight-up shot → elevation +90°");
  // camera straight down (gravity −Z): el −90
  p = poseFromGravity({ x: 0, y: 0, z: -g }, 0);
  approx(p.el, -90, 0.01, "capture pose: straight-down shot → elevation −90°");
  // THE DISCRIMINATING FIELD CASE: portrait leaning BACK 22° (camera aims UP).
  // iOS still reports the camera heading (241 read for truth 242); a top-edge
  // model would demand +180 here. Passthrough + positive elevation.
  const t22 = 22 / R2D;
  p = poseFromGravity({ x: 0, y: -g * Math.cos(t22), z: g * Math.sin(t22) }, 241, { orient: 0 });
  approx(p.az, 241, 0.1, "capture pose: portrait aimed UP → heading still passes through");
  approx(p.el, 22, 0.1, "capture pose: portrait aimed UP 22° → elevation +22°");
  // portrait leaning forward 11° (camera aims down): passthrough, el −11
  const t11 = 11 / R2D;
  p = poseFromGravity({ x: 0, y: -g * Math.cos(t11), z: -g * Math.sin(t11) }, 247, { orient: 0 });
  approx(p.az, 247, 0.1, "capture pose: portrait aimed down → heading passes through");
  approx(p.el, -11, 0.1, "capture pose: portrait aimed down 11° → elevation −11°");
  // rolled 30° (up leans toward +X ⇒ accel toward −X): roll ≈ 30
  p = poseFromGravity({ x: -Math.sin(30 / R2D) * g, y: -Math.cos(30 / R2D) * g, z: 0 }, 0);
  approx(p.roll, 30, 0.02, "capture pose: 30° tilt → roll 30°");
  // LANDSCAPE (orient ±90): heading is the portrait TOP edge, ±90° off the
  // camera — the horizontal-plane rotation recovers the camera az, and it
  // self-selects the sign for either hold. NO extra 180 anywhere.
  // landscape-left (top→West, up=+X): gravity −X, top-heading 270 → camera 0
  p = poseFromGravity({ x: -g, y: 0, z: 0 }, 270, { orient: 90 });
  approx(p.az, 0, 0.1, "capture pose: landscape-left → camera azimuth (top edge +90)");
  approx(p.el, 0, 0.01, "capture pose: landscape horizon → elevation 0°");
  approx(p.roll, 0, 0.01, "capture pose: landscape hold → roll folds to ~0°");
  // landscape-right (top→East, up=−X): gravity +X, top-heading 90 → camera 0
  p = poseFromGravity({ x: g, y: 0, z: 0 }, 90, { orient: 270 });
  approx(p.az, 0, 0.1, "capture pose: landscape-right → camera azimuth (top edge −90)");
  // without an orient hint the safe default is portrait passthrough
  p = poseFromGravity({ x: -g, y: 0, z: 0 }, 270);
  approx(p.az, 270, 0.1, "capture pose: no orient hint → portrait passthrough default");
  // ⇅ flip (elSign −1) inverts ONLY the tilt sense…
  const pFlip = poseFromGravity({ x: 0, y: 0, z: g }, 0, { elSign: -1 });
  approx(pFlip.el, -90, 0.01, "capture pose: ⇅ flip tilt inverts elevation (elSign −1)");
  // …and can never move the bearing, in either regime
  const pAz1 = poseFromGravity({ x: -g, y: 0, z: 0 }, 270, { orient: 90 });
  const pAz2 = poseFromGravity({ x: -g, y: 0, z: 0 }, 270, { orient: 90, elSign: -1 });
  approx(pAz2.az, pAz1.az, 0.01, "capture pose: flipping tilt does not move the bearing");

  /* ---- NON-iOS SENSOR PATHS ----
     Only iOS reports webkitCompassHeading and an accelerometer along the pull.
     Everything else has to come out of the W3C orientation angles. */
  head("capture pose — non-iOS sensor paths");

  // up vector: the three unambiguous holds
  let u = upFromOrientation(0, 0);
  approx(u.z, 1, 1e-9, "orientation up: flat, screen up → +Z");
  u = upFromOrientation(90, 0);
  approx(u.y, 1, 1e-9, "orientation up: upright portrait → +Y");
  u = upFromOrientation(0, 90);
  approx(u.x, -1, 1e-9, "orientation up: rolled 90° about Y → −X");

  // the closed forms must equal an EXPLICIT R = Rz(α)Rx(β)Ry(γ), everywhere
  {
    const mul = (A, B) => A.map((r) => [0, 1, 2].map((j) => r[0] * B[0][j] + r[1] * B[1][j] + r[2] * B[2][j]));
    const Rz = (a) => [[Math.cos(a), -Math.sin(a), 0], [Math.sin(a), Math.cos(a), 0], [0, 0, 1]];
    const Rx = (a) => [[1, 0, 0], [0, Math.cos(a), -Math.sin(a)], [0, Math.sin(a), Math.cos(a)]];
    const Ry = (a) => [[Math.cos(a), 0, Math.sin(a)], [0, 1, 0], [-Math.sin(a), 0, Math.cos(a)]];
    let maxUp = 0, maxAz = 0, maxEl = 0, seed = 7;
    const rnd = () => (seed = (seed * 1103515245 + 12345) % 2147483648) / 2147483648;
    for (let i = 0; i < 2000; i++) {
      const a = rnd() * 360, b = rnd() * 360 - 180, gm = rnd() * 180 - 90;
      const R = mul(Rz(a / R2D), mul(Rx(b / R2D), Ry(gm / R2D)));
      const cam = [0, 1, 2].map((k) => -R[k][2]);          // R·(0,0,−1): the back camera
      const uu = upFromOrientation(b, gm);
      maxUp = Math.max(maxUp, Math.abs(uu.x - R[2][0]), Math.abs(uu.y - R[2][1]), Math.abs(uu.z - R[2][2]));
      const po = poseFromOrientation(a, b, gm);
      const elT = Math.asin(Math.max(-1, Math.min(1, cam[2]))) * R2D;
      maxEl = Math.max(maxEl, Math.abs(po.el - elT));
      if (Math.abs(elT) < 89.5) {                          // azimuth is meaningless looking straight up/down
        const azT = ((Math.atan2(cam[0], cam[1]) * R2D) % 360 + 360) % 360;
        maxAz = Math.max(maxAz, Math.abs(((po.az - azT + 540) % 360) - 180));
      }
    }
    ok(maxUp < 1e-9, `orientation up matches an explicit rotation matrix (max ${maxUp.toExponential(1)})`);
    ok(maxAz <= 0.06, `orientation azimuth matches the explicit matrix over 2000 holds (max ${maxAz.toFixed(3)}°)`);
    ok(maxEl <= 0.06, `orientation elevation matches the explicit matrix over 2000 holds (max ${maxEl.toFixed(3)}°)`);
  }

  // named holds — α is NOT a camera heading, which is the whole reason this exists
  let po = poseFromOrientation(0, 90, 0);
  approx(po.az, 0, 0.06, "orientation pose: upright portrait, α=0 → camera faces north");
  approx(po.el, 0, 0.06, "orientation pose: upright portrait → horizon");
  approx(poseFromOrientation(45, 90, 0).az, 315, 0.06, "orientation pose: α=45 → heading 315 (α runs the other way)");
  approx(poseFromOrientation(0, 110, 0).el, 20, 0.06, "orientation pose: leaned back 20° → camera up 20°");
  approx(poseFromOrientation(0, 79, 0).el, -11, 0.06, "orientation pose: leaned forward 11° → camera down 11°");
  approx(poseFromOrientation(0, 0, -90).az, 90, 0.06, "orientation pose: landscape hold → camera east");
  approx(poseFromOrientation(0, 0, 0).el, -90, 0.06, "orientation pose: flat on a table → camera at the floor");
  approx(poseFromOrientation(0, 110, 0, { elSign: -1 }).el, -20, 0.06, "orientation pose: ⇅ flip inverts only elevation");

  /* GRAVITY SIGN. iOS reports the vector along the pull, the W3C/Chrome
     convention reports proper acceleration — the exact opposite. Detected by
     comparing against the orientation angles, never by sniffing the UA. */
  ok(gravitySign({ x: 0, y: 0, z: -g }, 0, 0) === 1, "gravity sign: iOS convention flat on a table → +1");
  ok(gravitySign({ x: 0, y: 0, z: g }, 0, 0) === -1, "gravity sign: W3C/Chrome convention flat on a table → −1");
  ok(gravitySign({ x: 0, y: -g, z: 0 }, 90, 0) === 1, "gravity sign: iOS convention held upright → +1");
  ok(gravitySign({ x: 0, y: g, z: 0 }, 90, 0) === -1, "gravity sign: W3C/Chrome convention held upright → −1");
  ok(gravitySign({ x: 20, y: 0, z: 0 }, 0, 0) === 0, "gravity sign: a swung phone can't answer → 0 (keep the last)");
  ok(gravitySign({ x: 0, y: 0, z: -g }, null, null) === 0, "gravity sign: no orientation angles → 0");

  /* Feeding a −1 device's reading through the correction must land on exactly
     the same pose the +1 device reports — that IS the compatibility fix. */
  {
    /* ONE physical hold — leaned back 24°, tilted 9° — expressed both ways */
    const B = 114, G = 9;
    const uh = upFromOrientation(B, G);
    const gi = { x: -uh.x * g, y: -uh.y * g, z: -uh.z * g };       // iOS: along the pull
    const gc = { x: -gi.x, y: -gi.y, z: -gi.z };                   // W3C/Chrome: proper acceleration
    ok(gravitySign(gi, B, G) === 1, "gravity sign: helper leaves an iOS reading alone");
    const k = gravitySign(gc, B, G);
    ok(k === -1, "gravity sign: the same hold read the other way is caught");
    const a = poseFromGravity(gi, 137, { orient: 0 });
    const b = poseFromGravity({ x: gc.x * k, y: gc.y * k, z: gc.z * k }, 137, { orient: 0 });
    approx(b.el, a.el, 1e-9, "gravity sign: corrected Chrome reading gives the iOS elevation");
    approx(b.roll, a.roll, 1e-9, "gravity sign: corrected Chrome reading gives the iOS roll");
    /* and UNCORRECTED it would have been wrong by exactly the amount that
       matters — a negated tilt, which is the whole bug */
    const bad = poseFromGravity(gc, 137, { orient: 0 });
    approx(bad.el, -a.el, 1e-9, "gravity sign: uncorrected, the elevation comes out negated");
    /* the orientation path must agree with the corrected gravity path on the
       same hold — two independent routes to one pose */
    const po2 = poseFromOrientation(0, B, G);
    approx(po2.el, a.el, 0.06, "orientation and corrected-gravity paths agree on elevation");
    approx(po2.roll, a.roll, 0.06, "orientation and corrected-gravity paths agree on roll");
  }

  /* headingIsCamera: the non-iOS path resolves the CAMERA heading itself, so
     the landscape regime correction — which exists only to undo iOS's own tilt
     compensation — must not fire. */
  {
    const gl = { x: -g, y: 0, z: 0 };
    const withRegime = poseFromGravity(gl, 270, { orient: 90 });
    const asCamera = poseFromGravity(gl, 270, { orient: 90, headingIsCamera: true });
    approx(asCamera.az, 270, 0.06, "headingIsCamera: an already-resolved camera heading passes through untouched");
    ok(Math.abs(((withRegime.az - asCamera.az + 540) % 360) - 180) > 45, "headingIsCamera: it really did suppress the iOS landscape correction");
  }

  /* honest wording per path */
  ok(/magnetic/.test(poseQuality(null, true, "orient").note), "poseQuality: the orientation path warns its bearing may be magnetic");
  ok(poseQuality(5, true, "orient").headingOk === false, "poseQuality: no accuracy claim on the orientation path");
  ok(poseQuality(5, true, "ios").headingOk === true, "poseQuality: iOS keeps its accuracy-backed confidence");
  ok(/set the bearing yourself/.test(poseQuality(null, true, null).note), "poseQuality: no compass at all is said plainly");
}

// --- Visibility-aware stereo: intermittently-visible tracks (field case) ---
// The drone was visible only in SECTIONS of each of two videos; the witness
// captured path where they could. Interpolating a direction across a
// visibility hole fabricates a ray nobody observed — the triangulation must
// use only instants inside EVERY witness's visible segments, and say how
// much it ignored. Truth here turns sharply INSIDE observer 1's hole, so the
// old cut-the-corner interpolation would be badly wrong there.
{
  head("visibility-aware stereo (gaps)");
  const segs = trackSegments([0, 0.5, 1, 1.5, 8, 8.5, 9]);
  ok(segs.length === 2 && segs[0][1] === 1.5 && segs[1][0] === 8, "a 6.5 s hole splits a 0.5 s-cadence track into two segments");
  ok(trackSegments([0, 2, 4, 6]).length === 1, "a sparse-but-steady track stays ONE segment (threshold rides the cadence)");
  approx(segsDur(interSegments([[0, 8], [14, 20]], [[4, 16]])), 6, 1e-9, "segment intersection duration");

  const ref = { lat: 42.164, lon: -123.648, alt: 0 };
  const truthAt = (t) => (t <= 10 ? [100 + 10 * t, 300, 50] : [200, 300 + 12 * (t - 10), 50]); // sharp 90° turn at t=10
  const obsDef = [{ ...ref }, { ...geoFromEnu([300, 0, 0], ref), alt: 0 }];
  const azelFor = (o, t) => {
    const P = enuFromGeo(...(() => { const g = geoFromEnu(truthAt(t), ref); return [g.lat, g.lon, g.alt]; })(), o);
    return dirToAzEl(unit(P));
  };
  const mkSrc = (o, name, times) => ({
    name, lat: o.lat, lon: o.lon, alt: o.alt, A: {}, B: {},
    track: times.map((t) => ({ t, ...azelFor(o, t) })),
  });
  const times1 = [], times2 = [];
  for (let t = 0; t <= 20 + 1e-9; t += 0.5) {
    if (t < 8 || t > 14) times1.push(+t.toFixed(1)); // obs1 loses the object across the turn
    times2.push(+t.toFixed(1));
  }
  const trk = analyzeTracks([mkSrc(obsDef[0], "A", times1), mkSrc(obsDef[1], "B", times2)]);
  ok(!!(trk.stereo && trk.stereo.k), "stereo path solves from the shared-visibility sections");
  ok(trk.stereo.times.every((t) => t <= 8.01 || t >= 13.99), "no triangulated instant falls inside the invisibility hole");
  let worst = 0;
  for (let i = 0; i < trk.stereo.times.length; i++)
    worst = Math.max(worst, mag(sub(trk.stereo.pos[i], truthAt(trk.stereo.times[i]))));
  approx(worst, 0, 1.5, "every triangulated point sits on the truth path (m worst)");
  // obs1's last pre-hole sample is 7.5 and first post-hole 14.5 (the generator
  // excludes the boundary samples), so shared = [0,7.5]+[14.5,20] = 13 s
  approx(trk.stereo.sharedDur, 13, 0.2, "shared-visibility duration reported");
  approx(trk.stereo.cutDur, 7, 0.2, "ignored (one-witness-blind) duration reported");
  // no shared visibility at all → named, not a bogus fix
  const trkNo = analyzeTracks([
    mkSrc(obsDef[0], "A", times2.filter((t) => t <= 8)),
    mkSrc(obsDef[1], "B", times2.filter((t) => t >= 14)),
  ]);
  ok(!!(trkNo.stereo && trkNo.stereo.overlapErr), "disjoint visibility → overlap error, not a fabricated fix");

  // dense two-video stereo: a low-q fabricated stretch (tracker held/guided
  // while the object was invisible) must be excluded the same way
  const when = Date.parse("2026-08-01T17:00:00Z") / 1000;
  const mkClip = (o, name, badLo, badHi) => {
    const op = [];
    for (let t = 0; t <= 20 + 1e-9; t += 0.25) {
      const tt = +t.toFixed(2);
      const bad = tt > badLo && tt < badHi;
      if (bad) {
        const g0 = azelFor(o, badLo); // frozen hold — the fabricated path
        op.push({ t: tt, az: g0.az, el: g0.el, q: 0.2 });
      } else op.push({ t: tt, ...azelFor(o, tt), q: 0.95 });
    }
    return { name, lat: o.lat, lon: o.lon, alt: o.alt, whenMs: when * 1000, objPath: op };
  };
  const sv = stereoVideo([mkClip(obsDef[0], "A", 10, 18), mkClip(obsDef[1], "B", -1, -1)]);
  ok(!!(sv && sv.ok), "two-video stereo solves around the low-q stretch");
  ok(sv.qDropped >= 30, `held/guided samples dropped as non-observations (${sv.qDropped})`);
  ok(sv.times.every((t) => t - when <= 10.01 || t - when >= 17.99), "no dense-stereo instant falls inside the fabricated stretch");
  let worstV = 0;
  for (let i = 0; i < sv.times.length; i++)
    worstV = Math.max(worstV, mag(sub(sv.pos[i], truthAt(sv.times[i] - when))));
  approx(worstV, 0, 2.0, "dense stereo points sit on the truth path (m worst)");
  ok(sv.cutDur > 6, `blind stretch reported as ignored (${sv.cutDur.toFixed(1)} s)`);
}

// --- Per-frame-pose waypoints: a panning camera must not fabricate speed ---
// Field measurement (two-camera drone pass vs the flight log): the handheld
// witness's pan added ~10 mph of phantom speed under the static-camera
// assumption (32 measured vs 21 logged). With a solved posePath, each pixel
// waypoint converts through ITS OWN frame's pose, subtracting the pan.
{
  head("per-frame-pose waypoints");
  const natW = 1920, natH = 1080, FOV = 60;
  // the camera PANS 40° across a HOVERING object — the truth is zero motion,
  // and the static-camera assumption reads the whole pan as object speed
  const camAt = (t) => ({ t, az: 200 + 2 * t, el: 10, roll: 0, fov: FOV, k: 0 });
  const objAt = (t) => ({ az: 218, el: 12 });
  const track = [], posePath = [];
  for (let t = 0; t <= 20 + 1e-9; t += 0.5) {
    const P = camAt(t);
    posePath.push(P);
    const g = objAt(t);
    const px = dirToPixK(dirFromAzEl(g.az, g.el), natW, natH, P.az, P.el, 0, FOV, 0);
    track.push({ t: +t.toFixed(1), x: px.px, y: px.py });
  }
  const base = { name: "P", lat: 42.164, lon: -123.648, alt: 0, fovH: FOV, natW, natH, whenMs: 0,
    A: { az: "218", el: "12" }, B: {}, mediaAim: { az: 200, el: 10, roll: 0 }, track };
  const withPose = trackDirections({ ...base, posePath });
  const withoutPose = trackDirections(base);
  const rate = (dirs) => {
    let sweep = 0;
    for (let i = 1; i < dirs.length; i++) sweep += Math.acos(Math.min(1, Math.max(-1, dot(dirs[i - 1].d, dirs[i].d)))) * R2D;
    return sweep / (dirs[dirs.length - 1].ct - dirs[0].ct);
  };
  approx(rate(withPose), 0, 0.02, "pose-path waypoints: a hovering object reads ~0°/s through a 2°/s pan");
  ok(rate(withoutPose) > 1.5, `static assumption reads the pan as motion (${rate(withoutPose).toFixed(2)}°/s) — the error the fix removes`);
  // absolute accuracy: each pose-path direction sits on the true object dir
  let worst = 0;
  withPose.forEach((d, i) => {
    const g = objAt(i * 0.5);
    worst = Math.max(worst, Math.acos(Math.min(1, Math.max(-1, dot(d.d, dirFromAzEl(g.az, g.el))))) * R2D);
  });
  approx(worst, 0, 0.05, "pose-path directions sit on the true object (deg worst)");

  // gap-aware kinematics: the straight-line jump across a blind stretch must
  // not enter path/avg/acceleration — two clusters moving at 10 m/s with an
  // 18 s hole between them used to average path/24 s ≈ misleadingly slow
  const times = [], pos = [];
  for (let t = 0; t <= 3 + 1e-9; t += 0.5) { times.push(t); pos.push([10 * t, 0, 50]); }
  for (let t = 21; t <= 24 + 1e-9; t += 0.5) { times.push(t); pos.push([400 + 10 * (t - 21), 100, 50]); }
  const kGap = kinematics(times, pos, { maxSegDt: 2 });
  approx(kGap.avgSpeed, 10, 0.01, "average speed uses measured segments only (m/s)");
  approx(kGap.path, 60, 0.5, "path length excludes the unseen jump (m)");
  const kOld = kinematics(times, pos);
  ok(kOld.avgSpeed < 20 && kOld.path > 350, "legacy no-option behavior unchanged (jump included)");
}

// --- Geometric clock sync: capture clocks lie; the object's motion doesn't ---
// Field case: one video's app-captured time was ~20 min wrong; hand-corrected
// to the minute it still sat ~41 s off (proven against the drone's own log).
// The stereo pipeline must recover decisive offsets from geometry alone —
// and must NOT invent a shift when the minimum is flat (a hovering object
// fits every offset equally).
{
  head("geometric clock sync");
  const ref = { lat: 42.164, lon: -123.648, alt: 0 };
  const truthAt = (t) => (t <= 10 ? [100 + 10 * t, 300, 50] : [200, 300 + 12 * (t - 10), 50]);
  const obsDef = [{ ...ref }, { ...geoFromEnu([300, 0, 0], ref), alt: 0 }];
  const azelFor = (o, t, tr = truthAt) => {
    const g = geoFromEnu(tr(t), ref);
    return dirToAzEl(unit(enuFromGeo(g.lat, g.lon, g.alt, o)));
  };
  const WHEN = Date.parse("2026-08-01T17:20:00Z");
  const mkSrc = (o, name, whenErrS, tr = truthAt) => {
    const track = [];
    for (let t = 0; t <= 20 + 1e-9; t += 0.5) track.push({ t: +t.toFixed(1), ...azelFor(o, t, tr) });
    // the witness's CLAIMED capture time is wrong by whenErrS
    return { name, lat: o.lat, lon: o.lon, alt: o.alt, whenMs: WHEN + whenErrS * 1000, A: {}, B: {}, track };
  };
  // 25 s clock error → recovered by the ±45 s scan
  {
    const trk = analyzeTracks([mkSrc(obsDef[0], "A", 0), mkSrc(obsDef[1], "B", 25)]);
    ok(!!(trk.stereo && trk.stereo.k), "stereo solves through a 25 s clock error");
    ok(!!(trk.stereo.sync && trk.stereo.sync.applied), "the offset was adopted (decisive minimum)");
    approx(trk.stereo.sync.delta, -25, 0.4, "recovered clock error (s)");
    let worst = 0;
    for (let i = 0; i < trk.stereo.times.length; i++) {
      const tTrue = trk.stereo.times[i] - WHEN / 1000;
      worst = Math.max(worst, mag(sub(trk.stereo.pos[i], truthAt(tTrue))));
    }
    approx(worst, 0, 1.5, "positions on the truth path after sync (m worst)");
  }
  // 20-minute class: tracks don't overlap at all → wide rescue finds them
  {
    const trk = analyzeTracks([mkSrc(obsDef[0], "A", 0), mkSrc(obsDef[1], "B", 1200)]);
    ok(!!(trk.stereo && trk.stereo.k && trk.stereo.sync && trk.stereo.sync.applied && trk.stereo.sync.rescued),
      "a 20-minute capture-time error is rescued by the wide search");
    approx(trk.stereo.sync.delta, -1200, 0.4, "recovered 20-minute error (s)");
  }
  // hover: every offset fits equally — the sync must refuse to invent one
  {
    const hover = () => [150, 300, 40];
    const trk = analyzeTracks([mkSrc(obsDef[0], "A", 0, hover), mkSrc(obsDef[1], "B", 25, hover)]);
    ok(!(trk.stereo && trk.stereo.sync && trk.stereo.sync.applied), "a hovering object (flat minimum) adopts NO shift");
  }
  // clocks agreed all along → no shift invented, path solves as before
  {
    const trk = analyzeTracks([mkSrc(obsDef[0], "A", 0), mkSrc(obsDef[1], "B", 0)]);
    ok(!!(trk.stereo && trk.stereo.k) && !(trk.stereo.sync && trk.stereo.sync.applied), "aligned clocks pass through untouched");
  }
}

// --- Headless analysis engine: the whole pipeline, no UI, one call ---
// The core of API access (/api/analyze, scripts/analyze.mjs): a session's
// measurements + a flight-log CSV in, every solver's verdict out. Driven by
// the same synthetic truth as the clock-sync section, so the expected
// numbers are exact.
{
  head("headless analysis engine");
  const ref = { lat: 42.164, lon: -123.648, alt: 0 };
  const truthAt = (t) => (t <= 10 ? [100 + 10 * t, 300, 50] : [200, 300 + 12 * (t - 10), 50]);
  const obsDef = [{ ...ref }, { ...geoFromEnu([300, 0, 0], ref), alt: 0 }];
  const WHEN = Date.parse("2026-08-01T17:20:00Z");
  const azelFor = (o, t) => {
    const g = geoFromEnu(truthAt(t), ref);
    return dirToAzEl(unit(enuFromGeo(g.lat, g.lon, g.alt, o)));
  };
  const mkSrc = (o, name, whenErrS) => {
    const track = [];
    for (let t = 0; t <= 20 + 1e-9; t += 0.5) track.push({ t: +t.toFixed(1), ...azelFor(o, t) });
    const ae = azelFor(o, 5);
    // mark at video t=5 → stated mark moment = whenMs + 5 s; truth range at
    // t=5 is 339 m for both (symmetric), so a 0.9 m span subtends 0.152°
    return { name, lat: o.lat, lon: o.lon, alt: o.alt, whenMs: WHEN + whenErrS * 1000,
      A: { az: ae.az.toFixed(2), el: ae.el.toFixed(2), videoTime: 5, angManual: "0.152" }, B: {}, track };
  };
  // Airdata-style CSV from the same truth (10 Hz, UTC clock, MSL altitude)
  const fmtUtc = (ms) => { const d = new Date(ms), p = (v) => String(v).padStart(2, "0"); return `${d.getUTCFullYear()}-${p(d.getUTCMonth() + 1)}-${p(d.getUTCDate())} ${p(d.getUTCHours())}:${p(d.getUTCMinutes())}:${p(d.getUTCSeconds())}`; };
  const rows = ["time(millisecond),datetime(utc),latitude,longitude,altitude_above_seaLevel(feet)"];
  for (let t = 0; t <= 20 + 1e-9; t += 0.1) {
    const g = geoFromEnu(truthAt(t), ref);
    rows.push(`${Math.round(t * 1000)},${fmtUtc(WHEN + Math.floor(t) * 1000)},${g.lat.toFixed(7)},${g.lon.toFixed(7)},${(g.alt / 0.3048).toFixed(1)}`);
  }
  const v = analyzeSession({
    sources: [mkSrc(obsDef[0], "A", 0), mkSrc(obsDef[1], "B", 25)],
    flightLogText: rows.join("\n"), spanM: 0.9,
  });
  ok(v.fix?.ok && v.fix.rating === "excellent", `engine fix solves (${v.fix?.rating})`);
  ok(v.trackStereo?.solved && v.trackStereo.clockSync?.applied, "engine stereo solves and recovers the 25 s clock error");
  approx(v.trackStereo.clockSync.deltaS, -25, 0.4, "engine-reported sync delta (s)");
  ok(v.flightLog?.ok && v.flightLog.calibration?.grade === "excellent", `engine calibration grade on perfect data (${v.flightLog?.calibration?.grade})`);
  approx(v.flightLog.calibration.fixErrM, 0, 3, "engine fix-vs-log error (m)");
  const badClock = v.flightLog.clocks.find((c) => c.name === "B");
  ok(badClock && badClock.sharp && Math.abs(badClock.offsetS + 25) < 1, `engine clock check catches B's 25 s error (${badClock?.offsetS} s)`);
  ok(v.warnings.some((w) => /clock/.test(w)), "engine surfaces the clock warning");
  // incomplete session → named gaps, no invented fix
  const v2 = analyzeSession({ sources: [{ name: "X", lat: 42.1, lon: -123.6, A: {}, B: {}, track: [] }] });
  ok(v2.fix && v2.fix.ok === false && v2.sources[0].missing.length === 1, "engine names what an incomplete witness is missing");
  // LEAN API sessions: the app always creates A/B objects, an external caller
  // may not — a source with no B (or no A at all) must analyze, not throw
  // (found by a live smoke test: analyze() crashed on o.s.B.az)
  const v3 = analyzeSession({ sources: [
    { name: "A", lat: 42.4, lon: -123.3, alt: 400, A: { az: 90, el: 30 }, whenMs: 1754000000000 },
    { name: "B", lat: 42.4, lon: -123.29, alt: 400, A: { az: 270, el: 35 }, whenMs: 1754000000000 },
    { name: "C", lat: 42.41, lon: -123.31 },
  ] });
  ok(v3.fix?.ok === true, "engine: lean two-witness session (no B objects) solves a fix");
  ok(v3.sources[2].missing.length === 1 && /sight-line/.test(v3.sources[2].missing[0]), "engine: a source with no A at all filters as missing, not a crash");
}

// --- Close-up pixel pin (pinStep): a poor track must not break the lock ---
// Field failure (real close-up export): the v2 clutter gate compared every
// find against the TRACK — the thing the pin exists to correct — so ~1° of
// track error rejected the pin's own correct finds and the miss path eased
// the camera back onto the bad track (object wandered ±20% of the frame and
// left it). This drives the REAL pinStep + pinFind over synthetic frames.
{
  head("close-up pixel pin");
  const natW = 1920, natH = 1080, FPS = 30, DUR = 12;
  const pose = { az: 252, el: 20.5, roll: 0, fov: 60, k: 0 };
  let seed = 12345;
  const rnd = () => (seed = (seed * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff - 0.5;
  const truthAzEl = (t) => ({ az: 250 + 3.2 * Math.sin(t * 0.5) + 0.16 * t, el: 20 + 1.8 * Math.sin(t * 0.31 + 1) });
  // 4 Hz track with a POOR solve: 0.2° bias + 1.2° smooth wander + noise
  const trk = [];
  for (let t = 0; t <= DUR + 1e-9; t += 0.25) {
    const g = truthAzEl(t);
    trk.push({ t, az: g.az + 0.2 + 1.2 * Math.sin(t * 0.9 + 2) + rnd() * 0.4, el: g.el + 0.84 * Math.sin(t * 1.1) + rnd() * 0.4 });
  }
  const predAt = (t) => {
    let lo = 0, hi = trk.length - 1;
    if (t <= trk[0].t) hi = 0; else if (t >= trk[trk.length - 1].t) lo = trk.length - 1;
    else while (hi - lo > 1) { const m = (lo + hi) >> 1; if (trk[m].t <= t) lo = m; else hi = m; }
    const a = trk[lo], b = trk[hi], u = hi === lo ? 0 : (t - a.t) / Math.max(1e-9, b.t - a.t);
    return { az: a.az + (b.az - a.az) * u, el: a.el + (b.el - a.el) * u };
  };
  const maxAng = 0.35;
  const objPxR = (natW * Math.tan((maxAng * D2R) / 2) / Math.tan((pose.fov * D2R) / 2)) / 2;
  const sampleAt = (t, visible) => (cx, cy, W2) => {
    const g = truthAzEl(t);
    const op = dirToPixK(dirFromAzEl(g.az, g.el), natW, natH, pose.az, pose.el, 0, pose.fov, 0);
    const data = new Uint8ClampedArray(W2 * W2 * 4);
    for (let y = 0; y < W2; y++) for (let x = 0; x < W2; x++) {
      const px = cx - W2 / 2 + x, py = cy - W2 / 2 + y;
      let v = 165 + 20 * (py / natH) + 6 * Math.sin(px * 0.006) + rnd() * 4;
      const d = op && visible ? Math.hypot(px - op.px, py - op.py) : 1e9;
      if (d < objPxR * 2) v -= 55 * Math.max(0, 1 - (d / (objPxR * 2)) ** 2);
      const i = (y * W2 + x) * 4; data[i] = data[i + 1] = data[i + 2] = v; data[i + 3] = 255;
    }
    return data;
  };
  const st = { prevDir: null, missRun: 0, emaDir: null, ok: 0 };
  const o = { natW, natH, pose, maxAng };
  const errs = []; let locks = 0;
  for (let fi = 0; fi < DUR * FPS; fi++) {
    const t = fi / FPS;
    const r = pinStep(st, predAt(t), o, sampleAt(t, true));
    if (r.mode === "lock") locks++;
    const g = truthAzEl(t);
    errs.push(Math.acos(Math.min(1, Math.max(-1, dot(dirFromAzEl(r.az, r.el), dirFromAzEl(g.az, g.el))))) * R2D);
  }
  const rms = Math.sqrt(errs.reduce((a, e) => a + e * e, 0) / errs.length);
  ok(locks === DUR * FPS, `pin locks every frame through a 1.2°-wander track (${locks}/${DUR * FPS})`);
  approx(rms, 0, 0.06, "object stays centered despite the poor track (deg rms)");
  // fade: brief loss world-holds (no drift toward the bad track), long loss
  // releases and GLIDES back to the track — never a snap
  const lastLock = { az: null, el: null };
  { const r = pinStep(st, predAt(DUR - 0.01), o, sampleAt(DUR - 0.01, true)); lastLock.az = r.az; lastLock.el = r.el; }
  let prev = lastLock, maxStep = 0, holds = 0, glides = 0, holdDriftMax = 0;
  for (let fi = 0; fi < 40; fi++) {
    const t = DUR - 0.01;
    const r = pinStep(st, predAt(t), o, sampleAt(t, false));
    if (r.mode === "hold") {
      holds++;
      const d = Math.acos(Math.min(1, Math.max(-1, dot(dirFromAzEl(r.az, r.el), dirFromAzEl(lastLock.az, lastLock.el))))) * R2D;
      holdDriftMax = Math.max(holdDriftMax, d);
    }
    if (r.mode === "glide") glides++;
    const step = Math.acos(Math.min(1, Math.max(-1, dot(dirFromAzEl(r.az, r.el), dirFromAzEl(prev.az, prev.el))))) * R2D;
    maxStep = Math.max(maxStep, step);
    prev = r;
  }
  ok(holds >= 8, `brief fade world-holds the last lock (${holds} hold frames)`);
  ok(holdDriftMax < 0.02, `held frames stay ON the last lock, not the track (max drift ${holdDriftMax.toFixed(3)}°)`);
  ok(glides >= 10, `long fade releases and glides toward the track (${glides} glide frames)`);
  ok(maxStep < 0.35, `no snap anywhere in the fade path (max step ${maxStep.toFixed(3)}°/frame)`);
  // MOVING-OBJECT fade (field case: a mover at ~7°/s tilted after by the
  // phone — a world-frozen hold parted from it immediately and the close-up
  // went empty). A hold must RIDE THE TRACK: pred moves, the held offset
  // stays, so the aim moves with the object.
  {
    const stM = { oAz: 0.31, oEl: -0.12, has: 1, missRun: 0, ok: 5 };
    let maxRide = 0;
    for (let fi = 1; fi <= 5; fi++) {
      const pm = { az: 250 + fi * 0.23, el: 20 + fi * 0.11 };  // the object keeps moving during the fade
      const r = pinStep(stM, pm, o, () => null);               // pixels gone — pure hold
      maxRide = Math.max(maxRide, Math.hypot(r.az - (pm.az + 0.31), r.el - (pm.el - 0.12)));
    }
    ok(maxRide < 1e-9, `a fade on a MOVING object rides the track with the held offset (max dev ${maxRide.toExponential(1)})`);
  }
  // clutter safety: a track 10° wrong must NOT let pixels capture the camera —
  // the acquire window never reaches the object, so the aim stays on the track
  const stBad = { prevDir: null, missRun: 0, emaDir: null, ok: 0 };
  const badPred = { az: truthAzEl(1).az + 10, el: truthAzEl(1).el };
  const rBad = pinStep(stBad, badPred, o, sampleAt(1, true));
  ok(stBad.ok === 0 && Math.abs(rBad.az - badPred.az) < 1e-9, "a wildly wrong track is followed, not overruled (human outranks pixels)");
}

// --- Drone flight-log check: parse → predict → time-sync → calibrate vs truth ---
// A synthetic calibration flight with EXACT ground truth: the drone flies due
// north at 5 m/s, 40 m above two observers. The Airdata-style CSV is generated
// from the truth via geoFromEnu, so every parsed value has a known answer.
{
  head("drone flight-log ground truth");
  const ref = { lat: 42.16380, lon: -123.64800, alt: 120 };
  const T0 = Date.parse("2026-08-01T17:00:00Z");
  const SPD = 5, ALT = 40; // m/s north, m above the observers
  const truthEnu = (tSec) => [120, -100 + SPD * tSec, ALT];
  const fmtUtc = (ms) => {
    const d = new Date(ms), p = (v) => String(v).padStart(2, "0");
    return `${d.getUTCFullYear()}-${p(d.getUTCMonth() + 1)}-${p(d.getUTCDate())} ${p(d.getUTCHours())}:${p(d.getUTCMinutes())}:${p(d.getUTCSeconds())}`;
  };
  const rows = ["time(millisecond),datetime(utc),latitude,longitude,altitude_above_seaLevel(feet),height_above_takeoff(feet),speed(mph),compass_heading(degrees)"];
  for (let i = 0; i <= 120; i++) {
    const t = i * 0.5, g = geoFromEnu(truthEnu(t), ref);
    // datetime truncated to the SECOND like a real Airdata export — the parser
    // must anchor on the first stamp and advance by time(millisecond)
    rows.push(`${i * 500},${fmtUtc(T0 + Math.floor(t) * 1000)},${g.lat.toFixed(7)},${g.lon.toFixed(7)},${(g.alt / 0.3048).toFixed(1)},${(ALT / 0.3048).toFixed(1)},${(SPD / 0.44704).toFixed(2)},0.0`);
  }
  const log = parseFlightLog(rows.join("\n"), "flight.csv");
  ok(log.ok && log.src === "csv" && log.absTime, "Airdata CSV parses (absolute clock)");
  approx(log.n, 121, 0, "row count (pre-GPS rows would be dropped)");
  approx(log.t0Ms, T0, 1, "first timestamp anchored to the UTC datetime");
  approx(log.t1Ms - log.t0Ms, 60000, 1, "sub-second cadence carried by the millisecond clock");
  ok(log.hasAbsAlt && log.hasRelAlt, "both altitude datums detected");
  approx(log.pts[0].speedMs, SPD, 0.01, "speed mph → m/s");
  approx(log.pts[0].altAbsM - ref.alt, ALT, 0.05, "altitude feet → metres (MSL)");

  // interpolation between samples, against exact truth
  const tQ = T0 + 30250;
  const st = logStateAt(log.pts, tQ);
  const stEnu = enuFromGeo(st.lat, st.lon, st.altAbsM, ref);
  const want = truthEnu(30.25);
  approx(mag(sub(stEnu, want)), 0, 0.15, "interpolated state sits on the truth path (m)");
  approx(st.headDeg, 0, 0.5, "interpolated heading");

  // witnesses: sight-lines computed from TRUTH geometry (not via the module),
  // angular sizes for the true 0.202 m span at each true range
  const span = DRONE_PRESETS[0].spanM;
  const tPick = T0 + 30000;
  const gTrue = geoFromEnu(truthEnu(30), ref);
  const obsDefs = [{ name: "A", ...ref }, { name: "B", ...geoFromEnu([300, 0, 0], ref), alt: 120 }];
  const wit = obsDefs.map((o) => {
    const P = enuFromGeo(gTrue.lat, gTrue.lon, gTrue.alt, o);
    const ae = dirToAzEl(unit(P));
    return { name: o.name, lat: o.lat, lon: o.lon, alt: o.alt, A: { az: ae.az, el: ae.el, angManual: 2 * Math.atan(span / 2 / mag(P)) * R2D }, B: {} };
  });
  const m = logMomentPer(wit, log.pts, tPick, span);
  approx(m.sepMax, 0, 0.05, "drone planted on both sight-lines → sep ~0°");
  const rng0 = m.per[0].rangeM;
  approx(m.per[0].predAng, 2 * Math.atan(span / 2 / rng0) * R2D, 0.02, "predicted angular size from the preset span");

  // clock skew: stated sighting time 7.5 s late → the stated moment misses,
  // the whole-log scan recovers the true instant (the drone's own motion is
  // the shared signal, as in stereoVideo's auto-sync)
  const stated = logMomentPer(wit, log.pts, tPick + 7500, span);
  ok(stated.sepMax > 3, `a 7.5 s clock error is visible (${stated.sepMax.toFixed(1)}° off)`);
  const best = syncLogTime(wit, log.pts, span);
  approx(best.tMs, tPick, 300, "whole-log sync recovers the true instant (ms)");

  // full calibration: a real analyze() fix graded against the log
  const fix = analyze(wit);
  ok(fix.ok, "two-witness fix solves");
  const sum = calibrationSummary({ sources: wit, fix, pts: log.pts, tMs: tPick, spanM: span });
  approx(sum.fixCmp.errM, 0, 3, "fix-vs-log 3D position error (m)");
  approx(sum.fixCmp.sizeRatio, 1, 0.05, "triangulated size / true span");
  const grade = gradeCalibration(sum);
  ok(grade.overall === "excellent", `calibration grade on perfect data: ${grade.overall}`);

  // relative-altitude-only logs: the datum choice is explicit and honest
  const relPts = log.pts.map(({ altAbsM, ...p }) => p);
  const dHome = droneAltM(relPts[0], ref.alt, null);
  approx(dHome.altM - ref.alt, ALT, 0.05, "rel-alt + home elevation → true MSL");
  const dAssumed = droneAltM(relPts[0], null, 120);
  ok(dAssumed.altAssumed === true, "no home elevation → assumption is flagged, not hidden");
  approx(dAssumed.altM - 120, ALT, 0.05, "assumed datum uses the observer's elevation");

  // DJI SRT captions (bracket format, including DJI's own 'longtitude' typo)
  const srt = [
    "1", "00:00:00,000 --> 00:00:00,033",
    "<font size=\"28\">FrameCnt: 1, DiffTime: 33ms", "2026-08-01 10:00:00.000",
    "[iso: 100] [latitude: 42.16400] [longtitude: -123.64750] [rel_alt: 30.000 abs_alt: 150.200]</font>",
    "", "2", "00:00:01,000 --> 00:00:01,033",
    "<font size=\"28\">FrameCnt: 31, DiffTime: 33ms", "2026-08-01 10:00:01.000",
    "[iso: 100] [latitude: 42.16410] [longtitude: -123.64750] [rel_alt: 31.000 abs_alt: 151.200]</font>",
  ].join("\n");
  const sl = parseFlightLog(srt, "clip.srt");
  ok(sl.ok && sl.src === "srt" && sl.n === 2, "DJI SRT captions parse");
  approx(sl.pts[0].lat, 42.164, 1e-9, "SRT latitude");
  approx(sl.pts[0].altAbsM, 150.2, 1e-9, "SRT abs_alt");
  approx(sl.pts[0].altRelM, 30, 1e-9, "SRT rel_alt");
  approx(sl.t1Ms - sl.t0Ms, 1000, 1, "SRT datetimes carry the clock");
  const slSt = logStateAt(sl.pts, sl.t0Ms + 500);
  ok(Number.isFinite(slSt.speedMs) && slSt.speedMs > 5 && slSt.speedMs < 20, `SRT speed derived from positions (${slSt.speedMs.toFixed(1)} m/s)`);

  // persistence downsample keeps the ends
  const thin = thinLog(log.pts, 50);
  ok(thin.length === 50 && thin[0].tMs === log.t0Ms && thin[49].tMs === log.t1Ms, "thinLog keeps first/last at the target count");

  // non-simultaneous witnesses (first field flight: photos 122 s apart): a
  // witness whose photo captured the drone at ITS OWN time must grade ~0°
  // there even when the joint match instant sits elsewhere on the path
  {
    const tLate = tPick + 20000; // drone has moved 100 m north by then
    const gLate = geoFromEnu(truthEnu(50), ref);
    const oLate = obsDefs[1];
    const PL = enuFromGeo(gLate.lat, gLate.lon, gLate.alt, oLate);
    const aeL = dirToAzEl(unit(PL));
    const wit2 = [
      { ...wit[0], whenMs: tPick, B: {} },
      { name: "B", lat: oLate.lat, lon: oLate.lon, alt: oLate.alt, whenMs: tLate, A: { az: aeL.az, el: aeL.el }, B: {} },
    ];
    const s2 = calibrationSummary({ sources: wit2, fix: null, pts: log.pts, tMs: tPick, spanM: span });
    ok(s2.per[0].ownSep == null, "own-time grading: a witness at the joint instant gets no redundant entry");
    ok(s2.per[1].sep > 3, `own-time grading: the moved drone is visibly off the late witness at the joint instant (${s2.per[1].sep.toFixed(1)}°)`);
    approx(s2.per[1].ownSep, 0, 0.05, "own-time grading: the late witness grades ~0° at its own photo time");
  }

  // per-witness clock check: a witness whose stated time is 41 s off must be
  // caught SHARPLY (the drone is moving); the same witness against a hover
  // must NOT be indicted (a flat minimum can't blame the clock)
  {
    const wErr = { ...wit[0], whenMs: tPick - 41000, A: { ...wit[0].A, videoTime: 0 } };
    const ck = witnessClockCheck(wErr, log.pts, span);
    ok(ck && ck.sharp, "a 41 s clock error is caught with a sharp minimum");
    approx(ck.dtS, 41, 0.5, "clock-error size recovered (s)");
    approx(ck.bestSep, 0, 0.05, "the sight-line matches the drone at the corrected time");
    ok(witnessStatedMs({ whenMs: 1000, A: { videoTime: 11.5 } }) === 12500, "stated moment = capture + video t");
    const hoverPts = log.pts.map((p) => ({ ...p, lat: log.pts[60].lat, lon: log.pts[60].lon, altAbsM: log.pts[60].altAbsM }));
    const ckH = witnessClockCheck(wErr, hoverPts, span);
    ok(!ckH || !ckH.sharp, "a hovering drone (flat minimum) does not indict the clock");
  }

  // a raw (encrypted, binary) DJI FlightRecord .txt is named for what it is,
  // not mis-diagnosed as a CSV with missing columns — the shape verified
  // against a real Mini record read the way FileReader.readAsText would
  const bin = parseFlightLog(")  " + "�".repeat(400), "FlightRecord_20260714_191224.txt");
  ok(!bin.ok && bin.binary === true && /encrypted/.test(bin.error), "encrypted FlightRecord .txt → honest binary rejection");

  // timezone-honest datetime parsing
  approx(parseWhen("2026-08-01 17:00:00", true), Date.parse("2026-08-01T17:00:00Z"), 0, "parseWhen assumeUtc");
  approx(parseWhen("2026-08-01T17:00:00+00:00"), Date.parse("2026-08-01T17:00:00Z"), 0, "parseWhen explicit zone wins");
}

// --- share-bundle ZIP (src/report/zip.js) — shared by the app's export and
// the server's phase-2 ingest, so a server bundle must be byte-compatible
// with what the app reads. CRC vector is the classic check value; the
// roundtrip goes through the module's own reader; python's zipfile
// independently validated a real produced bundle during development.
{
  const { crc32buf, makeZip, strU8, unzipEntryText, unzipBinEntries } = await import("../src/report/zip.js");
  approx(crc32buf(strU8("123456789")), 0xCBF43926, 0, "zip: crc32 check vector (123456789 → 0xCBF43926)");
  const media = new Uint8Array(300); for (let i = 0; i < 300; i++) media[i] = (i * 37) & 255;
  const z = makeZip([
    { name: strU8("sighting.phodar.json"), data: strU8('{"phodar":1,"sources":[]}') },
    { name: strU8("videos/observer-1-original.mp4"), data: media },
  ]);
  approx(z[0], 0x50, 0, "zip: local header magic");
  const eocd = z.length - 22;
  approx(z[eocd] | (z[eocd + 1] << 8) | (z[eocd + 2] << 16) | (z[eocd + 3] << 24), 0x06054b50 | 0, 0, "zip: EOCD record at the tail");
  ok(unzipEntryText(z, "sighting.phodar.json") === '{"phodar":1,"sources":[]}', "zip: text entry roundtrips");
  const vids = unzipBinEntries(z, "videos/");
  ok(vids.length === 1 && vids[0].bytes.length === 300 && vids[0].bytes[299] === media[299], "zip: binary entry roundtrips under its prefix");
}

// --- 🛣 road overlay (src/roads.js) — Overpass parse → ENU polylines →
// sight-lines with DEM elevations + approximate occlusion. Synthetic truth:
// a straight primary road due north on a flat plane (el must match
// -atan(eye/d) with the curvature term), plus a hill that must hide the far
// stretch of an east-running residential road while the near stretch stays.
{
  const { parseOverpassRoads, roadSightlines, roadElOf } = await import("../src/roads.js");
  const obsLat = 42, obsLon = -122;
  const mLat = 111320, mLon = 111320 * Math.cos(42 * Math.PI / 180);
  const wayN = { type: "way", tags: { highway: "primary", name: "Test Hwy" }, geometry: [] };
  for (let d = 20; d <= 2000; d += 60) wayN.geometry.push({ lat: obsLat + d / mLat, lon: obsLon });
  const wayE = { type: "way", tags: { highway: "residential" }, geometry: [] };
  for (let d = 20; d <= 4000; d += 100) wayE.geometry.push({ lat: obsLat, lon: obsLon + d / mLon });
  const parsed = parseOverpassRoads({ elements: [wayN, wayE, { type: "node" }] }, obsLat, obsLon, { maxM: 2600 });
  ok(parsed.n === 2 && parsed.shown === 2, `roads: both ways parsed and shown (${parsed.shown}/${parsed.n})`);
  ok(parsed.roads.every((r) => r.pts.every(([e, n]) => Math.hypot(e, n) <= 2600)), "roads: ways clipped to maxM");
  const north = parsed.roads.find((r) => r.major);
  ok(!!north && Math.abs(north.pts[0][0]) < 1 && north.pts[0][1] > 10, "roads: ENU conversion (due-north primary has e≈0, major flag set)");

  const flat = () => 100; // flat plane at 100 m MSL, observer ground = 100 m
  const polysF = roadSightlines(parsed, flat, 100, { eyeM: 1.6 });
  const pN = polysF.find((p) => p.major);
  ok(!!pN && pN.v.every((p) => p.az < 0.5 || p.az > 359.5), "roads: due-north road projects at az≈0");
  const far = pN.v[pN.v.length - 1];
  const wantEl = -Math.atan2(1.6 + (far.d * far.d * 0.87) / (2 * 6371000), far.d) * 180 / Math.PI;
  approx(roadElOf(far, 101.6), wantEl, 0.02, "roads: flat-plane el = -atan(eye/d) with the curvature+refraction term");
  ok(roadElOf(pN.v[0], 101.6) < roadElOf(far, 101.6), "roads: nearer pavement sits farther below the horizon");

  // RIBBON: edges at the road's real half-width, converging with distance —
  // the road you stand on must be a wide wedge at the feet, a thread far out
  ok(Array.isArray(pN.L) && Array.isArray(pN.R) && pN.L.length === pN.v.length, "roads: ribbon carries L/R edge polylines");
  const spanAt = (poly, idx) => Math.abs(((poly.R[idx].az - poly.L[idx].az + 540) % 360) - 180);
  const wantSpan0 = 2 * Math.atan2(pN.w / 2, pN.v[0].d) * 180 / Math.PI; // primary default width 10 m
  approx(spanAt(pN, 0), wantSpan0, 0.2, `roads: ribbon width at the feet = 2·atan(w/2 ÷ d) (${wantSpan0.toFixed(1)}°)`);
  ok(spanAt(pN, 0) > 8 * spanAt(pN, pN.v.length - 1), "roads: edges converge with distance (true perspective)");

  // NEAR-FIELD GROUND LOCK: a DEM-grid wobble at 60 m must not float the
  // road the observer stands on — near gz locks to the observer's ground,
  // the real DEM takes over beyond the blend ramp
  const noisy = (e, n) => (Math.hypot(e, n) < 100 ? 108 : 96);
  const polysN = roadSightlines(parsed, noisy, 100, { eyeM: 1.6 });
  const pNN = polysN.find((p) => p.major);
  const nearV = pNN.v.find((p) => p.d > 40 && p.d < 90);
  approx(nearV.gz, 100, 0.5, "roads: near-field DEM wobble locked to the observer's ground");
  approx(pNN.v[pNN.v.length - 1].gz, 96, 0.1, "roads: real DEM elevation beyond the lock ramp");

  // a 40 m hill spanning 800–1000 m due east hides the road beyond it
  const hill = (e, n) => (e > 800 && e < 1000 && Math.abs(n) < 200 ? 140 : 100);
  const polysH = roadSightlines(parsed, hill, 100, { eyeM: 1.6 });
  const vtxEastFlat = polysF.filter((p) => !p.major).reduce((a, p) => a + p.v.length, 0);
  const vtxEastHill = polysH.filter((p) => !p.major).reduce((a, p) => a + p.v.length, 0);
  ok(vtxEastHill < vtxEastFlat, `roads: the hill occludes the far stretch (${vtxEastHill} < ${vtxEastFlat} vertices)`);
  ok(polysH.some((p) => !p.major && p.v.length >= 2 && p.v[0].d < 700), "roads: the near stretch in front of the hill still draws");
}

// --- 🔬 authenticity checks (src/checks/authenticity.js) — synthetic files
// carrying real-world fingerprints must trip the right level; a clean buffer
// must trip nothing (a false AI alarm is worse than a miss).
{
  const { scanFileAuthenticity, pngTextChunks, jpegMarkers, authDerived, authSummary } = await import("../src/checks/authenticity.js");
  const enc = (s) => new TextEncoder().encode(s);

  ok(scanFileAuthenticity(enc("....Steps: 20, Sampler: Euler a, CFG scale: 7...."), "image").some((x) => x.id === "ai-params" && x.level === "alarm"), "auth: SD parameter block → alarm");
  ok(scanFileAuthenticity(enc("xmp:CreatorTool=Midjourney v6"), "image").some((x) => x.level === "alarm" && /Midjourney/.test(x.label)), "auth: Midjourney marker → alarm");

  const pngChunk = (type, body) => {
    const b = enc(body), out = new Uint8Array(12 + b.length);
    out[0] = (b.length >>> 24) & 255; out[1] = (b.length >>> 16) & 255; out[2] = (b.length >>> 8) & 255; out[3] = b.length & 255;
    out.set(enc(type), 4); out.set(b, 8);
    return out; // crc bytes left zero — the walker doesn't verify them
  };
  const png = new Uint8Array([0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a,
    ...pngChunk("tEXt", "parameters masterpiece, Steps: 30"), ...pngChunk("IEND", "")]);
  const pf2 = scanFileAuthenticity(png, "image");
  ok(pngTextChunks(png).some((c) => c.key === "parameters"), "auth: PNG tEXt chunk parsed");
  ok(pf2.some((x) => x.id === "png-genmeta" && x.level === "alarm"), "auth: PNG parameters chunk → alarm");
  ok(pf2.some((x) => x.id === "png" && x.level === "note"), "auth: PNG container noted (cameras write JPEG/HEIC)");

  const jpg = new Uint8Array([0xFF, 0xD8, 0xFF, 0xEC, 0x00, 0x07, ...enc("Ducky"), 0xFF, 0xC2, 0x00, 0x04, 0x00, 0x00, 0xFF, 0xD9]);
  const jm = jpegMarkers(jpg);
  ok(jm && jm.ducky && jm.progressive, "auth: JPEG marker walk finds Ducky + progressive SOF2");
  const jf = scanFileAuthenticity(jpg, "image");
  ok(jf.some((x) => x.id === "ducky" && x.level === "warn") && jf.some((x) => x.id === "progressive"), "auth: Save-for-Web warn + progressive note");

  ok(scanFileAuthenticity(enc("<x:xmp> Adobe Photoshop 25.0 </x:xmp>"), "image").some((x) => x.id === "edited" && x.level === "warn"), "auth: Photoshop fingerprint → warn");
  ok(scanFileAuthenticity(enc("....Lavf60.3.100...."), "video").some((x) => x.id === "ffmpeg" && x.level === "warn"), "auth: ffmpeg (Lavf) container → re-encode warn");
  ok(scanFileAuthenticity(enc("com.apple.quicktime.model=iPhone 14"), "video").some((x) => x.id === "apple-orig" && x.level === "info"), "auth: Apple camera-original keys → positive info");
  ok(scanFileAuthenticity(new Uint8Array([1, 2, 3, 4, 5, 6, 7, 8]), "image").length === 0, "auth: a clean buffer trips nothing");

  // derived physics: the sun cannot be argued with
  const base = { lat: "42.3", lon: "-122.9", authLum: { mean: 150, p90: 220 } };
  const night = Date.UTC(2026, 7, 10, 9, 30);  // ≈ 02:30 local — astronomical night
  const day = Date.UTC(2026, 7, 10, 20, 0);    // ≈ 13:00 local — high sun
  ok(authDerived({ ...base, whenMs: night }).some((x) => x.id === "sun-night" && x.level === "warn"), "auth: bright scene at astronomical night → warn");
  ok(authDerived({ ...base, whenMs: day }).some((x) => x.id === "sun-ok" && x.level === "info"), "auth: daylight brightness consistent → positive");
  ok(authDerived({ ...base, whenMs: day, meta: { timeMs: day - 26 * 3600000 } }).some((x) => x.id === "time-mismatch"), "auth: stated time 26 h from file clock → note");
  ok(authDerived({ ...base, whenMs: day, meta: { lat: 43.5, lon: -122.9 } }).some((x) => x.id === "gps-mismatch"), "auth: stated position ~130 km from file GPS → note");
  ok(authDerived({ ...base, whenMs: day, calib: { method: "stars", rms: 0.1 } }).some((x) => x.id === "cal-stars" && x.level === "info"), "auth: star-verified pointing → positive");

  const sum = authSummary(pf2);
  ok(sum.worst === "alarm" && sum.alarms.length === 1, "auth: summary surfaces the alarm for the report banner");
}

// --- ⚑ sun-shadow gadget (src/shadow.js) — exact geometry on flat ground
{
  const { poleShadow, poleDist, POLE_H } = await import("../src/shadow.js");
  // pole due east of the eye, sun due south at 45°: shadow falls due north, length = H
  const g = poleShadow({ az: 90, camH: 1.6, sunAz: 180, sunAlt: 45, D: 25, H: 5 });
  ok(g.pole.every((p) => Math.abs(p.az - 90) < 1e-9), "shadow: a vertical pole holds one azimuth top to bottom");
  approx(g.base.el, Math.atan2(-1.6, 25) * R2D, 1e-9, "shadow: base sits on the ground plane");
  approx(g.top.el, Math.atan2(5 - 1.6, 25) * R2D, 1e-9, "shadow: top at pole height above it");
  approx(g.len, 5, 1e-9, "shadow: 45° sun ⇒ shadow length = pole height");
  approx(g.dir, 0, 1e-9, "shadow: falls due north, away from a south sun");
  const tip = g.shadow[g.shadow.length - 1]; // ground point 25 m E + 5 m N of the eye
  approx(tip.az, Math.atan2(25, 5) * R2D, 1e-6, "shadow: tip azimuth from exact ground position");
  approx(tip.el, Math.atan2(-1.6, Math.hypot(25, 5)) * R2D, 1e-6, "shadow: tip elevation on the ground plane");
  ok(g.shadow.every((p) => p.el < 0) && !g.clipped, "shadow: whole shadow lies below the horizon, uncapped");
  // low sun stretches it; a grazing sun is capped for drawing but reported true
  const lo = poleShadow({ az: 0, camH: 1.6, sunAz: 270, sunAlt: 10, D: 25, H: 5 });
  approx(lo.len, 5 / Math.tan(10 * D2R), 1e-9, "shadow: 10° sun ⇒ H/tan(alt)");
  approx(lo.dir, 90, 1e-9, "shadow: west sun throws east");
  const gz = poleShadow({ az: 0, camH: 1.6, sunAz: 180, sunAlt: 0.08, D: 25, H: 5 });
  ok(gz.clipped && gz.len > 2500, "shadow: grazing sun capped for drawing, true length reported");
  // sun down ⇒ no shadow, pole still drawn; rooftop camera keeps the base in view
  const dn = poleShadow({ az: 0, camH: 1.6, sunAz: 0, sunAlt: -3 });
  ok(dn.shadow === null && dn.pole.length === 9, "shadow: sun below horizon ⇒ pole only, no fabricated shadow");
  approx(poleDist(100), 300, 1e-9, "shadow: pole distance scales with camera height");
  approx(poleShadow({ az: 45, camH: 100, sunAz: 180, sunAlt: 45 }).base.el, Math.atan2(-100, 300) * R2D, 1e-9, "shadow: rooftop base stays in a drawable band");
  ok(POLE_H === 5, "shadow: pole height is the documented 5 m");

  // gaze-following standoff: the pole stands where the view-center ray meets the ground
  const { poleDistView, shadowCast } = await import("../src/shadow.js");
  approx(poleDistView(1.6, -Math.atan2(1.6, 25) * R2D), 25, 1e-6, "shadow: looking at the 25 m ground point puts the pole there");
  approx(poleDistView(1.6, -45), 6, 1e-9, "shadow: steep look-down floors at 6 m, never underfoot");
  approx(poleDistView(1.6, 10), 300, 1e-9, "shadow: looking up parks the pole at the far cap");
  approx(poleDistView(100, -18.435), 300, 0.1, "shadow: rooftop gaze-to-ground matches the geometry");

  // light-source arbitration: sun, else moon (with % lit honesty), else nothing
  const dayC = shadowCast(Date.UTC(2026, 7, 10, 20, 0), 42.3, -122.9);   // high sun
  ok(dayC.kind === "sun" && Math.abs(dayC.ratio - 1 / Math.tan(dayC.alt * D2R)) < 1e-9, "shadow: daytime → sun casts, ratio = 1/tan(alt)");
  approx(dayC.dir, (dayC.az + 180) % 360, 1e-9, "shadow: cast direction opposite the source");
  const moonC = shadowCast(Date.UTC(2026, 7, 1, 6, 45), 42.3, -122.9);   // sun down, 93%-lit moon at 21°
  ok(moonC.kind === "moon" && moonC.frac > 0.85 && !moonC.dim, "shadow: bright moon after sunset → moonlight shadow, not dim");
  const noneC = shadowCast(Date.UTC(2026, 7, 10, 9, 30), 42.3, -122.9);  // astronomical night, moon down
  ok(noneC.kind === null && noneC.sunAlt < -18, "shadow: sun and moon both down → nothing casts (any crisp shadow contradicts the time)");

  // sundial inversion: recover the capture time from a shadow direction
  const { shadowTimes } = await import("../src/shadow.js");
  const t0 = Date.UTC(2026, 7, 10, 18, 0); // 11:00 PDT
  const s0 = sunPos(t0, 42.3, -122.9), dir0 = (s0.az + 180) % 360;
  const hits = shadowTimes(Date.UTC(2026, 7, 10, 20, 0), 42.3, -122.9, dir0);
  ok(hits.length === 1, "sundial: one sun-up instant matches a morning shadow direction");
  ok(Math.abs(hits[0].ms - t0) < 60000, `sundial: recovered the capture time to <1 min (off by ${Math.round(Math.abs(hits[0].ms - t0) / 1000)} s)`);
  approx(hits[0].alt, s0.alt, 0.1, "sundial: matched instant carries the right sun altitude");
  approx(hits[0].ratio, 1 / Math.tan(s0.alt * D2R), 0.02, "sundial: length ratio rides along for a second check");
  ok(shadowTimes(Date.UTC(2026, 7, 10, 20, 0), 42.3, -122.9, 180).length === 0, "sundial: a due-south shadow at 42°N matches NO time — the sun is never in the north");
}

// --- ⛰ terrain line-of-sight (rayClearance) — impossible-geometry detector
{
  const { rayClearance } = await import("../src/terrain.js");
  // a 300 m wall crossing the ray 5 km due north of a sea-level observer
  const wall = (e, n) => (n > 4800 && n < 5200 && Math.abs(e) < 3000) ? 300 : 0;
  const blk = rayClearance(wall, 0, 0, 0, 10000);
  ok(blk && blk.dBlock > 4600 && blk.dBlock < 5300, "LOS: a 300 m wall at 5 km blocks a level ray to a 10 km fix");
  ok(blk.belowM > 250 && blk.belowM < 305, `LOS: deficit ≈ the wall's height (${blk.belowM.toFixed(0)} m below the crest)`);
  ok(rayClearance(wall, 0, 0, 0, 4000) === null, "LOS: a fix NEARER than the wall is clear — distance matters, not just the skyline");
  ok(rayClearance(wall, 0, 0, 4, 10000) === null, "LOS: a 4° sight-line clears the wall");
  ok(rayClearance(wall, 0, 90, 0, 10000) === null, "LOS: looking east misses the wall entirely");
  const berm = (e, n) => (n > 4800 && n < 5200) ? 9 : 0;
  ok(rayClearance(berm, 0, 0, 0, 10000) === null, "LOS: a 9 m rise is DEM noise, never declared a wall");
}

// --- 🛣 vehicle-light candidate (roadCrossings) — sight-line × mapped roads
{
  const { roadCrossings } = await import("../src/roads.js");
  // a road running east–west, crossing due north of the observer at ~1015 m
  const road = { name: "Table Rock Rd", major: true, v: [{ az: 350, d: 1015, gz: 0 }, { az: 10, d: 1015, gz: 0 }] };
  const c = roadCrossings([road], 0, 1.6);
  ok(c.length === 1 && c[0].name === "Table Rock Rd", "roads: sight-line due north crosses the east–west road once");
  approx(c[0].d, 1015, 1, "roads: crossing distance interpolated");
  approx(c[0].el, Math.atan2(0 - 1.6 - (1015 * 1015 * 0.87) / (2 * 6371000), 1015) * 180 / Math.PI, 0.01, "roads: crossing elevation via the shared road-el model");
  ok(roadCrossings([road], 90, 1.6).length === 0, "roads: looking east misses it");
  // an S-curve crossing the bearing twice keeps the NEAR crossing
  const scurve = { v: [{ az: 355, d: 500, gz: 0 }, { az: 5, d: 500, gz: 0 }, { az: 355, d: 1500, gz: 0 }, { az: 5, d: 1500, gz: 0 }] };
  approx(roadCrossings([scurve], 0, 1.6)[0].d, 500, 30, "roads: double crossing keeps the nearest");
  ok(roadCrossings([{ v: [{ az: 170, d: 800, gz: 0 }, { az: 190, d: 800, gz: 0 }] }], 0, 1.6).length === 0, "roads: the ±180 wrap is not a fake crossing");
}

// --- 📡 tower/mast strobe candidates (parseMasts / mastsNear)
{
  const { parseMasts, mastsNear } = await import("../src/checks/masts.js");
  const oLat = 42.3, oLon = -122.9;
  const j = { elements: [
    { type: "node", lat: oLat + 5000 / 111320, lon: oLon, tags: { man_made: "mast", name: "KOBI mast", height: "150 m" } },
    { type: "way", center: { lat: oLat, lon: oLon + 9000 / (111320 * Math.cos(oLat * D2R)) }, tags: { man_made: "tower" } },
    { type: "node", lat: oLat + 0.5, lon: oLon, tags: { man_made: "mast", height: "80" } },   // ~55 km — out of range
    { type: "node", lat: oLat + 0.01, lon: oLon, tags: { building: "yes" } },                 // not a tall structure
  ] };
  const m = parseMasts(j, oLat, oLon, 25);
  ok(m.length === 2 && m[0].name === "KOBI mast", "masts: parse keeps mast+tower, drops far + non-structures, nearest first");
  ok(m[0].hM === 150 && m[0].hEst === false && m[1].hM === null, "masts: tagged height parsed, untagged honest null");
  approx(m[0].az, 0, 0.2, "masts: bearing computed (due north)");
  const near = mastsNear(m, 2, 5);
  ok(near.length === 1 && Math.abs(near[0].dAz + 2) < 0.3, "masts: 5° gate keeps the north mast for a 002° sight-line");
  ok(mastsNear(m, 200, 5).length === 0, "masts: opposite bearing matches nothing");
}

// --- ✈ ADS-B track-time matching — trajectory vs trajectory, synthetic truth
{
  const { trailStateAt, trackMatch, trackAbsSamples, acAzElRange } = await import("../src/checks/adsb.js");
  const obs = { lat: 42.3, lon: -122.9, alt: 0 };
  const mLat = 111320, mLon = 111320 * Math.cos(42.3 * D2R);
  // truth: an aircraft 5 km east, flying due north at 100 m/s, 3000 m AMSL
  const acAt = (t) => ({ lat: 42.3 + (100 * t - 2000) / mLat, lon: -122.9 + 5000 / mLon, altM: 3000 });
  const t0Ms = Date.UTC(2026, 7, 10, 20, 0);
  const trail = [];
  for (let t = -120; t <= 120; t += 10) { const p = acAt(t); trail.push([t, p.lat, p.lon, p.altM]); }
  // witness samples generated from the SAME truth through the same geometry
  const samples = [];
  for (let t = -30; t <= 30; t += 5) { const g = acAzElRange(obs, acAt(t)); samples.push({ ms: t0Ms + t * 1000, az: g.az, el: g.el }); }
  const st = trailStateAt(trail, 5);
  approx(st.lat, acAt(5).lat, 1e-9, "trackmatch: trail interpolation exact between samples");
  ok(trailStateAt(trail, 500) === null, "trackmatch: never extrapolates beyond the recorded trail");
  const m = trackMatch(obs, trail, t0Ms, samples);
  ok(m && m.n === 13 && m.overlapS === 60, "trackmatch: all samples land inside the trail window");
  ok(m.meanSep < 0.01 && m.maxSep < 0.01, `trackmatch: the true aircraft scores ≈0° (mean ${m.meanSep.toFixed(4)}°)`);
  // a 30 s clock-shifted decoy (same path, wrong time) must score badly
  const shifted = trail.map(([t, la, lo, al]) => [t + 30, la, lo, al]);
  const md = trackMatch(obs, shifted, t0Ms, samples);
  ok(md && md.meanSep > 15, `trackmatch: a 30 s time-shifted decoy diverges hard (mean ${md.meanSep.toFixed(1)}°)`);
  // a parallel path 2 km further east also diverges — position discriminates too
  const east = trail.map(([t, la, lo, al]) => [t, la, lo + 2000 / mLon, al]);
  const me = trackMatch(obs, east, t0Ms, samples);
  ok(me && me.meanSep > 5, `trackmatch: a parallel offset path diverges (mean ${me.meanSep.toFixed(1)}°)`);
  ok(trackMatch(obs, trail, t0Ms, samples.slice(0, 2)) === null, "trackmatch: <3 overlapping samples → honest null, no verdict");
  // absolute-clock sampling: objPath thinned + q-gated, anchored at whenMs + t
  const src = { whenMs: t0Ms - 10000, objPath: [
    { t: 10, az: 100, el: 20, q: 0.9 }, { t: 11, az: 101, el: 20.5, q: 0.9 },
    { t: 12, az: 102, el: 21, q: 0.1 }, { t: 13, az: 103, el: 21.5, q: 0.8 },
  ] };
  const abs = trackAbsSamples(src);
  ok(abs.length === 3 && abs[0].ms === t0Ms && abs[0].az === 100, "trackmatch: objPath samples on the wall clock, low-q predictions dropped");
}

// --- 🎈 radiosonde (weather-balloon) check — sites, launch windows, ranking
{
  const { parseSites, launchStateAt, balloonDiaM, rankSondes } = await import("../src/checks/sondes.js");
  const { acAzElRange } = await import("../src/checks/adsb.js");
  const sitesJson = {
    "72597": { station_name: "Medford (US)", position: [-122.8822, 42.3769], alt: 405, times: ["0:00:00", "0:12:00"], burst_altitude: 34000, ascent_rate: 5 },
    far: { station_name: "Far away", position: [-100, 30], alt: 0, times: ["0:12:00"] },
  };
  const sites = parseSites(sitesJson, 42.3, -122.9, 250);
  ok(sites.length === 1 && sites[0].name === "Medford (US)" && sites[0].distKm < 12, "sondes: site catalog filtered + ranged (Medford 9 km)");
  ok(sites[0].times.length === 2 && sites[0].times[1].h === 12, "sondes: launch schedule parsed (00Z & 12Z)");
  const site = sites[0];
  // 23:30Z: the 00Z balloon launched at 23:00Z is 30 min up → ~9.4 km ascending
  const D = Date.UTC(2026, 7, 11);
  const up = launchStateAt(site, D + (23 * 60 + 30) * 60000);
  ok(up && up.phase === "asc" && Math.abs(up.altM - (405 + 1800 * 5)) < 1, `sondes: 30 min after launch ⇒ ascending at ${Math.round(up.altM)} m`);
  ok(launchStateAt(site, D + 3 * 3600000 + 3600000) === null, "sondes: hours past the flight ⇒ honestly nothing airborne");
  const ascS = (34000 - 405) / 5;
  const dn = launchStateAt(site, D + 23 * 3600000 + (ascS + 1200) * 1000);
  ok(dn && dn.phase === "desc" && dn.altM < 34000 && dn.altM > 5000, "sondes: after burst ⇒ descending phase with falling altitude");
  approx(balloonDiaM(0), 1.5, 1e-9, "sondes: envelope ~1.5 m at release");
  approx(balloonDiaM(34000, 34000), 7.5, 1e-9, "sondes: envelope ~7.5 m at burst");
  // ranking: a sonde 5 km east at 3 km altitude, witness aimed straight at it
  const mLat = 111320, mLon = 111320 * Math.cos(42.3 * D2R);
  const sd = { serial: "Y123", type: "RS41", track: [[-60, 42.3 - 300 / mLat, -122.9 + 5000 / mLon, 2900], [60, 42.3 + 300 / mLat, -122.9 + 5000 / mLon, 3100]] };
  const g = acAzElRange({ lat: 42.3, lon: -122.9, alt: 0 }, { lat: 42.3, lon: -122.9 + 5000 / mLon, altM: 3000 });
  const cands = rankSondes([{ name: "W", lat: "42.3", lon: "-122.9", alt: 0, A: { az: g.az, el: g.el } }], [sd], Date.UTC(2026, 7, 11, 23, 30));
  ok(cands.length === 1 && cands[0].sepMax < 0.2, `sondes: on-sight-line sonde ranks at ≈0° (${cands[0].sepMax.toFixed(2)}°)`);
  ok(cands[0].predAng > 0.015 && cands[0].predAng < 0.04, `sondes: predicted angular size from the altitude-grown envelope (${cands[0].predAng.toFixed(3)}°)`);
  ok(rankSondes([{ lat: "42.3", lon: "-122.9", A: { az: (g.az + 120) % 360, el: g.el } }], [sd], 0)[0].sepMax > 90, "sondes: a sonde far off the bearing scores far off");
}

// --- 🪖 special-use airspace (MOA) geometry — pure, synthetic square zone
{
  const { parseAltFt, pointInRings, rayIntoZone, parseSua, suaActiveAt } = await import("../src/checks/airspace.js");
  approx(parseAltFt("SFC"), 0, 1e-9, "sua: SFC floor = 0 ft");
  approx(parseAltFt("180", "FL"), 18000, 1e-9, "sua: FL180 ceiling = 18,000 ft");
  approx(parseAltFt("11000", "FT"), 11000, 1e-9, "sua: plain feet pass through");
  // square MOA spanning 20–60 km north of the observer, ±30 km east–west
  const mLat = 111.32, mLon = 111.32 * Math.cos(42.3 * D2R);
  const ring = [[-122.9 - 30 / mLon, 42.3 + 20 / mLat], [-122.9 + 30 / mLon, 42.3 + 20 / mLat],
    [-122.9 + 30 / mLon, 42.3 + 60 / mLat], [-122.9 - 30 / mLon, 42.3 + 60 / mLat], [-122.9 - 30 / mLon, 42.3 + 20 / mLat]];
  ok(!pointInRings(42.3, -122.9, [ring]), "sua: observer south of the zone is outside");
  ok(pointInRings(42.3 + 40 / mLat, -122.9, [ring]), "sua: a point mid-zone is inside");
  const hitN = rayIntoZone([ring], 42.3, -122.9, 0);
  ok(hitN && Math.abs(hitN.enterKm - 20) <= 2 && Math.abs(hitN.exitKm - 60) <= 2, `sua: looking north ENTERS the MOA at ~20 km, leaves ~60 km (${hitN.enterKm}–${hitN.exitKm})`);
  ok(rayIntoZone([ring], 42.3, -122.9, 90) === null, "sua: looking east misses the zone");
  const zones = parseSua({ features: [{ attributes: { NAME: "DOLPHIN NORTH MOA", TYPE_CODE: "MOA", LOWER_VAL: "11000", LOWER_UOM: "FT", UPPER_VAL: "180", UPPER_UOM: "FL", TIMESOFUSE: "0800 - 1600, DAILY" }, geometry: { rings: [ring] } }] }, 42.3, -122.9);
  ok(zones.length === 1 && !zones[0].inside && Math.abs(zones[0].distKm - 20) < 2 && zones[0].floorFt === 11000 && zones[0].ceilFt === 18000, "sua: parse carries altitudes + nearest distance");
  ok(parseSua({ features: [{ attributes: { NAME: "X", TYPE_CODE: "MOA" }, geometry: { rings: [ring] } }] }, 42.3 + 40 / mLat, -122.9)[0].inside, "sua: observer inside flags inside");
  ok(suaActiveAt("0800 - 1600, DAILY", 1230) === true && suaActiveAt("0800 - 1600, DAILY", 2200) === false, "sua: simple schedule window read (12:30 in, 22:00 out)");
  ok(suaActiveAt("BY NOTAM", 1200) === null, "sua: unreadable schedule → honest unknown, never a guess");
}

// --- ✨ aurora / 🏮 lanterns / ✈ contrails — the small-context trio
{
  const { geomagLat, auroraVerdict, kpAt } = await import("../src/checks/aurora.js");
  const gmMed = geomagLat(42.3, -122.9), gmFai = geomagLat(64.8, -147.7);
  ok(gmMed > 46 && gmMed < 51, `aurora: Medford geomagnetic latitude ≈ 48° (${gmMed.toFixed(1)})`);
  ok(gmFai > 62 && gmFai < 68, `aurora: Fairbanks geomagnetic latitude ≈ 65° (${gmFai.toFixed(1)})`);
  ok(auroraVerdict(2, gmMed).level === "unlikely", "aurora: quiet Kp 2 at 48° gm — unlikely");
  ok(auroraVerdict(9, gmMed).level !== "unlikely", "aurora: a Kp 9 superstorm makes aurora visible from Medford");
  ok(auroraVerdict(9, 55).level === "overhead", "aurora: Kp 9 puts the oval overhead at geomagnetic 55°");
  ok(auroraVerdict(6.3, gmMed).level === "horizon", "aurora: Kp 6+ puts a glow on Medford's north horizon");
  ok(auroraVerdict(3, gmFai).level === "overhead", "aurora: Fairbanks sits in the oval at modest Kp");
  const kj = { Kp: [2.0, 5.3, 1.0], datetime: ["2026-08-09T00:00:00Z", "2026-08-09T03:00:00Z", "2026-08-09T06:00:00Z"] };
  approx(kpAt(kj, Date.UTC(2026, 7, 9, 4, 30)), 5.3, 1e-9, "aurora: 04:30 falls in the 03–06 UTC Kp bin");
  ok(kpAt(kj, Date.UTC(2026, 7, 20)) === null, "aurora: far outside the window → honest null");

  const { lanternContext } = await import("../src/checks/lanterns.js");
  ok(/July 4/.test(lanternContext(new Date(2026, 6, 4, 22).getTime())?.event || ""), "lanterns: July 4 night flagged");
  ok(/July 4/.test(lanternContext(new Date(2026, 6, 5, 1).getTime())?.event || ""), "lanterns: small hours of July 5 still count");
  ok(lanternContext(new Date(2026, 6, 12).getTime()) === null, "lanterns: an ordinary July night is not flagged");
  ok(/New Year/.test(lanternContext(new Date(2025, 11, 31, 23).getTime())?.event || ""), "lanterns: New Year's Eve flagged");
  ok(/Lunar New Year/.test(lanternContext(new Date(2026, 1, 17, 20).getTime())?.event || ""), "lanterns: Lunar New Year 2026 (Feb 17) flagged");

  const { contrailVerdict } = await import("../src/checks/weather.js");
  ok(contrailVerdict(78).level === "likely" && contrailVerdict(50).level === "shortlived" && contrailVerdict(15).level === "dry", "contrails: RH bands → likely / short-lived / dry");
  ok(contrailVerdict(null) === null, "contrails: no data → no verdict");
}

// --- 🌙 moon terminator forensic — limb geometry + disc measurement
{
  const { limbTargetPoint, measureLimbAngle, limbVerdict } = await import("../src/checks/moonlimb.js");
  // sun directly above the moon → bearing 0 (toward zenith); to the right → 90
  approx(limbTargetPoint(180, 30, 180, 50).paDeg, 0, 0.5, "moonlimb: sun above ⇒ limb bearing toward zenith");
  approx(limbTargetPoint(180, 30, 182, 30).paDeg, 90, 1, "moonlimb: nearby sun at higher azimuth ⇒ limb bearing ~90°");
  const tp = limbTargetPoint(180, 30, 180, 50);
  ok(tp.alt > 30 && Math.abs(tp.az - 180) < 0.01, "moonlimb: stepped sky point moves toward the sun");
  // synthetic half-lit disc, lit side toward 30° (image convention, y down)
  const W = 100, H = 100, g = new Float32Array(W * H).fill(5);
  const th = 30 * D2R, ux = Math.cos(th), uy = Math.sin(th);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    const dx = x - 50, dy = y - 50;
    if (dx * dx + dy * dy <= 900) g[y * W + x] = (dx * ux + dy * uy) > 0 ? 220 : 25;
  }
  const m = measureLimbAngle(g, W, H, 50, 50, 30);
  ok(m && Math.abs(((m.angle - 30 + 540) % 360) - 180) < 175 && Math.abs(((m.angle - 30 + 540) % 360) - 180) >= 0, "moonlimb: measurement returns an angle");
  approx(((m.angle - 30 + 540) % 360) - 180, 0, 4, "moonlimb: half-lit disc recovers the lit direction to a few degrees");
  ok(m.strength > 0.1, `moonlimb: half-disc asymmetry is strong (${m.strength.toFixed(2)})`);
  // a uniformly-lit (full-moon-like) disc must NOT testify
  const gf = new Float32Array(W * H).fill(5);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) { const dx = x - 50, dy = y - 50; if (dx * dx + dy * dy <= 900) gf[y * W + x] = 220; }
  const mf = measureLimbAngle(gf, W, H, 50, 50, 30);
  ok(mf.strength < 0.03 && limbVerdict(mf.angle, 10, mf.strength, 0.5) === null, "moonlimb: uniform disc → too weak, no verdict");
  ok(limbVerdict(35, 30, 0.2, 0.5).verdict === "match", "moonlimb: 5° agreement → positive match");
  ok(limbVerdict(210, 30, 0.2, 0.5).verdict === "mismatch", "moonlimb: lit side pointing away → mismatch warn");
  ok(limbVerdict(75, 30, 0.2, 0.5) === null, "moonlimb: 45° gray zone → silent, never a guess");
  ok(limbVerdict(210, 30, 0.2, 0.97) === null, "moonlimb: near-full moon → no verdict (no terminator to read)");
  ok(limbVerdict(355, 5, 0.2, 0.5).verdict === "match", "moonlimb: wrap-around 10° agreement still matches");
  // disc locating: the half-disc's bbox center lands near the true center,
  // span reads the true diameter; an empty sky refuses to answer
  const { discCenter } = await import("../src/checks/moonlimb.js");
  const dc = discCenter(g, W, H);
  ok(dc && Math.hypot(dc.cx - 50, dc.cy - 50) < 0.55 * 30 && Math.abs(dc.spanPx - 60) < 8, `moonlimb: disc located (center off by ${Math.hypot(dc.cx - 50, dc.cy - 50).toFixed(0)}px, span ${dc.spanPx})`);
  ok(discCenter(new Float32Array(W * H).fill(6), W, H) === null, "moonlimb: blank sky → no disc, no verdict");
  // center-error-free measurement: the limb axis is the lit region's MINOR
  // principal axis (mirror symmetry), signed by the width taper — no disc
  // center needed. Half, crescent AND gibbous must all recover ~30°.
  const { measureLimbDir } = await import("../src/checks/moonlimb.js");
  const err = (m) => Math.abs(((m.angle - 30 + 540) % 360) - 180);
  const md = measureLimbDir(g, W, H);
  ok(md && err(md) < 2 && md.strength > 0.2, `moonlimb: half-disc limb axis to <2° (Δ${err(md).toFixed(1)}°, strength ${md.strength.toFixed(2)})`);
  const mkPhase = (cut) => {
    const gg = new Float32Array(W * H).fill(5);
    for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
      const dx = x - 50, dy = y - 50;
      if (dx * dx + dy * dy <= 900 && (dx * ux + dy * uy) > cut) gg[y * W + x] = 220;
    }
    return gg;
  };
  const mc = measureLimbDir(mkPhase(12), W, H);
  ok(mc && err(mc) < 2, `moonlimb: crescent limb axis to <2° (Δ${err(mc).toFixed(1)}°)`);
  const mg = measureLimbDir(mkPhase(-18), W, H);
  ok(mg && err(mg) < 2 && mg.strength > 0.12, `moonlimb: gibbous limb axis to <2° (Δ${err(mg).toFixed(1)}°)`);
  const mfull = measureLimbDir(gf, W, H);
  ok(!mfull || mfull.strength < 0.12, "moonlimb: full disc → strength below the verdict gate (no false testimony)");
}

// --- 🔏 C2PA verification interpreter (pure — the SDK never enters it)
{
  const { interpretC2pa } = await import("../src/checks/c2paverify.js");
  const cam = {
    activeManifest: {
      claimGenerator: "Leica FOTOS", claimGeneratorInfo: [{ name: "Leica M11-P" }],
      signatureInfo: { issuer: "Leica Camera AG", time: "2026-08-01T12:00:00Z" },
      ingredients: [], assertions: [],
    },
    validationStatus: [],
  };
  const vc = interpretC2pa(cam);
  ok(vc.length === 1 && vc[0].id === "c2pa-valid" && vc[0].level === "info" && /Leica Camera AG/.test(vc[0].detail), "c2pa: valid camera signature → positive info naming the issuer");
  const tampered = interpretC2pa({ ...cam, validationStatus: [{ code: "assertion.dataHash.mismatch" }] });
  ok(tampered.length === 1 && tampered[0].id === "c2pa-invalid" && tampered[0].level === "alarm" && /dataHash\.mismatch/.test(tampered[0].detail), "c2pa: hash mismatch → tamper ALARM with the failing code");
  const ai = interpretC2pa({
    activeManifest: { claimGeneratorInfo: [{ name: "Adobe Firefly" }], signatureInfo: { issuer: "Adobe Inc." }, ingredients: [], assertions: [{ label: "c2pa.actions", data: { actions: [{ action: "c2pa.created", digitalSourceType: "http://cv.iptc.org/newscodes/digitalsourcetype/trainedAlgorithmicMedia" }] } }] },
    validationStatus: [],
  });
  ok(ai.length === 1 && ai[0].id === "c2pa-ai" && ai[0].level === "alarm" && /Firefly/.test(ai[0].detail), "c2pa: verified trained-algorithmic-media assertion → AI alarm naming the generator");
  const edited = interpretC2pa({
    activeManifest: { claimGenerator: "Adobe Photoshop 25.0 (Windows)", claimGeneratorInfo: [], signatureInfo: { issuer: "Adobe Inc." }, ingredients: [{}, {}], assertions: [] },
    validationStatus: [],
  });
  ok(edited.some((x) => x.id === "c2pa-valid" && /2 ingredients/.test(x.detail)) && edited.some((x) => x.id === "c2pa-edited" && x.level === "warn"), "c2pa: valid Photoshop chain → verified info + disclosed-editor warn");
  ok(interpretC2pa(null).length === 0 && interpretC2pa({ validationStatus: [] }).length === 0, "c2pa: no manifest → no findings, never a guess");
}

// --- ✨ satellite glint geometry — offline, via a fixed historical ISS TLE.
//     Exact invariant: for an observer at the SUB-SATELLITE POINT the
//     sat→observer direction IS the panel normal (nadir), and the mirror law
//     then makes the glint angle equal the phase angle exactly.
{
  const satjs = await import("satellite.js");
  const { satsAt } = await import("../src/checks/satellites.js");
  const rec = satjs.twoline2satrec(
    "1 25544U 98067A   24001.07953474  .00019243  00000+0  34460-3 0  9994",
    "2 25544  51.6437 305.4740 0002481 262.2661 208.6142 15.50123256432893");
  const ms = Date.UTC(2024, 0, 1, 12, 0, 0);
  const pv = satjs.propagate(rec, new Date(ms));
  const gd = satjs.eciToGeodetic(pv.position, satjs.gstime(new Date(ms)));
  const la = gd.latitude * R2D, lo = ((gd.longitude * R2D + 540) % 360) - 180;
  const out = satsAt([{ name: "ISS", rec }], ms, la, lo, 80);
  ok(out.length === 1 && out[0].el > 89, "glint: observer at the sub-satellite point sees the sat at zenith");
  ok(isFinite(out[0].glintDeg) && out[0].glintDeg >= 0 && out[0].glintDeg <= 180 && isFinite(out[0].phaseDeg), "glint: flare geometry computed through the real transforms");
  approx(out[0].glintDeg, out[0].phaseDeg, 0.6, "glint: at zenith the nadir-mirror law makes glint = phase (exact invariant)");
}

// --- 📍 Find my spot (geoloc.js) — haze-proof far-skyline detector, joint
//     multi-frame sweep recovering a known position on a synthetic world,
//     pin bearing geometry, and the decisive-vs-flat honesty gate.
{
  const GL = await import("../src/geoloc.js");
  const { dirToPixK } = await import("../src/math/projection.js");

  /* far-skyline detector on a synthetic hazy scene: bright sky over a
     DISTANT haze-blued ridge (dark in luminance, blue in chroma — the
     case that captured the blueness-keyed detectSkyline in the field)
     over a dark near band */
  const W = 288, H = 200;
  const mk = (skyLum) => {
    const d = new Uint8ClampedArray(W * H * 4);
    const yb = (x) => Math.round(60 + 20 * Math.sin(x / 40));
    for (let y = 0; y < H; y++)
      for (let x = 0; x < W; x++) {
        const i = (y * W + x) * 4;
        let r, g, b;
        if (y < yb(x)) { r = g = b = skyLum; }               // sky
        else if (y < 150) { r = 100; g = 110; b = 150; }     // hazy far ridge (lum ~110, bluer than sky)
        else { r = 30; g = 60; b = 30; }                     // near vegetation
        d[i] = r; d[i + 1] = g; d[i + 2] = b; d[i + 3] = 255;
      }
    return d;
  };
  const pts = GL.farSkyline(mk(205), W, H);
  ok(pts && pts.length >= 40, "geoloc: far skyline found on the hazy synthetic scene");
  const maxErr = Math.max(...pts.map((p) => Math.abs(p.y - (60 + 20 * Math.sin(p.x / 40)))));
  ok(maxErr <= 4, `geoloc: detected boundary is the FAR ridge to ≤4 px (got ${maxErr.toFixed(1)})`);
  ok(GL.farSkyline(mk(120), W, H) === null, "geoloc: dim top band fails the sky-quality gate → frame dropped, not garbage");

  /* joint sweep on a synthetic world: three cones, observer truth at a
     known grid cell, three "frames" panned across them at FOV 30 */
  const CONES = [
    { n: 6000, e: 2000, r: 2500, h: 900 },
    { n: 7000, e: -1500, r: 1800, h: 700 },
    { n: 4000, e: 5000, r: 1500, h: 600 },
  ];
  const world = (e, n) => {
    let z = 100;
    for (const c of CONES) {
      const d = Math.hypot(e - c.e, n - c.n);
      if (d < c.r) z = Math.max(z, 100 + c.h * (1 - d / c.r));
    }
    return z;
  };
  const mLat = 111320, mLonS = 111320 * Math.cos((30 * Math.PI) / 180);
  const C0 = { lat: 30, lon: 78 };
  const truth = { lat: 30 + 2000 / mLat, lon: 78 - 2000 / mLonS }; // grid cell (+2 km N, −2 km E)
  const sampAt = (lat, lon) => {
    const e0 = (lon - C0.lon) * mLonS, n0 = (lat - C0.lat) * mLat;
    return { sampleEN: (e, n) => world(e0 + e, n0 + n), h0: world(e0, n0) };
  };
  /* render frames from the truth cell */
  const tru = sampAt(truth.lat, truth.lon);
  const skT = skylineFromSampler(tru.sampleEN, tru.h0);
  const elT = (a) => skylineElAt(skT.els, a);
  const FW = 720, FH = 1280, TRUE_FOV = 30;
  const frameSets = [10, 40, 70].map((azF) => {
    const fpts = [];
    for (let i = 0; i < 44; i++) {
      const az = azF - 13 + (26 * i) / 43;
      const g = dirFromAzEl(az, elT(az));
      const p = dirToPixK(g, FW, FH, azF, 10, 0, TRUE_FOV, 0);
      if (p) fpts.push({ x: Math.round(p.px), y: Math.round(p.py) });
    }
    return { sets: GL.skySampleSets(fpts, FW, FH, [20, 30, 42]) };
  });
  const cands = GL.gridCandidates(C0.lat, C0.lon, 4, 2);
  ok(cands.length === 25, "geoloc: 4 km / 2 km grid → 25 candidates");
  const scored = cands.map((c) => {
    const s = sampAt(c.lat, c.lon);
    return { ...c, ...GL.scoreCandidate(frameSets, s.sampleEN, s.h0) };
  }).sort((a, b) => a.score - b.score);
  const wDist = Math.hypot((scored[0].lat - truth.lat) * mLat, (scored[0].lon - truth.lon) * mLonS);
  ok(wDist < 1, `geoloc: sweep recovers the true cell exactly (winner ${wDist.toFixed(0)} m off)`);
  const fAz = scored[0].frameAz.map((f) => f.az);
  ok(Math.max(...fAz.map((a, i) => GL.angDiff(a, [10, 40, 70][i]))) < 5,
    `geoloc: per-frame pointing recovered (got ${fAz.map((a) => a.toFixed(0)).join("/")} for 10/40/70)`);
  const verdict = GL.sweepVerdict(scored.map((s) => s.score));
  ok(verdict.decisive, `geoloc: distinctive terrain → decisive verdict (ratio ${verdict.ratio})`);
  ok(!GL.sweepVerdict([0.23, 0.24, 0.245, 0.25, 0.252, 0.255, 0.256, 0.259, 0.26]).decisive,
    "geoloc: the field-measured flat spread (best/med ~0.9) must NOT read as decisive");

  /* pin geometry */
  approx(GL.bearingDeg(30, 78, 30 + 300 / mLat, 78), 0, 0.01, "geoloc: bearing due north");
  approx(GL.bearingDeg(30, 78, 30, 78 + 300 / mLonS), 90, 0.1, "geoloc: bearing due east");
  approx(GL.pinAzOffsetDeg(FW / 2, FW, 30), 0, 1e-9, "geoloc: center pin → zero offset");
  approx(GL.pinAzOffsetDeg(FW, FW, 30), 15, 1e-6, "geoloc: right-edge pin → +FOV/2 (tan-true)");
  const ring = GL.ringCandidates(30, 78, 300, 8);
  ok(ring.length === 9 && ring[0].ringBrg === null, "geoloc: ring = twin + 8 offsets");
  const rd = Math.hypot((ring[3].lat - 30) * mLat, (ring[3].lon - 78) * mLonS);
  approx(rd, 300, 1, "geoloc: ring radius 300 m");
  const dev = GL.pinDeviation({ lat: 30, lon: 78 }, { lat: 30 + 300 / mLat, lon: 78 }, 350, 10);
  approx(dev, 0, 0.01, "geoloc: pin due north + frame az 350 + pixel offset +10° → deviation 0");

  /* multi-pin joint consistency (pinsDeviation): worst pin governs,
     range gate excludes far twins, unmapped kinds are skipped */
  const cand = { lat: 30, lon: 78 };
  const at = (dNm, dEm) => ({ lat: 30 + dNm / mLat, lon: 78 + dEm / mLonS });
  const twinsAll = [
    { ...at(400, 0), kind: "water" },        // due north, in range (1.5 km)
    { ...at(0, 900), kind: "mast" },          // due east, in range (3 km)
    { ...at(40000, 0), kind: "peak" },        // 40 km north — in PEAK range
    { ...at(9000, 0), kind: "water" },        // 9 km — OUTSIDE water range
  ];
  approx(GL.pinsDeviation(cand, [{ kind: "water", azDeg: 0 }, { kind: "mast", azDeg: 95 }], twinsAll), 5, 0.1,
    "geoloc: multi-pin — worst pin governs (water 0°, mast 5° off → 5)");
  ok(GL.pinsDeviation(cand, [{ kind: "water", azDeg: 180 }], [twinsAll[3]]) === null,
    "geoloc: the only matching twin beyond the kind's range → null, not a fake pass");
  ok(GL.pinsDeviation(cand, [{ kind: "chimney", azDeg: 45 }], twinsAll) === null,
    "geoloc: unmapped structure kind → null (absence of map data is not evidence)");
  approx(GL.pinsDeviation(cand, [{ kind: "peak", azDeg: 3 }], twinsAll), 3, 0.1,
    "geoloc: a peak 40 km out is testable — its range gate is the wide one");
  approx(GL.pinsDeviation(cand, [{ kind: "mast", azDeg: 90 }], [{ ...at(0, 900), kind: "tower" }]), 0, 0.1,
    "geoloc: a mast pin matches an OSM tower twin");
  ok(GL.pinsDeviation(cand, [{ kind: "water", azDeg: 0 }, { kind: "chimney", azDeg: 200 }], twinsAll) === 0 + GL.pinsDeviation(cand, [{ kind: "water", azDeg: 0 }], twinsAll),
    "geoloc: an untestable pin does not disturb the testable ones");

  /* EXTENDED twin (a bridge): matched along its whole SPAN, not by its
     map centre. The span below runs 1.2 km due east of the candidate,
     from 600 m north to 600 m south of the east axis — so its centre
     bears 90°, but every bearing from ~50° to ~130° hits the structure. */
  const brSpan = { kind: "bridge", ...at(0, 800), pts: [at(600, 800), at(0, 800), at(-600, 800)] };
  approx(GL.pinsDeviation(cand, [{ kind: "bridge", azDeg: 90 }], [brSpan]), 0, 0.1,
    "geoloc: bridge — a bearing through its centre hits the span (0°)");
  approx(GL.pinsDeviation(cand, [{ kind: "bridge", azDeg: 60 }], [brSpan]), 0, 0.1,
    "geoloc: bridge — a bearing onto its far END is a hit too, not a 30° miss");
  approx(GL.pinsDeviation(cand, [{ kind: "bridge", azDeg: 20 }], [brSpan]), 33.1, 1,
    "geoloc: bridge — a bearing that misses the whole span reports the miss to its nearest point");
  /* the same geometry as a POINT twin (its OSM centre) would punish the
     true spot for standing off one end — this is the bug the span fixes */
  approx(GL.pinsDeviation(cand, [{ kind: "bridge", azDeg: 60 }], [{ kind: "bridge", ...at(0, 800) }]), 30, 0.5,
    "geoloc: the same bridge as a bare centre point misses by 30° — why spans are kept");
  /* range gate uses the NEAREST point of the span: a bridge whose centre
     sits beyond the 5 km gate is still testable if one end is close */
  const brFar = { kind: "bridge", ...at(0, 7000), pts: [at(0, 2000), at(0, 12000)] };
  approx(GL.pinsDeviation(cand, [{ kind: "bridge", azDeg: 90 }], [brFar]), 0, 0.1,
    "geoloc: bridge — the nearest point of the span governs the range gate, not the centre");
  ok(GL.pinsDeviation(cand, [{ kind: "bridge", azDeg: 90 }], [{ kind: "bridge", ...at(0, 7000) }]) === null,
    "geoloc: a bridge CENTRE 7 km out is beyond the kind's range → untestable");
  /* the 20 m degeneracy guard: standing on the span is not a free pass on
     the near vertex, but the far end still carries a real bearing */
  const onIt = { lat: brSpan.pts[1].lat, lon: brSpan.pts[1].lon };
  approx(GL.pinsDeviation(onIt, [{ kind: "bridge", azDeg: 0 }], [brSpan]), 0, 0.1,
    "geoloc: standing on a bridge — the span's far end still answers (deck bearing 0°)");
  /* point twins must be byte-identical through the new code path */
  ok(GL.pinsDeviation(cand, [{ kind: "water", azDeg: 0 }, { kind: "mast", azDeg: 95 }], twinsAll) === 5 ||
    Math.abs(GL.pinsDeviation(cand, [{ kind: "water", azDeg: 0 }, { kind: "mast", azDeg: 95 }], twinsAll) - 5) < 0.1,
    "geoloc: point twins unchanged by the span path");
  /* ring anchors: a point twin offers itself, a span offers ends + middle */
  ok(GL.twinAnchors({ lat: 30, lon: 78 }).length === 1, "geoloc: a point twin spawns one ring anchor");
  const ans = GL.twinAnchors(brSpan);
  ok(ans.length === 3 && Math.abs((ans[0].lat - ans[2].lat) * mLat - 1200) < 1,
    "geoloc: a span twin spawns ring anchors at both ends and the middle (1.2 km apart)");
  /* a CLOSED outline (OSM maps big bridges as an area): first and last
     vertex are the same spot, so anchors must come from the diameter */
  const ring4 = { kind: "bridge", ...at(0, 800), pts: [[at(300, 400), at(300, 1200), at(-300, 1200), at(-300, 400), at(300, 400)]] };
  const ra = GL.twinAnchors(ring4);
  ok(ra.length === 3 && Math.hypot((ra[0].lat - ra[2].lat) * mLat, (ra[0].lon - ra[2].lon) * mLonS) > 900,
    "geoloc: a CLOSED span outline still yields anchors across its diameter, not one repeated corner");
  /* MULTI-PART twin: one bridge arrives as many OSM ways (the Golden Gate
     as 41). Parts are matched separately — the gap between two disjoint
     segments must NOT read as structure a ray can hit. */
  const twoPart = { kind: "bridge", ...at(0, 800), pts: [[at(600, 800), at(400, 800)], [at(-400, 800), at(-600, 800)]] };
  approx(GL.pinsDeviation(cand, [{ kind: "bridge", azDeg: 63.4 }], [twoPart]), 0, 0.2,
    "geoloc: multi-part bridge — a bearing onto the first way is a hit");
  approx(GL.pinsDeviation(cand, [{ kind: "bridge", azDeg: 90 }], [twoPart]), 26.5, 0.5,
    "geoloc: multi-part bridge — the GAP between two ways is not structure (no phantom hit)");
  /* the COMMONEST case: a bridge that is a single OSM way, i.e. ONE part.
     Its pts.length is 1, so it must be routed by having geometry at all —
     never by a length test, which would send it back to centre-matching */
  approx(GL.pinsDeviation(cand, [{ kind: "bridge", azDeg: 60 }], [{ kind: "bridge", ...at(0, 800), pts: [[at(600, 800), at(-600, 800)]] }]), 0, 0.1,
    "geoloc: a ONE-part bridge still takes the span path (0°, not the 30° centre miss)");

  /* setting-context filters */
  const places = [
    { ...at(0, 0), kind: "place", ptype: "town" },        // town centred on the candidate (radius 2.5 km)
    { ...at(9000, 0), kind: "place", ptype: "village" },  // village 9 km north (radius 1.2 km)
  ];
  ok(GL.settingOk(cand, { places }, "town") && !GL.settingOk(cand, { places }, "out"),
    "geoloc: inside a town radius → passes 'in a town', fails 'outside'");
  const wild = { ...at(0, 30000) };                        // 30 km east of everything
  ok(!GL.settingOk(wild, { places }, "town") && GL.settingOk(wild, { places }, "out"),
    "geoloc: far from every place → passes 'outside', fails 'in a town'");
  ok(GL.settingOk(cand, {}, "town") && GL.settingOk(cand, {}, "out"),
    "geoloc: no place data → never filter on a guess");
  /* looking north from inside the town: the DISTINCT village sits in the
     cone; the town you stand in must not count */
  ok(GL.lookOk(cand, 0, { places }, "town") && !GL.lookOk(cand, 0, { places }, "open"),
    "geoloc: distinct village in the view cone → 'looking at a town'");
  ok(!GL.lookOk(cand, 180, { places }, "town") && GL.lookOk(cand, 180, { places }, "open"),
    "geoloc: facing away → 'open country', even while standing IN a town");

  /* built-up LAND-USE boxes are the primary signal when mapped — this is
     the field-report case: a bare field 1 km from a village place node
     passed "in a town" under the node-radius rule */
  const urbans = [
    { kind: "urban", bbox: [30 - 500 / mLat, 30 + 500 / mLat, 78 - 500 / mLonS, 78 + 500 / mLonS] },
    { kind: "urban", bbox: [30 + 3000 / mLat, 30 + 4000 / mLat, 78 - 500 / mLonS, 78 + 500 / mLonS] },
  ];
  ok(GL.settingOk(cand, { urbans, places }, "town") && !GL.settingOk(cand, { urbans, places }, "out"),
    "geoloc: standing ON built-up land-use → 'in a town'");
  const field = at(1500, 0);
  ok(!GL.settingOk(field, { urbans, places }, "town") && GL.settingOk(field, { urbans, places }, "out"),
    "geoloc: a bare field 1 km off built-up land is NOT 'in a town' (the field-report bug)");
  ok(GL.lookOk(cand, 0, { urbans }, "town") && !GL.lookOk(cand, 0, { urbans }, "open"),
    "geoloc: view ray north enters the other built-up patch → 'looking at a town'");
  ok(!GL.lookOk(cand, 180, { urbans }, "town") && GL.lookOk(cand, 180, { urbans }, "open"),
    "geoloc: view south is open — the patch you stand in never counts");

  /* stabilized-pan lock: merge the same three frames with their KNOWN
     relative pointing (one frame deliberately pitched differently) and the
     sweep must still recover the true cell + the global rotation */
  const perFrame = [10, 40, 70].map((azF, fi) => {
    const elF = fi === 1 ? 14 : 10; // exercise the relative-el bake
    const fpts = [];
    for (let i = 0; i < 44; i++) {
      const az = azF - 13 + (26 * i) / 43;
      const g = dirFromAzEl(az, elT(az));
      const p = dirToPixK(g, FW, FH, azF, elF, 0, TRUE_FOV, 0);
      if (p) fpts.push({ x: Math.round(p.px), y: Math.round(p.py) });
    }
    return { pts: fpts, W: FW, H: FH, fov: TRUE_FOV, relAz: azF - 10, relEl: elF - 10 };
  });
  const lockedSet = GL.lockedFrameSet(perFrame);
  ok(lockedSet && lockedSet.ss.length > 100, "geoloc: locked pan merges all frames' samples");
  const scoredL = cands.map((c) => {
    const s = sampAt(c.lat, c.lon);
    return { ...c, ...GL.scoreCandidate([{ sets: [lockedSet] }], s.sampleEN, s.h0, { winDeg: 2 }) };
  }).sort((a, b) => a.score - b.score);
  const wDistL = Math.hypot((scoredL[0].lat - truth.lat) * mLat, (scoredL[0].lon - truth.lon) * mLonS);
  ok(wDistL < 1, `geoloc: locked-pan sweep recovers the true cell (${wDistL.toFixed(0)} m off)`);
  ok(GL.angDiff(scoredL[0].az, 10) < 3, `geoloc: locked-pan global rotation recovered (az ${scoredL[0].az.toFixed(0)} vs 10)`);
  ok(GL.sweepVerdict(scoredL.map((s) => s.score)).ratio <= verdict.ratio + 0.05,
    `geoloc: locking the pan does not blur the spread (${GL.sweepVerdict(scoredL.map((s) => s.score)).ratio} vs free ${verdict.ratio})`);

  /* near-ridge detector: sky / hazy wall / dark near ridge in one column */
  const mk3 = () => {
    const W3 = 288, H3 = 220, d3 = new Uint8ClampedArray(W3 * H3 * 4);
    const ybF = (x) => Math.round(50 + 12 * Math.sin(x / 40));
    const ybN = (x) => Math.round(130 + 10 * Math.cos(x / 30));
    for (let y = 0; y < H3; y++)
      for (let x = 0; x < W3; x++) {
        const i = (y * W3 + x) * 4;
        let r, g, b;
        if (y < ybF(x)) { r = g = b = 205; }
        else if (y < ybN(x)) { r = 100; g = 110; b = 150; }
        else { r = 30; g = 60; b = 30; }
        d3[i] = r; d3[i + 1] = g; d3[i + 2] = b; d3[i + 3] = 255;
      }
    return { d3, W3, H3, ybF, ybN };
  };
  const m3 = mk3();
  const farP = GL.farSkyline(m3.d3, m3.W3, m3.H3);
  const nearP = GL.nearSkyline(m3.d3, m3.W3, m3.H3, farP);
  ok(nearP && nearP.length >= 30, "geoloc: second ridge layer detected below the far wall");
  const nErr = Math.max(...nearP.map((p) => Math.abs(p.y - m3.ybN(p.x))));
  ok(nErr <= 4, `geoloc: near boundary to ≤4 px (got ${nErr.toFixed(1)})`);

  /* DEPTH: a pan has no parallax, but the near crest's placement against
     the far wall changes fast with position. World: a big wall 28 km
     north + a small near ridge 3 km north. Displacing the candidate
     ALONG the view axis barely moves the wall — the near layer is what
     separates it. */
  const world2 = (e, n) => {
    let z = 100;
    z = Math.max(z, 100 + 4000 * Math.exp(-(((n - 28000) / 3000) ** 2)) * Math.exp(-((e / 20000) ** 2)));
    const dc = Math.hypot(e, n - 3000);
    if (dc < 1000) z = Math.max(z, 100 + 120 * (1 - dc / 1000));
    return z;
  };
  const samp2 = (dNm, dEm) => ({ sampleEN: (e, n) => world2(dEm + e, dNm + n), h0: world2(dEm, dNm) });
  const tru2 = samp2(0, 0);
  const sk2 = skylineFromSampler(tru2.sampleEN, tru2.h0);
  const el2 = (a) => skylineElAt(sk2.els, a);
  const prof2 = GL.ridgeProfileOf(sk2);
  ok([...prof2].some(isFinite), "geoloc: DEM interior-crest profile has the near ridge");
  const mkPts = (elFn, filt) => {
    const fpts = [];
    for (let i = 0; i < 60; i++) {
      const az = -18 + (36 * i) / 59;
      const v = elFn(az);
      if (!isFinite(v) || (filt && !filt(az))) continue;
      const g = dirFromAzEl(az, v);
      const p = dirToPixK(g, FW, FH, 0, 10, 0, 40, 0);
      if (p && p.py > 0 && p.py < FH) fpts.push({ x: Math.round(p.px), y: Math.round(p.py) });
    }
    return fpts;
  };
  const farPts2 = mkPts(el2);
  const nearPts2 = mkPts((a) => GL.ridgeProfAt(prof2, a));
  ok(farPts2.length > 30 && nearPts2.length >= 12, `geoloc: synthetic two-layer frame built (${farPts2.length} far, ${nearPts2.length} near)`);
  const mkFS = (withNear) => [{
    sets: GL.skySampleSets(farPts2, FW, FH, [40]),
    nearSets: withNear ? GL.skySampleSets(nearPts2, FW, FH, [40]) : undefined,
  }];
  const D2 = [[0, 0], [2000, 0], [-2000, 0], [0, 2000], [0, -2000]];
  const farOnly = D2.map(([dn, de]) => { const s = samp2(dn, de); return GL.scoreCandidate(mkFS(false), s.sampleEN, s.h0).score; });
  const withNear = D2.map(([dn, de]) => { const s = samp2(dn, de); return GL.scoreCandidate(mkFS(true), s.sampleEN, s.h0).score; });
  ok(withNear[0] === Math.min(...withNear), "geoloc: with the depth layer the true position wins");
  const sepFar = Math.min(...farOnly.slice(1)) / Math.max(farOnly[0], 1e-6);
  const sepNear = Math.min(...withNear.slice(1)) / Math.max(withNear[0], 1e-6);
  /* far-only, a displaced spot actually BEATS the truth on this smooth
     wall (sep < 1); jointly the truth wins every displacement and the
     worst offender scores ~10× — assert the winner plus a real margin */
  ok(sepNear > Math.max(1.25, sepFar + 0.3),
    `geoloc: depth layer separates displaced candidates (${sepFar.toFixed(2)} → ${sepNear.toFixed(2)}×)`);
  ok(Math.max(...withNear.slice(1)) / withNear[0] > 4,
    `geoloc: a grossly displaced candidate is punished hard (${(Math.max(...withNear.slice(1)) / withNear[0]).toFixed(1)}×)`);

  /* weather cross-check: the clip's own sky read + the archive verdict */
  const mkSky = (r, g, b) => {
    const Ws = 200, Hs = 120, ds = new Uint8ClampedArray(Ws * Hs * 4);
    for (let i = 0; i < Ws * Hs; i++) { ds[i * 4] = r; ds[i * 4 + 1] = g; ds[i * 4 + 2] = b; ds[i * 4 + 3] = 255; }
    return GL.skyStats(ds, Ws, Hs);
  };
  ok(GL.skyCondition([mkSky(90, 140, 215)]) === "clear", "geoloc: saturated blue sky reads clear");
  ok(GL.skyCondition([mkSky(200, 202, 208)]) === "overcast", "geoloc: white-gray dome reads overcast");
  ok(GL.skyCondition([mkSky(150, 160, 172)]) === "mixed", "geoloc: hazy in-between reads mixed — no false confidence");
  ok(GL.skyCondition([mkSky(40, 45, 60)]) === null, "geoloc: dark sky → no daytime cloud read at all");
  ok(GL.cloudMatch("overcast", 92).verdict === "match" && GL.cloudMatch("overcast", 8).verdict === "mismatch",
    "geoloc: overcast clip vs archive — 92% cloud matches, 8% indicts the date/area");
  ok(GL.cloudMatch("clear", 12).verdict === "match" && GL.cloudMatch("clear", 95).verdict === "mismatch",
    "geoloc: clear clip vs archive — the symmetric case");
  ok(GL.cloudMatch("mixed", 92).verdict === "weak" && GL.cloudMatch("overcast", 45).verdict === "weak",
    "geoloc: ambiguous sky or middling cloud → weak, never a verdict from noise");

  /* land-use trust gate — the Dehradun field case: 1.35% coverage must
     demote to the place-node fallback, dense mapping must not */
  const sparse = Array.from({ length: 187 }, (_, i) => ({ kind: "urban", bbox: [30 + i * 1e-3, 30 + i * 1e-3 + 0.0019, 78, 78.0022] }));
  const cov = GL.urbanCoverage(sparse, 30.33, 14);
  ok(cov.frac < 0.02 && !cov.ok, `geoloc: Dehradun-grade sparse land-use fails the trust gate (${(cov.frac * 100).toFixed(1)}%)`);
  const dense = [{ kind: "urban", bbox: [30.29, 30.37, 77.97, 78.07] }];
  ok(GL.urbanCoverage(dense, 30.33, 14).ok, "geoloc: a real mapped town passes the trust gate");
}

// --- 🖼 retrospective panorama — pure geometry: azimuth unwrap across the
//     0/360 seam, tan-true frame extents, layout caps, equirect mapping,
//     zoom-detail-last render order.
{
  const PN = await import("../src/video/panorama.js");
  const uw = PN.unwrapSamples([{ az: 350, el: 5, fov: 40, t: 0 }, { az: 0, el: 5, fov: 40, t: 1 }, { az: 10, el: 5, fov: 40, t: 2 }]);
  ok(uw.map((p) => p.uAz).join() === "350,360,370", "pano: a pan across the 0/360 seam unwraps continuously");
  const border = PN.frameBorder({ uAz: 0, el: 0, roll: 0, fov: 40 }, 720, 1280);
  const azs = border.map((b) => b.az), els = border.map((b) => b.el);
  approx(Math.max(...azs), 20, 0.5, "pano: frame border reaches +fov/2 in azimuth (tan-true)");
  approx(Math.min(...azs), -20, 0.5, "pano: …and −fov/2");
  ok(Math.max(...els) > 25 && Math.max(...els) < 40, `pano: portrait vertical extent is tangent-scaled (${Math.max(...els).toFixed(1)}°)`);
  const lay = PN.panoLayout(PN.unwrapSamples([{ az: 0, el: 10, roll: 0, fov: 40, t: 0 }, { az: 60, el: 10, roll: 0, fov: 40, t: 1 }]), 720, 1280);
  ok(lay.azMax - lay.azMin > 95 && lay.azMax - lay.azMin < 110, `pano: two frames 60° apart span ~100° (${(lay.azMax - lay.azMin).toFixed(0)}°)`);
  ok(lay.W <= 4600 && lay.H <= 4600 && lay.W * lay.H <= 16e6, `pano: layout respects the iOS canvas guards (${lay.W}×${lay.H})`);
  const [x0, y0] = PN.equirectXY(lay, lay.azMin, lay.elMax);
  ok(Math.abs(x0) < 1e-9 && Math.abs(y0) < 1e-9, "pano: equirect origin maps to canvas (0,0)");
  const [x1] = PN.equirectXY(lay, lay.azMin + 10, lay.elMax);
  approx(x1, 10 * lay.ppd, 1e-6, "pano: 10° of azimuth = 10·ppd pixels");
  const ord = PN.renderOrder([{ fov: 40 }, { fov: 40 }, { fov: 12 }, { fov: 40 }, { fov: 11 }, { fov: 40 }].map((s, i) => ({ ...s, t: i })));
  ok(ord.slice(0, 6).join() === "0,1,2,3,4,5" && ord.slice(6).join() === "2,4",
    "pano: chronological pass, then the sharpest (most-zoomed) frames repainted on top");

  /* quality gate: held frames never qualify; weak solves and whip-pans
     are dropped when enough strong frames remain */
  const pth = [];
  for (let i = 0; i < 30; i++) pth.push({ t: i * 0.5, az: i * 2, el: 5, fov: 40, n: 30 });
  pth[4] = { ...pth[4], h: 1 };            // held
  pth[9] = { ...pth[9], n: 3 };            // starved solve
  pth[14] = { ...pth[14], az: pth[13].az + 20 }; // 40°/s whip
  const picked = PN.panoPick(pth, 90);
  ok(!picked.some((p) => p.h) && !picked.some((p) => p.n === 3) && picked.length >= 25,
    `pano: quality gate drops held/starved/whip frames, keeps the rest (${picked.length}/30)`);

  /* re-registration: a synthetic pattern shifted by a KNOWN (dx,dy) must
     be recovered by the equirect NCC, and near-zero overlap must refuse */
  const mkImg = (W2, H2, fill) => {
    const d = new Uint8ClampedArray(W2 * H2 * 4);
    for (let y = 0; y < H2; y++) for (let x = 0; x < W2; x++) {
      const i = (y * W2 + x) * 4;
      const v = fill(x, y);
      d[i] = d[i + 1] = d[i + 2] = v == null ? 0 : v;
      d[i + 3] = v == null ? 0 : 255;
    }
    return { data: d, width: W2, height: H2 };
  };
  const patF = (x, y) => 120 + 90 * Math.sin(x / 5) * Math.cos(y / 7);
  const base2 = mkImg(80, 60, patF);
  const shifted = mkImg(80, 60, (x, y) => (x - 3 < 0 || y + 2 >= 60) ? null : patF(x - 3, y + 2));
  /* patch content sits at (+3,−2) relative to base → the CORRECTION the
     caller applies (dx,dy) is (−3,+2); the browser harness asserts the
     same convention end-to-end through drawFramePano + uAz adjustment */
  const sh2 = PN.bestShift(base2, shifted, 6);
  ok(sh2 && sh2.dx === -3 && sh2.dy === 2, `pano: NCC measures a known mis-registration with the applied sign (got ${sh2 && sh2.dx},${sh2 && sh2.dy} score ${sh2 && sh2.score.toFixed(2)})`);
  const tiny = mkImg(80, 60, (x, y) => (x < 6 && y < 6) ? patF(x, y) : null);
  ok(PN.bestShift(base2, tiny, 6) === null, "pano: near-zero overlap → no correction, never a guess");

  /* zoom-scale registration (the two-trees field case): the frame's
     content is 15% larger than its claimed FOV implies — renderPatch(s)
     resamples about the center, and the ladder must pick the shrinking
     rung; an unbiased frame must keep scale 1 (the privileged rung). */
  const resample = (img, s) => mkImg(img.width, img.height, (x, y) => {
    const cx2 = img.width / 2, cy2 = img.height / 2;
    const sx2 = Math.round(cx2 + (x - cx2) / s), sy2 = Math.round(cy2 + (y - cy2) / s);
    if (sx2 < 0 || sy2 < 0 || sx2 >= img.width || sy2 >= img.height) return null;
    const i = (sy2 * img.width + sx2) * 4;
    return img.data[i + 3] ? img.data[i] : null;
  });
  const TRUE_BIAS = 1.15;
  const regBias = PN.registerFrame(base2, (s) => resample(base2, TRUE_BIAS * s), { R: 4 });
  ok(regBias && regBias.scale < 1 && Math.abs(TRUE_BIAS * regBias.scale - 1) < 0.06,
    `pano: 15% zoom bias corrected by the ladder (picked ×${regBias && regBias.scale} → residual ${regBias ? Math.abs(TRUE_BIAS * regBias.scale - 1).toFixed(3) : "-"})`);
  const regClean = PN.registerFrame(base2, (s) => resample(base2, s), { R: 4 });
  ok(regClean && regClean.scale === 1, "pano: an unbiased frame keeps scale 1 — no FOV jitter from the ladder");

  /* 📐 smoothCorrections: the per-frame registrations are noisy independent
     measurements of a SMOOTH drift — raw exact anchors imprinted the noise
     on the camera path (field: a 5.5° re-lock flip between adjacent
     samples + a ±0.6° sawtooth read as playback glitches). Despike must
     kill a lone flip, smoothing must damp the sawtooth, a genuine ramp
     must pass through, and unmeasured samples must stay null without
     contaminating neighbours. */
  {
    const mkC = (vals) => vals.map((v, i) => (v == null ? { t: i, dAz: 0, dEl: 0, r: 1, score: 0.2 } : { t: i, dAz: v, dEl: 0, r: 1, score: 0.9 }));
    const flip = mkC([0.2, 0.25, 0.3, 3.4, 0.4, 0.45, 0.5]);
    const sFlip = PN.smoothCorrections(flip);
    ok(Math.abs(sFlip[3].dAz - 0.35) < 0.25, `pano-fix smoothing: a lone 3° re-lock flip snaps back to its neighbours (got ${sFlip[3].dAz.toFixed(2)})`);
    const saw = mkC([0, 0.6, -0.5, 0.55, -0.45, 0.5, -0.5, 0.6, 0]);
    const sSaw = PN.smoothCorrections(saw);
    const rms = (a) => Math.sqrt(a.reduce((s, c) => s + c.dAz * c.dAz, 0) / a.length);
    ok(rms(sSaw.slice(1, 8)) < rms(saw.slice(1, 8)) * 0.6, `pano-fix smoothing: sawtooth damped (rms ${rms(saw.slice(1, 8)).toFixed(2)} → ${rms(sSaw.slice(1, 8)).toFixed(2)})`);
    const ramp = mkC([0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]);
    const sRamp = PN.smoothCorrections(ramp);
    ok(sRamp.every((c, i) => Math.abs(c.dAz - ramp[i].dAz) < 0.15), "pano-fix smoothing: a genuine drift ramp passes through");
    const gap = PN.smoothCorrections(mkC([0.4, 0.4, null, 0.4, 0.4]));
    ok(gap[2] === null && Math.abs(gap[1].dAz - 0.4) < 0.05 && Math.abs(gap[3].dAz - 0.4) < 0.05,
      "pano-fix smoothing: an unmeasured sample stays null and never contaminates its neighbours");
    const zoomSpike = [{ t: 0, dAz: 0, dEl: 0, r: 1, score: 0.9 }, { t: 1, dAz: 0, dEl: 0, r: 1.2, score: 0.9 }, { t: 2, dAz: 0, dEl: 0, r: 1, score: 0.9 }];
    const sZoom = PN.smoothCorrections(zoomSpike);
    ok(Math.abs(sZoom[1].r - 1) < 0.05, `pano-fix smoothing: a lone ×1.2 zoom-scale spike is pulled back (got ×${sZoom[1].r.toFixed(2)})`);
  }

  /* v3 adaptive registration windows: a deep-zoom frame must get enough
     pixels to correlate (the v2 fixed 2 px/° twin gave it ~20 — field-
     measured as zero zoom corrections exactly where they were needed),
     a wide frame must not balloon the window, and the resolution must
     never exceed the composite's own (upscaling invents no detail). */
  const wLayBig = PN.panoLayout(PN.unwrapSamples([
    { az: 0, el: 10, roll: 0, fov: 60, t: 0 }, { az: 40, el: 10, roll: 0, fov: 10, t: 1 },
  ]), 1920, 1080);
  const wZoom = PN.regWindow(wLayBig, { uAz: 40, el: 10, roll: 0, fov: 10 }, 1920, 1080, { target: 96, padDeg: 2.2 });
  ok(wZoom.ppd * wZoom.spanAz >= 90 && wZoom.ppd > 4,
    `pano: a 10° zoom frame spans ~target px in its window (${(wZoom.ppd * wZoom.spanAz).toFixed(0)} px at ${wZoom.ppd.toFixed(1)} px/°)`);
  const wWide = PN.regWindow(wLayBig, { uAz: 0, el: 10, roll: 0, fov: 60 }, 1920, 1080, { target: 96, padDeg: 2.2 });
  ok(wWide.ppd === 2 && wWide.W < 260,
    `pano: a wide frame keeps the coarse floor — bounded window (${wWide.W}×${wWide.H} at ${wWide.ppd} px/°)`);
  ok(wZoom.ppd <= wLayBig.ppd + 1e-9, "pano: window resolution never exceeds the composite's own ppd");
  ok(wZoom.azMin >= wLayBig.azMin - 1e-9 && wZoom.azMax <= wLayBig.azMax + 1e-9 &&
    wZoom.elMin >= wLayBig.elMin - 1e-9 && wZoom.elMax <= wLayBig.elMax + 1e-9,
    "pano: the window stays inside the composite it crops from");
  /* CONTENT-EQUATOR ROTATION (near-zenith clips): equirect degenerates at
     the poles — a tilt-to-70° clip's frames span enormous azimuth there and
     the layout ballooned to a fictional 347° (field case). panoRot builds
     the pano in a rotated frame whose equator runs through the content. */
  {
    const zen = [{ az: 350, el: 62, roll: 3, fov: 40, t: 0 }, { az: 10, el: 71, roll: -5, fov: 40, t: 1 }, { az: 355, el: 66, roll: 1, fov: 40, t: 2 }];
    const flatSet = [{ az: 250, el: 8, roll: 0, fov: 40, t: 0 }, { az: 270, el: 12, roll: 0, fov: 40, t: 1 }];
    ok(PN.panoRot(flatSet) === null, "panoRot: equatorial content keeps the old path (null — byte-identical)");
    const rot = PN.panoRot(zen);
    ok(rot && Math.abs(rot.el) > 60, `panoRot: engages on near-zenith content (mean el ${rot && rot.el}°)`);
    let rtWorst = 0, pixWorst = 0;
    for (const p of zen) {
      const rp = rot.pose(p, false), back = rot.pose(rp, true);
      rtWorst = Math.max(rtWorst, Math.abs(((back.az - p.az + 540) % 360) - 180), Math.abs(back.el - p.el), Math.abs(back.roll - p.roll));
      for (const [x, y] of [[80, 60], [900, 1500]]) {
        const gw = pixToDirK(x, y, 1080, 1920, p.az, p.el, p.roll, p.fov, 0);
        const gr = pixToDirK(x, y, 1080, 1920, rp.az, rp.el, rp.roll, rp.fov, 0);
        const gwr = rot.dir(gw, false);
        pixWorst = Math.max(pixWorst, Math.acos(Math.min(1, gr[0] * gwr[0] + gr[1] * gwr[1] + gr[2] * gwr[2])) * R2D);
      }
    }
    ok(rtWorst < 0.01, `panoRot: pose round-trip exact (worst ${rtWorst.toFixed(4)}°)`);
    ok(pixWorst < 0.01, `panoRot: rotated pose renders the same pixels — roll handled (worst ${pixWorst.toFixed(4)}°)`);
    const layRaw = PN.panoLayout(PN.unwrapSamples(zen), 1080, 1920);
    const layRot = PN.panoLayout(PN.unwrapSamples(zen.map((p) => rot.pose(p, false))), 1080, 1920);
    ok(layRot.azMax - layRot.azMin < 120 && layRaw.azMax - layRaw.azMin > 150,
      `panoRot: zenith layout un-balloons (${(layRaw.azMax - layRaw.azMin).toFixed(0)}° → ${(layRot.azMax - layRot.azMin).toFixed(0)}°)`);
  }

  /* the window layout is equirectXY-compatible: the whole frame footprint
     maps inside the window (padding may be asymmetric at layout edges) */
  const inWin = PN.frameBorder({ uAz: 40, el: 10, roll: 0, fov: 10 }, 1920, 1080).every((b) => {
    const [wx, wy] = PN.equirectXY(wZoom, b.az, b.el);
    return wx >= -0.5 && wx <= wZoom.W + 0.5 && wy >= -0.5 && wy <= wZoom.H + 0.5;
  });
  ok(inWin, "pano: the frame's full footprint maps inside its registration window");
}

if (fails) { console.error(`\nmathcheck: ${fails} assertion(s) failed`); process.exit(1); }
console.log("mathcheck: all assertions passed");
