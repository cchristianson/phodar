// Exercises the REAL math core (src/math/*) — not a copy. A regression in
// triangulation, geodesy, or angular sizing fails `npm test` here.
import { D2R, R2D, RE, enuFromGeo, geoFromEnu, dirFromAzEl, sub, mag } from "../src/math/geodesy.js";
import { intersectLines, aspectSpan } from "../src/math/triangulate.js";
import { sunPos, moonFrac } from "../src/math/astro.js";
import { nearestLevel, balloonVerdict } from "../src/checks/winds.js";
import { rankCandidates, spanForAircraft } from "../src/checks/adsb.js";
import { trackDirections } from "../src/math/kinematics.js";
import { skylineFromSampler, skylineElAt, AZ_STEP, matchSkyline, detectSkyline } from "../src/terrain.js";
import { raDecToAzEl } from "../src/math/astro.js";
import { declination } from "../src/math/geomag.js";
import { parseMediaMeta } from "../src/exif.js";
import { planetPositions } from "../src/math/planets.js";
import { STARS } from "../src/math/starcat.js";
import { photoBasis, solveRollFov, pixToDirK, dirToPixK, solvePoseAnchors } from "../src/math/projection.js";
import { unit, dot } from "../src/math/geodesy.js";
import { parseLaunches, haversineKm } from "../src/checks/launches.js";
import { parseFireballs } from "../src/checks/fireballs.js";
import { parsePeaks, bearingDeg, distM } from "../src/checks/peaks.js";
import { detectStars, autoStarAlign, blindStarAlign, gridStarAlign } from "../src/checks/platesolve.js";

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

if (fails) { console.error(`\nmathcheck: ${fails} assertion(s) failed`); process.exit(1); }
console.log("mathcheck: all assertions passed");
