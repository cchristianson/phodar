// Exercises the REAL math core (src/math/*) — not a copy. A regression in
// triangulation, geodesy, or angular sizing fails `npm test` here.
import { D2R, R2D, RE, enuFromGeo, geoFromEnu, dirFromAzEl, sub, mag } from "../src/math/geodesy.js";
import { intersectLines } from "../src/math/triangulate.js";
import { rankCandidates, spanForAircraft } from "../src/checks/adsb.js";
import { skylineFromSampler, skylineElAt, AZ_STEP } from "../src/terrain.js";
import { raDecToAzEl } from "../src/math/astro.js";
import { planetPositions } from "../src/math/planets.js";
import { STARS } from "../src/math/starcat.js";

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

if (fails) { console.error(`\nmathcheck: ${fails} assertion(s) failed`); process.exit(1); }
console.log("mathcheck: all assertions passed");
