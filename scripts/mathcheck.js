// Exercises the REAL math core (src/math/*) — not a copy. A regression in
// triangulation, geodesy, or angular sizing fails `npm test` here.
import { D2R, enuFromGeo, dirFromAzEl, sub, mag } from "../src/math/geodesy.js";
import { intersectLines } from "../src/math/triangulate.js";

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

if (fails) { console.error(`\nmathcheck: ${fails} assertion(s) failed`); process.exit(1); }
console.log("mathcheck: all assertions passed");
