// Drives the REAL trajectory pipeline (src/math/kinematics.js → analyzeTracks)
// with a simulated 3.5 g maneuver seen by two observers, and asserts that the
// recovered kinematics match the ground-truth simulation. Guards the actual
// app entry point, not a copy of it.
import { R2D, enuFromGeo, geoFromEnu, dirToAzEl, sub } from "../src/math/geodesy.js";
import { analyzeTracks } from "../src/math/kinematics.js";

let fails = 0;
const approx = (got, want, tol, msg) => {
  const ok = got != null && Math.abs(got - want) <= tol;
  if (!ok) { fails++; console.error(`  FAIL ${msg}: got ${got}, want ${want} ±${tol}`); }
  else console.log(`  ok   ${msg}: ${(+got).toFixed(2)}`);
  return ok;
};

// --- ground-truth simulation (matches demoSources) ---
const ref = { lat: 42.16380, lon: -123.64800, alt: 0 };
const obsP = [[0, 0, 0], [1999.5, 0, 0]];        // ENU positions
const v = 80, aM = 34, R = v * v / aM, om = v / R, C = [1000, 3000 + R, 2000];

// build each observer's az/el track by looking at the true object position
const sources = obsP.map((O, i) => {
  const geo = geoFromEnu(O, ref);
  const track = [];
  for (let t = 0; t <= 6.001; t += 0.5) {
    const th = -Math.PI / 2 + om * t;
    const P = [C[0] + R * Math.cos(th), C[1] + R * Math.sin(th), 2000];
    const w = sub(P, O);
    const az = ((Math.atan2(w[0], w[1]) * R2D) + 360) % 360;
    const el = Math.atan2(w[2], Math.hypot(w[0], w[1])) * R2D;
    track.push({ t, az: +az.toFixed(3), el: +el.toFixed(3) });
  }
  return {
    name: `Observer ${i + 1}`, lat: geo.lat, lon: geo.lon, alt: 0,
    fovH: 68, natW: null, natH: null, track, A: {}, B: {},
  };
});

const { stereo } = analyzeTracks(sources);
if (!stereo || !stereo.k) {
  console.error("  FAIL: analyzeTracks returned no stereo kinematics", stereo);
  process.exit(1);
}
const k = stereo.k;
console.log("expected: speed 80 m/s (179 mph), load ~3.6 g, turn ~24.4 deg/s, accel ~34 m/s2");
approx(k.n, 13, 0, "sample count");
approx(k.peakSpeed, 80, 1, "peak speed m/s");
approx(k.peakSpeed * 2.23694, 179, 3, "peak speed mph");
approx(k.peakA, 34, 1.5, "peak accel m/s2");
approx(k.peakLoad, 3.6, 0.2, "peak felt load g");
approx(k.peakTurn, 24.4, 1.5, "peak turn rate deg/s");
approx(stereo.avgMiss, 0, 1, "avg ray miss (m)");

if (fails) { console.error(`\ntrajcheck: ${fails} assertion(s) failed`); process.exit(1); }
console.log("trajcheck: all assertions passed");
