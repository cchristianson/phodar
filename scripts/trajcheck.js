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

// --- single-observer 3D reconstruction from per-point angular SIZE ---
// An object flies a straight line at constant velocity past ONE observer.
// A pure angular path would show spurious angular acceleration near closest
// approach; feeding the angular-size ratio (radial motion) must recover the
// true straight-line motion: constant 3D speed and ~zero acceleration.
{
  const S = 6, P0 = [-300, 500, 100], V = [40, 0, 0];
  const track = [], trueR = [];
  for (let t = 0; t <= 10.001; t += 1) {
    const P = [P0[0] + V[0] * t, P0[1] + V[1] * t, P0[2] + V[2] * t];
    const r = Math.hypot(P[0], P[1], P[2]); trueR.push(r);
    const az = ((Math.atan2(P[0], P[1]) * R2D) + 360) % 360;
    const el = Math.atan2(P[2], Math.hypot(P[0], P[1])) * R2D;
    const ang = 2 * Math.atan(S / (2 * r)) * R2D;             // apparent size at that range
    track.push({ t, az, el, ang });
  }
  const { solo } = analyzeTracks([{ name: "Solo", fovH: 68, track, A: {}, B: {} }]);
  const rad = solo && solo[0] && solo[0].rad;
  if (!rad) { console.error("  FAIL: solo radial reconstruction missing"); fails++; }
  else {
    const rRef = trueR[0];                                    // reference-point range (ρ_ref = 1)
    approx(rRef * rad.k3d.peakSpeed, 40, 0.6, "solo 3D peak speed m/s (radial+transverse)");
    approx(rRef * rad.k3d.avgSpeed, 40, 0.6, "solo 3D avg speed m/s");
    approx(rRef * (rad.k3d.peakA || 0), 0, 0.6, "solo 3D peak accel ~0 (straight, constant-V)");
    approx(rad.rangeRatio, Math.max(...trueR) / Math.min(...trueR), 0.01, "solo range ratio far/near");
  }
}

// --- aspect correction: a rotating object at CONSTANT range must not be
// misread as flying closer/farther. Its apparent size shrinks as it turns
// edge-on (foreshortening); feeding the per-point projection factor must
// divide that out so the recovered range stays flat (rangeRatio ≈ 1).
{
  const trueLong = 10, r = 500, projFs = [1, 0.7, 0.5, 0.7, 1];
  const track = projFs.map((f, i) => ({
    t: i * 2,
    az: 180 + i * 0.3, el: 25,                               // near-fixed line of sight
    ang: 2 * Math.atan((f * trueLong) / (2 * r)) * R2D,      // foreshortened apparent size
    projF: f,
  }));
  const { solo } = analyzeTracks([{ name: "Rotator", fovH: 68, track, A: {}, B: {} }]);
  const rad = solo && solo[0] && solo[0].rad;
  if (!rad) { console.error("  FAIL: aspect-corrected reconstruction missing"); fails++; }
  else approx(rad.rangeRatio, 1, 0.02, "aspect-corrected: pure rotation ⇒ flat range (ratio ~1)");
}

if (fails) { console.error(`\ntrajcheck: ${fails} assertion(s) failed`); process.exit(1); }
console.log("trajcheck: all assertions passed");
