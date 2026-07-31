/* ============================================================
   MATH ACCURACY AUDIT — measures the SHIPPED math against exact ground truth
   and independent references, and prints the size of every systematic error
   it finds. This is a REPORT, not a pass/fail gate: it exists so the numbers
   in docs/MATH-AUDIT.md can be re-derived on demand, and so the effect of any
   fix can be measured rather than asserted.

   Run: npm run mathaudit
   ============================================================ */
import { analyze, intersectLines } from "../src/math/triangulate.js";
import { raDecToAzEl, sunPos, moonPos } from "../src/math/astro.js";
import { STARS } from "../src/math/starcat.js";
import { angSizeFromPoints, pixToDirK } from "../src/math/projection.js";

const D2R = Math.PI / 180, R2D = 180 / Math.PI, RE = 6371000;
const A = 6378137.0, F = 1 / 298.257223563, E2 = F * (2 - F);
const hr = (t) => console.log(`\n${"═".repeat(78)}\n${t}\n${"═".repeat(78)}`);

/* ---------- exact WGS84 helpers (the reference, not the app) ---------- */
const ecef = (lat, lon, h) => {
  const p = lat * D2R, l = lon * D2R, s = Math.sin(p), N = A / Math.sqrt(1 - E2 * s * s);
  return [(N + h) * Math.cos(p) * Math.cos(l), (N + h) * Math.cos(p) * Math.sin(l), (N * (1 - E2) + h) * s];
};
const basis = (lat, lon) => {
  const p = lat * D2R, l = lon * D2R;
  return { E: [-Math.sin(l), Math.cos(l), 0],
    N: [-Math.sin(p) * Math.cos(l), -Math.sin(p) * Math.sin(l), Math.cos(p)],
    U: [Math.cos(p) * Math.cos(l), Math.cos(p) * Math.sin(l), Math.sin(p)] };
};
const dotv = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const trueAzEl = (o, t) => {
  const O = ecef(o.lat, o.lon, o.alt), P = ecef(t.lat, t.lon, t.alt);
  const v = [P[0] - O[0], P[1] - O[1], P[2] - O[2]], b = basis(o.lat, o.lon);
  const e = dotv(v, b.E), n = dotv(v, b.N), u = dotv(v, b.U);
  return { az: ((Math.atan2(e, n) * R2D) + 360) % 360, el: Math.atan2(u, Math.hypot(e, n)) * R2D, range: Math.hypot(e, n, u) };
};
const M_rad = (lat) => { const s = Math.sin(lat * D2R); return A * (1 - E2) / Math.pow(1 - E2 * s * s, 1.5); };
const N_rad = (lat) => { const s = Math.sin(lat * D2R); return A / Math.sqrt(1 - E2 * s * s); };
const offset = (ref, east, north) => ({
  lat: ref.lat + (north / M_rad(ref.lat)) * R2D,
  lon: ref.lon + (east / (N_rad(ref.lat) * Math.cos(ref.lat * D2R))) * R2D, alt: ref.alt });
const dist3 = (a, b) => { const p = ecef(a.lat, a.lon, a.alt), q = ecef(b.lat, b.lon, b.alt); return Math.hypot(q[0] - p[0], q[1] - p[1], q[2] - p[2]); };

/* ============================ 1. GEODESY ============================ */
hr("1 · GEODESY — analyze() driven with EXACT ground truth");
console.log(`
Truth is built on the WGS84 ellipsoid: an object at a known geodetic position,
each observer's az/el computed from its OWN true local vertical — the angles a
perfect instrument would read. Any error below is model error, not noise.
`);
console.log("SCENARIO                        baseline    range    3-D pos err   alt err   size err   ray-miss   grade");
const geoCases = [
  ["weathervane (field case)", 60, 120, 12, 42.3],
  ["drone over a field", 400, 1200, 300, 42.3],
  ["aircraft, 5 km baseline", 5000, 12000, 3000, 42.3],
  ["high object, 20 km baseline", 20000, 40000, 10000, 42.3],
  ["high object, 50 km baseline", 50000, 90000, 12000, 42.3],
  ["5 km baseline, equator", 5000, 12000, 3000, 0.5],
  ["5 km baseline, 60°N", 5000, 12000, 3000, 60.0],
];
for (const [name, base, rng, dalt, lat] of geoCases) {
  const o1 = { lat, lon: -122.90, alt: 400 }, o2 = offset(o1, base, 0);
  const obj = { ...offset(offset(o1, base / 2, 0), 0, rng), alt: 400 + dalt };
  const t1 = trueAzEl(o1, obj), t2 = trueAzEl(o2, obj);
  const trueSize = 4.0, ang = (r) => 2 * Math.atan(trueSize / 2 / r) * R2D;
  const r = analyze([
    { name: "A", lat: o1.lat, lon: o1.lon, alt: o1.alt, A: { az: t1.az, el: t1.el, angManual: ang(t1.range) }, B: {} },
    { name: "B", lat: o2.lat, lon: o2.lon, alt: o2.alt, A: { az: t2.az, el: t2.el, angManual: ang(t2.range) }, B: {} },
  ]);
  if (!r.ok) { console.log(`${name}: FIX FAILED`); continue; }
  const err = dist3({ lat: r.geoA.lat, lon: r.geoA.lon, alt: r.geoA.alt }, obj);
  console.log(`${name.padEnd(30)} ${(base / 1000).toFixed(1).padStart(6)}km ${(rng / 1000).toFixed(1).padStart(7)}km ` +
    `${err.toFixed(2).padStart(11)}m ${(r.geoA.alt - obj.alt).toFixed(1).padStart(8)}m ` +
    `${((r.sizeAvg - trueSize) * 100).toFixed(2).padStart(8)}cm ${r.solA.rmsMiss.toFixed(3).padStart(9)}m   ${r.rating}`);
}
console.log(`
CAUSE 1 — enuFromGeo uses one spherical radius (RE=6371 km) for both axes. The
correct local scales are the ellipsoid's radii of curvature, which differ from
RE and FROM EACH OTHER:`);
console.log("      lat      north scale err   east scale err   bearing skew");
for (const lat of [0, 30, 42.3, 60]) {
  console.log(`     ${String(lat).padStart(5)}°       ${((M_rad(lat) - RE) / RE * 100).toFixed(3).padStart(7)}%        ${((N_rad(lat) - RE) / RE * 100).toFixed(3).padStart(6)}%        ` +
    `${((N_rad(lat) - M_rad(lat)) / (2 * RE) * R2D).toFixed(3)}°`);
}
console.log(`
CAUSE 2 — every observer's az/el is measured against its OWN local vertical,
but analyze() reads them all in the reference observer's flat tangent frame.
Over a baseline b the verticals diverge by b/R:`);
for (const b of [1000, 5000, 20000, 50000])
  console.log(`     ${(b / 1000).toFixed(0).padStart(3)} km baseline → ${((b / RE) * R2D).toFixed(4)}° of elevation bias on the far observer`);
console.log(`
Neither shows up in the ray-miss residual — every ray is distorted together, so
they still meet. That is why the runs above are graded "excellent" while wrong.`);

/* ==================== 2. ASTRONOMICAL FRAME ==================== */
hr("2 · ASTRONOMY — reference frame and ephemeris");
const MS = Date.UTC(2026, 6, 30, 6, 0, 0), LAT = 42.30, LON = -122.90;
function precess(raDeg, decDeg, ms) {
  const T = (ms / 86400000 + 2440587.5 - 2451545.0) / 36525, S = D2R / 3600;
  const z1 = (2306.2181 * T + 0.30188 * T * T) * S, z2 = (2306.2181 * T + 1.09468 * T * T) * S, th = (2004.3109 * T - 0.42665 * T * T) * S;
  const ra = raDeg * D2R, dec = decDeg * D2R;
  const v = [Math.cos(dec) * Math.cos(ra), Math.cos(dec) * Math.sin(ra), Math.sin(dec)];
  const rz = (u, a) => [u[0] * Math.cos(a) - u[1] * Math.sin(a), u[0] * Math.sin(a) + u[1] * Math.cos(a), u[2]];
  const ry = (u, a) => [u[0] * Math.cos(a) + u[2] * Math.sin(a), u[1], -u[0] * Math.sin(a) + u[2] * Math.cos(a)];
  const w = rz(ry(rz(v, z1), -th), z2);
  return { ra: ((Math.atan2(w[1], w[0]) * R2D) + 360) % 360, dec: Math.asin(Math.max(-1, Math.min(1, w[2]))) * R2D };
}
const refr = (h) => h < -1 ? 0 : (1 / Math.tan((h + 7.31 / (h + 4.4)) * D2R)) / 60;
const dirOf = (az, el) => [Math.sin(az * D2R) * Math.cos(el * D2R), Math.cos(az * D2R) * Math.cos(el * D2R), Math.sin(el * D2R)];
const sep = (p, q) => { const a = dirOf(p.az, p.alt), b = dirOf(q.az, q.alt); return Math.acos(Math.max(-1, Math.min(1, dotv(a, b)))) * R2D; };
const vis = [];
for (const [ra, dec, mag, name] of STARS) {
  const app = raDecToAzEl(ra, dec, MS, LAT, LON);
  if (app.alt < 12 || mag > 3.0) continue;
  const pd = precess(ra, dec, MS), t0 = raDecToAzEl(pd.ra, pd.dec, MS, LAT, LON);
  vis.push({ name: name || "—", app, truth: { az: t0.az, alt: t0.alt + refr(t0.alt) }, prec: sep(app, t0), el: t0.alt });
}
const mean = (a) => a.reduce((x, y) => x + y, 0) / a.length;
console.log(`\nSTARS — the catalog is J2000; raDecToAzEl treats it as of-date.`);
console.log(`  ${vis.length} stars above 12°:  mean offset ${mean(vis.map((s) => s.prec)).toFixed(4)}°   max ${Math.max(...vis.map((s) => s.prec)).toFixed(4)}°`);
/* does a plate solve hide it? fit the best rigid rotation (= az/el/roll) */
const solve3x3 = (Ar, b) => {
  const Mx = Ar.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < 3; c++) {
    let p = c; for (let r = c + 1; r < 3; r++) if (Math.abs(Mx[r][c]) > Math.abs(Mx[p][c])) p = r;
    if (Math.abs(Mx[p][c]) < 1e-15) return null;
    [Mx[c], Mx[p]] = [Mx[p], Mx[c]];
    for (let r = 0; r < 3; r++) { if (r === c) continue; const f = Mx[r][c] / Mx[c][c]; for (let k = c; k < 4; k++) Mx[r][k] -= f * Mx[c][k]; }
  }
  return [Mx[0][3] / Mx[0][0], Mx[1][3] / Mx[1][1], Mx[2][3] / Mx[2][2]];
};
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
function fitRot(Fv, Tv) {
  const Ar = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], b = [0, 0, 0];
  for (let i = 0; i < Fv.length; i++) {
    const f = Fv[i], S = [[0, -f[2], f[1]], [f[2], 0, -f[0]], [-f[1], f[0], 0]];
    const d = [Tv[i][0] - f[0], Tv[i][1] - f[1], Tv[i][2] - f[2]];
    for (let r = 0; r < 3; r++) {
      for (let c = 0; c < 3; c++) Ar[r][c] += S[0][r] * S[0][c] + S[1][r] * S[1][c] + S[2][r] * S[2][c];
      b[r] += -(S[0][r] * d[0] + S[1][r] * d[1] + S[2][r] * d[2]);
    }
  }
  const w = solve3x3(Ar, b); if (!w) return null;
  let s2 = 0;
  for (let i = 0; i < Fv.length; i++) { const f = Fv[i], c = cross(w, f); s2 += (f[0] + c[0] - Tv[i][0]) ** 2 + (f[1] + c[1] - Tv[i][1]) ** 2 + (f[2] + c[2] - Tv[i][2]) ** 2; }
  return { rms: Math.sqrt(s2 / Fv.length) * R2D, ang: Math.hypot(w[0], w[1], w[2]) * R2D };
}
const fit = fitRot(vis.map((s) => dirOf(s.app.az, s.app.alt)), vis.map((s) => dirOf(s.truth.az, s.truth.alt)));
console.log(`\n  A pose has 3 rotational DOF, so a plate solve ABSORBS a rigid offset:`);
console.log(`    residual the solver reports : ${fit.rms.toFixed(4)}° rms   ← looks excellent`);
console.log(`    rotation it silently absorbs: ${fit.ang.toFixed(4)}°       ← lands on the object's direction`);
console.log(`  The app's own field result — "72 stars at 0.04° rms" — is this number.`);

/* the app's two internal solar longitudes */
const rev = (x) => ((x % 360) + 360) % 360;
const lonAstro = (ms) => { const d = ms / 86400000 - 0.5 + 2440588 - 2451545, M = 357.5291 + 0.98560028 * d;
  return rev(M + 1.9148 * Math.sin(M * D2R) + 0.02 * Math.sin(2 * M * D2R) + 0.0003 * Math.sin(3 * M * D2R) + 102.9372 + 180); };
const lonSch = (ms) => { const d = ms / 86400000 + 2440587.5 - 2451543.5;
  const w = 282.9404 + 4.70935e-5 * d, e = 0.016709 - 1.151e-9 * d, M = rev(356.0470 + 0.9856002585 * d);
  let E = M + e * R2D * Math.sin(M * D2R) * (1 + e * Math.cos(M * D2R));
  for (let k = 0; k < 8; k++) E = E - (E - e * R2D * Math.sin(E * D2R) - M) / (1 - e * Math.cos(E * D2R));
  return rev(Math.atan2(Math.sin(E * D2R) * Math.sqrt(1 - e * e), Math.cos(E * D2R) - e) * R2D + w); };
let dd = lonAstro(MS) - lonSch(MS); while (dd > 180) dd -= 360; while (dd < -180) dd += 360;
console.log(`\nINTERNAL INCONSISTENCY — the app carries two solar ephemerides:`);
console.log(`  astro.js (Sun/Moon/stars) ${lonAstro(MS).toFixed(4)}°   vs   planets.js ${lonSch(MS).toFixed(4)}°   → ${Math.abs(dd).toFixed(4)}° apart`);
console.log(`  astro.js fixes the perihelion longitude at its J2000 value; the Schlyter`);
console.log(`  elements carry secular rates (equinox of date). Both are self-consistent;`);
console.log(`  they are drawn on the SAME dome, so stars and planets sit ~0.4° apart.`);

/* Moon */
const sinD = (d) => Math.sin(d * D2R);
function moonRef(ms) {
  const T = (ms / 86400000 + 2440587.5 - 2451545.0) / 36525, EOB = 23.4397 * D2R;
  const Lp = 218.3164477 + 481267.88123421 * T, D = 297.8501921 + 445267.1114034 * T;
  const M = 357.5291092 + 35999.0502909 * T, Mp = 134.9633964 + 477198.8675055 * T, Fa = 93.2720950 + 483202.0175233 * T;
  const dL = 6.288774 * sinD(Mp) + 1.274027 * sinD(2 * D - Mp) + 0.658314 * sinD(2 * D) + 0.213618 * sinD(2 * Mp)
    - 0.185116 * sinD(M) - 0.114332 * sinD(2 * Fa) + 0.058793 * sinD(2 * D - 2 * Mp) + 0.057066 * sinD(2 * D - M - Mp)
    + 0.053322 * sinD(2 * D + Mp) + 0.045758 * sinD(2 * D - M) - 0.040923 * sinD(M - Mp) - 0.034720 * sinD(D);
  const B = 5.128122 * sinD(Fa) + 0.280602 * sinD(Mp + Fa) + 0.277693 * sinD(Mp - Fa) + 0.173237 * sinD(2 * D - Fa)
    + 0.055413 * sinD(2 * D - Mp + Fa) + 0.046271 * sinD(2 * D - Mp - Fa);
  const lam = (Lp + dL) * D2R, bet = B * D2R;
  return { ra: Math.atan2(Math.sin(lam) * Math.cos(EOB) - Math.tan(bet) * Math.sin(EOB), Math.cos(lam)) * R2D,
    dec: Math.asin(Math.sin(bet) * Math.cos(EOB) + Math.cos(bet) * Math.sin(EOB) * Math.sin(lam)) * R2D };
}
let mw = 0, mt = 0, mn = 0;
for (let day = 0; day < 30; day++) {
  const ms = Date.UTC(2026, 6, 1 + day, 6), app = moonPos(ms, LAT, LON);
  if (app.alt < 5) continue;
  const r = moonRef(ms), ref = raDecToAzEl(r.ra, r.dec, ms, LAT, LON);
  const s = sep(app, { az: ref.az, alt: ref.alt });
  mw = Math.max(mw, s); mt += s; mn++;
}
console.log(`\nMOON — astro.js keeps only the equation of the centre; evection (1.27°),`);
console.log(`  the variation (0.66°) and the annual equation (0.19°) are omitted.`);
console.log(`  vs a truncated ELP over one lunation: mean ${(mt / mn).toFixed(3)}°, worst ${mw.toFixed(3)}°`);
console.log(`  (the Moon's disc is 0.52° wide — the error exceeds the anchor itself)`);
console.log(`\nREFRACTION — moonPos() applies it; raDecToAzEl() (stars, planets, the Sun)`);
console.log(`  does not, so the layers are mutually inconsistent by ${refr(15).toFixed(3)}° at 15° up.`);

/* ==================== 3. ANGULAR SIZE ==================== */
hr("3 · ANGULAR SIZE — the primary measurement");
console.log(`
angSizeFromPoints() builds both rays through a PINHOLE (fov only). The plate
solve fits and stores a radial term k, and pixToDirK uses it — so the app knows
the lens is not a pinhole, then measures the object as if it were. True size is
2·d·tan(θ/2), so the size error equals the angular error exactly.
`);
const natW = 4032, natH = 3024, fov = 68;
console.log("   object position          k=-0.10   k=-0.05   k=+0.05");
for (const [nm, cx, cy] of [["dead centre", natW / 2, natH / 2], ["1/4 out", natW * 0.375, natH / 2],
  ["half way to edge", natW * 0.25, natH / 2], ["near the corner", natW * 0.10, natH * 0.12]]) {
  const p1 = { x: cx - 30, y: cy }, p2 = { x: cx + 30, y: cy };
  const naive = angSizeFromPoints(p1, p2, natW, natH, fov);
  const cols = [-0.10, -0.05, 0.05].map((k) => {
    const a = pixToDirK(p1.x, p1.y, natW, natH, 0, 0, 0, fov, k), b = pixToDirK(p2.x, p2.y, natW, natH, 0, 0, 0, fov, k);
    const t = Math.acos(Math.max(-1, Math.min(1, dotv(a, b)))) * R2D;
    return `${((naive - t) / t * 100).toFixed(2).padStart(7)}%`;
  });
  console.log(`   ${nm.padEnd(22)} ${cols.join("   ")}`);
}

/* ==================== 4. MODEL CONSISTENCY ==================== */
hr("4 · MODEL CONSISTENCY — measurement vs cross-check");
console.log(`
adsb.js drops predicted aircraft by earth curvature with standard refraction
(k≈0.13); terrain.js ray-marches with the same. triangulate.js — the path the
witness sight-line itself takes — models neither. The two are then DIFFERENCED
to rank candidates.
`);
console.log("   apparent el   refraction bends the ray   cross-range error at 20 km");
for (const el of [2, 5, 10, 20, 45])
  console.log(`   ${String(el).padStart(10)}°   ${refr(el).toFixed(4)}°                  ${(refr(el) * D2R * 20000).toFixed(1)} m`);
console.log(`   curvature drop over 20 km (k=0.13): ${(20000 * 20000 * 0.87 / (2 * RE)).toFixed(1)} m  = ${(Math.atan(20000 * 20000 * 0.87 / (2 * RE) / 20000) * R2D).toFixed(4)}°`);

hr("SUMMARY");
console.log(`
  #  finding                                              typical      worst
  1  ENU built on a sphere, not the ellipsoid             0.1–0.3%     0.56%
  2  local-vertical convergence ignored                   0.045°/5km   0.45°/50km
  3  star catalog J2000 used as of-date                   0.29°        0.37°
  4  stars/Sun vs planets in different equinoxes          0.46°        0.46°
  5  Moon ephemeris truncated to one term                 0.81°        1.16°
  6  refraction on the Moon only                          0.02°        0.30°
  7  angular size ignores the fitted lens term k          0.4–3%       8.1%
  8  sight-lines unrefracted, cross-checks refracted      0.09°@10°    0.30°@2°

  None of these are noise: they are model errors that repeat every time and do
  not average out with more witnesses. None of them raise the ray-miss residual,
  so the quality grade stays "excellent" while they are present.
`);
