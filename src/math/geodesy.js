/* ============================================================
   GEODESY & VECTORS
   Local ENU frame about a reference point, built EXACTLY on the WGS84
   ellipsoid via ECEF. Directions are unit vectors in that frame:
   x = east, y = north, z = up.

   It used to be an equirectangular projection about the reference with a
   single mean radius (RE), which was wrong in two ways that the audit
   measured (docs/MATH-AUDIT.md):

     · SCALE. The local scales are the ellipsoid's radii of curvature —
       N (prime vertical) east, M (meridional) north — which differ from
       6371 km by −0.56%…+0.43% AND from each other. Every observer
       baseline, and therefore every range, altitude and true size derived
       from it, carried that error; the anisotropy also skewed bearings by
       up to 0.19°.
     · CURVATURE. The frame was a flat plane, so a point at ground distance
       d sat at its full height instead of h − d²/2R.

   NOTE FOR CALLERS: the curvature drop is now INHERENT in the z that
   enuFromGeo returns. Anything that used to add its own d²/2R term on top
   (adsb.js did) must stop, or it double-counts.

   The companion error — every observer measuring az/el against its OWN
   local vertical while a shared frame assumes they are parallel — is
   handled by dirFromAzElAt() below, not here.
   ============================================================ */

export const D2R = Math.PI / 180, R2D = 180 / Math.PI, RE = 6371000;

/* WGS84 */
const WGS_A = 6378137.0, WGS_F = 1 / 298.257223563, WGS_E2 = WGS_F * (2 - WGS_F);

/* geodetic → earth-centred earth-fixed */
function ecefFromGeo(lat, lon, h) {
  const p = lat * D2R, l = lon * D2R, s = Math.sin(p);
  const N = WGS_A / Math.sqrt(1 - WGS_E2 * s * s);
  return [(N + h) * Math.cos(p) * Math.cos(l), (N + h) * Math.cos(p) * Math.sin(l), (N * (1 - WGS_E2) + h) * s];
}
/* the true local ENU basis (unit east / north / up) at a geodetic point */
function enuBasis(lat, lon) {
  const p = lat * D2R, l = lon * D2R, sp = Math.sin(p), cp = Math.cos(p), sl = Math.sin(l), cl = Math.cos(l);
  return { E: [-sl, cl, 0], N: [-sp * cl, -sp * sl, cp], U: [cp * cl, cp * sl, sp] };
}
/* ECEF → geodetic (Bowring seed + a few fixed-point steps; sub-mm) */
function geoFromEcef(P) {
  const [x, y, z] = P, lon = Math.atan2(y, x), p = Math.hypot(x, y);
  const ep2 = WGS_E2 / (1 - WGS_E2), b = WGS_A * Math.sqrt(1 - WGS_E2);
  const th = Math.atan2(z * WGS_A, p * b);
  let lat = Math.atan2(z + ep2 * b * Math.sin(th) ** 3, p - WGS_E2 * WGS_A * Math.cos(th) ** 3);
  for (let i = 0; i < 3; i++) {
    const s = Math.sin(lat), N = WGS_A / Math.sqrt(1 - WGS_E2 * s * s);
    lat = Math.atan2(z + WGS_E2 * N * s, p);
  }
  const s = Math.sin(lat), N = WGS_A / Math.sqrt(1 - WGS_E2 * s * s);
  const cl = Math.cos(lat);
  /* near the poles p→0, so fall back to the z form for altitude */
  const alt = Math.abs(cl) > 1e-9 ? p / cl - N : Math.abs(z) - N * (1 - WGS_E2);
  return { lat: lat * R2D, lon: lon * R2D, alt };
}

/* RAD is an alias for D2R, kept because the Sky Sense-derived astronomy
   and pose code was written against that name. */
export const RAD = Math.PI / 180;

export const clampN = (v, a, b) => Math.min(Math.max(v, a), b);

/* ---------- vector & geodesy helpers ---------- */
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
export const scl = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
export const mag = (a) => Math.sqrt(dot(a, a));
export const unit = (a) => { const m = mag(a) || 1; return scl(a, 1 / m); };

export function enuFromGeo(lat, lon, alt, ref) {
  const O = ecefFromGeo(+ref.lat, +ref.lon, +ref.alt || 0);
  const P = ecefFromGeo(+lat, +lon, +alt || 0);
  const b = enuBasis(+ref.lat, +ref.lon);
  const v = [P[0] - O[0], P[1] - O[1], P[2] - O[2]];
  return [dot(v, b.E), dot(v, b.N), dot(v, b.U)];
}
export function geoFromEnu(p, ref) {
  const O = ecefFromGeo(+ref.lat, +ref.lon, +ref.alt || 0), b = enuBasis(+ref.lat, +ref.lon);
  return geoFromEcef([
    O[0] + b.E[0] * p[0] + b.N[0] * p[1] + b.U[0] * p[2],
    O[1] + b.E[1] * p[0] + b.N[1] * p[1] + b.U[1] * p[2],
    O[2] + b.E[2] * p[0] + b.N[2] * p[1] + b.U[2] * p[2],
  ]);
}
export function dirFromAzEl(azDeg, elDeg) {
  const a = azDeg * D2R, e = elDeg * D2R;
  return [Math.sin(a) * Math.cos(e), Math.cos(a) * Math.cos(e), Math.sin(e)];
}
/* An observer's az/el is measured against ITS OWN local vertical and north.
   A shared multi-observer frame is anchored at ONE reference observer, and the
   two verticals diverge by baseline/R — 0.045° at 5 km, 0.18° at 20 km, 0.45°
   at 50 km. Treating them as parallel put a pure elevation bias on every
   non-reference sight-line (measured: −16 m of altitude at a 5 km baseline,
   −119 m at 20 km). This converts a local az/el into the reference frame
   exactly, by going out to ECEF and back.

   Returns dirFromAzEl unchanged when the observer IS the reference (or when no
   position is known), so every single-observer path — the sky dome, the photo
   pose, the video tracker — is bit-identical to before. */
export function dirFromAzElAt(azDeg, elDeg, lat, lon, ref) {
  const d = dirFromAzEl(azDeg, elDeg);
  if (!ref || !isFinite(lat) || !isFinite(lon) || !isFinite(ref.lat) || !isFinite(ref.lon)) return d;
  if (Math.abs(lat - ref.lat) < 1e-12 && Math.abs(lon - ref.lon) < 1e-12) return d;
  const o = enuBasis(+lat, +lon), r = enuBasis(+ref.lat, +ref.lon);
  /* local ENU → ECEF → reference ENU */
  const g = [
    o.E[0] * d[0] + o.N[0] * d[1] + o.U[0] * d[2],
    o.E[1] * d[0] + o.N[1] * d[1] + o.U[1] * d[2],
    o.E[2] * d[0] + o.N[2] * d[1] + o.U[2] * d[2],
  ];
  return [dot(g, r.E), dot(g, r.N), dot(g, r.U)];
}
export const dirToAzEl = (d) => ({ az: ((Math.atan2(d[0], d[1]) * R2D) + 360) % 360, el: Math.asin(clampN(d[2], -1, 1)) * R2D });
