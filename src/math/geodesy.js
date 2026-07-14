/* ============================================================
   GEODESY & VECTORS
   Local ENU frame via equirectangular projection about a reference
   point — fine under ~100 km, which is far beyond any sighting
   baseline. Directions are unit vectors in that ENU frame:
   x = east, y = north, z = up.
   ============================================================ */

export const D2R = Math.PI / 180, R2D = 180 / Math.PI, RE = 6371000;

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
  const e = (lon - ref.lon) * D2R * RE * Math.cos(ref.lat * D2R);
  const n = (lat - ref.lat) * D2R * RE;
  return [e, n, (alt || 0) - (ref.alt || 0)];
}
export function geoFromEnu(p, ref) {
  return {
    lat: ref.lat + (p[1] / RE) * R2D,
    lon: ref.lon + (p[0] / (RE * Math.cos(ref.lat * D2R))) * R2D,
    alt: (ref.alt || 0) + p[2],
  };
}
export function dirFromAzEl(azDeg, elDeg) {
  const a = azDeg * D2R, e = elDeg * D2R;
  return [Math.sin(a) * Math.cos(e), Math.cos(a) * Math.cos(e), Math.sin(e)];
}
export const dirToAzEl = (d) => ({ az: ((Math.atan2(d[0], d[1]) * R2D) + 360) % 360, el: Math.asin(clampN(d[2], -1, 1)) * R2D });
