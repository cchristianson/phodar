/* ============================================================
   FORMATTING — readouts are dual-unit on purpose: the instrument
   is metric, the witnesses are usually not.
   ============================================================ */

export const isNum = (v) => v !== "" && v !== null && v !== undefined && isFinite(+v);
export const n1 = (v) => (Math.abs(v) >= 100 ? Math.round(v).toLocaleString() : (+v).toFixed(1));

/* global unit system — module state so every formatter call site follows the
   toggle without prop-drilling; App re-renders the whole tree on change */
let IMPERIAL = false;
export const setImperialUnits = (v) => { IMPERIAL = !!v; };
export const isImperialUnits = () => IMPERIAL;

export function fmtLen(m) {
  if (!isFinite(m)) return "—";
  const ft = m * 3.28084;
  if (IMPERIAL) {
    if (Math.abs(ft) < 5280) return `${n1(ft)} ft · ${n1(m)} m`;
    return `${(m / 1609.344).toFixed(2)} mi · ${(m / 1000).toFixed(2)} km`;
  }
  if (Math.abs(m) < 1000) return `${n1(m)} m · ${n1(ft)} ft`;
  return `${(m / 1000).toFixed(2)} km · ${(m / 1609.344).toFixed(2)} mi`;
}
export function fmtLenShort(m) {
  if (!isFinite(m)) return "—";
  if (IMPERIAL) {
    const ft = m * 3.28084;
    return Math.abs(ft) < 5280 ? `${n1(ft)} ft` : `${(m / 1609.344).toFixed(2)} mi`;
  }
  return Math.abs(m) < 1000 ? `${n1(m)} m` : `${(m / 1000).toFixed(2)} km`;
}
export function fmtSpeed(ms) {
  if (!isFinite(ms)) return "—";
  if (IMPERIAL) return `${n1(ms * 2.23694)} mph · ${n1(ms * 3.6)} km/h · ${n1(ms)} m/s`;
  return `${n1(ms)} m/s · ${n1(ms * 3.6)} km/h · ${n1(ms * 2.23694)} mph`;
}
export const fmtDeg = (d) => (isFinite(d) ? `${(+d).toFixed(2)}°` : "—");
export const compass8 = (h) => ["N", "NE", "E", "SE", "S", "SW", "W", "NW"][((Math.round(h / 45) % 8) + 8) % 8]; // wrap-safe for negative headings (the horizon strip's window crosses 0°)
