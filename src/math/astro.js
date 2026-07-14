/* ============================================================
   ASTRONOMY — SunCalc algorithms, inlined.
   The Sun and Moon are the free calibration anchors: their az/el at
   the sighting time and place are known exactly, so a photo that
   contains either one can be pointed without trusting a compass.
   ============================================================ */

import { R2D, RAD } from "./geodesy.js";

const DAYMS = 86400000, J1970 = 2440588, J2000 = 2451545, EOB = RAD * 23.4397;
const toDays = (ms) => ms / DAYMS - 0.5 + J1970 - J2000;
const rightAsc = (l, b) => Math.atan2(Math.sin(l) * Math.cos(EOB) - Math.tan(b) * Math.sin(EOB), Math.cos(l));
const declin = (l, b) => Math.asin(Math.sin(b) * Math.cos(EOB) + Math.cos(b) * Math.sin(EOB) * Math.sin(l));
const azimC = (H, phi, dec) => Math.atan2(Math.sin(H), Math.cos(H) * Math.sin(phi) - Math.tan(dec) * Math.cos(phi));
const altdC = (H, phi, dec) => Math.asin(Math.sin(phi) * Math.sin(dec) + Math.cos(phi) * Math.cos(dec) * Math.cos(H));
const sidereal = (d, lw) => RAD * (280.16 + 360.9856235 * d) - lw;
const refractionC = (h) => { if (h < 0) h = 0; return 0.0002967 / Math.tan(h + 0.00312536 / (h + 0.08901179)); };

function sunCoordsC(d) { const M = RAD * (357.5291 + 0.98560028 * d); const L = M + RAD * (1.9148 * Math.sin(M) + 0.02 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M)) + RAD * 102.9372 + Math.PI; return { dec: declin(L, 0), ra: rightAsc(L, 0) }; }
function moonCoordsC(d) { const L = RAD * (218.316 + 13.176396 * d), M = RAD * (134.963 + 13.064993 * d), F = RAD * (93.272 + 13.22935 * d); const l = L + RAD * 6.289 * Math.sin(M), b = RAD * 5.128 * Math.sin(F), dt = 385001 - 20905 * Math.cos(M); return { ra: rightAsc(l, b), dec: declin(l, b), dist: dt }; }

export function sunPos(ms, lat, lng) { const lw = RAD * -lng, phi = RAD * lat, d = toDays(ms), c = sunCoordsC(d), H = sidereal(d, lw) - c.ra; return { az: (azimC(H, phi, c.dec) * R2D + 180 + 360) % 360, alt: altdC(H, phi, c.dec) * R2D }; }
export function moonPos(ms, lat, lng) { const lw = RAD * -lng, phi = RAD * lat, d = toDays(ms), c = moonCoordsC(d), H = sidereal(d, lw) - c.ra; let h = altdC(H, phi, c.dec); h += refractionC(h); return { az: (azimC(H, phi, c.dec) * R2D + 180 + 360) % 360, alt: h * R2D }; }
export function moonFrac(ms) { const d = toDays(ms), s = sunCoordsC(d), m = moonCoordsC(d), sd = 149598000; const phi = Math.acos(Math.sin(s.dec) * Math.sin(m.dec) + Math.cos(s.dec) * Math.cos(m.dec) * Math.cos(s.ra - m.ra)); const inc = Math.atan2(sd * Math.sin(phi), m.dist - sd * Math.cos(phi)); return (1 + Math.cos(inc)) / 2; }
