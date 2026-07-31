/* ============================================================
   ASTRONOMY — SunCalc algorithms, inlined.
   The Sun and Moon are the free calibration anchors: their az/el at
   the sighting time and place are known exactly, so a photo that
   contains either one can be pointed without trusting a compass.
   ============================================================ */

import { R2D, RAD } from "./geodesy.js";

const DAYMS = 86400000, J1970 = 2440588, J2000 = 2451545;
const toDays = (ms) => ms / DAYMS - 0.5 + J1970 - J2000;
const cy = (d) => d / 36525;                       // Julian centuries from J2000

/* Obliquity OF DATE. It was a fixed 23.4397° (the J2000 value); it drifts
   −0.0130°/century, which is 0.035° by 2026 — small on its own but part of the
   same frame error as the precession below. */
const eob = (d) => RAD * (23.439291 - 0.0130042 * cy(d));

/* GENERAL PRECESSION in ecliptic longitude since J2000, in degrees. The whole
   astronomical layer used to be computed in a J2000-mean frame and then read as
   if it were of-date; by 2026 that is a 0.37° error on every star, the Sun and
   the Moon (docs/MATH-AUDIT.md finding 3/4). It is very nearly a rigid rotation
   of the sky, which is exactly why a plate solve absorbed it into the pose and
   still reported a ~0.04° residual. */
const precLon = (d) => { const T = cy(d); return 1.396971 * T + 0.0003086 * T * T; };

/* Bennett's refraction (degrees) for a TRUE altitude in degrees — what the
   atmosphere adds to an astronomical object's apparent height. It was applied
   to the Moon and nothing else, leaving the layers on one dome mutually
   inconsistent (finding 6); every body now goes through it. */
export const refractionDeg = (hDeg) => (hDeg < -1 ? 0 : (1 / Math.tan((hDeg + 7.31 / (hDeg + 4.4)) * RAD)) / 60);

/* J2000 mean RA/Dec → mean of date (IAU 1976 / Lieske). The star catalogs are
   J2000; this is what makes them agree with the of-date planets. */
export function precessFromJ2000(raDeg, decDeg, ms) {
  const T = cy(toDays(ms)), S = RAD / 3600;
  const z1 = (2306.2181 * T + 0.30188 * T * T + 0.017998 * T ** 3) * S;
  const z2 = (2306.2181 * T + 1.09468 * T * T + 0.018203 * T ** 3) * S;
  const th = (2004.3109 * T - 0.42665 * T * T - 0.041833 * T ** 3) * S;
  const ra = raDeg * RAD, dec = decDeg * RAD;
  const v = [Math.cos(dec) * Math.cos(ra), Math.cos(dec) * Math.sin(ra), Math.sin(dec)];
  const rz = (u, a) => [u[0] * Math.cos(a) - u[1] * Math.sin(a), u[0] * Math.sin(a) + u[1] * Math.cos(a), u[2]];
  const ry = (u, a) => [u[0] * Math.cos(a) + u[2] * Math.sin(a), u[1], -u[0] * Math.sin(a) + u[2] * Math.cos(a)];
  const w = rz(ry(rz(v, z1), -th), z2);
  const m = Math.max(-1, Math.min(1, w[2]));
  return { ra: ((Math.atan2(w[1], w[0]) * R2D) + 360) % 360, dec: Math.asin(m) * R2D };
}
const rightAsc = (l, b, e) => Math.atan2(Math.sin(l) * Math.cos(e) - Math.tan(b) * Math.sin(e), Math.cos(l));
const declin = (l, b, e) => Math.asin(Math.sin(b) * Math.cos(e) + Math.cos(b) * Math.sin(e) * Math.sin(l));
const azimC = (H, phi, dec) => Math.atan2(Math.sin(H), Math.cos(H) * Math.sin(phi) - Math.tan(dec) * Math.cos(phi));
const altdC = (H, phi, dec) => Math.asin(Math.sin(phi) * Math.sin(dec) + Math.cos(phi) * Math.cos(dec) * Math.cos(H));
const sidereal = (d, lw) => RAD * (280.16 + 360.9856235 * d) - lw;

/* The mean-anomaly + equation-of-centre series is referred to J2000 (the
   perihelion longitude 102.9372° carries no rate), so precLon(d) carries it to
   the equinox of date — without it the Sun sat 0.37° from where it really was,
   and 0.46° from the app's own of-date planet ephemeris. */
function sunCoordsC(d) {
  const e = eob(d);
  const M = RAD * (357.5291 + 0.98560028 * d);
  const L = M + RAD * (1.9148 * Math.sin(M) + 0.02 * Math.sin(2 * M) + 0.0003 * Math.sin(3 * M))
    + RAD * (102.9372 + precLon(d)) + Math.PI;
  return { dec: declin(L, 0, e), ra: rightAsc(L, 0, e) };
}
/* The Moon used to keep ONLY the equation of the centre (6.289 sin M), which
   left it a mean 0.82° and worst 1.19° from the truth — wider than its own
   0.52° disc, on a body the manual calls "the strongest quick check that your
   bearing is right". These are the principal periodic terms of Meeus ch. 47
   (evection, the variation, the annual equation and the rest), which brings it
   inside ~0.05°. Arguments are of-date, so no precession term is needed. */
function moonCoordsC(d) {
  const e = eob(d), T = cy(d), S = (x) => Math.sin(x * RAD);
  const Lp = 218.3164477 + 13.176396474 * d, D = 297.8501921 + 12.190749117 * d;
  const M = 357.5291092 + 0.98560028 * d, Mp = 134.9633964 + 13.064992953 * d;
  const F = 93.2720950 + 13.229350449 * d, E = 1 - 0.002516 * T;
  const dL = 6.288774 * S(Mp) + 1.274027 * S(2 * D - Mp) + 0.658314 * S(2 * D) + 0.213618 * S(2 * Mp)
    - 0.185116 * E * S(M) - 0.114332 * S(2 * F) + 0.058793 * S(2 * D - 2 * Mp)
    + 0.057066 * E * S(2 * D - M - Mp) + 0.053322 * S(2 * D + Mp) + 0.045758 * E * S(2 * D - M)
    - 0.040923 * E * S(M - Mp) - 0.034720 * S(D) - 0.030383 * E * S(M + Mp)
    + 0.015327 * S(2 * D - 2 * F) - 0.012528 * S(Mp + 2 * F) + 0.010980 * S(Mp - 2 * F)
    + 0.010675 * S(4 * D - Mp) + 0.010034 * S(3 * Mp) + 0.008548 * S(4 * D - 2 * Mp);
  const B = 5.128122 * S(F) + 0.280602 * S(Mp + F) + 0.277693 * S(Mp - F) + 0.173237 * S(2 * D - F)
    + 0.055413 * S(2 * D - Mp + F) + 0.046271 * S(2 * D - Mp - F) + 0.032573 * S(2 * D + F)
    + 0.017198 * S(2 * Mp + F) + 0.009266 * S(2 * D + Mp - F) + 0.008822 * S(2 * Mp - F)
    + 0.008216 * E * S(2 * D - M - F) + 0.004324 * S(2 * D - 2 * Mp - F);
  /* distance (km) — the principal cosine terms; feeds the illuminated fraction */
  const dt = 385000.56 - 20905.355 * Math.cos(Mp * RAD) - 3699.111 * Math.cos((2 * D - Mp) * RAD)
    - 2955.968 * Math.cos(2 * D * RAD) - 569.925 * Math.cos(2 * Mp * RAD);
  const l = (Lp + dL) * RAD, b = B * RAD;
  return { ra: rightAsc(l, b, e), dec: declin(l, b, e), dist: dt };
}

export function sunPos(ms, lat, lng) {
  const lw = RAD * -lng, phi = RAD * lat, d = toDays(ms), c = sunCoordsC(d), H = sidereal(d, lw) - c.ra;
  const alt = altdC(H, phi, c.dec) * R2D;
  return { az: (azimC(H, phi, c.dec) * R2D + 180 + 360) % 360, alt: alt + refractionDeg(alt) };
}
/* RA/Dec OF DATE (planets) → APPARENT az/el at a time and place. The altitude
   includes atmospheric refraction, because that is where the object actually
   appears — which is what a photo shows and what a plate solve must match.
   It used to be geometric here while moonPos() refracted, so the layers on one
   dome disagreed with each other (docs/MATH-AUDIT.md finding 6). */
export function raDecToAzEl(raDeg, decDeg, ms, lat, lng) {
  const lw = RAD * -lng, phi = RAD * lat, d = toDays(ms);
  const H = sidereal(d, lw) - raDeg * RAD, dec = decDeg * RAD;
  const alt = altdC(H, phi, dec) * R2D;
  return { az: (azimC(H, phi, dec) * R2D + 180 + 360) % 360, alt: alt + refractionDeg(alt) };
}
/* J2000 RA/Dec (the star catalogs, meteor radiants) → apparent az/el. Precesses
   to the equinox of date first — without that the whole star field sat ~0.37°
   from the truth AND from the app's own of-date planets. Every J2000 catalog
   must go through THIS, and anything already of-date through raDecToAzEl. */
export function starAzEl(ra2000, dec2000, ms, lat, lng) {
  const p = precessFromJ2000(ra2000, dec2000, ms);
  return raDecToAzEl(p.ra, p.dec, ms, lat, lng);
}
export function moonPos(ms, lat, lng) {
  const lw = RAD * -lng, phi = RAD * lat, d = toDays(ms), c = moonCoordsC(d), H = sidereal(d, lw) - c.ra;
  const alt = altdC(H, phi, c.dec) * R2D;                 // one refraction model for every body
  return { az: (azimC(H, phi, c.dec) * R2D + 180 + 360) % 360, alt: alt + refractionDeg(alt) };
}
export function moonFrac(ms) { const d = toDays(ms), s = sunCoordsC(d), m = moonCoordsC(d), sd = 149598000; const phi = Math.acos(Math.sin(s.dec) * Math.sin(m.dec) + Math.cos(s.dec) * Math.cos(m.dec) * Math.cos(s.ra - m.ra)); const inc = Math.atan2(sd * Math.sin(phi), m.dist - sd * Math.cos(phi)); return (1 + Math.cos(inc)) / 2; }
