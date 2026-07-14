/* ============================================================
   PLANETS — Schlyter low-precision ephemeris (stjarnhimlen.se/comp/
   ppcomp.html, coefficients transcribed 2026-07-14). Accuracy is a
   few arcminutes for Venus/Mars, ~0.1° for Jupiter/Saturn with the
   longitude perturbations included — far tighter than any witness
   sight-line. Venus is the single most-reported "UFO" there is, so
   these are calibration anchors AND mundane-explanation candidates.
   ============================================================ */

import { D2R, R2D } from "./geodesy.js";
import { raDecToAzEl } from "./astro.js";

const rev = (x) => ((x % 360) + 360) % 360;

/* orbital elements at d = days since J2000 epoch 2000 Jan 0.0 UT
   (JD 2451543.5): [N, i, w, a, e, M] each as [base, rate] */
const EL = {
  Mercury: [[48.3313, 3.24587e-5], [7.0047, 5.0e-8], [29.1241, 1.01444e-5], [0.387098, 0], [0.205635, 5.59e-10], [168.6562, 4.0923344368]],
  Venus: [[76.6799, 2.4659e-5], [3.3946, 2.75e-8], [54.8910, 1.38374e-5], [0.723330, 0], [0.006773, -1.302e-9], [48.0052, 1.6021302244]],
  Mars: [[49.5574, 2.11081e-5], [1.8497, -1.78e-8], [286.5016, 2.92961e-5], [1.523688, 0], [0.093405, 2.516e-9], [18.6021, 0.5240207766]],
  Jupiter: [[100.4542, 2.76854e-5], [1.3030, -1.557e-7], [273.8777, 1.64505e-5], [5.20256, 0], [0.048498, 4.469e-9], [19.8950, 0.0830853001]],
  Saturn: [[113.6634, 2.3898e-5], [2.4886, -1.081e-7], [339.3939, 2.97661e-5], [9.55475, 0], [0.055546, -9.499e-9], [316.9670, 0.0334442282]],
};
const SUN_W = [282.9404, 4.70935e-5], SUN_E = [0.016709, -1.151e-9], SUN_M = [356.0470, 0.9856002585];
export const PLANET_META = {
  Mercury: { sym: "☿", mag: "−0…2" }, Venus: { sym: "♀", mag: "≈−4" },
  Mars: { sym: "♂", mag: "−2…+1.8" }, Jupiter: { sym: "♃", mag: "≈−2" }, Saturn: { sym: "♄", mag: "≈+0.5" },
};

function kepler(Mdeg, e) {
  const M = rev(Mdeg) * D2R;
  let E = M + e * Math.sin(M) * (1 + e * Math.cos(M));
  for (let k = 0; k < 8; k++) E = E - (E - e * Math.sin(E) - M) / (1 - e * Math.cos(E));
  return E;
}

/* heliocentric ecliptic xyz of a body from its elements at day d */
function helio(el, d) {
  const N = (el[0][0] + el[0][1] * d) * D2R, i = (el[1][0] + el[1][1] * d) * D2R;
  const w = (el[2][0] + el[2][1] * d) * D2R, a = el[3][0];
  const e = el[4][0] + el[4][1] * d, M = el[5][0] + el[5][1] * d;
  const E = kepler(M, e);
  const xv = a * (Math.cos(E) - e), yv = a * Math.sqrt(1 - e * e) * Math.sin(E);
  const v = Math.atan2(yv, xv), r = Math.hypot(xv, yv);
  return { r, lonlat: null, N, i, w, v, Mdeg: rev(M) };
}
const toXYZ = (h, dLon = 0) => {
  const u = h.v + h.w;
  let x = h.r * (Math.cos(h.N) * Math.cos(u) - Math.sin(h.N) * Math.sin(u) * Math.cos(h.i));
  let y = h.r * (Math.sin(h.N) * Math.cos(u) + Math.cos(h.N) * Math.sin(u) * Math.cos(h.i));
  const z = h.r * Math.sin(u) * Math.sin(h.i);
  if (dLon) { // apply a longitude perturbation by rotating in the ecliptic plane
    const lon = Math.atan2(y, x) + dLon * D2R, rxy = Math.hypot(x, y);
    x = rxy * Math.cos(lon); y = rxy * Math.sin(lon);
  }
  return [x, y, z];
};

/* geocentric az/el (+ distance AU) for the five naked-eye planets */
export function planetPositions(ms, lat, lng) {
  const d = ms / 86400000 + 2440587.5 - 2451543.5; // unix → days since 2000 Jan 0.0
  /* Sun (= Earth) position */
  const es = SUN_E[0] + SUN_E[1] * d;
  const Ms = SUN_M[0] + SUN_M[1] * d, ws = SUN_W[0] + SUN_W[1] * d;
  const Es = kepler(Ms, es);
  const xvs = Math.cos(Es) - es, yvs = Math.sqrt(1 - es * es) * Math.sin(Es);
  const vs = Math.atan2(yvs, xvs), rs = Math.hypot(xvs, yvs);
  const lonS = (vs * R2D + ws) * D2R;
  const xs = rs * Math.cos(lonS), ys = rs * Math.sin(lonS);
  const Mj = rev(EL.Jupiter[5][0] + EL.Jupiter[5][1] * d);
  const Msat = rev(EL.Saturn[5][0] + EL.Saturn[5][1] * d);
  const obl = (23.4393 - 3.563e-7 * d) * D2R;
  const out = [];
  for (const [name, el] of Object.entries(EL)) {
    const h = helio(el, d);
    let dLon = 0;
    if (name === "Jupiter") {
      dLon = -0.332 * Math.sin((2 * Mj - 5 * Msat - 67.6) * D2R)
        - 0.056 * Math.sin((2 * Mj - 2 * Msat + 21) * D2R)
        + 0.042 * Math.sin((3 * Mj - 5 * Msat + 21) * D2R)
        - 0.036 * Math.sin((Mj - 2 * Msat) * D2R);
    } else if (name === "Saturn") {
      dLon = 0.812 * Math.sin((2 * Mj - 5 * Msat - 67.6) * D2R)
        - 0.229 * Math.cos((2 * Mj - 4 * Msat - 2) * D2R)
        + 0.119 * Math.sin((Mj - 2 * Msat - 3) * D2R);
    }
    const [xh, yh, zh] = toXYZ(h, dLon);
    const xg = xh + xs, yg = yh + ys, zg = zh; // Sun's ecliptic z is 0
    /* ecliptic → equatorial */
    const xe = xg, ye = yg * Math.cos(obl) - zg * Math.sin(obl), ze = yg * Math.sin(obl) + zg * Math.cos(obl);
    const ra = rev(Math.atan2(ye, xe) * R2D), dec = Math.atan2(ze, Math.hypot(xe, ye)) * R2D;
    const ae = raDecToAzEl(ra, dec, ms, lat, lng);
    out.push({ name, sym: PLANET_META[name].sym, ra, dec, az: ae.az, alt: ae.alt, distAU: Math.hypot(xg, yg, zg) });
  }
  return out;
}
