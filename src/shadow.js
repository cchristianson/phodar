/* ⚑ Sun-shadow gadget — a schematic flagpole planted on level ground in front
   of the camera, plus where the sun at the sighting time would throw its
   shadow. Pure geometry, no DOM: the sky view projects the returned az/el
   polylines through its own dome projection. It exists as a physics
   cross-check — shadows visible in the photo must agree with the computed sun
   for the stated time and place, and stripping metadata can't fake that.
   Honesty: shadow DIRECTION is exact; LENGTH assumes flat level ground (no
   DEM), and the UI says so. The pole is fictitious, so this stays a dome-only
   overlay (like the winds stack) — never burned into world-locked exports. */
import { D2R, R2D } from "./math/geodesy.js";

export const POLE_H = 5; // m — flagpole height (a real flagpole, big enough to read at 25 m)

/* Keep the pole's base in view from rooftops: at eye height the base sits a
   few degrees below the horizon; from 100 m up, 25 m out would be −76°. */
export function poleDist(camH) { return Math.max(25, (+camH > 0 ? +camH : 1.6) * 3); }

/* az: where the pole stands (compass deg — the sky view passes its view-center
   azimuth so the pole is always centered). camH: eye height above the ground
   plane (m). sunAz/sunAlt: refracted apparent sun (compass deg / deg).
   Returns { pole, base, top, shadow, len, dir, clipped }:
   pole/shadow are [{az, el}] polylines (shadow null when the sun is down),
   len the true shadow length (m), dir its compass bearing (away from the
   sun), clipped true when a grazing sun's shadow was capped for drawing. */
export function poleShadow({ az, camH, sunAz, sunAlt, D, H }) {
  H = +H > 0 ? +H : POLE_H;
  D = +D > 0 ? +D : poleDist(camH);
  const h = +camH > 0 ? +camH : 1.6;
  const e0 = D * Math.sin(az * D2R), n0 = D * Math.cos(az * D2R);
  const pt = (e, n, z) => ({ az: (Math.atan2(e, n) * R2D + 360) % 360, el: Math.atan2(z, Math.hypot(e, n)) * R2D });
  const pole = [];
  for (let i = 0; i <= 8; i++) pole.push(pt(e0, n0, -h + (i / 8) * H));
  const base = pole[0], top = pole[8];
  if (!(sunAlt > 0.05)) return { pole, base, top, shadow: null, len: null, dir: null, clipped: false };
  const len = H / Math.tan(sunAlt * D2R);
  const dir = (sunAz + 180) % 360; // the shadow falls AWAY from the sun
  const de = Math.sin(dir * D2R), dn = Math.cos(dir * D2R);
  const drawn = Math.min(len, 2500); // grazing sun: cap the drawn line, report the true length
  const shadow = [];
  for (let i = 0; i <= 16; i++) {
    const u = Math.pow(i / 16, 1.5) * drawn; // denser near the base, where the projected curve bends
    shadow.push(pt(e0 + u * de, n0 + u * dn, -h));
  }
  return { pole, base, top, shadow, len, dir, clipped: drawn < len };
}
