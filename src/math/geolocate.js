/* ============================================================
   AERIAL GEOLOCATION  (looking DOWN — the dual of sky triangulation)

   A downward-looking sensor on a platform of KNOWN position and altitude
   (plane / drone / mast). Because the ground is a known surface, a single
   pixel's sight-line intersected with it geolocates the point it sees — no
   second observer needed (that's what triangulation buys you when the target's
   range is unknown, i.e. looking UP at the sky).

   Reuses the same camera model as the sky side: `pixToDirK` turns a pixel into
   a world unit direction in ENU given the sensor pose {az, el, roll, fov, k}.
   For a downward sensor `el` is NEGATIVE (a depression / look-down angle), so
   the direction's Up component is < 0 and the ray hits the ground ahead of /
   below the platform.

   POSE NOTE (matches the video roadmap): pose is a per-frame SAMPLE, not a
   per-clip constant — for FMV the platform moves and the gimbal slews every
   frame. Everything here takes an explicit pose + platform, so nothing assumes
   the sensor was still.

   Ground model here is a FLAT plane at a given MSL elevation (local-tangent, the
   same small-area approximation `enu`/`geo` already use). A DEM ray-march
   refinement (reusing terrain.js sampling) can replace the plane later; the API
   is written so that swap is internal.
   ============================================================ */

import { D2R, R2D, RE, geoFromEnu } from "./geodesy.js";
import { isNum } from "./format.js";
import { pixToDirK } from "./projection.js";

/* great-circle ground distance (m) between two {lat,lon} points */
export function haversineM(a, b) {
  const dLat = (b.lat - a.lat) * D2R, dLon = (b.lon - a.lon) * D2R;
  const la1 = a.lat * D2R, la2 = b.lat * D2R;
  const h = Math.sin(dLat / 2) ** 2 + Math.cos(la1) * Math.cos(la2) * Math.sin(dLon / 2) ** 2;
  return 2 * RE * Math.asin(Math.min(1, Math.sqrt(h)));
}

/* initial bearing (° from true North, clockwise) from a → b */
export function bearingDeg(a, b) {
  const la1 = a.lat * D2R, la2 = b.lat * D2R, dLon = (b.lon - a.lon) * D2R;
  const y = Math.sin(dLon) * Math.cos(la2);
  const x = Math.cos(la1) * Math.sin(la2) - Math.sin(la1) * Math.cos(la2) * Math.cos(dLon);
  return ((Math.atan2(y, x) * R2D) + 360) % 360;
}

/* Intersect a sight-line (ENU unit dir) from a platform {lat,lon,alt (MSL)} with
   a flat ground plane at `groundAlt` (MSL). Returns the ground point + slant
   range, or null if the platform isn't above the ground or the ray doesn't point
   below the horizontal (can't hit the ground ahead). */
export function rayToGround(platform, dir, groundAlt) {
  const H = (+platform.alt) - (+groundAlt);                 // height above the ground plane
  if (!(H > 0) || !Array.isArray(dir) || dir.length !== 3 || !(dir[2] < -1e-6)) return null;
  const t = -H / dir[2];                                    // slant range along the unit ray
  const g = geoFromEnu([t * dir[0], t * dir[1], 0], { lat: +platform.lat, lon: +platform.lon, alt: +groundAlt });
  return { lat: g.lat, lon: g.lon, slant: t, e: t * dir[0], n: t * dir[1], gAlt: +groundAlt };
}

/* pixel → ground point through the sensor pose.
   cam: {natW, natH, az, el, roll, fov, k}  (el negative = looking down) */
export function pixelToGround(px, py, cam, platform, groundAlt) {
  if (!cam || !(cam.natW > 0) || !isNum(cam.az) || !isNum(cam.el) || !isNum(cam.fov)) return null;
  const dir = pixToDirK(px, py, cam.natW, cam.natH, +cam.az, +cam.el, +cam.roll || 0, +cam.fov, +cam.k || 0);
  return rayToGround(platform, dir, groundAlt);
}

/* TRUE ground distance (m) between two pixels' ground points — an object's real
   size, or the width of anything you can bracket with two marks. */
export function groundSpanM(p1, p2, cam, platform, groundAlt) {
  const a = pixelToGround(p1.x, p1.y, cam, platform, groundAlt);
  const b = pixelToGround(p2.x, p2.y, cam, platform, groundAlt);
  return a && b ? haversineM(a, b) : null;
}

/* ground sample distance (m/px) at the image CENTRE for a pose — a quick sense
   of resolution: one pixel's angular size × slant, divided by the obliquity.
   Uses the along-track incidence so an oblique frame reports its stretched GSD. */
export function centerGSD(cam, platform, groundAlt) {
  const g = pixelToGround(cam.natW / 2, cam.natH / 2, cam, platform, groundAlt);
  if (!g) return null;
  const anglePerPx = (2 * Math.tan((+cam.fov * D2R) / 2)) / cam.natW;  // radians/px in tan-space ≈ small-angle
  const incidence = Math.max(5, 90 + (+cam.el)) * D2R;   // 90+el: 90°=nadir look straight down, →0 at horizon
  return g.slant * anglePerPx / Math.sin(incidence);
}

/* Ground kinematics from a geolocated track [{t (s), lat, lon}] (already
   projected to the ground each frame). Distance-true speed + heading per leg and
   overall — the aerial analog of the sky-side trajectory kinematics. */
export function groundKinematics(track) {
  const pts = (track || []).filter((p) => isNum(p.t) && isNum(p.lat) && isNum(p.lon)).sort((a, b) => a.t - b.t);
  if (pts.length < 2) return null;
  const legs = [];
  for (let i = 1; i < pts.length; i++) {
    const dt = pts[i].t - pts[i - 1].t;
    if (!(dt > 0)) continue;
    const d = haversineM(pts[i - 1], pts[i]);
    legs.push({ t: (pts[i].t + pts[i - 1].t) / 2, distM: d, speedMS: d / dt, headingDeg: bearingDeg(pts[i - 1], pts[i]) });
  }
  if (!legs.length) return null;
  const totalDist = legs.reduce((a, l) => a + l.distM, 0);
  const totalT = pts[pts.length - 1].t - pts[0].t;
  const speeds = legs.map((l) => l.speedMS);
  return {
    legs,
    distM: totalDist,
    durationS: totalT,
    avgSpeedMS: totalT > 0 ? totalDist / totalT : null,
    peakSpeedMS: Math.max(...speeds),
    headingDeg: bearingDeg(pts[0], pts[pts.length - 1]),
  };
}
