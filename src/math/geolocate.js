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

import { D2R, R2D, RE, geoFromEnu, enuFromGeo } from "./geodesy.js";
import { isNum } from "./format.js";
import { pixToDirK, solveN } from "./projection.js";

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

/* ─── GEOREFERENCE OFF KNOWN GROUND POINTS (no telemetry) ──────────────────
   The redacted-footage path: you don't know the platform's position/altitude,
   but you can identify GROUND CONTROL POINTS — image features whose real
   lat/lon you can read off a map (a road junction, a building corner). ≥4 of
   them on roughly-flat ground define a planar HOMOGRAPHY from image pixels to
   the ground, and then EVERY pixel geolocates — no platform pose, FOV, or gimbal
   angle needed. (For non-flat terrain or a moving platform, the ray-cast path
   above is better; the two are complementary.)

   `groundHomography` fits the pixel→ground-ENU homography by least squares (so
   >4 GCPs improve the fit), anchored at the GCP centroid. Returns { H, ref, rms }
   where rms is the ground reprojection error in metres (fit quality). */
function fitHomographyLS(src, dst) {
  const N = src.length;
  if (N < 4) return null;
  const rows = [];      // 2N × 8 design, target b
  const bb = [];
  for (let i = 0; i < N; i++) {
    const [x, y] = src[i], [X, Y] = dst[i];
    rows.push([x, y, 1, 0, 0, 0, -X * x, -X * y]); bb.push(X);
    rows.push([0, 0, 0, x, y, 1, -Y * x, -Y * y]); bb.push(Y);
  }
  // normal equations: (AᵀA) h = Aᵀb  (8×8) — least squares over all correspondences
  const M = Array.from({ length: 8 }, () => new Array(8).fill(0));
  const rhs = new Array(8).fill(0);
  for (let r = 0; r < rows.length; r++) {
    for (let i = 0; i < 8; i++) { rhs[i] += rows[r][i] * bb[r]; for (let j = 0; j < 8; j++) M[i][j] += rows[r][i] * rows[r][j]; }
  }
  const h = solveN(M, rhs);
  return h ? [[h[0], h[1], h[2]], [h[3], h[4], h[5]], [h[6], h[7], 1]] : null;
}
function applyH(H, px, py) {
  const w = H[2][0] * px + H[2][1] * py + H[2][2];
  if (Math.abs(w) < 1e-12) return null;
  return [(H[0][0] * px + H[0][1] * py + H[0][2]) / w, (H[1][0] * px + H[1][1] * py + H[1][2]) / w];
}
export function groundHomography(gcps) {
  const pts = (gcps || []).filter((g) => isNum(g.px) && isNum(g.py) && isNum(g.lat) && isNum(g.lon));
  if (pts.length < 4) return null;
  const ref = { lat: pts.reduce((a, g) => a + g.lat, 0) / pts.length, lon: pts.reduce((a, g) => a + g.lon, 0) / pts.length, alt: 0 };
  const src = pts.map((g) => [g.px, g.py]);
  const dst = pts.map((g) => { const e = enuFromGeo(g.lat, g.lon, 0, ref); return [e[0], e[1]]; });
  const H = fitHomographyLS(src, dst);
  if (!H) return null;
  let se = 0;
  for (let i = 0; i < pts.length; i++) { const p = applyH(H, src[i][0], src[i][1]); if (!p) continue; se += (p[0] - dst[i][0]) ** 2 + (p[1] - dst[i][1]) ** 2; }
  return { H, ref, rms: Math.sqrt(se / pts.length) };
}
/* pixel → ground {lat,lon} through a fitted GCP homography (geo = {H, ref}) */
export function pixelToGroundH(px, py, geo) {
  if (!geo || !geo.H) return null;
  const e = applyH(geo.H, px, py);
  if (!e) return null;
  const g = geoFromEnu([e[0], e[1], 0], geo.ref);
  return { lat: g.lat, lon: g.lon, e: e[0], n: e[1] };
}
/* true ground size (m) between two pixels via the GCP homography */
export function groundSpanH(p1, p2, geo) {
  const a = pixelToGroundH(p1.x, p1.y, geo), b = pixelToGroundH(p2.x, p2.y, geo);
  return a && b ? haversineM(a, b) : null;
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
