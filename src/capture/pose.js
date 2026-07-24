/* ============================================================
   CAMERA POSE FROM PHONE SENSORS  (web DeviceMotion / DeviceOrientation)

   The native-metadata gap, closed on the web: standard photo EXIF omits the
   up/down angle (and roll), so a straight-up sky shot defaults to 15° and the
   whole sky-view calibration exists to recover pointing by hand. When the photo
   is taken INSIDE Phodar (getUserMedia camera), the phone's motion sensors know
   the camera's orientation at the shutter — this turns that into the same
   {az, el, roll} pose the placement uses (`mediaAim`), as a SEED the sky view
   then refines.

   Device frame (W3C / iOS, portrait): X → screen-right, Y → screen-top,
   Z → out of the screen toward the viewer. The BACK camera looks along −Z.
   `gravity` is `DeviceMotionEvent.accelerationIncludingGravity` in that frame
   (iOS reports it ALONG the pull — screen-up on a table reads z ≈ −9.8 — so
   up_device = −ĝ, fixed, no longer user-flippable: an earlier ⇅ that flipped
   the WHOLE gravity sign silently negated the landscape bearing rotation too
   and poisoned two rounds of field calibration. The ⇅ button now owns ONLY
   the elevation sense, so bearing observations can't be corrupted again).

   The AZIMUTH model is empirical, settled by a portrait shot aimed UP 20°
   (the one case that discriminates the hypotheses) — see the comment in
   poseFromGravity. Everything is asserted in mathcheck.
   ============================================================ */

import { R2D } from "../math/geodesy.js";
import { isNum } from "../math/format.js";

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

/* Camera elevation + roll from the gravity vector, azimuth from the compass.
     gravity   : {x,y,z}  device-frame accelerationIncludingGravity
     headingDeg: webkitCompassHeading (° from true north), or null
     opts.orient: screen angle (screen.orientation.angle / window.orientation);
                 0/180 portrait, ±90/270 landscape — selects the azimuth regime.
     opts.elSign: elevation sense, owned by the ⇅ "flip tilt" button. Default
                 +1 (aim up → positive el — field truth +21.9° read +20 raw).
                 Independent of the bearing math by construction.
   Returns { az, el, roll } in degrees (el positive = camera tilted UP).

   Geometry: up_device = −ĝ. The camera axis is a = (0,0,−1).
     elevation = elSign · asin(dot(a, up)) = elSign · asin(−up.z)
       → horizon 0°; straight up +90°; straight down −90°.
     roll = angle of world-up projected into the screen (XY) plane, measured
            from screen-up (+Y): atan2(up.x, up.y) → 0° when the phone is level. */
export function poseFromGravity(gravity, headingDeg, opts = {}) {
  const elSign = opts.elSign === -1 ? -1 : 1;
  const gx = +gravity.x, gy = +gravity.y, gz = +gravity.z;
  const m = Math.hypot(gx, gy, gz) || 1;
  const ux = -gx / m, uy = -gy / m, uz = -gz / m;   // up in device frame (iOS: accel = pull)
  const el = elSign * Math.asin(clamp(-uz, -1, 1)) * R2D;
  let roll = Math.atan2(ux, uy) * R2D;
  /* Fold roll to the nearest quadrant: the phone can be held portrait OR
     landscape, but the photo's OWN EXIF orientation already bakes it upright, so
     only the RESIDUAL tilt beyond 0/90/180/270° is a real roll for the placement
     (a level landscape shot → ~0, not 90). */
  roll = roll - 90 * Math.round(roll / 90);

  /* AZIMUTH — what iOS actually reports, all field-measured on ONE device with
     terrain-aligned ground truth:
       · PORTRAIT hold: `webkitCompassHeading` is already tilt-compensated to
         the CAMERA's heading. Aiming down 11° it read 247 vs truth 244; aiming
         UP 20° it STILL read the camera heading (truth 242) — where a naive
         top-edge-projection model would flip 180° once the phone leans past
         vertical. The up-tilt shot is what discriminates: pass raw through,
         and any "correction" on top is exactly what broke it (the seed came
         out 61 = 241 + a spurious 180).
       · LANDSCAPE hold: the compensation does NOT follow the camera — the
         reading stays referenced to the portrait TOP edge (+Y), ±90° off the
         camera. Rotate from +Y's horizontal projection to the camera axis's,
         about world-up; the rotation self-selects +90 vs −90 for either
         landscape hold. Field: this rotation alone landed the bearing; a
         former extra +180 "iOS frame flip" (removed) made it read exactly
         opposite — that patch was fitted to an old reading taken while the
         since-removed whole-gravity ⇅ flip had negated the rotation's sign.
     The UI orientation (opts.orient) picks the regime — it's what iOS's own
     compensation keys off, and a deliberate photo is taken in one or the
     other, not diagonally. */
  let az = ((((isNum(headingDeg) ? +headingDeg : 0) % 360) + 360) % 360);
  const o = isNum(opts.orient) ? ((+opts.orient % 360) + 360) % 360 : 0;
  const landscape = (o >= 45 && o < 135) || (o >= 225 && o < 315);
  if (isNum(headingDeg) && landscape) {
    const u = [ux, uy, uz];                                   // up unit (device frame)
    const projH = (v) => { const d = v[0] * u[0] + v[1] * u[1] + v[2] * u[2]; return [v[0] - d * u[0], v[1] - d * u[1], v[2] - d * u[2]]; };
    const topH = projH([0, 1, 0]);                            // +Y top edge, horizontal part
    const camH = projH([0, 0, -1]);                           // −Z camera axis, horizontal part
    const tn = Math.hypot(topH[0], topH[1], topH[2]), cn = Math.hypot(camH[0], camH[1], camH[2]);
    if (tn > 0.05 && cn > 1e-3) {                             // in landscape tn ≈ 1; tiny floor is numerical only
      // Signed angle from the top edge to the camera axis about world-up:
      // atan2(crUp, dotTC) is the MATH-positive rotation; compass heading runs
      // CLOCKWISE from north (the negative sense seen from above), hence the
      // leading minus. Both landscape holds asserted in mathcheck.
      const crx = topH[1] * camH[2] - topH[2] * camH[1], cry = topH[2] * camH[0] - topH[0] * camH[2], crz = topH[0] * camH[1] - topH[1] * camH[0];
      const crUp = crx * u[0] + cry * u[1] + crz * u[2];
      const dotTC = topH[0] * camH[0] + topH[1] * camH[1] + topH[2] * camH[2];
      const deltaCW = -Math.atan2(crUp, dotTC) * R2D;
      az = ((az + deltaCW) % 360 + 360) % 360;
    }
  }
  return { az: +az.toFixed(1), el: +el.toFixed(1), roll: +roll.toFixed(1) };
}

/* How trustworthy the seed is — surfaced honestly in the capture UI and stored
   with the shot. Compass accuracy is the weak link (metal), elevation/roll from
   gravity are near-exact when the phone is still. */
export function poseQuality(compassAccDeg, still) {
  const acc = isNum(compassAccDeg) && compassAccDeg >= 0 ? compassAccDeg : null;
  const headingOk = acc != null && acc <= 20;
  return {
    headingAccDeg: acc,
    headingOk,
    tiltOk: still !== false,
    note: acc == null ? "compass accuracy unknown — verify the bearing in the sky view"
      : headingOk ? "sensor pose captured — refine in the sky view if needed"
        : `compass ±${Math.round(acc)}° (near metal?) — the up/down angle is solid, but check the bearing`,
  };
}
