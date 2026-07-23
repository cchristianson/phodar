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
   `gravity` is `DeviceMotionEvent.accelerationIncludingGravity` in that frame.

   The one empirical unknown is the SIGN of iOS's gravity reading (some builds
   report the reaction "up", some the pull "down"). `gSign` selects it; the
   capture UI exposes a one-tap "flip" that persists the choice per device, so
   it's self-correcting on-device without a code change. Everything else is
   unambiguous geometry, asserted in mathcheck.
   ============================================================ */

import { R2D } from "../math/geodesy.js";
import { isNum } from "../math/format.js";

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

/* Camera elevation + roll from the gravity vector, azimuth from the compass.
     gravity   : {x,y,z}  device-frame accelerationIncludingGravity
     headingDeg: webkitCompassHeading (° from true north), or null
     opts.gSign: +1 if the reading points "up" (reaction), −1 if it points
                 "down" (the pull). Default +1.
   Returns { az, el, roll } in degrees (el positive = camera tilted UP).

   Geometry: up_device = gSign·ĝ. The camera axis is a = (0,0,−1).
     elevation = asin(dot(a, up)) = asin(−up.z)
       → horizon: up≈(0,1,0)  → 0°;  straight up: up≈(0,0,−1) → +90°;
         straight down: up≈(0,0,1) → −90°.
     roll = angle of world-up projected into the screen (XY) plane, measured
            from screen-up (+Y): atan2(up.x, up.y) → 0° when the phone is level. */
export function poseFromGravity(gravity, headingDeg, opts = {}) {
  /* DEFAULT −1: iOS `accelerationIncludingGravity` points ALONG gravity (down)
     — screen-up on a table reads z ≈ −9.8 — so up_device = −ĝ. gSill=+1 is the
     opposite convention (some non-iOS builds); the UI's ⇅ flip switches it. */
  const gSign = opts.gSign === 1 ? 1 : -1;
  const gx = +gravity.x, gy = +gravity.y, gz = +gravity.z;
  const m = Math.hypot(gx, gy, gz) || 1;
  const ux = (gSign * gx) / m, uy = (gSign * gy) / m, uz = (gSign * gz) / m;  // up in device frame
  const el = Math.asin(clamp(-uz, -1, 1)) * R2D;
  let roll = Math.atan2(ux, uy) * R2D;
  /* Fold roll to the nearest quadrant: the phone can be held portrait OR
     landscape, but the photo's OWN EXIF orientation already bakes it upright, so
     only the RESIDUAL tilt beyond 0/90/180/270° is a real roll for the placement
     (a level landscape shot → ~0, not 90). */
  roll = roll - 90 * Math.round(roll / 90);

  /* AZIMUTH — orientation-independent. `webkitCompassHeading` gives the compass
     heading of the phone's TOP edge (+Y), but the camera looks along −Z. Held
     PORTRAIT the top roughly matches the aim; held LANDSCAPE the top points 90°
     sideways, so using the heading directly is ~90° wrong (field-confirmed:
     landscape read 148° when the true aim was 244°). Fix: rotate from the +Y
     heading to the −Z camera axis IN THE HORIZONTAL PLANE (found via gravity).
     Degenerates only when +Y is near-vertical (portrait aimed at the horizon) —
     there the heading is the best available, so fall back to it. */
  let az = ((((isNum(headingDeg) ? +headingDeg : 0) % 360) + 360) % 360);
  if (isNum(headingDeg)) {
    const u = [ux, uy, uz];                                   // up unit (device frame)
    const projH = (v) => { const d = v[0] * u[0] + v[1] * u[1] + v[2] * u[2]; return [v[0] - d * u[0], v[1] - d * u[1], v[2] - d * u[2]]; };
    const topH = projH([0, 1, 0]);                            // +Y top edge, horizontal part
    const camH = projH([0, 0, -1]);                           // −Z camera axis, horizontal part
    const tn = Math.hypot(topH[0], topH[1], topH[2]), cn = Math.hypot(camH[0], camH[1], camH[2]);
    if (tn > 0.3 && cn > 1e-3) {
      // signed angle top→cam, clockwise viewed from above (+u): compass grows CW
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
