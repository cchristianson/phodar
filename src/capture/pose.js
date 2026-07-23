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
  const gSign = opts.gSign === -1 ? -1 : 1;
  const gx = +gravity.x, gy = +gravity.y, gz = +gravity.z;
  const m = Math.hypot(gx, gy, gz) || 1;
  const ux = (gSign * gx) / m, uy = (gSign * gy) / m, uz = (gSign * gz) / m;  // up in device frame
  const el = Math.asin(clamp(-uz, -1, 1)) * R2D;
  const roll = Math.atan2(ux, uy) * R2D;
  const az = ((((isNum(headingDeg) ? +headingDeg : 0) % 360) + 360) % 360);
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
