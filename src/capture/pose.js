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

   ONE more empirical fact, field-measured: `webkitCompassHeading` is reported
   in a frame that FLIPS 180° when the interface rotates to landscape (portrait
   reads dead-on; the same scene held landscape reads exactly 180° off — a clean
   flip, not a ±90° edge confusion, so it's symmetric across both landscape
   holds). `opts.orient` (the screen angle: 0 portrait, ±90 landscape) applies
   the +180° in landscape. Asserted in mathcheck.
   ============================================================ */

import { R2D } from "../math/geodesy.js";
import { isNum } from "../math/format.js";

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

/* Camera elevation + roll from the gravity vector, azimuth from the compass.
     gravity   : {x,y,z}  device-frame accelerationIncludingGravity
     headingDeg: webkitCompassHeading (° from true north), or null
     opts.gSign: sign of the gravity reading for the UP VECTOR (az + roll). −1
                 (iOS points along the pull) is field-correct and fixed.
     opts.elSign: elevation sense (the ⇅ flip). −1 by default — field-measured
                 iOS reads aim-up as inverted with +1. Kept SEPARATE from gSign
                 so flipping tilt never disturbs the bearing.
   Returns { az, el, roll } in degrees (el positive = camera tilted UP).

   Geometry: up_device = gSign·ĝ. The camera axis is a = (0,0,−1).
     elevation = elSign · asin(dot(a, up)) = elSign · asin(−up.z).
     roll = angle of world-up projected into the screen (XY) plane, measured
            from screen-up (+Y): atan2(up.x, up.y) → 0° when the phone is level. */
export function poseFromGravity(gravity, headingDeg, opts = {}) {
  /* `gSign` sets the gravity sign for the UP VECTOR used by azimuth + roll; −1
     (iOS accelerationIncludingGravity points along the pull) is field-correct
     for the bearing, so it's FIXED here — the ⇅ flip no longer touches it, so
     it can't disturb the (working) landscape bearing. */
  const gSign = opts.gSign === 1 ? 1 : -1;
  /* `elSign` is the ELEVATION sense, DECOUPLED from gSign and owned by the ⇅
     "flip tilt" button. DEFAULT −1: field-measured — with +1 the horizon rose
     when the camera aimed UP (inverted). Independent so flipping tilt can never
     move the bearing. */
  const elSign = opts.elSign === 1 ? 1 : -1;
  const gx = +gravity.x, gy = +gravity.y, gz = +gravity.z;
  const m = Math.hypot(gx, gy, gz) || 1;
  const ux = (gSign * gx) / m, uy = (gSign * gy) / m, uz = (gSign * gz) / m;  // up in device frame
  const el = elSign * Math.asin(clamp(-uz, -1, 1)) * R2D;
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
      // Rotate the compass heading from the top edge (+Y) to the camera axis
      // (−Z) about world-up. `atan2(crUp, dotTC)` is the MATH-positive angle
      // from top→cam about +u; compass heading runs CLOCKWISE from north (the
      // negative sense viewed from above), so heading(cam) = heading(top) −
      // that angle. Hence the leading minus — verified against both landscape
      // orientations in mathcheck (camera-north maps to az 0 either way).
      const crx = topH[1] * camH[2] - topH[2] * camH[1], cry = topH[2] * camH[0] - topH[0] * camH[2], crz = topH[0] * camH[1] - topH[1] * camH[0];
      const crUp = crx * u[0] + cry * u[1] + crz * u[2];
      const dotTC = topH[0] * camH[0] + topH[1] * camH[1] + topH[2] * camH[2];
      const deltaCW = -Math.atan2(crUp, dotTC) * R2D;
      az = ((az + deltaCW) % 360 + 360) % 360;
    }
  }
  /* LANDSCAPE 180° flip (field-measured, see header). iOS reports the compass
     heading in a frame that reverses when the UI is landscape; the geometry
     above is otherwise exact, so the whole output is a clean 180° off. `orient`
     is the screen angle from the capture UI (window.orientation /
     screen.orientation.angle): 45–135° or 225–315° ⇒ landscape. */
  if (isNum(headingDeg) && isNum(opts.orient)) {
    const o = ((+opts.orient % 360) + 360) % 360;
    const landscape = (o >= 45 && o < 135) || (o >= 225 && o < 315);
    if (landscape) az = (az + 180) % 360;
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
