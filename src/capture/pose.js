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
  /* opts.headingIsCamera: the caller already resolved the CAMERA's heading (the
     non-iOS path does, out of the orientation angles), so no regime correction
     applies — that whole dance exists to undo what iOS's own tilt compensation
     does to webkitCompassHeading. */
  const landscape = !opts.headingIsCamera && ((o >= 45 && o < 135) || (o >= 225 && o < 315));
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

/* ============================================================
   NON-iOS PATHS.

   Everything above is calibrated against what an iPhone reports:
   `webkitCompassHeading` (tilt-compensated to the camera in portrait) and an
   `accelerationIncludingGravity` that points ALONG the pull. Neither is
   universal — the two functions below cover every other browser without
   touching the iOS path, which stays exactly as field-calibrated.
   ============================================================ */

/* Device UP vector from the W3C orientation angles (β pitch, γ roll).

   R = Rz(α)Rx(β)Ry(γ) takes device coordinates into the Earth frame, so world
   up expressed in the device frame is R's third row. Unlike the accelerometer,
   these angles mean the same thing on every platform — which is what makes
   them usable as the reference below. */
export function upFromOrientation(beta, gamma) {
  if (!isNum(beta) || !isNum(gamma)) return null;
  const b = beta / R2D, g = gamma / R2D;
  const cb = Math.cos(b), sb = Math.sin(b), cg = Math.cos(g), sg = Math.sin(g);
  return { x: -cb * sg, y: sb, z: cb * cg };
}

/* WHICH SIGN CONVENTION this device's accelerometer uses.

   iOS reports `accelerationIncludingGravity` along the pull — flat on a table,
   screen up, z ≈ −9.8. The W3C/Chrome convention reports proper acceleration,
   which is the exact opposite (z ≈ +9.8), so the same phone-in-hand yields an
   inverted up vector and `poseFromGravity` would return a negated elevation
   and a roll rotated 180°.

   Rather than sniff the user agent — which is a guess about a physical
   convention, and wrong the moment a browser changes — compare the reading
   against the orientation angles, which are unambiguous everywhere.

   Returns +1 when the reading matches the iOS convention this module is
   calibrated against, −1 when it is inverted, and 0 when it can't tell (the
   phone is being swung, so the accelerometer isn't measuring gravity alone).
   A caller should keep the last non-zero answer. */
export function gravitySign(gravity, beta, gamma) {
  const u = upFromOrientation(beta, gamma);
  if (!u || !gravity) return 0;
  const gx = +gravity.x, gy = +gravity.y, gz = +gravity.z;
  const m = Math.hypot(gx, gy, gz);
  if (!(m > 5 && m < 15)) return 0;                    // not a gravity-dominated sample
  const d = -(gx * u.x + gy * u.y + gz * u.z) / m;     // iOS convention: up = −ĝ
  return Math.abs(d) < 0.6 ? 0 : (d > 0 ? 1 : -1);
}

/* FULL POSE from the W3C orientation angles alone — the path for a device that
   reports an ABSOLUTE alpha (compass-referenced, via `deviceorientationabsolute`
   or `absolute: true`) but no `webkitCompassHeading`. That is Android, and
   alpha there is NOT the tilt-compensated camera heading iOS gives: it is the
   rotation about the device's own Z axis, so feeding it to `poseFromGravity`
   as a heading is wrong by however far the phone is tilted.

   Doing it properly is just the rotation matrix. The back camera looks along
   −Z in the device frame, so its Earth-frame direction is minus the third
   column of R = Rz(α)Rx(β)Ry(γ) (Earth frame: X east, Y north, Z up). Bearing
   is that direction's horizontal angle clockwise from north, elevation its
   vertical component. Roll comes from the same up vector the iOS path uses, so
   both paths fold it the same way.

   No accelerometer is involved, so the sign question above doesn't arise here.
   Asserted in mathcheck against hand-computed holds. */
export function poseFromOrientation(alpha, beta, gamma, opts = {}) {
  if (!isNum(alpha) || !isNum(beta) || !isNum(gamma)) return null;
  const elSign = opts.elSign === -1 ? -1 : 1;
  const a = alpha / R2D, b = beta / R2D, g = gamma / R2D;
  const ca = Math.cos(a), sa = Math.sin(a);
  const cb = Math.cos(b), sb = Math.sin(b), cg = Math.cos(g), sg = Math.sin(g);
  /* third column of R, then the camera axis = −that */
  const ex = -(ca * sg + sa * sb * cg);
  const ny = -(sa * sg - ca * sb * cg);
  const uz = -(cb * cg);
  const az = ((Math.atan2(ex, ny) * R2D) % 360 + 360) % 360;     // CW from north
  const el = elSign * Math.asin(clamp(uz, -1, 1)) * R2D;
  const u = upFromOrientation(beta, gamma);
  let roll = Math.atan2(u.x, u.y) * R2D;
  roll = roll - 90 * Math.round(roll / 90);                      // same quadrant fold as poseFromGravity
  return { az: +az.toFixed(1), el: +el.toFixed(1), roll: +roll.toFixed(1) };
}

/* How trustworthy the seed is — surfaced honestly in the capture UI and stored
   with the shot. Compass accuracy is the weak link (metal), elevation/roll from
   gravity are near-exact when the phone is still. */
export function poseQuality(compassAccDeg, still, mode) {
  const acc = isNum(compassAccDeg) && compassAccDeg >= 0 ? compassAccDeg : null;
  const headingOk = acc != null && acc <= 20;
  /* The orientation path has no accuracy figure to report, and its bearing may
     be referenced to MAGNETIC north — the platform decides, and there is no way
     to ask which it did, so say so rather than quietly assuming true north
     (declination runs to 20° in places). The sky view's terrain/star
     calibration is the fix, and the pose was only ever a seed. */
  const note = mode === "orient"
    ? "bearing from the device's orientation sensor — it may be magnetic, so check it against the terrain or stars in the sky view"
    : mode === null || mode === "none"
      ? "no compass reference — tilt and roll are solid, but set the bearing yourself in the sky view"
      : acc == null ? "compass accuracy unknown — verify the bearing in the sky view"
        : headingOk ? "sensor pose captured — refine in the sky view if needed"
          : `compass ±${Math.round(acc)}° (near metal?) — the up/down angle is solid, but check the bearing`;
  return { headingAccDeg: acc, headingOk: headingOk && mode !== "orient", tiltOk: still !== false, note };
}
