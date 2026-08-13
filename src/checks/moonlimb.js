/* ============================================================
   MOON TERMINATOR FORENSIC — when the moon is IN the photo, its lit
   limb must point at the sun. That is pure celestial mechanics, and it
   is the classic composite-killer: a pasted-in moon routinely has its
   phase rotated wrong for the claimed time, place and framing, and no
   metadata stripping can repair it.

   Pieces (pure, mathcheck-asserted):
   - `limbTargetPoint` — a sky point a small step from the moon toward
     the sun along the great circle. The caller projects BOTH the moon
     and this point through the photo's own pose (dirToPixK), so the
     predicted limb direction in IMAGE coordinates inherits roll and
     lens distortion from the real projection instead of a hand-derived
     convention (the classic place to get a sign wrong).
   - `measureLimbAngle` — brightness-weighted centroid of the moon disc
     relative to its center: the lit side pulls the centroid toward the
     bright limb. `strength` (offset / radius) is the confidence gate —
     a full or new moon has no measurable asymmetry and must yield NO
     verdict, not a random one.
   - `limbVerdict` — compare measured vs predicted image angles with an
     inconclusive band between "agrees" and "mismatch": a forensic that
     guesses in the gray zone is worse than one that stays silent.
   ============================================================ */

import { isNum } from "../math/format.js";

const D2R = Math.PI / 180, R2D = 180 / Math.PI;

/* great-circle initial bearing from the moon toward the sun in alt/az space
   (bearing 0 = toward zenith, 90 = toward increasing azimuth), plus a sky
   point stepped that way — project it beside the moon to get the predicted
   limb direction in image pixels. */
export function limbTargetPoint(moonAz, moonAlt, sunAz, sunAlt, stepDeg = 0.35) {
  const dAz = (((sunAz - moonAz + 540) % 360) - 180) * D2R;
  const a1 = moonAlt * D2R, a2 = sunAlt * D2R;
  const B = Math.atan2(Math.sin(dAz) * Math.cos(a2), Math.cos(a1) * Math.sin(a2) - Math.sin(a1) * Math.cos(a2) * Math.cos(dAz));
  return {
    az: moonAz + (stepDeg * Math.sin(B)) / Math.max(0.05, Math.cos(a1)),
    alt: moonAlt + stepDeg * Math.cos(B),
    paDeg: ((B * R2D) + 360) % 360,
  };
}

/* gray: luminance array (W×H, row-major). Returns the direction the disc's
   brightness centroid is displaced from its center — image convention
   (x right, y down, angle = atan2(dy, dx)) — and how strongly. */
export function measureLimbAngle(gray, W, H, cx, cy, rPx) {
  const r2 = rPx * rPx;
  let sw = 0, sx = 0, sy = 0, n = 0;
  for (let y = Math.max(0, Math.floor(cy - rPx)); y <= Math.min(H - 1, Math.ceil(cy + rPx)); y++) {
    for (let x = Math.max(0, Math.floor(cx - rPx)); x <= Math.min(W - 1, Math.ceil(cx + rPx)); x++) {
      const dx = x - cx, dy = y - cy;
      if (dx * dx + dy * dy > r2) continue;
      const v = gray[y * W + x];
      sw += v; sx += v * dx; sy += v * dy; n++;
    }
  }
  if (n < 16 || sw <= 0) return null;
  const mx = sx / sw, my = sy / sw;
  return { angle: ((Math.atan2(my, mx) * R2D) + 360) % 360, strength: Math.hypot(mx, my) / rPx };
}

/* Locate the bright disc inside a crop: sky level from the median, lit
   pixels above halfway to the max, bounding-box center. For any phase past
   a thin crescent the outer limb spans most of the true disc, so the bbox
   center lands near the real disc center even though the lit mass is all on
   one side; spanPx lets the caller reject blobs that are not moon-sized.
   Null when nothing bright stands off the sky — never a guess. */
export function discCenter(gray, W, H) {
  const sample = [];
  for (let i = 0; i < gray.length; i += 7) sample.push(gray[i]);
  sample.sort((a, b) => a - b);
  const sky = sample[Math.floor(sample.length * 0.5)];
  let max = -1;
  for (let i = 0; i < gray.length; i++) if (gray[i] > max) max = gray[i];
  if (!(max > sky + 12)) return null;
  const th = sky + 0.5 * (max - sky);
  let x0 = 1e9, x1 = -1, y0 = 1e9, y1 = -1, n = 0;
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    if (gray[y * W + x] >= th) { n++; if (x < x0) x0 = x; if (x > x1) x1 = x; if (y < y0) y0 = y; if (y > y1) y1 = y; }
  }
  if (n < 12) return null;
  return { cx: (x0 + x1) / 2, cy: (y0 + y1) / 2, spanPx: Math.max(x1 - x0, y1 - y0), n };
}

/* The center-error-free limb measurement, and the one the report uses.
   Every lunar phase shape is mirror-symmetric about the limb axis, so the
   limb axis is a PRINCIPAL axis of the lit region — and always the MINOR one
   (a phase is widest across the terminator chord). That gives the axis with
   no disc center needed; which END is the limb comes from the width taper:
   the terminator end cuts off wide, the limb end tapers like a circle's rim,
   so the limb is the low-mass end. A full moon is symmetric both ways —
   the end-mass asymmetry collapses (strength → 0 → no verdict), which is
   the honest failure mode. */
export function measureLimbDir(gray, W, H) {
  const dc = discCenter(gray, W, H);
  if (!dc) return null;
  const sample = [];
  for (let i = 0; i < gray.length; i += 7) sample.push(gray[i]);
  sample.sort((a, b) => a - b);
  const sky = sample[Math.floor(sample.length * 0.5)];
  let max = -1;
  for (let i = 0; i < gray.length; i++) if (gray[i] > max) max = gray[i];
  const th = sky + 0.5 * (max - sky);
  /* weighted second moments of the lit region */
  let sw = 0, sx = 0, sy = 0;
  const px = [], py = [], pw = [];
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    const raw = gray[y * W + x];
    if (raw < th) continue;
    const v = raw - sky;
    if (v <= 0) continue;
    sw += v; sx += v * x; sy += v * y;
    px.push(x); py.push(y); pw.push(v);
  }
  if (sw <= 0 || px.length < 16) return null;
  const cx = sx / sw, cy = sy / sw;
  let Sxx = 0, Syy = 0, Sxy = 0;
  for (let i = 0; i < px.length; i++) {
    const dx = px[i] - cx, dy = py[i] - cy, w = pw[i];
    Sxx += w * dx * dx; Syy += w * dy * dy; Sxy += w * dx * dy;
  }
  /* minor principal axis (smaller eigenvalue) of the covariance */
  const tr = Sxx + Syy, det = Sxx * Syy - Sxy * Sxy;
  const lMin = tr / 2 - Math.sqrt(Math.max(0, (tr / 2) * (tr / 2) - det));
  let ux, uy;
  if (Math.abs(Sxy) > 1e-9) { ux = lMin - Syy; uy = Sxy; }
  else if (Sxx <= Syy) { ux = 1; uy = 0; }
  else { ux = 0; uy = 1; }
  const um = Math.hypot(ux, uy) || 1; ux /= um; uy /= um;
  /* which end is the limb: project onto the axis, compare the mass in the
     outer 25% band at each end — the terminator end is the heavy one */
  let sMin = Infinity, sMax = -Infinity;
  const ss = new Float64Array(px.length);
  for (let i = 0; i < px.length; i++) {
    const s = (px[i] - cx) * ux + (py[i] - cy) * uy;
    ss[i] = s;
    if (s < sMin) sMin = s;
    if (s > sMax) sMax = s;
  }
  const span = sMax - sMin;
  if (!(span > 4)) return null;
  let m0 = 0, m1 = 0;
  for (let i = 0; i < px.length; i++) {
    if (ss[i] < sMin + 0.25 * span) m0 += pw[i];
    else if (ss[i] > sMax - 0.25 * span) m1 += pw[i];
  }
  const asym = (m0 + m1) > 0 ? Math.abs(m0 - m1) / (m0 + m1) : 0;
  const sgn = m0 > m1 ? 1 : -1; // limb = the light end
  return {
    angle: ((Math.atan2(sgn * uy, sgn * ux) * R2D) + 360) % 360,
    strength: asym,
    spanPx: dc.spanPx,
  };
}

/* measured vs predicted IMAGE angles (deg). frac = moon illuminated
   fraction. Verdicts: "match" (positive evidence), "mismatch" (warn — the
   pasted-moon tell), or null when the geometry can't testify (near-full or
   near-new moon, weak asymmetry, or the in-between gray zone). */
export function limbVerdict(measuredDeg, predictedDeg, strength, frac) {
  if (!isNum(measuredDeg) || !isNum(predictedDeg) || !isNum(strength)) return null;
  if (isNum(frac) && (frac < 0.10 || frac > 0.92)) return null; // no measurable terminator
  if (strength < 0.12) return null;                              // asymmetry too weak to trust
  const d = Math.abs(((measuredDeg - predictedDeg + 540) % 360) - 180);
  if (d <= 30) return { verdict: "match", offDeg: d };
  if (d >= 55) return { verdict: "mismatch", offDeg: d };
  return null; // gray zone — stay silent rather than guess
}
