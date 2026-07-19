/* ============================================================
   OBJECT PHOTOMETRY — colour, brightness/saturation, and (when a catalogued
   star shares the frame) a ROUGH apparent magnitude, measured from the sighting
   photo's own pixels. Phone HDR/tone-mapping is nonlinear, so the magnitude is
   order-of-magnitude only and is labelled that way; colour and saturation are
   robust. Pure fns take RGBA pixel arrays so they're unit-testable.
   ============================================================ */

const lum = (r, g, b) => 0.2126 * r + 0.7152 * g + 0.0722 * b;

/* Aperture photometry: net (background-subtracted) flux + mean colour over a
   disk of radius r at (cx,cy); background is the mean of the r..2r annulus.
   Net flux is a background-subtracted SUM, so a big object aperture and a small
   star aperture are directly comparable (total flux). Returns null off-image. */
export function aperture(data, w, h, cx, cy, r) {
  r = Math.max(1.5, r);
  const r2 = r * r, ro2 = (2 * r) * (2 * r);
  let aSum = 0, aN = 0, aR = 0, aG = 0, aB = 0, peak = 0, sat = 0, bSum = 0, bN = 0;
  const x0 = Math.max(0, Math.floor(cx - 2 * r)), x1 = Math.min(w - 1, Math.ceil(cx + 2 * r));
  const y0 = Math.max(0, Math.floor(cy - 2 * r)), y1 = Math.min(h - 1, Math.ceil(cy + 2 * r));
  if (x1 <= x0 || y1 <= y0) return null;
  for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
    const dx = x - cx, dy = y - cy, dd = dx * dx + dy * dy;
    const i = (y * w + x) * 4, R = data[i], G = data[i + 1], B = data[i + 2], l = lum(R, G, B);
    if (dd <= r2) {
      aSum += l; aN++; aR += R; aG += G; aB += B;
      if (l > peak) peak = l;
      if (R >= 250 && G >= 250 && B >= 250) sat++;
    } else if (dd <= ro2) { bSum += l; bN++; }
  }
  if (aN === 0) return null;
  const bg = bN ? bSum / bN : 0;
  return { flux: Math.max(0, aSum - bg * aN), meanLum: aSum / aN, peak, satFrac: sat / aN, bg, n: aN, r: aR / aN, g: aG / aN, b: aB / aN };
}

/* rough apparent magnitude from a flux ratio vs a reference of known magnitude */
export function relMag(fluxObj, fluxRef, magRef) {
  if (!(fluxObj > 0) || !(fluxRef > 0)) return null;
  return magRef - 2.5 * Math.log10(fluxObj / fluxRef);
}

/* colour descriptor from mean RGB (0..255) */
export function colorDesc(r, g, b) {
  if (r > 235 && g > 235 && b > 235) return "white (saturated core)";
  const mx = Math.max(r, g, b, 1), rr = r / mx, gg = g / mx, bb = b / mx;
  const warm = rr - bb, greenish = gg - (rr + bb) / 2;
  if (warm > 0.28 && gg < 0.62) return "red / orange";
  if (greenish > 0.16 && warm < 0.1) return "greenish";
  if (warm < -0.2) return "blue-white";
  if (warm > 0.12) return "warm white / amber";
  if (Math.abs(warm) <= 0.12) return "neutral white";
  return "warm white";
}
