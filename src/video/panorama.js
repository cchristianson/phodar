/* ============================================================
   Retrospective panorama from a stabilized clip.

   An iPhone panorama registers frames as you sweep; the stabilize walk
   already did that in retrospect — posePath knows every frame's az /
   el / roll / FOV, however messy the hand motion was, zooms included.
   This module projects each frame into ONE equirectangular canvas
   (x = azimuth, y = elevation — the projection that tolerates any
   sweep width, where a rectilinear virtual camera degenerates past
   ~100°) and composites them.

   Two composition decisions that matter:
   - Frames are painted chronologically, then the SHARPEST third
     (narrowest FOV) is repainted on top in a second pass — a zoomed
     stretch becomes a high-resolution inset instead of being buried
     under the wide shots that came later.
   - Each frame's texture gets a feathered alpha edge before warping,
     so exposure differences between passes blend instead of leaving
     hard seams.

   Pure geometry (layout, mapping, ordering) is node-testable; the two
   canvas helpers at the bottom touch 2D contexts only (no DOM queries)
   and are exercised in a browser harness.
   ============================================================ */

import { pixToDirK } from "../math/projection.js";
import { dirToAzEl } from "../math/geodesy.js";

const sd = (a, b) => ((a - b + 540) % 360) - 180; // signed shortest az difference

/* posePath entries → samples with a CONTINUOUS azimuth axis (a pan that
   crosses 359→0 must not tear the canvas in half) */
export function unwrapSamples(path) {
  let u = null;
  return path.map((p, i) => {
    u = i === 0 ? p.az : u + sd(p.az, path[i - 1].az);
    return { ...p, uAz: u };
  });
}

/* the frame's border directions under its pose (continuous az) —
   9 points per edge catch the roll/lens bowing of the frame outline */
export function frameBorder(pose, W, H, n = 9) {
  const pts = [];
  const push = (x, y) => {
    const g = pixToDirK(x, y, W, H, pose.uAz, pose.el, pose.roll || 0, pose.fov, pose.k || 0);
    const ae = dirToAzEl(g);
    pts.push({ az: pose.uAz + sd(ae.az, ((pose.uAz % 360) + 360) % 360), el: ae.el });
  };
  for (let i = 0; i < n; i++) { const t = i / (n - 1); push(t * W, 0); push(t * W, H); push(0, t * H); push(W, t * H); }
  return pts;
}

/* canvas layout covering every frame, at the sharpest frame's native
   angular resolution, capped by the iOS canvas guards */
export function panoLayout(samples, W, H, opts = {}) {
  const maxSide = opts.maxSide ?? 4600, maxPx = opts.maxPx ?? 16e6, pad = opts.padDeg ?? 0.4;
  let azMin = 1e9, azMax = -1e9, elMin = 1e9, elMax = -1e9, ppd = 0;
  for (const p of samples) {
    for (const b of frameBorder(p, W, H)) {
      if (b.az < azMin) azMin = b.az;
      if (b.az > azMax) azMax = b.az;
      if (b.el < elMin) elMin = b.el;
      if (b.el > elMax) elMax = b.el;
    }
    ppd = Math.max(ppd, W / p.fov);
  }
  azMin -= pad; azMax += pad; elMin -= pad; elMax += pad;
  const spanAz = azMax - azMin, spanEl = elMax - elMin;
  ppd = Math.min(ppd, maxSide / spanAz, maxSide / spanEl, Math.sqrt(maxPx / (spanAz * spanEl)));
  return { azMin, azMax, elMin, elMax, ppd, W: Math.max(2, Math.round(spanAz * ppd)), H: Math.max(2, Math.round(spanEl * ppd)) };
}

export function equirectXY(layout, az, el) {
  return [(az - layout.azMin) * layout.ppd, (layout.elMax - el) * layout.ppd];
}

/* chronological pass, then the sharpest third repainted on top (still in
   time order, so exposure varies smoothly within the pass) */
export function renderOrder(samples) {
  const idx = samples.map((_, i) => i);
  const fovs = samples.map((s) => s.fov).slice().sort((a, b) => a - b);
  const cut = fovs[Math.floor((fovs.length - 1) / 3)];
  const sharp = idx.filter((i) => samples[i].fov <= cut);
  return sharp.length && sharp.length < samples.length ? [...idx, ...sharp] : idx;
}

/* which frames deserve to be in the panorama at all — the first field
   render trusted every pose and the solve's weak stretches (few anchors,
   chained drift, motion blur mid-whip) landed tiles visibly wrong. Held
   frames never qualify; frames with few background references or high
   angular rate are dropped unless that would gut the set. */
export function panoPick(path, maxN = 90) {
  let keep = path.filter((p) => !p.h);
  const strong = keep.filter((p, i) => {
    if (p.n != null && p.n < 8) return false;
    const q = keep[i - 1];
    if (q && p.t > q.t) {
      const rate = Math.abs(sd(p.az, q.az)) / Math.max(0.05, p.t - q.t);
      if (rate > 25) return false; // whip-pan: motion blur + solve lag
    }
    return true;
  });
  if (strong.length >= Math.max(8, keep.length * 0.4)) keep = strong;
  const step = Math.max(1, Math.ceil(keep.length / maxN));
  return keep.filter((_, i) => i % step === 0);
}

/* re-registration: in equirect space a pixel shift IS an angular shift,
   so aligning a frame against the growing panorama is a plain 2D
   zero-mean NCC over small shifts of the coarse grayscale, masked to
   where BOTH have content (alpha > 0). This is what real stitchers do —
   the solved pose seeds the placement, the image itself finishes it. */
export function bestShift(base, patch, R = 6) {
  const W = base.width, H = base.height;
  const gb = new Float32Array(W * H), gp = new Float32Array(W * H);
  const ab = new Uint8Array(W * H), ap = new Uint8Array(W * H);
  for (let i = 0, j = 0; i < W * H; i++, j += 4) {
    gb[i] = 0.299 * base.data[j] + 0.587 * base.data[j + 1] + 0.114 * base.data[j + 2];
    gp[i] = 0.299 * patch.data[j] + 0.587 * patch.data[j + 1] + 0.114 * patch.data[j + 2];
    ab[i] = base.data[j + 3] > 40 ? 1 : 0;
    ap[i] = patch.data[j + 3] > 40 ? 1 : 0;
  }
  let best = null;
  for (let dy = -R; dy <= R; dy++) {
    for (let dx = -R; dx <= R; dx++) {
      let n = 0, sb = 0, sp = 0, sbb = 0, spp = 0, sbp = 0;
      for (let y = Math.max(0, dy); y < Math.min(H, H + dy); y++) {
        const yp = y - dy;
        for (let x = Math.max(0, dx); x < Math.min(W, W + dx); x++) {
          const xp = x - dx;
          const ib = y * W + x, ip = yp * W + xp;
          if (!ab[ib] || !ap[ip]) continue;
          const b = gb[ib], p = gp[ip];
          n++; sb += b; sp += p; sbb += b * b; spp += p * p; sbp += b * p;
        }
      }
      if (n < 200) continue;
      const vb = sbb - (sb * sb) / n, vp = spp - (sp * sp) / n;
      if (vb < 1e-6 || vp < 1e-6) continue;
      const ncc = (sbp - (sb * sp) / n) / Math.sqrt(vb * vp);
      if (!best || ncc > best.score) best = { dx, dy, score: ncc, n };
    }
  }
  return best && best.score >= 0.35 ? best : null;
}

/* ---------- canvas side (browser harness–tested) ---------- */

/* feather the texture's edges to transparent so overlapping frames blend */
export function featherAlpha(ctx, W, H, frac = 0.05) {
  const fw = Math.max(2, Math.round(W * frac)), fh = Math.max(2, Math.round(H * frac));
  ctx.save();
  ctx.globalCompositeOperation = "destination-out";
  const edge = (x0, y0, x1, y1, rx, ry, rw, rh) => {
    const g = ctx.createLinearGradient(x0, y0, x1, y1);
    g.addColorStop(0, "rgba(0,0,0,1)"); g.addColorStop(1, "rgba(0,0,0,0)");
    ctx.fillStyle = g; ctx.fillRect(rx, ry, rw, rh);
  };
  edge(0, 0, fw, 0, 0, 0, fw, H);
  edge(W, 0, W - fw, 0, W - fw, 0, fw, H);
  edge(0, 0, 0, fh, 0, 0, W, fh);
  edge(0, H, 0, H - fh, 0, H - fh, W, fh);
  ctx.restore();
}

/* warp one (feathered) frame texture into the pano — triangle grid,
   affine per cell, the same rasterizing trick as the sky-view mesh */
export function drawFramePano(ctx, tex, tw, th, pose, layout, grid = 16) {
  const NC = grid, NR = Math.max(6, Math.round(grid * th / tw));
  const az0 = ((pose.uAz % 360) + 360) % 360;
  const dst = [];
  for (let r = 0; r <= NR; r++) {
    const row = [];
    for (let c = 0; c <= NC; c++) {
      const g = pixToDirK((c / NC) * tw, (r / NR) * th, tw, th, pose.uAz, pose.el, pose.roll || 0, pose.fov, pose.k || 0);
      const ae = dirToAzEl(g);
      row.push(equirectXY(layout, pose.uAz + sd(ae.az, az0), ae.el));
    }
    dst.push(row);
  }
  const tri = (s0, s1, s2, d0, d1, d2) => {
    const cx = (d0[0] + d1[0] + d2[0]) / 3, cy = (d0[1] + d1[1] + d2[1]) / 3;
    const ex = (p) => { const dx = p[0] - cx, dy = p[1] - cy, L = Math.hypot(dx, dy) || 1; return [p[0] + (dx / L) * 0.5, p[1] + (dy / L) * 0.5]; };
    const e0 = ex(d0), e1 = ex(d1), e2 = ex(d2);
    ctx.save();
    ctx.beginPath(); ctx.moveTo(e0[0], e0[1]); ctx.lineTo(e1[0], e1[1]); ctx.lineTo(e2[0], e2[1]); ctx.closePath(); ctx.clip();
    const [x0, y0] = s0, [x1, y1] = s1, [x2, y2] = s2;
    const den = x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1);
    if (den) {
      const aM = (d0[0] * (y1 - y2) + d1[0] * (y2 - y0) + d2[0] * (y0 - y1)) / den;
      const bM = (d0[1] * (y1 - y2) + d1[1] * (y2 - y0) + d2[1] * (y0 - y1)) / den;
      const cM = (d0[0] * (x2 - x1) + d1[0] * (x0 - x2) + d2[0] * (x1 - x0)) / den;
      const dM = (d0[1] * (x2 - x1) + d1[1] * (x0 - x2) + d2[1] * (x1 - x0)) / den;
      ctx.transform(aM, bM, cM, dM, d0[0] - aM * x0 - cM * y0, d0[1] - bM * x0 - dM * y0);
      ctx.drawImage(tex, 0, 0);
    }
    ctx.restore();
  };
  const sx = (c) => (c / NC) * tw, sy = (r) => (r / NR) * th;
  for (let r = 0; r < NR; r++) for (let c = 0; c < NC; c++) {
    const d00 = dst[r][c], d10 = dst[r][c + 1], d01 = dst[r + 1][c], d11 = dst[r + 1][c + 1];
    tri([sx(c), sy(r)], [sx(c + 1), sy(r)], [sx(c + 1), sy(r + 1)], d00, d10, d11);
    tri([sx(c), sy(r)], [sx(c + 1), sy(r + 1)], [sx(c), sy(r + 1)], d00, d11, d01);
  }
}
