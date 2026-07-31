/* ============================================================
   TRIANGULATION & ANALYSIS
   N sight lines → least-squares 3D fix, with honest quality
   reporting: convergence angle, per-ray miss distance, and a
   rating that says "poor" when it is poor.
   ============================================================ */

import { D2R, R2D, dot, sub, scl, mag, enuFromGeo, geoFromEnu, dirFromAzEl, dirFromAzElAt } from "./geodesy.js";
import { isNum } from "./format.js";
import { angSizeFromPoints } from "./projection.js";

export function solve3(A, b) {
  const M = A.map((r, i) => [...r, b[i]]);
  for (let c = 0; c < 3; c++) {
    let p = c;
    for (let r = c + 1; r < 3; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-12) return null;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = 0; r < 3; r++) {
      if (r === c) continue;
      const f = M[r][c] / M[c][c];
      for (let k = c; k < 4; k++) M[r][k] -= f * M[c][k];
    }
  }
  return [M[0][3] / M[0][0], M[1][3] / M[1][1], M[2][3] / M[2][2]];
}

/* Least-squares intersection of N sight lines {P, d} */
export function intersectLines(lines) {
  const A = [[0, 0, 0], [0, 0, 0], [0, 0, 0]], b = [0, 0, 0];
  for (const { P, d } of lines) {
    const M = [
      [1 - d[0] * d[0], -d[0] * d[1], -d[0] * d[2]],
      [-d[1] * d[0], 1 - d[1] * d[1], -d[1] * d[2]],
      [-d[2] * d[0], -d[2] * d[1], 1 - d[2] * d[2]],
    ];
    for (let i = 0; i < 3; i++) {
      b[i] += M[i][0] * P[0] + M[i][1] * P[1] + M[i][2] * P[2];
      for (let j = 0; j < 3; j++) A[i][j] += M[i][j];
    }
  }
  const X = solve3(A, b);
  if (!X) return null;
  let missSq = 0; const ts = [];
  for (const { P, d } of lines) {
    const w = sub(X, P); const t = dot(w, d); ts.push(t);
    const perp = sub(w, scl(d, t)); missSq += dot(perp, perp);
  }
  return { X, ts, rmsMiss: Math.sqrt(missSq / lines.length) };
}

export function analyze(sources, sigmaDeg = 1) {
  const valid = sources.filter(
    (s) => isNum(s.lat) && isNum(s.lon) && isNum(s.A.az) && isNum(s.A.el)
  );
  if (valid.length < 2) return { ok: false, validCount: valid.length };

  const ref = { lat: +valid[0].lat, lon: +valid[0].lon, alt: isNum(valid[0].alt) ? +valid[0].alt : 0 };
  const obs = valid.map((s) => ({
    s,
    P: enuFromGeo(+s.lat, +s.lon, isNum(s.alt) ? +s.alt : 0, ref),
    /* az/el is measured against THIS observer's local vertical; rotate it into
       the shared reference frame (identity for the reference observer) */
    dA: dirFromAzElAt(+s.A.az, +s.A.el, +s.lat, +s.lon, ref),
  }));

  /* baseline & convergence */
  let baseline = 0;
  for (let i = 0; i < obs.length; i++)
    for (let j = i + 1; j < obs.length; j++)
      baseline = Math.max(baseline, mag(sub(obs[i].P, obs[j].P)));

  let conv = Infinity;
  for (let i = 0; i < obs.length; i++)
    for (let j = i + 1; j < obs.length; j++) {
      const c = Math.min(1, Math.max(-1, dot(obs[i].dA, obs[j].dA)));
      conv = Math.min(conv, Math.acos(c) * R2D);
    }

  const solA = intersectLines(obs.map((o) => ({ P: o.P, d: o.dA })));
  if (!solA) return { ok: false, validCount: valid.length, parallel: true, baseline };

  const behind = solA.ts.some((t) => t <= 0);
  const meanDist = solA.ts.reduce((a, b) => a + b, 0) / solA.ts.length;
  const missRatio = solA.rmsMiss / Math.max(1, meanDist);

  /* per-source distance & implied size */
  const perSource = obs.map((o, i) => {
    const dist = solA.ts[i];
    const ang =
      angSizeFromPoints(o.s.A.p1, o.s.A.p2, o.s.natW, o.s.natH, +o.s.fovH) ??
      (isNum(o.s.A.angManual) ? +o.s.A.angManual : null);
    const size = ang != null && dist > 0 ? 2 * dist * Math.tan((ang * D2R) / 2) : null;
    return { name: o.s.name, dist, ang, size };
  });
  const sizes = perSource.filter((p) => p.size != null).map((p) => p.size);
  const sizeAvg = sizes.length ? sizes.reduce((a, b) => a + b, 0) / sizes.length : null;

  const geoA = geoFromEnu(solA.X, ref);

  /* position uncertainty heuristic: dist * sigma / sin(convergence) */
  const sigma = sigmaDeg * D2R;
  const posErr = conv > 0.01 ? (meanDist * sigma) / Math.sin(Math.min(conv, 90) * D2R) : Infinity;

  let rating = "poor";
  if (!behind && conv >= 4 && missRatio < 0.02) rating = "excellent";
  else if (!behind && conv >= 2 && missRatio < 0.05) rating = "good";
  else if (!behind && missRatio < 0.15) rating = "fair";

  /* ---- Moment B: velocity ---- */
  let motion = null;
  const obsB = obs.filter((o) => isNum(o.s.B.az) && isNum(o.s.B.el));
  if (obsB.length >= 2) {
    const solB = intersectLines(
      obsB.map((o) => ({ P: o.P, d: dirFromAzElAt(+o.s.B.az, +o.s.B.el, +o.s.lat, +o.s.lon, ref) }))
    );
    if (solB && !solB.ts.some((t) => t <= 0)) {
      const timed = obsB.filter((o) => isNum(o.s.A.t) && isNum(o.s.B.t));
      let dt = null;
      if (timed.length) {
        dt =
          timed.reduce((a, o) => a + (+o.s.B.t - +o.s.A.t), 0) / timed.length;
      }
      const disp = sub(solB.X, solA.X);
      const geoB = geoFromEnu(solB.X, ref);
      motion = {
        disp: mag(disp),
        geoB,
        XB: solB.X,
        rmsMissB: solB.rmsMiss,
        dt,
        v: dt && dt > 0 ? scl(disp, 1 / dt) : null,
      };
      if (motion.v) {
        motion.speed = mag(motion.v);
        motion.heading = ((Math.atan2(motion.v[0], motion.v[1]) * R2D) + 360) % 360;
        motion.vRate = motion.v[2];
      }
    }
  }

  return {
    ok: true, ref, obs, solA, geoA, perSource, sizeAvg,
    baseline, conv, missRatio, meanDist, posErr, behind, rating, motion,
  };
}

/* When two bearings can't meet in front of both observers, use the per-photo
   ANGULAR SIZES (range ratio) to test each compass for self-consistency —
   phone magnetometers near vehicles are routinely 20–60° off, and this
   identifies which one to distrust and what it should have read. */
export function arbitrateBearings(sources) {
  const obs = sources
    .filter((s) => isNum(s.lat) && isNum(s.lon) && isNum(s.A?.az))
    .map((s) => ({
      s,
      ang: angSizeFromPoints(s.A?.p1, s.A?.p2, s.natW, s.natH, +s.fovH) ??
        (isNum(s.A?.angManual) ? +s.A.angManual : null),
    }))
    .filter((o) => o.ang != null);
  if (obs.length < 2) return null;
  const [a, b] = obs;
  const mE = 111320 * Math.cos(+a.s.lat * D2R), mN = 111320;
  const Pa = [0, 0];
  const Pb = [(+b.s.lon - +a.s.lon) * mE, (+b.s.lat - +a.s.lat) * mN];
  const da = [Math.sin(+a.s.A.az * D2R), Math.cos(+a.s.A.az * D2R)];
  const db = [Math.sin(+b.s.A.az * D2R), Math.cos(+b.s.A.az * D2R)];
  const det = da[0] * -db[1] - da[1] * -db[0];
  let t = null, u = null;
  if (Math.abs(det) > 1e-9) {
    t = (Pb[0] * -db[1] - Pb[1] * -db[0]) / det;
    u = (da[0] * Pb[1] - da[1] * Pb[0]) / det;
  }
  if (t != null && t > 0 && u > 0) return null; // geometry is fine — nothing to arbitrate
  const ratio = a.ang / b.ang; // = rangeB / rangeA
  const solveOn = (Po, d, Pother, k) => {
    const px = Po[0] - Pother[0], py = Po[1] - Pother[1];
    const bq = 2 * (d[0] * px + d[1] * py), cq = px * px + py * py, aq = k * k - 1;
    if (Math.abs(aq) < 1e-9) return null;
    const disc = bq * bq + 4 * aq * cq;
    if (disc < 0) return null;
    const tt = (bq + Math.sqrt(disc)) / (2 * aq);
    return tt > 0 ? tt : null;
  };
  const mk = (trust) => {
    const self = trust === "A" ? a : b, other = trust === "A" ? b : a;
    const Pself = trust === "A" ? Pa : Pb, Poth = trust === "A" ? Pb : Pa;
    const d = trust === "A" ? da : db, k = trust === "A" ? ratio : 1 / ratio;
    const tt = solveOn(Pself, d, Poth, k);
    if (tt == null) return null;
    const Po = [Pself[0] + d[0] * tt, Pself[1] + d[1] * tt];
    const azTrue = ((Math.atan2(Po[0] - Poth[0], Po[1] - Poth[1]) * R2D) + 360) % 360;
    const err = Math.abs(((azTrue - +other.s.A.az + 540) % 360) - 180);
    return { trustName: self.s.name, otherName: other.s.name, range: tt, otherRange: k * tt, azOtherTrue: azTrue, err, size: self.ang * D2R * tt };
  };
  const cA = mk("A"), cB = mk("B");
  const best = cA && cB ? (cA.err < cB.err ? cA : cB) : cA || cB;
  return { nameA: a.s.name, nameB: b.s.name, ratio, best };
}

/* Two views of an elongated/planar object foreshorten differently — solve
   the pair jointly for TRUE span + long-axis azimuth (two-fold ambiguous;
   a third viewpoint or knowing the facing breaks the tie). */
export function aspectSpan(fix) {
  if (!fix?.ok || !fix.perSource || !fix.obs) return null;
  const V = fix.perSource
    .map((p, i) => ({ w: p.size, b: +fix.obs[i].s.A.az * D2R }))
    .filter((v) => v.w != null && isFinite(v.b));
  if (V.length < 2) return null;
  const ws = V.map((v) => v.w);
  if (Math.max(...ws) / Math.min(...ws) < 1.12) return null; // no elongation signal
  /* least-squares over long-axis azimuth ψ: S(ψ) closed-form, scan ψ.
     With 2 views both mirror solutions fit exactly; 3+ views break the tie. */
  const cands = [];
  for (let psiDeg = 0; psiDeg < 180; psiDeg += 0.5) {
    const psi = psiDeg * D2R;
    let num = 0, den = 0;
    for (const v of V) { const s = Math.abs(Math.sin(psi - v.b)); num += v.w * s; den += s * s; }
    if (den < 1e-6) continue;
    const S = num / den;
    let r2 = 0;
    for (const v of V) { const e = v.w - S * Math.abs(Math.sin(psi - v.b)); r2 += e * e; }
    cands.push({ S, psi: psiDeg, r2 });
  }
  if (!cands.length) return null;
  cands.sort((a, b) => a.r2 - b.r2);
  const top = cands[0];
  const out = [{ S: top.S, psi: top.psi, rms: Math.sqrt(top.r2 / V.length), n: V.length }];
  if (out[0].rms / out[0].S > 0.18) return null; // widths don't fit ONE elongated object — stay silent
  if (V.length === 2) { // mirror ambiguity only exists for exactly two views
    /* The two views' widths have a SECOND exact-fit long axis (the ratio
       sin²(ψ−b₁)=R²sin²(ψ−b₂) has two roots). Its span may equal OR differ
       from the primary's — a same-span mirror (e.g. bearings ~90° apart) is
       just as real, so DON'T gate on the span differing; report the nearest
       distinct-axis low-residual candidate. ψ is an axis (mod 180). */
    const thresh = Math.max(top.r2 * 2, (0.02 * top.S) ** 2 * V.length);
    for (const c of cands) {
      if (c.r2 > thresh) break;
      let d = Math.abs(c.psi - top.psi); d = Math.min(d, 180 - d); // axis distance, mod 180
      if (d > 20) { out.push({ S: c.S, psi: c.psi }); break; }
    }
  }
  return out;
}

/* Covariance ellipse of a 2-D point cloud (e.g. a fix perturbed by ±1° of
   pointing). Returns 1σ semi-axes and the major-axis bearing (from North,
   clockwise, mod 180 since an axis is undirected). Pure — asserted. */
export function covEllipse(pts) {
  const n = pts.length;
  if (n < 3) return null;
  let mx = 0, my = 0;
  for (const p of pts) { mx += p[0]; my += p[1]; }
  mx /= n; my /= n;
  let sxx = 0, syy = 0, sxy = 0;
  for (const p of pts) { const dx = p[0] - mx, dy = p[1] - my; sxx += dx * dx; syy += dy * dy; sxy += dx * dy; }
  sxx /= n; syy /= n; sxy /= n;
  const tr = sxx + syy, det = sxx * syy - sxy * sxy;
  const disc = Math.sqrt(Math.max(0, tr * tr / 4 - det));
  const l1 = tr / 2 + disc, l2 = Math.max(0, tr / 2 - disc);
  const major = Math.sqrt(Math.max(0, l1)), minor = Math.sqrt(l2);
  const ang = Math.atan2(l1 - sxx, sxy); // major eigenvector angle from +x (East)
  let brg = (90 - ang * 180 / Math.PI) % 180; if (brg < 0) brg += 180;
  return { major, minor, bearing: brg };
}
