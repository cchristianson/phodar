/* ============================================================
   3D SHAPE SYSTEM — wireframe solids for the photo shape-fitter.
   Pick a solid, rotate it in 3D; the PROJECTED silhouette writes
   A.p1/p2 while the stored rotation matrix records the object's
   orientation in space — a foreshortened tic-tac is a rotated
   capsule, not a mislabeled orb.
   ============================================================ */

import { D2R } from "./math/geodesy.js";
import { isNum } from "./math/format.js";

/* --- 3D wireframe fits: pick a solid, drag to rotate it in 3D, slider for
       size. The PROJECTED silhouette writes A.p1/p2, while the stored pose
       (rotation matrix) records the object's orientation in space — a
       foreshortened tic-tac is a rotated capsule, not a mislabeled orb. --- */
export const SHAPES = [
  { k: "orb", label: "● Orb" },
  { k: "saucer", label: "🛸 Saucer" },
  { k: "capsule", label: "💊 Tic-tac" },
  { k: "tri", label: "▲ Triangle" },
  { k: "cube", label: "⬛ Cube" },
  { k: "plane", label: "✈ Jet" },
  { k: "prop", label: "🛩 Small plane" },
  { k: "heli", label: "🚁 Helicopter" },
  { k: "bird", label: "🕊 Bird" },
  { k: "drone", label: "❖ Drone" },
  { k: "jelly", label: "🪼 Jellyfish" },
];
export const I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
export const rotX3 = (d) => { const a = d * D2R, c = Math.cos(a), s = Math.sin(a); return [1, 0, 0, 0, c, -s, 0, s, c]; };
export const rotY3 = (d) => { const a = d * D2R, c = Math.cos(a), s = Math.sin(a); return [c, 0, s, 0, 1, 0, -s, 0, c]; };
export const rotZ3 = (d) => { const a = d * D2R, c = Math.cos(a), s = Math.sin(a); return [c, -s, 0, s, c, 0, 0, 0, 1]; };
export const mul3 = (A, B) => { const R = new Array(9); for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) { let v = 0; for (let k = 0; k < 3; k++) v += A[i * 3 + k] * B[k * 3 + j]; R[i * 3 + j] = v; } return R; };
export const app3 = (M, p) => [M[0] * p[0] + M[1] * p[1] + M[2] * p[2], M[3] * p[0] + M[4] * p[1] + M[5] * p[2], M[6] * p[0] + M[7] * p[1] + M[8] * p[2]];

/* --- ROTATION INTERPOLATION (quaternion SLERP) — a 3D orientation can't be
   lerped as a matrix (that shears and shrinks it); convert to a quaternion,
   spherically interpolate, convert back. Used to smoothly tumble the fitted
   model between the attitudes the user keyframed along the track. --- */
export function quatFromMat3(m) { // m row-major, orthonormal
  const t = m[0] + m[4] + m[8];
  let w, x, y, z;
  if (t > 0) { const s = Math.sqrt(t + 1) * 2; w = 0.25 * s; x = (m[7] - m[5]) / s; y = (m[2] - m[6]) / s; z = (m[3] - m[1]) / s; }
  else if (m[0] > m[4] && m[0] > m[8]) { const s = Math.sqrt(1 + m[0] - m[4] - m[8]) * 2; w = (m[7] - m[5]) / s; x = 0.25 * s; y = (m[1] + m[3]) / s; z = (m[2] + m[6]) / s; }
  else if (m[4] > m[8]) { const s = Math.sqrt(1 + m[4] - m[0] - m[8]) * 2; w = (m[2] - m[6]) / s; x = (m[1] + m[3]) / s; y = 0.25 * s; z = (m[5] + m[7]) / s; }
  else { const s = Math.sqrt(1 + m[8] - m[0] - m[4]) * 2; w = (m[3] - m[1]) / s; x = (m[2] + m[6]) / s; y = (m[5] + m[7]) / s; z = 0.25 * s; }
  const n = Math.hypot(w, x, y, z) || 1; return [w / n, x / n, y / n, z / n];
}
export function mat3FromQuat(q) {
  const [w, x, y, z] = q;
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y),
  ];
}
export function slerp3(mA, mB, u) {
  let a = quatFromMat3(mA), b = quatFromMat3(mB);
  let d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
  if (d < 0) { b = b.map((v) => -v); d = -d; }            // shortest arc
  if (d > 0.9995) {                                        // near-parallel → nlerp
    const r = a.map((v, i) => v + (b[i] - v) * u), n = Math.hypot(r[0], r[1], r[2], r[3]) || 1;
    return mat3FromQuat(r.map((v) => v / n));
  }
  const th0 = Math.acos(Math.min(1, d)), th = th0 * u, s0 = Math.sin(th0);
  const sa = Math.sin(th0 - th) / s0, sb = Math.sin(th) / s0;
  return mat3FromQuat(a.map((v, i) => sa * v + sb * b[i]));
}
function interp1(kf, t) { // kf sorted [{t, v}] → linear, clamped at the ends
  if (t <= kf[0].t) return kf[0].v;
  if (t >= kf[kf.length - 1].t) return kf[kf.length - 1].v;
  let i = 0; while (i < kf.length - 1 && kf[i + 1].t < t) i++;
  const a = kf[i], b = kf[i + 1];
  return a.v + (b.v - a.v) * ((t - a.t) / Math.max(1e-9, b.t - a.t));
}
/* Sample the fitted model's SIZE + ORIENTATION at time t, interpolating the
   keyframes the user set along the track: `wpx` (apparent width, px) points and
   `rotM` (attitude) points are independent — each interpolated over its own
   set, clamped past the ends, and falling back to the fitted shape where the
   user set none. Two size marks ⇒ a smooth ramp between them; N marks ⇒
   piecewise; same for attitude via SLERP. Pure. The caller normalises the
   returned `wpx` to a real sizeNat through shapeProjNat (projection-aware).

   `opts = { markT, wFit }` (optional): the FITTED shape is the implicit
   BASELINE keyframe at the frame it was fit on (markT) — apparent width wFit
   and attitude shapeFit.rotM. Injecting it makes a SINGLE user adjustment ramp
   from the fit (or from whichever earlier point WAS adjusted) instead of
   snapping the whole track: "changes transition from changes, not from
   un-adjusted points". It's injected only when the caller passes the baseline
   AND the user set ≥1 keyframe of that kind (so the un-keyframed case still
   returns the fit unchanged), and skipped if a real keyframe already sits on
   markT. Without opts the behaviour is exactly the old 3-arg one. */
export function sampleShapeAt(track, shapeFit, t, opts) {
  const tk = Array.isArray(track) ? track : [];
  const markT = opts && isNum(opts.markT) ? +opts.markT : null;
  const wFit = opts && isNum(opts.wFit) ? +opts.wFit : null;
  let sk = tk.filter((p) => isNum(p.t) && isNum(p.wpx) && +p.wpx > 0).map((p) => ({ t: +p.t, v: +p.wpx })).sort((a, b) => a.t - b.t);
  let rk = tk.filter((p) => isNum(p.t) && Array.isArray(p.rotM) && p.rotM.length === 9).map((p) => ({ t: +p.t, m: p.rotM })).sort((a, b) => a.t - b.t);
  if (markT != null) {
    if (wFit != null && sk.length && !sk.some((k) => Math.abs(k.t - markT) < 1e-4))
      sk = [...sk, { t: markT, v: wFit }].sort((a, b) => a.t - b.t);
    if (shapeFit?.rotM && shapeFit.rotM.length === 9 && rk.length && !rk.some((k) => Math.abs(k.t - markT) < 1e-4))
      rk = [...rk, { t: markT, m: shapeFit.rotM }].sort((a, b) => a.t - b.t);
  }
  let rotM = (shapeFit?.rotM && shapeFit.rotM.length === 9) ? shapeFit.rotM : I3;
  if (rk.length === 1) rotM = rk[0].m;
  else if (rk.length >= 2) {
    if (t <= rk[0].t) rotM = rk[0].m;
    else if (t >= rk[rk.length - 1].t) rotM = rk[rk.length - 1].m;
    else { let i = 0; while (i < rk.length - 1 && rk[i + 1].t < t) i++; const a = rk[i], b = rk[i + 1]; rotM = slerp3(a.m, b.m, (t - a.t) / Math.max(1e-9, b.t - a.t)); }
  }
  return { rotM, wpx: sk.length ? interp1(sk, t) : null };
}

export function shapeWire(kind, aspect, opts) { // unit major dimension, centered at origin
  const C = [];
  const circ = (r, axis, off = 0, n = 40) => Array.from({ length: n + 1 }, (_, i) => {
    const a = (i / n) * Math.PI * 2, u = Math.cos(a) * r, v = Math.sin(a) * r;
    return axis === "z" ? [u, v, off] : axis === "y" ? [u, off, v] : [off, u, v];
  });
  if (kind === "orb") {
    const r = 0.5;
    C.push(circ(r, "z", 0), circ(r, "y", 0), circ(r, "x", 0));
    const zr = 0.25, rr = Math.sqrt(r * r - zr * zr);
    C.push(circ(rr, "z", zr), circ(rr, "z", -zr));
  } else if (kind === "saucer") {
    const r = 0.5, h = 0.11;
    C.push(circ(r, "z", 0)); // rim
    for (let m = 0; m < 4; m++) {
      const rm = rotZ3(m * 45);
      C.push(Array.from({ length: 41 }, (_, i) => {
        const a = (i / 40) * Math.PI * 2;
        return app3(rm, [Math.cos(a) * r, 0, Math.sin(a) * h]);
      }));
    }
    C.push(circ(0.3, "z", h * 0.75), circ(0.3, "z", -h * 0.75));
  } else if (kind === "capsule") {
    const r = 0.5 / Math.max(1.2, aspect || 3), hl = 0.5 - r;
    const stad = (plane) => {
      const pts = [];
      for (let i = 0; i <= 20; i++) { const a = -Math.PI / 2 + (i / 20) * Math.PI; pts.push([hl + Math.cos(a) * r, Math.sin(a) * r]); }
      for (let i = 0; i <= 20; i++) { const a = Math.PI / 2 + (i / 20) * Math.PI; pts.push([-hl + Math.cos(a) * r, Math.sin(a) * r]); }
      pts.push(pts[0]);
      return pts.map(([x, u]) => (plane === "y" ? [x, u, 0] : [x, 0, u]));
    };
    C.push(stad("y"), stad("z"), circ(r, "x", hl), circ(r, "x", -hl));
  } else if (kind === "plane") {
    // stylized airliner — wingspan = 1, fuselage along +X
    const w = 0.036, h = 0.046;
    C.push([[0.5, 0], [0.44, w], [-0.40, w], [-0.5, w * 0.35], [-0.5, -w * 0.35], [-0.40, -w], [0.44, -w], [0.5, 0]]
      .map(([x, y]) => [x, y, 0]));                                    // fuselage planform
    C.push([[0.5, 0], [0.43, -h * 0.7], [-0.38, -h], [-0.5, -h * 0.5], [-0.5, h * 0.4], [-0.42, h], [0.44, h * 0.75], [0.5, 0]]
      .map(([x, z]) => [x, 0, z]));                                    // fuselage side profile
    for (const s of [1, -1]) {
      C.push([[0.13, s * 0.05, 0], [-0.02, s * 0.5, 0], [-0.13, s * 0.5, 0], [-0.11, s * 0.05, 0], [0.13, s * 0.05, 0]]);          // swept wing
      C.push([[-0.40, s * 0.03, 0], [-0.47, s * 0.19, 0], [-0.51, s * 0.19, 0], [-0.485, s * 0.03, 0], [-0.40, s * 0.03, 0]]);     // h-stab
    }
    C.push([[-0.37, 0, 0], [-0.47, 0, 0.17], [-0.52, 0, 0.17], [-0.50, 0, 0], [-0.37, 0, 0]]);                                     // vertical fin (points up, +z = spine)
  } else if (kind === "prop") {
    // light propeller plane — STRAIGHT wings (vs the jet's swept ones), a nose
    // propeller and a taller tail. wingspan = 1, fuselage along +X.
    const w = 0.03, h = 0.04;
    C.push([[0.42, 0], [0.36, w], [-0.34, w], [-0.42, w * 0.4], [-0.42, -w * 0.4], [-0.34, -w], [0.36, -w], [0.42, 0]]
      .map(([x, y]) => [x, y, 0]));                                     // fuselage planform
    C.push([[0.42, 0], [0.4, h * 0.6], [0.2, h], [0.0, h * 1.35], [-0.34, h * 0.55], [-0.42, h * 0.7], [-0.42, -h * 0.4], [-0.2, -h * 0.7], [0.2, -h * 0.6], [0.42, 0]]
      .map(([x, z]) => [x, 0, z]));                                     // fuselage side profile (cabin hump)
    for (const s of [1, -1]) {
      // straight (unswept) wing, high on the fuselage, rounded tip
      C.push([[0.16, s * 0.03, h * 0.7], [0.15, s * 0.5, h * 0.7], [0.05, s * 0.5, h * 0.7], [0.03, s * 0.03, h * 0.7], [0.16, s * 0.03, h * 0.7]]);
      C.push([[0.09, s * 0.05, h * 0.7], [0.11, s * 0.05, 0]]);         // wing strut down to the belly
      C.push([[-0.34, s * 0.03, 0], [-0.4, s * 0.2, 0], [-0.44, s * 0.2, 0], [-0.42, s * 0.03, 0], [-0.34, s * 0.03, 0]]); // h-stab
    }
    C.push([[-0.3, 0, 0.02], [-0.42, 0, 0.24], [-0.48, 0, 0.24], [-0.44, 0, 0], [-0.3, 0, 0.02]]); // tall vertical fin
    // propeller disc at the nose (spins about +X) + two blades + spinner
    const pr = 0.14, px = 0.44;
    C.push(Array.from({ length: 25 }, (_, i) => { const a = (i / 24) * Math.PI * 2; return [px, Math.cos(a) * pr, Math.sin(a) * pr]; }));
    C.push([[px, -pr, 0], [px, pr, 0]], [[px, 0, -pr], [px, 0, pr]]);
  } else if (kind === "heli") {
    // helicopter — nose +X, tail −X; main rotor disc horizontal about +Z (the
    // lift/up axis), tail rotor vertical at the boom tip. Main-rotor diameter = 1.
    const R = 0.5, mz = 0.2;                                     // rotor radius, plane height
    C.push(circ(R, "z", mz));                                    // main rotor disc
    C.push([[0, 0, 0.05], [0, 0, mz]]);                          // rotor mast
    C.push([[-R, 0, mz], [R, 0, mz]], [[0, -R, mz], [0, R, mz]]); // two blades
    // fuselage — planform (top, XY) + side profile (XZ): cockpit at the nose,
    // tapering to a slim tail boom
    C.push([[0.34, 0], [0.27, 0.085], [0.06, 0.1], [-0.16, 0.06], [-0.44, 0.026], [-0.58, 0.018], [-0.58, -0.018], [-0.44, -0.026], [-0.16, -0.06], [0.06, -0.1], [0.27, -0.085], [0.34, 0]]
      .map(([x, y]) => [x, y, 0]));
    C.push([[0.34, 0.01], [0.31, 0.09], [0.12, 0.13], [-0.08, 0.11], [-0.42, 0.055], [-0.58, 0.06], [-0.58, 0.0], [-0.34, -0.06], [-0.06, -0.1], [0.2, -0.075], [0.34, 0.01]]
      .map(([x, z]) => [x, 0, z]));
    // a couple of cabin cross-section rings for volume
    for (const xo of [0.12, -0.06]) C.push(Array.from({ length: 25 }, (_, i) => { const a = (i / 24) * Math.PI * 2; return [xo, Math.cos(a) * 0.092, 0.015 + Math.sin(a) * 0.1]; }));
    // tail rotor — vertical disc (about the Y axis) + two blades, at the boom tip
    const tr = 0.12, tx = -0.58, tz = 0.07;
    C.push(Array.from({ length: 25 }, (_, i) => { const a = (i / 24) * Math.PI * 2; return [tx + Math.cos(a) * tr, 0.03, tz + Math.sin(a) * tr]; }));
    C.push([[tx - tr, 0.03, tz], [tx + tr, 0.03, tz]], [[tx, 0.03, tz - tr], [tx, 0.03, tz + tr]]);
    // vertical tail fin + horizontal stabilizer
    C.push([[-0.44, 0, 0.04], [-0.58, 0, 0.2], [-0.64, 0, 0.12], [-0.56, 0, 0.0], [-0.44, 0, 0.04]]);
    for (const s of [1, -1]) C.push([[-0.46, s * 0.02, 0.03], [-0.55, s * 0.12, 0.04], [-0.6, s * 0.11, 0.04], [-0.52, s * 0.02, 0.03]]);
    // landing skids — two tubes below, on struts
    for (const s of [1, -1]) {
      C.push([[0.24, s * 0.12, -0.16], [0.18, s * 0.12, -0.19], [-0.2, s * 0.12, -0.19], [-0.26, s * 0.12, -0.16]]); // skid tube (turned up fore/aft)
      C.push([[0.12, s * 0.05, -0.03], [0.08, s * 0.12, -0.185]]);   // front strut
      C.push([[-0.1, s * 0.05, -0.03], [-0.14, s * 0.12, -0.185]]);  // rear strut
    }
  } else if (kind === "bird") {
    // gliding bird — head along +X, slight dihedral. wingF scales the
    // lateral span (wingtip reach); wingX sweeps the wing root fore/aft.
    const wingF = opts && isFinite(opts.wing) ? opts.wing : 1;
    const wingX = opts && isFinite(opts.wingX) ? opts.wingX : 0;
    C.push([[0.17, 0], [0.13, 0.03], [-0.12, 0.025], [-0.14, 0], [-0.12, -0.025], [0.13, -0.03], [0.17, 0]]
      .map(([x, y]) => [x, y, 0]));                                   // body planform
    C.push([[0.17, 0], [0.12, -0.035], [-0.12, -0.03], [-0.14, 0], [-0.11, 0.028], [0.13, 0.03], [0.17, 0]]
      .map(([x, z]) => [x, 0, z]));                                   // body profile
    for (const s of [1, -1]) {
      // pointed, swept wing: leading edge sweeps aft to a single tip point,
      // trailing edge is concave — the classic gliding-bird silhouette
      C.push([
        [0.08 + wingX, s * 0.03, 0],                                  // root leading (shoulder)
        [0.05 + wingX, s * 0.22 * wingF, -0.018 * wingF],             // leading edge
        [-0.01 + wingX, s * 0.40 * wingF, -0.035 * wingF],            // leading edge
        [-0.05 + wingX, s * 0.5 * wingF, -0.05 * wingF],              // POINTED TIP (swept aft)
        [-0.10 + wingX, s * 0.38 * wingF, -0.035 * wingF],            // trailing edge (concave)
        [-0.09 + wingX, s * 0.18 * wingF, -0.018 * wingF],            // trailing edge
        [-0.06 + wingX, s * 0.03, 0],                                 // root trailing
        [0.08 + wingX, s * 0.03, 0],                                  // close
      ]);                                                             // wing with dihedral
    }
    C.push([[-0.12, 0.02, 0], [-0.23, 0.08, 0], [-0.25, 0, 0], [-0.23, -0.08, 0], [-0.12, -0.02, 0], [-0.12, 0.02, 0]]); // tail fan
  } else if (kind === "drone") {
    // quadcopter, X-frame: diagonal motor-to-motor span (incl. props) = 1.
    // frame in XY; rotor discs spin about +Z (the lift axis)
    const rm = 0.35, rp = 0.15, zt = 0.06, bx = 0.1, bz = 0.05;
    const top = [[bx, bx, bz], [bx, -bx, bz], [-bx, -bx, bz], [-bx, bx, bz], [bx, bx, bz]];
    C.push(top, top.map(([x, y]) => [x, y, -bz]));                       // body box top + bottom
    for (const sx of [bx, -bx]) for (const sy of [bx, -bx]) C.push([[sx, sy, bz], [sx, sy, -bz]]); // box verticals
    for (const d of [45, 135, 225, 315]) {
      const cx = Math.cos(d * D2R), sy = Math.sin(d * D2R), mx = cx * rm, my = sy * rm;
      C.push([[cx * bx, sy * bx, 0], [mx, my, zt * 0.6]]);               // arm out to the motor
      C.push([[mx, my, zt * 0.6], [mx, my, zt]]);                        // motor post
      C.push(Array.from({ length: 33 }, (_, i) => { const a = (i / 32) * Math.PI * 2; return [mx + Math.cos(a) * rp, my + Math.sin(a) * rp, zt]; })); // rotor disc
      C.push([[mx - cx * rp, my - sy * rp, zt], [mx + cx * rp, my + sy * rp, zt]]);  // prop blade
      C.push([[mx + sy * rp, my - cx * rp, zt], [mx - sy * rp, my + cx * rp, zt]]);  // prop blade
    }
    for (const s of [1, -1]) C.push([[bx * 0.6, s * bx, -bz], [bx * 0.8, s * bx * 1.5, -bz - 0.1], [-bx * 0.8, s * bx * 1.5, -bz - 0.1], [-bx * 0.6, s * bx, -bz]]); // landing skids
  } else if (kind === "jelly") {
    // jellyfish — domed bell up (+z), tentacles trailing down (−z); axis = z,
    // total height ≈ 1, centered by zc. bell diameter = 2·rb.
    const rb = 0.34, hb = 0.34, zt = -0.66, zc = 0.16;
    const tent = opts && isFinite(opts.tent) ? opts.tent : 1; // tendril-length factor
    const Z = (z) => z + zc;
    // bell horizontal rings (rim → top)
    for (const t of [0, 0.4, 0.68, 0.86, 0.96]) {
      const z = hb * t, rr = rb * Math.sqrt(Math.max(0, 1 - t * t));
      C.push(Array.from({ length: 37 }, (_, i) => { const a = (i / 36) * Math.PI * 2; return [Math.cos(a) * rr, Math.sin(a) * rr, Z(z)]; }));
    }
    // bell meridian arcs (rim → over the top → far rim)
    for (const md of [0, 45, 90, 135]) {
      const ca = Math.cos(md * D2R), sa = Math.sin(md * D2R);
      C.push(Array.from({ length: 33 }, (_, i) => { const f = (i / 32) * Math.PI; const rr = rb * Math.cos(f); return [rr * ca, rr * sa, Z(hb * Math.sin(f))]; }));
    }
    // outer tentacles hanging from the rim, gently swaying and tapering inward
    for (let k = 0; k < 8; k++) {
      const a = (k / 8) * Math.PI * 2, ca = Math.cos(a), sa = Math.sin(a), dir = k % 2 ? 1 : -1;
      C.push(Array.from({ length: 9 }, (_, j) => {
        const t = j / 8, rr = rb * (0.96 - 0.55 * t), sway = 0.05 * Math.sin(t * Math.PI * 1.8) * dir;
        return [rr * ca - sa * sway, rr * sa + ca * sway, Z(t * zt * tent)];
      }));
    }
    // shorter inner oral arms, frillier
    for (let k = 0; k < 4; k++) {
      const a = (k / 4) * Math.PI * 2 + 0.4, ca = Math.cos(a), sa = Math.sin(a), dir = k % 2 ? 1 : -1;
      C.push(Array.from({ length: 8 }, (_, j) => {
        const t = j / 7, rr = rb * (0.4 - 0.28 * t), sway = 0.07 * Math.sin(t * Math.PI * 2.5) * dir;
        return [rr * ca - sa * sway, rr * sa + ca * sway, Z(t * zt * 0.62 * tent)];
      }));
    }
  } else if (kind === "cube") {
    /* CUBE ↔ DIAMOND. `squash` tapers the top and bottom faces toward the
       vertical axis while the waist stays put, so the solid sweeps
       continuously from a cube (0) through a truncated gem to a square
       bipyramid (1) — the shape people mean by "diamond". Edge = 1 at
       squash 0, so a face-on cube spans the same as an orb's diameter.
       The waist ring is what makes the morph well-defined: without it the
       side edges would be straight lines from a shrunken top corner to a
       shrunken bottom corner, which is a frustum pair, not a diamond. */
    const q = Math.max(0, Math.min(1, opts && isFinite(opts.squash) ? opts.squash : 0));
    const h = 0.5, m = 0.5, a = m * (1 - q);        // half-height, waist half-width, cap half-width
    const sq = (r, z) => [[r, r, z], [r, -r, z], [-r, -r, z], [-r, r, z], [r, r, z]];
    /* a collapsed cap is a point — drawing its degenerate square would leave a
       dot artefact at the apex, and drawing the waist on an un-squashed cube
       would band it with a line that isn't an edge */
    if (a > 0.005) C.push(sq(a, h), sq(a, -h));
    if (q > 0.01) C.push(sq(m, 0));
    for (const [sx, sy] of [[1, 1], [1, -1], [-1, -1], [-1, 1]])
      C.push([[sx * a, sy * a, h], [sx * m, sy * m, 0], [sx * a, sy * a, -h]]);
  } else { // tri — thin equilateral plate
    const R = 0.5774, th = 0.05;
    const v = [90, 210, 330].map((d) => [Math.cos(d * D2R) * R, Math.sin(d * D2R) * R]);
    for (const z of [th, -th]) C.push([...v, v[0]].map(([x, y]) => [x, y, z]));
    for (const [x, y] of v) C.push([[x, y, th], [x, y, -th]]);
  }
  return C;
}
/* Default 3/4 poses. A shape's +z is its "up" (dome / spine / lift axis /
   bell); on screen y is DOWN, so rotX3(+θ) points +z UP (right-side-up) while
   rotX3(−θ) points it down (inverted). Every shape uses +θ so it starts
   right-side-up — the tilt magnitude sets the (unchanged) viewing angle. The
   triangle also gets an in-plane 180° so its apex points up, not down. */
export const SHAPE_R0 = () => ({ orb: I3, saucer: rotX3(62), capsule: I3, tri: mul3(rotX3(24), rotZ3(180)), cube: mul3(rotX3(26), rotZ3(35)), plane: rotX3(55), prop: rotX3(55), heli: rotX3(48), bird: rotX3(60), drone: rotX3(40), jelly: rotX3(82) });

export function shapeProjNat(sf) { // orthographic project → natural-px curves + silhouette extremes
  const R = sf.roll ? mul3(sf.rotM || I3, rotZ3(sf.roll)) : (sf.rotM || I3);
  const s = sf.sizeNat || 100;
  const curves = shapeWire(sf.kind, sf.aspect, sf).map((c) => c.map((p) => {
    const q = app3(R, p);
    return { x: sf.cx + q[0] * s, y: sf.cy + q[1] * s, z: q[2] };
  }));
  const pts = curves.flat();
  const c0 = pts.reduce((m, p) => ({ x: m.x + p.x / pts.length, y: m.y + p.y / pts.length }), { x: 0, y: 0 });
  let A = pts[0], best = -1;
  for (const p of pts) { const d = (p.x - c0.x) ** 2 + (p.y - c0.y) ** 2; if (d > best) { best = d; A = p; } }
  let B = pts[0]; best = -1;
  for (const p of pts) { const d = (p.x - A.x) ** 2 + (p.y - A.y) ** 2; if (d > best) { best = d; B = p; } }
  let minor = 0;
  const ax = B.x - A.x, ay = B.y - A.y, al = Math.hypot(ax, ay) || 1;
  for (const p of pts) { const d = Math.abs((-ay * (p.x - A.x) + ax * (p.y - A.y)) / al); if (d > minor) minor = d; }
  return { curves, p1: { x: A.x, y: A.y }, p2: { x: B.x, y: B.y }, minorNat: minor * 2 };
}
