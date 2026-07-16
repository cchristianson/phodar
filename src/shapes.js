/* ============================================================
   3D SHAPE SYSTEM — wireframe solids for the photo shape-fitter.
   Pick a solid, rotate it in 3D; the PROJECTED silhouette writes
   A.p1/p2 while the stored rotation matrix records the object's
   orientation in space — a foreshortened tic-tac is a rotated
   capsule, not a mislabeled orb.
   ============================================================ */

import { D2R } from "./math/geodesy.js";

/* --- 3D wireframe fits: pick a solid, drag to rotate it in 3D, slider for
       size. The PROJECTED silhouette writes A.p1/p2, while the stored pose
       (rotation matrix) records the object's orientation in space — a
       foreshortened tic-tac is a rotated capsule, not a mislabeled orb. --- */
export const SHAPES = [
  { k: "orb", label: "● Orb" },
  { k: "saucer", label: "🛸 Saucer" },
  { k: "capsule", label: "💊 Tic-tac" },
  { k: "tri", label: "▲ Triangle" },
  { k: "plane", label: "✈ Plane" },
  { k: "bird", label: "🕊 Bird" },
  { k: "drone", label: "❖ Drone" },
];
export const I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
export const rotX3 = (d) => { const a = d * D2R, c = Math.cos(a), s = Math.sin(a); return [1, 0, 0, 0, c, -s, 0, s, c]; };
export const rotY3 = (d) => { const a = d * D2R, c = Math.cos(a), s = Math.sin(a); return [c, 0, s, 0, 1, 0, -s, 0, c]; };
export const rotZ3 = (d) => { const a = d * D2R, c = Math.cos(a), s = Math.sin(a); return [c, -s, 0, s, c, 0, 0, 0, 1]; };
export const mul3 = (A, B) => { const R = new Array(9); for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) { let v = 0; for (let k = 0; k < 3; k++) v += A[i * 3 + k] * B[k * 3 + j]; R[i * 3 + j] = v; } return R; };
export const app3 = (M, p) => [M[0] * p[0] + M[1] * p[1] + M[2] * p[2], M[3] * p[0] + M[4] * p[1] + M[5] * p[2], M[6] * p[0] + M[7] * p[1] + M[8] * p[2]];

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
    C.push([[-0.37, 0, 0], [-0.47, 0, -0.17], [-0.52, 0, -0.17], [-0.50, 0, 0], [-0.37, 0, 0]]);                                   // vertical fin
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
      C.push([
        [0.06 + wingX, s * 0.03, 0], [0.05 + wingX, s * 0.30 * wingF, -0.02 * wingF], [0.02 + wingX, s * 0.5 * wingF, -0.05 * wingF],
        [-0.08 + wingX, s * 0.5 * wingF, -0.05 * wingF], [-0.07 + wingX, s * 0.28 * wingF, -0.02 * wingF], [-0.06 + wingX, s * 0.03, 0], [0.06 + wingX, s * 0.03, 0],
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
  } else { // tri — thin equilateral plate
    const R = 0.5774, th = 0.05;
    const v = [90, 210, 330].map((d) => [Math.cos(d * D2R) * R, Math.sin(d * D2R) * R]);
    for (const z of [th, -th]) C.push([...v, v[0]].map(([x, y]) => [x, y, z]));
    for (const [x, y] of v) C.push([[x, y, th], [x, y, -th]]);
  }
  return C;
}
export const SHAPE_R0 = () => ({ orb: I3, saucer: rotX3(-62), capsule: I3, tri: rotX3(-24), plane: rotX3(-55), bird: rotX3(-60), drone: rotX3(-40) });

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
