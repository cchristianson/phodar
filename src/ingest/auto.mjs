/* AUTO-MEASUREMENT PIPELINE (phase 2, server-side) — turn raw media plus
   whatever context exists into an app-compatible sighting the human reviews.

   Division of labour, stated plainly: the AI (or human caller) supplies the
   JUDGMENT — where the object is (a tap on a returned keyframe), roughly which
   way the camera looked, position/time gleaned from the report text — and this
   pipeline runs the SAME pure measurement modules the app runs (EXIF parser,
   stabilize walk, object tracker, shape projection), then hands back a
   .phodar.json + bundle the app imports for human review. Nothing here invents
   a measurement silently: every field that came from a default or a caller's
   guess is listed in source.ingest.guessed, and the biggest judgment of all —
   the SKY PLACEMENT — is carried as approximate until a human (or a
   star/terrain solve) refines it in the app.

   NODE ONLY (imports media.mjs → ffmpeg). The math imports are the same pure
   ES modules the browser uses. */

import { readFileSync } from "node:fs";
import { probeMedia, decodeFrameAt, openFrameStream, encodeJpeg } from "./media.mjs";
import { parseMediaMeta } from "../exif.js";
import { initTracker, stepTracker, stepObject, snapToObject, smearDrift, despikePath, smoothPathAt, smoothObjPath, smoothObjPathAt, posePathAt, snapDirsToAnchors } from "../video/postrack.js";
import { pixToDirK } from "../math/projection.js";
import { dirToAzEl, unit, clampN, D2R, R2D } from "../math/geodesy.js";
import { shapeProjNat, SHAPE_R0 } from "../shapes.js";
import { analyzeSession, summarizeVerdict } from "../analyze/engine.js";
import { makeZip, strU8 } from "../report/zip.js";

const isNum = (v) => v != null && v !== "" && Number.isFinite(+v);
const round = (v, d) => +(+v).toFixed(d);

/* fovH default when neither EXIF nor context supplies one — the app's guess:
   phone main lens ≈ 68° on the long side, tan-converted for portrait */
const guessFov = (w, h) => (h > w ? round(2 * Math.atan(Math.tan(34 * D2R) * (w / h)) * R2D, 1) : 68);

const whenMsOf = (v) => {
  if (v == null) return null;
  if (isNum(v)) return +v;
  const t = Date.parse(String(v));
  return Number.isFinite(t) ? t : null;
};

/* ============================== the pipeline ============================== */
/* opts:
   context: { lat, lon, elevM, whenMs|whenText, bearingDeg, elevationDeg,
              fovH, name, witnessText, trim:{t0,t1} }
   object:  { t, fx, fy, wfrac? }  — the object mark, fractions of the frame
   track:   [{t, fx, fy}, ...]     — optional extra waypoints (become the
                                     guided track, video only)
   trackW / objW / maxDurS / onProgress(stage, frac)
   Returns { source, session, sessionJson, verdict, summary, bundle:Uint8Array,
             bundleName, notes } */
export async function autoMeasure(filePath, opts = {}) {
  const ctx = opts.context || {};
  const onP = opts.onProgress || (() => {});
  const probe = await probeMedia(filePath);
  const natW = probe.w, natH = probe.h;
  const guessed = [], notes = [];

  /* ---- metadata: EXIF/QuickTime via the app's own parser ---- */
  let meta = null;
  try {
    const buf = readFileSync(filePath);
    meta = parseMediaMeta(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.length), probe.kind === "video") || null;
  } catch (e) { notes.push("metadata parse failed: " + e.message); }

  /* ---- the witness facts: context beats EXIF beats default ---- */
  const lat = isNum(ctx.lat) ? +ctx.lat : (isNum(meta?.lat) ? +meta.lat : null);
  const lon = isNum(ctx.lon) ? +ctx.lon : (isNum(meta?.lon) ? +meta.lon : null);
  const alt = isNum(ctx.elevM) ? +ctx.elevM : (isNum(meta?.alt) ? +meta.alt : null);
  if (lat == null) notes.push("no observer position — the fix, trajectory and every cross-check need lat/lon (set it on step 2)");
  else if (!isNum(ctx.lat)) notes.push("observer position from the file's GPS");
  const whenMs = whenMsOf(ctx.whenMs) ?? whenMsOf(ctx.whenText) ?? (isNum(meta?.timeMs) ? +meta.timeMs : null);
  if (whenMs == null) { guessed.push("capture time (defaulted to now — fix on step 2)"); }
  const fovH = isNum(meta?.fovH) ? +meta.fovH
    : isNum(ctx.fovH) ? +ctx.fovH
      : (guessed.push("camera field of view (no lens metadata — assumed a phone main lens)"), guessFov(natW, natH));
  const bearing = isNum(ctx.bearingDeg) ? +ctx.bearingDeg
    : isNum(meta?.azTrue) ? +meta.azTrue
      : isNum(meta?.az) ? +meta.az
        : (guessed.push("compass bearing (nothing in the file or context — placement points NORTH until corrected)"), 0);
  const elevDeg = isNum(ctx.elevationDeg) ? +ctx.elevationDeg
    : (guessed.push("camera up-angle (assumed 25° — set it in the sky view)"), 25);
  guessed.push("sky placement is approximate — refine in the sky view (snap to ridges / star align / drag) before trusting directions");

  /* ---- the object mark (fractions → native px) ---- */
  const object = opts.object || null;
  if (!object || !isNum(object.fx) || !isNum(object.fy)) {
    throw new Error("an object mark is required: {t, fx, fy} as fractions of the frame (look at the keyframes and mark the object)");
  }
  const markT = probe.kind === "video" ? clampN(+object.t || 0, 0, Math.max(0, probe.durS - 0.05)) : 0;
  let cx = clampN(+object.fx, 0, 1) * natW, cy = clampN(+object.fy, 0, 1) * natH;
  const wpx0 = isNum(object.wfrac) ? clampN(+object.wfrac, 0.001, 0.9) * natW
    : (guessed.push("object apparent size (no wfrac given — assumed small; set it on step 1)"), Math.max(8, natW * 0.012));

  /* ---- trim (video): analyse only the caller's span ---- */
  const durS = probe.durS;
  const trim = probe.kind === "video" && ctx.trim && isNum(ctx.trim.t0) && isNum(ctx.trim.t1)
    ? { t0: clampN(+ctx.trim.t0, 0, durS), t1: clampN(+ctx.trim.t1, 0, durS) } : null;
  const spanT0 = trim ? trim.t0 : 0, spanT1 = trim ? trim.t1 : durS;
  const maxDur = opts.maxDurS || 90;
  if (probe.kind === "video" && spanT1 - spanT0 > maxDur) {
    throw new Error(`clip span is ${(spanT1 - spanT0).toFixed(0)}s — cap is ${maxDur}s. Pass context.trim {t0,t1} around the sighting.`);
  }

  /* ---- refine the mark onto the object (same snap the app does) ---- */
  onP("decoding the marked frame", 0.05);
  const seedRes = Math.min(opts.objW || 1280, natW);
  const seedFrame = await decodeFrameAt(filePath, markT, { maxW: seedRes });
  const osc = seedFrame.w / natW;
  const sn = snapToObject(seedFrame.data, seedFrame.w, seedFrame.h, cx * osc, cy * osc, Math.min(16, Math.max(6, wpx0 * osc * 0.35)));
  cx = sn.x / osc; cy = sn.y / osc;

  /* ---- the source object, app-shaped ---- */
  const rotM = SHAPE_R0().orb;
  const fit1 = shapeProjNat({ kind: "orb", cx, cy, sizeNat: 1, rotM, roll: 0 });
  const w1 = Math.hypot(fit1.p1.x - fit1.p2.x, fit1.p1.y - fit1.p2.y) || 1;
  const shapeFit = { kind: "orb", cx: round(cx, 1), cy: round(cy, 1), sizeNat: round(wpx0 / w1, 2), rotM, roll: 0, hue: 36 };
  const pr = shapeProjNat(shapeFit);

  const source = {
    id: "ing" + Math.random().toString(36).slice(2, 8),
    name: ctx.observerName || "Observer 1",
    lat: lat != null ? String(lat) : "", lon: lon != null ? String(lon) : "", alt: alt != null ? String(alt) : "",
    whenMs: whenMs ?? Date.now(),
    fovH: round(fovH, 1),
    natW, natH,
    mediaKind: probe.kind,
    statement: ctx.witnessText || "",
    mediaAim: { az: round(bearing, 2), el: round(elevDeg, 2), roll: 0 },
    placed: true,
    calib: { method: "auto-ingest", vt: probe.kind === "video" ? round(markT, 3) : null },
    shapeFit,
    A: { p1: { x: round(pr.p1.x, 1), y: round(pr.p1.y, 1) }, p2: { x: round(pr.p2.x, 1), y: round(pr.p2.y, 1) } },
    B: {},
    track: [],
    moments: [],
    ...(trim ? { trim } : {}),
    ...(meta ? { meta } : {}),
  };

  /* the sight-line: the object's pixel through the placement pose — exactly
     the app's commitPlacement derivation */
  const poseAt0 = { az: bearing, el: elevDeg, roll: 0, fov: fovH, k: 0 };
  const dirOf = (px, py, P) => dirToAzEl(pixToDirK(px, py, natW, natH, P.az, P.el, P.roll || 0, P.fov, P.k || 0));

  /* ---- VIDEO: stabilize + object track, the app's own walk ---- */
  if (probe.kind === "video") {
    source.A.videoTime = round(markT, 3);
    source.A.t = markT.toFixed(2);
    source.alignT = round(markT, 3);

    const TW0 = Math.min(opts.trackW || 640, natW);
    const evenW = Math.max(2, Math.round(TW0 / 2) * 2);
    const sc = evenW / natW;
    const TH = Math.max(2, Math.round(natH * sc / 2) * 2);
    const dt = Math.max(0.25, (spanT1 - spanT0) / 140);
    const times = [];
    for (let t = spanT0; t <= spanT1 - 0.005; t += dt) times.push(round(t, 3));
    if (times.length && times[times.length - 1] < spanT1 - dt * 0.5) times.push(round(Math.max(spanT0, spanT1 - 0.06), 3));
    const refT = round(clampN(markT, spanT0, Math.max(spanT0, spanT1 - 0.05)), 3);
    const refPose = { ...poseAt0 };

    onP("stabilizing", 0.1);
    const stream = openFrameStream(filePath, { fps: 12, maxW: evenW, t0: spanT0, t1: spanT1, W: evenW, H: TH });
    const refFrame = await stream.frameAt(refT);
    if (!refFrame) throw new Error("could not decode the reference frame");
    const lensFov = isNum(meta?.fovH) ? +meta.fovH : null;
    const fovMax = lensFov ? Math.max(fovH, lensFov) * 1.06 : fovH * 1.3;
    const tOpts = { minMatch: 6, maxN: 50, patch: 13, search: 16, fovMax };
    const mkTracker = (frame) => initTracker(frame.data, evenW, TH, natW, natH, refPose, tOpts);
    const tracker = mkTracker(refFrame);
    if (tracker.features.length < 8) notes.push(`only ${tracker.features.length} background features on the marked frame — the solve may be weak`);

    const entry = (t2, r2) => ({ t: t2, az: round(r2.pose.az, 3), el: round(r2.pose.el, 3), roll: round(r2.pose.roll, 3), fov: round(r2.pose.fov, 2), k: round(r2.pose.k || 0, 5), n: r2.nInliers, h: r2.held ? 1 : 0, ...(r2.chained != null ? { c: 1 } : {}) });
    const path = [{ t: refT, az: round(refPose.az, 3), el: round(refPose.el, 3), roll: 0, fov: round(refPose.fov, 2), k: 0, n: tracker.features.length }];
    const fwd = times.filter((t) => t > refT + 1e-6);
    const bwd = times.filter((t) => t < refT - 1e-6).reverse();
    let done = 0;
    const total = (fwd.length + bwd.length) * 2 || 1;

    const walk = async (list, trk, frameOf) => {
      let prevT = refT, segFrom = path.length, segT = refT;
      for (const t of list) {
        const snap = () => ({ prevData: trk.prevData, prevG: trk.prevG, lastPose: trk.lastPose, features: trk.features, nextId: trk.nextId });
        const stepAt = async (tt) => { const f = await frameOf(tt); return f ? stepTracker(trk, f.data) : null; };
        const tryStep = async (tFrom, tTo, depth) => {
          const s0 = snap();
          let r = await stepAt(tTo);
          if (!r) return null;
          if (r.nInliers < 6 && depth > 0 && Math.abs(tTo - tFrom) > 0.09) {
            Object.assign(trk, s0);
            const tm = round((tFrom + tTo) / 2, 3);
            const rm = await tryStep(tFrom, tm, depth - 1);
            if (rm) path.push(entry(tm, rm));
            r = await tryStep(tm, tTo, depth - 1);
          }
          return r;
        };
        const r = await tryStep(prevT, t, 2);
        if (!r) break;
        path.push(entry(t, r));
        if (r.anchored) {
          const k = path.length - 1;
          path[k].anc = 1;
          if (r.drift && (Math.abs(r.drift.dAz) > 0.02 || Math.abs(r.drift.dEl) > 0.02 || Math.abs(r.drift.dRoll) > 0.05 || Math.abs(r.drift.dFov) > 0.1)) smearDrift(path, segFrom, k, segT, r.drift);
          segFrom = k + 1; segT = path[k].t;
        }
        prevT = t;
        done++; onP("stabilizing", 0.1 + 0.5 * (done / total));
      }
    };
    await walk(fwd, tracker, (t) => stream.frameAt(t));
    if (bwd.length) {
      /* streams only run forward — the backward leg decodes per-frame */
      const bTracker = mkTracker(refFrame);
      await walk(bwd, bTracker, (t) => decodeFrameAt(filePath, t, { maxW: evenW }));
    }
    stream.close();

    /* the app's exact post-processing: sort → bridge short held runs →
       despike → smooth (0.25) */
    path.sort((a, b) => a.t - b.t);
    const dropIdx = new Set();
    for (let i = 1; i < path.length - 1; i++) {
      if (!path[i].h) continue;
      let j = i; while (j < path.length && path[j].h) j++;
      if (j < path.length && (path[j].t - path[i - 1].t) <= 0.55) for (let k = i; k < j; k++) dropIdx.add(k);
      i = j - 1;
    }
    if (dropIdx.size) { const kept = path.filter((_, i) => !dropIdx.has(i)); path.length = 0; path.push(...kept); }
    const held = path.filter((p) => p.h).length;
    path.forEach((p) => delete p.h);
    despikePath(path);
    const pathRaw = path.map((p) => ({ ...p }));
    smoothPathAt(path, 0.25);
    if (held) notes.push(`${held} frame(s) held the previous pose (too few references) — expect drift there`);
    const chainN = path.filter((p) => p.c).length;
    if (chainN) notes.push(`${chainN} frame(s) tracked frame-to-frame (view left the marked reference frame)`);

    /* ---- object pass, guided by any caller waypoints ---- */
    onP("tracking the object", 0.65);
    const OW = seedFrame.w, OH = seedFrame.h;
    const seedPose = posePathAt(path, refT) || refPose;
    const sn2 = snapToObject(seedFrame.data, OW, OH, cx * osc, cy * osc, Math.min(16, Math.max(6, wpx0 * osc * 0.35)));
    const objSeed = { tx: sn2.x, ty: sn2.y, g: pixToDirK(sn2.x / osc, sn2.y / osc, natW, natH, seedPose.az, seedPose.el, seedPose.roll || 0, seedPose.fov, seedPose.k || 0) };
    const ae0 = dirToAzEl(objSeed.g);
    const objPath = [{ t: refT, az: round(ae0.az, 3), el: round(ae0.el, 3), q: 1 }];
    const wp = (opts.track || []).filter((p) => isNum(p.t) && isNum(p.fx) && isNum(p.fy) && +p.t >= spanT0 - 1e-6 && +p.t <= spanT1 + 1e-6)
      .map((p) => ({ t: +p.t, x: round(clampN(+p.fx, 0, 1) * natW, 1), y: round(clampN(+p.fy, 0, 1) * natH, 1) }))
      .sort((a, b) => a.t - b.t);
    source.track = wp.map((p) => ({ t: round(p.t, 3), x: p.x, y: p.y }));
    const guides = wp.map((p) => {
      const ps = posePathAt(path, p.t) || refPose;
      return { t: p.t, g: pixToDirK(p.x, p.y, natW, natH, ps.az, ps.el, ps.roll || 0, ps.fov, ps.k || 0) };
    });
    const guideAt = (tt) => {
      if (guides.length < 2) return null;
      if (tt <= guides[0].t - 1 || tt >= guides[guides.length - 1].t + 1) return null;
      let lo = 0, hi = guides.length - 1;
      if (tt <= guides[0].t) hi = 0; else if (tt >= guides[guides.length - 1].t) lo = guides.length - 1;
      else while (hi - lo > 1) { const m = (lo + hi) >> 1; if (guides[m].t <= tt) lo = m; else hi = m; }
      const a = guides[lo], b = guides[hi], u = hi === lo ? 0 : (tt - a.t) / Math.max(1e-9, b.t - a.t);
      return unit([a.g[0] + (b.g[0] - a.g[0]) * u, a.g[1] + (b.g[1] - a.g[1]) * u, a.g[2] + (b.g[2] - a.g[2]) * u]);
    };
    let objOk = 0, objMiss = 0;
    const objPx = wpx0 * osc;
    const runObj = async (list, frameOf) => {
      let prevO = seedFrame.data, st2 = { ...objSeed };
      for (const tt of list) {
        const f = await frameOf(tt);
        if (!f) break;
        const ps = posePathAt(path, tt) || refPose;
        const gd = guideAt(tt);
        const o = stepObject(prevO, f.data, OW, OH, st2, ps, { natW, natH, objPx, guide: gd, guideGate: clampN(ps.fov * 0.05, 0.6, 2), seed: { data: seedFrame.data, tx: objSeed.tx, ty: objSeed.ty } });
        prevO = f.data;
        st2 = { tx: o.tx, ty: o.ty, g: o.g, gPrev: o.gPrev };
        const ae = dirToAzEl(o.g);
        objPath.push({ t: tt, az: round(ae.az, 3), el: round(ae.el, 3), q: o.ok ? round(Math.max(0.01, o.ncc), 2) : (gd ? 0.25 : 0) });
        if (o.ok) objOk++; else objMiss++;
        done++; onP("tracking the object", 0.65 + 0.3 * (done / total));
      }
    };
    const oStream = openFrameStream(filePath, { fps: 12, maxW: OW, t0: spanT0, t1: spanT1, W: OW, H: OH });
    await oStream.frameAt(refT); // advance to the seed
    await runObj(fwd, (t) => oStream.frameAt(t));
    oStream.close();
    if (bwd.length) await runObj(bwd, (t) => decodeFrameAt(filePath, t, { maxW: OW }));

    objPath.sort((a, b) => a.t - b.t);
    smoothObjPath(objPath, { passes: 0 });
    const objRaw = objPath.map((p) => ({ ...p }));
    smoothObjPathAt(objPath, 0.25, { despiked: true });
    if (guides.length && objPath.length > 1) {
      const anchors = guides.map((g2) => { const ae2 = dirToAzEl(g2.g); return { t: g2.t, az: ae2.az, el: ae2.el }; });
      const snapped = snapDirsToAnchors(objPath, anchors);
      objPath.length = 0; objPath.push(...snapped);
    }
    const objGood = guides.length >= 2 || objOk >= Math.max(4, (objOk + objMiss) * 0.3);
    if (!objGood) notes.push(`object track lost (${objOk}/${objOk + objMiss} frames matched) — mark a few track waypoints and re-run, or track in the app`);
    source.posePath = path;
    source.posePathRaw = pathRaw;
    if (objGood) { source.objPath = objPath; source.objPathRaw = objRaw; }

    /* sight-line through the SOLVED pose at the marked frame */
    const P = posePathAt(path, refT) || refPose;
    const ae = dirOf(cx, cy, P);
    source.A.az = ae.az.toFixed(2); source.A.el = ae.el.toFixed(2);
  } else {
    /* ---- PHOTO: static pose, embedded normalized JPEG so the app shows it ---- */
    const ae = dirOf(cx, cy, poseAt0);
    source.A.az = ae.az.toFixed(2); source.A.el = ae.el.toFixed(2);
    onP("embedding the photo", 0.6);
    try {
      const full = await decodeFrameAt(filePath, 0, { maxW: 1600 });
      const jpg = await encodeJpeg(full.data, full.w, full.h, { quality: 3 });
      source.mediaJpeg = "data:image/jpeg;base64," + Buffer.from(jpg).toString("base64");
    } catch (e) { notes.push("photo embed failed: " + e.message); }
  }

  source.ingest = { v: 1, at: new Date().toISOString(), guessed, notes };

  /* ---- session + verdict + bundle ---- */
  onP("packing", 0.96);
  const est = { name: ctx.name || "", size: "", dist: "", speed: "" };
  const session = { phodar: 1, created: new Date().toISOString(), sources: [source], est };
  const sessionJson = JSON.stringify(session, null, 1);
  let verdict = null, summary = "";
  try { verdict = analyzeSession({ session }); summary = summarizeVerdict(verdict); } catch (e) { summary = "verdict failed: " + e.message; }

  const files = [{ name: strU8("sighting.phodar.json"), data: strU8(sessionJson) }];
  if (probe.kind === "video") {
    const ext = /webm/i.test(filePath) ? "webm" : /\.mov$/i.test(filePath) ? "mov" : "mp4";
    files.push({ name: strU8(`videos/observer-1-original.${ext}`), data: new Uint8Array(readFileSync(filePath)) });
  }
  const bundle = makeZip(files);
  const slug = (est.name || "sighting").toLowerCase().replace(/[^a-z0-9]+/g, "-").replace(/^-+|-+$/g, "") || "sighting";
  onP("done", 1);
  return { source, session, sessionJson, verdict, summary, bundle, bundleName: `${slug}-auto.zip`, guessed, notes };
}
