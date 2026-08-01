/* ============================================================
   HEADLESS ANALYSIS ENGINE — the full results pipeline with no UI.
   Input: a session's sources (exactly the shape the app's share
   .phodar.json carries — positions, marks, tracks, solved paths),
   plus optionally a drone flight-log CSV/SRT for ground-truth
   calibration. Output: one machine-readable verdict object with
   every solver's result and honest reasons for anything that
   didn't solve.

   This is the core of API access (server /api/analyze, CLI
   scripts/analyze.mjs) and the substrate a future agentic layer
   drives. It deliberately contains NO pixel work: media ingestion
   (auto shape fit / tracking from raw video) is a later phase —
   this engine consumes measurements and re-derives every
   conclusion from them, deterministically. Pure: no fetch, no DOM.
   ============================================================ */

import { analyze } from "../math/triangulate.js";
import { analyzeTracks, stereoVideo } from "../math/kinematics.js";
import { angSizeFromPoints, lensK } from "../math/projection.js";
import { isNum } from "../math/format.js";
import {
  parseFlightLog, syncLogTime, calibrationSummary, gradeCalibration,
  witnessClockCheck, DRONE_PRESETS,
} from "../checks/flightlog.js";

const round = (v, d = 2) => (isNum(v) ? +(+v).toFixed(d) : null);

function sourceSummary(s, i) {
  const pos = isNum(s.lat) && isNum(s.lon);
  const dir = isNum(s.A?.az) && isNum(s.A?.el);
  const missing = [];
  if (!pos) missing.push("position");
  if (!dir) missing.push("sky placement (committed sight-line)");
  return {
    name: s.name || `Observer ${i + 1}`,
    position: pos ? { lat: +s.lat, lon: +s.lon, alt: isNum(s.alt) ? +s.alt : null } : null,
    sightline: dir ? { az: +s.A.az, el: +s.A.el } : null,
    whenMs: isNum(s.whenMs) ? +s.whenMs : null,
    trackPts: (s.track || []).length,
    posePathPts: (s.posePath || []).length,
    objPathPts: (s.objPath || []).length,
    angSizeDeg: round(
      angSizeFromPoints(s.A?.p1, s.A?.p2, s.natW, s.natH, +s.fovH, lensK(s))
      ?? (isNum(s.A?.angManual) ? +s.A.angManual : null), 4),
    missing,
  };
}

export function analyzeSession(input = {}) {
  const sources = Array.isArray(input.sources) ? input.sources
    : Array.isArray(input.session?.sources) ? input.session.sources : [];
  const warnings = [];
  const verdict = {
    engine: "phodar-analyze/1",
    nSources: sources.length,
    sources: sources.map(sourceSummary),
    fix: null, trackStereo: null, videoStereo: null, flightLog: null,
    warnings,
  };
  if (!sources.length) { warnings.push("no sources supplied"); return verdict; }

  /* ---- Moment-A fix ---- */
  const fix = analyze(sources);
  if (fix.ok) {
    verdict.fix = {
      ok: true, rating: fix.rating,
      lat: round(fix.geoA.lat, 6), lon: round(fix.geoA.lon, 6),
      altAboveRefM: round(fix.solA.X[2], 1),
      baselineM: round(fix.baseline, 1), convergenceDeg: round(fix.conv, 2),
      rmsMissM: round(fix.solA.rmsMiss, 2), posErrM: round(fix.posErr, 1),
      rangesM: fix.perSource.map((p) => round(p.dist, 1)),
      sizeAvgM: round(fix.sizeAvg, 3),
      behind: !!fix.behind,
      motion: fix.motion && isNum(fix.motion.speed) ? {
        speedMs: round(fix.motion.speed, 2), headingDeg: round(fix.motion.heading, 1),
        vRateMs: round(fix.motion.vRate, 2),
      } : null,
    };
  } else {
    verdict.fix = { ok: false, validCount: fix.validCount ?? 0, reason: fix.parallel ? "sight-lines parallel" : "needs 2 observers with position + committed sight-line" };
    const gaps = verdict.sources.filter((s) => s.missing.length);
    if (gaps.length) warnings.push(...gaps.map((s) => `${s.name} is missing: ${s.missing.join(", ")}`));
  }

  /* ---- track stereo (waypoints/moments; visibility- and clock-aware) ---- */
  const trk = analyzeTracks(sources);
  if (trk.stereo && trk.stereo.k) {
    const st = trk.stereo;
    verdict.trackStereo = {
      solved: true, n: st.k.n, durS: round(st.k.dur, 1),
      avgMissM: round(st.avgMiss, 2),
      sharedDurS: round(st.sharedDur, 1), ignoredDurS: round(st.cutDur, 1),
      avgSpeedMs: round(st.k.avgSpeed, 2), peakSpeedMs: round(st.k.peakSpeed, 2),
      peakLoadG: round(st.k.peakLoad, 2), pathM: round(st.k.path, 1),
      clockSync: st.sync ? { applied: !!st.sync.applied, deltaS: st.sync.delta ?? null, rescued: !!st.sync.rescued, flatMinimum: !!st.sync.flat } : null,
      windowMs: st.window.map((t) => Math.round(t * 1000)),
    };
    if (st.sync?.applied) warnings.push(`witness clocks disagreed — second track shifted ${st.sync.delta} s by geometric sync`);
    if (st.cutDur > 0.5) warnings.push(`${round(st.cutDur, 1)} s of time overlap ignored where a witness had lost sight`);
  } else if (trk.stereo) {
    verdict.trackStereo = {
      solved: false,
      reason: trk.stereo.noShared ? "tracks overlap in time but never both see the object"
        : trk.stereo.overlapErr ? "track time windows don't overlap"
          : trk.stereo.sparse ? "too few triangulable instants" : "unsolved",
    };
  }

  /* ---- dense two-video stereo (needs solved objPaths) ---- */
  const sv = stereoVideo(sources);
  if (sv && sv.ok) {
    verdict.videoStereo = {
      solved: true, n: sv.n, offsetS: round(sv.offset, 2), syncConfidence: round(sv.syncConf, 2),
      meanMissM: round(sv.meanMiss, 2), baselineM: round(sv.baseline, 1), convergenceDeg: round(sv.conv, 1),
      sharedDurS: round(sv.sharedDur, 1), ignoredDurS: round(sv.cutDur, 1), qDropped: sv.qDropped,
      avgSpeedMs: round(sv.k?.avgSpeed, 2), peakSpeedMs: round(sv.k?.peakSpeed, 2),
    };
  }

  /* ---- flight-log ground truth (calibration flights) ---- */
  const logText = input.flightLogText ?? null;
  const stored = sources.map((s) => s.flightLog).find((f) => f && f.pts && f.pts.length);
  if (logText || stored) {
    const preset = DRONE_PRESETS.find((p) => p.id === (input.droneId || stored?.droneId)) || null;
    const spanM = isNum(input.spanM) ? +input.spanM : (isNum(stored?.spanM) ? +stored.spanM : preset?.spanM ?? null);
    const homeElevM = isNum(input.homeElevM) ? +input.homeElevM : (isNum(stored?.homeElevM) ? +stored.homeElevM : null);
    let pts = stored?.pts || null, meta = stored || null;
    if (logText) {
      const parsed = parseFlightLog(String(logText), input.flightLogName || "flightlog");
      if (!parsed.ok) verdict.flightLog = { ok: false, reason: parsed.error };
      else { pts = parsed.pts; meta = parsed; }
    }
    if (pts && !verdict.flightLog) {
      const valid = sources.filter((s) => isNum(s.lat) && isNum(s.lon) && isNum(s.A?.az) && isNum(s.A?.el));
      const best = valid.length ? syncLogTime(valid, pts, spanM, homeElevM) : null;
      const sum = best ? calibrationSummary({ sources: valid, fix: fix.ok ? fix : null, pts, tMs: best.tMs, spanM, homeElevM }) : null;
      const grade = gradeCalibration(sum);
      verdict.flightLog = {
        ok: true, samples: pts.length, spanS: meta ? round(((meta.t1Ms ?? 0) - (meta.t0Ms ?? 0)) / 1000, 1) : null,
        spanMUsed: spanM,
        calibration: sum ? {
          grade: grade?.overall ?? null, direction: grade?.dir ?? null, position: grade?.pos ?? null, size: grade?.size ?? null,
          matchedMs: sum.tMs, sepMaxDeg: round(sum.sepMax, 2),
          fixErrM: round(sum.fixCmp?.errM, 2), fixErrPct: round(sum.fixCmp?.errPct, 1),
          altErrM: round(sum.fixCmp?.errV, 2), sizeRatio: round(sum.fixCmp?.sizeRatio, 2),
          perWitness: sum.per.map((p) => ({ name: p.name, sepDeg: round(p.sep, 2), ownSepDeg: round(p.ownSep, 2), rangeM: round(p.rangeM, 1) })),
        } : null,
        clocks: (meta?.absTime !== false) ? valid.map((s) => {
          const ck = witnessClockCheck(s, pts, spanM, homeElevM);
          return ck ? { name: s.name, offsetS: round(ck.dtS, 1), sharp: !!ck.sharp, bestSepDeg: round(ck.bestSep, 2) } : null;
        }).filter(Boolean) : [],
      };
      for (const c of verdict.flightLog.clocks || [])
        if (c.sharp && Math.abs(c.offsetS) > 5) warnings.push(`${c.name}'s clock looks ~${c.offsetS} s off (sharp match at ${c.bestSepDeg}°)`);
    }
  }
  return verdict;
}
