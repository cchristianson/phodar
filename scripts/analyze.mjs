// Headless phodar analysis: run the full results pipeline on a session file.
//   node scripts/analyze.mjs sighting.phodar.json [--log flight.csv]
//        [--span 0.202] [--drone mini1] [--home-elev 485] [--out verdict.json]
// The session file is what the app's 💾 Share file exports (or a bundle's
// sighting.phodar.json). Prints a human summary; --out writes the full JSON.
import fs from "node:fs";
import { analyzeSession } from "../src/analyze/engine.js";

const args = process.argv.slice(2);
const file = args.find((a) => !a.startsWith("--"));
if (!file) { console.error("usage: node scripts/analyze.mjs <session.phodar.json> [--log flight.csv] [--span m] [--drone id] [--home-elev m] [--out verdict.json]"); process.exit(2); }
const opt = (name) => { const i = args.indexOf(name); return i >= 0 ? args[i + 1] : null; };

const session = JSON.parse(fs.readFileSync(file, "utf8"));
const logPath = opt("--log");
const verdict = analyzeSession({
  session,
  flightLogText: logPath ? fs.readFileSync(logPath, "utf8") : null,
  flightLogName: logPath || undefined,
  spanM: opt("--span") != null ? +opt("--span") : undefined,
  droneId: opt("--drone") || undefined,
  homeElevM: opt("--home-elev") != null ? +opt("--home-elev") : undefined,
});

const mph = (ms) => (ms == null ? "—" : (ms * 2.23694).toFixed(1) + " mph");
console.log(`sources: ${verdict.nSources}`);
for (const s of verdict.sources)
  console.log(`  ${s.name}: ${s.position ? "pos ✓" : "pos ✗"} ${s.sightline ? "sight-line ✓" : "sight-line ✗"} · ${s.trackPts} track pts${s.missing.length ? " · MISSING " + s.missing.join(" + ") : ""}`);
if (verdict.fix?.ok) console.log(`fix: ${verdict.fix.rating} · ${verdict.fix.lat}, ${verdict.fix.lon} · ${verdict.fix.altAboveRefM} m up · miss ${verdict.fix.rmsMissM} m${verdict.fix.sizeAvgM ? ` · size ${verdict.fix.sizeAvgM} m` : ""}`);
else console.log(`fix: not solved (${verdict.fix?.reason})`);
if (verdict.trackStereo?.solved) console.log(`track stereo: ${verdict.trackStereo.n} pts · miss ${verdict.trackStereo.avgMissM} m · avg ${mph(verdict.trackStereo.avgSpeedMs)} peak ${mph(verdict.trackStereo.peakSpeedMs)}${verdict.trackStereo.clockSync?.applied ? ` · clock sync ${verdict.trackStereo.clockSync.deltaS} s` : ""}`);
else if (verdict.trackStereo) console.log(`track stereo: ${verdict.trackStereo.reason}`);
if (verdict.videoStereo?.solved) console.log(`video stereo: ${verdict.videoStereo.n} pts · miss ${verdict.videoStereo.meanMissM} m · avg ${mph(verdict.videoStereo.avgSpeedMs)}`);
if (verdict.flightLog?.ok) {
  const c = verdict.flightLog.calibration;
  console.log(`flight log: ${verdict.flightLog.samples} samples · calibration ${c?.grade ?? "—"}${c ? ` · sep ${c.sepMaxDeg}° · fix err ${c.fixErrM} m (${c.fixErrPct}%) · size ×${c.sizeRatio}` : ""}`);
  for (const ck of verdict.flightLog.clocks) if (ck.sharp && Math.abs(ck.offsetS) > 5) console.log(`  ⏱ ${ck.name}: clock ~${ck.offsetS} s off (sharp, ${ck.bestSepDeg}°)`);
} else if (verdict.flightLog) console.log(`flight log: ${verdict.flightLog.reason}`);
for (const w of verdict.warnings) console.log(`⚠ ${w}`);

const out = opt("--out");
if (out) { fs.writeFileSync(out, JSON.stringify(verdict, null, 1)); console.log(`wrote ${out}`); }
