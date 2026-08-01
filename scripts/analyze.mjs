// Headless phodar analysis: run the full results pipeline on a session file.
//   node scripts/analyze.mjs sighting.phodar.json [--log flight.csv]
//        [--span 0.202] [--drone mini1] [--home-elev 485] [--out verdict.json]
// The session file is what the app's 💾 Share file exports (or a bundle's
// sighting.phodar.json). Prints a human summary; --out writes the full JSON.
import fs from "node:fs";
import { analyzeSession, summarizeVerdict } from "../src/analyze/engine.js";

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

for (const line of summarizeVerdict(verdict)) console.log(line);

const out = opt("--out");
if (out) { fs.writeFileSync(out, JSON.stringify(verdict, null, 1)); console.log(`wrote ${out}`); }
