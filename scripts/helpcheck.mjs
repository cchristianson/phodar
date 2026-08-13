/* HELP COVERAGE CHECK — the in-app "?" manual (HELP_SECTIONS) is the only
   documentation a user gets, and it drifts silently: a feature ships, the help
   doesn't mention it, and nothing fails. This closes that loop.

   It extracts every distinctive control glyph that appears on a BUTTON in the
   app (buttons are how features are reached) and asserts each one is mentioned
   somewhere in HELP_SECTIONS. A glyph that is deliberately not worth a manual
   entry — a close ✕, a nudge arrow, a zoom +/− — must be listed in STRUCTURAL
   below WITH a reason, so adding one is a decision rather than an oversight.

   Run: npm run helpcheck
*/
import fs from "node:fs";

const SRC = "src/phodar.jsx";
const src = fs.readFileSync(SRC, "utf8");
const s = src.indexOf("const HELP_SECTIONS = [");
const e = src.indexOf("\n];", s) + 3;
if (s < 0 || e < 3) { console.error("helpcheck: could not find HELP_SECTIONS in " + SRC); process.exit(1); }
const help = src.slice(s, e);
const app = src.slice(0, s) + src.slice(e);

/* Glyphs that are navigation or micro-adjustment furniture, not features. Each
   needs a reason — that is the point of the list. */
const STRUCTURAL = {
  "←": "nudge arrow (fix-frames / rotation fine-tune)",
  "→": "nudge arrow + 'next' affordance on step buttons",
  "↑": "nudge arrow",
  "↓": "nudge arrow",
  "‹": "back chevron, documented as '‹ Back'",
  "🏠": "home — the wizard's last two steps step BACK one page on ‹, so this is their unconditional way out to the sighting list; navigation, not a feature",
  "▸": "'Open ▸' disclosure on the observer row",
  "▾": "dropdown caret",
  "▼": "dropdown caret",
  "⌃": "collapse caret (documented in the sky-view tips)",
  "⌄": "collapse caret (documented in the sky-view tips)",
  "✕": "close / remove — used on every panel; the meaningful ones (✕ remove shape, ✕ clear) are documented by name",
  "✓": "confirm — always paired with the action it confirms",
  "＋": "add, always paired ('＋ Add object', '+ Ref')",
  "−": "zoom out / decrement",
  "🗑": "delete the selected item (cam ref)",
  "⏹": "stop recording, paired with 🎬 Record",
  "⟲": "roll-left nudge (documented as the pair '⟲ ⟳')",
  "↺": "reset, always paired with what it resets",
  "⇱": "fit-to-frame",
  "✦": "used by star-align entries, which are documented by full name",
};

/* every literal button label in the app (React expressions are skipped — those
   are dynamic labels whose glyphs show up in a literal somewhere too) */
const labels = new Set();
for (const m of app.matchAll(/<button\b[^>]*>\s*([^<>{}\n]{1,60}?)\s*<\/button>/g)) labels.add(m[1].trim());
for (const m of app.matchAll(/>\s*([^<>{}\n]{1,60}?)\s*<\/button>/g)) labels.add(m[1].trim());

/* pull the distinctive glyphs out of those labels */
const GLYPH = /[\u2190-\u21FF\u2300-\u27BF\u2B00-\u2BFF\uFF0B\u{1F300}-\u{1FAFF}]/u;
const seen = new Map();                    // glyph → an example label
for (const l of labels) {
  for (const ch of [...l]) {
    if (!GLYPH.test(ch)) continue;
    if (!seen.has(ch)) seen.set(ch, l);
  }
}

const missing = [];
for (const [g, example] of [...seen].sort()) {
  if (STRUCTURAL[g]) continue;
  if (help.includes(g)) continue;
  missing.push({ g, example });
}

console.log(`helpcheck: ${seen.size} control glyphs on buttons, ${Object.keys(STRUCTURAL).length} structural, ${missing.length} undocumented`);
for (const { g, example } of missing) console.log(`  MISSING  ${g}   (e.g. button "${example}")`);
if (missing.length) {
  console.log(`\nEach glyph above reaches a feature the in-app manual never mentions.`);
  console.log(`Add an entry to HELP_SECTIONS in ${SRC}, or — if it is furniture — list it in STRUCTURAL in this script with a reason.`);
  process.exit(1);
}

/* Second half: named features whose help entry must exist, keyed on the code
   that implements them. These have no distinctive glyph of their own, so the
   glyph sweep can't see them. */
const NAMED = [
  ["trimOn", /Trimming the clip/, "the ✂ clip trim"],
  ["saveToRoll", /Save to camera roll/, "saving a clip to the camera roll"],
  ["clearStab", /✕ clear/, "clearing the stabilization"],
  ["preStab", /↶ Undo/, "undoing a stabilize run"],
  ["sensorPath", /Record with motion data/, "in-app capture with the attitude log"],
  ["posePath", /Stabilize video/, "video stabilization"],
  ["objPath", /auto-tracks the MARKED OBJECT/, "object auto-tracking"],
  ["poseFixes", /Fix frames/, "manual per-frame pose correction"],
  ["horiz3d", /▲ 3D \/ ▤ profile/, "the 3D terrain vista on step 2"],
  ["setPeek", /📷 hold/, "hold-to-compare the photo against the terrain preview"],
  ["autoStarAlign", /Auto star align/, "automatic plate solving"],
  ["matchSkyline", /Snap to ridges/, "terrain skyline snapping"],
  ["FlightLogCheck", /flight-log check/i, "the drone flight-log calibration check"],
  ["slugName", /Sighting name/, "naming the sighting (report title + export filenames)"],
  ["viewOnly", /View only/, "view-only review mode (the master Edit/View toggle)"],
  ["trackQuality", /Track quality/, "the trajectory's track-quality rating"],
  ["camHeavy", /upper bound/, "the 'g is an upper bound' caveat on heavily-stabilized clips"],
  ["chained", /frame-to-frame/, "frame-to-frame locking when the view leaves the reference frame"],
  ["softMin", /cloud/, "tracking a cloud-only sky"],
  ["goReport", /‹ always steps back/, "the ‹ / 🏠 navigation on the last two steps"],
  ["fetchAircraftAt", /sky-track/i, "aircraft sky-tracks drawn on the dome"],
  ["predictedRoadDirs", /🛣 roads/, "the roads overlay (true-perspective ribbons)"],
  ["copyAiPrompt", /Bring your own AI/, "the bring-your-own-AI card + copyable prompt"],
  ["APPLE_INSTALL", /Install Phodar as an app/, "the PWA install hint"],
  ["importFromLink", /Fill from a report link/, "the report-link import"],
  ["metaHint", /keeping the metadata/, "the keep-the-metadata how-to on the capture step"],
  ["gest(", /DESKTOP/, "the desktop (mouse) gesture wording"],
  ["scanFileAuthenticity", /File authenticity/, "the authenticity checks (upload forensics + physics consistency)"],
  ["poleShadow", /⚑ Shadow check/, "the sun-shadow flagpole gadget on the sky view"],
  ["shadowTimes", /🕐 dial/, "the sundial-inversion dial (shadow direction → implied time)"],
  ["close-subject", /close-subject tells/, "the EXIF close-subject tells (focus distance, flash return)"],
  ["TerrainLosCheck", /Terrain line-of-sight/, "the terrain line-of-sight validation of the fix"],
  ["roadCrossings", /Vehicle-light/, "the vehicle-light road-crossing check"],
  ["fetchMasts", /tower checks|mast/, "the tower/mast strobe check"],
  ["glintDeg", /flare/, "satellite flare-geometry notes"],
  ["trackMatch", /AT THE SAME TIMES/, "the ADS-B trajectory (track-time) match"],
  ["rankSondes", /Weather-balloon check \(radiosonde\)/, "the radiosonde launch-schedule + received-track check"],
  ["fetchAirspace", /Military airspace check/, "the FAA special-use-airspace (MOA) check"],
];
const bad = [];
for (const [needle, re, what] of NAMED) {
  if (!app.includes(needle)) continue;             // feature not present — nothing to document
  if (!re.test(help)) bad.push(what);
}
if (bad.length) {
  console.log("\nImplemented but undocumented:");
  for (const w of bad) console.log(`  MISSING  ${w}`);
  process.exit(1);
}

/* every shape in the picker must appear in the shape list the help prints */
const shapes = fs.readFileSync("src/shapes.js", "utf8");
const list = [...shapes.matchAll(/\{\s*k:\s*"([a-z]+)",\s*label:\s*"([^"]+)"/g)].map((m) => m[2]);
const noShape = list.filter((lab) => !help.includes(lab));
if (noShape.length) {
  console.log(`\nShapes offered in the picker but missing from the help's shape list: ${noShape.join(", ")}`);
  process.exit(1);
}
console.log(`helpcheck: all ${list.length} shapes listed, ${NAMED.length} named features documented`);
console.log("HELPCHECK-PASS");
