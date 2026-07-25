/* METACHECK — run the APP'S OWN EXIF/QuickTime parser over a candidate file
   and report exactly what Phodar will see, before you waste a session on it.

   Most photos on the web have EXIF stripped (every social network does it),
   and plenty that keep EXIF still lack the two fields that matter most here:
   GPS position and the compass bearing (GPSImgDirection). This says which of
   them are present, and prints ready-made Overpass queries so you can check
   the same spot has 3D building data before shooting the test.

   Usage:
       node scripts/metacheck.mjs path/to/IMG_1234.jpg
       node scripts/metacheck.mjs path/to/IMG_1234.mov
*/
import fs from "fs";
import path from "path";
import { parseMediaMeta } from "../src/exif.js";

const file = process.argv[2];
if (!file) { console.error("usage: node scripts/metacheck.mjs <image-or-video>"); process.exit(2); }
if (!fs.existsSync(file)) { console.error("no such file: " + file); process.exit(2); }

const buf = fs.readFileSync(file);
const isVideo = /\.(mov|mp4|m4v)$/i.test(path.extname(file));
const m = parseMediaMeta(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength), isVideo) || {};

const yes = (v) => (v ? "✓" : "✗");
const line = (k, v) => console.log(`  ${k.padEnd(22)} ${v}`);

console.log(`\n${path.basename(file)}  (${(buf.length / 1048576).toFixed(1)} MB, ${isVideo ? "video" : "image"})`);
console.log("\nWhat Phodar reads:");
line("camera", m.model || "— (no Make/Model tag)");
line("GPS position", m.lat != null ? `${m.lat}, ${m.lon}` : "— MISSING");
line("GPS altitude", m.alt != null ? `${m.alt} m` : "—");
line("timestamp", m.timeMs ? new Date(m.timeMs).toISOString().replace("T", " ").slice(0, 19) : "— MISSING");
line("compass bearing", m.az != null ? `${m.az}° (${m.azRef})` : "— MISSING");
line("35 mm focal → FOV", m.fovH != null ? `${m.f35} mm → ${m.fovH}° horizontal` : "— MISSING");

/* the three things that decide whether a test is worth running */
const okPos = m.lat != null, okAz = m.az != null, okFov = m.fovH != null;
console.log("\nReady for a buildings-overlay test?");
line(`${yes(okPos)} position`, okPos ? "step 2 auto-fills; buildings/terrain load for this spot" : "you'd have to place yourself on the map by hand");
line(`${yes(okAz)} bearing`, okAz ? "the sky view opens aimed the right way" : "you'd have to find the right azimuth by hand — the slow part");
line(`${yes(okFov)} lens FOV`, okFov ? "photo scale is correct before you touch anything" : "falls back to a 68° guess; pinch to calibrate");

if (!okPos) {
  console.log("\n⚠ No GPS — this file can't test the buildings overlay meaningfully.");
  console.log("  (Social networks and most re-uploads strip EXIF. AirDrop/Files keeps it;");
  console.log("   iMessage and Mail 'reduce size' do not.)");
  process.exit(1);
}

/* second half of the requirement: does OSM actually have 3D buildings there?
   Phodar's 🏙 layer needs `height` or `building:levels` — a footprint alone
   gets an assumed height, which is NOT a fair alignment test. */
const { lat, lon } = m;
const d = 0.004;                       // ~450 m box
const bbox = `${(lat - d).toFixed(5)},${(lon - d).toFixed(5)},${(lat + d).toFixed(5)},${(lon + d).toFixed(5)}`;
const q = `[out:json][timeout:25];(way["building"]["height"](${bbox});way["building"]["building:levels"](${bbox}););out count;`;
console.log("\nCheck OSM has REAL heights here (not just footprints):");
console.log("  " + `https://overpass-api.de/api/interpreter?data=${encodeURIComponent(q)}`);
console.log("  → a count of 0 means the 🏙 boxes would all use assumed heights.");
console.log("\nSee the same spot in 3D (sanity-check what should be visible):");
console.log(`  https://demo.f4map.com/#lat=${lat}&lon=${lon}&zoom=17`);
console.log(`  https://www.openstreetmap.org/#map=18/${lat}/${lon}`);
console.log("");
