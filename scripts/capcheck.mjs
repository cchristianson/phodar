/* CAPCHECK — drive the in-app SENSOR CAPTURE end to end with stubbed hardware.

   The capture path can't be exercised by hand in CI and can't be reasoned
   about reliably either: it spans a permission dance, getUserMedia,
   MediaRecorder callbacks bound once at record time, DeviceOrientation
   events, and an async geolocation fix — and a failure anywhere in that
   chain surfaces only as "the next step is empty". This stubs the hardware
   before any app script runs, records a clip, and then asserts the sighting
   actually came out with a position, a pose and a motion log.

   Usage:
       npm run build && npm run preview -- --port 4173 &
       node scripts/capcheck.mjs http://localhost:4173
*/
import { chromium } from "playwright";

const URL = process.argv[2] || "http://localhost:4173";
const LAT = 42.1638, LON = -123.648;

const browser = await chromium.launch(
  process.env.PLAYWRIGHT_BROWSERS_PATH ? { executablePath: "/opt/pw-browsers/chromium" } : {}
).catch(() => chromium.launch());
const ctx = await browser.newContext({
  viewport: { width: 420, height: 900 },
  /* REAL geolocation, granted — navigator.geolocation is a read-only getter,
     so a page-side stub silently does nothing and the app sees the real API */
  permissions: ["geolocation"],
  geolocation: { latitude: LAT, longitude: LON, accuracy: 8 },
});
const page = await ctx.newPage();

const fatal = [];
page.on("pageerror", (e) => fatal.push("pageerror: " + e.message));
page.on("console", (m) => {
  if (m.type() !== "error") return;
  const t = m.text();
  if (/Failed to load resource|ERR_TUNNEL|ERR_INTERNET|ERR_NAME_NOT_RESOLVED|net::/i.test(t)) return;
  fatal.push("console: " + t);
});

page.on("console", (m) => { if (/CAPCHECK/.test(m.text())) console.log("  page> " + m.text()); });

/* stub the phone BEFORE the app loads */
await page.addInitScript(({ lat, lon }) => {
  /* surface swallowed async failures — an async MediaRecorder.onstop that
     throws produces an unhandled rejection and nothing else */
  window.addEventListener("unhandledrejection", (e) => console.log("CAPCHECK unhandledrejection: " + (e.reason && (e.reason.stack || e.reason.message) || e.reason)));
  window.addEventListener("error", (e) => console.log("CAPCHECK error: " + (e.message || "")));

  /* camera: a moving canvas so MediaRecorder has real frames to encode */
  const cv = document.createElement("canvas"); cv.width = 640; cv.height = 480;
  const cx = cv.getContext("2d");
  let f = 0;
  setInterval(() => {
    f++;
    cx.fillStyle = "#123"; cx.fillRect(0, 0, 640, 480);
    cx.fillStyle = "#cde"; cx.fillRect((f * 7) % 600, 100, 40, 40);
  }, 33);
  navigator.mediaDevices = navigator.mediaDevices || {};
  navigator.mediaDevices.getUserMedia = async () => cv.captureStream(30);

  /* motion permission gates (iOS-only APIs) */
  window.DeviceMotionEvent = window.DeviceMotionEvent || function () { };
  window.DeviceOrientationEvent = window.DeviceOrientationEvent || function () { };
  window.DeviceMotionEvent.requestPermission = async () => "granted";
  window.DeviceOrientationEvent.requestPermission = async () => "granted";

  /* a phone held near-level, panning slowly — gravity mostly on -z */
  let t0 = performance.now();
  setInterval(() => {
    const s = (performance.now() - t0) / 1000;
    const head = (30 + s * 6) % 360;
    const om = new Event("deviceorientation");
    Object.defineProperty(om, "webkitCompassHeading", { value: head });
    Object.defineProperty(om, "webkitCompassAccuracy", { value: 8 });
    Object.defineProperty(om, "alpha", { value: 360 - head });
    Object.defineProperty(om, "beta", { value: 80 });
    Object.defineProperty(om, "gamma", { value: 0 });
    window.dispatchEvent(om);
    const mm = new Event("devicemotion");
    Object.defineProperty(mm, "accelerationIncludingGravity", { value: { x: 0.2, y: -9.5, z: -1.6 } });
    window.dispatchEvent(mm);
  }, 40);

}, { lat: LAT, lon: LON });

const text = () => page.evaluate(() => document.body.innerText);
const buttons = () => page.evaluate(() => [...document.querySelectorAll("button")].map((b) => b.innerText.trim().replace(/\s+/g, " ")).filter(Boolean));
const click = async (label) => {
  const b = page.locator(`button:has-text("${label}")`).first();
  if (!(await b.count())) return false;
  await b.click().catch(() => { });
  return true;
};
const trace = async (tag) => console.log(`[${tag}] buttons: ${JSON.stringify((await buttons()).slice(0, 12))}`);

await page.goto(URL, { waitUntil: "networkidle", timeout: 30000 });
await page.waitForTimeout(700);
for (const l of ["New sighting", "Start"]) if (await click(l)) break;
await page.waitForTimeout(700);

if (!(await click("📷 Capture"))) { console.log("could not find the 📷 Capture button"); await browser.close(); process.exit(1); }
await page.waitForTimeout(600);
if (!(await click("Start"))) { console.log("capture opened but no ▶ Start button"); await browser.close(); process.exit(1); }
await page.waitForTimeout(2000);                 // permissions + camera + first GPS fix
await trace("after Start");

if (!(await click("Record with motion data"))) { console.log("no 🎬 Record button — is ENABLE_CAPTURE on?"); await browser.close(); process.exit(1); }
await page.waitForTimeout(2500);                 // record a couple of seconds
await trace("recording");
if (!(await click("Stop"))) { console.log("no ⏹ Stop button while recording"); await browser.close(); process.exit(1); }
await page.waitForTimeout(7000);                 // stop → last-chance GPS (bounded ~4 s) → ingest
await trace("after Stop");
console.log("screen text:", (await text()).slice(0, 400).replace(/\n/g, " | "));

/* what actually landed on the sighting */
const state = await page.evaluate(() => {
  let best = null;
  for (let i = 0; i < localStorage.length; i++) {
    const k = localStorage.key(i);
    let v; try { v = JSON.parse(localStorage.getItem(k)); } catch (e) { continue; }
    const srcs = v && (v.sources || (v.state && v.state.sources));
    if (Array.isArray(srcs) && srcs.length) best = srcs[0];
  }
  return best ? {
    mediaKind: best.mediaKind, lat: best.lat, lon: best.lon,
    hasAim: !!best.mediaAim, aim: best.mediaAim,
    sensorPath: Array.isArray(best.sensorPath) ? best.sensorPath.length : 0,
    natW: best.natW, natH: best.natH,
  } : null;
});
const screen = (await text()).slice(0, 140).replace(/\n/g, " | ");

console.log("sighting after an instrumented recording:");
console.log("  " + JSON.stringify(state));
console.log("  screen: " + screen);

/* autosave strips media handles, so assert on what MUST persist: the
   position, the measured pose, the motion log and the decoded dimensions */
const ok = state && state.lat && state.lon && state.hasAim && state.sensorPath > 4 && state.natW > 0;
console.log(fatal.length ? "FATAL:\n" + fatal.join("\n") : "no uncaught errors");
await browser.close();
if (!ok || fatal.length) { console.log("CAPCHECK-FAIL"); process.exit(1); }
console.log("CAPCHECK-PASS");
