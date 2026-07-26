/* CAPCHECK — drive the in-app SENSOR CAPTURE end to end with stubbed hardware.

   The capture path can't be exercised by hand in CI and can't be reasoned
   about reliably either: it spans a permission dance, getUserMedia,
   MediaRecorder callbacks bound once at record time, DeviceOrientation
   events, and an async geolocation fix — and a failure anywhere in that
   chain surfaces only as "the next step is empty". This stubs the hardware
   before any app script runs, records a clip, and then asserts the sighting
   actually came out with a position, a pose and a motion log.

   It runs TWICE, because there are two entirely different sensor stories:

     ios      — webkitCompassHeading (already tilt-compensated to the camera)
                and an accelerometer pointing ALONG the pull. The path the app
                was field-calibrated against.
     android  — no webkitCompassHeading at all; a compass-referenced alpha on
                `deviceorientationabsolute`, and an accelerometer using the
                W3C convention, which is the exact OPPOSITE sign. Feeding that
                to the iOS math gives a negated tilt and a bearing that is
                wrong by however far the phone is tilted, so this run asserts
                the pose against the value the orientation angles imply.

   Usage:
       npm run build && npm run preview -- --port 4173 &
       node scripts/capcheck.mjs http://localhost:4173
*/
import { chromium } from "playwright";
import { poseFromOrientation, upFromOrientation } from "../src/capture/pose.js";

const URL = process.argv[2] || "http://localhost:4173";
const LAT = 42.1638, LON = -123.648;
/* the constant hold the android stub reports: leaned back 10°, tilted 5°,
   rotated 30° — nothing symmetric, so a sign or axis slip shows up */
const A_ALPHA = 30, A_BETA = 100, A_GAMMA = 5;

const browser = await chromium.launch(
  process.env.PLAYWRIGHT_BROWSERS_PATH ? { executablePath: "/opt/pw-browsers/chromium" } : {}
).catch(() => chromium.launch());

async function run(platform) {
  console.log(`\n=== ${platform} ===`);
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
  await page.addInitScript(({ plat, al, be, ga }) => {
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

    if (plat === "ios") {
      /* a phone held near-level, panning slowly — gravity mostly on −z, which
         is iOS's convention: the vector points ALONG the pull */
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
    } else {
      /* Android: the app only listens for the absolute event when the property
         exists, exactly as Chrome advertises it */
      if (!("ondeviceorientationabsolute" in window)) window.ondeviceorientationabsolute = null;
      const D = Math.PI / 180, b = be * D, g = ga * D;
      /* world up in the device frame, from the same angles the events report */
      const up = { x: -Math.cos(b) * Math.sin(g), y: Math.sin(b), z: Math.cos(b) * Math.cos(g) };
      setInterval(() => {
        const om = new Event("deviceorientationabsolute");
        Object.defineProperty(om, "absolute", { value: true });
        Object.defineProperty(om, "alpha", { value: al });
        Object.defineProperty(om, "beta", { value: be });
        Object.defineProperty(om, "gamma", { value: ga });
        window.dispatchEvent(om);
        /* W3C / Chrome convention: PROPER acceleration, i.e. +up, the exact
           opposite of what iOS reports for the same hold */
        const mm = new Event("devicemotion");
        Object.defineProperty(mm, "accelerationIncludingGravity", { value: { x: up.x * 9.80665, y: up.y * 9.80665, z: up.z * 9.80665 } });
        window.dispatchEvent(mm);
      }, 40);
    }
  }, { plat: platform, al: A_ALPHA, be: A_BETA, ga: A_GAMMA });

  const text = () => page.evaluate(() => document.body.innerText);
  const buttons = () => page.evaluate(() => [...document.querySelectorAll("button")].map((b) => b.innerText.trim().replace(/\s+/g, " ")).filter(Boolean));
  const click = async (label) => {
    const b = page.locator(`button:has-text("${label}")`).first();
    if (!(await b.count())) return false;
    await b.click().catch(() => { });
    return true;
  };
  const trace = async (tag) => console.log(`[${tag}] buttons: ${JSON.stringify((await buttons()).slice(0, 12))}`);
  const bail = async (msg) => { console.log("  " + msg); await ctx.close(); return false; };

  await page.goto(URL, { waitUntil: "networkidle", timeout: 30000 });
  await page.waitForTimeout(700);
  for (const l of ["New sighting", "Start"]) if (await click(l)) break;
  await page.waitForTimeout(700);

  if (!(await click("📷 Capture"))) return await bail("could not find the 📷 Capture button");
  await page.waitForTimeout(600);
  if (!(await click("Start"))) return await bail("capture opened but no ▶ Start button");
  await page.waitForTimeout(2000);                 // permissions + camera + first GPS fix
  await trace("after Start");

  if (!(await click("Record with motion data"))) return await bail("no 🎬 Record button — is ENABLE_CAPTURE on?");
  await page.waitForTimeout(2500);                 // record a couple of seconds
  if (!(await click("Stop"))) return await bail("no ⏹ Stop button while recording");
  await page.waitForTimeout(7000);                 // stop → last-chance GPS (bounded ~4 s) → ingest
  await trace("after Stop");

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
      raw: best.capture && best.capture.raw,
    } : null;
  });
  const screen = (await text()).slice(0, 140).replace(/\n/g, " | ");

  console.log("sighting after an instrumented recording:");
  console.log("  " + JSON.stringify(state));
  console.log("  screen: " + screen);

  /* autosave strips media handles, so assert on what MUST persist: the
     position, the measured pose, the motion log and the decoded dimensions */
  let ok = !!(state && state.lat && state.lon && state.hasAim && state.sensorPath > 4 && state.natW > 0);
  if (!ok) console.log("  FAIL: the sighting is missing position / pose / motion log");

  if (ok && platform === "android") {
    /* the whole point of the android run: the pose must be what the
       ORIENTATION ANGLES imply, not what the iOS math would make of them */
    const want = poseFromOrientation(A_ALPHA, A_BETA, A_GAMMA);
    const d = (a, b) => Math.abs(((a - b + 540) % 360) - 180);
    const dAz = d(state.aim.az, want.az), dEl = Math.abs(state.aim.el - want.el), dRoll = Math.abs((state.aim.roll || 0) - want.roll);
    console.log(`  pose want ${JSON.stringify(want)} got ${JSON.stringify(state.aim)}`);
    if (dAz > 1 || dEl > 1 || dRoll > 1) { ok = false; console.log(`  FAIL: pose off by az ${dAz.toFixed(2)}° el ${dEl.toFixed(2)}° roll ${dRoll.toFixed(2)}°`); }
    /* and the tilt must not be the NEGATED one the uncorrected accelerometer
       would have produced — the specific bug this run exists to catch */
    if (ok && Math.abs(state.aim.el + want.el) < 0.5 && Math.abs(want.el) > 2) { ok = false; console.log("  FAIL: elevation came out negated (gravity sign not corrected)"); }
    if (ok && state.raw && state.raw.mode !== "orient") { ok = false; console.log(`  FAIL: expected the orientation path, got mode="${state.raw.mode}"`); }
  }
  if (ok && platform === "ios" && state.raw && state.raw.mode !== "ios") { ok = false; console.log(`  FAIL: expected the iOS path, got mode="${state.raw.mode}"`); }

  if (fatal.length) { ok = false; console.log("  FATAL:\n" + fatal.join("\n")); }
  await ctx.close();
  console.log("  " + (ok ? "ok" : "FAILED"));
  return ok;
}

const results = [];
for (const p of ["ios", "android"]) results.push(await run(p));
await browser.close();
if (results.some((r) => !r)) { console.log("\nCAPCHECK-FAIL"); process.exit(1); }
console.log("\nCAPCHECK-PASS");
