/* SKY-VIEW SMOKE TEST — drives the built app through the wizard until
   SkyAimer actually MOUNTS, and fails on any uncaught page error.

   Why this exists: a `const` declared below the effect that reads it shipped
   a temporal-dead-zone crash ("Cannot access 'X' before initialization") that
   blanked the whole sky view. A home-screen-only smoke test passed it, because
   the home screen never mounts SkyAimer — the biggest component in the app and
   the one every field session lives in. Anything that renders only after a
   photo + position exist needs to be exercised here.

   Usage (playwright must be available; it is NOT a project dependency, so this
   is deliberately outside `npm test`):
       npm run build && npm run preview -- --port 4173 &
       node scripts/skycheck.mjs http://localhost:4173
   Exits non-zero on a page error or if the sky view never mounts.

   Network calls (tiles, DEM, /api) are expected to fail in a sandbox — only
   uncaught JS errors count. */
import { chromium } from "playwright";

const URL = process.argv[2] || "http://localhost:4173";
const browser = await chromium.launch(
  process.env.PLAYWRIGHT_BROWSERS_PATH ? { executablePath: "/opt/pw-browsers/chromium" } : {}
).catch(() => chromium.launch());
const page = await (await browser.newContext({ viewport: { width: 420, height: 900 } })).newPage();

const fatal = [];
page.on("pageerror", (e) => fatal.push("pageerror: " + e.message));
page.on("console", (m) => {
  if (m.type() !== "error") return;
  const t = m.text();
  /* blocked network in CI/sandbox is not an app fault */
  if (/Failed to load resource|ERR_TUNNEL|ERR_INTERNET|ERR_NAME_NOT_RESOLVED|net::/i.test(t)) return;
  fatal.push("console: " + t);
});

const text = () => page.evaluate(() => document.body.innerText);
await page.goto(URL, { waitUntil: "networkidle", timeout: 30000 });
await page.waitForTimeout(800);

/* leave the home screen — the wizard (and every mount below) starts here */
for (const label of ["New sighting", "New Sighting", "Start"]) {
  const b = page.locator(`button:has-text("${label}")`).first();
  if (await b.count()) { await b.click().catch(() => { }); break; }
}
await page.waitForTimeout(900);

/* a synthetic sky/ridge photo so the wizard can advance */
const jpg = Buffer.from(await page.evaluate(() => {
  const c = document.createElement("canvas"); c.width = 1200; c.height = 900;
  const x = c.getContext("2d");
  const g = x.createLinearGradient(0, 0, 0, 900);
  g.addColorStop(0, "#5aa6e0"); g.addColorStop(0.62, "#bcd8ee");
  g.addColorStop(0.63, "#2b3a2a"); g.addColorStop(1, "#16210f");
  x.fillStyle = g; x.fillRect(0, 0, 1200, 900);
  x.fillStyle = "#eeeeee"; x.beginPath(); x.arc(700, 300, 9, 0, 7); x.fill();
  return c.toDataURL("image/jpeg", 0.9).split(",")[1];
}), "base64");
const file = page.locator('input[type="file"]').first();
if (await file.count()) await file.setInputFiles({ name: "smoke.jpg", mimeType: "image/jpeg", buffer: jpg }).catch(() => { });
await page.waitForTimeout(2500);

/* the position step blocks until coordinates exist — type them like a user */
const fillCoords = async () => {
  const ins = page.locator('input[inputmode="decimal"], input[type="number"], input[type="text"]');
  const n = await ins.count();
  const vals = ["42.3265", "-122.8756"];
  let k = 0;
  for (let i = 0; i < n && k < 2; i++) {
    const el = ins.nth(i);
    const ph = ((await el.getAttribute("placeholder")) || "").toLowerCase();
    if (/search|name|address/.test(ph)) continue;
    if (!(await el.boundingBox().catch(() => null))) continue;
    await el.fill(vals[k]).catch(() => { });
    await el.dispatchEvent("input").catch(() => { });
    k++;
  }
  await page.waitForTimeout(500);
};

/* SkyAimer is mounted once its tool row is on screen */
const inSky = async () => { const t = await text(); return /Trajectory/.test(t) && /Compare/.test(t); };

let opened = false;
for (let i = 0; i < 8 && !opened; i++) {
  if (await inSky()) { opened = true; break; }
  if (/YOUR POSITION/i.test(await text())) await fillCoords();
  let clicked = false;
  for (const label of ["Continue", "Next", "Aim", "Sky"]) {
    const b = page.locator(`button:has-text("${label}")`).first();
    if (await b.count()) { await b.click().catch(() => { }); clicked = true; break; }
  }
  if (!clicked) break;
  await page.waitForTimeout(1400);
}
opened = await inSky();
await page.waitForTimeout(1200);

/* exercise the sky view's own modes — each mounts different subtrees */
for (const mode of ["Place", "Trajectory", "Size", "Compare"]) {
  const b = page.locator(`button:has-text("${mode}")`).first();
  if (await b.count()) { await b.click().catch(() => { }); await page.waitForTimeout(700); }
}

console.log("sky view mounted:", opened);
if (!opened) console.log("last screen:", (await text()).slice(0, 200).replace(/\n/g, " | "));
console.log(fatal.length ? "FATAL:\n" + fatal.join("\n") : "no uncaught errors");
await browser.close();
if (!opened || fatal.length) { console.log("SKYCHECK-FAIL"); process.exit(1); }
console.log("SKYCHECK-PASS");
