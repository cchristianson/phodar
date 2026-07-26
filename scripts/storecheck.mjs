/* STORECHECK — boot the app with localStorage BLOCKED.

   Not every browser has site storage. Safari private browsing has historically
   thrown on write; Firefox with site data turned off, and some in-app webviews,
   throw on the property ACCESS itself. Unguarded that is not a degraded
   experience, it is a blank screen — the exception fires during module init,
   before anything renders, and the person never learns why.

   `src/storageShim.js` probes once and falls back to an in-memory map. This
   asserts that: the app boots, the shim round-trips, `window.storageVolatile`
   is set so the UI can warn that nothing will survive a reload, and nothing
   throws. (It earned its keep immediately — the first version of that probe
   touched window.localStorage once more OUTSIDE its own try/catch, which threw
   in exactly the case it existed to handle and left window.storage undefined.)

   Usage:
       npm run build && npm run preview -- --port 4173 &
       node scripts/storecheck.mjs http://localhost:4173
*/
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
  if (/Failed to load resource|ERR_TUNNEL|ERR_INTERNET|ERR_NAME_NOT_RESOLVED|net::/i.test(t)) return;
  fatal.push("console: " + t);
});

/* the strict-privacy case: the getter itself throws */
await page.addInitScript(() => {
  Object.defineProperty(window, "localStorage", {
    configurable: true,
    get() { throw new DOMException("The operation is insecure.", "SecurityError"); },
  });
});

await page.goto(URL, { waitUntil: "networkidle", timeout: 30000 });
await page.waitForTimeout(1200);

const screen = (await page.evaluate(() => document.body.innerText)).replace(/\n/g, " | ");
const volatile = await page.evaluate(() => window.storageVolatile);
const roundTrip = await page.evaluate(async () => {
  try {
    await window.storage.set("storecheck", "v");
    const g = await window.storage.get("storecheck");
    const l = await window.storage.list("");
    return g.value + "/" + l.keys.length;
  } catch (e) { return "THREW: " + (e && e.message); }
});

/* and it must still be usable, not just non-blank */
for (const l of ["New sighting", "Start"]) {
  const b = page.locator(`button:has-text("${l}")`).first();
  if (await b.count()) { await b.click().catch(() => { }); break; }
}
await page.waitForTimeout(900);
const wizard = (await page.evaluate(() => document.body.innerText)).slice(0, 90).replace(/\n/g, " | ");

const booted = screen.trim().length > 0;
const warned = /blocking site storage/i.test(screen);
console.log("boots with localStorage blocked :", booted ? "yes" : "NO — blank screen");
console.log("  window.storageVolatile        :", volatile);
console.log("  shim round-trip (want v/1)    :", roundTrip);
console.log("  warns the user                :", warned ? "yes" : "NO");
console.log("  reaches the wizard            :", wizard);
console.log(fatal.length ? "FATAL:\n" + fatal.join("\n") : "no uncaught errors");

await browser.close();
const ok = booted && volatile === true && roundTrip === "v/1" && warned && !fatal.length;
if (!ok) { console.log("STORECHECK-FAIL"); process.exit(1); }
console.log("STORECHECK-PASS");
