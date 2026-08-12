/* Ingest SERVING layer: media store, keyframes, async measurement jobs, and
   the best-effort report-page scraper. The heavy solve runs in a FORKED
   worker (src/ingest/worker.mjs) — the NCC math is minutes of blocking CPU,
   and the same process serves the app, so it must never run on the server's
   event loop. State is temp files + an in-memory registry on the single dyno
   (same honesty as the rest of the server: a redeploy forgets in-flight jobs;
   artifacts are re-creatable from the media, which the caller still has). */

import { mkdirSync, writeFileSync, readFileSync, createWriteStream, existsSync, rmSync, readdirSync, statSync } from "node:fs";
import { tmpdir } from "node:os";
import path from "node:path";
import { fork } from "node:child_process";
import { fileURLToPath } from "node:url";
import { probeMedia, decodeFrameAt, encodeJpeg } from "./media.mjs";
import { parseMediaMeta } from "../exif.js";
import { snapToObject } from "../video/postrack.js";

const ROOT = path.join(tmpdir(), "phodar-ingest");
const MAX_MEDIA = 300 * 1024 * 1024;
const id6 = () => Math.random().toString(36).slice(2, 8);
const isNum = (v) => v != null && v !== "" && Number.isFinite(+v);

/* ---------------- media store ---------------- */

const extOf = (name, ctype) => {
  const m = /\.(mp4|mov|m4v|webm|avi|mkv|jpe?g|png|webp|heic|heif|gif|bmp|tiff?)(\?|#|$)/i.exec(name || "");
  if (m) return m[1].toLowerCase().replace("jpeg", "jpg");
  const c = String(ctype || "").toLowerCase();
  if (c.includes("mp4")) return "mp4";
  if (c.includes("quicktime")) return "mov";
  if (c.includes("webm")) return "webm";
  if (c.includes("png")) return "png";
  if (c.includes("jpeg") || c.includes("jpg")) return "jpg";
  if (c.includes("heic") || c.includes("heif")) return "heic";
  return "bin";
};

/* private-network guard for URL fetches (SSRF): the ingest fetcher reaches
   the public internet on the caller's behalf, not the dyno's own network.
   PHODAR_INGEST_ALLOW_LOCAL=1 lifts it for local testing. */
const privateHost = (host) => {
  const h = String(host || "").toLowerCase();
  return h === "localhost" || h === "::1" || /^127\.|^10\.|^192\.168\.|^169\.254\./.test(h)
    || /^172\.(1[6-9]|2\d|3[01])\./.test(h) || h.endsWith(".local") || h.endsWith(".internal");
};

export async function ingestFromUrl(url, { filename } = {}) {
  let u;
  try { u = new URL(url); } catch (e) { throw new Error("not a valid URL"); }
  if (!/^https?:$/.test(u.protocol)) throw new Error("only http(s) URLs");
  if (privateHost(u.hostname) && process.env.PHODAR_INGEST_ALLOW_LOCAL !== "1") throw new Error("refusing to fetch private-network hosts");
  const ctl = new AbortController();
  const t = setTimeout(() => ctl.abort(), 180000);
  let resp;
  try { resp = await fetch(url, { signal: ctl.signal, redirect: "follow", headers: { "user-agent": "phodar-ingest/1" } }); }
  finally { clearTimeout(t); }
  if (!resp.ok) throw new Error(`fetch failed: ${resp.status} ${resp.statusText}`);
  const len = +resp.headers.get("content-length") || 0;
  if (len > MAX_MEDIA) throw new Error(`media is ${(len / 1e6).toFixed(0)} MB — cap is ${MAX_MEDIA / 1e6} MB`);
  const mediaId = "m" + id6() + id6();
  const dir = path.join(ROOT, "media-" + mediaId);
  mkdirSync(dir, { recursive: true });
  const ext = extOf(filename || u.pathname, resp.headers.get("content-type"));
  const filePath = path.join(dir, "media." + ext);
  const chunks = [];
  let size = 0;
  for await (const c of resp.body) {
    size += c.length;
    if (size > MAX_MEDIA) { rmSync(dir, { recursive: true, force: true }); throw new Error(`media exceeds the ${MAX_MEDIA / 1e6} MB cap`); }
    chunks.push(c);
  }
  writeFileSync(filePath, Buffer.concat(chunks));
  return finishIngest(mediaId, dir, filePath, filename || u.pathname.split("/").pop() || "media");
}

/* raw upload path (curl a file at it) — the request stream is spooled
   straight to disk so a 100 MB clip never sits in RAM */
export function ingestFromStream(req, { filename } = {}) {
  return new Promise((resolve, reject) => {
    const mediaId = "m" + id6() + id6();
    const dir = path.join(ROOT, "media-" + mediaId);
    mkdirSync(dir, { recursive: true });
    const ext = extOf(filename, req.headers["content-type"]);
    const filePath = path.join(dir, "media." + ext);
    const out = createWriteStream(filePath);
    let size = 0;
    req.on("data", (c) => {
      size += c.length;
      if (size > MAX_MEDIA) { req.destroy(); out.destroy(); rmSync(dir, { recursive: true, force: true }); reject(new Error(`media exceeds the ${MAX_MEDIA / 1e6} MB cap`)); }
    });
    req.on("error", (e) => { out.destroy(); reject(e); });
    req.pipe(out);
    out.on("finish", () => finishIngest(mediaId, dir, filePath, filename || "upload").then(resolve, reject));
  });
}

async function finishIngest(mediaId, dir, filePath, name) {
  let probe;
  try { probe = await probeMedia(filePath); }
  catch (e) { rmSync(dir, { recursive: true, force: true }); throw e; }
  let meta = null;
  try {
    const buf = readFileSync(filePath);
    meta = parseMediaMeta(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.length), probe.kind === "video") || null;
  } catch (e) { }
  writeFileSync(path.join(dir, "info.json"), JSON.stringify({ mediaId, name, filePath, probe, meta, at: Date.now() }));
  /* keyframes for the caller's eyes: 1 for a photo, 5 spread for a video */
  const times = probe.kind === "video"
    ? [0.02, 0.25, 0.5, 0.75, 0.98].map((f) => +(f * Math.max(0.05, probe.durS - 0.1)).toFixed(2))
    : [0];
  const keyframes = [];
  for (const t of times) {
    try {
      const f = await decodeFrameAt(filePath, t, { maxW: 512 });
      keyframes.push({ t, jpeg: await encodeJpeg(f.data, f.w, f.h, { quality: 6 }) });
    } catch (e) { }
  }
  return { mediaId, name, probe, meta, keyframes };
}

export function mediaInfo(mediaId) {
  const p = path.join(ROOT, "media-" + String(mediaId).replace(/[^a-z0-9]/gi, ""), "info.json");
  if (!existsSync(p)) return null;
  try { return JSON.parse(readFileSync(p, "utf8")); } catch (e) { return null; }
}

/* zoomed look at a spot (and a snap refine) so the caller can confirm the
   object before committing a mark */
export async function inspectFrame(mediaId, { t = 0, fx, fy, zoomW = 240 } = {}) {
  const info = mediaInfo(mediaId);
  if (!info) throw new Error("unknown mediaId (media expires after ~2 h)");
  const full = await decodeFrameAt(info.filePath, +t || 0, { maxW: Math.min(1600, info.probe.w) });
  if (!isNum(fx) || !isNum(fy)) {
    return { jpeg: await encodeJpeg(full.data, full.w, full.h, { quality: 5, maxW: 768 }), refined: null };
  }
  const px = Math.max(0, Math.min(1, +fx)) * full.w, py = Math.max(0, Math.min(1, +fy)) * full.h;
  const sn = snapToObject(full.data, full.w, full.h, px, py, 14);
  /* crop around the refined point, upscaled ×3 so a 10 px object is legible */
  const half = Math.round(zoomW / 2);
  const x0 = Math.round(Math.max(0, Math.min(full.w - zoomW, sn.x - half)));
  const y0 = Math.round(Math.max(0, Math.min(full.h - zoomW, sn.y - half)));
  const cw = Math.min(zoomW, full.w - x0), ch = Math.min(zoomW, full.h - y0);
  const crop = new Uint8ClampedArray(cw * ch * 4);
  for (let y = 0; y < ch; y++) crop.set(full.data.subarray(((y0 + y) * full.w + x0) * 4, ((y0 + y) * full.w + x0 + cw) * 4), y * cw * 4);
  /* draw a small crosshair at the refined point so the confirmation is visual */
  const mx = Math.round(sn.x - x0), my = Math.round(sn.y - y0);
  for (let d = -8; d <= 8; d++) {
    if (Math.abs(d) < 3) continue;
    for (const [xx, yy] of [[mx + d, my], [mx, my + d]]) {
      if (xx >= 0 && yy >= 0 && xx < cw && yy < ch) { const i = (yy * cw + xx) * 4; crop[i] = 245; crop[i + 1] = 169; crop[i + 2] = 63; crop[i + 3] = 255; }
    }
  }
  const scale = 3;
  const up = new Uint8ClampedArray(cw * scale * ch * scale * 4);
  for (let y = 0; y < ch * scale; y++) for (let x = 0; x < cw * scale; x++) {
    const si = ((y / scale | 0) * cw + (x / scale | 0)) * 4, di = (y * cw * scale + x) * 4;
    up[di] = crop[si]; up[di + 1] = crop[si + 1]; up[di + 2] = crop[si + 2]; up[di + 3] = 255;
  }
  return {
    jpeg: await encodeJpeg(up, cw * scale, ch * scale, { quality: 4 }),
    refined: { fx: +(sn.x / full.w).toFixed(4), fy: +(sn.y / full.h).toFixed(4) },
  };
}

/* ---------------- measurement jobs (forked worker) ---------------- */

const jobs = new Map(); // id → { state, stage, frac, error, dir, startedMs, child }
let running = 0;
const queue = [];
const WORKER = path.join(path.dirname(fileURLToPath(import.meta.url)), "worker.mjs");

export function startJob(mediaId, opts) {
  const info = mediaInfo(mediaId);
  if (!info) throw new Error("unknown mediaId (media expires after ~2 h — re-ingest)");
  const jobId = "j" + id6() + id6();
  const dir = path.join(ROOT, "job-" + jobId);
  mkdirSync(dir, { recursive: true });
  writeFileSync(path.join(dir, "job.json"), JSON.stringify({ filePath: info.filePath, opts }));
  const job = { state: "queued", stage: "queued", frac: 0, error: null, dir, startedMs: Date.now(), child: null };
  jobs.set(jobId, job);
  queue.push(jobId);
  pump();
  return jobId;
}

function pump() {
  if (running >= 1 || !queue.length) return; // one at a time — the dyno also serves the app
  const jobId = queue.shift();
  const job = jobs.get(jobId);
  if (!job) return pump();
  running++;
  job.state = "running"; job.stage = "starting";
  const child = fork(WORKER, [job.dir], { stdio: ["ignore", "ignore", "pipe", "ipc"] });
  job.child = child;
  const errTail = [];
  child.stderr?.on("data", (c) => { if (errTail.length < 20) errTail.push(c); });
  child.on("message", (m) => { if (m && m.stage) { job.stage = m.stage; job.frac = m.frac || 0; } });
  const kill = setTimeout(() => { try { child.kill("SIGKILL"); } catch (e) { } }, 15 * 60 * 1000);
  child.on("exit", (code) => {
    clearTimeout(kill);
    running--; job.child = null;
    if (code === 0 && existsSync(path.join(job.dir, "result.json"))) { job.state = "done"; job.frac = 1; job.stage = "done"; }
    else {
      job.state = "error";
      const werr = existsSync(path.join(job.dir, "error.txt")) ? readFileSync(path.join(job.dir, "error.txt"), "utf8") : "";
      job.error = (werr || Buffer.concat(errTail).toString().slice(-400) || `worker exited ${code}`).trim();
    }
    pump();
  });
}

export function jobStatus(jobId) {
  const job = jobs.get(String(jobId));
  if (!job) return null;
  const out = { jobId, state: job.state, stage: job.stage, frac: +(job.frac || 0).toFixed(2) };
  if (job.state === "error") out.error = job.error;
  if (job.state === "done") {
    try { Object.assign(out, JSON.parse(readFileSync(path.join(job.dir, "result.json"), "utf8"))); } catch (e) { }
  }
  return out;
}

export function jobFile(jobId, which) {
  const job = jobs.get(String(jobId));
  if (!job || job.state !== "done") return null;
  const p = path.join(job.dir, which === "bundle" ? "bundle.zip" : "session.json");
  return existsSync(p) ? p : null;
}

/* ---------------- housekeeping ---------------- */

export function gcSweep(maxAgeMs = 2 * 3600 * 1000) {
  try {
    if (!existsSync(ROOT)) return;
    const now = Date.now();
    for (const d of readdirSync(ROOT)) {
      const p = path.join(ROOT, d);
      try { if (now - statSync(p).mtimeMs > maxAgeMs) rmSync(p, { recursive: true, force: true }); } catch (e) { }
    }
    for (const [id, j] of jobs) if (now - j.startedMs > maxAgeMs && !j.child) jobs.delete(id);
  } catch (e) { }
}

/* ---------------- report-page scrape (best effort, written blind) ----------
   Pulls the machine-readable bones out of a sighting-report page (og:/
   twitter: metas, JSON-LD, media links, datetime/geo hints) so the AI can
   combine them with its own reading of the page. This was written WITHOUT
   access to ufosighting.report (no network in the dev sandbox) — it leans on
   standards, not that site's markup, and says what it found rather than
   pretending to understand the page. */
export async function fetchReport(url) {
  let u;
  try { u = new URL(url); } catch (e) { throw new Error("not a valid URL"); }
  if (!/^https?:$/.test(u.protocol)) throw new Error("only http(s) URLs");
  if (privateHost(u.hostname) && process.env.PHODAR_INGEST_ALLOW_LOCAL !== "1") throw new Error("refusing to fetch private-network hosts");
  const ctl = new AbortController();
  const t = setTimeout(() => ctl.abort(), 30000);
  let resp;
  try { resp = await fetch(url, { signal: ctl.signal, redirect: "follow", headers: { "user-agent": "phodar-ingest/1", accept: "text/html,application/json" } }); }
  finally { clearTimeout(t); }
  if (!resp.ok) throw new Error(`fetch failed: ${resp.status}`);
  const ctype = String(resp.headers.get("content-type") || "");
  const text = (await resp.text()).slice(0, 3 * 1024 * 1024);
  if (ctype.includes("json")) return { url, kind: "json", json: text.slice(0, 64 * 1024) };
  const abs = (href) => { try { return new URL(href, url).href; } catch (e) { return null; } };
  const metas = {};
  for (const m of text.matchAll(/<meta[^>]+(?:property|name)=["']((?:og|twitter|article)[:.][^"']+)["'][^>]*content=["']([^"']*)["']/gi)) metas[m[1].toLowerCase()] = m[2];
  for (const m of text.matchAll(/<meta[^>]+content=["']([^"']*)["'][^>]*(?:property|name)=["']((?:og|twitter|article)[:.][^"']+)["']/gi)) metas[m[2].toLowerCase()] = m[1];
  const jsonLd = [...text.matchAll(/<script[^>]*type=["']application\/ld\+json["'][^>]*>([\s\S]*?)<\/script>/gi)].map((m) => m[1].trim().slice(0, 4096)).slice(0, 4);
  const mediaUrls = new Set();
  for (const k of ["og:video", "og:video:url", "og:video:secure_url", "og:image", "og:image:url", "twitter:image", "twitter:player:stream"]) if (metas[k]) { const a = abs(metas[k]); if (a) mediaUrls.add(a); }
  for (const m of text.matchAll(/<(?:video|source|img|a)[^>]+(?:src|href)=["']([^"']+\.(?:mp4|mov|webm|m4v|jpe?g|png|heic))(\?[^"']*)?["']/gi)) { const a = abs(m[1] + (m[2] || "")); if (a) mediaUrls.add(a); }
  const title = metas["og:title"] || (/<title[^>]*>([\s\S]*?)<\/title>/i.exec(text) || [])[1] || "";
  const datetimes = [...new Set([...text.matchAll(/datetime=["']([^"']+)["']|(\d{4}-\d{2}-\d{2}[T ]\d{2}:\d{2}(?::\d{2})?(?:Z|[+-]\d{2}:?\d{2})?)/g)].map((m) => m[1] || m[2]).filter(Boolean))].slice(0, 8);
  const geo = [...new Set([...text.matchAll(/["'(]?(-?\d{1,2}\.\d{3,8})["')]?\s*[,;]\s*["'(]?(-?\d{1,3}\.\d{3,8})["')]?/g)].map((m) => `${m[1]},${m[2]}`))].slice(0, 6);
  const bodyText = text.replace(/<script[\s\S]*?<\/script>/gi, " ").replace(/<style[\s\S]*?<\/style>/gi, " ").replace(/<[^>]+>/g, " ").replace(/\s+/g, " ").trim().slice(0, 4000);
  return {
    url, kind: "html", title: title.trim().slice(0, 300),
    description: (metas["og:description"] || metas["twitter:description"] || "").slice(0, 600),
    mediaUrls: [...mediaUrls].slice(0, 12), jsonLd, datetimes, geoCandidates: geo, textExcerpt: bodyText,
  };
}
