/* ============================================================
   PHODAR server — static host + historical ADS-B proxy.
   No dependencies on purpose (repo ethos: the hand-rolled parser
   is a feature). Serves dist/ and one API:

   GET /api/hist?lat=&lon=&nm=&t=<ms>&win=<min>
     → { ac: [...], src, sampleT, sliceT } — aircraft near (lat,lon)
       at time t, from the tar1090 globe_history archives
       (airplanes.live primary, adsbexchange fallback; both serve
       ~2+ years back AND today progressively, no key, no CORS —
       hence this proxy).

   Data path: 30-min binary heatmap slice (10–25 MB) → 30 s
   sub-slices, 16-byte entries {u32 hex, i32 lat×1e6, i32 lon×1e6,
   i16 alt/25ft, i16 gs×10kt}; file head is a 60×u32 table of
   sub-slice entry-offsets; each sub-slice opens with a timestamp
   marker (hex 0xe7f7c9d, lat/lon = ms>>32 / ms&0xffffffff). INFO
   entries carry the callsign in bytes 8..15. Format validated
   empirically against trace_full ground truth 2026-07-14.
   Nearby hexes are refined via traces/{xx}/trace_full_{hex}.json
   → exact interpolated state + registration + type designator.
   ============================================================ */

import http from "node:http";
import { readFile, stat } from "node:fs/promises";
import { createReadStream } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import zlib from "node:zlib";

const PORT = +(process.env.PORT || 8787);
const DIST = path.join(path.dirname(fileURLToPath(import.meta.url)), "..", "dist");
const BASES = [
  { host: "https://globe.airplanes.live", name: "airplanes.live history" },
  { host: "https://globe.adsbexchange.com", name: "adsbexchange history" },
];
const MARKER = 0x0e7f7c9d;
const FT = 0.3048, KT = 0.514444;

/* ---------- tiny LRU byte-cache (slices are 10–25 MB; keep few) ---------- */
class LRU {
  constructor(maxBytes) { this.max = maxBytes; this.size = 0; this.map = new Map(); }
  get(k) { const v = this.map.get(k); if (v) { this.map.delete(k); this.map.set(k, v); } return v; }
  set(k, v) {
    if (this.map.has(k)) { this.size -= this.map.get(k).length; this.map.delete(k); }
    this.map.set(k, v); this.size += v.length;
    for (const [kk, vv] of this.map) { if (this.size <= this.max) break; this.map.delete(kk); this.size -= vv.length; }
  }
}
const sliceCache = new LRU(80 * 1024 * 1024);
const traceCache = new LRU(30 * 1024 * 1024);

async function fetchBin(url, referer) {
  const r = await fetch(url, { headers: { Referer: referer + "/" }, signal: AbortSignal.timeout(30000) });
  if (!r.ok) throw Object.assign(new Error(`HTTP ${r.status}`), { status: r.status });
  return Buffer.from(await r.arrayBuffer());
}

async function getSlice(day, sliceIdx) {
  const p = `/globe_history/${day}/heatmap/${String(sliceIdx).padStart(2, "0")}.bin.ttf`;
  for (const b of BASES) {
    const key = b.host + p;
    const hit = sliceCache.get(key);
    if (hit) return { buf: hit, src: b.name };
    try {
      const buf = await fetchBin(b.host + p, b.host);
      sliceCache.set(key, buf);
      return { buf, src: b.name };
    } catch (e) { /* try next base */ }
  }
  return null;
}

/* decode one heatmap slice; return positions + callsigns within tMs±winMs and bbox */
function decodeSlice(buf, tMs, winMs, bbox) {
  const n = Math.floor(buf.length / 16);
  const idx = [];
  for (let i = 0; i < 60 && i < n; i++) idx.push(buf.readUInt32LE(i * 16));
  const byHex = new Map(); // hex -> {lat,lon,altM,gs,ts,callsign}
  for (let s = 0; s < idx.length; s++) {
    const from = idx[s], to = s < idx.length - 1 ? idx[s + 1] : n;
    if (from >= n) break;
    const mHex = buf.readUInt32LE(from * 16);
    if ((mHex & 0x0fffffff) !== MARKER) continue;
    const ts = buf.readUInt32LE(from * 16 + 4) * 4294967296 + buf.readUInt32LE(from * 16 + 8);
    if (Math.abs(ts - tMs) > winMs) continue;
    for (let i = from + 1; i < to; i++) {
      const rawHex = buf.readUInt32LE(i * 16);
      const hex = (rawHex & 0x00ffffff).toString(16).padStart(6, "0");
      const lat = buf.readInt32LE(i * 16 + 4);
      if (Math.abs(lat) > 90e6) { // INFO entry: callsign in bytes 8..15
        const cs = buf.subarray(i * 16 + 8, i * 16 + 16).toString("latin1").replace(/[^\x20-\x7e]/g, "").trim();
        const rec = byHex.get(hex);
        if (rec && cs) rec.callsign = rec.callsign || cs;
        else if (cs) byHex.set(hex, { callsign: cs });
        continue;
      }
      const la = lat / 1e6, lo = buf.readInt32LE(i * 16 + 8) / 1e6;
      if (la < bbox.la0 || la > bbox.la1 || lo < bbox.lo0 || lo > bbox.lo1) continue;
      const altRaw = buf.readInt16LE(i * 16 + 12);
      if (altRaw === -123) continue; // readsb ground sentinel — parked/taxiing can't be a sky sighting
      const rec = byHex.get(hex);
      const cand = {
        lat: la, lon: lo,
        altM: altRaw * 25 * FT,
        gs: (buf.readInt16LE(i * 16 + 14) / 10) * KT,
        ts, callsign: rec?.callsign,
      };
      if (!rec || rec.lat == null || Math.abs(cand.ts - tMs) < Math.abs(rec.ts - tMs)) byHex.set(hex, { ...rec, ...cand });
    }
  }
  return byHex;
}

/* refine one aircraft through its full-day trace: exact state at t + identity */
async function refineViaTrace(day, hex, tMs) {
  const p = `/globe_history/${day}/traces/${hex.slice(-2)}/trace_full_${hex}.json`;
  for (const b of BASES) {
    const key = b.host + p;
    let raw = traceCache.get(key);
    if (!raw) {
      try { raw = await fetchBin(b.host + p, b.host); traceCache.set(key, raw); }
      catch (e) { continue; }
    }
    try {
      const j = JSON.parse(raw.toString("utf8"));
      const t0 = tMs / 1000 - j.timestamp;
      const tr = j.trace || [];
      let lo = 0, hi = tr.length - 1;
      while (hi - lo > 1) { const m = (lo + hi) >> 1; (tr[m][0] < t0 ? lo = m : hi = m); }
      const a = tr[lo], bpt = tr[Math.min(hi, tr.length - 1)];
      if (!a) return null;
      const use = (p1, p2) => {
        const f = p2[0] > p1[0] ? Math.min(1, Math.max(0, (t0 - p1[0]) / (p2[0] - p1[0]))) : 0;
        const num = (x, y) => (typeof x === "number" && typeof y === "number") ? x + (y - x) * f : (typeof x === "number" ? x : y);
        return {
          lat: p1[1] + (p2[1] - p1[1]) * f, lon: p1[2] + (p2[2] - p1[2]) * f,
          altFt: num(p1[3], p2[3]), gsKt: num(p1[4], p2[4]), track: num(p1[5], p2[5]),
          dt: Math.min(Math.abs(p1[0] - t0), Math.abs(p2[0] - t0)),
        };
      };
      const st = use(a, bpt);
      if (st.dt > 300) return null; // trace has a >5 min hole at t — don't fake it
      let callsign = null;
      for (let i = lo; i >= 0 && i > lo - 40; i--) { const ao = tr[i][8]; if (ao && ao.flight) { callsign = ao.flight.trim(); break; } }
      /* trail: the aircraft's path ±4 min around t, thinned to ≥10 s spacing —
         the sky view draws these as faint sky-tracks near the sight-line */
      const trail = [];
      const TRAIL_S = 240, MIN_GAP = 10;
      let lastT = -1e9;
      for (let i = 0; i < tr.length; i++) {
        const p = tr[i];
        if (p[0] < t0 - TRAIL_S) continue;
        if (p[0] > t0 + TRAIL_S) break;
        if (p[0] - lastT < MIN_GAP) continue;
        if (typeof p[3] !== "number") continue; // skip ground/holes
        lastT = p[0];
        trail.push([+(p[0] - t0).toFixed(1), +p[1].toFixed(5), +p[2].toFixed(5), Math.round(p[3] * 0.3048)]);
      }
      /* guarantee a sample at the exact sighting instant — the chip is drawn
         there, and without it the chords visibly miss the chip */
      if (trail.length && typeof st.altFt === "number" && !trail.some((q) => Math.abs(q[0]) < 2)) {
        trail.push([0, +st.lat.toFixed(5), +st.lon.toFixed(5), Math.round(st.altFt * 0.3048)]);
        trail.sort((x, y) => x[0] - y[0]);
      }
      return {
        hex, reg: j.r || null, t: j.t || null, desc: j.desc || null, dbFlags: j.dbFlags,
        callsign, trail: trail.length > 1 ? trail : null,
        lat: st.lat, lon: st.lon,
        altM: typeof st.altFt === "number" ? st.altFt * FT : (st.altFt === "ground" ? 0 : null),
        ground: st.altFt === "ground",
        gs: typeof st.gsKt === "number" ? st.gsKt * KT : null,
        track: typeof st.track === "number" ? st.track : null,
        dt: st.dt,
      };
    } catch (e) { return null; }
  }
  return null;
}

async function apiHist(q, res) {
  const lat = +q.get("lat"), lon = +q.get("lon"), t = +q.get("t");
  const nm = Math.min(250, Math.max(5, +(q.get("nm") || 60)));
  const winMin = Math.min(30, Math.max(1, +(q.get("win") || 8)));
  if (!isFinite(lat) || !isFinite(lon) || !isFinite(t)) return json(res, 400, { error: "lat, lon, t required" });
  if (t > Date.now() - 60000) return json(res, 400, { error: "t is in the future — the archive lags a few minutes; use the live API" });

  const d = new Date(t);
  const day = `${d.getUTCFullYear()}/${String(d.getUTCMonth() + 1).padStart(2, "0")}/${String(d.getUTCDate()).padStart(2, "0")}`;
  const sod = d.getUTCHours() * 3600 + d.getUTCMinutes() * 60 + d.getUTCSeconds();
  const sliceIdx = Math.floor(sod / 1800);

  const degLat = (nm * 1852) / 111320, degLon = degLat / Math.max(0.2, Math.cos(lat * Math.PI / 180));
  const bbox = { la0: lat - degLat, la1: lat + degLat, lo0: lon - degLon, lo1: lon + degLon };

  /* pull the covering slice, plus the neighbor if the window crosses a boundary */
  const winMs = winMin * 60000;
  const need = [sliceIdx];
  const within = sod % 1800;
  if (within * 1000 < winMs && sliceIdx > 0) need.push(sliceIdx - 1);
  if ((1800 - within) * 1000 < winMs && sliceIdx < 47) need.push(sliceIdx + 1);

  const byHex = new Map(); let src = null;
  for (const si of need) {
    const got = await getSlice(day, si);
    if (!got) continue;
    src = src || got.src;
    for (const [hex, rec] of decodeSlice(got.buf, t, winMs, bbox)) {
      if (rec.lat == null) continue;
      const prev = byHex.get(hex);
      if (!prev || Math.abs(rec.ts - t) < Math.abs(prev.ts - t)) byHex.set(hex, rec);
    }
  }
  if (!src) return json(res, 404, { error: "no archive slice for that date — the history archives cover roughly the last two years (and today with a few minutes' lag)" });

  /* nearest N by ground distance → refine via traces (parallel, capped).
     Near an airport many picks turn out to be on the ground AT t (they were
     airborne within the window) and get dropped — pick deep enough to
     survive that. */
  const all = [...byHex.entries()].map(([hex, r]) => ({
    hex, ...r,
    dKm: Math.hypot((r.lat - lat) * 111.32, (r.lon - lon) * 111.32 * Math.cos(lat * Math.PI / 180)),
  })).sort((a, b) => a.dKm - b.dKm);
  const pick = all.slice(0, 32);
  const refined = [];
  const CONC = 6;
  for (let i = 0; i < pick.length; i += CONC) {
    const chunk = await Promise.all(pick.slice(i, i + CONC).map((p) => refineViaTrace(day, p.hex, t).catch(() => null)));
    chunk.forEach((r, k) => {
      const base = pick[i + k];
      refined.push(r || {
        hex: base.hex, callsign: base.callsign || null, reg: null, t: null, desc: null,
        lat: base.lat, lon: base.lon, altM: base.altM, gs: base.gs, track: null,
        ground: false, dt: Math.abs(base.ts - t) / 1000, coarse: true,
      });
    });
  }
  const ac = refined.filter((a) => a && !a.ground).map((a) => ({
    hex: a.hex, flight: a.callsign, reg: a.reg, t: a.t, desc: a.desc, category: null,
    lat: a.lat, lon: a.lon, altM: a.altM, gs: a.gs, track: a.track, seen: a.dt, ground: false, coarse: !!a.coarse,
    trail: a.trail || null, // [[dtSec, lat, lon, altM], ...] ±4 min around t
  }));
  json(res, 200, { ac, src, sampleT: t, nm, win: winMin, total: all.length });
}

/* ---------- static + plumbing ---------- */
const MIME = { ".html": "text/html", ".js": "text/javascript", ".css": "text/css", ".json": "application/json", ".png": "image/png", ".svg": "image/svg+xml", ".ico": "image/x-icon", ".webmanifest": "application/manifest+json" };
function json(res, code, obj) {
  const body = JSON.stringify(obj);
  res.writeHead(code, { "content-type": "application/json", "access-control-allow-origin": "*" });
  res.end(body);
}
/* Esri tile proxy — lets the browser composite the report's satellite basemap
   + map-data overlay onto a canvas WITHOUT a CORS taint (same origin here), so
   toDataURL succeeds and the tiles bake into the self-contained report.
   Layers: img = World Imagery, trans = roads, ref = place names/boundaries. */
const TILE_SVC = { img: "World_Imagery", trans: "Reference/World_Transportation", ref: "Reference/World_Boundaries_and_Places" };
async function apiTile(u, res) {
  const m = u.pathname.match(/^\/api\/tile\/(img|trans|ref)\/(\d{1,2})\/(\d{1,7})\/(\d{1,7})$/);
  if (!m) { res.writeHead(400); return res.end("bad tile"); }
  const [, layer, z, y, x] = m;
  if (+z > 21) { res.writeHead(400); return res.end("zoom"); }
  try {
    const r = await fetch(`https://server.arcgisonline.com/ArcGIS/rest/services/${TILE_SVC[layer]}/MapServer/tile/${z}/${y}/${x}`, { signal: AbortSignal.timeout(15000) });
    if (!r.ok) { res.writeHead(r.status, { "cache-control": "no-store" }); return res.end(); }
    const buf = Buffer.from(await r.arrayBuffer());
    res.writeHead(200, {
      "content-type": r.headers.get("content-type") || "image/jpeg",
      "access-control-allow-origin": "*",
      "cache-control": "public, max-age=604800",
    });
    res.end(buf);
  } catch (e) { res.writeHead(502, { "cache-control": "no-store" }); res.end("tile fetch failed"); }
}
/* rocket-launch correlator proxy — Launch Library 2 (no key; CORS not
   guaranteed browser-side, so proxy it same-origin). Trimmed to what the
   client needs. */
async function apiLaunches(q, res) {
  const net0 = q.get("net0"), net1 = q.get("net1");
  if (!net0 || !net1) return json(res, 400, { error: "net0/net1 required" });
  try {
    const url = `https://ll.thespacedevs.com/2.2.0/launch/?net__gte=${encodeURIComponent(net0)}&net__lte=${encodeURIComponent(net1)}&limit=40&ordering=net`;
    const r = await fetch(url, { headers: { "user-agent": "phodar/1 (sighting correlator)" }, signal: AbortSignal.timeout(20000) });
    if (!r.ok) return json(res, r.status === 429 ? 429 : 502, { error: `upstream ${r.status}` });
    const j = await r.json();
    const results = (j.results || []).map((x) => ({
      name: x.name, net: x.net,
      rocket: { configuration: { name: x?.rocket?.configuration?.name, full_name: x?.rocket?.configuration?.full_name } },
      mission: { name: x?.mission?.name },
      pad: { name: x?.pad?.name, latitude: x?.pad?.latitude, longitude: x?.pad?.longitude, location: { name: x?.pad?.location?.name } },
    }));
    return json(res, 200, { results });
  } catch (e) { return json(res, 502, { error: String(e.message || e) }); }
}
/* fireball correlator proxy — NASA CNEOS (no key). Returns the raw
   {fields, data} shape; the client parses it. */
async function apiFireballs(q, res) {
  const dmin = q.get("dmin"), dmax = q.get("dmax");
  try {
    const url = `https://ssd-api.jpl.nasa.gov/fireball.api?req-loc=true${dmin ? `&date-min=${encodeURIComponent(dmin)}` : ""}${dmax ? `&date-max=${encodeURIComponent(dmax)}` : ""}`;
    const r = await fetch(url, { signal: AbortSignal.timeout(20000) });
    if (!r.ok) return json(res, 502, { error: `upstream ${r.status}` });
    return json(res, 200, await r.json());
  } catch (e) { return json(res, 502, { error: String(e.message || e) }); }
}
/* named-peak proxy — OSM Overpass summits/volcanoes near the observer. CORS
   is unreliable on the Overpass mirrors, so proxy it. Two mirrors for
   resilience. */
async function apiPeaks(q, res) {
  const lat = +q.get("lat"), lon = +q.get("lon"), r = Math.min(80000, Math.max(1000, +q.get("r") || 40000));
  if (!isFinite(lat) || !isFinite(lon)) return json(res, 400, { error: "lat/lon required" });
  const ql = `[out:json][timeout:20];(node["natural"="peak"](around:${r},${lat},${lon});node["natural"="volcano"](around:${r},${lat},${lon}););out qt 400;`;
  const eps = ["https://overpass-api.de/api/interpreter", "https://overpass.kumi.systems/api/interpreter"];
  for (const ep of eps) {
    try {
      const rr = await fetch(ep, { method: "POST", headers: { "content-type": "application/x-www-form-urlencoded" }, body: "data=" + encodeURIComponent(ql), signal: AbortSignal.timeout(25000) });
      if (!rr.ok) continue;
      return json(res, 200, await rr.json());
    } catch (e) { /* try the next mirror */ }
  }
  return json(res, 502, { error: "overpass unreachable" });
}
const server = http.createServer(async (req, res) => {
  try {
    const u = new URL(req.url, "http://x");
    if (u.pathname === "/api/hist") return await apiHist(u.searchParams, res);
    if (u.pathname.startsWith("/api/tile/")) return await apiTile(u, res);
    if (u.pathname === "/api/launches") return await apiLaunches(u.searchParams, res);
    if (u.pathname === "/api/fireballs") return await apiFireballs(u.searchParams, res);
    if (u.pathname === "/api/peaks") return await apiPeaks(u.searchParams, res);
    if (u.pathname === "/api/health") return json(res, 200, { ok: true, cacheMB: Math.round(sliceCache.size / 1048576) });
    /* static from dist/ */
    let fp = path.normalize(path.join(DIST, decodeURIComponent(u.pathname)));
    if (!fp.startsWith(DIST)) { res.writeHead(403); return res.end(); }
    let st = await stat(fp).catch(() => null);
    let isAsset = u.pathname.startsWith("/assets/");
    if (!st || st.isDirectory()) {
      /* NEVER fall back to index.html for hashed assets: mid-deploy, a stale
         hash would get HTML served as JS — and the immutable cache header
         would brick that browser for a year. 404 lets it retry/reload. */
      if (isAsset) { res.writeHead(404, { "cache-control": "no-store" }); return res.end("not found"); }
      fp = path.join(DIST, "index.html"); st = await stat(fp).catch(() => null);
      isAsset = false;
    }
    if (!st) { res.writeHead(404, { "cache-control": "no-store" }); return res.end("not found"); }
    res.writeHead(200, {
      "content-type": MIME[path.extname(fp)] || "application/octet-stream",
      "content-length": st.size,
      "cache-control": isAsset ? "public, max-age=31536000, immutable" : "no-cache",
    });
    createReadStream(fp).pipe(res);
  } catch (e) {
    json(res, 500, { error: String(e?.message || e) });
  }
});
server.listen(PORT, () => console.log(`phodar server on :${PORT} (dist=${DIST})`));
