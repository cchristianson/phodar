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
  const lat = coord(q, "lat", 90), lon = coord(q, "lon", 180), t = +q.get("t");
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
/* A MISSING search param coerces to 0, not NaN — so `+q.get("lat")` turned an
   omitted coordinate into a perfectly finite 0,0 and every handler happily
   forwarded a query about the Gulf of Guinea to its upstream. Parse coordinates
   through this instead: absent, unparseable or out-of-range all yield NaN, which
   the existing isFinite guards reject as a 400 before anything leaves the box. */
const coord = (q, key, lim) => {
  const raw = q.get(key);
  if (raw == null || raw === "") return NaN;
  const v = Number(raw);
  return Math.abs(v) <= lim ? v : NaN;
};
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
   is unreliable on the Overpass mirrors, so proxy it. The mirrors are RACED in
   parallel (first success wins) — querying them one-by-one could exceed the
   client's timeout when a mirror is slow/queuing ("Fetch is aborted"). Results
   are cached (peaks don't move) so repeats are instant. */
/* nearby airports/aerodromes (OSM Overpass) → context for the ADS-B check
   ("nearest field 6 km NW — expect approach/departure traffic"). Same
   CORS-unreliable reason as /api/peaks: proxy + race mirrors + cache. */
const airportsCache = new Map();
async function apiAirports(q, res) {
  const lat = coord(q, "lat", 90), lon = coord(q, "lon", 180), r = Math.min(80000, Math.max(2000, +q.get("r") || 40000));
  if (!isFinite(lat) || !isFinite(lon)) return json(res, 400, { error: "lat/lon required" });
  const key = `${lat.toFixed(3)},${lon.toFixed(3)},${r}`;
  const hit = airportsCache.get(key);
  if (hit && Date.now() - hit.t < 24 * 3600 * 1000) return json(res, 200, hit.body);
  const la = lat.toFixed(5), lo = lon.toFixed(5);
  const ql = `[out:json][timeout:20];(node["aeroway"="aerodrome"](around:${r},${la},${lo});way["aeroway"="aerodrome"](around:${r},${la},${lo}););out center tags qt;`;
  const eps = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.osm.ch/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
  ];
  const attempt = (ep) => fetch(`${ep}?data=${encodeURIComponent(ql)}`, { headers: { "user-agent": "phodar/1 (sighting airports)", accept: "application/json" }, signal: AbortSignal.timeout(18000) })
    .then(async (rr) => {
      const host = ep.split("/")[2];
      if (!rr.ok) throw new Error(`${host} HTTP ${rr.status}`);
      const j = await rr.json();
      if (!j || !Array.isArray(j.elements)) throw new Error(`${host} bad body`);
      if (j.remark && /timed out|runtime error|memory/i.test(j.remark)) throw new Error(`${host} BUSY`);
      if (j.elements.length === 0) throw new Error(`${host} EMPTY`);
      return j;
    });
  try {
    const j = await Promise.any(eps.map(attempt));
    airportsCache.set(key, { t: Date.now(), body: j });
    return json(res, 200, j);
  } catch (e) {
    const errs = (e && e.errors ? e.errors : [e]).map((x) => String(x.message || x));
    if (errs.every((m) => /EMPTY/.test(m))) return json(res, 200, { elements: [], note: "reachable; no airfields in range" });
    return json(res, 502, { error: `overpass busy (${errs.join("; ")})` });
  }
}
const peaksCache = new Map(); // key → { t, body }
async function apiPeaks(q, res) {
  const lat = coord(q, "lat", 90), lon = coord(q, "lon", 180), r = Math.min(160000, Math.max(1000, +q.get("r") || 40000));
  if (!isFinite(lat) || !isFinite(lon)) return json(res, 400, { error: "lat/lon required" });
  const key = `${lat.toFixed(3)},${lon.toFixed(3)},${r}`;
  const hit = peaksCache.get(key);
  if (hit && Date.now() - hit.t < 6 * 3600 * 1000) return json(res, 200, hit.body);
  const la = lat.toFixed(5), lo = lon.toFixed(5), rHill = Math.min(r, 30000); // hills are LOCAL landmarks — keep them near
  const ql = `[out:json][timeout:20];(node["natural"="peak"](around:${r},${la},${lo});node["natural"="volcano"](around:${r},${la},${lo});node["natural"="hill"](around:${rHill},${la},${lo}););out qt;`;
  const eps = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.osm.ch/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
  ];
  const attempt = (ep) => fetch(`${ep}?data=${encodeURIComponent(ql)}`, { headers: { "user-agent": "phodar/1 (sighting skyline)", accept: "application/json" }, signal: AbortSignal.timeout(18000) })
    .then(async (rr) => {
      const host = ep.split("/")[2];
      if (!rr.ok) throw new Error(`${host} HTTP ${rr.status}`);
      const j = await rr.json();
      if (!j || !Array.isArray(j.elements)) throw new Error(`${host} bad body`);
      // Overpass under load answers a timed-out query 200 elements:[] + a
      // `remark`. Treat THAT as BUSY (retryable), NOT genuine emptiness — else a
      // busy day looks like "no peaks within 120 km" in a place full of them.
      if (j.remark && /timed out|runtime error|memory/i.test(j.remark)) throw new Error(`${host} BUSY`);
      if (j.elements.length === 0) throw new Error(`${host} EMPTY`); // a mirror WITH data wins the race
      return j;
    });
  try {
    const j = await Promise.any(eps.map(attempt)); // fastest mirror WITH DATA wins
    peaksCache.set(key, { t: Date.now(), body: j });   // only non-empty is cached
    return json(res, 200, j);
  } catch (e) {
    const errs = (e && e.errors ? e.errors : [e]).map((x) => String(x.message || x));
    // 200 [] ONLY when every mirror genuinely returned zero peaks (real empty
    // area, not cached so a retry can still find data). If any was busy/timeout/
    // unreachable, 502 so the client says "busy — retry" instead of "none here".
    if (errs.every((m) => /EMPTY/.test(m))) return json(res, 200, { elements: [], note: "reachable; 0 peaks in range" });
    return json(res, 502, { error: `overpass busy (${errs.join("; ")})` });
  }
}
/* building-footprint proxy — OSM Overpass building ways WITH GEOMETRY around
   the observer, for the urban skyline (buildings.js). Same CORS-unreliable
   reason as /api/peaks, so proxy + race the mirrors + cache. Radius is capped
   tighter than peaks (city footprints are dense — `out geom` can be MBs). An
   empty result is legitimate here (rural — no buildings), returned as 200 []. */
const bldgCache = new Map(); // key → { t, body }
async function apiBuildings(q, res) {
  const lat = coord(q, "lat", 90), lon = coord(q, "lon", 180), r = Math.min(2000, Math.max(200, +q.get("r") || 1200));
  if (!isFinite(lat) || !isFinite(lon)) return json(res, 400, { error: "lat/lon required" });
  const key = `${lat.toFixed(4)},${lon.toFixed(4)},${r}`;
  const hit = bldgCache.get(key);
  if (hit && Date.now() - hit.t < 24 * 3600 * 1000) return json(res, 200, hit.body);
  // a BBOX query is dramatically faster in Overpass than around() for dense
  // building sets — the around() form distance-tests every element and was
  // timing out (returning 200 [] with a "remark"), which looked like "no data".
  const dLat = r / 111320, dLon = r / (111320 * Math.max(0.2, Math.cos(lat * Math.PI / 180)));
  const s = (lat - dLat).toFixed(6), w = (lon - dLon).toFixed(6), n = (lat + dLat).toFixed(6), e = (lon + dLon).toFixed(6);
  const ql = `[out:json][timeout:40];(way["building"](${s},${w},${n},${e}););out geom;`;
  const eps = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.osm.ch/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
  ];
  const attempt = (ep) => fetch(`${ep}?data=${encodeURIComponent(ql)}`, { headers: { "user-agent": "phodar/1 (sighting skyline)", accept: "application/json" }, signal: AbortSignal.timeout(42000) })
    .then(async (rr) => {
      const host = ep.split("/")[2];
      if (!rr.ok) throw new Error(`${host} HTTP ${rr.status}`);
      const j = await rr.json();
      if (!j || !Array.isArray(j.elements)) throw new Error(`${host} bad body`);
      // Overpass answers a timed-out/overloaded query with 200 elements:[] + a
      // `remark`. Treat that as BUSY (retryable), NOT as genuine emptiness.
      if (j.remark && /timed out|runtime error|memory/i.test(j.remark)) throw new Error(`${host} BUSY`);
      if (j.elements.length === 0) throw new Error(`${host} EMPTY`); // a mirror WITH data wins the race
      return j;
    });
  try {
    const j = await Promise.any(eps.map(attempt));
    bldgCache.set(key, { t: Date.now(), body: j }); // only non-empty is cached
    return json(res, 200, j);
  } catch (e) {
    const errs = (e && e.errors ? e.errors : [e]).map((x) => String(x.message || x));
    // 200 [] ONLY when every mirror genuinely returned zero buildings (rural).
    // If any was busy/timeout/unreachable, 502 so the client says "retry".
    if (errs.every((m) => /EMPTY/.test(m))) return json(res, 200, { elements: [], note: "reachable; 0 buildings in range" });
    return json(res, 502, { error: `overpass busy (${errs.join("; ")})` });
  }
}
/* winds-aloft proxy — Open-Meteo pressure-level winds. Browser-direct was
   flaky for OLD sightings (they need the ERA5 archive host, which can be slow to
   warm and inconsistent about CORS → "Load failed"). Forward the client's query
   to the forecast host first, then the archive, server-side (no CORS, 30 s). */
async function apiWinds(q, res) {
  if (!q.get("latitude") || !q.get("hourly")) return json(res, 400, { error: "lat/lon + hourly required" });
  const qs = q.toString();
  const hosts = [
    ["https://api.open-meteo.com/v1/forecast", "open-meteo forecast"],
    ["https://archive-api.open-meteo.com/v1/archive", "open-meteo ERA5 archive"],
  ];
  let lastErr = null;
  for (const [host, name] of hosts) {
    try {
      const r = await fetch(`${host}?${qs}`, { headers: { accept: "application/json" }, signal: AbortSignal.timeout(30000) });
      const j = await r.json();
      if (j && j.error) { lastErr = j.reason || "error"; continue; } // e.g. date outside this host's range → try the other
      if (!j || !j.hourly) { lastErr = `${name} no hourly`; continue; }
      j._src = name;
      return json(res, 200, j);
    } catch (e) { lastErr = String(e.message || e); }
  }
  return json(res, 502, { error: `winds unreachable (${lastErr})` });
}
/* live aircraft — MERGE several keyless ADS-B aggregators (each has its own
   ground-receiver coverage, so the union catches craft any single one misses)
   PLUS OpenSky, which adds MLAT / Mode-S targets that pure ADS-B feeds lack.
   Deduped by ICAO hex, gaps back-filled across feeds. Browser-direct only ever
   reached ONE feed (airplanes.live; adsb.lol/adsb.fi/OpenSky send no CORS), so
   this is a strict coverage win — hence server-side. Any feed that errors or
   rate-limits (OpenSky anon ~400/day per IP) is just dropped; the rest still
   answer. */
async function fetchAdsbxFeed(url, name) {
  const r = await fetch(url, { headers: { accept: "application/json", "user-agent": "phodar/1 (sighting correlator)" }, signal: AbortSignal.timeout(8000) });
  if (!r.ok) throw new Error(`${name} HTTP ${r.status}`);
  const j = await r.json();
  return (j.ac || []).filter((a) => a && a.hex);   // shared ADSBx-v2 record shape
}
const OS_CAT = ["", "A1", "A2", "A3", "A4", "A5", "A6", "A7", "B1", "B2", "B3", "B4", "B5", "B6", "B7", "C1", "C2", "C3"]; // OpenSky category idx → ADSBx emitter cat (approx)
async function fetchOpenSky(lat, lon, nm) {
  const dLat = (nm * 1852) / 111320;
  const dLon = dLat / Math.max(0.2, Math.cos((lat * Math.PI) / 180));
  const url = `https://opensky-network.org/api/states/all?lamin=${(lat - dLat).toFixed(4)}&lomin=${(lon - dLon).toFixed(4)}&lamax=${(lat + dLat).toFixed(4)}&lomax=${(lon + dLon).toFixed(4)}`;
  const r = await fetch(url, { headers: { accept: "application/json" }, signal: AbortSignal.timeout(8000) });
  if (!r.ok) throw new Error(`opensky HTTP ${r.status}`);
  const j = await r.json();
  const now = j.time || Math.floor(Date.now() / 1000);
  /* state vector: [icao24,callsign,country,tPos,lastContact,lon,lat,baroAlt,
     onGround,vel,track,vRate,sensors,geoAlt,squawk,spi,posSrc,category]. Alt in
     METRES, vel in m/s → convert to the ADSBx ft/kt the client's normAc expects. */
  return (j.states || []).filter((s) => s && s[0] && s[5] != null && s[6] != null).map((s) => ({
    hex: String(s[0]).toLowerCase(),
    flight: s[1] ? String(s[1]).trim() : undefined,
    lat: s[6], lon: s[5],
    alt_baro: s[8] ? "ground" : (s[7] != null ? Math.round(s[7] / FT) : (s[13] != null ? Math.round(s[13] / FT) : null)),
    gs: s[9] != null ? +(s[9] / KT).toFixed(1) : null,
    track: s[10] != null ? s[10] : null,
    category: (s[17] != null && OS_CAT[s[17]]) || null,
    seen: s[4] != null ? Math.max(0, now - s[4]) : null,
  }));
}
async function apiLive(q, res) {
  const lat = coord(q, "lat", 90), lon = coord(q, "lon", 180);
  const nm = Math.min(250, Math.max(5, +q.get("nm") || 60)), R = Math.round(nm);
  if (!isFinite(lat) || !isFinite(lon)) return json(res, 400, { error: "lat/lon required" });
  const feeds = [
    ["airplanes.live", fetchAdsbxFeed(`https://api.airplanes.live/v2/point/${lat}/${lon}/${R}`, "airplanes.live")],
    ["adsb.lol", fetchAdsbxFeed(`https://api.adsb.lol/v2/lat/${lat}/lon/${lon}/dist/${R}`, "adsb.lol")],
    ["adsb.fi", fetchAdsbxFeed(`https://opendata.adsb.fi/api/v2/lat/${lat}/lon/${lon}/dist/${R}`, "adsb.fi")],
    /* another independent community network, same ADSBx-v2 shape. Every one of
       these has a DIFFERENT set of volunteer receivers, and low-flying light
       aircraft are exactly the targets a distant receiver loses to the horizon
       (a plane at 2,000 ft AGL is line-of-sight to ~55 nm on flat ground, far
       less across a valley), so unioning one more network is the cheapest real
       coverage gain available without a receiver of your own. */
    ["adsb.one", fetchAdsbxFeed(`https://api.adsb.one/v2/point/${lat}/${lon}/${R}`, "adsb.one")],
    ["opensky", fetchOpenSky(lat, lon, nm)],
  ];
  const settled = await Promise.allSettled(feeds.map(([, p]) => p));
  const ok = [], errs = [], used = [];
  settled.forEach((s, i) => {
    if (s.status === "fulfilled") { ok.push([feeds[i][0], s.value]); used.push({ src: feeds[i][0], n: s.value.length }); }
    else errs.push(`${feeds[i][0]}: ${String(s.reason?.message || s.reason)}`);
  });
  if (!ok.length) return json(res, 502, { error: `no live feed reachable (${errs.join("; ")})` });
  /* union by hex; when a hex repeats, back-fill any fields the kept record is
     missing (a type designator from one feed, a fresh position from another) —
     so OpenSky-only craft are added while ADSBx type/reg is never lost. */
  const byHex = new Map();
  const FILL = ["flight", "r", "t", "category", "gs", "track", "alt_baro", "alt_geom", "lat", "lon", "seen"];
  for (const [name, list] of ok) {
    for (const a of list) {
      const hex = String(a.hex).toLowerCase();
      const prev = byHex.get(hex);
      if (!prev) { byHex.set(hex, { ...a, hex, _src: name }); continue; }
      for (const k of FILL) if ((prev[k] == null || prev[k] === "") && a[k] != null && a[k] !== "") prev[k] = a[k];
    }
  }
  const ac = [...byHex.values()];
  return json(res, 200, { ac, sources: used, errors: errs.length ? errs : undefined, now: Date.now(), merged: ac.length });
}
/* ---------- per-IP rate limit ----------
   Everything this proxy forwards to is free and most of it is volunteer-run
   (Overpass, the tar1090 archives, the ADS-B feeders). A public instance is
   one enthusiastic script away from being the reason those services start
   blocking people, and the app itself is nowhere near these ceilings — a full
   report is a few dozen tile fetches and a handful of queries.

   Two token buckets per client, because the cost classes are genuinely
   different: tiles arrive in bursts of ~36 while a report bakes its basemap
   and are cached hard downstream; the query endpoints each cost an upstream
   round trip (a history slice is 10–25 MB). Set PHODAR_RATELIMIT=off to
   disable — reasonable when you are the only user. */
const RL_ON = String(process.env.PHODAR_RATELIMIT || "").toLowerCase() !== "off";
const BUCKETS = {
  tile: { burst: 400, perSec: 8 },   // one report's basemap is up to 108 fetches
  query: { burst: 40, perSec: 0.5 },
};
const rlSeen = new Map(); // ip -> { tile: {n, t}, query: {n, t} }
const clientIp = (req) => {
  const fwd = req.headers["x-forwarded-for"];
  if (typeof fwd === "string" && fwd) return fwd.split(",")[0].trim();
  return req.socket?.remoteAddress || "?";
};
/* returns 0 when allowed, else the seconds to wait */
function rateLimit(req, kind) {
  if (!RL_ON) return 0;
  const cfg = BUCKETS[kind], ip = clientIp(req), now = Date.now();
  let rec = rlSeen.get(ip);
  if (!rec) {
    /* bound the table: sweep anything idle for 10 min before growing it */
    if (rlSeen.size > 5000) for (const [k, v] of rlSeen) if (now - Math.max(v.tile.t, v.query.t) > 600000) rlSeen.delete(k);
    rec = { tile: { n: BUCKETS.tile.burst, t: now }, query: { n: BUCKETS.query.burst, t: now } };
    rlSeen.set(ip, rec);
  }
  const b = rec[kind];
  b.n = Math.min(cfg.burst, b.n + ((now - b.t) / 1000) * cfg.perSec);
  b.t = now;
  if (b.n < 1) return Math.ceil((1 - b.n) / cfg.perSec);
  b.n -= 1;
  return 0;
}

/* ---------- /api/analyze — headless analysis engine (API access) ----------
   POST a session (the app's .phodar.json share shape) + optionally a drone
   flight-log CSV, get the full analysis verdict back. Key-gated: keys live
   in PHODAR_API_KEYS (comma-separated) and the endpoint is DISABLED until
   that env var is set — no accidental public compute. Media ingestion (raw
   photos/videos) is a later phase; this analyses measurements. */
const API_KEYS = new Set(String(process.env.PHODAR_API_KEYS || "").split(",").map((s) => s.trim()).filter(Boolean));
const ANALYZE_MAX_BODY = 32 * 1024 * 1024; // share JSONs carry 1600px JPEGs — a few MB each
async function apiAnalyze(req, res) {
  if (!API_KEYS.size) return json(res, 503, { error: "analysis API not enabled (set PHODAR_API_KEYS)" });
  const key = req.headers["x-api-key"] || "";
  if (!API_KEYS.has(String(key))) return json(res, 401, { error: "invalid or missing X-API-Key" });
  if (req.method !== "POST") return json(res, 405, { error: "POST a JSON body: { session | sources, flightLogText?, spanM?, droneId?, homeElevM? }" });
  let body;
  try {
    body = await new Promise((resolve, reject) => {
      const chunks = []; let size = 0;
      req.on("data", (c) => {
        size += c.length;
        if (size > ANALYZE_MAX_BODY) { reject(Object.assign(new Error("body too large"), { status: 413 })); req.destroy(); }
        else chunks.push(c);
      });
      req.on("end", () => resolve(Buffer.concat(chunks)));
      req.on("error", reject);
    });
  } catch (e) { return json(res, e.status || 400, { error: e.message }); }
  let input;
  try { input = JSON.parse(body.toString("utf8")); } catch (e) { return json(res, 400, { error: "invalid JSON body" }); }
  try {
    const { analyzeSession } = await import("../src/analyze/engine.js");
    return json(res, 200, analyzeSession(input));
  } catch (e) {
    return json(res, 500, { error: `analysis failed: ${e.message || e}` });
  }
}

/* ---------- /mcp — Model Context Protocol server (BYO-AI access) ----------
   The same analysis engine exposed as MCP tools so users drive it with
   THEIR OWN AI subscription — Claude, ChatGPT (custom connectors /
   developer mode / Agents SDK), Gemini SDKs, LangChain and friends all
   speak MCP's Streamable HTTP transport. Hand-rolled JSON-RPC (repo ethos:
   no dependencies), STATELESS mode (each POST answered directly; no
   session ids, no server-push SSE — spec-permitted, and what both Claude
   and ChatGPT accept). Auth reuses PHODAR_API_KEYS; because OAuth support
   varies by client, the key can ride the URL path (/mcp/<key>) — every
   client can paste a URL — or Authorization: Bearer / X-API-Key headers.
   Keys in URLs can end up in logs; docs say to treat them as revocable. */
const MCP_PROTOS = ["2024-11-05", "2025-03-26", "2025-06-18"];
const MCP_INSTRUCTIONS =
  "Phodar turns UFO/UAP sighting photos and videos into measured claims: triangulated position, altitude, true size, speed and heading, cross-checked against aircraft/satellites/stars/weather. " +
  "These tools consume a SESSION's measurements — the .phodar.json file the phodar app exports via '💾 Share file' (or the sighting.phodar.json inside a bundle .zip). " +
  "Workflow: the witness measures in the app (positions, sky placement, tracks), exports the share file, and you analyze it here — optionally with the drone's flight-log CSV when this is a calibration flight. " +
  "The verdict includes honest quality grades and warnings; relay them faithfully, including 'poor' and every caveat. Large share files: you may delete mediaJpeg/detailJpeg fields before sending — the engine never reads pixels.";
const MCP_TOOLS = [
  {
    name: "analyze_session",
    title: "Analyze a phodar session",
    description: "Run the full phodar analysis pipeline on a session's measurements (.phodar.json content): two-witness triangulated fix, visibility- and clock-aware trajectory stereo, dense two-video stereo, and — when a drone flight log is supplied — ground-truth calibration grades and per-witness clock checks. Returns a structured verdict plus a plain-text summary. Honest by design: incomplete witnesses and quality caveats are named, never hidden.",
    inputSchema: {
      type: "object",
      properties: {
        session: { description: "The parsed .phodar.json object ({sources:[...]}), or its raw JSON text. You may strip mediaJpeg/detailJpeg fields to shrink it — pixels are never read.", anyOf: [{ type: "object" }, { type: "string" }] },
        flightLogText: { type: "string", description: "Optional drone flight log as text: Airdata CSV export, decoded DJI Fly CSV, or DJI video .SRT captions. Turns the analysis into a graded calibration against the craft's GPS truth." },
        flightLogName: { type: "string", description: "Optional filename of the flight log (helps format detection, e.g. 'flight.csv' or 'clip.srt')." },
        spanM: { type: "number", description: "True span of the craft in metres (DJI Mavic Mini 0.202, DJI Neo 0.157). Enables size grading." },
        droneId: { type: "string", enum: ["mini1", "neo"], description: "Drone preset (sets spanM if not given)." },
        homeElevM: { type: "number", description: "Takeoff ground elevation in metres MSL, for logs that only record height above takeoff." },
      },
      required: ["session"],
    },
  },
  {
    name: "parse_flight_log",
    title: "Parse a drone flight log",
    description: "Parse a drone flight record (Airdata CSV, decoded DJI Fly CSV, or DJI .SRT captions) and report what it contains: sample count, time span, whether it carries an absolute clock and absolute (MSL) altitude, and first/mid/last states. Raw DJI FlightRecord .txt files are encrypted and rejected with guidance.",
    inputSchema: {
      type: "object",
      properties: {
        text: { type: "string", description: "The flight log file content as text." },
        name: { type: "string", description: "Optional filename (helps format detection)." },
      },
      required: ["text"],
    },
  },
];
function mcpAuthKey(req, u, pathKey) {
  const h = req.headers;
  const bearer = /^Bearer\s+(.+)$/i.exec(String(h.authorization || ""));
  return pathKey || (bearer && bearer[1]) || String(h["x-api-key"] || "") || u.searchParams.get("key") || "";
}
function mcpToolCall(name, args) {
  if (name === "parse_flight_log") {
    const r = parseFlightLogRef(String(args?.text ?? ""), String(args?.name ?? ""));
    if (!r.ok) return { ok: false, error: r.error };
    const pick = (p) => p && { tMs: p.tMs, lat: p.lat, lon: p.lon, altAbsM: p.altAbsM ?? null, altRelM: p.altRelM ?? null, speedMs: p.speedMs ?? null };
    return {
      ok: true, src: r.src, samples: r.n, spanS: +((r.t1Ms - r.t0Ms) / 1000).toFixed(1),
      absoluteClock: !!r.absTime, absoluteAltitude: !!r.hasAbsAlt,
      startsUtc: r.absTime ? new Date(r.t0Ms).toISOString() : null,
      first: pick(r.pts[0]), mid: pick(r.pts[r.n >> 1]), last: pick(r.pts[r.n - 1]),
    };
  }
  if (name === "analyze_session") {
    let session = args?.session;
    if (typeof session === "string") session = JSON.parse(session);
    const verdict = analyzeSessionRef({
      session, flightLogText: args?.flightLogText ?? null, flightLogName: args?.flightLogName,
      spanM: args?.spanM, droneId: args?.droneId, homeElevM: args?.homeElevM,
    });
    return { ok: true, summary: summarizeVerdictRef(verdict).join("\n"), verdict };
  }
  throw Object.assign(new Error(`unknown tool: ${name}`), { code: -32602 });
}
let analyzeSessionRef, summarizeVerdictRef, parseFlightLogRef;
async function mcpEnsureEngine() {
  if (!analyzeSessionRef) {
    const eng = await import("../src/analyze/engine.js");
    const fl = await import("../src/checks/flightlog.js");
    analyzeSessionRef = eng.analyzeSession; summarizeVerdictRef = eng.summarizeVerdict; parseFlightLogRef = fl.parseFlightLog;
  }
}
async function apiMcp(req, res, u, pathKey) {
  const cors = {
    "access-control-allow-origin": "*",
    "access-control-allow-methods": "POST, GET, OPTIONS",
    "access-control-allow-headers": "content-type, authorization, x-api-key, mcp-protocol-version, mcp-session-id",
  };
  if (req.method === "OPTIONS") { res.writeHead(204, cors); return res.end(); }
  if (!API_KEYS.size) { res.writeHead(503, { ...cors, "content-type": "application/json" }); return res.end(JSON.stringify({ error: "MCP server not enabled (set PHODAR_API_KEYS)" })); }
  if (!API_KEYS.has(mcpAuthKey(req, u, pathKey))) { res.writeHead(401, { ...cors, "content-type": "application/json" }); return res.end(JSON.stringify({ error: "invalid or missing API key — use /mcp/<key>, Authorization: Bearer, or X-API-Key" })); }
  if (req.method === "GET") { res.writeHead(405, { ...cors, allow: "POST" }); return res.end(); } // no server-push stream in stateless mode
  if (req.method !== "POST") { res.writeHead(405, { ...cors, allow: "POST" }); return res.end(); }
  let body;
  try {
    body = await new Promise((resolve, reject) => {
      const chunks = []; let size = 0;
      req.on("data", (c) => { size += c.length; if (size > ANALYZE_MAX_BODY) { reject(new Error("body too large")); req.destroy(); } else chunks.push(c); });
      req.on("end", () => resolve(Buffer.concat(chunks))); req.on("error", reject);
    });
  } catch (e) { res.writeHead(413, cors); return res.end(); }
  let msg; try { msg = JSON.parse(body.toString("utf8")); } catch (e) {
    res.writeHead(200, { ...cors, "content-type": "application/json" });
    return res.end(JSON.stringify({ jsonrpc: "2.0", id: null, error: { code: -32700, message: "parse error" } }));
  }
  await mcpEnsureEngine();
  const one = (m) => {
    const id = m?.id;
    const reply = (result) => ({ jsonrpc: "2.0", id, result });
    const fail = (code, message) => ({ jsonrpc: "2.0", id, error: { code, message } });
    try {
      switch (m?.method) {
        case "initialize": {
          const want = m.params?.protocolVersion;
          return reply({
            protocolVersion: MCP_PROTOS.includes(want) ? want : MCP_PROTOS[MCP_PROTOS.length - 1],
            capabilities: { tools: { listChanged: false } },
            serverInfo: { name: "phodar", title: "Phodar sighting analysis", version: "1.0.0" },
            instructions: MCP_INSTRUCTIONS,
          });
        }
        case "ping": return reply({});
        case "tools/list": return reply({ tools: MCP_TOOLS });
        case "tools/call": {
          try {
            const out = mcpToolCall(m.params?.name, m.params?.arguments || {});
            const text = out.ok === false ? `error: ${out.error}` : (out.summary || JSON.stringify(out, null, 1));
            return reply({ content: [{ type: "text", text }], structuredContent: out, isError: out.ok === false });
          } catch (e) {
            if (e.code === -32602) return fail(-32602, e.message);
            return reply({ content: [{ type: "text", text: `analysis failed: ${e.message || e}` }], isError: true });
          }
        }
        default:
          if (m?.method && String(m.method).startsWith("notifications/")) return null; // acknowledged silently
          return fail(-32601, `method not found: ${m?.method}`);
      }
    } catch (e) { return fail(-32603, String(e.message || e)); }
  };
  const out = Array.isArray(msg) ? msg.map(one).filter(Boolean) : one(msg);
  if (out == null || (Array.isArray(out) && !out.length)) { res.writeHead(202, cors); return res.end(); }
  res.writeHead(200, { ...cors, "content-type": "application/json", "cache-control": "no-store" });
  return res.end(JSON.stringify(out));
}

const server = http.createServer(async (req, res) => {
  try {
    const u = new URL(req.url, "http://x");
    if (u.pathname.startsWith("/api/") && u.pathname !== "/api/health") {
      const wait = rateLimit(req, u.pathname.startsWith("/api/tile/") ? "tile" : "query");
      if (wait) {
        res.writeHead(429, { "content-type": "application/json", "retry-after": String(wait), "cache-control": "no-store" });
        return res.end(JSON.stringify({ error: "rate limited — this proxy forwards to free, volunteer-run services", retryAfter: wait }));
      }
    }
    if (u.pathname === "/api/analyze") return await apiAnalyze(req, res);
    {
      const mm = u.pathname === "/mcp" ? [null, null] : u.pathname.match(/^\/mcp\/([^/]+)$/);
      if (mm) {
        const wait = rateLimit(req, "query");
        if (wait) { res.writeHead(429, { "content-type": "application/json", "retry-after": String(wait) }); return res.end(JSON.stringify({ error: "rate limited", retryAfter: wait })); }
        return await apiMcp(req, res, u, mm[1] ? decodeURIComponent(mm[1]) : "");
      }
    }
    if (u.pathname === "/api/hist") return await apiHist(u.searchParams, res);
    if (u.pathname === "/api/live") return await apiLive(u.searchParams, res);
    if (u.pathname.startsWith("/api/tile/")) return await apiTile(u, res);
    if (u.pathname === "/api/launches") return await apiLaunches(u.searchParams, res);
    if (u.pathname === "/api/fireballs") return await apiFireballs(u.searchParams, res);
    if (u.pathname === "/api/peaks") return await apiPeaks(u.searchParams, res);
    if (u.pathname === "/api/buildings") return await apiBuildings(u.searchParams, res);
    if (u.pathname === "/api/winds") return await apiWinds(u.searchParams, res);
    if (u.pathname === "/api/airports") return await apiAirports(u.searchParams, res);
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
