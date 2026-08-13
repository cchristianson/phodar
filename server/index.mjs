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
/* GFZ Kp geomagnetic index around a time — full history, CC-BY, but no CORS
   header, hence the proxy. Tiny payload; cached an hour. */
const kpCache = new Map();
async function apiKp(q, res) {
  const t = +q.get("t");
  if (!isFinite(t)) return json(res, 400, { error: "t required" });
  const key = Math.round(t / 3600000);
  const hit = kpCache.get(key);
  if (hit && Date.now() - hit.t < 3600000) return json(res, 200, hit.body);
  try {
    const iso = (ms) => new Date(ms).toISOString().replace(/\.\d{3}Z$/, "Z");
    const r = await fetch(`https://kp.gfz.de/app/json/?start=${iso(t - 6 * 3600000)}&end=${iso(t + 6 * 3600000)}&index=Kp`,
      { headers: { accept: "application/json" }, signal: AbortSignal.timeout(15000) });
    if (!r.ok) throw new Error(`GFZ HTTP ${r.status}`);
    const body = await r.json();
    kpCache.set(key, { t: Date.now(), body });
    return json(res, 200, body);
  } catch (e) {
    return json(res, 502, { error: `Kp unavailable (${e.message || e})` });
  }
}
/* FAA special-use airspace (MOAs/Restricted/etc.) near a point — ArcGIS
   FeatureServer, keyless (probed 2026-08). Geometry included so the client
   can do point-in-polygon + sight-line entry; precision trimmed to keep the
   payload sane. US-only data; cached a day (SUA boundaries barely move). */
const airspaceCache = new Map();
async function apiAirspace(q, res) {
  const lat = coord(q, "lat", 90), lon = coord(q, "lon", 180);
  const km = Math.min(300, Math.max(20, +q.get("km") || 120));
  if (!isFinite(lat) || !isFinite(lon)) return json(res, 400, { error: "lat/lon required" });
  const key = `${lat.toFixed(2)},${lon.toFixed(2)},${km}`;
  const hit = airspaceCache.get(key);
  if (hit && Date.now() - hit.t < 24 * 3600000) return json(res, 200, hit.body);
  try {
    const u = "https://services6.arcgis.com/ssFJjBXIUyZDrSYZ/arcgis/rest/services/Special_Use_Airspace/FeatureServer/0/query" +
      `?where=1%3D1&outFields=NAME,TYPE_CODE,LOWER_VAL,LOWER_UOM,UPPER_VAL,UPPER_UOM,TIMESOFUSE,CITY` +
      `&geometry=${lon},${lat}&geometryType=esriGeometryPoint&inSR=4326&distance=${km * 1000}&units=esriSRUnit_Meter` +
      `&returnGeometry=true&geometryPrecision=3&outSR=4326&f=json`;
    const r = await fetch(u, { headers: { accept: "application/json" }, signal: AbortSignal.timeout(25000) });
    if (!r.ok) throw new Error(`FAA ArcGIS HTTP ${r.status}`);
    const j = await r.json();
    if (j.error) throw new Error(j.error.message || "ArcGIS error");
    const body = { features: j.features || [] };
    airspaceCache.set(key, { t: Date.now(), body });
    return json(res, 200, body);
  } catch (e) {
    return json(res, 502, { error: `FAA airspace unavailable (${e.message || e})` });
  }
}
/* ─── radiosonde (weather-balloon) check — SondeHub proxy ─────────────
   /sites is a small global catalog (launch positions + schedules), cached a
   week and filtered to the observer's neighbourhood. Telemetry has NO
   spatial filter upstream (global ~0.5 MB per window), so the proxy pulls
   the window around the sighting, keeps only sondes that came within range,
   and returns compact aircraft-style tracks [[dtSec, lat, lon, altM], ...]
   with dt relative to the sighting instant. Fresh sightings use the live
   near-point endpoint instead. Probed 2026-08: both endpoints keyless. */
const sondeKm = (la1, lo1, la2, lo2) => {
  const p = Math.PI / 180, dphi = (la2 - la1) * p, dl = (lo2 - lo1) * p;
  const a = Math.sin(dphi / 2) ** 2 + Math.cos(la1 * p) * Math.cos(la2 * p) * Math.sin(dl / 2) ** 2;
  return 12742 * Math.asin(Math.min(1, Math.sqrt(a)));
};
let sondeSitesCache = null; // { t, body }
async function apiSondeSites(q, res) {
  const lat = coord(q, "lat", 90), lon = coord(q, "lon", 180);
  if (!isFinite(lat) || !isFinite(lon)) return json(res, 400, { error: "lat/lon required" });
  try {
    if (!sondeSitesCache || Date.now() - sondeSitesCache.t > 7 * 86400000) {
      const r = await fetch("https://api.v2.sondehub.org/sites", { headers: { accept: "application/json" }, signal: AbortSignal.timeout(20000) });
      if (!r.ok) throw new Error(`sondehub sites HTTP ${r.status}`);
      sondeSitesCache = { t: Date.now(), body: await r.json() };
    }
    const near = {};
    for (const [k, s] of Object.entries(sondeSitesCache.body || {})) {
      const lo = s?.position?.[0], la = s?.position?.[1];
      if (typeof la !== "number" || typeof lo !== "number") continue;
      if (sondeKm(lat, lon, la, lo) <= 400) near[k] = s;
    }
    return json(res, 200, { sites: near });
  } catch (e) {
    return json(res, 502, { error: `sondehub sites unavailable (${e.message || e})` });
  }
}
const sondesCache = new Map(); // key → { t, body }
async function apiSondes(q, res) {
  const lat = coord(q, "lat", 90), lon = coord(q, "lon", 180);
  const t = +q.get("t"), km = Math.min(400, Math.max(20, +q.get("km") || 250));
  if (!isFinite(lat) || !isFinite(lon) || !isFinite(t)) return json(res, 400, { error: "lat/lon/t required" });
  const key = `${lat.toFixed(2)},${lon.toFixed(2)},${Math.round(t / 600000)},${km}`;
  const hit = sondesCache.get(key);
  if (hit && Date.now() - hit.t < 30 * 60000) return json(res, 200, hit.body);
  try {
    let body;
    if (Math.abs(Date.now() - t) < 30 * 60000) {
      const r = await fetch(`https://api.v2.sondehub.org/sondes?lat=${lat}&lon=${lon}&distance=${km * 1000}&last=10800`, { headers: { accept: "application/json" }, signal: AbortSignal.timeout(20000) });
      if (!r.ok) throw new Error(`sondehub live HTTP ${r.status}`);
      const j = await r.json();
      const sondes = Object.values(j || {}).map((f) => ({
        serial: f.serial, type: f.type || f.subtype || null,
        track: [[+(((Date.parse(f.datetime) || Date.now()) - t) / 1000).toFixed(0), +(+f.lat).toFixed(5), +(+f.lon).toFixed(5), Math.round(+f.alt || 0)]],
      })).filter((s) => s.serial && isFinite(s.track[0][1]));
      body = { sondes, src: "sondehub live", hist: false };
    } else {
      const end = new Date(t + 45 * 60000).toISOString();
      const r = await fetch(`https://api.v2.sondehub.org/sondes/telemetry?duration=3h&datetime=${encodeURIComponent(end)}`, { headers: { accept: "application/json" }, signal: AbortSignal.timeout(40000) });
      if (!r.ok) throw new Error(`sondehub telemetry HTTP ${r.status}`);
      const j = await r.json();
      const sondes = [];
      for (const [serial, frames] of Object.entries(j || {})) {
        const pts = Object.values(frames || {})
          .filter((f) => typeof f.lat === "number" && typeof f.lon === "number")
          .map((f) => ({ ms: Date.parse(f.datetime), lat: f.lat, lon: f.lon, alt: +f.alt || 0, type: f.type || f.subtype }))
          .filter((p) => isFinite(p.ms))
          .sort((a, b) => a.ms - b.ms);
        if (pts.length < 2) continue;
        if (!pts.some((p) => sondeKm(lat, lon, p.lat, p.lon) <= km)) continue;
        sondes.push({
          serial, type: pts[0].type || null,
          track: pts.map((p) => [+(((p.ms - t) / 1000)).toFixed(0), +p.lat.toFixed(5), +p.lon.toFixed(5), Math.round(p.alt)]),
        });
      }
      body = { sondes, src: "sondehub archive", hist: true };
    }
    sondesCache.set(key, { t: Date.now(), body });
    return json(res, 200, body);
  } catch (e) {
    return json(res, 502, { error: `sondehub unavailable (${e.message || e})` });
  }
}
/* tall lit structures (masts/towers/chimneys/lighthouses) — the strobe-light
   candidate check. Same proxy + mirror-race + cache pattern as /api/peaks. */
const mastsCache = new Map();
async function apiMasts(q, res) {
  const lat = coord(q, "lat", 90), lon = coord(q, "lon", 180), r = Math.min(40000, Math.max(1000, +q.get("r") || 25000));
  if (!isFinite(lat) || !isFinite(lon)) return json(res, 400, { error: "lat/lon required" });
  const key = `${lat.toFixed(3)},${lon.toFixed(3)},${r}`;
  const hit = mastsCache.get(key);
  if (hit && Date.now() - hit.t < 24 * 3600 * 1000) return json(res, 200, hit.body);
  const la = lat.toFixed(5), lo = lon.toFixed(5);
  const sel = `["man_made"~"^(mast|tower|communications_tower|chimney|lighthouse)$"](around:${r},${la},${lo})`;
  const ql = `[out:json][timeout:20];(node${sel};way${sel};);out center tags qt;`;
  const eps = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.osm.ch/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
  ];
  const attempt = (ep) => fetch(`${ep}?data=${encodeURIComponent(ql)}`, { headers: { "user-agent": "phodar/1 (sighting masts)", accept: "application/json" }, signal: AbortSignal.timeout(18000) })
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
    mastsCache.set(key, { t: Date.now(), body: j });
    return json(res, 200, j);
  } catch (e) {
    const errs = (e && e.errors ? e.errors : [e]).map((x) => String(x.message || x));
    if (errs.every((m) => /EMPTY/.test(m))) { const body = { elements: [], note: "reachable; no tall structures in range" }; mastsCache.set(key, { t: Date.now(), body }); return json(res, 200, body); }
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
const roadCache = new Map(); // key → { t, body }

/* /api/roads — OSM road centerlines near the observer (Overpass, mirror-raced
   + cached like /api/buildings). The client projects them onto the dome and
   the horizon strip in true perspective as an azimuth-alignment aid (a road
   is the best anchor a flat-terrain daytime photo has). `out geom 2500`
   bounds a dense town's answer; _link variants match via the ^ prefixes. */
async function apiRoads(q, res) {
  const lat = coord(q, "lat", 90), lon = coord(q, "lon", 180), r = Math.min(4000, Math.max(200, +q.get("r") || 2600));
  if (!isFinite(lat) || !isFinite(lon)) return json(res, 400, { error: "lat/lon required" });
  const key = `${lat.toFixed(4)},${lon.toFixed(4)},${r}`;
  const hit = roadCache.get(key);
  if (hit && Date.now() - hit.t < 24 * 3600 * 1000) return json(res, 200, hit.body);
  const dLat = r / 111320, dLon = r / (111320 * Math.max(0.2, Math.cos(lat * Math.PI / 180)));
  const s = (lat - dLat).toFixed(6), w = (lon - dLon).toFixed(6), n = (lat + dLat).toFixed(6), e = (lon + dLon).toFixed(6);
  const ql = `[out:json][timeout:40];(way["highway"~"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|living_street|service|track)"](${s},${w},${n},${e}););out geom 2500;`;
  const eps = [
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.osm.ch/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
  ];
  const attempt = (ep) => fetch(`${ep}?data=${encodeURIComponent(ql)}`, { headers: { "user-agent": "phodar/1 (sighting alignment)", accept: "application/json" }, signal: AbortSignal.timeout(42000) })
    .then(async (rr) => {
      const host = ep.split("/")[2];
      if (!rr.ok) throw new Error(`${host} HTTP ${rr.status}`);
      const j = await rr.json();
      if (!j || !Array.isArray(j.elements)) throw new Error(`${host} bad body`);
      if (j.remark && /timed out|runtime error|memory/i.test(j.remark)) throw new Error(`${host} BUSY`);
      if (j.elements.length === 0) throw new Error(`${host} EMPTY`); // a mirror WITH data wins the race
      return j;
    });
  try {
    const j = await Promise.any(eps.map(attempt));
    roadCache.set(key, { t: Date.now(), body: j });
    return json(res, 200, j);
  } catch (e) {
    const errs = (e && e.errors ? e.errors : [e]).map((x) => String(x.message || x));
    if (errs.every((m) => /EMPTY/.test(m))) return json(res, 200, { elements: [], note: "reachable; 0 roads in range" });
    return json(res, 502, { error: `overpass busy (${errs.join("; ")})` });
  }
}
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
  "TWO ENTRY POINTS. (1) A finished session: the witness measured in the app and exported the .phodar.json ('💾 Share file', or inside a bundle .zip) — call analyze_session on it, optionally with a drone flight-log CSV for a calibration flight. " +
  "(2) RAW MEDIA (phase 2): given a video/photo URL — or a report page via fetch_report — run ingest_media, LOOK at the returned keyframes, confirm the object with inspect_frame, then auto_measure with every fact you can supply (position, time, camera bearing and up-angle, witness text; trim long clips to the sighting). You are the eyes and the context-gatherer; phodar is the instrument. The output is a DRAFT sighting for HUMAN review in the app — always hand over the bundle link and the guessed-values list, and say plainly that the sky placement is approximate until refined there. " +
  "The verdict includes honest quality grades and warnings; relay them faithfully, including 'poor' and every caveat. Large share files: you may delete mediaJpeg/detailJpeg fields before sending — the engine never reads pixels.";
const MCP_TOOLS = [
  {
    name: "ingest_media",
    title: "Ingest raw media (photo or video) by URL",
    description: "Phase-2 entry point: fetch a sighting photo/video from a URL, probe it, read its EXIF/QuickTime metadata (GPS, capture time, bearing, lens FOV), and return keyframe IMAGES so you can see the footage. Look at the keyframes, identify the anomalous object, then inspect_frame to confirm the spot and auto_measure to run the measurement pipeline. Media is held ~2 hours. Requires ffmpeg on the server.",
    inputSchema: { type: "object", properties: {
      url: { type: "string", description: "Direct http(s) URL of the media file (≤300 MB as a whole download). For a report page, call fetch_report first to find the media URL." },
      filename: { type: "string", description: "Optional filename hint (helps container detection)." },
      trim: { type: "object", description: "For LONG/LARGE clips (a 4K phone video easily tops 500 MB): fetch only this span via range requests — {t0, t1} seconds, span ≤150 s. Trim to where the object is visible; this is the route past the size cap, and it makes every later step faster too.", properties: { t0: { type: "number" }, t1: { type: "number" } } },
    }, required: ["url"] },
  },
  {
    name: "inspect_frame",
    title: "Zoom into a frame to confirm the object",
    description: "Decode one frame of ingested media. With fx/fy (fractions of frame width/height, 0..1) it snaps the mark onto the nearest compact object and returns a ×3 zoomed crop with a crosshair at the refined centre — LOOK at it and verify the crosshair is on the object, not a star/light/artifact. Without fx/fy it returns the full frame. Iterate until the mark is right; use the refined fractions in auto_measure.",
    inputSchema: { type: "object", properties: {
      mediaId: { type: "string" },
      t: { type: "number", description: "Frame time in seconds (0 for a photo)." },
      fx: { type: "number" }, fy: { type: "number" },
    }, required: ["mediaId"] },
  },
  {
    name: "auto_measure",
    title: "Run the auto-measurement pipeline",
    description: "Runs phodar's real measurement pipeline on ingested media: EXIF + your context become the witness facts, the object mark is snapped and sized, and for video the full stabilizer solves every frame's camera pose and auto-tracks the object (minutes of compute — returns a jobId; poll job_status). Output: a .phodar.json + an importable bundle the HUMAN reviews in the app. Every defaulted/guessed value is listed for review — relay that list faithfully. The sky placement is approximate until refined in the app (star-align / terrain snap / by hand); directions inherit its error until then.",
    inputSchema: { type: "object", properties: {
      mediaId: { type: "string" },
      context: { type: "object", description: "Everything you know: {lat, lon, elevM, whenMs or whenText (ISO), bearingDeg (compass the camera faced), elevationDeg (camera up-angle), fovH, name, observerName, witnessText, trim:{t0,t1} (seconds — REQUIRED for clips over 90s, and wise anyway: trim to the sighting)}. Omit what you don't know; EXIF fills what it can, the rest is defaulted and flagged.", properties: {
        lat: { type: "number" }, lon: { type: "number" }, elevM: { type: "number" },
        whenMs: { type: "number" }, whenText: { type: "string" },
        bearingDeg: { type: "number" }, elevationDeg: { type: "number" }, fovH: { type: "number" },
        name: { type: "string" }, observerName: { type: "string" }, witnessText: { type: "string" },
        trim: { type: "object", properties: { t0: { type: "number" }, t1: { type: "number" } } },
      } },
      object: { type: "object", description: "The object mark from your keyframe inspection: {t (seconds; the clearest frame), fx, fy (fractions 0..1), wfrac (apparent width as a fraction of frame width — estimate from the zoom crop)}.", properties: {
        t: { type: "number" }, fx: { type: "number" }, fy: { type: "number" }, wfrac: { type: "number" },
      }, required: ["fx", "fy"] },
      track: { type: "array", description: "Video: optional waypoints [{t, fx, fy}] marking the object on OTHER frames (use inspect_frame per frame). Two or more become the guide the auto-tracker follows — strongly recommended when the object is small or the camera moves a lot.", items: { type: "object", properties: { t: { type: "number" }, fx: { type: "number" }, fy: { type: "number" } } } },
    }, required: ["mediaId", "object"] },
  },
  {
    name: "job_status",
    title: "Poll a measurement job",
    description: "State of an auto_measure job: queued/running (stage + %), error, or done with the summary, the guessed-values list the human must review, and download links for the session (.phodar.json) and the importable bundle (.zip with the original media).",
    inputSchema: { type: "object", properties: { jobId: { type: "string" } }, required: ["jobId"] },
  },
  {
    name: "fetch_report",
    title: "Scrape a sighting-report page",
    description: "Best-effort scrape of a report page (e.g. a ufosighting.report record): og/twitter metas, JSON-LD, media-file URLs, datetime and coordinate-looking hints, plus a text excerpt. It does NOT understand the page — you do: combine the hints with your own reading, pick the media URL for ingest_media, and carry position/time/witness text into auto_measure context.",
    inputSchema: { type: "object", properties: { url: { type: "string" } }, required: ["url"] },
  },
  {
    name: "analyze_session",
    title: "Analyze a phodar session",
    description: "Run the full phodar analysis pipeline on a session's measurements (.phodar.json content): two-witness triangulated fix, visibility- and clock-aware trajectory stereo, dense two-video stereo, and — when a drone flight log is supplied — ground-truth calibration grades and per-witness clock checks. Returns a structured verdict plus a plain-text summary. Honest by design: incomplete witnesses and quality caveats are named, never hidden. Session shape (normally you feed an exported file, not hand-build one): {sources:[{lat, lon, alt?, whenMs?, fovH?, A:{az, el} (the committed sight-line, degrees true / above horizon), B?, track?, posePath?, objPath?}], est?}.",
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
  if (name === "ingest_media") {
    return (async () => {
      const r = await ingestRef.ingestFromUrl(String(args?.url || ""), { filename: args?.filename, trim: args?.trim });
      const p = r.probe;
      return {
        ok: true, mediaId: r.mediaId,
        probe: { kind: p.kind, w: p.w, h: p.h, durS: +p.durS.toFixed(2), codec: p.codec },
        metadata: metaBrief(r.meta),
        meta: r.meta ? { lat: r.meta.lat ?? null, lon: r.meta.lon ?? null, timeMs: r.meta.timeMs ?? null, azTrue: r.meta.azTrue ?? r.meta.az ?? null, fovH: r.meta.fovH ?? null } : null,
        summary: `Ingested ${p.kind} ${p.w}×${p.h}${p.kind === "video" ? ` · ${p.durS.toFixed(1)}s` : ""} (${p.codec}). ${metaBrief(r.meta)}. mediaId: ${r.mediaId}. The images below are keyframes${p.kind === "video" ? " at spread times — their t values are " + r.keyframes.map((k) => k.t + "s").join(", ") : ""}. LOOK at them, find the anomalous object, then call inspect_frame to confirm the exact spot before auto_measure.`,
        keyframeTimes: r.keyframes.map((k) => k.t),
        images: kfImages(r.keyframes),
      };
    })();
  }
  if (name === "inspect_frame") {
    return (async () => {
      const r = await ingestRef.inspectFrame(String(args?.mediaId || ""), { t: +args?.t || 0, fx: args?.fx, fy: args?.fy, zoomW: 240 });
      return {
        ok: true,
        refined: r.refined,
        summary: r.refined
          ? `Zoomed crop around your mark (crosshair = the snapped centre, refined to fx=${r.refined.fx}, fy=${r.refined.fy}). If the crosshair sits ON the object, use these refined fractions in auto_measure; if it latched onto something else, call again with a better fx/fy.`
          : "Full frame at the requested time — mark the object with fx/fy (fractions of width/height) to get a zoomed confirmation crop.",
        images: kfImages([{ t: +args?.t || 0, jpeg: r.jpeg }]),
      };
    })();
  }
  if (name === "auto_measure") {
    return (async () => {
      const jobId = ingestRef.startJob(String(args?.mediaId || ""), {
        context: args?.context || {}, object: args?.object || null, track: args?.track || [],
      });
      return { ok: true, jobId, summary: `Measurement job ${jobId} started (a video runs one to several minutes — the stabilizer solves every frame's camera pose). Poll job_status every ~20s until state is done, then relay the guessed/notes lists faithfully: they are what the human must review.` };
    })();
  }
  if (name === "job_status") {
    const st = ingestRef.jobStatus(String(args?.jobId || ""));
    if (!st) return { ok: false, error: "unknown jobId (jobs are forgotten on redeploy and after ~2 h)" };
    if (st.state === "done") {
      const base = process.env.PHODAR_PUBLIC_URL || "https://phodar.app";
      st.download = {
        bundle: `${base}/api/job/${args.jobId}/bundle.zip?key=<api-key>`,
        session: `${base}/api/job/${args.jobId}/session.json?key=<api-key>`,
      };
      st.summary = `Done. ${st.summary || ""}
Sight-line ${st.sightLine ? `${st.sightLine.az}°/${st.sightLine.el}°` : "—"} · posePath ${st.posePathPts} pts · objPath ${st.objPathPts} pts.
GUESSED (the human must review these in the app): ${(st.guessed || []).join("; ") || "nothing"}.
Notes: ${(st.notes || []).join("; ") || "none"}.
Hand the human the bundle link (fill in the api key): ${st.download.bundle} — they import it on phodar.app (📥 Import a shared sighting) and adjust from there.`;
    } else {
      st.summary = `Job ${args.jobId}: ${st.state} — ${st.stage} ${(st.frac * 100).toFixed(0)}%${st.error ? ` · ${st.error}` : ""}`;
    }
    return { ok: st.state !== "error", ...st };
  }
  if (name === "fetch_report") {
    return (async () => {
      const r = await ingestRef.fetchReport(String(args?.url || ""));
      return {
        ok: true, ...r,
        summary: r.kind === "json" ? "The URL returned JSON (excerpt in structuredContent.json) — read it for media URLs and metadata."
          : `Scraped "${r.title}". Media URLs found: ${r.mediaUrls.length ? r.mediaUrls.join(" · ") : "none — read textExcerpt/jsonLd for links or ask the human"}. Datetime hints: ${r.datetimes.join(", ") || "none"}. Coordinate-looking pairs: ${r.geoCandidates.join(" | ") || "none"}. This scrape is best-effort: combine it with your own reading of the page, pick the best media URL, and pass what you learned as auto_measure context.`,
      };
    })();
  }
  throw Object.assign(new Error(`unknown tool: ${name}`), { code: -32602 });
}
let analyzeSessionRef, summarizeVerdictRef, parseFlightLogRef, ingestRef;
async function mcpEnsureEngine() {
  if (!analyzeSessionRef) {
    const eng = await import("../src/analyze/engine.js");
    const fl = await import("../src/checks/flightlog.js");
    analyzeSessionRef = eng.analyzeSession; summarizeVerdictRef = eng.summarizeVerdict; parseFlightLogRef = fl.parseFlightLog;
  }
  if (!ingestRef) {
    ingestRef = await import("../src/ingest/serve.mjs");
    setInterval(() => ingestRef.gcSweep(), 10 * 60 * 1000).unref?.();
  }
}

/* keyframe images → MCP image blocks, downscaled hard: they ride in the tool
   result the model reads, so every extra pixel is context spent */
const kfImages = (keyframes) => (keyframes || []).slice(0, 5).map((k) => ({ data: Buffer.from(k.jpeg).toString("base64"), mimeType: "image/jpeg" }));

const metaBrief = (meta) => {
  if (!meta) return "no EXIF/QuickTime metadata (stripped or re-encoded)";
  const bits = [];
  if (meta.lat != null) bits.push(`GPS ${meta.lat}, ${meta.lon}${meta.alt != null ? ` · ${meta.alt} m` : ""}`);
  if (meta.timeMs) bits.push(`captured ${new Date(meta.timeMs).toISOString()}`);
  if (meta.azTrue != null) bits.push(`camera bearing ${meta.azTrue}° true`);
  else if (meta.az != null) bits.push(`camera bearing ${meta.az}° ${meta.azRef || ""}`);
  if (meta.fovH != null) bits.push(`FOV ${meta.fovH}°`);
  if (meta.model) bits.push(meta.model);
  return bits.join(" · ") || "metadata present but carries no position/time/lens facts";
};
/* ── phase-2 ingest over plain HTTP (the MCP tools' curl-able mirror) ──
   POST /api/ingest            {url} JSON, or raw media bytes (spooled to disk)
   GET  /api/job/<id>          job status
   GET  /api/job/<id>/bundle.zip | session.json   artifacts (browser-friendly:
                               the key rides ?key= so a human can click it) */
async function apiIngest(req, res, u) {
  if (!API_KEYS.size) return json(res, 503, { error: "ingest API not enabled (set PHODAR_API_KEYS)" });
  const key = req.headers["x-api-key"] || u.searchParams.get("key") || "";
  if (!API_KEYS.has(String(key))) return json(res, 401, { error: "invalid or missing key (X-API-Key header or ?key=)" });
  await mcpEnsureEngine();
  if (u.pathname === "/api/ingest") {
    if (req.method !== "POST") return json(res, 405, { error: "POST {url, filename?} as JSON, or the raw media bytes (with ?filename=)" });
    const ctype = String(req.headers["content-type"] || "");
    if (ctype.includes("json")) {
      const chunks = [];
      for await (const c of req) { chunks.push(c); if (chunks.reduce((a, b) => a + b.length, 0) > 1e6) return json(res, 413, { error: "JSON body too large" }); }
      let body; try { body = JSON.parse(Buffer.concat(chunks).toString()); } catch (e) { return json(res, 400, { error: "bad JSON" }); }
      try {
        const r = await ingestRef.ingestFromUrl(String(body.url || ""), { filename: body.filename, trim: body.trim });
        return json(res, 200, { mediaId: r.mediaId, probe: r.probe, meta: r.meta, keyframeTimes: r.keyframes.map((k) => k.t) });
      } catch (e) { return json(res, 400, { error: String(e.message || e) }); }
    }
    try {
      const r = await ingestRef.ingestFromStream(req, { filename: u.searchParams.get("filename") || "" });
      return json(res, 200, { mediaId: r.mediaId, probe: r.probe, meta: r.meta, keyframeTimes: r.keyframes.map((k) => k.t) });
    } catch (e) { return json(res, 400, { error: String(e.message || e) }); }
  }
  const mm = u.pathname.match(/^\/api\/job\/([a-z0-9]+)(?:\/(bundle\.zip|session\.json))?$/i);
  if (!mm) return json(res, 404, { error: "unknown ingest path" });
  if (mm[2]) {
    const p = ingestRef.jobFile(mm[1], mm[2] === "bundle.zip" ? "bundle" : "session");
    if (!p) return json(res, 404, { error: "artifact not ready (or the job was forgotten on a redeploy)" });
    res.writeHead(200, {
      "content-type": mm[2] === "bundle.zip" ? "application/zip" : "application/json",
      "content-disposition": `attachment; filename="phodar-${mm[2]}"`,
      "cache-control": "no-store",
    });
    return createReadStream(p).pipe(res);
  }
  const st = ingestRef.jobStatus(mm[1]);
  if (!st) return json(res, 404, { error: "unknown job (jobs are forgotten on redeploy and after ~2 h)" });
  return json(res, 200, st);
}

/* POST /api/measure — start a job over HTTP: {mediaId, context, object, track} */
async function apiMeasure(req, res, u) {
  if (!API_KEYS.size) return json(res, 503, { error: "ingest API not enabled (set PHODAR_API_KEYS)" });
  const key = req.headers["x-api-key"] || u.searchParams.get("key") || "";
  if (!API_KEYS.has(String(key))) return json(res, 401, { error: "invalid or missing key" });
  if (req.method !== "POST") return json(res, 405, { error: "POST {mediaId, context, object:{t,fx,fy,wfrac?}, track?}" });
  await mcpEnsureEngine();
  const chunks = [];
  for await (const c of req) { chunks.push(c); if (chunks.reduce((a, b) => a + b.length, 0) > 2e6) return json(res, 413, { error: "body too large" }); }
  let body; try { body = JSON.parse(Buffer.concat(chunks).toString()); } catch (e) { return json(res, 400, { error: "bad JSON" }); }
  try {
    const jobId = ingestRef.startJob(String(body.mediaId || ""), { context: body.context || {}, object: body.object || null, track: body.track || [] });
    return json(res, 200, { jobId, poll: `/api/job/${jobId}` });
  } catch (e) { return json(res, 400, { error: String(e.message || e) }); }
}

/* GET /api/report?url= — the APP's report-link import (home screen: paste a
   sighting-report link, get the page's machine-readable bones back to
   pre-fill a sighting). UN-KEYED so the app itself can call it — which is
   exactly why the fetch target is HOST-ALLOWLISTED: an open un-keyed
   endpoint would otherwise be a free scrape/SSRF proxy. Default allowlist
   is ufosighting.report; extend with PHODAR_REPORT_HOSTS (comma-separated
   host suffixes). PHODAR_INGEST_ALLOW_LOCAL=1 bypasses for tests (the
   fetchReport SSRF guard is separately bypassed by the same var). */
async function apiReport(u, res) {
  const raw = String(u.searchParams.get("url") || "");
  let target;
  try { target = new URL(raw); } catch (e) { return json(res, 400, { error: "invalid url" }); }
  const hosts = (process.env.PHODAR_REPORT_HOSTS || "ufosighting.report").split(",").map((s) => s.trim().toLowerCase()).filter(Boolean);
  const hn = target.hostname.toLowerCase();
  const okHost = hosts.some((h) => hn === h || hn.endsWith("." + h)) || process.env.PHODAR_INGEST_ALLOW_LOCAL === "1";
  if (!okHost) return json(res, 403, { error: `report links are limited to: ${hosts.join(", ")}` });
  try {
    const ing = await import("../src/ingest/serve.mjs");
    return json(res, 200, await ing.fetchReport(raw));
  } catch (e) {
    return json(res, 502, { error: String(e?.message || e) });
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
  const one = async (m) => {
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
            const out = await mcpToolCall(m.params?.name, m.params?.arguments || {});
            const text = out.ok === false ? `error: ${out.error}` : (out.summary || JSON.stringify(out, null, 1));
            /* image blocks (keyframes, crops) ride the content array so vision
               clients SEE them; they are stripped from structuredContent —
               duplicating megabytes of base64 there helps no one */
            const { images, ...structured } = out;
            const content = [{ type: "text", text }];
            for (const im of images || []) content.push({ type: "image", data: im.data, mimeType: im.mimeType || "image/jpeg" });
            return reply({ content, structuredContent: structured, isError: out.ok === false });
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
  const out = Array.isArray(msg) ? (await Promise.all(msg.map(one))).filter(Boolean) : await one(msg);
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
    if (u.pathname === "/api/ingest" || u.pathname.startsWith("/api/job/")) return await apiIngest(req, res, u);
    if (u.pathname === "/api/measure") return await apiMeasure(req, res, u);
    if (u.pathname === "/api/report") return await apiReport(u, res);
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
    if (u.pathname === "/api/masts") return await apiMasts(u.searchParams, res);
    if (u.pathname === "/api/sondesites") return await apiSondeSites(u.searchParams, res);
    if (u.pathname === "/api/airspace") return await apiAirspace(u.searchParams, res);
    if (u.pathname === "/api/kp") return await apiKp(u.searchParams, res);
    if (u.pathname === "/api/sondes") return await apiSondes(u.searchParams, res);
    if (u.pathname === "/api/buildings") return await apiBuildings(u.searchParams, res);
    if (u.pathname === "/api/roads") return await apiRoads(u.searchParams, res);
    if (u.pathname === "/api/winds") return await apiWinds(u.searchParams, res);
    if (u.pathname === "/api/airports") return await apiAirports(u.searchParams, res);
    if (u.pathname === "/api/health") {
      let ingest = false;
      try { const m = await import("../src/ingest/media.mjs"); ingest = !!(await m.ffmpegAvailable()); } catch (e) { }
      return json(res, 200, { ok: true, ingest, cacheMB: Math.round(sliceCache.size / 1048576) });
    }
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
