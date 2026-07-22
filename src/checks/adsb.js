/* ============================================================
   ADS-B CHECK — the first cross-check source (CLAUDE.md roadmap #3).
   Given observer position + sight-line (az/el) + angular size, query
   live aircraft and rank candidates by angular separation from the
   sight-line. Aircraft type → wingspan gives absolute ground truth:
   if the predicted angular size matches what the witness measured,
   the mundane explanation writes itself.

   Design principle (shared by every future check — satellites, stars,
   balloons): output candidates as
     { label, az, el, sepDeg, rangeM, spanM, predAngDeg, ... }
   so the report can rank ALL mundane explanations in one table.

   APIs: the app's own server /api/live MERGES several keyless feeds
   (airplanes.live + adsb.lol + adsb.fi, ADSBx-v2 shape) AND OpenSky
   (state-vector shape; adds MLAT / Mode-S targets pure ADS-B misses),
   deduped by ICAO hex — every network has different ground-receiver
   coverage, so the union catches craft any single feed misses. Only
   airplanes.live is CORS-open, so the merge must run server-side; the
   browser falls back to airplanes.live direct when the server is absent.
   ============================================================ */

import { D2R, R2D, RE, sub, mag, dot, unit, enuFromGeo, dirFromAzEl, dirToAzEl } from "../math/geodesy.js";
import { isNum } from "../math/format.js";

const FT = 0.3048, KT = 0.514444, NM_M = 1852;

/* ICAO type designator → wingspan (m). Common types seen over North
   America/Europe; the category fallback below catches the rest. */
export const WINGSPANS = {
  // Boeing narrowbody / widebody
  B712: 28.4, B722: 32.9, B732: 28.4, B733: 28.9, B734: 28.9, B735: 28.9,
  B736: 34.3, B737: 34.3, B738: 35.8, B739: 35.8, B37M: 35.9, B38M: 35.9, B39M: 35.9,
  B741: 59.6, B742: 59.6, B744: 64.4, B748: 68.4,
  B752: 38.0, B753: 38.0, B762: 47.6, B763: 47.6, B764: 51.9,
  B772: 60.9, B773: 60.9, B77L: 64.8, B77W: 64.8, B788: 60.1, B789: 60.1, B78X: 60.1,
  // Airbus
  A318: 34.1, A319: 34.1, A320: 34.1, A321: 34.1, A19N: 35.8, A20N: 35.8, A21N: 35.8,
  A306: 44.8, A310: 43.9, A332: 60.3, A333: 60.3, A338: 64.0, A339: 64.0,
  A342: 60.3, A343: 60.3, A345: 63.4, A346: 63.4, A359: 64.8, A35K: 64.8, A388: 79.8,
  // regional jets & turboprops
  E135: 20.0, E145: 20.0, E170: 26.0, E175: 26.0, E75L: 26.0, E75S: 26.0,
  E190: 28.7, E195: 28.7, E290: 33.7, E295: 35.1,
  CRJ2: 21.2, CRJ7: 23.2, CRJ9: 24.9, CRJX: 26.2,
  DH8A: 25.9, DH8B: 25.9, DH8C: 27.4, DH8D: 28.4, AT45: 24.6, AT46: 24.6, AT75: 27.1, AT76: 27.1,
  SF34: 21.4, SB20: 24.8, F70: 28.1, F100: 28.1, MD82: 32.8, MD83: 32.8, MD88: 32.8, MD90: 32.9,
  // GA / bizjets / military-ish
  C150: 10.2, C152: 10.2, C172: 11.0, C175: 11.0, C182: 11.0, C206: 11.0, C208: 15.9, C210: 11.2,
  P28A: 10.8, P28B: 10.8, PA31: 12.4, PA34: 11.9, BE33: 10.2, BE35: 10.2, BE36: 10.2,
  BE58: 11.5, BE20: 16.6, B350: 17.7, PC12: 16.3, TBM9: 12.8, SR20: 11.7, SR22: 11.7,
  DA40: 11.9, DA42: 13.4, M20P: 11.0, RV10: 9.7, GLID: 15.0,
  C25A: 14.3, C25B: 15.5, C25C: 16.4, C525: 14.3, C56X: 17.2, C680: 19.2, C700: 21.2,
  CL30: 18.6, CL35: 21.0, CL60: 19.6, GLF4: 23.7, GLF5: 28.5, GLF6: 30.4, GL7T: 31.7,
  LJ35: 12.0, LJ45: 14.6, LJ60: 13.4, E50P: 12.3, E55P: 16.2, HDJT: 12.1,
  F900: 19.3, FA7X: 26.2, FA8X: 26.3, F2TH: 19.3,
  // helicopters
  R22: 7.7, R44: 10.1, R66: 10.1, EC30: 10.7, EC35: 10.2, EC45: 11.0, AS50: 10.7, B06: 10.2, B407: 10.7, B429: 11.0, S76: 13.4, H60: 16.4, UH1: 14.6,
  // heavies / cargo / military transports
  C130: 40.4, C17: 51.7, C5M: 67.9, K35R: 39.9, A400: 42.4, B52: 56.4, E3TF: 44.4, P8: 37.6, KC46: 47.6,
};

/* ADS-B emitter category → rough span when the type is unknown */
const CATEGORY_SPANS = { A1: 11, A2: 18, A3: 34, A4: 45, A5: 62, A7: 11, B1: 15, B2: 18, B4: 15 };

export function spanForAircraft(t, category) {
  if (t && WINGSPANS[t] != null) return { span: WINGSPANS[t], src: t };
  if (category && CATEGORY_SPANS[category] != null) return { span: CATEGORY_SPANS[category], src: `cat ${category}` };
  return { span: null, src: null };
}

/* az/el/range of an aircraft from an observer, in the observer's ENU
   frame, with an earth-curvature + standard-refraction (k≈0.13) drop —
   at 100 km that correction is ~0.4° of elevation, which matters when
   the whole test is "how many degrees off the sight-line". */
export function acAzElRange(obs, ac) {
  const P = enuFromGeo(+ac.lat, +ac.lon, ac.altM || 0, { lat: +obs.lat, lon: +obs.lon, alt: isNum(obs.alt) ? +obs.alt : 0 });
  const dGround = Math.hypot(P[0], P[1]);
  P[2] -= (dGround * dGround * (1 - 0.13)) / (2 * RE);
  const rng = mag(P);
  const ae = dirToAzEl(unit(P));
  return { az: ae.az, el: ae.el, rangeM: rng, d: unit(P) };
}

/* Rank live aircraft against every witness sight-line.
   sources: app sources (need lat/lon + A.az/el; ang optional).
   aircraft: normalized records from fetchAircraft().
   Returns candidates sorted by worst-witness separation — a real match
   must sit near EVERY witness's sight-line, so the max governs. */
export function rankCandidates(sources, aircraft) {
  const wit = sources
    .filter((s) => isNum(s.lat) && isNum(s.lon) && isNum(s.A?.az) && isNum(s.A?.el))
    .map((s) => ({ s, d: dirFromAzEl(+s.A.az, +s.A.el) }));
  if (!wit.length) return null;
  const out = [];
  for (const ac of aircraft) {
    if (!isNum(ac.lat) || !isNum(ac.lon)) continue;
    if (ac.ground) continue; // taxiing at an airport can't be a sky sighting
    const { span, src: spanSrc } = spanForAircraft(ac.t, ac.category);
    const per = wit.map((w) => {
      const g = acAzElRange(w.s, ac);
      const sep = Math.acos(Math.min(1, Math.max(-1, dot(w.d, g.d)))) * R2D;
      const predAng = span != null ? 2 * Math.atan(span / 2 / g.rangeM) * R2D : null;
      return { name: w.s.name, az: g.az, el: g.el, rangeM: g.rangeM, sep, predAng };
    });
    out.push({
      hex: ac.hex, flight: (ac.flight || "").trim() || null, reg: ac.reg || null,
      t: ac.t || null, category: ac.category || null,
      span, spanSrc,
      altM: ac.altM, gs: ac.gs, track: ac.track, seen: ac.seen,
      per,
      sepMax: Math.max(...per.map((p) => p.sep)),
      sepMin: Math.min(...per.map((p) => p.sep)),
      rangeM: per[0].rangeM,
      predAng: per[0].predAng,
    });
  }
  out.sort((a, b) => a.sepMax - b.sepMax);
  return out;
}

/* normalize one ADSBx-v2 record; both APIs share this shape */
function normAc(a) {
  const altFt = isNum(a.alt_geom) ? +a.alt_geom : (isNum(a.alt_baro) ? +a.alt_baro : null);
  return {
    hex: a.hex, flight: a.flight, reg: a.r, t: a.t, category: a.category,
    lat: a.lat, lon: a.lon,
    ground: a.alt_baro === "ground",
    altM: altFt != null ? altFt * FT : (a.alt_baro === "ground" ? 0 : null),
    gs: isNum(a.gs) ? +a.gs * KT : null,        // m/s
    track: isNum(a.track) ? +a.track : null,
    seen: isNum(a.seen) ? +a.seen : null,        // s since last message
  };
}

export async function fetchAircraft(lat, lon, nm = 60) {
  const R = Math.min(250, Math.round(nm));
  /* PRIMARY: our server's /api/live MERGES several keyless aggregators
     (airplanes.live + adsb.lol + adsb.fi) AND OpenSky (adds MLAT / Mode-S
     targets pure ADS-B misses), deduped by hex — strictly more aircraft than any
     single browser-direct feed. Falls back to browser-direct when the server
     isn't there (dev without it) or is unreachable. */
  try {
    const r = await fetch(`/api/live?lat=${lat}&lon=${lon}&nm=${R}`, { signal: AbortSignal.timeout(15000) });
    if (r.ok) {
      const j = await r.json();
      if (Array.isArray(j.ac) && j.ac.length >= 0 && Array.isArray(j.sources)) {
        const ac = j.ac.filter((a) => a.lat != null).map(normAc);
        return { ac, source: "merged: " + j.sources.map((s) => s.src).join(" + "), now: j.now || Date.now(), merged: true };
      }
    }
  } catch (e) { /* server absent/unreachable → browser-direct fallback below */ }
  const urls = [
    `https://api.airplanes.live/v2/point/${lat}/${lon}/${R}`,
    `https://api.adsb.lol/v2/lat/${lat}/lon/${lon}/dist/${R}`,
  ];
  let lastErr = null;
  for (const url of urls) {
    try {
      const r = await fetch(url, { signal: AbortSignal.timeout(12000) });
      if (!r.ok) { lastErr = new Error(`HTTP ${r.status}`); continue; }
      const j = await r.json();
      const ac = (j.ac || []).filter((a) => a.lat != null).map(normAc);
      return { ac, source: new URL(url).host, now: j.now || Date.now() };
    } catch (e) { lastErr = e; }
  }
  throw lastErr || new Error("no ADS-B source reachable");
}

/* Historical: aircraft near (lat,lon) AT time tMs, via our own server's
   /api/hist (tar1090 globe_history archives — ~2 years back, and today
   with a few minutes' lag). Records come back already normalized; `seen`
   is |sample − t| in seconds and `coarse` marks heatmap-only fixes
   (30 s cadence, no type/callsign refinement). */
export async function fetchAircraftAt(lat, lon, tMs, nm = 60, winMin = 8) {
  const r = await fetch(`/api/hist?lat=${lat}&lon=${lon}&t=${Math.round(tMs)}&nm=${Math.round(nm)}&win=${winMin}`,
    { signal: AbortSignal.timeout(45000) });
  const j = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(j.error || `HTTP ${r.status}`);
  return { ac: j.ac || [], source: j.src, sampleT: j.sampleT, hist: true };
}

/* Identity + scheduled route for one aircraft, via adsbdb.com (free,
   CORS-open, probed 2026-07-14). Route is looked up by CALLSIGN and is
   the schedule for that flight number — label it "scheduled", it can
   differ from the actual leg (repositioning, diversions). */
const acInfoCache = new Map();
export async function fetchAcInfo(hex, callsign) {
  const key = `${hex || ""}|${callsign || ""}`;
  if (acInfoCache.has(key)) return acInfoCache.get(key);
  const get = async (url) => {
    try {
      const r = await fetch(url, { signal: AbortSignal.timeout(8000) });
      if (!r.ok) return null;
      return (await r.json()).response || null;
    } catch (e) { return null; }
  };
  const [rt, acr] = await Promise.all([
    callsign ? get(`https://api.adsbdb.com/v0/callsign/${encodeURIComponent(callsign.trim())}`) : null,
    hex ? get(`https://api.adsbdb.com/v0/aircraft/${encodeURIComponent(hex)}`) : null,
  ]);
  const out = {
    route: rt && typeof rt === "object" && rt.flightroute ? rt.flightroute : null,
    aircraft: acr && typeof acr === "object" && acr.aircraft ? acr.aircraft : null,
  };
  if (acInfoCache.size > 300) acInfoCache.clear();
  acInfoCache.set(key, out);
  return out;
}

/* search radius from the sight-line: a jet at 45,000 ft on a shallow
   sight-line can be far away horizontally; clamp to the API cap */
export function radiusNmForSources(sources) {
  const els = sources.filter((s) => isNum(s.A?.el)).map((s) => Math.max(3, +s.A.el));
  const el = els.length ? Math.min(...els) : 10;
  const horizM = 13716 / Math.tan(el * D2R); // 45 kft ceiling
  return Math.max(20, Math.min(150, Math.ceil(horizM / NM_M) + 10));
}
