/* ============================================================
   WEATHER-BALLOON CHECK (radiosondes) — the #1 mundane explanation gets
   the same treatment aircraft got: not "could it have been a balloon"
   but "was a real one actually there".

   Two independent layers:
   1. LAUNCH SITES (SondeHub /sites — ~900 stations worldwide with
      position, scheduled launch times, measured ascent rate and burst
      altitude). Works for ANY sighting age: was a scheduled synoptic
      balloon plausibly AIRBORNE at the stated time, and where was it in
      its flight? Real launches go ~45–75 min before the synoptic hour.
   2. ACTUAL TRACKS (SondeHub live + telemetry archive via /api/sondes):
      received radiosonde positions near the observer, ranked against the
      sight-lines exactly like aircraft — including the trajectory
      (track-time) match when the witness has a timed track, reusing the
      same pure geometry (`acAzElRange`, `trailStateAt`, `trackMatch`).

   Probed 2026-08: api.v2.sondehub.org is CORS-open (ACAO:*); /sites is
   ~50 KB; /sondes/telemetry has no spatial filter (global, ~0.5 MB per
   window) hence the server-side proxy + distance filter + cache.
   Honesty: receiver coverage is volunteer — an empty track list rules
   out RECEIVED sondes, not balloons.
   ============================================================ */

import { R2D, D2R } from "../math/geodesy.js";
import { isNum } from "../math/format.js";
import { acAzElRange, trailStateAt, trackMatch, trackAbsSamples } from "./adsb.js";
import { bearingDeg, distM } from "./peaks.js";

/* SondeHub /sites JSON → nearest launch sites. times come as "0:HH:MM"
   day-offset strings (synoptic UTC hours); keep the raw strings and the
   parsed UTC hour+minute. */
export function parseSites(json, lat, lon, maxKm = 250) {
  const out = [];
  for (const s of Object.values(json || {})) {
    const lo = s?.position?.[0], la = s?.position?.[1];
    if (!isNum(la) || !isNum(lo)) continue;
    const d = distM(lat, lon, la, lo);
    if (d > maxKm * 1000) continue;
    const times = (Array.isArray(s.times) ? s.times : []).map((t) => {
      const m = String(t).match(/^(\d+):(\d{1,2}):(\d{2})$/);
      return m ? { day: +m[1], h: +m[2], min: +m[3] } : null;
    }).filter(Boolean);
    out.push({
      name: s.station_name || s.station || "launch site",
      lat: la, lon: lo, altM: isNum(s.alt) ? +s.alt : 0,
      distKm: d / 1000, az: bearingDeg(lat, lon, la, lo),
      times,
      burstM: isNum(s.burst_altitude) ? +s.burst_altitude : 32000,
      ascMS: isNum(s.ascent_rate) ? +s.ascent_rate : 5,
    });
  }
  return out.sort((a, b) => a.distKm - b.distKm);
}

/* Was a scheduled balloon from this site plausibly AIRBORNE at whenMs?
   Launches go PRE_S before the synoptic hour; ascent = (burst−ground)/rate;
   descent ≈ 40 min. Checks the site's schedule across the surrounding days
   and returns the best-fitting flight: { launchMs, tS (seconds airborne),
   phase asc|desc, altM estimate } — or null when nothing was up. */
export function launchStateAt(site, whenMs) {
  const PRE_S = 3600;            // launch ~1 h before the synoptic hour
  const DESC_S = 40 * 60;
  const ascS = Math.max(600, (site.burstM - site.altM) / (site.ascMS || 5));
  const day0 = Math.floor(whenMs / 86400000) * 86400000;
  let best = null;
  for (let d = -1; d <= 1; d++) {
    for (const t of site.times || []) {
      const synMs = day0 + d * 86400000 + ((t.day * 24 + t.h) * 60 + t.min) * 60000;
      const launchMs = synMs - PRE_S * 1000;
      const tS = (whenMs - launchMs) / 1000;
      if (tS < 0 || tS > ascS + DESC_S) continue;
      const phase = tS <= ascS ? "asc" : "desc";
      const altM = phase === "asc"
        ? site.altM + tS * (site.ascMS || 5)
        : Math.max(0, site.burstM - (tS - ascS) * (site.burstM / DESC_S)); // fast, chute-slowed fall
      const cand = { launchMs, tS, phase, altM: Math.min(altM, site.burstM) };
      if (!best || Math.abs(tS - ascS / 2) < Math.abs(best.tS - ascS / 2)) best = cand;
    }
  }
  return best;
}

/* balloon envelope grows as it rises (~1.5 m at release → ~7–9 m at burst);
   linear in altitude is plenty for an angular-size sanity check */
export function balloonDiaM(altM, burstM = 32000) {
  return 1.5 + 6 * Math.max(0, Math.min(1, altM / Math.max(1, burstM)));
}

/* Rank received sonde tracks against every witness sight-line at the
   sighting instant — the aircraft pattern: worst witness governs, and a
   timed witness track upgrades the verdict to a trajectory match.
   sondes: [{serial, type, track: [[dtSec, lat, lon, altM], ...]}] with dt
   relative to t0Ms. */
export function rankSondes(sources, sondes, t0Ms) {
  const wit = sources.filter((s) => isNum(s.lat) && isNum(s.lon) && isNum(s.A?.az) && isNum(s.A?.el));
  if (!wit.length) return [];
  const out = [];
  for (const sd of sondes || []) {
    const st = trailStateAt(sd.track, 0) ||
      (Array.isArray(sd.track) && sd.track.length ? { lat: sd.track[0][1], lon: sd.track[0][2], altM: sd.track[0][3] } : null);
    if (!st) continue;
    const per = wit.map((w) => {
      const g = acAzElRange(w, st);
      const d1 = Math.sin((+w.A.el) * D2R) * Math.sin(g.el * D2R) +
        Math.cos((+w.A.el) * D2R) * Math.cos(g.el * D2R) * Math.cos((+w.A.az - g.az) * D2R);
      const sep = Math.acos(Math.min(1, Math.max(-1, d1))) * R2D;
      const dia = balloonDiaM(st.altM);
      return { name: w.name, az: g.az, el: g.el, rangeM: g.rangeM, sep, predAng: 2 * Math.atan(dia / 2 / g.rangeM) * R2D };
    });
    const cand = {
      serial: sd.serial, type: sd.type || null, altM: st.altM,
      per, sepMax: Math.max(...per.map((p) => p.sep)),
      rangeM: per[0].rangeM, predAng: per[0].predAng,
    };
    /* trajectory match when a witness carries a timed track */
    const tms = wit.map((w) => {
      const samples = trackAbsSamples(w);
      return samples.length >= 3 ? trackMatch(w, sd.track, t0Ms, samples) : null;
    }).filter(Boolean);
    if (tms.length) cand.tm = { worstMean: Math.max(...tms.map((r) => r.meanSep)), n: tms.reduce((a, r) => a + r.n, 0), overlapS: Math.max(...tms.map((r) => r.overlapS)) };
    out.push(cand);
  }
  out.sort((a, b) => (a.tm ? a.tm.worstMean : a.sepMax + 999) - (b.tm ? b.tm.worstMean : b.sepMax + 999));
  return out;
}

export async function fetchSondeSites(lat, lon) {
  const r = await fetch(`/api/sondesites?lat=${lat.toFixed(4)}&lon=${lon.toFixed(4)}`, { signal: AbortSignal.timeout(20000) });
  if (!r.ok) throw new Error(`sondesites HTTP ${r.status}`);
  const j = await r.json();
  if (j.error) throw new Error(j.error);
  return j.sites || [];
}

export async function fetchSondes(lat, lon, tMs, km = 250) {
  const r = await fetch(`/api/sondes?lat=${lat.toFixed(4)}&lon=${lon.toFixed(4)}&t=${Math.round(tMs)}&km=${Math.round(km)}`, { signal: AbortSignal.timeout(45000) });
  if (!r.ok) throw new Error(`sondes HTTP ${r.status}`);
  const j = await r.json();
  if (j.error) throw new Error(j.error);
  return j;
}
