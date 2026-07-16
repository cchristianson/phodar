/* ============================================================
   SATELLITE CHECK — the night ADS-B (CLAUDE.md calibration roadmap).
   ISS passes and Starlink trains are a huge share of night reports.
   Data: CelesTrak "visual" group TLEs (~160 brightest satellites,
   CORS-open, no key, probed 2026-07-14), propagated client-side with
   satellite.js (SGP4).

   HONESTY NOTE: TLEs describe the orbit near their epoch. SGP4 drifts
   ~1–3 km/day, so a sighting more than ~5 days from the TLE epoch gets
   an explicit staleness warning — and CelesTrak only serves CURRENT
   elements, so old sightings compare against today's orbit geometry
   (pass timing shifts, the orbital planes mostly don't).

   Output shapes match the aircraft check: positions {az, el, rangeKm,
   lit} and trails [{dt, az, el}] so the dome and report reuse the
   same rendering and ranking patterns.
   ============================================================ */

import * as sat from "satellite.js";
import { D2R, R2D } from "../math/geodesy.js";

const CACHE_MS = 6 * 3600 * 1000;
/* CelesTrak GROUPs. "visual" = ~160 brightest (default). "starlink" = the full
   constellation (~7k) — opt-in, heavier: one SGP4 per sat, filtered to lit +
   above the horizon and capped before drawing. Both CORS-open, no key. */
const GROUPS = {
  visual: "phodar-tle-visual",
  starlink: "phodar-tle-starlink",
};
const groupUrl = (g) => `https://celestrak.org/NORAD/elements/gp.php?GROUP=${g}&FORMAT=tle`;

const groupP = {};
export async function loadSatGroup(group = "visual") {
  if (groupP[group]) return groupP[group];
  const cacheKey = GROUPS[group];
  if (!cacheKey) throw new Error("unknown satellite group: " + group);
  groupP[group] = (async () => {
    let text = null, fetchedAt = 0;
    try {
      const c = JSON.parse(localStorage.getItem(cacheKey) || "null");
      if (c && Date.now() - c.t < CACHE_MS) { text = c.text; fetchedAt = c.t; }
    } catch (e) { }
    if (!text) {
      const r = await fetch(groupUrl(group), { signal: AbortSignal.timeout(20000) });
      if (!r.ok) throw new Error(`TLE fetch HTTP ${r.status}`);
      text = await r.text();
      fetchedAt = Date.now();
      try { localStorage.setItem(cacheKey, JSON.stringify({ t: fetchedAt, text })); } catch (e) { /* the starlink set may exceed the quota — fine, just refetch next time */ }
    }
    const lines = text.split(/\r?\n/);
    const out = [];
    for (let i = 0; i + 2 < lines.length + 1; i++) {
      if (lines[i + 1]?.startsWith("1 ") && lines[i + 2]?.startsWith("2 ")) {
        const rec = sat.twoline2satrec(lines[i + 1], lines[i + 2]);
        if (rec.error === 0) out.push({ name: lines[i].trim(), rec });
        i += 2;
      }
    }
    return { sats: out, fetchedAt };
  })();
  groupP[group].catch(() => { groupP[group] = null; });
  return groupP[group];
}
export const loadSats = () => loadSatGroup("visual");

/* sun unit vector in ECI (TEME ≈ ECI at this accuracy) for the shadow test */
function sunEci(date) {
  const d = date.valueOf() / 86400000 - 0.5 + 2440588 - 2451545;
  const M = (357.5291 + 0.98560028 * d) * D2R;
  const L = M + (1.9148 * Math.sin(M) + 0.02 * Math.sin(2 * M)) * D2R + (102.9372 + 180) * D2R;
  const e = 23.4393 * D2R;
  const x = Math.cos(L), y = Math.sin(L) * Math.cos(e), z = Math.sin(L) * Math.sin(e);
  return [x, y, z];
}

/* cylindrical Earth-shadow: eclipsed iff behind the terminator plane AND
   inside the shadow cylinder */
function isLit(posEci, sun) {
  const r = [posEci.x, posEci.y, posEci.z];
  const rs = r[0] * sun[0] + r[1] * sun[1] + r[2] * sun[2];
  if (rs >= 0) return true;
  const r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
  const perp = Math.sqrt(Math.max(0, r2 - rs * rs));
  return perp > 6371;
}

function lookFrom(rec, date, gd) {
  const pv = sat.propagate(rec, date);
  if (!pv || !pv.position || typeof pv.position !== "object") return null;
  const gmst = sat.gstime(date);
  const ecf = sat.eciToEcf(pv.position, gmst);
  const la = sat.ecfToLookAngles(gd, ecf);
  return {
    az: ((la.azimuth * R2D) % 360 + 360) % 360,
    el: la.elevation * R2D,
    rangeKm: la.rangeSat,
    lit: isLit(pv.position, sunEci(date)),
  };
}

/* all visual-group satellites above elMin at ms, from (lat, lon) */
export function satsAt(sats, ms, lat, lon, elMin = -2) {
  const date = new Date(ms);
  const gd = { latitude: lat * D2R, longitude: lon * D2R, height: 0 };
  const out = [];
  for (const s of sats) {
    const p = lookFrom(s.rec, date, gd);
    if (!p || p.el < elMin) continue;
    const epochMs = s.rec.jdsatepoch ? (s.rec.jdsatepoch - 2440587.5) * 86400000 : null;
    out.push({ name: s.name, ...p, epochAgeDays: epochMs != null ? Math.abs(ms - epochMs) / 86400000 : null, rec: s.rec });
  }
  return out.sort((a, b) => b.el - a.el);
}

/* ±winS trail as [{dt, az, el}] — same drawing shape as aircraft trails */
export function satTrail(rec, ms, lat, lon, winS = 240, stepS = 10) {
  const gd = { latitude: lat * D2R, longitude: lon * D2R, height: 0 };
  const out = [];
  for (let dt = -winS; dt <= winS; dt += stepS) {
    const p = lookFrom(rec, new Date(ms + dt * 1000), gd);
    if (p) out.push({ dt, az: p.az, el: p.el });
  }
  return out;
}
