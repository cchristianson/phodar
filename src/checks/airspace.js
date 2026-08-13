/* ============================================================
   MILITARY / SPECIAL-USE AIRSPACE CHECK — FAA SUA polygons (MOAs,
   Restricted, Prohibited, Alert, Warning areas) around the observer.
   A sighting inside or looking into an active MOA has fast jets, flares,
   refueling tracks and formation lights as heavily-weighted mundane
   candidates — this is where "impossible" lights routinely come from.

   Data: FAA AIS open data (ArcGIS FeatureServer, keyless; probed
   2026-08 — Dolphin North/South MOAs return for the Rogue Valley) via
   the /api/airspace proxy (24 h cache). US airspace only — stated.
   Geometry is pure + mathcheck-asserted: point-in-polygon by ray
   casting, and a sampled march along the sight-line azimuth that finds
   where (if anywhere) the ray's ground track enters each zone.
   ============================================================ */

import { D2R } from "../math/geodesy.js";
import { isNum } from "../math/format.js";

export const SUA_TYPES = {
  MOA: "Military Operations Area", R: "Restricted Area", P: "Prohibited Area",
  A: "Alert Area", W: "Warning Area", D: "Danger Area", NSA: "National Security Area",
};

/* "SFC"/"GND" → 0; FL uom → ×100 ft; plain FT value passes through */
export function parseAltFt(val, uom) {
  if (val == null) return null;
  const s = String(val).trim().toUpperCase();
  if (s === "SFC" || s === "GND" || s === "0") return 0;
  const v = parseFloat(s.replace(/[^0-9.]/g, ""));
  if (!isFinite(v)) return null;
  return String(uom || "").toUpperCase() === "FL" ? v * 100 : v;
}

/* ray-casting point-in-polygon over esri rings ([[ [lon,lat], ... ], ...]);
   even-odd across all rings so holes behave */
export function pointInRings(lat, lon, rings) {
  let inside = false;
  for (const ring of rings || []) {
    for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
      const xi = ring[i][0], yi = ring[i][1], xj = ring[j][0], yj = ring[j][1];
      if (((yi > lat) !== (yj > lat)) && (lon < ((xj - xi) * (lat - yi)) / (yj - yi) + xi)) inside = !inside;
    }
  }
  return inside;
}

/* march the sight-line's ground track outward and report where it first
   enters / leaves the zone (km); null when it never does. Sampled every
   stepKm to maxKm — approximate by construction, plenty for "you were
   looking INTO the MOA". */
export function rayIntoZone(rings, obsLat, obsLon, azDeg, maxKm = 250, stepKm = 2) {
  const mLat = 111.32, mLon = 111.32 * Math.cos(obsLat * D2R);
  const sa = Math.sin(azDeg * D2R), ca = Math.cos(azDeg * D2R);
  let enter = null, exit = null;
  for (let d = stepKm; d <= maxKm; d += stepKm) {
    const la = obsLat + (d * ca) / mLat, lo = obsLon + (d * sa) / mLon;
    const inz = pointInRings(la, lo, rings);
    if (inz && enter == null) enter = d;
    if (!inz && enter != null) { exit = d - stepKm; break; }
  }
  return enter == null ? null : { enterKm: enter, exitKm: exit ?? maxKm };
}

/* ArcGIS query JSON → zones sorted inside-first then by distance */
export function parseSua(json, obsLat, obsLon) {
  const out = [];
  for (const f of json?.features || []) {
    const a = f.attributes || {}, rings = f.geometry?.rings;
    if (!rings || !rings.length) continue;
    const inside = pointInRings(obsLat, obsLon, rings);
    /* distance to the nearest EDGE (a big zone's nearest vertex can be a far
       corner) — flat local approx is fine at these ranges */
    const mLatK = 111.32, mLonK = 111.32 * Math.cos(obsLat * D2R);
    let minKm = Infinity, az = null;
    for (const ring of rings) for (let i = 0; i + 1 < ring.length; i++) {
      const ax = (ring[i][0] - obsLon) * mLonK, ay = (ring[i][1] - obsLat) * mLatK;
      const bx = (ring[i + 1][0] - obsLon) * mLonK, by = (ring[i + 1][1] - obsLat) * mLatK;
      const dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
      const t = L2 > 0 ? Math.max(0, Math.min(1, (-ax * dx - ay * dy) / L2)) : 0;
      const px = ax + t * dx, py = ay + t * dy;
      const d = Math.hypot(px, py);
      if (d < minKm) { minKm = d; az = (Math.atan2(px, py) / D2R + 360) % 360; }
    }
    out.push({
      name: a.NAME || "unnamed", type: a.TYPE_CODE || "?",
      typeLong: SUA_TYPES[a.TYPE_CODE] || a.TYPE_CODE || "special-use airspace",
      floorFt: parseAltFt(a.LOWER_VAL, a.LOWER_UOM), ceilFt: parseAltFt(a.UPPER_VAL, a.UPPER_UOM),
      times: a.TIMESOFUSE || null,
      inside, distKm: inside ? 0 : minKm, az, rings,
    });
  }
  return out.sort((x, y) => (y.inside - x.inside) || (x.distKm - y.distKm));
}

/* a very light schedule read: "0800 - 1600, DAILY" style strings → was the
   sighting's LOCAL clock time inside the window? Anything the pattern can't
   read returns null (unknown) rather than a guess — schedules also change
   by NOTAM, which the caller must say. */
export function suaActiveAt(times, localHHMM) {
  const m = String(times || "").match(/(\d{3,4})\s*-\s*(\d{3,4})/);
  if (!m || !isNum(localHHMM)) return null;
  const a = +m[1], b = +m[2];
  return a <= b ? (localHHMM >= a && localHHMM <= b) : (localHHMM >= a || localHHMM <= b);
}

export async function fetchAirspace(lat, lon, km = 120) {
  const r = await fetch(`/api/airspace?lat=${lat.toFixed(4)}&lon=${lon.toFixed(4)}&km=${Math.round(km)}`, { signal: AbortSignal.timeout(30000) });
  if (!r.ok) throw new Error(`airspace HTTP ${r.status}`);
  const j = await r.json();
  if (j.error) throw new Error(j.error);
  return parseSua(j, lat, lon);
}
