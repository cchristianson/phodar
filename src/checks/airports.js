/* ============================================================
   NEARBY AIRFIELDS — OSM Overpass aerodromes near the fix, for context in the
   ADS-B section ("nearest field 6 km NW — expect approach/departure traffic").
   Fetched through the /api/airports proxy (CORS-unreliable Overpass, same as
   peaks/buildings). parseAirports is pure — asserted in mathcheck.
   ============================================================ */
import { bearingDeg, distM } from "./peaks.js";

export function parseAirports(elements, lat, lon) {
  const out = [];
  for (const el of (elements || [])) {
    const la = el.lat != null ? el.lat : (el.center && el.center.lat);
    const lo = el.lon != null ? el.lon : (el.center && el.center.lon);
    if (la == null || lo == null) continue;
    const t = el.tags || {};
    if (t.aeroway !== "aerodrome") continue;
    out.push({
      name: t.name || t["name:en"] || t.icao || t.iata || "unnamed airfield",
      iata: t.iata || null,
      icao: t.icao || t.faa || null,
      kind: t.military || t.landuse === "military" ? "military" : (t.aerodrome === "international" ? "international" : null),
      distM: distM(lat, lon, la, lo),
      bearing: bearingDeg(lat, lon, la, lo),
    });
  }
  out.sort((a, b) => a.distM - b.distM);
  return out;
}

export async function fetchAirports(lat, lon, r = 40000) {
  const rr = await fetch(`/api/airports?lat=${lat.toFixed(5)}&lon=${lon.toFixed(5)}&r=${Math.round(r)}`, { signal: AbortSignal.timeout(30000) });
  const j = await rr.json().catch(() => null);
  if (!rr.ok || !j) throw new Error(j?.error || `airports HTTP ${rr.status}`);
  return parseAirports(j.elements, lat, lon);
}
