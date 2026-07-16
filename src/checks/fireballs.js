/* ============================================================
   FIREBALL / BOLIDE CORRELATOR — the meteor explanation.
   NASA CNEOS logs bright bolides detected by US Government sensors
   (date, radiated + total impact energy, and often lat/lon/alt/speed).
   A match within minutes and a few hundred km is a strong meteor call.
   Source: https://ssd-api.jpl.nasa.gov/fireball.api (free, no key).
   Routed through the app server proxy (/api/fireballs) for a stable
   same-origin call; parsing/ranking is pure and unit-tested.
   ============================================================ */

import { haversineKm } from "./launches.js";

/* CNEOS {fields, data} → ranked bolides near (lat,lon) at time ms.
   The API returns UTC datetimes and signed magnitude lat/lon with a
   separate hemisphere field ("N"/"S", "E"/"W"). */
export function parseFireballs(jsonBody, lat, lon, ms) {
  const fields = jsonBody?.fields || [];
  const ix = (k) => fields.indexOf(k);
  const iDate = ix("date"), iLat = ix("lat"), iLatD = ix("lat-dir"), iLon = ix("lon"), iLonD = ix("lon-dir"),
    iEn = ix("energy"), iImp = ix("impact-e"), iAlt = ix("alt"), iVel = ix("vel");
  const num = (v) => (v != null && v !== "" && isFinite(+v)) ? +v : null;
  const out = [];
  for (const row of (jsonBody?.data || [])) {
    if (iDate < 0 || !row[iDate]) continue;
    const t = Date.parse(String(row[iDate]).replace(" ", "T") + "Z");
    if (!isFinite(t)) continue;
    let flat = iLat >= 0 ? num(row[iLat]) : null; if (flat != null && row[iLatD] === "S") flat = -flat;
    let flon = iLon >= 0 ? num(row[iLon]) : null; if (flon != null && row[iLonD] === "W") flon = -flon;
    out.push({
      t, dtHours: (t - ms) / 3600000,
      energyKt: iEn >= 0 ? num(row[iEn]) : null,       // radiated energy, kt TNT
      impactKt: iImp >= 0 ? num(row[iImp]) : null,     // total impact energy, kt TNT
      altKm: iAlt >= 0 ? num(row[iAlt]) : null,
      velKmS: iVel >= 0 ? num(row[iVel]) : null,
      lat: flat, lon: flon,
      distKm: (flat != null && flon != null && lat != null && lon != null) ? haversineKm(lat, lon, flat, flon) : null,
    });
  }
  out.sort((a, b) => Math.abs(a.dtHours) - Math.abs(b.dtHours));
  return out;
}

export async function fetchFireballs(lat, lon, ms, backDays = 2, fwdDays = 1) {
  const day = (d) => new Date(d).toISOString().slice(0, 10);
  const r = await fetch(`/api/fireballs?dmin=${day(ms - backDays * 86400000)}&dmax=${day(ms + fwdDays * 86400000)}`, { signal: AbortSignal.timeout(15000) });
  if (!r.ok) throw new Error(`fireballs HTTP ${r.status}`);
  const j = await r.json();
  if (j.error) throw new Error(j.error);
  return parseFireballs(j, lat, lon, ms);
}
