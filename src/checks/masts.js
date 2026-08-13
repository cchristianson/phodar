/* ============================================================
   TALL LIT STRUCTURES near the observer — OSM masts, towers, chimneys,
   lighthouses. A red or white obstruction strobe on a distant mast is a
   classic "hovering pulsing light" report; ranking them against the
   sight-line gives the mundane candidate a name, a height and a distance.
   Fetch goes through the app-server proxy (/api/masts — same Overpass CORS
   reality as /api/peaks); parsing/ranking is pure + mathcheck-tested.
   Heights reuse the buildings' tag parser ({m, est} or null when untagged).
   ============================================================ */

import { heightMeters } from "../buildings.js";
import { bearingDeg, distM } from "./peaks.js";

const KINDS = /^(mast|tower|communications_tower|chimney|lighthouse)$/;

export function parseMasts(jsonBody, obsLat, obsLon, maxKm = 25) {
  const out = [];
  for (const el of (jsonBody?.elements || [])) {
    const la = el.lat ?? el.center?.lat, lo = el.lon ?? el.center?.lon, t = el.tags || {};
    if (la == null || lo == null || !KINDS.test(t.man_made || "")) continue;
    const d = distM(obsLat, obsLon, la, lo);
    if (d < 100 || d > maxKm * 1000) continue;
    const h = heightMeters(t);
    out.push({
      name: t.name || null, kind: t.man_made,
      lat: la, lon: lo, hM: h ? h.m : null, hEst: h ? h.est : true,
      distKm: d / 1000, az: bearingDeg(obsLat, obsLon, la, lo),
    });
  }
  return out.sort((a, b) => a.distKm - b.distKm);
}

/* structures within sepDeg of a sight-line azimuth, closest bearing first */
export function mastsNear(masts, azDeg, sepDeg = 5) {
  const wrap = (x) => ((x % 360) + 540) % 360 - 180;
  return (masts || [])
    .map((m) => ({ ...m, dAz: wrap(m.az - azDeg) }))
    .filter((m) => Math.abs(m.dAz) <= sepDeg)
    .sort((a, b) => Math.abs(a.dAz) - Math.abs(b.dAz));
}

export async function fetchMasts(lat, lon, radiusKm = 25) {
  const r = await fetch(`/api/masts?lat=${lat.toFixed(4)}&lon=${lon.toFixed(4)}&r=${Math.round(radiusKm * 1000)}`, { signal: AbortSignal.timeout(30000) });
  if (!r.ok) throw new Error(`masts HTTP ${r.status}`);
  const j = await r.json();
  if (j.error) throw new Error(j.error);
  return parseMasts(j, lat, lon, radiusKm);
}
