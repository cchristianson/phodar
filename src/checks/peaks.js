/* ============================================================
   NAMED PEAKS on the skyline — OSM Overpass summits/volcanoes near the
   observer, placed by true bearing + curvature/refraction-corrected
   elevation so the label sits on the DEM ridge already drawn in the sky
   view. Doubles as azimuth anchors (a named peak on the horizon is a
   compass check). Overpass CORS is unreliable, so the fetch goes through
   the app server proxy (/api/peaks). Parsing/geometry is pure + tested.

   Elevation matches terrain.js exactly:  atan2(h − eye − d²(1−k)/2R, d),
   k ≈ 0.13 refraction — so peak markers and the ridge line agree.
   ============================================================ */

import { D2R, R2D, RE } from "../math/geodesy.js";

const K_REFR = 0.13, EYE_M = 1.6;

export function bearingDeg(la1, lo1, la2, lo2) {
  const p1 = la1 * D2R, p2 = la2 * D2R, dL = (lo2 - lo1) * D2R;
  const y = Math.sin(dL) * Math.cos(p2);
  const x = Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dL);
  return ((Math.atan2(y, x) * R2D) + 360) % 360;
}
export function distM(la1, lo1, la2, lo2) {
  const p1 = la1 * D2R, p2 = la2 * D2R, dp = (la2 - la1) * D2R, dL = (lo2 - lo1) * D2R;
  const a = Math.sin(dp / 2) ** 2 + Math.cos(p1) * Math.cos(p2) * Math.sin(dL / 2) ** 2;
  return 2 * RE * Math.asin(Math.min(1, Math.sqrt(a)));
}

/* Overpass elements → peaks with (az, el, distKm), nearest first.
   el is null when the summit has no `ele` tag (draw it on the ridge line). */
export function parsePeaks(jsonBody, obsLat, obsLon, obsEleM, maxKm = 40) {
  const eye = (obsEleM || 0) + EYE_M;
  const out = [];
  for (const el of (jsonBody?.elements || [])) {
    if (el.lat == null || el.lon == null || !el.tags?.name) continue;
    const d = distM(obsLat, obsLon, el.lat, el.lon);
    if (d < 200 || d > maxKm * 1000) continue;
    const eleRaw = el.tags.ele != null ? parseFloat(String(el.tags.ele).replace(/[^0-9.\-]/g, "")) : NaN;
    const eleM = isFinite(eleRaw) ? eleRaw : null;
    out.push({
      name: el.tags.name,
      lat: el.lat, lon: el.lon, eleM,
      distKm: d / 1000,
      az: bearingDeg(obsLat, obsLon, el.lat, el.lon),
      el: eleM != null ? Math.atan2(eleM - eye - (d * d * (1 - K_REFR)) / (2 * RE), d) * R2D : null,
    });
  }
  return out.sort((a, b) => a.distKm - b.distKm);
}

export async function fetchPeaks(lat, lon, obsEleM, radiusKm = 40) {
  const r = await fetch(`/api/peaks?lat=${lat.toFixed(4)}&lon=${lon.toFixed(4)}&r=${Math.round(radiusKm * 1000)}`, { signal: AbortSignal.timeout(30000) });
  if (!r.ok) throw new Error(`peaks HTTP ${r.status}`);
  const j = await r.json();
  if (j.error) throw new Error(j.error);
  return parsePeaks(j, lat, lon, obsEleM, radiusKm);
}
