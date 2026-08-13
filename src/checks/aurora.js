/* ============================================================
   AURORA CHECK — geomagnetic activity (GFZ Kp index, CC-BY, full
   history) vs the observer's GEOMAGNETIC latitude. A storm pushes the
   auroral oval equatorward; a shifting red/green glow low on the
   poleward horizon is a classic "strange lights" report during storms,
   from latitudes that never otherwise see it.

   Rule of thumb encoded (documented, approximate): the oval's
   equatorward edge sits near geomagnetic latitude 67° − 2·Kp for the
   overhead display, and the glow is visible low on the horizon several
   degrees equatorward of that. GFZ has no CORS → /api/kp proxy.
   ============================================================ */

import { D2R, R2D } from "../math/geodesy.js";
import { isNum } from "../math/format.js";

/* centered-dipole north pole ≈ 2025 (IGRF-13); the check needs ~1° truth */
const POLE = { lat: 80.9, lon: -72.6 };

export function geomagLat(lat, lon) {
  const s = Math.sin(lat * D2R) * Math.sin(POLE.lat * D2R) +
    Math.cos(lat * D2R) * Math.cos(POLE.lat * D2R) * Math.cos((lon - POLE.lon) * D2R);
  return Math.asin(Math.max(-1, Math.min(1, s))) * R2D;
}

export function auroraVerdict(kp, gmLat) {
  if (!isNum(kp) || !isNum(gmLat)) return null;
  const a = Math.abs(gmLat);
  const overhead = 67 - 2 * kp;   // equatorward oval edge (approx)
  const horizon = overhead - 7;   // low-on-the-horizon glow reaches further south
  const level = a >= overhead ? "overhead" : a >= horizon ? "horizon" : "unlikely";
  return { level, kp, gmLat: a, overhead, horizon };
}

/* GFZ json { Kp: [...], datetime: [...] } → the 3-hour bin covering tMs */
export function kpAt(json, tMs) {
  const ks = json?.Kp, ds = json?.datetime;
  if (!Array.isArray(ks) || !Array.isArray(ds)) return null;
  let best = null;
  for (let i = 0; i < ks.length; i++) {
    const t0 = Date.parse(ds[i]);
    if (!isFinite(t0) || !isNum(ks[i])) continue;
    if (tMs >= t0 && tMs < t0 + 3 * 3600000) return ks[i];
    if (best == null || Math.abs(tMs - t0) < best.d) best = { d: Math.abs(tMs - t0), kp: ks[i] };
  }
  return best && best.d < 6 * 3600000 ? best.kp : null;
}

export async function fetchKpAt(tMs) {
  const r = await fetch(`/api/kp?t=${Math.round(tMs)}`, { signal: AbortSignal.timeout(20000) });
  if (!r.ok) throw new Error(`kp HTTP ${r.status}`);
  const j = await r.json();
  if (j.error) throw new Error(j.error);
  return kpAt(j, tMs);
}
