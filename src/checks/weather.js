/* ============================================================
   WEATHER / CLOUD conditions at the sighting time — Open-Meteo (the same host
   the winds check uses, reached through the /api/winds proxy which is a generic
   Open-Meteo forwarder: forecast host for recent dates, ERA5 archive beyond).

   Why it earns a place in the report:
   - CLOUD BASE is a hard altitude bound. If the object was clearly BELOW the
     deck, its height above the observer < cloud base, which for a single
     witness (known sight-line elevation) caps its RANGE — and therefore its
     SIZE. If it was plainly ABOVE the deck, that's a floor instead. Either way
     it narrows the otherwise-unbounded size↔distance for one photo.
   - Visibility/haze contextualises brightness and how far the object could be.

   Open-Meteo has no direct cloud-base field on the free API, so we estimate it
   from surface temperature and dew point via Espy's rule (the lifted
   condensation level): base AGL ≈ 125 · (T − Td) metres, T,Td in °C. This is
   the standard back-of-envelope cloud-base estimate; we label it as such.
   ============================================================ */

const WX_VARS = [
  "cloud_cover", "cloud_cover_low", "cloud_cover_mid", "cloud_cover_high",
  "visibility", "temperature_2m", "relative_humidity_2m", "dew_point_2m",
];

/* Espy's estimate of cloud base height above ground (metres) from surface
   temperature and dew point (°C). Pure — asserted in mathcheck. */
export function cloudBaseAGL(tempC, dewC) {
  if (tempC == null || dewC == null) return null;
  return Math.max(0, 125 * (tempC - dewC));
}

/* Cloud base → range/size cap for a SINGLE witness whose sight-line elevation
   is elDeg and whose object subtends angSizeDeg. If the object was below the
   deck, its height above the observer h = range·sin(el) < baseAGL, so
   range < baseAGL / sin(el), and size < 2·range·tan(ang/2). Pure — asserted.
   Returns null when the geometry can't bound it (looking at/below horizon). */
export function cloudRangeBound(baseAGL, elDeg, angSizeDeg) {
  if (baseAGL == null || elDeg == null || elDeg <= 0.5) return null;
  const maxRange = baseAGL / Math.sin(elDeg * Math.PI / 180);
  const maxSize = (angSizeDeg != null && angSizeDeg > 0)
    ? 2 * maxRange * Math.tan((angSizeDeg * Math.PI / 180) / 2)
    : null;
  return { maxRange, maxSize };
}

/* Fetch the weather snapshot at (lat, lon, ms). Returns null-ish fields when a
   variable is missing; throws on transport/no-data so the caller can omit the
   section rather than guess. */
export async function fetchWeatherAt(lat, lon, ms) {
  const d = new Date(ms);
  const day = `${d.getUTCFullYear()}-${String(d.getUTCMonth() + 1).padStart(2, "0")}-${String(d.getUTCDate()).padStart(2, "0")}`;
  const hour = d.getUTCHours();
  const q = `latitude=${lat.toFixed(3)}&longitude=${lon.toFixed(3)}&hourly=${WX_VARS.join(",")}&start_date=${day}&end_date=${day}&timezone=UTC`;
  const r = await fetch(`/api/winds?${q}`, { signal: AbortSignal.timeout(30000) });
  const j = await r.json().catch(() => null);
  if (!r.ok || !j) throw new Error(j?.error || `weather HTTP ${r.status}`);
  if (j.error) throw new Error(j.reason || j.error);
  const H = j.hourly || {};
  const at = (k) => { const a = H[k]; return a && a[hour] != null ? +a[hour] : null; };
  const tempC = at("temperature_2m"), dewC = at("dew_point_2m");
  const cloud = at("cloud_cover");
  if (cloud == null && tempC == null) throw new Error("no weather for that hour");
  return {
    cloud, low: at("cloud_cover_low"), mid: at("cloud_cover_mid"), high: at("cloud_cover_high"),
    visM: at("visibility"), tempC, dewC, rh: at("relative_humidity_2m"),
    baseAGL: cloudBaseAGL(tempC, dewC),
    src: j._src || "open-meteo",
  };
}
