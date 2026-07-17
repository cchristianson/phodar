/* ============================================================
   WINDS-ALOFT CHECK — the balloon signature test (CLAUDE.md roadmap).
   A drifting balloon moves WITH the wind at its altitude: same heading,
   same speed. Open-Meteo serves pressure-level winds (no key, CORS-open,
   probed 2026-07-14) for past dates: the forecast API covers roughly the
   last three months; the ERA5 archive covers everything older.
   ============================================================ */

/* ISA pressure levels → approximate geometric altitude (m MSL) */
const LEVELS = [
  [1000, 110], [925, 760], [850, 1460], [700, 3010], [600, 4200],
  [500, 5570], [400, 7180], [300, 9160], [250, 10360], [200, 11780],
];

export function nearestLevel(altM) {
  let best = LEVELS[0];
  for (const L of LEVELS) if (Math.abs(L[1] - altM) < Math.abs(best[1] - altM)) best = L;
  return best;
}

/* wind at (lat, lon, ms, altitude MSL) → {speedMs, fromDeg, driftDeg, hPa, levelM, src} */
export async function fetchWindAt(lat, lon, ms, altM) {
  const [hPa, levelM] = nearestLevel(altM);
  const d = new Date(ms);
  const day = `${d.getUTCFullYear()}-${String(d.getUTCMonth() + 1).padStart(2, "0")}-${String(d.getUTCDate()).padStart(2, "0")}`;
  const hour = d.getUTCHours();
  const q = `latitude=${lat.toFixed(3)}&longitude=${lon.toFixed(3)}&hourly=wind_speed_${hPa}hPa,wind_direction_${hPa}hPa&start_date=${day}&end_date=${day}&wind_speed_unit=ms&timezone=UTC`;
  const hosts = [
    ["https://api.open-meteo.com/v1/forecast", "open-meteo forecast"],
    ["https://archive-api.open-meteo.com/v1/archive", "open-meteo ERA5 archive"],
  ];
  let lastErr = null;
  for (const [host, name] of hosts) {
    try {
      const r = await fetch(`${host}?${q}`, { signal: AbortSignal.timeout(12000) });
      const j = await r.json();
      if (j.error) { lastErr = new Error(j.reason); continue; }
      const sp = j.hourly?.[`wind_speed_${hPa}hPa`]?.[hour];
      const dir = j.hourly?.[`wind_direction_${hPa}hPa`]?.[hour];
      if (sp == null || dir == null) { lastErr = new Error("no data for that hour"); continue; }
      return { speedMs: +sp, fromDeg: +dir, driftDeg: ((+dir + 180) % 360), hPa, levelM, src: name };
    } catch (e) { lastErr = e; }
  }
  throw lastErr || new Error("wind sources unreachable");
}

/* full vertical wind profile at (lat, lon, ms) — ALL pressure levels in one
   call — for the sky-view wind-aloft overlay (balloon-drift visualization).
   Returns [{ hPa, levelM, speedMs, fromDeg, driftDeg }] surface→top, plus src. */
export async function fetchWindProfile(lat, lon, ms) {
  const d = new Date(ms);
  const day = `${d.getUTCFullYear()}-${String(d.getUTCMonth() + 1).padStart(2, "0")}-${String(d.getUTCDate()).padStart(2, "0")}`;
  const hour = d.getUTCHours();
  const vars = LEVELS.map(([hPa]) => `wind_speed_${hPa}hPa,wind_direction_${hPa}hPa`).join(",");
  // via the server proxy (/api/winds) — browser-direct to the ERA5 archive host
  // that OLD sightings need was flaky (CORS / slow warm-up → "Load failed").
  const q = `latitude=${lat.toFixed(3)}&longitude=${lon.toFixed(3)}&hourly=${vars}&start_date=${day}&end_date=${day}&wind_speed_unit=ms&timezone=UTC`;
  const r = await fetch(`/api/winds?${q}`, { signal: AbortSignal.timeout(35000) });
  const j = await r.json().catch(() => null);
  if (!r.ok || !j) throw new Error(j?.error || `winds HTTP ${r.status}`);
  if (j.error) throw new Error(j.reason || j.error);
  const out = [];
  for (const [hPa, levelM] of LEVELS) {
    const sp = j.hourly?.[`wind_speed_${hPa}hPa`]?.[hour];
    const dir = j.hourly?.[`wind_direction_${hPa}hPa`]?.[hour];
    if (sp == null || dir == null) continue;
    out.push({ hPa, levelM, speedMs: +sp, fromDeg: +dir, driftDeg: ((+dir + 180) % 360) });
  }
  if (out.length) return { levels: out, src: j._src || "open-meteo" };
  throw new Error("no wind data for that hour");
}

/* compare an object's motion to balloon drift at that altitude */
export function balloonVerdict(objSpeedMs, objHeadingDeg, wind) {
  const dHead = Math.abs(((objHeadingDeg - wind.driftDeg + 540) % 360) - 180);
  const ratio = wind.speedMs > 0.3 ? objSpeedMs / wind.speedMs : Infinity;
  if (dHead <= 25 && ratio >= 0.5 && ratio <= 1.6) return { verdict: "balloon-consistent", dHead, ratio };
  if (dHead <= 40 && ratio >= 0.3 && ratio <= 2.5) return { verdict: "partially wind-like", dHead, ratio };
  return { verdict: "not wind-borne", dHead, ratio };
}
