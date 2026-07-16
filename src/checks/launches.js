/* ============================================================
   ROCKET-LAUNCH CORRELATOR — the "was that a Starlink?" check.
   A fresh Starlink batch is a moving "train" of dots visible for days
   after launch; a twilight launch plume can be seen for hundreds of km.
   Source: The Space Devs Launch Library 2 (free, no key). CORS is not
   guaranteed on ll.thespacedevs.com, so the fetch goes through the app's
   own server proxy (/api/launches → server/index.mjs), like the ADS-B
   archive and map tiles. Parsing/ranking is pure and unit-tested.
   ============================================================ */

const R_KM = 6371;
export function haversineKm(la1, lo1, la2, lo2) {
  const d = Math.PI / 180;
  const dLa = (la2 - la1) * d, dLo = (lo2 - lo1) * d;
  const a = Math.sin(dLa / 2) ** 2 + Math.cos(la1 * d) * Math.cos(la2 * d) * Math.sin(dLo / 2) ** 2;
  return 2 * R_KM * Math.asin(Math.min(1, Math.sqrt(a)));
}

/* Launch Library 2 results → ranked candidates near (lat,lon) at time ms.
   Sorted by |time from sighting| (closest first). */
export function parseLaunches(jsonBody, lat, lon, ms) {
  const out = [];
  for (const r of (jsonBody?.results || [])) {
    const net = r?.net ? Date.parse(r.net) : NaN;
    if (!isFinite(net)) continue;
    const plat = r?.pad?.latitude != null ? +r.pad.latitude : null;
    const plon = r?.pad?.longitude != null ? +r.pad.longitude : null;
    const mission = r?.mission?.name || "";
    const name = r?.name || mission || "launch";
    const rocket = r?.rocket?.configuration?.name || r?.rocket?.configuration?.full_name || "";
    out.push({
      name, rocket, mission,
      net, dtHours: (net - ms) / 3600000,
      padName: r?.pad?.location?.name || r?.pad?.name || "",
      lat: plat, lon: plon,
      distKm: (plat != null && plon != null && lat != null && lon != null) ? haversineKm(lat, lon, plat, plon) : null,
      starlink: /starlink/i.test(mission) || /starlink/i.test(name),
    });
  }
  out.sort((a, b) => Math.abs(a.dtHours) - Math.abs(b.dtHours));
  return out;
}

/* fetch launches in [ms - backDays, ms + fwdDays] via the server proxy */
export async function fetchLaunches(lat, lon, ms, backDays = 14, fwdDays = 1) {
  const net0 = new Date(ms - backDays * 86400000).toISOString();
  const net1 = new Date(ms + fwdDays * 86400000).toISOString();
  const r = await fetch(`/api/launches?net0=${encodeURIComponent(net0)}&net1=${encodeURIComponent(net1)}`, { signal: AbortSignal.timeout(15000) });
  if (!r.ok) throw new Error(`launches HTTP ${r.status}`);
  const j = await r.json();
  if (j.error) throw new Error(j.error);
  return parseLaunches(j, lat, lon, ms);
}
