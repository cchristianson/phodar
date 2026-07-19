/* ============================================================
   METEOR SHOWERS — the major annual showers (IMO/AMS), radiant J2000 in
   DEGREES (RA, Dec), peak + active window as [month, day], ZHR and geocentric
   velocity (km/s). A fast streak coming FROM near an above-horizon radiant is a
   strong meteor explanation; the Taurids are fireball-rich (slow, bright).
   Static + tiny — no network, no key. Pure fns asserted in mathcheck.
   ============================================================ */

export const SHOWERS = [
  { name: "Quadrantids", ra: 230, dec: 49, zhr: 120, v: 41, peak: [1, 3], from: [12, 28], to: [1, 12] },
  { name: "Lyrids", ra: 271, dec: 34, zhr: 18, v: 49, peak: [4, 22], from: [4, 16], to: [4, 25] },
  { name: "Eta Aquariids", ra: 338, dec: -1, zhr: 50, v: 66, peak: [5, 6], from: [4, 19], to: [5, 28] },
  { name: "Southern δ Aquariids", ra: 340, dec: -16, zhr: 25, v: 41, peak: [7, 30], from: [7, 12], to: [8, 23] },
  { name: "Perseids", ra: 48, dec: 58, zhr: 100, v: 59, peak: [8, 12], from: [7, 17], to: [8, 24] },
  { name: "Orionids", ra: 95, dec: 16, zhr: 20, v: 66, peak: [10, 21], from: [10, 2], to: [11, 7] },
  { name: "Southern Taurids", ra: 52, dec: 13, zhr: 5, v: 27, peak: [11, 5], from: [9, 10], to: [11, 20], fireball: true },
  { name: "Northern Taurids", ra: 58, dec: 22, zhr: 5, v: 29, peak: [11, 12], from: [10, 20], to: [12, 10], fireball: true },
  { name: "Leonids", ra: 152, dec: 22, zhr: 15, v: 71, peak: [11, 17], from: [11, 6], to: [11, 30] },
  { name: "Geminids", ra: 112, dec: 33, zhr: 150, v: 35, peak: [12, 14], from: [12, 4], to: [12, 17] },
  { name: "Ursids", ra: 217, dec: 76, zhr: 10, v: 33, peak: [12, 22], from: [12, 17], to: [12, 26] },
];

const doy = (y, m, d) => Math.round((Date.UTC(y, m - 1, d) - Date.UTC(y, 0, 1)) / 86400000) + 1;

/* Showers active on the sighting date, each with signed days-from-peak, sorted
   nearest-peak first. Windows that wrap the new year (Quadrantids) are handled.
   Pure. */
export function activeShowers(ms) {
  const dt = new Date(ms), y = dt.getUTCFullYear();
  const nd = doy(y, dt.getUTCMonth() + 1, dt.getUTCDate());
  const inWin = (from, to) => {
    const a = doy(y, from[0], from[1]), b = doy(y, to[0], to[1]);
    return a <= b ? (nd >= a && nd <= b) : (nd >= a || nd <= b); // year wrap
  };
  const out = [];
  for (const s of SHOWERS) {
    if (!inWin(s.from, s.to)) continue;
    let dd = nd - doy(y, s.peak[0], s.peak[1]);
    if (dd > 183) dd -= 365; if (dd < -183) dd += 365;
    out.push({ ...s, daysFromPeak: dd });
  }
  out.sort((a, b) => Math.abs(a.daysFromPeak) - Math.abs(b.daysFromPeak));
  return out;
}
