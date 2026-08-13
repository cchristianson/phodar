/* ============================================================
   SKY-LANTERN / FIREWORK DATE CONTEXT — no API, pure calendar. Sky
   lanterns (drifting orange points that flicker and fade) and fireworks
   concentrate hard on a few nights of the year; a night sighting on one
   of those dates earns the context line. Lunar New Year needs a lookup
   table (embedded 2015–2035); the others are fixed dates. Local
   calendar — celebrations follow the observer's clock.
   ============================================================ */

const LNY = {
  2015: [2, 19], 2016: [2, 8], 2017: [1, 28], 2018: [2, 16], 2019: [2, 5],
  2020: [1, 25], 2021: [2, 12], 2022: [2, 1], 2023: [1, 22], 2024: [2, 10],
  2025: [1, 29], 2026: [2, 17], 2027: [2, 6], 2028: [1, 26], 2029: [2, 13],
  2030: [2, 3], 2031: [1, 23], 2032: [2, 11], 2033: [1, 31], 2034: [2, 19],
  2035: [2, 8],
};

const dayDiff = (whenMs, y, m, d) =>
  Math.round((whenMs - new Date(y, m - 1, d, 12).getTime()) / 86400000);

/* → { event, offsetDays } | null */
export function lanternContext(whenMs) {
  if (!isFinite(whenMs)) return null;
  const y = new Date(whenMs).getFullYear();
  for (const yy of [y - 1, y, y + 1]) {
    const d4 = dayDiff(whenMs, yy, 7, 4);
    if (Math.abs(d4) <= 1) return { event: "Independence Day (July 4) fireworks", offsetDays: d4 };
    const dn = dayDiff(whenMs, yy, 12, 31);
    if (dn === 0 || dn === 1) return { event: "New Year's Eve", offsetDays: dn };
    const ln = LNY[yy];
    if (ln) {
      const dl = dayDiff(whenMs, yy, ln[0], ln[1]);
      if (dl >= -1 && dl <= 2) return { event: "Lunar New Year", offsetDays: dl };
    }
  }
  return null;
}
