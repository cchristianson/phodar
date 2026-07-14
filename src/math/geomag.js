/* ============================================================
   GEOMAGNETIC DECLINATION — World Magnetic Model 2025.
   Phone compasses read MAGNETIC north; the triangulation needs TRUE.
   EXIF azRef="M" bearings get +declination before use — up to
   ±25° depending on where on Earth the witness stood, i.e. bigger
   than everything else in the error budget combined.

   Coefficients: official WMM2025.COF (NOAA/NCEI + BGS, released
   2024-11-13, valid through 2029). Algorithm adapted from the NOAA
   geomagc C code via Christopher Weiss's geomagJS (2012, adapted from
   http://www.ngdc.noaa.gov/geomag/WMM/DoDWMM.shtml). Validated in
   scripts/mathcheck.js against NOAA's official WMM2025 test values.
   ============================================================ */

const EPOCH = 2025.0;
/* [n, m, gnm, hnm, dgnm, dhnm] */
const COF = [
  [1,0,-29351.8,0,12,0],
  [1,1,-1410.8,4545.4,9.7,-21.5],
  [2,0,-2556.6,0,-11.6,0],
  [2,1,2951.1,-3133.6,-5.2,-27.7],
  [2,2,1649.3,-815.1,-8,-12.1],
  [3,0,1361,0,-1.3,0],
  [3,1,-2404.1,-56.6,-4.2,4],
  [3,2,1243.8,237.5,0.4,-0.3],
  [3,3,453.6,-549.5,-15.6,-4.1],
  [4,0,895,0,-1.6,0],
  [4,1,799.5,278.6,-2.4,-1.1],
  [4,2,55.7,-133.9,-6,4.1],
  [4,3,-281.1,212,5.6,1.6],
  [4,4,12.1,-375.6,-7,-4.4],
  [5,0,-233.2,0,0.6,0],
  [5,1,368.9,45.4,1.4,-0.5],
  [5,2,187.2,220.2,0,2.2],
  [5,3,-138.7,-122.9,0.6,0.4],
  [5,4,-142,43,2.2,1.7],
  [5,5,20.9,106.1,0.9,1.9],
  [6,0,64.4,0,-0.2,0],
  [6,1,63.8,-18.4,-0.4,0.3],
  [6,2,76.9,16.8,0.9,-1.6],
  [6,3,-115.7,48.8,1.2,-0.4],
  [6,4,-40.9,-59.8,-0.9,0.9],
  [6,5,14.9,10.9,0.3,0.7],
  [6,6,-60.7,72.7,0.9,0.9],
  [7,0,79.5,0,0,0],
  [7,1,-77,-48.9,-0.1,0.6],
  [7,2,-8.8,-14.4,-0.1,0.5],
  [7,3,59.3,-1,0.5,-0.8],
  [7,4,15.8,23.4,-0.1,0],
  [7,5,2.5,-7.4,-0.8,-1],
  [7,6,-11.1,-25.1,-0.8,0.6],
  [7,7,14.2,-2.3,0.8,-0.2],
  [8,0,23.2,0,-0.1,0],
  [8,1,10.8,7.1,0.2,-0.2],
  [8,2,-17.5,-12.6,0,0.5],
  [8,3,2,11.4,0.5,-0.4],
  [8,4,-21.7,-9.7,-0.1,0.4],
  [8,5,16.9,12.7,0.3,-0.5],
  [8,6,15,0.7,0.2,-0.6],
  [8,7,-16.8,-5.2,0,0.3],
  [8,8,0.9,3.9,0.2,0.2],
  [9,0,4.6,0,0,0],
  [9,1,7.8,-24.8,-0.1,-0.3],
  [9,2,3,12.2,0.1,0.3],
  [9,3,-0.2,8.3,0.3,-0.3],
  [9,4,-2.5,-3.3,-0.3,0.3],
  [9,5,-13.1,-5.2,0,0.2],
  [9,6,2.4,7.2,0.3,-0.1],
  [9,7,8.6,-0.6,-0.1,-0.2],
  [9,8,-8.7,0.8,0.1,0.4],
  [9,9,-12.9,10,-0.1,0.1],
  [10,0,-1.3,0,0.1,0],
  [10,1,-6.4,3.3,0,0],
  [10,2,0.2,0,0.1,0],
  [10,3,2,2.4,0.1,-0.2],
  [10,4,-1,5.3,0,0.1],
  [10,5,-0.6,-9.1,-0.3,-0.1],
  [10,6,-0.9,0.4,0,0.1],
  [10,7,1.5,-4.2,-0.1,0],
  [10,8,0.9,-3.8,-0.1,-0.1],
  [10,9,-2.7,0.9,0,0.2],
  [10,10,-3.9,-9.1,0,0],
  [11,0,2.9,0,0,0],
  [11,1,-1.5,0,0,0],
  [11,2,-2.5,2.9,0,0.1],
  [11,3,2.4,-0.6,0,0],
  [11,4,-0.6,0.2,0,0.1],
  [11,5,-0.1,0.5,-0.1,0],
  [11,6,-0.6,-0.3,0,0],
  [11,7,-0.1,-1.2,0,0.1],
  [11,8,1.1,-1.7,-0.1,0],
  [11,9,-1,-2.9,-0.1,0],
  [11,10,-0.2,-1.8,-0.1,0],
  [11,11,2.6,-2.3,-0.1,0],
  [12,0,-2,0,0,0],
  [12,1,-0.2,-1.3,0,0],
  [12,2,0.3,0.7,0,0],
  [12,3,1.2,1,0,-0.1],
  [12,4,-1.3,-1.4,0,0.1],
  [12,5,0.6,0,0,0],
  [12,6,0.6,0.6,0.1,0],
  [12,7,0.5,-0.1,0,0],
  [12,8,-0.1,0.8,0,0],
  [12,9,-0.4,0.1,0,0],
  [12,10,-0.2,-1,-0.1,0],
  [12,11,-1.3,0.1,0,0],
  [12,12,-0.7,0.2,-0.1,-0.1],
];

const MAXORD = 12;
const zz = () => new Array(13).fill(0);
const c = Array.from({ length: 13 }, zz), cd = Array.from({ length: 13 }, zz);
const snorm = Array.from({ length: 13 }, zz), kk = Array.from({ length: 13 }, zz);
const fn = [0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13], fm = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12];
for (const [n, m, gnm, hnm, dgnm, dhnm] of COF) {
  if (m > n) continue;
  c[m][n] = gnm; cd[m][n] = dgnm;
  if (m !== 0) { c[n][m - 1] = hnm; cd[n][m - 1] = dhnm; }
}
/* Schmidt-normalized → unnormalized Gauss coefficients (one-time) */
snorm[0][0] = 1;
for (let n = 1; n <= MAXORD; n++) {
  snorm[0][n] = snorm[0][n - 1] * (2 * n - 1) / n;
  let j = 2;
  for (let m = 0, D2 = n - m + 1; D2 > 0; D2--, m++) {
    kk[m][n] = ((n - 1) * (n - 1) - m * m) / ((2 * n - 1) * (2 * n - 3));
    if (m > 0) {
      const flnmj = ((n - m + 1) * j) / (n + m);
      snorm[m][n] = snorm[m - 1][n] * Math.sqrt(flnmj);
      j = 1;
      c[n][m - 1] *= snorm[m][n]; cd[n][m - 1] *= snorm[m][n];
    }
    c[m][n] *= snorm[m][n]; cd[m][n] *= snorm[m][n];
  }
}
kk[1][1] = 0;

const A = 6378.137, B = 6356.7523142, RE = 6371.2;
const A2 = A * A, B2 = B * B, C2 = A2 - B2, A4 = A2 * A2, B4 = B2 * B2, C4 = A4 - B4;
const D2R = Math.PI / 180, R2D = 180 / Math.PI;

/* declination (deg, east +) at geodetic lat/lon (deg), altitude m, Date */
export function declination(glat, glon, altM = 0, date = new Date()) {
  const year = date.getUTCFullYear();
  const leap = (year % 400 === 0) || (year % 4 === 0 && year % 100 !== 0);
  const time = year + (date.valueOf() - Date.UTC(year, 0)) / ((leap ? 366 : 365) * 86400000);
  const dt = time - EPOCH;
  const alt = (altM || 0) / 1000;
  const rlat = glat * D2R, rlon = glon * D2R;
  const srlat = Math.sin(rlat), crlat = Math.cos(rlat);
  const srlat2 = srlat * srlat, crlat2 = crlat * crlat;
  const sp = zz(), cp = zz(), pp = zz();
  const p = Array.from({ length: 13 }, zz), dp = Array.from({ length: 13 }, zz);
  const tc = Array.from({ length: 13 }, zz);
  sp[0] = 0; cp[0] = 1; pp[0] = 1; p[0][0] = 1;
  sp[1] = Math.sin(rlon); cp[1] = Math.cos(rlon);
  const q = Math.sqrt(A2 - C2 * srlat2);
  const q1 = alt * q;
  const q2 = ((q1 + A2) / (q1 + B2)) ** 2;
  const ct = srlat / Math.sqrt(q2 * crlat2 + srlat2);
  const st = Math.sqrt(1 - ct * ct);
  const r = Math.sqrt(alt * alt + 2 * q1 + (A4 - C4 * srlat2) / (q * q));
  const d = Math.sqrt(A2 * crlat2 + B2 * srlat2);
  const ca = (alt + d) / r, sa = C2 * crlat * srlat / (r * d);
  for (let m = 2; m <= MAXORD; m++) {
    sp[m] = sp[1] * cp[m - 1] + cp[1] * sp[m - 1];
    cp[m] = cp[1] * cp[m - 1] - sp[1] * sp[m - 1];
  }
  const aor = RE / r;
  let ar = aor * aor, br = 0, bt = 0, bp = 0, bpp = 0;
  for (let n = 1; n <= MAXORD; n++) {
    ar *= aor;
    for (let m = 0, D4 = n + m + 1; D4 > 0; D4--, m++) {
      if (n === m) {
        p[m][n] = st * p[m - 1][n - 1];
        dp[m][n] = st * dp[m - 1][n - 1] + ct * p[m - 1][n - 1];
      } else if (n === 1 && m === 0) {
        p[m][n] = ct * p[m][n - 1];
        dp[m][n] = ct * dp[m][n - 1] - st * p[m][n - 1];
      } else if (n > 1 && n !== m) {
        if (m > n - 2) { p[m][n - 2] = 0; dp[m][n - 2] = 0; }
        p[m][n] = ct * p[m][n - 1] - kk[m][n] * p[m][n - 2];
        dp[m][n] = ct * dp[m][n - 1] - st * p[m][n - 1] - kk[m][n] * dp[m][n - 2];
      }
      tc[m][n] = c[m][n] + dt * cd[m][n];
      if (m !== 0) tc[n][m - 1] = c[n][m - 1] + dt * cd[n][m - 1];
      const par = ar * p[m][n];
      let t1, t2;
      if (m === 0) { t1 = tc[m][n] * cp[m]; t2 = tc[m][n] * sp[m]; }
      else { t1 = tc[m][n] * cp[m] + tc[n][m - 1] * sp[m]; t2 = tc[m][n] * sp[m] - tc[n][m - 1] * cp[m]; }
      bt -= ar * t1 * dp[m][n];
      bp += fm[m] * t2 * par;
      br += fn[n] * t1 * par;
      if (st === 0 && m === 1) {
        pp[n] = n === 1 ? pp[n - 1] : ct * pp[n - 1] - kk[m][n] * pp[n - 2];
        bpp += fm[m] * t2 * ar * pp[n];
      }
    }
  }
  bp = st === 0 ? bpp : bp / st;
  const bx = -bt * ca - br * sa;
  const by = bp;
  return Math.atan2(by, bx) * R2D;
}
