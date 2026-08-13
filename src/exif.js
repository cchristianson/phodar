import { R2D } from "./math/geodesy.js";

/* ============================================================
   MEDIA METADATA — minimal EXIF/QuickTime readers (no libraries)
   Pulls GPS position/altitude, capture time, camera compass
   bearing (GPSImgDirection), and FOV from the 35 mm-equivalent
   focal length. One tap applies them to the observer.
   ============================================================ */
function parseJpegExif(u8) {
  if (u8[0] !== 0xFF || u8[1] !== 0xD8) return null;
  let o = 2;
  while (o + 4 < u8.length) {
    if (u8[o] !== 0xFF) break;
    const marker = u8[o + 1], size = (u8[o + 2] << 8) | u8[o + 3];
    if (marker === 0xE1 && u8[o + 4] === 0x45 && u8[o + 5] === 0x78 && u8[o + 6] === 0x69 && u8[o + 7] === 0x66) {
      return parseTiff(u8, o + 10);
    }
    if (marker === 0xDA) break;
    o += 2 + size;
  }
  return null;
}
function parseTiff(u8, base) {
  const le = u8[base] === 0x49;
  const u16 = (p) => le ? (u8[p] | (u8[p + 1] << 8)) : ((u8[p] << 8) | u8[p + 1]);
  const u32 = (p) => (le ? (u8[p] | (u8[p + 1] << 8) | (u8[p + 2] << 16) | (u8[p + 3] << 24)) : ((u8[p] << 24) | (u8[p + 1] << 16) | (u8[p + 2] << 8) | u8[p + 3])) >>> 0;
  const rat = (p) => { const n = u32(p), d = u32(p + 4); return d ? n / d : 0; };
  const ascii = (p, n) => { let s = ""; for (let i = 0; i < n && u8[p + i]; i++) s += String.fromCharCode(u8[p + i]); return s; };
  const SZ = [0, 1, 1, 2, 4, 8, 1, 1, 2, 4, 8, 4, 8];
  const walk = (off, cb) => {
    const n = u16(base + off);
    for (let i = 0; i < n; i++) {
      const e = base + off + 2 + i * 12;
      const tag = u16(e), type = u16(e + 2), cnt = u32(e + 4);
      const vsz = (SZ[type] || 1) * cnt;
      const vo = vsz <= 4 ? e + 8 : base + u32(e + 8);
      cb(tag, type, cnt, vo);
    }
  };
  const out = {};
  let exifOff = 0, gpsOff = 0, orient = 1;
  walk(u32(base + 4), (tag, type, cnt, vo) => {
    if (tag === 0x8769) exifOff = u32(vo);
    if (tag === 0x8825) gpsOff = u32(vo);
    if (tag === 0x0112) orient = u16(vo);
    if (tag === 0x0110) out.model = ascii(vo, cnt).trim();
  });
  if (exifOff) walk(exifOff, (tag, type, cnt, vo) => {
    if ((tag === 0x9003 || tag === 0x0132) && !out.timeMs) {
      const m = ascii(vo, cnt).match(/(\d{4}):(\d{2}):(\d{2})[ T](\d{2}):(\d{2}):(\d{2})/);
      if (m) out.timeMs = new Date(+m[1], +m[2] - 1, +m[3], +m[4], +m[5], +m[6]).getTime();
    }
    if (tag === 0xA405) {
      const f35 = type === 3 ? u16(vo) : u32(vo);
      if (f35 > 0) {
        const half = (orient === 6 || orient === 8) ? 12 : 18; // portrait uses the 24 mm side
        out.fovH = +(2 * Math.atan(half / f35) * R2D).toFixed(1);
        out.f35 = f35;
      }
    }
    /* close-subject tells (authenticity): where the camera itself says its
       subject was. 0xFFFFFFFF numerator = infinity, 0 = unknown — skip both. */
    if (tag === 0x9206 && type === 5) { const n = u32(vo), d = u32(vo + 4); if (d && n && n !== 0xFFFFFFFF) out.subjDist = +(n / d).toFixed(2); }
    if (tag === 0x9209) out.flash = u16(vo);
    if (tag === 0xA40C) out.subjRange = u16(vo); // 1 macro · 2 close · 3 distant
  });
  if (gpsOff) {
    let latR, lat, lonR, lon, altR = 0, alt, dirRef, dir;
    walk(gpsOff, (tag, type, cnt, vo) => {
      if (tag === 1) latR = ascii(vo, cnt);
      if (tag === 2) lat = rat(vo) + rat(vo + 8) / 60 + rat(vo + 16) / 3600;
      if (tag === 3) lonR = ascii(vo, cnt);
      if (tag === 4) lon = rat(vo) + rat(vo + 8) / 60 + rat(vo + 16) / 3600;
      if (tag === 5) altR = u8[vo];
      if (tag === 6) alt = rat(vo);
      if (tag === 16) dirRef = ascii(vo, cnt);
      if (tag === 17) dir = rat(vo);
    });
    if (lat != null && lon != null && (lat || lon)) {
      out.lat = +((latR === "S" ? -lat : lat)).toFixed(6);
      out.lon = +((lonR === "W" ? -lon : lon)).toFixed(6);
    }
    if (alt != null) out.alt = +((altR === 1 ? -alt : alt)).toFixed(1);
    if (dir != null && isFinite(dir)) { out.az = +dir.toFixed(1); out.azRef = dirRef === "M" ? "magnetic" : "true"; }
  }
  return Object.keys(out).length ? out : null;
}
/* QuickTime metadata (GPS as ISO-6709, creation time in `mvhd`) lives in the
   `moov` atom — and an iPhone writes `moov` at the END of the file, not the
   start (no faststart). Scanning only the head therefore missed the position
   and timestamp of EVERY clip bigger than the window: measured on a 27 MB
   field recording, `moov` began at byte 26,968,202 and the location string sat
   in the last 200 bytes. So scan a head window AND a tail window; a
   faststart/streaming file is still caught by the head. Offsets are tracked
   per window because the byte reads below index the ORIGINAL array. */
function parseMovMeta(u8) {
  const out = {};
  const HEAD = 3000000, TAIL = 8000000;
  const dec = new TextDecoder("latin1");
  const wins = [{ off: 0, txt: dec.decode(u8.subarray(0, Math.min(u8.length, HEAD))) }];
  if (u8.length > HEAD) {
    const off = Math.max(HEAD, u8.length - TAIL);
    wins.push({ off, txt: dec.decode(u8.subarray(off)) });
  }
  for (const w of wins) {
    if (out.lat == null) {
      const m = w.txt.match(/([+-]\d{1,2}\.\d{2,})([+-]\d{1,3}\.\d{2,})([+-]\d+(\.\d+)?)?\//);
      if (m) { out.lat = +(+m[1]).toFixed(6); out.lon = +(+m[2]).toFixed(6); if (m[3]) out.alt = +(+m[3]).toFixed(1); }
    }
    if (!out.timeMs) {
      const mi = w.txt.indexOf("mvhd");
      if (mi > 0 && u8[w.off + mi + 4] === 0) {
        const p = w.off + mi + 8; // version(1)+flags(3) then creation u32 (seconds since 1904)
        const sec = ((u8[p] << 24) | (u8[p + 1] << 16) | (u8[p + 2] << 8) | u8[p + 3]) >>> 0;
        if (sec > 2082844800) out.timeMs = (sec - 2082844800) * 1000;
      }
    }
  }
  return Object.keys(out).length ? out : null;
}
export function parseMediaMeta(buf, isVideo) {
  try {
    const u8 = new Uint8Array(buf);
    return isVideo ? parseMovMeta(u8) : parseJpegExif(u8);
  } catch (e) { return null; }
}
