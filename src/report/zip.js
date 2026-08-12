/* STORE-method zip writer + reader — the share-bundle format. Pure (no DOM,
   no node APIs), shared by the browser app and the server's ingest pipeline so
   a server-built bundle is byte-compatible with what the app writes and
   imports. STORE (no compression) is deliberate: the payload is JPEGs and
   H.264, already compressed, and skipping deflate keeps both halves
   dependency-free. Extracted from phodar.jsx (module-split roadmap). */

let _crcT = null;
export function crc32buf(u8) {
  if (!_crcT) {
    _crcT = new Int32Array(256);
    for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320 ^ (c >>> 1) : c >>> 1; _crcT[n] = c; }
  }
  let crc = -1;
  for (let i = 0; i < u8.length; i++) crc = (crc >>> 8) ^ _crcT[(crc ^ u8[i]) & 255];
  return (crc ^ -1) >>> 0;
}

/* files: [{name: Uint8Array, data: Uint8Array}] → one Uint8Array of zip bytes.
   (Returns bytes, not a Blob, so it runs server-side; the app wraps the
   result in a Blob at its call site.) */
export function makeZip(files) {
  const chunks = [], central = [];
  let offset = 0;
  const u16 = (v) => new Uint8Array([v & 255, (v >> 8) & 255]);
  const u32 = (v) => new Uint8Array([v & 255, (v >> 8) & 255, (v >> 16) & 255, (v >>> 24) & 255]);
  for (const f of files) {
    const crc = crc32buf(f.data), sz = f.data.length;
    chunks.push(u32(0x04034b50), u16(20), u16(0), u16(0), u16(0), u16(0), u32(crc), u32(sz), u32(sz), u16(f.name.length), u16(0), f.name, f.data);
    central.push({ name: f.name, crc, sz, offset });
    offset += 30 + f.name.length + sz;
  }
  const cdStart = offset;
  for (const c of central) {
    chunks.push(u32(0x02014b50), u16(20), u16(20), u16(0), u16(0), u16(0), u16(0), u32(c.crc), u32(c.sz), u32(c.sz), u16(c.name.length), u16(0), u16(0), u16(0), u16(0), u32(0), u32(c.offset), c.name);
    offset += 46 + c.name.length;
  }
  chunks.push(u32(0x06054b50), u16(0), u16(0), u16(central.length), u16(central.length), u32(offset - cdStart), u32(cdStart), u16(0));
  let total = 0; for (const c of chunks) total += c.length;
  const out = new Uint8Array(total);
  let o = 0; for (const c of chunks) { out.set(c, o); o += c.length; }
  return out;
}

export const strU8 = (s) => new TextEncoder().encode(s);

/* pull one STORE-method entry's text out of a zip (the reader half of makeZip
   — the share bundle is uncompressed, so no inflate needed). Scans local file
   headers; returns the named entry decoded as UTF-8, or null. */
export function unzipEntryText(u8, name) {
  const u16 = (o) => u8[o] | (u8[o + 1] << 8);
  const u32 = (o) => u8[o] + u8[o + 1] * 256 + u8[o + 2] * 65536 + u8[o + 3] * 16777216;
  let o = 0;
  while (o + 30 <= u8.length && u32(o) === 0x04034b50) {
    const method = u16(o + 8), compSize = u32(o + 18);
    const nameLen = u16(o + 26), extraLen = u16(o + 28);
    const nm = new TextDecoder().decode(u8.subarray(o + 30, o + 30 + nameLen));
    const dataStart = o + 30 + nameLen + extraLen;
    if (nm === name) return method === 0 ? new TextDecoder().decode(u8.subarray(dataStart, dataStart + compSize)) : null;
    o = dataStart + compSize;
  }
  return null;
}

/* every stored entry under a path prefix — how the bundle's video files come
   back out on import. The bundle writer stores uncompressed (method 0), so
   the bytes are a straight subarray. */
export function unzipBinEntries(u8, prefix) {
  const u16 = (o) => u8[o] | (u8[o + 1] << 8);
  const u32 = (o) => u8[o] + u8[o + 1] * 256 + u8[o + 2] * 65536 + u8[o + 3] * 16777216;
  const out = [];
  let o = 0;
  while (o + 30 <= u8.length && u32(o) === 0x04034b50) {
    const method = u16(o + 8), compSize = u32(o + 18);
    const nameLen = u16(o + 26), extraLen = u16(o + 28);
    const nm = new TextDecoder().decode(u8.subarray(o + 30, o + 30 + nameLen));
    const dataStart = o + 30 + nameLen + extraLen;
    if (method === 0 && nm.startsWith(prefix)) out.push({ name: nm, bytes: u8.subarray(dataStart, dataStart + compSize) });
    o = dataStart + compSize;
  }
  return out;
}
