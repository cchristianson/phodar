/* ============================================================
   MINIMAL MP4 (ISO-BMFF) MUXER — one H.264 video track, fixed fps.
   Companion to the WebCodecs export path: VideoEncoder (avc format)
   emits length-prefixed samples + an avcC decoder description; this
   wraps them into a playable .mp4. Hand-rolled like the EXIF parser
   and the zip writer — a feature, not an oversight: the whole file
   is deterministic byte-packing, no dependency required.

   Assumes PTS == DTS (no B-frames) — the export encodes with
   latencyMode "realtime", which is low-delay by definition, so
   sample order == presentation order and stts alone carries timing.
   Layout: ftyp | moov | mdat (offsets known before writing — all
   samples live in ONE chunk, so stco has a single entry).
   ============================================================ */

const te = new TextEncoder();

/* box = length-prefixed payload; build bottom-up into Uint8Arrays */
function box(type, ...parts) {
  let n = 8;
  for (const p of parts) n += p.length;
  const b = new Uint8Array(n);
  const dv = new DataView(b.buffer);
  dv.setUint32(0, n);
  b.set(te.encode(type), 4);
  let o = 8;
  for (const p of parts) { b.set(p, o); o += p.length; }
  return b;
}
const u32 = (v) => { const b = new Uint8Array(4); new DataView(b.buffer).setUint32(0, v >>> 0); return b; };
const u16 = (v) => { const b = new Uint8Array(2); new DataView(b.buffer).setUint16(0, v & 0xffff); return b; };
const u8a = (...vals) => new Uint8Array(vals);
const zeros = (n) => new Uint8Array(n);
const fixed1616 = (v) => u32(Math.round(v * 65536));

/* Mux length-prefixed H.264 samples into an mp4.
   width/height: coded size. fps: exact frame rate (samples are 1/fps apart).
   avcC: Uint8Array — the AVCDecoderConfigurationRecord (VideoEncoder's
   decoderConfig.description). samples: [{ data: Uint8Array, key: bool }].
   Returns a Uint8Array of the complete file. */
export function muxMp4({ width, height, fps, avcC, samples }) {
  const N = samples.length;
  const TS = 30000;                                   // movie/media timescale
  const delta = Math.round(TS / fps);                 // per-sample duration
  const dur = N * delta;

  const ftyp = box("ftyp", te.encode("isom"), u32(512), te.encode("isomiso2avc1mp41"));

  /* ---- stbl ---- */
  const avc1 = box("avc1",
    zeros(6), u16(1),                                 // reserved + data_reference_index
    zeros(16),                                        // pre_defined/reserved
    u16(width), u16(height),
    u32(0x00480000), u32(0x00480000),                 // 72 dpi
    u32(0), u16(1),                                   // reserved + frame_count
    zeros(32),                                        // compressorname
    u16(0x0018), u16(0xffff),                         // depth 24, pre_defined -1
    box("avcC", avcC));
  const stsd = box("stsd", u32(0), u32(1), avc1);
  const stts = box("stts", u32(0), u32(1), u32(N), u32(delta));
  const keys = [];
  for (let i = 0; i < N; i++) if (samples[i].key) keys.push(i + 1);
  const stss = box("stss", u32(0), u32(keys.length), ...keys.map(u32));
  const stsc = box("stsc", u32(0), u32(1), u32(1), u32(N), u32(1));
  const stsz = box("stsz", u32(0), u32(0), u32(N), ...samples.map((s) => u32(s.data.length)));
  const stco = box("stco", u32(0), u32(1), u32(0));   // chunk offset patched below
  const stbl = box("stbl", stsd, stts, stss, stsc, stsz, stco);

  /* ---- the rest of the hierarchy ---- */
  const url_ = box("url ", u32(1));
  const dinf = box("dinf", box("dref", u32(0), u32(1), url_));
  const vmhd = box("vmhd", u32(1), zeros(8));
  const minf = box("minf", vmhd, dinf, stbl);
  const hdlr = box("hdlr", u32(0), u32(0), te.encode("vide"), zeros(12), te.encode("PhodarVideo\0"));
  const mdhd = box("mdhd", u32(0), u32(0), u32(0), u32(TS), u32(dur), u16(0x55c4), u16(0));
  const mdia = box("mdia", mdhd, hdlr, minf);
  const tkhd = box("tkhd",
    u32(3),                                           // version 0, flags: enabled+in-movie
    u32(0), u32(0), u32(1), u32(0),                   // times, track id 1, reserved
    u32(dur), zeros(8), u16(0), u16(0), u16(0), u16(0),
    fixed1616(1), u32(0), u32(0), u32(0), fixed1616(1), u32(0), u32(0), u32(0), fixed1616(16384), // identity matrix
    fixed1616(width), fixed1616(height));
  const trak = box("trak", tkhd, mdia);
  const mvhd = box("mvhd", u32(0), u32(0), u32(0), u32(TS), u32(dur),
    fixed1616(1), u16(0x0100), u16(0), zeros(8),
    fixed1616(1), u32(0), u32(0), u32(0), fixed1616(1), u32(0), u32(0), u32(0), fixed1616(16384),
    zeros(24), u32(2));                               // pre_defined, next_track_id
  const moov = box("moov", mvhd, trak);

  /* ---- patch the chunk offset now that every size is known ---- */
  let mdatPayload = 0;
  for (const s of samples) mdatPayload += s.data.length;
  const mdatOff = ftyp.length + moov.length;          // start of the mdat BOX
  const out = new Uint8Array(mdatOff + 8 + mdatPayload);
  out.set(ftyp, 0);
  out.set(moov, ftyp.length);
  /* locate the (single) stco box by signature and patch its one entry:
     [size 4]["stco" 4][ver/flags 4][count 4][entry 4] → entry at sig+12 */
  let stcoSig = -1;
  for (let i = ftyp.length; i < mdatOff - 3; i++)
    if (out[i] === 0x73 && out[i + 1] === 0x74 && out[i + 2] === 0x63 && out[i + 3] === 0x6f) { stcoSig = i; break; }
  new DataView(out.buffer).setUint32(stcoSig + 12, mdatOff + 8);
  new DataView(out.buffer).setUint32(mdatOff, 8 + mdatPayload);
  out.set(te.encode("mdat"), mdatOff + 4);
  let o = mdatOff + 8;
  for (const s of samples) { out.set(s.data, o); o += s.data.length; }
  return out;
}
