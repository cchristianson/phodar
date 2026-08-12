/* Server-side media decode — the piece the browser's <canvas> provides in the
   app, provided here by ffmpeg/ffprobe (child processes, no npm deps). The
   measurement modules (postrack, platesolve, exif) are pure and operate on
   RGBA buffers + metadata; this module is the only place pixels are minted
   server-side. NODE ONLY — never import from the app bundle.

   Everything degrades honestly: no ffmpeg → a named error the API can relay
   ("raw-media ingestion needs ffmpeg on the server"), an undecodable file →
   ffmpeg's stderr tail, HEIC → a specific message (stock ffmpeg lacks
   libheif; iPhone users should share as JPEG / "Most Compatible"). */

import { spawn, execFile } from "node:child_process";

let _ff = null, _ffMissAt = 0; // cache success forever; retry a miss after 60 s
export async function ffmpegAvailable() {
  if (_ff) return _ff;
  if (_ff === false && Date.now() - _ffMissAt < 60000) return false;
  const which = (bin) => new Promise((res) => {
    /* generous timeout — a cold container's first spawn can take seconds, and
       a false negative here disables ingestion for the process lifetime */
    execFile(bin, ["-version"], { timeout: 15000 }, (err) => res(err ? null : bin));
  });
  const ffmpeg = await which("ffmpeg"), ffprobe = await which("ffprobe");
  _ff = ffmpeg && ffprobe ? { ffmpeg, ffprobe } : false;
  if (!_ff) _ffMissAt = Date.now();
  return _ff;
}

/* remux a SPAN of a remote file straight to local disk without downloading the
   whole thing: ffmpeg's http demuxer uses range requests, and -c copy means no
   re-encode — a 60 s slice of a 750 MB 4K clip lands as its own ~300 MB of
   bytes fetched, not the whole file. This is how clips over the download cap
   get ingested: the caller trims to the sighting. Keyframe-aligned (copy
   cuts on keyframes), which is fine — the span is analysis material, not an
   exhibit edit. */
export async function remuxSpanFromUrl(url, outPath, t0, t1) {
  const ff = await ffmpegAvailable();
  if (!ff) throw new Error("ffmpeg/ffprobe not available on this server — raw-media ingestion is disabled");
  const span = Math.max(0.5, t1 - t0);
  /* -map_metadata 0 + use_metadata_tags: without them a copy remux silently
     DROPS the QuickTime metadata phodar's parser mines (ISO-6709 GPS,
     creation time) — proven with a metacheck before/after. On a fresh phone
     clip that metadata is the position and clock for the whole sighting. */
  const args = ["-v", "error", "-ss", String(Math.max(0, t0)), "-i", url, "-t", String(span),
    "-c", "copy", "-map_metadata", "0", "-movflags", "use_metadata_tags+faststart", "-y", outPath];
  await run(ff.ffmpeg, args, { timeout: 300000 });
}

const run = (bin, args, { input, maxOut = 512 * 1024 * 1024, timeout = 300000 } = {}) =>
  new Promise((resolve, reject) => {
    const p = spawn(bin, args, { stdio: ["pipe", "pipe", "pipe"] });
    const out = [], err = [];
    let outLen = 0;
    const t = setTimeout(() => { p.kill("SIGKILL"); reject(new Error(`${bin} timed out`)); }, timeout);
    p.stdout.on("data", (c) => { outLen += c.length; if (outLen > maxOut) { p.kill("SIGKILL"); } else out.push(c); });
    p.stderr.on("data", (c) => { if (err.length < 64) err.push(c); });
    p.on("error", (e) => { clearTimeout(t); reject(e); });
    p.on("close", (code) => {
      clearTimeout(t);
      if (code === 0) resolve(Buffer.concat(out));
      else reject(new Error(`${bin} failed (${code}): ${Buffer.concat(err).toString().slice(-400)}`));
    });
    if (input) p.stdin.write(input);
    p.stdin.end();
  });

/* ffprobe → { kind, w, h, durS, codec, fps } (rotation is folded in: ffmpeg's
   decode path auto-rotates per the display matrix, so w/h here are the
   DISPLAY dimensions every downstream consumer sees). */
export async function probeMedia(filePath) {
  const ff = await ffmpegAvailable();
  if (!ff) throw new Error("ffmpeg/ffprobe not available on this server — raw-media ingestion is disabled");
  const raw = await run(ff.ffprobe, ["-v", "error", "-print_format", "json", "-show_streams", "-show_format", filePath], { timeout: 30000 });
  const d = JSON.parse(raw.toString());
  const v = (d.streams || []).find((s) => s.codec_type === "video");
  if (!v) throw new Error("no video/image stream found in the file");
  if (/^hevc$|^h265$/.test(v.codec_name) && !(await canDecode(ff, filePath))) {
    throw new Error("this file is HEVC and the server's ffmpeg can't decode it — re-export as H.264/JPEG");
  }
  let w = +v.width || 0, h = +v.height || 0;
  /* display-matrix rotation swaps the visible axes; ffprobe reports the
     STORED dims. Fold it so natW/natH match what ffmpeg's decode emits. */
  const rot = Math.abs(+((v.side_data_list || []).find((s) => s.rotation != null) || {}).rotation || 0) % 360;
  if (rot === 90 || rot === 270) { const t = w; w = h; h = t; }
  const durS = +(v.duration ?? d.format?.duration) || 0;
  const fpsF = String(v.r_frame_rate || "0/1").split("/");
  const fps = +fpsF[1] ? +fpsF[0] / +fpsF[1] : 0;
  const isImage = durS < 0.5 && (+v.nb_frames || 1) <= 1 || /mjpeg|png|webp|bmp|tiff/.test(v.codec_name || "");
  return { kind: isImage ? "image" : "video", w, h, durS: isImage ? 0 : durS, codec: v.codec_name, fps };
}

const canDecode = async (ff, filePath) => {
  try { await run(ff.ffmpeg, ["-v", "error", "-i", filePath, "-frames:v", "1", "-f", "null", "-"], { timeout: 30000 }); return true; }
  catch (e) { return false; }
};

const evenScale = (w, h, maxW) => {
  const s = Math.min(1, maxW / w);
  const W = Math.max(2, Math.round(w * s / 2) * 2), H = Math.max(2, Math.round(h * s / 2) * 2);
  return { W, H };
};

/* one frame at time t (0 for images) → { t, w, h, data: Uint8ClampedArray RGBA } */
export async function decodeFrameAt(filePath, t, { maxW = 1600 } = {}) {
  const ff = await ffmpegAvailable();
  if (!ff) throw new Error("ffmpeg not available");
  const p = await probeMedia(filePath);
  const { W, H } = evenScale(p.w, p.h, maxW);
  const args = ["-v", "error"];
  if (p.kind === "video" && t > 0.01) args.push("-ss", String(Math.max(0, Math.min(t, Math.max(0, p.durS - 0.05)))));
  args.push("-i", filePath, "-frames:v", "1", "-vf", `scale=${W}:${H}`, "-f", "rawvideo", "-pix_fmt", "rgba", "pipe:1");
  const raw = await run(ff.ffmpeg, args, { timeout: 60000 });
  if (raw.length < W * H * 4) throw new Error(`decode produced ${raw.length} bytes, expected ${W * H * 4}`);
  return { t, w: W, h: H, data: new Uint8ClampedArray(raw.buffer, raw.byteOffset, W * H * 4) };
}

/* sequential frame reader for the stabilize walk: one ffmpeg pass at a fixed
   fps, frames pulled in order. next() → {t, data}|null. The walk's bisection
   only ever looks BETWEEN the previous and current cadence samples, so a read
   fps a few × the cadence gives it frames to land on; the reader keeps a small
   tail of recent frames so a rewind inside the current gap always hits. */
export function openFrameStream(filePath, { fps = 12, maxW = 640, t0 = 0, t1 = 1e9, W, H }) {
  let proc = null, buf = Buffer.alloc(0), idx = 0, done = false, err = null, wake = null;
  const frameBytes = W * H * 4;
  const tail = new Map(); // recent frames by index (bisection rewinds)
  const start = async () => {
    const ff = await ffmpegAvailable();
    if (!ff) throw new Error("ffmpeg not available");
    const args = ["-v", "error"];
    if (t0 > 0.01) args.push("-ss", String(t0));
    args.push("-i", filePath);
    if (t1 < 1e8) args.push("-t", String(Math.max(0.05, t1 - t0)));
    args.push("-vf", `fps=${fps},scale=${W}:${H}`, "-f", "rawvideo", "-pix_fmt", "rgba", "pipe:1");
    proc = spawn(ff.ffmpeg, args, { stdio: ["ignore", "pipe", "pipe"] });
    const errs = [];
    proc.stderr.on("data", (c) => { if (errs.length < 32) errs.push(c); });
    proc.stdout.on("data", (c) => { buf = buf.length ? Buffer.concat([buf, c]) : c; if (wake) { const w = wake; wake = null; w(); } });
    proc.on("close", (code) => {
      done = true;
      if (code !== 0 && code !== null && buf.length < frameBytes) err = new Error(`ffmpeg stream failed: ${Buffer.concat(errs).toString().slice(-300)}`);
      if (wake) { const w = wake; wake = null; w(); }
    });
    /* backpressure: pause ffmpeg when we're holding > ~40 undelivered frames */
    const guard = setInterval(() => {
      if (!proc || done) { clearInterval(guard); return; }
      if (buf.length > frameBytes * 40) proc.stdout.pause(); else proc.stdout.resume();
    }, 100);
  };
  let started = null;
  return {
    /* frame i (0-based) is at t0 + i/fps. frameAt(t) serves the NEAREST
       decoded frame, from the tail when rewinding, else by advancing. */
    async frameAt(t) {
      if (!started) started = start();
      await started;
      const want = Math.max(0, Math.round((t - t0) * fps));
      if (tail.has(want)) return { t: t0 + want / fps, data: tail.get(want) };
      if (want < idx) { // rewound past the tail — nearest kept frame
        let best = null;
        for (const k of tail.keys()) if (best == null || Math.abs(k - want) < Math.abs(best - want)) best = k;
        if (best != null) return { t: t0 + best / fps, data: tail.get(best) };
      }
      while (idx <= want) {
        while (buf.length < frameBytes) {
          if (err) throw err;
          if (done) return null;
          await new Promise((res) => { wake = res; });
        }
        const fb = buf.subarray(0, frameBytes);
        buf = buf.subarray(frameBytes);
        const data = new Uint8ClampedArray(fb.buffer.slice(fb.byteOffset, fb.byteOffset + frameBytes));
        tail.set(idx, data);
        for (const k of tail.keys()) if (k < idx - Math.max(8, fps)) tail.delete(k); // keep ~1s of history
        idx++;
        if (idx > want) return { t: t0 + (idx - 1) / fps, data };
      }
      return null;
    },
    close() { try { proc && proc.kill("SIGKILL"); } catch (e) { } },
  };
}

/* RGBA → JPEG bytes (keyframes handed to the AI, embedded photo thumbnails) */
export async function encodeJpeg(data, w, h, { quality = 4, maxW = 0 } = {}) {
  const ff = await ffmpegAvailable();
  if (!ff) throw new Error("ffmpeg not available");
  const args = ["-v", "error", "-f", "rawvideo", "-pix_fmt", "rgba", "-s", `${w}x${h}`, "-i", "pipe:0"];
  if (maxW && maxW < w) { const { W, H } = evenScale(w, h, maxW); args.push("-vf", `scale=${W}:${H}`); }
  args.push("-frames:v", "1", "-c:v", "mjpeg", "-q:v", String(quality), "-f", "image2", "pipe:1");
  return await run(ff.ffmpeg, args, { input: Buffer.from(data.buffer, data.byteOffset, data.length), timeout: 30000 });
}
