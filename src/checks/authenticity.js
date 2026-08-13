/* ============================================================
   AUTHENTICITY CHECKS — is this file what it claims to be?
   Two layers, each honest about what it can and cannot prove:

   1. FILE FORENSICS (`scanFileAuthenticity`, runs ONCE at upload on the
      ORIGINAL bytes, before Phodar's own normalization re-encode):
      AI-generator markers (Stable Diffusion parameter blocks, ComfyUI
      workflows, Midjourney/DALL·E tags), C2PA provenance manifests,
      editing-software fingerprints (Photoshop/Lightroom/CapCut/ffmpeg…),
      and container tells (a "photo" that is a PNG, progressive JPEG,
      screen-recording keys). String/byte scans of the head and tail of
      the file — metadata lives at both ends (the QuickTime lesson).

   2. DERIVED CONSISTENCY (`authDerived`, pure, re-evaluated whenever the
      inputs exist): scene brightness vs the COMPUTED sun elevation at
      the stated time and place (a bright daylight scene at astronomical
      night is a strong inconsistency), the stated time vs the file's own
      clock, the stated position vs the file's GPS, plus POSITIVE signals
      (star-field / terrain calibration passed — very hard to fake).

   Levels: "alarm"  = high-confidence manipulation/AI indicator (the
                      report banners these),
           "warn"   = edited / re-encoded / inconsistent — needs an
                      explanation, does not prove fakery,
           "note"   = weak signal or common-in-sharing artifact,
           "info"   = positive authenticity evidence.
   ABSENCE OF FINDINGS PROVES NOTHING — a clean scan means the file
   carries no tells, not that it is authentic. The report says so.
   ============================================================ */

import { sunPos } from "../math/astro.js";

const F = (id, level, label, detail) => ({ id, level, label, detail });

/* latin-1 haystack of the file's head + tail (metadata lives at both ends) */
function haystack(u8, headKB = 512, tailKB = 192) {
  const head = u8.subarray(0, Math.min(u8.length, headKB * 1024));
  const tail = u8.length > headKB * 1024 ? u8.subarray(Math.max(0, u8.length - tailKB * 1024)) : new Uint8Array(0);
  let s = "";
  const CH = 32768;
  for (let i = 0; i < head.length; i += CH) s += String.fromCharCode.apply(null, head.subarray(i, i + CH));
  for (let i = 0; i < tail.length; i += CH) s += String.fromCharCode.apply(null, tail.subarray(i, i + CH));
  return s;
}

/* AI generators that write their name/params into the file. Each entry is
   [regex, human label]. Deliberately specific — a false "AI" alarm is worse
   than a miss, so generic words are avoided. */
const AI_MARKS = [
  [/AUTOMATIC1111|sd-webui|Stable Diffusion|StableDiffusionPipeline/i, "Stable Diffusion"],
  [/ComfyUI/i, "ComfyUI"],
  [/InvokeAI/i, "InvokeAI"],
  [/NovelAI/i, "NovelAI"],
  [/Midjourney/i, "Midjourney"],
  [/DALL[·-]E|openai\.com\/dall/i, "DALL·E"],
  [/Adobe Firefly/i, "Adobe Firefly"],
  [/leonardo\.ai/i, "Leonardo AI"],
  [/ideogram\.ai/i, "Ideogram"],
  [/Draw Things/i, "Draw Things"],
  [/black-forest-labs|[^a-z]flux\.1/i, "FLUX"],
];

/* editing / re-encoding software fingerprints (warn — edited ≠ fabricated) */
const EDIT_MARKS = [
  [/Adobe Photoshop(?! Lightroom)/i, "Adobe Photoshop"],
  [/Lightroom/i, "Adobe Lightroom"],
  [/Adobe Premiere|Premiere Pro/i, "Adobe Premiere"],
  [/After Effects/i, "After Effects"],
  [/GIMP/i, "GIMP"],
  [/Affinity Photo/i, "Affinity Photo"],
  [/Pixelmator/i, "Pixelmator"],
  [/PicsArt/i, "PicsArt"],
  [/Facetune/i, "Facetune"],
  [/Snapseed/i, "Snapseed"],
  [/photopea/i, "Photopea"],
  [/CapCut/i, "CapCut"],
  [/InShot/i, "InShot"],
  [/KineMaster/i, "KineMaster"],
  [/DaVinci Resolve/i, "DaVinci Resolve"],
  [/HandBrake/i, "HandBrake"],
];

/* PNG chunk walk → [{key, textHead}] for tEXt/iTXt (where SD/Comfy stash
   their generation parameters) */
export function pngTextChunks(u8) {
  const out = [];
  if (!(u8.length > 16 && u8[0] === 0x89 && u8[1] === 0x50 && u8[2] === 0x4e && u8[3] === 0x47)) return out;
  let p = 8;
  while (p + 12 <= u8.length && out.length < 32) {
    const len = (u8[p] << 24) | (u8[p + 1] << 16) | (u8[p + 2] << 8) | u8[p + 3];
    const type = String.fromCharCode(u8[p + 4], u8[p + 5], u8[p + 6], u8[p + 7]);
    if (len < 0 || p + 12 + len > u8.length) break;
    if (type === "tEXt" || type === "iTXt") {
      const body = u8.subarray(p + 8, p + 8 + Math.min(len, 4096));
      let z = 0; while (z < body.length && body[z] !== 0) z++;
      const key = String.fromCharCode.apply(null, body.subarray(0, z));
      const textHead = String.fromCharCode.apply(null, body.subarray(z + 1, Math.min(body.length, z + 1 + 600)));
      out.push({ key, textHead });
    }
    if (type === "IEND") break;
    p += 12 + len;
  }
  return out;
}

/* JPEG marker walk → { progressive, adobeApp14, ducky, jfifOnly } */
export function jpegMarkers(u8) {
  const out = { progressive: false, adobeApp14: false, ducky: false, hasExif: false, hasJfif: false };
  if (!(u8.length > 4 && u8[0] === 0xff && u8[1] === 0xd8)) return null;
  let p = 2;
  while (p + 4 <= u8.length) {
    if (u8[p] !== 0xff) break;
    const m = u8[p + 1];
    if (m === 0xd9 || m === 0xda) break;              // EOI / start of scan
    const len = (u8[p + 2] << 8) | u8[p + 3];
    if (len < 2) break;
    const seg = u8.subarray(p + 4, p + 2 + len);
    const tag = String.fromCharCode.apply(null, seg.subarray(0, Math.min(12, seg.length)));
    if (m === 0xc2) out.progressive = true;           // SOF2 progressive
    if (m === 0xee && /^Adobe/.test(tag)) out.adobeApp14 = true;
    if (m === 0xec && /^Ducky/.test(tag)) out.ducky = true; // Photoshop Save-for-Web
    if (m === 0xe1 && /^Exif/.test(tag)) out.hasExif = true;
    if (m === 0xe0 && /^JFIF/.test(tag)) out.hasJfif = true;
    p += 2 + len;
  }
  return out;
}

/* ---- the upload-time scan. u8 = ORIGINAL file bytes; kind "image"|"video" */
export function scanFileAuthenticity(u8, kind) {
  const f = [];
  const hay = haystack(u8);

  /* AI generation markers — the loud ones */
  for (const [re, label] of AI_MARKS) {
    if (re.test(hay)) { f.push(F("ai-marker", "alarm", `${label} marker in the file`, `The file carries ${label}'s own fingerprint — AI-generated or AI-processed content.`)); break; }
  }
  /* Stable-Diffusion-style parameter blocks, wherever they appear */
  if (!f.some((x) => x.id === "ai-marker") && /Steps: \d+, Sampler: /.test(hay)) {
    f.push(F("ai-params", "alarm", "AI generation parameters in the file", "A Stable-Diffusion-style parameter block (Steps/Sampler/CFG) is embedded — the image was AI-generated."));
  }

  /* C2PA / Content Credentials provenance manifest */
  if (/c2pa|contentauth|urn:c2pa/i.test(hay) && /jumb|jumdc2/i.test(hay)) {
    if (/trainedAlgorithmicMedia|compositeWithTrainedAlgorithmicMedia/i.test(hay)) {
      f.push(F("c2pa-ai", "alarm", "Provenance manifest declares AI-generated content", "The file carries a C2PA (Content Credentials) manifest whose assertions mark the content as produced by a trained algorithmic model."));
    } else {
      f.push(F("c2pa", "note", "Provenance manifest present (C2PA)", "The file carries a Content Credentials manifest. Phodar does not verify the signature — inspect it at contentcredentials.org/verify for the signer and edit history."));
    }
  }

  /* editing software fingerprints */
  for (const [re, label] of EDIT_MARKS) {
    if (re.test(hay)) { f.push(F("edited", "warn", `${label} fingerprint in the file`, `The file passed through ${label}. Edited does not mean fabricated — cropping and exposure edits are common — but the pixels are not straight off the sensor.`)); break; }
  }
  if (!f.some((x) => x.id === "edited") && /stEvt:action="(?:edited|saved|converted)"/i.test(hay)) {
    f.push(F("edited", "warn", "XMP edit history in the file", "The file's XMP metadata records editing steps — the pixels are not straight off the sensor."));
  }

  if (kind === "image") {
    const png = pngTextChunks(u8);
    if (u8.length > 4 && u8[0] === 0x89 && u8[1] === 0x50) {
      for (const c of png) {
        if ((c.key === "parameters" || c.key === "prompt" || c.key === "workflow") && !f.some((x) => x.level === "alarm")) {
          f.push(F("png-genmeta", "alarm", `AI generation metadata ("${c.key}" chunk)`, "The PNG carries a generation-parameters chunk of the kind Stable Diffusion / ComfyUI write — AI-generated content."));
          break;
        }
      }
      f.push(F("png", "note", "PNG container", "Cameras produce JPEG/HEIC; PNG usually means a screenshot, an export, or an AI generator — the capture chain is not intact."));
    }
    const jm = jpegMarkers(u8);
    if (jm) {
      if (jm.progressive) f.push(F("progressive", "note", "Progressive JPEG", "Cameras write baseline JPEGs; progressive encoding is a web/software re-encode."));
      if (jm.ducky) f.push(F("ducky", "warn", "Photoshop Save-for-Web marker", "The 'Ducky' segment is written by Photoshop's Save for Web — this exact file came out of Photoshop."));
      else if (jm.adobeApp14 && !f.some((x) => x.id === "edited")) f.push(F("adobe14", "note", "Adobe encoder marker (APP14)", "The file passed through an Adobe encoder at some point."));
    }
  }

  if (kind === "video") {
    if (/Lavf|Lavc/.test(hay)) f.push(F("ffmpeg", "warn", "ffmpeg re-encode (Lavf)", "The container was written by ffmpeg/libav — a re-encode or download, not a camera original. Compression artifacts and dropped metadata follow."));
    if (/com\.apple\.ReplayKit|screen-?record/i.test(hay)) f.push(F("screenrec", "warn", "Screen recording", "The file carries screen-recording markers — it is a capture of a display, not of the sky."));
    if (/com\.apple\.quicktime\.(?:make|model|creationdate|location)/.test(hay) && !f.some((x) => x.level === "warn" || x.level === "alarm")) {
      f.push(F("apple-orig", "info", "Apple camera-original container keys", "The QuickTime metadata keys a phone camera writes are intact — consistent with an unmodified camera file."));
    }
  }

  return f;
}

/* ---- derived consistency checks — pure; call whenever inputs may exist.
   src needs: authLum {mean,p90} (images), lat/lon/whenMs, meta, calib. */
export function authDerived(src) {
  const f = [];
  const lat = +src?.lat, lon = +src?.lon, when = +src?.whenMs;
  const hasPos = isFinite(lat) && isFinite(lon) && (lat !== 0 || lon !== 0);

  /* scene brightness vs computed sun elevation */
  const lum = src?.authLum;
  if (lum && isFinite(lum.mean) && hasPos && isFinite(when) && when > 0) {
    const el = sunPos(when, lat, lon).alt;
    if (el < -9 && lum.mean > 115) {
      f.push(F("sun-night", "warn", "Bright scene at astronomical night",
        `At the stated time and place the sun sat ${el.toFixed(0)}° below the horizon, yet the photo's overall brightness reads as daylight. The stated time or place may simply be wrong — but as given, the scene and the sky disagree.`));
    } else if (el > 15 && lum.mean < 25 && lum.p90 < 60) {
      f.push(F("sun-day", "note", "Very dark scene in full daylight",
        `The sun sat ${el.toFixed(0)}° up at the stated time and place, but the photo is very dark — heavy underexposure, or the stated time/place is off.`));
    } else {
      f.push(F("sun-ok", "info", "Scene brightness consistent with the computed sun",
        `Sun elevation ${el.toFixed(0)}° at the stated time and place matches the photo's overall light.`));
    }
  }

  /* stated time vs the file's own clock */
  if (isFinite(when) && when > 0 && src?.meta && isFinite(src.meta.timeMs) && src.meta.timeMs > 0) {
    const dh = Math.abs(when - src.meta.timeMs) / 3600000;
    if (dh > 3) f.push(F("time-mismatch", "note", "Stated time differs from the file's clock",
      `The sighting time entered differs from the file's own timestamp by ${dh > 48 ? Math.round(dh / 24) + " days" : dh.toFixed(1) + " h"} — a timezone slip, a later re-save, or a repurposed file.`));
  }

  /* stated position vs the file's GPS */
  if (hasPos && src?.meta && isFinite(src.meta.lat) && isFinite(src.meta.lon)) {
    const dKm = Math.hypot((lat - src.meta.lat) * 111.32, (lon - src.meta.lon) * 111.32 * Math.cos(lat * Math.PI / 180));
    if (dKm > 10) f.push(F("gps-mismatch", "note", "Stated position far from the file's GPS",
      `The position entered sits ${Math.round(dKm)} km from where the file says it was taken.`));
  }

  /* positive: astronomically / terrain verified pointing (hard to fake) */
  const m = src?.calib?.method;
  if (m === "stars") f.push(F("cal-stars", "info", "Pointing verified against the real star field",
    `The photo's pointing was plate-solved against the actual night sky for this time and place${isFinite(src.calib?.rms) ? ` (fit ±${src.calib.rms}°)` : ""} — a fabricated sky would not solve.`));
  if (m === "terrain") f.push(F("cal-terrain", "info", "Pointing verified against the real terrain",
    `The photo's skyline matches the digital elevation model for this spot${isFinite(src.calib?.rms) ? ` (fit ${src.calib.rms}°)` : ""} — the backdrop is the real landscape.`));

  return f;
}

/* everything known about one source, upload-time + derived */
export function authFindings(src) {
  return [...(Array.isArray(src?.authFile) ? src.authFile : []), ...authDerived(src)];
}

/* worst level across findings; the report banners on any "alarm" */
const RANK = { alarm: 3, warn: 2, note: 1, info: 0 };
export function authSummary(findings) {
  let worst = "info", alarms = [], warns = 0;
  for (const x of findings || []) {
    if (RANK[x.level] > RANK[worst]) worst = x.level;
    if (x.level === "alarm") alarms.push(x);
    if (x.level === "warn") warns++;
  }
  return { worst, alarms, warns };
}
