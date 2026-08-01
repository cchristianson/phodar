/* ============================================================
   DRONE FLIGHT-LOG CHECK — the ground-truth / calibration feature.
   Fly your own drone, photograph it like a sighting, then upload the
   drone's flight record: every phodar output (direction, angular size,
   range, true size, altitude, speed, heading) gets graded against what
   the drone's own GPS logged. This is the same candidate shape as the
   ADS-B check — predicted az/el/angular-size vs the witness sight-line —
   but the "candidate" is a craft whose truth you control, which turns a
   sighting run into a calibration run.

   Formats (all parsed offline from an uploaded file — no network):
   · Airdata CSV export (the practical route for DJI Fly logs — the
     phone's .txt flight records are encrypted; Airdata/PhantomHelp
     decode them). Columns like `time(millisecond)`, `datetime(utc)`,
     `latitude`, `longitude`, `altitude_above_seaLevel(feet)`,
     `height_above_takeoff(feet)`, `speed(mph)`, `compass_heading(degrees)`.
   · DJI Fly decoded CSV (PhantomHelp / flight-reader style): `OSD.latitude`,
     `OSD.longitude`, `OSD.height [ft]`, `OSD.altitude [ft]`,
     `OSD.hSpeed [MPH]`, `OSD.yaw`, `CUSTOM.updateTime`.
   · DJI video-caption SRT (bracket format with a datetime line per frame:
     `[latitude: …] [longitude: …] [rel_alt: … abs_alt: …]`).
   Header matching is fuzzy (normalized names) and units are read from the
   header text (`(feet)`, `[ft]`, `(mph)`, `[m/s]`) — Airdata and DJI both
   move column names around between export versions.

   Altitude honesty: many DJI logs carry only height ABOVE TAKEOFF, not MSL.
   When no absolute altitude exists, the caller supplies the takeoff ground
   elevation (`homeElevM`) — or we assume the drone took off at the reference
   observer's elevation and SAY SO (`altAssumed`), which is usually true for
   a backyard calibration flight and wrong when it isn't.

   Clock honesty: phone EXIF time and the drone's log clock can disagree
   (timezone parsing of a local-time log is the big one). So every comparison
   is made twice — at the STATED sighting time, and at the log instant that
   best fits every witness sight-line (a whole-log scan, the same
   "the object's own motion is the shared signal" idea as stereoVideo's
   auto-sync) — and the offset between them is reported, never hidden.
   ============================================================ */

import { D2R, R2D, RE, dot, mag, unit, enuFromGeo, dirFromAzEl, dirToAzEl } from "../math/geodesy.js";
import { isNum } from "../math/format.js";

const FT = 0.3048, MPH = 0.44704, KMH = 1 / 3.6;

/* Spans are the widest silhouette a witness would mark on a photo of the
   hovering craft (body, not the prop-blur disc). Editable in the UI —
   these are defaults with their provenance stated, not gospel. */
export const DRONE_PRESETS = [
  { id: "mini1", label: "DJI Mavic Mini / Mini 1", spanM: 0.202, note: "160×202 mm unfolded body (213 mm diagonal wheelbase); spinning props blur out to ~0.33 m" },
  { id: "neo", label: "DJI Neo", spanM: 0.157, note: "130×157 mm with the built-in prop guards" },
  { id: "custom", label: "Custom / other", spanM: null, note: "enter the widest dimension a photo would show" },
];

/* ---------- parsing ---------- */

/* split one CSV line honouring double-quoted fields */
function splitCsv(line) {
  const out = []; let cur = "", q = false;
  for (let i = 0; i < line.length; i++) {
    const c = line[i];
    if (q) {
      if (c === '"') { if (line[i + 1] === '"') { cur += '"'; i++; } else q = false; }
      else cur += c;
    } else if (c === '"') q = true;
    else if (c === ",") { out.push(cur); cur = ""; }
    else cur += c;
  }
  out.push(cur);
  return out;
}

const normH = (h) => h.toLowerCase().replace(/[^a-z0-9]/g, "");

/* metres-per-unit from the header's own unit annotation */
function lenUnit(raw) {
  if (/\(\s*feet\s*\)|\[\s*ft\s*\]|\(\s*ft\s*\)/i.test(raw)) return FT;
  return 1; // metres (DJI raw values and any unannotated column)
}
function spdUnit(raw) {
  if (/mph/i.test(raw)) return MPH;
  if (/km\/?h|kph/i.test(raw)) return KMH;
  if (/\bkt|knot/i.test(raw)) return 0.514444;
  return 1; // m/s
}

/* "2026-08-01 17:05:11.297", "2026/08/01 17:05:11", ISO — → ms epoch.
   assumeUtc appends Z when the string carries no zone (Airdata's
   datetime(utc)); otherwise the string is taken as local time, which is
   what DJI's CUSTOM.updateTime and SRT datetimes actually are. */
export function parseWhen(s, assumeUtc = false) {
  if (!s) return null;
  let t = String(s).trim().replace(/\//g, "-");
  const m = t.match(/^(\d{4})-(\d{1,2})-(\d{1,2})[ T](\d{1,2}):(\d{2}):(\d{2})([.,](\d{1,3}))?/);
  if (!m) return null;
  const hasZone = /(Z|[+-]\d{2}:?\d{2})$/.test(t);
  const iso = `${m[1]}-${m[2].padStart(2, "0")}-${m[3].padStart(2, "0")}T${m[4].padStart(2, "0")}:${m[5]}:${m[6]}${m[8] ? "." + m[8].padEnd(3, "0") : ""}`;
  const d = new Date(hasZone ? t.replace(" ", "T") : assumeUtc ? iso + "Z" : iso);
  const ms = d.getTime();
  return isNaN(ms) ? null : ms;
}

const latOk = (v) => isNum(v) && Math.abs(v) <= 90 && v !== 0;
const lonOk = (v) => isNum(v) && Math.abs(v) <= 180 && v !== 0;

function parseCsvLog(text) {
  const lines = text.split(/\r?\n/).filter((l) => l.trim().length);
  if (lines.length < 2) return { ok: false, error: "CSV has no data rows" };
  /* some DJI exports put a title line first — the header is the first line
     that mentions a latitude column */
  let hi = lines.findIndex((l) => /latitude/i.test(l));
  if (hi < 0 || hi > 4) hi = 0;
  const rawH = splitCsv(lines[hi]);
  const H = rawH.map(normH);
  const find = (...subs) => {
    for (const sub of subs) {
      const i = H.findIndex((h) => h.includes(sub));
      if (i >= 0) return i;
    }
    return -1;
  };
  const iLat = find("latitude"), iLon = find("longitude", "longtitude");
  if (iLat < 0 || iLon < 0) return { ok: false, error: "no latitude/longitude columns found" };
  const iAbsAlt = find("altitudeabovesealevel", "abovesealevel", "ascentmsl");
  /* OSD.altitude in DJI Fly decodes is above takeoff, NOT MSL — treat it as
     relative alongside the explicit height columns */
  const iRelAlt = find("heightabovetakeoff", "osdheight", "osdaltitude", "relativealtitude", "relalt");
  const iSpeed = find("speedmph", "osdhspeed", "horizontalspeed", "speed");
  const iHead = find("compassheading", "osdyaw", "heading", "yaw");
  const iAbsT = find("datetimeutc", "customupdatetime", "customdatetime", "datetime", "utctime", "timestamp");
  const iRelT = find("timemillisecond", "flighttimemillisecond", "osdflytime", "elapsed");
  const absIsUtc = iAbsT >= 0 && /utc/i.test(rawH[iAbsT]);
  const relIsSec = iRelT >= 0 && /flytime|\[s\]|\(s(econds)?\)/i.test(rawH[iRelT]);
  const absAltU = iAbsAlt >= 0 ? lenUnit(rawH[iAbsAlt]) : 1;
  const relAltU = iRelAlt >= 0 ? lenUnit(rawH[iRelAlt]) : 1;
  const spdU = iSpeed >= 0 ? spdUnit(rawH[iSpeed]) : 1;

  const pts = [];
  let t0Abs = null, t0Rel = null;
  for (let li = hi + 1; li < lines.length; li++) {
    const c = splitCsv(lines[li]);
    const lat = parseFloat(c[iLat]), lon = parseFloat(c[iLon]);
    if (!latOk(lat) || !lonOk(lon)) continue; // pre-GPS-lock rows
    let tMs = null;
    const rel = iRelT >= 0 ? parseFloat(c[iRelT]) * (relIsSec ? 1000 : 1) : null;
    if (iAbsT >= 0) {
      const abs = parseWhen(c[iAbsT], absIsUtc);
      if (abs != null) {
        /* anchor on the first absolute stamp and advance by the relative
           clock when there is one — Airdata's datetime(utc) is 1 s
           resolution while time(millisecond) is 100 ms */
        if (t0Abs == null) { t0Abs = abs; t0Rel = isNum(rel) ? rel : null; }
        tMs = isNum(rel) && t0Rel != null ? t0Abs + (rel - t0Rel) : abs;
      }
    }
    if (tMs == null && isNum(rel)) tMs = rel; // relative-only log
    if (tMs == null) continue;
    const p = { tMs, lat, lon };
    if (iAbsAlt >= 0) { const v = parseFloat(c[iAbsAlt]); if (isNum(v)) p.altAbsM = v * absAltU; }
    if (iRelAlt >= 0) { const v = parseFloat(c[iRelAlt]); if (isNum(v)) p.altRelM = v * relAltU; }
    if (iSpeed >= 0) { const v = parseFloat(c[iSpeed]); if (isNum(v)) p.speedMs = v * spdU; }
    if (iHead >= 0) { const v = parseFloat(c[iHead]); if (isNum(v)) p.headDeg = ((v % 360) + 360) % 360; }
    pts.push(p);
  }
  if (!pts.length) return { ok: false, error: "no rows with a GPS fix and a timestamp" };
  return { ok: true, pts, src: "csv", absTime: t0Abs != null };
}

/* DJI video-caption SRT: blocks with a `… --> …` timecode line, a
   `YYYY-MM-DD HH:MM:SS.mmm` datetime line, and bracketed telemetry
   (`[latitude: …] [longtitude: …] [rel_alt: … abs_alt: …]` — the
   longitude typo is DJI's own, in real firmware). */
function parseSrtLog(text) {
  const blocks = text.split(/\r?\n\r?\n/);
  const pts = [];
  for (const b of blocks) {
    if (!/-->/.test(b)) continue;
    const lat = b.match(/\[?\s*latitude\s*:?\s*(-?\d+(\.\d+)?)/i);
    const lon = b.match(/\[?\s*long?titude\s*:?\s*(-?\d+(\.\d+)?)/i) || b.match(/\[?\s*longitude\s*:?\s*(-?\d+(\.\d+)?)/i);
    if (!lat || !lon) continue;
    const la = parseFloat(lat[1]), lo = parseFloat(lon[1]);
    if (!latOk(la) || !lonOk(lo)) continue;
    const dt = b.match(/\d{4}[-/]\d{1,2}[-/]\d{1,2}[ T]\d{1,2}:\d{2}:\d{2}([.,]\d{1,3})?/);
    let tMs = dt ? parseWhen(dt[0]) : null;
    if (tMs == null) {
      const tc = b.match(/(\d{2}):(\d{2}):(\d{2})[,.](\d{3})\s*-->/);
      if (tc) tMs = ((+tc[1] * 60 + +tc[2]) * 60 + +tc[3]) * 1000 + +tc[4];
    }
    if (tMs == null) continue;
    const p = { tMs, lat: la, lon: lo };
    const rel = b.match(/rel_alt\s*:?\s*(-?\d+(\.\d+)?)/i);
    const abs = b.match(/abs_alt\s*:?\s*(-?\d+(\.\d+)?)/i);
    const altOnly = b.match(/\[\s*altitude\s*:?\s*(-?\d+(\.\d+)?)/i);
    if (rel) p.altRelM = parseFloat(rel[1]);
    if (abs) p.altAbsM = parseFloat(abs[1]);
    else if (!rel && altOnly) p.altRelM = parseFloat(altOnly[1]);
    pts.push(p);
  }
  if (!pts.length) return { ok: false, error: "no GPS telemetry found in the SRT (needs the bracket-format captions with latitude/longitude)" };
  return { ok: true, pts, src: "srt", absTime: pts.some((p) => p.tMs > 1e11) };
}

export function parseFlightLog(text, name = "") {
  if (!text || !text.trim()) return { ok: false, error: "empty file" };
  const isSrt = /\.srt$/i.test(name) || /-->\s*\d{2}:\d{2}/.test(text.slice(0, 4000));
  const r = isSrt ? parseSrtLog(text) : parseCsvLog(text);
  if (!r.ok) return r;
  r.pts.sort((a, b) => a.tMs - b.tMs);
  /* collapse duplicate timestamps (Airdata sometimes repeats a stamp) */
  r.pts = r.pts.filter((p, i) => i === 0 || p.tMs > r.pts[i - 1].tMs);
  r.n = r.pts.length;
  r.t0Ms = r.pts[0].tMs;
  r.t1Ms = r.pts[r.pts.length - 1].tMs;
  r.hasAbsAlt = r.pts.some((p) => isNum(p.altAbsM));
  r.hasRelAlt = r.pts.some((p) => isNum(p.altRelM));
  return r;
}

/* even-stride downsample for persistence (autosave/share must stay lean;
   a 100 ms Airdata log of a 15-min flight is 9000 rows) */
export function thinLog(pts, maxN = 900) {
  if (pts.length <= maxN) return pts;
  const out = [];
  const step = (pts.length - 1) / (maxN - 1);
  for (let i = 0; i < maxN; i++) out.push(pts[Math.round(i * step)]);
  return out;
}

/* ---------- interpolation & kinematics from the log ---------- */

const lerp = (a, b, f) => a + (b - a) * f;
const lerpDeg = (a, b, f) => {
  const d = ((b - a + 540) % 360) - 180;
  return ((a + d * f) % 360 + 360) % 360;
};

/* interpolated drone state at tMs; speed/heading are derived from the
   positions over a ±win window when the log has no columns for them
   (SRT logs don't) */
export function logStateAt(pts, tMs, winMs = 1000) {
  if (!pts.length) return null;
  if (tMs <= pts[0].tMs) return { ...pts[0], clamped: tMs < pts[0].tMs - 1 };
  const last = pts[pts.length - 1];
  if (tMs >= last.tMs) return { ...last, clamped: tMs > last.tMs + 1 };
  let lo = 0, hi = pts.length - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; (pts[m].tMs <= tMs ? lo = m : hi = m); }
  const a = pts[lo], b = pts[hi], f = (tMs - a.tMs) / (b.tMs - a.tMs);
  const st = { tMs, lat: lerp(a.lat, b.lat, f), lon: lerp(a.lon, b.lon, f) };
  if (isNum(a.altAbsM) && isNum(b.altAbsM)) st.altAbsM = lerp(a.altAbsM, b.altAbsM, f);
  if (isNum(a.altRelM) && isNum(b.altRelM)) st.altRelM = lerp(a.altRelM, b.altRelM, f);
  if (isNum(a.speedMs) && isNum(b.speedMs)) st.speedMs = lerp(a.speedMs, b.speedMs, f);
  if (isNum(a.headDeg) && isNum(b.headDeg)) st.headDeg = lerpDeg(a.headDeg, b.headDeg, f);
  if (!isNum(st.speedMs) || !isNum(st.headDeg)) {
    const d = logVelocity(pts, tMs, winMs);
    if (d) {
      if (!isNum(st.speedMs)) st.speedMs = d.speedMs;
      if (!isNum(st.headDeg)) st.headDeg = d.headDeg;
    }
  }
  return st;
}

/* ground velocity from positions over ±winMs — central difference in the
   local ENU frame about the earlier sample */
export function logVelocity(pts, tMs, winMs = 1000) {
  const a = stateNear(pts, tMs - winMs / 2), b = stateNear(pts, tMs + winMs / 2);
  if (!a || !b || b.tMs - a.tMs < 1) return null;
  const ref = { lat: a.lat, lon: a.lon, alt: 0 };
  const P = enuFromGeo(b.lat, b.lon, 0, ref);
  const dt = (b.tMs - a.tMs) / 1000;
  const vx = P[0] / dt, vy = P[1] / dt;
  return { speedMs: Math.hypot(vx, vy), headDeg: ((Math.atan2(vx, vy) * R2D) + 360) % 360 };
}
function stateNear(pts, tMs) {
  if (!pts.length) return null;
  if (tMs <= pts[0].tMs) return pts[0];
  const last = pts[pts.length - 1];
  if (tMs >= last.tMs) return last;
  let lo = 0, hi = pts.length - 1;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; (pts[m].tMs <= tMs ? lo = m : hi = m); }
  const a = pts[lo], b = pts[hi], f = (tMs - a.tMs) / (b.tMs - a.tMs);
  return { tMs, lat: lerp(a.lat, b.lat, f), lon: lerp(a.lon, b.lon, f) };
}

/* the altitude to hand the geometry: absolute MSL when the log has it, else
   takeoff-relative + a home elevation — the caller's, or the observer's own
   elevation as the stated assumption of last resort */
export function droneAltM(st, homeElevM, obsAltM) {
  if (isNum(st.altAbsM)) return { altM: st.altAbsM, src: "abs" };
  if (isNum(st.altRelM)) {
    if (isNum(homeElevM)) return { altM: homeElevM + st.altRelM, src: "home" };
    return { altM: (isNum(obsAltM) ? +obsAltM : 0) + st.altRelM, src: "assumed", altAssumed: true };
  }
  return { altM: isNum(obsAltM) ? +obsAltM : 0, src: "none", altAssumed: true };
}

/* az/el/range of the drone from an observer, in the observer's own ENU
   frame. Same refraction convention as the ADS-B check — negligible at
   drone ranges but kept identical so the two checks can never disagree
   about the same geometry. */
export function droneAzElRange(obs, st, homeElevM) {
  const { altM, altAssumed } = droneAltM(st, homeElevM, obs.alt);
  const P = enuFromGeo(st.lat, st.lon, altM, { lat: +obs.lat, lon: +obs.lon, alt: isNum(obs.alt) ? +obs.alt : 0 });
  const dGround = Math.hypot(P[0], P[1]);
  P[2] += (0.13 * dGround * dGround) / (2 * RE);
  const ae = dirToAzEl(unit(P));
  return { az: ae.az, el: ae.el, rangeM: mag(P), d: unit(P), altM, altAssumed: !!altAssumed };
}

/* ---------- comparison against the witnesses ---------- */

/* predictions for every valid witness at one log instant — the ADS-B
   candidate shape: per-witness sep / az / el / range / predicted angular
   size, plus the worst-witness separation that governs a match */
export function logMomentPer(sources, pts, tMs, spanM, homeElevM) {
  const st = logStateAt(pts, tMs);
  if (!st) return null;
  const per = [];
  for (const s of sources) {
    if (!isNum(s.lat) || !isNum(s.lon) || !isNum(s.A?.az) || !isNum(s.A?.el)) continue;
    const g = droneAzElRange(s, st, homeElevM);
    const w = dirFromAzEl(+s.A.az, +s.A.el);
    const sep = Math.acos(Math.min(1, Math.max(-1, dot(w, g.d)))) * R2D;
    per.push({
      name: s.name, az: g.az, el: g.el, rangeM: g.rangeM, sep,
      predAng: isNum(spanM) && spanM > 0 ? 2 * Math.atan(spanM / 2 / g.rangeM) * R2D : null,
      altM: g.altM, altAssumed: g.altAssumed,
    });
  }
  if (!per.length) return null;
  return { tMs, st, per, sepMax: Math.max(...per.map((p) => p.sep)), clamped: !!st.clamped };
}

/* scan the WHOLE log for the instant that best fits every witness
   sight-line. Clock skew between phone EXIF and a local-time drone log can
   be seconds (drift) or hours (timezone) — scanning everything instead of a
   window around the stated time makes the timezone case just work, and a
   calibration flight log is only minutes long. Coarse pass at stepMs, then
   a fine pass at 50 ms around the winner. */
export function syncLogTime(sources, pts, spanM, homeElevM, stepMs = 500) {
  if (!pts.length) return null;
  const t0 = pts[0].tMs, t1 = pts[pts.length - 1].tMs;
  let best = null;
  const tryT = (t) => {
    const m = logMomentPer(sources, pts, t, spanM, homeElevM);
    if (m && (!best || m.sepMax < best.sepMax)) best = m;
  };
  for (let t = t0; t <= t1; t += stepMs) tryT(t);
  tryT(t1);
  if (!best) return null;
  const c0 = Math.max(t0, best.tMs - stepMs), c1 = Math.min(t1, best.tMs + stepMs);
  for (let t = c0; t <= c1; t += 50) tryT(t);
  return best;
}

/* fix-vs-log truth comparison. fix = analyze(sources) result (may be null /
   not ok — angular comparisons still stand on their own). All errors are
   signed where a sign means something (phodar − truth). */
export function calibrationSummary({ sources, fix, pts, tMs, spanM, homeElevM }) {
  const valid = (sources || []).filter((s) => isNum(s.lat) && isNum(s.lon) && isNum(s.A?.az) && isNum(s.A?.el));
  const m = logMomentPer(valid, pts, tMs, spanM, homeElevM);
  if (!m) return null;
  const out = { tMs, sepMax: m.sepMax, per: m.per, st: m.st, clamped: m.clamped };
  if (fix && fix.ok) {
    const { altM, altAssumed } = droneAltM(m.st, homeElevM, fix.ref.alt);
    const D = enuFromGeo(m.st.lat, m.st.lon, altM, fix.ref);
    const X = fix.solA.X;
    const dv = [X[0] - D[0], X[1] - D[1], X[2] - D[2]];
    out.fixCmp = {
      errM: mag(dv), errH: Math.hypot(dv[0], dv[1]), errV: dv[2],
      rangeM: mag(D),
      droneAltRel: D[2], fixAltRel: X[2],
      altAssumed: !!altAssumed,
    };
    out.fixCmp.errPct = out.fixCmp.rangeM > 1 ? (out.fixCmp.errM / out.fixCmp.rangeM) * 100 : null;
    if (isNum(fix.sizeAvg) && isNum(spanM) && spanM > 0) {
      out.fixCmp.sizeRatio = fix.sizeAvg / spanM;
      out.fixCmp.sizeM = fix.sizeAvg;
      out.fixCmp.spanM = spanM;
    }
    if (fix.motion && isNum(fix.motion.speed)) {
      const v = logStateAt(pts, tMs);
      if (v && isNum(v.speedMs)) {
        out.fixCmp.speedFix = fix.motion.speed;
        out.fixCmp.speedLog = v.speedMs;
        if (isNum(fix.motion.heading) && isNum(v.headDeg) && v.speedMs > 0.5)
          out.fixCmp.headErr = ((fix.motion.heading - v.headDeg + 540) % 360) - 180;
      }
    }
  }
  return out;
}

/* grade the calibration the way the app grades a fix — honest words, not
   just numbers. Position by error-as-%-of-range (the quantity triangulation
   actually controls), size by ratio. */
export function gradeCalibration(sum) {
  if (!sum) return null;
  const g = {};
  if (sum.fixCmp && isNum(sum.fixCmp.errPct)) {
    const p = sum.fixCmp.errPct;
    g.pos = p < 3 ? "excellent" : p < 8 ? "good" : p < 18 ? "fair" : "poor";
  }
  if (sum.fixCmp && isNum(sum.fixCmp.sizeRatio)) {
    const r = Math.abs(Math.log(sum.fixCmp.sizeRatio));
    g.size = r < Math.log(1.25) ? "excellent" : r < Math.log(1.6) ? "good" : r < Math.log(2.5) ? "fair" : "poor";
  }
  g.dir = sum.sepMax < 1 ? "excellent" : sum.sepMax < 2.5 ? "good" : sum.sepMax < 6 ? "fair" : "poor";
  const order = { excellent: 0, good: 1, fair: 2, poor: 3 };
  const worst = ["pos", "size", "dir"].filter((k) => g[k]).sort((a, b) => order[g[b]] - order[g[a]])[0];
  g.overall = g[worst];
  return g;
}
