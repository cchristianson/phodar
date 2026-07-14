import React, { useState, useRef, useEffect, useMemo, useCallback } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import { D2R, R2D, RAD, clampN, dot, sub, add, scl, unit, geoFromEnu, dirFromAzEl, dirToAzEl } from "./math/geodesy.js";
import { isNum, n1, fmtLenShort, fmtSpeed, fmtDeg, compass8, setImperialUnits } from "./math/format.js";
import { photoBasis, angSizeFromPoints, pixelDirFromAnchor } from "./math/projection.js";
import { analyze, arbitrateBearings, aspectSpan } from "./math/triangulate.js";
import { trackDirections, kinematics, analyzeTracks } from "./math/kinematics.js";
import { sunPos, moonPos, moonFrac, raDecToAzEl } from "./math/astro.js";
import { fetchAircraft, fetchAircraftAt, fetchAcInfo, rankCandidates, radiusNmForSources, acAzElRange } from "./checks/adsb.js";
import { declination } from "./math/geomag.js";
import { loadSats, satsAt, satTrail } from "./checks/satellites.js";
import { predictedSkyline, skylineElAt, demElevation, TERRAIN_ATTRIB } from "./terrain.js";
import { mediaPut, mediaGet, mediaDel, mediaClear } from "./mediaStore.js";
import { planetPositions } from "./math/planets.js";
import { STARS } from "./math/starcat.js";

/* ============================================================
   PHODAR — PHOtogrammetric Detection And Ranging
   (stereo sighting triangulator)
   Two observers + compass bearings + elevation angles
   -> 3D position fix, true size, altitude, speed & heading.
   ============================================================ */


/* ---------- reference objects for size context ---------- */
const REF_OBJECTS = [
  { name: "Mylar balloon", size: 0.5 },
  { name: "Bird (hawk)", size: 1.2 },
  { name: "Consumer drone", size: 0.35 },
  { name: "Car-sized craft", size: 4.5 },
  { name: "Cessna 172", size: 11 },
  { name: "Fighter jet", size: 15 },
  { name: "Airliner (737)", size: 36 },
  { name: "Airliner (747)", size: 68 },
  { name: "Football field", size: 100 },
];

const FOV_PRESETS = [
  { label: "Phone main (1×) ≈ 68°", v: 68 },
  { label: "Phone ultra-wide (0.5×) ≈ 104°", v: 104 },
  { label: "Phone 2× ≈ 37°", v: 37 },
  { label: "Phone 3× ≈ 25°", v: 25 },
  { label: "Phone 5× ≈ 15°", v: 15 },
];

const blankMomentA = () => ({ t: "", az: "", el: "", p1: null, p2: null, angManual: "", videoTime: null });
const blankMomentB = () => ({ t: "", az: "", el: "", pb: null, videoTime: null });
const makeSource = (i) => ({
  id: Math.random().toString(36).slice(2, 9),
  name: `Observer ${i}`,
  lat: "", lon: "", alt: "",
  whenMs: Date.now(),
  fovH: 68,
  track: [],
  natW: null, natH: null,
  A: blankMomentA(),
  B: blankMomentB(),
  open: true,
});



/* speed + felt-load strip chart */
function TrajChart({ k }) {
  const ref = useRef(null);
  useEffect(() => {
    const cv = ref.current; if (!cv || !k || !k.segs.length) return;
    const dpr = window.devicePixelRatio || 1, W = cv.clientWidth, H = 150;
    cv.width = W * dpr; cv.height = H * dpr;
    const ctx = cv.getContext("2d"); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.fillStyle = "#08101F"; ctx.fillRect(0, 0, W, H);
    const t0 = k.segs[0].t, t1 = k.segs[k.segs.length - 1].t;
    const X = (t) => 8 + ((t - t0) / Math.max(t1 - t0, 1e-9)) * (W - 16);
    const maxS = Math.max(...k.segs.map((s) => s.speed), 1e-9);
    const maxL = Math.max(1.4, ...k.acc.map((a) => a.load));
    const Ys = (v) => H - 18 - (v / maxS) * (H - 40);
    const Yl = (v) => H - 18 - (v / maxL) * (H - 40);
    ctx.strokeStyle = "rgba(245,169,63,.35)"; ctx.setLineDash([3, 4]);
    ctx.beginPath(); ctx.moveTo(8, Yl(1)); ctx.lineTo(W - 8, Yl(1)); ctx.stroke(); ctx.setLineDash([]);
    ctx.strokeStyle = "#5FD3BC"; ctx.lineWidth = 2; ctx.beginPath();
    k.segs.forEach((s, i) => { const x = X(s.t), y = Ys(s.speed); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
    ctx.stroke();
    if (k.acc.length) {
      ctx.strokeStyle = "#F5A93F"; ctx.beginPath();
      k.acc.forEach((a, i) => { const x = X(a.t), y = Yl(a.load); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      ctx.stroke();
    }
    ctx.font = "10px ui-monospace,Menlo,monospace";
    ctx.fillStyle = "#5FD3BC"; ctx.fillText("speed — peak " + n1(k.peakSpeed * 2.23694) + " mph", 8, 12);
    ctx.fillStyle = "#F5A93F"; ctx.fillText("felt load — peak " + (k.peakLoad != null ? k.peakLoad.toFixed(1) + " g" : "—") + "  (dashed = 1 g)", 8, 24);
    ctx.fillStyle = "#8A96B3"; ctx.fillText(k.dur.toFixed(1) + " s", W - 44, H - 6);
  }, [k]);
  return <canvas ref={ref} style={{ width: "100%", height: 150, borderRadius: 10, border: "1px solid var(--line)", display: "block" }} />;
}

function TrajectoryStereoSection({ stereo }) {
  const k = stereo.k;
  return (
    <Section title="Trajectory & kinematics (triangulated)">
      <TrajChart k={k} />
      <div className="grid2" style={{ marginTop: 12 }}>
        <div>
          <ML>Peak speed</ML>
          <div className="readout" style={{ fontSize: 26 }}>{n1(k.peakSpeed * 2.23694)} mph</div>
          <div className="readsub">{fmtSpeed(k.peakSpeed)}</div>
        </div>
        <div>
          <ML>Peak felt load</ML>
          <div className="readout amber" style={{ fontSize: 26 }}>{k.peakLoad != null ? k.peakLoad.toFixed(1) + " g" : "—"}</div>
          <div className="readsub">{k.peakA != null ? n1(k.peakA) + " m/s² coordinate accel" : "needs ≥4 samples"}</div>
        </div>
      </div>
      <table className="tbl" style={{ marginTop: 10 }}>
        <tbody>
          <tr><td>Average speed</td><td>{n1(k.avgSpeed * 2.23694)} mph</td></tr>
          <tr><td>Peak turn rate</td><td>{k.peakTurn != null ? n1(k.peakTurn) + " °/s" : "—"}</td></tr>
          <tr><td>Path length / duration</td><td>{fmtLenShort(k.path)} / {k.dur.toFixed(1)} s</td></tr>
          <tr><td>Samples / observers</td><td>{k.n} / {stereo.nObs}</td></tr>
          <tr><td>Avg ray miss along track</td><td>{fmtLenShort(stereo.avgMiss)}</td></tr>
        </tbody>
      </table>
      <div style={{ marginTop: 8, fontSize: 11, color: "var(--dim)" }}>
        A steady airliner reads ~1 g; a hard fighter turn ~7–9 g. Differentiation amplifies tap jitter — mark every few frames rather than every frame, and distrust single-sample g spikes that the speed trace doesn't corroborate.
      </div>
    </Section>
  );
}

function SoloTrackSection({ solo, t, setT }) {
  const D = Math.pow(10, Math.log10(50) + clampN(+t, 0, 1) * (Math.log10(50000) - Math.log10(50)));
  const gAt = (k, d) => (k.peakA != null ? (d * k.peakA) / 9.81 : null);
  return (
    <Section title="Trajectory — single observer (needs assumed distance)">
      <div style={{ fontSize: 12, color: "var(--dim)", marginBottom: 10 }}>
        One viewpoint gives the angular path only. Every result below scales with the distance you assume — and radial motion is invisible, so speeds and g are lower bounds on the transverse component.
      </div>
      <ML>Assumed distance</ML>
      <div className="readout amber" style={{ fontSize: 22 }}>{fmtLenShort(D)}</div>
      <input type="range" min={0} max={1} step={0.001} value={t} onChange={(e) => setT(e.target.value)} />
      <div style={{ display: "flex", justifyContent: "space-between", fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)" }}>
        <span>50 m</span><span>50 km</span>
      </div>
      {solo.map((s, i) => {
        const k = s.k;
        const g = gAt(k, D);
        const felt = g != null ? Math.sqrt(g * g + 1) : null;
        return (
          <div key={i}>
            <div className="hr" />
            <ML style={{ color: "var(--track)" }}>{s.name} — {k.n} pts · {k.dur.toFixed(1)} s · peak {n1(k.peakSpeed * R2D)}°/s across the sky</ML>
            <div className="grid2" style={{ marginTop: 6 }}>
              <div>
                <ML>Speed at {fmtLenShort(D)}</ML>
                <div className="readout" style={{ fontSize: 22 }}>{n1(D * k.peakSpeed * 2.23694)} mph</div>
                <div className="readsub">avg {n1(D * k.avgSpeed * 2.23694)} mph</div>
              </div>
              <div>
                <ML>Maneuver load</ML>
                <div className="readout amber" style={{ fontSize: 22 }}>{felt != null ? felt.toFixed(1) + " g" : "—"}</div>
                <div className="readsub">{g != null ? g.toFixed(1) + " g maneuver + gravity" : "needs ≥4 points"}</div>
              </div>
            </div>
            {k.peakA != null && k.peakA > 1e-6 && (
              <table className="tbl" style={{ marginTop: 8 }}>
                <thead><tr><th>For the maneuver to stay…</th><th>It must be within</th></tr></thead>
                <tbody>
                  {[[1, "≤ 1 g — balloons, birds, drones"], [3, "≤ 3 g — sporty light aircraft"], [9, "≤ 9 g — fighter-jet limit"], [25, "≤ 25 g — beyond crewed airframes"]].map(([n, lbl]) => (
                    <tr key={n}><td>{lbl}</td><td style={{ color: "var(--teal)" }}>{fmtLenShort((n * 9.81) / k.peakA)}</td></tr>
                  ))}
                </tbody>
              </table>
            )}
          </div>
        );
      })}
    </Section>
  );
}

/* ============================================================
   UI ATOMS
   ============================================================ */
const css = `
:root{
  --bg:#070B14; --panel:#0E1526; --panel2:#131D33; --line:#24345C;
  --ink:#DCE3F2; --dim:#8A96B3; --amber:#F5A93F; --teal:#5FD3BC; --red:#E8604C; --track:#8FB4FF;
  --mono:ui-monospace,'SF Mono','Roboto Mono',Menlo,Consolas,monospace;
}
*{box-sizing:border-box; -webkit-tap-highlight-color:transparent;}
.phodar{background:var(--bg); color:var(--ink); min-height:100vh;
  font-family:-apple-system,'Segoe UI',Roboto,system-ui,sans-serif; font-size:14px;
  padding-bottom:84px;}
.phodar input,.phodar select{
  background:#0A1122; border:1px solid var(--line); color:var(--ink);
  border-radius:8px; padding:9px 10px; font-size:16px; width:100%;
  font-family:var(--mono); outline:none;}
.phodar input:focus,.phodar select:focus{border-color:var(--amber);}
.phodar input[type=range]{padding:0; height:34px; accent-color:var(--amber); font-size:14px;}
.btn{background:var(--panel2); border:1px solid var(--line); color:var(--ink);
  border-radius:8px; padding:10px 14px; font-size:13px; font-weight:600; cursor:pointer;
  letter-spacing:.03em;}
.btn:active{transform:translateY(1px);}
.btn.amber{background:#3A2B10; border-color:#7A5A22; color:var(--amber);}
.btn.teal{background:#0E2B26; border-color:#2A6157; color:var(--teal);}
.btn.ghost{background:transparent;}
.btn.sm{padding:6px 10px; font-size:12px;}
.chip{display:inline-block; background:#0A1122; border:1px solid var(--line);
  border-radius:999px; padding:4px 10px; font-size:11px; color:var(--dim);
  margin:2px 4px 2px 0; cursor:pointer; font-family:var(--mono);}
.card{background:var(--panel); border:1px solid var(--line); border-radius:14px;
  padding:14px; margin:10px 12px;}
.microlabel{font-size:10px; letter-spacing:.16em; text-transform:uppercase;
  color:var(--dim); font-weight:700; margin-bottom:4px;}
.readout{font-family:var(--mono); font-size:22px; color:var(--teal); font-weight:600;
  line-height:1.15;}
.readout.amber{color:var(--amber);}
.readsub{font-family:var(--mono); font-size:12px; color:var(--dim); margin-top:2px;}
.grid2{display:grid; grid-template-columns:1fr 1fr; gap:8px;}
.grid3{display:grid; grid-template-columns:1fr 1fr 1fr; gap:8px;}
.hr{border-top:1px dashed var(--line); margin:12px 0;}
.tabbar{position:fixed; bottom:0; left:0; right:0; display:flex;
  background:rgba(10,15,28,.96); backdrop-filter:blur(8px);
  border-top:1px solid var(--line); z-index:50; padding-bottom:env(safe-area-inset-bottom);}
.tab{flex:1; padding:12px 4px 10px; text-align:center; font-size:10px;
  letter-spacing:.12em; text-transform:uppercase; font-weight:700; color:var(--dim);
  border:none; background:none; cursor:pointer;}
.tab.on{color:var(--amber);}
.tab .ic{display:block; font-size:17px; margin-bottom:3px; letter-spacing:0;}
.warn{background:#2A1512; border:1px solid #5A2C24; color:#F0A79A;
  border-radius:10px; padding:10px 12px; font-size:13px; margin-top:10px;}
.ok{background:#0E2B26; border:1px solid #2A6157; color:var(--teal);
  border-radius:10px; padding:10px 12px; font-size:13px; margin-top:10px;}
.tbl{width:100%; border-collapse:collapse; font-family:var(--mono); font-size:12px;}
.tbl td,.tbl th{padding:7px 6px; border-bottom:1px solid var(--line); text-align:right;}
.tbl th{color:var(--dim); font-weight:600; font-size:10px; letter-spacing:.1em;
  text-transform:uppercase;}
.tbl td:first-child,.tbl th:first-child{text-align:left; font-family:inherit;}
.marker{position:absolute; width:26px; height:26px; margin:-13px 0 0 -13px;
  border-radius:50%; border:2px solid; display:flex; align-items:center;
  justify-content:center; font-size:10px; font-weight:800; font-family:var(--mono);
  touch-action:none; pointer-events:none;}
.pinmapwrap{position:relative; isolation:isolate; z-index:0; height:240px;
  border-radius:10px; border:1px solid var(--line); overflow:hidden;
  background:#08101F;}
.pinmapwrap .leaflet-container{width:100%; height:100%; background:#08101F;
  font-family:var(--mono); cursor:grab;}
.pinmapwrap .leaflet-container:active{cursor:grabbing;}
.pinmapwrap .leaflet-control-attribution{background:rgba(7,11,20,.55);
  color:var(--dim); font-size:8px; padding:1px 4px;}
.pinmapwrap .leaflet-control-attribution a{color:var(--dim);}
.pinmap-street-tiles{filter:invert(1) hue-rotate(180deg) brightness(.85)
  contrast(.9) saturate(.55);}
.pinmap-cross{position:absolute; left:50%; top:50%; margin:-14px 0 0 -14px;
  z-index:900; pointer-events:none; filter:drop-shadow(0 1px 2px rgba(0,0,0,.8));}
.pinmap-you{position:absolute; left:50%; top:50%; margin:-26px 0 0 12px;
  z-index:900; pointer-events:none; color:var(--teal); font-family:var(--mono);
  font-size:9px; font-weight:700; text-shadow:0 1px 2px #000;}
.pinmap-hud{position:absolute; left:8px; bottom:6px; z-index:900;
  pointer-events:none; font-family:var(--mono); font-size:10px; color:var(--ink);
  text-shadow:0 1px 2px #000; line-height:1.5;}
.lmk{position:relative; transform:translate(-50%,-50%); color:var(--amber);
  font-size:11px; text-align:center; white-space:nowrap; width:0; height:0;
  display:flex; align-items:center; justify-content:center;
  text-shadow:0 1px 2px #000; font-family:var(--mono);}
.lmk span{position:absolute; left:8px; top:-6px; font-size:10px; color:var(--ink);}
.lmk-dot{font-size:13px;}
.lmk-fix{color:var(--teal); font-size:17px; font-weight:800;}
.lmk-fix span{color:var(--teal); font-weight:700;}
.plotwrap{position:relative; isolation:isolate; z-index:0; height:300px;
  border-radius:10px; border:1px solid var(--line); overflow:hidden;
  background:#08101F;}
.plotwrap .leaflet-container{width:100%; height:100%; background:#08101F;
  font-family:var(--mono);}
.plotwrap .leaflet-control-attribution{background:rgba(7,11,20,.55);
  color:var(--dim); font-size:8px; padding:1px 4px;}
.plotwrap .leaflet-control-attribution a{color:var(--dim);}
.plotwrap .leaflet-control-scale-line{background:rgba(7,11,20,.6);
  color:var(--ink); border-color:rgba(138,150,179,.6); font-size:9px;}
.plotwrap .leaflet-bar a{background:var(--panel2); color:var(--ink);
  border-color:var(--line);}
/* portrait lock — phones only (coarse pointer + landscape-phone height).
   Tablets and desktops never match. */
.rotate-lock{display:none;}
@media screen and (orientation: landscape) and (pointer: coarse) and (max-height: 520px){
  .rotate-lock{display:flex; position:fixed; inset:0; z-index:9999;
    background:var(--bg); color:var(--ink); flex-direction:column;
    align-items:center; justify-content:center; gap:14px; text-align:center;
    font-family:var(--mono); padding:20px;}
  .rotate-lock .ic{font-size:44px; animation:rot-nudge 1.6s ease-in-out infinite;}
  @keyframes rot-nudge{0%,100%{transform:rotate(0)}50%{transform:rotate(-90deg)}}
}
`;

const ML = ({ children, style }) => <div className="microlabel" style={style}>{children}</div>;

function Num({ label, value, onChange, unit, ph, after }) {
  return (
    <div>
      {label && <ML>{label}{unit ? <span style={{ opacity: .7 }}> ({unit})</span> : null}</ML>}
      <input inputMode="decimal" value={value ?? ""} placeholder={ph || ""}
        onChange={(e) => onChange(e.target.value)} />
      {after}
    </div>
  );
}

function Section({ title, right, children, collapsible, defaultOpen = true }) {
  const [open, setOpen] = useState(defaultOpen);
  return (
    <div className="card">
      <div onClick={collapsible ? () => setOpen((o) => !o) : undefined}
        style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: (!collapsible || open) ? 8 : 0, cursor: collapsible ? "pointer" : "default" }}>
        <ML style={{ marginBottom: 0, color: "var(--ink)" }}>{title}{collapsible ? (open ? "  ▾" : "  ▸") : ""}</ML>
        {right}
      </div>
      {(!collapsible || open) && children}
    </div>
  );
}

/* Checklist step inside an observer card: status dot + one-line summary */

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
function parseMovMeta(u8) {
  const out = {};
  const txt = new TextDecoder("latin1").decode(u8.subarray(0, Math.min(u8.length, 3000000)));
  const m = txt.match(/([+-]\d{1,2}\.\d{2,})([+-]\d{1,3}\.\d{2,})([+-]\d+(\.\d+)?)?\//);
  if (m) { out.lat = +(+m[1]).toFixed(6); out.lon = +(+m[2]).toFixed(6); if (m[3]) out.alt = +(+m[3]).toFixed(1); }
  const mi = txt.indexOf("mvhd");
  if (mi > 0 && u8[mi + 4] === 0) {
    const p = mi + 8; // version(1)+flags(3) then creation u32 (seconds since 1904)
    const sec = ((u8[p] << 24) | (u8[p + 1] << 16) | (u8[p + 2] << 8) | u8[p + 3]) >>> 0;
    if (sec > 2082844800) out.timeMs = (sec - 2082844800) * 1000;
  }
  return Object.keys(out).length ? out : null;
}
function parseMediaMeta(buf, isVideo) {
  try {
    const u8 = new Uint8Array(buf);
    return isVideo ? parseMovMeta(u8) : parseJpegExif(u8);
  } catch (e) { return null; }
}

/* ============================================================
   MEDIA MEASURE — photo/video + tap-to-mark angular measurement
   ============================================================ */
function MediaMeasure({ src, update, wizard }) {
  const media = src.mediaUrl ? { url: src.mediaUrl, kind: src.mediaKind } : null;
  const fileRef = useRef(null);
  const [triedData, setTriedData] = useState(false);
  const [loading, setLoading] = useState(false);
  const [loadErr, setLoadErr] = useState("");
  const [active, setActive] = useState("shape");
  const [drag, setDrag] = useState(false);
  const [dispW, setDispW] = useState(0);
  const [vidT, setVidT] = useState(0);
  const [vidDur, setVidDur] = useState(0);
  const [trkAdv, setTrkAdv] = useState(3); // frames to auto-advance after dropping a track point
  const [view, setView] = useState({ z: 1, ox: 0, oy: 0 }); // pinch-zoom/pan of the marking canvas
  const [finger, setFinger] = useState(null);               // last pointer pos (wrapper-relative) for the loupe
  const ptsRef = useRef(new Map());
  const pinchRef = useRef(null);
  const pendingRef = useRef(null); // undecided first touch: tap? drag? or pinch about to start?
  const holdRef = useRef(null);
  const [touching, setTouching] = useState(false); // any finger down on the canvas
  const wrapRef = useRef(null), mediaRef = useRef(null), loupeRef = useRef(null);

  const natW = src.natW, natH = src.natH;
  const scale = natW && dispW ? dispW / natW : 0;
  const dispH = natH && scale ? natH * scale : 0;
  const TT = (x, y) => [x * scale * view.z + view.ox, y * scale * view.z + view.oy];
  const clampView = (v) => {
    const cw = dispW || 1, chh = dispH || 1;
    return { z: v.z, ox: clampN(v.ox, Math.min(0, cw - cw * v.z), 0), oy: clampN(v.oy, Math.min(0, chh - chh * v.z), 0) };
  };

  /* --- 3D shape fit: projected silhouette writes A.p1/p2; pose is stored --- */
  const syncShape = (sf) => {
    const pr = shapeProjNat(sf);
    update({ shapeFit: sf, A: { ...src.A, p1: pr.p1, p2: pr.p2 } });
  };
  const updShape = (patch) => { if (src.shapeFit) syncShape({ ...src.shapeFit, ...patch }); };
  const startShape = (kind) => {
    if (src.shapeFit?.kind === kind) { setActive("shape"); return; }
    const cx = clampN(((dispW || 1) / 2 - view.ox) / (scale * view.z || 1), 0, natW || 100);
    const cy = clampN(((dispH || 1) / 2 - view.oy) / (scale * view.z || 1), 0, natH || 100);
    const base = {
      kind, cx, cy,
      sizeNat: (natW || 1000) * 0.14,
      aspect: 3, roll: 0,
      rotM: SHAPE_R0()[kind],
      hue: src.shapeFit?.hue ?? 36,
    };
    setActive("shape");
    syncShape(base);
  };
  const clearShape = () => { update({ shapeFit: null }); setActive("shape"); };
  const [shapeMag, setShapeMag] = useState(false);
  const magTimer = useRef(null);
  const shapeLoupeFor = (sf) => {
    if (!sf || !wrapRef.current) return;
    const onScreen = (sf.sizeNat || 0) * scale * (view.z || 1);
    if (onScreen > 130) { setShapeMag(false); return; } // big on screen — a magnifier adds nothing
    const r = wrapRef.current.getBoundingClientRect();
    const [sx2, sy2] = TT(sf.cx, sf.cy);
    setFinger({ x: sx2, y: sy2, cx: r.left + sx2, cy: r.top + sy2 });
    setShapeMag(true);
    if (magTimer.current) clearTimeout(magTimer.current);
    magTimer.current = setTimeout(() => setShapeMag(false), 1100);
    requestAnimationFrame(() => requestAnimationFrame(() => drawLoupe({ x: sf.cx, y: sf.cy }, sf)));
  };

  const rotRef = useRef(null);   // body-drag → 3D trackball
  const hDragRef = useRef(null); // center-grab → move
  /* sessions saved by the old 2D fitter lack sizeNat/rotM — upgrade in place */
  useEffect(() => {
    const sf = src.shapeFit;
    if (!sf || (sf.sizeNat != null && sf.rotM)) return;
    syncShape({
      kind: sf.kind || "orb", cx: sf.cx, cy: sf.cy,
      sizeNat: sf.sizeNat != null ? sf.sizeNat : (sf.L != null ? sf.L : (natW || 1000) * 0.14),
      aspect: sf.aspect || (sf.W && sf.L ? clampN(sf.L / Math.max(sf.W, 1), 1.4, 6) : 3),
      roll: 0,
      rotM: sf.rotM || SHAPE_R0()[sf.kind] || I3,
      hue: sf.hue ?? 36,
    });
  }, [src.id, src.shapeFit]); // eslint-disable-line

  const measureWrap = useCallback(() => {
    if (wrapRef.current) setDispW(wrapRef.current.clientWidth);
  }, []);
  useEffect(() => {
    measureWrap();
    window.addEventListener("resize", measureWrap);
    return () => window.removeEventListener("resize", measureWrap);
  }, [measureWrap, src.mediaUrl]);

  /* While dragging a marker, hard-lock page scroll & pull-to-refresh.
     pointermove preventDefault doesn't stop iOS scrolling — a non-passive
     touchmove listener at the document level does. */
  useEffect(() => {
    if (!drag && !touching) return;
    const prevent = (e) => { if (e.cancelable) e.preventDefault(); };
    document.addEventListener("touchmove", prevent, { passive: false });
    const prevOverflow = document.body.style.overflow;
    const prevOverscroll = document.body.style.overscrollBehavior;
    document.body.style.overflow = "hidden";
    document.body.style.overscrollBehavior = "none";
    return () => {
      document.removeEventListener("touchmove", prevent);
      document.body.style.overflow = prevOverflow;
      document.body.style.overscrollBehavior = prevOverscroll;
    };
  }, [drag, touching]);

  /* iOS ignores touch-action:none for multi-touch — hard-block native
     scroll & page pinch-zoom for ANY touch that starts on the marking
     canvas, attached natively so there's no React-timing gap. Buttons
     inside still work (taps produce no touchmove). */
  useEffect(() => {
    const el = wrapRef.current;
    if (!el) return;
    const block = (e) => { if (e.cancelable) e.preventDefault(); };
    const blockStart = (e) => {
      if (e.target && e.target.closest && e.target.closest("button")) return; // keep chip taps clickable
      if (e.cancelable) e.preventDefault();
    };
    el.addEventListener("touchstart", blockStart, { passive: false });
    el.addEventListener("touchmove", block, { passive: false });
    el.addEventListener("gesturestart", block);
    el.addEventListener("gesturechange", block);
    el.addEventListener("gestureend", block);
    return () => {
      el.removeEventListener("touchstart", blockStart);
      el.removeEventListener("touchmove", block);
      el.removeEventListener("gesturestart", block);
      el.removeEventListener("gesturechange", block);
      el.removeEventListener("gestureend", block);
    };
  }, [src.mediaUrl, loadErr]);

  const readAsDataURL = (f) =>
    new Promise((res, rej) => {
      const r = new FileReader();
      r.onload = () => res(r.result);
      r.onerror = () => rej(new Error("read failed"));
      r.readAsDataURL(f);
    });

  const onFile = async (e) => {
    const f = e.target.files && e.target.files[0];
    e.target.value = ""; // allow re-picking the same file
    if (!f) return;
    setLoadErr(""); setLoading(true);
    const kind = f.type.startsWith("video") ? "video" : "image";
    let url = null;
    try { url = URL.createObjectURL(f); } catch (err) { url = null; }
    if (!url) {
      try { url = await readAsDataURL(f); }
      catch (err) {
        setLoading(false);
        setLoadErr("Couldn't read that file — try a different photo or video.");
        return;
      }
    }
    fileRef.current = f; setTriedData(url.startsWith("data:"));
    update({
      mediaUrl: url, mediaKind: kind, mediaNorm: false,
      natW: null, natH: null, meta: null,
      A: { ...src.A, p1: null, p2: null }, B: { ...src.B, pb: null }, track: [],
    });
    if (kind === "video") mediaPut(src.id, { kind: "video", data: f }); // survives reload via IndexedDB
    /* mine the file for EXIF / QuickTime metadata and AUTO-APPLY it —
       the photo is the authority on its own capture conditions */
    f.arrayBuffer().then((buf) => {
      const m = parseMediaMeta(buf, kind === "video");
      if (!m) {
        if (/hei[cf]/i.test(f.type) || /\.hei[cf]$/i.test(f.name || "")) update({ meta: { heic: true } });
        return;
      }
      const patch = { meta: m };
      if (isNum(m.lat)) { patch.lat = String(m.lat); patch.lon = String(m.lon); }
      if (isNum(m.alt)) patch.alt = String(m.alt);
      if (m.timeMs) patch.whenMs = m.timeMs;
      if (isNum(m.fovH)) patch.fovH = m.fovH;
      if (isNum(m.az)) {
        /* MAGNETIC bearings become TRUE via WMM before anything uses them —
           declination runs to ±25° and would otherwise pass through silently */
        let azUse = +m.az;
        if (m.azRef === "magnetic" && isNum(m.lat) && isNum(m.lon)) {
          const dec = declination(+m.lat, +m.lon, isNum(m.alt) ? +m.alt : 0, new Date(m.timeMs || Date.now()));
          azUse = ((azUse + dec) % 360 + 360) % 360;
          patch.meta = { ...m, decl: +dec.toFixed(2), azTrue: +azUse.toFixed(1) };
        }
        patch.mediaAim = { az: azUse, el: 15, roll: 0 }; // pre-aims the sky placement
        if (!isNum(src.A?.az)) patch.A = { ...src.A, p1: null, p2: null, az: azUse.toFixed(1) };
      }
      update(patch);
    }).catch(() => { });
  };

  /* Sandboxed browsers sometimes refuse blob: URLs — retry as a data URL */
  const onMediaError = async () => {
    if (!triedData && fileRef.current) {
      try {
        const url = await readAsDataURL(fileRef.current);
        setTriedData(true);
        update({ mediaUrl: url });
        return;
      } catch (err) { /* fall through */ }
    }
    setLoading(false);
    setLoadErr("The browser refused to display that file. Try a smaller image (a screenshot of it works) or another format.");
  };

  const onLoaded = () => {
    const el = mediaRef.current;
    if (!el) return;
    setView({ z: 1, ox: 0, oy: 0 });
    if (media.kind === "video") {
      update({ natW: el.videoWidth, natH: el.videoHeight });
      setVidDur(el.duration || 0);
    } else {
      /* IMAGE: Safari can report UNROTATED naturals for EXIF-oriented photos
         while displaying them rotated — and it composes its hidden orientation
         transform with our Look-mode matrix3d unpredictably. Cure: bake the
         image through a canvas ONCE, producing clean EXIF-free pixels where
         naturals, display, and the homography box all agree. */
      let nw = el.naturalWidth, nh = el.naturalHeight;
      const cw = el.clientWidth, ch = el.clientHeight;
      if (cw > 0 && ch > 0 && nw > 0 && nh > 0) {
        const da = cw / ch, na = nw / nh;
        if (Math.abs(da - 1 / na) < Math.abs(da - na) * 0.5) { const s = nw; nw = nh; nh = s; }
      }
      if (!src.mediaNorm) {
        try {
          const MAX = 4300, sc = Math.min(1, MAX / Math.max(nw, nh)); // original res (iOS canvas ceiling) — sky warp uses its own 1280px texture
          const W = Math.max(1, Math.round(nw * sc)), Hh = Math.max(1, Math.round(nh * sc));
          const cv = document.createElement("canvas");
          cv.width = W; cv.height = Hh;
          cv.getContext("2d").drawImage(el, 0, 0, W, Hh); // modern browsers draw the ORIENTED image
          const durl = cv.toDataURL("image/jpeg", 0.94);
          update({ mediaUrl: durl, mediaNorm: true, natW: W, natH: Hh });
          mediaPut(src.id, { kind: "image", data: durl }); // survives reload via IndexedDB
          setLoading(false); setLoadErr("");
          measureWrap();
          return;
        } catch (err) { /* canvas blocked — fall through with guarded naturals */ }
      }
      update({ natW: nw, natH: nh, mediaNorm: true });
    }
    setLoading(false); setLoadErr("");
    measureWrap();
  };

  const toNat = (clientX, clientY) => {
    const r = wrapRef.current.getBoundingClientRect();
    let x = ((clientX - r.left) - view.ox) / (scale * view.z);
    let y = ((clientY - r.top) - view.oy) / (scale * view.z);
    x = Math.max(0, Math.min(natW, x)); y = Math.max(0, Math.min(natH, y));
    return { x, y };
  };

  const pts = () => ({
    p1: src.A.p1, p2: src.A.p2, pb: src.B.pb,
  });
  const setPt = (key, p) => {
    if (key === "pb") update({ B: { ...src.B, pb: p } });
    else update({ A: { ...src.A, [key]: p } });
  };

  const nearest = (p) => {
    const all = pts(); let best = null, bd = 34 / (scale * view.z);
    for (const k of ["p1", "p2", "pb"]) {
      const q = all[k]; if (!q) continue;
      const d = Math.hypot(q.x - p.x, q.y - p.y);
      if (d < bd) { bd = d; best = k; }
    }
    return best;
  };

  const killPending = () => {
    pendingRef.current = null;
    if (holdRef.current) { clearTimeout(holdRef.current); holdRef.current = null; }
  };
  /* Commit an undecided touch: as a drag (with loupe) or as a clean tap */
  const commitPending = (dragging, curNat, curFinger) => {
    const pd = pendingRef.current;
    if (!pd) return;
    killPending();
    if (pd.mode === "trk") {
      const el = mediaRef.current;
      const tv = el ? el.currentTime : vidT;
      update({ track: [...(src.track || []), { t: +tv.toFixed(3), x: pd.nat.x, y: pd.nat.y }] });
      if (trkAdv > 0) seek(Math.min(vidDur, tv + trkAdv * 0.03337));
      return;
    }
    if (pd.mode === "shapeMove") {
      if (!src.shapeFit) return;
      if (dragging) { hDragRef.current = { which: "c" }; setDrag(true); setFinger(null); }
      else {
        const nsf = { ...src.shapeFit, cx: (curNat || pd.nat).x, cy: (curNat || pd.nat).y };
        syncShape(nsf); shapeLoupeFor(nsf);
      }
      return;
    }
    if (pd.mode === "shape") {
      if (!src.shapeFit) return;
      if (dragging) {
        rotRef.current = { R0: src.shapeFit.rotM || I3, sx: pd.sx, sy: pd.sy }; // drag = rotate in 3D
        setDrag(true); setFinger(null);
      } else {
        const nsf = { ...src.shapeFit, cx: (curNat || pd.nat).x, cy: (curNat || pd.nat).y };
        syncShape(nsf); shapeLoupeFor(nsf);                                     // tap = move it here
      }
      return;
    }
    setActive(pd.key);
    const nat = curNat || pd.nat;
    setPt(pd.key, nat);
    if (dragging) {
      setDrag(true);
      setFinger(curFinger || { x: pd.fx, y: pd.fy, cx: pd.sx, cy: pd.sy });
      requestAnimationFrame(() => requestAnimationFrame(() => drawLoupe(nat))); // canvas mounts next frame
    } else if (pd.key === "p1" && !src.A.p2) setActive("p2");
  };

  const onDown = (e) => {
    if (!scale) return;
    e.preventDefault();
    const r = wrapRef.current.getBoundingClientRect();
    ptsRef.current.set(e.pointerId, { x: e.clientX, y: e.clientY });
    setTouching(true);
    try { e.currentTarget.setPointerCapture(e.pointerId); } catch (_) { }
    if (ptsRef.current.size >= 2) {
      /* second finger: a pinch — discard any undecided touch, place nothing */
      killPending();
      setDrag(false); setFinger(null);
      const [pa, pb2] = [...ptsRef.current.values()];
      pinchRef.current = {
        d: Math.hypot(pa.x - pb2.x, pa.y - pb2.y) || 1,
        mx: (pa.x + pb2.x) / 2 - r.left, my: (pa.y + pb2.y) / 2 - r.top,
        z: view.z, ox: view.ox, oy: view.oy,
      };
      return;
    }
    if (active === "trk" && media?.kind !== "video") return;
    if (active === "shape" && !src.shapeFit) return; // pick a shape first — no stray marks
    const p = toNat(e.clientX, e.clientY);
    const shapeMode = active === "shape" && !!src.shapeFit;
    let shapeCenter = false;
    if (shapeMode) {
      const [scx, scy] = TT(src.shapeFit.cx, src.shapeFit.cy);
      shapeCenter = Math.hypot((e.clientX - r.left) - scx, (e.clientY - r.top) - scy) < 22;
    }
    const hit = active === "trk" || shapeMode ? null : nearest(p);
    pendingRef.current = {
      id: e.pointerId,
      mode: active === "trk" ? "trk" : shapeMode ? (shapeCenter ? "shapeMove" : "shape") : "mark",
      key: active === "trk" || shapeMode ? null : (hit || active),
      nat: p, sx: e.clientX, sy: e.clientY,
      fx: e.clientX - r.left, fy: e.clientY - r.top,
    };
    if (active !== "trk") {
      holdRef.current = setTimeout(() => commitPending(true), 260); // press-and-hold → precision drag
    }
  };
  const onMove = (e) => {
    if (ptsRef.current.has(e.pointerId)) ptsRef.current.set(e.pointerId, { x: e.clientX, y: e.clientY });
    if (!scale || !wrapRef.current) return;
    const r = wrapRef.current.getBoundingClientRect();
    if (pinchRef.current && ptsRef.current.size >= 2) {
      const [pa, pb2] = [...ptsRef.current.values()];
      const pz = pinchRef.current;
      const nz = clampN((pz.z * (Math.hypot(pa.x - pb2.x, pa.y - pb2.y) || 1)) / pz.d, 1, 20);
      const cx = (pz.mx - pz.ox) / pz.z, cy = (pz.my - pz.oy) / pz.z;
      const mxNow = (pa.x + pb2.x) / 2 - r.left, myNow = (pa.y + pb2.y) / 2 - r.top;
      setView(clampView({ z: nz, ox: mxNow - cx * nz, oy: myNow - cy * nz }));
      return;
    }
    const pd = pendingRef.current;
    if (pd && pd.id === e.pointerId) {
      if (Math.hypot(e.clientX - pd.sx, e.clientY - pd.sy) > 7) {
        if (pd.mode === "trk") { killPending(); return; } // swipe in track mode places nothing
        commitPending(true, toNat(e.clientX, e.clientY), { x: e.clientX - r.left, y: e.clientY - r.top, cx: e.clientX, cy: e.clientY });
      }
      return;
    }
    if (!drag) return;
    const p = toNat(e.clientX, e.clientY);
    if (active === "shape") {
      if (hDragRef.current && src.shapeFit) {
        const nsf = { ...src.shapeFit, cx: p.x, cy: p.y };
        syncShape(nsf); shapeLoupeFor(nsf);
        return;
      }
      const rr = rotRef.current;
      if (rr && src.shapeFit) {
        const k = 0.45; // deg per px — front face follows the finger
        const dR = mul3(rotX3(-(e.clientY - rr.sy) * k), rotY3((e.clientX - rr.sx) * k));
        const nsf = { ...src.shapeFit, rotM: mul3(dR, rr.R0) };
        syncShape(nsf); shapeLoupeFor(nsf);
      }
      return;
    }
    setPt(active, p);
    setFinger({ x: e.clientX - r.left, y: e.clientY - r.top, cx: e.clientX, cy: e.clientY });
    drawLoupe(p);
  };
  const onUp = (e) => {
    ptsRef.current.delete(e.pointerId);
    try { e.currentTarget.releasePointerCapture(e.pointerId); } catch (_) { }
    if (ptsRef.current.size < 2) pinchRef.current = null;
    if (pendingRef.current && pendingRef.current.id === e.pointerId) {
      if (e.type === "pointercancel") killPending(); // OS ate the touch — place nothing
      else commitPending(false);                     // clean tap
    }
    if (ptsRef.current.size === 0) {
      setTouching(false);
      setFinger(null);
      rotRef.current = null;
      hDragRef.current = null;
      if (drag) {
        setDrag(false);
        if (active === "p1" && !src.A.p2) setActive("p2");
      }
    }
  };

  /* safety valve: app-switch or system gesture mid-touch must release the lock */
  useEffect(() => {
    const hardReset = () => {
      ptsRef.current.clear(); pinchRef.current = null;
      killPending(); setTouching(false); setDrag(false); setFinger(null);
    };
    window.addEventListener("blur", hardReset);
    document.addEventListener("visibilitychange", hardReset);
    return () => {
      window.removeEventListener("blur", hardReset);
      document.removeEventListener("visibilitychange", hardReset);
    };
  }, []); // eslint-disable-line

  const drawLoupe = (p, sf) => {
    const cv = loupeRef.current, el = mediaRef.current;
    if (!cv || !el || !natW) return;
    const ctx = cv.getContext("2d");
    const S = 110;
    const dprL = cv.width / S || 1;               // device-res backing store
    ctx.setTransform(dprL, 0, 0, dprL, 0, 0);     // draw in CSS px, render at native res
    let pxPerNat = Math.max(4, 3 * (view.z || 1) * scale); // never weaker than 4× absolute
    if (sf && sf.sizeNat) {
      /* frame the WHOLE shape: fit it to ~62% of the glass, never dropping
         below ~screen scale (at which point the loupe adds nothing) */
      pxPerNat = clampN((S * 0.62) / sf.sizeNat, 1.15 * (view.z || 1) * scale, 46);
    }
    const half = (S / 2) / pxPerNat;
    ctx.imageSmoothingEnabled = false;
    ctx.fillStyle = "#000"; ctx.fillRect(0, 0, S, S);
    try {
      ctx.drawImage(el, p.x - half, p.y - half, half * 2, half * 2, 0, 0, S, S);
    } catch (err) { /* ignore */ }
    const col = active === "pb" ? "#5FD3BC" : "#F5A93F";
    ctx.strokeStyle = col; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(S / 2, 0); ctx.lineTo(S / 2, S / 2 - 7);
    ctx.moveTo(S / 2, S / 2 + 7); ctx.lineTo(S / 2, S);
    ctx.moveTo(0, S / 2); ctx.lineTo(S / 2 - 7, S / 2);
    ctx.moveTo(S / 2 + 7, S / 2); ctx.lineTo(S, S / 2); ctx.stroke();
    ctx.beginPath(); ctx.arc(S / 2, S / 2, 7, 0, Math.PI * 2); ctx.stroke();
    if (sf) { // magnified wireframe so you can judge the fit at point-source scale
      const pr = shapeProjNat(sf);
      ctx.lineWidth = 1;
      for (const c of pr.curves) for (let i = 0; i < c.length - 1; i++) {
        const a2 = c[i], b2 = c[i + 1];
        ctx.strokeStyle = `hsla(${sf.hue ?? 36},88%,60%,${(0.25 + 0.75 * clampN((a2.z + b2.z) / 2 + 0.5, 0, 1)).toFixed(2)})`;
        ctx.beginPath();
        ctx.moveTo(S / 2 + (a2.x - p.x) * pxPerNat, S / 2 + (a2.y - p.y) * pxPerNat);
        ctx.lineTo(S / 2 + (b2.x - p.x) * pxPerNat, S / 2 + (b2.y - p.y) * pxPerNat);
        ctx.stroke();
      }
    }
  };

  const seek = (t) => {
    const el = mediaRef.current;
    if (el && media?.kind === "video") { el.currentTime = t; setVidT(t); }
  };

  const ang = angSizeFromPoints(src.A.p1, src.A.p2, natW, natH, +src.fovH);
  const markStyle = {
    p1: { borderColor: "var(--amber)", color: "var(--amber)" },
    p2: { borderColor: "var(--amber)", color: "var(--amber)" },
    pb: { borderColor: "var(--teal)", color: "var(--teal)" },
    trk: { borderColor: "var(--track)", color: "var(--track)" },
  };
  const markText = { p1: "A1", p2: "A2", pb: "B" };

  return (
    <div>
      <ML>Photo / video (optional — used to measure angular size)</ML>
      <div style={{ display: "flex", gap: 8, alignItems: "center", flexWrap: "wrap" }}>
        <label className="btn sm amber" style={{ display: "inline-block" }}>
          {media ? "Replace media" : "Load photo or video"}
          <input type="file" accept="image/*,video/*" onChange={onFile} style={{ display: "none" }} />
        </label>
        {media && (
          <div style={{ display: "flex", gap: 4, flexWrap: "wrap" }}>
            {(wizard ? [] : ["pb", ...(media.kind === "video" ? ["trk"] : [])]).map((k) => (
              <button key={k} className="btn sm" onClick={() => setActive(k)}
                style={active === k ? { borderColor: markStyle[k].borderColor, color: markStyle[k].color } : {}}>
                {k === "p1" ? "Edge 1" : k === "p2" ? "Edge 2" : k === "pb" ? "Pos @ B" : "Track"}
              </button>
            ))}
          </div>
        )}
      </div>

      {loadErr && <div className="warn">{loadErr}</div>}
      {loading && !natW && (
        <div style={{ marginTop: 10, padding: "18px 12px", border: "1px dashed var(--line)", borderRadius: 10, textAlign: "center", fontFamily: "var(--mono)", fontSize: 12, color: "var(--dim)" }}>
          Loading media…
        </div>
      )}
      {media && !loadErr && (
        <>
          <div style={{ display: "flex", gap: 4, flexWrap: "wrap", alignItems: "center", marginTop: 8 }}>
            <span className="microlabel" style={{ marginRight: 2, marginBottom: 0 }}>Fit a 3D shape:</span>
            {SHAPES.map((sh) => (
              <button key={sh.k} className={"btn sm" + (src.shapeFit?.kind === sh.k ? " amber" : "")}
                onClick={() => startShape(sh.k)}>{sh.label}</button>
            ))}
            {src.shapeFit && <button className="btn sm" onClick={clearShape} title="remove shape">✕</button>}
          </div>
          {src.shapeFit && (
            <>
              <div style={{ display: "flex", gap: 8, alignItems: "center", marginTop: 6 }}>
                <span className="microlabel" style={{ marginBottom: 0 }}>size</span>
                <input type="range" min={0} max={1} step={0.002}
                  value={(() => {
                    const lo = Math.log10(0.0012), hi = Math.log10(0.55);
                    const v = Math.log10(clampN((src.shapeFit.sizeNat || 1) / (natW || 1000), 0.0012, 0.55));
                    return (v - lo) / (hi - lo);
                  })()}
                  onChange={(e) => {
                    const lo = Math.log10(0.0012), hi = Math.log10(0.55);
                    const nsf = { ...src.shapeFit, sizeNat: (natW || 1000) * Math.pow(10, lo + (+e.target.value) * (hi - lo)) };
                    syncShape(nsf); shapeLoupeFor(nsf);
                  }} style={{ flex: 1 }} />
                <span className="microlabel" style={{ marginBottom: 0 }}>color</span>
                <input type="range" min={0} max={360} step={2} value={src.shapeFit.hue ?? 36}
                  onChange={(e) => updShape({ hue: +e.target.value })} style={{ width: 74 }} />
                <span style={{ width: 14, height: 14, borderRadius: 8, flex: "0 0 auto", background: `hsl(${src.shapeFit.hue ?? 36},88%,60%)`, border: "1px solid rgba(255,255,255,.35)" }} />
              </div>
              {src.shapeFit.kind === "capsule" && (
                <div style={{ display: "flex", gap: 8, alignItems: "center", marginTop: 4 }}>
                  <span className="microlabel" style={{ marginBottom: 0 }}>aspect {(src.shapeFit.aspect || 3).toFixed(1)}:1</span>
                  <input type="range" min={1.4} max={6} step={0.1} value={src.shapeFit.aspect || 3}
                    onChange={(e) => { const nsf = { ...src.shapeFit, aspect: +e.target.value }; syncShape(nsf); shapeLoupeFor(nsf); }} style={{ flex: 1 }} />
                </div>
              )}
              {(src.shapeFit.kind === "tri" || src.shapeFit.kind === "plane" || src.shapeFit.kind === "bird") && (
                <div style={{ display: "flex", gap: 8, alignItems: "center", marginTop: 4 }}>
                  <span className="microlabel" style={{ marginBottom: 0 }}>spin</span>
                  <input type="range" min={-180} max={180} step={1} value={src.shapeFit.roll || 0}
                    onChange={(e) => { const nsf = { ...src.shapeFit, roll: +e.target.value }; syncShape(nsf); shapeLoupeFor(nsf); }} style={{ flex: 1 }} />
                </div>
              )}
              {(() => {
                const pr = shapeProjNat(src.shapeFit);
                const aM = angSizeFromPoints(pr.p1, pr.p2, natW, natH, +src.fovH);
                const fpx = natW && isNum(src.fovH) ? (natW / 2) / Math.tan((+src.fovH * D2R) / 2) : null;
                const aN = fpx ? (pr.minorNat / fpx) * R2D : null;
                return aM != null ? (
                  <div style={{ fontFamily: "var(--mono)", fontSize: 11, color: "var(--amber)", marginTop: 4 }}>
                    projected {aM.toFixed(3)}°{aN != null ? ` × ${aN.toFixed(3)}° · aspect ${(aM / Math.max(aN, 1e-6)).toFixed(1)}:1` : ""} — drag the shape to rotate it in 3D · tap to move it
                  </div>
                ) : null;
              })()}
            </>
          )}
          <div style={{ position: "relative", marginTop: 10 }}>
            <div ref={wrapRef}
              style={{ position: "relative", borderRadius: 10, overflow: "hidden", border: "1px solid var(--line)", touchAction: "none", height: dispH || "auto" }}
              onPointerDown={onDown} onPointerMove={onMove} onPointerUp={onUp} onPointerCancel={onUp}>
              <div style={{ transform: `translate(${view.ox}px, ${view.oy}px) scale(${view.z})`, transformOrigin: "0 0", willChange: "transform" }}>
                {media.kind === "video" ? (
                  <video ref={mediaRef} src={media.url} playsInline muted preload="auto"
                    onLoadedMetadata={onLoaded} onError={onMediaError} onTimeUpdate={(e) => setVidT(e.target.currentTime)}
                    style={{ width: "100%", display: "block", pointerEvents: "none" }} />
                ) : (
                  <img ref={mediaRef} src={media.url} alt="sighting" onLoad={onLoaded} onError={onMediaError}
                    style={{ width: "100%", display: "block", pointerEvents: "none", imageRendering: view.z > 4 ? "pixelated" : "auto" }} draggable={false} />
                )}
              </div>
              {scale > 0 && Object.entries(pts()).map(([k, p]) => {
                if (!p) return null;
                if (src.shapeFit && (k === "p1" || k === "p2")) return null; // the shape shows these
                const [sx2, sy2] = TT(p.x, p.y);
                return (
                  <div key={k} className="marker"
                    style={{ left: sx2, top: sy2, ...markStyle[k], background: "rgba(7,11,20,.45)" }}>
                    {markText[k]}
                  </div>
                );
              })}
              {scale > 0 && !src.shapeFit && src.A.p1 && src.A.p2 && (() => {
                const A1 = TT(src.A.p1.x, src.A.p1.y), A2 = TT(src.A.p2.x, src.A.p2.y);
                return (
                  <svg style={{ position: "absolute", inset: 0, pointerEvents: "none" }} width="100%" height="100%">
                    <line x1={A1[0]} y1={A1[1]} x2={A2[0]} y2={A2[1]}
                      stroke="var(--amber)" strokeDasharray="4 4" strokeWidth="1.5" />
                  </svg>
                );
              })()}
              {scale > 0 && (src.track || []).length > 0 && (() => {
                const tp = [...src.track].filter((p) => p.x != null).sort((a, b) => a.t - b.t).map((p) => TT(p.x, p.y));
                return (
                  <svg style={{ position: "absolute", inset: 0, pointerEvents: "none" }} width="100%" height="100%">
                    <polyline points={tp.map((q) => q.join(",")).join(" ")}
                      fill="none" stroke="var(--track)" strokeWidth="1.5" strokeDasharray="2 3" />
                    {tp.map((q, i) => (
                      <circle key={i} cx={q[0]} cy={q[1]} r={i === tp.length - 1 ? 4 : 2.5}
                        fill={i === tp.length - 1 ? "var(--track)" : "rgba(143,180,255,.75)"} />
                    ))}
                  </svg>
                );
              })()}
              {scale > 0 && src.shapeFit && (() => {
                const sf = src.shapeFit;
                const pr = shapeProjNat(sf);
                const segs = [];
                for (const c of pr.curves) for (let i = 0; i < c.length - 1; i++) {
                  const a = c[i], b = c[i + 1];
                  segs.push({ a: TT(a.x, a.y), b: TT(b.x, b.y), z: (a.z + b.z) / 2 });
                }
                const [sx, sy] = TT(sf.cx, sf.cy);
                const op = 0.8;
                const col = `hsl(${sf.hue ?? 36},88%,60%)`;
                return (
                  <svg style={{ position: "absolute", inset: 0, pointerEvents: "none", overflow: "visible" }} width="100%" height="100%">
                    {segs.map((s2, i) => (
                      <line key={i} x1={s2.a[0]} y1={s2.a[1]} x2={s2.b[0]} y2={s2.b[1]}
                        stroke={col} strokeWidth={s2.z > 0 ? 1.6 : 0.9}
                        opacity={op * (0.22 + 0.78 * clampN(s2.z + 0.5, 0, 1))} />
                    ))}
                    <g style={{ pointerEvents: "none" }}>
                      <circle cx={sx} cy={sy} r={3.5} fill="var(--teal)" />
                      <circle cx={sx} cy={sy} r={8.5} fill="none" stroke="var(--teal)" strokeWidth="1" opacity="0.75" />
                    </g>
                  </svg>
                );
              })()}
              {view.z > 1.01 && (
                <button className="btn sm" onPointerDown={(e) => e.stopPropagation()}
                  onClick={() => setView({ z: 1, ox: 0, oy: 0 })}
                  style={{ position: "absolute", left: 8, bottom: 8, zIndex: 4 }}>
                  ×{view.z.toFixed(1)} · reset
                </button>
              )}
            </div>
            {(drag || shapeMag) && finger && (() => {
              /* viewport-fixed: always centered directly above the fingertip,
                 free to overlap toolbars/cards above the viewing window */
              const S = 110;
              const vw = typeof window !== "undefined" ? window.innerWidth : 400;
              const left = clampN((finger.cx ?? 0) - S / 2, 4, vw - S - 4);
              const top = Math.max((finger.cy ?? 0) - S - 30, 6);
              const dprL = Math.min(window.devicePixelRatio || 1, 3);
              return (
                <canvas ref={loupeRef} width={S * dprL} height={S * dprL}
                  style={{ position: "fixed", left, top, width: S, height: S, borderRadius: 14, zIndex: 60, pointerEvents: "none", background: "#000", border: `2px solid ${active === "pb" ? "var(--teal)" : "var(--amber)"}`, boxShadow: "0 4px 14px rgba(0,0,0,.55)" }} />
              );
            })()}
          </div>

          {media.kind === "video" && (
            <div style={{ marginTop: 8 }}>
              <input type="range" min={0} max={vidDur || 0} step={0.033} value={vidT}
                onChange={(e) => seek(+e.target.value)} />
              <div style={{ display: "flex", gap: 6, alignItems: "center", flexWrap: "wrap" }}>
                <button className="btn sm" onClick={() => seek(Math.max(0, vidT - 0.033))}>−1 fr</button>
                <button className="btn sm" onClick={() => seek(Math.min(vidDur, vidT + 0.033))}>+1 fr</button>
                <span style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--dim)" }}>{vidT.toFixed(2)} s</span>
                <button className="btn sm amber" onClick={() => update({ A: { ...src.A, t: vidT.toFixed(2), videoTime: vidT } })}>
                  {wizard ? "✓ Use this frame" : "Set time A"}
                </button>
                {!wizard && <button className="btn sm teal" onClick={() => update({ B: { ...src.B, t: vidT.toFixed(2), videoTime: vidT } })}>Set time B</button>}
              </div>
              {!wizard && (active === "trk" || (src.track || []).length > 0) && (
                <div style={{ marginTop: 8, padding: "8px 10px", border: "1px solid var(--track)", borderRadius: 10, background: "rgba(143,180,255,.06)" }}>
                  <div style={{ display: "flex", gap: 6, alignItems: "center", flexWrap: "wrap" }}>
                    <span style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--track)", fontWeight: 700 }}>
                      Track · {(src.track || []).length} pts
                    </span>
                    <span style={{ fontSize: 11, color: "var(--dim)" }}>step after tap:</span>
                    <select value={trkAdv} onChange={(e) => setTrkAdv(+e.target.value)} style={{ width: "auto", padding: "4px 8px", fontSize: 12 }}>
                      <option value={0}>off</option><option value={1}>1 fr</option><option value={2}>2 fr</option>
                      <option value={3}>3 fr</option><option value={6}>6 fr</option><option value={15}>15 fr</option>
                    </select>
                    <button className="btn sm" disabled={!(src.track || []).length}
                      onClick={() => update({ track: (src.track || []).slice(0, -1) })}>Undo</button>
                    <button className="btn sm" disabled={!(src.track || []).length} style={{ color: "var(--red)" }}
                      onClick={() => update({ track: [] })}>Clear</button>
                  </div>
                  {(() => {
                    if ((src.track || []).length < 3) return (
                      <div style={{ marginTop: 6, fontSize: 11, color: "var(--dim)" }}>
                        Scrub to the frame Moment A's bearing was taken, tap the object for point 1 (it anchors the absolute direction), then keep tapping as the video steps forward.
                      </div>
                    );
                    const d = trackDirections(src);
                    if (!d || d.length < 3) return (
                      <div style={{ marginTop: 6, fontSize: 11, color: "var(--amber)" }}>
                        Enter Moment A bearing + elevation to anchor the track's absolute direction.
                      </div>
                    );
                    const k = kinematics(d.map((x) => x.ct), d.map((x) => x.d));
                    return k ? (
                      <div style={{ marginTop: 6, fontSize: 11, fontFamily: "var(--mono)", color: "var(--dim)" }}>
                        span {k.dur.toFixed(2)} s · peak {n1(k.peakSpeed * R2D)}°/s across the sky
                      </div>
                    ) : null;
                  })()}
                </div>
              )}
            </div>
          )}

          <div style={{ marginTop: 8, fontSize: 12, color: "var(--dim)" }}>
            {wizard
              ? <>Fit a 3D shape to the object: <b style={{ color: "var(--amber)" }}>drag it to rotate in 3D</b>, tap to move it, drag the <b style={{ color: "var(--teal)" }}>center dot</b> to fine-place, sliders for size &amp; spin. Pinch to zoom in on small objects — the loupe follows your adjustments.</>
              : <>Fit a 3D shape to the object — drag rotates it in 3D, tap moves it, sliders set size. The projected silhouette becomes the measurement.
            Optionally mark where it moved to (<b style={{ color: "var(--teal)" }}>B</b>), or for video switch to <b style={{ color: "var(--track)" }}>Track</b> and tap the object frame by frame.
            Pinch with two fingers to zoom (two-finger drag pans) — a second finger never places points.</>}
          </div>
        </>
      )}

      {wizard && isNum(src.meta?.fovH) ? (
        <div style={{ marginTop: 10, fontSize: 12, color: "var(--dim)", fontFamily: "var(--mono)" }}>
          FOV {(+src.fovH).toFixed(1)}° — from the lens metadata ✓
        </div>
      ) : (
      <div className="grid2" style={{ marginTop: 10 }}>
        <div>
          <ML>Camera field of view</ML>
          <select value={FOV_PRESETS.some(p => p.v === +src.fovH) ? src.fovH : "custom"}
            onChange={(e) => e.target.value !== "custom" && update({ fovH: +e.target.value })}>
            {FOV_PRESETS.map((p) => <option key={p.v} value={p.v}>{p.label}</option>)}
            <option value="custom">Custom…</option>
          </select>
        </div>
        <Num label="FOV horizontal" unit="°" value={src.fovH} onChange={(v) => update({ fovH: v })} />
      </div>
      )}

      {src.meta && !src.meta.heic && (
        <div style={{ marginTop: 10, padding: "8px 10px", border: "1px solid var(--amber)", borderRadius: 10, background: "rgba(245,169,63,.06)" }}>
          <ML style={{ color: "var(--amber)" }}>📎 Auto-filled from the file ✓</ML>
          <div style={{ fontFamily: "var(--mono)", fontSize: 11, color: "var(--dim)", lineHeight: 1.6 }}>
            {isNum(src.meta.lat) && <div>GPS {src.meta.lat}, {src.meta.lon}{isNum(src.meta.alt) ? ` · ${src.meta.alt} m` : ""}</div>}
            {src.meta.timeMs && <div>{new Date(src.meta.timeMs).toLocaleString()}</div>}
            {isNum(src.meta.az) && <div>camera bearing {src.meta.az}° {src.meta.azRef}{src.meta.azRef === "magnetic" ? (isNum(src.meta.azTrue) ? ` → ${src.meta.azTrue}° true (WMM declination ${src.meta.decl >= 0 ? "+" : ""}${src.meta.decl}°)` : " (true ≈ magnetic + local declination)") : ""}</div>}
            {isNum(src.meta.fovH) && <div>FOV {src.meta.fovH}° (from {src.meta.f35} mm-eq lens)</div>}
            {src.meta.model && <div>{src.meta.model}</div>}
          </div>
          <div style={{ marginTop: 4, fontSize: 11, color: "var(--dim)" }}>
            Position, time, FOV{src.meta.az != null ? ", bearing & photo placement" : ""} were applied — every field below stays editable.
          </div>
        </div>
      )}
      {src.meta?.heic && (
        <div style={{ marginTop: 10, fontSize: 11, color: "var(--dim)" }}>
          📎 HEIC file — metadata unreadable here. Export or share as JPEG to auto-fill GPS, time, bearing, and FOV.
        </div>
      )}

      <div style={{ marginTop: 10 }}>
        <ML>Measured angular size</ML>
        <div className="readout amber" style={{ fontSize: 18 }}>
          {ang != null ? `${ang.toFixed(3)}°` : "— fit a shape above —"}
          {ang != null && <span style={{ fontSize: 12, color: "var(--dim)" }}>  ({(ang / 0.52).toFixed(1)}× full-moon width)</span>}
        </div>
      </div>
    </div>
  );
}

/* ============================================================
   SKY AIMER — POV sky view (ported from Sky Sense) used as an
   input instrument: aim the crosshair where the object was and
   capture azimuth/elevation. Sun & Moon are computed for the
   sighting time/place as calibration anchors.
   ============================================================ */
function toLocalInput(d) { const p = (n) => String(n).padStart(2, "0"); return d.getFullYear() + "-" + p(d.getMonth() + 1) + "-" + p(d.getDate()) + "T" + p(d.getHours()) + ":" + p(d.getMinutes()); }


function useSize() {
  const ref = useRef(null);
  const [size, setSize] = useState({ w: 0, h: 0 });
  useEffect(() => {
    const el = ref.current; if (!el) return;
    const ro = new ResizeObserver((es) => { for (const e of es) setSize({ w: e.contentRect.width, h: e.contentRect.height }); });
    ro.observe(el); setSize({ w: el.clientWidth, h: el.clientHeight });
    return () => ro.disconnect();
  }, []);
  return [ref, size];
}

function MoonDiscA({ width, fraction }) {
  const R = 50, w = Math.max(width, 0);
  const rx = R * Math.abs(1 - 2 * fraction), sweep = fraction < 0.5 ? 0 : 1;
  const lit = "M0 " + (-R) + " A " + R + " " + R + " 0 0 1 0 " + R + " A " + rx + " " + R + " 0 0 " + sweep + " 0 " + (-R) + " Z";
  return (
    <svg width={w} height={w} viewBox="-50 -50 100 100" style={{ display: "block", overflow: "visible" }}>
      <circle cx="0" cy="0" r="70" fill="rgba(210,225,255,0.10)" />
      <circle cx="0" cy="0" r={R} fill="#2c3242" />
      <path d={lit} fill="#eef1f7" />
    </svg>
  );
}
function SunDiscA({ width }) {
  const w = Math.max(width, 0);
  return (
    <svg width={w} height={w} viewBox="-50 -50 100 100" style={{ display: "block", overflow: "visible" }}>
      <defs>
        <radialGradient id="sf-sun" cx="50%" cy="50%" r="50%">
          <stop offset="0%" stopColor="#fff7d6" /><stop offset="55%" stopColor="#ffd84d" /><stop offset="100%" stopColor="#ffb02e" />
        </radialGradient>
        <radialGradient id="sf-sunglow" cx="50%" cy="50%" r="50%">
          <stop offset="0%" stopColor="rgba(255,228,140,0.5)" /><stop offset="100%" stopColor="rgba(255,228,140,0)" />
        </radialGradient>
      </defs>
      <circle cx="0" cy="0" r="150" fill="url(#sf-sunglow)" />
      <circle cx="0" cy="0" r="50" fill="url(#sf-sun)" />
    </svg>
  );
}


/* --- 3D wireframe fits: pick a solid, drag to rotate it in 3D, slider for
       size. The PROJECTED silhouette writes A.p1/p2, while the stored pose
       (rotation matrix) records the object's orientation in space — a
       foreshortened tic-tac is a rotated capsule, not a mislabeled orb. --- */
const SHAPES = [
  { k: "orb", label: "● Orb" },
  { k: "saucer", label: "🛸 Saucer" },
  { k: "capsule", label: "💊 Tic-tac" },
  { k: "tri", label: "▲ Triangle" },
  { k: "plane", label: "✈ Plane" },
  { k: "bird", label: "🕊 Bird" },
];
const I3 = [1, 0, 0, 0, 1, 0, 0, 0, 1];
const rotX3 = (d) => { const a = d * D2R, c = Math.cos(a), s = Math.sin(a); return [1, 0, 0, 0, c, -s, 0, s, c]; };
const rotY3 = (d) => { const a = d * D2R, c = Math.cos(a), s = Math.sin(a); return [c, 0, s, 0, 1, 0, -s, 0, c]; };
const rotZ3 = (d) => { const a = d * D2R, c = Math.cos(a), s = Math.sin(a); return [c, -s, 0, s, c, 0, 0, 0, 1]; };
const mul3 = (A, B) => { const R = new Array(9); for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) { let v = 0; for (let k = 0; k < 3; k++) v += A[i * 3 + k] * B[k * 3 + j]; R[i * 3 + j] = v; } return R; };
const app3 = (M, p) => [M[0] * p[0] + M[1] * p[1] + M[2] * p[2], M[3] * p[0] + M[4] * p[1] + M[5] * p[2], M[6] * p[0] + M[7] * p[1] + M[8] * p[2]];

function shapeWire(kind, aspect) { // unit major dimension, centered at origin
  const C = [];
  const circ = (r, axis, off = 0, n = 40) => Array.from({ length: n + 1 }, (_, i) => {
    const a = (i / n) * Math.PI * 2, u = Math.cos(a) * r, v = Math.sin(a) * r;
    return axis === "z" ? [u, v, off] : axis === "y" ? [u, off, v] : [off, u, v];
  });
  if (kind === "orb") {
    const r = 0.5;
    C.push(circ(r, "z", 0), circ(r, "y", 0), circ(r, "x", 0));
    const zr = 0.25, rr = Math.sqrt(r * r - zr * zr);
    C.push(circ(rr, "z", zr), circ(rr, "z", -zr));
  } else if (kind === "saucer") {
    const r = 0.5, h = 0.11;
    C.push(circ(r, "z", 0)); // rim
    for (let m = 0; m < 4; m++) {
      const rm = rotZ3(m * 45);
      C.push(Array.from({ length: 41 }, (_, i) => {
        const a = (i / 40) * Math.PI * 2;
        return app3(rm, [Math.cos(a) * r, 0, Math.sin(a) * h]);
      }));
    }
    C.push(circ(0.3, "z", h * 0.75), circ(0.3, "z", -h * 0.75));
  } else if (kind === "capsule") {
    const r = 0.5 / Math.max(1.2, aspect || 3), hl = 0.5 - r;
    const stad = (plane) => {
      const pts = [];
      for (let i = 0; i <= 20; i++) { const a = -Math.PI / 2 + (i / 20) * Math.PI; pts.push([hl + Math.cos(a) * r, Math.sin(a) * r]); }
      for (let i = 0; i <= 20; i++) { const a = Math.PI / 2 + (i / 20) * Math.PI; pts.push([-hl + Math.cos(a) * r, Math.sin(a) * r]); }
      pts.push(pts[0]);
      return pts.map(([x, u]) => (plane === "y" ? [x, u, 0] : [x, 0, u]));
    };
    C.push(stad("y"), stad("z"), circ(r, "x", hl), circ(r, "x", -hl));
  } else if (kind === "plane") {
    // stylized airliner — wingspan = 1, fuselage along +X
    const w = 0.036, h = 0.046;
    C.push([[0.5, 0], [0.44, w], [-0.40, w], [-0.5, w * 0.35], [-0.5, -w * 0.35], [-0.40, -w], [0.44, -w], [0.5, 0]]
      .map(([x, y]) => [x, y, 0]));                                    // fuselage planform
    C.push([[0.5, 0], [0.43, -h * 0.7], [-0.38, -h], [-0.5, -h * 0.5], [-0.5, h * 0.4], [-0.42, h], [0.44, h * 0.75], [0.5, 0]]
      .map(([x, z]) => [x, 0, z]));                                    // fuselage side profile
    for (const s of [1, -1]) {
      C.push([[0.13, s * 0.05, 0], [-0.02, s * 0.5, 0], [-0.13, s * 0.5, 0], [-0.11, s * 0.05, 0], [0.13, s * 0.05, 0]]);          // swept wing
      C.push([[-0.40, s * 0.03, 0], [-0.47, s * 0.19, 0], [-0.51, s * 0.19, 0], [-0.485, s * 0.03, 0], [-0.40, s * 0.03, 0]]);     // h-stab
    }
    C.push([[-0.37, 0, 0], [-0.47, 0, -0.17], [-0.52, 0, -0.17], [-0.50, 0, 0], [-0.37, 0, 0]]);                                   // vertical fin
  } else if (kind === "bird") {
    // gliding bird — wingspan = 1, head along +X, slight dihedral
    C.push([[0.17, 0], [0.13, 0.03], [-0.12, 0.025], [-0.14, 0], [-0.12, -0.025], [0.13, -0.03], [0.17, 0]]
      .map(([x, y]) => [x, y, 0]));                                   // body planform
    C.push([[0.17, 0], [0.12, -0.035], [-0.12, -0.03], [-0.14, 0], [-0.11, 0.028], [0.13, 0.03], [0.17, 0]]
      .map(([x, z]) => [x, 0, z]));                                   // body profile
    for (const s of [1, -1]) {
      C.push([
        [0.06, s * 0.03, 0], [0.05, s * 0.30, -0.02], [0.02, s * 0.5, -0.05],
        [-0.08, s * 0.5, -0.05], [-0.07, s * 0.28, -0.02], [-0.06, s * 0.03, 0], [0.06, s * 0.03, 0],
      ]);                                                             // wing with dihedral
    }
    C.push([[-0.12, 0.02, 0], [-0.23, 0.08, 0], [-0.25, 0, 0], [-0.23, -0.08, 0], [-0.12, -0.02, 0], [-0.12, 0.02, 0]]); // tail fan
  } else { // tri — thin equilateral plate
    const R = 0.5774, th = 0.05;
    const v = [90, 210, 330].map((d) => [Math.cos(d * D2R) * R, Math.sin(d * D2R) * R]);
    for (const z of [th, -th]) C.push([...v, v[0]].map(([x, y]) => [x, y, z]));
    for (const [x, y] of v) C.push([[x, y, th], [x, y, -th]]);
  }
  return C;
}
const SHAPE_R0 = () => ({ orb: I3, saucer: rotX3(-62), capsule: I3, tri: rotX3(-24), plane: rotX3(-55), bird: rotX3(-60) });

function shapeProjNat(sf) { // orthographic project → natural-px curves + silhouette extremes
  const R = sf.roll ? mul3(sf.rotM || I3, rotZ3(sf.roll)) : (sf.rotM || I3);
  const s = sf.sizeNat || 100;
  const curves = shapeWire(sf.kind, sf.aspect).map((c) => c.map((p) => {
    const q = app3(R, p);
    return { x: sf.cx + q[0] * s, y: sf.cy + q[1] * s, z: q[2] };
  }));
  const pts = curves.flat();
  const c0 = pts.reduce((m, p) => ({ x: m.x + p.x / pts.length, y: m.y + p.y / pts.length }), { x: 0, y: 0 });
  let A = pts[0], best = -1;
  for (const p of pts) { const d = (p.x - c0.x) ** 2 + (p.y - c0.y) ** 2; if (d > best) { best = d; A = p; } }
  let B = pts[0]; best = -1;
  for (const p of pts) { const d = (p.x - A.x) ** 2 + (p.y - A.y) ** 2; if (d > best) { best = d; B = p; } }
  let minor = 0;
  const ax = B.x - A.x, ay = B.y - A.y, al = Math.hypot(ax, ay) || 1;
  for (const p of pts) { const d = Math.abs((-ay * (p.x - A.x) + ax * (p.y - A.y)) / al); if (d > minor) minor = d; }
  return { curves, p1: { x: A.x, y: A.y }, p2: { x: B.x, y: B.y }, minorNat: minor * 2 };
}

const ENABLE_SENSORS = false; // 🧭 point-with-phone + 📷 camera AR — parked for now, flip to bring back
const ENABLE_GPS_BUTTON = false; // 📍 use-my-GPS — parked (unreliable in the field), flip to bring back

/* reference silhouettes for the in-sky Compare tool (from Sky Sense) */
const GHOSTW = [
  { name: "Mylar balloon", m: 0.5, shape: "c" },
  { name: "Drone", m: 0.35, shape: "c" },
  { name: "Cessna 172", m: 11, shape: "p" },
  { name: "Airliner (737)", m: 36, shape: "p" },
];
function GhostSil({ shape, w, color }) {
  /* true angular size, floored at 1 px — a balloon at 3 km IS sub-pixel,
     and faking it bigger defeats the whole comparison */
  const ww = Math.max(w, 1);
  if (shape === "p") return (
    <svg width={ww} height={ww * 1.2} viewBox="0 0 100 120" style={{ display: "block", overflow: "visible" }}>
      <g fill={color}>
        <path d="M50 4 C56 4 57 18 56 34 L56 92 C56 104 54 116 50 116 C46 116 44 104 44 92 L44 34 C43 18 44 4 50 4 Z" />
        <path d="M50 48 L3 84 L3 88 L16 88 L50 64 L84 88 L97 88 L97 84 Z" />
        <path d="M50 96 L24 110 L24 113 L34 113 L50 104 L66 113 L76 113 L76 110 Z" />
      </g>
    </svg>
  );
  return <svg width={ww} height={ww} viewBox="0 0 100 100" style={{ display: "block" }}><circle cx="50" cy="50" r="47" fill={color} /></svg>;
}

function SkyAimer({ open, onClose, lat, lng, whenMs, initAz, initAlt, marks, which, onCapture, source, update, wizard, onWizardBack, onWizardNext }) {
  const [vpRef, vp] = useSize();
  const [viewAz, setViewAz] = useState(180);
  const [viewAlt, setViewAlt] = useState(30);
  const [fov, setFov] = useState(55);
  const [motionOn, setMotionOn] = useState(false);
  const [motionMsg, setMotionMsg] = useState("");
  const motionRef = useRef({ got: false, init: false, vx: 0, vy: 0, vz: 0 });
  const [cameraOn, setCameraOn] = useState(false);
  const [cameraMsg, setCameraMsg] = useState("");
  const videoRef = useRef(null), streamRef = useRef(null);
  const panRef = useRef(null);
  const pointersRef = useRef(new Map());
  const pinchRef = useRef(null);

  /* photo-in-sky placement */
  const [photoOn, setPhotoOn] = useState(false);
  const [pMode, setPMode] = useState("look"); // 'look' | 'place'
  const [pAz, setPAz] = useState(180);
  const [pEl, setPEl] = useState(30);
  const [pRoll, setPRoll] = useState(0);
  const [fovM, setFovM] = useState(68);      // photo's own FOV (calibrated by pinch)
  const openPoseRef = useRef(null);          // placement as of aimer-open — Reset target
  const PH_OP = 0.85; // photo opacity — fixed; the grid/terrain still reads through the warp
  const [flash, setFlash] = useState("");
  const [selSeg, setSelSeg] = useState(null);   // Δt chip being edited
  const [selPt, setSelPt] = useState(null);     // trajectory point whose turn radius is being edited
  /* compare ghost — buttons only, NO sliders and NO draggable elements:
     the aimer holds a document-level touch lock (invariant: iOS multi-touch),
     which silently eats native drags on anything inside it. Drop the ghost
     at the crosshair like a trajectory point; distance via preset chips. */
  const [cmpOn, setCmpOn] = useState(false);
  const [cmpD, setCmpD] = useState(1000);       // ghost's assumed distance, meters
  const [ghostIdx, setGhostIdx] = useState(3);
  const [cmpPos, setCmpPos] = useState(null);   // ghost's sky anchor {az, el}
  const [objD, setObjD] = useState(1000);       // YOUR OBJECT's assumed distance — size↔distance guesstimate
  const lastDtRef = useRef(2);
  const poseRafRef = useRef(0);
  const pendPoseRef = useRef(null);
  const queuePose = (kind, az, el) => {
    pendPoseRef.current = { kind, az, el };
    if (!poseRafRef.current) poseRafRef.current = requestAnimationFrame(() => {
      poseRafRef.current = 0;
      const p = pendPoseRef.current; pendPoseRef.current = null;
      if (!p) return;
      if (p.kind === "place") { setPAz(p.az); setPEl(p.el); }
      else { setViewAz(p.az); setViewAlt(p.el); }
    });
  };
  useEffect(() => {
    if (!flash) return;
    const id = setTimeout(() => setFlash(""), 2200);
    return () => clearTimeout(id);
  }, [flash]);
  const [vidT2, setVidT2] = useState(0);
  const [vidDur2, setVidDur2] = useState(0);
  const aimVidRef = useRef(null);
  const placeRef = useRef(null);
  const twistRef = useRef(null);
  const warpRef = useRef(null);   // canvas that draws the Look-mode warp ourselves
  const texRef = useRef(null);
  const [, setTexReady] = useState(0);

  /* decode the (already EXIF-normalized) image once for canvas texturing */
  useEffect(() => {
    texRef.current = null; setTexReady((v) => v + 1);
    if (!source?.mediaUrl || source?.mediaKind === "video") return;
    const im = new Image();
    im.onload = () => {
      let tex = im;
      try {
        const MAXT = 1280;
        if (im.naturalWidth > MAXT || im.naturalHeight > MAXT) {
          const sc = MAXT / Math.max(im.naturalWidth, im.naturalHeight);
          const cv = document.createElement("canvas");
          cv.width = Math.round(im.naturalWidth * sc);
          cv.height = Math.round(im.naturalHeight * sc);
          cv.getContext("2d").drawImage(im, 0, 0, cv.width, cv.height);
          tex = cv;
        }
      } catch (e) { /* keep full-res image */ }
      texRef.current = tex; setTexReady((v) => v + 1);
    };
    im.src = source.mediaUrl;
  }, [source?.mediaUrl, source?.mediaKind]);

  /* aim starts on the previously entered direction, if any */
  useEffect(() => {
    if (open) {
      setViewAz(isNum(initAz) ? ((+initAz % 360) + 360) % 360 : 180);
      setViewAlt(isNum(initAlt) ? clampN(+initAlt, -15, 88) : 30);
      setMotionMsg(""); setCameraMsg("");
      const ma = source?.mediaAim;
      const p0 = {
        az: ma ? ma.az : (isNum(source?.A?.az) ? +source.A.az : (isNum(initAz) ? +initAz : 180)),
        el: ma ? ma.el : (isNum(source?.A?.el) ? +source.A.el : 30),
        roll: ma ? (ma.roll || 0) : 0,
        fov: isNum(source?.fovH) ? +source.fovH : 68,
      };
      openPoseRef.current = p0; // Reset restores the WHOLE placement to this
      setPAz(p0.az); setPEl(p0.el); setPRoll(p0.roll); setFovM(p0.fov);
      setPhotoOn(!!source?.mediaUrl);
      setPMode(source?.mediaAim ? "look" : (source?.mediaUrl ? "place" : "look"));
      if (wizard && source?.mediaUrl) setPMode("place"); // wizard always starts in adjust mode
    }
  }, [open]); // eslint-disable-line

  /* scroll lock while the aimer is open. Range inputs are whitelisted:
     this document-level preventDefault is what silently killed every
     slider drag inside the aimer on touch devices. */
  useEffect(() => {
    if (!open) return;
    const prevent = (e) => {
      if (e.target && e.target.closest && e.target.closest("input[type=range]")) return;
      if (e.cancelable) e.preventDefault();
    };
    document.addEventListener("touchmove", prevent, { passive: false });
    const prevOverflow = document.body.style.overflow;
    document.body.style.overflow = "hidden";
    return () => { document.removeEventListener("touchmove", prevent); document.body.style.overflow = prevOverflow; };
  }, [open]);

  /* shut sensors off when closed */
  useEffect(() => {
    if (!open) {
      setMotionOn(false);
      if (streamRef.current) { streamRef.current.getTracks().forEach((t) => t.stop()); streamRef.current = null; }
      setCameraOn(false);
    }
  }, [open]);

  /* --- astronomy for the sighting time & place --- */
  const LAT = isNum(lat) ? +lat : 42.16, LNG = isNum(lng) ? +lng : -123.66;
  const T = isNum(whenMs) ? +whenMs : Date.now();
  const sun = useMemo(() => sunPos(T, LAT, LNG), [T, LAT, LNG]);
  const moon = useMemo(() => ({ ...moonPos(T, LAT, LNG), frac: moonFrac(T) }), [T, LAT, LNG]);
  const isNight = sun.alt < -4;

  /* --- air traffic on the dome (ADS-B), at the SIGHTING time.
     Fresh sighting (≤15 min): live API, refreshed every 20 s.
     Older: our /api/hist archive proxy — traffic that was actually in
     the air at T (covers today back ~2 years). If the archive fails,
     fall back to live with an amber "context only" warning. --- */
  const [acOn, setAcOn] = useState(true);
  const [acData, setAcData] = useState(null); // {ac, fetchedAt, src, hist} | {err}
  const acSnapRef = useRef(null);
  const liveTrailRef = useRef(new Map()); // hex → [[ms, lat, lon, altM], ...] built from the 20 s polls
  const hasPos = isNum(lat) && isNum(lng);
  const wantHist = Math.abs(Date.now() - T) > 900000;
  useEffect(() => {
    if (!open || !acOn || !hasPos) return;
    let dead = false;
    const keep = (ac, apiSrc, hist) => {
      const air = ac.filter((a) => !a.ground && a.altM != null);
      setAcData({ ac: air, fetchedAt: Date.now(), src: apiSrc, hist });
      if (!hist) {
        const m = liveTrailRef.current, now = Date.now();
        for (const a of air) {
          const arr = m.get(a.hex) || [];
          if (!arr.length || now - arr[arr.length - 1][0] > 8000) arr.push([now, a.lat, a.lon, a.altM]);
          while (arr.length > 32) arr.shift();
          m.set(a.hex, arr);
        }
      }
      const trimmed = air
        .map((a) => ({ ...a, _r: acAzElRange({ lat: LAT, lon: LNG, alt: 0 }, a).rangeM }))
        .sort((x, y) => x._r - y._r).slice(0, 40)
        .map(({ _r, trail, ...a }) => a); // trails stay out of the stored snapshot
      acSnapRef.current = { fetchedAt: Date.now(), src: apiSrc, nm: 60, ac: trimmed, hist, t: hist ? T : Date.now() };
    };
    const pull = async () => {
      if (wantHist) {
        try {
          const { ac, source: apiSrc } = await fetchAircraftAt(LAT, LNG, T, 60);
          if (!dead) keep(ac, apiSrc, true);
          return;
        } catch (e) { /* archive miss → live fallback below */ }
      }
      try {
        const { ac, source: apiSrc } = await fetchAircraft(LAT, LNG, 60);
        if (!dead) keep(ac, apiSrc, false);
      } catch (e) { if (!dead) setAcData({ err: String(e?.message || e) }); }
    };
    pull();
    const iv = wantHist ? null : setInterval(pull, 20000);
    return () => { dead = true; if (iv) clearInterval(iv); };
  }, [open, acOn, hasPos, LAT, LNG, wantHist, T]);
  /* persist the last snapshot onto the source when the aimer closes.
     the wizard UNMOUNTS the aimer on step change, so the
     unmount cleanup must drain too (updateRef dodges the stale closure). */
  const updateRef = useRef(update); updateRef.current = update;
  const drainSnap = () => {
    if (acSnapRef.current && updateRef.current) { updateRef.current({ adsb: acSnapRef.current }); acSnapRef.current = null; }
  };
  useEffect(() => { if (!open) drainSnap(); }, [open]);
  useEffect(() => drainSnap, []);
  const acView = useMemo(() => {
    if (!acOn || !acData?.ac) return [];
    return acData.ac
      .map((a) => {
        const g = acAzElRange({ lat: LAT, lon: LNG, alt: 0 }, a);
        return { a, ...g };
      })
      .filter((v) => v.el > -2 && v.rangeM < 160000)
      .sort((x, y) => x.rangeM - y.rangeM)
      .slice(0, 30);
  }, [acOn, acData, LAT, LNG]);

  /* --- predicted terrain skyline (DEM ray-march) — the no-compass
     calibration: drag the photo until its ridges sit on this line and
     az + pitch + roll lock simultaneously, day or night. --- */
  const [terrOn, setTerrOn] = useState(true);
  const [terr, setTerr] = useState(null); // {els, h0} | {err} | null
  useEffect(() => {
    if (!open || !terrOn || !hasPos) return;
    let dead = false;
    setTerr((t) => t && t.els ? t : null);
    predictedSkyline(LAT, LNG)
      .then((sk) => { if (!dead) setTerr(sk); })
      .catch((e) => { if (!dead) setTerr({ err: String(e?.message || e) }); });
    return () => { dead = true; };
  }, [open, terrOn, hasPos, LAT, LNG]);

  /* --- satellites (the night ADS-B): CelesTrak visual group via SGP4,
     at the SIGHTING time. auto = shown when the sky is dark enough;
     on = any time; off = hidden. --- */
  const [satMode, setSatMode] = useState("auto");
  const [satDb, setSatDb] = useState(null); // {sats, fetchedAt} | {err}
  const satsWanted = satMode === "on" || (satMode === "auto" && sun.alt < -6);
  useEffect(() => {
    if (!open || !satsWanted || satDb) return;
    let dead = false;
    loadSats().then((db) => { if (!dead) setSatDb(db); })
      .catch((e) => { if (!dead) setSatDb({ err: String(e?.message || e) }); });
    return () => { dead = true; };
  }, [open, satsWanted, satDb]);
  const satView = useMemo(() => {
    if (!satsWanted || !satDb?.sats || !hasPos) return [];
    return satsAt(satDb.sats, T, LAT, LNG, 0).slice(0, 20)
      .map((s) => ({ ...s, trail: satTrail(s.rec, T, LAT, LNG) }));
  }, [satsWanted, satDb, T, LAT, LNG, hasPos]);
  const satStaleDays = satView.length ? Math.round(Math.max(...satView.map((s) => s.epochAgeDays || 0))) : 0;

  /* tap a plane chip → detail card (identity via adsbdb, scheduled route) */
  const [selHex, setSelHex] = useState(null);
  const [selInfo, setSelInfo] = useState(null); // {route, aircraft} | {busy} | null
  const selV = selHex ? acView.find((v) => v.a.hex === selHex) : null;
  useEffect(() => {
    if (!selHex || !selV) return;
    let dead = false;
    setSelInfo({ busy: true });
    fetchAcInfo(selHex, selV.a.flight).then((info) => { if (!dead) setSelInfo(info); });
    return () => { dead = true; };
  }, [selHex]);
  useEffect(() => { if (!acOn) setSelHex(null); }, [acOn]);

  /* --- true pinhole (gnomonic) projection ---
     While placing, the view is SLAVED to the photo's camera axis. Two
     gnomonic projections sharing an axis differ by pure scale+rotation,
     so the photo renders as an exactly rigid rectangle — no warp. */
  const FRAME = 0.78; // fraction of viewport width the photo occupies while placing
  const placing = pMode === "place" && photoOn && !!source?.natW;
  const effAz = placing ? pAz : viewAz;
  const effAlt = placing ? clampN(pEl, -20, 88) : viewAlt;
  const effFov = placing
    ? clampN(2 * Math.atan(Math.tan((fovM * RAD) / 2) / FRAME) * R2D, 12, 135)
    : fov;
  const fovH = effFov;
  const tanH = Math.tan((fovH * RAD) / 2);
  const tanV = vp.w > 0 ? tanH * (vp.h / vp.w) : tanH; // square pixels: scale the TANGENT, not degrees
  const fovV = 2 * Math.atan(tanV) * R2D;
  const vAzR = effAz * RAD, vAltR = clampN(effAlt, -89, 89) * RAD;
  const camF = [Math.cos(vAltR) * Math.sin(vAzR), Math.cos(vAltR) * Math.cos(vAzR), Math.sin(vAltR)];
  let camR = [camF[1], -camF[0], 0];
  const camRL = Math.hypot(camR[0], camR[1], camR[2]) || 1; camR = [camR[0] / camRL, camR[1] / camRL, camR[2] / camRL];
  const camU = [camR[1] * camF[2] - camR[2] * camF[1], camR[2] * camF[0] - camR[0] * camF[2], camR[0] * camF[1] - camR[1] * camF[0]];
  const dirOf = (azDg, altDg) => { const A = azDg * RAD, h = altDg * RAD, ch = Math.cos(h); return [ch * Math.sin(A), ch * Math.cos(A), Math.sin(h)]; };
  const projectD = (d) => {
    const zc = d[0] * camF[0] + d[1] * camF[1] + d[2] * camF[2];
    if (zc <= 0.001) return { x: 0, y: 0, inFront: false };
    const xc = d[0] * camR[0] + d[1] * camR[1] + d[2] * camR[2];
    const yc = d[0] * camU[0] + d[1] * camU[1] + d[2] * camU[2];
    return { x: 0.5 + (xc / zc) / (2 * tanH), y: 0.5 - (yc / zc) / (2 * tanV), inFront: true };
  };
  const project = (azDg, altDg) => projectD(dirOf(azDg, altDg));
  const unproject = (xf, yf) => {
    const sx = (xf - 0.5) * 2 * tanH, sy = -(yf - 0.5) * 2 * tanV;
    return unit([
      camR[0] * sx + camU[0] * sy + camF[0],
      camR[1] * sx + camU[1] * sy + camF[1],
      camR[2] * sx + camU[2] * sy + camF[2],
    ]);
  };
  /* screen-space tilt of the world-vertical at a direction (deg, clockwise from screen-up) */
  const vertTilt = (d) => {
    const p0 = projectD(d); if (!p0.inFront) return 0;
    const ae = dirToAzEl(d);
    const p1 = projectD(dirOf(ae.az, Math.min(ae.el + 0.5, 89.5)));
    if (!p1.inFront) return 0;
    return Math.atan2((p1.x - p0.x) * (vp.w || 1), -(p1.y - p0.y) * (vp.h || 1)) * R2D;
  };

  /* --- gestures: look = pan/zoom the view; place = move/scale/level the photo frame --- */
  const twoPtGeom = (ids) => {
    const ks = ids || [...pointersRef.current.keys()].slice(0, 2);
    if (ks.length < 2) return null;
    const a = pointersRef.current.get(ks[0]), b = pointersRef.current.get(ks[1]);
    if (!a || !b) return null;
    return { ids: ks, dist: Math.hypot(a.x - b.x, a.y - b.y) || 1, ang: Math.atan2(b.y - a.y, b.x - a.x) };
  };
  const onBgDown = (e) => {
    e.currentTarget.setPointerCapture(e.pointerId);
    pointersRef.current.set(e.pointerId, { x: e.clientX, y: e.clientY });
    const n = pointersRef.current.size;
    if (n >= 2) {
      panRef.current = null; placeRef.current = null;
      const g = twoPtGeom();
      if (placing) twistRef.current = g;       // {ids, dist, ang} — rebaselined every event
      else pinchRef.current = g;
    } else if (placing) {
      placeRef.current = { x: e.clientX, y: e.clientY, az: pAz, el: pEl };
    } else {
      panRef.current = { x: e.clientX, y: e.clientY, az: viewAz, alt: viewAlt };
    }
  };
  const onBgMove = (e) => {
    if (pointersRef.current.has(e.pointerId)) pointersRef.current.set(e.pointerId, { x: e.clientX, y: e.clientY });
    const n = pointersRef.current.size;
    if (n >= 2) {
      /* incremental per-event deltas from the SAME two pointer ids, with
         angle unwrap and glitch rejection — kills the 90° rotation snaps */
      if (twistRef.current) {
        const t = twistRef.current;
        const g = twoPtGeom(t.ids); if (!g) return;
        let dA = g.ang - t.ang;
        if (dA > Math.PI) dA -= 2 * Math.PI;
        if (dA < -Math.PI) dA += 2 * Math.PI;
        const ratio = g.dist / t.dist;
        t.dist = g.dist; t.ang = g.ang;
        if (Math.abs(dA) > 0.6 || ratio < 0.67 || ratio > 1.5) return; // pointer glitch — skip this event
        /* accumulate, commit past a threshold — so a pinch doesn't dribble
           rotation and a twist doesn't dribble scale */
        t.pendRot = (t.pendRot || 0) + dA * R2D;
        t.pendScale = (t.pendScale || 1) * ratio;
        let doRot = 0, doScale = 1;
        if (Math.abs(t.pendRot) > 0.8) { doRot = t.pendRot; t.pendRot = 0; }
        if (t.pendScale > 1.015 || t.pendScale < 1 / 1.015) { doScale = t.pendScale; t.pendScale = 1; }
        if (doRot || doScale !== 1) {
          if (doScale !== 1) setFovM((f) => clampN(f * doScale, 12, 120));
          if (doRot) setPRoll((r) => clampN(r - doRot, -90, 90)); // rotate(−roll): photo tracks the fingers
        }
      } else if (pinchRef.current) {
        const t = pinchRef.current;
        const g = twoPtGeom(t.ids); if (!g) return;
        const ratio = g.dist / t.dist;
        t.dist = g.dist;
        if (ratio < 0.67 || ratio > 1.5) return;
        setFov((f) => clampN(f / ratio, 10, 90));
      }
      return;
    }
    if (placeRef.current && vp.w) {
      const pr = placeRef.current; // snapshot: pointerup may null the ref before React flushes
      const dx = (e.clientX - pr.x) / vp.w, dy = (e.clientY - pr.y) / (vp.h || vp.w);
      const nAz = (((pr.az + dx * fovH) % 360) + 360) % 360;
      const nEl = clampN(pr.el - dy * fovV, -20, 88);
      queuePose("place", nAz, nEl);
      return;
    }
    if (panRef.current && vp.w && !motionOn) {
      const dx = (e.clientX - panRef.current.x) / vp.w, dy = (e.clientY - panRef.current.y) / (vp.h || vp.w);
      queuePose("look",
        (((panRef.current.az - dx * fovH) % 360) + 360) % 360,
        clampN(panRef.current.alt + dy * fovV, -15, 88));
    }
  };
  const onBgUp = (e) => {
    pointersRef.current.delete(e.pointerId);
    try { e.currentTarget.releasePointerCapture(e.pointerId); } catch (_) { }
    const n = pointersRef.current.size;
    if (n < 2) { pinchRef.current = null; twistRef.current = null; }
    if (n === 1) {
      const p = [...pointersRef.current.values()][0];
      if (placing) placeRef.current = { x: p.x, y: p.y, az: pAz, el: pEl };
      else panRef.current = { x: p.x, y: p.y, az: viewAz, alt: viewAlt };
    } else if (n === 0) {
      panRef.current = null; placeRef.current = null;
      if (placing) commitPlacement();
    }
  };
  useEffect(() => {
    const el = vpRef.current; if (!el || !open) return;
    const onWheel = (ev) => { ev.preventDefault(); setFov((f) => clampN(Math.round(f * (ev.deltaY > 0 ? 1.08 : 1 / 1.08)), 10, 90)); };
    el.addEventListener("wheel", onWheel, { passive: false });
    return () => el.removeEventListener("wheel", onWheel);
  }, [open]); // eslint-disable-line

  /* --- point with the phone (DeviceOrientation) --- */
  const enableMotion = async () => {
    setMotionMsg("");
    const DOE = window.DeviceOrientationEvent;
    if (!DOE) { setMotionMsg("This browser doesn't expose device orientation."); return; }
    motionRef.current.init = false;
    try {
      if (typeof DOE.requestPermission === "function") {
        const res = await DOE.requestPermission();
        if (res !== "granted") { setMotionMsg("Motion permission denied — drag to aim instead."); return; }
      }
      setMotionOn(true);
    } catch (err) {
      setMotionMsg("Motion blocked here (embedded previews usually block sensors). Open in a browser tab to point with the phone.");
    }
  };
  useEffect(() => {
    if (!motionOn) return;
    const m = motionRef.current; m.got = false; m.init = false;
    const handler = (e) => {
      if (e.alpha == null && e.beta == null && e.gamma == null && e.webkitCompassHeading == null) return;
      m.got = true;
      const heading = (typeof e.webkitCompassHeading === "number" && !isNaN(e.webkitCompassHeading)) ? (360 - e.webkitCompassHeading) : (e.alpha || 0);
      const aZ = heading * RAD, aX = (e.beta || 0) * RAD, aY = (e.gamma || 0) * RAD;
      const cX = Math.cos(aX), cY = Math.cos(aY), cZ = Math.cos(aZ), sX = Math.sin(aX), sY = Math.sin(aY), sZ = Math.sin(aZ);
      const r02 = cZ * sY + cY * sZ * sX, r12 = sZ * sY - cZ * cY * sX, r22 = cX * cY;
      const vx = -r02, vy = -r12, vz = -r22;
      if (!m.init) { m.vx = vx; m.vy = vy; m.vz = vz; m.init = true; }
      else { const k = 0.18; m.vx += (vx - m.vx) * k; m.vy += (vy - m.vy) * k; m.vz += (vz - m.vz) * k; }
      const L = Math.hypot(m.vx, m.vy, m.vz) || 1; const ux = m.vx / L, uy = m.vy / L, uz = m.vz / L;
      setViewAlt(clampN(Math.asin(clampN(uz, -1, 1)) * R2D, -20, 89));
      setViewAz(((Math.atan2(ux, uy) * R2D) + 360) % 360);
    };
    const evName = ("ondeviceorientationabsolute" in window) ? "deviceorientationabsolute" : "deviceorientation";
    window.addEventListener(evName, handler, true);
    const t = setTimeout(() => { if (!motionRef.current.got) { setMotionMsg("No sensor data — the embedded preview blocks motion. Open in a browser tab to use it."); setMotionOn(false); } }, 1800);
    return () => { clearTimeout(t); window.removeEventListener(evName, handler, true); };
  }, [motionOn]); // eslint-disable-line

  /* --- camera passthrough (AR) --- */
  const enableCamera = async () => {
    setCameraMsg("");
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) { setCameraMsg("Camera isn't available in this browser."); return; }
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ video: { facingMode: { ideal: "environment" } }, audio: false });
      streamRef.current = stream; setCameraOn(true);
    } catch (err) {
      setCameraMsg("Camera blocked or denied — embedded previews usually block it. Open in a browser tab and allow the camera.");
    }
  };
  const disableCamera = () => { const s = streamRef.current; if (s) s.getTracks().forEach((t) => t.stop()); streamRef.current = null; setCameraOn(false); };
  useEffect(() => { if (cameraOn && videoRef.current && streamRef.current) { videoRef.current.srcObject = streamRef.current; const p = videoRef.current.play(); if (p && p.catch) p.catch(() => { }); } }, [cameraOn]);

  /* --- scene elements --- */
  /* REAL night sky: the 327 catalog stars + naked-eye planets at their true
     az/el for the sighting time & place — calibration anchors ("it was two
     fists left of Vega") and, for Venus & co, mundane-explanation candidates.
     No hard night cliff: a twilight limiting magnitude fades the sky in the
     way the real one does — Venus minutes after sunset, Sirius-class stars
     near −5° sun, the full mag-3.6 field by about −10°. */
  const [starMode, setStarMode] = useState("auto"); // auto: real twilight fade · on: force full field · off: hidden
  const limMagAuto = sun.alt > -1.5 ? -99 : clampN(-3.5 + (-sun.alt - 2) * 0.95, -4, 7);
  const limMag = starMode === "on" ? 7 : starMode === "off" ? -99 : limMagAuto;
  const stars = useMemo(() => {
    if (limMag < -4) return [];
    return STARS.map(([ra, dec, mag, name]) => {
      const p = raDecToAzEl(ra, dec, T, LAT, LNG);
      return {
        az: p.az, alt: p.alt, mag, name,
        r: clampN(1.65 - 0.3 * mag, 0.35, 2.3),
        o: clampN(1.05 - 0.18 * mag, 0.25, 1) * clampN((limMag - mag) / 1.2, 0.15, 1),
      };
    }).filter((s) => s.alt > -1 && s.mag <= limMag);
  }, [T, LAT, LNG, limMag]);
  const planets = useMemo(() => planetPositions(T, LAT, LNG).filter((p) => p.alt > -2), [T, LAT, LNG]);
  const planetsVisible = starMode === "on" ? true : starMode === "off" ? false : sun.alt < -1;
  const gpath = (pts) => { let d = "", pen = false; for (const p of pts) { if (p.inFront && p.x > -0.6 && p.x < 1.6 && p.y > -0.6 && p.y < 1.6) { d += (pen ? " L " : " M ") + (p.x * 100).toFixed(2) + " " + (p.y * 100).toFixed(2); pen = true; } else pen = false; } return d; };
  const gridColor = cameraOn ? "rgba(255,255,255,0.30)" : (isNight ? "rgba(150,180,230,0.20)" : "rgba(255,255,255,0.28)");
  const ALTS = [15, 30, 45, 60, 75];
  const altLines = ALTS.map((h) => { const pts = []; for (let a = -130; a <= 130; a += 4) pts.push(project(effAz + a, h)); return gpath(pts); });
  const altLabels = ALTS.map((h) => ({ h, ...project(effAz, h) })).filter((p) => p.inFront && p.y > 0.03 && p.y < 0.97);
  const azLines = []; for (let A = 0; A < 360; A += 30) { const pts = []; for (let h = 0; h <= 87; h += 3) pts.push(project(A, h)); azLines.push(gpath(pts)); }
  const horizonPts = []; for (let a = -130; a <= 130; a += 4) horizonPts.push(project(effAz + a, 0)); const horizonPath = gpath(horizonPts);
  const terrainPath = (terrOn && terr?.els) ? (() => {
    const pts = []; for (let a = -130; a <= 130; a += 0.8) pts.push(project(effAz + a, skylineElAt(terr.els, effAz + a)));
    return gpath(pts);
  })() : null;
  const terrainLbl = (terrainPath) ? (() => {
    const p = project(effAz, skylineElAt(terr.els, effAz));
    return p.inFront && p.y > 0.04 && p.y < 0.96 ? p : null;
  })() : null;
  const horizonY = project(effAz, 0).y;
  const cardinals = [[0, "N"], [45, "NE"], [90, "E"], [135, "SE"], [180, "S"], [225, "SW"], [270, "W"], [315, "NW"]].map(([az, lbl]) => ({ ...project(az, 1.8), lbl })).filter((c) => c.inFront && c.x > 0.02 && c.x < 0.98 && c.y > -0.05 && c.y < 1.05);
  const starDots = !cameraOn ? stars.map((s) => ({ ...project(s.az, s.alt), r: s.r, o: s.o, name: s.name, mag: s.mag })).filter((p) => p.inFront && p.x > -0.05 && p.x < 1.05 && p.y > -0.05 && p.y < 1.05) : [];
  const starLabels = starDots.filter((p) => p.name && (p.mag <= 1.4 || (fovH < 42 && p.mag <= 2.2)));
  const planetDots = planetsVisible && !cameraOn ? planets.map((p) => ({ ...p, p: project(p.az, p.alt) })).filter((x) => x.p.inFront && x.p.x > -0.05 && x.p.x < 1.05 && x.p.y > -0.05 && x.p.y < 1.05) : [];

  const bodyPx = vp.w > 0 ? Math.max((vp.w * Math.tan((0.53 * RAD) / 2)) / tanH, 12) : 0;
  const sunProj = project(sun.az, sun.alt);
  const moonProj = project(moon.az, moon.alt);
  const markProjs = (marks || []).map((mk) => ({ ...mk, p: project(mk.az, mk.el) })).filter((mk) => mk.p.inFront && mk.p.x > -0.1 && mk.p.x < 1.1 && mk.p.y > -0.1 && mk.p.y < 1.1);

  const skyBg = sun.alt < -8
    ? "linear-gradient(180deg,#070b16 0%,#0e1424 60%,#18203a 100%)"
    : sun.alt < -1
      ? "linear-gradient(180deg,#0a1226 0%,#16224a 55%,#4a3a55 100%)" // twilight
      : "linear-gradient(180deg,#1f64b8 0%,#4f93d6 55%,#bfe2ff 100%)";
  /* --- the pose IS the state; place-mode gestures mutate it directly --- */
  const poseNow = { az: pAz, el: pEl, roll: pRoll, fov: fovM };

  const photo = (photoOn && source?.mediaUrl && source?.natW) ? (() => {
    const natW = source.natW, natH = source.natH;
    const b = photoBasis(poseNow.az, poseNow.el, poseNow.roll);
    const tHm = Math.tan((poseNow.fov * RAD) / 2), tVm = tHm * (natH / natW);
    return { natW, natH, ...b, tHm, tVm };
  })() : null;

  const pixDir = (px, py) => {
    const { natW, natH, f, r, u, tHm } = photo;
    const fpx = (natW / 2) / tHm;
    const x = (px - natW / 2) / fpx, y = (natH / 2 - py) / fpx;
    return unit([f[0] + r[0] * x + u[0] * y, f[1] + r[1] * x + u[1] * y, f[2] + r[2] * x + u[2] * y]);
  };

  const enterPlace = () => {
    /* first-ever placement: put the photo where you're looking */
    if (!source?.mediaAim) { setPAz(viewAz); setPEl(clampN(viewAlt, -20, 88)); }
    if (motionOn) setMotionOn(false);
    setPMode("place");
  };
  const donePlace = () => {
    /* hand the (already photo-centered) view back seamlessly — nothing moves */
    setViewAz(pAz); setViewAlt(clampN(pEl, -15, 88));
    setFov(clampN(effFov, 10, 90));
    commitPlacement();
    setPMode("look");
  };

  /* --- Look-mode marks + visibility (the image itself is drawn by our own
         canvas mesh warp — no CSS matrix3d, nothing for Safari to reinterpret) --- */
  let photoMarks = null, photoHidden = false;
  if (photo && vp.w > 0) {
    const centerOK = projectD(photo.f).inFront;
    photoHidden = !placing && !centerOK;
    if (centerOK) {
      const P = (pt) => { const pr = projectD(pixDir(pt.x, pt.y)); return pr.inFront ? [pr.x * 100, pr.y * 100] : null; };
      const tr = (source.track || []).filter((p) => p.x != null).sort((a, b) => a.t - b.t).map(P).filter(Boolean);
      photoMarks = {
        a1: source.A?.p1 ? P(source.A.p1) : null,
        a2: source.A?.p2 ? P(source.A.p2) : null,
        pb: source.B?.pb ? P(source.B.pb) : null,
        trk: tr,
      };
    }
  }

  /* --- canvas mesh warp: forward-map the photo through projectD with a
         triangle grid, affine per cell. Same math as the sky grid, fully
         deterministic on every browser. --- */
  useEffect(() => {
    const cv = warpRef.current;
    if (!cv) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2); // 3× buys nothing on a warped photo
    const W = vp.w, H = vp.h;
    if (!W || !H) return;
    cv.width = W * dpr; cv.height = H * dpr;
    const ctx = cv.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    if (placing || !photoOn || !photo) return;
    const isVid = source?.mediaKind === "video";
    const tex = isVid ? aimVidRef.current : texRef.current;
    if (!tex) return;
    const tw = isVid ? tex.videoWidth : (tex.naturalWidth || tex.width);
    const th = isVid ? tex.videoHeight : (tex.naturalHeight || tex.height);
    if (!tw || !th) return;
    const NC = 7, NR = Math.max(4, Math.round(NC * photo.natH / photo.natW));
    const dst = [];
    for (let r = 0; r <= NR; r++) {
      const row = [];
      for (let c = 0; c <= NC; c++) {
        const pr = projectD(pixDir((c / NC) * photo.natW, (r / NR) * photo.natH));
        row.push(pr.inFront ? [pr.x * W, pr.y * H] : null);
      }
      dst.push(row);
    }
    ctx.globalAlpha = PH_OP;
    const tri = (s0, s1, s2, d0, d1, d2) => {
      const cx = (d0[0] + d1[0] + d2[0]) / 3, cy = (d0[1] + d1[1] + d2[1]) / 3;
      const ex = (p) => { const dx = p[0] - cx, dy = p[1] - cy, L = Math.hypot(dx, dy) || 1; return [p[0] + (dx / L) * 0.6, p[1] + (dy / L) * 0.6]; };
      const e0 = ex(d0), e1 = ex(d1), e2 = ex(d2);
      ctx.save();
      ctx.beginPath(); ctx.moveTo(e0[0], e0[1]); ctx.lineTo(e1[0], e1[1]); ctx.lineTo(e2[0], e2[1]); ctx.closePath(); ctx.clip();
      const [x0, y0] = s0, [x1, y1] = s1, [x2, y2] = s2;
      const den = x0 * (y1 - y2) + x1 * (y2 - y0) + x2 * (y0 - y1);
      if (den) {
        const aM = (d0[0] * (y1 - y2) + d1[0] * (y2 - y0) + d2[0] * (y0 - y1)) / den;
        const bM = (d0[1] * (y1 - y2) + d1[1] * (y2 - y0) + d2[1] * (y0 - y1)) / den;
        const cM = (d0[0] * (x2 - x1) + d1[0] * (x0 - x2) + d2[0] * (x1 - x0)) / den;
        const dM = (d0[1] * (x2 - x1) + d1[1] * (x0 - x2) + d2[1] * (x1 - x0)) / den;
        ctx.transform(aM, bM, cM, dM, d0[0] - aM * x0 - cM * y0, d0[1] - bM * x0 - dM * y0);
        ctx.drawImage(tex, 0, 0);
      }
      ctx.restore();
    };
    const sx = (c) => (c / NC) * tw, sy = (r) => (r / NR) * th;
    for (let r = 0; r < NR; r++) for (let c = 0; c < NC; c++) {
      const d00 = dst[r][c], d10 = dst[r][c + 1], d01 = dst[r + 1][c], d11 = dst[r + 1][c + 1];
      if (!d00 || !d10 || !d01 || !d11) continue;
      tri([sx(c), sy(r)], [sx(c + 1), sy(r)], [sx(c + 1), sy(r + 1)], d00, d10, d11);
      tri([sx(c), sy(r)], [sx(c + 1), sy(r + 1)], [sx(c), sy(r + 1)], d00, d11, d01);
    }
    ctx.globalAlpha = 1;
  }); // every render — ~100 projections, trivially cheap

  const commitPlacement = () => {
    if (!update || !photoOn) return;
    const patch = { mediaAim: { az: +pAz.toFixed(2), el: +pEl.toFixed(2), roll: +pRoll.toFixed(1) }, fovH: +fovM.toFixed(1) };
    /* placement + marked points fully determine the sight-lines — derive
       A (object marks / shape fit) and B (motion mark) automatically, so
       the fix never dies for want of an elevation the user already gave us */
    if (source && !(source.track || []).length && source.natW) {
      const fpx = (source.natW / 2) / Math.tan((fovM * D2R) / 2);
      const bb = photoBasis(pAz, pEl, pRoll);
      const dirAt = (px, py) => {
        const x = (px - source.natW / 2) / fpx, y = (source.natH / 2 - py) / fpx;
        return dirToAzEl(unit([bb.f[0] + bb.r[0] * x + bb.u[0] * y, bb.f[1] + bb.r[1] * x + bb.u[1] * y, bb.f[2] + bb.r[2] * x + bb.u[2] * y]));
      };
      const c = source.shapeFit ? { x: source.shapeFit.cx, y: source.shapeFit.cy }
        : (source.A?.p1 && source.A?.p2 ? { x: (source.A.p1.x + source.A.p2.x) / 2, y: (source.A.p1.y + source.A.p2.y) / 2 } : null);
      if (c) { const ae = dirAt(c.x, c.y); patch.A = { ...source.A, az: ae.az.toFixed(2), el: ae.el.toFixed(2) }; }
      if (source.B?.pb) { const ae = dirAt(source.B.pb.x, source.B.pb.y); patch.B = { ...source.B, az: ae.az.toFixed(2), el: ae.el.toFixed(2) }; }
    }
    update(patch);
  };
  /* --- wizard trajectory: world-anchored points with per-segment Δt --- */
  const sortedTrack = source ? [...(source.track || [])].sort((x, y) => x.t - y.t) : [];
  const trajTotal = sortedTrack.length > 1 ? sortedTrack[sortedTrack.length - 1].t - sortedTrack[0].t : 0;
  const objAngW = source
    ? (angSizeFromPoints(source.A?.p1, source.A?.p2, source.natW, source.natH, +source.fovH)
      ?? (isNum(source.A?.angManual) ? +source.A.angManual : null))
    : null;
  const syncAB = (track) => {
    const p = { track };
    if (track.length) p.A = { ...source.A, az: String(track[0].az), el: String(track[0].el), t: String(track[0].t) };
    if (track.length >= 2) {
      const L = track[track.length - 1];
      p.B = { ...source.B, az: String(L.az), el: String(L.el), t: String(L.t) };
    } else p.B = { ...source.B, az: "", el: "", t: "" };
    return p;
  };
  const dropPoint = (az, el) => {
    if (!update || !source) return;
    const dt = lastDtRef.current || 2;
    const tN = sortedTrack.length ? +(sortedTrack[sortedTrack.length - 1].t + dt).toFixed(2) : 0;
    /* new points default to a realistic arc (r 0.3) — a hard corner is a
       deliberate claim the witness makes by selecting the point */
    const track = [...sortedTrack, { t: tN, az: +az.toFixed(2), el: +el.toFixed(2), r: 0.3 }];
    update(syncAB(track));
    /* timing is freshest right after the drop — open the editor for it */
    setSelSeg(track.length >= 2 ? track.length - 1 : null);
    setSelPt(null);
    setFlash(track.length === 1 ? "Point 1 set — pan to where it moved, drop point 2" : `Point ${track.length} ⊕ — set how long it took below`);
  };
  const point1FromMarks = () => {
    if (!photo || !source?.A?.p1 || !source?.A?.p2) return;
    const ae = dirToAzEl(pixDir((source.A.p1.x + source.A.p2.x) / 2, (source.A.p1.y + source.A.p2.y) / 2));
    dropPoint(ae.az, ae.el);
  };
  const undoPoint = () => { if (!sortedTrack.length) return; update(syncAB(sortedTrack.slice(0, -1))); setSelSeg(null); setSelPt(null); };
  const setSegDt = (i, nd) => {
    const tr = sortedTrack.map((p) => ({ ...p }));
    const shift = nd - (tr[i].t - tr[i - 1].t);
    for (let j = i; j < tr.length; j++) tr[j].t = +(tr[j].t + shift).toFixed(2);
    lastDtRef.current = nd;
    update(syncAB(tr));
  };

  const handleClose = () => { if (photoOn) commitPlacement(); onClose(); };

  const aimColor = which === "B" ? "var(--teal)" : "var(--amber)";
  const recenter = (b) => { if (placing) { setPAz(b.az); setPEl(clampN(b.alt, -20, 88)); } else { setViewAz(b.az); setViewAlt(clampN(b.alt, -10, 80)); } };
  const fmtBody = (b) => `${Math.round(b.az)}°/${b.alt.toFixed(0)}°`;

  return (
    <div style={{ position: "fixed", inset: 0, zIndex: 200, display: open ? "block" : "none", background: "#070B14", fontFamily: "-apple-system,'Segoe UI',Roboto,system-ui,sans-serif", touchAction: "none", userSelect: "none", WebkitUserSelect: "none" }}>
      {/* viewport */}
      <div ref={vpRef} onPointerDown={onBgDown} onPointerMove={onBgMove} onPointerUp={onBgUp} onPointerCancel={onBgUp}
        style={{ position: "absolute", inset: 0, backgroundImage: cameraOn ? "none" : skyBg, backgroundColor: "#070B14", touchAction: "none", cursor: "grab", overflow: "hidden" }}>

        {cameraOn && <video ref={videoRef} playsInline muted autoPlay style={{ position: "absolute", inset: 0, width: "100%", height: "100%", objectFit: "cover", pointerEvents: "none" }} />}

        {!cameraOn && horizonY < 1.05 && (
          <div style={{ position: "absolute", left: 0, right: 0, top: (clampN(horizonY, 0, 1) * 100) + "%", bottom: 0, background: isNight ? "#0b0f18" : "#2c3729", pointerEvents: "none" }} />
        )}

        {starDots.length > 0 && (
          <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }} preserveAspectRatio="none" viewBox="0 0 100 100">
            {starDots.map((p, i) => <circle key={i} cx={p.x * 100} cy={p.y * 100} r={p.r * 0.18} fill="#fff" opacity={p.o} />)}
          </svg>
        )}
        {starLabels.map((p) => (
          <div key={"sl" + p.name} style={{ position: "absolute", left: (p.x * 100) + "%", top: (p.y * 100) + "%", transform: "translate(6px,-4px)", fontSize: 8.5, fontFamily: "var(--mono)", color: "rgba(220,230,255,.75)", textShadow: "0 1px 2px rgba(0,0,0,.8)", pointerEvents: "none", whiteSpace: "nowrap" }}>{p.name}</div>
        ))}
        {planetDots.map((pl) => (
          <div key={"pl" + pl.name} style={{ position: "absolute", left: (pl.p.x * 100) + "%", top: (pl.p.y * 100) + "%", transform: "translate(-50%,-50%)", textAlign: "center", pointerEvents: "none" }}>
            <div style={{ width: 5, height: 5, borderRadius: "50%", background: "#ffe9b0", boxShadow: "0 0 6px 2px rgba(255,225,150,.55)", margin: "0 auto" }} />
            <div style={{ fontSize: 8.5, fontFamily: "var(--mono)", fontWeight: 700, color: "#ffe9b0", textShadow: "0 1px 2px rgba(0,0,0,.85)", marginTop: 2, whiteSpace: "nowrap" }}>{pl.sym} {pl.name}</div>
          </div>
        ))}

        {/* photo/video — Look mode: our own canvas mesh warp */}
        {!placing && photoOn && (
          <canvas ref={warpRef} style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }} />
        )}
        {!placing && photoOn && source?.mediaKind === "video" && (
          <video ref={aimVidRef} src={source.mediaUrl} muted playsInline preload="auto"
            onLoadedMetadata={(e) => setVidDur2(e.target.duration || 0)}
            style={{ position: "absolute", width: 2, height: 2, opacity: 0, pointerEvents: "none" }} />
        )}
        {photoHidden && (
          <div style={{ position: "absolute", left: "50%", top: 56, transform: "translateX(-50%)", background: "rgba(15,23,42,.75)", border: "1px solid var(--line)", borderRadius: 999, padding: "4px 12px", fontSize: 11, fontFamily: "var(--mono)", color: "var(--dim)", pointerEvents: "none" }}>
            🖼 photo is off-view — pan toward {Math.round(pAz)}° / {Math.round(pEl)}°
          </div>
        )}
        {!placing && photoMarks && (
          <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }} preserveAspectRatio="none" viewBox="0 0 100 100">
            {photoMarks.trk.length > 1 && (
              <polyline points={photoMarks.trk.map((p) => p.join(",")).join(" ")} fill="none"
                stroke="var(--track)" strokeWidth="1.4" strokeDasharray="1.5 2" vectorEffect="non-scaling-stroke" />
            )}
            {photoMarks.trk.map((p, i) => <circle key={"t" + i} cx={p[0]} cy={p[1]} r="0.55" fill="var(--track)" />)}
            {photoMarks.a1 && photoMarks.a2 && (
              <line x1={photoMarks.a1[0]} y1={photoMarks.a1[1]} x2={photoMarks.a2[0]} y2={photoMarks.a2[1]}
                stroke="var(--amber)" strokeWidth="1.4" strokeDasharray="2 2" vectorEffect="non-scaling-stroke" />
            )}
            {[photoMarks.a1, photoMarks.a2].map((p, i) => p && <circle key={"a" + i} cx={p[0]} cy={p[1]} r="0.8" fill="none" stroke="var(--amber)" strokeWidth="1.4" vectorEffect="non-scaling-stroke" />)}
            {photoMarks.pb && <circle cx={photoMarks.pb[0]} cy={photoMarks.pb[1]} r="0.8" fill="none" stroke="var(--teal)" strokeWidth="1.4" vectorEffect="non-scaling-stroke" />}
          </svg>
        )}

        {/* photo — Place mode: ONE pinned element. Image, marks, and frame are
            physically the same box, so they cannot diverge; the on-axis proof
            guarantees this rigid rectangle equals the projective truth. */}
        {placing && source?.mediaUrl && (
          <div style={{
            position: "absolute", left: "50%", top: "50%",
            width: (FRAME * 100) + "%",
            transform: `translate(-50%,-50%) rotate(${-pRoll}deg)`,
            pointerEvents: "none",
          }}>
            {source.mediaKind === "video" ? (
              <video ref={aimVidRef} src={source.mediaUrl} muted playsInline preload="auto"
                onLoadedMetadata={(e) => setVidDur2(e.target.duration || 0)}
                style={{ width: "100%", display: "block", opacity: PH_OP }} />
            ) : (
              <img src={source.mediaUrl} alt="" style={{ width: "100%", display: "block", opacity: PH_OP }} />
            )}
            {source?.natW && (
              <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }}
                viewBox={`0 0 ${source.natW} ${source.natH}`} preserveAspectRatio="none">
                {(() => {
                  const tp = (source.track || []).filter((p) => p.x != null).sort((a, b) => a.t - b.t);
                  return (
                    <>
                      {tp.length > 1 && <polyline points={tp.map((p) => `${p.x},${p.y}`).join(" ")} fill="none" stroke="var(--track)" strokeWidth="2" strokeDasharray="4 5" vectorEffect="non-scaling-stroke" />}
                      {tp.map((p, i) => <circle key={i} cx={p.x} cy={p.y} r={source.natW / 220} fill="var(--track)" />)}
                      {source.A?.p1 && source.A?.p2 && <line x1={source.A.p1.x} y1={source.A.p1.y} x2={source.A.p2.x} y2={source.A.p2.y} stroke="var(--amber)" strokeWidth="2" strokeDasharray="5 5" vectorEffect="non-scaling-stroke" />}
                      {[source.A?.p1, source.A?.p2].map((p, i) => p && <circle key={"a" + i} cx={p.x} cy={p.y} r={source.natW / 160} fill="none" stroke="var(--amber)" strokeWidth="2" vectorEffect="non-scaling-stroke" />)}
                      {source.B?.pb && <circle cx={source.B.pb.x} cy={source.B.pb.y} r={source.natW / 160} fill="none" stroke="var(--teal)" strokeWidth="2" vectorEffect="non-scaling-stroke" />}
                    </>
                  );
                })()}
              </svg>
            )}
            <div style={{ position: "absolute", inset: 0, border: "1.5px dashed var(--amber)", boxSizing: "border-box" }} />
            <div style={{ position: "absolute", left: "50%", top: "50%", width: 14, height: 2, background: "var(--amber)", transform: "translate(-50%,-50%)" }} />
            <div style={{ position: "absolute", left: "50%", top: "50%", width: 2, height: 14, background: "var(--amber)", transform: "translate(-50%,-50%)" }} />
            {[["0%", "0%"], ["100%", "0%"], ["100%", "100%"], ["0%", "100%"]].map(([l, tp2], i) => (
              <div key={i} style={{ position: "absolute", left: l, top: tp2, width: 10, height: 10, margin: "-5px 0 0 -5px", borderRadius: "50%", background: "var(--amber)" }} />
            ))}
          </div>
        )}

        {/* alt-az grid + horizon */}
        <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }} preserveAspectRatio="none" viewBox="0 0 100 100">
          {altLines.map((d, i) => d ? <path key={"al" + i} d={d} fill="none" stroke={gridColor} strokeWidth="1" vectorEffect="non-scaling-stroke" /> : null)}
          {azLines.map((d, i) => d ? <path key={"az" + i} d={d} fill="none" stroke={gridColor} strokeWidth="1" vectorEffect="non-scaling-stroke" /> : null)}
          {horizonPath && <path d={horizonPath} fill="none" stroke={cameraOn ? "rgba(255,255,255,0.8)" : (isNight ? "rgba(170,190,230,0.6)" : "rgba(255,255,255,0.75)")} strokeWidth="1.8" vectorEffect="non-scaling-stroke" />}
          {terrainPath && <path d={terrainPath} fill="none" stroke="rgba(158,224,138,0.9)" strokeWidth="1.6" strokeDasharray="7 4" vectorEffect="non-scaling-stroke" />}
        </svg>
        {terrainLbl && (
          <div style={{ position: "absolute", left: (terrainLbl.x * 100) + "%", top: (terrainLbl.y * 100) + "%", transform: "translate(-50%,-130%)", fontSize: 8.5, fontFamily: "var(--mono)", fontWeight: 700, letterSpacing: ".14em", color: "rgba(158,224,138,0.95)", textShadow: "0 1px 2px rgba(0,0,0,.8)", pointerEvents: "none" }}>TERRAIN</div>
        )}
        {altLabels.map((p) => (
          <div key={"hl" + p.h} style={{ position: "absolute", left: (p.x * 100) + "%", top: (p.y * 100) + "%", transform: "translate(-50%,-50%)", fontSize: 9, fontFamily: "var(--mono)", color: gridColor.replace(/[\d.]+\)$/, "0.9)"), background: "rgba(7,11,20,.35)", borderRadius: 4, padding: "0 3px", pointerEvents: "none" }}>{p.h}°</div>
        ))}
        {cardinals.map((c, i) => (
          <div key={"cd" + i} style={{ position: "absolute", left: (c.x * 100) + "%", top: (c.y * 100) + "%", transform: "translate(-50%,-50%)", fontSize: 12, fontWeight: 800, color: "#fff", textShadow: "0 1px 3px rgba(0,0,0,.7)", pointerEvents: "none" }}>{c.lbl}</div>
        ))}

        {/* Sun & Moon — calibration anchors at their true positions */}
        {sunProj.inFront && sun.alt > -1 && (
          <div style={{ position: "absolute", left: (sunProj.x * 100) + "%", top: (sunProj.y * 100) + "%", transform: "translate(-50%,-50%)", pointerEvents: "none", textAlign: "center" }}>
            <SunDiscA width={bodyPx} />
          </div>
        )}
        {moonProj.inFront && moon.alt > -1 && (
          <div style={{ position: "absolute", left: (moonProj.x * 100) + "%", top: (moonProj.y * 100) + "%", transform: "translate(-50%,-50%)", pointerEvents: "none", textAlign: "center" }}>
            <MoonDiscA width={bodyPx} fraction={moon.frac} />
          </div>
        )}

        {/* satellites: markers + full-pass trails (cyan, dotted) */}
        {satView.length > 0 && (
          <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }}>
            {satView.map((s) => {
              const segs = []; let seg = [];
              for (const q of s.trail) {
                const pr = projectD(dirFromAzEl(q.az, q.el));
                if (pr.inFront && q.el > -1 && pr.x > -0.2 && pr.x < 1.2 && pr.y > -0.2 && pr.y < 1.2) seg.push(`${(pr.x * (vp.w || 1)).toFixed(1)},${(pr.y * (vp.h || 1)).toFixed(1)}`);
                else { if (seg.length > 1) segs.push(seg); seg = []; }
              }
              if (seg.length > 1) segs.push(seg);
              return segs.map((sg, k) => (
                <polyline key={s.name + k} points={sg.join(" ")} fill="none" stroke="#9fdcff"
                  strokeWidth="1.4" strokeLinecap="round" strokeDasharray="0.1 7" opacity={s.lit ? 0.45 : 0.18} />
              ));
            })}
          </svg>
        )}
        {satView.map((s) => {
          const pr = projectD(dirFromAzEl(s.az, s.el));
          if (!pr.inFront || pr.x < -0.05 || pr.x > 1.05 || pr.y < -0.05 || pr.y > 1.05) return null;
          const col = s.lit ? "#9fdcff" : "rgba(159,220,255,.35)";
          return (
            <div key={"sat" + s.name} style={{ position: "absolute", left: (pr.x * 100) + "%", top: (pr.y * 100) + "%", transform: "translate(-50%,-50%)", pointerEvents: "none", textAlign: "center" }}>
              <div style={{ width: 5, height: 5, transform: "rotate(45deg)", background: col, margin: "0 auto", boxShadow: s.lit ? "0 0 5px 1px rgba(159,220,255,.5)" : "none" }} />
              <div style={{ fontSize: 8.5, fontFamily: "var(--mono)", fontWeight: 700, color: col, textShadow: "0 1px 2px rgba(0,0,0,.85)", marginTop: 2, whiteSpace: "nowrap" }}>
                🛰 {s.name}{s.lit ? "" : " · in shadow"}<br />{Math.round(s.rangeKm)} km
              </div>
            </div>
          );
        })}

        {/* faint sky-tracks: each aircraft's path ±4 min (archive) or from the
           live polls — drawn only near the sight-line, or when selected */}
        {(() => {
          if (!acView.length) return null;
          const sight = isNum(source?.A?.az) && isNum(source?.A?.el) ? dirFromAzEl(+source.A.az, +source.A.el) : null;
          const lines = [];
          for (const v of acView) {
            const sel = v.a.hex === selHex;
            const raw = acData?.hist ? v.a.trail : liveTrailRef.current.get(v.a.hex);
            if (!raw || raw.length < 2) continue;
            if (!sel) {
              if (!sight) continue;
              const sep = Math.acos(clampN(dot(sight, v.d), -1, 1)) * R2D;
              if (sep > 25) continue;
            }
            /* densify by lerping in geo space (positions ~10 s apart are locally
               straight in 3D) — the projected curve then bends with the sky
               view's own curvature instead of cutting straight chords */
            const segs = []; let seg = [];
            const pushPt = (la, lo, alt) => {
              const g = acAzElRange({ lat: LAT, lon: LNG, alt: 0 }, { lat: la, lon: lo, altM: alt });
              const pr = projectD(g.d);
              if (pr.inFront && pr.x > -0.3 && pr.x < 1.3 && pr.y > -0.3 && pr.y < 1.3) seg.push(`${(pr.x * (vp.w || 1)).toFixed(1)},${(pr.y * (vp.h || 1)).toFixed(1)}`);
              else { if (seg.length > 1) segs.push(seg); seg = []; }
            };
            for (let i = 0; i < raw.length; i++) {
              if (i === 0) { pushPt(raw[0][1], raw[0][2], raw[0][3]); continue; }
              const a0 = raw[i - 1], a1 = raw[i], K = 5;
              for (let k = 1; k <= K; k++) {
                const f = k / K;
                pushPt(a0[1] + (a1[1] - a0[1]) * f, a0[2] + (a1[2] - a0[2]) * f, a0[3] + (a1[3] - a0[3]) * f);
              }
            }
            if (seg.length > 1) segs.push(seg);
            segs.forEach((sg, k) => lines.push({ sel, pts: sg.join(" "), key: v.a.hex + "-" + k }));
            if (lines.length > 12) break;
          }
          if (!lines.length) return null;
          return (
            <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }}>
              {lines.map((l) => (
                <polyline key={l.key} points={l.pts} fill="none" stroke="var(--track)"
                  strokeWidth={l.sel ? 2.2 : 1.6} strokeLinecap="round"
                  strokeDasharray={l.sel ? "0.1 6" : "0.1 9"}
                  opacity={l.sel ? 0.9 : 0.4} />
              ))}
            </svg>
          );
        })()}

        {/* live air traffic at true az/el (ADS-B) */}
        {acView.map((v) => {
          const pr = projectD(v.d);
          if (!pr.inFront || pr.x < -0.05 || pr.x > 1.05 || pr.y < -0.05 || pr.y > 1.05) return null;
          /* glyph heading: project the position ~15 s ahead, rotate ✈ toward it */
          let rot = 0;
          if (isNum(v.a.track) && isNum(v.a.gs) && v.a.gs > 5) {
            const dM = v.a.gs * 15;
            const ahead = {
              ...v.a,
              lat: +v.a.lat + (dM * Math.cos(v.a.track * D2R)) / 111320,
              lon: +v.a.lon + (dM * Math.sin(v.a.track * D2R)) / (111320 * Math.cos(LAT * D2R)),
            };
            const p2 = projectD(acAzElRange({ lat: LAT, lon: LNG, alt: 0 }, ahead).d);
            if (p2.inFront) rot = Math.atan2((p2.y - pr.y) * (vp.h || 1), (p2.x - pr.x) * (vp.w || 1)) * R2D;
          }
          const id = (v.a.flight || "").trim() || v.a.reg || v.a.hex;
          const sel = v.a.hex === selHex;
          const col = sel ? "var(--amber)" : "var(--track)";
          return (
            <div key={"ac" + v.a.hex}
              onPointerDown={(e) => e.stopPropagation()}
              onClick={(e) => { e.stopPropagation(); setSelHex(sel ? null : v.a.hex); }}
              style={{ position: "absolute", left: (pr.x * 100) + "%", top: (pr.y * 100) + "%", transform: "translate(-50%,-50%)", pointerEvents: "auto", cursor: "pointer", textAlign: "center", opacity: 0.94, padding: 6, zIndex: sel ? 6 : 5 }}>
              <div style={{ fontSize: 13, color: col, transform: `rotate(${rot}deg)`, textShadow: "0 1px 3px rgba(0,0,0,.85)", lineHeight: 1 }}>✈</div>
              <div style={{ fontSize: 8.5, fontFamily: "var(--mono)", fontWeight: 700, color: col, textShadow: "0 1px 2px rgba(0,0,0,.85)", marginTop: 1, whiteSpace: "nowrap" }}>
                {id}{v.a.t ? ` ${v.a.t}` : ""}<br />{fmtLenShort(v.rangeM)}{v.a.altM != null ? ` · ${Math.round(v.a.altM * 3.28084 / 100) / 10} kft` : ""}
              </div>
            </div>
          );
        })}

        {/* selected-aircraft detail card */}
        {selHex && (() => {
          const v = selV;
          const a = v ? v.a : (acData?.ac || []).find((x) => x.hex === selHex);
          if (!a) return null;
          const rt = selInfo?.route, acr = selInfo?.aircraft;
          const line = (k, val) => val ? <div style={{ display: "flex", justifyContent: "space-between", gap: 12 }}><span style={{ color: "var(--dim)" }}>{k}</span><span style={{ textAlign: "right" }}>{val}</span></div> : null;
          return (
            <div onPointerDown={(e) => e.stopPropagation()}
              style={{ position: "absolute", left: 10, right: 10, bottom: 108, zIndex: 230, background: "rgba(10,15,28,.94)", border: "1px solid var(--line)", borderRadius: 12, padding: "10px 12px", fontFamily: "var(--mono)", fontSize: 12, color: "var(--ink)", pointerEvents: "auto", maxWidth: 480, margin: "0 auto", lineHeight: 1.6 }}>
              <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
                <span style={{ fontWeight: 800, fontSize: 14, color: "var(--amber)" }}>
                  ✈ {(a.flight || "").trim() || a.reg || a.hex}{a.t ? ` · ${a.t}` : (acr?.icao_type ? ` · ${acr.icao_type}` : "")}
                </span>
                <button className="btn sm" style={{ background: "transparent", border: "none", color: "var(--dim)", padding: "0 2px" }} onClick={() => setSelHex(null)}>✕</button>
              </div>
              {(a.desc || acr) && <div style={{ color: "var(--dim)", fontSize: 11 }}>{a.desc || `${acr.manufacturer || ""} ${acr.type || ""}`.trim()}{a.reg || acr?.registration ? ` · ${a.reg || acr.registration}` : ""}{acr?.registered_owner ? ` · ${acr.registered_owner}` : ""}</div>}
              {selInfo?.busy && <div style={{ color: "var(--dim)", fontSize: 11 }}>looking up route…</div>}
              {rt?.airline?.name && <div style={{ color: "var(--track)", fontSize: 11 }}>{rt.airline.name}</div>}
              {rt?.origin && rt?.destination && (
                <div style={{ margin: "4px 0", fontWeight: 700 }}>
                  {rt.origin.iata_code || rt.origin.icao_code} <span style={{ color: "var(--dim)", fontWeight: 400 }}>({rt.origin.municipality})</span>
                  {" → "}{rt.destination.iata_code || rt.destination.icao_code} <span style={{ color: "var(--dim)", fontWeight: 400 }}>({rt.destination.municipality})</span>
                  <span style={{ color: "var(--dim)", fontWeight: 400, fontSize: 10 }}> · scheduled route</span>
                </div>
              )}
              {selInfo && !selInfo.busy && !rt && (a.flight || "").trim() && <div style={{ color: "var(--dim)", fontSize: 11 }}>no route on file for this callsign</div>}
              {line("altitude", a.altM != null ? `${Math.round(a.altM * 3.28084).toLocaleString()} ft · ${Math.round(a.altM).toLocaleString()} m` : null)}
              {line("speed", a.gs != null ? `${Math.round(a.gs * 2.23694)} mph · ${Math.round(a.gs * 1.94384)} kt` : null)}
              {line("track", a.track != null ? `${Math.round(a.track)}° ${compass8(a.track)}` : null)}
              {v && line("range · az/el", `${fmtLenShort(v.rangeM)} · ${v.az.toFixed(1)}°/${v.el.toFixed(1)}°`)}
              {line("data", `${acData?.hist ? "archive @ sighting time" : "live"} · ${acData?.src || ""}${a.seen != null ? ` · ±${Math.round(a.seen)}s` : ""}`)}
            </div>
          );
        })()}

        {/* previously captured directions */}
        {markProjs.map((mk, i) => (
          <div key={"mk" + i} style={{ position: "absolute", left: (mk.p.x * 100) + "%", top: (mk.p.y * 100) + "%", transform: "translate(-50%,-50%)", pointerEvents: "none", textAlign: "center" }}>
            <div style={{ width: 14, height: 14, border: `2px solid ${mk.color}`, transform: "rotate(45deg)", margin: "0 auto" }} />
            <div style={{ marginTop: 3, fontSize: 10, fontFamily: "var(--mono)", fontWeight: 800, color: mk.color, textShadow: "0 1px 2px rgba(0,0,0,.7)" }}>{mk.label}</div>
          </div>
        ))}

        {/* wizard trajectory — world-anchored points (may run past the photo) */}
        {(() => {
          if (!source) return null;
          const dirs = trackDirections(source);
          if (!dirs || !dirs.length) return null;
          const ps = dirs.map((d) => { const pr = projectD(d.d); return pr.inFront ? [pr.x, pr.y] : null; });
          const poly = ps.filter(Boolean);
          return (
            <>
              {poly.length > 1 && (
                <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }} preserveAspectRatio="none" viewBox="0 0 100 100">
                  <polyline points={poly.map((p) => `${p[0] * 100},${p[1] * 100}`).join(" ")} fill="none" stroke="var(--track)" strokeWidth="1.6" strokeDasharray="2 2.5" vectorEffect="non-scaling-stroke" opacity="0.9" />
                </svg>
              )}
              {(() => {
                let n = -1;
                return ps.map((p, i) => {
                  if (dirs[i].virt) return null;
                  n++;
                  if (!p) return null;
                  const idx = n, sel = selPt === idx;
                  const tappable = wizard;
                  const col = sel ? "var(--amber)" : "var(--track)";
                  return (
                    <div key={"tj" + i}
                      onPointerDown={tappable ? (e) => e.stopPropagation() : undefined}
                      onClick={tappable ? (e) => { e.stopPropagation(); setSelPt(sel ? null : idx); setSelSeg(null); } : undefined}
                      style={{ position: "absolute", left: (p[0] * 100) + "%", top: (p[1] * 100) + "%", transform: "translate(-50%,-50%)", pointerEvents: tappable ? "auto" : "none", cursor: tappable ? "pointer" : "default", textAlign: "center", padding: 6 }}>
                      <div style={{ width: 11, height: 11, borderRadius: "50%", border: `2px solid ${col}`, background: "rgba(7,11,20,.55)", margin: "0 auto" }} />
                      <div style={{ fontSize: 9, fontFamily: "var(--mono)", fontWeight: 800, color: col, textShadow: "0 1px 2px rgba(0,0,0,.8)", marginTop: 1 }}>{idx + 1}</div>
                    </div>
                  );
                });
              })()}
            </>
          );
        })()}

        {/* compare ghost — pure display, world-anchored where it was dropped.
           No pointer handlers: nothing here may interfere with look/pan. */}
        {wizard && cmpOn && cmpPos && (() => {
          const pr = projectD(dirFromAzEl(cmpPos.az, cmpPos.el));
          if (!pr.inFront) return null;
          const g = GHOSTW[ghostIdx];
          const gAng = 2 * Math.atan(g.m / (2 * cmpD)) * R2D;
          const fpxS = (vp.w || window.innerWidth || 1) / (2 * tanH);
          const gPx = gAng * D2R * fpxS; // TRUE angular size — no vanity floor
          return (
            <div style={{ position: "absolute", left: (pr.x * 100) + "%", top: (pr.y * 100) + "%", transform: "translate(-50%,-50%)", pointerEvents: "none", textAlign: "center", opacity: 0.92 }}>
              <div style={{ position: "relative", display: "inline-block" }}>
                {gPx < 6 && (
                  <div style={{ position: "absolute", left: "50%", top: "50%", width: 18, height: 18, margin: "-9px 0 0 -9px", border: "1px dashed rgba(159,180,216,.6)", borderRadius: "50%" }} />
                )}
                <GhostSil shape={g.shape} w={gPx} color="#9fb4d8" />
              </div>
              <div style={{ fontSize: 9, fontFamily: "var(--mono)", color: "#9fb4d8", textShadow: "0 1px 2px rgba(0,0,0,.8)", whiteSpace: "nowrap", marginTop: 2 }}>
                {g.name} ({g.m} m) @ {fmtLenShort(cmpD)}{gPx < 3 ? " · sub-pixel at this range" : ""}
              </div>
            </div>
          );
        })()}

        {/* aiming crosshair — fixed at screen center (hidden while placing) */}
        {pMode !== "place" && (
        <svg style={{ position: "absolute", left: "50%", top: "50%", transform: "translate(-50%,-50%)", pointerEvents: "none", overflow: "visible" }} width="64" height="64" viewBox="-32 -32 64 64">
          <circle cx="0" cy="0" r="14" fill="none" stroke={aimColor} strokeWidth="1.6" />
          <line x1="0" y1="-26" x2="0" y2="-8" stroke={aimColor} strokeWidth="1.6" />
          <line x1="0" y1="8" x2="0" y2="26" stroke={aimColor} strokeWidth="1.6" />
          <line x1="-26" y1="0" x2="-8" y2="0" stroke={aimColor} strokeWidth="1.6" />
          <line x1="8" y1="0" x2="26" y2="0" stroke={aimColor} strokeWidth="1.6" />
          <circle cx="0" cy="0" r="1.6" fill={aimColor} />
        </svg>
        )}
      </div>

      {flash && (
        <div style={{ position: "absolute", top: 96, left: "50%", transform: "translateX(-50%)", zIndex: 220, background: "rgba(14,43,38,.92)", border: "1px solid #2A6157", color: "var(--teal)", borderRadius: 999, padding: "7px 16px", fontSize: 12, fontWeight: 700, whiteSpace: "nowrap", pointerEvents: "none" }}>
          {flash}
        </div>
      )}

      {/* top HUD */}
      <div style={{ position: "absolute", top: 0, left: 0, right: 0, padding: "10px 12px", display: "flex", justifyContent: "space-between", alignItems: "flex-start", pointerEvents: "none", zIndex: 210 }}>
        <div>
          <div style={{ fontFamily: "var(--mono)", fontWeight: 800, fontSize: 20, color: aimColor, textShadow: "0 1px 4px rgba(0,0,0,.6)" }}>
            {effAz.toFixed(1)}° <span style={{ fontSize: 12, color: "#fff" }}>{compass8(effAz)}</span> · {effAlt.toFixed(1)}°<span style={{ fontSize: 12, color: "#fff" }}> up</span>
          </div>
          <div style={{ fontSize: 10, letterSpacing: ".14em", textTransform: "uppercase", fontWeight: 700, color: "rgba(255,255,255,.75)", textShadow: "0 1px 3px rgba(0,0,0,.6)" }}>
            {pMode === "place" ? "Placing photo" : `Aiming moment ${which}`} · FOV {Math.round(effFov)}°
          </div>
          <div style={{ display: "flex", gap: 6, marginTop: 6, pointerEvents: "auto", flexWrap: "wrap" }}>
            {sun.alt > -1 && <button className="btn sm" style={{ background: "rgba(15,23,42,.7)" }} onClick={() => recenter(sun)}>☀ {fmtBody(sun)}</button>}
            {moon.alt > -1 && <button className="btn sm" style={{ background: "rgba(15,23,42,.7)" }} onClick={() => recenter(moon)}>☾ {fmtBody(moon)} · {Math.round(moon.frac * 100)}%</button>}
            {planetsVisible && planets.filter((p) => p.alt > 0).map((p) => (
              <button key={p.name} className="btn sm" style={{ background: "rgba(15,23,42,.7)", color: "#ffe9b0" }} onClick={() => recenter(p)}>{p.sym} {fmtBody(p)}</button>
            ))}
            {hasPos && (
              <button className="btn sm" style={{ background: "rgba(15,23,42,.7)", color: !acOn ? "var(--dim)" : (acData?.ac && wantHist && !acData.hist) ? "var(--amber)" : "var(--track)" }}
                onClick={() => setAcOn((v) => !v)}>
                ✈ {acOn ? (acData?.ac ? `${acView.length}${acData.hist ? " @ sighting" : " live"}` : acData?.err ? "?" : "…") : "off"}
              </button>
            )}
            {hasPos && (
              <button className="btn sm" title={TERRAIN_ATTRIB} style={{ background: "rgba(15,23,42,.7)", color: !terrOn ? "var(--dim)" : terr?.err ? "var(--amber)" : "rgba(158,224,138,0.95)" }}
                onClick={() => setTerrOn((v) => !v)}>
                ⛰ {terrOn ? (terr?.els ? "ridge" : terr?.err ? "?" : "…") : "off"}
              </button>
            )}
            <button className="btn sm" title="Stars & planets: auto follows real twilight; on forces the full field any time"
              style={{ background: "rgba(15,23,42,.7)", color: starMode === "off" ? "var(--dim)" : (starMode === "on" || limMag > -4) ? "#dfe8ff" : "var(--dim)" }}
              onClick={() => setStarMode((m) => (m === "auto" ? "on" : m === "on" ? "off" : "auto"))}>
              ★ {starMode}
            </button>
            {hasPos && (
              <button className="btn sm" title="Satellites (CelesTrak visual group, SGP4 at the sighting time): auto shows when dark; on forces"
                style={{ background: "rgba(15,23,42,.7)", color: satMode === "off" ? "var(--dim)" : satView.length ? "#9fdcff" : "var(--dim)" }}
                onClick={() => setSatMode((m) => (m === "auto" ? "on" : m === "on" ? "off" : "auto"))}>
                🛰 {satMode === "off" ? "off" : satDb?.err ? "?" : satsWanted && !satDb ? "…" : `${satView.length}${satMode === "auto" ? "" : " on"}`}
              </button>
            )}
          </div>
          {satView.length > 0 && satStaleDays > 5 && (
            <div style={{ fontSize: 10, color: "var(--amber)", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 4 }}>
              🛰 TLE epoch ≈ {satStaleDays} d from the sighting — satellite positions degrade; treat as approximate
            </div>
          )}
          {acOn && acData?.ac && acData.hist && (
            <div style={{ fontSize: 10, color: "var(--track)", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 4 }}>
              ✈ archived traffic at the sighting time ({new Date(T).toLocaleString()})
            </div>
          )}
          {acOn && acData?.ac && wantHist && !acData.hist && (
            <div style={{ fontSize: 10, color: "var(--amber)", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 4 }}>
              ✈ archive unavailable — showing traffic in the air NOW, {Math.round(Math.abs(Date.now() - T) / 3600000) || "<1"} h from the sighting
            </div>
          )}
        </div>
        <button className="btn sm" style={{ pointerEvents: "auto", background: "rgba(15,23,42,.7)" }}
          onClick={() => { if (wizard) { if (photoOn) commitPlacement(); onWizardBack && onWizardBack(); } else handleClose(); }}>
          {wizard ? "‹ Back" : "✕ Close"}
        </button>
      </div>

      {/* view zoom — vertical stack on the right, out of the cramped bottom bar */}
      {pMode !== "place" && (
        <div style={{ position: "absolute", right: 10, top: "50%", transform: "translateY(-50%)", display: "flex", flexDirection: "column", gap: 6, zIndex: 205, pointerEvents: "auto" }}>
          <button className="btn" style={{ width: 42, height: 42, padding: 0, fontSize: 19, background: "rgba(15,23,42,.75)" }} onClick={() => setFov((f) => clampN(f - 12, 10, 90))}>+</button>
          <button className="btn" style={{ width: 42, height: 42, padding: 0, fontSize: 19, background: "rgba(15,23,42,.75)" }} onClick={() => setFov((f) => clampN(f + 12, 10, 90))}>−</button>
        </div>
      )}

      {/* bottom controls */}
      <div style={{ position: "absolute", left: 0, right: 0, bottom: 0, padding: "10px 12px calc(12px + env(safe-area-inset-bottom))", background: "linear-gradient(0deg, rgba(7,11,20,.92) 55%, rgba(7,11,20,0))", zIndex: 210 }}>
        {(motionMsg || cameraMsg) && <div className="warn" style={{ marginBottom: 8, marginTop: 0 }}>{motionMsg || cameraMsg}</div>}
        {source?.mediaUrl && (
          <div style={{ display: "flex", gap: 6, flexWrap: "wrap", marginBottom: 8, alignItems: "center" }}>
            {photoOn && (
              <>
                <button className={"btn sm" + (pMode === "place" ? " amber" : "")}
                  onClick={() => (pMode === "place" ? donePlace() : enterPlace())}>
                  {pMode === "place" ? "✓ Done placing" : "✥ Place"}
                </button>
                {pMode === "place" && (
                  <>
                    <button className="btn sm" onClick={() => setPRoll(0)}>⟺ Level</button>
                    <button className="btn sm" onClick={() => {
                      const p0 = openPoseRef.current;
                      if (p0) { setPAz(p0.az); setPEl(p0.el); setPRoll(p0.roll); setFovM(p0.fov); }
                      else { setFovM(isNum(source?.fovH) ? +source.fovH : 68); setPRoll(0); }
                    }}>Reset placement</button>
                  </>
                )}
                {source.mediaKind === "video" && vidDur2 > 0 && (
                  <input type="range" min={0} max={vidDur2} step={0.033} value={vidT2}
                    onChange={(e) => { const t = +e.target.value; setVidT2(t); if (aimVidRef.current) aimVidRef.current.currentTime = t; }}
                    style={{ width: 110 }} />
                )}
                {pMode === "place" && (
                  <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--amber)", width: "100%" }}>
                    → {pAz.toFixed(1)}° az · {pEl.toFixed(1)}° up · FOV {fovM.toFixed(1)}° · roll {pRoll.toFixed(1)}°
                  </span>
                )}
              </>
            )}
          </div>
        )}
        {pMode !== "place" && (
        <div style={{ display: "flex", gap: 6, flexWrap: "wrap", marginBottom: 8 }}>
          {ENABLE_SENSORS && (
            <>
              <button className={"btn sm" + (motionOn ? " teal" : "")} onClick={() => (motionOn ? setMotionOn(false) : enableMotion())}>{motionOn ? "◉ Motion on" : "🧭 Point with phone"}</button>
              <button className={"btn sm" + (cameraOn ? " teal" : "")} onClick={() => (cameraOn ? disableCamera() : enableCamera())}>{cameraOn ? "◉ Camera on" : "📷 Camera AR"}</button>
            </>
          )}
        </div>
        )}
        {wizard && pMode !== "place" && (
          <div style={{ marginBottom: 8 }}>
            <div style={{ display: "flex", gap: 6, flexWrap: "wrap", alignItems: "center" }}>
              {sortedTrack.length === 0 && source?.A?.p1 && source?.A?.p2 && (
                <button className="btn sm amber" onClick={point1FromMarks}>⌖ Start at marked object</button>
              )}
              <button className="btn sm amber" onClick={() => dropPoint(viewAz, viewAlt)}>⊕ Drop point {sortedTrack.length + 1}</button>
              {sortedTrack.length > 0 && <button className="btn sm" onClick={undoPoint}>↩</button>}
              <button className={"btn sm" + (cmpOn ? " teal" : "")} onClick={() => {
                setCmpOn((v) => !v);
                if (!cmpOn) setCmpPos({ az: viewAz, el: clampN(viewAlt, -10, 85) }); // drop at the crosshair
              }}>⚖</button>
            </div>
            {sortedTrack.length > 0 && (
              <div style={{ display: "flex", gap: 4, flexWrap: "wrap", alignItems: "center", marginTop: 6 }}>
                <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--track)" }}>t₀</span>
                {sortedTrack.slice(1).map((p, i) => {
                  const dt = +(p.t - sortedTrack[i].t).toFixed(1);
                  const on = selSeg === i + 1;
                  return (
                    <button key={i} className="btn sm" style={{ fontFamily: "var(--mono)", ...(on ? { borderColor: "var(--track)", color: "var(--track)" } : {}) }}
                      onClick={() => { setSelSeg((s) => (s === i + 1 ? null : i + 1)); setSelPt(null); }}>+{dt}s</button>
                  );
                })}
                <span style={{ marginLeft: "auto", fontFamily: "var(--mono)", fontSize: 11, color: "var(--track)" }}>
                  total {trajTotal.toFixed(1)} s · {sortedTrack.length} pt{sortedTrack.length > 1 ? "s" : ""}
                </span>
              </div>
            )}
            {selSeg != null && sortedTrack[selSeg] && (() => {
              const dt = sortedTrack[selSeg].t - sortedTrack[selSeg - 1].t;
              return (
                <div style={{ marginTop: 6, background: "rgba(15,23,42,.55)", border: "1px solid var(--line)", borderRadius: 10, padding: "8px 10px" }}>
                  <div style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)", marginBottom: 6 }}>
                    How long from point {selSeg} to point {selSeg + 1}?
                  </div>
                  <div style={{ display: "flex", gap: 4, flexWrap: "wrap", alignItems: "center" }}>
                    {[0.5, 1, 2, 3, 5, 10].map((v) => (
                      <button key={v} className={"btn sm" + (Math.abs(dt - v) < 0.05 ? " amber" : "")}
                        style={{ fontFamily: "var(--mono)" }} onClick={() => setSegDt(selSeg, v)}>{v}s</button>
                    ))}
                  </div>
                  <div style={{ display: "flex", gap: 6, alignItems: "center", marginTop: 6 }}>
                    <button className="btn sm" onClick={() => setSegDt(selSeg, Math.max(0.1, +(dt - 0.1).toFixed(1)))}>−0.1</button>
                    <span style={{ fontFamily: "var(--mono)", fontSize: 15, fontWeight: 700, color: "var(--amber)", minWidth: 56, textAlign: "center" }}>{dt.toFixed(1)} s</span>
                    <button className="btn sm" onClick={() => setSegDt(selSeg, +(dt + 0.1).toFixed(1))}>+0.1</button>
                    <button className="btn sm teal" style={{ marginLeft: "auto" }} onClick={() => setSelSeg(null)}>✓ Done</button>
                  </div>
                </div>
              );
            })()}
            {selPt != null && sortedTrack[selPt] && (() => {
              const interior = selPt > 0 && selPt < sortedTrack.length - 1;
              const r = +(sortedTrack[selPt].r ?? 0);
              const setR = (v) => update({ track: sortedTrack.map((p, i) => (i === selPt ? { ...p, r: v } : p)) });
              const deletePt = () => { update(syncAB(sortedTrack.filter((_, i) => i !== selPt))); setSelPt(null); setSelSeg(null); };
              return (
                <div style={{ marginTop: 6, background: "rgba(15,23,42,.55)", border: "1px solid var(--line)", borderRadius: 10, padding: "8px 10px" }}>
                  <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: interior ? 6 : 0 }}>
                    <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)", flex: 1 }}>
                      Point {selPt + 1}{interior ? " — how tight was the turn?" : selPt === 0 ? " (path start)" : " (path end)"}
                    </span>
                    <button className="btn sm" style={{ color: "var(--red)", borderColor: "#5A2C24" }} onClick={deletePt}>🗑 Delete</button>
                    <button className="btn sm teal" onClick={() => setSelPt(null)}>✓ Done</button>
                  </div>
                  {interior && (
                    <>
                      <div style={{ display: "flex", gap: 4, flexWrap: "wrap", alignItems: "center" }}>
                        {[["Hard corner", 0], ["Tight", 0.15], ["Normal", 0.3], ["Wide", 0.45]].map(([l, v]) => (
                          <button key={l} className={"btn sm" + (Math.abs(r - v) < 0.03 ? " amber" : "")} onClick={() => setR(v)}>{l}</button>
                        ))}
                      </div>
                      <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 6, lineHeight: 1.5 }}>
                        Real aircraft and birds fly arcs — a hard corner means an instantaneous direction change, which is itself an extraordinary claim. The arc feeds the g-load math.
                      </div>
                    </>
                  )}
                </div>
              );
            })()}
            {cmpOn && (
              <div style={{ marginTop: 6 }}>
                <div style={{ display: "flex", gap: 4, flexWrap: "wrap" }}>
                  {GHOSTW.map((g, i) => (
                    <button key={i} className={"btn sm" + (ghostIdx === i ? " teal" : "")} onClick={() => {
                      setGhostIdx(i);
                      /* pull the distance into THIS object's meaningful band —
                         a drone at an airliner's 10 km is an invisible dot */
                      const lo = Math.max(5, g.m * 10), hi = Math.min(120000, g.m * 3000);
                      setCmpD((d) => {
                        const dd = clampN(d, lo, hi);
                        const ang = 2 * Math.atan(g.m / (2 * dd)) * R2D;
                        return (ang < 0.08 || ang > 8) ? Math.round(g.m * 115) /* ≈0.5° — clearly visible */ : Math.round(dd);
                      });
                    }}>{g.name}</button>
                  ))}
                </div>
                <div style={{ display: "flex", gap: 8, alignItems: "center", marginTop: 6 }}>
                  <button className="btn sm teal" onClick={() => setCmpPos({ az: viewAz, el: clampN(viewAlt, -10, 85) })}>⌖ Drop at crosshair</button>
                  {(() => {
                    /* slider range adapts to the object: full sweep runs from
                       "fills a fist" to "sub-pixel dot" for whatever's selected */
                    const g = GHOSTW[ghostIdx];
                    const lo = Math.max(5, g.m * 10), hi = Math.min(120000, g.m * 3000);
                    const t = clampN(Math.log(cmpD / lo) / Math.log(hi / lo), 0, 1);
                    return (
                      <input type="range" min={0} max={1} step={0.004} value={t}
                        onPointerDown={(e) => e.stopPropagation()} onTouchStart={(e) => e.stopPropagation()}
                        onChange={(e) => setCmpD(Math.round(lo * Math.pow(hi / lo, +e.target.value)))}
                        style={{ flex: 1, touchAction: "auto", pointerEvents: "auto" }} />
                    );
                  })()}
                </div>
                {(() => {
                  const g = GHOSTW[ghostIdx];
                  return (
                    <div style={{ fontFamily: "var(--mono)", fontSize: 11, color: "var(--teal)", marginTop: 4 }}>
                      {g.name} at {fmtLenShort(cmpD)} → looks {(2 * Math.atan(g.m / (2 * cmpD)) * R2D).toFixed(2)}°
                      {objAngW != null && <> · your object measured {objAngW.toFixed(2)}° (= {fmtLenShort(2 * cmpD * Math.tan(objAngW * D2R / 2))} at that range)</>}
                    </div>
                  );
                })()}
                <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 3 }}>Aim the crosshair, ⌖ drop the ghost there, then slide its assumed distance</div>
                {objAngW != null && (() => {
                  /* the measured object: sweep assumed distance, read implied size live */
                  const t = clampN(Math.log(objD / 50) / Math.log(80000 / 50), 0, 1);
                  const size = 2 * objD * Math.tan(objAngW * D2R / 2);
                  return (
                    <div style={{ marginTop: 8, borderTop: "1px dashed var(--line)", paddingTop: 6 }}>
                      <div style={{ fontSize: 10, letterSpacing: ".12em", textTransform: "uppercase", fontWeight: 700, color: "var(--amber)" }}>
                        Your object — measured {objAngW.toFixed(2)}°
                      </div>
                      <input type="range" min={0} max={1} step={0.004} value={t}
                        onPointerDown={(e) => e.stopPropagation()} onTouchStart={(e) => e.stopPropagation()}
                        onChange={(e) => setObjD(Math.round(50 * Math.pow(80000 / 50, +e.target.value)))}
                        style={{ width: "100%", marginTop: 4, touchAction: "auto", pointerEvents: "auto" }} />
                      <div style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--amber)" }}>
                        if it was <b>{fmtLenShort(objD)}</b> away → it's <b>{fmtLenShort(size)}</b> across
                        <span style={{ color: "var(--dim)" }}> · nearest: {REF_OBJECTS.reduce((b, o) => Math.abs(Math.log(o.size / size)) < Math.abs(Math.log(b.size / size)) ? o : b).name}</span>
                      </div>
                    </div>
                  );
                })()}
              </div>
            )}
          </div>
        )}
        <div style={{ fontSize: 11, color: "rgba(255,255,255,.65)", marginBottom: 8 }}>
          {pMode === "place" && photoOn
            ? "The photo is pinned, undistorted, center-screen — drag to slide the SKY behind it, pinch to change how much sky it covers (calibrates FOV), twist to rotate. Line the photo's horizon onto the horizon line, then ✓ Done — nothing will shift."
            : wizard
              ? "Aim the crosshair where the object was at each moment and ⊕ drop points — the path can run right past the photo's edges. Tap a +Δt chip to adjust timing, or tap a numbered point to set how tight its turn was (hard corner ↔ wide arc)."
              : motionOn
                ? "Point the phone exactly where the object was, then capture."
                : "Drag to look around · pinch to zoom · put the crosshair where the object was. The Sun/Moon are drawn where they really were at the sighting time — use them to anchor your bearing."}
        </div>
        {!wizard && pMode !== "place" && (
        <div style={{ display: "flex", gap: 8 }}>
          <button className="btn amber" style={{ flex: 1 }} onClick={() => { onCapture("A", viewAz, viewAlt); setFlash("Moment A locked ✓ — sky view stays open; aim B or Close"); }}>Set Moment A</button>
          <button className="btn teal" style={{ flex: 1 }} onClick={() => { onCapture("B", viewAz, viewAlt); setFlash("Moment B locked ✓ — Close when done"); }}>Set Moment B</button>
        </div>
        )}
        {wizard && pMode === "place" && (
          <button className="btn amber" style={{ width: "100%" }} onClick={donePlace}>✓ Horizon lined up — continue</button>
        )}
        {wizard && pMode !== "place" && (
          <div style={{ display: "flex", gap: 8 }}>
            <button className="btn" onClick={() => { if (photoOn) commitPlacement(); onWizardBack && onWizardBack(); }}>‹ Back</button>
            <button className="btn amber" style={{ flex: 1 }} onClick={() => { if (photoOn) commitPlacement(); onWizardNext && onWizardNext(); }}>Continue →</button>
          </div>
        )}
      </div>
    </div>
  );
}

/* ============================================================
   PIN MAP — tactical position refiner on real map tiles (Leaflet).
   You are the fixed pin at center; dragging slides the ground
   beneath you and releasing commits the new center. Esri World
   Imagery by default (a rooftop is a better anchor than a street
   name), OSM street as the toggle.
   ============================================================ */
function PinMap({ lat, lon, origin, others, onChange }) {
  const boxRef = useRef(null);
  const mapRef = useRef(null);
  const layersRef = useRef(null);     // {sat, street}
  const overlayRef = useRef(null);    // marker layer group
  const originLineRef = useRef(null);
  const originRef = useRef(origin);
  const onChangeRef = useRef(onChange);
  const progRef = useRef(false);      // programmatic setView — don't commit
  const coordElRef = useRef(null);
  const distElRef = useRef(null);
  const [ready, setReady] = useState(false);
  const [baseSat, setBaseSat] = useState(true);
  originRef.current = origin;
  onChangeRef.current = onChange;

  const mPerDegN = 111320;
  const mPerDegE = (la) => 111320 * Math.max(0.2, Math.cos((+la || 0) * D2R));

  const hud = (c) => {
    if (coordElRef.current) coordElRef.current.textContent = `${c.lat.toFixed(6)}, ${c.lng.toFixed(6)}`;
    const og = originRef.current;
    if (distElRef.current) {
      if (og && isNum(og.lat)) {
        const dE = (c.lng - og.lon) * mPerDegE(c.lat), dN = (c.lat - og.lat) * mPerDegN;
        const dd = Math.hypot(dE, dN);
        distElRef.current.textContent = dd < 1 ? "on the photo's GPS fix" : `${fmtLenShort(dd)} ${dN >= 0 ? "N" : "S"}${dE >= 0 ? "E" : "W"} of photo GPS`;
      } else distElRef.current.textContent = "";
    }
    if (originLineRef.current && og && isNum(og.lat)) originLineRef.current.setLatLngs([[c.lat, c.lng], [+og.lat, +og.lon]]);
  };

  useEffect(() => {
    const el = boxRef.current; if (!el || mapRef.current) return;
    const map = L.map(el, {
      center: [+lat, +lon], zoom: 17, zoomControl: false,
      attributionControl: true, doubleClickZoom: false,
      /* the pin IS the map center — zooming must never move it. Pinch and
         wheel zoom about the center; only a one-finger drag pans. */
      touchZoom: "center", scrollWheelZoom: "center", bounceAtZoomLimits: false,
    });
    map.attributionControl.setPrefix(false);
    const sat = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
      maxZoom: 21, maxNativeZoom: 19, attribution: "© Esri, Maxar, Earthstar Geographics",
    });
    const street = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 21, maxNativeZoom: 19, attribution: "© OpenStreetMap contributors", className: "pinmap-street-tiles",
    });
    sat.addTo(map);
    map.on("move", () => hud(map.getCenter()));
    map.on("moveend", () => {
      if (progRef.current) { progRef.current = false; return; }
      const c = map.getCenter();
      onChangeRef.current(c.lat, c.lng);
    });
    mapRef.current = map; layersRef.current = { sat, street };
    hud(map.getCenter());
    setReady(true);
    return () => { map.remove(); mapRef.current = null; setReady(false); };
  }, []);

  /* external coordinate edits re-center the map; sub-meter echoes of our own
     commit (PositionEditor rounds to 1e-6°) must not — they'd loop */
  useEffect(() => {
    const map = mapRef.current; if (!map || !isNum(lat) || !isNum(lon)) return;
    const c = map.getCenter();
    if (Math.abs(c.lat - +lat) > 2e-6 || Math.abs(c.lng - +lon) > 2e-6) {
      progRef.current = true;
      map.setView([+lat, +lon], map.getZoom(), { animate: false });
      hud(map.getCenter());
    }
  }, [lat, lon]);

  useEffect(() => {
    const map = mapRef.current; if (!map || !ready) return;
    const { sat, street } = layersRef.current;
    if (baseSat) { map.removeLayer(street); sat.addTo(map); }
    else { map.removeLayer(sat); street.addTo(map); }
  }, [baseSat, ready]);

  /* photo-GPS origin + fellow observers */
  useEffect(() => {
    const map = mapRef.current; if (!map || !ready) return;
    if (overlayRef.current) { map.removeLayer(overlayRef.current); overlayRef.current = null; }
    originLineRef.current = null;
    const g = L.layerGroup();
    if (origin && isNum(origin.lat)) {
      const c = map.getCenter();
      originLineRef.current = L.polyline([[c.lat, c.lng], [+origin.lat, +origin.lon]],
        { color: "#F5A93F", weight: 1.5, dashArray: "4 4", opacity: 0.7, interactive: false }).addTo(g);
      L.marker([+origin.lat, +origin.lon], {
        interactive: false,
        icon: L.divIcon({ className: "", iconSize: [0, 0], html: `<div class="lmk lmk-dot">●<span>photo GPS</span></div>` }),
      }).addTo(g);
    }
    (others || []).filter((o) => isNum(o.lat) && isNum(o.lon)).forEach((o) => {
      L.marker([+o.lat, +o.lon], {
        interactive: false,
        icon: L.divIcon({ className: "", iconSize: [0, 0], html: `<div class="lmk lmk-tri">▲<span>${String(o.name || "").replace(/[<>&]/g, "")}</span></div>` }),
      }).addTo(g);
    });
    g.addTo(map); overlayRef.current = g;
    hud(map.getCenter());
  }, [others, origin, ready]);

  return (
    <div style={{ marginTop: 10 }}>
      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 4 }}>
        <ML style={{ marginBottom: 0 }}>Refine position — drag the ground under your pin</ML>
        <div style={{ display: "flex", gap: 4 }}>
          <button className="btn sm" onClick={() => setBaseSat((s) => !s)}>{baseSat ? "🗺 street" : "🛰 sat"}</button>
          <button className="btn sm" onClick={() => mapRef.current && mapRef.current.zoomOut()}>−</button>
          <button className="btn sm" onClick={() => mapRef.current && mapRef.current.zoomIn()}>+</button>
        </div>
      </div>
      <div className="pinmapwrap">
        <div ref={boxRef} style={{ position: "absolute", inset: 0 }} />
        <svg className="pinmap-cross" viewBox="-14 -14 28 28" width="28" height="28">
          <circle cx="0" cy="0" r="7" fill="none" stroke="#5FD3BC" strokeWidth="2" />
          <path d="M-12 0H12M0 -12V12" stroke="#5FD3BC" strokeWidth="2" />
        </svg>
        <div className="pinmap-you">YOU</div>
        <div className="pinmap-hud">
          <div ref={distElRef} style={{ color: "var(--amber)" }} />
          <div ref={coordElRef} />
        </div>
      </div>
    </div>
  );
}

/* ============================================================
   POSITION EDITOR — the wizard's map step
   ============================================================ */
function PositionEditor({ src, update, others }) {
  const [geoBusy, setGeoBusy] = useState(false);
  const [geoErr, setGeoErr] = useState("");
  const [demBusy, setDemBusy] = useState(false);
  const [demMsg, setDemMsg] = useState("");
  const posDone = isNum(src.lat) && isNum(src.lon);
  const grabDem = async () => {
    setDemBusy(true); setDemMsg("");
    try {
      const h = await demElevation(+src.lat, +src.lon);
      update({ alt: h.toFixed(0) });
      setDemMsg(`✓ terrain elevation ${h.toFixed(0)} m — steadier than phone GPS altitude (±5 m wobble)`);
    } catch (e) { setDemMsg("Terrain lookup failed — check the connection."); }
    setDemBusy(false);
  };
  const grabLocation = () => {
    setGeoErr(""); setGeoBusy(true);
    if (!navigator.geolocation) { setGeoErr("No GPS here — long-press your spot in a maps app and paste the coordinates."); setGeoBusy(false); return; }
    navigator.geolocation.getCurrentPosition(
      (pos) => {
        update({ lat: pos.coords.latitude.toFixed(6), lon: pos.coords.longitude.toFixed(6), alt: pos.coords.altitude ? pos.coords.altitude.toFixed(0) : src.alt });
        setGeoBusy(false);
      },
      () => { setGeoErr("GPS blocked — long-press your spot in Google/Apple Maps and paste the coordinates."); setGeoBusy(false); },
      { enableHighAccuracy: true, timeout: 8000 }
    );
  };
  return (
    <>
            <div className="grid3">
              <Num label="Latitude" value={src.lat} onChange={(v) => {
                const m = String(v).match(/(-?\d+(?:\.\d+)?)[,\s]+(-?\d+(?:\.\d+)?)/);
                if (m) update({ lat: m[1], lon: m[2] }); else update({ lat: v });
              }} ph="e.g. 42.1638" />
              <Num label="Longitude" value={src.lon} onChange={(v) => {
                const m = String(v).match(/(-?\d+(?:\.\d+)?)[,\s]+(-?\d+(?:\.\d+)?)/);
                if (m) update({ lat: m[1], lon: m[2] }); else update({ lon: v });
              }} ph="e.g. −123.6480" />
              <Num label="Elev" unit="m, opt" value={src.alt} onChange={(v) => update({ alt: v })} ph="0" />
            </div>
            <div style={{ marginTop: 8, display: "flex", gap: 8, alignItems: "center", flexWrap: "wrap" }}>
              {ENABLE_GPS_BUTTON && (
                <button className="btn sm teal" onClick={grabLocation}>{geoBusy ? "Locating…" : "📍 Use my GPS"}</button>
              )}
              {!posDone && src.meta && isNum(src.meta.lat) && (
                <button className="btn sm amber" onClick={() => update({ lat: String(src.meta.lat), lon: String(src.meta.lon), ...(isNum(src.meta.alt) ? { alt: String(src.meta.alt) } : {}) })}>
                  📎 Use the photo's GPS
                </button>
              )}
              {posDone && (
                <button className="btn sm" onClick={grabDem} disabled={demBusy}>{demBusy ? "…" : "⛰ Use terrain elevation"}</button>
              )}
              <span style={{ fontSize: 11, color: "var(--dim)" }}>{ENABLE_GPS_BUTTON ? "or long-press the spot in a maps app → paste" : "long-press your spot in a maps app → paste, or drag the map below"}</span>
            </div>
            {demMsg && <div style={{ fontSize: 12, color: demMsg.startsWith("✓") ? "var(--teal)" : "var(--red)", marginTop: 6 }}>{demMsg}</div>}
            {geoErr && <div className="warn">{geoErr}</div>}
            <div style={{ marginTop: 10 }}>
              <ML>Sighting date &amp; time</ML>
              <input type="datetime-local" value={toLocalInput(new Date(isNum(src.whenMs) ? +src.whenMs : Date.now()))}
                onChange={(e) => { const t = new Date(e.target.value).getTime(); if (!isNaN(t)) update({ whenMs: t }); }} />
            </div>
            {posDone && (
              <PinMap lat={+src.lat} lon={+src.lon}
                origin={src.meta && isNum(src.meta.lat) ? { lat: +src.meta.lat, lon: +src.meta.lon } : null}
                others={others}
                onChange={(la, lo) => update({ lat: la.toFixed(6), lon: lo.toFixed(6) })} />
            )}
    </>
  );
}

/* ============================================================
   PLOT BOARD — top-down view of observers, rays, fix and trajectory
   on real satellite imagery (Leaflet + Esri World Imagery, same base
   as PinMap). ENU solution points convert back to geo through the
   fix's own reference frame.
   ============================================================ */
function PlotBoard({ result, traj }) {
  const boxRef = useRef(null);
  const mapRef = useRef(null);
  const layerRef = useRef(null);
  useEffect(() => {
    const el = boxRef.current; if (!el || mapRef.current) return;
    const map = L.map(el, { attributionControl: true });
    map.attributionControl.setPrefix(false);
    L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
      maxZoom: 21, maxNativeZoom: 19, attribution: "© Esri, Maxar, Earthstar Geographics",
    }).addTo(map);
    L.control.scale({ imperial: true }).addTo(map);
    mapRef.current = map;
    return () => { map.remove(); mapRef.current = null; };
  }, []);
  useEffect(() => {
    const map = mapRef.current; if (!map || !result?.ok) return;
    if (layerRef.current) { map.removeLayer(layerRef.current); layerRef.current = null; }
    const g = L.layerGroup();
    const geo = (P) => { const w = geoFromEnu(P, result.ref); return [w.lat, w.lon]; };
    const esc = (t) => String(t || "").replace(/[<>&]/g, "");
    const bounds = [];
    result.obs.forEach((o, i) => {
      const far = add(o.P, scl(o.dA, Math.max((result.solA.ts[i] || 0) * 1.15, 500)));
      L.polyline([geo(o.P), geo(far)], { color: "#F5A93F", weight: 1.5, dashArray: "5 5", opacity: 0.8, interactive: false }).addTo(g);
      L.marker(geo(o.P), { interactive: false, icon: L.divIcon({ className: "", iconSize: [0, 0], html: `<div class="lmk lmk-tri">▲<span>${esc(o.s.name || `Obs ${i + 1}`)}</span></div>` }) }).addTo(g);
      bounds.push(geo(o.P));
    });
    if (traj && traj.length > 1) {
      L.polyline(traj.map(geo), { color: "#8FB4FF", weight: 2.5, opacity: 0.95, interactive: false }).addTo(g);
    }
    const A = geo(result.solA.X); bounds.push(A);
    L.marker(A, { interactive: false, icon: L.divIcon({ className: "", iconSize: [0, 0], html: `<div class="lmk lmk-fix">⊕<span>FIX A</span></div>` }) }).addTo(g);
    if (result.motion?.XB) {
      const B = geo(result.motion.XB); bounds.push(B);
      L.polyline([A, B], { color: "#5FD3BC", weight: 2, opacity: 0.9, interactive: false }).addTo(g);
      L.marker(B, { interactive: false, icon: L.divIcon({ className: "", iconSize: [0, 0], html: `<div class="lmk lmk-fix">◯<span>B</span></div>` }) }).addTo(g);
    }
    g.addTo(map); layerRef.current = g;
    try { map.fitBounds(L.latLngBounds(bounds).pad(0.3), { maxZoom: 16, animate: false }); } catch (e) { }
  }, [result, traj]);
  if (!result?.ok) return null;
  return <div className="plotwrap"><div ref={boxRef} style={{ position: "absolute", inset: 0 }} /></div>;
}

/* ============================================================
   ADS-B CHECK — rank live aircraft against the witness sight-lines.
   Live only (historical replay is on the roadmap), so it's most
   meaningful right after a sighting; the time-gap warning is honest
   about that.
   ============================================================ */
function AdsbCheck({ sources }) {
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState("");
  const [data, setData] = useState(null);
  const valid = sources.filter((s) => isNum(s.lat) && isNum(s.lon) && isNum(s.A?.az) && isNum(s.A?.el));
  if (!valid.length) return null;
  const whenMs = +valid[0].whenMs || Date.now();
  const ageMin = Math.abs(Date.now() - whenMs) / 60000;

  const run = async () => {
    setBusy(true); setErr(""); setData(null);
    try {
      const nm = radiusNmForSources(valid);
      let got = null;
      if (ageMin > 15) {
        try { got = await fetchAircraftAt(+valid[0].lat, +valid[0].lon, whenMs, nm); }
        catch (e) { /* archive miss — fall back to live below */ }
      }
      if (!got) got = await fetchAircraft(+valid[0].lat, +valid[0].lon, nm);
      const cands = rankCandidates(valid, got.ac) || [];
      setData({ cands, source: got.source, nm, fetchedAt: Date.now(), total: got.ac.length, hist: !!got.hist });
    } catch (e) {
      setErr(`Couldn't reach an ADS-B source (${e.message || e}). Check the connection and try again.`);
    }
    setBusy(false);
  };

  const measAng = (s) =>
    angSizeFromPoints(s.A?.p1, s.A?.p2, s.natW, s.natH, +s.fovH) ??
    (isNum(s.A?.angManual) ? +s.A.angManual : null);

  return (
    <Section title="✈ Aircraft check (ADS-B)" collapsible>
      <div style={{ fontSize: 12, color: "var(--dim)", lineHeight: 1.5 }}>
        Queries live air traffic around the observers and ranks every aircraft by how far
        it sits off each witness's sight-line. Type → wingspan gives an absolute size check.
      </div>
      <button className="btn teal" style={{ width: "100%", marginTop: 10 }} onClick={run} disabled={busy}>
        {busy ? "Querying traffic…" : ageMin > 15 ? "🛰 Check archived traffic at the sighting time" : "🛰 Check live aircraft now"}
      </button>
      {err && <div className="warn">{err}</div>}
      {data && !data.hist && ageMin > 15 && (
        <div className="warn">
          ⚠ Archive unavailable for that time — showing aircraft in the air <b>now</b>, but the sighting is{" "}
          {ageMin > 2880 ? `${Math.round(ageMin / 1440)} days` : ageMin > 120 ? `${Math.round(ageMin / 60)} h` : `${Math.round(ageMin)} min`} away.
          Treat as context only.
        </div>
      )}
      {data && (
        <div style={{ marginTop: 10 }}>
          <div style={{ fontSize: 11, color: "var(--dim)", fontFamily: "var(--mono)" }}>
            {data.total} aircraft within {data.nm} nm · {data.source}{data.hist ? ` · at ${new Date(whenMs).toLocaleString()}` : ` · live ${new Date(data.fetchedAt).toLocaleTimeString()}`}
          </div>
          {data.cands.length === 0 && (
            <div className="ok" style={{ marginTop: 8 }}>
              No transponder-equipped aircraft in range right now. (Some military and older light
              aircraft carry no ADS-B — absence here rules out airliners, not everything.)
            </div>
          )}
          {data.cands.slice(0, 8).map((c) => {
            const on = c.sepMax < 2.5, near = c.sepMax < 8;
            return (
              <div key={c.hex} style={{ borderTop: "1px solid var(--line)", padding: "8px 0", fontFamily: "var(--mono)", fontSize: 12 }}>
                <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
                  <span style={{ color: "var(--ink)", fontWeight: 700 }}>
                    {c.flight || c.reg || c.hex}{c.t ? ` · ${c.t}` : ""}{c.span != null ? ` · ${c.span.toFixed(0)} m span` : ""}
                  </span>
                  <span style={{ color: on ? "var(--teal)" : near ? "var(--amber)" : "var(--dim)", fontWeight: 700 }}>
                    {on ? "◉ ON the sight-line" : near ? "◎ near" : `${c.sepMax.toFixed(1)}° off`}
                  </span>
                </div>
                {c.per.map((p, i) => {
                  const m = measAng(valid[i]);
                  return (
                    <div key={i} style={{ color: "var(--dim)", marginTop: 2 }}>
                      {valid.length > 1 ? `${p.name}: ` : ""}{p.sep.toFixed(1)}° off · seen at {p.az.toFixed(0)}°/{p.el.toFixed(0)}° (witness {(+valid[i].A.az).toFixed(0)}°/{(+valid[i].A.el).toFixed(0)}°) · {fmtLenShort(p.rangeM)}
                      {p.predAng != null && <> · would appear <span style={{ color: "var(--teal)" }}>{p.predAng.toFixed(2)}°</span>{m != null && <> vs measured <span style={{ color: "var(--amber)" }}>{m.toFixed(2)}°</span></>}</>}
                    </div>
                  );
                })}
                <div style={{ color: "var(--dim)", marginTop: 2 }}>
                  {c.altM != null ? `FL ${(c.altM / FT_M / 100).toFixed(0)} · ` : ""}{c.gs != null ? `${(c.gs * 2.23694).toFixed(0)} mph` : ""}{c.track != null ? ` · trk ${c.track.toFixed(0)}°` : ""}
                </div>
              </div>
            );
          })}
        </div>
      )}
    </Section>
  );
}
const FT_M = 0.3048;

/* ============================================================
   RESULTS PANEL
   ============================================================ */
function ResultsPanel({ sources }) {
  const result = useMemo(() => analyze(sources), [sources]);
  const trk = useMemo(() => analyzeTracks(sources), [sources]);
  const [soloT, setSoloT] = useState(0.42);

  const hasTrackContent = (trk.stereo && (trk.stereo.k || trk.stereo.overlapErr)) || trk.solo.length > 0;
  if (!result.ok && !hasTrackContent) return null;
  const r = result;
  const ratingColor = { excellent: "var(--teal)", good: "var(--teal)", fair: "var(--amber)", poor: "var(--red)" }[r.rating];

  return (
    <div>
      {!result.ok && (
        <Section title="Fix status">
          <div style={{ fontSize: 13, color: "var(--dim)" }}>
            Trajectory data found, but a position fix needs 2 observers with lat/lon + Moment-A bearing and elevation (currently complete: {result.validCount}). Distances and g-forces below use an assumed range until then.
          </div>
        </Section>
      )}
      {result.ok && (<>
      <Section title="Position fix — Moment A"
        right={<span style={{ fontFamily: "var(--mono)", fontSize: 11, color: ratingColor, letterSpacing: ".1em", textTransform: "uppercase" }}>◉ {r.rating}</span>}>
        <PlotBoard result={r} traj={trk.stereo && trk.stereo.k ? trk.stereo.pos : null} />
        {r.behind && <div className="warn">⚠ At least one solution point falls <b>behind</b> an observer — the bearings likely don't describe the same object, or one compass reading is off.</div>}

        <div className="grid2" style={{ marginTop: 12 }}>
          <div>
            <ML>Object altitude (above Obs 1)</ML>
            <div className="readout">{fmtLenShort(r.solA.X[2])}</div>
            <div className="readsub">{n1(r.solA.X[2] * 3.28084)} ft</div>
          </div>
          <div>
            <ML>Object ground position</ML>
            <div className="readout" style={{ fontSize: 14 }}>{r.geoA.lat.toFixed(5)}, {r.geoA.lon.toFixed(5)}</div>
            <div className="readsub">± {fmtLenShort(r.posErr)} (assuming ±1° pointing error)</div>
          </div>
        </div>

        <table className="tbl" style={{ marginTop: 12 }}>
          <thead><tr><th>Observer</th><th>Range</th><th>Ang size</th><th>→ True size</th></tr></thead>
          <tbody>
            {r.perSource.map((p, i) => (
              <tr key={i}>
                <td>{p.name}</td>
                <td>{fmtLenShort(p.dist)}</td>
                <td>{p.ang != null ? fmtDeg(p.ang) : "—"}</td>
                <td style={{ color: "var(--teal)" }}>{p.size != null ? fmtLenShort(p.size) : "—"}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </Section>

      {r.sizeAvg != null && (
        <Section title="Object size (from triangulated range)">
          <div className="readout" style={{ fontSize: 30 }}>{fmtLenShort(r.sizeAvg)}</div>
          <div className="readsub">{n1(r.sizeAvg * 3.28084)} ft across (longest marked dimension)</div>
          <div style={{ marginTop: 8, fontSize: 12, color: "var(--dim)" }}>
            Nearest reference: <b style={{ color: "var(--ink)" }}>{REF_OBJECTS.reduce((best, o) => Math.abs(Math.log(o.size / r.sizeAvg)) < Math.abs(Math.log(best.size / r.sizeAvg)) ? o : best).name}</b>
          </div>
        </Section>
      )}

      {r.motion && (
        <Section title="Motion — Moment A → B">
          <div className="grid2">
            <div>
              <ML>Displacement</ML>
              <div className="readout">{fmtLenShort(r.motion.disp)}</div>
              <div className="readsub">Δalt {n1(r.motion.XB[2] - r.solA.X[2])} m</div>
            </div>
            <div>
              <ML>Δ time</ML>
              <div className="readout">{r.motion.dt != null ? `${r.motion.dt.toFixed(2)} s` : "—"}</div>
              {r.motion.dt == null && <div className="readsub" style={{ color: "var(--amber)" }}>Enter clock times at A & B for both observers to get speed</div>}
            </div>
          </div>
          {r.motion.v && (
            <>
              <div className="hr" />
              <ML>Ground speed</ML>
              <div className="readout" style={{ fontSize: 26 }}>{n1(r.motion.speed * 2.23694)} mph</div>
              <div className="readsub">{fmtSpeed(r.motion.speed)}</div>
              <div className="grid2" style={{ marginTop: 10 }}>
                <div>
                  <ML>Heading</ML>
                  <div className="readout" style={{ fontSize: 18 }}>{Math.round(r.motion.heading)}° {compass8(r.motion.heading)}</div>
                </div>
                <div>
                  <ML>Vertical rate</ML>
                  <div className="readout" style={{ fontSize: 18 }}>{r.motion.vRate >= 0 ? "▲" : "▼"} {n1(Math.abs(r.motion.vRate))} m/s</div>
                </div>
              </div>
            </>
          )}
        </Section>
      )}

      <Section title="Solution quality" collapsible>
        <table className="tbl">
          <tbody>
            <tr><td>Baseline (observer separation)</td><td>{fmtLenShort(r.baseline)}</td></tr>
            <tr><td>Ray convergence angle</td><td>{fmtDeg(r.conv)}</td></tr>
            <tr><td>Ray miss distance (RMS)</td><td>{fmtLenShort(r.solA.rmsMiss)} ({(r.missRatio * 100).toFixed(1)}% of range)</td></tr>
            <tr><td>Range / baseline ratio</td><td>{(r.meanDist / Math.max(1, r.baseline)).toFixed(1)} : 1</td></tr>
          </tbody>
        </table>
        {r.conv < 2 && <div className="warn">⚠ Convergence under 2° — observers are too close together relative to the object's distance. Range and size are weakly constrained. A baseline of at least 1/10 of the range is ideal.</div>}
        {r.missRatio > 0.1 && !r.behind && <div className="warn">⚠ The sightlines pass {fmtLenShort(r.solA.rmsMiss)} apart — either the bearings/elevations carry error, or the observations weren't simultaneous. Treat results as rough.</div>}
        {r.rating === "excellent" && <div className="ok">✓ Tight geometry. Sightlines nearly intersect and the convergence angle is healthy — these numbers are trustworthy to roughly ±{fmtLenShort(r.posErr)}.</div>}
      </Section>
      </>)}

      {trk.stereo && trk.stereo.overlapErr && (
        <div className="warn" style={{ margin: "10px 12px" }}>
          ⚠ Multiple observers have tracks, but their time windows don't overlap. Track times sync through each source's Moment-A clock time — set "Set time A" on the anchor frame, then put both observers' A times on a common clock.
        </div>
      )}
      {trk.stereo && trk.stereo.k && <TrajectoryStereoSection stereo={trk.stereo} />}
      {!(trk.stereo && trk.stereo.k) && trk.solo.length > 0 && (
        <SoloTrackSection solo={trk.solo} t={soloT} setT={setSoloT} />
      )}

      <AdsbCheck sources={sources} />
    </div>
  );
}

/* ============================================================
   APP
   ============================================================ */
/* ============================================================
   WIZARD — one page at a time. The only workflow.
   ============================================================ */
let _crcT = null;
function crc32buf(u8) {
  if (!_crcT) {
    _crcT = new Int32Array(256);
    for (let n = 0; n < 256; n++) { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xEDB88320 ^ (c >>> 1) : c >>> 1; _crcT[n] = c; }
  }
  let crc = -1;
  for (let i = 0; i < u8.length; i++) crc = (crc >>> 8) ^ _crcT[(crc ^ u8[i]) & 255];
  return (crc ^ -1) >>> 0;
}
function makeZip(files) { // STORE-method zip: JPEGs are incompressible anyway
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
  return new Blob(chunks, { type: "application/zip" });
}
const strU8 = (s) => new TextEncoder().encode(s);
const dataUrlU8 = (durl) => {
  const b64 = durl.slice(durl.indexOf(",") + 1);
  const bin = atob(b64);
  const u8 = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u8[i] = bin.charCodeAt(i);
  return u8;
};

const download = (name, payload, mime) => {
  try {
    const b = payload instanceof Blob ? payload : new Blob([payload], { type: mime });
    const u = URL.createObjectURL(b);
    const a = document.createElement("a");
    a.href = u; a.download = name;
    document.body.appendChild(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(u), 4000);
    return true;
  } catch (e) { return false; }
};

const isEmptySource = (s) =>
  !s.mediaUrl && !isNum(s.lat) && !isNum(s.A?.az) && !(s.track || []).length && !s.shapeFit && !s.A?.p1;

/* Bundle each observer with a 1600px copy of their photo, rescaling ALL
   pixel-space data to match so the export is self-consistent (angles are
   pixel-ratio invariant). Analysis always runs on the full-res originals. */
async function packSources(sources) {
  const act = sources.filter((s) => !isEmptySource(s));
  const out = [];
  for (const s of act) {
    const { mediaUrl, mediaKind, mediaNorm, open, ...r } = s;
    if (mediaUrl && mediaKind === "image" && r.natW) {
      try {
        const im = await new Promise((res, rej) => { const i = new Image(); i.onload = () => res(i); i.onerror = rej; i.src = mediaUrl; });
        const k = Math.min(1600, r.natW) / r.natW;
        const cv = document.createElement("canvas");
        cv.width = Math.round(r.natW * k); cv.height = Math.round(r.natH * k);
        cv.getContext("2d").drawImage(im, 0, 0, cv.width, cv.height);
        /* detail crop of just the object — bbox of the fitted shape (or the
           edge marks) + 65% outer margin, cut from the full-res pixels and
           enlarged with honest (unsmoothed) pixels when the object is tiny */
        try {
          let bb = null;
          if (r.shapeFit && r.shapeFit.sizeNat) {
            const pr = shapeProjNat(r.shapeFit);
            bb = pr.curves.flat().reduce(
              (m, p) => ({ x0: Math.min(m.x0, p.x), y0: Math.min(m.y0, p.y), x1: Math.max(m.x1, p.x), y1: Math.max(m.y1, p.y) }),
              { x0: 1e9, y0: 1e9, x1: -1e9, y1: -1e9 });
          } else if (r.A?.p1 && r.A?.p2) {
            bb = { x0: Math.min(r.A.p1.x, r.A.p2.x), y0: Math.min(r.A.p1.y, r.A.p2.y), x1: Math.max(r.A.p1.x, r.A.p2.x), y1: Math.max(r.A.p1.y, r.A.p2.y) };
          }
          if (bb && bb.x1 > bb.x0 - 1) {
            const pad = Math.max(28, Math.max(bb.x1 - bb.x0, bb.y1 - bb.y0) * 0.65);
            const cx0 = clampN(bb.x0 - pad, 0, r.natW), cy0 = clampN(bb.y0 - pad, 0, r.natH);
            const cw = Math.max(8, clampN(bb.x1 + pad, 0, r.natW) - cx0);
            const ch = Math.max(8, clampN(bb.y1 + pad, 0, r.natH) - cy0);
            const oz = Math.min(720, Math.max(320, cw)) / cw;
            const dc = document.createElement("canvas");
            dc.width = Math.round(cw * oz); dc.height = Math.round(ch * oz);
            const dctx = dc.getContext("2d");
            if (oz > 3) dctx.imageSmoothingEnabled = false;
            dctx.drawImage(im, cx0, cy0, cw, ch, 0, 0, dc.width, dc.height);
            r.detailJpeg = dc.toDataURL("image/jpeg", 0.85);
            r.detailZoom = +oz.toFixed(1);
          }
        } catch (e) { /* no detail crop */ }
        const sp = (p) => (p ? { ...p, x: p.x * k, y: p.y * k } : p);
        r.natW = cv.width; r.natH = cv.height;
        r.A = { ...r.A, p1: sp(r.A?.p1), p2: sp(r.A?.p2) };
        r.B = { ...r.B, pb: sp(r.B?.pb) };
        r.track = (r.track || []).map((p) => (p.x != null ? { ...p, x: p.x * k, y: p.y * k } : p));
        if (r.shapeFit) r.shapeFit = { ...r.shapeFit, cx: r.shapeFit.cx * k, cy: r.shapeFit.cy * k, sizeNat: r.shapeFit.sizeNat * k };
        r.mediaJpeg = cv.toDataURL("image/jpeg", 0.8);
      } catch (e) { /* export without the image */ }
    }
    out.push(r);
  }
  return out;
}

async function buildShareJson(sources, est) {
  return JSON.stringify({
    phodar: 1,
    created: new Date().toISOString(),
    sources: await packSources(sources),
    est,
  }, null, 1);
}

async function reportHtml(sources, est, opts = {}) {
  const fix = analyze(sources);            // numbers from full-res originals
  const tr = analyzeTracks(sources);
  const packed = await packSources(sources); // bundle: filtered + 1600px media
  const origAct = sources.filter((s) => !isEmptySource(s)); // aligned with packed
  const e2 = (s) => String(s ?? "").replace(/&/g, "&amp;").replace(/</g, "&lt;");
  const dv = (s) => (s === "" || s == null ? "—" : e2(s));
  const ft = (m) => Math.round(m * 3.28084);
  const row = (k, v) => `<tr><td>${k}</td><td>${v ?? "—"}</td></tr>`;
  const obsRows = packed.map((s, i) =>
    `<tr><td>${e2(s.name || "Observer " + (i + 1))}</td><td>${dv(s.lat)}, ${dv(s.lon)} · ${dv(s.alt || 0)} m</td><td>${s.whenMs ? new Date(+s.whenMs).toLocaleString() : "—"}</td><td>${dv(s.A?.az)}° / ${dv(s.A?.el)}°</td><td>${isNum(s.fovH) ? (+s.fovH).toFixed(1) + "°" : "—"}</td><td>${(s.track || []).length}</td></tr>`
  ).join("");
  let fixHtml;
  if (fix.ok) {
    fixHtml = `<table>` +
      row("Altitude above observer 1", `${fmtLenShort(fix.solA.X[2])} (${ft(fix.solA.X[2])} ft)`) +
      row("Range from observer 1", fmtLenShort(fix.perSource[0].dist)) +
      (fix.sizeAvg != null ? row("Object size (avg)", `${fmtLenShort(fix.sizeAvg)} (${ft(fix.sizeAvg)} ft)`) : "") +
      ((asp) => asp ? row("Aspect-corrected span (if elongated)", `${asp.map((x) => `${fmtLenShort(x.S)} @ long-axis ${Math.round(x.psi)}°`).join(" or ")} (${asp[0].n} views${asp[0].rms != null ? `, fit rms ${fmtLenShort(asp[0].rms)}` : ""})`) : "")(aspectSpan(fix)) +
      (fix.motion?.speed != null ? row("Speed A→B", `${Math.round(fix.motion.speed)} m/s (${Math.round(fix.motion.speed * 2.23694)} mph), heading ${Math.round(fix.motion.heading)}°`) : "") +
      row("Baseline", fmtLenShort(fix.baseline)) +
      row("Ray convergence", fix.conv.toFixed(1) + "°") +
      row("Quality", fix.rating + (fix.behind ? " — rays cross BEHIND an observer; treat as unreliable (see caveats)" : "")) +
      `</table>`;
  } else {
    fixHtml = `<p><i>Fewer than two complete observers — angular data only. Import this file into Phodar and add a second perspective to triangulate.</i></p>`;
    /* single witness: the honest deliverable is the size↔distance line —
       every assumed distance implies a size; reference objects pin intuition */
    const wAng = (() => {
      for (const s of origAct) {
        const a = angSizeFromPoints(s.A?.p1, s.A?.p2, s.natW, s.natH, +s.fovH) ?? (isNum(s.A?.angManual) ? +s.A.angManual : null);
        if (a != null && a > 0) return a;
      }
      return null;
    })();
    if (wAng != null) {
      const W = 560, H = 300, L = 62, Rm = 16, T = 34, B = 44;
      const D0 = 50, D1 = 50000;
      const s0 = 2 * D0 * Math.tan(wAng * D2R / 2), s1 = 2 * D1 * Math.tan(wAng * D2R / 2);
      const sLo = Math.min(s0, 0.2), sHi = Math.max(s1, 120);
      const X = (Dm) => L + ((Math.log10(Dm) - Math.log10(D0)) / (Math.log10(D1) - Math.log10(D0))) * (W - L - Rm);
      const Y = (Sm) => T + (1 - (Math.log10(Sm) - Math.log10(sLo)) / (Math.log10(sHi) - Math.log10(sLo))) * (H - T - B);
      const refs = REF_OBJECTS.filter((o) => {
        const Dq = o.size / (2 * Math.tan(wAng * D2R / 2));
        return Dq >= D0 && Dq <= D1;
      }).map((o) => {
        const Dq = o.size / (2 * Math.tan(wAng * D2R / 2));
        return `<line x1="${L}" y1="${Y(o.size).toFixed(1)}" x2="${W - Rm}" y2="${Y(o.size).toFixed(1)}" stroke="#ddd" stroke-dasharray="4 4"/>` +
          `<circle cx="${X(Dq).toFixed(1)}" cy="${Y(o.size).toFixed(1)}" r="3.5" fill="#C77B14"/>` +
          `<text x="${(X(Dq) + 6).toFixed(1)}" y="${(Y(o.size) - 5).toFixed(1)}" font-size="10" fill="#555">${e2(o.name)} — ${fmtLenShort(Dq)}</text>`;
      }).join("");
      const xTicks = [100, 1000, 10000].map((d) => `<line x1="${X(d)}" y1="${T}" x2="${X(d)}" y2="${H - B}" stroke="#eee"/><text x="${X(d)}" y="${H - B + 16}" font-size="10" fill="#555" text-anchor="middle">${fmtLenShort(d)}</text>`).join("");
      const yTicks = [1, 10, 100].filter((s) => s >= sLo && s <= sHi).map((s) => `<text x="${L - 6}" y="${(Y(s) + 3).toFixed(1)}" font-size="10" fill="#555" text-anchor="end">${fmtLenShort(s)}</text>`).join("");
      fixHtml += `<svg viewBox="0 0 ${W} ${H}" style="max-width:100%;border:1px solid #ddd;border-radius:6px;background:#fff">
<text x="${L}" y="20" font-size="12" font-weight="700" fill="#333">Assumed distance ⇄ implied size (measured ${wAng.toFixed(2)}°)</text>
${xTicks}${yTicks}${refs}
<line x1="${X(D0)}" y1="${Y(s0).toFixed(1)}" x2="${X(D1)}" y2="${Y(s1).toFixed(1)}" stroke="#0e7d6f" stroke-width="2.5"/>
<text x="${W / 2}" y="${H - 8}" font-size="10" fill="#888" text-anchor="middle">assumed distance →</text>
<text x="14" y="${H / 2}" font-size="10" fill="#888" transform="rotate(-90 14 ${H / 2})" text-anchor="middle">implied size →</text>
</svg>
<p class="cap">One witness can't fix the distance — but every assumed distance implies a size. Dots mark where common objects would sit on this sight-line.</p>`;
    }
  }
  const kin = tr.stereo?.k ? `<table>` +
    row("Samples / duration", `${tr.stereo.k.n} pts · ${tr.stereo.k.dur.toFixed(1)} s`) +
    row("Path length", fmtLenShort(tr.stereo.k.path)) +
    row("Avg / peak speed", `${Math.round(tr.stereo.k.avgSpeed)} / ${Math.round(tr.stereo.k.peakSpeed)} m/s (${Math.round(tr.stereo.k.peakSpeed * 2.23694)} mph peak)`) +
    (tr.stereo.k.peakA != null ? row("Peak acceleration", tr.stereo.k.peakA.toFixed(1) + " m/s²") : "") +
    (tr.stereo.k.peakLoad != null ? row("Peak felt load", tr.stereo.k.peakLoad.toFixed(2) + " g") : "") +
    (tr.stereo.k.peakTurn != null ? row("Peak turn rate", tr.stereo.k.peakTurn.toFixed(1) + " °/s") : "") +
    `</table>` : "";
  const soloKin = (!tr.stereo?.k && tr.solo?.length) ? `<p class="cap">Single-view angular trajectory: ${tr.solo[0].k.n} pts over ${tr.solo[0].k.dur.toFixed(1)} s, peak angular rate ${(tr.solo[0].k.peakSpeed * R2D).toFixed(2)} °/s (distance-free).</p>` : "";

  /* --- ADS-B: rank the captured traffic snapshot against the final sight-lines --- */
  let adsbHtml = "";
  {
    const snaps = origAct.map((s) => s.adsb).filter((a) => a && a.ac && a.ac.length >= 0);
    const snap = snaps.sort((a, b) => (b.fetchedAt || 0) - (a.fetchedAt || 0))[0];
    if (snap) {
      const cands = rankCandidates(sources, snap.ac) || [];
      const when = origAct.find((s) => isNum(s.whenMs))?.whenMs;
      const gapMin = when ? Math.abs(snap.fetchedAt - +when) / 60000 : null;
      const gapTxt = snap.hist
        ? `archived traffic <b>at the sighting time</b> (${when ? new Date(+when).toLocaleString() : "—"}).`
        : gapMin == null ? "" : gapMin < 20 ? `live capture ${Math.round(gapMin)} min from the sighting time — a fair comparison.`
          : gapMin < 1440 ? `live capture <b>${(gapMin / 60).toFixed(1)} h from the sighting time</b> — traffic has turned over; treat as context, not evidence.`
            : `live capture <b>${Math.round(gapMin / 1440)} days from the sighting time</b> — traffic has fully turned over; treat as context only.`;
      const measA = (() => {
        const w = sources.find((s) => isNum(s.lat) && isNum(s.A?.az) && isNum(s.A?.el));
        if (!w) return null;
        return angSizeFromPoints(w.A?.p1, w.A?.p2, w.natW, w.natH, +w.fovH) ?? (isNum(w.A?.angManual) ? +w.A.angManual : null);
      })();
      const top = cands.slice(0, 6);
      const rows = top.map((c) => {
        const p = c.per[0];
        return `<tr><td>${e2(c.flight || c.reg || c.hex)}${c.t ? ` · ${e2(c.t)}` : ""}</td><td>${c.span != null ? c.span.toFixed(0) + " m" : "—"}</td><td>${c.sepMax.toFixed(1)}°</td><td>${p.az.toFixed(0)}° / ${p.el.toFixed(0)}°</td><td>${fmtLenShort(p.rangeM)}</td><td>${p.predAng != null ? p.predAng.toFixed(2) + "°" : "—"}${measA != null ? ` vs ${measA.toFixed(2)}°` : ""}</td><td>${c.altM != null ? Math.round(c.altM * 3.28084).toLocaleString() + " ft" : "—"}${c.gs != null ? ` · ${Math.round(c.gs * 2.23694)} mph` : ""}</td></tr>`;
      }).join("");
      const best = cands[0];
      const verdict = !cands.length
        ? `No airborne transponder aircraft were within ${snap.nm} nm at check time. ADS-B absence rules out airliners and most GA — not military or non-transponder traffic.`
        : best.sepMax < 2.5
          ? `<b>${e2(best.flight || best.reg || best.hex)}</b> sat within ${best.sepMax.toFixed(1)}° of every witness sight-line at check time — a strong mundane candidate; compare its predicted angular size against the measurement above.`
          : `No aircraft sat on the sight-line (closest: ${e2(best.flight || best.reg || best.hex)}, ${best.sepMax.toFixed(1)}° off at the worst witness).`;
      adsbHtml = `<h2>Aircraft check (ADS-B)</h2>
<p class="cap">Source: ${e2(snap.src)} · ${gapTxt || `captured ${new Date(snap.fetchedAt).toLocaleString()}`}</p>
${cands.length ? `<table><tr><th>Flight</th><th>Span</th><th>Off sight-line (worst witness)</th><th>Seen at az/el</th><th>Range</th><th>Would appear vs measured</th><th>Alt · speed</th></tr>${rows}</table>` : ""}
<p>${verdict}</p>`;
    }
  }
  const exhibits = packed.map((s, i) => {
    let imgSrc = s.mediaJpeg;
    if (opts.exhibits === "full" && origAct[i]?.mediaUrl && origAct[i].mediaKind === "image") imgSrc = origAct[i].mediaUrl;
    else if (opts.exhibits === "files") imgSrc = s.mediaJpeg ? `photos/observer-${i + 1}.jpg` : null;
    if (!imgSrc) return "";
    let overlay = "";
    if (s.shapeFit) {
      const pr = shapeProjNat(s.shapeFit);
      const colR = `hsl(${s.shapeFit.hue ?? 36},85%,38%)`;
      const paths = pr.curves.map((c) =>
        `<polyline fill="none" stroke="${colR}" stroke-width="${Math.max(1.2, s.natW / 950)}" opacity="0.55" points="${c.map((p) => p.x.toFixed(1) + "," + p.y.toFixed(1)).join(" ")}"/>`
      ).join("");
      overlay = `<svg viewBox="0 0 ${s.natW} ${s.natH}" style="position:absolute;left:0;top:0;width:100%;height:100%">${paths}</svg>`;
    } else if (s.A?.p1 && s.A?.p2) {
      overlay = `<svg viewBox="0 0 ${s.natW} ${s.natH}" style="position:absolute;left:0;top:0;width:100%;height:100%"><line x1="${s.A.p1.x}" y1="${s.A.p1.y}" x2="${s.A.p2.x}" y2="${s.A.p2.y}" stroke="#C77B14" stroke-width="${Math.max(2, s.natW / 500)}"/><circle cx="${s.A.p1.x}" cy="${s.A.p1.y}" r="${Math.max(6, s.natW / 160)}" fill="none" stroke="#C77B14" stroke-width="${Math.max(2, s.natW / 500)}"/><circle cx="${s.A.p2.x}" cy="${s.A.p2.y}" r="${Math.max(6, s.natW / 160)}" fill="none" stroke="#C77B14" stroke-width="${Math.max(2, s.natW / 500)}"/></svg>`;
    }
    return `<h2>Exhibit — ${e2(s.name || "Observer " + (i + 1))}</h2>
<div style="position:relative;display:inline-block;max-width:100%"><img src="${imgSrc}" style="max-width:100%;display:block"/>${overlay}</div>
<div class="cap">${s.meta?.model ? e2(s.meta.model) + " · " : ""}${s.whenMs ? new Date(+s.whenMs).toLocaleString() : ""}${s.mediaAim ? ` · placed ${(+s.mediaAim.az).toFixed(1)}° az / ${(+s.mediaAim.el).toFixed(1)}° el` : ""}${s.shapeFit ? ` · ${e2(s.shapeFit.kind)} fit` : ""}</div>
${s.detailJpeg ? `<div style="margin-top:8px"><img src="${s.detailJpeg}" style="max-width:min(380px,100%);display:block;border:1px solid #ccc;border-radius:4px"/><div class="cap">detail — cropped at the fitted shape, ×${s.detailZoom} enlargement, no overlay</div></div>` : ""}`;
  }).join("");
  let diagHtml = "";
  if (fix.ok) {
    const alts = sources.filter((s) => isNum(s.lat) && isNum(s.A?.az)).map((s) => (isNum(s.alt) ? +s.alt : 0));
    const spread = alts.length > 1 ? Math.max(...alts) - Math.min(...alts) : 0;
    if (spread > 3 && spread > fix.baseline * 0.35)
      diagHtml += `<p>⚠ Observer GPS altitudes differ by ${spread.toFixed(1)} m on a ${fmtLenShort(fix.baseline)} baseline — phone altitude wobble tilts the rays at short range. If the observers stood on level ground, set their elevations equal and regenerate.</p>`;
    try {
      const misses = fix.obs.map((o) => {
        const d = dirFromAzEl(+o.s.A.az, +o.s.A.el);
        const Po = [
          (+o.s.lon - fix.ref.lon) * 111320 * Math.cos(fix.ref.lat * D2R),
          (+o.s.lat - fix.ref.lat) * 111320,
          (isNum(o.s.alt) ? +o.s.alt : 0) - fix.ref.alt,
        ];
        const v = [fix.solA.X[0] - Po[0], fix.solA.X[1] - Po[1], fix.solA.X[2] - Po[2]];
        const tt = v[0] * d[0] + v[1] * d[1] + v[2] * d[2];
        return { name: o.s.name, m: Math.hypot(v[0] - d[0] * tt, v[1] - d[1] * tt, v[2] - d[2] * tt) };
      });
      diagHtml += `<p class="cap">Per-observer ray miss vs the joint fix: ${misses.map((x) => `${e2(x.name)} ${fmtLenShort(x.m)}`).join(" · ")}.`;
      const srt = [...misses].sort((x, y) => y.m - x.m);
      const med = srt[Math.floor(srt.length / 2)].m;
      if (srt.length > 2 && srt[0].m > 1 && srt[0].m > 2.5 * (med + 0.01))
        diagHtml += ` <b>${e2(srt[0].name)}</b> is the outlier — re-check that shot's compass (metal structures and vehicles deflect it) and its sky placement.`;
      diagHtml += `</p>`;
    } catch (e) { }
  }
  const arbR = arbitrateBearings(sources);
  if (arbR?.best)
    diagHtml += `<p>⚠ Bearings inconsistent: trusting <b>${e2(arbR.best.trustName)}</b>, <b>${e2(arbR.best.otherName)}</b>'s compass reads ≈ ${Math.round(arbR.best.err)}° off (true bearing ≈ ${arbR.best.azOtherTrue.toFixed(1)}°).</p>`;
  /* --- sky-object check: Sun, Moon, planets, brightest stars vs each
     sight-line at the sighting time — Venus is the most-reported "UFO"
     there is, so this table earns its place in every report. --- */
  let skyHtml = "";
  {
    const wit = origAct.filter((s) => isNum(s.lat) && isNum(s.lon) && isNum(s.A?.az) && isNum(s.A?.el) && isNum(s.whenMs));
    if (wit.length) {
      let satDbR = null;
      try { satDbR = await loadSats(); } catch (e) { /* offline — the rest of the check still runs */ }
      const hits = [];
      for (const w of wit) {
        const Tw = +w.whenMs, la = +w.lat, lo = +w.lon;
        const d = dirFromAzEl(+w.A.az, +w.A.el);
        const cand = [];
        const sunW = sunPos(Tw, la, lo); if (sunW.alt > -2) cand.push({ label: "☀ Sun", az: sunW.az, alt: sunW.alt });
        const moonW = moonPos(Tw, la, lo); if (moonW.alt > -2) cand.push({ label: "☾ Moon", az: moonW.az, alt: moonW.alt });
        for (const p of planetPositions(Tw, la, lo)) if (p.alt > -2) cand.push({ label: `${p.sym} ${p.name}`, az: p.az, alt: p.alt });
        for (const [ra, dec, mag, name] of STARS) {
          if (mag > 1.6 || !name) continue;
          const p = raDecToAzEl(ra, dec, Tw, la, lo);
          if (p.alt > -2) cand.push({ label: `★ ${name}`, az: p.az, alt: p.alt, mag });
        }
        if (satDbR && sunPos(Tw, la, lo).alt < -4) {
          for (const s of satsAt(satDbR.sats, Tw, la, lo, 0)) {
            if (s.lit) cand.push({ label: `🛰 ${s.name}`, az: s.az, alt: s.el, stale: s.epochAgeDays });
          }
        }
        for (const c of cand) {
          const sep = Math.acos(Math.min(1, Math.max(-1,
            d[0] * dirFromAzEl(c.az, c.alt)[0] + d[1] * dirFromAzEl(c.az, c.alt)[1] + d[2] * dirFromAzEl(c.az, c.alt)[2]))) * R2D;
          if (sep <= 5) hits.push({ wit: w.name, ...c, sep });
        }
      }
      hits.sort((a, b) => a.sep - b.sep);
      const venusHit = hits.find((h) => h.label.includes("Venus"));
      const satHit = hits.find((h) => h.label.startsWith("🛰"));
      const satStale = satHit && satHit.stale > 5 ? ` (TLE epoch ≈ ${Math.round(satHit.stale)} d from the sighting — position approximate)` : "";
      skyHtml = `<h2>Sky-object check</h2>` + (hits.length
        ? `<table><tr><th>Object</th><th>Witness</th><th>Off sight-line</th><th>At az/el</th></tr>` +
        hits.map((h) => `<tr><td>${e2(h.label)}</td><td>${e2(h.wit)}</td><td>${h.sep.toFixed(1)}°</td><td>${h.az.toFixed(1)}° / ${h.alt.toFixed(1)}°</td></tr>`).join("") +
        `</table>` + (venusHit ? `<p>⚠ <b>Venus sat ${venusHit.sep.toFixed(1)}° from the sight-line</b> — Venus is the single most-reported "UFO"; a stationary, slowly-setting brilliant light is its signature.</p>` : "")
        + (satHit ? `<p>🛰 <b>${e2(satHit.label.slice(2).trim())} was ${satHit.sep.toFixed(1)}° from the sight-line</b> and sunlit${satStale} — a steady point gliding across the sky in minutes is a satellite's signature.</p>` : "")
        : `<p class="cap">No bright planet, star, satellite, Sun or Moon within 5° of any witness sight-line at the sighting time.</p>`);
    }
  }
  const data = JSON.stringify({ phodar: 1, created: new Date().toISOString(), sources: packed, est }, null, 1).replace(/<\//g, "<\\/");
  return `<!doctype html><html><head><meta charset="utf-8"><title>PHODAR sighting report</title><style>
body{font:14px/1.55 -apple-system,"Segoe UI",Roboto,sans-serif;color:#141414;max-width:760px;margin:32px auto;padding:0 18px}
h1{font:800 22px ui-monospace,Menlo,monospace;letter-spacing:.12em}h1 span{color:#C77B14}
h2{font:700 12px ui-monospace,monospace;letter-spacing:.16em;text-transform:uppercase;color:#555;margin-top:26px}
table{border-collapse:collapse;width:100%;font-size:13px;margin:6px 0}td,th{border:1px solid #ccc;padding:6px 8px;text-align:left;vertical-align:top}
.cap{color:#666;font-size:12px}@media print{.noprint{display:none}}
</style></head><body>
<h1>PHO<span>DAR</span> · SIGHTING REPORT</h1>
<div class="cap">Generated ${new Date().toLocaleString()} · photogrammetric detection &amp; ranging · phodar v1</div>
<h2>Observers (${packed.length})</h2>
<table><tr><th>Name</th><th>Position</th><th>Time</th><th>Bearing az/el</th><th>FOV</th><th>Traj pts</th></tr>${obsRows}</table>
<h2>Result</h2>${fixHtml}
${kin ? `<h2>Trajectory kinematics (stereo)</h2>${kin}` : soloKin}
${adsbHtml}
${skyHtml}
${exhibits}
<h2>Method</h2><p>Each photo is pixel-normalized and its lens field of view read from EXIF. The object's sky direction is fixed by aligning the photo on an astronomically anchored alt-azimuth grid (Sun/Moon computed for the reported time and place). With two or more observers, sight-lines are intersected by least squares in a local ENU frame; ray convergence and rms miss distance grade the fix. Object size = measured angular size × range. Trajectories interpolate each witness's directions to common instants before triangulating each instant; speeds, accelerations and felt g-loads follow by finite differences with 3-point smoothing.</p>
<h2>Caveats</h2><p>${fix.ok ? `Quality <b>${fix.rating}</b>: baseline ${fmtLenShort(fix.baseline)}, convergence ${fix.conv.toFixed(1)}°, rms ray miss ${fmtLenShort(fix.solA.rmsMiss)}; a ±1° bearing error implies ≈ ${fmtLenShort(fix.posErr)} of position uncertainty.` : `Single-perspective data — directions and angular sizes are honest; absolute range, size and speed require a second viewpoint.`} Compass bearings may be magnetic rather than true; EXIF times are device-local.</p>
${diagHtml}
<p class="cap"> ${opts.exhibits === "full" || opts.exhibits === "files" ? "Exhibit photos are full resolution; the embedded share data carries 1600 px working copies." : "Bundled photos are 1600 px working copies; analysis used the originals."}</p>
<p class="noprint"><b>Add your perspective:</b> open Phodar → Import and choose this very file — the sighting data and photos are embedded below.</p>
<script type="application/json" id="phodar-data">${data}</script>
</body></html>`;
}

function WizStep({ n, title, children, onBack, onNext, nextLabel, nextDisabled, disabledLabel }) {
  return (
    <div style={{ padding: "14px 12px 96px" }}>
      <div style={{ display: "flex", alignItems: "center", gap: 10, marginBottom: 10 }}>
        <button className="btn sm" onClick={onBack}>‹</button>
        <div>
          <div style={{ fontFamily: "var(--mono)", fontWeight: 800, letterSpacing: ".12em", fontSize: 14 }}>{title}</div>
          <div style={{ display: "flex", gap: 5, marginTop: 4 }}>
            {[1, 2, 3, 4].map((i) => (
              <div key={i} style={{ width: i === n ? 18 : 7, height: 7, borderRadius: 4, background: i <= n ? "var(--amber)" : "var(--line)", transition: "all .2s" }} />
            ))}
          </div>
        </div>
      </div>
      <div className="card" style={{ margin: 0 }}>{children}</div>
      <div style={{ position: "fixed", left: 0, right: 0, bottom: 0, zIndex: 40, padding: "10px 12px calc(12px + env(safe-area-inset-bottom))", background: "linear-gradient(0deg, rgba(7,11,20,.96) 60%, rgba(7,11,20,0))" }}>
        <div style={{ maxWidth: 520, margin: "0 auto" }}>
          <button className="btn amber" disabled={nextDisabled} style={{ width: "100%", padding: 13, opacity: nextDisabled ? 0.45 : 1 }} onClick={nextDisabled ? undefined : onNext}>
            {nextDisabled ? (disabledLabel || "Complete this step to continue") : (nextLabel || "Next →")}
          </button>
        </div>
      </div>
    </div>
  );
}

function WizHome({ sources, est, onNew, onAddWitness, onResume, onRemove, onImport, onReport, unitsImp, onToggleUnits }) {
  const fileRef = useRef(null);
  const [impMsg, setImpMsg] = useState("");
  const real = sources.filter((s) => !isEmptySource(s));
  const fix = analyze(sources);
  const dot = (on, k) => <span key={k} style={{ display: "inline-block", width: 7, height: 7, borderRadius: 4, background: on ? "var(--teal)" : "var(--line)", marginRight: 4 }} />;
  return (
    <div style={{ padding: "26px 14px 40px" }}>
      <div style={{ textAlign: "center", marginTop: 16 }}>
        <div style={{ fontFamily: "var(--mono)", fontWeight: 800, fontSize: 34, letterSpacing: ".16em" }}>PHO<span style={{ color: "var(--amber)" }}>DAR</span></div>
        <div className="microlabel" style={{ marginTop: 4 }}>Photogrammetric detection &amp; ranging</div>
        <div style={{ color: "var(--dim)", fontSize: 12, marginTop: 10, lineHeight: 1.5 }}>
          Turn a sighting photo into real numbers — direction, size, altitude, speed.
          Two witnesses make it true triangulation.
        </div>
        <button className="chip" style={{ marginTop: 8 }} onClick={onToggleUnits}>
          units: <b style={{ color: "var(--amber)" }}>{unitsImp ? "ft · mi · mph" : "m · km · m/s"}</b> — tap to switch
        </button>
      </div>
      <button className="btn amber" style={{ width: "100%", padding: 16, fontSize: 15, marginTop: 22 }} onClick={onNew}>📸 New sighting</button>
      <button className="btn" style={{ width: "100%", padding: 12, marginTop: 8 }} onClick={() => fileRef.current?.click()}>📥 Import a shared sighting</button>
      {real.length > 0 && (
        <button className="btn sm" style={{ width: "100%", marginTop: 6 }} onClick={async () => {
          const lite = JSON.stringify({
            phodar: 1, created: new Date().toISOString(),
            sources: sources.filter((s) => !isEmptySource(s)).map(({ mediaUrl, mediaKind, mediaNorm, open, ...r }) => r),
            est,
          }, null, 1);
          try { await navigator.clipboard.writeText(lite); setImpMsg("✓ session data copied — paste it somewhere safe (preview storage can reset between versions)"); }
          catch (e) { setImpMsg("clipboard blocked — use Report → 🪶 Data only"); }
        }}>⬆ Backup session to clipboard</button>
      )}
      <input ref={fileRef} type="file" accept=".json,.html,application/json,text/html" style={{ display: "none" }}
        onChange={(e) => {
          const f = e.target.files?.[0]; if (!f) return;
          f.text().then((tx) => {
            const n = onImport(tx);
            setImpMsg(n ? `✓ imported ${n} observer${n > 1 ? "s" : ""}` : "Couldn't read that — expected a .phodar.json or a Phodar report.");
          });
          e.target.value = "";
        }} />
      {impMsg && <div style={{ fontSize: 12, color: impMsg.startsWith("✓") ? "var(--teal)" : "var(--red)", marginTop: 6, textAlign: "center" }}>{impMsg}</div>}
      {real.length > 0 && (
        <div className="card" style={{ margin: "18px 0 0" }}>
          <ML>This sighting — {real.length} observer{real.length > 1 ? "s" : ""}</ML>
          {sources.map((s, i) => (
            <div key={s.id} style={{ display: "flex", alignItems: "center", gap: 8, padding: "7px 0", borderBottom: "1px solid var(--line)" }}>
              <div style={{ flex: 1, fontSize: 13 }}>
                {s.name || `Observer ${i + 1}`}
                <div style={{ marginTop: 3 }}>
                  {dot(!!s.mediaUrl, "m")}{dot(isNum(s.lat), "p")}{dot(isNum(s.A?.az), "d")}{dot((s.track || []).length > 1, "t")}
                </div>
              </div>
              <button className="btn sm" onClick={() => onResume(s.id)}>Open ▸</button>
              <button className="btn sm ghost" style={{ color: "var(--red)", padding: "6px 8px" }}
                onClick={() => {
                  if (window.confirm(`Remove ${s.name || `Observer ${i + 1}`} from this sighting? Their photo and measurements go with them.`)) onRemove(s.id);
                }}>✕</button>
            </div>
          ))}
          {fix.ok && (
            <div style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--teal)", marginTop: 8 }}>
              FIX: {fmtLenShort(fix.solA.X[2])} up{fix.sizeAvg != null ? ` · ${fmtLenShort(fix.sizeAvg)} across` : ""} · {fix.rating}
            </div>
          )}
          <button className="btn amber" style={{ width: "100%", marginTop: 10 }} onClick={onAddWitness}>➕ Add a witness / perspective</button>
          <button className="btn teal" style={{ width: "100%", marginTop: 8 }} onClick={onReport}>📄 Report</button>
        </div>
      )}
    </div>
  );
}

function WizFinish({ sources, est, onAdd, onReport, onShare, onHome }) {
  const fix = analyze(sources);
  const tr = analyzeTracks(sources);
  return (
    <div style={{ padding: "14px 12px 40px" }}>
      <div style={{ display: "flex", alignItems: "center", gap: 10, marginBottom: 10 }}>
        <button className="btn sm" onClick={onHome}>‹</button>
        <div style={{ fontFamily: "var(--mono)", fontWeight: 800, letterSpacing: ".12em", fontSize: 14 }}>SIGHTING CAPTURED</div>
      </div>
      <div className="card" style={{ margin: 0 }}>
        {fix.ok ? (
          <>
            <ML>Triangulated fix — {fix.obs.length} observers</ML>
            <div style={{ fontFamily: "var(--mono)", fontSize: 13, lineHeight: 1.9 }}>
              altitude <b style={{ color: "var(--teal)" }}>{fmtLenShort(fix.solA.X[2])}</b> ({Math.round(fix.solA.X[2] * 3.28084)} ft)<br />
              {fix.sizeAvg != null && <>size <b style={{ color: "var(--teal)" }}>{fmtLenShort(fix.sizeAvg)}</b> ({Math.round(fix.sizeAvg * 3.28084)} ft)<br /></>}
              {fix.motion?.speed != null && <>speed <b style={{ color: "var(--teal)" }}>{Math.round(fix.motion.speed)} m/s</b> ({Math.round(fix.motion.speed * 2.23694)} mph)<br /></>}
              quality <b>{fix.rating}</b> · baseline {fmtLenShort(fix.baseline)} · conv {fix.conv.toFixed(1)}°
            </div>
          </>
        ) : (
          <>
            <ML>One perspective so far</ML>
            <div style={{ fontSize: 12, color: "var(--dim)", lineHeight: 1.6 }}>
              A single viewpoint gives honest angular data — direction, angular size, angular motion — but can't pin absolute distance.
              <b style={{ color: "var(--ink)" }}> Add a second perspective</b> (another witness, or your own shot from a different spot) and Phodar triangulates real altitude, size and speed.
            </div>
            {tr.solo?.length > 0 && (
              <div style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--track)", marginTop: 8 }}>
                trajectory: {tr.solo[0].k.n} pts · {tr.solo[0].k.dur.toFixed(1)} s · peak {(tr.solo[0].k.peakSpeed * R2D).toFixed(1)}°/s
              </div>
            )}
          </>
        )}
        {(() => {
          const arb = arbitrateBearings(sources);
          return arb?.best ? (
            <div className="warn" style={{ marginTop: 8 }}>
              ⚠ The bearings don't truly converge{fix.ok ? " (the fix above crosses behind an observer — treat it as unreliable)" : ""} — <b>{arb.best.otherName}</b>'s compass looks ≈ <b>{Math.round(arb.best.err)}° off</b> (should read ≈ {arb.best.azOtherTrue.toFixed(1)}° if {arb.best.trustName} is right; object then ≈ {fmtLenShort(arb.best.range)} away, ≈ {fmtLenShort(arb.best.size)} across). Open that photo's sky view and calibrate against the Sun or a landmark.
            </div>
          ) : null;
        })()}
        {(() => {
          if (!fix.ok) return null;
          const alts = sources.filter((s) => isNum(s.lat) && isNum(s.A?.az)).map((s) => (isNum(s.alt) ? +s.alt : 0));
          const spread = alts.length > 1 ? Math.max(...alts) - Math.min(...alts) : 0;
          return spread > 3 && spread > fix.baseline * 0.35 ? (
            <div className="warn" style={{ marginTop: 8 }}>
              ⚠ GPS altitudes differ {spread.toFixed(1)} m over a {fmtLenShort(fix.baseline)} baseline — phone altitude wobble; if you stood on level ground, set both elevations equal for a tighter fix.
            </div>
          ) : null;
        })()}
      </div>
      <div style={{ display: "grid", gap: 8, marginTop: 14 }}>
        <button className="btn amber" style={{ padding: 13 }} onClick={onAdd}>➕ Add another perspective</button>
        <button className="btn teal" style={{ padding: 13 }} onClick={onReport}>📄 Generate report</button>
        <button className="btn" style={{ padding: 13 }} onClick={onShare}>💾 Share file (.phodar.json)</button>
      </div>
      {/* full results: top-down plot, per-observer table, motion, quality, trajectory, ADS-B */}
      <div style={{ margin: "6px -12px 0" }}>
        <ResultsPanel sources={sources} />
      </div>
    </div>
  );
}

function ReportView({ sources, est, onBack }) {
  const [msg, setMsg] = useState("");
  const [prevHtml, setPrevHtml] = useState(null);   // iframe preview
  const [manual, setManual] = useState(null);       // { name, text } fallback copy box
  const fix = analyze(sources);
  const copyText = async (txt) => {
    try { await navigator.clipboard.writeText(txt); return true; } catch (e) { }
    try {
      const ta = document.createElement("textarea");
      ta.value = txt; ta.style.position = "fixed"; ta.style.left = "-999px";
      document.body.appendChild(ta); ta.focus(); ta.select();
      const ok = document.execCommand("copy"); ta.remove(); return ok;
    } catch (e) { return false; }
  };
  const deliver = async (name, text, mime) => {
    const dl = download(name, text, mime);       // works when deployed; silently blocked in this preview
    const cp = await copyText(text);
    if (cp) setMsg(`✓ ${name} copied to clipboard${dl ? " (download also attempted)" : ""} — paste into Notes or a file`);
    else if (dl) setMsg(`✓ downloading ${name}`);
    else { setMsg(""); setManual({ name, text }); }
  };
  return (
    <div style={{ padding: "14px 12px 40px" }}>
      <div style={{ display: "flex", alignItems: "center", gap: 10, marginBottom: 10 }}>
        <button className="btn sm" onClick={onBack}>‹</button>
        <div style={{ fontFamily: "var(--mono)", fontWeight: 800, letterSpacing: ".12em", fontSize: 14 }}>REPORT &amp; SHARE</div>
      </div>
      <div className="card" style={{ margin: 0 }}>
        <ML>Sighting report</ML>
        <div style={{ fontSize: 12, color: "var(--dim)", lineHeight: 1.6 }}>
          White-paper style: observers, the triangulated result, kinematics, methodology, caveats. It prints to PDF from any browser, and the sighting data is <b style={{ color: "var(--ink)" }}>embedded inside it</b> — anyone with Phodar can import the report itself to add their perspective. The .phodar.json is the bare data.
        </div>
        {fix.ok ? (
          <div style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--teal)", marginTop: 8 }}>
            includes fix: {fmtLenShort(fix.solA.X[2])} up · quality {fix.rating}
          </div>
        ) : (
          <div style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--amber)", marginTop: 8 }}>
            single-perspective report — invites a second witness to complete the triangulation
          </div>
        )}
        <div style={{ display: "grid", gap: 8, marginTop: 12 }}>
          <button className="btn amber" style={{ padding: 12 }} onClick={async () => { setMsg("packing…"); setPrevHtml(await reportHtml(sources, est, { exhibits: "full" })); setMsg(""); }}>👁 Preview the report (full-res)</button>
          <button className="btn teal" style={{ padding: 12 }} onClick={async () => { setMsg("packing…"); deliver("phodar-report.html", await reportHtml(sources, est), "text/html"); }}>📄 Report → clipboard / download</button>
          <button className="btn" style={{ padding: 12 }} onClick={async () => { setMsg("packing…"); deliver("sighting.phodar.json", await buildShareJson(sources, est), "application/json"); }}>💾 Share file → clipboard / download</button>
          <button className="btn" style={{ padding: 12 }} onClick={async () => {
            setMsg("building zip…");
            const html = await reportHtml(sources, est, { exhibits: "files" });
            const json = await buildShareJson(sources, est);
            const act = sources.filter((s) => !isEmptySource(s));
            const files = [
              { name: strU8("report.html"), data: strU8(html) },
              { name: strU8("sighting.phodar.json"), data: strU8(json) },
            ];
            act.forEach((s, i) => {
              if (s.mediaUrl && s.mediaKind === "image") {
                try { files.push({ name: strU8(`photos/observer-${i + 1}.jpg`), data: dataUrlU8(s.mediaUrl) }); } catch (e) { }
              }
            });
            const blob = makeZip(files);
            if (download("phodar-sighting.zip", blob, "application/zip"))
              setMsg(`✓ downloading zip — ${(blob.size / 1048576).toFixed(1)} MB, full-resolution photos inside`);
            else setMsg("Zip downloads need the deployed app — this preview can't save binaries. Preview / Copy work here.");
          }}>🗜 Bundle (.zip — report + full-res photos + data)</button>
          <button className="btn sm" onClick={() => {
            const lite = JSON.stringify({
              phodar: 1, created: new Date().toISOString(),
              sources: sources.filter((s) => !isEmptySource(s)).map(({ mediaUrl, mediaKind, mediaNorm, open, ...r }) => r),
              est,
            }, null, 1);
            deliver("sighting-data.phodar.json", lite, "application/json");
          }}>🪶 Data only, no photos (~10 KB — pastes anywhere)</button>
        </div>
        {msg && <div style={{ fontSize: 12, color: "var(--teal)", marginTop: 8 }}>{msg}</div>}
        <div style={{ fontSize: 11, color: "var(--dim)", marginTop: 8 }}>
          Real file downloads arrive with the deployed (Railway) build — this preview falls back to your clipboard.
        </div>
      </div>

      {prevHtml && (
        <div style={{ position: "fixed", inset: 0, zIndex: 300, background: "#0009", display: "flex", flexDirection: "column" }}>
          <div style={{ display: "flex", gap: 8, padding: "10px 12px", background: "var(--bg)", alignItems: "center" }}>
            <span style={{ fontFamily: "var(--mono)", fontSize: 12, fontWeight: 800, flex: 1 }}>REPORT PREVIEW</span>
            <button className="btn sm teal" onClick={async () => { const ok = await copyText(prevHtml); setMsg(ok ? "✓ report HTML copied" : ""); if (!ok) setManual({ name: "phodar-report.html", text: prevHtml }); setPrevHtml(null); }}>⧉ Copy HTML</button>
            <button className="btn sm" onClick={() => setPrevHtml(null)}>✕ Close</button>
          </div>
          <iframe title="report" srcDoc={prevHtml} style={{ flex: 1, border: 0, background: "#fff" }} />
        </div>
      )}

      {manual && (
        <div style={{ position: "fixed", inset: 0, zIndex: 300, background: "#000c", display: "flex", flexDirection: "column", padding: 12 }}>
          <div style={{ display: "flex", gap: 8, alignItems: "center", marginBottom: 8 }}>
            <span style={{ fontFamily: "var(--mono)", fontSize: 12, fontWeight: 800, flex: 1, color: "var(--ink)" }}>{manual.name} — select all, then Copy</span>
            <button className="btn sm" onClick={() => setManual(null)}>✕ Close</button>
          </div>
          <textarea readOnly value={manual.text} onFocus={(e) => e.target.select()}
            style={{ flex: 1, width: "100%", background: "#0B1424", color: "var(--ink)", border: "1px solid var(--line)", borderRadius: 10, padding: 10, fontFamily: "var(--mono)", fontSize: 10 }} />
        </div>
      )}
    </div>
  );
}

export default function App() {
  const [sources, setSources] = useState(() => [makeSource(1)]);
  const [est, setEst] = useState({ size: "", dist: "", speed: "" });
  const [ui, setUi] = useState({ view: "home", srcId: null });
  const [unitsImp, setUnitsImp] = useState(() => {
    try { return localStorage.getItem("phodar-units") === "imp"; } catch (e) { return false; }
  });
  setImperialUnits(unitsImp); // module state — every formatter call this render follows it
  const toggleUnits = () => {
    setUnitsImp((v) => {
      try { localStorage.setItem("phodar-units", !v ? "imp" : "met"); } catch (e) { }
      return !v;
    });
  };
  const loadedRef = useRef(false);

  /* restore session (migrates pre-rename SkyFix sessions transparently) */
  useEffect(() => {
    (async () => {
      let d = null;
      for (const key of ["phodar-v1", "skyfix-v1"]) {
        try {
          const r = await window.storage.get(key);
          if (r?.value) { d = JSON.parse(r.value); break; }
        } catch (e) { /* key absent */ }
      }
      if (d) {
        if (d.sources?.length) {
          setSources(d.sources);
          /* re-attach media from IndexedDB (autosave strips mediaUrl) */
          Promise.all(d.sources.map(async (s) => ({ id: s.id, rec: await mediaGet(s.id) }))).then((rs) => {
            setSources((ss) => ss.map((s) => {
              const hit = rs.find((r) => r.id === s.id)?.rec;
              if (!hit || s.mediaUrl) return s;
              const url = hit.kind === "video" ? URL.createObjectURL(hit.data) : hit.data;
              return { ...s, mediaUrl: url, mediaKind: hit.kind, mediaNorm: hit.kind === "image" };
            }));
          }).catch(() => { });
        }
        if (d.est) setEst(d.est);
      }
      loadedRef.current = true;
    })();
  }, []);

  /* autosave (media object URLs can't persist; points/dims do) */
  useEffect(() => {
    if (!loadedRef.current) return;
    const id = setTimeout(() => {
      try { window.storage.set("phodar-v1", JSON.stringify({ sources: sources.map(({ mediaUrl, mediaKind, mediaNorm, ...rest }) => rest), est })); } catch (e) { }
    }, 800);
    return () => clearTimeout(id);
  }, [sources, est]);

  const updateSource = (id, patch) =>
    setSources((ss) => ss.map((s) => (s.id === id ? { ...s, ...patch } : s)));
  const removeSource = (id) => { mediaDel(id); setSources((ss) => ss.filter((s) => s.id !== id)); };
  /* a SIGHTING is the event; each witness/perspective is a source within it */
  const addWitness = () => {
    const blank = sources.find(isEmptySource);
    if (blank) { setUi({ view: "s1", srcId: blank.id }); return; }
    const ns = makeSource(sources.length + 1);
    setSources((ss) => [...ss.map((s) => ({ ...s, open: false })), ns]);
    setUi({ view: "s1", srcId: ns.id });
  };
  const newSighting = () => {
    const real = sources.filter((s) => !isEmptySource(s));
    if (real.length && !window.confirm(
      `Start a NEW sighting? The current one (${real.length} observer${real.length > 1 ? "s" : ""}) will be cleared.\n\nExport a report or backup first if you want to keep it.`)) return;
    const ns = makeSource(1);
    mediaClear();
    setSources([ns]);
    setEst({ size: "", dist: "", speed: "" });
    setUi({ view: "s1", srcId: ns.id });
  };
  /* ——— guided wizard shell — the one and only workflow ——— */
  const goView = (view) => setUi((u) => ({ ...u, view }));
  const shareJsonNow = async () => {
    const j = await buildShareJson(sources, est);
    const dl = download("sighting.phodar.json", j, "application/json");
    let cp = false;
    try { await navigator.clipboard.writeText(j); cp = true; } catch (e) { }
    alert(cp ? "Share file copied to clipboard ✓" + (dl ? " (download also attempted)" : "") : dl ? "Download started" : "Open the Report page to copy the share file.");
  };
  const importShared = (text) => {
    try {
      let str = text;
      const m = text.match(/<script[^>]*id="phodar-data"[^>]*>([\s\S]*?)<\/script>/);
      if (m) str = m[1];
      const d = JSON.parse(str);
      if (!d || !Array.isArray(d.sources) || !d.sources.length) return 0;
      const base = sources.length;
      const merged = d.sources.map((s, i) => {
        const { mediaJpeg, ...rest } = s;
        return {
          ...makeSource(base + i + 1), ...rest,
          id: "s" + Math.random().toString(36).slice(2, 9),
          open: false,
          mediaUrl: mediaJpeg || null,
          mediaKind: mediaJpeg ? "image" : null,
          mediaNorm: !!mediaJpeg,
        };
      });
      setSources((ss) => [...ss, ...merged]);
      merged.forEach((s) => { if (s.mediaUrl) mediaPut(s.id, { kind: "image", data: s.mediaUrl }); });
      return merged.length;
    } catch (e) { return 0; }
  };
    const wsrc = sources.find((s) => s.id === ui.srcId) || null;
    let page = null;
    if (ui.view === "report") {
      page = <ReportView sources={sources} est={est} onBack={() => goView("home")} />;
    } else if (ui.view === "s4") {
      page = <WizFinish sources={sources} est={est} onAdd={addWitness} onReport={() => goView("report")} onShare={shareJsonNow} onHome={() => goView("home")} />;
    } else if (ui.view !== "home" && wsrc) {
      if (ui.view === "s1") {
        page = (
          <WizStep n={1} title="THE PHOTO" onBack={() => goView("home")} onNext={() => goView("s2")}
            nextLabel={wsrc.mediaUrl ? "Next · where were you? →" : "Skip media — enter data by hand →"}>
            <MediaMeasure wizard src={wsrc} update={(p) => updateSource(wsrc.id, p)} />
          </WizStep>
        );
      } else if (ui.view === "s2") {
        page = (
          <WizStep n={2} title="YOUR POSITION" onBack={() => goView("s1")} onNext={() => goView("s3")}
            nextDisabled={!(isNum(wsrc.lat) && isNum(wsrc.lon))} nextLabel="Next · place it in the sky →"
            disabledLabel="Enter where you stood — GPS, paste coords, or type — to continue">
            <PositionEditor src={wsrc} update={(p) => updateSource(wsrc.id, p)}
              others={sources.filter((x) => x.id !== wsrc.id && isNum(x.lat) && isNum(x.lon)).map((x) => ({ lat: +x.lat, lon: +x.lon, name: x.name }))} />
          </WizStep>
        );
      } else if (ui.view === "s3") {
        page = (
          <SkyAimer open wizard source={wsrc} update={(p) => updateSource(wsrc.id, p)}
            onClose={() => goView("s4")} onWizardBack={() => goView("s2")} onWizardNext={() => goView("s4")}
            lat={isNum(wsrc.lat) ? +wsrc.lat : 42.16} lng={isNum(wsrc.lon) ? +wsrc.lon : -123.66}
            whenMs={isNum(wsrc.whenMs) ? +wsrc.whenMs : Date.now()}
            initAz={isNum(wsrc.A?.az) ? +wsrc.A.az : (wsrc.mediaAim && isNum(wsrc.mediaAim.az) ? +wsrc.mediaAim.az : 180)}
            initAlt={isNum(wsrc.A?.el) ? +wsrc.A.el : 20}
            marks={[]} which="A"
            onCapture={(wh, az, el) => {
              if (wh === "A") updateSource(wsrc.id, { A: { ...wsrc.A, az: az.toFixed(2), el: el.toFixed(2) } });
              else updateSource(wsrc.id, { B: { ...wsrc.B, az: az.toFixed(2), el: el.toFixed(2) } });
            }} />
        );
      }
    }
    if (!page) page = <WizHome sources={sources} est={est} onNew={newSighting} onAddWitness={addWitness} onResume={(id) => setUi({ view: "s1", srcId: id })} onRemove={removeSource} onImport={importShared} onReport={() => goView("report")} unitsImp={unitsImp} onToggleUnits={toggleUnits} />;
    return (
      <div className="phodar" style={{ maxWidth: 520, margin: "0 auto", minHeight: "100vh" }}>
        <style>{css}</style>
        {page}
        <div className="rotate-lock">
          <div className="ic">📱</div>
          <div style={{ fontWeight: 800, letterSpacing: ".1em" }}>ROTATE TO PORTRAIT</div>
          <div style={{ fontSize: 12, color: "var(--dim)" }}>Phodar is built for an upright phone.</div>
        </div>
      </div>
    );
}
