import React, { useState, useRef, useEffect, useMemo, useCallback } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import { D2R, R2D, RAD, clampN, dot, sub, add, scl, unit, geoFromEnu, dirFromAzEl, dirToAzEl } from "./math/geodesy.js";
import { isNum, n1, fmtLenShort, fmtSpeed, fmtDeg, compass8, setImperialUnits } from "./math/format.js";
import { photoBasis, angSizeFromPoints, pixelDirFromAnchor, dirToPixK, solvePoseAnchors } from "./math/projection.js";
import { analyze, arbitrateBearings, aspectSpan } from "./math/triangulate.js";
import { trackDirections, kinematics, analyzeTracks } from "./math/kinematics.js";
import { sunPos, moonPos, moonFrac, raDecToAzEl } from "./math/astro.js";
import { fetchAircraft, fetchAircraftAt, fetchAcInfo, rankCandidates, radiusNmForSources, acAzElRange } from "./checks/adsb.js";
import { declination } from "./math/geomag.js";
import { loadSats, loadSatGroup, satsAt, satTrail } from "./checks/satellites.js";
import { fetchWindAt, balloonVerdict } from "./checks/winds.js";
import { fetchLaunches } from "./checks/launches.js";
import { fetchFireballs } from "./checks/fireballs.js";
import { predictedSkyline, skylineElAt, demElevation, detectSkyline, matchSkyline, TERRAIN_ATTRIB } from "./terrain.js";
import { fetchPeaks } from "./checks/peaks.js";
import { detectStars, autoStarAlign, blindStarAlign } from "./checks/platesolve.js";
import { mediaPut, mediaGet, mediaDel, mediaClear } from "./mediaStore.js";
import { parseMediaMeta } from "./exif.js";
import { SHAPES, I3, rotX3, rotY3, rotZ3, mul3, SHAPE_R0, shapeProjNat, shapeWire } from "./shapes.js";
import { planetPositions } from "./math/planets.js";
import { STARS } from "./math/starcat.js";
import phodarLogo from "./assets/phodar-logo.svg";
import phodarLogoRaw from "./assets/phodar-logo.svg?raw";

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
  const anyRad = solo.some((s) => s.rad);
  return (
    <Section title="Trajectory — single observer (needs assumed distance)">
      <div style={{ fontSize: 12, color: "var(--dim)", marginBottom: 10 }}>
        {anyRad
          ? <>Because you sized the object at each point, the <b style={{ color: "var(--track)" }}>radial (closer/farther) motion is captured</b> — the 3D path and its speed are real. Only the overall <b>scale</b> is unknown, so pick the distance to point 1 and everything below follows.</>
          : <>One viewpoint gives the angular path only. Every result below scales with the distance you assume — and radial motion is invisible, so speeds and g are lower bounds on the transverse component. (Size the object at each trajectory point to capture closer/farther motion.)</>}
      </div>
      <ML>{anyRad ? "Assumed distance to point 1" : "Assumed distance"}</ML>
      <div className="readout amber" style={{ fontSize: 22 }}>{fmtLenShort(D)}</div>
      <input type="range" min={0} max={1} step={0.001} value={t} onChange={(e) => setT(e.target.value)} />
      <div style={{ display: "flex", justifyContent: "space-between", fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)" }}>
        <span>50 m</span><span>50 km</span>
      </div>
      {solo.map((s, i) => {
        const rad = s.rad;
        const k = rad ? rad.k3d : s.k;
        const g = gAt(k, D);
        const felt = g != null ? Math.sqrt(g * g + 1) : null;
        const near = rad ? D * rad.rho[rad.iNear] : null, far = rad ? D * rad.rho[rad.iFar] : null;
        return (
          <div key={i}>
            <div className="hr" />
            <ML style={{ color: "var(--track)" }}>{s.name} — {k.n} pts · {k.dur.toFixed(1)} s · {rad ? <>3D path (radial + transverse)</> : <>peak {n1(s.k.peakSpeed * R2D)}°/s across the sky</>}</ML>
            {rad && (
              <div style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--ink)", margin: "6px 0 2px" }}>
                range {fmtLenShort(near)} → {fmtLenShort(far)} · <span style={{ color: "var(--track)" }}>{rad.rangeRatio.toFixed(2)}× span</span> (pt {rad.iNear + 1} closest, pt {rad.iFar + 1} farthest){rad.oriented ? <span style={{ color: "var(--dim)" }}> · aspect-corrected</span> : ""}
              </div>
            )}
            <div className="grid2" style={{ marginTop: 6 }}>
              <div>
                <ML>{rad ? "True speed" : "Speed"} at {fmtLenShort(D)}</ML>
                <div className="readout" style={{ fontSize: 22 }}>{n1(D * k.peakSpeed * 2.23694)} mph</div>
                <div className="readsub">avg {n1(D * k.avgSpeed * 2.23694)} mph{rad ? " · incl. radial" : ""}</div>
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
  /* clear the iOS notch / Dynamic Island (installed PWA runs under the status
     bar via black-translucent) and the home indicator + tab bar at the bottom */
  padding-top:env(safe-area-inset-top);
  padding-bottom:calc(84px + env(safe-area-inset-bottom));}
.phodar input,.phodar select{
  background:#0A1122; border:1px solid var(--line); color:var(--ink);
  border-radius:8px; padding:9px 10px; font-size:16px; width:100%;
  font-family:var(--mono); outline:none;}
.phodar input:focus,.phodar select:focus{border-color:var(--amber);}
.phodar input[type=range]{padding:0; height:42px; accent-color:var(--amber); font-size:14px; cursor:pointer;}
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
.pinmapwrap{position:relative; isolation:isolate; z-index:0; height:200px;
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
.pinmap-ray{position:absolute; left:50%; top:50%; z-index:899; overflow:visible;
  pointer-events:none; transform-origin:0 0; filter:drop-shadow(0 1px 2px rgba(0,0,0,.85));}
.pinmap-you{position:absolute; left:50%; top:50%; margin:-26px 0 0 12px;
  z-index:900; pointer-events:none; color:var(--teal); font-family:var(--mono);
  font-size:9px; font-weight:700; text-shadow:0 1px 2px #000;}
.pinmap-hud{position:absolute; left:8px; bottom:6px; z-index:900;
  pointer-events:none; font-family:var(--mono); font-size:10px; color:var(--ink);
  text-shadow:0 1px 2px #000; line-height:1.5;}
.map-north{position:absolute; top:6px; left:8px; z-index:900; pointer-events:none;
  font-family:var(--mono); font-size:11px; font-weight:800; color:#fff;
  background:rgba(7,11,20,.6); border:1px solid rgba(255,255,255,.28);
  border-radius:6px; padding:2px 6px; text-shadow:0 1px 2px #000; letter-spacing:.06em;}
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
/* portrait lock — phones only. The trigger is the DEVICE orientation
   (html.dev-landscape, set from screen.orientation in JS), NOT a CSS
   orientation query: the on-screen keyboard squishes the viewport wider
   than tall, which made the old media query fire while typing
   coordinates in step 2 and lock the user out. Plus coarse pointer +
   small screen so tablets and desktops never match. */
.rotate-lock{display:none;}
@media screen and (pointer: coarse) and (max-height: 520px){
  html.dev-landscape .rotate-lock{display:flex; position:fixed; inset:0; z-index:9999;
    background:var(--bg); color:var(--ink); flex-direction:column;
    align-items:center; justify-content:center; gap:14px; text-align:center;
    font-family:var(--mono); padding:20px;}
  .rotate-lock .ic{font-size:44px; animation:rot-nudge 1.6s ease-in-out infinite;}
  @keyframes rot-nudge{0%,100%{transform:rotate(0)}50%{transform:rotate(-90deg)}}
}
`;

/* Non-destructive brightness/contrast for DISPLAY only. Values are percentages
   (100 = neutral) stored on the source as `imgAdj`; the ORIGINAL pixels are
   never modified (measurement — star detection, plate solve, marks — always
   reads the raw image). The pixel pass replicates the CSS `brightness()
   contrast()` math EXACTLY, so canvas-baked surfaces (the sky-view warp texture,
   report crops) match CSS-filtered <img> surfaces (measure step, place mode). */
const imgAdjNeutral = (a) => !a || ((a.bri == null || a.bri === 100) && (a.con == null || a.con === 100));
const imgAdjFilter = (a) => imgAdjNeutral(a) ? "none" : `brightness(${(a.bri ?? 100) / 100}) contrast(${(a.con ?? 100) / 100})`;
function applyImgAdj(ctx, w, h, a) {
  if (imgAdjNeutral(a)) return;
  const b = (a.bri ?? 100) / 100, c = (a.con ?? 100) / 100;
  const id = ctx.getImageData(0, 0, w, h), d = id.data;
  for (let i = 0; i < d.length; i += 4) {
    for (let k = 0; k < 3; k++) {
      let v = (d[i + k] * b - 127.5) * c + 127.5; // brightness then contrast, matching CSS filter order
      d[i + k] = v < 0 ? 0 : v > 255 ? 255 : v;
    }
  }
  ctx.putImageData(id, 0, 0);
}

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
  const twistRef = useRef(null); // second finger anchors a view-axis twist (roll) once rotation is underway
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
        /* valid pixels but no GPS/time/bearing: the file was re-encoded and
           stripped in sharing (messaging apps, "All Photos Data" off). Say so
           instead of failing silently — otherwise it reads as a load bug. */
        else update({ meta: { stripped: true } });
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

  /* Force the first video frame to render. A muted, unplayed <video> shows
     blank on iOS Safari (and often desktop) until a frame is decoded — the
     bug where the clip appeared only after leaving the step and coming back
     (a remount over a now-buffered file). A hair of seek triggers the paint.
     Guarded to fire once, near t=0, so it never fights the scrubber. */
  const paintFirstFrame = () => {
    const el = mediaRef.current;
    if (!el || media?.kind !== "video") return;
    /* restore the frame the object was marked on when revisiting this step —
       otherwise it reloads at the start and loses the mark. Falls back to a
       hair past 0 so iOS still decodes a frame instead of showing blank.
       Only fires on (re)load events, so it never fights the scrubber. */
    const marked = isNum(src?.A?.videoTime) ? +src.A.videoTime : 0;
    const target = marked > 0.01 ? marked : Math.min(0.04, (el.duration || 1) / 4);
    try { if (Math.abs(el.currentTime - target) > 0.02) { el.currentTime = target; setVidT(target); } } catch (e) { /* seek not ready yet — onLoadedData retries */ }
  };
  const onLoaded = () => {
    const el = mediaRef.current;
    if (!el) return;
    setView({ z: 1, ox: 0, oy: 0 });
    if (media.kind === "video") {
      update({ natW: el.videoWidth, natH: el.videoHeight });
      setVidDur(el.duration || 0);
      paintFirstFrame(); // iOS Safari leaves a <video> blank until it decodes a frame
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
        rotRef.current = { R0: src.shapeFit.rotM || I3, sx: pd.sx, sy: pd.sy, pid: pd.id }; // drag = rotate in 3D
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
      /* Second finger, and a shape rotation is ALREADY underway → don't pinch:
         use this finger as an anchor and let the twist between the two fingers
         roll the shape about the view axis (fine control the 1-finger drag
         can't give). Only when the first finger already started rotating —
         otherwise a second finger is a pinch, as before. */
      if (drag && active === "shape" && rotRef.current && !hDragRef.current && src.shapeFit) {
        killPending();
        const anchorId = e.pointerId;
        const driverId = rotRef.current.pid;
        const anc = ptsRef.current.get(anchorId);
        const drv = ptsRef.current.get(driverId) || anc;
        twistRef.current = {
          anchorId, driverId,
          a0: Math.atan2(drv.y - anc.y, drv.x - anc.x),
          R0: rotRef.current.cur || src.shapeFit.rotM || I3, // continue from the live orientation, no jump
        };
        return;
      }
      /* second finger: a pinch — discard any undecided touch, place nothing */
      killPending();
      setDrag(false); setFinger(null);
      twistRef.current = null;
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
    if (twistRef.current && ptsRef.current.size >= 2 && src.shapeFit) {
      const tw = twistRef.current;
      const anc = ptsRef.current.get(tw.anchorId), drv = ptsRef.current.get(tw.driverId);
      if (anc && drv) {
        const a1 = Math.atan2(drv.y - anc.y, drv.x - anc.x);
        const nsf = { ...src.shapeFit, rotM: mul3(rotZ3((a1 - tw.a0) * R2D), tw.R0) }; // roll about the view axis
        tw.cur = nsf.rotM; // live orientation, so lifting back to one finger continues from here
        syncShape(nsf); shapeLoupeFor(nsf);
      }
      return;
    }
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
        rr.cur = nsf.rotM; // live orientation, so a second-finger twist starts from here
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
    if (twistRef.current && ptsRef.current.size < 2) {
      /* lifting out of a two-finger twist: hand control back to whichever
         finger remains, seeded at the post-twist orientation so it doesn't jump */
      const cur = twistRef.current.cur || twistRef.current.R0;
      twistRef.current = null;
      const rem = [...ptsRef.current.entries()][0];
      if (rem && active === "shape" && src.shapeFit && drag)
        rotRef.current = { R0: cur, sx: rem[1].x, sy: rem[1].y, pid: rem[0] };
    }
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
      ptsRef.current.clear(); pinchRef.current = null; twistRef.current = null;
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
      applyImgAdj(ctx, cv.width, cv.height, src.imgAdj); // match the on-screen brightness/contrast
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
        <label className="btn sm teal" style={{ display: "inline-block" }} title="Open the camera and shoot right now — freshest possible EXIF (GPS, time, bearing)">
          📷 Shoot now
          <input type="file" accept="image/*" capture="environment" onChange={onFile} style={{ display: "none" }} />
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
          {!src.shapeFit && (
            <div style={{ marginTop: 5, fontSize: 11.5, color: "var(--dim)", fontStyle: "italic", lineHeight: 1.4 }}>
              Not sure of the shape? Choose ● Orb — it assumes no particular form and still measures the object's size.
            </div>
          )}
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
              {(src.shapeFit.kind === "tri" || src.shapeFit.kind === "plane" || src.shapeFit.kind === "bird" || src.shapeFit.kind === "drone") && (
                <div style={{ display: "flex", gap: 8, alignItems: "center", marginTop: 4 }}>
                  <span className="microlabel" style={{ marginBottom: 0 }}>spin</span>
                  <input type="range" min={-180} max={180} step={1} value={src.shapeFit.roll || 0}
                    onChange={(e) => { const nsf = { ...src.shapeFit, roll: +e.target.value }; syncShape(nsf); shapeLoupeFor(nsf); }} style={{ flex: 1 }} />
                </div>
              )}
              {src.shapeFit.kind === "bird" && (
                <>
                  <div style={{ display: "flex", gap: 8, alignItems: "center", marginTop: 4 }}>
                    <span className="microlabel" style={{ marginBottom: 0, minWidth: 88 }}>wingspan {(src.shapeFit.wing ?? 1).toFixed(2)}×</span>
                    <input type="range" min={0.5} max={1.8} step={0.02} value={src.shapeFit.wing ?? 1}
                      onChange={(e) => { const nsf = { ...src.shapeFit, wing: +e.target.value }; syncShape(nsf); shapeLoupeFor(nsf); }} style={{ flex: 1 }} />
                  </div>
                  <div style={{ display: "flex", gap: 8, alignItems: "center", marginTop: 4 }}>
                    <span className="microlabel" style={{ marginBottom: 0, minWidth: 88 }}>wing pos {(src.shapeFit.wingX ?? 0) >= 0 ? "+" : ""}{(src.shapeFit.wingX ?? 0).toFixed(2)}</span>
                    <input type="range" min={-0.15} max={0.15} step={0.005} value={src.shapeFit.wingX ?? 0}
                      onChange={(e) => { const nsf = { ...src.shapeFit, wingX: +e.target.value }; syncShape(nsf); shapeLoupeFor(nsf); }} style={{ flex: 1 }} />
                  </div>
                </>
              )}
              {src.shapeFit.kind === "jelly" && (
                <div style={{ display: "flex", gap: 8, alignItems: "center", marginTop: 4 }}>
                  <span className="microlabel" style={{ marginBottom: 0, minWidth: 88 }}>tendrils {(src.shapeFit.tent ?? 1).toFixed(2)}×</span>
                  <input type="range" min={0.3} max={2.2} step={0.02} value={src.shapeFit.tent ?? 1}
                    onChange={(e) => { const nsf = { ...src.shapeFit, tent: +e.target.value }; syncShape(nsf); shapeLoupeFor(nsf); }} style={{ flex: 1 }} />
                </div>
              )}
              {(() => {
                const pr = shapeProjNat(src.shapeFit);
                const aM = angSizeFromPoints(pr.p1, pr.p2, natW, natH, +src.fovH);
                const fpx = natW && isNum(src.fovH) ? (natW / 2) / Math.tan((+src.fovH * D2R) / 2) : null;
                const aN = fpx ? (pr.minorNat / fpx) * R2D : null;
                return aM != null ? (
                  <div style={{ fontFamily: "var(--mono)", fontSize: 11, color: "var(--amber)", marginTop: 4 }}>
                    projected {aM.toFixed(3)}°{aN != null ? ` × ${aN.toFixed(3)}° · aspect ${(aM / Math.max(aN, 1e-6)).toFixed(1)}:1` : ""} — drag to rotate in 3D · add a second finger to twist (roll) · tap to move it
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
                    onLoadedMetadata={onLoaded} onLoadedData={paintFirstFrame} onError={onMediaError} onTimeUpdate={(e) => setVidT(e.target.currentTime)}
                    style={{ width: "100%", display: "block", pointerEvents: "none", filter: imgAdjFilter(src.imgAdj) }} />
                ) : (
                  <img ref={mediaRef} src={media.url} alt="sighting" onLoad={onLoaded} onError={onMediaError}
                    style={{ width: "100%", display: "block", pointerEvents: "none", imageRendering: view.z > 4 ? "pixelated" : "auto", filter: imgAdjFilter(src.imgAdj) }} draggable={false} />
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

          {media && (
            <div style={{ marginTop: 8 }}>
              <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
                <ML style={{ marginBottom: 1 }}>Brightness / contrast</ML>
                <span style={{ display: "flex", gap: 8, alignItems: "baseline" }}>
                  <span style={{ fontSize: 10, color: "var(--dim)", fontFamily: "var(--mono)" }}>display only · original kept</span>
                  {!imgAdjNeutral(src.imgAdj) && <button className="btn sm" style={{ padding: "2px 8px" }} onClick={() => update({ imgAdj: { bri: 100, con: 100 } })}>↺ reset</button>}
                </span>
              </div>
              <div style={{ display: "flex", gap: 9, alignItems: "center" }}>
                <span style={{ fontSize: 14, width: 16, textAlign: "center" }} title="Brightness">☀</span>
                <input type="range" min={20} max={400} step={2} value={src.imgAdj?.bri ?? 100}
                  onChange={(e) => update({ imgAdj: { bri: +e.target.value, con: src.imgAdj?.con ?? 100 } })} style={{ flex: 1 }} />
              </div>
              <div style={{ display: "flex", gap: 9, alignItems: "center" }}>
                <span style={{ fontSize: 14, width: 16, textAlign: "center" }} title="Contrast">◐</span>
                <input type="range" min={50} max={300} step={2} value={src.imgAdj?.con ?? 100}
                  onChange={(e) => update({ imgAdj: { bri: src.imgAdj?.bri ?? 100, con: +e.target.value } })} style={{ flex: 1 }} />
              </div>
              <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 1 }}>Lifts a dark night shot so you can see stars &amp; the object — carries into the sky view and report; measurements still use the original.</div>
            </div>
          )}

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

      {src.meta && !src.meta.heic && !src.meta.stripped && (
        <div style={{ marginTop: 10, padding: "8px 10px", border: "1px solid var(--amber)", borderRadius: 10, background: "rgba(245,169,63,.06)" }}>
          <ML style={{ color: "var(--amber)" }}>📎 Auto-filled from the file ✓</ML>
          <div style={{ fontFamily: "var(--mono)", fontSize: 11, color: "var(--dim)", lineHeight: 1.6 }}>
            {isNum(src.meta.lat) && <div>GPS {src.meta.lat}, {src.meta.lon}{isNum(src.meta.alt) ? ` · ${src.meta.alt} m` : ""}</div>}
            {src.meta.timeMs && <div>{new Date(src.meta.timeMs).toLocaleString()}</div>}
            {isNum(src.meta.az) && <div>camera bearing {src.meta.az}° {src.meta.azRef}{src.meta.azRef === "magnetic" ? (isNum(src.meta.azTrue) ? ` → ${src.meta.azTrue}° true (WMM declination ${src.meta.decl >= 0 ? "+" : ""}${src.meta.decl}°)` : " (true ≈ magnetic + local declination)") : ""}</div>}
            {isNum(src.meta.fovH) && <div>FOV {src.meta.fovH}° (from {src.meta.f35} mm-eq lens)</div>}
            {src.meta.model && <div>{src.meta.model}</div>}
          </div>
          {isNum(src.meta.lat) ? (
            <div style={{ marginTop: 4, fontSize: 11, color: "var(--dim)" }}>
              Position, time, FOV{src.meta.az != null ? ", bearing & photo placement" : ""} were applied — every field below stays editable.
            </div>
          ) : (
            <div style={{ marginTop: 4, fontSize: 11, color: "var(--dim)" }}>
              No GPS in this file (Location was likely off when it was shot) — search your spot by name or drop the pin on the position step.
            </div>
          )}
        </div>
      )}
      {src.meta?.heic && (
        <div style={{ marginTop: 10, fontSize: 11, color: "var(--dim)" }}>
          📎 HEIC file — metadata unreadable here. Export or share as JPEG to auto-fill GPS, time, bearing, and FOV.
        </div>
      )}
      {src.meta?.stripped && (
        <div style={{ marginTop: 10, padding: "8px 10px", border: "1px solid var(--amber)", borderRadius: 10, background: "rgba(245,169,63,.06)" }}>
          <ML style={{ color: "var(--amber)" }}>⚠ No location, time, or direction in this photo</ML>
          <div style={{ fontSize: 11, color: "var(--dim)", lineHeight: 1.6 }}>
            The image loaded fine, but its metadata was stripped before it reached here — the tell-tale of a re-encoded copy (sent through Messages/WhatsApp/email, or shared with the Share Sheet's <b>“All Photos Data”</b> turned off). Nothing was lost in transit here; the geodata simply isn’t in the file.
            <div style={{ marginTop: 5 }}>To keep it next time: in Photos, tap <b>Share → Options (top) → All Photos Data ON</b>, Location ON, then AirDrop the original — and don’t route it through a messaging app.</div>
            <div style={{ marginTop: 5, color: "var(--teal)" }}>You can still measure this photo — just set the location by name or pin, the date/time, and the FOV on the steps that follow.</div>
          </div>
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


const ENABLE_SENSORS = false; // 🧭 point-with-phone + 📷 camera AR — parked for now, flip to bring back
const ENABLE_GPS_BUTTON = false; // 📍 use-my-GPS — parked (unreliable in the field), flip to bring back

/* reference silhouettes for the in-sky Compare tool (from Sky Sense) */
const GHOSTW = [
  { name: "Mylar balloon", short: "Balloon", m: 0.5, shape: "c" },
  { name: "Drone", short: "Drone", m: 0.35, shape: "d" },
  { name: "Cessna 172", short: "Cessna", m: 11, shape: "p" },
  { name: "Airliner (737)", short: "737", m: 36, shape: "p" },
];
function GhostSil({ shape, w }) {
  /* true angular size, floored at 1 px — a balloon at 3 km IS sub-pixel,
     and faking it bigger defeats the whole comparison. Opaque black with a
     thin white outline so the reference reads clearly against any sky. */
  const ww = Math.max(w, 1);
  if (shape === "p") return (
    <svg width={ww} height={ww * 1.2} viewBox="0 0 100 120" style={{ display: "block", overflow: "visible" }}>
      <g fill="#000" stroke="#fff" strokeWidth="1.5" strokeLinejoin="round" vectorEffect="non-scaling-stroke">
        <path d="M50 4 C56 4 57 18 56 34 L56 92 C56 104 54 116 50 116 C46 116 44 104 44 92 L44 34 C43 18 44 4 50 4 Z" />
        <path d="M50 48 L3 84 L3 88 L16 88 L50 64 L84 88 L97 88 L97 84 Z" />
        <path d="M50 96 L24 110 L24 113 L34 113 L50 104 L66 113 L76 113 L76 110 Z" />
      </g>
    </svg>
  );
  if (shape === "d") return ( // quadcopter, top-down X-frame: 4 rotor discs + arms + hub
    <svg width={ww} height={ww} viewBox="0 0 100 100" style={{ display: "block", overflow: "visible" }}>
      <g fill="#000" stroke="#fff" strokeWidth="1.5" strokeLinejoin="round" vectorEffect="non-scaling-stroke">
        <rect x="8" y="45.5" width="84" height="9" rx="4" transform="rotate(45 50 50)" />
        <rect x="8" y="45.5" width="84" height="9" rx="4" transform="rotate(-45 50 50)" />
        <circle cx="19" cy="19" r="16" /><circle cx="81" cy="19" r="16" />
        <circle cx="19" cy="81" r="16" /><circle cx="81" cy="81" r="16" />
        <circle cx="50" cy="50" r="12" />
      </g>
    </svg>
  );
  return <svg width={ww} height={ww} viewBox="0 0 100 100" style={{ display: "block", overflow: "visible" }}><circle cx="50" cy="50" r="47" fill="#000" stroke="#fff" strokeWidth="1.5" vectorEffect="non-scaling-stroke" /></svg>;
}

/* The user's FITTED shape drawn at a given apparent pixel size — used at each
   trajectory point so the object visibly swells (closer) or shrinks (farther).
   Falls back to a ring when no shape is fitted. Major axis spans `px`. */
function TrackObj({ sf, px, color }) {
  const d = Math.max(px, 1);
  if (!sf) return (
    <svg width={d} height={d} viewBox="0 0 100 100" style={{ display: "block", overflow: "visible" }}>
      <circle cx="50" cy="50" r="46" fill="rgba(7,11,20,.30)" stroke={color} strokeWidth="1.6" vectorEffect="non-scaling-stroke" />
    </svg>
  );
  const pr = shapeProjNat(sf);
  const cx = sf.cx, cy = sf.cy;
  let mnx = 1e9, mny = 1e9, mxx = -1e9, mxy = -1e9;
  for (const c of pr.curves) for (const p of c) { const x = p.x - cx, y = p.y - cy; if (x < mnx) mnx = x; if (x > mxx) mxx = x; if (y < mny) mny = y; if (y > mxy) mxy = y; }
  const majorNat = Math.hypot(pr.p1.x - pr.p2.x, pr.p1.y - pr.p2.y) || 1;
  const scale = d / majorNat;
  const half = Math.max(Math.abs(mnx), Math.abs(mxx), Math.abs(mny), Math.abs(mxy), 1);
  return (
    <svg width={2 * half * scale} height={2 * half * scale} viewBox={`${-half} ${-half} ${2 * half} ${2 * half}`} style={{ display: "block", overflow: "visible" }}>
      {pr.curves.map((c, i) => (
        <polyline key={i} fill="none" stroke={color} strokeWidth="1.6" vectorEffect="non-scaling-stroke"
          points={c.map((p) => `${(p.x - cx).toFixed(1)},${(p.y - cy).toFixed(1)}`).join(" ")} />
      ))}
    </svg>
  );
}

function SkyAimer({ open, onClose, lat, lng, whenMs, initAz, initAlt, marks, which, onCapture, source, update, wizard, onWizardBack, onWizardNext }) {
  const [vpRef, vp] = useSize();
  /* Highest elevation the view/placement may reach. NOT 90°: at exactly the
     zenith the az/el basis is a gimbal singularity (the "right" vector →0), so
     the projection breaks. 89.5° is visually straight-up yet numerically stable,
     and lets you spin azimuth around the pole to bring zenith stars to center. */
  const EL_MAX = 89.5;
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
  const calibTapRef = useRef(null);   // pending dome tap while aligning — pick nearest named star on release
  const pointersRef = useRef(new Map());
  const pinchRef = useRef(null);

  /* photo-in-sky placement */
  const [photoOn, setPhotoOn] = useState(false);
  const [pMode, setPMode] = useState("look"); // 'look' | 'place'
  const [pAz, setPAz] = useState(180);
  const [pEl, setPEl] = useState(30);
  const [pRoll, setPRoll] = useState(0);
  const [fovM, setFovM] = useState(68);      // photo's own FOV (calibrated by pinch)
  const [pDist, setPDist] = useState(0);     // radial lens distortion (tan-space k) — 0 unless star-calibrated
  const openPoseRef = useRef(null);          // placement as of aimer-open — Reset target
  const PH_OP = 0.85; // photo opacity — fixed; the grid/terrain still reads through the warp
  const [flash, setFlash] = useState("");
  const [selSeg, setSelSeg] = useState(null);   // Δt chip being edited
  const [selPt, setSelPt] = useState(null);     // trajectory point whose turn radius is being edited
  const [rotMode, setRotMode] = useState(false); // when on, dragging the dome rotates the selected point's shape
  const rotDragRef = useRef(null), rotRafRef = useRef(0);
  const rotTwistRef = useRef(null); // second finger anchors a view-axis twist (roll) once a point rotation is underway
  /* compare ghost — buttons only, NO sliders and NO draggable elements:
     the aimer holds a document-level touch lock (invariant: iOS multi-touch),
     which silently eats native drags on anything inside it. Drop the ghost
     at the crosshair like a trajectory point; distance via preset chips. */
  const [cmpOn, setCmpOn] = useState(false);
  const [cmpD, setCmpD] = useState(1000);       // ghost's assumed distance, meters
  const [ghostIdx, setGhostIdx] = useState(3);
  const [cmpPos, setCmpPos] = useState(null);   // ghost's sky anchor {az, el}
  const [objD, setObjD] = useState(1000);       // YOUR OBJECT's assumed distance — size↔distance guesstimate
  const [sizeOn, setSizeOn] = useState(false);  // object size↔distance tool — its own toggle (was stacked under compare)
  /* two-tap star align: tap a known object, tap where it really sits in the
     photo → solve the photo's roll + FOV (center kept, so the terrain match
     is preserved) so the object lands exactly. */
  const [calibOn, setCalibOn] = useState(false);
  const [calibAnchor, setCalibAnchor] = useState(null); // {name, az, el}
  const [calibMsg, setCalibMsg] = useState("");
  const [calibApplied, setCalibApplied] = useState(false);
  const [calibCount, setCalibCount] = useState(0); // # of stars aligned (2+ enables lens-distortion fit)
  const calibPrevRef = useRef(null);   // {fov, roll, dist} before the first align — for reset
  const calibAnchorsRef = useRef([]);  // [{px, py, g}] accumulated star correspondences
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
  /* Video in the aimer is a SINGLE still: the frame the object was marked on
     (A.videoTime). It's baked to a static texture once (below) — the analysis
     is single-frame, so there is no scrubber here, and the warp never touches
     a live <video> (repeated per-render video→canvas draws were the source of
     the jank and the iOS memory crashes that kicked back to the start). */
  const [vidFrameUrl, setVidFrameUrl] = useState(null); // baked marked-frame data URL for Place mode
  const placeRef = useRef(null);
  const twistRef = useRef(null);
  const warpRef = useRef(null);   // canvas that draws the Look-mode warp ourselves
  const texRef = useRef(null);
  const [, setTexReady] = useState(0);

  /* decode the still texture for the canvas warp ONCE. Image: the EXIF-
     normalized photo. Video: the marked frame (A.videoTime) baked to a
     canvas off the render path, so the warp draws a static texture — never
     a live <video> — which is what made the sky view fast and stable. */
  const MAXT = 1280;
  const bakeTex = (drawable, w, h) => {
    const adj = source?.imgAdj, needAdj = !imgAdjNeutral(adj);
    let tex = drawable;
    try {
      if (w > MAXT || h > MAXT || needAdj) {   // draw to a canvas to downscale and/or bake the B/C adjustment in
        const sc = Math.min(1, MAXT / Math.max(w, h));
        const cv = document.createElement("canvas");
        cv.width = Math.round(w * sc); cv.height = Math.round(h * sc);
        const cx = cv.getContext("2d", { willReadFrequently: needAdj });
        cx.drawImage(drawable, 0, 0, cv.width, cv.height);
        if (needAdj) applyImgAdj(cx, cv.width, cv.height, adj);
        tex = cv;
      }
    } catch (e) { /* keep full-res */ }
    texRef.current = tex; setTexReady((v) => v + 1);
  };
  useEffect(() => {
    texRef.current = null; setTexReady((v) => v + 1); setVidFrameUrl(null);
    if (!source?.mediaUrl) return;
    if (source?.mediaKind === "video") {
      const v = document.createElement("video");
      v.muted = true; v.playsInline = true; v.preload = "auto";
      let dead = false;
      const t = isNum(source?.A?.videoTime) ? +source.A.videoTime : 0;
      v.onloadeddata = () => { if (!dead) { try { v.currentTime = t > 0.01 ? t : Math.min(0.04, (v.duration || 1) / 4); } catch (e) { } } };
      v.onseeked = () => {
        if (dead || !v.videoWidth) return;
        try {
          const cv = document.createElement("canvas");
          cv.width = v.videoWidth; cv.height = v.videoHeight;
          cv.getContext("2d").drawImage(v, 0, 0);
          bakeTex(cv, cv.width, cv.height);
          setVidFrameUrl(cv.toDataURL("image/jpeg", 0.9));
        } catch (e) { /* frame not paintable yet */ }
      };
      v.src = source.mediaUrl;
      return () => { dead = true; v.removeAttribute("src"); v.load(); };
    }
    const im = new Image();
    im.onload = () => bakeTex(im, im.naturalWidth, im.naturalHeight);
    im.src = source.mediaUrl;
  }, [source?.mediaUrl, source?.mediaKind, source?.A?.videoTime, source?.imgAdj?.bri, source?.imgAdj?.con]);

  /* aim starts on the previously entered direction, if any */
  useEffect(() => {
    if (open) {
      setViewAz(isNum(initAz) ? ((+initAz % 360) + 360) % 360 : 180);
      setViewAlt(isNum(initAlt) ? clampN(+initAlt, -15, EL_MAX) : 30);
      setMotionMsg(""); setCameraMsg("");
      const ma = source?.mediaAim;
      const p0 = {
        az: ma ? ma.az : (isNum(source?.A?.az) ? +source.A.az : (isNum(initAz) ? +initAz : 180)),
        el: clampN(ma ? ma.el : (isNum(source?.A?.el) ? +source.A.el : 30), -20, EL_MAX), // never exactly 90° — photo basis is singular at the zenith
        roll: ma ? (ma.roll || 0) : 0,
        fov: isNum(source?.fovH) ? +source.fovH : 68,
        dist: ma && isNum(ma.dist) ? +ma.dist : 0,
      };
      openPoseRef.current = p0; // Reset restores the WHOLE placement to this
      setPAz(p0.az); setPEl(p0.el); setPRoll(p0.roll); setFovM(p0.fov); setPDist(p0.dist);
      setCalibOn(false); setCalibAnchor(null); setCalibMsg(""); setCalibApplied(false); setCalibCount(0);
      calibPrevRef.current = null; calibAnchorsRef.current = [];
      setPhotoOn(!!source?.mediaUrl);
      /* start in Place only until this observer has been placed ONCE in the
         sky view (source.placed) — after that, return straight to Look and
         skip the adjust step. `mediaAim` alone can't gate this: it's also
         pre-set from the photo's EXIF bearing before any placement. The
         ✥ Place button is still there to re-adjust. */
      setPMode(source?.placed || !source?.mediaUrl ? "look" : "place");
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
  /* ridge/terrain line hue — a display preference (the default green washes
     out over green hillsides for some photos); persisted across sessions.
     Default 106° ≈ the original rgba(158,224,138). */
  const [ridgeHue, setRidgeHue] = useState(() => {
    try { const raw = localStorage.getItem("phodar:ridgeHue"); if (raw != null && Number.isFinite(+raw)) return +raw; } catch (e) { }
    return 106;
  });
  useEffect(() => { try { localStorage.setItem("phodar:ridgeHue", String(ridgeHue)); } catch (e) { } }, [ridgeHue]);
  const ridgeCol = (a) => `hsla(${ridgeHue},58%,71%,${a})`;
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

  /* Starlink layer — opt-in (the full ~7k constellation). One SGP4 per sat at
     the sighting instant (memoised on T/pos, not on pan/zoom), then keep only
     the sunlit ones above the horizon and cap the count so the dome stays
     legible. Markers only (no per-sat trails — too many). */
  const [starlinkOn, setStarlinkOn] = useState(false);
  const [slDb, setSlDb] = useState(null); // {sats} | {err}
  useEffect(() => {
    if (!open || !starlinkOn || slDb) return;
    let dead = false;
    loadSatGroup("starlink").then((db) => { if (!dead) setSlDb(db); })
      .catch((e) => { if (!dead) setSlDb({ err: String(e?.message || e) }); });
    return () => { dead = true; };
  }, [open, starlinkOn, slDb]);
  const slView = useMemo(() => {
    if (!starlinkOn || !slDb?.sats || !hasPos) return [];
    return satsAt(slDb.sats, T, LAT, LNG, 0).filter((s) => s.lit).slice(0, 60);
  }, [starlinkOn, slDb, T, LAT, LNG, hasPos]);

  /* named peaks (OSM Overpass) — placed on the terrain skyline by bearing +
     curvature-corrected elevation; opt-in, fetched once per open. */
  const [peaksOn, setPeaksOn] = useState(false);
  const [peaks, setPeaks] = useState(null); // [] | {err}
  useEffect(() => {
    if (!open || !peaksOn || peaks || !hasPos) return;
    let dead = false;
    fetchPeaks(LAT, LNG, isNum(source?.alt) ? +source.alt : 0, 120) // wide net: tall far peaks (Shasta, McLoughlin…) sit well past 40 km
      .then((ps) => { if (!dead) setPeaks(ps); })
      .catch((e) => { if (!dead) setPeaks({ err: String(e?.message || e) }); });
    return () => { dead = true; };
  }, [open, peaksOn, peaks, hasPos, LAT, LNG]); // eslint-disable-line
  /* Show the named summits/hills near the observer. Those that sit ON the
     terrain silhouette (elevation at/above the DEM skyline at their azimuth)
     are prioritised, but nothing is HARD-hidden — a peak the app thinks is
     occluded may just be DEM/OSM height disagreement, and the user wants to see
     the named peaks either way. Sort: on-silhouette first, then nearest; cap. */
  const peakMarks = (() => {
    if (!(peaksOn && Array.isArray(peaks)) || !peaks.length) return [];
    const onSil = (pk) => {
      if (pk.el == null || !terr?.els) return true;
      return pk.el >= skylineElAt(terr.els, pk.az) - 1.0;
    };
    return peaks.slice()
      .sort((a, b) => (onSil(b) - onSil(a)) || (a.distKm - b.distKm))
      .slice(0, 60);
  })();

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
  const effAlt = placing ? clampN(pEl, -20, EL_MAX) : viewAlt;
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
      /* Second finger while a point rotation is ALREADY underway → don't pinch:
         anchor with this finger and let the twist between the two fingers roll
         the shape about the view axis (fine control the 1-finger trackball
         can't give), mirroring the photo-placement twist. */
      if (rotDragRef.current && rotMode && selPt != null && source?.shapeFit) {
        const rd = rotDragRef.current;
        const anchorId = e.pointerId, driverId = rd.pid;
        const anc = pointersRef.current.get(anchorId);
        const drv = pointersRef.current.get(driverId) || anc;
        if (rotRafRef.current) { cancelAnimationFrame(rotRafRef.current); rotRafRef.current = 0; }
        rotTwistRef.current = {
          idx: rd.idx, anchorId, driverId,
          a0: Math.atan2(drv.y - anc.y, drv.x - anc.x),
          R0: rd.pending || rd.R0, // continue from the live orientation, no jump
        };
        rotDragRef.current = null;
        return;
      }
      panRef.current = null; placeRef.current = null; rotDragRef.current = null; calibTapRef.current = null;
      const g = twoPtGeom();
      if (placing) twistRef.current = g;       // {ids, dist, ang} — rebaselined every event
      else pinchRef.current = g;
    } else if (placing) {
      placeRef.current = { x: e.clientX, y: e.clientY, az: pAz, el: pEl };
    } else if (rotMode && selPt != null && source?.shapeFit) {
      rotDragRef.current = { idx: selPt, x: e.clientX, y: e.clientY, R0: ptRotM(selPt), pid: e.pointerId };
    } else {
      panRef.current = { x: e.clientX, y: e.clientY, az: viewAz, alt: viewAlt };
      /* while aligning, a single-finger DRAG still pans (above); a TAP (little
         movement, resolved on release) picks the nearest named star — so the
         many sky labels never have to become pan-blocking tap targets */
      if (calibOn && !calibAnchor) calibTapRef.current = { x: e.clientX, y: e.clientY };
    }
  };
  const onBgMove = (e) => {
    if (pointersRef.current.has(e.pointerId)) pointersRef.current.set(e.pointerId, { x: e.clientX, y: e.clientY });
    const n = pointersRef.current.size;
    if (n >= 2) {
      /* two-finger view-axis twist rolling the selected point's shape */
      if (rotTwistRef.current) {
        const tw = rotTwistRef.current;
        const anc = pointersRef.current.get(tw.anchorId), drv = pointersRef.current.get(tw.driverId);
        if (anc && drv) {
          const a1 = Math.atan2(drv.y - anc.y, drv.x - anc.x);
          const rm = mul3(rotZ3((a1 - tw.a0) * R2D), tw.R0); // roll about the view axis
          tw.cur = rm; // live orientation, so lifting back to one finger continues from here
          if (!rotRafRef.current) rotRafRef.current = requestAnimationFrame(() => {
            rotRafRef.current = 0;
            const p = rotTwistRef.current; if (p && p.cur) setPtRot(p.idx, p.cur);
          });
        }
        return;
      }
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
        setFov((f) => clampN(f / ratio, 2, 90));
      }
      return;
    }
    if (placeRef.current && vp.w) {
      const pr = placeRef.current; // snapshot: pointerup may null the ref before React flushes
      const dx = (e.clientX - pr.x) / vp.w, dy = (e.clientY - pr.y) / (vp.h || vp.w);
      const nAz = (((pr.az + dx * fovH) % 360) + 360) % 360;
      const nEl = clampN(pr.el - dy * fovV, -20, EL_MAX);
      queuePose("place", nAz, nEl);
      return;
    }
    if (rotDragRef.current && vp.w) {
      /* drag = 3D trackball on the selected point's shape (rAF-coalesced) */
      const rd = rotDragRef.current, kk = 0.5;
      rd.pending = mul3(mul3(rotX3(-(e.clientY - rd.y) * kk), rotY3((e.clientX - rd.x) * kk)), rd.R0);
      if (!rotRafRef.current) rotRafRef.current = requestAnimationFrame(() => {
        rotRafRef.current = 0;
        const p = rotDragRef.current; if (p && p.pending) setPtRot(p.idx, p.pending);
      });
      return;
    }
    if (panRef.current && vp.w && !motionOn) {
      const dx = (e.clientX - panRef.current.x) / vp.w, dy = (e.clientY - panRef.current.y) / (vp.h || vp.w);
      queuePose("look",
        (((panRef.current.az - dx * fovH) % 360) + 360) % 360,
        clampN(panRef.current.alt + dy * fovV, -15, EL_MAX));
    }
  };
  const onBgUp = (e) => {
    pointersRef.current.delete(e.pointerId);
    try { e.currentTarget.releasePointerCapture(e.pointerId); } catch (_) { }
    const n = pointersRef.current.size;
    if (n < 2) { pinchRef.current = null; twistRef.current = null; }
    if (rotTwistRef.current && n < 2) {
      /* lifting out of a two-finger twist: hand control back to whichever
         finger remains, seeded at the post-twist orientation so it doesn't jump */
      const tw = rotTwistRef.current, cur = tw.cur || tw.R0;
      rotTwistRef.current = null;
      if (rotRafRef.current) { cancelAnimationFrame(rotRafRef.current); rotRafRef.current = 0; }
      setPtRot(tw.idx, cur); // flush the final twist frame (rAF may have been dropped)
      const rem = [...pointersRef.current.entries()][0];
      if (rem && rotMode && selPt != null && source?.shapeFit)
        rotDragRef.current = { idx: tw.idx, x: rem[1].x, y: rem[1].y, R0: cur, pid: rem[0] };
    }
    if (n === 1) {
      const p = [...pointersRef.current.values()][0];
      if (placing) placeRef.current = { x: p.x, y: p.y, az: pAz, el: pEl };
      else if (rotDragRef.current) { /* twist handed control back to this finger — keep the rotate drag */ }
      else panRef.current = { x: p.x, y: p.y, az: viewAz, alt: viewAlt };
    } else if (n === 0) {
      /* align mode: a tap (barely moved) picks the nearest named star/planet;
         a drag panned instead and is ignored here */
      const ct = calibTapRef.current; calibTapRef.current = null;
      if (ct && calibOn && !calibAnchor && Math.hypot(e.clientX - ct.x, e.clientY - ct.y) < 12 && vpRef.current) {
        const r = vpRef.current.getBoundingClientRect();
        const tx = e.clientX - r.left, ty = e.clientY - r.top;
        let best = null, bestD = Infinity;
        for (const o of skyRefs) {
          const pr = project(o.az, o.el); if (!pr.inFront) continue;
          const d = Math.hypot(pr.x * vp.w - tx, pr.y * vp.h - ty);
          if (d < bestD) { bestD = d; best = o; }
        }
        if (best && bestD < 60) pickCalib(best);
        else setCalibMsg("No named star near your tap — pan/zoom to bring one into view, then tap it");
      }
      panRef.current = null; placeRef.current = null; rotDragRef.current = null; rotTwistRef.current = null;
      if (rotRafRef.current) { cancelAnimationFrame(rotRafRef.current); rotRafRef.current = 0; }
      if (placing) commitPlacement();
    }
  };
  useEffect(() => {
    const el = vpRef.current; if (!el || !open) return;
    const onWheel = (ev) => { ev.preventDefault(); setFov((f) => clampN(+(f * (ev.deltaY > 0 ? 1.08 : 1 / 1.08)).toFixed(1), 2, 90)); };
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
  /* nearer visible ridge crests below the silhouette — same green, faded
     with distance like real haze; occluded stretches never emit a crest
     so the layering is already respected in the data */
  const ridgePaths = (terrOn && terr?.ridges) ? terr.ridges.map((r) => {
    const pts = r.pts.map(([raz, rel]) => {
      const da = ((raz - effAz + 540) % 360) - 180;
      return Math.abs(da) <= 130 ? project(effAz + da, rel) : { inFront: false };
    });
    const d = gpath(pts);
    if (!d) return null;
    const t = clampN(Math.log(r.dist / 800) / Math.log(35000 / 800), 0, 1);
    return { d, o: 0.60 - 0.35 * t };
  }).filter(Boolean) : [];
  const terrainLbl = (terrainPath) ? (() => {
    const p = project(effAz, skylineElAt(terr.els, effAz));
    return p.inFront && p.y > 0.04 && p.y < 0.96 ? p : null;
  })() : null;
  const horizonY = project(effAz, 0).y;
  const cardinals = [[0, "N"], [45, "NE"], [90, "E"], [135, "SE"], [180, "S"], [225, "SW"], [270, "W"], [315, "NW"]].map(([az, lbl]) => ({ ...project(az, 1.8), lbl })).filter((c) => c.inFront && c.x > 0.02 && c.x < 0.98 && c.y > -0.05 && c.y < 1.05);
  const starDots = !cameraOn ? stars.map((s) => ({ ...project(s.az, s.alt), r: s.r, o: s.o, name: s.name, mag: s.mag, az: s.az, el: s.alt })).filter((p) => p.inFront && p.x > -0.05 && p.x < 1.05 && p.y > -0.05 && p.y < 1.05) : [];
  /* while aligning, label EVERY named star in view (you're picking anchors);
     otherwise keep it to the bright ones so the sky isn't cluttered */
  const starLabels = starDots.filter((p) => p.name && (calibOn ? p.mag <= 3.4 : (p.mag <= 1.6 || (fovH < 42 && p.mag <= 2.6))));
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
    const s = 1 + pDist * (x * x + y * y); // radial lens distortion (0 unless star-calibrated)
    return unit([f[0] + (r[0] * x + u[0] * y) * s, f[1] + (r[1] * x + u[1] * y) * s, f[2] + (r[2] * x + u[2] * y) * s]);
  };

  /* known sky objects usable as calibration anchors (bright + labeled) */
  const skyRefs = (() => {
    const out = [];
    if (planetsVisible) for (const p of planets) if (p.alt > 0.5) out.push({ name: p.name, az: p.az, el: p.alt, sym: p.sym });
    for (const st of stars) if (st.name && st.mag <= 3.4 && st.alt > 0.5) out.push({ name: st.name, az: st.az, el: st.alt, mag: st.mag });
    if (sun.alt > 0.5) out.push({ name: "Sun", az: sun.az, el: sun.alt, sym: "☀" });
    if (moon.alt > 0.5) out.push({ name: "Moon", az: moon.az, el: moon.alt, sym: "☾" });
    return out;
  })();
  /* the subset currently on screen — offered as chips to pick an align anchor */

  /* pick which object to align to (from a chip), then aim the crosshair on it in
     the photo and press the button — no fiddly tap while zoomed. */
  const pickCalib = (obj) => { setCalibAnchor(obj); setCalibMsg(`Center the crosshair on ${obj.name} in the photo, then press ✓ Set`); };
  const alignAtCrosshair = () => {
    if (!calibAnchor || !photo) return;
    const vC = unproject(0.5, 0.5);                        // crosshair world dir = object's apparent spot
    if (vC[0] * photo.f[0] + vC[1] * photo.f[1] + vC[2] * photo.f[2] <= 0.02) { setCalibMsg("Aim the crosshair onto the object in the photo first"); return; }
    const pix = dirToPixK(vC, photo.natW, photo.natH, pAz, pEl, pRoll, fovM, pDist); // the object's fixed pixel
    if (!pix) { setCalibMsg("Couldn't read that spot — re-aim the crosshair on the object"); return; }
    const g = dirOf(calibAnchor.az, calibAnchor.el);
    if (calibPrevRef.current == null) calibPrevRef.current = { az: pAz, el: pEl, fov: fovM, roll: pRoll, dist: pDist };
    const list = [...calibAnchorsRef.current, { px: pix.px, py: pix.py, g }];
    calibAnchorsRef.current = list;
    const oldFov = fovM;
    const sol = solvePoseAnchors(list, photo.natW, photo.natH, pAz, pEl, { roll: pRoll, fov: fovM, k: pDist });
    if (list.length >= 3) { setPAz(sol.az); setPEl(clampN(sol.el, -20, EL_MAX)); } // full plate solve moves the pointing too
    setPRoll(clampN(((sol.roll + 180) % 360 + 360) % 360 - 180, -90, 90));
    setFovM(clampN(sol.fov, 8, 135));
    setPDist(sol.k);
    setCalibApplied(true); setCalibCount(list.length);
    setCalibMsg(list.length >= 3
      ? `✓ ${list.length} stars — full solve · FOV ${sol.fov.toFixed(0)}° · lens ${sol.k >= 0 ? "+" : ""}${sol.k.toFixed(3)} · fit ${sol.rms.toFixed(2)}° (whole sky matched)`
      : list.length === 2
        ? `✓ 2 stars · FOV ${sol.fov.toFixed(0)}° · lens fit ${sol.rms.toFixed(2)}° · add a 3rd star for a full plate-solve (matches the whole sky)`
        : `✓ Aligned to ${calibAnchor.name} · FOV ${oldFov.toFixed(0)}→${sol.fov.toFixed(0)}° · add a 2nd star for lens distortion`);
    setCalibAnchor(null);   // stay in align mode so more stars can be added
  };
  const resetCalib = () => {
    const p = calibPrevRef.current;
    if (p) { setFovM(p.fov); setPRoll(p.roll); setPDist(p.dist || 0); if (isNum(p.az)) setPAz(p.az); if (isNum(p.el)) setPEl(p.el); }
    calibPrevRef.current = null; calibAnchorsRef.current = []; setCalibCount(0);
    setCalibApplied(false); setCalibAnchor(null); setCalibMsg("");
  };

  const enterPlace = () => {
    /* first-ever placement: put the photo where you're looking */
    if (!source?.mediaAim) { setPAz(viewAz); setPEl(clampN(viewAlt, -20, EL_MAX)); }
    if (motionOn) setMotionOn(false);
    if (calibOn) { setCalibOn(false); setCalibAnchor(null); setCalibMsg(""); }
    calibAnchorsRef.current = []; setCalibCount(0);   // manual place invalidates the star anchors
    setPMode("place");
  };
  const donePlace = () => {
    /* hand the (already photo-centered) view back seamlessly — nothing moves */
    setViewAz(pAz); setViewAlt(clampN(pEl, -15, EL_MAX));
    setFov(clampN(effFov, 2, 90));
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
    const tex = texRef.current; // static image OR the baked video frame — never a live <video>
    if (!tex) return;
    const tw = tex.naturalWidth || tex.width;
    const th = tex.naturalHeight || tex.height;
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
    const patch = { mediaAim: { az: +pAz.toFixed(2), el: +pEl.toFixed(2), roll: +pRoll.toFixed(1), dist: +pDist.toFixed(5) }, fovH: +fovM.toFixed(1), placed: true };
    /* placement + marked points fully determine the sight-lines — derive
       A (object marks / shape fit) and B (motion mark) automatically, so
       the fix never dies for want of an elevation the user already gave us */
    if (source && !(source.track || []).length && source.natW) {
      const fpx = (source.natW / 2) / Math.tan((fovM * D2R) / 2);
      const bb = photoBasis(pAz, pEl, pRoll);
      const dirAt = (px, py) => {
        const x = (px - source.natW / 2) / fpx, y = (source.natH / 2 - py) / fpx;
        const s = 1 + pDist * (x * x + y * y); // match the calibrated lens distortion
        return dirToAzEl(unit([bb.f[0] + (bb.r[0] * x + bb.u[0] * y) * s, bb.f[1] + (bb.r[1] * x + bb.u[1] * y) * s, bb.f[2] + (bb.r[2] * x + bb.u[2] * y) * s]));
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
  /* --- per-point 3D ORIENTATION (optional): drawing the shape at its attitude
     records how it was oriented, AND lets the range math divide out
     foreshortening (a rotating tic-tac isn't flying away). baseRotM folds the
     shape's own roll in; projMajorFor gives the silhouette's projected extent
     at an orientation, so projF = extent(rm) / extent(base). --- */
  const baseRotM = source?.shapeFit ? mul3(source.shapeFit.rotM || I3, rotZ3(source.shapeFit.roll || 0)) : I3;
  const projMajorFor = (rm) => {
    if (!source?.shapeFit) return 1;
    const pr = shapeProjNat({ ...source.shapeFit, rotM: rm, roll: 0 });
    return Math.hypot(pr.p1.x - pr.p2.x, pr.p1.y - pr.p2.y) || 1;
  };
  const baseMajor = source?.shapeFit ? projMajorFor(baseRotM) : 1;
  const ptRotM = (i) => (Array.isArray(sortedTrack[i]?.rotM) ? sortedTrack[i].rotM : baseRotM);
  const setPtRot = (i, rm) => {
    const projF = +(projMajorFor(rm) / baseMajor).toFixed(5);
    update({ track: sortedTrack.map((p, j) => (j === i ? { ...p, rotM: rm, projF } : p)) });
  };
  const resetPtRot = (i) => update({ track: sortedTrack.map((p, j) => { if (j !== i) return p; const { rotM, projF, ...rest } = p; return rest; }) });
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
       deliberate claim the witness makes by selecting the point.
       Seed the apparent SIZE from the previous point (or the measured size);
       resizing a point later signifies it moving closer/farther (radial). */
    const prevAng = sortedTrack.length ? sortedTrack[sortedTrack.length - 1].ang : null;
    const seedAng = isNum(prevAng) ? +prevAng : (objAngW != null ? +objAngW : null);
    const np = { t: tN, az: +az.toFixed(2), el: +el.toFixed(2), r: 0.3 };
    if (seedAng != null) np.ang = +seedAng.toFixed(4);
    const track = [...sortedTrack, np];
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

  /* ⛰ SNAP TO RIDGES — one-tap absolute pose: detect the photo's skyline,
     cross-correlate against the DEM skyline, apply the az/pitch/roll fix. */
  const snapToRidges = async () => {
    if (!terr?.els || !source?.mediaUrl || !source?.natW || !photo) { setFlash("⛰ needs terrain data + a placed photo"); return; }
    try {
      const im = await new Promise((res, rej) => {
        const i = new Image(); i.onload = () => res(i); i.onerror = rej; i.src = source.mediaUrl;
      });
      const DW = 480, k = DW / source.natW, DH = Math.max(60, Math.round(source.natH * k));
      const cv = document.createElement("canvas");
      cv.width = DW; cv.height = DH;
      const ctx = cv.getContext("2d", { willReadFrequently: true });
      ctx.drawImage(im, 0, 0, DW, DH);
      const pts = detectSkyline(ctx.getImageData(0, 0, DW, DH), DW, DH);
      if (!pts) { setFlash("⛰ no clean skyline in the photo — align manually against the dashed TERRAIN line"); return; }
      const fpx = (source.natW / 2) / Math.tan((fovM * D2R) / 2);
      /* two passes: the correction linearizes around the current pose, so
         re-derive the sample directions under the pass-1 result and refine.
         Verified against a synthetic DEM photo: pass 1 lands within ~0.6°,
         pass 2 within ~0.1° of the known truth pose. */
      let az = pAz, el = pEl, roll = pRoll, m = null;
      for (let pass = 0; pass < 2; pass++) {
        const bb = photoBasis(az, el, roll);
        const samples = pts.map((p) => {
          const nx = p.x / k, ny = p.y / k;
          const x = (nx - source.natW / 2) / fpx, y = (source.natH / 2 - ny) / fpx;
          const ae = dirToAzEl(unit([bb.f[0] + bb.r[0] * x + bb.u[0] * y, bb.f[1] + bb.r[1] * x + bb.u[1] * y, bb.f[2] + bb.r[2] * x + bb.u[2] * y]));
          return { az: ae.az, el: ae.el, thx: Math.atan2(nx - source.natW / 2, fpx) };
        });
        m = matchSkyline(samples, (a) => skylineElAt(terr.els, a));
        if (!m || m.rms > 0.8) { setFlash(`⛰ ridges don't match the DEM cleanly (rms ${m ? m.rms.toFixed(2) : "—"}°) — align manually`); return; }
        /* az/el errors add; roll subtracts (signs verified empirically) */
        az = ((az + m.dAz) % 360 + 360) % 360;
        el = clampN(el + m.dEl, -20, EL_MAX);
        roll -= m.dRollDeg;
      }
      setPAz(az); setPEl(el); setPRoll(roll);
      setFlash(`⛰ locked to terrain: ${az.toFixed(1)}° az · ${el.toFixed(1)}° up · roll ${roll.toFixed(1)}° · fit ${m.rms.toFixed(2)}° (${m.n} pts)`);
    } catch (e) { setFlash("⛰ snap failed on this image"); }
  };

  /* ✦ AUTO STAR-ALIGN — a plate solve. Detects the bright stars in the photo,
     then SEEDLESSLY matches their PATTERN to the catalog (asterism matching:
     star-to-star angular distances are pose-invariant, FOV known from EXIF) —
     the human never has to get it close. Falls back to a seeded solve from the
     current placement if the blind match doesn't lock. Solves az/el/roll/FOV/
     lens distortion at once. */
  const autoAlign = async () => {
    if (!source?.mediaUrl || !source?.natW || !photo) { setFlash("✦ open the photo first, then auto-align"); return; }
    setFlash("✦ finding stars & matching the sky…");
    try {
      const im = await new Promise((res, rej) => { const i = new Image(); i.onload = () => res(i); i.onerror = rej; i.src = source.mediaUrl; });
      const DW = Math.min(1600, source.natW), sc = DW / source.natW, DH = Math.max(60, Math.round(source.natH * sc));
      const cv = document.createElement("canvas"); cv.width = DW; cv.height = DH;
      const ctx = cv.getContext("2d", { willReadFrequently: true });
      ctx.drawImage(im, 0, 0, DW, DH);
      const det = detectStars(ctx.getImageData(0, 0, DW, DH).data, DW, DH, {}).map((s) => ({ x: s.x / sc, y: s.y / sc }));
      if (det.length < 6) { setFlash(`✦ only ${det.length} star(s) detected — too few to solve; a clearer night-sky frame is needed`); return; }
      /* full bright catalog above the horizon (independent of the display mag
         limit / star toggle) */
      const cat = STARS.map(([ra, dec, mag]) => { const p = raDecToAzEl(ra, dec, T, LAT, LNG); return { g: dirOf(p.az, p.alt), mag, alt: p.alt }; }).filter((c) => c.alt > 0);
      const fovGuess = isNum(source?.fovH) ? +source.fovH : 85; // no EXIF FOV on many night shots → mid guess + a wide search
      /* STRICT acceptance: only a genuinely TIGHT fit counts as a lock. A loose
         partial match (the wide/soft/hazy case) is declined honestly rather than
         presented as a false alignment. Wide fovFactors cover 40–120° from the
         mid guess so an ultra-wide lens is still searched. */
      /* Elevation prior: you reliably know HOW HIGH you looked (the position-step
         tilt) even when you can't recall which way you were rotated looking
         straight up — so anchor elevation near it and let the solver search all
         rotations. The az/roll are NEVER taken from your guess. */
      const elPrior = isNum(source?.mediaAim?.el) ? +source.mediaAim.el : (isNum(pEl) ? pEl : null);
      const strict = { minInl: 8, minMatch: 10, maxRms: 0.6, fovFactors: [0.5, 0.65, 0.8, 1.0, 1.2, 1.4], elPrior, elBand: 20 };
      await new Promise((r) => setTimeout(r, 20)); // let the flash paint before the solve blocks
      let sol = blindStarAlign(det, cat, source.natW, source.natH, fovGuess, strict);
      if (!sol) sol = autoStarAlign(det, cat, source.natW, source.natH, { az: pAz, el: pEl, roll: pRoll, fov: fovM, k: pDist }, { minMatch: 10, maxRms: 0.6 });
      if (!sol) { setFlash(`✦ couldn't confidently solve this frame (${det.length} points). Wide/soft/hazy night shots are hard to solve blind — tap ✦ align-to-star to set 2–3 named stars yourself, and raise brightness on the photo step to see them.`); return; }
      calibAnchorsRef.current = [];
      setPAz(sol.az); setPEl(clampN(sol.el, -20, EL_MAX));
      setPRoll(clampN(((sol.roll + 180) % 360 + 360) % 360 - 180, -90, 90));
      setFovM(clampN(sol.fov, 8, 135)); setPDist(sol.k);
      /* be honest about confidence — a wide phone lens + a bright-only catalog
         can yield a LOOSE partial fit; don't present that as a clean lock */
      const loose = sol.rms > 0.7 || sol.n < 12;
      setFlash(loose
        ? `✦ matched ${sol.n} stars, but the fit is loose (±${sol.rms.toFixed(1)}°) — toggle ★ stars ON and check they sit on the photo's stars; nudge/retry if it's off`
        : `✦ auto-aligned · ${sol.n} stars · fit ±${sol.rms.toFixed(2)}° · FOV ${sol.fov.toFixed(0)}° — toggle ★ stars to verify`);
    } catch (e) { setFlash("✦ auto-align failed on this image"); }
  };

  const handleClose = () => { if (photoOn) commitPlacement(); onClose(); };

  const aimColor = which === "B" ? "var(--teal)" : "var(--amber)";
  const recenter = (b) => { if (placing) { setPAz(b.az); setPEl(clampN(b.alt, -20, EL_MAX)); } else { setViewAz(b.az); setViewAlt(clampN(b.alt, -10, 80)); } };
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

        {/* stars & planets are drawn AFTER the photo (below) so they overlay it
            like the Sun/Moon anchors — see the sky-object layer past the grid */}

        {/* photo/video — Look mode: our own canvas mesh warp (static texture) */}
        {!placing && photoOn && (
          <canvas ref={warpRef} style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }} />
        )}
        {photoHidden && (
          <div style={{ position: "absolute", left: "50%", top: "calc(56px + env(safe-area-inset-top))", transform: "translateX(-50%)", background: "rgba(15,23,42,.75)", border: "1px solid var(--line)", borderRadius: 999, padding: "4px 12px", fontSize: 11, fontFamily: "var(--mono)", color: "var(--dim)", pointerEvents: "none" }}>
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
            {/* video shows the baked marked frame — the same still the warp uses */}
            {(() => {
              const imgSrc = source.mediaKind === "video" ? vidFrameUrl : source.mediaUrl;
              return imgSrc
                ? <img src={imgSrc} alt="" style={{ width: "100%", display: "block", opacity: PH_OP, filter: imgAdjFilter(source.imgAdj) }} />
                : <div style={{ width: "100%", aspectRatio: (source.natW && source.natH) ? `${source.natW} / ${source.natH}` : "16 / 9", background: "rgba(15,23,42,.6)", display: "flex", alignItems: "center", justifyContent: "center", color: "var(--dim)", fontSize: 11, fontFamily: "var(--mono)" }}>rendering frame…</div>;
            })()}
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
          {ridgePaths.map((r, i) => <path key={"rg" + i} d={r.d} fill="none" stroke={ridgeCol(r.o)} strokeWidth="1.15" strokeDasharray="7 4" vectorEffect="non-scaling-stroke" />)}
          {terrainPath && <path d={terrainPath} fill="none" stroke={ridgeCol(0.9)} strokeWidth="1.6" strokeDasharray="7 4" vectorEffect="non-scaling-stroke" />}
        </svg>
        {terrainLbl && (
          <div style={{ position: "absolute", left: (terrainLbl.x * 100) + "%", top: (terrainLbl.y * 100) + "%", transform: "translate(-50%,-130%)", fontSize: 8.5, fontFamily: "var(--mono)", fontWeight: 700, letterSpacing: ".14em", color: ridgeCol(0.95), textShadow: "0 1px 2px rgba(0,0,0,.8)", pointerEvents: "none" }}>TERRAIN</div>
        )}
        {/* named peaks on the skyline — el from the summit's own height, or the
            drawn ridge elevation when OSM has no `ele` tag */}
        {peakMarks.map((pk, i) => {
          const elv = pk.el != null ? pk.el : (terr?.els ? skylineElAt(terr.els, pk.az) : 0);
          const pr = project(pk.az, elv);
          if (!pr.inFront || pr.x < 0.01 || pr.x > 0.99 || pr.y < -0.02 || pr.y > 1.02) return null;
          return (
            <div key={"pk" + i} style={{ position: "absolute", left: (pr.x * 100) + "%", top: (pr.y * 100) + "%", transform: "translate(-50%,-100%)", textAlign: "center", pointerEvents: "none" }}>
              <div style={{ fontSize: 9, fontFamily: "var(--mono)", fontWeight: 700, color: ridgeCol(0.98), textShadow: "0 0 3px rgba(0,0,0,.95), 0 1px 2px rgba(0,0,0,.9)", whiteSpace: "nowrap" }}>
                {pk.name}{pk.eleM != null ? ` ${Math.round(pk.eleM)}m` : ""}
              </div>
              <div style={{ width: 0, height: 0, margin: "1px auto 0", borderLeft: "4px solid transparent", borderRight: "4px solid transparent", borderBottom: `6px solid ${ridgeCol(0.98)}` }} />
            </div>
          );
        })}
        {altLabels.map((p) => (
          <div key={"hl" + p.h} style={{ position: "absolute", left: (p.x * 100) + "%", top: (p.y * 100) + "%", transform: "translate(-50%,-50%)", fontSize: 9, fontFamily: "var(--mono)", color: gridColor.replace(/[\d.]+\)$/, "0.9)"), background: "rgba(7,11,20,.35)", borderRadius: 4, padding: "0 3px", pointerEvents: "none" }}>{p.h}°</div>
        ))}
        {cardinals.map((c, i) => (
          <div key={"cd" + i} style={{ position: "absolute", left: (c.x * 100) + "%", top: (c.y * 100) + "%", transform: "translate(-50%,-50%)", fontSize: 12, fontWeight: 800, color: "#fff", textShadow: "0 1px 3px rgba(0,0,0,.7)", pointerEvents: "none" }}>{c.lbl}</div>
        ))}

        {/* stars & planets — overlaid ON the photo (a bright halo behind each so
            they read against a twilight image; brighter stars glow more) */}
        {starDots.length > 0 && (
          <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }} preserveAspectRatio="none" viewBox="0 0 100 100">
            {starDots.map((p, i) => <circle key={"sh" + i} cx={p.x * 100} cy={p.y * 100} r={p.r * 0.5} fill="#bcd2ff" opacity={p.o * 0.35} />)}
            {starDots.map((p, i) => <circle key={"sc" + i} cx={p.x * 100} cy={p.y * 100} r={p.r * 0.24} fill="#fff" opacity={clampN(p.o + 0.1, 0, 1)} />)}
          </svg>
        )}
        {starLabels.map((p) => (
          /* labels are non-interactive; while aligning you TAP the sky (dome
             handles it) and the nearest named star is picked — so the many
             labels never become pan dead-zones */
          <div key={"sl" + p.name} style={{ position: "absolute", left: (p.x * 100) + "%", top: (p.y * 100) + "%", transform: "translate(7px,-5px)", fontSize: 9.5, fontFamily: "var(--mono)", fontWeight: calibAnchor?.name === p.name ? 800 : 600, color: calibAnchor?.name === p.name ? "var(--amber)" : "#eaf1ff", textShadow: "0 0 3px rgba(0,0,0,.95), 0 1px 2px rgba(0,0,0,.9)", pointerEvents: "none", whiteSpace: "nowrap" }}>{p.name}</div>
        ))}
        {planetDots.map((pl) => (
          /* dot sits EXACTLY on the planet's true az/el (translate -50%,-50%);
             the label floats below as a SEPARATE element so its height never
             pushes the marker off the true point (it did when both were stacked
             in one centered box — the marker landed ~7px high of Venus) */
          <React.Fragment key={"pl" + pl.name}>
            <div style={{ position: "absolute", left: (pl.p.x * 100) + "%", top: (pl.p.y * 100) + "%", transform: "translate(-50%,-50%)", width: 8, height: 8, borderRadius: "50%", background: "#fff6d8", boxShadow: "0 0 9px 3px rgba(255,225,150,.75)", pointerEvents: "none" }} />
            <div style={{ position: "absolute", left: (pl.p.x * 100) + "%", top: (pl.p.y * 100) + "%", transform: "translate(-50%,8px)", textAlign: "center", fontSize: 9.5, fontFamily: "var(--mono)", fontWeight: 700, color: calibAnchor?.name === pl.name ? "var(--amber)" : "#ffe9b0", textShadow: "0 0 3px rgba(0,0,0,.95), 0 1px 2px rgba(0,0,0,.9)", whiteSpace: "nowrap", pointerEvents: "none" }}>{pl.sym} {pl.name}</div>
          </React.Fragment>
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
            <div key={"sat" + s.name} style={{ position: "absolute", left: (pr.x * 100) + "%", top: (pr.y * 100) + "%", transform: "translate(-50%,-50%)", pointerEvents: "none" }}>
              {/* diamond sits ON the point (and its trail); label hangs below */}
              <div style={{ width: 5, height: 5, transform: "rotate(45deg)", background: col, boxShadow: s.lit ? "0 0 5px 1px rgba(159,220,255,.5)" : "none" }} />
              <div style={{ position: "absolute", top: "100%", left: "50%", transform: "translateX(-50%)", marginTop: 3, textAlign: "center", fontSize: 8.5, fontFamily: "var(--mono)", fontWeight: 700, color: col, textShadow: "0 1px 2px rgba(0,0,0,.85)", whiteSpace: "nowrap" }}>
                🛰 {s.name}{s.lit ? "" : " · in shadow"}<br />{Math.round(s.rangeKm)} km
              </div>
            </div>
          );
        })}

        {/* Starlink — small unlabeled dots (up to 60 sunlit); a fresh batch
            reads as a tight arc/train. Distinct violet so they don't read as
            the brighter visual-group satellites above. */}
        {slView.length > 0 && (
          <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }} preserveAspectRatio="none" viewBox="0 0 100 100">
            {slView.map((s, i) => {
              const pr = projectD(dirFromAzEl(s.az, s.el));
              if (!pr.inFront || pr.x < -0.03 || pr.x > 1.03 || pr.y < -0.03 || pr.y > 1.03) return null;
              return <circle key={"sl" + i} cx={pr.x * 100} cy={pr.y * 100} r="0.32" fill="#c9b6ff" opacity="0.9" />;
            })}
          </svg>
        )}

        {/* faint sky-tracks: each aircraft's path ±4 min (archive) or from the
           live polls — drawn for craft near what you're looking at OR near the
           sight-line, and always when selected. (Before Moment A there is no
           sight-line, so keying only off it left the dome with no tracks at
           all — now the current view direction is the fallback reference.) */}
        {(() => {
          if (!acView.length) return null;
          const sight = isNum(source?.A?.az) && isNum(source?.A?.el) ? dirFromAzEl(+source.A.az, +source.A.el) : null;
          const look = dirFromAzEl(effAz, effAlt);
          const near = (r, d) => r && Math.acos(clampN(dot(r, d), -1, 1)) * R2D <= 25;
          const lines = [];
          for (const v of acView) {
            const sel = v.a.hex === selHex;
            const raw = acData?.hist ? v.a.trail : liveTrailRef.current.get(v.a.hex);
            if (!raw || raw.length < 2) continue;
            if (!sel && !near(look, v.d) && !near(sight, v.d)) continue;
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
              style={{ position: "absolute", left: (pr.x * 100) + "%", top: (pr.y * 100) + "%", transform: "translate(-50%,-50%)", pointerEvents: "auto", cursor: "pointer", opacity: 0.94, padding: 6, zIndex: sel ? 6 : 5 }}>
              {/* glyph sits ON the point (and its sky-track); label hangs below */}
              <div style={{ fontSize: 13, color: col, transform: `rotate(${rot}deg)`, textShadow: "0 1px 3px rgba(0,0,0,.85)", lineHeight: 1 }}>✈</div>
              <div style={{ position: "absolute", top: "100%", left: "50%", transform: "translateX(-50%)", marginTop: 1, textAlign: "center", fontSize: 8.5, fontFamily: "var(--mono)", fontWeight: 700, color: col, textShadow: "0 1px 2px rgba(0,0,0,.85)", whiteSpace: "nowrap" }}>
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
                  /* TRUE apparent size: angular size → screen px via the LIVE FOV,
                     so the shape scales with the sky as you zoom in/out (a distant
                     object really is tiny — zoom in to see it; the rotation loupe
                     helps while it's small). Floor 1 px; tap target stays ≥32 px. */
                  /* a point with no explicit size defaults to the MEASURED object
                     size, so it scales with zoom correctly by default (a point
                     dropped before the object was measured otherwise rendered at a
                     fixed 12 px that ignored zoom). */
                  const angI = isNum(sortedTrack[idx]?.ang) ? +sortedTrack[idx].ang
                    : (objAngW != null && objAngW > 0 ? +objAngW : null);
                  const fpxS = (vp.w || window.innerWidth || 1) / (2 * tanH);
                  const dispPx = angI != null ? clampN(angI * D2R * fpxS, 1, 1400) : 12;
                  const hit = Math.max(dispPx + 22, 32);
                  const sfI = source.shapeFit ? { ...source.shapeFit, rotM: ptRotM(idx), roll: 0 } : null;
                  return (
                    <div key={"tj" + i}
                      onPointerDown={tappable ? (e) => e.stopPropagation() : undefined}
                      onClick={tappable ? (e) => { e.stopPropagation(); setSelPt(sel ? null : idx); setSelSeg(null); setRotMode(false); } : undefined}
                      style={{ position: "absolute", left: (p[0] * 100) + "%", top: (p[1] * 100) + "%", transform: "translate(-50%,-50%)", width: hit, height: hit, display: "flex", alignItems: "center", justifyContent: "center", pointerEvents: tappable ? "auto" : "none", cursor: tappable ? "pointer" : "default" }}>
                      {/* shape centered exactly on the point so the path line runs
                          THROUGH it; the number floats below without moving that center */}
                      <div style={{ position: "relative", filter: "drop-shadow(0 1px 2px rgba(0,0,0,.85))" }}>
                        <TrackObj sf={sfI} px={dispPx} color={col} />
                        <div style={{ position: "absolute", top: "100%", left: "50%", transform: "translateX(-50%)", fontSize: 9, fontFamily: "var(--mono)", fontWeight: 800, color: col, textShadow: "0 1px 2px rgba(0,0,0,.8)", marginTop: 1, whiteSpace: "nowrap" }}>{idx + 1}</div>
                      </div>
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
            <div style={{ position: "absolute", left: (pr.x * 100) + "%", top: (pr.y * 100) + "%", transform: "translate(-50%,-50%)", pointerEvents: "none", textAlign: "center", opacity: 1 }}>
              <div style={{ position: "relative", display: "inline-block" }}>
                {gPx < 6 && (
                  <div style={{ position: "absolute", left: "50%", top: "50%", width: 18, height: 18, margin: "-9px 0 0 -9px", border: "1px dashed rgba(255,255,255,.7)", borderRadius: "50%" }} />
                )}
                <GhostSil shape={g.shape} w={gPx} />
              </div>
              <div style={{ fontSize: 9, fontFamily: "var(--mono)", color: "#9fb4d8", textShadow: "0 1px 2px rgba(0,0,0,.8)", whiteSpace: "nowrap", marginTop: 2 }}>
                {g.name} ({g.m} m) @ {fmtLenShort(cmpD)}{gPx < 3 ? " · sub-pixel at this range" : ""}
              </div>
            </div>
          );
        })()}

        {/* aiming crosshair — fixed at screen center (hidden while placing) */}
        {pMode !== "place" && (
        <svg style={{ position: "absolute", left: "50%", top: "50%", transform: "translate(-50%,-50%)", pointerEvents: "none", overflow: "visible", opacity: 0.75 }} width="48" height="48" viewBox="-32 -32 64 64">
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
        <div style={{ position: "absolute", top: "calc(96px + env(safe-area-inset-top))", left: "50%", transform: "translateX(-50%)", zIndex: 220, maxWidth: "92%", textAlign: "center", background: "rgba(14,43,38,.92)", border: "1px solid #2A6157", color: "var(--teal)", borderRadius: 14, padding: "7px 16px", fontSize: 12, fontWeight: 700, lineHeight: 1.35, pointerEvents: "none" }}>
          {flash}
        </div>
      )}
      {calibMsg && (
        <div style={{ position: "absolute", top: "calc(124px + env(safe-area-inset-top))", left: "50%", transform: "translateX(-50%)", zIndex: 220, maxWidth: "90%", textAlign: "center", background: "rgba(43,34,14,.94)", border: "1px solid var(--amber)", color: "var(--amber)", borderRadius: 999, padding: "7px 16px", fontSize: 12, fontWeight: 700, fontFamily: "var(--mono)", pointerEvents: "none" }}>
          {calibMsg}
        </div>
      )}
      {calibOn && calibAnchor && (() => {
        const pr = project(calibAnchor.az, calibAnchor.el);
        return pr.inFront ? (
          <div style={{ position: "absolute", left: (pr.x * 100) + "%", top: (pr.y * 100) + "%", transform: "translate(-50%,-50%)", width: 26, height: 26, border: "2px solid var(--amber)", borderRadius: "50%", boxShadow: "0 0 8px 2px rgba(240,180,80,.6)", zIndex: 219, pointerEvents: "none" }} />
        ) : null;
      })()}

      {/* rotation loupe — a magnified view of the selected point's shape at its
          attitude, so you can judge/adjust orientation even when the on-dome
          icon is a couple of pixels (mirrors the shape loupe in image prep) */}
      {wizard && rotMode && selPt != null && source?.shapeFit && (
        <div style={{ position: "absolute", top: "calc(118px + env(safe-area-inset-top))", right: 12, zIndex: 220, background: "rgba(7,11,20,.82)", border: "1px solid var(--amber)", borderRadius: 12, padding: "8px 10px 6px", pointerEvents: "none", display: "flex", flexDirection: "column", alignItems: "center", gap: 4 }}>
          <div style={{ width: 118, height: 118, display: "flex", alignItems: "center", justifyContent: "center", filter: "drop-shadow(0 1px 2px rgba(0,0,0,.85))" }}>
            <TrackObj sf={{ ...source.shapeFit, rotM: ptRotM(selPt), roll: 0 }} px={94} color="var(--amber)" />
          </div>
          <div style={{ fontFamily: "var(--mono)", fontSize: 9, color: "var(--amber)", letterSpacing: ".08em" }}>PT {selPt + 1} ATTITUDE</div>
        </div>
      )}

      {/* top HUD — pad past the notch/Dynamic Island in the installed PWA */}
      <div style={{ position: "absolute", top: 0, left: 0, right: 0, padding: "calc(10px + env(safe-area-inset-top)) 12px 10px", pointerEvents: "none", zIndex: 210 }}>
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "flex-start", gap: 10 }}>
          {/* left: back + progress, matching the other wizard pages */}
          <div style={{ display: "flex", alignItems: "center", gap: 8, pointerEvents: "auto", flex: "0 0 auto" }}>
            <button className="btn sm" style={{ background: "rgba(15,23,42,.7)" }}
              onClick={() => { if (wizard) { if (photoOn) commitPlacement(); onWizardBack && onWizardBack(); } else handleClose(); }}>
              {wizard ? "‹ Back" : "✕ Close"}
            </button>
            {wizard && <WizDots n={3} style={{ background: "rgba(15,23,42,.7)", padding: "6px 8px", borderRadius: 6 }} />}
          </div>
          {/* right: readouts */}
          <div style={{ textAlign: "right", flex: "1 1 auto", minWidth: 0 }}>
            <div style={{ fontFamily: "var(--mono)", fontWeight: 800, fontSize: 20, color: aimColor, textShadow: "0 1px 4px rgba(0,0,0,.6)", whiteSpace: "nowrap" }}>
              {effAz.toFixed(1)}° <span style={{ fontSize: 12, color: "#fff" }}>{compass8(effAz)}</span> · {effAlt.toFixed(1)}°<span style={{ fontSize: 12, color: "#fff" }}> up</span>
            </div>
            <div style={{ fontSize: 10, letterSpacing: ".14em", textTransform: "uppercase", fontWeight: 700, color: "rgba(255,255,255,.75)", textShadow: "0 1px 3px rgba(0,0,0,.6)", whiteSpace: "nowrap" }}>
              {pMode === "place" ? "Placing photo" : `Aiming moment ${which}`} · FOV {Math.round(effFov)}°
            </div>
          </div>
        </div>
        {/* sky-layer toggles — one row (terrain is always on) */}
        <div style={{ display: "flex", gap: 6, marginTop: 8, pointerEvents: "auto", flexWrap: "wrap" }}>
          {sun.alt > -1 && <button className="btn sm" title={`Sun ${fmtBody(sun)} — tap to center`} style={{ background: "rgba(15,23,42,.7)", padding: "6px 9px" }} onClick={() => recenter(sun)}>☀</button>}
          {moon.alt > -1 && <button className="btn sm" title={`Moon ${fmtBody(moon)} · ${Math.round(moon.frac * 100)}% lit — tap to center`} style={{ background: "rgba(15,23,42,.7)", padding: "6px 9px" }} onClick={() => recenter(moon)}>☾</button>}
          {hasPos && (
            <button className="btn sm" style={{ background: "rgba(15,23,42,.7)", color: !acOn ? "var(--dim)" : (acData?.ac && wantHist && !acData.hist) ? "var(--amber)" : "var(--track)" }}
              onClick={() => setAcOn((v) => !v)}>
              ✈ {acOn ? (acData?.ac ? `${acView.length}${acData.hist ? " @ sighting" : " live"}` : acData?.err ? "?" : "…") : "off"}
            </button>
          )}
          <button className="btn sm" title="Stars & planets: auto on at night, off by day — tap to force on/off"
            style={{ background: "rgba(15,23,42,.7)", padding: "6px 9px", color: starMode === "off" ? "var(--dim)" : (starMode === "on" || limMag > -4) ? "#dfe8ff" : "var(--dim)" }}
            onClick={() => setStarMode((m) => (m === "auto" ? "on" : m === "on" ? "off" : "auto"))}>
            ★
          </button>
          {hasPos && (
            <button className="btn sm" title="Satellites (CelesTrak visual group, SGP4 at the sighting time): auto shows when dark; on forces"
              style={{ background: "rgba(15,23,42,.7)", color: satMode === "off" ? "var(--dim)" : satView.length ? "#9fdcff" : "var(--dim)" }}
              onClick={() => setSatMode((m) => (m === "auto" ? "on" : m === "on" ? "off" : "auto"))}>
              🛰 {satMode === "off" ? "off" : satDb?.err ? "?" : satsWanted && !satDb ? "…" : `${satView.length}${satMode === "auto" ? "" : " on"}`}
            </button>
          )}
          {hasPos && (
            <button className="btn sm" title="Starlink — the full constellation (opt-in). Sunlit Starlinks above the horizon at the sighting time; a fresh batch appears as a tight train."
              style={{ background: "rgba(15,23,42,.7)", color: !starlinkOn ? "var(--dim)" : slDb?.err ? "var(--amber)" : "#c9b6ff" }}
              onClick={() => setStarlinkOn((v) => !v)}>
              ✦ {starlinkOn ? (slDb?.err ? "?" : !slDb ? "…" : `Starlink ${slView.length}`) : "Starlink"}
            </button>
          )}
          {hasPos && (
            <button className="btn sm" title="Named peaks (OpenStreetMap) placed on the terrain skyline — a labeled summit on the horizon is also a compass check"
              style={{ background: "rgba(15,23,42,.7)", color: !peaksOn ? "var(--dim)" : peaks?.err ? "var(--amber)" : "rgba(158,224,138,0.95)" }}
              onClick={() => setPeaksOn((v) => !v)}>
              ⛰ {peaksOn ? (peaks?.err ? "?" : !peaks ? "…" : `peaks ${peakMarks.length}`) : "peaks"}
            </button>
          )}
        </div>
        {satView.length > 0 && satStaleDays > 5 && (
          <div style={{ fontSize: 10, color: "var(--amber)", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 4 }}>
            🛰 TLE epoch ≈ {satStaleDays} d from the sighting — satellite positions degrade; treat as approximate
          </div>
        )}
        {peaksOn && peaks?.err && (
          <div style={{ fontSize: 10, color: "var(--amber)", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 4 }}>
            ⛰ peaks unavailable — {peaks.err}
          </div>
        )}
        {peaksOn && Array.isArray(peaks) && peaks.length === 0 && (
          <div style={{ fontSize: 10, color: "var(--dim)", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 4 }}>
            ⛰ no named peaks or hills within 120 km of {LAT.toFixed(3)}, {LNG.toFixed(3)}
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

      {/* view zoom — vertical stack on the right, out of the cramped bottom bar */}
      {pMode !== "place" && (
        <div style={{ position: "absolute", right: 10, top: "50%", transform: "translateY(-50%)", display: "flex", flexDirection: "column", gap: 6, zIndex: 205, pointerEvents: "auto" }}>
          <button className="btn" style={{ width: 42, height: 42, padding: 0, fontSize: 19, background: "rgba(15,23,42,.75)" }} onClick={() => setFov((f) => clampN(+(f * 0.72).toFixed(1), 2, 90))}>+</button>
          <button className="btn" style={{ width: 42, height: 42, padding: 0, fontSize: 19, background: "rgba(15,23,42,.75)" }} onClick={() => setFov((f) => clampN(+(f / 0.72).toFixed(1), 2, 90))}>−</button>
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
                    <button className="btn sm amber" onClick={autoAlign} title="Auto star-align: detects the stars in the photo and matches their pattern to the sky to solve the exact pose (az/el/roll/FOV/lens) — no need to line it up first. Needs correct date/time & location.">✦ Auto star-align</button>
                    {terr?.els && <button className="btn sm teal" onClick={snapToRidges}>⛰ Snap to ridges</button>}
                    <button className="btn sm" onClick={() => setPRoll(0)}>⟺ Level</button>
                    <button className="btn sm" onClick={() => {
                      const p0 = openPoseRef.current;
                      if (p0) { setPAz(p0.az); setPEl(p0.el); setPRoll(p0.roll); setFovM(p0.fov); setPDist(p0.dist || 0); }
                      else { setFovM(isNum(source?.fovH) ? +source.fovH : 68); setPRoll(0); setPDist(0); }
                      calibAnchorsRef.current = []; setCalibCount(0);
                    }}>Reset placement</button>
                    {terrOn && terr?.els && (
                      <span style={{ display: "inline-flex", alignItems: "center", gap: 6 }} title="Recolor the terrain & ridge lines so they stand out over your photo">
                        <span className="microlabel" style={{ marginBottom: 0 }}>ridge</span>
                        <input type="range" min={0} max={360} step={2} value={ridgeHue} onChange={(e) => setRidgeHue(+e.target.value)} style={{ width: 70 }} />
                        <span style={{ width: 13, height: 13, borderRadius: 7, flex: "0 0 auto", background: ridgeCol(0.95), border: "1px solid rgba(255,255,255,.3)" }} />
                      </span>
                    )}
                  </>
                )}
                {pMode !== "place" && skyRefs.length > 0 && (
                  <button className={"btn sm" + (calibOn ? " amber" : "")} title="Align the sky to a known star or planet: pick the object, aim the crosshair on it in the photo, press ✓ Set. Solves the lens FOV + roll and keeps the terrain match."
                    onClick={() => { const n = !calibOn; setCalibOn(n); setCalibAnchor(null); setCalibMsg(n ? "👆 Tap a named star or planet in the sky" : ""); }}>
                    ✦ {calibOn ? "done aligning" : "align to star"}
                  </button>
                )}
                {pMode !== "place" && calibApplied && !calibOn && (
                  <button className="btn sm" onClick={resetCalib} title="Undo the star alignment — restore the lens FOV & roll">↺ align</button>
                )}
                {source.mediaKind === "video" && (
                  <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)" }}>🎞 frame {isNum(source?.A?.videoTime) ? (+source.A.videoTime).toFixed(2) + "s" : "start"} (locked — set it on the measure step)</span>
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
        {pMode !== "place" && calibOn && (
          <div style={{ marginBottom: 8, background: "rgba(43,34,14,.6)", border: "1px solid var(--amber)", borderRadius: 10, padding: "8px 10px" }}>
            {!calibAnchor ? (
              <div style={{ fontSize: 11, color: "var(--amber)", fontFamily: "var(--mono)", lineHeight: 1.5 }}>
                👆 <b>Tap a named star or planet in the sky</b> to align to it{calibCount > 0 ? ` (${calibCount} set` + (calibCount === 1 ? " — add a 2nd for lens distortion)" : calibCount === 2 ? " — add a 3rd for a full plate-solve)" : ")") : ""}. Pan/zoom to bring more into view.
              </div>
            ) : (
              <div style={{ display: "flex", gap: 6, flexWrap: "wrap", alignItems: "center" }}>
                <span style={{ fontSize: 11, color: "var(--amber)", fontFamily: "var(--mono)" }}>Aim the ⊕ crosshair onto <b>{calibAnchor.name}</b> in the photo (the ◯ ring marks where it's predicted), then Set. Nothing changes until you Set.</span>
                <button className="btn sm amber" onClick={alignAtCrosshair}>✓ Set {calibAnchor.name}</button>
                <button className="btn sm" onClick={() => { setCalibAnchor(null); setCalibMsg("👆 Tap a named star or planet in the sky"); }}>✕ cancel</button>
              </div>
            )}
            {calibApplied && (
              <div style={{ marginTop: 6, display: "flex", gap: 8, alignItems: "center", flexWrap: "wrap" }}>
                <button className="btn sm" onClick={resetCalib} title="Undo the star alignment — restore the lens FOV, roll & distortion">↺ reset alignment</button>
                {calibCount >= 2 && <span style={{ fontSize: 10, color: "var(--dim)", fontFamily: "var(--mono)" }}>lens {pDist >= 0 ? "+" : ""}{pDist.toFixed(3)}</span>}
              </div>
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
              <button className={"btn sm" + (sizeOn ? " teal" : "")} title="Object size vs distance" onClick={() => setSizeOn((v) => !v)}>📏 size</button>
              <button className={"btn sm" + (cmpOn ? " teal" : "")} title="Compare to a reference object (balloon, drone, aircraft)" onClick={() => {
                setCmpOn((v) => !v);
                if (!cmpOn) setCmpPos({ az: viewAz, el: clampN(viewAlt, -10, 85) }); // drop at the crosshair
              }}>⚖ compare</button>
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
              const deletePt = () => { update(syncAB(sortedTrack.filter((_, i) => i !== selPt))); setSelPt(null); setSelSeg(null); setRotMode(false); };
              return (
                <div style={{ marginTop: 6, background: "rgba(15,23,42,.55)", border: "1px solid var(--line)", borderRadius: 10, padding: "8px 10px" }}>
                  <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: interior ? 6 : 0 }}>
                    <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)", flex: 1 }}>
                      Point {selPt + 1}{interior ? " — how tight was the turn?" : selPt === 0 ? " (path start)" : " (path end)"}
                    </span>
                    <button className="btn sm" style={{ color: "var(--red)", borderColor: "#5A2C24" }} onClick={deletePt}>🗑 Delete</button>
                    <button className="btn sm teal" onClick={() => { setSelPt(null); setRotMode(false); }}>✓ Done</button>
                  </div>
                  {(() => {
                    /* apparent SIZE at this point → captures radial (closer/farther) motion.
                       Anchored on the measured object size; range shown relative to point 1. */
                    const angRef0 = isNum(sortedTrack[0]?.ang) ? +sortedTrack[0].ang : (objAngW != null ? +objAngW : null);
                    const pAng = isNum(sortedTrack[selPt].ang) ? +sortedTrack[selPt].ang : (objAngW != null ? +objAngW : null);
                    if (angRef0 == null || pAng == null) return (
                      <div style={{ fontSize: 10, color: "var(--amber)", margin: "6px 0 2px", lineHeight: 1.5 }}>
                        Fit a shape (or mark the object's width on the photo in step 1) to size it here — resizing each point is what captures closer/farther motion.
                      </div>
                    );
                    const anchor = objAngW != null ? +objAngW : angRef0;
                    /* slider floor = the angular size that renders as 1 px at MAX
                       zoom (FOV 2°) — smaller than that is meaningless, so the
                       slider (and −) stop there. Ceiling: 40× the measured size. */
                    const fpxSmax = (vp.w || window.innerWidth || 400) / (2 * Math.tan(1 * D2R));
                    const angMin = 1 / (D2R * fpxSmax);
                    const angMax = clampN(anchor * 40, angMin * 8, 60);
                    const setAng = (a) => update({ track: sortedTrack.map((p, i) => (i === selPt ? { ...p, ang: +clampN(a, angMin, 60).toFixed(5) } : p)) });
                    const sv = clampN(Math.log(pAng / angMin) / Math.log(angMax / angMin), 0, 1);
                    const rho = Math.tan(angRef0 * D2R / 2) / Math.tan(pAng * D2R / 2);
                    return (
                      <div style={{ marginTop: interior ? 6 : 8 }}>
                        <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                          <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)" }}>apparent size · bigger = closer</span>
                          <span style={{ marginLeft: "auto", fontFamily: "var(--mono)", fontSize: 11, color: "var(--track)" }}>
                            {selPt === 0 ? "range reference" : `≈ ${rho.toFixed(2)}× pt 1${rho < 0.97 ? " · closer" : rho > 1.03 ? " · farther" : ""}`}
                          </span>
                        </div>
                        <div style={{ display: "flex", alignItems: "center", gap: 6, marginTop: 4 }}>
                          <button className="btn sm" onClick={() => setAng(pAng / 1.12)}>−</button>
                          <input type="range" min={0} max={1} step={0.005} value={sv}
                            onPointerDown={(e) => e.stopPropagation()} onTouchStart={(e) => e.stopPropagation()}
                            onChange={(e) => setAng(angMin * Math.pow(angMax / angMin, +e.target.value))}
                            style={{ flex: 1, touchAction: "auto", pointerEvents: "auto" }} />
                          <button className="btn sm" onClick={() => setAng(pAng * 1.12)}>+</button>
                          {selPt !== 0 && <button className="btn sm" title="same range as point 1" onClick={() => setAng(angRef0)}>= pt1</button>}
                        </div>
                      </div>
                    );
                  })()}
                  {source?.shapeFit && (
                    <div style={{ marginTop: 8 }}>
                      <div style={{ display: "flex", alignItems: "center", gap: 6, flexWrap: "wrap" }}>
                        <button className={"btn sm" + (rotMode ? " amber" : "")} onClick={() => setRotMode((v) => !v)}>🔄 Rotate</button>
                        {rotMode && <>
                          <button className="btn sm" title="roll left" onClick={() => setPtRot(selPt, mul3(rotZ3(-15), ptRotM(selPt)))}>⟲</button>
                          <button className="btn sm" title="roll right" onClick={() => setPtRot(selPt, mul3(rotZ3(15), ptRotM(selPt)))}>⟳</button>
                        </>}
                        {Array.isArray(sortedTrack[selPt]?.rotM) && <button className="btn sm" onClick={() => resetPtRot(selPt)}>reset</button>}
                        {Array.isArray(sortedTrack[selPt]?.rotM) && !rotMode && (
                          <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--track)" }}>oriented · {(sortedTrack[selPt].projF ?? 1).toFixed(2)}× broadside</span>
                        )}
                      </div>
                      {rotMode && <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 4, lineHeight: 1.5 }}>Drag the sky to tumble point {selPt + 1}'s shape; ⟲ ⟳ to roll. Matching its true attitude removes foreshortening from the range — worthwhile for an elongated object, pointless for an orb.</div>}
                    </div>
                  )}
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
                <div style={{ fontSize: 10, letterSpacing: ".12em", textTransform: "uppercase", fontWeight: 700, color: "var(--track)", marginBottom: 4 }}>Compare to a reference object</div>
                <div style={{ display: "flex", gap: 4, flexWrap: "wrap" }}>
                  {GHOSTW.map((g, i) => (
                    <button key={i} className={"btn sm" + (ghostIdx === i ? " teal" : "")}
                      style={{ padding: "4px 8px", fontSize: 11, flex: "1 1 0", minWidth: 0, whiteSpace: "nowrap" }} onClick={() => {
                      setGhostIdx(i);
                      /* pull the distance into THIS object's meaningful band —
                         a drone at an airliner's 10 km is an invisible dot */
                      const lo = Math.max(5, g.m * 10), hi = Math.min(120000, g.m * 3000);
                      setCmpD((d) => {
                        const dd = clampN(d, lo, hi);
                        const ang = 2 * Math.atan(g.m / (2 * dd)) * R2D;
                        return (ang < 0.08 || ang > 8) ? Math.round(g.m * 115) /* ≈0.5° — clearly visible */ : Math.round(dd);
                      });
                    }}>{g.short}</button>
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
              </div>
            )}
            {sizeOn && (
              <div style={{ marginTop: 6 }}>
                <div style={{ fontSize: 10, letterSpacing: ".12em", textTransform: "uppercase", fontWeight: 700, color: "var(--amber)", marginBottom: 4 }}>Object size vs distance</div>
                {objAngW != null ? (() => {
                  /* the measured object: sweep assumed distance, read implied size AND
                     altitude above the observer (D·sin(el)) live */
                  const t = clampN(Math.log(objD / 30) / Math.log(80000 / 30), 0, 1); // floor 30 m ≈ 100 ft
                  const size = 2 * objD * Math.tan(objAngW * D2R / 2);
                  const objEl = isNum(source?.A?.el) ? +source.A.el : effAlt;
                  const alt = isNum(objEl) ? objD * Math.sin(objEl * D2R) : null;
                  return (
                    <>
                      <div style={{ fontFamily: "var(--mono)", fontSize: 11, color: "var(--amber)" }}>measured {objAngW.toFixed(2)}° wide{isNum(objEl) ? ` · ${objEl.toFixed(0)}° up` : ""}</div>
                      <input type="range" min={0} max={1} step={0.004} value={t}
                        onPointerDown={(e) => e.stopPropagation()} onTouchStart={(e) => e.stopPropagation()}
                        onChange={(e) => setObjD(Math.round(30 * Math.pow(80000 / 30, +e.target.value)))}
                        style={{ width: "100%", marginTop: 4, touchAction: "auto", pointerEvents: "auto" }} />
                      <div style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--amber)" }}>
                        if it was <b>{fmtLenShort(objD)}</b> away → <b>{fmtLenShort(size)}</b> across
                        {alt != null && <> · <b>{fmtLenShort(Math.abs(alt))}</b> {alt >= 0 ? "above" : "below"} you</>}
                        <span style={{ color: "var(--dim)" }}> · nearest: {REF_OBJECTS.reduce((b, o) => Math.abs(Math.log(o.size / size)) < Math.abs(Math.log(b.size / size)) ? o : b).name}</span>
                      </div>
                    </>
                  );
                })() : (
                  <div style={{ fontSize: 11, color: "var(--dim)" }}>Mark the object's width first (fit a shape, or set the two size marks on the photo) — then slide an assumed distance to read its true size.</div>
                )}
              </div>
            )}
          </div>
        )}
        <div style={{ fontSize: 9.5, lineHeight: 1.3, color: "rgba(255,255,255,.6)", marginTop: 2, marginBottom: 6 }}>
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
function PinMap({ lat, lon, origin, others, onChange, bearing, tilt }) {
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
        <div className="map-north">N ↑</div>
        {isNum(bearing) && (() => {
          /* which way you were looking — north-up map, so screen rotation = bearing.
             A steep sight-line projects SHORT onto the ground (cos foreshortening),
             so the ray shrinks toward the pin as you aim up; a dot rising at the pin
             cues "looking up out of the map" and grows to dominate near the zenith. */
          const t = clampN(isNum(tilt) ? +tilt : 0, -20, 90);
          const upF = Math.max(0, Math.sin(t * D2R));        // 0 at/below horizon → 1 straight up
          const grF = Math.max(0.14, Math.cos(t * D2R));     // ground foreshortening (keep a stub)
          const len = 78 * grF;
          return (
            <>
              <svg className="pinmap-ray" width="0" height="0" style={{ transform: `rotate(${((+bearing % 360) + 360) % 360}deg)`, opacity: 0.55 + 0.45 * grF }}>
                <line x1="0" y1="0" x2="0" y2={-len} stroke="#5FD3BC" strokeWidth="2.5" />
                <polygon points={`0,${-(len + 12)} -6,${-len} 6,${-len}`} fill="#5FD3BC" />
              </svg>
              {upF > 0.03 && (
                <svg className="pinmap-ray" width="0" height="0" style={{ transform: "none" }}>
                  <circle cx="0" cy="0" r={4 + upF * 13} fill="none" stroke="#5FD3BC" strokeWidth="2" opacity={0.3 + 0.55 * upF} />
                  <circle cx="0" cy="0" r={1.5 + upF * 5} fill="#5FD3BC" opacity={0.5 + 0.5 * upF} />
                </svg>
              )}
            </>
          );
        })()}
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
  const [q, setQ] = useState("");
  const [places, setPlaces] = useState(null); // [{lat,lon,name}] | {err} | null
  const [findBusy, setFindBusy] = useState(false);
  const posDone = isNum(src.lat) && isNum(src.lon);
  /* Viewing direction = the placement-center azimuth, held in mediaAim.az —
     the SAME field the sky view uses for photo placement. Both screens read
     and write it, so they always mirror: change the ray here and the sky view
     opens aimed there; rotate the placement in the sky view and (on commit)
     the ray here follows. Auto-seeded from the EXIF compass at load. */
  const bearing = isNum(src.mediaAim?.az) ? +src.mediaAim.az
    : (isNum(src.meta?.azTrue) ? +src.meta.azTrue
      : (isNum(src.meta?.az) ? +src.meta.az
        : (isNum(src.A?.az) ? +src.A.az : null)));
  const setBearing = (deg) => {
    const b = ((+deg % 360) + 360) % 360;
    const old = isNum(src.mediaAim?.az) ? +src.mediaAim.az : b;
    const d = b - old; // repointing rotates the whole placement in azimuth — the sight-lines ride along
    const rot = (a) => ((((+a + d) % 360) + 360) % 360);
    const patch = { mediaAim: { az: +b.toFixed(2), el: src.mediaAim?.el ?? 15, roll: src.mediaAim?.roll ?? 0 } };
    if (isNum(src.A?.az)) patch.A = { ...src.A, az: rot(src.A.az).toFixed(1) };
    if (isNum(src.B?.az)) patch.B = { ...src.B, az: rot(src.B.az).toFixed(1) };
    update(patch);
  };
  /* how high you were looking — the ONE piece the metadata can't give us (iOS
     hides camera pitch in a proprietary MakerNote), so a straight-up night-sky
     shot otherwise starts at the default 15° near the horizon. This seeds
     mediaAim.el directly; the sky view opens at this elevation. */
  const tilt = isNum(src.mediaAim?.el) ? +src.mediaAim.el : 15;
  const setTilt = (deg) => {
    const el = clampN(+deg, -20, 89.5); // cap just below the zenith — the sky view's gimbal limit
    update({ mediaAim: { az: isNum(src.mediaAim?.az) ? +src.mediaAim.az : (isNum(bearing) ? bearing : 0), el: +el.toFixed(1), roll: src.mediaAim?.roll ?? 0 } });
  };
  /* forward geocode by place name so no one has to source coordinates
     elsewhere — Nominatim/OSM, CORS-open, no key. Pin the map afterward
     to refine to the exact standing spot. */
  const searchPlace = async () => {
    const query = q.trim();
    if (!query) return;
    setFindBusy(true); setPlaces(null);
    const hits = [];
    /* US street addresses resolve to the actual house via the Census
       geocoder (TIGER/parcel) — Nominatim usually only knows the road, so
       "5101 Caves Hwy" lands at the start of the highway. Free, no key. If
       it's CORS-blocked or the query isn't a US address it just throws and
       we fall through to Nominatim (which also covers landmarks / non-US). */
    if (/\d/.test(query)) {
      try {
        const cr = await fetch("https://geocoding.geo.census.gov/geocoder/locations/onelineaddress?benchmark=Public_AR_Current&format=json&address=" + encodeURIComponent(query));
        if (cr.ok) {
          const cj = await cr.json();
          (cj?.result?.addressMatches || []).forEach((mm) => hits.push({ lat: +mm.coordinates.y, lon: +mm.coordinates.x, name: mm.matchedAddress, precise: true }));
        }
      } catch (e) { /* fall back to Nominatim */ }
    }
    try {
      const r = await fetch("https://nominatim.openstreetmap.org/search?format=jsonv2&addressdetails=1&limit=6&q=" + encodeURIComponent(query), { headers: { Accept: "application/json" } });
      if (r.ok) {
        const j = await r.json();
        (Array.isArray(j) ? j : []).forEach((p) => hits.push({ lat: +p.lat, lon: +p.lon, name: p.display_name, precise: !!(p.address && p.address.house_number) }));
      }
    } catch (e) { /* handled by the empty-hits check below */ }
    const clean = hits.filter((p) => isNum(p.lat) && isNum(p.lon));
    if (!clean.length) setPlaces({ err: "No match — try a nearby town or landmark, paste coordinates, or drag the pin below." });
    else setPlaces(clean);
    setFindBusy(false);
  };
  const pickPlace = (p) => { update({ lat: p.lat.toFixed(6), lon: p.lon.toFixed(6) }); setPlaces(null); setQ(p.name.split(",").slice(0, 2).join(",").trim()); };
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
            <ML>Find your spot by name</ML>
            <div style={{ display: "flex", gap: 6, alignItems: "stretch" }}>
              <input value={q} onChange={(e) => setQ(e.target.value)}
                onKeyDown={(e) => { if (e.key === "Enter") { e.preventDefault(); searchPlace(); } }}
                placeholder="town, address, or landmark — e.g. Grants Pass, OR"
                style={{ flex: 1, minWidth: 0 }} />
              <button className="btn sm teal" onClick={searchPlace} disabled={findBusy || !q.trim()}>{findBusy ? "…" : "🔎 Search"}</button>
            </div>
            {Array.isArray(places) && places.length > 0 && (
              <div style={{ marginTop: 6, border: "1px solid var(--line)", borderRadius: 10, overflow: "hidden" }}>
                {places.map((p, i) => (
                  <button key={i} onClick={() => pickPlace(p)}
                    style={{ display: "block", width: "100%", textAlign: "left", padding: "7px 10px", background: "transparent", border: "none", borderTop: i ? "1px solid var(--line)" : "none", color: "var(--ink)", fontSize: 12, cursor: "pointer" }}>
                    <span style={{ color: "var(--teal)" }}>📍</span> {p.name}
                    <span style={{ color: "var(--dim)", fontFamily: "var(--mono)", fontSize: 10 }}>  {p.lat.toFixed(4)}, {p.lon.toFixed(4)}</span>
                    <span style={{ fontSize: 10, fontWeight: 700, color: p.precise ? "var(--teal)" : "var(--amber)" }}>  {p.precise ? "· exact address" : "· road/area — drag pin to your spot"}</span>
                  </button>
                ))}
              </div>
            )}
            {places && places.err && <div className="warn">{places.err}</div>}
            <div style={{ fontSize: 10, color: "var(--dim)", margin: "3px 0 7px" }}>OpenStreetMap · Nominatim — then drag the pin to your exact spot.</div>
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
            <div style={{ marginTop: 6, display: "flex", gap: 8, alignItems: "center", flexWrap: "wrap" }}>
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
            {demMsg && <div style={{ fontSize: 11, color: demMsg.startsWith("✓") ? "var(--teal)" : "var(--red)", marginTop: 5 }}>{demMsg}</div>}
            {geoErr && <div className="warn">{geoErr}</div>}
            <div style={{ marginTop: 8 }}>
              <ML>Sighting date &amp; time</ML>
              <input type="datetime-local" value={toLocalInput(new Date(isNum(src.whenMs) ? +src.whenMs : Date.now()))}
                onChange={(e) => { const t = new Date(e.target.value).getTime(); if (!isNaN(t)) update({ whenMs: t }); }} />
            </div>
            {posDone && (
              <div style={{ marginTop: 8 }}>
                <PinMap lat={+src.lat} lon={+src.lon}
                  origin={src.meta && isNum(src.meta.lat) ? { lat: +src.meta.lat, lon: +src.meta.lon } : null}
                  others={others} bearing={bearing} tilt={tilt}
                  onChange={(la, lo) => update({ lat: la.toFixed(6), lon: lo.toFixed(6) })} />
                {/* aim sliders live UNDER the map so you set the ray you can see */}
                <div style={{ marginTop: 6 }}>
                  <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
                    <ML style={{ marginBottom: 1 }}>Viewing direction</ML>
                    <span style={{ color: "var(--teal)", fontFamily: "var(--mono)", fontSize: 11 }}>{isNum(bearing) ? `${Math.round(bearing)}° ${compass8(bearing)}` : "drag to set"}</span>
                  </div>
                  <input type="range" min={0} max={359} step={1} value={isNum(bearing) ? bearing : 0} onChange={(e) => setBearing(+e.target.value)} />
                </div>
                <div style={{ marginTop: 2 }}>
                  <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
                    <ML style={{ marginBottom: 1 }}>How high you looked</ML>
                    <span style={{ color: "var(--teal)", fontFamily: "var(--mono)", fontSize: 11 }}>{Math.round(tilt)}° {tilt >= 75 ? "straight up" : tilt <= 5 ? "horizon" : "up"}</span>
                  </div>
                  <input type="range" min={-20} max={90} step={1} value={tilt} onChange={(e) => setTilt(+e.target.value)} />
                  <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 1 }}>Photo metadata has no up/down angle — set 90° for straight up; fine-tune later in Place mode.</div>
                </div>
              </div>
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
  return <div className="plotwrap"><div ref={boxRef} style={{ position: "absolute", inset: 0 }} /><div className="map-north">N ↑</div></div>;
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
/* pull one STORE-method entry's text out of a zip (the reader half of makeZip
   — the share bundle is uncompressed, so no inflate needed). Scans local file
   headers; returns the named entry decoded as UTF-8, or null. */
function unzipEntryText(u8, name) {
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
            /* crop rect in the SCALED (post-k) coord frame — matches the
               packed shapeFit / shapeProjNat, so the report can lay the shape
               overlay over this zoomed crop with an SVG viewBox */
            r.detailCrop = { x: cx0 * k, y: cy0 * k, w: cw * k, h: ch * k };
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

/* print-friendly top-down plot for the report: observers, rays, fix,
   trajectory — pure SVG string, self-contained, no tiles */
function plotGeom(fix, traj) {
  const pts = [...fix.obs.map((o) => o.P), fix.solA.X];
  if (fix.motion?.XB) pts.push(fix.motion.XB);
  if (traj) for (const p of traj) pts.push(p);
  let minE = 1e12, maxE = -1e12, minN = 1e12, maxN = -1e12;
  for (const p of pts) { minE = Math.min(minE, p[0]); maxE = Math.max(maxE, p[0]); minN = Math.min(minN, p[1]); maxN = Math.max(maxN, p[1]); }
  const span = Math.max(maxE - minE, maxN - minN, 100) * 1.35;
  const cE = (minE + maxE) / 2, cN = (minN + maxN) / 2;
  const W = 560, H = 420, s = Math.min(W, H) / span;
  const px = (p) => [(W / 2 + (p[0] - cE) * s).toFixed(1), (H / 2 - (p[1] - cN) * s).toFixed(1)];
  return { W, H, cE, cN, s, span, px };
}

/* Esri World Imagery basemap for the report plot, composited to a canvas and
   returned as a data URI so the report stays self-contained (viewable offline,
   shareable as a file). North-up Web-Mercator aligns with the ENU plot at
   these scales. Returns null on any failure (unreachable / CORS-tainted) so
   the plot falls back to the plain background — never blocks the report. */
async function satBasemap(fix, G) {
  try {
    if (!fix?.ref || typeof document === "undefined") return null;
    const c = geoFromEnu([G.cE, G.cN, 0], fix.ref);
    const latR = c.lat * D2R, pow = (z) => Math.pow(2, z);
    const plotMpp = 1 / G.s;
    let z = clampN(Math.round(Math.log2(156543.03392 * Math.cos(latR) / plotMpp)), 1, 19);
    const res = 156543.03392 * Math.cos(latR) / pow(z);   // m per mercator px
    const k = G.s * res;                                   // SVG px per mercator px
    const worldPx = (la, lo) => {
      const sinL = Math.sin(la * D2R);
      return [(lo + 180) / 360 * 256 * pow(z),
        (0.5 - Math.log((1 + sinL) / (1 - sinL)) / (4 * Math.PI)) * 256 * pow(z)];
    };
    const [mcx, mcy] = worldPx(c.lat, c.lon);
    const halfW = (G.W / 2) / k * 1.06, halfH = (G.H / 2) / k * 1.06;
    const x0 = Math.floor((mcx - halfW) / 256), x1 = Math.floor((mcx + halfW) / 256);
    const y0 = Math.floor((mcy - halfH) / 256), y1 = Math.floor((mcy + halfH) / 256);
    if ((x1 - x0 + 1) * (y1 - y0 + 1) > 36) return null; // safety cap
    const nT = pow(z), cw = (x1 - x0 + 1) * 256, ch = (y1 - y0 + 1) * 256;
    const cv = document.createElement("canvas"); cv.width = cw; cv.height = ch;
    const ctx = cv.getContext("2d");
    /* same-origin proxy (server/index.mjs) → no CORS taint on toDataURL;
       any 404s (server not running) fall through to the plain plot */
    const loadImg = (src) => new Promise((r) => { const im = new Image(); im.crossOrigin = "anonymous"; im.onload = () => r(im); im.onerror = () => r(null); im.src = src; });
    const jobs = [];
    for (let tx = x0; tx <= x1; tx++) for (let ty = y0; ty <= y1; ty++) {
      if (ty < 0 || ty >= nT) continue;
      const wx = ((tx % nT) + nT) % nT, dx = (tx - x0) * 256, dy = (ty - y0) * 256;
      jobs.push((async () => {
        /* imagery first, then roads + place-name/boundary overlays on top */
        const layers = await Promise.all(["img", "trans", "ref"].map((L) => loadImg(`/api/tile/${L}/${z}/${ty}/${wx}`)));
        layers.forEach((im) => { if (im) { try { ctx.drawImage(im, dx, dy); } catch (e) { } } });
        return layers[0] != null; // base imagery loaded
      })());
    }
    if (!(await Promise.all(jobs)).some(Boolean)) return null;
    let href;
    try { href = cv.toDataURL("image/jpeg", 0.82); } catch (e) { return null; } // tainted → bail
    return { href, x: G.W / 2 - (mcx - x0 * 256) * k, y: G.H / 2 - (mcy - y0 * 256) * k, w: cw * k, h: ch * k };
  } catch (e) { return null; }
}

async function reportPlotSvg(fix, traj) {
  const G = plotGeom(fix, traj), { W, H, s, span, px } = G;
  const bm = await satBasemap(fix, G);
  const e2 = (t) => String(t || "").replace(/&/g, "&amp;").replace(/</g, "&lt;");
  /* legible over imagery: white text with a dark halo when a basemap is shown */
  const tx = (x, y, str, o = {}) => `<text x="${x}" y="${y}" font-size="${o.size || 11}"${o.weight ? ` font-weight="${o.weight}"` : ""} fill="${bm ? "#fff" : (o.fill || "#333")}"${bm ? ` stroke="#000" stroke-width="2.6" paint-order="stroke"` : ""}>${str}</text>`;
  let g = bm ? `<image href="${bm.href}" x="${bm.x.toFixed(1)}" y="${bm.y.toFixed(1)}" width="${bm.w.toFixed(1)}" height="${bm.h.toFixed(1)}" preserveAspectRatio="none"/>` : "";
  for (const o of fix.obs) {
    const a = px(o.P), i = fix.obs.indexOf(o);
    const far = px(add(o.P, scl(o.dA, Math.max((fix.solA.ts[i] || 0) * 1.15, span * 0.3))));
    g += `<line x1="${a[0]}" y1="${a[1]}" x2="${far[0]}" y2="${far[1]}" stroke="#F5A93F" stroke-dasharray="6 5" stroke-width="1.8" opacity=".95"/>`;
  }
  if (traj && traj.length > 1) g += `<polyline points="${traj.map((p) => px(p).join(",")).join(" ")}" fill="none" stroke="#6ea0ff" stroke-width="2.4" opacity=".95"/>`;
  fix.obs.forEach((o, i) => {
    const [x, y] = px(o.P);
    g += `<path d="M${x} ${+y - 7} l-6 12 h12 z" fill="#F5A93F" stroke="#000" stroke-width=".6"/>` + tx(+x + 9, +y + 4, e2(o.s.name || `Obs ${i + 1}`));
  });
  const A = px(fix.solA.X);
  g += `<circle cx="${A[0]}" cy="${A[1]}" r="7" fill="none" stroke="#2ee6c8" stroke-width="2.6"/><line x1="${+A[0] - 12}" y1="${A[1]}" x2="${+A[0] + 12}" y2="${A[1]}" stroke="#2ee6c8" stroke-width="2.2"/><line x1="${A[0]}" y1="${+A[1] - 12}" x2="${A[0]}" y2="${+A[1] + 12}" stroke="#2ee6c8" stroke-width="2.2"/>` + tx(+A[0] + 14, +A[1] - 9, "FIX A", { weight: 700, fill: "#0e7d6f" });
  if (fix.motion?.XB) {
    const B = px(fix.motion.XB);
    g += `<line x1="${A[0]}" y1="${A[1]}" x2="${B[0]}" y2="${B[1]}" stroke="#2ee6c8" stroke-width="2"/><circle cx="${B[0]}" cy="${B[1]}" r="5" fill="none" stroke="#2ee6c8" stroke-width="2.2"/>` + tx(+B[0] + 9, +B[1] + 4, "B", { fill: "#0e7d6f" });
  }
  const target = span / 4;
  const nice = Math.pow(10, Math.floor(Math.log10(target))) * ([1, 2, 5, 10].find((f) => Math.pow(10, Math.floor(Math.log10(target))) * f >= target) || 10);
  g += `<line x1="16" y1="${H - 16}" x2="${16 + nice * s}" y2="${H - 16}" stroke="${bm ? "#fff" : "#555"}" stroke-width="2.5"/>` + tx(16, H - 22, fmtLenShort(nice), { size: 10, fill: "#555" }) + tx(16, 20, "N ↑", { size: 12, fill: "#555" });
  if (bm) g += tx(W - 6, H - 6, "© Esri, Maxar", { size: 9 }).replace("<text ", '<text text-anchor="end" opacity=".9" ');
  return `<svg viewBox="0 0 ${W} ${H}" style="max-width:100%;border:1px solid #ddd;border-radius:6px;background:${bm ? "#111" : "#fafafa"}">${g}</svg>`;
}

/* speed + felt-load strip chart for the report */
function reportTrajSvg(k) {
  if (!k?.segs?.length) return "";
  const W = 560, H = 220, L = 46, Rm = 46, T = 16, B = 30;
  const t0 = k.segs[0].t, t1 = k.segs[k.segs.length - 1].t || t0 + 1;
  const vMax = Math.max(...k.segs.map((s) => s.speed)) * 1.15 || 1;
  const gMax = Math.max(1.5, ...(k.acc || []).map((a) => a.load)) * 1.15;
  const X = (t) => L + ((t - t0) / (t1 - t0)) * (W - L - Rm);
  const Yv = (v) => T + (1 - v / vMax) * (H - T - B);
  const Yg = (gg) => T + (1 - gg / gMax) * (H - T - B);
  const spd = k.segs.map((s) => `${X(s.t).toFixed(1)},${Yv(s.speed).toFixed(1)}`).join(" ");
  const lod = (k.acc || []).map((a) => `${X(a.t).toFixed(1)},${Yg(a.load).toFixed(1)}`).join(" ");
  return `<svg viewBox="0 0 ${W} ${H}" style="max-width:100%;border:1px solid #ddd;border-radius:6px;background:#fff">
<polyline points="${spd}" fill="none" stroke="#0e7d6f" stroke-width="2.2"/>
${lod ? `<polyline points="${lod}" fill="none" stroke="#C77B14" stroke-width="2" stroke-dasharray="5 4"/>` : ""}
<text x="${L}" y="${T - 3}" font-size="10" fill="#0e7d6f">speed (peak ${Math.round(k.peakSpeed)} m/s · ${Math.round(k.peakSpeed * 2.23694)} mph)</text>
${lod ? `<text x="${W - Rm}" y="${T - 3}" font-size="10" fill="#C77B14" text-anchor="end">felt load (peak ${k.peakLoad?.toFixed(1)} g, dashed)</text>` : ""}
<text x="${W / 2}" y="${H - 8}" font-size="10" fill="#888" text-anchor="middle">${(t1 - t0).toFixed(1)} s</text>
</svg>`;
}

/* Which orthographic views make a shape's size unambiguous. Each entry:
   [title, horizontalAxis, verticalAxis, hLabel, vLabel]. An orb needs one
   view; a bird/plane wants three. Axes are the object's OWN model axes
   (x = fore–aft, y = left–right / span, z = up), so photo foreshortening is
   removed and the true extents show. */
const SHAPE_VIEWS = {
  orb: [["Top", "x", "y", "diameter", "diameter"]],
  saucer: [["Top", "x", "y", "diameter", "diameter"], ["Side", "x", "z", "diameter", "height"]],
  capsule: [["Side", "x", "z", "length", "diameter"], ["End", "y", "z", "diameter", "diameter"]],
  tri: [["Top", "x", "y", "width", "depth"], ["Side", "x", "z", "width", "thickness"]],
  plane: [["Top", "x", "y", "length", "wingspan"], ["Side", "x", "z", "length", "height"], ["Front", "y", "z", "wingspan", "height"]],
  bird: [["Top", "x", "y", "length", "wingspan"], ["Side", "x", "z", "length", "height"], ["Front", "y", "z", "wingspan", "height"]],
  drone: [["Top", "x", "y", "width", "depth"], ["Side", "x", "z", "width", "height"], ["Front", "y", "z", "depth", "height"]],
  jelly: [["Side", "x", "z", "bell width", "height"], ["Top", "x", "y", "diameter", "diameter"]],
};

/* Build the dimensioned orthographic 3-view figure for a fitted shape.
   modelUnitMeters converts model units → metres (null ⇒ show proportions,
   normalised so the largest extent = 1.00×). Returns { html, ext } where
   ext is the model-unit extent along each axis. */
function buildShapeViews(sf, modelUnitMeters, e2) {
  const wire = shapeWire(sf.kind, sf.aspect, sf);
  const idx = { x: 0, y: 1, z: 2 };
  const mn = { x: 1e9, y: 1e9, z: 1e9 }, mx = { x: -1e9, y: -1e9, z: -1e9 };
  for (const c of wire) for (const p of c) for (const a of ["x", "y", "z"]) {
    const v = p[idx[a]]; if (v < mn[a]) mn[a] = v; if (v > mx[a]) mx[a] = v;
  }
  const ext = { x: mx.x - mn.x, y: mx.y - mn.y, z: mx.z - mn.z };
  const maxExt = Math.max(ext.x, ext.y, ext.z, 1e-6);
  const dimTxt = (u) => modelUnitMeters != null ? fmtLenShort(u * modelUnitMeters) : (u / maxExt).toFixed(2) + "×";
  const views = SHAPE_VIEWS[sf.kind] || SHAPE_VIEWS.plane;
  /* one common px-per-unit scale so the views read to-scale against each other */
  let maxH = 1e-6, maxV = 1e-6;
  for (const [, ha, va] of views) { maxH = Math.max(maxH, ext[ha]); maxV = Math.max(maxV, ext[va]); }
  const box = 168, ml = 30, mr = 12, mt = 20, mb = 30, innerW = box - ml - mr, innerH = box - mt - mb;
  const spx = Math.min(innerW / maxH, innerH / maxV);
  const colR = `hsl(${sf.hue ?? 36},80%,38%)`;
  const svgs = views.map(([title, ha, va, hl, vl]) => {
    const hMid = (mn[ha] + mx[ha]) / 2, vMid = (mn[va] + mx[va]) / 2;
    const cx = ml + innerW / 2, cy = mt + innerH / 2;
    const SX = (p) => (cx + (p[idx[ha]] - hMid) * spx).toFixed(1);
    const SY = (p) => (cy - (p[idx[va]] - vMid) * spx).toFixed(1);   // up = up
    const paths = wire.map((c) => `<polyline fill="none" stroke="${colR}" stroke-width="1.4" points="${c.map((p) => SX(p) + "," + SY(p)).join(" ")}"/>`).join("");
    const hw = ext[ha] * spx, vh = ext[va] * spx;
    const bx0 = (cx - hw / 2).toFixed(1), bx1 = (cx + hw / 2).toFixed(1);
    const by0 = (cy - vh / 2).toFixed(1), by1 = (cy + vh / 2).toFixed(1);
    const dy = box - 14;
    const hDim = `<line x1="${bx0}" y1="${dy}" x2="${bx1}" y2="${dy}" stroke="#aaa"/><line x1="${bx0}" y1="${dy - 3}" x2="${bx0}" y2="${dy + 3}" stroke="#aaa"/><line x1="${bx1}" y1="${dy - 3}" x2="${bx1}" y2="${dy + 3}" stroke="#aaa"/><text x="${cx}" y="${dy + 12}" font-size="9.5" fill="#444" text-anchor="middle">${e2(hl)} ${dimTxt(ext[ha])}</text>`;
    const dx = 11;
    const vDim = `<line x1="${dx}" y1="${by0}" x2="${dx}" y2="${by1}" stroke="#aaa"/><line x1="${dx - 3}" y1="${by0}" x2="${dx + 3}" y2="${by0}" stroke="#aaa"/><line x1="${dx - 3}" y1="${by1}" x2="${dx + 3}" y2="${by1}" stroke="#aaa"/><text x="${dx + 1}" y="${cy}" font-size="9.5" fill="#444" text-anchor="middle" transform="rotate(-90 ${dx + 1} ${cy})">${e2(vl)} ${dimTxt(ext[va])}</text>`;
    return `<svg viewBox="0 0 ${box} ${box}" style="width:${box}px;max-width:46vw;height:auto;border:1px solid #e2e2e2;border-radius:6px;background:#fff">
<text x="${cx}" y="13" font-size="11" font-weight="700" fill="#333" text-anchor="middle">${title}</text>
${paths}${hDim}${vDim}</svg>`;
  }).join("");
  return { html: `<div style="display:flex;gap:8px;flex-wrap:wrap;align-items:flex-start">${svgs}</div>`, ext };
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
    const mslA = fix.solA.X[2] + (fix.ref.alt || 0);
    const geomTbl = `<table><tr><th>Observer</th><th>Range</th><th>Angular size</th><th>→ True size</th></tr>` +
      fix.perSource.map((p) => `<tr><td>${e2(p.name || "—")}</td><td>${fmtLenShort(p.dist)}</td><td>${p.ang != null ? fmtDeg(p.ang) : "—"}</td><td>${p.size != null ? `${fmtLenShort(p.size)} (${ft(p.size)} ft)` : "—"}</td></tr>`).join("") +
      `</table>`;
    const qualTbl = `<table>` +
      row("Baseline (observer separation)", fmtLenShort(fix.baseline)) +
      row("Ray convergence angle", fix.conv.toFixed(1) + "°") +
      row("Ray miss distance (RMS)", `${fmtLenShort(fix.solA.rmsMiss)} (${(fix.missRatio * 100).toFixed(1)}% of range)`) +
      row("Range / baseline ratio", `${(fix.meanDist / Math.max(1, fix.baseline)).toFixed(1)} : 1`) +
      row("Position uncertainty", `± ${fmtLenShort(fix.posErr)} (from a ±1° pointing error)`) +
      row("Quality rating", fix.rating + (fix.behind ? " — rays cross BEHIND an observer; treat as unreliable (see caveats)" : "")) +
      `</table>`;
    fixHtml = `<table>` +
      row("Object ground position", `${fix.geoA.lat.toFixed(5)}, ${fix.geoA.lon.toFixed(5)} (± ${fmtLenShort(fix.posErr)})`) +
      row("Altitude above observer 1", `${fmtLenShort(fix.solA.X[2])} (${ft(fix.solA.X[2])} ft)`) +
      row("Altitude (MSL)", `${fmtLenShort(mslA)} (${ft(mslA)} ft)`) +
      row("Range from observer 1", fmtLenShort(fix.perSource[0].dist)) +
      (fix.sizeAvg != null ? row("Object size (avg)", `${fmtLenShort(fix.sizeAvg)} (${ft(fix.sizeAvg)} ft)`) : "") +
      ((asp) => asp ? row("Aspect-corrected span (if elongated)", `${asp.map((x) => `${fmtLenShort(x.S)} @ long-axis ${Math.round(x.psi)}°`).join(" or ")} (${asp[0].n} views${asp[0].rms != null ? `, fit rms ${fmtLenShort(asp[0].rms)}` : ""})`) : "")(aspectSpan(fix)) +
      (fix.motion?.speed != null ? row("Speed A→B", `${Math.round(fix.motion.speed)} m/s (${Math.round(fix.motion.speed * 2.23694)} mph), heading ${Math.round(fix.motion.heading)}° ${compass8(fix.motion.heading)}${isNum(fix.motion.vRate) ? `, vertical ${fix.motion.vRate >= 0 ? "climb" : "descent"} ${n1(Math.abs(fix.motion.vRate))} m/s` : ""}`) : "") +
      (fix.motion?.disp != null ? row("Displacement A→B", `${fmtLenShort(fix.motion.disp)}${fix.motion.dt != null ? ` over ${fix.motion.dt.toFixed(2)} s` : ""} (Δalt ${n1(fix.motion.XB[2] - fix.solA.X[2])} m)`) : "") +
      `</table>` +
      (await reportPlotSvg(fix, tr.stereo?.k ? tr.stereo.pos : null)) +
      `<p class="cap">Top-down (satellite basemap): observers (▲), sight rays (dashed), triangulated fix (⊕)${tr.stereo?.k ? ", trajectory (blue)" : ""}.</p>` +
      `<p class="cap" style="margin-top:14px"><b>Per-observer geometry</b> — range × measured angular size gives each witness's independent size estimate.</p>${geomTbl}` +
      `<p class="cap" style="margin-top:14px"><b>Solution quality</b> — how trustworthy the geometry is.</p>${qualTbl}`;
  } else {
    fixHtml = `<p><i>Fewer than two complete observers — angular data only. Import this file into Phodar and add a second perspective to triangulate.</i></p>`;
    /* single witness: the honest deliverable is the size↔distance line —
       every assumed distance implies a size; reference objects pin intuition */
    const w = (() => {
      for (const s of origAct) {
        const a = angSizeFromPoints(s.A?.p1, s.A?.p2, s.natW, s.natH, +s.fovH) ?? (isNum(s.A?.angManual) ? +s.A.angManual : null);
        if (a != null && a > 0) return { ang: a, el: isNum(s.A?.el) ? +s.A.el : null };
      }
      return null;
    })();
    if (w != null) {
      const wAng = w.ang, el = w.el;
      const W = 560, H = 300, L = 62, Rm = 16, T = 34, B = 44;
      const D0 = 50, D1 = 50000;
      const s0 = 2 * D0 * Math.tan(wAng * D2R / 2), s1 = 2 * D1 * Math.tan(wAng * D2R / 2);
      /* single witness: assumed distance implies BOTH size (2·D·tan(ang/2))
         and altitude above the observer (D·sin(el)) — both in metres, so they
         share the log axis. */
      const drawAlt = el != null && el > 0.2;
      const sinEl = drawAlt ? Math.sin(el * D2R) : 0;
      const a0 = drawAlt ? D0 * sinEl : null, a1 = drawAlt ? D1 * sinEl : null;
      const sLo = Math.min(s0, 0.2, drawAlt ? a0 : Infinity), sHi = Math.max(s1, 120, drawAlt ? a1 : 0);
      const X = (Dm) => L + ((Math.log10(Dm) - Math.log10(D0)) / (Math.log10(D1) - Math.log10(D0))) * (W - L - Rm);
      const Y = (Sm) => T + (1 - (Math.log10(Sm) - Math.log10(sLo)) / (Math.log10(sHi) - Math.log10(sLo))) * (H - T - B);
      const refs = REF_OBJECTS.filter((o) => {
        const Dq = o.size / (2 * Math.tan(wAng * D2R / 2));
        return Dq >= D0 && Dq <= D1;
      }).map((o) => {
        const Dq = o.size / (2 * Math.tan(wAng * D2R / 2));
        const x = X(Dq), near = x > L + (W - L - Rm) * 0.6;
        return `<line x1="${L}" y1="${Y(o.size).toFixed(1)}" x2="${W - Rm}" y2="${Y(o.size).toFixed(1)}" stroke="#eee" stroke-dasharray="4 4"/>` +
          `<circle cx="${x.toFixed(1)}" cy="${Y(o.size).toFixed(1)}" r="3.5" fill="#C77B14"/>` +
          `<text x="${(near ? x - 6 : x + 6).toFixed(1)}" y="${(Y(o.size) - 5).toFixed(1)}" font-size="10" fill="#555" text-anchor="${near ? "end" : "start"}">${e2(o.name)} — ${fmtLenShort(Dq)}</text>`;
      }).join("");
      const altRefs = drawAlt ? [{ alt: 120, name: "drone ceiling" }, { alt: 11000, name: "jet cruise" }]
        .map((r) => ({ ...r, Dr: r.alt / sinEl }))
        .filter((r) => r.alt >= sLo && r.alt <= sHi && r.Dr >= D0 && r.Dr <= D1)
        .map((r) => { const x = X(r.Dr), near = x > L + (W - L - Rm) * 0.6; return `<circle cx="${x.toFixed(1)}" cy="${Y(r.alt).toFixed(1)}" r="3.2" fill="#2563c9"/><text x="${(near ? x - 6 : x + 6).toFixed(1)}" y="${(Y(r.alt) + 12).toFixed(1)}" font-size="10" fill="#2563c9" text-anchor="${near ? "end" : "start"}">${r.name} — ${fmtLenShort(r.Dr)}</text>`; }).join("") : "";
      const xTicks = [100, 1000, 10000].map((d) => `<line x1="${X(d)}" y1="${T}" x2="${X(d)}" y2="${H - B}" stroke="#eee"/><text x="${X(d)}" y="${H - B + 16}" font-size="10" fill="#555" text-anchor="middle">${fmtLenShort(d)}</text>`).join("");
      const yTicks = [1, 10, 100, 1000, 10000].filter((s) => s >= sLo && s <= sHi).map((s) => `<text x="${L - 6}" y="${(Y(s) + 3).toFixed(1)}" font-size="10" fill="#555" text-anchor="end">${fmtLenShort(s)}</text>`).join("");
      const altLine = drawAlt ? `<line x1="${X(D0)}" y1="${Y(a0).toFixed(1)}" x2="${X(D1)}" y2="${Y(a1).toFixed(1)}" stroke="#2563c9" stroke-width="2.5"/>${altRefs}` : "";
      fixHtml += `<svg viewBox="0 0 ${W} ${H}" style="max-width:100%;border:1px solid #ddd;border-radius:6px;background:#fff">
<text x="${L}" y="20" font-size="12" font-weight="700" fill="#333">Assumed distance ⇄ implied size${drawAlt ? " &amp; altitude" : ""} (${wAng.toFixed(2)}° wide${drawAlt ? `, ${el.toFixed(0)}° up` : ""})</text>
<text x="${W - Rm}" y="13" font-size="10" fill="#0e7d6f" text-anchor="end">■ size</text>${drawAlt ? `<text x="${W - Rm}" y="26" font-size="10" fill="#2563c9" text-anchor="end">■ altitude above you</text>` : ""}
${xTicks}${yTicks}${refs}${altLine}
<line x1="${X(D0)}" y1="${Y(s0).toFixed(1)}" x2="${X(D1)}" y2="${Y(s1).toFixed(1)}" stroke="#0e7d6f" stroke-width="2.5"/>
<text x="${W / 2}" y="${H - 8}" font-size="10" fill="#888" text-anchor="middle">assumed distance →</text>
<text x="14" y="${H / 2}" font-size="10" fill="#888" transform="rotate(-90 14 ${H / 2})" text-anchor="middle">size / altitude (m) →</text>
</svg>
<p class="cap">One witness can't fix the distance — but every assumed distance implies both a size and ${drawAlt ? `an altitude above you (from the ${el.toFixed(0)}° sight-line). Amber dots mark common objects at that size; blue dots mark notable altitudes.` : "a size. Dots mark where common objects would sit on this sight-line. (Add the object's elevation to also read altitude.)"}</p>`;
    }
  }
  /* --- object dimensions: dimensioned front/side/top of the fitted shape.
     A raw size number is clear for an orb but ambiguous for a bird — the
     3-view makes span, length and height explicit, scaled to the fix. --- */
  let dimsHtml = "";
  {
    const sh = packed.find((s) => s.shapeFit && s.shapeFit.sizeNat);
    if (sh) {
      const sf = sh.shapeFit;
      const pr = shapeProjNat(sf);
      const projMajorUnits = Math.hypot(pr.p1.x - pr.p2.x, pr.p1.y - pr.p2.y) / (sf.sizeNat || 1);
      const mum = (fix.ok && fix.sizeAvg != null && projMajorUnits > 1e-6) ? fix.sizeAvg / projMajorUnits : null;
      const { html: viewsHtml, ext } = buildShapeViews(sf, mum, e2);
      const kindLabel = (SHAPES.find((x) => x.k === sf.kind) || {}).label || sf.kind;
      const dimRow = (lbl, u) => row(lbl, mum != null ? `${fmtLenShort(u * mum)} (${ft(u * mum)} ft)` : `${(u / Math.max(ext.x, ext.y, ext.z, 1e-6)).toFixed(2)}× (relative — no absolute scale)`);
      dimsHtml = `<h2>Object dimensions (${sf.kind === "orb" ? "1-view" : SHAPE_VIEWS[sf.kind]?.length === 2 ? "2-view" : "3-view"})</h2>` +
        viewsHtml +
        `<table>` +
        dimRow("Length (fore–aft)", ext.x) +
        dimRow("Width / span", ext.y) +
        dimRow("Height", ext.z) +
        `</table>` +
        `<p class="cap">${e2(kindLabel)} shape fitted to ${e2(sh.name || "the photo")}. Views are the object's own axes, so photo foreshortening is removed and each true extent shows. ${mum != null ? "Scaled to the triangulated object size." : "Proportions only — an absolute size needs a second viewpoint to triangulate range."}</p>`;
    }
  }
  const kin = tr.stereo?.k ? `<table>` +
    row("Samples / duration", `${tr.stereo.k.n} pts · ${tr.stereo.k.dur.toFixed(1)} s`) +
    row("Path length", fmtLenShort(tr.stereo.k.path)) +
    row("Avg / peak speed", `${Math.round(tr.stereo.k.avgSpeed)} / ${Math.round(tr.stereo.k.peakSpeed)} m/s (${Math.round(tr.stereo.k.peakSpeed * 2.23694)} mph peak)`) +
    (tr.stereo.k.peakA != null ? row("Peak acceleration", tr.stereo.k.peakA.toFixed(1) + " m/s²") : "") +
    (tr.stereo.k.peakLoad != null ? row("Peak felt load", tr.stereo.k.peakLoad.toFixed(2) + " g") : "") +
    (tr.stereo.k.peakTurn != null ? row("Peak turn rate", tr.stereo.k.peakTurn.toFixed(1) + " °/s") : "") +
    `</table>` + reportTrajSvg(tr.stereo.k) : "";
  const soloKin = (!tr.stereo?.k && tr.solo?.length) ? (() => {
    const s0 = tr.solo[0];
    const base = `Single-view angular trajectory: ${s0.k.n} pts over ${s0.k.dur.toFixed(1)} s, peak angular rate ${(s0.k.peakSpeed * R2D).toFixed(2)} °/s.`;
    return s0.rad
      ? `<p class="cap">${base} The object was sized along the path, so radial (closer/farther) motion is recovered: its range varied over a <b>${s0.rad.rangeRatio.toFixed(2)}× span</b> (point ${s0.rad.iNear + 1} closest, point ${s0.rad.iFar + 1} farthest). The 3D path shape and speed profile are real — only the absolute scale needs an assumed distance (see the size ⇄ distance chart above).</p>`
      : `<p class="cap">${base} Distance-free — size the object at each trajectory point in the sky view to also capture closer/farther motion.</p>`;
  })() : "";

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
    /* carry the display brightness/contrast into the report via CSS filter
       (same non-destructive model as the app) — baked JPEGs stay the raw pixels */
    const adjF = imgAdjFilter(s.imgAdj);
    const adjSty = adjF === "none" ? "" : `filter:${adjF};`;
    const adjCap = adjF === "none" ? "" : " · brightness/contrast adjusted for viewing (original retained)";
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
    let detailBlock = "";
    if (s.detailJpeg) {
      const both = !!(s.shapeFit && s.detailCrop);
      /* both crops share the row as equal halves; a lone crop stays modest */
      const childCss = both ? "flex:1 1 0;min-width:0" : "max-width:min(330px,100%)";
      const noOv = `<div style="${childCss}"><img src="${s.detailJpeg}" style="width:100%;display:block;border:1px solid #ccc;border-radius:4px;${adjSty}"/><div class="cap">detail — cropped at the fitted shape, ×${s.detailZoom}, no overlay</div></div>`;
      let withOv = "";
      if (both) {
        const pr2 = shapeProjNat(s.shapeFit);
        const colR2 = `hsl(${s.shapeFit.hue ?? 36},85%,42%)`;
        const sw2 = Math.max(0.8, s.detailCrop.w / 240);
        const paths2 = pr2.curves.map((c) =>
          `<polyline fill="none" stroke="${colR2}" stroke-width="${sw2.toFixed(2)}" opacity="0.8" points="${c.map((p) => p.x.toFixed(1) + "," + p.y.toFixed(1)).join(" ")}"/>`
        ).join("");
        const kindLabel = (SHAPES.find((x) => x.k === s.shapeFit.kind) || {}).label || s.shapeFit.kind;
        const cr = s.detailCrop;
        withOv = `<div style="${childCss}"><div style="position:relative;border:1px solid #ccc;border-radius:4px;overflow:hidden"><img src="${s.detailJpeg}" style="width:100%;display:block;${adjSty}"/><svg viewBox="${cr.x.toFixed(1)} ${cr.y.toFixed(1)} ${cr.w.toFixed(1)} ${cr.h.toFixed(1)}" preserveAspectRatio="xMidYMid meet" style="position:absolute;left:0;top:0;width:100%;height:100%">${paths2}</svg></div><div class="cap">detail — same crop with the ${e2(kindLabel)} shape overlaid (your colour)</div></div>`;
      }
      detailBlock = `<div style="display:flex;gap:8px;margin-top:8px;align-items:flex-start">${noOv}${withOv}</div>`;
    }
    return `<h2>Exhibit — ${e2(s.name || "Observer " + (i + 1))}</h2>
<div style="position:relative;display:inline-block;max-width:100%"><img src="${imgSrc}" style="max-width:100%;display:block;${adjSty}"/>${overlay}</div>
<div class="cap">${s.meta?.model ? e2(s.meta.model) + " · " : ""}${s.whenMs ? new Date(+s.whenMs).toLocaleString() : ""}${s.mediaAim ? ` · placed ${(+s.mediaAim.az).toFixed(1)}° az / ${(+s.mediaAim.el).toFixed(1)}° el` : ""}${s.shapeFit ? ` · ${e2(s.shapeFit.kind)} fit` : ""}${adjCap}</div>
${detailBlock}`;
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
  /* --- sighting conditions: exact Sun/Moon geometry + magnetic declination
     at the primary observer's time & place. Flags glare, a bright Moon as
     the light source, and pins the local-time / twilight state. --- */
  let condHtml = "";
  {
    const w = origAct.find((s) => isNum(s.lat) && isNum(s.lon) && isNum(s.whenMs));
    if (w) {
      const Tw = +w.whenMs, la = +w.lat, lo = +w.lon;
      const sun = sunPos(Tw, la, lo);
      const moon = moonPos(Tw, la, lo);
      const ill = Math.round(moonFrac(Tw) * 100);
      const tw = sun.alt > 0 ? "daylight" : sun.alt > -6 ? "civil twilight" : sun.alt > -12 ? "nautical twilight" : sun.alt > -18 ? "astronomical twilight" : "night (full dark)";
      let dec = null;
      try { dec = declination(la, lo, isNum(w.alt) ? +w.alt : 0, new Date(Tw)); } catch (e) { }
      condHtml = `<h2>Sighting conditions</h2><table>` +
        row("Local sky", `${tw} — Sun ${Math.abs(sun.alt).toFixed(1)}° ${sun.alt >= 0 ? "above" : "below"} the horizon at az ${Math.round(sun.az)}° ${compass8(sun.az)}`) +
        row("Moon", `${ill}% illuminated · ${moon.alt > 0 ? `${moon.alt.toFixed(0)}° up at az ${Math.round(moon.az)}° ${compass8(moon.az)}` : "below the horizon"}`) +
        (dec != null ? row("Magnetic declination", `${dec >= 0 ? "+" : ""}${dec.toFixed(1)}° (WMM2025 — added to any magnetic compass bearing to get true)`) : "") +
        `</table>` +
        `<p class="cap">Computed for ${e2(w.name || "observer 1")} at ${new Date(Tw).toLocaleString()}. Sun/Moon geometry is exact — use it to sanity-check the reported time, and to rule the Sun/Moon in or out as glare or the light source.</p>`;
    }
  }
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
  /* --- wind check: does the motion match balloon drift at the fix altitude? --- */
  let windHtml = "";
  if (fix.ok) {
    let objSpeed = null, objHeading = null, motionSrc = null;
    if (tr.stereo?.k && tr.stereo.pos?.length > 1) {
      const p0 = tr.stereo.pos[0], p1 = tr.stereo.pos[tr.stereo.pos.length - 1];
      const dur = tr.stereo.times[tr.stereo.times.length - 1] - tr.stereo.times[0];
      if (dur > 0) {
        const vE = (p1[0] - p0[0]) / dur, vN = (p1[1] - p0[1]) / dur;
        objSpeed = Math.hypot(vE, vN);
        objHeading = ((Math.atan2(vE, vN) * R2D) + 360) % 360;
        motionSrc = "triangulated track";
      }
    } else if (fix.motion?.v) {
      objSpeed = Math.hypot(fix.motion.v[0], fix.motion.v[1]);
      objHeading = fix.motion.heading;
      motionSrc = "Moment A→B";
    }
    if (objSpeed != null && objSpeed > 0.2) {
      try {
        const when = +(origAct.find((s) => isNum(s.whenMs))?.whenMs || Date.now());
        const altMSL = fix.solA.X[2] + (fix.ref.alt || 0);
        const wind = await fetchWindAt(fix.ref.lat, fix.ref.lon, when, altMSL);
        const v = balloonVerdict(objSpeed, objHeading, wind);
        const cls = v.verdict === "balloon-consistent" ? "" : "cap";
        windHtml = `<h2>Wind check (balloon test)</h2>
<p class="${cls}">Wind at ${wind.hPa} hPa (≈ ${fmtLenShort(wind.levelM)} MSL; fix ≈ ${fmtLenShort(altMSL)}): <b>${n1(wind.speedMs)} m/s from ${Math.round(wind.fromDeg)}°</b> → drift toward ${Math.round(wind.driftDeg)}°.
The object (${motionSrc}) moved <b>${n1(objSpeed)} m/s toward ${Math.round(objHeading)}°</b> — heading off by ${Math.round(v.dHead)}°, speed ${isFinite(v.ratio) ? v.ratio.toFixed(1) + "×" : "≫"} the wind.
<b>${v.verdict === "balloon-consistent" ? "⚠ Consistent with a wind-borne object (balloon signature)." : v.verdict === "partially wind-like" ? "Partially wind-like — not conclusive either way." : "Not wind-borne: a balloon cannot do this."}</b>
<span class="cap">(${wind.src})</span></p>`;
      } catch (e) { /* offline or no data — say nothing rather than guess */ }
    }
  }
  /* known-event correlators (rocket launches, bolides) — run for any single
     observer with a location + time; both degrade to nothing if the server
     proxy or the upstream API is unreachable. */
  let launchHtml = "", fireballHtml = "";
  {
    const obs0 = origAct.find((s) => isNum(s.lat) && isNum(s.lon));
    const when = +(origAct.find((s) => isNum(s.whenMs))?.whenMs || Date.now());
    if (obs0) {
      const fmtDt = (h) => Math.abs(h) < 48 ? `${h >= 0 ? "+" : ""}${h.toFixed(1)} h` : `${h >= 0 ? "+" : ""}${(h / 24).toFixed(1)} d`;
      try {
        const L = (await fetchLaunches(+obs0.lat, +obs0.lon, when)).filter((x) => Math.abs(x.dtHours) <= 14 * 24).slice(0, 8);
        if (L.length) launchHtml = `<h2>Launch check (rocket launches)</h2>
<p class="cap">Rocket launches near the sighting (Launch Library 2). A fresh Starlink batch is a moving &ldquo;train&rdquo; of dots for days after launch; a twilight launch plume is visible for hundreds of km.</p>
<table><tr><th>When (Δ)</th><th>Rocket / mission</th><th>Pad</th><th>Range</th></tr>${L.map((x) => `<tr><td>${new Date(x.net).toLocaleString()}<br><span class="cap">${fmtDt(x.dtHours)}</span></td><td>${e2(x.rocket || x.name)}${x.starlink ? " · <b>🛰 STARLINK</b>" : ""}<br><span class="cap">${e2(x.mission || "")}</span></td><td>${e2(x.padName || "")}</td><td>${x.distKm != null ? Math.round(x.distKm) + " km" : "—"}</td></tr>`).join("")}</table>`;
      } catch (e) { /* offline / rate-limited — omit */ }
      try {
        const F = (await fetchFireballs(+obs0.lat, +obs0.lon, when)).filter((x) => Math.abs(x.dtHours) <= 24).slice(0, 6);
        if (F.length) fireballHtml = `<h2>Fireball check (NASA CNEOS)</h2>
<p class="cap">Bright bolides logged by US Government sensors near the sighting time. A match within minutes and a few hundred km is a strong meteor explanation.</p>
<table><tr><th>When (Δ)</th><th>Energy (kt TNT)</th><th>Alt / speed</th><th>Range</th></tr>${F.map((x) => `<tr><td>${new Date(x.t).toLocaleString()}<br><span class="cap">${fmtDt(x.dtHours)}</span></td><td>${x.energyKt != null ? x.energyKt : "—"}${x.impactKt != null ? ` <span class="cap">(${x.impactKt} total)</span>` : ""}</td><td>${x.altKm != null ? x.altKm + " km" : "—"}${x.velKmS != null ? ` · ${x.velKmS} km/s` : ""}</td><td>${x.distKm != null ? Math.round(x.distKm) + " km" : "—"}</td></tr>`).join("")}</table>`;
      } catch (e) { /* omit */ }
    }
  }
  const data = JSON.stringify({ phodar: 1, created: new Date().toISOString(), sources: packed, est }, null, 1).replace(/<\//g, "<\\/");
  return `<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>PHODAR sighting report</title><style>
html,body{max-width:100%;overflow-x:hidden}
body{font:14px/1.55 -apple-system,"Segoe UI",Roboto,sans-serif;color:#141414;max-width:760px;margin:32px auto;padding:0 18px;overflow-wrap:break-word}
h1{font:800 22px ui-monospace,Menlo,monospace;letter-spacing:.12em}h1 span{color:#C77B14}
h2{font:700 12px ui-monospace,monospace;letter-spacing:.16em;text-transform:uppercase;color:#555;margin-top:26px}
table{border-collapse:collapse;width:100%;font-size:13px;margin:6px 0}td,th{border:1px solid #ccc;padding:6px 8px;text-align:left;vertical-align:top;overflow-wrap:break-word}
img,svg{max-width:100%;height:auto}
.cap{color:#666;font-size:12px}@media print{.noprint{display:none}}
@media(max-width:640px){body{margin:16px auto;padding:0 12px}table{table-layout:fixed;font-size:12px}}
</style></head><body>
<div style="display:flex;align-items:center;gap:14px;flex-wrap:wrap;margin-bottom:2px"><img src="data:image/svg+xml,${encodeURIComponent(phodarLogoRaw)}" alt="PHODAR" style="height:48px;width:auto;border-radius:8px;display:block"/><span style="font:700 13px ui-monospace,Menlo,monospace;letter-spacing:.16em;color:#555">SIGHTING REPORT</span></div>
<div class="cap">Generated ${new Date().toLocaleString()} · photogrammetric detection &amp; ranging · phodar v1</div>
<h2>Observers (${packed.length})</h2>
<table><tr><th>Name</th><th>Position</th><th>Time</th><th>Bearing az/el</th><th>FOV</th><th>Traj pts</th></tr>${obsRows}</table>
<h2>Result</h2>${fixHtml}
${dimsHtml}
${kin ? `<h2>Trajectory kinematics (stereo)</h2>${kin}` : soloKin}
${adsbHtml}
${condHtml}
${skyHtml}
${windHtml}
${launchHtml}
${fireballHtml}
${exhibits}
<h2>Method</h2><p>Each photo is pixel-normalized and its lens field of view read from EXIF. The object's sky direction is fixed by aligning the photo on an astronomically anchored alt-azimuth grid (Sun/Moon computed for the reported time and place). With two or more observers, sight-lines are intersected by least squares in a local ENU frame; ray convergence and rms miss distance grade the fix. Object size = measured angular size × range. Trajectories interpolate each witness's directions to common instants before triangulating each instant; speeds, accelerations and felt g-loads follow by finite differences with 3-point smoothing.</p>
<h2>Caveats</h2><p>${fix.ok ? `Quality <b>${fix.rating}</b>: baseline ${fmtLenShort(fix.baseline)}, convergence ${fix.conv.toFixed(1)}°, rms ray miss ${fmtLenShort(fix.solA.rmsMiss)}; a ±1° bearing error implies ≈ ${fmtLenShort(fix.posErr)} of position uncertainty.` : `Single-perspective data — directions and angular sizes are honest; absolute range, size and speed require a second viewpoint.`} Compass bearings may be magnetic rather than true; EXIF times are device-local.</p>
${diagHtml}
<p class="cap"> ${opts.exhibits === "full" || opts.exhibits === "files" ? "Exhibit photos are full resolution; the embedded share data carries 1600 px working copies." : "Bundled photos are 1600 px working copies; analysis used the originals."}</p>
<p class="noprint"><b>Add your perspective:</b> open Phodar → Import and choose this very file — the sighting data and photos are embedded below.</p>
<script type="application/json" id="phodar-data">${data}</script>
</body></html>`;
}

/* Wizard progress: 4 steps (photo · position · sky · finish). Shown on every
   step so the dots don't vanish after the first couple. */
function WizDots({ n, style }) {
  return (
    <div style={{ display: "flex", gap: 5, ...style }}>
      {[1, 2, 3, 4].map((i) => (
        <div key={i} style={{ width: i === n ? 18 : 7, height: 7, borderRadius: 4, background: i <= n ? "var(--amber)" : "var(--line)", transition: "all .2s" }} />
      ))}
    </div>
  );
}

function WizStep({ n, title, children, onBack, onNext, nextLabel, nextDisabled, disabledLabel }) {
  return (
    <div style={{ padding: "14px 12px 96px" }}>
      <div style={{ display: "flex", alignItems: "center", gap: 10, marginBottom: 10 }}>
        <button className="btn sm" onClick={onBack}>‹</button>
        <div>
          <div style={{ fontFamily: "var(--mono)", fontWeight: 800, letterSpacing: ".12em", fontSize: 14 }}>{title}</div>
          <WizDots n={n} style={{ marginTop: 4 }} />
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
  const dot = (on, k, title) => <span key={k} title={title} style={{ display: "inline-block", width: 7, height: 7, borderRadius: 4, background: on ? "var(--teal)" : "var(--line)", marginRight: 4 }} />;
  return (
    <div style={{ padding: "26px 14px 40px" }}>
      <div style={{ textAlign: "center", marginTop: 16 }}>
        <img src={phodarLogo} alt="PHODAR" style={{ display: "block", width: "min(460px, 94%)", margin: "0 auto", borderRadius: 12 }} />
        <div className="microlabel" style={{ marginTop: 6 }}>Photogrammetric detection &amp; ranging</div>
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
      <input ref={fileRef} type="file" accept=".json,.html,.zip,application/json,text/html,application/zip" style={{ display: "none" }}
        onChange={(e) => {
          const f = e.target.files?.[0]; if (!f) return;
          const finish = (tx) => {
            const n = tx ? onImport(tx) : 0;
            setImpMsg(n ? `✓ imported ${n} observer${n > 1 ? "s" : ""}` : "Couldn't read that — expected a .phodar.json, a Phodar report, or a sighting .zip.");
          };
          f.arrayBuffer().then((buf) => {
            const u8 = new Uint8Array(buf);
            /* a sighting .zip (PK\x03\x04) → pull the data file out of it;
               otherwise it's a .phodar.json or a report .html — read as text */
            if (u8[0] === 0x50 && u8[1] === 0x4B && u8[2] === 0x03 && u8[3] === 0x04) {
              finish(unzipEntryText(u8, "sighting.phodar.json") || unzipEntryText(u8, "report.html"));
            } else finish(new TextDecoder().decode(u8));
          }).catch(() => setImpMsg("Couldn't read that file."));
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
                  {/* photo · position · direction are the completable facets;
                     a trajectory dot only appears (and is always green) when a
                     track exists — it's optional, so it never blocks "complete" */}
                  {dot(!!s.mediaUrl, "m", "photo")}{dot(isNum(s.lat) && isNum(s.lon), "p", "position")}{dot(isNum(s.A?.az) && isNum(s.A?.el), "d", "direction")}{(s.track || []).length > 1 ? dot(true, "t", "trajectory") : null}
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

function WizFinish({ sources, est, onAdd, onReport, onShare, onHome, onFixAlt }) {
  const fix = analyze(sources);
  const tr = analyzeTracks(sources);
  return (
    <div style={{ padding: "14px 12px 40px" }}>
      <div style={{ display: "flex", alignItems: "center", gap: 10, marginBottom: 10 }}>
        <button className="btn sm" onClick={onHome}>‹</button>
        <div>
          <div style={{ fontFamily: "var(--mono)", fontWeight: 800, letterSpacing: ".12em", fontSize: 14 }}>SIGHTING CAPTURED</div>
          <WizDots n={4} style={{ marginTop: 4 }} />
        </div>
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
              <button className="btn sm teal" style={{ display: "block", marginTop: 6 }} onClick={async () => {
                for (const s of sources) {
                  if (!isNum(s.lat) || !isNum(s.lon)) continue;
                  try { const h = await demElevation(+s.lat, +s.lon); onFixAlt(s.id, h.toFixed(0)); } catch (e) { }
                }
              }}>⛰ Set every observer's elevation from terrain (DEM)</button>
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
  /* the full bundle as a single .zip: the report, the importable data, and
     the FULL-res photos — a download, importable back into Phodar */
  const downloadBundle = async () => {
    setMsg("packing bundle…");
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
      setMsg(`✓ downloading bundle — ${(blob.size / 1048576).toFixed(1)} MB · report + full-res photos + data`);
    else setMsg("Bundle download needs the deployed app — this preview can't save binaries.");
  };
  /* share the viewable report page itself via the OS share sheet (text,
     email, AirDrop…); falls back to a download/copy where share is unsupported */
  const shareReportHtml = async () => {
    const html = prevHtml || await reportHtml(sources, est, { exhibits: "full" });
    try {
      const file = new File([html], "phodar-report.html", { type: "text/html" });
      if (navigator.canShare && navigator.canShare({ files: [file] })) {
        await navigator.share({ files: [file], title: "PHODAR sighting report" });
        return;
      }
    } catch (e) { if (e && e.name === "AbortError") return; }
    deliver("phodar-report.html", html, "text/html");
  };
  const openReport = async () => { setMsg("packing…"); setPrevHtml(await reportHtml(sources, est, { exhibits: "full" })); setMsg(""); };
  return (
    <div style={{ padding: "14px 12px 40px" }}>
      <div style={{ display: "flex", alignItems: "center", gap: 10, marginBottom: 10 }}>
        <button className="btn sm" onClick={onBack}>‹</button>
        <div style={{ fontFamily: "var(--mono)", fontWeight: 800, letterSpacing: ".12em", fontSize: 14 }}>REPORT &amp; SHARE</div>
      </div>
      <div className="card" style={{ margin: 0 }}>
        <ML>Sighting report</ML>
        <div style={{ fontSize: 12, color: "var(--dim)", lineHeight: 1.6 }}>
          White-paper style: observers, the triangulated result, kinematics, methodology, caveats. It prints to PDF from any browser, and the sighting data is <b style={{ color: "var(--ink)" }}>embedded inside it</b> — anyone with Phodar can import the report itself to add their perspective. The share bundle also carries the full-resolution photos.
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
          <button className="btn amber" style={{ padding: 14, fontSize: 15 }} onClick={openReport}>👁 View report</button>
          <button className="btn" style={{ padding: 12 }} onClick={downloadBundle}>⬇ Download bundle (.zip — report + full-res photos + data)</button>
        </div>
        {msg && <div style={{ fontSize: 12, color: "var(--teal)", marginTop: 8 }}>{msg}</div>}
        <div style={{ fontSize: 11, color: "var(--dim)", marginTop: 8 }}>
          The bundle re-imports into Phodar. Open the report to read it and share the page itself (text / email) from the top.
        </div>
      </div>

      {prevHtml && (
        <div style={{ position: "fixed", inset: 0, zIndex: 300, background: "#0009", display: "flex", flexDirection: "column" }}>
          <div style={{ display: "flex", gap: 6, padding: "10px 12px", background: "var(--bg)", alignItems: "center", flexWrap: "wrap" }}>
            <span style={{ fontFamily: "var(--mono)", fontSize: 12, fontWeight: 800, flex: 1, minWidth: 60 }}>REPORT</span>
            <button className="btn sm amber" onClick={shareReportHtml}>📤 Share/Download page</button>
            <button className="btn sm" onClick={() => setPrevHtml(null)}>✕ Close</button>
          </div>
          {msg && <div style={{ padding: "0 12px 8px", background: "var(--bg)", fontSize: 11, color: "var(--teal)" }}>{msg}</div>}
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

  /* device-orientation class for the portrait lock — screen.orientation is
     the physical sensor and ignores keyboard-squished viewports */
  useEffect(() => {
    const upd = () => {
      const land = screen.orientation
        ? String(screen.orientation.type).startsWith("landscape")
        : Math.abs(+window.orientation || 0) === 90;
      document.documentElement.classList.toggle("dev-landscape", land);
    };
    upd();
    if (screen.orientation) screen.orientation.addEventListener("change", upd);
    window.addEventListener("orientationchange", upd);
    return () => {
      if (screen.orientation) screen.orientation.removeEventListener("change", upd);
      window.removeEventListener("orientationchange", upd);
    };
  }, []);
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
      page = <WizFinish sources={sources} est={est} onAdd={addWitness} onReport={() => goView("report")} onShare={shareJsonNow} onHome={() => goView("home")} onFixAlt={(id, alt) => updateSource(id, { alt })} />;
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
            initAlt={isNum(wsrc.A?.el) ? +wsrc.A.el : (wsrc.mediaAim && isNum(wsrc.mediaAim.el) ? +wsrc.mediaAim.el : 20)}
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
