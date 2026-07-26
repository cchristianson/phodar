import React, { useState, useRef, useEffect, useLayoutEffect, useMemo, useCallback } from "react";
import L from "leaflet";
import "leaflet/dist/leaflet.css";
import { D2R, R2D, RAD, clampN, dot, sub, add, scl, unit, mag, geoFromEnu, dirFromAzEl, dirToAzEl } from "./math/geodesy.js";
import { isNum, n1, fmtLen, fmtLenShort, fmtSpeed, fmtDeg, compass8, setImperialUnits, isImperialUnits } from "./math/format.js";
/* storeys for a height in metres — a friendly cross-check beside the length */
const storeys = (m) => Math.max(1, Math.round(m / 3.3));
/* wind-speed → colour ramp for the winds-aloft overlay (calm→gale) */
const windColor = (ms) => ms < 2.5 ? "#5FD3BC" : ms < 7 ? "#8FB4FF" : ms < 13 ? "#F5A93F" : "#E8604C";
/* the COMPLEMENTARY unit for a secondary readout (so an imperial user doesn't
   see "20 ft / 20 ft"): imperial primary ⇒ metric sub, and vice-versa */
const fmtLenAlt = (m) => isImperialUnits() ? `${n1(m)} m` : `${n1(m * 3.28084)} ft`;
/* compact single-unit speed in the user's system (mph vs km/h) */
const fmtSpeedShort = (ms) => isImperialUnits() ? `${n1(ms * 2.23694)} mph` : `${n1(ms * 3.6)} km/h`;
import { photoBasis, angSizeFromPoints, pixelDirFromAnchor, pixToDirK, dirToPixK, solvePoseAnchors, reanchorPose, reanchorAzEl } from "./math/projection.js";
import { syncSensor, fuseSensorVisual, fuseStats, motionDisagreement, sensorOnlyPath } from "./video/sensorpath.js";
import { initTracker, stepTracker, stepObject, snapToObject, pinFind, smearDrift, despikePath, smoothPath, smoothObjPath, smoothPathAt, smoothObjPathAt, posePathAt, applyPoseFixes, applyDirFixes, snapDirsToAnchors } from "./video/postrack.js";
import { solveManualPoses } from "./video/manualpose.js";
import { pixelToGround, groundSpanM, centerGSD, haversineM, bearingDeg as bearingDegGround, rayToGround, groundHomography, pixelToGroundH, groundSpanH, groundKinematics } from "./math/geolocate.js";
import { poseFromGravity, poseQuality, poseFromOrientation, upFromOrientation, gravitySign } from "./capture/pose.js";
import { muxMp4 } from "./video/mp4mux.js";
import { analyze, arbitrateBearings, aspectSpan, covEllipse } from "./math/triangulate.js";
import { trackDirections, kinematics, analyzeTracks, videoKinematics, stereoVideo, mixedStereo } from "./math/kinematics.js";
import { sunPos, moonPos, moonFrac, raDecToAzEl } from "./math/astro.js";
import { fetchAircraft, fetchAircraftAt, fetchAcInfo, rankCandidates, radiusNmForSources, acAzElRange } from "./checks/adsb.js";
import { declination } from "./math/geomag.js";
import { loadSats, loadSatGroup, satsAt, satTrail } from "./checks/satellites.js";
import { fetchWindProfile, balloonVerdict } from "./checks/winds.js";
import { fetchWeatherAt, cloudRangeBound } from "./checks/weather.js";
import { activeShowers } from "./checks/meteorshowers.js";
import { aperture, relMag, colorDesc } from "./checks/photometry.js";
import { fetchAirports } from "./checks/airports.js";
import { fetchLaunches } from "./checks/launches.js";
import { fetchFireballs } from "./checks/fireballs.js";
import { predictedSkyline, skylineElAt, demElevation, detectSkyline, matchSkyline, TERRAIN_ATTRIB } from "./terrain.js";
import { predictedBuildingBoxes, convexHull2, visibleSegs, bboxHit, BLDG_RADIUS_M } from "./buildings.js";
import { fetchPeaks } from "./checks/peaks.js";
import { detectStars, autoStarAlign, blindStarAlign, gridStarAlign } from "./checks/platesolve.js";
import { DEEP_STARS } from "./math/starcatDeep.js";
import { mediaPut, mediaGet, mediaDel, mediaClear } from "./mediaStore.js";
import { parseMediaMeta } from "./exif.js";
import { SHAPES, I3, rotX3, rotY3, rotZ3, mul3, SHAPE_R0, shapeProjNat, shapeWire, sampleShapeAt } from "./shapes.js";
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

/* The fitted model KEYFRAMED at time t: interpolate the attitude + apparent
   size the user marked along the track (sampleShapeAt), then normalise the
   target apparent WIDTH to a real sizeNat through the rotated shape's own
   projection — so a tumbling elongated object still hits the width set on that
   frame. Returns a shapeFit clone (or the original when nothing is keyframed). */
const shapeAt = (shapeFit, track, t, markT) => {
  if (!shapeFit || !isNum(t)) return shapeFit;
  /* the fit's own projected width = the baseline apparent size at markT, so a
     single adjustment ramps from the fit instead of jumping the whole track */
  const wFit = (() => { const pr = shapeProjNat(shapeFit); return Math.hypot(pr.p2.x - pr.p1.x, pr.p2.y - pr.p1.y) || 1; })();
  const s = sampleShapeAt(track, shapeFit, t, { markT, wFit });
  let sf = { ...shapeFit, rotM: s.rotM, roll: 0 };
  if (s.wpx != null) {
    const pr = shapeProjNat(sf);
    const pw = Math.hypot(pr.p2.x - pr.p1.x, pr.p2.y - pr.p1.y) || 1;
    sf = { ...sf, sizeNat: (shapeFit.sizeNat || 1) * s.wpx / pw };
  }
  return sf;
};
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
  statement: "",
  moments: [],
  open: true,
});

/* A MOMENT is an additional timestamped photo of the SAME object from the SAME
   observer. It is shaped like a mini-source so MediaMeasure + SkyAimer can edit
   it unchanged (they read/write `mediaUrl`, `natW/H`, `fovH`, `A.p1/p2`,
   `mediaAim`, …). Once placed, its A.az/A.el @ whenMs becomes one point on the
   observer's single trajectory (see `sourceTrack` in math/kinematics.js). The
   observer's own primary photo is the implicit first moment. */
const makeMoment = (fovH) => ({
  id: Math.random().toString(36).slice(2, 9),
  whenMs: Date.now(),
  tSource: "manual",       // 'exif' once a capture time is mined from the file
  fovH: isNum(fovH) ? fovH : 68,
  natW: null, natH: null,
  A: blankMomentA(),
  B: blankMomentB(),
  track: [],
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
          <div className="readout" style={{ fontSize: 26 }}>{fmtSpeedShort(k.peakSpeed)}</div>
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
                <div className="readout" style={{ fontSize: 22 }}>{fmtSpeedShort(D * k.peakSpeed)}</div>
                <div className="readsub">avg {fmtSpeedShort(D * k.avgSpeed)}{rad ? " · incl. radial" : ""}</div>
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
/* inline loading spinner — replaces the old "…" load indicators everywhere */
.spin{display:inline-block; width:1em; height:1em; vertical-align:-0.15em;
  border:2px solid currentColor; border-right-color:transparent; border-radius:50%;
  animation:spin .7s linear infinite; opacity:.9;}
@keyframes spin{to{transform:rotate(360deg)}}
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
.pinmapwrap{position:relative; isolation:isolate; z-index:0;
  height:min(46vh, 340px);
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
.lmk-cam{color:var(--teal); font-size:15px; font-weight:800;}
.lmk-cam span{color:var(--teal); font-weight:700; left:10px;}
.lmk-obj{color:var(--amber); font-size:17px; font-weight:800;}
.lmk-obj span{color:var(--amber); font-weight:700; left:11px; top:-7px;}
.mappick-modal{position:fixed; inset:0; z-index:4000; display:flex; flex-direction:column;
  background:var(--bg); padding-top:env(safe-area-inset-top);
  /* its Cancel / ✓ buttons live in a footer — without this they sit under the
     home indicator in the installed PWA, same reachability bug the report
     preview had at the top */
  padding-bottom:env(safe-area-inset-bottom);}
.mappick-modal .leaflet-container{width:100%; height:100%; background:#08101F;}
.mappick-modal .leaflet-control-attribution{background:rgba(7,11,20,.55); color:var(--dim); font-size:9px;}
.mappick-modal .leaflet-control-attribution a{color:var(--dim);}
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
/* sensor capture is a full-screen camera — allow landscape (no portrait lock) */
html.capturing .rotate-lock{display:none !important;}
.help-q{width:30px; height:30px; flex:0 0 auto; border-radius:50%; border:1px solid var(--line);
  background:var(--panel2); color:var(--amber); font-weight:800; font-size:15px; line-height:1;
  cursor:pointer; pointer-events:auto; display:inline-flex; align-items:center; justify-content:center;}
.help-q:active{transform:translateY(1px);}
.help-back{position:fixed; inset:0; z-index:4000; background:rgba(3,6,12,.82);
  -webkit-backdrop-filter:blur(2px); backdrop-filter:blur(2px); display:flex; justify-content:center;}
.help-panel{width:100%; max-width:560px; background:var(--bg); border-left:1px solid var(--line);
  border-right:1px solid var(--line); display:flex; flex-direction:column; height:100%;}
.help-head{display:flex; align-items:center; gap:10px;
  padding:calc(10px + env(safe-area-inset-top)) 14px 10px; border-bottom:1px solid var(--line);}
.help-head .ttl{font-family:var(--mono); font-weight:800; letter-spacing:.14em; font-size:13px; flex:1;}
.help-scroll{overflow-y:auto; -webkit-overflow-scrolling:touch; overscroll-behavior:contain;
  padding:12px 14px calc(34px + env(safe-area-inset-bottom));}
.help-index{display:flex; flex-wrap:wrap; gap:6px; margin-bottom:16px;}
.help-index .chip{cursor:pointer;}
.help-sec{margin:0 0 24px; scroll-margin-top:8px;}
.help-sec h3{font-family:var(--mono); font-size:15px; letter-spacing:.05em; color:var(--ink); margin:0 0 6px;}
.help-sec h4{font-size:11px; letter-spacing:.12em; text-transform:uppercase; color:var(--amber); margin:13px 0 5px;}
.help-sec p{font-size:13px; line-height:1.55; color:var(--dim); margin:0 0 8px;}
.help-item{font-size:13px; line-height:1.5; color:var(--ink); padding:3px 0;}
.help-item b{color:var(--teal); font-family:var(--mono); font-weight:700;}
.help-tip{font-size:12px; line-height:1.5; color:var(--amber); background:rgba(245,169,63,.08);
  border:1px solid #7A5A22; border-radius:8px; padding:6px 9px; margin:9px 0;}
.help-top{margin-top:8px; background:transparent; border:0; color:var(--dim); font-size:11px;
  cursor:pointer; padding:2px 0;}
.help-foot{font-size:12px; color:var(--dim); border-top:1px solid var(--line); padding-top:12px; line-height:1.55;}
`;

/* Non-destructive brightness/contrast for DISPLAY only. Values are percentages
   (100 = neutral) stored on the source as `imgAdj`; the ORIGINAL pixels are
   never modified (measurement — star detection, plate solve, marks — always
   reads the raw image). The pixel pass replicates the CSS `brightness()
   contrast()` math EXACTLY, so canvas-baked surfaces (the sky-view warp texture,
   report crops) match CSS-filtered <img> surfaces (measure step, place mode). */
const imgAdjNeutral = (a) => !a || ((a.bri == null || a.bri === 100) && (a.con == null || a.con === 100));
const imgAdjFilter = (a) => imgAdjNeutral(a) ? "none" : `brightness(${(a.bri ?? 100) / 100}) contrast(${(a.con ?? 100) / 100})`;
/* 256-entry brightness→contrast lookup — the transform depends only on the
   input byte, so precompute it once and the per-pixel work is a table read
   instead of two multiplies + a clamp × 3 channels. This is what keeps a
   per-frame bake (world-locked video playback) from bogging: the loop over a
   ~1600 px texture used to do ~13 float ops/pixel every frame. Uint8ClampedArray
   rounds+clamps on assignment, so the [0,255] clamp is free. */
function imgAdjLut(a) {
  const b = (a.bri ?? 100) / 100, c = (a.con ?? 100) / 100;
  const lut = new Uint8ClampedArray(256);
  for (let v = 0; v < 256; v++) lut[v] = (v * b - 127.5) * c + 127.5; // brightness then contrast, matching CSS filter order
  return lut;
}
function applyImgAdj(ctx, w, h, a) {
  if (imgAdjNeutral(a)) return;
  const lut = imgAdjLut(a);
  const id = ctx.getImageData(0, 0, w, h), d = id.data;
  for (let i = 0; i < d.length; i += 4) { d[i] = lut[d[i]]; d[i + 1] = lut[d[i + 1]]; d[i + 2] = lut[d[i + 2]]; }
  ctx.putImageData(id, 0, 0);
}

const ML = ({ children, style }) => <div className="microlabel" style={style}>{children}</div>;

/* ============================================================
   HELP / GUIDE — one linear manual with a section index. A small "?" sits in
   the top-right of every screen (HelpButton) and opens this overlay scrolled to
   that screen's section. Content is data (HELP_SECTIONS); the overlay just
   renders it. Keep entries in sync when controls change — this is the manual.
   ============================================================ */
const HELP_SECTIONS = [
  {
    id: "start", icon: "🛰", title: "How Phodar works",
    intro: "Phodar turns a sighting photo into real numbers. One photo gives honest angular data — which way, how big it looked, how fast it crossed the sky. Two or more viewpoints of the same object triangulate a true fix: position, altitude, real size, speed and heading. You move through four steps, then get a shareable report. Nothing you can't measure is ever invented — Phodar shows warnings instead of confident guesses.",
    groups: [
      { h: "The four steps", items: [
        { t: "1 · The photo", d: "Load your image or video and fit a shape to the object so Phodar knows how large it appeared (its angular size)." },
        { t: "2 · Your position", d: "Set exactly where you stood — map pin, address search, GPS, or typed coordinates — plus the date & time." },
        { t: "3 · The sky view", d: "Seat the photo onto an astronomical dome so its true pointing (compass + up-angle) is fixed. This is the calibration step." },
        { t: "4 · Results", d: "See the fix (or, with one viewpoint, the honest angular numbers), run cross-checks, and grade the quality." },
      ]},
      { h: "On the home screen", items: [
        { t: "📸 New sighting", d: "Clears the current sighting and starts fresh at step 1." },
        { t: "📥 Import a shared sighting", d: "Load a .phodar.json, a Phodar report .html, or a sighting .zip — merges its observers in (this is how a second witness's data joins yours)." },
        { t: "➕ Add a witness / perspective", d: "Add another observer to the SAME sighting — the second viewpoint that makes triangulation possible." },
        { t: "📄 Report", d: "Open the report & share screen." },
        { t: "units: … — tap to switch", d: "Flip every readout in the app between metric (m · km · m/s) and imperial (ft · mi · mph)." },
        { t: "Observer row dots", d: "Green marks which facets are done — photo · position · direction · (trajectory). Open ▸ resumes that observer; ✕ removes them." },
        { t: "＋ Add moment", d: "Under each observer is a moment tree — the primary photo is Moment 1. Add another photo of the SAME object taken a moment later (from the same spot) and place it too; two or more placed photos build that observer's trajectory (direction over time) without any manual drawing. Each moment carries its own capture time (from EXIF, or set by hand)." },
      ]},
    ],
    tips: [
      "One viewpoint is still useful — it pins direction and angular size honestly. It just can't give absolute distance until a second viewpoint is added.",
      "Trajectory has two paths: multiple timestamped photos (moments) placed in the sky, OR — with a single photo/video — tap the path on step 1 with the ⊕ Track points tool. The two interleave; the sky view shows the result.",
    ],
  },
  {
    id: "photo", icon: "📸", title: "Step 1 — The photo",
    intro: "Load the sighting media and tell Phodar how big the object appeared by fitting a 3D wireframe shape over it. The projected silhouette of that shape becomes the measured angular size — the seed for real size once distance is known. The original pixels are never altered; brightness/contrast here is display-only.",
    groups: [
      { h: "Load the media", items: [
        { t: "Load photo or video / Replace media", d: "Pick any image or video from your device." },
        { t: "📎 Auto-filled from the file ✓", d: "When EXIF is present, Phodar reads GPS, time, camera bearing, FOV and model and pre-fills later steps — every field stays editable." },
      ]},
      { h: "Fit a 3D shape (the measurement)", items: [
        { t: "＋ Add object", d: "Opens the shape menu — ● Orb · 🛸 Saucer · 💊 Tic-tac · ▲ Triangle · ✈ Jet · 🛩 Small plane · 🚁 Helicopter · 🕊 Bird · ❖ Drone · 🪼 Jellyfish. Tap one to drop that wireframe on the object; the button then shows the current shape (tap again to change). Not sure? Use ● Orb — it assumes no form and still measures size." },
        { t: "Rotate / move / twist", d: "Drag the shape body to tumble it in 3D, tap to move it, drag the centre dot to fine-place, add a second finger to twist (roll)." },
        { t: "size", d: "Slider (log scale) that sets the object's on-image size — this drives the angular-size number." },
        { t: "color", d: "Recolours the wireframe (hue slider) so it stands out against the photo." },
        { t: "aspect / spin / wingspan / wing pos / tendrils", d: "Shape-specific sliders — tic-tac length:width, flat-craft spin, bird wing width & fore/aft, jellyfish tendril length." },
        { t: "✕ remove shape", d: "Deletes the fitted shape." },
        { t: "Measured angular size", d: "The amber readout at the bottom — e.g. 0.42° (0.9× full-moon width). This is what step 3 and the fix use." },
        { t: "In your words (optional)", d: "A free-text witness statement — shape, colour, motion, sound, how it ended. It's shown in the report as this observer's account, in a \"Witness accounts\" section." },
      ]},
      { h: "Lay down the path & references (the tool row)", items: [
        { t: "◆ 3D object", d: "The default tool — place, size and rotate the wireframe (the angular-size measurement above)." },
        { t: "⊕ Track points", d: "Trace the object's PATH — this is the only place the trajectory is laid (the sky view just shows it, read-only). On a video: scrub the frame slider and tap the object every second or so. On a still: tap where the object was at each moment across the one photo. Two or more points build the trajectory — and on video they become the GUIDE the auto-tracker locks onto during Stabilize." },
        { t: "✎ Adjust", d: "Re-open any dropped point to set that leg's Δt timing, how tight its turn was (hard corner ↔ wide arc), its apparent size (closer/farther), and its rotation (to remove foreshortening). Drag a point to move it; it snaps to the placed object." },
        { t: "🎥 Cam refs (video)", d: "Hand-mark fixed background features (a cloud edge, a star, a ground light) across a few frames to stabilize a clip the automatic pass can't hold — a manual fallback for low-contrast or near-black footage." },
      ]},
      { h: "See it clearly", items: [
        { t: "Pinch-zoom / two-finger pan", d: "Magnify a small object; two-finger drag pans; a second finger never places a point. ×N · reset returns to fit." },
        { t: "Loupe", d: "A magnifier pops up above your fingertip during any drag, showing a sharp, brightness-matched close-up with crosshair for precise placement." },
        { t: "☀ Brightness / ◐ Contrast", d: "Display-only sliders that lift a dark night shot so you can see the object — carries into the sky view and report; measurements still use the original. ↺ reset restores neutral." },
      ]},
      { h: "Video", items: [
        { t: "Scrub / −1 fr / +1 fr", d: "Seek to any frame; step one frame (~1/30 s) at a time. The object's marks stamp whichever frame you fit them on — no separate 'use this frame' step." },
        { t: "⛰ Align on this frame", d: "Scrub to the moment with the clearest horizon or stars and lock it as the ALIGNMENT frame — the sky-view calibration is done there. It's independent of the frame the object was marked on (which may be zoomed in or horizon-free). Leave it unset and the object's own frame is used." },
      ]},
      { h: "Field of view (FOV)", items: [
        { t: "FOV … from the lens metadata ✓", d: "When EXIF carries the lens, FOV is set for you." },
        { t: "Camera field of view", d: "No metadata? Pick a preset (phone main ≈68°, ultra-wide ≈104°, 2×/3×/5×) or enter FOV horizontal by hand. FOV is central to every angle — get it right." },
      ]},
    ],
    tips: [
      "Shared/messaged copies are usually stripped of EXIF. To keep it: Photos → Share → Options (top) → All Photos Data ON, then AirDrop the original.",
      "HEIC files can't expose metadata in-browser — export or share as JPEG to auto-fill GPS, time, bearing and FOV.",
    ],
  },
  {
    id: "position", icon: "📍", title: "Step 2 — Your position",
    intro: "Where you stood is one end of every sight-line, so it has to be right. Set it any way you like, then refine on the map by dragging the ground under the fixed pin. Also set the date/time (drives the Sun/Moon/star positions) and, optionally, which way and how high you looked.",
    groups: [
      { h: "Set your location", items: [
        { t: "Find your spot by name", d: "Type a town, address, or landmark and 🔎 Search. Results are tagged exact address (teal) or road/area (amber — drag the pin to your spot)." },
        { t: "Latitude / Longitude", d: "Type them, or paste a “lat, lon” pair into either field and it splits automatically." },
        { t: "📎 Use the photo's GPS", d: "Copy the location embedded in the photo's EXIF straight into the fields." },
        { t: "Elev + ⛰ Use terrain elevation", d: "Your ground height in metres. The ⛰ button looks up the DEM terrain height at the pin — steadier than phone-GPS altitude (which wobbles ±5 m)." },
      ]},
      { h: "The map", items: [
        { t: "Drag the ground under your pin", d: "The crosshair is fixed at centre; drag the map so it lands on your exact standing spot. YOU marks the pin, ● photo GPS shows the photo's location, ▲ are other observers." },
        { t: "🛰 sat / 🗺 street", d: "Toggle between satellite imagery and street map (label shows the mode you'll switch to). +/− zoom." },
      ]},
      { h: "Time & aim", items: [
        { t: "Sighting date & time", d: "When it happened — anchors the Sun, Moon, stars, satellites and archived aircraft to the real sky." },
        { t: "Viewing direction", d: "The compass bearing you faced (slider + live readout), drawn on the map as the teal aim line down the middle of the field-of-view cone. This is the SAME field as the sky view's placement — set it here and the sky opens aimed there." },
        { t: "Field of view", d: "How wide the shot was, drawn as a cone that reaches ~25 miles out over the map (roads, towns and landmarks show under it — zoom out to see where you were looking). It foreshortens as you raise the up-angle: the far end pulls in and rounds into an ellipse, becoming a full circle around you when you look straight up. Comes from the lens metadata when the photo carries it (locked, ✓); otherwise a slider you set to match the shot — the same FOV the measurement uses. Tap ✥ Move to reposition the pin; leave it off to pan/zoom around freely without moving your spot." },
        { t: "How high you looked", d: "Your up-angle (−20° to straight-up 90°). A side-view diagram pops up as you slide. Metadata has no up/down angle — set it roughly, fine-tune in the sky view." },
        { t: "Camera height off the ground", d: "How high the camera was above ground. Only matters for the 🏙 building layer — raise it if you shot from an upstairs window or balcony; leave at ground for a normal shot." },
      ]},
    ],
    tips: ["No GPS in the photo? Long-press your spot in Google/Apple Maps, copy the coordinates, and paste them into the Latitude field."],
  },
  {
    id: "sky", icon: "🔭", title: "Step 3 — The sky view",
    intro: "The calibration heart of Phodar. Your photo is seated onto a dome showing the real sky at your time and place (Sun, Moon, stars, horizon, terrain skyline). Getting the photo's pointing right here is what makes every downstream number trustworthy. A row of tool buttons — ✥ Place · ⊕ Trajectory · 📏 Size · ⚖ Compare — switches modes; only one is active at a time, and each reveals its own controls.",
    groups: [
      { h: "The four tools (one at a time)", items: [
        { t: "✥ Place", d: "Seat the photo. It's pinned undistorted and fills the space; drag to slide the SKY behind it (grab-style), pinch to calibrate its FOV (fingers apart = tighter), and twist to roll it — the roll pivots on your fingers (or on the first finger if you set one down before the other). ONE AXIS PER GESTURE: the first movement (twist or pinch) claims the gesture so the other can't bleed in — lift your fingers to switch, or use 🎛 fine-tune taps. Line its horizon/ridges onto the dome. Tap ✥ Place again (or Continue) to commit and auto-derive your sight-lines." },
        { t: "⊕ Trajectory", d: "Shows the object's path over the dome — read-only. You lay it down and edit it back on step 1 (⊕ Track points / ✎ Adjust); here it's drawn for reference so you can see it against the real sky." },
        { t: "📏 Size / ⚖ Compare", d: "Gauge distance: read the object's size/altitude at an assumed range, or compare a reference ghost — including by placing it on a map." },
      ]},
      { h: "Get the pointing exact (Place tools)", items: [
        { t: "✦ Auto star align", d: "On a night photo, detects your stars and plate-solves the exact az/el/roll/FOV/lens automatically — no manual lining-up. The most accurate calibration when stars are visible." },
        { t: "⛰ Snap to ridges", d: "One tap matches the photo's skyline to the DEM terrain skyline and applies the az/pitch/roll fix. The calibration answer when you can see a horizon of hills." },
        { t: "✦ Manual star align", d: "The hands-on alternative to auto: pick a named star/planet, aim the crosshair on it in the photo, ✓ Set. One star fixes roll+FOV, two adds lens distortion, three+ is a full solve. It drops to the warped view so you can aim, then ✓ Done aligning returns you; ↺ reset undoes it." },
        { t: "+ / − zoom · ✋ pan", d: "The +/− buttons (right) magnify the photo and sky together to line up fine detail — a distant ridge, a rooftop — without changing the calibration. Once zoomed, ✋ lets you drag around the magnified view; sky-slide also gets finer." },
        { t: "🎛 fine tune", d: "Precise one-axis nudge buttons for roll (⟲ ⟳, 0.2° per tap) and FOV (− ＋, 0.3°). The two-finger twist and pinch are great for the rough placement but bleed into each other at small adjustments — these move exactly one thing. Each tap is undoable." },
        { t: "↩ Undo · Reset placement", d: "Undo steps back the last placement change (a gesture or a button); Reset restores the whole placement to how the screen opened." },
        { t: "color (slider under the tool row)", d: "One hue for every overlay drawn over your photo — the crosshair, the object outline, and the terrain ridge/peak lines — so you can pick a color that stands out against your particular sky or scene. Set it before entering a mode; saved for next time." },
        { t: "🎞 Stabilize video (video only)", d: "Tracks the static background (skyline, stars) through every frame and solves each frame's camera pose — align first (place mode: snap/star-align) so the whole path inherits an accurate anchor. On the MEASURE step, '⛰ Align on this frame' picks WHICH frame the alignment is done on (scrub to the clearest horizon/stars) — independent of the frame the object was marked on; the object still measures on its own frame through the solved path. The button lives OUTSIDE place mode so a running solve can't be nudged; progress shows in the button (n/total). It also auto-tracks the MARKED OBJECT through the clip: during playback the outline rides the real object, and the Object close-up export follows it. BEST RESULTS: on the measure step, use the Track tool to tap the object at a few moments through the clip — 2+ points become a GUIDE, and the tracker only fine-tunes each frame around your trajectory instead of finding the object on its own. Frames with too few background references hold the previous pose and are reported honestly." },
        { t: "▶ world-locked playback", d: "After stabilizing, a ▶ + scrubber appears in look mode. Each frame is drawn at its own solved pose: the sky, terrain and stars stay frozen on the dome while the video frame visibly moves around — the object traces its TRUE angular path. The object outline stays pinned at its marked sky position (the video's object passes through it at the marked frame). ↺ returns to the marked frame; the readout shows each frame's time and how many background references held it." },
        { t: "⬇ export the stabilized clip", d: "Renders the whole clip world-locked — every frame at its own solved pose from a fixed camera — and saves it as a real video file (mp4 on iPhone). Three framings: World view (the dome framing you see in playback, with the az/el grid, pose readout, and every visible sky layer burned in), Max resolution (CLEAN footage, no overlays, at native source detail — sized so the most-zoomed frames keep every pixel), and Object close-up (a clean full-resolution crop centered on the marked object, no overlays). Tap again to cancel. Great as report evidence and for judging stabilization quality frame by frame." },
      ]},
      { h: "Distance, size & the path", items: [
        { t: "⊕ Trajectory (read-only)", d: "The path and its numbered points show here for reference against the real sky, but you lay them down and edit them on step 1 — ⊕ Track points to tap the path, ✎ Adjust for a point's timing (Δt), turn tightness, size and rotation. This is why the sky view no longer needs an aiming crosshair." },
        { t: "📏 size", d: "Object size vs distance — slide an assumed distance to see the size and altitude it implies, with the nearest everyday reference. If there was a cloud deck, it also shows the cloud-base range cap (a below-cloud object can't be farther than that)." },
        { t: "📍 Set distance on a map", d: "In the size or compare tool, opens a satellite map centred near where the photo was taken, with your camera's field-of-view wedge and the object's sight-line drawn on it. Tap (or drag the ✦) where the object was overhead; the straight-line distance from the camera — carried up the sight-line to a true line-of-sight range — sets the assumed distance. Often easier than guessing on the slider: you place it over the ridge/field/town it was above." },
        { t: "⚖ compare", d: "Drop a reference ghost (balloon, drone, aircraft…) at the crosshair and slide its distance to compare its apparent size to your object's — or set that distance on the map." },
        { t: "Path vs moments", d: "The trajectory can come from the points you tap on step 1, OR from two or more timestamped photos (moments) placed in the sky — and the two interleave on one path when you have both. Timing comes from the video's frames, the moments' capture times, or the Δt chips you set." },
      ]},
      { h: "Aim readout & navigation", items: [
        { t: "Top-right readout", d: "Live azimuth + compass + up-angle, and FOV. Re-opening an already-placed photo auto-fits it to the frame; use +/− to re-zoom or ✥ Place to re-adjust." },
        { t: "‹ Back / Continue →", d: "Both commit the placement first, so your calibration is never lost when you leave." },
      ]},
    ],
    tips: [
      "Order of preference for calibration: Auto star-align (night) or Snap to ridges (visible hills) beat eyeballing. Use the Sun/Moon discs — drawn where they really were — to sanity-check your bearing.",
      "See the 🛰 Sky layers section for what every header toggle (Sun, Moon, stars, satellites, Starlink, aircraft, peaks, buildings, cloud, wind) shows.",
      "Clean viewing: ⌃ next to ? tucks the sky-layer toggles away; ⌄ on the bottom row tucks the controls away. The bottom row and the video playback scrubber always stay.",
      "🎛 on the playback row opens the smoothing sliders — 🎥 steadiness (camera path) and 🛸 track smooth (object path). Non-destructive: re-applied from the raw solve. Left keeps hard corners; right smooths an airplane's jitter into its clean curve — heavier smoothing also damps real fast maneuvers in the measured rates.",
      "⚓ on the playback row opens Fix frames: scrub to where the auto-stabilize lost the world lock, drag the photo back onto the true horizon/terrain (two-finger twist tilts it), fine-tune with the always-on nudge taps (az/el arrows, roll, − ＋ photo size), then ⚓ Anchor that frame. Corrections blend smoothly between anchors and hold past the ends; the object trajectory and waypoints move with them (toggle 🛸 to watch live). Re-stabilizing clears anchors.",
    ],
  },
  {
    id: "results", icon: "🎯", title: "Step 4 — Results",
    intro: "With two or more calibrated viewpoints, Phodar intersects the sight-lines and reports the fix — with an honest quality grade and the warnings that matter. With one viewpoint you get angular data and can still run some checks.",
    groups: [
      { h: "The fix", items: [
        { t: "altitude / size / speed", d: "Object height above the reference observer, its true size across, and ground speed if you set Moment A→B (with heading and climb/descent rate)." },
        { t: "quality · baseline · conv", d: "The grade (excellent/good/fair/poor), how far apart the observers were, and the sight-line convergence angle. Wide separation + healthy convergence = trustworthy numbers." },
        { t: "Object ground position", d: "The object's lat/lon on the ground, with a ± uncertainty derived from a ±1° pointing error." },
      ]},
      { h: "Honesty guards", items: [
        { t: "⚠ Bearings don't converge", d: "Names which observer's compass looks off and by how much — usually metal near the phone (see Accuracy)." },
        { t: "⚠ GPS altitudes differ + ⛰ Set every observer's elevation from terrain", d: "Phone-altitude wobble warning, with a one-tap fix that sets every observer's elevation from DEM terrain." },
        { t: "Solution quality (table)", d: "Baseline, convergence angle, ray-miss distance (how close the sight-lines actually pass), and range/baseline ratio — the raw geometry behind the grade." },
      ]},
      { h: "Cross-checks & plots", items: [
        { t: "Top-down plot", d: "A satellite-imagery map showing the observers, their sight-line rays, the fix, and any trajectory — read-only." },
        { t: "✈ Aircraft check (ADS-B)", d: "🛰 Check … aircraft ranks nearby transponder-equipped aircraft by how close they sit to every witness's sight-line — a real match must satisfy all witnesses. Tags: ◉ ON the sight-line, ◎ near, …° off." },
      ]},
    ],
    tips: ["“No ADS-B aircraft in range” rules out airliners, not everything — some military and older light aircraft carry no transponder. Phodar says so rather than overclaiming."],
  },
  {
    id: "report", icon: "📄", title: "Report & share",
    intro: "Produce a self-contained white-paper report of the sighting — embedded data, photo exhibits, plots, and every cross-check ranked in one place. It opens offline and can be re-imported into Phodar by another witness.",
    groups: [
      { h: "What you can export", items: [
        { t: "Report (.html)", d: "A single self-contained page: the fix, quality, photo exhibits with detail crops, top-down + trajectory charts, and the sky-object / wind / aircraft checks." },
        { t: "💾 Share file (.phodar.json)", d: "Just the data — the importable file another observer loads to add their perspective, or that you keep as a backup." },
        { t: "Bundle (.zip)", d: "Report + data + full-resolution photos + videos (the original clip, and the world-locked stabilized render if you exported one) in one download, re-importable into Phodar." },
      ]},
      { h: "Extra checks in the report", items: [
        { t: "Video analysis", d: "For a stabilized, object-tracked clip: the object's dense per-frame angular trajectory measured with NO distance assumption — total sky sweep, average & peak angular rate (°/s, a strong discriminator: satellites track at a near-constant rate, aircraft vary, a hover ≈ 0) and an angular-rate plot. Then a size/distance/speed table showing what every candidate distance implies (or the one triangulated distance if a second observer fixed it), plus its apparent-size range if you sized the object across frames (that recovers toward/away motion). Ends with a keyframe strip — sampled frames with the tracked object marked and captioned (time, az/el, angular size, rate)." },
        { t: "Sky-object check", d: "Flags the Sun, Moon, planets or bright stars within a few degrees of any sight-line — with a Venus warning (the most-reported “UFO”)." },
        { t: "Wind check", d: "Compares the object's motion to winds aloft at its altitude — the balloon test: a free balloon rides the wind at its height. Includes a wind-rose (each altitude's drift arrow, length = speed, with the object's own motion overlaid) and the drift arrow drawn on each photo, so you can see whether the object's apparent motion matches any layer." },
        { t: "Weather & cloud base", d: "Cloud cover, visibility and an estimated cloud base at the sighting time. If the object was below the deck, that caps its range and size for a single witness — drawn right on the size↔distance chart." },
        { t: "Object photometry", d: "Colour and brightness measured from the photo's pixels, plus a rough apparent magnitude when a catalogued star shares the frame (a red/green pair reads as aircraft nav lights)." },
        { t: "Meteor-shower & fireball checks", d: "Annual showers active that night (radiant position vs your sight-line) and bright bolides logged by NASA CNEOS near the time." },
        { t: "Aircraft, airfields & routes", d: "ADS-B traffic ranked against the sight-lines, the best match's origin→destination, and nearby airfields whose approach corridors concentrate low, slow traffic." },
        { t: "Uncertainty ellipse", d: "With two witnesses, the ground-position error is shown as a 1σ ellipse (weakest across the baseline), not a single ± number." },
      ]},
    ],
    tips: ["Every check outputs the same shape — a predicted direction/size/motion and how far it sits from your sight-line — so the report ranks all the mundane explanations in one table."],
  },
  {
    id: "checks", icon: "🛰", title: "Sky layers & cross-checks",
    intro: "The header toggles in the sky view each overlay a real, independently-sourced layer on the dome. They serve two jobs: calibration anchors (things whose true position is known, to check your pointing) and mundane-explanation candidates (things a “UFO” might actually be).",
    groups: [
      { h: "Calibration anchors", items: [
        { t: "☀ Sun / ☾ Moon", d: "Drawn where they really were at your time and place; tap to centre on them. The strongest quick check that your bearing is right." },
        { t: "★ Stars & planets", d: "The real catalog sky — mag-scaled stars, labelled bright stars, glowing planet markers. Auto-on at night. Feeds ✦ Auto star-align." },
        { t: "Terrain skyline (always on)", d: "The dashed DEM horizon (in your chosen overlay color) — mountains and hills as they truly sit. The answer for ⛰ Snap to ridges (a tree/mountain skyline is NOT a true flat horizon)." },
        { t: "⛰ peaks", d: "Labels named summits sitting on the terrain skyline (name + elevation) — each is also a compass landmark." },
      ]},
      { h: "Explanation candidates", items: [
        { t: "✈ aircraft (ADS-B)", d: "Live or archived-at-the-sighting-time air traffic as heading-rotated ✈ glyphs at true az/el, with range, altitude and faint track trails. Tap one for its identity/route." },
        { t: "🛰 satellites / ✦ Starlink", d: "CelesTrak orbital elements propagated to your time — passing satellites and (opt-in) sunlit Starlink trains, a common “string of lights” report. Both draw their ±4-min pass trajectory as a dotted trail (Starlink trails show for trains near your view)." },
        { t: "☁ cloud", d: "Shades the sky region of the dome grey in proportion to the % cloud cover at the sighting time (Open-Meteo, low/mid/high) — a light haze for scattered cloud through to solid grey for overcast — plus the estimated cloud base. It represents how overcast it was (not individual clouds). A low deck also caps a below-cloud object's range & size (see the size tool)." },
        { t: "🎈 wind", d: "Winds-aloft drift arrows layered by height across the dome, coloured by speed — see whether the object could be a balloon riding the wind at its altitude." },
        { t: "🏙 buildings", d: "OSM building footprints as wireframe boxes — for aligning a town/skyline photo. Uses your Camera height off the ground; nudge with ± if rooftops sit wrong." },
      ]},
    ],
    tips: ["A staleness/provenance line appears when data is old (e.g. archived traffic vs live, or aging satellite elements) — Phodar tells you when to treat a layer as approximate."],
  },
  {
    id: "accuracy", icon: "⚖", title: "Accuracy & honest limits",
    intro: "Phodar was validated against ground truth (a rooftop weathervane resolved to ~1 inch of its true span). Just as important, it knows its failure modes and warns you instead of guessing. Read these before trusting — or dismissing — a number.",
    groups: [
      { h: "What limits accuracy", items: [
        { t: "Compass near metal", d: "Phone compasses are sub-degree on foot but 14–66° wrong near metal (in a car, under a steel roof). Phodar cross-checks bearings and names the suspect one — recalibrate away from metal if warned." },
        { t: "GPS altitude wobble", d: "Phone altitude drifts ±5 m. On level ground, set every observer's elevation from terrain (one tap in Results) so a false altitude spread doesn't skew the fix." },
        { t: "Geometry (baseline & convergence)", d: "Two viewpoints too close together, or nearly in line with the object, give a weak fix. Wider separation and a healthy convergence angle earn a better grade." },
        { t: "One viewpoint", d: "A single photo can't give absolute distance — only direction, angular size and angular motion. Add a second perspective for real size, altitude and speed." },
      ]},
    ],
    tips: ["When Phodar grades a fix “poor,” believe it over an impressive-looking number. Honest uncertainty is the product."],
  },
  {
    id: "data", icon: "💾", title: "Saving, sharing & privacy",
    intro: "Your work is saved automatically on this device and stays on it. Nothing is uploaded unless you export and share it yourself.",
    groups: [
      { h: "How data lives", items: [
        { t: "Autosave", d: "Points, positions and settings save to this browser automatically; photos/videos are kept in the browser's local media store and re-attached when you return." },
        { t: "Import / export", d: "Move a sighting between devices or witnesses with the .phodar.json share file, the report .html, or the .zip bundle — all re-importable from the home screen." },
        { t: "units", d: "The metric/imperial choice is remembered for next time." },
      ]},
      { h: "Privacy", items: [
        { t: "On-device by design", d: "Location and photos stay local. Only the cross-checks reach out — anonymously — to public data sources (aircraft, terrain, weather, map tiles) for the area and time you set." },
      ]},
    ],
    tips: ["Starting a New sighting or removing an observer clears their stored photos too — export a report or share file first if you want to keep them."],
  },
];

function HelpButton({ section, style }) {
  const [open, setOpen] = useState(false);
  return (
    <>
      <button className="help-q" title="Help & guide" aria-label="Help and guide" style={style} onClick={() => setOpen(true)}>?</button>
      {open && <HelpOverlay start={section} onClose={() => setOpen(false)} />}
    </>
  );
}

function HelpOverlay({ start, onClose }) {
  const scRef = useRef(null);
  const jump = (id) => { const el = document.getElementById("help-" + id); if (el) el.scrollIntoView({ behavior: "smooth", block: "start" }); };
  useEffect(() => {
    if (!start) return;
    const el = document.getElementById("help-" + start);
    if (el) el.scrollIntoView({ block: "start" });
  }, [start]);
  return (
    <div className="help-back" onClick={onClose}>
      <div className="help-panel" onClick={(e) => e.stopPropagation()}>
        <div className="help-head">
          <img src={phodarLogo} alt="PHODAR" style={{ height: 24, width: "auto", borderRadius: 5, display: "block" }} />
          <span className="ttl">GUIDE</span>
          <button className="btn sm" onClick={onClose}>✕ Close</button>
        </div>
        <div className="help-scroll" ref={scRef}>
          <div className="help-index">
            {HELP_SECTIONS.map((s) => (
              <button key={s.id} className="chip" onClick={() => jump(s.id)}>{s.icon} {s.title}</button>
            ))}
          </div>
          {HELP_SECTIONS.map((s) => (
            <section key={s.id} id={"help-" + s.id} className="help-sec">
              <h3>{s.icon} {s.title}</h3>
              {s.intro && <p>{s.intro}</p>}
              {(s.groups || []).map((g, gi) => (
                <div key={gi}>
                  {g.h && <h4>{g.h}</h4>}
                  {(g.items || []).map((it, ii) => (
                    <div key={ii} className="help-item"><b>{it.t}</b>{it.d ? <span> — {it.d}</span> : null}</div>
                  ))}
                </div>
              ))}
              {(s.tips || []).map((t, ti) => <div key={ti} className="help-tip">💡 {t}</div>)}
              <button className="help-top" onClick={() => scRef.current && scRef.current.scrollTo({ top: 0, behavior: "smooth" })}>↑ back to index</button>
            </section>
          ))}
          <div className="help-foot">Phodar is honest by design — it shows warnings instead of silent guesses. When it says “quality: poor,” trust that over a confident-looking wrong number.</div>
        </div>
      </div>
    </div>
  );
}

/* small inline spinner for loading states (replaces "…" so it's obvious the
   user should wait). Inherits color from its context via currentColor. */
const Spin = ({ style }) => <span className="spin" style={style} aria-label="loading" />;

function Num({ label, value, onChange, unit, ph, after, compact }) {
  return (
    <div>
      {label && <ML style={compact ? { marginBottom: 1, fontSize: 9 } : undefined}>{label}{unit ? <span style={{ opacity: .7 }}> ({unit})</span> : null}</ML>}
      <input inputMode="decimal" value={value ?? ""} placeholder={ph || ""}
        onChange={(e) => onChange(e.target.value)}
        style={compact ? { padding: "6px 8px", fontSize: 12 } : undefined} />
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
  const isVid = media?.kind === "video";
  const fileRef = useRef(null);
  const [triedData, setTriedData] = useState(false);
  const [loading, setLoading] = useState(false);
  const [loadErr, setLoadErr] = useState("");
  const [active, setActive] = useState("shape");
  const [capOpen, setCapOpen] = useState(false);
  const [drag, setDrag] = useState(false);
  const [dispW, setDispW] = useState(0);
  const [winH, setWinH] = useState(() => (typeof window !== "undefined" ? window.innerHeight : 800)); // stable portrait-cap reference
  const [vidT, setVidT] = useState(0);
  const [vidDur, setVidDur] = useState(0);
  const [trkAdv, setTrkAdv] = useState(15); // frames to auto-advance after dropping a track point (½ s at 30 fps)
  const [trkGap, setTrkGap] = useState(1);  // STILL: seconds between tapped trajectory points (a photo has no frame clock)
  const [selTrk, setSelTrk] = useState(-1); // STILL: tap-selected track point (video selects by scrub position instead)
  const [trkAdjust, setTrkAdjust] = useState(false); // Track sub-mode: place points (false) vs adjust size/shape at the nearest point (true)
  const [colorOpen, setColorOpen] = useState(null);  // which colour slider is open in the Track panel: null | "obj" | "pts"
  const [selCref, setSelCref] = useState(0);         // CAM REFS: which reference feature the taps mark (index into source.camRefs)
  const [view, setView] = useState({ z: 1, ox: 0, oy: 0 }); // pinch-zoom/pan of the marking canvas
  const [finger, setFinger] = useState(null);               // last pointer pos (wrapper-relative) for the loupe
  const ptsRef = useRef(new Map());
  const pinchRef = useRef(null);
  const pendingRef = useRef(null); // undecided first touch: tap? drag? or pinch about to start?
  const holdRef = useRef(null);
  const [touching, setTouching] = useState(false); // any finger down on the canvas
  const wrapRef = useRef(null), mediaRef = useRef(null), loupeRef = useRef(null);
  const trkDragRef = useRef(null);   // live position of a loupe-assisted Track drag (committed on lift)
  const trkMoveRef = useRef(null);   // ADJUST mode: index (in trkSorted) of the point being dragged to a new spot
  const trkRotRef = useRef(null);    // ADJUST mode: live 3D-rotation gesture on the selected point's model ({R0,sx,sy,pid,cur})
  const trkHistRef = useRef([]);     // Track-point undo stack (snapshots before each place/delete/clear)

  const natW = src.natW, natH = src.natH;
  const scale = natW && dispW ? dispW / natW : 0;
  const dispH = natH && scale ? natH * scale : 0;
  const TT = (x, y) => [x * scale * view.z + view.ox, y * scale * view.z + view.oy];
  /* Zoom is split so the image stays sharp: SIZE the <img> up to its native
     resolution (elZ) so the browser samples from full-res source pixels, and
     apply only any zoom BEYOND native (extra ≥ 1, where no real detail remains)
     as a cheap transform:scale. natZ is the zoom at which 1 source px = 1 CSS px.
     Composed together they equal the old scale·z, so TT() is unchanged. */
  const natZ = natW && dispW ? natW / dispW : 1;
  const elZ = Math.min(view.z, natZ || view.z);
  const extraZ = elZ > 0 ? view.z / elZ : 1;
  const clampView = (v) => {
    const cw = dispW || 1, chh = dispH || 1;
    return { z: v.z, ox: clampN(v.ox, Math.min(0, cw - cw * v.z), 0), oy: clampN(v.oy, Math.min(0, chh - chh * v.z), 0) };
  };

  /* --- 3D shape fit: projected silhouette writes A.p1/p2; pose is stored --- */
  const syncShape = (sf) => {
    const pr = shapeProjNat(sf);
    const A2 = { ...src.A, p1: pr.p1, p2: pr.p2 };
    /* video: the marks belong to the frame they were DRAWN on — stamp it here.
       Relying on the "✓ Use this frame" button alone let marks and videoTime
       disagree (fit on frame X while videoTime said Y or nothing): the sky
       view then baked the wrong frame and the object tracker seeded its
       template where the object wasn't — track lost immediately
       (field-observed). Scrubbing alone never re-stamps; only touching the
       shape on a new frame moves the mark time with it. */
    if (media?.kind === "video" && isNum(vidT)) { A2.videoTime = vidT; A2.t = vidT.toFixed(2); }
    update({ shapeFit: sf, A: A2 });
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
  /* removing the object clears its edge marks too — A.p1/A.p2 are DERIVED from
     the shape's silhouette (syncShape), so leaving them behind stranded two
     orphan points on the photo until a new shape was added */
  const clearShape = () => { update({ shapeFit: null, A: { ...src.A, p1: null, p2: null } }); setActive("shape"); setShapePicker(false); };
  const [shapeMag, setShapeMag] = useState(false);
  const [shapePicker, setShapePicker] = useState(false);   // the collapsed "＋ Add object" shape menu
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
  /* the Track undo stack is per-observer — clear it when the source changes
     (the component instance is reused, so refs would otherwise leak across) */
  useEffect(() => { trkHistRef.current = []; }, [src.id]);

  const measureWrap = useCallback(() => {
    if (wrapRef.current) setDispW(wrapRef.current.clientWidth);
    /* portrait cap height — update only on a BIG change (orientation flip),
       ignoring the ±small iOS URL-bar deltas that fire while scrolling, so the
       video window doesn't jitter its size every scrubbed frame */
    if (typeof window !== "undefined") setWinH((h) => Math.abs(window.innerHeight - h) > 120 ? window.innerHeight : h);
  }, []);
  useEffect(() => {
    measureWrap();
    window.addEventListener("resize", measureWrap);
    return () => window.removeEventListener("resize", measureWrap);
  }, [measureWrap, src.mediaUrl, natW, natH]); // natW/natH: the portrait height-cap changes the wrap width once dimensions are known

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

  /* Load a File into the source: mine its EXIF, normalize on display, reset marks.
     `opts.sensorPose` (from the in-app sensor capture) overlays the up/down angle
     and roll the photo's EXIF can't carry — see the SENSOR OVERLAY note below. */
  const ingestFile = async (f, opts = {}) => {
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
    trkHistRef.current = []; // new media is a fresh start — don't let Undo reach back into the old clip's track
    update({
      mediaUrl: url, mediaKind: kind, mediaNorm: false,
      natW: null, natH: null, meta: null, capture: null,
      A: { ...src.A, p1: null, p2: null }, B: { ...src.B, pb: null }, track: [],
    });
    if (kind === "video") mediaPut(src.id, { kind: "video", data: f }); // survives reload via IndexedDB
    const sp = opts.sensorPose || null;
    /* instrumented capture: the continuous attitude log rides on the source and
       is fused with the visual solve later (src/video/sensorpath.js) */
    const sensorPath = Array.isArray(opts.sensorPath) && opts.sensorPath.length > 4 ? opts.sensorPath : null;
    if (sensorPath) update({ sensorPath });
    /* mine the file for EXIF / QuickTime metadata and AUTO-APPLY it —
       the photo is the authority on its own capture conditions */
    f.arrayBuffer().then((buf) => {
      const m = parseMediaMeta(buf, kind === "video");
      const patch = {};
      if (sensorPath) patch.sensorPath = sensorPath;
      let meta;
      if (!m) {
        /* valid pixels but no GPS/time/bearing: HEIC can't expose it in-browser,
           or the file was re-encoded and stripped in sharing. Say so instead of
           failing silently — otherwise it reads as a load bug. */
        meta = (/hei[cf]/i.test(f.type) || /\.hei[cf]$/i.test(f.name || "")) ? { heic: true } : { stripped: true };
      } else {
        meta = m;
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
            meta = { ...m, decl: +dec.toFixed(2), azTrue: +azUse.toFixed(1) };
          }
          patch.mediaAim = { az: azUse, el: 15, roll: 0 }; // pre-aims the sky placement
          if (!isNum(src.A?.az)) patch.A = { ...src.A, p1: null, p2: null, az: azUse.toFixed(1) };
        }
      }
      /* SENSOR OVERLAY (in-app capture): the phone measured the pose the shutter
         couldn't record. el/roll come from the sensors (EXIF never carries them);
         azimuth prefers the photo's OWN EXIF compass (read at the true shutter),
         falling back to the sensor heading (sampled when you tapped). GPS/time
         fill any gaps. The full-res native photo keeps its megapixels + real FOV;
         the sensors add only what EXIF drops. */
      if (sp) {
        const exifAz = isNum(patch.mediaAim?.az) ? patch.mediaAim.az
          : (isNum(meta?.azTrue) ? meta.azTrue : (isNum(meta?.az) ? meta.az : null));
        const az = isNum(exifAz) ? exifAz : (isNum(sp.pose?.az) ? sp.pose.az : (isNum(sp.heading) ? ((sp.heading % 360) + 360) % 360 : 0));
        patch.mediaAim = { az: +(+az).toFixed(1), el: isNum(sp.pose?.el) ? sp.pose.el : 15, roll: isNum(sp.pose?.roll) ? sp.pose.roll : 0 };
        meta = { ...(meta || {}), sensor: true, ...(isNum(sp.heading) ? { sensorAz: +(((sp.heading % 360) + 360) % 360).toFixed(1) } : {}) };
        patch.capture = { heading: sp.heading, compassAcc: sp.compassAcc, gravity: sp.gravity, elSign: sp.elSign, orient: sp.orient, raw: sp.raw, gps: sp.gps, pose: sp.pose, whenMs: sp.whenMs };
        if (!isNum(patch.lat) && sp.gps && isNum(sp.gps.lat)) { patch.lat = sp.gps.lat.toFixed(6); patch.lon = sp.gps.lon.toFixed(6); if (isNum(sp.gps.alt)) patch.alt = sp.gps.alt.toFixed(0); }
        if (!isNum(patch.whenMs) && sp.whenMs) patch.whenMs = sp.whenMs;
      }
      patch.meta = meta;
      update(patch);
    }).catch(() => { if (sp && sp.pose) update({ meta: { sensor: true }, mediaAim: { az: sp.pose.az, el: sp.pose.el, roll: sp.pose.roll }, capture: { heading: sp.heading, compassAcc: sp.compassAcc, gravity: sp.gravity, elSign: sp.elSign, gps: sp.gps, pose: sp.pose, whenMs: sp.whenMs } }); });
  };
  const onFile = (e) => { const f = e.target.files && e.target.files[0]; e.target.value = ""; if (f) ingestFile(f); };

  /* an in-app SENSOR capture (ENABLE_CAPTURE): the frame arrives with its pose
     already measured — write the same fields the EXIF path fills, but from the
     phone's motion sensors, so the sky view opens already aimed. mediaAim is a
     SEED (snap-to-ridges / star-align refine it); the raw sensor block is kept
     for the report. */
  const applyCapture = (cap) => {
    /* FULL-RES path: the shot came from the native camera (`<input capture>`) at
       full megapixels with its own EXIF. Route it through ingestFile so it keeps
       resolution + real FOV/GPS, and overlay the sensor pose EXIF can't carry. */
    if (cap.file) {
      ingestFile(cap.file, {
        sensorPose: { pose: cap.pose, heading: cap.heading, compassAcc: cap.compassAcc, gravity: cap.gravity, raw: cap.raw, gps: cap.gps, whenMs: cap.whenMs },
        /* instrumented video: the continuous attitude log rides along */
        sensorPath: Array.isArray(cap.sensorPath) && cap.sensorPath.length > 4 ? cap.sensorPath : null,
      });
      setCapOpen(false);
      return;
    }
    /* QUICK path: an instant getUserMedia frame (lower-res, pose exactly synced) */
    trkHistRef.current = [];
    const patch = {
      mediaUrl: cap.dataUrl, mediaKind: "image", mediaNorm: true,
      natW: cap.w, natH: cap.h,
      A: { ...src.A, p1: null, p2: null }, B: { ...src.B, pb: null }, track: [],
      whenMs: cap.whenMs || Date.now(),
      meta: { sensor: true, ...(isNum(cap.heading) ? { azTrue: +(((cap.heading % 360) + 360) % 360).toFixed(1) } : {}), ...(cap.gps && isNum(cap.gps.lat) ? { lat: +cap.gps.lat.toFixed(6), lon: +cap.gps.lon.toFixed(6), ...(isNum(cap.gps.alt) ? { alt: +cap.gps.alt.toFixed(0) } : {}) } : {}) },
      capture: { heading: cap.heading, compassAcc: cap.compassAcc, gravity: cap.gravity, raw: cap.raw, gps: cap.gps, pose: cap.pose, whenMs: cap.whenMs },
    };
    if (cap.pose) patch.mediaAim = { az: cap.pose.az, el: cap.pose.el, roll: cap.pose.roll };
    if (cap.gps && isNum(cap.gps.lat)) { patch.lat = cap.gps.lat.toFixed(6); patch.lon = cap.gps.lon.toFixed(6); if (isNum(cap.gps.alt)) patch.alt = cap.gps.alt.toFixed(0); }
    update(patch);
    mediaPut(src.id, { kind: "image", data: cap.dataUrl });
    setCapOpen(false);
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
    /* NAME the likely cause. The common one is cross-platform: iPhones shoot
       HEIC/HEVC, and a browser that isn't Safari usually can't decode either —
       so an Android or desktop witness handed an iPhone original sees a file
       that simply won't open, and a generic message reads as an app bug. */
    const f = fileRef.current, n = (f && f.name) || "", t = (f && f.type) || "";
    const heic = /hei[cf]/i.test(t) || /\.hei[cf]$/i.test(n);
    const hevc = /\.(mov|mp4)$/i.test(n) || /quicktime/i.test(t);
    setLoadErr(heic
      ? "This is an Apple HEIC photo, which only Safari can display. On the iPhone that took it, share or export it as JPEG (Settings › Camera › Formats › Most Compatible shoots JPEG from now on) — that also lets Phodar read its GPS, time and lens data."
      : hevc
        ? "The browser couldn't decode that video — iPhone clips are often HEVC, which Safari plays but most other browsers don't. Open it in Safari, or re-export the clip as H.264."
        : "The browser refused to display that file. Try a smaller image (a screenshot of it works) or another format.");
  };

  /* Force the first video frame to render. A muted, unplayed <video> shows
     blank on iOS Safari (and often desktop) until a frame is decoded — the
     bug where the clip appeared only after leaving the step and coming back
     (a remount over a now-buffered file). A hair of seek triggers the paint.
     Guarded to fire once, near t=0, so it never fights the scrubber. */
  const restoreRunRef = useRef(0);
  const paintFirstFrame = () => {
    const el = mediaRef.current;
    if (!el || media?.kind !== "video") return;
    /* restore the frame the object was marked on when revisiting this step —
       otherwise it reloads at the start and loses the mark. Falls back to a
       hair past 0 so iOS still decodes a frame instead of showing blank.
       iOS Safari CLAMPS currentTime while the seekable range is still empty
       (and never fires loadeddata for a fresh video), so a single seek can
       silently land back at 0 — field-observed as "the frame I chose resets
       after a refresh". Verify the seek actually LANDED and retry on a short
       timer until it does; any user scrub (seek()) cancels the retries. */
    const marked = isNum(src?.A?.videoTime) ? +src.A.videoTime : 0;
    const target = marked > 0.01 ? marked : Math.min(0.04, (el.duration || 1) / 4);
    const run = ++restoreRunRef.current;
    const tryTo = (n) => {
      const v2 = mediaRef.current;
      if (!v2 || restoreRunRef.current !== run) return;
      try { if (Math.abs(v2.currentTime - target) > 0.02) { v2.currentTime = target; setVidT(target); } } catch (e) { /* not seekable yet — the timer below retries */ }
      if (n < 12) setTimeout(() => {
        const v3 = mediaRef.current;
        if (v3 && restoreRunRef.current === run && Math.abs(v3.currentTime - target) > 0.05) tryTo(n + 1);
      }, 250);
    };
    tryTo(0);
  };
  /* The seek nudge alone isn't always enough on the FIRST load of a fresh
     file: iOS Safari can leave a brand-new blob-URL <video> blank until the
     decoder actually runs (field bug: the clip only appeared after leaving the
     step and coming back — a remount over a now-buffered file). A muted
     play()→pause() forces a frame out of the decoder; runs once per file and
     re-lands on the marked/first frame afterwards. */
  const kickedUrlRef = useRef(null);
  const kickVideoPaint = () => {
    const el = mediaRef.current;
    if (!el || media?.kind !== "video" || kickedUrlRef.current === media.url) return;
    kickedUrlRef.current = media.url;
    const kick = () => {
      const v = mediaRef.current; if (!v) return;
      try {
        const p = v.play();
        if (p && p.then) p.then(() => { v.pause(); paintFirstFrame(); }).catch(() => { /* refused (e.g. Low Power Mode) — the retry below and the seek nudge remain */ });
        else { v.pause(); paintFirstFrame(); }
      } catch (e) { /* the seek nudge already ran */ }
    };
    kick();
    /* Low Power Mode (and first-load races) can refuse the first play() —
       one delayed retry when no frame data has arrived yet */
    setTimeout(() => { const v = mediaRef.current; if (v && v.readyState < 2) kick(); }, 700);
  };
  const onLoaded = () => {
    const el = mediaRef.current;
    if (!el) return;
    setView({ z: 1, ox: 0, oy: 0 });
    /* PORTRAIT media with NO lens metadata: the generic 68° default is a
       LONG-side (landscape) FOV — held sideways, the horizontal FOV is much
       narrower (tan-scaled by the aspect: 9:16 ≈ 41.5°). Convert once at
       load, only while fovH is still the untouched default, so a user-set
       value is never clobbered; EXIF (which handles orientation itself)
       overrides later anyway. */
    const autoPortraitFov = (nw2, nh2) => {
      if (!(nw2 > 0 && nh2 > nw2)) return;
      if (isNum(src.meta?.fovH) || +src.fovH !== 68) return;
      return +(2 * Math.atan(Math.tan(34 * D2R) * (nw2 / nh2)) * R2D).toFixed(1);
    };
    if (media.kind === "video") {
      const pf = autoPortraitFov(el.videoWidth, el.videoHeight);
      update(pf ? { natW: el.videoWidth, natH: el.videoHeight, fovH: pf } : { natW: el.videoWidth, natH: el.videoHeight });
      setVidDur(el.duration || 0);
      paintFirstFrame(); // iOS Safari leaves a <video> blank until it decodes a frame
      /* the decoder kick MUST hang off loadedMETADATA: iOS Safari doesn't
         fetch media data for a fresh video until playback is initiated, so
         loadeddata (where the kick first lived) can simply never fire —
         which was exactly the "blank until you leave and come back" bug */
      kickVideoPaint();
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
          /* Keep the working copy as sharp as iOS Safari's canvas allows, so
             the image holds up when the artist pinch-zooms to mark the object
             (the sky-view warp uses its own 1280px texture). iOS caps canvas
             AREA (~16.7 Mpx) — NOT side length — so scale by area: a 12 MP phone
             photo passes through at full resolution; 24/48 MP shots downscale
             only as far as the ceiling forces, keeping far more detail than the
             old flat 4300px side cap. The area cap also protects near-square
             images the old side cap could push over the limit. Near-lossless
             JPEG (0.98) makes the one re-encode (needed to bake out EXIF
             orientation) visually invisible. */
          const AREA_MAX = 16.0e6;                       // headroom under the ~16.7 Mpx iOS ceiling
          const SIDE_MAX = 4600;                         // GPU/side guard for extreme aspect ratios
          let sc = Math.min(1, SIDE_MAX / Math.max(nw, nh));
          if (nw * nh * sc * sc > AREA_MAX) sc = Math.sqrt(AREA_MAX / (nw * nh));
          const W = Math.max(1, Math.round(nw * sc)), Hh = Math.max(1, Math.round(nh * sc));
          const cv = document.createElement("canvas");
          cv.width = W; cv.height = Hh;
          cv.getContext("2d").drawImage(el, 0, 0, W, Hh); // modern browsers draw the ORIENTED image
          const durl = cv.toDataURL("image/jpeg", 0.98);
          const pf = autoPortraitFov(W, Hh);
          update(pf ? { mediaUrl: durl, mediaNorm: true, natW: W, natH: Hh, fovH: pf } : { mediaUrl: durl, mediaNorm: true, natW: W, natH: Hh });
          mediaPut(src.id, { kind: "image", data: durl }); // survives reload via IndexedDB
          setLoading(false); setLoadErr("");
          measureWrap();
          return;
        } catch (err) { /* canvas blocked — fall through with guarded naturals */ }
      }
      {
        const pf = autoPortraitFov(nw, nh);
        update(pf ? { natW: nw, natH: nh, mediaNorm: true, fovH: pf } : { natW: nw, natH: nh, mediaNorm: true });
      }
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
    if (pd.mode === "cref") { markCref(pd.nat.x, pd.nat.y); return; }  // tap = mark the selected reference on this frame
    if (pd.mode === "trk") {
      if (trkAdjust) { if (!isVid) selectNearestTrk(pd.nat); return; }   // adjust: video scrubs to select, still taps to select
      if (isVid) {
        const el = mediaRef.current;
        const tv = el ? el.currentTime : vidT;
        addTrkPt({ t: +tv.toFixed(3), x: pd.nat.x, y: pd.nat.y });
        if (trkAdv > 0) seek(Math.min(vidDur, tv + trkAdv * 0.03337));
      } else {
        const sp = snapPlace(pd.nat.x, pd.nat.y);   // tapping the object pins the path to it
        addTrkPt({ t: +stillNextT().toFixed(3), ...sp });   // still: sequential time from the gap selector
      }
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
      /* same twist-to-roll, but ADJUSTING the selected track point's model */
      if (drag && active === "trk" && trkRotRef.current && !trkMoveRef.current && src.shapeFit) {
        killPending();
        const anchorId = e.pointerId, driverId = trkRotRef.current.pid;
        const anc = ptsRef.current.get(anchorId), drv = ptsRef.current.get(driverId) || anc;
        twistRef.current = {
          anchorId, driverId, trk: true,
          a0: Math.atan2(drv.y - anc.y, drv.x - anc.x),
          R0: trkRotRef.current.cur || ptRotOf(selIdx) || I3,
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
    if (active === "trk" && !media) return;   // trk works on a still OR a video (lay the recalled path on the photo)
    if (active === "cref" && media?.kind !== "video") return;   // camera refs are a video-only stabilization aid
    if (active === "shape" && !src.shapeFit) return; // pick a shape first — no stray marks
    const p = toNat(e.clientX, e.clientY);
    const shapeMode = active === "shape" && !!src.shapeFit;
    let shapeCenter = false;
    if (shapeMode) {
      const [scx, scy] = TT(src.shapeFit.cx, src.shapeFit.cy);
      shapeCenter = Math.hypot((e.clientX - r.left) - scx, (e.clientY - r.top) - scy) < 22;
    }
    const hit = active === "trk" || shapeMode ? null : nearest(p);
    /* ADJUST mode: was the press ON an existing track point? If so, dragging
       MOVES that point (grabbed index remembered here, acted on in onMove). */
    let trkGrab = -1;
    if (active === "trk" && trkAdjust && trkSorted.length) {
      let bd = 44 / (scale * (view.z || 1));
      trkSorted.forEach((pt, i) => { const dd = Math.hypot(pt.x - p.x, pt.y - p.y); if (dd < bd) { bd = dd; trkGrab = i; } });
    }
    pendingRef.current = {
      id: e.pointerId,
      mode: active === "trk" ? "trk" : active === "cref" ? "cref" : shapeMode ? (shapeCenter ? "shapeMove" : "shape") : "mark",
      key: active === "trk" || shapeMode ? null : (hit || active),
      grab: trkGrab,
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
        const m = mul3(rotZ3((a1 - tw.a0) * R2D), tw.R0); // roll about the view axis
        tw.cur = m; // live orientation, so lifting back to one finger continues from here
        if (tw.trk) setPtRotM(m);
        else { const nsf = { ...src.shapeFit, rotM: m }; syncShape(nsf); shapeLoupeFor(nsf); }
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
        if (pd.mode === "cref") {
          /* cam refs: a drag is loupe-assisted placement (the features are
             faint/small) — magnifier follows the finger, the mark commits where
             you LIFT. A clean tap still places instantly. */
          killPending();
          trkDragRef.current = toNat(e.clientX, e.clientY);
          setDrag(true);
          setFinger({ x: e.clientX - r.left, y: e.clientY - r.top, cx: e.clientX, cy: e.clientY });
          requestAnimationFrame(() => requestAnimationFrame(() => { if (trkDragRef.current) drawLoupe(trkDragRef.current); }));
          return;
        }
        if (pd.mode === "trk") {
          if (trkAdjust) {
            /* adjust mode: a drag never PLACES a point, but grabbing an existing
               one MOVES it (loupe-assisted), live, until you lift. */
            if (pd.grab >= 0) {
              killPending();
              pushTrkHist();                    // one undo for the whole drag
              trkMoveRef.current = pd.grab;
              setSelTrk(pd.grab);               // keep the moved point selected for the panel
              const np = toNat(e.clientX, e.clientY);
              trkDragRef.current = np;
              setDrag(true);
              setFinger({ x: e.clientX - r.left, y: e.clientY - r.top, cx: e.clientX, cy: e.clientY });
              moveTrkTo(pd.grab, np.x, np.y);
              requestAnimationFrame(() => requestAnimationFrame(() => { if (trkDragRef.current) drawLoupe(trkDragRef.current); }));
              return;
            }
            /* empty space with a point selected + a shape fitted: drag ROTATES
               that point's model in 3D (front face follows the finger), just
               like the original object-placement rotate gesture. */
            if (selIdx >= 0 && src.shapeFit) {
              killPending();
              pushTrkHist();
              trkRotRef.current = { R0: ptRotOf(selIdx), sx: e.clientX, sy: e.clientY, pid: e.pointerId, cur: ptRotOf(selIdx) };
              setDrag(true); setFinger(null);
              return;
            }
            killPending(); return;              // nothing selected: dragging does nothing
          }
          /* Track DRAG = loupe-assisted placement (the object is almost
             always small): magnifier follows the finger, the point commits
             where you LIFT. A clean tap still places instantly. */
          killPending();
          trkDragRef.current = toNat(e.clientX, e.clientY);
          setDrag(true);
          setFinger({ x: e.clientX - r.left, y: e.clientY - r.top, cx: e.clientX, cy: e.clientY });
          requestAnimationFrame(() => requestAnimationFrame(() => { if (trkDragRef.current) drawLoupe(trkDragRef.current); }));
          return;
        }
        commitPending(true, toNat(e.clientX, e.clientY), { x: e.clientX - r.left, y: e.clientY - r.top, cx: e.clientX, cy: e.clientY });
      }
      return;
    }
    if (!drag) return;
    const p = toNat(e.clientX, e.clientY);
    if (active === "cref") {
      if (!trkDragRef.current) return;
      trkDragRef.current = p;
      setFinger({ x: e.clientX - r.left, y: e.clientY - r.top, cx: e.clientX, cy: e.clientY });
      drawLoupe(p);
      return;
    }
    if (active === "trk") {
      if (trkRotRef.current) {
        /* adjust: rotate the selected point's model — same mapping as the shape
           rotate (drag-right yaws, drag-down pitches). */
        const rr = trkRotRef.current, k = 0.45;
        const m = mul3(rotX3(-(e.clientY - rr.sy) * k), rotY3((e.clientX - rr.sx) * k));
        rr.cur = mul3(m, rr.R0);
        setPtRotM(rr.cur);
        return;
      }
      if (!trkDragRef.current) return;
      trkDragRef.current = p;
      setFinger({ x: e.clientX - r.left, y: e.clientY - r.top, cx: e.clientX, cy: e.clientY });
      if (trkMoveRef.current != null) moveTrkTo(trkMoveRef.current, p.x, p.y);   // adjust: drag the point live
      drawLoupe(p);
      return;
    }
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
      const wasTrk = twistRef.current.trk;
      twistRef.current = null;
      const rem = [...ptsRef.current.entries()][0];
      if (rem && src.shapeFit && drag) {
        if (wasTrk && active === "trk") trkRotRef.current = { R0: cur, sx: rem[1].x, sy: rem[1].y, pid: rem[0], cur };
        else if (active === "shape") rotRef.current = { R0: cur, sx: rem[1].x, sy: rem[1].y, pid: rem[0] };
      }
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
        if (active === "trk" && !trkAdjust && trkDragRef.current && e.type !== "pointercancel") {
          /* commit the loupe-assisted Track drag where the finger lifted */
          const p2 = trkDragRef.current;
          if (isVid) {
            const el2 = mediaRef.current;
            const tv = el2 ? el2.currentTime : vidT;
            addTrkPt({ t: +tv.toFixed(3), x: p2.x, y: p2.y });
            if (trkAdv > 0) seek(Math.min(vidDur, tv + trkAdv * 0.03337));
          } else {
            const sp = snapPlace(p2.x, p2.y);
            addTrkPt({ t: +stillNextT().toFixed(3), ...sp });
          }
        }
        if (active === "cref" && trkDragRef.current && e.type !== "pointercancel") {
          markCref(trkDragRef.current.x, trkDragRef.current.y);   // commit the loupe-assisted cam-ref where the finger lifted
        }
        trkDragRef.current = null;
        trkMoveRef.current = null;
        trkRotRef.current = null;
        if (active === "p1" && !src.A.p2) setActive("p2");
      }
    }
  };

  /* safety valve: app-switch or system gesture mid-touch must release the lock */
  useEffect(() => {
    const hardReset = () => {
      ptsRef.current.clear(); pinchRef.current = null; twistRef.current = null; trkDragRef.current = null; trkMoveRef.current = null; trkRotRef.current = null;
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
    let pxPerNat = Math.max(2, 1.5 * (view.z || 1) * scale); // half the old zoom — more surrounding context to line a feature up
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
    restoreRunRef.current++;   // a user scrub owns the frame — cancel any marked-frame restore retries
    if (el && media?.kind === "video") { el.currentTime = t; setVidT(t); }
  };

  const ang = angSizeFromPoints(src.A.p1, src.A.p2, natW, natH, +src.fovH);
  /* --- per-frame apparent size: resize the fitted shape ON a track point's
     own video frame. The size CHANGE across frames is what recovers the
     radial (closer/farther) side of the trajectory — track points store
     `wpx` (native px width at their frame) plus `ang` (degrees via the
     current fovH; re-derived from the solved per-frame FOV after
     stabilization, so a camera zoom can't masquerade as approach). --- */
  const trkSorted = [...(src.track || [])].filter((p) => p.x != null && isNum(p.t)).sort((a, b) => a.t - b.t);
  const trackHue = isNum(src.trackHue) ? +src.trackHue : 210;       // track-point colour (recolourable in the Track panel)
  const trkCol = (a = 1) => `hsla(${trackHue},90%,68%,${a})`;
  /* Track-point UNDO: snapshot the current list BEFORE any place/delete/clear,
     so Undo steps back through ALL of them (not just the last placement). The
     stack is session-local (a ref) and reset when the observer changes. */
  const pushTrkHist = () => {
    trkHistRef.current.push(JSON.stringify(src.track || []));
    if (trkHistRef.current.length > 40) trkHistRef.current.shift();
  };
  const undoTrack = () => {
    if (trkHistRef.current.length) { update({ track: JSON.parse(trkHistRef.current.pop()) }); return; }
    update({ track: (src.track || []).slice(0, -1) }); // no history (e.g. after reload) → drop the last point
  };
  const addTrkPt = (pt) => {
    pushTrkHist();
    /* a new ANCHOR point (snapped onto the object) is the ONLY anchor — drop the
       flag from any earlier point */
    const base = pt.anchor ? (src.track || []).map((p) => { if (!p.anchor) return p; const { anchor, ...rest } = p; return rest; }) : (src.track || []);
    update({ track: [...base, pt] });
  };
  /* STILL trajectory helpers — a photo has no frame clock, so each tapped point
     is stamped `t` = (latest so far) + the chosen gap, and selection is by TAP
     (find the nearest dot) instead of the video path's scrub position. */
  const stillNextT = () => {
    const ts = (src.track || []).map((p) => +p.t).filter(isNum);
    return ts.length ? Math.max(...ts) + Math.max(0.05, +trkGap || 1) : 0;
  };
  /* placing a point ONTO the fitted object snaps it to the object's centre and
     makes it the trajectory anchor — so tapping the object where it actually
     appears in the photo pins the whole recalled path to its measured direction,
     with no separate step. Radius scales with the object's on-screen size. */
  const snapPlace = (x, y) => {
    if (!src.shapeFit) return { x, y };
    const cx = src.shapeFit.cx, cy = src.shapeFit.cy, rad = Math.max(28 / (scale * (view.z || 1)), wFit * 0.6);
    return Math.hypot(x - cx, y - cy) <= rad ? { x: cx, y: cy, anchor: true } : { x, y };
  };
  /* ADJUST mode: drag a placed point to a new spot on the photo. Timing, size,
     attitude and turn ride along. Like PLACING, the drag SNAPS to the fitted 3D
     object: come within the object's radius and the point locks to its centre
     and becomes the trajectory anchor (the sole point pinned to the measured
     object direction — any previous anchor is cleared); drag away and it lets go.
     Dragging the anchor thus re-references the whole recalled path, which is the
     point. */
  const moveTrkTo = (i, x, y) => {
    if (i < 0 || i >= trkSorted.length) return;
    const sp = snapPlace(x, y);          // {x,y} or {x:cx,y:cy,anchor:true} when on the object
    const snap = !!sp.anchor;
    update({ track: trkSorted.map((p, k) => {
      if (k === i) { const { anchor, ...rest } = p; return snap ? { ...rest, x: sp.x, y: sp.y, anchor: true } : { ...rest, x: sp.x, y: sp.y }; }
      if (snap && p.anchor) { const { anchor, ...rest } = p; return rest; }   // only one anchor at a time
      return p;
    }) });
  };
  /* ── CAMERA REFERENCE FEATURES (manual stabilization fallback) ──────────
     For clips the auto stabilizer can't solve (dark, soft clouds, near-black
     stretches), the user hand-marks a few WORLD-FIXED features across frames;
     the sky view then solves each frame's pose from them (solveManualPoses).
     source.camRefs = [{ marks: [{t,x,y}] }, …]; one feature, tracked frame to
     frame. Marking is tap-only + pinch-zoom for precision — no drag, so it
     can't fight the object-track gestures. Self-contained: delete this block +
     the cref render/panel + the module to remove the feature entirely. */
  const camRefs = src.camRefs || [];
  const FRAME_EPS = 0.05;   // "this frame" tolerance in seconds (~1 frame)
  const crefAtFrame = (r) => (r?.marks || []).find((m) => Math.abs(+m.t - vidT) < FRAME_EPS);
  const addCref = () => { const next = [...camRefs, { marks: [] }]; update({ camRefs: next }); setSelCref(next.length - 1); };
  const markCref = (x, y) => {
    let refs = camRefs.map((r) => ({ ...r, marks: [...(r.marks || [])] }));
    let i = selCref;
    if (i < 0 || i >= refs.length) { refs = [...refs, { marks: [] }]; i = refs.length - 1; setSelCref(i); }
    const marks = refs[i].marks.filter((m) => Math.abs(+m.t - vidT) >= FRAME_EPS); // replace this frame's mark
    marks.push({ t: +vidT.toFixed(3), x, y });
    marks.sort((a, b) => a.t - b.t);
    refs[i] = { ...refs[i], marks };
    update({ camRefs: refs });
  };
  const delCref = (i) => { const next = camRefs.filter((_, k) => k !== i); update({ camRefs: next }); setSelCref(Math.max(0, Math.min(i, next.length - 1))); };
  const selectNearestTrk = (nat) => {
    if (!trkSorted.length) { setSelTrk(-1); return; }
    let bi = -1, bd = 40 / (scale * (view.z || 1));   // ~40 screen px tap radius
    trkSorted.forEach((p, i) => { const d = Math.hypot(p.x - nat.x, p.y - nat.y); if (d < bd) { bd = d; bi = i; } });
    setSelTrk(bi);
  };
  /* which placed point the size/attitude controls target. Normally the point
     owning the frame you're on (within 0.55 s); in ADJUST mode the NEAREST
     placed point regardless of distance — scrub anywhere and the model snaps
     to the closest point so you can tune its size/attitude. */
  const szIdx = (media?.kind === "video" && src.shapeFit && trkSorted.length) ? (() => {
    let bi = -1, bd = trkAdjust ? Infinity : 0.55;
    trkSorted.forEach((p, i) => { const d = Math.abs(+p.t - vidT); if (d < bd) { bd = d; bi = i; } });
    return bi;
  })() : -1;
  /* the DELETE target is the dot you're parked ON — the one nearest the
     current frame AND within ~2 frames of it (the same dot the trash removes
     and that renders highlighted). Deliberately tighter than szIdx's snap so
     you can't accidentally delete a distant point while scrubbing. */
  const delIdx = (media?.kind === "video" && trkSorted.length) ? (() => {
    let bi = -1, bd = 2.2 * 0.03337; // ~2 frames at 30 fps
    trkSorted.forEach((p, i) => { const d = Math.abs(+p.t - vidT); if (d < bd) { bd = d; bi = i; } });
    return bi;
  })() : -1;
  /* unified SELECTED-point index: video derives it from the scrub position
     (szIdx / delIdx); a still uses the tap-selected index (selTrk), clamped in
     case the track shrank under it. selIdx feeds the adjust panel, delTarget the
     trash. */
  const selClamped = selTrk >= 0 && selTrk < trkSorted.length ? selTrk : -1;
  const selIdx = isVid ? szIdx : selClamped;
  const delTarget = isVid ? delIdx : selClamped;
  const wFit = src.shapeFit ? (() => { const pr = shapeProjNat(src.shapeFit); return Math.hypot(pr.p2.x - pr.p1.x, pr.p2.y - pr.p1.y) || 1; })() : 1;
  const fpxM = natW && isNum(src.fovH) ? (natW / 2) / Math.tan((+src.fovH * D2R) / 2) : null;
  const angOfW = (w) => (fpxM ? 2 * Math.atan((w / 2) / fpxM) * R2D : null);
  /* the selected point's ghost model as a loupe-ready shapeFit (centred on the
     point, width normalised through the rotated projection to hit `w`) — so
     tuning a SMALL object at a point pops the same magnifier as the initial
     3D-object placement instead of leaving you squinting at a few pixels */
  const ptGhostSf = (p, w, rotM) => {
    if (!src.shapeFit || !p) return null;
    let sfG = { ...src.shapeFit, cx: p.x, cy: p.y, rotM: (rotM && rotM.length === 9) ? rotM : (src.shapeFit.rotM || I3), roll: 0 };
    const pr = shapeProjNat(sfG); const pw = Math.hypot(pr.p2.x - pr.p1.x, pr.p2.y - pr.p1.y) || 1;
    sfG = { ...sfG, sizeNat: (src.shapeFit.sizeNat || 1) * w / pw };
    /* pin the SILHOUETTE midpoint on the point (matches the on-photo ghost):
       origin-anchoring let asymmetric shapes slide off the point as they scaled */
    const pr2 = shapeProjNat(sfG);
    return { ...sfG, cx: sfG.cx + (p.x - (pr2.p1.x + pr2.p2.x) / 2), cy: sfG.cy + (p.y - (pr2.p1.y + pr2.p2.y) / 2) };
  };
  /* size/attitude target the SELECTED point (video = scrub position szIdx;
     still = tap-selected selTrk) — so a still can also record the object
     shrinking (moving away) / growing (coming closer) and its attitude at
     each recalled point, exactly like video. */
  const setPtW = (w) => {
    if (selIdx < 0) return;
    const w2 = clampN(w, 2, natW * 0.6);
    const a2 = angOfW(w2);
    update({ track: trkSorted.map((p, i) => (i === selIdx ? { ...p, wpx: +w2.toFixed(1), ...(a2 != null ? { ang: +a2.toFixed(5) } : {}) } : p)) });
    const sfG = ptGhostSf(trkSorted[selIdx], w2, ptRotOf(selIdx)); if (sfG) shapeLoupeFor(sfG);
  };
  /* rotate the targeted point's model (per-point attitude keyframe). Left-
     multiply so the nudge is in the VIEW frame (drag-right yaws right etc.). */
  const ptRotOf = (i) => (Array.isArray(trkSorted[i]?.rotM) && trkSorted[i].rotM.length === 9) ? trkSorted[i].rotM : (src.shapeFit?.rotM || I3);
  const setPtRotM = (m) => {
    if (selIdx < 0) return;
    update({ track: trkSorted.map((p, i) => (i === selIdx ? { ...p, rotM: m } : p)) });
    const p = trkSorted[selIdx]; const sfG = ptGhostSf(p, isNum(p?.wpx) ? +p.wpx : wFit, m); if (sfG) shapeLoupeFor(sfG);
  };
  const nudgeRot = (which, deg) => { const R = which === "x" ? rotX3(deg) : which === "y" ? rotY3(deg) : rotZ3(deg); setPtRotM(mul3(R, ptRotOf(selIdx))); };
  const resetPtRotM = () => { if (selIdx < 0) return; update({ track: trkSorted.map((p, i) => (i === selIdx ? (({ rotM, ...rest }) => rest)(p) : p)) }); };
  /* STILL per-point timing/shape: the leg's DURATION (Δt from the previous
     point) and the TURN tightness (r) — on top of the size + attitude controls
     (shared with video). Setting Δt shifts this point and every later point by
     the same delta so downstream legs keep their timing. */
  const setPtDt = (i, dt) => {
    if (i <= 0 || i >= trkSorted.length) return;      // the first point has no "previous" leg
    const newT = +trkSorted[i - 1].t + Math.max(0.05, +dt || 0);
    const delta = newT - +trkSorted[i].t;
    update({ track: trkSorted.map((p, k) => (k >= i ? { ...p, t: +(+p.t + delta).toFixed(3) } : p)) });
  };
  const setPtR = (i, r) => { if (i < 0) return; update({ track: trkSorted.map((p, k) => (k === i ? { ...p, r } : p)) }); };
  /* SNAP a point onto the fitted 3D object: it takes the object's centre pixel
     and becomes the trajectory's ANCHOR (its direction = the object's measured
     A.az/el), with its size/attitude cleared back to the fit. The anchor can be
     ANY point along the path — the object may have been photographed mid-flight,
     not at the start. Clears any previous anchor. */
  const snapPtToObject = (i) => {
    if (i < 0 || !src.shapeFit) return;
    pushTrkHist();
    update({ track: trkSorted.map((p, k) => {
      if (k === i) { const { wpx, ang, rotM, ...rest } = p; return { ...rest, x: src.shapeFit.cx, y: src.shapeFit.cy, anchor: true }; }
      if (p.anchor) { const { anchor, ...rest } = p; return rest; }
      return p;
    }) });
  };
  const deleteTrkAt = (i) => { if (i < 0) return; pushTrkHist(); update({ track: (src.track || []).filter((p) => p !== trkSorted[i]) }); setSelTrk(-1); };
  const markStyle = {
    p1: { borderColor: "var(--amber)", color: "var(--amber)" },
    p2: { borderColor: "var(--amber)", color: "var(--amber)" },
    pb: { borderColor: "var(--teal)", color: "var(--teal)" },
    trk: { borderColor: "var(--track)", color: "var(--track)" },
  };
  const markText = { p1: "A1", p2: "A2", pb: "B" };

  return (
    <div>
      {capOpen && <SensorCapture onCapture={applyCapture} onClose={() => setCapOpen(false)} />}
      <ML>Photo / video (optional — used to measure angular size)</ML>
      <div style={{ display: "flex", gap: 8, alignItems: "center", flexWrap: "wrap" }}>
        <label className="btn sm amber" style={{ display: "inline-block" }}>
          {media ? "Replace media" : "Load photo or video"}
          <input type="file" accept="image/*,video/*" onChange={onFile} style={{ display: "none" }} />
        </label>
        {ENABLE_CAPTURE && (
          <button className="btn sm" title="Shoot the sighting in-app so the phone records the up/down angle, roll and heading EXIF leaves out" onClick={() => setCapOpen(true)}>📷 Capture{media ? "" : " with sensors"}</button>
        )}
        {media && wizard ? (
          /* explicit MODE TOGGLE — tapping the photo either places/rotates the
             3D object (measures size) or drops trajectory track points. A
             segmented control so it's always obvious which one is live. Both a
             VIDEO (tap across frames) and a STILL (tap the recalled path on the
             one photo) lay the trajectory HERE — the sky view just shows it. */
          <div style={{ display: "inline-flex", borderRadius: 9, overflow: "hidden", border: "1px solid var(--line)" }}>
            {[["shape", "◆ 3D object", "var(--amber)", "rgba(245,169,63,.18)"], ["trk", "⊕ Track points", "var(--track)", "rgba(143,180,255,.18)"],
              ...(isVid ? [["cref", "🎥 Cam refs", "var(--green)", "rgba(90,200,140,.18)"]] : [])].map(([k, label, col, bg]) => (
              <button key={k} className="btn sm"
                title={k === "shape" ? "Place, size and rotate the 3D object (measures its angular width)" : k === "cref" ? "Hand-mark fixed background features (a cloud edge, star, light) across frames to stabilize a clip the auto pass can't" : (isVid ? "Tap the object across frames to lay down its trajectory" : "Tap the object's path across the photo — where it was at each moment")}
                onClick={() => setActive(k)}
                style={{ borderRadius: 0, border: "none", padding: "6px 10px", fontWeight: active === k ? 700 : 500, background: active === k ? bg : "transparent", color: active === k ? col : "var(--dim)" }}>
                {label}
              </button>
            ))}
          </div>
        ) : media && (
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
          <Spin style={{ marginRight: 6 }} />Loading media
        </div>
      )}
      {media && !loadErr && (
        <>
          <div style={{ display: "flex", gap: 6, flexWrap: "wrap", alignItems: "center", marginTop: 8 }}>
            <button className={"btn sm" + (src.shapeFit ? " amber" : "")} onClick={() => setShapePicker((v) => !v)}
              title="Pick a 3D shape to fit to the object">
              {src.shapeFit ? `${(SHAPES.find((s) => s.k === src.shapeFit.kind) || {}).label || "Object"} ▾` : "＋ Add object"}
            </button>
            {src.shapeFit && <button className="btn sm" onClick={clearShape} title="remove shape">✕</button>}
            {!src.shapeFit && <span style={{ fontSize: 11.5, color: "var(--dim)", fontStyle: "italic" }}>use ● Orb if unsure</span>}
          </div>
          {shapePicker && (
            <div style={{ marginTop: 6, padding: 8, border: "1px solid var(--line)", borderRadius: 10, background: "rgba(255,255,255,.02)" }}>
              <div style={{ display: "flex", gap: 4, flexWrap: "wrap" }}>
                {SHAPES.map((sh) => (
                  <button key={sh.k} className={"btn sm" + (src.shapeFit?.kind === sh.k ? " amber" : "")}
                    onClick={() => { startShape(sh.k); setShapePicker(false); }}>{sh.label}</button>
                ))}
              </div>
              <div style={{ marginTop: 6, fontSize: 11.5, color: "var(--dim)", fontStyle: "italic", lineHeight: 1.4 }}>
                Not sure of the shape? Choose ● Orb — it assumes no particular form and still measures the object's size.
              </div>
            </div>
          )}
          {/* the guided-track on-ramp: this step is the ONLY place the
              trajectory can be laid down frame-accurately (the frame slider
              below scrubs the clip BEFORE any stabilization; sky-view taps
              can't — pre-solve, only the marked frame's pose is known) */}
          {wizard && media.kind === "video" && src.shapeFit && (src.track || []).length < 2 && active !== "trk" && (
            <div style={{ marginTop: 8, padding: "8px 10px", border: "1px solid var(--track)", borderRadius: 10, background: "rgba(143,180,255,.06)", fontSize: 11.5, color: "var(--dim)", lineHeight: 1.5 }}>
              🎯 <b style={{ color: "var(--track)" }}>Moving object?</b> Tap <b style={{ color: "var(--track)" }}>Track</b> (top row), then scrub the clip with the slider below and tap the object every second or so. Your taps become the trajectory the auto-tracker locks onto during Stabilize.
            </div>
          )}
          {wizard && media.kind === "video" && src.shapeFit && !isNum(src.alignT) && active !== "trk" && (
            <div style={{ marginTop: 8, padding: "8px 10px", border: "1px solid var(--teal)", borderRadius: 10, background: "rgba(64,199,178,.06)", fontSize: 11.5, color: "var(--dim)", lineHeight: 1.5 }}>
              ⛰ <b style={{ color: "var(--teal)" }}>Pick an alignment frame:</b> scrub to the moment with the clearest horizon (or stars) and tap <b style={{ color: "var(--teal)" }}>⛰ Align on this frame</b> below the slider. The sky alignment is done on that frame — otherwise the object's frame is used, which may be zoomed or horizon-free.
            </div>
          )}
          {src.shapeFit && active !== "trk" && (
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
              {(src.shapeFit.kind === "tri" || src.shapeFit.kind === "plane" || src.shapeFit.kind === "prop" || src.shapeFit.kind === "bird" || src.shapeFit.kind === "drone") && (
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
            {/* PORTRAIT clips are height-capped (≈ half the screen) — at full
               column width a 9:16 video pushed the frame slider and the whole
               Track panel below the fold. Width follows from the cap via the
               aspect ratio; dispW/TT() adapt automatically off clientWidth. */}
            <div ref={wrapRef}
              style={{
                position: "relative", borderRadius: 10, overflow: "hidden", border: "1px solid var(--line)", touchAction: "none", height: dispH || "auto",
                ...(media.kind === "video" && natW && natH > natW ? {
                  width: `min(100%, ${Math.round(Math.min(440, winH * 0.5) * natW / natH)}px)`,
                  margin: "0 auto",
                } : {}),
              }}
              onPointerDown={onDown} onPointerMove={onMove} onPointerUp={onUp} onPointerCancel={onUp}>
              {/* Zoom by SIZING the media element (width = dispW·z), not by
                 CSS-scaling a width:100% copy. A transform:scale() magnifies the
                 element's container-width rasterization — so a 12–48 MP photo was
                 shown blown up from only ~container-width pixels and went to mush
                 on zoom. Sizing the <img>/<video> itself makes the browser sample
                 from the full-resolution source at each zoom level (like the native
                 photo viewer). Coordinates are unchanged: width dispW·z ⇒ a natural
                 point x lands at x·scale·z, exactly what TT() already computes, so
                 only translate() remains on the wrapper. */}
              <div style={{ transform: `translate(${view.ox}px, ${view.oy}px) scale(${extraZ})`, transformOrigin: "0 0", willChange: "transform" }}>
                {media.kind === "video" ? (
                  <video ref={mediaRef} src={media.url} playsInline muted preload="auto"
                    onLoadedMetadata={onLoaded} onLoadedData={() => { paintFirstFrame(); kickVideoPaint(); }} onError={onMediaError} onTimeUpdate={(e) => setVidT(e.target.currentTime)}
                    style={{ width: dispW ? dispW * elZ : "100%", height: "auto", display: "block", pointerEvents: "none", filter: imgAdjFilter(src.imgAdj) }} />
                ) : (
                  <img ref={mediaRef} src={media.url} alt="sighting" onLoad={onLoaded} onError={onMediaError}
                    style={{ width: dispW ? dispW * elZ : "100%", height: "auto", display: "block", pointerEvents: "none", filter: imgAdjFilter(src.imgAdj) }} draggable={false} />
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
                const pts2 = [...src.track].filter((p) => p.x != null).sort((a, b) => a.t - b.t);
                const tp = pts2.map((p) => TT(p.x, p.y));
                /* video: a tap belongs to ITS frame — fade markers out as the
                   scrubber moves away from their time (full ≤0.3 s, gone by
                   1.1 s; a faint floor while the Track tool is active so Undo
                   targets stay findable). The connecting route stays as thin
                   context. Photos keep everything (no time axis). */
                const isVid = media.kind === "video";
                const fadeT = (p) => {
                  if (!isVid || !isNum(p.t)) return 1;
                  const d = Math.abs(p.t - vidT);
                  const f = d <= 0.3 ? 1 : d >= 1.1 ? 0 : 1 - (d - 0.3) / 0.8;
                  return active === "trk" ? Math.max(0.15, f) : f;
                };
                return (
                  <svg style={{ position: "absolute", inset: 0, pointerEvents: "none" }} width="100%" height="100%">
                    {/* the connecting path ROUNDS each interior corner by its turn
                       tightness r (0 = hard corner, up to ~0.49 = wide arc) — the
                       same quadratic the trajectory uses, so setting turn on the
                       measure step visibly curves the path instead of doing
                       nothing here. Video points carry no r ⇒ straight. */}
                    {tp.length >= 2 && (() => {
                      const lerp = (a, b, ff) => [a[0] + (b[0] - a[0]) * ff, a[1] + (b[1] - a[1]) * ff];
                      let d = `M ${tp[0][0]} ${tp[0][1]}`;
                      for (let i = 1; i < tp.length; i++) {
                        const rr = clampN(+pts2[i]?.r || 0, 0, 0.49);
                        if (i < tp.length - 1 && rr > 0) {
                          const V = tp[i], cs = lerp(V, tp[i - 1], rr), ce = lerp(V, tp[i + 1], rr);
                          const nrm = (a) => { const m = Math.hypot(a[0], a[1]) || 1; return [a[0] / m, a[1] / m]; };
                          const uin = nrm([V[0] - cs[0], V[1] - cs[1]]), wout = nrm([ce[0] - V[0], ce[1] - V[1]]);
                          const ma = [uin[0] + wout[0], uin[1] + wout[1]], mm = Math.hypot(ma[0], ma[1]);
                          if (pts2[i]?.anchor && mm > 1e-3) {
                            /* the anchor is the MEASURED object position — the arc must
                               pass THROUGH it AND stay smooth: two cubic-Hermite halves
                               (cs→V, V→ce) sharing an averaged tangent at V, tangent to
                               the legs at cs/ce. Matches roundCorners. */
                            const m = [ma[0] / mm, ma[1] / mm];
                            const l1 = Math.hypot(V[0] - cs[0], V[1] - cs[1]) / 3, l2 = Math.hypot(ce[0] - V[0], ce[1] - V[1]) / 3;
                            const c1 = [cs[0] + uin[0] * l1, cs[1] + uin[1] * l1], c2 = [V[0] - m[0] * l1, V[1] - m[1] * l1];
                            const c3 = [V[0] + m[0] * l2, V[1] + m[1] * l2], c4 = [ce[0] - wout[0] * l2, ce[1] - wout[1] * l2];
                            d += ` L ${cs[0]} ${cs[1]} C ${c1[0]} ${c1[1]} ${c2[0]} ${c2[1]} ${V[0]} ${V[1]} C ${c3[0]} ${c3[1]} ${c4[0]} ${c4[1]} ${ce[0]} ${ce[1]}`;
                          } else {
                            d += ` L ${cs[0]} ${cs[1]} Q ${V[0]} ${V[1]} ${ce[0]} ${ce[1]}`;
                          }
                        } else d += ` L ${tp[i][0]} ${tp[i][1]}`;
                      }
                      return <path d={d} fill="none" stroke={trkCol(1)} strokeWidth="1.5" strokeDasharray="2 3" opacity={isVid ? 0.22 : 1} />;
                    })()}
                    {tp.map((q, i) => {
                      /* the selected/delete target (video: dot you're parked on;
                         still: the tapped dot) gets a bright ring — what 🗑 removes
                         and what Adjust edits */
                      const isDel = delTarget >= 0 && pts2[i] === trkSorted[delTarget];
                      const op = isDel ? 1 : fadeT(pts2[i]);
                      if (op <= 0.01) return null;
                      return (
                        <g key={i}>
                          {isDel && <circle cx={q[0]} cy={q[1]} r={7} fill="none" stroke={trkCol(1)} strokeWidth="1.6" />}
                          <circle cx={q[0]} cy={q[1]} r={isDel ? 3.5 : (i === tp.length - 1 ? 4 : 2.5)}
                            fill={trkCol(isDel || i === tp.length - 1 ? 1 : 0.75)} opacity={op} />
                        </g>
                      );
                    })}
                    {/* STILL only: a little Δt stamp at each leg's midpoint — a
                       photo has no scrubber, so the leg duration (time from the
                       previous point) is otherwise invisible. Videos read timing
                       off the frame clock, so no stamp there. */}
                    {!isVid && tp.length >= 2 && pts2.map((p, i) => {
                      if (i === 0) return null;
                      const dt = +p.t - +pts2[i - 1].t;
                      if (!(dt > 0)) return null;
                      const mx = (tp[i][0] + tp[i - 1][0]) / 2, my = (tp[i][1] + tp[i - 1][1]) / 2;
                      const label = `${dt >= 10 ? dt.toFixed(0) : dt.toFixed(1)}s`;
                      const w = label.length * 6.2 + 8;
                      return (
                        <g key={"dt" + i}>
                          <rect x={mx - w / 2} y={my - 8} width={w} height={16} rx={5} fill="rgba(7,11,20,.72)" stroke={trkCol(0.5)} strokeWidth="0.75" />
                          <text x={mx} y={my + 0.5} textAnchor="middle" dominantBaseline="central" fontFamily="ui-monospace, monospace" fontSize="10.5" fill={trkCol(1)}>{label}</text>
                        </g>
                      );
                    })}
                    {/* wireframe GHOSTS at sized points (and the point being
                       sized): the fitted shape re-centred on the tap, scaled to
                       that frame's stored width — frame-local like the dots */}
                    {src.shapeFit && pts2.map((p, i) => {
                      const has = isNum(p.wpx);
                      /* the SELECTED point sprouts an adjustable model ONLY in
                         Adjust mode — while PLACING, taps should just drop
                         trajectory dots, not conjure a 3D object at each one */
                      const isSz = active === "trk" && trkAdjust && i === selIdx;
                      if (!has && !isSz) return null;
                      /* video fades ghosts by scrub distance; a still has no
                         scrubber, so all its sized ghosts stay fully visible */
                      const f = isVid ? (() => { const d = Math.abs(p.t - vidT); return d <= 0.3 ? 1 : d >= 1.1 ? 0 : 1 - (d - 0.3) / 0.8; })() : 1;
                      const op = isSz ? Math.max(0.5, f) : f;
                      if (op <= 0.02) return null;
                      /* size + attitude INTERPOLATED at this point's time (fit as
                         the baseline keyframe) — so an un-adjusted point shows the
                         value that ramps from the nearest adjustment/fit, not a
                         flat fitted size that jumps at the next keyframe */
                      const smp = sampleShapeAt(src.track, src.shapeFit, p.t, { markT: isNum(src.A?.videoTime) ? +src.A.videoTime : null, wFit });
                      const w = smp.wpx != null ? smp.wpx : wFit;
                      const rot = (smp.rotM && smp.rotM.length === 9) ? smp.rotM : (src.shapeFit.rotM || I3);
                      let sfG = { ...src.shapeFit, cx: p.x, cy: p.y, rotM: rot, roll: 0 };
                      const pwG = (() => { const pr = shapeProjNat(sfG); return Math.hypot(pr.p2.x - pr.p1.x, pr.p2.y - pr.p1.y) || 1; })();
                      sfG = { ...sfG, sizeNat: (src.shapeFit.sizeNat || 1) * w / pwG };
                      /* anchor the SILHOUETTE midpoint on the tapped point, not the
                         shape's 3D origin: for asymmetric shapes/attitudes the
                         projected centre sits off the origin and that offset GROWS
                         with size — origin-anchoring made the outline slide off the
                         point as it scaled (field: "scales from a weird center").
                         Translation is exact, so one correction re-pins it. */
                      let prG = shapeProjNat(sfG);
                      const mgx = (prG.p1.x + prG.p2.x) / 2, mgy = (prG.p1.y + prG.p2.y) / 2;
                      if (Math.abs(mgx - p.x) + Math.abs(mgy - p.y) > 0.01)
                        prG = shapeProjNat({ ...sfG, cx: sfG.cx + (p.x - mgx), cy: sfG.cy + (p.y - mgy) });
                      const colG = `hsl(${src.shapeFit.hue ?? 36},88%,60%)`;
                      return prG.curves.map((c, j) => (
                        <polyline key={`g${i}-${j}`} points={c.map((pt2) => TT(pt2.x, pt2.y).join(",")).join(" ")}
                          fill="none" stroke={colG} strokeWidth={isSz ? 1.4 : 1} opacity={op * 0.8} />
                      ));
                    })}
                  </svg>
                );
              })()}
              {scale > 0 && src.shapeFit && (() => {
                const sf = src.shapeFit;
                /* video: the shape belongs to the frame it was FIT on
                   (A.videoTime) — fade it out as the scrubber leaves that
                   frame, with a floor while the shape tool is active so it
                   stays adjustable (touching it re-stamps to this frame) */
                let frameOp = 1;
                if (media.kind === "video" && isNum(src.A?.videoTime)) {
                  const dT2 = Math.abs(vidT - +src.A.videoTime);
                  const f2 = dT2 <= 0.3 ? 1 : dT2 >= 1.1 ? 0 : 1 - (dT2 - 0.3) / 0.8;
                  frameOp = active === "shape" || shapeMag ? Math.max(0.25, f2) : f2;
                }
                if (frameOp <= 0.01) return null;
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
                  <svg style={{ position: "absolute", inset: 0, pointerEvents: "none", overflow: "visible", opacity: frameOp }} width="100%" height="100%">
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
              {/* CAMERA REFERENCE marks — numbered dots for features marked on
                 THIS frame (bright) + faint ghosts of the nearest mark for refs
                 not yet placed here, so you can re-mark them. Only while the Cam
                 refs tool is active. */}
              {scale > 0 && active === "cref" && camRefs.length > 0 && (
                <svg style={{ position: "absolute", inset: 0, pointerEvents: "none", overflow: "visible" }} width="100%" height="100%">
                  {camRefs.map((r, i) => {
                    const here = crefAtFrame(r);
                    const hue = (i * 47) % 360;
                    const col = `hsl(${hue},85%,62%)`;
                    if (here) {
                      const [x, y] = TT(here.x, here.y);
                      return (
                        <g key={i}>
                          {i === selCref && <circle cx={x} cy={y} r={9} fill="none" stroke={col} strokeWidth="1.4" opacity="0.9" />}
                          <circle cx={x} cy={y} r={4} fill={col} />
                          <text x={x} y={y - 9} textAnchor="middle" fontSize="10" fontWeight="700" fill={col}>{i + 1}</text>
                        </g>
                      );
                    }
                    const g = (r.marks || []).filter((m) => isNum(m.x)).reduce((a, m) => (a && Math.abs(a.t - vidT) < Math.abs(m.t - vidT) ? a : m), null);
                    if (!g) return null;
                    const [x, y] = TT(g.x, g.y);
                    return (
                      <g key={i} opacity="0.35">
                        <circle cx={x} cy={y} r={4} fill="none" stroke={col} strokeWidth="1.2" strokeDasharray="2 2" />
                        <text x={x} y={y - 8} textAnchor="middle" fontSize="9" fill={col}>{i + 1}</text>
                      </g>
                    );
                  })}
                </svg>
              )}
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
                  style={{ position: "fixed", left, top, width: S, height: S, borderRadius: 14, zIndex: 60, pointerEvents: "none", background: "#000", border: `2px solid ${active === "pb" ? "var(--teal)" : active === "trk" ? "var(--track)" : "var(--amber)"}`, boxShadow: "0 4px 14px rgba(0,0,0,.55)" }} />
              );
            })()}
          </div>

          {media && (
            <div style={{ marginTop: 8 }}>
              {/* the frame slider + align/set-time controls are VIDEO-only; the
                 Track panel below renders for a STILL too (tap the recalled path
                 on the one photo) */}
              {isVid && (<>
              {/* frame slider with placement TICKS: amber ▾ = the frame the
                  shape/marks live on, blue dots = trajectory taps — so you can
                  find your way back to where things were placed */}
              <div style={{ position: "relative" }}>
                {vidDur > 0 && (
                  <div style={{ position: "absolute", left: 8, right: 8, top: -5, height: 6, pointerEvents: "none" }}>
                    {(src.track || []).filter((p) => isNum(p.t)).map((p, i) => (
                      <span key={"tk" + i} style={{ position: "absolute", left: `${clampN((p.t / vidDur) * 100, 0, 100)}%`, transform: "translateX(-50%)", width: 5, height: 5, borderRadius: 3, background: "var(--track)", display: "block" }} />
                    ))}
                    {isNum(src.A?.videoTime) && (
                      <span style={{ position: "absolute", left: `${clampN((+src.A.videoTime / vidDur) * 100, 0, 100)}%`, transform: "translateX(-50%)", color: "var(--amber)", fontSize: 9, lineHeight: "6px", display: "block" }}>▾</span>
                    )}
                    {isNum(src.alignT) && (
                      <span style={{ position: "absolute", left: `${clampN((+src.alignT / vidDur) * 100, 0, 100)}%`, transform: "translateX(-50%)", color: "var(--teal)", fontSize: 9, lineHeight: "6px", display: "block" }}>▾</span>
                    )}
                  </div>
                )}
                <input type="range" min={0} max={vidDur || 0} step={0.033} value={vidT}
                  onChange={(e) => seek(+e.target.value)} />
              </div>
              <div style={{ display: "flex", gap: 6, alignItems: "center" }}>
                <button className="btn sm" style={{ padding: "6px 8px" }} onClick={() => seek(Math.max(0, vidT - 0.033))}>−1 fr</button>
                <button className="btn sm" style={{ padding: "6px 8px" }} onClick={() => seek(Math.min(vidDur, vidT + 0.033))}>+1 fr</button>
                <span style={{ fontFamily: "var(--mono)", fontSize: 11, color: "var(--dim)" }}>{vidT.toFixed(2)}s</span>
                {wizard ? (
                  /* the ALIGNMENT frame is chosen HERE (cheap scrubbing) — the
                     sky view bakes it and the world alignment describes it.
                     The OBJECT's frame is stamped automatically by the shape
                     fit, so the two are independent: clearest horizon for
                     alignment, clearest object for measurement. Compact label
                     so it shares the frame-step row on a phone (tooltip + the
                     teal slider tick carry the detail). */
                  <button className="btn sm teal" style={{ marginLeft: "auto", padding: "6px 8px" }}
                    title="Alignment frame: the sky view shows THIS frame and the horizon/star alignment is done on it. Scrub to the clearest-horizon (or star) moment and tap. The object stays measured on the frame where you fitted the shape."
                    onClick={() => update({ alignT: +vidT.toFixed(3) })}>
                    ⛰ Align{isNum(src.alignT) ? " ✓" : ""}
                  </button>
                ) : (
                  <button className="btn sm amber" onClick={() => update({ A: { ...src.A, t: vidT.toFixed(2), videoTime: vidT } })}>Set time A</button>
                )}
                {!wizard && <button className="btn sm teal" onClick={() => update({ B: { ...src.B, t: vidT.toFixed(2), videoTime: vidT } })}>Set time B</button>}
              </div>
              </>)}
              {/* CAM REFS panel — manual stabilization fallback. Pick a reference
                 slot, then tap the same fixed background feature on each frame
                 (scrub between). The sky view solves the pose from these marks. */}
              {active === "cref" && isVid && (() => {
                const kf = new Set();
                camRefs.forEach((r) => (r.marks || []).forEach((m) => { if (isNum(m.x)) kf.add(+(+m.t).toFixed(2)); }));
                const hereCount = camRefs.filter((r) => crefAtFrame(r)).length;
                return (
                  <div style={{ marginTop: 8, padding: "8px 10px", border: "1px solid var(--green)", borderRadius: 10, background: "rgba(90,200,140,.06)" }}>
                    <div style={{ fontSize: 11, color: "var(--dim)", marginBottom: 6, lineHeight: 1.4 }}>
                      For a clip the auto stabilizer can't do: mark the SAME fixed feature — a cloud edge, a star, a ground light, the horizon — on several frames (scrub between them). Tap to place, or <b>drag for a magnifier</b> on faint features. Mark 3–5 features, spread across the frame, on the align frame and a handful of others. If one pans out of view, add a fresh one — it hands off as long as it <b>overlaps an existing ref on ≥1 frame</b>. <b>Never clouds that drift.</b> Then <b>Solve from marks</b> in the sky view (a smoothing slider pops up to average out imperfect placement).
                    </div>
                    <div style={{ display: "flex", gap: 5, flexWrap: "wrap", alignItems: "center" }}>
                      {camRefs.map((r, i) => {
                        const on = i === selCref, marked = !!crefAtFrame(r), hue = (i * 47) % 360;
                        return (
                          <button key={i} className="btn sm" onClick={() => setSelCref(i)}
                            style={{ padding: "4px 9px", fontWeight: on ? 700 : 500, borderColor: `hsl(${hue},70%,50%)`, color: `hsl(${hue},85%,65%)`, background: on ? `hsla(${hue},70%,45%,.22)` : "transparent" }}>
                            {i + 1}{marked ? " ●" : " ○"}<span style={{ fontSize: 9, color: "var(--dim)" }}> {(r.marks || []).filter((m) => isNum(m.x)).length}</span>
                          </button>
                        );
                      })}
                      <button className="btn sm green" onClick={addCref} style={{ padding: "4px 9px" }}>+ Ref</button>
                      {camRefs.length > 0 && <button className="btn sm" onClick={() => delCref(selCref)} style={{ padding: "4px 9px" }} title="Delete the selected reference">🗑</button>}
                    </div>
                    <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 6 }}>
                      {camRefs.length} ref{camRefs.length === 1 ? "" : "s"} · {kf.size} frame{kf.size === 1 ? "" : "s"} marked · {hereCount} on this frame{isNum(src.alignT) ? "" : " · set an align frame (⛰) first"}
                    </div>
                  </div>
                );
              })()}
              {/* Track tools follow the mode toggle in the wizard — hidden in
                 3D-object mode even when points already exist (they reappear on
                 ⊕ Track points). Non-wizard keeps the "show if any points" rule. */}
              {(!wizard || media) && (active === "trk" || (!wizard && (src.track || []).length > 0)) && (
                <div style={{ marginTop: 8, padding: "8px 10px", border: "1px solid var(--track)", borderRadius: 10, background: "rgba(143,180,255,.06)" }}>
                  {/* one compact row: count · auto-step · Undo · Clear (was two
                     rows that wrapped on a phone) */}
                  <div style={{ display: "flex", gap: 6, alignItems: "center" }}>
                    <span style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--track)", fontWeight: 700, whiteSpace: "nowrap" }}>
                      Track · {(src.track || []).length}
                    </span>
                    {isVid ? (<>
                      <span style={{ fontSize: 11, color: "var(--dim)" }} title="how many frames to auto-advance after each tap">step</span>
                      <select value={trkAdv} onChange={(e) => setTrkAdv(+e.target.value)} style={{ width: "auto", padding: "4px 6px", fontSize: 12 }}>
                        <option value={0}>off</option><option value={1}>1 fr</option><option value={2}>2 fr</option>
                        <option value={3}>3 fr</option><option value={6}>6 fr</option><option value={15}>15 fr</option>
                        <option value={30}>30 fr</option><option value={45}>45 fr</option><option value={60}>60 fr</option><option value={90}>90 fr</option>
                      </select>
                    </>) : (<>
                      {/* STILL: no frame clock — each tap is `gap` seconds after the last */}
                      <span style={{ fontSize: 11, color: "var(--dim)" }} title="seconds between each tapped point (edit any leg later in Adjust)">gap</span>
                      <select value={trkGap} onChange={(e) => setTrkGap(+e.target.value)} style={{ width: "auto", padding: "4px 6px", fontSize: 12 }}>
                        <option value={0.1}>0.1 s</option><option value={0.25}>0.25 s</option><option value={0.5}>0.5 s</option>
                        <option value={1}>1 s</option><option value={2}>2 s</option><option value={3}>3 s</option><option value={5}>5 s</option>
                      </select>
                    </>)}
                    <button className="btn sm" style={{ marginLeft: "auto", padding: "6px 8px" }} disabled={!(src.track || []).length && !trkHistRef.current.length}
                      title="undo the last place or delete" onClick={undoTrack}>Undo</button>
                    {/* selected/parked dot → trash deletes JUST it (undoable);
                       otherwise it clears the whole track */}
                    <button className="btn sm" style={{ padding: "6px 8px" }} disabled={!(src.track || []).length}
                      title={delTarget >= 0 ? `delete point ${delTarget + 1} (the highlighted dot)` : "remove all track points"}
                      onClick={() => {
                        if (delTarget >= 0) deleteTrkAt(delTarget);
                        else { pushTrkHist(); update({ track: [] }); }
                      }}>{delTarget >= 0 ? `🗑 pt ${delTarget + 1}` : "🗑 all"}</button>
                  </div>
                  {/* END-OF-CLIP notice: once auto-advance clamps at the last
                     frame, more taps just stack points on the SAME frame. Warn
                     (and hint at scrubbing back / removing the pile-up). */}
                  {media.kind === "video" && !trkAdjust && vidDur > 0 && vidT >= vidDur - 0.034 && (
                    <div style={{ marginTop: 6, padding: "6px 8px", border: "1px solid var(--amber)", borderRadius: 8, background: "rgba(245,169,63,.08)", fontSize: 11, color: "var(--amber)", lineHeight: 1.4 }}>
                      ⛔ Last frame of the clip{trkAdv > 0 ? " — auto-advance has stopped" : ""}. More taps stack on this same frame; scrub back to place earlier points.
                    </div>
                  )}
                  {/* PLACE vs ADJUST — lay all the points down first (taps add),
                     then flip to Adjust: scrub to any point and tune its size +
                     attitude; taps no longer add. */}
                  {wizard && (src.shapeFit || !isVid) && (src.track || []).length > 0 && (
                    <div style={{ display: "flex", gap: 6, alignItems: "center", marginTop: 6 }}>
                      <div style={{ display: "inline-flex", borderRadius: 8, overflow: "hidden", border: "1px solid var(--line)" }}>
                        {[["＋ Place points", false], [isVid ? "✎ Adjust size/shape" : "✎ Adjust", true]].map(([label, v]) => (
                          <button key={String(v)} className="btn sm"
                            style={{ borderRadius: 0, border: "none", padding: "5px 9px", fontSize: 11, fontWeight: trkAdjust === v ? 700 : 500, background: trkAdjust === v ? "rgba(143,180,255,.18)" : "transparent", color: trkAdjust === v ? "var(--track)" : "var(--dim)" }}
                            onClick={() => {
                              setTrkAdjust(v);
                              /* entering Adjust with nothing selected? auto-pick the
                                 last point so the controls show immediately (a still
                                 has no scrubber to select one for you) */
                              if (!v) setSelTrk(-1);
                              else if (!isVid && selTrk < 0 && trkSorted.length) setSelTrk(trkSorted.length - 1);
                            }}>{label}</button>
                        ))}
                      </div>
                      {trkAdjust && <span style={{ fontSize: 10, color: "var(--dim)" }}>{isVid ? "scrub to a point · drag it to move · drag off it to rotate · twist = roll" : "tap a point · drag it to move · drag off it to rotate · twist = roll"}</span>}
                    </div>
                  )}
                  {/* COLOUR — recolour the object and/or the track points so they
                     stand out against the video (each swatch opens its hue slider) */}
                  <div style={{ display: "flex", gap: 8, alignItems: "center", marginTop: 6 }}>
                    <span className="microlabel" style={{ marginBottom: 0 }}>colour</span>
                    {src.shapeFit && (
                      <button title="object colour" onClick={() => setColorOpen((o) => (o === "obj" ? null : "obj"))}
                        style={{ width: 18, height: 18, borderRadius: 9, padding: 0, flex: "0 0 auto", cursor: "pointer", background: `hsl(${src.shapeFit.hue ?? 36},88%,60%)`, border: colorOpen === "obj" ? "2px solid #fff" : "1px solid rgba(255,255,255,.35)" }} />
                    )}
                    <button title="track-point colour" onClick={() => setColorOpen((o) => (o === "pts" ? null : "pts"))}
                      style={{ width: 18, height: 18, borderRadius: 9, padding: 0, flex: "0 0 auto", cursor: "pointer", background: trkCol(1), border: colorOpen === "pts" ? "2px solid #fff" : "1px solid rgba(255,255,255,.35)" }} />
                    {colorOpen === "obj" && src.shapeFit ? (
                      <input type="range" min={0} max={360} step={2} value={src.shapeFit.hue ?? 36} onChange={(e) => updShape({ hue: +e.target.value })} style={{ flex: 1 }} />
                    ) : colorOpen === "pts" ? (
                      <input type="range" min={0} max={360} step={2} value={trackHue} onChange={(e) => update({ trackHue: +e.target.value })} style={{ flex: 1 }} />
                    ) : (
                      <span style={{ fontSize: 10, color: "var(--dim)" }}>{src.shapeFit ? "tap a swatch: object · points" : "tap the swatch to recolour points"}</span>
                    )}
                  </div>
                  {/* SIZE ON THIS FRAME — scrub to a tapped point and match the
                     outline to the object as it appears RIGHT THERE. Apparent
                     size across frames ⇒ range ratio over time (the radial
                     side of the trajectory the bearings alone can't see). */}
                  {wizard && src.shapeFit && trkAdjust && selIdx >= 0 && (() => {
                    const p = trkSorted[selIdx];
                    const w = isNum(p.wpx) ? +p.wpx : wFit;
                    const lo = Math.max(2, wFit / 40), hi = Math.min(natW * 0.6, wFit * 40);
                    const sv = clampN(Math.log(w / lo) / Math.log(hi / lo), 0, 1);
                    const aP = angOfW(w), aF = angOfW(wFit);
                    const rho = aP != null && aF != null ? Math.tan(aF * D2R / 2) / Math.tan(aP * D2R / 2) : null;
                    return (
                      <div style={{ marginTop: 8, borderTop: "1px solid rgba(143,180,255,.25)", paddingTop: 6 }}>
                        <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                          <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)" }}>
                            point {selIdx + 1} @ {(+p.t).toFixed(2)}s — size + attitude · bigger = closer
                          </span>
                          <span style={{ marginLeft: "auto", fontFamily: "var(--mono)", fontSize: 11, color: "var(--track)" }}>
                            {rho == null ? "" : Math.abs(rho - 1) < 0.03 ? "≈ fitted range" : rho < 1 ? `≈ ${(1 / rho).toFixed(2)}× closer` : `≈ ${rho.toFixed(2)}× farther`}
                          </span>
                        </div>
                        <div style={{ display: "flex", alignItems: "center", gap: 6, marginTop: 4 }}>
                          <span className="microlabel" style={{ marginBottom: 0, minWidth: 30 }}>size</span>
                          <button className="btn sm" onClick={() => setPtW(w / 1.08)}>−</button>
                          <input type="range" min={0} max={1} step={0.004} value={sv}
                            onChange={(e) => setPtW(lo * Math.pow(hi / lo, +e.target.value))} style={{ flex: 1 }} />
                          <button className="btn sm" onClick={() => setPtW(w * 1.08)}>+</button>
                          {isNum(p.wpx) && (
                            <button className="btn sm" title="forget this point's size (back to the fitted size)"
                              onClick={() => update({ track: trkSorted.map((q, i) => { if (i !== selIdx) return q; const { wpx, ang: _a, ...rest } = q; return rest; }) })}>✕</button>
                          )}
                        </div>
                        {/* per-point ATTITUDE — tumble/roll this point's model; it
                           SLERPs to the next attitude keyframe in playback/export */}
                        <div style={{ display: "flex", alignItems: "center", gap: 6, marginTop: 5 }}>
                          <span className="microlabel" style={{ marginBottom: 0, minWidth: 30 }}>tilt</span>
                          <button className="btn sm" title="pitch up" onClick={() => nudgeRot("x", -12)}>↑</button>
                          <button className="btn sm" title="pitch down" onClick={() => nudgeRot("x", 12)}>↓</button>
                          <button className="btn sm" title="yaw left" onClick={() => nudgeRot("y", -12)}>←</button>
                          <button className="btn sm" title="yaw right" onClick={() => nudgeRot("y", 12)}>→</button>
                          <button className="btn sm" title="roll left" onClick={() => nudgeRot("z", -12)}>⟲</button>
                          <button className="btn sm" title="roll right" onClick={() => nudgeRot("z", 12)}>⟳</button>
                          {Array.isArray(trkSorted[selIdx]?.rotM) && <button className="btn sm" style={{ marginLeft: "auto" }} title="clear this point's attitude" onClick={resetPtRotM}>reset</button>}
                        </div>
                        {!trkSorted.some((q) => isNum(q.wpx)) && (
                          <div style={{ marginTop: 4, fontSize: 10.5, color: "var(--dim)", lineHeight: 1.4 }}>
                            {isVid
                              ? "Scrub to a point and match the outline to the object as it appears there — the size change between frames is what recovers closer/farther motion."
                              : "Set the object smaller where it looked FARTHER, bigger where it looked CLOSER, and tilt it to match its attitude — this records the object's distance + orientation along the recalled path."}
                          </div>
                        )}
                      </div>
                    );
                  })()}
                  {/* STILL adjust: a photo shows the object only once, so there's
                     no per-point size — instead tune the leg's DURATION (Δt) and
                     the TURN tightness that turn the recalled path into speed +
                     g-load (the same controls the sky view used to own). */}
                  {wizard && !isVid && trkAdjust && selIdx >= 0 && trkSorted[selIdx] && (() => {
                    const p = trkSorted[selIdx];
                    const interior = selIdx > 0 && selIdx < trkSorted.length - 1;
                    const dt = selIdx > 0 ? +p.t - +trkSorted[selIdx - 1].t : null;
                    const r = +(p.r ?? 0);
                    return (
                      <div style={{ marginTop: 8, borderTop: "1px solid rgba(143,180,255,.25)", paddingTop: 6 }}>
                        <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 4 }}>
                          <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)" }}>
                            point {selIdx + 1} of {trkSorted.length}{selIdx === 0 ? " (path start)" : ` @ ${(+p.t).toFixed(1)}s`}{p.anchor ? " · ⌖ object" : ""}
                          </span>
                          {/* SNAP this point onto the measured 3D object — it becomes
                             the trajectory anchor at the object's real direction.
                             Any point can be it (the object may be mid-path). */}
                          {src.shapeFit && !p.anchor && (
                            <button className="btn sm" style={{ marginLeft: "auto", padding: "4px 8px" }}
                              title="Move this point onto the fitted 3D object and anchor the whole path to its measured direction (the object may have been photographed anywhere along the path)."
                              onClick={() => snapPtToObject(selIdx)}>⌖ Snap to object</button>
                          )}
                        </div>
                        {dt != null && (
                          <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
                            <span className="microlabel" style={{ marginBottom: 0, minWidth: 30 }}>Δt</span>
                            <button className="btn sm" onClick={() => setPtDt(selIdx, Math.max(0.05, +(dt - 0.1).toFixed(2)))}>−0.1</button>
                            <span style={{ fontFamily: "var(--mono)", fontSize: 14, fontWeight: 700, color: "var(--amber)", minWidth: 52, textAlign: "center" }}>{dt.toFixed(1)} s</span>
                            <button className="btn sm" onClick={() => setPtDt(selIdx, +(dt + 0.1).toFixed(2))}>+0.1</button>
                            <span style={{ marginLeft: 6, display: "flex", gap: 3, flexWrap: "wrap" }}>
                              {[0.5, 1, 2, 5].map((v) => (
                                <button key={v} className={"btn sm" + (Math.abs(dt - v) < 0.05 ? " amber" : "")} style={{ padding: "3px 6px" }} onClick={() => setPtDt(selIdx, v)}>{v}s</button>
                              ))}
                            </span>
                          </div>
                        )}
                        {interior && (
                          <div style={{ display: "flex", alignItems: "center", gap: 4, marginTop: 6, flexWrap: "wrap" }}>
                            <span className="microlabel" style={{ marginBottom: 0, minWidth: 30 }}>turn</span>
                            {[["Hard corner", 0], ["Tight", 0.15], ["Normal", 0.3], ["Wide", 0.45]].map(([l, v]) => (
                              <button key={l} className={"btn sm" + (Math.abs(r - v) < 0.03 ? " amber" : "")} style={{ padding: "4px 7px", fontSize: 11 }} onClick={() => setPtR(selIdx, v)}>{l}</button>
                            ))}
                          </div>
                        )}
                        <div style={{ marginTop: 5, fontSize: 10, color: "var(--dim)", lineHeight: 1.4 }}>
                          {interior ? "A hard corner = an instantaneous direction change (an extraordinary claim); a wider arc is what most real objects fly." : "Δt is the time from the previous point — it sets the speed along this leg."}
                        </div>
                      </div>
                    );
                  })()}
                  {(() => {
                    /* onboarding paragraph only until the guide is armed (2+ pts) —
                       after that the header count + size block say everything */
                    if (wizard) return (src.track || []).length >= 2 ? null : (
                      <div style={{ marginTop: 6, fontSize: 11, color: "var(--dim)" }}>
                        {isVid
                          ? "Rough trajectory for the auto-tracker: scrub through the clip and tap the object every second or so (2+ points activate the guided track). Big steps are fine — precision comes from the pixel matcher."
                          : "Tap the object's PATH across the photo — where it was at each moment (point 1 = its marked spot). Each tap is one “gap” apart in time; tune any leg later in ✎ Adjust timing."}
                      </div>
                    );
                    if ((src.track || []).length < 3) return (
                      <div style={{ marginTop: 6, fontSize: 11, color: "var(--dim)" }}>
                        {isVid
                          ? "Scrub to the frame Moment A's bearing was taken, tap the object for point 1 (it anchors the absolute direction), then keep tapping as the video steps forward."
                          : "Point 1 anchors the absolute direction (drop it where the object actually is in the photo); keep tapping along its recalled path."}
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
          {(() => {
            /* PORTRAIT media: preset labels are long-side (landscape) lens
               FOVs — selecting one stores the tan-converted sideways
               (horizontal) FOV, which is what every measurement uses */
            const isPort = natW > 0 && natH > natW;
            const cvtFov = (v) => isPort ? +(2 * Math.atan(Math.tan((v / 2) * D2R) * (natW / natH)) * R2D).toFixed(1) : v;
            return (
              <>
                <select value={(FOV_PRESETS.find((p) => Math.abs(cvtFov(p.v) - +src.fovH) < 0.06) || {}).v ?? "custom"}
                  onChange={(e) => e.target.value !== "custom" && update({ fovH: cvtFov(+e.target.value) })}>
                  {FOV_PRESETS.map((p) => <option key={p.v} value={p.v}>{p.label}</option>)}
                  <option value="custom">Custom…</option>
                </select>
                {isPort && <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 2 }}>portrait — presets auto-convert to the sideways (horizontal) FOV</div>}
              </>
            );
          })()}
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

      <div style={{ marginTop: 14 }}>
        <ML>In your words — what did you see? (optional)</ML>
        <textarea value={src.statement || ""} onChange={(e) => update({ statement: e.target.value })}
          placeholder="Shape, colour, brightness, motion, sound, how long it lasted, how it ended…"
          rows={3}
          style={{ width: "100%", marginTop: 4, background: "var(--panel2)", border: "1px solid var(--line)", borderRadius: 8, color: "var(--ink)", padding: "8px 10px", fontSize: 13, fontFamily: "inherit", lineHeight: 1.5, resize: "vertical", boxSizing: "border-box" }} />
        <div style={{ fontSize: 11, color: "var(--dim)", marginTop: 3 }}>Shown on the report as this observer's witness statement.</div>
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
const AERIAL_ENABLED = false; // 🛰 looking-DOWN aerial mode (platform/GCP geolocation) — hidden while the sky app is refined; flip to bring back. All aerial code (AerialMeasure/AerialGroundMap/geolocate.js) stays intact.
const ENABLE_CAPTURE = true; // 📷 in-app sensor camera (getUserMedia + DeviceMotion) — records az/el/roll at the shutter that EXIF omits. Separate from ENABLE_SENSORS (the older SkyAimer AR buttons). Real-device feature; the on-screen readout is self-calibrating.

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

function SkyAimer({ open, onClose, lat, lng, whenMs, initAz, initAlt, marks, which, onCapture, source, update, wizard, onWizardBack, onWizardNext, single }) {
  const [vpRef, vp] = useSize();
  const [topBarRef, topBar] = useSize(); // top HUD height — reserve it while placing
  const [botBarRef, botBar] = useSize(); // bottom controls height — reserve it while placing
  const [bandPx, setBandPx] = useState(null); // exact clear band {top,bot} in container px while placing; bot is high-water (tallest controls) so a shrinking hint never rescales the photo
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
  const [objOn, setObjOn] = useState(true);   // 🛸 object overlay (wireframe + marks) in the dome AND burned into exports
  const [hueOpen, setHueOpen] = useState(false); // the overlay-color slider, folded behind its swatch
  const [pMode, setPMode] = useState("look"); // 'look' | 'place'
  const [pAz, setPAz] = useState(180);
  const [pEl, setPEl] = useState(30);
  const [pRoll, setPRoll] = useState(0);
  const [fovM, setFovM] = useState(68);      // photo's own FOV (calibrated by pinch)
  const [pDist, setPDist] = useState(0);     // radial lens distortion (tan-space k) — 0 unless star-calibrated
  /* place-mode DISPLAY zoom + pan — magnifies the photo+sky together to line up
     fine detail (a distant ridge, a rooftop). Purely cosmetic: it does NOT touch
     the pose (pAz/pEl/fovM/pRoll), only how big the locked photo+sky pair draws. */
  const [pZoom, setPZoom] = useState(1);
  const [pPan, setPPan] = useState({ x: 0, y: 0 });
  const [panMode, setPanMode] = useState(false);
  const dispPanRef = useRef(null);
  const resetPlaceView = () => { setPZoom(1); setPPan({ x: 0, y: 0 }); setPanMode(false); };
  /* measure the EXACT gap between the top HUD and the bottom controls (rects
     include padding + safe-area, unlike contentRect), so the placement can fill
     it. bot is high-water (keep the highest controls-top seen this session) so a
     shorter hint line never re-scales the photo. */
  useLayoutEffect(() => {
    if (pMode !== "place") return;
    const c = vpRef.current, tb = topBarRef.current, bb = botBarRef.current;
    if (!c || !tb || !bb) return;
    const cr = c.getBoundingClientRect();
    const top = tb.getBoundingClientRect().bottom - cr.top;
    const bot = bb.getBoundingClientRect().top - cr.top;
    setBandPx((p) => {
      const nBot = p ? Math.min(p.bot, bot) : bot; // high-water: tallest controls
      if (p && Math.abs(p.top - top) < 1 && p.bot === nBot) return p;
      return { top, bot: nBot };
    });
  }, [pMode, vp.w, vp.h, topBar.h, botBar.h]);
  /* placement UNDO — snapshot the pose before each change (gesture or button) so
     an accidental nudge right before ✓ Done can be stepped back. Display zoom/pan
     are cosmetic and intentionally NOT part of it. */
  const [placeUndo, setPlaceUndo] = useState([]);
  const pendUndoRef = useRef(null);   // pose at the START of the current touch gesture
  const placeMovedRef = useRef(false); // did that gesture actually change the pose?
  const snapPose = () => ({ az: pAz, el: pEl, roll: pRoll, fov: fovM, dist: pDist, px: pPan.x, py: pPan.y, zoom: pZoom });
  const posesEq = (a, b) => a && b && a.az === b.az && a.el === b.el && a.roll === b.roll && a.fov === b.fov && a.dist === b.dist;
  const pushUndo = (snap) => setPlaceUndo((st) => posesEq(st[st.length - 1], snap) ? st : [...st, snap].slice(-24));
  const undoPlace = () => setPlaceUndo((st) => {
    if (!st.length) return st;
    const s = st[st.length - 1];
    setPAz(s.az); setPEl(s.el); setPRoll(s.roll); setFovM(s.fov); setPDist(s.dist);
    if (s.px != null) setPPan({ x: s.px, y: s.py }); // restore the view (roll pivots shift the pan)
    if (s.zoom != null) setPZoom(s.zoom);
    calibRecRef.current = null; // hand-restored → no longer a star/terrain-calibrated pose
    return st.slice(0, -1);
  });
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
  const [trajOn, setTrajOn] = useState(false);  // trajectory drop-point tools — its own mode
  /* clean-viewing collapses (field ask): tuck the header layer chips and the
     bottom control stack away. The bottom ACTION row and the playback
     scrubber always persist — never strand the user without navigation or
     the video controls. */
  const [hdrMin, setHdrMin] = useState(false);
  const [botMin, setBotMin] = useState(false);
  const [mapPick, setMapPick] = useState(null); // {mode:'size'|'compare'} → distance-on-a-map modal open
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
  const calibNamesRef = useRef([]);    // names of the tapped calibration objects
  const calibRecRef = useRef(null);    // {method, ...} — HOW the image was aligned, persisted to source.calib for the report
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
  const [vidFrameUrl, setVidFrameUrl] = useState(null); // baked align-frame data URL for Place mode
  /* --- stabilized (world-locked) video playback state ---
     `playPose` is a DISPLAY OVERLAY pose: while set, the warp draws the current
     playback frame at ITS solved pose instead of the placement pose. It never
     touches pAz/pEl/..., so commitPlacement can't accidentally write a mid-video
     pose into mediaAim. Cleared when playback exits (after the marked frame is
     re-baked, so texture and pose always agree). */
  const [playPose, setPlayPose] = useState(null);   // {az,el,roll,fov,k} | null
  /* --- ⚓ FIX FRAMES: manual correction of the auto-stabilized pose path ---
     Scrub to a frame where the solve lost the world lock, drag the photo back
     onto the true horizon/terrain (twist = tilt), and ⚓ Anchor it. Anchors are
     ABSOLUTE poses in source.poseFixes; applyPoseFixes turns them into a delta
     field (exact at anchors, linear between, held beyond the ends) over the
     smoothed base path, and applyDirFixes shifts the object track through the
     same field — so waypoints/trajectory/wireframe/export all follow. The
     pending (un-anchored) adjustment lives ONLY in playPose (display override),
     so scrubbing away discards it and commitPlacement can never absorb it. */
  const [fixOn, setFixOn] = useState(false);
  useEffect(() => { setPanMode(false); }, [fixOn]); // ✋ never carries across a fix-mode toggle
  const [fineOn, setFineOn] = useState(false); // 🎛 place-mode fine-tune nudge buttons
  const fixDragRef = useRef(null);   // {x, y, az, el} — one-finger photo drag baseline
  const fixTwistRef = useRef(null);  // two-finger: twist=roll, pinch=view zoom, mid-drag=view pan
  const fixRafRef = useRef(0);
  const fixPendRef = useRef(null);
  const queueFix = (mut) => {        // rAF-coalesced playPose edit (120 Hz phones flood React otherwise)
    const p = fixPendRef.current || {};
    if (mut.az != null) { p.az = mut.az; p.el = mut.el; }
    if (mut.dRoll) p.dRoll = (p.dRoll || 0) + mut.dRoll;
    fixPendRef.current = p;
    if (!fixRafRef.current) fixRafRef.current = requestAnimationFrame(() => {
      fixRafRef.current = 0;
      const q = fixPendRef.current; fixPendRef.current = null;
      if (!q) return;
      setPlayPose((pp) => {
        if (!pp) return pp;
        const n2 = { ...pp };
        if (q.az != null) { n2.az = ((q.az % 360) + 360) % 360; n2.el = clampN(q.el, -89, 89); }
        if (q.dRoll) n2.roll = clampN((pp.roll || 0) - q.dRoll, -180, 180); // rotate(−roll): photo tracks the fingers
        return n2;
      });
    });
  };
  const [playIdx, setPlayIdx] = useState(0);
  const [playing, setPlaying] = useState(false);
  const playingRef = useRef(false);
  const playVidRef = useRef(null);   // offscreen <video> for playback seeks (never in the render path)
  const seekBusyRef = useRef(false); // single-in-flight seek guard (iOS: concurrent seeks jank/crash)
  const pendingIdxRef = useRef(null);
  const [stabBusy, setStabBusy] = useState(0);      // 0 idle | frames-done counter while solving
  const [mSolveOpen, setMSolveOpen] = useState(false); // "Solve from marks" smoothing slider popup open?
  const [mSmooth, setMSmooth] = useState(40);          // manual-solve smoothing amount 0..100
  const [stabTotal, setStabTotal] = useState(0);    // total steps of the current solve (camera + object pass) — progress lives in the button, not the auto-hiding flash
  const stabAbortRef = useRef(0);
  const placeRef = useRef(null);
  const twistRef = useRef(null);
  const warpRef = useRef(null);   // canvas that draws the Look-mode warp ourselves
  const texRef = useRef(null);
  const bakeCvRef = useRef(null);   // reused bake canvas — allocating one per playback frame was GC-thrashing the world view
  const [, setTexReady] = useState(0);

  /* decode the still texture for the canvas warp ONCE. Image: the EXIF-
     normalized photo. Video: the marked frame (A.videoTime) baked to a
     canvas off the render path, so the warp draws a static texture — never
     a live <video> — which is what made the sky view fast and stable. */
  /* Warp texture resolution. 1280 was the original drag-perf/memory cap; 1600
     noticeably sharpens the placed photo when the artist zooms into the sky
     view to line up a ridge, at ~1.6× the texture memory (still small) and no
     change to the 7-column mesh triangle count, so per-frame draw cost barely
     moves. If a slower device ever stutters here, this is the dial to lower. */
  const MAXT = 1600;
  const bakeTex = (drawable, w, h) => {
    const adj = source?.imgAdj, needAdj = !imgAdjNeutral(adj);
    let tex = drawable;
    try {
      if (w > MAXT || h > MAXT || needAdj) {   // draw to a canvas to downscale and/or bake the B/C adjustment in
        const sc = Math.min(1, MAXT / Math.max(w, h));
        const tw = Math.round(w * sc), th = Math.round(h * sc);
        /* reuse one canvas across frames — a fresh 1600 px canvas per playback
           frame allocated ~5 MB each time and thrashed GC. Bake is synchronous
           (getImageData→loop→putImageData) so the mesh warp, which reads this
           canvas, can never catch a half-written frame. willReadFrequently only
           when we actually read it back (the B/C pass). */
        let cv = bakeCvRef.current;
        /* getContext caches its options — a canvas made without the read hint
           keeps a GPU backing even when we later need fast getImageData, so
           re-create it if the hint requirement flips (toggling B/C on/off) */
        if (!cv || cv._wrf !== needAdj) { cv = bakeCvRef.current = document.createElement("canvas"); cv._wrf = needAdj; }
        if (cv.width !== tw) cv.width = tw;
        if (cv.height !== th) cv.height = th;
        const cx = cv.getContext("2d", { willReadFrequently: needAdj });
        cx.drawImage(drawable, 0, 0, tw, th);
        if (needAdj) applyImgAdj(cx, tw, th, adj);
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
      /* the ALIGNMENT frame: scrubbable in place mode (source.alignT) so the
         world can be aligned on the clearest-horizon frame, independent of
         the frame the object was marked on (A.videoTime) — the falls back
         keep them coupled until the user moves the align scrubber */
      const t = isNum(source?.alignT) ? +source.alignT : isNum(source?.A?.videoTime) ? +source.A.videoTime : 0;
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
  }, [source?.mediaUrl, source?.mediaKind, source?.A?.videoTime, source?.alignT, source?.imgAdj?.bri, source?.imgAdj?.con]);

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
      calibPrevRef.current = null; calibAnchorsRef.current = []; calibNamesRef.current = [];
      calibRecRef.current = source?.calib || null; // keep a prior session's calibration record unless re-aligned/dragged
      setPhotoOn(!!source?.mediaUrl);
      /* start in Place only until this observer has been placed ONCE in the
         sky view (source.placed) — after that, return straight to Look and
         skip the adjust step. `mediaAim` alone can't gate this: it's also
         pre-set from the photo's EXIF bearing before any placement. The
         ✥ Place button is still there to re-adjust. */
      setPMode(source?.placed || !source?.mediaUrl ? "look" : "place");
      /* only auto-fit the FOV when we OPEN straight into Look on an already-placed
         photo — not after a place→done (donePlace sets its own framing) */
      didFitRef.current = !(source?.placed && source?.mediaUrl);
    }
  }, [open]); // eslint-disable-line
  /* Opening into Look left the warped photo tiny against the wide default sky.
     Fit the view FOV so the photo fills the frame — the same "fit to extents"
     Place mode does — once the viewport + photo dimensions are known. */
  /* the FOV that makes the photo fill the frame — i.e. the world view shown at
     the SAME scale as the flat photo on the measure step. Returns null until the
     dimensions are known. */
  const fitFovToPhoto = () => {
    if (!(source?.natW > 0 && source?.natH > 0) || !(vp.w > 0 && vp.h > 0)) return null;
    const fovMm = isNum(source?.fovH) ? +source.fovH : 68;
    const aspect = source.natH / source.natW;
    const fitT = (Math.tan((fovMm * RAD) / 2) / 0.92) * Math.max(1, aspect * (vp.w / vp.h));
    return clampN(+(2 * Math.atan(fitT) * R2D).toFixed(1), 2, 90);
  };
  const didFitRef = useRef(false);
  useEffect(() => {
    if (!open || didFitRef.current || pMode !== "look") return;
    const f = fitFovToPhoto();
    if (f == null) return;
    setFov(f);
    didFitRef.current = true;
  }, [open, pMode, vp.w, vp.h, source?.natW, source?.natH]); // eslint-disable-line

  /* scroll lock while the aimer is open. Range inputs are whitelisted:
     this document-level preventDefault is what silently killed every
     slider drag inside the aimer on touch devices. */
  useEffect(() => {
    if (!open) return;
    const prevent = (e) => {
      if (e.target && e.target.closest && e.target.closest("input[type=range], .help-scroll, .mappick")) return;
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
  /* urban building silhouette: a DEDICATED rooftop line (OSM footprints +
     heights) drawn beside the terrain line, for aligning a photo shot in town
     where there are no mountains. Opt-in — most sightings are rural, and the
     Overpass fetch isn't free. Kept separate from `terr` so "Snap to ridges"
     stays terrain-only (assumed rooftop heights must not drive calibration). */
  const [bldgOn, setBldgOn] = useState(false);
  const [bldg, setBldg] = useState(null); // {boxes, peak, buildings} | {err} | null
  /* ridge/terrain line hue — a display preference (the default green washes
     out over green hillsides for some photos); persisted across sessions.
     Default 106° ≈ the original rgba(158,224,138). */
  /* one overlay-colour hue for everything drawn OVER the photo/sky — crosshair,
     object outline, terrain ridges & peak labels — so the artist can pick a
     tint that stands out against their own photo. Default ≈ the app amber. */
  const [ridgeHue, setRidgeHue] = useState(() => {
    try { const raw = localStorage.getItem("phodar:uiHue"); if (raw != null && Number.isFinite(+raw)) return +raw; } catch (e) { }
    return 40;
  });
  useEffect(() => { try { localStorage.setItem("phodar:uiHue", String(ridgeHue)); } catch (e) { } }, [ridgeHue]);
  const ridgeCol = (a) => `hsla(${ridgeHue},58%,71%,${a})`;             // softer tint for ridge/terrain lines
  const accentCol = `hsl(${ridgeHue},92%,58%)`;                        // punchy accent for crosshair + marks + ridges
  /* the OBJECT wireframe carries its OWN colour (shapeFit.hue, set by the
     measure-step "color" slider) — consistent across measure/dome/export, so
     that slider actually recolours the object everywhere. accentCol (the
     sky-view swatch) stays for the crosshair, marks and terrain accents. */
  const objCol = source?.shapeFit ? `hsl(${source.shapeFit.hue ?? 36},88%,60%)` : accentCol;
  useEffect(() => {
    if (!open || !terrOn || !hasPos) return;
    let dead = false;
    setTerr((t) => t && t.els ? t : null);
    predictedSkyline(LAT, LNG)
      .then((sk) => { if (!dead) setTerr(sk); })
      .catch((e) => { if (!dead) setTerr({ err: String(e?.message || e) }); });
    return () => { dead = true; };
  }, [open, terrOn, hasPos, LAT, LNG]);

  /* building silhouette (opt-in) — its own dedicated rooftop line. Clearing on
     toggle-off means a toggle-off/on shows the spinner and re-fetches cleanly
     (the cache dropped any empty/errored result, so this is a real retry). */
  useEffect(() => {
    if (!open || !bldgOn || !hasPos) { setBldg(null); return; }
    let dead = false;
    setBldg(null);
    predictedBuildingBoxes(LAT, LNG)
      .then((b) => { if (!dead) setBldg(b); })
      .catch((e) => { if (!dead) setBldg({ err: String(e?.message || e) }); });
    return () => { dead = true; };
  }, [open, bldgOn, hasPos, LAT, LNG]);

  /* winds aloft (opt-in) — the balloon test made visual: a vertical profile of
     which way, and how fast, the wind pushes at each altitude AT THE SIGHTING
     TIME. If an object drifted with one of these, it's likely a balloon. */
  const [windOn, setWindOn] = useState(false);
  const [cloudOn, setCloudOn] = useState(false); // ☁ cloud layer on the dome
  const [windProf, setWindProf] = useState(null); // {levels, src} | {err} | null
  useEffect(() => {
    if (!open || !windOn || !hasPos) { setWindProf(null); return; }
    let dead = false;
    setWindProf(null);
    fetchWindProfile(LAT, LNG, T)
      .then((w) => { if (!dead) setWindProf(w); })
      .catch((e) => { if (!dead) setWindProf({ err: String(e?.message || e) }); });
    return () => { dead = true; };
  }, [open, windOn, hasPos, LAT, LNG, T]);
  /* weather for the size tool's cloud-base cap AND the ☁ cloud dome layer —
     fetched only when one of those is open (one call; the report fetches its
     own). `err` on failure so the cloud chip can say so instead of vanishing. */
  const [wxSky, setWxSky] = useState(null);
  const wxWanted = sizeOn || cloudOn;
  useEffect(() => {
    if (!open || !wxWanted || !hasPos) { setWxSky(null); return; }
    let dead = false;
    fetchWeatherAt(LAT, LNG, T).then((w) => { if (!dead) setWxSky(w); }).catch(() => { if (!dead) setWxSky({ err: true }); });
    return () => { dead = true; };
  }, [open, wxWanted, hasPos, LAT, LNG, T]);

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
     legible. Per-sat pass trails are computed for every sunlit member (SGP4 is
     memoised on time/position, off the pan/zoom path) so whichever ones sit
     near the photo/sight-line — even low on the horizon — always have a path;
     the render then only DRAWS the ones near the view/sight-line. */
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
    const lit = satsAt(slDb.sats, T, LAT, LNG, 0).filter((s) => s.lit).slice(0, 60);
    return lit.map((s) => ({ ...s, trail: satTrail(s.rec, T, LAT, LNG) }));
  }, [starlinkOn, slDb, T, LAT, LNG, hasPos]);

  /* named peaks (OSM Overpass) — placed on the terrain skyline by bearing +
     curvature-corrected elevation; opt-in, fetched once per open. */
  const [peaksOn, setPeaksOn] = useState(false);
  const [peaks, setPeaks] = useState(null); // [] | {err}
  useEffect(() => {
    if (!open || !peaksOn || !hasPos) { setPeaks(null); return; }
    let dead = false;
    setPeaks(null); // spinner while (re)fetching — toggle-off/on is a clean retry
    fetchPeaks(LAT, LNG, isNum(source?.alt) ? +source.alt : 0, 120) // wide net: tall far peaks (Shasta, McLoughlin…) sit well past 40 km
      .then((ps) => { if (!dead) setPeaks(ps); })
      .catch((e) => { if (!dead) setPeaks({ err: String(e?.message || e) }); });
    return () => { dead = true; };
  }, [open, peaksOn, hasPos, LAT, LNG]); // eslint-disable-line
  /* Named summits that actually sit ON the drawn DEM silhouette — a peak's own
     angular elevation (from its OSM `ele`) must land within a small band of the
     terrain skyline at its azimuth: below it ⇒ occluded by nearer terrain (drop
     it), well above ⇒ the DEM undersampled/didn't reach it (also drop — it's not
     on the line you align to). No prominence cap, so EVERY visible silhouette
     peak is caught; `elv` is where to sit the marker on the line. Until the
     terrain loads we can't test, so show a nearest-first sample provisionally. */
  const peakMarks = (() => {
    if (!(peaksOn && Array.isArray(peaks)) || !peaks.length) return [];
    if (!terr?.els) return peaks.slice().sort((a, b) => a.distKm - b.distKm).slice(0, 40).map((pk) => ({ ...pk, elv: pk.el ?? 0 }));
    const out = [];
    for (const pk of peaks) {
      const sky = skylineElAt(terr.els, pk.az);
      if (pk.el == null) { out.push({ ...pk, elv: sky }); continue; } // no ele → assume it's the local ridge
      if (Math.abs(pk.el - sky) <= 0.9) out.push({ ...pk, elv: sky }); // its summit coincides with the drawn skyline → on it
    }
    return out;
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
  /* Keep the WHOLE pinned photo visible while placing: lift it into the clear
     band between the top HUD and the bottom controls, and shrink it to fit if
     it's tall (portrait / wide-FOV). Purely cosmetic — the pose (pAz/pEl/fovM)
     is untouched, and because the sky shifts by the same placeDY and scales
     with FRAMEeff, the photo rectangle stays exactly locked to the projected
     frustum (proven: the frame edges map to ±fovM/2 for any FRAME/offset). */
  let placeDY = 0, FRAMEeff = FRAME;
  if (placing && vp.h > 0 && vp.w > 0 && bandPx && bandPx.bot - bandPx.top > 80) {
    const bandTop = bandPx.top + 8, bandBot = bandPx.bot - 8; // exact measured gap, small breathing room
    placeDY = ((bandTop + bandBot) / 2 - vp.h / 2) / vp.h;     // shift the placement center into the band
    const aspect = (source?.natW && source?.natH) ? source.natH / source.natW : 9 / 16;
    const fit = (bandBot - bandTop) / (vp.w * aspect);         // width fraction that fills the band height
    FRAMEeff = clampN(Math.min(0.96, fit), 0.5, 0.96);         // fill up to ~full width; height-bound for tall photos
  }
  /* display zoom (pZoom) + pan (pPan) fold straight into the placement frame and
     centre — magnifying photo+sky together while keeping them locked (the frame
     edges still map to ±fovM/2). Off (zoom 1, pan 0) unless placing. */
  const FRAMEz = FRAMEeff * (placing ? pZoom : 1);
  /* When the size/compare panel opens it grows the bottom bar and covers the
     lower sky. Re-centre the aim into the still-visible band (top HUD → panel
     top) by shifting the projection centre (and the crosshair) up half the
     bottom-bar-minus-top-bar height, so the crosshair never hides behind the
     panel. Look mode only — place mode has its own band-centering (placeDY). */
  const panelOpen = !placing && (sizeOn || cmpOn || trajOn);
  const lookDY = (panelOpen && vp.h > 0) ? clampN((topBar.h - botBar.h) / (2 * vp.h), -0.4, 0.05) : 0;
  const cx = 0.5 + (placing ? pPan.x : 0);
  const cy = 0.5 + placeDY + lookDY + (placing ? pPan.y : 0);
  /* vertical centre of the VISIBLE band (top HUD → bottom controls), for the
     right-side zoom/pan buttons — so a mode's panel never covers them */
  const ctrlBandPct = vp.h > 0 ? clampN((topBar.h + (vp.h - botBar.h)) / 2 / vp.h, 0.18, 0.82) * 100 : 50;
  const effAz = placing ? pAz : viewAz;
  const effAlt = placing ? clampN(pEl, -20, EL_MAX) : viewAlt;
  const effFov = placing
    /* The photo (width FRAMEz) always spans fovM, so the sky must be projected at
       exactly this FOV to stay locked to it. A 12° floor used to clamp this once
       the display zoom pushed the true value below 12° — which un-locked the
       terrain/grid overlays from the pinned photo (they drifted toward centre as
       you zoomed). Floor is now 0.3° so the projection tracks any zoom. */
    ? clampN(2 * Math.atan(Math.tan((fovM * RAD) / 2) / FRAMEz) * R2D, 0.3, 135)
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
    return { x: cx + (xc / zc) / (2 * tanH), y: cy - (yc / zc) / (2 * tanV), inFront: true };
  };
  const project = (azDg, altDg) => projectD(dirOf(azDg, altDg));
  const unproject = (xf, yf) => {
    const sx = (xf - cx) * 2 * tanH, sy = -(yf - cy) * 2 * tanV;
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
    return { ids: ks, dist: Math.hypot(a.x - b.x, a.y - b.y) || 1, ang: Math.atan2(b.y - a.y, b.x - a.x), mx: (a.x + b.x) / 2, my: (a.y + b.y) / 2 };
  };
  const onBgDown = (e) => {
    e.currentTarget.setPointerCapture(e.pointerId);
    pointersRef.current.set(e.pointerId, { x: e.clientX, y: e.clientY, t: e.timeStamp });
    const n = pointersRef.current.size;
    if (placing && n === 1) { pendUndoRef.current = snapPose(); placeMovedRef.current = false; } // arm undo for this gesture
    if (n >= 2) {
      /* fix mode: two fingers = twist rolls the FRAME POSE, pinch zooms the
         view, midpoint drag pans the view — the photo edit stays in playPose */
      if (fixOn && playPose && !placing) {
        fixDragRef.current = null; panRef.current = null; calibTapRef.current = null;
        const g = twoPtGeom();
        if (g) fixTwistRef.current = { ...g, vAz: viewAz, vAlt: viewAlt, mx0: g.mx, my0: g.my };
        return;
      }
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
      panRef.current = null; placeRef.current = null; dispPanRef.current = null; rotDragRef.current = null; calibTapRef.current = null;
      const g = twoPtGeom();
      if (placing && g) {
        /* pivot mode: fingers landing TOGETHER roll about their midpoint; one
           finger first THEN the other → the first finger is the pivot and the
           second swings around it (decided by the gap between the two touches). */
        const pa = pointersRef.current.get(g.ids[0]), pb = pointersRef.current.get(g.ids[1]);
        const gap = (pa && pb && pa.t != null && pb.t != null) ? Math.abs(pa.t - pb.t) : 0;
        g.pivotMode = gap > 140 ? "anchor" : "mid";
        g.anchorId = (pa && pb && pa.t <= pb.t) ? g.ids[0] : g.ids[1]; // the earlier finger anchors
      }
      if (placing) twistRef.current = g;       // {ids, dist, ang, pivotMode, anchorId} — rebaselined every event
      else pinchRef.current = g;
    } else if (placing && panMode) {
      dispPanRef.current = { x: e.clientX, y: e.clientY, px: pPan.x, py: pPan.y };
    } else if (placing) {
      placeRef.current = { x: e.clientX, y: e.clientY, az: pAz, el: pEl };
    } else if (fixOn && playPose && !panMode) {
      /* fix mode: one finger drags the PHOTO onto the true horizon (the view
         camera stays put) — baseline at the gesture start, like place mode.
         With ✋ pan mode on, this falls through to the normal view pan. */
      fixDragRef.current = { x: e.clientX, y: e.clientY, az: playPose.az, el: playPose.el };
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
    if (pointersRef.current.has(e.pointerId)) pointersRef.current.set(e.pointerId, { x: e.clientX, y: e.clientY, t: pointersRef.current.get(e.pointerId)?.t ?? e.timeStamp });
    const n = pointersRef.current.size;
    if (n >= 2) {
      /* fix mode: twist → frame roll, pinch → view zoom, midpoint drag → view pan */
      if (fixTwistRef.current) {
        const t = fixTwistRef.current;
        const g = twoPtGeom(t.ids); if (!g) return;
        let dA = g.ang - t.ang;
        if (dA > Math.PI) dA -= 2 * Math.PI;
        if (dA < -Math.PI) dA += 2 * Math.PI;
        const ratio = g.dist / t.dist;
        t.ang = g.ang; t.dist = g.dist;
        if (Math.abs(dA) > 0.6 || ratio < 0.67 || ratio > 1.5) return; // pointer glitch — skip this event
        /* same one-axis-per-gesture lock as place mode: the first of
           twist(roll) / pinch(view zoom) to cross its threshold wins */
        t.pendRot = (t.pendRot || 0) + dA * R2D;
        t.pendScale = (t.pendScale || 1) * ratio;
        if (!t.lock) {
          if (Math.abs(t.pendRot) > 0.8) t.lock = "rot";
          else if (t.pendScale > 1.015 || t.pendScale < 1 / 1.015) t.lock = "scale";
        }
        if (t.lock === "rot" && Math.abs(t.pendRot) > 0.001) { queueFix({ dRoll: t.pendRot }); t.pendRot = 0; t.pendScale = 1; }
        if (t.lock === "scale" && t.pendScale !== 1) { setFov((f) => clampN(f / t.pendScale, 2, 90)); t.pendScale = 1; t.pendRot = 0; }
        if (vp.w) queuePose("look",
          (((t.vAz - (g.mx - t.mx0) / vp.w * fovH) % 360) + 360) % 360,
          clampN(t.vAlt + (g.my - t.my0) / (vp.h || vp.w) * fovV, -15, EL_MAX));
        return;
      }
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
        /* ONE AXIS PER GESTURE (field ask): the first channel to cross its
           threshold claims the whole gesture — twisting no longer bleeds
           zoom and pinching no longer bleeds roll. Lift fingers to switch. */
        if (!t.lock) {
          if (Math.abs(t.pendRot) > 0.8) t.lock = "rot";
          else if (t.pendScale > 1.015 || t.pendScale < 1 / 1.015) t.lock = "scale";
        }
        if (t.lock === "rot") t.pendScale = 1;
        if (t.lock === "scale") t.pendRot = 0;
        let doRot = 0, doScale = 1;
        if (t.lock === "rot" && Math.abs(t.pendRot) > 0.8) { doRot = t.pendRot; t.pendRot = 0; }
        if (t.lock === "scale" && (t.pendScale > 1.015 || t.pendScale < 1 / 1.015)) { doScale = t.pendScale; t.pendScale = 1; }
        if (doRot || doScale !== 1) {
          placeMovedRef.current = true;
          if (doScale !== 1) setFovM((f) => clampN(f / doScale, 12, 120)); // inverted pinch: fingers apart → tighter FOV
          if (doRot) {
            setPRoll((r) => clampN(r - doRot, -90, 90)); // rotate(−roll): photo tracks the fingers
            /* pivot the roll on the FINGER MIDPOINT, not the photo center (which
               is off-screen when zoomed). Orbit the center around the midpoint by
               the same angle so the content under your fingers stays put. */
            if (vpRef.current && vp.w) {
              const rect = vpRef.current.getBoundingClientRect();
              /* pivot: the first finger (anchor mode) or the finger midpoint (mid mode) */
              const anc = t.pivotMode === "anchor" ? pointersRef.current.get(t.anchorId) : null;
              const Px = anc ? anc.x : g.mx, Py = anc ? anc.y : g.my;
              const vX = cx * vp.w - (Px - rect.left), vY = cy * vp.h - (Py - rect.top); // (center − pivot), px
              const phi = doRot * RAD, cph = Math.cos(phi), sph = Math.sin(phi);            // CSS applies +doRot°
              const dPx = (cph * vX - sph * vY - vX) / vp.w, dPy = (sph * vX + cph * vY - vY) / (vp.h || vp.w);
              setPPan((p) => ({ x: clampN(p.x + dPx, -2.5, 2.5), y: clampN(p.y + dPy, -2.5, 2.5) }));
            }
          }
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
    if (dispPanRef.current && vp.w) {
      const d = dispPanRef.current; // display pan: slide the magnified photo+sky together (pose untouched)
      const lim = 2.5; // generous — matches the roll-pivot correction so panning never snaps
      setPPan({
        x: clampN(d.px + (e.clientX - d.x) / vp.w, -lim, lim),
        y: clampN(d.py + (e.clientY - d.y) / (vp.h || vp.w), -lim, lim),
      });
      return;
    }
    if (fixDragRef.current && vp.w) {
      /* fix mode: the photo follows the finger across the fixed sky — screen
         right = +az, screen down = −el, scaled by the VIEW fov (the photo is
         world-locked, so its screen motion is angular motion) */
      const fd = fixDragRef.current;
      const dx = (e.clientX - fd.x) / vp.w, dy = (e.clientY - fd.y) / (vp.h || vp.w);
      queueFix({ az: fd.az + dx * fovH, el: fd.el - dy * fovV });
      return;
    }
    if (placeRef.current && vp.w) {
      const pr = placeRef.current; // snapshot: pointerup may null the ref before React flushes
      const dx = (e.clientX - pr.x) / vp.w, dy = (e.clientY - pr.y) / (vp.h || vp.w);
      const nAz = (((pr.az - dx * fovH) % 360) + 360) % 360; // inverted: drag the SKY with your finger (grab), matching pan mode
      const nEl = clampN(pr.el + dy * fovV, -20, EL_MAX);
      if (Math.abs(dx) + Math.abs(dy) > 0.01) { calibRecRef.current = null; placeMovedRef.current = true; } // hand-dragged → no longer a star/terrain-calibrated pose
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
    if (n < 2) { pinchRef.current = null; twistRef.current = null; fixTwistRef.current = null; }
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
      else if (fixOn && playPose && !panMode) fixDragRef.current = { x: p.x, y: p.y, az: playPose.az, el: playPose.el }; // hand back to photo drag (a pending rAF pose is at most one frame stale)
      else if (rotDragRef.current) { /* twist handed control back to this finger — keep the rotate drag */ }
      else panRef.current = { x: p.x, y: p.y, az: viewAz, alt: viewAlt };
    } else if (n === 0) {
      /* gesture ended — if it actually moved the pose, bank the pre-gesture state for Undo */
      if (placing && pendUndoRef.current && placeMovedRef.current) pushUndo(pendUndoRef.current);
      pendUndoRef.current = null; placeMovedRef.current = false;
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
      panRef.current = null; placeRef.current = null; dispPanRef.current = null; rotDragRef.current = null; rotTwistRef.current = null; fixDragRef.current = null; fixTwistRef.current = null;
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
  /* building rooftops — each OSM footprint as its own projected wireframe box
     (roof outline + vertical corner edges), amber, so distinct buildings can be
     matched to the photo instead of one merged silhouette. Base is the
     observer's ground plane (flat-city assumption); brighter for measured/
     floor-count heights, fainter for assumed ones. */
  /* camera height above ground for the building boxes. Handheld ground shots
     are eye height (1.6 m); a photo from an upper floor / balcony sits metres
     higher, which is exactly why near rooftops looked "too tall". We recover it
     from EXIF: GPS altitude (meta.alt, above sea level) minus the DEM ground at
     the observer (terr.h0). GPS altitude wobbles ±5 m, so we only auto-elevate
     when the difference is clearly real (> 3 m) and let the user nudge it. */
  const autoCamH = (isNum(source?.meta?.alt) && terr?.h0 != null) ? +source.meta.alt - terr.h0 : null;
  const camH = isNum(source?.camH) ? clampN(+source.camH, 1.6, 300)
    : (autoCamH != null && autoCamH > 3 ? clampN(autoCamH, 1.6, 300) : 1.6);
  const bldgBoxes = (bldgOn && bldg?.boxes && !cameraOn) ? (() => {
    const K = 0.13, eye = camH;
    const xy = (p) => (p[0] * 100).toFixed(2) + " " + (p[1] * 100).toFixed(2);
    /* Project each in-view building. We KNOW the footprint exactly, but almost
       never the height — so draw the accurate footprint (ground outline) for
       every building, and extrude a full box ONLY where the height is real
       (measured or floor-count). Guessing one uniform height for thousands of
       untagged buildings produced a meaningless "barcode" of same-top boxes;
       the ground outlines instead read as a floor-plan you can match. */
    const foot = [], known = []; // footprint paths; known-height boxes to extrude
    for (const b of bldg.boxes) {
      let cE = 0, cN = 0;
      for (const p of b.ring) { cE += p[0]; cN += p[1]; }
      cE /= b.ring.length; cN /= b.ring.length;
      const da = ((((Math.atan2(cE, cN) * R2D) + 360) % 360) - effAz + 540) % 360 - 180;
      if (Math.abs(da) > 115) continue; // outside the visible dome window
      const base = [], roof = [], all = []; let ok = true;
      for (const [e, n] of b.ring) {
        const dist = Math.hypot(e, n);
        const az = ((Math.atan2(e, n) * R2D) + 360) % 360;
        const curv = (dist * dist * (1 - K)) / (2 * 6371000);
        const bp = project(az, Math.atan2(-eye - curv, dist) * R2D);
        if (!bp.inFront) { ok = false; break; }
        base.push([bp.x, bp.y]); all.push([bp.x, bp.y]);
        if (!b.assumed) {
          const rp = project(az, Math.atan2(b.h - eye - curv, dist) * R2D);
          if (!rp.inFront) { ok = false; break; }
          roof.push([rp.x, rp.y]); all.push([rp.x, rp.y]);
        }
      }
      if (!ok || base.length < 3) continue;
      foot.push("M " + base.map(xy).join(" L ") + " Z");
      if (!b.assumed) {
        let x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
        for (const [x, y] of all) { if (x < x0) x0 = x; if (y < y0) y0 = y; if (x > x1) x1 = x; if (y > y1) y1 = y; }
        known.push({ base, roof, hull: convexHull2(all), bbox: [x0, y0, x1, y1] });
      }
    }
    const out = foot.map((d) => ({ d, faint: true })); // accurate footprints, faint
    /* extrude the known-height boxes with hidden-line removal against nearer ones */
    for (let i = 0; i < known.length; i++) {
      const B = known[i], hulls = [];
      for (let j = 0; j < i && hulls.length < 40; j++) if (bboxHit(B.bbox, known[j].bbox)) hulls.push(known[j].hull);
      const edges = [], N = B.base.length;
      for (let k = 0; k < N; k++) edges.push([B.roof[k], B.roof[(k + 1) % N]]); // roofline
      for (let k = 0; k < N; k++) edges.push([B.base[k], B.roof[k]]);             // vertical corners
      let d = "";
      for (const [p, q] of edges) for (const [t0, t1] of visibleSegs(p, q, hulls)) {
        const ax = p[0] + (q[0] - p[0]) * t0, ay = p[1] + (q[1] - p[1]) * t0;
        const bx = p[0] + (q[0] - p[0]) * t1, by = p[1] + (q[1] - p[1]) * t1;
        d += `M ${(ax * 100).toFixed(2)} ${(ay * 100).toFixed(2)} L ${(bx * 100).toFixed(2)} ${(by * 100).toFixed(2)} `;
      }
      if (d) out.push({ d, faint: false });
    }
    return out;
  })() : [];
  /* tallest rooftop (using the effective camera height) — for the label + note */
  const bldgPeak = (bldgOn && bldg?.boxes && bldg.boxes.length) ? (() => {
    let pk = { el: -999, az: 0 };
    for (const b of bldg.boxes) for (const [e, n] of b.ring) {
      const dist = Math.hypot(e, n); if (dist < 1) continue;
      const el = Math.atan2(b.h - camH - (dist * dist * 0.87) / (2 * 6371000), dist) * R2D;
      if (el > pk.el) pk = { el, az: ((Math.atan2(e, n) * R2D) + 360) % 360 };
    }
    return pk.el > -900 ? pk : null;
  })() : null;
  const bldgLbl = bldgPeak ? (() => {
    const p = project(bldgPeak.az, bldgPeak.el);
    return p.inFront && p.y > 0.04 && p.y < 0.96 ? p : null;
  })() : null;
  /* peaks projected into THIS view — the header count reflects what actually
     renders (peaks span all 360°, only those you're facing are on screen).
     peakMarks is prominence-ordered, so keep the tallest and drop any whose
     label would pile onto an already-kept one; cap the on-screen labels. */
  /* on-silhouette peaks currently in the view window (for the "N of M" count) */
  const peakInView = (() => {
    const iv = [];
    for (const pk of peakMarks) {
      const pr = project(pk.az, pk.elv);
      if (pr.inFront && pr.x > 0.01 && pr.x < 0.99 && pr.y > -0.02 && pr.y < 1.02) iv.push({ pk, pr });
    }
    return iv;
  })();
  /* label only the TOP FEW major (tallest) summits — a dense range would bury
     the view in names — tiered so the handful that show don't collide. */
  const peakDraw = (() => {
    const top = peakInView.slice().sort((a, b) => (b.pk.eleM || 0) - (a.pk.eleM || 0)).slice(0, 8);
    const kept = [];
    for (const c of top) {
      let row = 0;
      while (row < 3 && kept.some((k) => k.row === row && Math.abs(k.pr.x - c.pr.x) < 0.13)) row++;
      kept.push({ ...c, row: Math.min(row, 2) });
    }
    return kept;
  })();
  const horizonY = project(effAz, 0).y;
  /* WINDS ALOFT drawn IN the dome — each pressure level is a layer of drift
     arrows draped across the sky, world-anchored (az spokes every 30°) so you
     pan through them; the layers are spread EVENLY from just above the horizon
     up toward the zenith (higher altitude = higher up), a schematic stack since
     true height→elevation needs a range we don't have — each arrow still labels
     its real altitude. Arrow points the way that layer's wind pushes, projected
     into the view; colour = speed. Compare the object's motion to the layer near it. */
  const windDomeField = (windOn && windProf?.levels && !cameraOn && vp) ? (() => {
    const out = [], nL = windProf.levels.length;
    windProf.levels.forEach((L, li) => {
      // spread the layers EVENLY across the whole dome (surface→near-zenith) so
      // they never vanish above the aim point; heights are schematic anyway,
      // and each arrow carries its true altitude label.
      const elL = nL > 1 ? 7 + (li / (nL - 1)) * 76 : 45; // 7°..83°
      const col = windColor(L.speedMs);
      const beta = L.driftDeg * RAD, hE = Math.sin(beta), hN = Math.cos(beta);
      const kLen = clampN(L.speedMs / 16, 0.2, 1.1) * 0.16; // displacement ∝ speed
      let leftmost = null;
      for (let az = 0; az < 360; az += 30) {
        const u = dirOf(az, elL), b = projectD(u);
        if (!b.inFront || b.x < 0.04 || b.x > 0.96 || b.y < 0.03 || b.y > 0.97) continue;
        const v = [u[0] + kLen * hE, u[1] + kLen * hN, u[2]];
        const m = Math.hypot(v[0], v[1], v[2]), t = projectD([v[0] / m, v[1] / m, v[2] / m]);
        if (!t.inFront) continue;
        const dx = (t.x - b.x) * vp.w, dy = (t.y - b.y) * vp.h;
        out.push({ x: b.x, y: b.y, ang: Math.atan2(dy, dx) * R2D, len: clampN(Math.hypot(dx, dy), 13, 58), col });
        if (!leftmost || b.x < leftmost.x) leftmost = { x: b.x, y: b.y };
      }
      if (leftmost) out.push({ label: true, x: leftmost.x, y: leftmost.y, col, alt: L.levelM, spd: L.speedMs });
    });
    return out;
  })() : [];
  /* ☁ cloud layer — a grey SHADING of the sky (not literal cloud blobs), which
     read as fake next to a photo's real clouds. A deck seen from below fills the
     whole sky ABOVE THE HORIZON, so we wash the sky region of the dome grey with
     opacity ∝ Open-Meteo % cover (a hint of veil at low cover → solid overcast).
     Anchored to the horizon line (`horizonY`), so it stays tied to the sky's
     elevation as you pan, and clipped so it never greys the ground. */
  const cloudCover = (cloudOn && wxSky && !wxSky.err)
    ? (isNum(wxSky.cloud) ? +wxSky.cloud
      : Math.max(isNum(wxSky.low) ? wxSky.low : 0, isNum(wxSky.mid) ? wxSky.mid : 0, isNum(wxSky.high) ? wxSky.high : 0))
    : 0;
  // Opacity via a gamma curve (pow<1), not linear: over a saturated blue dome a
  // linear wash reads as faint haze until near-overcast. This lifts mid-cover so
  // 40-60 % already looks meaningfully grey, up to a near-solid overcast at 100 %.
  const cloudShade = cloudCover >= 1 ? clampN(Math.pow(cloudCover / 100, 0.62) * 0.92, 0.06, 0.92) : 0;  // peak grey opacity aloft
  const cloudSkyBot = clampN(horizonY, 0, 1);                 // sky occupies y ∈ [0, horizon]
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
  /* --- the pose IS the state; place-mode gestures mutate it directly.
     During stabilized playback, `playPose` (the current frame's solved pose)
     overrides for DISPLAY only — placement state stays untouched. --- */
  const poseNow = playPose || { az: pAz, el: pEl, roll: pRoll, fov: fovM };
  const poseK = playPose ? (playPose.k || 0) : pDist;

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
    const s = 1 + poseK * (x * x + y * y); // radial lens distortion (0 unless star-calibrated)
    return unit([f[0] + (r[0] * x + u[0] * y) * s, f[1] + (r[1] * x + u[1] * y) * s, f[2] + (r[2] * x + u[2] * y) * s]);
  };
  /* frame bookkeeping: the ALIGNMENT frame (what the placement pose
     describes; scrubbable in place mode) vs the MARKED frame (where the
     object was fitted). They default to the same frame; when decoupled, the
     marks' true pose comes from the solved camera path at their own time. */
  const alignT = isNum(source?.alignT) ? +source.alignT : (isNum(source?.A?.videoTime) ? +source.A.videoTime : 0);
  const markT = isNum(source?.A?.videoTime) ? +source.A.videoTime : alignT;
  /* MARKED-frame pixel → world dir. The object marks/track pixels live on the
     MARKED frame, so their sky position is fixed by that frame's pose. During
     stabilized playback pixDir follows the playing frame's pose (playPose) —
     using it for the marks would drag the object outline along with the frame.
     With align/marked frames DECOUPLED and a solved path available, the marks
     project through their own frame's solved pose; otherwise the placement
     pose stands (coupled = identical; pre-solve = best available). */
  const pixDirMarked = (px, py) => {
    if (!source?.natW) return pixDir(px, py);
    if (Array.isArray(source?.posePath) && source.posePath.length > 1 && Math.abs(markT - alignT) > 0.05) {
      const pp = posePathAt(source.posePath, markT);
      if (pp) return pixToDirK(px, py, source.natW, source.natH, pp.az, pp.el, pp.roll || 0, pp.fov, pp.k || 0);
    }
    return playPose ? pixToDirK(px, py, source.natW, source.natH, pAz, pEl, pRoll, fovM, pDist) : pixDir(px, py);
  };
  /* dense object-direction timeline for every FOLLOW consumer (the dome
     wireframe during playback, the close-up export camera): the auto-tracked
     objPath when it survived, otherwise the user's own TIMED WAYPOINTS
     converted through their frames' solved poses — so follow features work
     from manual taps alone (field: the close-up export sat still and the 3D
     wireframe froze at the marked spot when the auto-track was absent). */
  const followPath = useMemo(() => {
    const op = source?.objPath;
    if (Array.isArray(op) && op.length > 1) return op;
    const pp = source?.mediaKind === "video" && Array.isArray(source?.posePath) && source.posePath.length > 1 ? source.posePath : null;
    if (!pp || !source?.natW) return null;
    const pts = [...(source.track || [])].filter((q) => isNum(q.t) && isNum(q.x) && isNum(q.y)).sort((a, b) => a.t - b.t);
    if (pts.length < 2) return null;
    const out = [];
    for (const q of pts) {
      const ps = posePathAt(pp, +q.t);
      if (!ps || !isNum(ps.az)) continue;
      const ae = dirToAzEl(pixToDirK(+q.x, +q.y, source.natW, source.natH, ps.az, ps.el, ps.roll || 0, ps.fov, ps.k || 0));
      out.push({ t: +q.t, az: +ae.az.toFixed(3), el: +ae.el.toFixed(3) });
    }
    return out.length > 1 ? out : null;
  }, [source?.objPath, source?.posePath, source?.track, source?.natW, source?.natH, source?.mediaKind]);

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
    const vC = unproject(cx, cy);                          // crosshair world dir = object's apparent spot
    if (vC[0] * photo.f[0] + vC[1] * photo.f[1] + vC[2] * photo.f[2] <= 0.02) { setCalibMsg("Aim the crosshair onto the object in the photo first"); return; }
    const pix = dirToPixK(vC, photo.natW, photo.natH, pAz, pEl, pRoll, fovM, pDist); // the object's fixed pixel
    if (!pix) { setCalibMsg("Couldn't read that spot — re-aim the crosshair on the object"); return; }
    const g = dirOf(calibAnchor.az, calibAnchor.el);
    if (calibPrevRef.current == null) calibPrevRef.current = { az: pAz, el: pEl, fov: fovM, roll: pRoll, dist: pDist };
    const list = [...calibAnchorsRef.current, { px: pix.px, py: pix.py, g }];
    calibAnchorsRef.current = list;
    calibNamesRef.current = [...calibNamesRef.current, calibAnchor.name];
    const oldFov = fovM;
    const sol = solvePoseAnchors(list, photo.natW, photo.natH, pAz, pEl, { roll: pRoll, fov: fovM, k: pDist });
    calibRecRef.current = { method: "stars", mode: "tap", n: list.length, refs: [...calibNamesRef.current], rms: +sol.rms.toFixed(2), fov: +clampN(sol.fov, 8, 135).toFixed(1) };
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
    calibPrevRef.current = null; calibAnchorsRef.current = []; calibNamesRef.current = []; setCalibCount(0);
    calibRecRef.current = null; // reset alignment → back to whatever the placement is by hand
    setCalibApplied(false); setCalibAnchor(null); setCalibMsg("");
  };

  const enterPlace = () => {
    /* first-ever placement: put the photo where you're looking */
    if (!source?.mediaAim) { setPAz(viewAz); setPEl(clampN(viewAlt, -20, EL_MAX)); }
    if (motionOn) setMotionOn(false);
    if (calibOn) { setCalibOn(false); setCalibAnchor(null); setCalibMsg(""); }
    calibAnchorsRef.current = []; setCalibCount(0);   // manual place invalidates the star anchors
    resetPlaceView();
    setPlaceUndo([]); pendUndoRef.current = null; placeMovedRef.current = false; // fresh undo history
    setBandPx(null);                                  // re-measure the clear band for this session
    setPMode("place");
  };
  const donePlace = () => {
    /* hand the (already photo-centered) view back seamlessly — nothing moves.
       effFov is the ZOOMED view FOV; divide the display zoom back out so Look
       mode opens at the true photo framing, not magnified. */
    setViewAz(pAz); setViewAlt(clampN(pEl, -15, EL_MAX));
    setFov(clampN(2 * Math.atan(Math.tan((effFov * RAD) / 2) * pZoom) * R2D, 2, 90));
    resetPlaceView();
    commitPlacement();
    setPMode("look");
  };
  /* ——— sky-view MODES: place · trajectory · size · compare — one at a time.
     Each mode button toggles its own tools; selecting one clears the others. */
  const exitCalib = () => { if (calibOn) { setCalibOn(false); setCalibAnchor(null); setCalibMsg(""); } };
  const selectMode = (m) => {
    if (m === "place") {
      if (pMode === "place") { donePlace(); return; }   // toggle off
      setTrajOn(false); setSizeOn(false); setCmpOn(false); exitCalib();
      enterPlace();
      return;
    }
    if (pMode === "place") donePlace();                  // leaving place for a look-mode tool
    exitCalib();
    if (m === "traj") {
      /* opening the trajectory view snaps the dome back to the photo's FOV so
         the objects show at the SAME scale as the measure step — no leftover
         zoom making them look bigger here than where you set them */
      if (!trajOn) { const f = fitFovToPhoto(); if (f != null) setFov(f); }
      setTrajOn((v) => !v); setSizeOn(false); setCmpOn(false);
    }
    else if (m === "size") { setSizeOn((v) => !v); setTrajOn(false); setCmpOn(false); }
    else if (m === "compare") {
      setCmpOn((v) => { if (!v) setCmpPos({ az: viewAz, el: clampN(viewAlt, -10, 85) }); return !v; });
      setTrajOn(false); setSizeOn(false);
    }
  };
  /* manual star align lives under Place, but the aim needs the free-look
     crosshair (in place mode the crosshair is locked to the photo centre), so
     it commits the placement, drops to look centred on the photo, and opens the
     tap-a-star calibration — which solves the SAME placement pose. */
  const startManualAlign = () => {
    if (pMode === "place") donePlace();
    setTrajOn(false); setSizeOn(false); setCmpOn(false);
    setCalibOn(true); setCalibAnchor(null); setCalibMsg("👆 Tap a named star or planet to align to it");
  };

  /* --- Look-mode marks + visibility (the image itself is drawn by our own
         canvas mesh warp — no CSS matrix3d, nothing for Safari to reinterpret) --- */
  let photoMarks = null, photoHidden = false;
  if (photo && vp.w > 0) {
    const centerOK = projectD(photo.f).inFront;
    photoHidden = !placing && !centerOK;
    /* during playback the marks are pinned to the MARKED spot, which can be
       visible even when the playing frame has wandered off-view — don't gate
       them on the frame's center then (P() drops behind-camera points itself) */
    if (centerOK || playPose) {
      /* during stabilized playback, the object marks RIDE THE OBJECT TRACK:
         rotate their directions by the rotation carrying the marked object
         direction onto the tracked direction at the playing time, so the
         outline stays on the real (moving) object instead of freezing at
         the marked spot. Sight-line B and track points stay pinned — they
         are their own observations. */
      const objFollow = (() => {
        if (!playPose || !isNum(playPose.t)) return null;
        /* auto-track when it survived, else the tapped-waypoint sky path —
           the wireframe rides the trajectory either way */
        const op = followPath;
        if (!Array.isArray(op) || op.length < 2 || !source?.A?.p1 || !source?.A?.p2) return null;
        const t = +playPose.t;
        let lo = 0, hi = op.length - 1;
        if (t <= op[0].t) hi = 0; else if (t >= op[op.length - 1].t) lo = op.length - 1;
        else while (hi - lo > 1) { const m = (lo + hi) >> 1; if (op[m].t <= t) lo = m; else hi = m; }
        const a = op[lo], b = op[hi], u = hi === lo ? 0 : (t - a.t) / Math.max(1e-9, b.t - a.t);
        const dAzT = ((b.az - a.az + 540) % 360) - 180;
        const dT = dirFromAzEl(a.az + dAzT * u, a.el + (b.el - a.el) * u);
        const mid = { x: (source.A.p1.x + source.A.p2.x) / 2, y: (source.A.p1.y + source.A.p2.y) / 2 };
        const d0 = pixDirMarked(mid.x, mid.y);
        const ax = [d0[1] * dT[2] - d0[2] * dT[1], d0[2] * dT[0] - d0[0] * dT[2], d0[0] * dT[1] - d0[1] * dT[0]];
        const s = Math.hypot(ax[0], ax[1], ax[2]), c = clampN(dot(d0, dT), -1, 1);
        if (s < 1e-9) return null;
        const k = [ax[0] / s, ax[1] / s, ax[2] / s];
        return (v2) => {                                   // Rodrigues rotation d0 → dT
          const kv = [k[1] * v2[2] - k[2] * v2[1], k[2] * v2[0] - k[0] * v2[2], k[0] * v2[1] - k[1] * v2[0]];
          const kd = dot(k, v2);
          return [v2[0] * c + kv[0] * s + k[0] * kd * (1 - c), v2[1] * c + kv[1] * s + k[1] * kd * (1 - c), v2[2] * c + kv[2] * s + k[2] * kd * (1 - c)];
        };
      })();
      const PF = (pt) => { const d = pixDirMarked(pt.x, pt.y); const pr = projectD(objFollow ? objFollow(d) : d); return pr.inFront ? [pr.x * 100, pr.y * 100] : null; };
      /* the fitted 3D WIREFRAME rides the same pipeline as the marks: each
         curve point (native px on the marked frame) → world dir under the
         placement pose → rotated onto the tracked dir during playback */
      let wire = null;
      /* NOT while the Trajectory tool is open: its numbered points already draw
         the object at every point, so the standalone fitted-object wireframe was
         an EXTRA saucer floating at the measured spot (field report). */
      if (objOn && source.shapeFit && !trajOn) {
        /* KEYFRAMED size + attitude: during playback interpolate the size/
           rotation the user marked along the track at the playing time; on the
           static marked frame use its own time. So the model breathes and
           tumbles between keyframes instead of holding one fitted pose. */
        const wireT = playPose && isNum(playPose.t) ? +playPose.t : markT;
        const sfNow = shapeAt(source.shapeFit, source.track, wireT, markT);
        wire = shapeProjNat(sfNow).curves.map((c) => {
          const segs = []; let cur = [];
          for (const pt of c) { const q = PF(pt); if (q) cur.push(q); else if (cur.length > 1) { segs.push(cur); cur = []; } else cur = []; }
          if (cur.length > 1) segs.push(cur);
          return segs;
        }).flat();
        /* TRUE SIZE, always: the wireframe renders at exactly the angular size
           fitted on the measure step and scales with the dome zoom through the
           projection — no magnification floor (tried once; the user wants the
           set size honoured — a distant object really is tiny, zoom in to see
           it, same doctrine as the trajectory chips). */
        if (!wire.length) wire = null;
      }
      /* look-mode overlay carries ONLY the object (wireframe, or the two mark
         rings as a fallback when no shape is fitted) — track dots and the B
         point cluttered the world view and were dropped by request */
      photoMarks = {
        a1: objOn && !wire && !trajOn && source.A?.p1 ? PF(source.A.p1) : null,
        a2: objOn && !wire && !trajOn && source.A?.p2 ? PF(source.A.p2) : null,
        wire,
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
    const patch = { mediaAim: { az: +pAz.toFixed(2), el: +pEl.toFixed(2), roll: +pRoll.toFixed(1), dist: +pDist.toFixed(5) }, fovH: +fovM.toFixed(1), placed: true, calib: { ...(calibRecRef.current || { method: "manual" }), vt: source?.mediaKind === "video" ? +alignT.toFixed(3) : null } };
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
    /* RE-ANCHOR the solved paths: the stabilized camera path + object track
       were solved RELATIVE to the previous placement of the alignment frame.
       A re-align (drag, snap-to-ridges, star-align) rotates the whole world
       solution by exactly the old→new placement rotation — rotate the stored
       paths along, so the trajectory, wireframe-follow and close-up export
       keep matching the fresh calibration instead of silently going stale
       (field: "the trajectory still seems off" after re-aligning a
       stabilized clip). Rotation only — a big FOV recalibration still
       warrants re-running the stabilization. */
    const oldAim = source?.mediaAim;
    if (oldAim && isNum(oldAim.az) && Array.isArray(source?.posePath) && source.posePath.length) {
      const from = { az: +oldAim.az, el: isNum(oldAim.el) ? +oldAim.el : 0, roll: isNum(oldAim.roll) ? +oldAim.roll : 0 };
      const to = { az: pAz, el: pEl, roll: pRoll };
      const dAz = Math.abs(((to.az - from.az + 540) % 360) - 180);
      if (dAz > 0.02 || Math.abs(to.el - from.el) > 0.02 || Math.abs(to.roll - from.roll) > 0.02) {
        const rotP = (arr) => arr.map((p) => { const q = reanchorPose(p, from, to); return { ...p, az: +q.az.toFixed(3), el: +q.el.toFixed(3), roll: +q.roll.toFixed(2) }; });
        const rotO = (arr) => arr.map((p) => { const q = reanchorAzEl(+p.az, +p.el, from, to); return { ...p, az: +q.az.toFixed(3), el: +q.el.toFixed(3) }; });
        patch.posePath = rotP(source.posePath);
        if (Array.isArray(source.posePathRaw) && source.posePathRaw.length) patch.posePathRaw = rotP(source.posePathRaw); // raws ride along, or a later re-smooth resurrects the old anchor
        if (Array.isArray(source.objPath) && source.objPath.length) patch.objPath = rotO(source.objPath);
        if (Array.isArray(source.objPathRaw) && source.objPathRaw.length) patch.objPathRaw = rotO(source.objPathRaw);
        if (Array.isArray(source.poseFixes) && source.poseFixes.length) patch.poseFixes = rotP(source.poseFixes); // ⚓ anchors are absolute world poses — they rotate with the solution
      }
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
      calibRecRef.current = { method: "terrain", n: m.n, rms: +m.rms.toFixed(2) };
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
      await new Promise((r) => setTimeout(r, 20)); // let the flash paint before the solve blocks
      let sol = null;
      /* BEST path when the FOV is known (EXIF) and we have an elevation prior:
         lock FOV + elevation, scan the ROTATION against a DEEP catalog. This is
         what makes a wide "straight-up" frame solvable regardless of rotation. */
      if (isNum(source?.fovH) && elPrior != null) {
        const deep = DEEP_STARS.map(([ra, dec, mag]) => { const p = raDecToAzEl(ra, dec, T, LAT, LNG); return { g: dirOf(p.az, p.alt), mag, alt: p.alt }; }).filter((c) => c.alt > 0);
        sol = gridStarAlign(det, deep, source.natW, source.natH, { fov: +source.fovH, elPrior, elBand: 10, minGrid: 12, minMatch: 14, maxRms: 0.6 });
      }
      /* fallbacks: seedless asterism (no FOV needed), then a seeded refine */
      if (!sol) sol = blindStarAlign(det, cat, source.natW, source.natH, fovGuess, { minInl: 8, minMatch: 10, maxRms: 0.6, fovFactors: [0.5, 0.65, 0.8, 1.0, 1.2, 1.4], elPrior, elBand: 20 });
      if (!sol) sol = autoStarAlign(det, cat, source.natW, source.natH, { az: pAz, el: pEl, roll: pRoll, fov: fovM, k: pDist }, { minMatch: 10, maxRms: 0.6 });
      if (!sol) { setFlash(`✦ couldn't confidently solve this frame (${det.length} points). Wide/soft/hazy night shots are hard to solve blind — tap ✦ align-to-star to set 2–3 named stars yourself, and raise brightness on the photo step to see them.`); return; }
      calibAnchorsRef.current = []; calibNamesRef.current = [];
      setPAz(sol.az); setPEl(clampN(sol.el, -20, EL_MAX));
      setPRoll(clampN(((sol.roll + 180) % 360 + 360) % 360 - 180, -90, 90));
      setFovM(clampN(sol.fov, 8, 135)); setPDist(sol.k);
      /* be honest about confidence — a wide phone lens + a bright-only catalog
         can yield a LOOSE partial fit; don't present that as a clean lock */
      const loose = sol.rms > 0.7 || sol.n < 12;
      calibRecRef.current = { method: "stars", mode: "auto", n: sol.n, rms: +sol.rms.toFixed(2), fov: +clampN(sol.fov, 8, 135).toFixed(1), loose };
      setFlash(loose
        ? `✦ matched ${sol.n} stars, but the fit is loose (±${sol.rms.toFixed(1)}°) — toggle ★ stars ON and check they sit on the photo's stars; nudge/retry if it's off`
        : `✦ auto-aligned · ${sol.n} stars · fit ±${sol.rms.toFixed(2)}° · FOV ${sol.fov.toFixed(0)}° — toggle ★ stars to verify`);
    } catch (e) { setFlash("✦ auto-align failed on this image"); }
  };

  /* 🎞 STABILIZE — reference-locked video (video roadmap phase 2, rung B).
     The aligned marked frame's pose turns every background pixel into a FIXED
     world direction, so every other frame is the star-align problem: track the
     background features frame-to-adjacent-frame (src/video/postrack.js), solve
     each frame's pose with the moving object trimmed as an outlier, and store
     the pose path on the source. Walks OUTWARD from the marked frame (forward
     then backward) so every step is between adjacent frames. All seeks happen
     on an offscreen <video>, off the render path. */
  /* MANUAL stabilization: solve the pose path from the hand-marked camera
     references (source.camRefs) instead of the automatic feature walk — the
     fallback for clips too dark/soft/empty for the auto pass. Pure + instant
     (no video seeking); writes the same posePath everything downstream reads. */
  const solveFromMarks = (smoothPct) => {
    if (source?.mediaKind !== "video" || !source?.natW) { setFlash("🎯 solve from marks needs a video"); return; }
    const refs = source?.camRefs || [];
    const nMarks = refs.reduce((a, r) => a + (r.marks || []).filter((m) => isNum(m.x)).length, 0);
    if (nMarks < 2) { setFlash("🎯 mark a fixed feature on ≥2 frames first (Cam refs, on the measure step)"); return; }
    const refPose = { t: alignT, az: pAz, el: pEl, roll: pRoll, fov: fovM, k: pDist };
    const smoothAmt = isNum(smoothPct) ? clampN(smoothPct / 100, 0, 1) : 0.4;
    const path = solveManualPoses(refs, refPose, { natW: source.natW, natH: source.natH }, { smoothAmt });
    if (!path || !path.length) { setFlash("🎯 couldn't solve — mark more features / more frames"); return; }
    if (update) update({ posePath: path });
    setFlash(`🎯 solved ${path.length} keyframe${path.length === 1 ? "" : "s"} from your marks · smoothing ${Math.round(smoothAmt * 100)}% — ▶ play to check`);
  };
  /* UNDO A STABILIZE RUN. The walk is destructive in five directions at once:
     it overwrites posePath AND posePathRaw, replaces the object track and its
     raw, clears EVERY ⚓ anchor, and can rewrite the keyframed track. A run that
     makes things worse — a clip the tracker cannot hold, or a sensor capture
     whose object pass latches onto the wrong thing — used to be unrecoverable,
     with no way back to footage that was fine before you asked.

     So snapshot what the run is about to overwrite, and keep ONE level: the
     semantic that matches "the thing I just did was wrong". Undoing restores
     the snapshot exactly, anchors included.

     With no snapshot (a clip stabilized before this existed) undo still has
     somewhere useful to go — it clears the solve, and since preview playback
     no longer needs a posePath, that lands you on the original clip rather
     than a dead screen. */
  const preStabSnap = () => ({
    posePath: source?.posePath || null, posePathRaw: source?.posePathRaw || null,
    objPath: source?.objPath || null, objPathRaw: source?.objPathRaw || null,
    poseFixes: source?.poseFixes || null, sensorSync: source?.sensorSync || null,
    track: Array.isArray(source?.track) && source.track.length ? source.track : null,
  });
  const undoStabilize = () => {
    const p = source?.preStab;
    playingRef.current = false; setPlaying(false);
    setPlayPose(null); setPlayIdx(0);
    setFixOn(false); setSmoothOpen(false); setExportMenu(false);
    mediaDel(source.id + ":stab");   // the exported render belongs to the solve being discarded
    if (p) {
      update({
        posePath: p.posePath || null, posePathRaw: p.posePathRaw || null,
        objPath: p.objPath || null, objPathRaw: p.objPathRaw || null,
        poseFixes: p.poseFixes || null, sensorSync: p.sensorSync || null,
        ...(p.track ? { track: p.track } : {}), preStab: null,
      });
      setFlash(p.posePath && p.posePath.length > 1 ? "↶ back to the previous stabilize" : "↶ stabilization removed — the original clip is back");
    } else {
      update({ posePath: null, posePathRaw: null, objPath: null, objPathRaw: null, poseFixes: null, preStab: null });
      setFlash("↶ stabilization removed — the original clip is back");
    }
  };
  const stabilize = async () => {
    if (source?.mediaKind !== "video" || !source?.mediaUrl || !source?.natW) { setFlash("🎞 stabilize needs a video with a marked frame"); return; }
    const preStab = preStabSnap();
    const run = ++stabAbortRef.current;
    const refPose = { az: pAz, el: pEl, roll: pRoll, fov: fovM, k: pDist };
    /* the walk anchors on the ALIGNMENT frame (what the placement pose
       describes) — which may differ from the frame the object was marked on */
    const refT = alignT;
    setStabBusy(1); setFlash("🎞 loading video…");
    const v = document.createElement("video");
    v.muted = true; v.playsInline = true; v.preload = "auto";
    try {
      await new Promise((res, rej) => { v.onloadeddata = res; v.onerror = rej; v.src = source.mediaUrl; });
      const dur = v.duration || 0;
      /* watchdogged seek: Safari can skip `seeked` for a same-time seek (hence
         the nudge) and a stalled decoder must never hang the whole solve */
      const seek = (t) => new Promise((res) => {
        const tt = clampN(t, 0, Math.max(0, dur - 0.01));
        let fired = false;
        const wd = setTimeout(() => { if (!fired) { fired = true; res(); } }, 4000);
        v.onseeked = () => { if (!fired) { fired = true; clearTimeout(wd); res(); } };
        try { v.currentTime = tt + (Math.abs((v.currentTime || 0) - tt) < 0.001 ? 0.0001 : 0); }
        catch (e) { if (!fired) { fired = true; clearTimeout(wd); res(); } }
      });
      /* sample cadence: every 0.25 s, capped ~140 samples on long clips — small
         inter-frame rotation keeps the NCC search tight, and pose is angular so
         the path interpolates cleanly between samples */
      const dt = Math.max(0.25, dur / 140);
      const times = []; for (let t = 0; t <= dur - 0.005; t += dt) times.push(+t.toFixed(3));
      /* track at ≤768 px — pose is resolution-independent (FOV normalizes it),
         so the low-res path applies verbatim to the 1600 px playback texture */
      const TW = Math.min(768, source.natW), sc = TW / source.natW, TH = Math.max(40, Math.round(source.natH * sc));
      const cv = document.createElement("canvas"); cv.width = TW; cv.height = TH;
      const cx = cv.getContext("2d", { willReadFrequently: true });
      const grab = () => { cx.drawImage(v, 0, 0, TW, TH); return cx.getImageData(0, 0, TW, TH).data; };
      /* OBJECT track rides the same walk: the camera poses being solved
         anyway turn the marked object's template match into a true angular
         path (see stepObject). The object gets its OWN buffers at native
         resolution (≤1600, the playback-texture cap): at the camera solve's
         768 px + video compression, a small object can wash out to a few
         grey levels of contrast and the matcher has nothing to hold
         (ground-truth e2e: a 12 px dot read 192 against 184 grass). */
      const objMid = source?.A?.p1 && source?.A?.p2
        ? { x: (source.A.p1.x + source.A.p2.x) / 2, y: (source.A.p1.y + source.A.p2.y) / 2 } : null;
      const OW = Math.min(1600, source.natW), osc = OW / source.natW, OH = Math.max(40, Math.round(source.natH * osc));
      const cvO = document.createElement("canvas"); cvO.width = OW; cvO.height = OH;
      const cxO = cvO.getContext("2d", { willReadFrequently: true });
      const grabO = () => { cxO.drawImage(v, 0, 0, OW, OH); return cxO.getImageData(0, 0, OW, OH).data; };
      const objPxO = objMid ? Math.hypot(source.A.p1.x - source.A.p2.x, source.A.p1.y - source.A.p2.y) * osc : 0;
      await seek(refT);
      const refData = grab();
      /* physical FOV cap: no frame can be WIDER than the lens's widest —
         digital zoom only narrows. This kills the impossible 110°+ solves the
         smallest-template ladder rungs win at the zoom-out landing. With lens
         metadata the cap is tight (6% margin so it never fights an honest
         near-wide solve); without it the marked frame might itself be zoomed,
         so only a generous 30% margin is safe. */
      const lensFov = isNum(source?.meta?.fovH) ? +source.meta.fovH : null;
      const fovMax = lensFov ? Math.max(fovM, lensFov) * 1.06
        : Math.max(fovM, isNum(source?.fovH) ? +source.fovH : 0) * 1.3;
      const opts = { minMatch: 6, maxN: 50, patch: 13, search: 16, fovMax };
      const mkTracker = () => initTracker(refData, TW, TH, source.natW, source.natH, refPose, opts);
      const t0 = mkTracker();
      if (t0.features.length < 8) { setStabBusy(0); setFlash(`🎞 only ${t0.features.length} background feature(s) on the marked frame — too few to track. A frame with skyline/terrain edges or stars stabilizes best.`); return; }
      const fwd = times.filter((t) => t > refT + 1e-6);
      const bwd = times.filter((t) => t < refT - 1e-6).reverse();
      const entry = (t2, r2) => ({ t: t2, az: +r2.pose.az.toFixed(3), el: +r2.pose.el.toFixed(3), roll: +r2.pose.roll.toFixed(3), fov: +r2.pose.fov.toFixed(2), k: +(r2.pose.k || 0).toFixed(5), n: r2.nInliers, h: r2.held ? 1 : 0 });
      const path = [{ t: +refT.toFixed(3), az: +refPose.az.toFixed(3), el: +refPose.el.toFixed(3), roll: +refPose.roll.toFixed(3), fov: +refPose.fov.toFixed(2), k: +(refPose.k || 0).toFixed(5), n: t0.features.length }];
      let done = 0, ancCount = 0;
      const total = (fwd.length + bwd.length) * (objMid ? 2 : 1);
      setStabTotal(total);
      const walk = async (list, tracker) => {
        let prevT = refT;
        /* "meet in the middle": when a step RE-ANCHORS absolutely against the
           reference frame, distribute its drift correction back across this
           pass's entries since the last anchor, so the incremental chain bends
           smoothly onto the fix instead of snapping (smearDrift). */
        let segFrom = path.length, segT = refT;
        for (const t of list) {
          if (stabAbortRef.current !== run) return false;
          /* snapshot so a failed step can rewind and BISECT in time — a fast
             zoom/whip-pan at the 0.25 s cadence can outrun even the zoom
             rescue; halving the gap halves the per-step change exactly where
             the motion is fastest. Two levels → down to ~0.06 s. */
          const snap = () => ({ prevData: tracker.prevData, lastPose: tracker.lastPose, features: tracker.features, nextId: tracker.nextId });
          const stepAt = async (tt) => { await seek(tt); return stepTracker(tracker, grab()); };
          const tryStep = async (tFrom, tTo, depth) => { // steps the tracker onto tTo (either direction); returns tTo's result
            const s0 = snap();
            let r = await stepAt(tTo);
            if (r.nInliers < 6 && depth > 0 && Math.abs(tTo - tFrom) > 0.09) {
              Object.assign(tracker, s0);               // rewind — take it in two halves
              const tm = +((tFrom + tTo) / 2).toFixed(3);
              const rm = await tryStep(tFrom, tm, depth - 1);
              path.push(entry(tm, rm));
              r = await tryStep(tm, tTo, depth - 1);
            }
            return r;
          };
          const r = await tryStep(prevT, t, 2);
          path.push(entry(t, r));
          if (r.anchored) {
            const k = path.length - 1;
            path[k].anc = 1; ancCount++;
            if (r.drift && (Math.abs(r.drift.dAz) > 0.02 || Math.abs(r.drift.dEl) > 0.02 || Math.abs(r.drift.dRoll) > 0.05 || Math.abs(r.drift.dFov) > 0.1)) smearDrift(path, segFrom, k, segT, r.drift);
            segFrom = k + 1; segT = path[k].t;
          }
          prevT = t;
          done++;
          if (done % 3 === 0) { setStabBusy(done); await new Promise((r2) => setTimeout(r2, 0)); } // progress lives in the button (the flash auto-hides and would blink); yield so it paints
        }
        return true;
      };
      const okF = await walk(fwd, t0);
      let okB = true;
      if (okF && bwd.length) { await seek(refT); okB = await walk(bwd, mkTracker()); }
      if (!okF || !okB || stabAbortRef.current !== run) { setStabBusy(0); setStabTotal(0); setFlash("🎞 stabilization cancelled"); return; }
      path.sort((a, b) => a.t - b.t);
      /* BRIDGE short held runs: a frame that neither solved nor globally
         locked carries the PREVIOUS frame's pose, frozen — a repeat, so
         despike can never repair it (its deviation from interpolation is
         always smaller than the neighbours' own disagreement). Across a
         SHORT gap (≤0.55 s: one held sample, or bisected fragments) the
         time-interpolated pose beats the freeze+snap, so drop those and let
         posePathAt bridge. LONG held runs stay frozen: interpolating a
         1 s+ gap fabricates motion (verified on the real clip — it would
         ramp a zoom in 0.75 s early, up to 26° of invented FOV), and runs
         touching either end of the path have nothing to bridge to. This is
         also the honest answer to "run it again": a blind re-run is
         deterministic and returns the identical path. */
      const drop = new Set();
      for (let i2 = 1; i2 < path.length - 1; i2++) {
        if (!path[i2].h) continue;
        let j2 = i2; while (j2 < path.length && path[j2].h) j2++;
        if (j2 < path.length && (path[j2].t - path[i2 - 1].t) <= 0.55) for (let k2 = i2; k2 < j2; k2++) drop.add(k2);
        i2 = j2 - 1;
      }
      const bridged = drop.size;
      if (bridged) { const kept2 = path.filter((_, i2) => !drop.has(i2)); path.length = 0; path.push(...kept2); }
      path.forEach((p) => delete p.h);
      /* single blurred frames can solve a hair (or wildly) off and read as a
         brief "jump out of lock" in playback — despike against neighbours
         (real motion is a ramp across samples and is preserved) */
      const deglitched = despikePath(path);
      /* keep the RAW (despiked, unsmoothed) path so the steadiness slider can
         re-derive the smoothed path non-destructively at any strength later */
      const pathRaw = path.map((p) => ({ ...p }));
      /* then damp sub-degree solve noise (background jitter in the render):
         evidence-weighted pull toward the neighbours — strong solves barely
         move, weak ones lean on the interpolation. Real motion is a ramp
         across samples and passes through. Strength = the source's saved
         slider value (0.25 default = the historical fixed behaviour). */
      smoothPathAt(path, isNum(source?.smoothCam) ? +source.smoothCam : 0.25);
      /* honesty: frames with too few background references held the previous
         pose instead of fabricating a lock — say so when it's a lot of them */
      const weak = path.filter((p) => p.n < 6).length;
      /* ---- OBJECT PASS (second pass, after the camera path is final) ----
         The camera poses are now despiked+smoothed, so every conversion uses
         the best estimates — and every measure-step TRACK point ({t,x,y}:
         the object tapped on ITS OWN frame) converts through its own frame's
         solved pose, so waypoints marked on different frames stay mutually
         consistent. When ≥2 timed waypoints exist they become the HYBRID
         GUIDE: the human's trajectory owns the prediction and the pixel
         matcher only fine-tunes around it (stepObject opts.guide). */
      const objPath = [];
      let objOk = 0, objMiss = 0, guideN = 0, guides = [];
      if (objMid) {
        /* the object seed lives on the MARKED frame (which may differ from
           the alignment frame): its pose comes from the just-solved camera
           path at that time, its template from that frame's buffer, and the
           seed is SNAPPED onto the object — a mark a few px off a small
           object leaves a half-background template, lost immediately */
        await seek(markT);
        const seedO = grabO();
        const seedPose = posePathAt(path, markT);
        const sn = snapToObject(seedO, OW, OH, objMid.x * osc, objMid.y * osc, Math.min(16, Math.max(6, objPxO * 0.35)));
        const objSeed = {
          tx: sn.x, ty: sn.y,
          g: pixToDirK(sn.x / osc, sn.y / osc, source.natW, source.natH, seedPose.az, seedPose.el, seedPose.roll || 0, seedPose.fov, seedPose.k || 0),
        };
        const ae0 = dirToAzEl(objSeed.g);
        objPath.push({ t: +markT.toFixed(3), az: +ae0.az.toFixed(3), el: +ae0.el.toFixed(3), q: 1 });
        guides = (source.track || [])
          .filter((p) => isNum(p.t) && isNum(p.x) && isNum(p.y))
          .map((p) => {
            const ps = posePathAt(path, +p.t);
            return { t: +p.t, g: pixToDirK(+p.x, +p.y, source.natW, source.natH, ps.az, ps.el, ps.roll || 0, ps.fov, ps.k || 0) };
          })
          .sort((a, b) => a.t - b.t);
        guideN = guides.length;
        const guideAt = (tt) => {
          if (guides.length < 2) return null;
          if (tt <= guides[0].t - 1 || tt >= guides[guides.length - 1].t + 1) return null; // beyond the drawn path (+1 s grace) the matcher is on its own
          let lo = 0, hi = guides.length - 1;
          if (tt <= guides[0].t) hi = 0; else if (tt >= guides[guides.length - 1].t) lo = guides.length - 1;
          else while (hi - lo > 1) { const m = (lo + hi) >> 1; if (guides[m].t <= tt) lo = m; else hi = m; }
          const a = guides[lo], b = guides[hi], u = hi === lo ? 0 : (tt - a.t) / Math.max(1e-9, b.t - a.t);
          return unit([a.g[0] + (b.g[0] - a.g[0]) * u, a.g[1] + (b.g[1] - a.g[1]) * u, a.g[2] + (b.g[2] - a.g[2]) * u]);
        };
        const fwdO = times.filter((t) => t > markT + 1e-6);
        const bwdO = times.filter((t) => t < markT - 1e-6).reverse();
        for (const list of [fwdO, bwdO]) {
          if (stabAbortRef.current !== run) break;
          if (!list.length) continue;
          await seek(markT);
          let prevO = seedO;
          let st2 = { ...objSeed };
          for (const tt of list) {
            if (stabAbortRef.current !== run) break;
            await seek(tt);
            const bufO = grabO();
            const ps = posePathAt(path, tt);
            const gd = guideAt(tt);
            /* guide gate scales with the frame's FOV: a fixed 2° is half a
               deep-zoom frame — a background latch could sit visibly off the
               object while still "within the gate" (field: wireframe off the
               object at the zoomed clip end) */
            const o = stepObject(prevO, bufO, OW, OH, st2, ps, { natW: source.natW, natH: source.natH, objPx: objPxO, guide: gd, guideGate: clampN(ps.fov * 0.05, 0.6, 2), seed: { data: seedO, tx: objSeed.tx, ty: objSeed.ty } });
            prevO = bufO;
            st2 = { tx: o.tx, ty: o.ty, g: o.g, gPrev: o.gPrev };
            const ae = dirToAzEl(o.g);
            objPath.push({ t: tt, az: +ae.az.toFixed(3), el: +ae.el.toFixed(3), q: o.ok ? +Math.max(0.01, o.ncc).toFixed(2) : (gd ? 0.25 : 0) });
            if (o.ok) objOk++; else objMiss++;
            done++;
            if (done % 3 === 0) { setStabBusy(done); await new Promise((r2) => setTimeout(r2, 0)); }
          }
        }
        if (stabAbortRef.current !== run) { setStabBusy(0); setStabTotal(0); setFlash("🎞 stabilization cancelled"); return; }
      }
      /* keep the track only when the template held on for a usable fraction —
         with a manual guide the human's path stands even where pixels failed */
      objPath.sort((a, b) => a.t - b.t);
      /* smooth the object track: the per-frame matcher jitters and
         occasionally latches a background lookalike. DESPIKE (outlier
         rejection) always runs — a single-frame latch is a tracker error, not
         a maneuver — then keep the RAW despiked track and apply the source's
         saved SMOOTHING STRENGTH (the track-smoothing slider: 0 preserves
         hard corners, high strength reads an airplane as its clean curve). */
      const objSpiked = smoothObjPath(objPath, { passes: 0 });
      const objRaw = objPath.map((p) => ({ ...p }));
      smoothObjPathAt(objPath, isNum(source?.smoothObj) ? +source.smoothObj : 0.25, { despiked: true });
      /* WAYPOINTS ARE GROUND TRUTH: the witness tapped the object on those
         frames, so the final track must pass exactly through them — matcher
         drift/misses and smoothing pull are corrected by an interpolated
         delta field (snapDirsToAnchors), keeping the matcher's detail
         between taps. This is what keeps the playback wireframe ON the
         object wherever the user actually marked it. */
      if (guides.length && objPath.length > 1) {
        const anchors = guides.map((g2) => { const ae2 = dirToAzEl(g2.g); return { t: g2.t, az: ae2.az, el: ae2.el }; });
        const snapped = snapDirsToAnchors(objPath, anchors);
        objPath.length = 0; objPath.push(...snapped);
      }
      const objGood = objMid && (guideN >= 2 || objOk >= Math.max(4, (objOk + objMiss) * 0.3));
      /* per-frame SIZED track points (wpx from the measure step) get their
         angular size re-derived from each frame's SOLVED FOV — sized under a
         zoom, the constant-fovH conversion would read lens zoom as approach */
      let resized = 0;
      const track2 = Array.isArray(source?.track) && source.track.some((p) => isNum(p.wpx) && isNum(p.t))
        ? source.track.map((p) => {
          if (!isNum(p.wpx) || !isNum(p.t)) return p;
          const pp = posePathAt(path, +p.t);
          if (!pp || !isNum(pp.fov)) return p;
          const fpxT = (source.natW / 2) / Math.tan((pp.fov * D2R) / 2);
          resized++;
          return { ...p, ang: +(2 * Math.atan((+p.wpx / 2) / fpxT) * R2D).toFixed(5) };
        })
        : null;
      /* INSTRUMENTED CLIP: recover the log's clock offset against THIS solve
         (the gap between "start recording" and the first encoded frame is
         device-specific), then fuse — vision keeps the absolute frame, the
         sensors carry the frames vision couldn't hold. Sync compares only the
         SHAPE of the motion, so the compass bias can't drag the placement. */
      let sensorNote = "", sensorSync = null;
      if (Array.isArray(source?.sensorPath) && source.sensorPath.length > 4) {
        const log = source.sensorPath;
        /* DID THE VISION ACTUALLY TRACK? Inlier count says nothing about that —
           a tracker that loses the scene and re-acquires on whatever drifted in
           reports a confident solve that barely moves. Field case: the phone
           swept 95° (gravity agrees: 27° of elevation with it) while the solve
           reported 11.5° at 34-46 inliers, so every frame passed as "strong"
           and the sensors were never consulted. Gravity can't be wrong about
           that, so compare how far each source says the camera travelled. */
        const dis = motionDisagreement(path, log, 0);
        if (dis && dis.sen > 15 && dis.ratio < 0.45) {
          /* the log is the better witness for MOTION — but the absolute frame
             still comes from the placement, so the compass bias cancels */
          const anchor = { t: refT, az: refPose.az, el: refPose.el, roll: refPose.roll, fov: refPose.fov, k: refPose.k || 0 };
          const only = sensorOnlyPath(log, path.map((p) => p.t), anchor, { offset: 0 });
          if (only) {
            for (let i2 = 0; i2 < path.length; i2++) path[i2] = { ...path[i2], ...only[i2] };
            sensorSync = { offset: 0, mode: "sensor", vis: dis.vis, sen: dis.sen };
            sensorNote = ` · ⚠ the tracker lost the scene (it followed ${dis.vis.toFixed(0)}° while the phone turned ${dis.sen.toFixed(0)}°) — using the RECORDED MOTION instead, anchored to your alignment`;
          }
        } else {
          sensorSync = syncSensor(log, path, { range: 2, step: 0.02 });
          /* a railed or hopeless sync is worse than none — don't fuse on it */
          const bad = !sensorSync || sensorSync.conf < 0.45 || Math.abs(sensorSync.offset) > 1.9;
          if (bad) { sensorNote = " · motion log couldn't be matched to the clip — visual solve used alone"; sensorSync = null; }
          else {
            const fused = fuseSensorVisual(path, log, { offset: sensorSync.offset });
            const st = fuseStats(fused);
            for (let i2 = 0; i2 < path.length; i2++) path[i2] = fused[i2];
            const carried = (st.s || 0) + (st.b || 0);
            sensorNote = ` · motion log synced ${sensorSync.offset >= 0 ? "+" : ""}${sensorSync.offset.toFixed(2)}s (${Math.round(sensorSync.conf * 100)}% confident)${carried ? `, ${carried} weak frame${carried > 1 ? "s" : ""} carried on the phone's own attitude` : ""}${st.h ? `, ${st.h} still held` : ""}`;
          }
        }
      }
      /* poseFixes cleared: ⚓ anchors were corrections OF THE OLD SOLVE — carrying
         them onto a fresh solve would re-apply stale deltas to good frames */
      if (update) update({ posePath: path, posePathRaw: pathRaw, objPath: objGood ? objPath : null, objPathRaw: objGood ? objRaw : null, poseFixes: null, preStab, ...(sensorSync ? { sensorSync } : {}), ...(track2 ? { track: track2 } : {}) });
      mediaDel(source.id + ":stab");   // any previously exported render is stale under the new path
      setStabBusy(0); setStabTotal(0);
      const fovs = path.map((p) => p.fov), fovLo = Math.min(...fovs), fovHi = Math.max(...fovs);
      const zoomNote = fovHi - fovLo > 3 ? ` · zoom tracked (FOV ${fovHi.toFixed(0)}°→${fovLo.toFixed(0)}°)` : "";
      const ancNote = ancCount ? ` · ${ancCount} drift anchors` : "";
      const glitchNote = deglitched ? ` · ${deglitched} glitch${deglitched > 1 ? "es" : ""} smoothed` : "";
      const bridgeNote = bridged ? ` · ${bridged} weak frame${bridged > 1 ? "s" : ""} bridged` : "";
      const sizeNote = resized ? ` · ${resized} sized point${resized > 1 ? "s" : ""} re-scaled to the solved zoom` : "";
      const guideNote = guideN >= 2 ? `, guided by your ${guideN} track points` : "";
      const objSmoothNote = objGood && objSpiked ? `, ${objSpiked} jump${objSpiked > 1 ? "s" : ""} smoothed` : "";
      const objNote = objMid ? (objGood ? ` · object tracked (${objOk}/${objOk + objMiss} frames${guideNote}${objSmoothNote})` : ` · object lost (${objOk}/${objOk + objMiss} matched — outline stays at the marked spot; tip: mark a few Track points on the measure step and re-stabilize for a guided track)`) : "";
      setFlash(weak > path.length * 0.25
        ? `🎞 solved ${path.length} frames, but ${weak} had too few background references (pose held) — expect drift there. Play it with ▶ in look mode.`
        : `🎞 stabilized: ${path.length} frames solved${weak ? ` (${weak} held)` : ""}${zoomNote}${ancNote}${glitchNote}${bridgeNote}${sizeNote}${sensorNote}${objNote}. ▶ play in look mode — the sky stays locked, the frame moves.`);
    } catch (e) { setStabBusy(0); setStabTotal(0); setFlash("🎞 stabilization failed on this video"); }
    finally { v.removeAttribute("src"); try { v.load(); } catch (e) { } }
  };

  /* world-locked playback: a single-in-flight SEEK loop (never a live <video>
     in the warp — invariant #1). Each step: seek the offscreen video → bake
     that frame to the warp texture → set playPose to the frame's solved pose.
     The mesh warp redraws it; the dome layers project through the untouched
     view pose, so the sky/terrain stay frozen while the frame moves. */
  /* watchdogged seek shared by playback paths: same-time nudge (Safari may
     never fire `seeked` when seeking to the current time) + a timeout so a
     stalled decoder can NEVER leave the busy-lock stuck and kill the controls */
  const seekSafe = (v, t, onDone, ms) => {
    let fired = false;
    const wd = setTimeout(() => { if (!fired) { fired = true; v.onseeked = null; onDone(false); } }, ms || 2500);
    v.onseeked = () => { if (fired) return; fired = true; clearTimeout(wd); v.onseeked = null; onDone(true); };
    try { v.currentTime = t + (Math.abs((v.currentTime || 0) - t) < 0.001 ? 0.0001 : 0); }
    catch (e) { if (!fired) { fired = true; clearTimeout(wd); v.onseeked = null; onDone(false); } }
  };
  const ensurePlayVid = () => {
    const cur = playVidRef.current;
    if (cur && cur.readyState >= 2 && !cur.error) return Promise.resolve(cur);
    /* a broken/half-dead element must not be reused forever — drop and rebuild */
    if (cur) { try { cur.removeAttribute("src"); cur.load(); } catch (e) { } playVidRef.current = null; }
    return new Promise((res, rej) => {
      const v = document.createElement("video");
      v.muted = true; v.playsInline = true; v.preload = "auto";
      let settled = false;
      const wd = setTimeout(() => { if (!settled) { settled = true; rej(new Error("video load timeout")); } }, 6000);
      v.onloadeddata = () => { if (!settled) { settled = true; clearTimeout(wd); playVidRef.current = v; res(v); } }; // ref only once actually loaded
      v.onerror = () => { if (!settled) { settled = true; clearTimeout(wd); rej(new Error("video error")); } };
      v.src = source.mediaUrl;
      try { v.load(); } catch (e) { }
    });
  };
  const SCRUB_MIN_MS = 75;   // min gap between chased scrub seeks (iOS decoder pacing)
  const PLAY_DT = 0.1;       // target video-time per playback step (≈10 fps, see below)
  /* A seek's `seeked` event can fire BEFORE the decoded frame is actually
     presented — drawImage then bakes the PREVIOUS frame, so the texture lags
     the pose by one step and the photo appears to twitch against the fixed
     terrain/grid (field report: "scrubbing has a lot of jitter"). rVFC fires
     only once a frame is available; the timeout keeps a browser without it
     (or a stalled decoder) on the old behaviour. */
  const nextFrame = (v) => new Promise((res) => {
    if (typeof v.requestVideoFrameCallback !== "function") { res(); return; }
    let done = false, id = 0;
    const fin = () => { if (done) return; done = true; try { v.cancelVideoFrameCallback(id); } catch (e) { } res(); };
    try { id = v.requestVideoFrameCallback(fin); } catch (e) { res(); return; }
    setTimeout(fin, 100);   // armed BEFORE the seek, so a frame presented with `seeked` still resolves promptly
  });
  /* PREVIEW BEFORE STABILIZING. Playback used to require a solved posePath,
     which meant you could not even look through a clip in the sky view until
     you had sat through a stabilize run — and stabilizing is the slow step you
     most want to make an informed decision about. So when there is no solved
     path, synthesise one: the same 0.25 s cadence over the clip's duration,
     every sample carrying the CURRENT placement pose.

     That is honest by construction rather than by promise — with one pose for
     every frame the dome cannot world-lock anything, so the sky stays put and
     the footage moves against it exactly as the camera moved. It is a preview
     of the FOOTAGE, not of the result, and the row says so.

     Everything downstream (scrub, play, the readout, ‹ ›) indexes a path and
     does not care which kind it got. What genuinely needs solved poses — ⚓ fix
     frames, the smoothing sliders, export, the trajectory overlay — stays
     gated on source.posePath and is unreachable until you stabilize. */
  const [previewDur, setPreviewDur] = useState(0);
  const solvedPath = Array.isArray(source?.posePath) && source.posePath.length > 1 ? source.posePath : null;
  const playPath = useMemo(() => {
    if (solvedPath) return solvedPath;
    if (source?.mediaKind !== "video" || !(previewDur > 0.1)) return null;
    const out = [];
    for (let t = 0; t < previewDur - 0.02; t += 0.25) out.push({ t: +t.toFixed(3), az: pAz, el: pEl, roll: pRoll, fov: fovM, k: pDist });
    if (out.length < 2) return null;
    return out;
  }, [solvedPath, source?.mediaKind, previewDur, pAz, pEl, pRoll, fovM, pDist]);
  const playPathRef = useRef(playPath); playPathRef.current = playPath;
  /* read the duration once, lazily, so an unstabilized clip can be scrubbed.
     Cheap: the element is the same one playback reuses. */
  useEffect(() => {
    if (!open || solvedPath || source?.mediaKind !== "video" || previewDur > 0) return;
    let dead = false;
    ensurePlayVid().then((v) => { if (!dead && v.duration && isFinite(v.duration)) setPreviewDur(v.duration); }).catch(() => { });
    return () => { dead = true; };
  }, [open, solvedPath, source?.mediaKind, source?.mediaUrl]); // eslint-disable-line
  /* i is a FLOAT sample index: playback advances in sub-sample steps and takes
     the pose from posePathAt, so the motion is a ramp at ~10 fps instead of a
     jump at the 0.25 s solve cadence. That cadence is why playback looked
     worse than the export — the export always rendered interpolated poses at
     the clip's own frame rate. Integer indices (the scrubber, ‹ ›) land exactly
     on a solved sample, unchanged. */
  const showFrame = (i) => {
    const path = playPathRef.current; if (!path || !path.length) return;
    pendingIdxRef.current = clampN(i, 0, path.length - 1);
    if (seekBusyRef.current) return;              // one seek in flight; the latest request wins
    seekBusyRef.current = true;
    ensurePlayVid().then((v) => {
      const step = () => {
        const j = pendingIdxRef.current;
        const i0 = Math.floor(j), i1 = Math.min(path.length - 1, i0 + 1), u = j - i0;
        const tt = path[i0].t + (path[i1].t - path[i0].t) * u;
        const p = posePathAt(path, tt) || path[i0];
        const t0 = Date.now();
        const framed = nextFrame(v);   // arm before the seek — see nextFrame
        seekSafe(v, tt, (okSeek) => {
          if (!okSeek) { seekBusyRef.current = false; playingRef.current = false; setPlaying(false); setFlash("🎞 playback seek stalled — tap ▶ to retry"); return; }
          framed.then(() => {
            if (v.videoWidth) { try { bakeTex(v, v.videoWidth, v.videoHeight); } catch (e) { } }
            setPlayPose({ t: tt, az: p.az, el: p.el, roll: p.roll, fov: p.fov, k: p.k || 0 }); // t drives the object-track follow
            setPlayIdx(Math.round(j));
            const cost = Date.now() - t0;
            if (pendingIdxRef.current !== j) {
              /* a fast drag outran this seek — CHASE the latest, but PACE it: a
                 back-to-back seek→decode→bake loop at drag speed floods the iOS
                 video decoder (buffers accumulate faster than they're freed) and
                 crashes the tab. ~75 ms floor caps the loop to ~13 seeks/s — a
                 smooth scrub preview the decoder can actually sustain. */
              const gap = SCRUB_MIN_MS - cost;
              if (gap > 0) setTimeout(step, gap); else step();
              return;
            }
            seekBusyRef.current = false;
            if (playingRef.current) {
              if (j < path.length - 1) {
                /* sub-sample step, but only while the device keeps up: a seek
                   costing more than the budget falls back to whole samples so a
                   slow phone plays at today's cadence instead of stuttering */
                const dtS = Math.max(1e-3, path[i1].t - path[i0].t);
                const frac = cost > 90 ? 1 : clampN(PLAY_DT / dtS, 0.25, 1);
                const next = Math.min(path.length - 1, j + frac);
                const dtMs = clampN((next - j) * dtS * 1000, 60, 1500);
                setTimeout(() => { if (playingRef.current) showFrameRef.current(next); }, dtMs);
              } else { playingRef.current = false; setPlaying(false); }
            }
          });
        });
      };
      step();
    }).catch(() => { seekBusyRef.current = false; playingRef.current = false; setPlaying(false); setFlash("🎞 couldn't open the video for playback — tap ▶ to retry"); });
  };
  const showFrameRef = useRef(showFrame); showFrameRef.current = showFrame;
  const togglePlay = () => {
    if (playingRef.current) { playingRef.current = false; setPlaying(false); return; }
    playingRef.current = true; setPlaying(true);
    const path = playPath || [];
    showFrame(playIdx >= path.length - 1 ? 0 : playIdx + (playPose ? 1 : 0));
  };
  /* exit playback: re-bake the MARKED frame, then drop the pose override —
     texture and (placement) pose agree again, exactly as before playback */
  const exitPlayback = () => {
    playingRef.current = false; setPlaying(false);
    const refT2 = alignT;   // the static texture outside playback is the ALIGNMENT frame
    const path = playPath || [];
    let ri = 0; for (let i = 0; i < path.length; i++) if (Math.abs(path[i].t - refT2) < Math.abs(path[ri].t - refT2)) ri = i;
    pendingIdxRef.current = ri;
    const v = playVidRef.current;
    if (!v) { setPlayPose(null); setPlayIdx(ri); return; }
    seekBusyRef.current = true;
    seekSafe(v, refT2, (okSeek) => {
      if (okSeek && v.videoWidth) { try { bakeTex(v, v.videoWidth, v.videoHeight); } catch (e) { } }
      setPlayPose(null); setPlayIdx(ri); seekBusyRef.current = false; // drop the override regardless — never leave the lock stuck
    });
  };
  /* ⬇ EXPORT the stabilized clip — every video frame rendered through the
     mesh warp at its own (interpolated) pose from a FIXED virtual camera that
     frames the whole path, with a burned-in az/el grid + pose readout, and
     captured via canvas.captureStream + MediaRecorder into a real file. The
     render is paced against the wall clock so the output duration matches the
     clip (MediaRecorder stamps wall time); effective fps = seek throughput. */
  const [exporting, setExporting] = useState(0); // 0 idle | progress fraction
  const [exportMenu, setExportMenu] = useState(false);
  const [smoothOpen, setSmoothOpen] = useState(false);
  /* SCREEN WAKE LOCK — a long stabilize/export stalls when the phone dozes
     (field report: sleep pauses the processing). Uses the shared refcounted
     holder (wakeHold/wakeRelease) so it composes with the report/bundle
     builders; held only while a solve or export is actually running. */
  useEffect(() => {
    if (!stabBusy && !exporting) return;
    wakeHold();
    return () => wakeRelease();
  }, [!!stabBusy, !!exporting]); // eslint-disable-line
  /* SMOOTHING SLIDERS — re-derive the smoothed paths from the RAW (despiked)
     solves at the chosen strength, non-destructively: raw is kept on the
     source, so any strength can be revisited without re-running the whole
     stabilization. Legacy sources (no raw stored) adopt their current path
     as raw on first touch. */
  /* ONE derivation for the stored paths: smoothing at the chosen strengths
     over the raw solves, then the ⚓ Fix frames anchors applied on top —
     absolute pose fixes whose deltas interpolate between anchors and hold
     beyond the outermost ones (applyPoseFixes), with the object track
     shifted through the same delta field (applyDirFixes). Every consumer
     (playback, trajectory, waypoint conversion, exports, reports) reads the
     stored posePath/objPath, so anchors flow everywhere automatically. */
  const rederivePaths = (fixes, camS, objS) => {
    const patch = {};
    const rawP = Array.isArray(source?.posePathRaw) && source.posePathRaw.length ? source.posePathRaw : source?.posePath;
    if (!Array.isArray(rawP) || !rawP.length) return patch;
    if (!Array.isArray(source.posePathRaw) || !source.posePathRaw.length) patch.posePathRaw = rawP.map((p) => ({ ...p }));
    let base = smoothPathAt(rawP.map((p) => ({ ...p })), camS);
    /* SENSOR FUSION: an instrumented clip carries a continuous attitude log.
       Vision keeps the absolute frame; the log supplies MOTION across frames
       the tracker solved weakly or held, so they stop freezing. Re-applied
       here (not just in the walk) so smoothing and ⚓ anchors compose with it. */
    if (Array.isArray(source.sensorPath) && source.sensorPath.length > 4 && source.sensorSync) {
      /* sensor-only clips were already rebuilt from the log at stabilize time
         and must not be re-fused against a solve that never tracked */
      if (source.sensorSync.mode !== "sensor") base = fuseSensorVisual(base, source.sensorPath, { offset: isNum(source.sensorSync.offset) ? +source.sensorSync.offset : 0 });
    }
    const fx = Array.isArray(fixes) ? fixes.filter((f) => isNum(f?.t) && isNum(f?.az)) : [];
    patch.posePath = fx.length ? applyPoseFixes(base, fx) : base;
    const rawO = Array.isArray(source.objPathRaw) && source.objPathRaw.length ? source.objPathRaw : (Array.isArray(source.objPath) ? source.objPath : null);
    if (rawO && rawO.length) {
      if (!Array.isArray(source.objPathRaw) || !source.objPathRaw.length) patch.objPathRaw = rawO.map((p) => ({ ...p }));
      let o = smoothObjPathAt(rawO.map((p) => ({ ...p })), objS, { despiked: true });
      if (fx.length) o = applyDirFixes(o, base, fx, { natW: source.natW, natH: source.natH });
      /* waypoints are ground truth (same snap the stabilize pass applies):
         converted through the CURRENT (fixed) pose path, so the track stays
         pinned to the taps across smoothing and ⚓ anchor changes alike */
      if (source.natW && Array.isArray(source.track)) {
        const anchors = source.track
          .filter((p) => isNum(p.t) && isNum(p.x) && isNum(p.y))
          .map((p) => {
            const ps = posePathAt(patch.posePath, +p.t);
            return ps && isNum(ps.az) ? { t: +p.t, ...dirToAzEl(pixToDirK(+p.x, +p.y, source.natW, source.natH, ps.az, ps.el, ps.roll || 0, ps.fov, ps.k || 0)) } : null;
          })
          .filter(Boolean);
        if (anchors.length && o.length > 1) o = snapDirsToAnchors(o, anchors);
      }
      patch.objPath = o;
    }
    return patch;
  };
  const camSNow = isNum(source?.smoothCam) ? +source.smoothCam : 0.25;
  const objSNow = isNum(source?.smoothObj) ? +source.smoothObj : 0.25;
  /* Auto-anchor-to-terrain was REMOVED (see below): matchSkyline scans the
     full 360° of azimuth, so an edge that isn't really the terrain horizon —
     a nearby tree line — can fit best at a completely wrong bearing. The
     resulting anchors were inconsistent frame to frame, the world-view export
     grew to contain them, and the photo rendered as a tiny tilted sliver in a
     mostly-empty frame (field report). Any anchor it left behind is dropped
     here so a saved sighting recovers itself on open; hand anchors stay. */
  const fixesNow = (Array.isArray(source?.poseFixes) ? source.poseFixes : []).filter((f) => f && f.src !== "terrain");
  const applySmooth = (kind, v) => {
    if (!source || !update) return;
    const s = clampN(+v, 0, 1);
    const patch = {
      ...(kind === "cam" ? { smoothCam: s } : { smoothObj: s }),
      ...rederivePaths(fixesNow, kind === "cam" ? s : camSNow, kind === "obj" ? s : objSNow),
    };
    mediaDel(source.id + ":stab");   // a previously exported render is stale under the new path
    update(patch);
  };
  /* commit the pending playPose adjustment as an anchor (upsert by frame time) */
  const setFixAnchor = () => {
    if (!source || !update || !playPose || !isNum(playPose.t)) return;
    const fx = { t: +(+playPose.t).toFixed(3), az: +(+playPose.az).toFixed(3), el: +(+playPose.el).toFixed(3), roll: +(+(playPose.roll || 0)).toFixed(2), fov: +(+playPose.fov).toFixed(2) };
    const list = fixesNow.filter((f) => Math.abs(+f.t - fx.t) > 1e-3).concat([fx]).sort((a, b) => a.t - b.t);
    const patch = { poseFixes: list, ...rederivePaths(list, camSNow, objSNow) };
    mediaDel(source.id + ":stab");
    update(patch);
    /* no flash — users drop many anchors in a row and the toast slowed the
       rhythm; the readout's "anchored · N⚓" and the scrubber tick confirm it */
  };
  const dropFixAnchor = (t) => {
    if (!source || !update) return;
    const list = fixesNow.filter((f) => Math.abs(+f.t - t) > 1e-3);
    const patch = { poseFixes: list.length ? list : null, ...rederivePaths(list, camSNow, objSNow) };
    mediaDel(source.id + ":stab");
    update(patch);
    /* refresh the displayed pose from the re-derived path so the frame on
       screen snaps back to what the path now says */
    const np = patch.posePath?.[playIdx];
    if (np && playPose) setPlayPose({ t: np.t, az: np.az, el: np.el, roll: np.roll, fov: np.fov, k: np.k || 0 });
  };
  /* 🎛 one-axis nudge on the frame pose being fixed (same field ask as the
     place-mode fine tune: gestures are for rough moves, taps for exact) */
  const nudgeFix = (daz, del, drl, dfov) => setPlayPose((pp) => pp
    ? { ...pp, az: (((pp.az + daz) % 360) + 360) % 360, el: clampN(pp.el + del, -89, 89), roll: clampN((pp.roll || 0) + drl, -180, 180), fov: dfov ? clampN(pp.fov + dfov, 2, 150) : pp.fov }
    : pp);
  /* revert the pending (un-anchored) adjustment on the current frame */
  const revertFixFrame = () => {
    const p = source?.posePath?.[playIdx];
    if (p && playPose) setPlayPose({ t: p.t, az: p.az, el: p.el, roll: p.roll, fov: p.fov, k: p.k || 0 });
  };
  /* LIVE PREVIEW while adjusting: the pending pose applied as one more anchor
     over the smoothed base — exactly what ⚓ Anchor would produce, so the
     trajectory overlay shows the real outcome as you drag. When the frame is
     unadjusted the pending delta equals the interpolated delta at t, so the
     preview is identical to the stored path (no visual jump on scrub). */
  const fixBase = useMemo(() => {
    if (!fixOn || !Array.isArray(source?.posePath) || source.posePath.length < 2) return null;
    const rawP = Array.isArray(source.posePathRaw) && source.posePathRaw.length ? source.posePathRaw : source.posePath;
    return smoothPathAt(rawP.map((p) => ({ ...p })), camSNow);
  }, [fixOn, source?.posePathRaw, source?.posePath, camSNow]); // eslint-disable-line
  const fixPreview = useMemo(() => {
    if (!fixOn || !fixBase || !playPose || !isNum(playPose.t)) return null;
    const pend = { t: +playPose.t, az: +playPose.az, el: +playPose.el, roll: +(playPose.roll || 0), fov: +playPose.fov };
    const list = fixesNow.filter((f) => Math.abs(+f.t - pend.t) > 1e-3).concat([pend]).sort((a, b) => a.t - b.t);
    return applyPoseFixes(fixBase, list);
  }, [fixOn, fixBase, playPose, source?.poseFixes]); // eslint-disable-line
  /* pending = the on-screen pose differs from the stored path at this frame */
  const fixPending = (() => {
    if (!fixOn || !playPose) return false;
    const p = source?.posePath?.[playIdx];
    if (!p) return false;
    const dAz2 = ((playPose.az - p.az + 540) % 360) - 180;
    return Math.abs(dAz2) > 0.01 || Math.abs(playPose.el - p.el) > 0.01 || Math.abs((playPose.roll || 0) - (p.roll || 0)) > 0.01 || Math.abs(playPose.fov - p.fov) > 0.05;
  })();
  /* SELF-HEAL: purge anchors left by the removed terrain auto-anchor. The
     filtered list alone isn't enough — the STORED posePath was already
     derived through those anchors, so re-derive once and write both back. */
  useEffect(() => {
    if (!source || !update) return;
    const all = Array.isArray(source.poseFixes) ? source.poseFixes : null;
    if (!all || !all.some((f) => f && f.src === "terrain")) return;
    const kept = all.filter((f) => f && f.src !== "terrain");
    update({ poseFixes: kept.length ? kept : null, ...rederivePaths(kept, camSNow, objSNow) });
    mediaDel(source.id + ":stab");
    setFlash("⛰ removed the terrain auto-anchors — the stabilized path is back to the tracker's own solve");
  }, [source?.poseFixes]); // eslint-disable-line

  /* SELF-HEAL: a shipped bug briefly made the track-smoothing slider write the
     despike COUNT (a number) into objPath, which hid the slider itself and
     every other objPath consumer. The raw track was saved before the
     corruption — restore from it at the saved strength. */
  useEffect(() => {
    if (!source || !update) return;
    if (source.objPath != null && !Array.isArray(source.objPath) && Array.isArray(source.objPathRaw) && source.objPathRaw.length > 1) {
      update({ objPath: smoothObjPathAt(source.objPathRaw.map((p) => ({ ...p })), isNum(source.smoothObj) ? +source.smoothObj : 0.25, { despiked: true }) });
    }
  }, [source?.objPath]); // eslint-disable-line
  const exportAbortRef = useRef(0);
  /* mode: "view" (dome framing as shown in playback) | "full" (same framing,
     output sized so the most-zoomed frames keep native pixel density) |
     "crop" (camera centered on the marked object + both sight-lines, small
     FOV, full source resolution) */
  const exportStabilized = async (mode) => {
    if (typeof mode !== "string") mode = "view";
    const path = source?.posePath;
    if (!path || path.length < 2 || source?.mediaKind !== "video" || !source?.mediaUrl || !source?.natW) { setFlash("🎞 stabilize first, then export"); return; }
    if (typeof MediaRecorder === "undefined") { setFlash("⬇ this browser can't record video (no MediaRecorder)"); return; }
    if (mode === "crop" && !(source?.A?.p1 && source?.A?.p2)) { setFlash("◎ the object close-up needs the object marked on the measure step"); return; }
    if (exporting) { exportAbortRef.current++; return; }  // tap again = cancel
    const run = ++exportAbortRef.current;
    setExporting(0.01); setFlash(`⬇ rendering the world-locked clip${mode === "crop" ? " (object close-up)" : mode === "full" ? " (max resolution)" : ""}…`);
    const natW = source.natW, natH = source.natH;
    const v = document.createElement("video");
    v.muted = true; v.playsInline = true; v.preload = "auto";
    let cvs = null;
    try {
      await new Promise((res, rej) => { v.onloadeddata = res; v.onerror = rej; v.src = source.mediaUrl; try { v.load(); } catch (e) { } });
      /* fixed virtual camera. view/full: frame the union of every pose's
         corners. crop: center on the marked OBJECT (marks → world dirs
         through the placement pose, widened to cover the second sight-line
         when one exists) with generous margin — a small FOV, so the source's
         zoomed detail survives at full resolution. */
      const corners = [];
      for (const p of path) for (const [px, py] of [[0, 0], [natW, 0], [natW, natH], [0, natH]])
        corners.push(pixToDirK(px, py, natW, natH, p.az, p.el, p.roll, p.fov, p.k || 0));
      let sx = 0, sy = 0, sz = 0;
      for (const d of corners) { sx += d[0]; sy += d[1]; sz += d[2]; }
      let ce = dirToAzEl(unit([sx, sy, sz]));
      let camFov;
      const aspect = natH / natW;
      const minFov = Math.min(...path.map((p) => p.fov));
      const maxFov = Math.max(...path.map((p) => p.fov));
      /* the follow source decides the crop framing below, so resolve it first:
         auto-track when it survived, else the tapped-waypoint sky path */
      const objAll = Array.isArray(source?.objPath) && source.objPath.length > 1 ? source.objPath : followPath;
      let maxAngX = 2; // object's largest angular size (deg) — set in the crop branch, drives the pixel-pin window
      if (mode === "crop") {
        /* the object marks live on the MARKED frame — when it differs from the
           alignment frame, project them through THAT frame's solved pose (the
           placement pose describes the align frame only) */
        const mkp = (Math.abs(markT - alignT) > 0.05 && posePathAt(path, markT)) || { az: pAz, el: pEl, roll: pRoll, fov: fovM, k: pDist };
        const g1 = pixToDirK(source.A.p1.x, source.A.p1.y, natW, natH, mkp.az, mkp.el, mkp.roll || 0, mkp.fov, mkp.k || 0);
        const g2 = pixToDirK(source.A.p2.x, source.A.p2.y, natW, natH, mkp.az, mkp.el, mkp.roll || 0, mkp.fov, mkp.k || 0);
        const objAng = Math.acos(clampN(dot(g1, g2), -1, 1)) * R2D;
        let cd = unit([g1[0] + g2[0], g1[1] + g2[1], g1[2] + g2[2]]);
        let sep = 0;
        if (isNum(source?.B?.az) && isNum(source?.B?.el)) {
          const gB = dirFromAzEl(+source.B.az, +source.B.el);
          sep = Math.acos(clampN(dot(cd, gB), -1, 1)) * R2D;
          cd = unit([cd[0] + gB[0], cd[1] + gB[1], cd[2] + gB[2]]);
        }
        ce = dirToAzEl(cd);
        /* CLOSE-UP framing: a zoomed-in video of the OBJECT, with a little
           space around it at its LARGEST size along the track (keyframed
           sizes included) — user decision, replacing the clip-zoom-window
           average, whose "never tighter than the most-zoomed frame" floor
           meant it never actually read as a close-up. 2.2× the max angular
           size ⇒ the object fills ~45% of the frame at its biggest. Only
           safe when the camera FOLLOWS the object; with nothing to follow
           the static camera keeps the old zoom-window framing so a moving
           object can't walk out of a tight fixed crop. */
        const sized = (source.track || []).filter((p) => isNum(p.ang)).map((p) => +p.ang);
        const maxAng = Math.max(objAng, ...sized);
        maxAngX = maxAng;
        camFov = objAll
          ? clampN(maxAng * 2.2, 0.8, 70)
          : clampN((minFov + maxFov) / 2, Math.max(minFov, objAng * 3, sep * 1.2, 1.6), 70);
      }
      let B = photoBasis(ce.az, ce.el, 0);
      let mx = 0.05, my = 0.05;
      for (const d of corners) { const z = dot(d, B.f); if (z <= 0.05) continue; mx = Math.max(mx, Math.abs(dot(d, B.r) / z)); my = Math.max(my, Math.abs(dot(d, B.u) / z)); }
      if (mode !== "crop") camFov = clampN(2 * Math.atan(Math.max(mx, my / aspect) / 0.94) * R2D, 20, 118);
      /* close-up FOLLOWS the object when a track exists (objAll, resolved
         above): the virtual camera re-centers on the tracked direction each
         frame, so the object stays in the middle of the crop even as it
         crosses the sky */
      const camFollow = mode === "crop" && objAll;
      const objAt = (t) => {
        const op = objAll;
        let lo = 0, hi = op.length - 1;
        if (t <= op[0].t) hi = 0; else if (t >= op[op.length - 1].t) lo = op.length - 1;
        else while (hi - lo > 1) { const m = (lo + hi) >> 1; if (op[m].t <= t) lo = m; else hi = m; }
        const a = op[lo], b = op[hi], u = hi === lo ? 0 : (t - a.t) / Math.max(1e-9, b.t - a.t);
        const dAzT = ((b.az - a.az + 540) % 360) - 180;
        return { az: (((a.az + dAzT * u) % 360) + 360) % 360, el: a.el + (b.el - a.el) * u };
      };
      /* PIXEL-PINNED FOLLOW (close-up export only): the track is a solve at
         ~4 samples/s — between samples the interpolated center drifts off the
         object by a fraction of a degree, and a tight crop magnifies that
         into visible wander. At export time each frame's PIXELS are in hand,
         so re-lock the camera on the MEASURED object: a center-surround
         contrast snap (the same detector that seeds the auto-tracker) in a
         small window around the predicted pixel. EMA-damped so refine noise
         can't inject its own jitter; any failed/edge find falls back to the
         track prediction. Display-only — the objPath measurement is never
         touched. */
      /* v2, rebuilt against the user's real close-up (measured: the object
         swung ±20% of the frame; the v1 snapToObject window's 16 px reach
         never even saw it). The pin is now a SELF-CHAINING 30 fps tracker:
         · ACQUIRE — pinFind's integral-image contrast sweep over a WIDE
           window (±~2× the wander measured in the field clip), full-res so
           a faint 12 px object survives;
         · FOLLOW — once locked, search a tight window around the pin's OWN
           previous find (at 30 fps the object moves a few px/frame, no
           matter how wrong the low-rate track is);
         · the TRACK stays the seed + CLUTTER GATE (a find far from the
           track's prediction is a bird/lookalike — the human-guided track
           outranks pixels), and misses world-hold the last lock, easing
           toward the track so a fade never reads as a jump. */
      const pinCvs = camFollow ? document.createElement("canvas") : null;
      const pinCtx = pinCvs ? pinCvs.getContext("2d", { willReadFrequently: true }) : null;
      let pinPrevDir = null, pinMissRun = 0, pinEmaDir = null, pinOk = 0, pinTry = 0;
      const lerpDir = (a2, b2, u2) => unit([a2[0] + (b2[0] - a2[0]) * u2, a2[1] + (b2[1] - a2[1]) * u2, a2[2] + (b2[2] - a2[2]) * u2]);
      const refinePin = (p, pred) => {
        if (!pinCtx || !v.videoWidth) return pred;
        pinTry++;
        const predDir = dirFromAzEl(pred.az, pred.el);
        const locked = pinPrevDir && pinMissRun < 10;
        const baseDir = locked ? pinPrevDir : predDir;
        const outD = (d2) => { const ae = dirToAzEl(d2); return { az: ae.az, el: ae.el }; };
        const miss = () => {
          pinMissRun++;
          if (pinPrevDir && pinMissRun >= 10) { pinPrevDir = null; pinEmaDir = null; return pred; }
          if (pinEmaDir) { pinEmaDir = lerpDir(pinEmaDir, predDir, 0.12); return outD(pinEmaDir); }
          return pred;
        };
        const bp = dirToPixK(baseDir, natW, natH, p.az, p.el, p.roll || 0, p.fov, p.k || 0);
        if (!bp) return miss();
        const objPx = clampN(natW * Math.tan((Math.max(0.05, maxAngX) * RAD) / 2) / Math.tan((p.fov * RAD) / 2), 4, 220);
        const objR = clampN(Math.round(objPx / 2), 2, 14);
        const reach = locked ? Math.max(10, objR * 2) : Math.round(clampN(objPx * 6, 110, 240));
        const half = reach + objR * 3 + 4;
        const W2 = Math.min(560, 2 * half);
        if (pinCvs.width !== W2) { pinCvs.width = W2; pinCvs.height = W2; }
        const sv = (v.videoWidth || natW) / natW;
        try {
          pinCtx.fillStyle = "#000"; pinCtx.fillRect(0, 0, W2, W2);
          pinCtx.drawImage(v, (bp.px - W2 / 2) * sv, (bp.py - W2 / 2) * sv, W2 * sv, W2 * sv, 0, 0, W2, W2);
          const f = pinFind(pinCtx.getImageData(0, 0, W2, W2).data, W2, W2, W2 / 2, W2 / 2, { objR, reach: Math.min(reach, W2 / 2 - objR * 3 - 1), step: locked ? 2 : 3 });
          if (!f || f.score < 5) return miss();   // faded below sky noise — nothing to pin
          const fd = pixToDirK(bp.px + (f.x - W2 / 2), bp.py + (f.y - W2 / 2), natW, natH, p.az, p.el, p.roll || 0, p.fov, p.k || 0);
          const gate = (maxAngX * 2.5 + 0.6) * RAD;
          if (Math.acos(clampN(dot(fd, predDir), -1, 1)) > gate) return miss();
          pinPrevDir = fd; pinMissRun = 0; pinOk++;
          pinEmaDir = pinEmaDir ? lerpDir(pinEmaDir, fd, 0.6) : fd;
          return outD(pinEmaDir);
        } catch (e) { return miss(); }
      };
      /* output size: "view" caps at 1920; "full"/"crop" size the canvas so the
         most-zoomed frame's pixel density survives (tan-space ratio of the
         camera FOV to the finest source FOV). PREFERRED ENCODER: WebCodecs
         VideoEncoder, encoding OFFLINE frame-by-frame with explicit
         timestamps + backpressure — MediaRecorder is a REALTIME API, and a
         phone encoder that can't keep 30 fps at high resolution silently
         drops/queues frames until the output truncates (field-observed twice
         on an iPhone 14 at 3840 then 2560 wide). The offline path has no
         realtime constraint, so full 3840 is safe everywhere it's supported;
         the MediaRecorder fallback keeps the conservative iOS cap. */
      const isIOS = /iP(hone|ad|od)/.test(navigator.userAgent) || (navigator.platform === "MacIntel" && navigator.maxTouchPoints > 1);
      let OUT_W = Math.min(1920, natW);
      if (mode !== "view") {
        /* `ideal` IS native density: output sized so the most-zoomed frame's
           source pixels map 1:1 through the warp. Cap at 4096 (H.264 level
           5.1/5.2 hardware ceiling) — the runtime ladder steps down through
           3840/2560/1920 if the encoder refuses, so asking high is safe. */
        const ideal = natW * Math.tan((camFov * RAD) / 2) / Math.tan((minFov * RAD) / 2);
        OUT_W = mode === "full" ? Math.min(4096, Math.max(OUT_W, Math.round(ideal / 2) * 2))
          : Math.min(3840, Math.max(1280, Math.round(ideal / 2) * 2));
      }
      const bpsFor = (w2, h2) => clampN(Math.round(8e6 * (w2 * h2) / (1920 * 1080)), 6e6, isIOS ? 16e6 : 25e6);
      const desiredW = OUT_W;
      /* mutable render geometry: the encode ladder below can retry at a
         smaller size after a RUNTIME encoder failure (isConfigSupported can
         say yes and the hardware still refuse — field-observed), rebuilding
         the hidden canvas in place */
      let OUT_H = 0, tH = 0, tV = 0, ctx = null;
      const setSize = (w2) => {
        OUT_W = w2; OUT_H = Math.round(w2 * aspect / 2) * 2;
        tH = Math.tan((camFov * RAD) / 2); tV = tH * OUT_H / OUT_W;
        if (cvs && cvs.parentNode) cvs.parentNode.removeChild(cvs);
        cvs = document.createElement("canvas"); cvs.width = OUT_W; cvs.height = OUT_H;
        cvs.style.cssText = "position:fixed;left:0;top:0;width:1px;height:1px;opacity:0.01;pointer-events:none;z-index:-1";
        document.body.appendChild(cvs);
        ctx = cvs.getContext("2d");
      };
      const proj = (d) => { const z = dot(d, B.f); if (z <= 0.001) return null; return [(0.5 + (dot(d, B.r) / z) / (2 * tH)) * OUT_W, (0.5 - (dot(d, B.u) / z) / (2 * tV)) * OUT_H]; };
      const tex = document.createElement("canvas");
      /* warp-texture cap: 1600 px matches live playback for "view"; the
         resolution modes keep the source frame NATIVE (their whole point is
         native detail — the old 2048 cap silently halved 4K sources before
         the warp ever saw them), guarded only by the iOS canvas ceiling
         (4600 px side / 16 Mpx area, the media-normalize doctrine). */
      const vw = v.videoWidth || natW, vh = v.videoHeight || natH;
      let tsc = mode === "view" ? Math.min(1, 1600 / Math.max(vw, vh)) : Math.min(1, 4600 / Math.max(vw, vh));
      if (vw * vh * tsc * tsc > 16e6) tsc *= Math.sqrt(16e6 / (vw * vh * tsc * tsc));
      tex.width = Math.max(2, Math.round(vw * tsc)); tex.height = Math.max(2, Math.round(vh * tsc));
      const tctx = tex.getContext("2d");
      const gpath = (pts) => { let s2 = "", started = false; for (const q of pts) { if (!q) { started = false; continue; } s2 += (started ? "L" : "M") + q[0].toFixed(1) + " " + q[1].toFixed(1); started = true; } return s2; };
      /* the fitted 3D WIREFRAME rides the object track in every framing while
         the 🛸 overlay is on: curve points (native px on the MARKED frame) →
         dirs under that frame's SOLVED pose (align frame may differ), rotated
         onto the tracked dir per frame */
      const mkP = posePathAt(path, markT) || { az: pAz, el: pEl, roll: pRoll, fov: fovM, k: pDist };
      const wireBase = objOn && objAll && source?.shapeFit && source?.A?.p1 && source?.A?.p2;
      /* KEYFRAMED per frame: rebuild the wireframe at the size/attitude the user
         marked at THIS frame's time (shapeAt), projected through the marked
         pose, then Rodrigues-rotated onto the tracked dir in drawFrame. */
      const wireDirsAt = (t) => wireBase
        ? shapeProjNat(shapeAt(source.shapeFit, source.track, t, markT)).curves.map((c) => c.map((pt) => pixToDirK(pt.x, pt.y, natW, natH, mkP.az, mkP.el, mkP.roll || 0, mkP.fov, mkP.k || 0)))
        : null;
      const objD0 = wireBase ? pixToDirK(source.shapeFit.cx, source.shapeFit.cy, natW, natH, mkP.az, mkP.el, mkP.roll || 0, mkP.fov, mkP.k || 0) : null;
      /* --- every dome layer that is VISIBLE in the world view is burned into
         the export, honouring the same toggles: terrain skyline + ridges +
         named peaks, building boxes, stars/planets/Sun/Moon, satellites +
         Starlink (with trails), aircraft chips + sky-tracks, compass letters.
         All drawn from WORLD az/el data through the export camera each frame
         (the crop camera moves), so they stay world-locked like the grid.
         The schematic overlays (winds-aloft stack, cloud veil) are screen-
         space aids with made-up heights — burning them into a world-locked
         exhibit would lie, so they stay dome-only. --- */
      let bldgLayerCache = null; // hidden-line removal is the pricey bit — cache for the fixed camera
      const drawSkyLayers = (span) => {
        const u = OUT_W / 100, lw = Math.max(1, OUT_W / 1400);
        const lfs = Math.max(9, Math.round(OUT_H / 70));
        const P = (az, el) => proj(dirFromAzEl(az, el));
        const text = (s, x, y, col, size, weight, align) => {
          ctx.font = `${weight || 700} ${size || lfs}px Menlo, monospace`;
          ctx.textAlign = align || "center"; ctx.shadowColor = "rgba(0,0,0,.9)"; ctx.shadowBlur = 3;
          ctx.fillStyle = col; ctx.fillText(s, x, y);
          ctx.shadowBlur = 0; ctx.textAlign = "left";
        };
        const poly = (pts, col, width, dash) => { // pts: [x,y]|null — null breaks the pen
          ctx.strokeStyle = col; ctx.lineWidth = width; ctx.setLineDash(dash || []); ctx.lineCap = dash ? "round" : "butt";
          ctx.beginPath();
          let pen = false;
          for (const q of pts) {
            if (!q || q[0] < -OUT_W || q[0] > 2 * OUT_W || q[1] < -OUT_H || q[1] > 2 * OUT_H) { pen = false; continue; }
            if (pen) ctx.lineTo(q[0], q[1]); else { ctx.moveTo(q[0], q[1]); pen = true; }
          }
          ctx.stroke(); ctx.setLineDash([]);
        };
        const sightD = isNum(source?.A?.az) && isNum(source?.A?.el) ? dirFromAzEl(+source.A.az, +source.A.el) : null;
        const lookD = dirFromAzEl(ce.az, ce.el);
        const near = (r, d, deg) => r && Math.acos(clampN(dot(r, d), -1, 1)) * R2D <= deg;
        /* terrain skyline + interior ridges (same green, ridges faded by distance) */
        if (terrOn && terr?.els) {
          for (const r of terr.ridges || []) {
            const t = clampN(Math.log(r.dist / 800) / Math.log(35000 / 800), 0, 1);
            poly(r.pts.map(([raz, rel]) => { const da = ((raz - ce.az + 540) % 360) - 180; return Math.abs(da) <= span + 10 ? P(ce.az + da, rel) : null; }), ridgeCol(0.60 - 0.35 * t), lw, [7 * lw, 4 * lw]);
          }
          const pts = []; for (let a = -span - 10; a <= span + 10; a += 0.4) pts.push(P(ce.az + a, skylineElAt(terr.els, ce.az + a)));
          poly(pts, ridgeCol(0.9), 1.4 * lw, [7 * lw, 4 * lw]);
          const tl = P(ce.az, skylineElAt(terr.els, ce.az));
          if (tl) text("TERRAIN", tl[0], tl[1] - 6 * lw, ridgeCol(0.95), Math.max(8, lfs - 1));
        }
        /* named peaks that sit on the drawn silhouette (top 8 by height, like the dome) */
        if (peaksOn && peakMarks.length) {
          const inv = peakMarks.map((pk) => ({ pk, q: P(pk.az, pk.elv) })).filter((c) => c.q && c.q[0] > 0.01 * OUT_W && c.q[0] < 0.99 * OUT_W && c.q[1] > -0.02 * OUT_H && c.q[1] < 1.02 * OUT_H)
            .sort((a, b) => (b.pk.eleM || 0) - (a.pk.eleM || 0)).slice(0, 8);
          for (const { pk, q } of inv) {
            ctx.fillStyle = ridgeCol(0.98);
            ctx.beginPath(); ctx.moveTo(q[0], q[1]); ctx.lineTo(q[0] - 3.5 * lw, q[1] - 6 * lw); ctx.lineTo(q[0] + 3.5 * lw, q[1] - 6 * lw); ctx.closePath(); ctx.fill();
            const ele = pk.eleM != null ? (isImperialUnits() ? Math.round(pk.eleM * 3.28084).toLocaleString() + " ft" : Math.round(pk.eleM).toLocaleString() + " m") : null;
            text(pk.name, q[0], q[1] - (ele ? 22 : 10) * lw, ridgeCol(0.98));
            if (ele) text(ele, q[0], q[1] - 10 * lw, ridgeCol(0.78), Math.max(8, lfs - 1), 400);
          }
        }
        /* building boxes — accurate footprints faint, known-height extrusions
           with hidden-line removal (mirrors the dome). The crop camera moves
           per frame and the occlusion pass is too heavy to redo 30×/s, so the
           object close-up skips this layer. */
        if (bldgOn && bldg?.boxes && mode !== "crop") {
          if (!bldgLayerCache) bldgLayerCache = (() => {
            const K = 0.13, eye = camH, foot = [], known = [];
            for (const b of bldg.boxes) {
              let cE = 0, cN = 0;
              for (const q2 of b.ring) { cE += q2[0]; cN += q2[1]; }
              cE /= b.ring.length; cN /= b.ring.length;
              const da = ((((Math.atan2(cE, cN) * R2D) + 360) % 360) - ce.az + 540) % 360 - 180;
              if (Math.abs(da) > span + 15) continue;
              const base = [], roof = [], all = []; let ok = true;
              for (const [e, n] of b.ring) {
                const dist = Math.hypot(e, n);
                const az = ((Math.atan2(e, n) * R2D) + 360) % 360;
                const curv = (dist * dist * (1 - K)) / (2 * 6371000);
                const bp = P(az, Math.atan2(-eye - curv, dist) * R2D);
                if (!bp) { ok = false; break; }
                base.push(bp); all.push(bp);
                if (!b.assumed) {
                  const rp = P(az, Math.atan2(b.h - eye - curv, dist) * R2D);
                  if (!rp) { ok = false; break; }
                  roof.push(rp); all.push(rp);
                }
              }
              if (!ok || base.length < 3) continue;
              foot.push(base);
              if (!b.assumed) {
                let x0 = 1e9, y0 = 1e9, x1 = -1e9, y1 = -1e9;
                for (const [x, y] of all) { if (x < x0) x0 = x; if (y < y0) y0 = y; if (x > x1) x1 = x; if (y > y1) y1 = y; }
                known.push({ base, roof, hull: convexHull2(all), bbox: [x0, y0, x1, y1] });
              }
            }
            const segs = [];
            for (let i = 0; i < known.length; i++) {
              const Bx = known[i], hulls = [];
              for (let j = 0; j < i && hulls.length < 40; j++) if (bboxHit(Bx.bbox, known[j].bbox)) hulls.push(known[j].hull);
              const edges = [], N = Bx.base.length;
              for (let k = 0; k < N; k++) edges.push([Bx.roof[k], Bx.roof[(k + 1) % N]]);
              for (let k = 0; k < N; k++) edges.push([Bx.base[k], Bx.roof[k]]);
              for (const [p2, q2] of edges) for (const [t0, t1] of visibleSegs(p2, q2, hulls))
                segs.push([[p2[0] + (q2[0] - p2[0]) * t0, p2[1] + (q2[1] - p2[1]) * t0], [p2[0] + (q2[0] - p2[0]) * t1, p2[1] + (q2[1] - p2[1]) * t1]]);
            }
            return { foot, segs };
          })();
          ctx.strokeStyle = "rgba(255,178,74,0.5)"; ctx.lineWidth = lw; ctx.lineJoin = "round";
          for (const f of bldgLayerCache.foot) { ctx.beginPath(); f.forEach((q, i) => i ? ctx.lineTo(q[0], q[1]) : ctx.moveTo(q[0], q[1])); ctx.closePath(); ctx.stroke(); }
          ctx.strokeStyle = "rgba(255,178,74,0.95)"; ctx.lineWidth = 1.4 * lw;
          ctx.beginPath();
          for (const [a2, b2] of bldgLayerCache.segs) { ctx.moveTo(a2[0], a2[1]); ctx.lineTo(b2[0], b2[1]); }
          ctx.stroke();
        }
        /* stars (halo + core, brightness-scaled) + labels for the bright named ones */
        for (const s of stars) {
          const q = P(s.az, s.alt); if (!q || q[0] < -0.05 * OUT_W || q[0] > 1.05 * OUT_W || q[1] < -0.05 * OUT_H || q[1] > 1.05 * OUT_H) continue;
          ctx.globalAlpha = s.o * 0.35; ctx.fillStyle = "#bcd2ff";
          ctx.beginPath(); ctx.arc(q[0], q[1], s.r * 0.5 * u, 0, 7); ctx.fill();
          ctx.globalAlpha = clampN(s.o + 0.1, 0, 1); ctx.fillStyle = "#fff";
          ctx.beginPath(); ctx.arc(q[0], q[1], s.r * 0.24 * u, 0, 7); ctx.fill();
          ctx.globalAlpha = 1;
          if (s.name && (s.mag <= 1.6 || (camFov < 42 && s.mag <= 2.6))) text(s.name, q[0] + 7 * lw, q[1] - 5 * lw, "#eaf1ff", Math.max(8, lfs - 1), 600, "left");
        }
        /* planets — glowing labeled markers */
        if (planetsVisible) for (const pl of planets) {
          const q = P(pl.az, pl.alt); if (!q || q[0] < -0.05 * OUT_W || q[0] > 1.05 * OUT_W || q[1] < -0.05 * OUT_H || q[1] > 1.05 * OUT_H) continue;
          ctx.shadowColor = "rgba(255,225,150,.75)"; ctx.shadowBlur = 9 * lw; ctx.fillStyle = "#fff6d8";
          ctx.beginPath(); ctx.arc(q[0], q[1], Math.max(3.5, 0.005 * OUT_W), 0, 7); ctx.fill(); ctx.shadowBlur = 0;
          text(`${pl.sym} ${pl.name}`, q[0], q[1] + 14 * lw, "#ffe9b0");
        }
        /* Sun & Moon at true angular size (0.53°) */
        const bodyR = Math.max(OUT_W * Math.tan((0.53 * RAD) / 2) / (2 * tH), 5 * lw);
        if (sun.alt > -1) { const q = P(sun.az, sun.alt); if (q) { ctx.shadowColor = "rgba(255,214,90,.85)"; ctx.shadowBlur = bodyR * 1.2; ctx.fillStyle = "#ffd76a"; ctx.beginPath(); ctx.arc(q[0], q[1], bodyR, 0, 7); ctx.fill(); ctx.shadowBlur = 0; } }
        if (moon.alt > -1) { const q = P(moon.az, moon.alt); if (q) { ctx.shadowColor = "rgba(220,230,250,.5)"; ctx.shadowBlur = bodyR * 0.8; ctx.fillStyle = "#e6ebf5"; ctx.beginPath(); ctx.arc(q[0], q[1], bodyR, 0, 7); ctx.fill(); ctx.shadowBlur = 0; } }
        /* satellites: dotted pass trails + diamond markers with labels */
        for (const s of satView) {
          poly((s.trail || []).map((q2) => q2.el > -1 ? P(q2.az, q2.el) : null), `rgba(159,220,255,${s.lit ? 0.45 : 0.18})`, 1.4 * lw, [0.1, 7 * lw]);
          const q = P(s.az, s.el); if (!q) continue;
          const col = s.lit ? "#9fdcff" : "rgba(159,220,255,.35)", sz = Math.max(3, 0.004 * OUT_W);
          ctx.save(); ctx.translate(q[0], q[1]); ctx.rotate(Math.PI / 4); ctx.fillStyle = col;
          if (s.lit) { ctx.shadowColor = "rgba(159,220,255,.5)"; ctx.shadowBlur = 5 * lw; }
          ctx.fillRect(-sz / 2, -sz / 2, sz, sz); ctx.restore(); ctx.shadowBlur = 0;
          text(`🛰 ${s.name}${s.lit ? "" : " · in shadow"}`, q[0], q[1] + 12 * lw, col, Math.max(8, lfs - 1));
          text(`${Math.round(s.rangeKm)} km`, q[0], q[1] + 12 * lw + lfs, col, Math.max(8, lfs - 1), 400);
        }
        /* Starlink: violet dots; trails only near the view/sight-line (dome rule) */
        let slLines = 0;
        for (const s of slView) {
          if (s.trail && slLines <= 14 && (near(lookD, dirFromAzEl(s.az, s.el), 30) || near(sightD, dirFromAzEl(s.az, s.el), 30))) {
            poly(s.trail.map((q2) => q2.el > -1 ? P(q2.az, q2.el) : null), "rgba(201,182,255,.72)", 1.6 * lw, [0.1, 5 * lw]); slLines++;
          }
          const q = P(s.az, s.el); if (!q) continue;
          ctx.fillStyle = "rgba(201,182,255,.9)"; ctx.beginPath(); ctx.arc(q[0], q[1], 0.32 * u, 0, 7); ctx.fill();
        }
        /* aircraft: sky-tracks near the view/sight-line (or selected) + heading-rotated chips */
        for (const v2 of acView) {
          const sel = v2.a.hex === selHex;
          const raw = acData?.hist ? v2.a.trail : liveTrailRef.current.get(v2.a.hex);
          if (raw && raw.length > 1 && (sel || near(lookD, v2.d, 25) || near(sightD, v2.d, 25))) {
            const tp = [];
            for (let i2 = 0; i2 < raw.length; i2++) {
              if (i2 === 0) { const g = acAzElRange({ lat: LAT, lon: LNG, alt: 0 }, { lat: raw[0][1], lon: raw[0][2], altM: raw[0][3] }); tp.push(proj(g.d)); continue; }
              const a0 = raw[i2 - 1], a1 = raw[i2], K2 = 5;
              for (let k2 = 1; k2 <= K2; k2++) { const f2 = k2 / K2; const g = acAzElRange({ lat: LAT, lon: LNG, alt: 0 }, { lat: a0[1] + (a1[1] - a0[1]) * f2, lon: a0[2] + (a1[2] - a0[2]) * f2, altM: a0[3] + (a1[3] - a0[3]) * f2 }); tp.push(proj(g.d)); }
            }
            poly(tp, sel ? "#F5A93F" : "#8FB4FF", (sel ? 2.2 : 1.6) * lw, sel ? [0.1, 6 * lw] : [0.1, 9 * lw]);
          }
          const q = proj(v2.d); if (!q || q[0] < -0.05 * OUT_W || q[0] > 1.05 * OUT_W || q[1] < -0.05 * OUT_H || q[1] > 1.05 * OUT_H) continue;
          let rot = 0;
          if (isNum(v2.a.track) && isNum(v2.a.gs) && v2.a.gs > 5) {
            const dM = v2.a.gs * 15;
            const p2 = proj(acAzElRange({ lat: LAT, lon: LNG, alt: 0 }, { ...v2.a, lat: +v2.a.lat + (dM * Math.cos(v2.a.track * D2R)) / 111320, lon: +v2.a.lon + (dM * Math.sin(v2.a.track * D2R)) / (111320 * Math.cos(LAT * D2R)) }).d);
            if (p2) rot = Math.atan2(p2[1] - q[1], p2[0] - q[0]);
          }
          const col = sel ? "#F5A93F" : "#8FB4FF";
          ctx.save(); ctx.translate(q[0], q[1]); ctx.rotate(rot);
          ctx.font = `${Math.max(11, Math.round(OUT_H / 46))}px Menlo, monospace`; ctx.textAlign = "center"; ctx.textBaseline = "middle";
          ctx.shadowColor = "rgba(0,0,0,.85)"; ctx.shadowBlur = 3; ctx.fillStyle = col; ctx.fillText("✈", 0, 0);
          ctx.restore(); ctx.shadowBlur = 0; ctx.textBaseline = "alphabetic";
          const id = (v2.a.flight || "").trim() || v2.a.reg || v2.a.hex;
          text(`${id}${v2.a.t ? ` ${v2.a.t}` : ""}`, q[0], q[1] + 13 * lw, col, Math.max(8, lfs - 1));
          text(`${fmtLenShort(v2.rangeM)}${v2.a.altM != null ? ` · ${Math.round(v2.a.altM * 3.28084 / 100) / 10} kft` : ""}`, q[0], q[1] + 13 * lw + lfs, col, Math.max(8, lfs - 1), 400);
        }
        /* compass letters just above the horizon */
        for (const [caz, lbl] of [[0, "N"], [45, "NE"], [90, "E"], [135, "SE"], [180, "S"], [225, "SW"], [270, "W"], [315, "NW"]]) {
          const q = P(caz, 1.8);
          if (q && q[0] > 0.02 * OUT_W && q[0] < 0.98 * OUT_W && q[1] > -0.05 * OUT_H && q[1] < 1.05 * OUT_H) text(lbl, q[0], q[1], "#fff", Math.max(11, Math.round(OUT_H / 52)), 800);
        }
      };
      const drawFrame = (p) => {
        if (camFollow && isNum(p.t)) { ce = refinePin(p, objAt(p.t)); B = photoBasis(ce.az, ce.el, 0); }
        ctx.fillStyle = "#0a0f1c"; ctx.fillRect(0, 0, OUT_W, OUT_H);
        /* az/el grid, world-locked — THE debugging reference: if stabilization
           holds, the scenery rides these lines while the frame moves. Grid
           pitch scales with the camera FOV so a tight object crop still shows
           usable lines. Drawn OVER the frame (like the dome) so the reference
           is visible across the whole photo, not just past its edges. */
        const span = camFov <= 24 ? camFov : camFov * 0.75 + 15;
        const drawGrid = () => {
          const G = camFov <= 3 ? 0.5 : camFov <= 8 ? 1 : camFov <= 24 ? 2 : camFov <= 60 ? 5 : 10;
          const ss = Math.min(2, G / 2);
          ctx.lineWidth = Math.max(1, OUT_W / 1400); ctx.strokeStyle = "rgba(140,165,200,0.30)";
          for (let az = Math.floor((ce.az - span) / G) * G; az <= ce.az + span; az += G) {
            ctx.beginPath();
            for (let el2 = Math.max(-88, ce.el - span), first = true; el2 <= Math.min(88, ce.el + span); el2 += ss) { const q = proj(dirFromAzEl(az, el2)); if (!q) { first = true; continue; } if (first) { ctx.moveTo(q[0], q[1]); first = false; } else ctx.lineTo(q[0], q[1]); }
            ctx.stroke();
          }
          for (let el2 = Math.max(-88, Math.floor((ce.el - span) / G) * G); el2 <= Math.min(88, ce.el + span); el2 += G) {
            ctx.beginPath(); ctx.strokeStyle = el2 === 0 ? "rgba(200,220,255,0.55)" : "rgba(140,165,200,0.30)";
            for (let az = ce.az - span, first = true; az <= ce.az + span; az += ss) { const q = proj(dirFromAzEl(az, el2)); if (!q) { first = true; continue; } if (first) { ctx.moveTo(q[0], q[1]); first = false; } else ctx.lineTo(q[0], q[1]); }
            ctx.stroke();
          }
        };
        /* the frame, mesh-warped at ITS pose */
        const fb = photoBasis(p.az, p.el, p.roll);
        const fpx = (natW / 2) / Math.tan((p.fov * RAD) / 2);
        const pixD = (px, py) => { const x = (px - natW / 2) / fpx, y = (natH / 2 - py) / fpx; const s2 = 1 + (p.k || 0) * (x * x + y * y); return unit([fb.f[0] + (fb.r[0] * x + fb.u[0] * y) * s2, fb.f[1] + (fb.r[1] * x + fb.u[1] * y) * s2, fb.f[2] + (fb.r[2] * x + fb.u[2] * y) * s2]); };
        /* mesh density: 8 cols is fine for the small on-screen "view" render,
           but the clean full/crop exports are up to 4096 px — 8 cols there makes
           the piecewise-linear warp visibly FACET (the "mesh shapes" showing
           through). 16 cols smooths it; the export is offline so the extra
           triangles cost nothing. */
        const NC = mode === "view" ? 8 : 16, NR = Math.max(4, Math.round(NC * natH / natW));
        const dst = [];
        for (let r2 = 0; r2 <= NR; r2++) { const row = []; for (let c2 = 0; c2 <= NC; c2++) row.push(proj(pixD((c2 / NC) * natW, (r2 / NR) * natH))); dst.push(row); }
        const sxp = (c2) => (c2 / NC) * tex.width, syp = (r2) => (r2 / NR) * tex.height;
        /* clip-mesh SEAM cover: each clipped triangle's edge is anti-aliased to
           transparent, so adjacent triangles leave a thin see-through seam. We
           overlap them by expanding each triangle radially — but the AA seam is
           ~1 device px REGARDLESS of resolution, so a fixed 0.6 px (tuned for the
           ~800 px dome) is far too small at 4096 px and the seams show. Scale the
           overlap with the output size. */
        const EXP = clampN(OUT_W / 1400, 0.6, 3);
        const tri = (s0, s1, s2, d0, d1, d2) => {
          const cx2 = (d0[0] + d1[0] + d2[0]) / 3, cy2 = (d0[1] + d1[1] + d2[1]) / 3;
          const ex = (q) => { const dx = q[0] - cx2, dy = q[1] - cy2, L = Math.hypot(dx, dy) || 1; return [q[0] + (dx / L) * EXP, q[1] + (dy / L) * EXP]; };
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
        for (let r2 = 0; r2 < NR; r2++) for (let c2 = 0; c2 < NC; c2++) {
          const d00 = dst[r2][c2], d10 = dst[r2][c2 + 1], d01 = dst[r2 + 1][c2], d11 = dst[r2 + 1][c2 + 1];
          if (!d00 || !d10 || !d01 || !d11) continue;
          // cull cells fully outside the output (a tight crop sees few cells)
          if (Math.max(d00[0], d10[0], d01[0], d11[0]) < -2 || Math.min(d00[0], d10[0], d01[0], d11[0]) > OUT_W + 2 ||
            Math.max(d00[1], d10[1], d01[1], d11[1]) < -2 || Math.min(d00[1], d10[1], d01[1], d11[1]) > OUT_H + 2) continue;
          tri([sxp(c2), syp(r2)], [sxp(c2 + 1), syp(r2)], [sxp(c2 + 1), syp(r2 + 1)], d00, d10, d11);
          tri([sxp(c2), syp(r2)], [sxp(c2 + 1), syp(r2 + 1)], [sxp(c2), syp(r2 + 1)], d00, d11, d01);
        }
        /* OVERLAYS ONLY ON THE WORLD VIEW: max-res and object close-up are
           CLEAN evidence renders — no grid, no layers, no wireframe, no
           readout (user decision: those two downloads are the footage). */
        if (mode === "view") {
          drawGrid();
          try { drawSkyLayers(span); } catch (e) { if (!drawFrame.lw) { drawFrame.lw = 1; console.warn("export layer draw:", e); } } // an overlay layer must never kill the export
        }
        const wireDirs = mode === "view" && wireBase && isNum(p.t) ? wireDirsAt(p.t) : null;
        if (wireDirs && isNum(p.t)) {
          const oT = objAt(p.t);
          const dT = dirFromAzEl(oT.az, oT.el);
          const ax = [objD0[1] * dT[2] - objD0[2] * dT[1], objD0[2] * dT[0] - objD0[0] * dT[2], objD0[0] * dT[1] - objD0[1] * dT[0]];
          const s3 = Math.hypot(ax[0], ax[1], ax[2]), c3 = clampN(dot(objD0, dT), -1, 1);
          const k3 = s3 > 1e-9 ? [ax[0] / s3, ax[1] / s3, ax[2] / s3] : null;
          const rotW = (v2) => {                        // Rodrigues: marked dir → tracked dir
            if (!k3) return v2;
            const kv = [k3[1] * v2[2] - k3[2] * v2[1], k3[2] * v2[0] - k3[0] * v2[2], k3[0] * v2[1] - k3[1] * v2[0]];
            const kd = dot(k3, v2);
            return [v2[0] * c3 + kv[0] * s3 + k3[0] * kd * (1 - c3), v2[1] * c3 + kv[1] * s3 + k3[1] * kd * (1 - c3), v2[2] * c3 + kv[2] * s3 + k3[2] * kd * (1 - c3)];
          };
          ctx.strokeStyle = objCol; ctx.lineWidth = Math.max(1, OUT_W / 1600); ctx.globalAlpha = 0.85;
          for (const c2 of wireDirs) {
            ctx.beginPath();
            let first = true;
            for (const d2 of c2) { const q = proj(rotW(d2)); if (!q) { first = true; continue; } if (first) { ctx.moveTo(q[0], q[1]); first = false; } else ctx.lineTo(q[0], q[1]); }
            ctx.stroke();
          }
          ctx.globalAlpha = 1;
        }
        if (mode === "view") {
          const fs = Math.max(12, Math.round(OUT_H / 42));
          ctx.font = fs + "px Menlo, monospace"; ctx.fillStyle = "rgba(235,243,255,0.92)";
          ctx.fillText(`PHODAR · world-locked`, 12, fs + 8);
          ctx.fillText(`t ${p.t.toFixed(2)}s · az ${p.az.toFixed(1)}° · el ${p.el.toFixed(1)}° · FOV ${p.fov.toFixed(1)}° · refs ${p.n == null ? "—" : p.n}`, 12, OUT_H - 12);
        }
        void gpath; // (kept for future vector overlays)
      };
      /* seeks are CLAMPED below the media duration (the stabilize walk always
         did this; the export didn't, and near-end seeks stalled the decoder —
         field-observed as an export truncated 1.1 s early with a frozen tail) */
      const durX = v.duration || (path[path.length - 1].t + 0.1);
      const seekX = (t) => new Promise((res) => {
        const tt = clampN(t, 0, Math.max(0, durX - 0.05));
        let f = false;
        const wd = setTimeout(() => { if (!f) { f = true; res(false); } }, 3000);
        v.onseeked = () => { if (!f) { f = true; clearTimeout(wd); res(true); } };
        try { v.currentTime = tt + (Math.abs((v.currentTime || 0) - tt) < 0.001 ? 0.0001 : 0); }
        catch (e) { if (!f) { f = true; clearTimeout(wd); res(false); } }
      });
      const t0 = path[0].t, t1 = path[path.length - 1].t;
      let blob = null, ext = "mp4", lastErr = null;
      const probeWC = async (w2) => {
        const h2 = Math.round(w2 * aspect / 2) * 2;
        for (const codec of ["avc1.640033", "avc1.640032", "avc1.64002a", "avc1.4d0028", "avc1.42e01f"]) {
          const cfg = { codec, width: w2, height: h2, bitrate: bpsFor(w2, h2), framerate: 30, latencyMode: "realtime", avc: { format: "avc" } };
          try { const s2 = await VideoEncoder.isConfigSupported(cfg); if (s2 && s2.supported) return cfg; } catch (e) { }
        }
        return null;
      };
      /* ---- OFFLINE encode (WebCodecs): each frame gets an explicit
         timestamp, so duration is exact by construction; the encoder is
         throttled by its own queue, never by the wall clock. A failed seek
         re-encodes the previous texture at the right timestamp (a brief
         freeze, never lost time). ---- */
      const wcAttempt = async (cfg) => {
        setSize(cfg.width);
        const samples = []; let avcC = null, encErr = null;
        const enc = new VideoEncoder({
          output: (c, m) => {
            const desc = m && m.decoderConfig && m.decoderConfig.description;
            if (!avcC && desc) avcC = desc instanceof ArrayBuffer ? new Uint8Array(desc.slice(0)) : new Uint8Array(desc.buffer.slice(desc.byteOffset, desc.byteOffset + desc.byteLength));
            const d = new Uint8Array(c.byteLength); c.copyTo(d);
            samples.push({ data: d, key: c.type === "key" });
          },
          error: (e) => { encErr = e; },
        });
        enc.configure(cfg);
        const total = Math.max(2, Math.round((t1 - t0) * 30));
        for (let fi = 0; fi < total; fi++) {
          if (exportAbortRef.current !== run || encErr) break;
          const mt = t0 + fi / 30;
          const ok = await seekX(mt);
          if (ok && v.videoWidth) { tctx.drawImage(v, 0, 0, tex.width, tex.height); drawFrame(posePathAt(path, mt)); }
          const vf = new VideoFrame(cvs, { timestamp: Math.round(fi * 1e6 / 30), duration: Math.round(1e6 / 30) });
          enc.encode(vf, { keyFrame: fi % 60 === 0 });
          vf.close();
          while (enc.encodeQueueSize > 4 && !encErr) await new Promise((r) => setTimeout(r, 5));
          setExporting(clampN(fi / total, 0.01, 0.99));
          if ((fi & 7) === 0) await new Promise((r) => setTimeout(r, 0));
        }
        if (!encErr && exportAbortRef.current === run) await enc.flush();
        try { enc.close(); } catch (e) { }
        if (encErr) throw encErr;
        if (exportAbortRef.current !== run || !avcC || !samples.length) return null;
        return new Blob([muxMp4({ width: OUT_W, height: OUT_H, fps: 30, avcC, samples })], { type: "video/mp4" });
      };
      if (typeof VideoEncoder !== "undefined" && typeof VideoFrame !== "undefined") {
        /* runtime step-down ladder: isConfigSupported can accept a size the
           hardware then refuses at encode time — try smaller before giving
           up on the offline path entirely */
        for (const w2 of [...new Set([desiredW, Math.min(desiredW, 3840), Math.min(desiredW, 2560), Math.min(desiredW, 1920)])]) {
          if (exportAbortRef.current !== run) break;
          const cfg = await probeWC(w2);
          if (!cfg) continue;
          try { blob = await wcAttempt(cfg); } catch (e) { lastErr = e; blob = null; }
          if (blob || exportAbortRef.current !== run) break;
        }
      }
      if (!blob && exportAbortRef.current === run) {
        /* ---- REALTIME fallback (MediaRecorder) — no WebCodecs, or every
           offline attempt failed. Wall-clock paced both ways: neither lag the
           clock (slow seeks → skip ahead) nor outrun it (fast seeks → wait);
           a stalled seek pauses the recorder with the span excluded from the
           pacing clock. Conservative iOS size cap (realtime 4K crashes). ---- */
        setSize(mode === "full" && isIOS ? Math.min(desiredW, 2560) : desiredW);
        const stream = cvs.captureStream(30);
        const mime = ["video/mp4", "video/webm;codecs=vp9", "video/webm"].find((m) => { try { return MediaRecorder.isTypeSupported(m); } catch (e) { return false; } }) || "";
        const rec = new MediaRecorder(stream, mime ? { mimeType: mime, videoBitsPerSecond: bpsFor(OUT_W, OUT_H) } : { videoBitsPerSecond: bpsFor(OUT_W, OUT_H) });
        const chunks = [];
        rec.ondataavailable = (e) => { if (e.data && e.data.size) chunks.push(e.data); };
        const recDone = new Promise((res) => { rec.onstop = res; });
        rec.start(1000);
        const wall0 = performance.now();
        let pausedMs = 0, mediaT = t0;
        const effElapsed = () => (performance.now() - wall0 - pausedMs) / 1000;
        while (mediaT < t1) {
          if (exportAbortRef.current !== run) break;
          const st = performance.now();
          let recPaused = false;
          const guard = setTimeout(() => { try { rec.pause(); recPaused = true; } catch (e) { } }, 300);
          const ok = await seekX(mediaT);
          clearTimeout(guard);
          if (recPaused) { pausedMs += performance.now() - st - 300; try { rec.resume(); } catch (e) { } }
          if (ok && v.videoWidth) { tctx.drawImage(v, 0, 0, tex.width, tex.height); drawFrame(posePathAt(path, mediaT)); }
          const next = (mediaT - t0) + 1 / 30;
          const ahead = next - effElapsed();
          if (ahead > 0.002) await new Promise((r) => setTimeout(r, ahead * 1000));
          mediaT = t0 + Math.max(effElapsed(), next);
          setExporting(clampN((mediaT - t0) / (t1 - t0), 0.01, 0.99));
          await new Promise((r) => setTimeout(r, 0));
        }
        rec.stop(); await recDone;
        if (exportAbortRef.current === run && chunks.length) {
          blob = new Blob(chunks, { type: mime || "video/webm" });
          ext = (mime || "").includes("mp4") ? "mp4" : "webm";
        }
      }
      if (!blob && exportAbortRef.current === run && lastErr) throw lastErr;
      if (blob) {
        /* keep the render: the share bundle packs the before/after pair, so
           the stabilized video must survive past this download (re-export
           overwrites; a new stabilize run deletes it as stale) */
        mediaPut(source.id + ":stab", { kind: "video", data: blob });
        const a = document.createElement("a");
        a.href = URL.createObjectURL(blob);
        a.download = `phodar-${mode === "crop" ? "object" : mode === "full" ? "stabilized-maxres" : "stabilized"}.${ext}`;
        document.body.appendChild(a); a.click(); a.remove();
        setTimeout(() => { try { URL.revokeObjectURL(a.href); } catch (e) { } }, 60000);
        const pinNote = camFollow && pinTry ? ` · pixel-pinned on the object (${Math.round(100 * pinOk / pinTry)}% of frames re-locked)` : "";
        setFlash(`⬇ exported ${(blob.size / 1e6).toFixed(1)} MB .${ext} (${OUT_W}×${OUT_H}) — world-locked at ${camFov.toFixed(0)}° camera FOV${pinNote}. It's also packed into the report bundle.`);
      } else setFlash("⬇ export cancelled");
    } catch (e) {
      /* say WHAT failed — "failed on this video" alone made field reports
         undiagnosable */
      const msg = e && (e.message || e.name) ? `${e.name || "error"}: ${e.message || ""}`.slice(0, 90) : "the browser refused the render";
      setFlash(`⬇ export failed — ${msg}`);
    }
    finally {
      setExporting(0);
      if (cvs && cvs.parentNode) cvs.parentNode.removeChild(cvs);
      v.removeAttribute("src"); try { v.load(); } catch (e) { }
    }
  };

  /* entering PLACE mode must never inherit the playback override — place
     gestures edit the REAL placement pose, which describes the ALIGNMENT
     frame, so the texture snaps back to that frame (calibration integrity
     over continuity). The read-only tools (trajectory · size · compare) KEEP
     the frame you scrubbed to — they only need a backdrop, and losing your
     spot on every mode switch was a field annoyance — playback just pauses
     there and resumes where you left it back in look mode. */
  useEffect(() => {
    if (pMode === "place" && (playPose || playingRef.current)) { setFixOn(false); exitPlayback(); return; }
    if ((trajOn || sizeOn || cmpOn) && playingRef.current) { playingRef.current = false; setPlaying(false); }
    /* trajectory stays compatible with fix mode (it's read-only in the world
       view — no gesture conflict, and watching it move is the live feedback);
       the other tools take over the gestures, so a fix drag must not linger */
    if ((sizeOn || cmpOn || calibOn) && fixOn) setFixOn(false);
  }, [pMode, trajOn, sizeOn, cmpOn, calibOn]); // eslint-disable-line
  /* teardown: cancel any running solve and release the playback video */
  useEffect(() => () => {
    stabAbortRef.current++;
    playingRef.current = false;
    const v = playVidRef.current;
    if (v) { v.removeAttribute("src"); try { v.load(); } catch (e) { } playVidRef.current = null; }
  }, []);
  /* new media (or a new marked frame) invalidates the playback element/pose */
  useEffect(() => {
    playingRef.current = false; setPlaying(false); setPlayPose(null); setPlayIdx(0); setFixOn(false);
    const v = playVidRef.current;
    if (v) { v.removeAttribute("src"); try { v.load(); } catch (e) { } playVidRef.current = null; }
  }, [source?.mediaUrl]);

  const handleClose = () => { if (photoOn) commitPlacement(); onClose(); };

  const aimColor = which === "B" ? "var(--teal)" : accentCol;
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

        {/* ☁ cloud SHADING — a grey wash over the sky (above the horizon) whose
            opacity tracks the % cover; behind the photo, so it tints the open sky
            without drawing fake cloud blobs over the real ones in the image */}
        {cloudShade > 0 && cloudSkyBot > 0.01 && (
          <div style={{
            position: "absolute", left: 0, right: 0, top: 0, height: (cloudSkyBot * 100) + "%",
            pointerEvents: "none",
            background: `linear-gradient(180deg, rgba(198,202,208,${cloudShade.toFixed(3)}) 0%, rgba(202,206,212,${(cloudShade * 0.88).toFixed(3)}) 58%, rgba(214,218,223,${(cloudShade * 0.5).toFixed(3)}) 100%)`,
          }} />
        )}

        {/* stars & planets are drawn AFTER the photo (below) so they overlay it
            like the Sun/Moon anchors — see the sky-object layer past the grid */}

        {/* photo/video — Look mode: our own canvas mesh warp (static texture) */}
        {!placing && photoOn && (
          <canvas ref={warpRef} style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }} />
        )}
        {photoHidden && (
          <div style={{ position: "absolute", left: "50%", top: "calc(56px + env(safe-area-inset-top))", transform: "translateX(-50%)", background: "rgba(15,23,42,.75)", border: "1px solid var(--line)", borderRadius: 999, padding: "4px 12px", fontSize: 11, fontFamily: "var(--mono)", color: "var(--dim)", pointerEvents: "none" }}>
            🖼 photo is off-view — pan toward {Math.round(poseNow.az)}° / {Math.round(poseNow.el)}°
          </div>
        )}
        {!placing && photoMarks && (
          <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }} preserveAspectRatio="none" viewBox="0 0 100 100">
            {photoMarks.wire && photoMarks.wire.map((seg, i) => (
              <polyline key={"w" + i} points={seg.map((p) => p.join(",")).join(" ")} fill="none"
                stroke={objCol} strokeWidth="1.2" opacity="0.9" vectorEffect="non-scaling-stroke" />
            ))}
            {photoMarks.a1 && photoMarks.a2 && !photoMarks.wire && (
              <line x1={photoMarks.a1[0]} y1={photoMarks.a1[1]} x2={photoMarks.a2[0]} y2={photoMarks.a2[1]}
                stroke={accentCol} strokeWidth="1.4" strokeDasharray="2 2" vectorEffect="non-scaling-stroke" />
            )}
            {!photoMarks.wire && [photoMarks.a1, photoMarks.a2].map((p, i) => p && <circle key={"a" + i} cx={p[0]} cy={p[1]} r="0.8" fill="none" stroke={accentCol} strokeWidth="1.4" vectorEffect="non-scaling-stroke" />)}
          </svg>
        )}

        {/* photo — Place mode: ONE pinned element. Image, marks, and frame are
            physically the same box, so they cannot diverge; the on-axis proof
            guarantees this rigid rectangle equals the projective truth. */}
        {placing && source?.mediaUrl && (
          <div style={{
            position: "absolute", left: (cx * 100) + "%", top: (cy * 100) + "%",
            width: (FRAMEz * 100) + "%",
            /* translate3d + willChange: keep the pinned photo on its OWN
               compositor layer — pairs with the promoted sky-vehicle overlay
               layer below to stop iOS-standalone ghost trails over the photo */
            transform: `translate3d(-50%,-50%,0) rotate(${-pRoll}deg)`,
            willChange: "transform, left, top",
            pointerEvents: "none",
          }}>
            {/* video shows the baked marked frame — the same still the warp uses */}
            {(() => {
              const imgSrc = source.mediaKind === "video" ? vidFrameUrl : source.mediaUrl;
              return imgSrc
                ? <img src={imgSrc} alt="" style={{ width: "100%", display: "block", opacity: PH_OP, filter: imgAdjFilter(source.imgAdj) }} />
                : <div style={{ width: "100%", aspectRatio: (source.natW && source.natH) ? `${source.natW} / ${source.natH}` : "16 / 9", background: "rgba(15,23,42,.6)", display: "flex", alignItems: "center", justifyContent: "center", color: "var(--dim)", fontSize: 11, fontFamily: "var(--mono)" }}><Spin style={{ marginRight: 6 }} />rendering frame</div>;
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
                      {source.A?.p1 && source.A?.p2 && <line x1={source.A.p1.x} y1={source.A.p1.y} x2={source.A.p2.x} y2={source.A.p2.y} stroke={accentCol} strokeWidth="2" strokeDasharray="5 5" vectorEffect="non-scaling-stroke" />}
                      {[source.A?.p1, source.A?.p2].map((p, i) => p && <circle key={"a" + i} cx={p.x} cy={p.y} r={source.natW / 160} fill="none" stroke={accentCol} strokeWidth="2" vectorEffect="non-scaling-stroke" />)}
                      {source.B?.pb && <circle cx={source.B.pb.x} cy={source.B.pb.y} r={source.natW / 160} fill="none" stroke="var(--teal)" strokeWidth="2" vectorEffect="non-scaling-stroke" />}
                    </>
                  );
                })()}
              </svg>
            )}
            <div style={{ position: "absolute", inset: 0, border: `1.5px dashed ${accentCol}`, boxSizing: "border-box" }} />
            <div style={{ position: "absolute", left: "50%", top: "50%", width: 14, height: 2, background: accentCol, transform: "translate(-50%,-50%)" }} />
            <div style={{ position: "absolute", left: "50%", top: "50%", width: 2, height: 14, background: accentCol, transform: "translate(-50%,-50%)" }} />
            {[["0%", "0%"], ["100%", "0%"], ["100%", "100%"], ["0%", "100%"]].map(([l, tp2], i) => (
              <div key={i} style={{ position: "absolute", left: l, top: tp2, width: 10, height: 10, margin: "-5px 0 0 -5px", borderRadius: "50%", background: accentCol }} />
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
          {bldgBoxes.map((b, i) => <path key={"bx" + i} d={b.d} fill="none" stroke={b.faint ? "rgba(255,178,74,0.5)" : "rgba(255,178,74,0.95)"} strokeWidth={b.faint ? "1" : "1.4"} strokeLinejoin="round" vectorEffect="non-scaling-stroke" />)}
        </svg>
        {terrainLbl && (
          <div style={{ position: "absolute", left: (terrainLbl.x * 100) + "%", top: (terrainLbl.y * 100) + "%", transform: "translate(-50%,-130%)", fontSize: 8.5, fontFamily: "var(--mono)", fontWeight: 700, letterSpacing: ".14em", color: ridgeCol(0.95), textShadow: "0 1px 2px rgba(0,0,0,.8)", pointerEvents: "none" }}>TERRAIN</div>
        )}
        {bldgLbl && (
          <div style={{ position: "absolute", left: (bldgLbl.x * 100) + "%", top: (bldgLbl.y * 100) + "%", transform: "translate(-50%,-130%)", fontSize: 8.5, fontFamily: "var(--mono)", fontWeight: 700, letterSpacing: ".14em", color: "rgba(255,178,74,0.98)", textShadow: "0 1px 2px rgba(0,0,0,.85)", pointerEvents: "none" }}>BUILDINGS</div>
        )}
        {/* winds-aloft profile — the balloon test made visual. Arrows point the
            way the wind PUSHES (drift) at each altitude, true-north-up; colour +
            number give speed. An object riding one of these is likely a balloon. */}
        {windDomeField.map((a, i) => a.label ? (
          <div key={"wl" + i} style={{ position: "absolute", left: (a.x * 100) + "%", top: (a.y * 100) + "%", transform: "translate(-102%,-50%)", fontSize: 7.5, fontFamily: "var(--mono)", fontWeight: 700, color: a.col, textShadow: "0 0 3px rgba(0,0,0,.95)", whiteSpace: "nowrap", pointerEvents: "none", zIndex: 199 }}>
            {isImperialUnits() ? `${(Math.round(a.alt * 3.28084 / 100) * 100).toLocaleString()} ft` : `${(a.alt / 1000).toFixed(1)} km`} · {fmtSpeedShort(a.spd)}
          </div>
        ) : (
          <div key={"wa" + i} style={{ position: "absolute", left: (a.x * 100) + "%", top: (a.y * 100) + "%", width: a.len, height: 9, marginTop: -4.5, transformOrigin: "0 4.5px", transform: `rotate(${a.ang}deg)`, pointerEvents: "none", zIndex: 199 }}>
            <svg width={a.len} height="9" style={{ overflow: "visible", filter: "drop-shadow(0 1px 1px rgba(0,0,0,.8))" }}>
              <line x1="0" y1="4.5" x2={a.len - 5} y2="4.5" stroke={a.col} strokeWidth="1.7" opacity="0.92" />
              <polygon points={`${a.len},4.5 ${a.len - 6},1.6 ${a.len - 6},7.4`} fill={a.col} opacity="0.92" />
            </svg>
          </div>
        ))}
        {windOn && windProf?.levels && (
          <div style={{ position: "absolute", left: 8, top: "calc(146px + env(safe-area-inset-top))", fontSize: 8, fontFamily: "var(--mono)", color: "#9fdcff", textShadow: "0 1px 2px rgba(0,0,0,.8)", pointerEvents: "none", zIndex: 200 }}>🎈 winds aloft — layers by height, arrow = drift · <span style={{ color: "var(--dim)" }}>heights schematic</span></div>
        )}
        {windOn && windProf?.err && (
          <div style={{ position: "absolute", left: 8, top: "calc(150px + env(safe-area-inset-top))", background: "rgba(7,11,20,.8)", border: "1px solid var(--amber)", borderRadius: 10, padding: "6px 8px", pointerEvents: "none", zIndex: 200, fontFamily: "var(--mono)", fontSize: 9, color: "var(--amber)", maxWidth: 150 }}>🎈 winds unavailable — {windProf.err}</div>
        )}
        {/* named peaks on the skyline — el from the summit's own height, or the
            drawn ridge elevation when OSM has no `ele` tag */}
        {peakDraw.map(({ pk, pr, row }, i) => {
          const lift = row * 26; // tier the labels well apart so names don't collide
          // summit ELEVATION (MSL) in the user's unit — NOT via fmtLenShort, which
          // rolls >5280 ft into miles and made "Middle Sister 1.90 mi" (its height!)
          const ele = pk.eleM != null ? (isImperialUnits() ? Math.round(pk.eleM * 3.28084).toLocaleString() + " ft" : Math.round(pk.eleM).toLocaleString() + " m") : null;
          return (
            <div key={"pk" + i} style={{ position: "absolute", left: (pr.x * 100) + "%", top: (pr.y * 100) + "%", transform: "translate(-50%,-100%)", display: "flex", flexDirection: "column", alignItems: "center", pointerEvents: "none" }}>
              <div style={{ lineHeight: 1.05, textAlign: "center" }}>
                <div style={{ fontSize: 8.5, fontFamily: "var(--mono)", fontWeight: 700, color: ridgeCol(0.98), textShadow: "0 0 3px rgba(0,0,0,.95), 0 1px 2px rgba(0,0,0,.9)", whiteSpace: "nowrap" }}>{pk.name}</div>
                {ele && <div style={{ fontSize: 7.5, fontFamily: "var(--mono)", color: ridgeCol(0.78), textShadow: "0 0 3px rgba(0,0,0,.95)", whiteSpace: "nowrap" }}>{ele}</div>}
              </div>
              {lift > 0 && <div style={{ width: 1, height: lift, background: ridgeCol(0.5) }} />}
              <div style={{ width: 0, height: 0, marginTop: 1, borderLeft: "3px solid transparent", borderRight: "3px solid transparent", borderBottom: `5px solid ${ridgeCol(0.98)}` }} />
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

        {/* SKY-VEHICLE OVERLAYS (satellites · Starlink · aircraft) — one
            promoted compositor layer. iOS STANDALONE (home-screen web app)
            mode has a partial-invalidation bug: these chips/trails repaint on
            every placement drag, and where they cross the composited photo
            layer the stale rasters are never cleared — field report: dozens
            of ghost satellite chips smeared inside the photo rect, PWA only
            (a Safari tab renders the same build clean). translateZ(0) gives
            the whole moving stack its own layer, recomposited wholesale
            instead of partially repainted — the standard WebKit ghosting
            cure. */}
        <div style={{ position: "absolute", inset: 0, pointerEvents: "none", transform: "translateZ(0)" }}>
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

        {/* Starlink pass trails — violet dotted paths (±4 min), drawn only for
            the train members near what you're looking at OR near the sight-line
            (a full 60× would clutter the dome), so the "string of lights" shows
            its trajectory across the sky. Same shape as the satellite/aircraft
            trails, in the Starlink violet. */}
        {slView.some((s) => s.trail) && (() => {
          const sight = isNum(source?.A?.az) && isNum(source?.A?.el) ? dirFromAzEl(+source.A.az, +source.A.el) : null;
          const look = dirFromAzEl(effAz, effAlt);
          const near = (r, d) => r && Math.acos(clampN(dot(r, d), -1, 1)) * R2D <= 30;
          const lines = [];
          for (const s of slView) {
            if (!s.trail) continue;
            const d = dirFromAzEl(s.az, s.el);
            if (!near(look, d) && !near(sight, d)) continue;
            const segs = []; let seg = [];
            for (const q of s.trail) {
              const pr = projectD(dirFromAzEl(q.az, q.el));
              if (pr.inFront && q.el > -1 && pr.x > -0.2 && pr.x < 1.2 && pr.y > -0.2 && pr.y < 1.2) seg.push(`${(pr.x * (vp.w || 1)).toFixed(1)},${(pr.y * (vp.h || 1)).toFixed(1)}`);
              else { if (seg.length > 1) segs.push(seg); seg = []; }
            }
            if (seg.length > 1) segs.push(seg);
            segs.forEach((sg, k) => lines.push({ pts: sg.join(" "), key: s.name + k }));
            if (lines.length > 14) break;
          }
          if (!lines.length) return null;
          return (
            <svg style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }}>
              {lines.map((l) => (
                <polyline key={l.key} points={l.pts} fill="none" stroke="#c9b6ff"
                  strokeWidth="1.6" strokeLinecap="round" strokeDasharray="0.1 5" opacity="0.72" />
              ))}
            </svg>
          );
        })()}

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
        </div>

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
              {selInfo?.busy && <div style={{ color: "var(--dim)", fontSize: 11 }}><Spin style={{ marginRight: 6 }} />looking up route</div>}
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

        {/* wizard trajectory — world-anchored points (may run past the photo).
            Shown ONLY while the ⊕ Trajectory tool is active: outside it the
            numbered chips + dashed path just clutter the world view (the
            object itself is represented by the wireframe overlay). */}
        {(() => {
          /* also shown while ⚓ Fix frames is active (gated by the 🛸 toggle):
             the preview path feeds the conversion, so the trajectory slides
             LIVE as you drag a frame back onto the horizon */
          if (!source || !(trajOn || (fixOn && objOn))) return null;
          /* SKY PATH, not frame path: on a stabilized video each waypoint was
             tapped on ITS OWN frame, so its direction must go through THAT
             frame's solved pose — converting them all through the single
             placement pose (trackDirections' "camera never moved" assumption)
             draws where the object sat IN THE FRAME, which a panning camera
             makes meaningless. With a posePath, build the dirs per-frame;
             everything else (stills, unstabilized clips) keeps the old path. */
          let dirs = null;
          const ppSrc = (fixOn && fixPreview) || source.posePath;
          const pp2 = source.mediaKind === "video" && Array.isArray(ppSrc) && ppSrc.length > 1 ? ppSrc : null;
          if (pp2 && source.natW) {
            const ptsT = [...(source.track || [])].filter((q) => isNum(q.t) && isNum(q.x) && isNum(q.y)).sort((a, b) => a.t - b.t);
            const conv = ptsT.map((q) => {
              const ps2 = posePathAt(pp2, +q.t);
              return ps2 && isNum(ps2.az) ? { d: pixToDirK(+q.x, +q.y, source.natW, source.natH, ps2.az, ps2.el, ps2.roll || 0, ps2.fov, ps2.k || 0) } : null;
            });
            if (conv.filter(Boolean).length >= 2) dirs = conv.filter(Boolean);
          }
          if (!dirs) dirs = trackDirections(source);
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
                /* fix mode (trajectory tool closed): just small dots on the
                   waypoints — the sliding polyline is the live feedback, the
                   numbered chips + wireframes would clutter and cost frames */
                if (!trajOn) return ps.map((p, i) => (p && !dirs[i].virt ? (
                  <div key={"fx" + i} style={{ position: "absolute", left: (p[0] * 100) + "%", top: (p[1] * 100) + "%", transform: "translate(-50%,-50%)", width: 6, height: 6, borderRadius: "50%", background: "var(--track)", opacity: 0.85, pointerEvents: "none" }} />
                ) : null));
                let n = -1;
                return ps.map((p, i) => {
                  if (dirs[i].virt) return null;
                  n++;
                  if (!p) return null;
                  const idx = n, sel = selPt === idx;
                  /* the trajectory is laid down on the MEASURE step now (for both
                     stills and video), so the world view is READ-ONLY — points
                     are shown for reference, never tappable here. */
                  const tappable = false;
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

        {/* aiming crosshair — sits at the projection centre (cx,cy). Only the
            tools that actually AIM with it still show it: ⚖ Compare (drop a ghost
            at the centre) and ✦ manual star-align (centre a star). Trajectory
            moved to the measure step, so the plain look/trajectory view no longer
            needs a reticle. */}
        {pMode !== "place" && (cmpOn || calibOn) && (
        <svg style={{ position: "absolute", left: (cx * 100) + "%", top: (cy * 100) + "%", transform: "translate(-50%,-50%)", pointerEvents: "none", overflow: "visible", opacity: 0.75 }} width="48" height="48" viewBox="-32 -32 64 64">
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
      <div ref={topBarRef} style={{ position: "absolute", top: 0, left: 0, right: 0, padding: "calc(10px + env(safe-area-inset-top)) 12px 10px", pointerEvents: "none", zIndex: 210 }}>
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
          <div style={{ pointerEvents: "auto", flex: "0 0 auto", display: "flex", gap: 6 }}>
            <button className="btn sm" title={hdrMin ? "Show the sky-layer toggles" : "Hide the sky-layer toggles for a cleaner view"}
              style={{ background: "rgba(15,23,42,.7)", padding: "6px 9px" }} onClick={() => setHdrMin((v) => !v)}>{hdrMin ? "⌄" : "⌃"}</button>
            <HelpButton section="sky" style={{ background: "rgba(15,23,42,.7)" }} />
          </div>
        </div>
        {!hdrMin && (<>
        {/* sky-layer toggles — one row (terrain is always on) */}
        <div style={{ display: "flex", gap: 6, marginTop: 8, pointerEvents: "auto", flexWrap: "wrap" }}>
          {/* celestial group first (sun · moon · stars · satellites · Starlink),
             then air traffic, then terrain, then weather */}
          {sun.alt > -1 && <button className="btn sm" title={`Sun ${fmtBody(sun)} — tap to center`} style={{ background: "rgba(15,23,42,.7)", padding: "6px 9px" }} onClick={() => recenter(sun)}>☀</button>}
          {moon.alt > -1 && <button className="btn sm" title={`Moon ${fmtBody(moon)} · ${Math.round(moon.frac * 100)}% lit — tap to center`} style={{ background: "rgba(15,23,42,.7)", padding: "6px 9px" }} onClick={() => recenter(moon)}>☾</button>}
          <button className="btn sm" title="Stars & planets: auto on at night, off by day — tap to force on/off"
            style={{ background: "rgba(15,23,42,.7)", padding: "6px 9px", color: starMode === "off" ? "var(--dim)" : (starMode === "on" || limMag > -4) ? "#dfe8ff" : "var(--dim)" }}
            onClick={() => setStarMode((m) => (m === "auto" ? "on" : m === "on" ? "off" : "auto"))}>
            ★
          </button>
          {source?.A?.p1 && source?.A?.p2 && (
            <button className="btn sm" title="Object overlay: the fitted 3D wireframe + marks over the photo — rides the tracked path during stabilized playback, and is burned into exports only while ON"
              style={{ background: "rgba(15,23,42,.7)", padding: "6px 9px", color: objOn ? accentCol : "var(--dim)" }}
              onClick={() => setObjOn((v) => !v)}>🛸</button>
          )}
          {hasPos && (
            <button className="btn sm" title="Satellites (CelesTrak visual group, SGP4 at the sighting time): auto shows when dark; on forces"
              style={{ background: "rgba(15,23,42,.7)", color: satMode === "off" ? "var(--dim)" : satView.length ? "#9fdcff" : "var(--dim)" }}
              onClick={() => setSatMode((m) => (m === "auto" ? "on" : m === "on" ? "off" : "auto"))}>
              🛰 {satMode === "off" ? "off" : satDb?.err ? "?" : satsWanted && !satDb ? <Spin /> : `${satView.length}${satMode === "auto" ? "" : " on"}`}
            </button>
          )}
          {hasPos && (
            <button className="btn sm" title="Starlink — the full constellation (opt-in). Sunlit Starlinks above the horizon at the sighting time; a fresh batch appears as a tight train. Trains near your view show their ±4-min pass trajectory."
              style={{ background: "rgba(15,23,42,.7)", color: !starlinkOn ? "var(--dim)" : slDb?.err ? "var(--amber)" : "#c9b6ff" }}
              onClick={() => setStarlinkOn((v) => !v)}>
              ✦ {starlinkOn ? (slDb?.err ? "?" : !slDb ? <Spin /> : `Starlink ${slView.length}`) : "Starlink"}
            </button>
          )}
          {hasPos && (
            <button className="btn sm" style={{ background: "rgba(15,23,42,.7)", color: !acOn ? "var(--dim)" : (acData?.ac && wantHist && !acData.hist) ? "var(--amber)" : "var(--track)" }}
              onClick={() => setAcOn((v) => !v)}>
              ✈ {acOn ? (acData?.ac ? `${acView.length}${acData.hist ? " @ sighting" : " live"}` : acData?.err ? "?" : <Spin />) : "off"}
            </button>
          )}
          {hasPos && (
            <button className="btn sm" title="Named peaks (OpenStreetMap) placed on the terrain skyline — a labeled summit on the horizon is also a compass check"
              style={{ background: "rgba(15,23,42,.7)", color: !peaksOn ? "var(--dim)" : peaks?.err ? "var(--amber)" : "rgba(158,224,138,0.95)" }}
              onClick={() => setPeaksOn((v) => !v)}>
              ⛰ {peaksOn ? (peaks?.err ? "?" : !peaks ? <Spin /> : `peaks ${peakDraw.length}${peakInView.length > peakDraw.length ? ` of ${peakInView.length}` : ""}`) : "peaks"}
            </button>
          )}
          {hasPos && (
            <button className="btn sm" title="Buildings — each OSM footprint drawn as its own amber wireframe box (roof outline + corner edges), for aligning a photo shot in town. Untagged footprints use an assumed height; it does NOT drive Snap to ridges."
              style={{ background: "rgba(15,23,42,.7)", color: !bldgOn ? "var(--dim)" : bldg?.err ? "var(--amber)" : "rgba(255,178,74,0.95)" }}
              onClick={() => setBldgOn((v) => !v)}>
              🏙 {bldgOn ? (bldg?.err ? "?" : !bldg?.boxes ? <Spin /> : bldg?.buildings ? `${bldg.buildings.shown} bldgs` : "on") : "buildings"}
            </button>
          )}
          {hasPos && (
            <button className="btn sm" title="Cloud cover at the sighting time (Open-Meteo) — a grey sky shading scaled by % cover (low/mid/high) on the dome, plus the estimated cloud base. A low deck caps a below-cloud object's range & size."
              style={{ background: "rgba(15,23,42,.7)", color: !cloudOn ? "var(--dim)" : wxSky?.err ? "var(--amber)" : "#cfe0ee" }}
              onClick={() => setCloudOn((v) => !v)}>
              ☁ {cloudOn ? (wxSky?.err ? "?" : !wxSky ? <Spin /> : (isNum(wxSky.cloud) ? `${Math.round(wxSky.cloud)}%` : "on")) : "cloud"}
            </button>
          )}
          {hasPos && (
            <button className="btn sm" title="Winds aloft at the sighting time — a vertical profile of which way and how fast the wind pushes at each altitude. A balloon drifts WITH the wind at its altitude, so compare the object's motion to these."
              style={{ background: "rgba(15,23,42,.7)", color: !windOn ? "var(--dim)" : windProf?.err ? "var(--amber)" : "#9fdcff" }}
              onClick={() => setWindOn((v) => !v)}>
              🎈 {windOn ? (windProf?.err ? "?" : !windProf?.levels ? <Spin /> : "wind") : "wind"}
            </button>
          )}
        </div>
        {satView.length > 0 && satStaleDays > 5 && (
          <div style={{ fontSize: 10, color: "var(--amber)", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 4 }}>
            🛰 TLE epoch ≈ {satStaleDays} d from the sighting — satellite positions degrade; treat as approximate
          </div>
        )}
        {cloudOn && (
          <div style={{ fontSize: 10, color: wxSky?.err ? "var(--amber)" : "#cfe0ee", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 4 }}>
            {wxSky?.err ? "☁ cloud data unavailable for this time/place — toggle off/on to retry"
              : !wxSky ? "☁ fetching cloud cover…"
                : `☁ ${isNum(wxSky.cloud) ? Math.round(wxSky.cloud) + "% cover" : "cover"}${[isNum(wxSky.low) ? "low " + Math.round(wxSky.low) + "%" : "", isNum(wxSky.mid) ? "mid " + Math.round(wxSky.mid) + "%" : "", isNum(wxSky.high) ? "high " + Math.round(wxSky.high) + "%" : ""].filter(Boolean).length ? " (" + [isNum(wxSky.low) ? "low " + Math.round(wxSky.low) + "%" : "", isNum(wxSky.mid) ? "mid " + Math.round(wxSky.mid) + "%" : "", isNum(wxSky.high) ? "high " + Math.round(wxSky.high) + "%" : ""].filter(Boolean).join(" · ") + ")" : ""}${isNum(wxSky.baseAGL) && ((isNum(wxSky.low) ? wxSky.low : wxSky.cloud) || 0) >= 40 ? " · base ≈ " + fmtLenShort(wxSky.baseAGL) + " AGL" : ""} — sky shading by % cover`}
          </div>
        )}
        {peaksOn && peaks?.err && (
          <div style={{ fontSize: 10, color: "var(--amber)", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 4 }}>
            ⛰ peaks unavailable — {/busy|timed|502/i.test(peaks.err) ? "Overpass was busy. Toggle ⛰ off/on to retry." : peaks.err}
          </div>
        )}
        {peaksOn && Array.isArray(peaks) && peaks.length === 0 && (
          <div style={{ fontSize: 10, color: "var(--dim)", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 4 }}>
            ⛰ no named peaks or hills within 120 km of {LAT.toFixed(3)}, {LNG.toFixed(3)} — if you expected some, Overpass may be busy; toggle ⛰ off/on to retry
          </div>
        )}
        {bldgOn && bldg?.buildings && (
          <div style={{ fontSize: 10, color: bldg.buildings.shown === 0 ? "var(--amber)" : "var(--dim)", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 4 }}>
            {bldg.buildings.n === 0
              ? `🏙 no buildings returned near ${LAT.toFixed(3)}, ${LNG.toFixed(3)} — either OSM has none mapped here, or Overpass was busy. Toggle 🏙 off/on to retry.`
              : bldg.buildings.shown === 0
                ? `🏙 ${bldg.buildings.n} footprint${bldg.buildings.n === 1 ? "" : "s"} nearby but none in the ${fmtLenShort(12)}–${fmtLenShort(BLDG_RADIUS_M)} draw range`
                : `🏙 ${bldg.buildings.shown} building${bldg.buildings.shown === 1 ? "" : "s"}${bldg.buildings.n > bldg.buildings.shown ? ` (nearest of ${bldg.buildings.n})` : ""} — ${bldg.buildings.known + bldg.buildings.est} extruded (real height)${bldg.buildings.assumed ? `, ${bldg.buildings.assumed} as ground footprints (OSM has no height)` : ""}`}
          </div>
        )}
        {bldgOn && bldg?.buildings && bldg.buildings.shown > 0 && (() => {
          const step = isImperialUnits() ? 0.3048 : 1, stepLbl = isImperialUnits() ? "1 ft" : "1 m";
          return (
          <div style={{ display: "flex", alignItems: "center", gap: 6, flexWrap: "wrap", fontSize: 10, color: "var(--dim)", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 3, pointerEvents: "auto" }}>
            <span style={{ color: "rgba(255,178,74,0.9)" }}>📷 camera ≈ {fmtLenShort(camH)} up {camH > 2 ? `(≈${storeys(camH)} fl)` : ""}</span>
            <span>{isNum(source?.camH) ? "(set by hand)" : autoCamH != null ? "(GPS alt − terrain — nudge if rooftops sit wrong)" : "(assumed eye height — no GPS altitude in photo)"}</span>
            <button className="btn sm" style={{ padding: "1px 8px", pointerEvents: "auto" }} onClick={() => update({ camH: +clampN(camH - step, 1.6, 300).toFixed(2) })}>−{stepLbl}</button>
            <button className="btn sm" style={{ padding: "1px 8px", pointerEvents: "auto" }} onClick={() => update({ camH: +clampN(camH + step, 1.6, 300).toFixed(2) })}>+{stepLbl}</button>
            {isNum(source?.camH) && autoCamH != null && <button className="btn sm" style={{ padding: "1px 8px", pointerEvents: "auto" }} onClick={() => update({ camH: null })}>auto</button>}
          </div>
          );
        })()}
        {bldgOn && bldg?.err && (
          <div style={{ fontSize: 10, color: "var(--amber)", textShadow: "0 1px 2px rgba(0,0,0,.7)", marginTop: 4 }}>
            🏙 buildings unavailable — {bldg.err}
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
        </>)}
      </div>

      {/* view zoom — vertical stack on the right, out of the cramped bottom bar */}
      {pMode !== "place" && (
        <div style={{ position: "absolute", right: 10, top: ctrlBandPct + "%", transform: "translateY(-50%)", display: "flex", flexDirection: "column", gap: 6, zIndex: 205, pointerEvents: "auto" }}>
          <button className="btn" style={{ width: 42, height: 42, padding: 0, fontSize: 19, background: "rgba(15,23,42,.75)" }} onClick={() => setFov((f) => clampN(+(f * 0.72).toFixed(1), 2, 90))}>+</button>
          <button className="btn" style={{ width: 42, height: 42, padding: 0, fontSize: 19, background: "rgba(15,23,42,.75)" }} onClick={() => setFov((f) => clampN(+(f / 0.72).toFixed(1), 2, 90))}>−</button>
          {/* fix mode: one finger normally drags the PHOTO — ✋ flips it to
              panning the view (like place mode's ✋), so a zoomed-in horizon
              can be reached without disturbing the frame pose */}
          {fixOn && (
            <button className={"btn" + (panMode ? " amber" : "")} title="Pan mode: drag looks around the view instead of moving the photo"
              style={{ width: 42, height: 42, padding: 0, fontSize: 17, background: panMode ? undefined : "rgba(15,23,42,.75)" }}
              onClick={() => setPanMode((v) => !v)}>✋</button>
          )}
        </div>
      )}

      {/* place-mode DISPLAY zoom + pan — magnify photo+sky together to line up a
          ridge/rooftop, then ✋ pan to reach it. Does not change the calibration. */}
      {placing && (
        <div style={{ position: "absolute", right: 10, top: ctrlBandPct + "%", transform: "translateY(-50%)", display: "flex", flexDirection: "column", gap: 6, zIndex: 206, pointerEvents: "auto" }}>
          <button className="btn" title="Zoom in (magnify for fine alignment)" style={{ width: 42, height: 42, padding: 0, fontSize: 19, background: "rgba(15,23,42,.8)" }}
            onClick={() => setPZoom((z) => clampN(+(z * 1.5).toFixed(2), 1, 6))}>+</button>
          <button className="btn" title="Zoom out" style={{ width: 42, height: 42, padding: 0, fontSize: 19, background: "rgba(15,23,42,.8)" }}
            onClick={() => setPZoom((z) => { const nz = clampN(+(z / 1.5).toFixed(2), 1, 6); if (nz <= 1.001) { setPPan({ x: 0, y: 0 }); setPanMode(false); } return nz; })}>−</button>
          {/* pan button + zoom readout keep their slots ALWAYS (visibility only) so
             appearing after a zoom doesn't shift the +/− buttons up */}
          <button className={"btn" + (panMode ? " amber" : "")} title="Pan mode: drag to move around the magnified view (instead of sliding the sky)"
            style={{ width: 42, height: 42, padding: 0, fontSize: 17, background: panMode ? undefined : "rgba(15,23,42,.8)", visibility: pZoom > 1.001 ? "visible" : "hidden" }}
            onClick={() => setPanMode((v) => !v)}>✋</button>
          <div style={{ textAlign: "center", fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)", textShadow: "0 1px 2px rgba(0,0,0,.8)", visibility: pZoom > 1.001 ? "visible" : "hidden" }}>{pZoom.toFixed(1)}×</div>
        </div>
      )}

      {/* bottom controls */}
      <div ref={botBarRef} style={{ position: "absolute", left: 0, right: 0, bottom: 0, padding: "10px 12px calc(12px + env(safe-area-inset-bottom))", background: "linear-gradient(0deg, rgba(7,11,20,.92) 55%, rgba(7,11,20,0))", zIndex: 210 }}>
        {(motionMsg || cameraMsg) && <div className="warn" style={{ marginBottom: 8, marginTop: 0 }}>{motionMsg || cameraMsg}</div>}
        {source?.mediaUrl && photoOn && (
          <>
            {!botMin && (<>
            {/* MODE BAR — one line, one mode at a time: place · trajectory · size · compare.
               The overlay-color SLIDER hides behind the swatch at the end of
               the row (shows the current color) — a full-time slider row was
               pure vertical cost. */}
            {!single && !calibOn && (
              <div style={{ display: "flex", gap: 6, marginBottom: 8, alignItems: "center" }}>
                {[["place", "✥ Place", pMode === "place"], ["traj", "⊕ Trajectory", trajOn], ["size", "📏 Size", sizeOn], ["compare", "⚖ Compare", cmpOn]].map(([k, label, on]) => (
                  <button key={k} className={"btn sm" + (on ? " amber" : "")} style={{ flex: "1 1 0", minWidth: 0, whiteSpace: "nowrap", padding: "6px 2px", fontSize: 11, overflow: "hidden" }}
                    onClick={() => selectMode(k)}>{label}</button>
                ))}
                <button title="Accent color — recolors the crosshair, marks and terrain ridge lines so they stand out against your photo (the object keeps its own colour, set on the measure step). Tap to open the hue slider."
                  onClick={() => setHueOpen((v) => !v)}
                  style={{ width: 22, height: 22, borderRadius: 11, flex: "0 0 auto", padding: 0, background: accentCol, border: hueOpen ? "2px solid #fff" : "1px solid rgba(255,255,255,.35)" }} />
              </div>
            )}
            {!single && !calibOn && hueOpen && (
              <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 8 }} title="Recolor the crosshair, marks and terrain ridges (the object keeps its own colour from the measure step)">
                <span className="microlabel" style={{ marginBottom: 0 }}>color</span>
                <input type="range" min={0} max={360} step={2} value={ridgeHue} onChange={(e) => setRidgeHue(+e.target.value)} style={{ flex: 1 }} />
                <button className="btn sm" style={{ padding: "2px 8px" }} onClick={() => setHueOpen(false)}>✓</button>
              </div>
            )}
            {single && (
              <div style={{ marginBottom: 8 }}>
                <button className={"btn sm" + (pMode === "place" ? " amber" : "")} onClick={() => (pMode === "place" ? donePlace() : enterPlace())}>{pMode === "place" ? "✓ Done placing" : "✥ Place photo"}</button>
              </div>
            )}
            {/* manual star align is a Place sub-flow that runs in look mode (needs the free crosshair) */}
            {calibOn && (
              <div style={{ display: "flex", gap: 6, marginBottom: 8, alignItems: "center" }}>
                <span style={{ fontFamily: "var(--mono)", fontSize: 11, color: "var(--amber)", flex: 1 }}>✦ Manual star align</span>
                {calibApplied && <button className="btn sm" onClick={resetCalib} title="Undo the star alignment — restore the lens FOV & roll">↺ reset</button>}
                <button className="btn sm amber" onClick={exitCalib}>✓ Done aligning</button>
              </div>
            )}
            {/* PLACE sub-tools */}
            {pMode === "place" && !calibOn && (
              <div style={{ display: "flex", gap: 6, flexWrap: "wrap", marginBottom: 8, alignItems: "center" }}>
                <button className="btn sm" disabled={!placeUndo.length} style={{ opacity: placeUndo.length ? 1 : 0.4 }} title="Undo the last placement change (a slip won't cost you the whole alignment)" onClick={undoPlace}>↩ Undo</button>
                <button className="btn sm amber" onClick={() => { pushUndo(snapPose()); autoAlign(); }} title="Auto star-align: detects the stars in the photo and matches their pattern to the sky to solve the exact pose (az/el/roll/FOV/lens) — no need to line it up first. Needs correct date/time & location.">✦ Auto star align</button>
                {terr?.els && <button className="btn sm teal" onClick={() => { pushUndo(snapPose()); snapToRidges(); }}>⛰ Snap to ridges</button>}
                {skyRefs.length > 0 && (
                  <button className="btn sm" onClick={startManualAlign} title="Manual star align: pick a named star or planet, aim the crosshair on it in the photo, ✓ Set — solves the lens FOV + roll. Drops to look mode so you can aim.">✦ Manual star align</button>
                )}
                <button className="btn sm" onClick={() => {
                  pushUndo(snapPose());
                  const p0 = openPoseRef.current;
                  if (p0) { setPAz(p0.az); setPEl(p0.el); setPRoll(p0.roll); setFovM(p0.fov); setPDist(p0.dist || 0); }
                  else { setFovM(isNum(source?.fovH) ? +source.fovH : 68); setPRoll(0); setPDist(0); }
                  calibAnchorsRef.current = []; setCalibCount(0); resetPlaceView();
                }}>Reset placement</button>
                {calibApplied && <button className="btn sm" onClick={resetCalib} title="Undo the star alignment — restore the lens FOV & roll">↺ align</button>}
                <button className="btn sm" style={fineOn ? { borderColor: "var(--amber)", color: "var(--amber)" } : undefined}
                  onClick={() => setFineOn((o) => !o)}
                  title="Fine tune: precise roll/FOV nudge buttons — the two-finger twist and pinch bleed into each other at small adjustments">🎛</button>
                {/* FINE TUNE row (field ask): twist and pinch are great for the
                    initial rough placement but bleed into each other when fine
                    tuning — these nudge exactly one axis per tap. Behind a
                    toggle so the cramped place screen stays clean by default. */}
                {fineOn && (
                  <div style={{ display: "flex", gap: 6, alignItems: "center", width: "100%" }}>
                    <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)" }}>roll</span>
                    <button className="btn sm" title="Roll −0.2°" onClick={() => { pushUndo(snapPose()); calibRecRef.current = null; setPRoll((r) => clampN(r - 0.2, -90, 90)); }}>⟲</button>
                    <button className="btn sm" title="Roll +0.2°" onClick={() => { pushUndo(snapPose()); calibRecRef.current = null; setPRoll((r) => clampN(r + 0.2, -90, 90)); }}>⟳</button>
                    <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)", marginLeft: 8 }}>fov</span>
                    <button className="btn sm" title="FOV −0.3° (photo covers less sky)" onClick={() => { pushUndo(snapPose()); calibRecRef.current = null; setFovM((f) => clampN(f - 0.3, 12, 120)); }}>−</button>
                    <button className="btn sm" title="FOV +0.3° (photo covers more sky)" onClick={() => { pushUndo(snapPose()); calibRecRef.current = null; setFovM((f) => clampN(f + 0.3, 12, 120)); }}>＋</button>
                  </div>
                )}
                {/* the ALIGNMENT frame is chosen on the MEASURE step (⛰ Align
                    on this frame — cheap scrubbing there; a scrubber here
                    re-rendered the whole dome per tick and crawled). This line
                    just says which frame the alignment describes. */}
                {source.mediaKind === "video" && (
                  <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)", width: "100%" }}
                    title="The frame this alignment describes — change it on the measure step (⛰ Align on this frame)">
                    🎞 aligning on {alignT.toFixed(2)}s{Math.abs(markT - alignT) > 0.1 ? ` · object on ${markT.toFixed(2)}s` : ""}
                  </span>
                )}
                <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--amber)", width: "100%" }}>
                  → {pAz.toFixed(1)}° az · {pEl.toFixed(1)}° up · FOV {fovM.toFixed(1)}° · roll {pRoll.toFixed(1)}°
                </span>
              </div>
            )}
            {source.mediaKind === "video" && (
              <div style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)", marginBottom: 8 }}>
                {/* one row: the stabilize button with its frame/status info
                    tucked to the RIGHT at button height — the full-width text
                    block above the button cost two lines of vertical space */}
                <div style={{ display: "flex", gap: 8, alignItems: "center" }}>
                  {/* stabilize LIVES IN LOOK MODE — running it from place mode made
                      it too easy to nudge the placement mid-solve (field report) */}
                  {pMode !== "place" && !calibOn && !trajOn && !sizeOn && !cmpOn && (
                    <button className="btn sm teal" disabled={!!stabBusy} style={{ opacity: stabBusy ? 0.6 : 1, flex: "0 0 auto" }}
                      title="Stabilize: track the static background (skyline, stars) through every frame and solve each frame's camera pose. Then ▶ play — the sky stays locked to the dome and only the object moves. Align the marked frame first (place mode: snap/star-align) for an accurate result."
                      onClick={stabilize}>{stabBusy ? `🎞 ${stabBusy}${stabTotal ? `/${stabTotal}` : ""}…` : Array.isArray(source?.posePath) && source.posePath.length > 1 ? "🎞 Re-stabilize" : "🎞 Stabilize video"}</button>
                  )}
                  {/* ↶ UNDO — a bad run is otherwise unrecoverable. Restores the
                      snapshot taken before the last run (anchors included), or
                      clears the solve when there is no snapshot, which now lands
                      on the original clip rather than a dead screen. */}
                  {pMode !== "place" && !calibOn && !trajOn && !sizeOn && !cmpOn && !stabBusy && Array.isArray(source?.posePath) && source.posePath.length > 1 && (
                    <button className="btn sm" style={{ flex: "0 0 auto" }}
                      title={source?.preStab
                        ? (source.preStab.posePath && source.preStab.posePath.length > 1
                          ? "Undo the last stabilize — back to the previous solve, with its ⚓ anchors and object track"
                          : "Undo the stabilize — back to the original clip, before any solve")
                        : "Remove the stabilization — back to the original clip. (Recorded before undo existed, so there is no earlier solve to return to.)"}
                      onClick={undoStabilize}>↶ Undo</button>
                  )}
                  {/* MANUAL solve: only when the clip has hand-marked camera refs
                      (the auto pass couldn't do it) — solves instantly from marks,
                      then a smoothing slider pops up to tune the result live */}
                  {pMode !== "place" && !calibOn && !trajOn && !sizeOn && !cmpOn && !stabBusy && (source?.camRefs || []).some((r) => (r.marks || []).filter((m) => isNum(m.x)).length) && (
                    <button className="btn sm green" style={{ flex: "0 0 auto" }}
                      title="Solve each frame's pose from your hand-marked camera references (Cam refs on the measure step) — the fallback when the automatic stabilizer can't lock on"
                      onClick={() => { setMSolveOpen(true); solveFromMarks(mSmooth); }}>🎯 Solve from marks</button>
                  )}
                  {/* hidden while placing — the place tools' "aligning on" line
                      already says which frame matters there (screen space) */}
                  {pMode !== "place" && (
                    <span style={{ flex: 1, minWidth: 0, fontSize: 9.5, lineHeight: 1.35 }}>
                      🎞 frame {isNum(source?.A?.videoTime) ? (+source.A.videoTime).toFixed(2) + "s" : "start"} (set on the measure step)
                      {Array.isArray(source?.posePath) && source.posePath.length > 1 && <span style={{ color: "var(--teal)" }}> · stabilized: {source.posePath.length} frames</span>}
                    </span>
                  )}
                </div>
                {/* SMOOTHING slider popup — appears after Solve from marks, re-
                    solves live as you drag (the manual solve is instant), then
                    Done dismisses it. Averages out imperfect placement. */}
                {mSolveOpen && pMode !== "place" && !calibOn && !trajOn && !sizeOn && !cmpOn && (source?.camRefs || []).some((r) => (r.marks || []).filter((m) => isNum(m.x)).length) && (
                  <div style={{ marginTop: 6, padding: "8px 10px", border: "1px solid var(--green)", borderRadius: 10, background: "rgba(90,200,140,.08)", display: "flex", alignItems: "center", gap: 10, flexWrap: "wrap" }}>
                    <span style={{ fontSize: 11, color: "var(--track)", whiteSpace: "nowrap" }}>Smoothing <b style={{ fontFamily: "var(--mono)" }}>{mSmooth}%</b></span>
                    <input type="range" min="0" max="100" step="5" value={mSmooth} style={{ flex: 1, minWidth: 120, accentColor: "var(--green)" }}
                      onChange={(e) => { const v = +e.target.value; setMSmooth(v); solveFromMarks(v); }} />
                    <button className="btn sm green" style={{ flex: "0 0 auto" }} onClick={() => setMSolveOpen(false)}>✓ Done</button>
                    <span style={{ width: "100%", fontSize: 9.5, color: "var(--dim)" }}>drag to average out imperfect placement — ▶ play to see the effect. Higher = smoother but flattens real motion.</span>
                  </div>
                )}
                {/* the trajectory guide can only be laid down on the MEASURE
                    step (frame-accurate scrubbing exists there before any
                    stabilization) — point back when it's missing */}
                {pMode !== "place" && !calibOn && source?.A?.p1 && (source.track || []).filter((p) => isNum(p.t) && isNum(p.x)).length < 2 && (
                  <div style={{ marginTop: 4 }}>
                    tip: object moves? ‹ Back to the measure step → <b style={{ color: "var(--track)" }}>Track</b>: scrub the clip and tap the object at a few moments — those taps guide the tracker here
                  </div>
                )}
                {/* the alignment belongs to ONE frame: if the marks moved to a
                    different frame after placing, the pose no longer describes
                    the baked frame — say so instead of silently stabilizing
                    from a stale anchor */}
                {source?.placed && isNum(source?.calib?.vt) && Math.abs(+source.calib.vt - alignT) > 0.1 && (
                  <div style={{ color: "var(--amber)", marginTop: 4 }}>
                    ⚠ aligned on frame {(+source.calib.vt).toFixed(2)}s but the align frame is now {alignT.toFixed(2)}s — re-align (✥ Place) before stabilizing
                  </div>
                )}
              </div>
            )}
            </>)}
            {/* world-locked playback — each frame drawn at ITS solved pose; the
                sky/terrain/stars stay frozen, the frame rectangle moves.
                NOT inside the bottom collapse — the scrubber persists (field
                ask: keep the video controls while viewing clean). */}
            {/* trajOn does NOT hide this row — watching the trajectory while
                scrubbing/fixing frames is the whole point (field ask) */}
            {!calibOn && pMode !== "place" && !sizeOn && !cmpOn && source?.mediaKind === "video" && playPath && playPath.length > 1 && (
              <div style={{ display: "grid", gap: 6, marginBottom: 8 }}
                title={solvedPath
                  ? "World-locked playback: every frame is drawn at its own solved camera pose, so the sky and terrain stay fixed on the dome and only the object moves."
                  : "Preview only — the clip has not been stabilized, so every frame is drawn at the placement pose and the footage moves against a fixed sky. Stabilize to world-lock it."}>
                {/* the SCRUBBER gets its own full-width line (precise thumb travel —
                    the growing button row had squeezed it unusable), flanked by
                    single-frame step buttons for exact frame selection */}
                <div style={{ display: "flex", gap: 6, alignItems: "center" }}>
                  <button className="btn sm" style={{ minWidth: 34 }} title="Back one frame"
                    onClick={() => { playingRef.current = false; setPlaying(false); showFrame(playIdx - 1); }}>‹</button>
                  <div style={{ position: "relative", flex: 1, display: "flex", alignItems: "center" }}>
                    <input type="range" min={0} max={playPath.length - 1} step={1} value={playIdx}
                      onChange={(e) => { playingRef.current = false; setPlaying(false); showFrame(+e.target.value); }} style={{ width: "100%" }} />
                    {/* ⚓ anchor ticks — where the manual pose fixes sit on the clip */}
                    {fixOn && fixesNow.map((f) => {
                      const pp3 = playPath, span = (pp3[pp3.length - 1].t - pp3[0].t) || 1;
                      const pct = clampN(((+f.t - pp3[0].t) / span) * 100, 0, 100);
                      return <span key={"tk" + f.t} style={{ position: "absolute", left: pct + "%", top: -3, transform: "translateX(-50%)", fontSize: 8, lineHeight: 1, color: "var(--amber)", pointerEvents: "none" }}>▾</span>;
                    })}
                  </div>
                  <button className="btn sm" style={{ minWidth: 34 }} title="Forward one frame"
                    onClick={() => { playingRef.current = false; setPlaying(false); showFrame(playIdx + 1); }}>›</button>
                </div>
                <div style={{ display: "flex", gap: 6, alignItems: "center" }}>
                  <button className="btn sm amber" style={{ minWidth: 34 }} onClick={togglePlay}>{playing ? "⏸" : "▶"}</button>
                  <span style={{ flex: 1, minWidth: 0, fontFamily: "var(--mono)", fontSize: 10, color: !solvedPath ? "var(--amber)" : playPose ? "var(--teal)" : "var(--dim)", whiteSpace: "nowrap", overflow: "hidden", textOverflow: "ellipsis" }}>
                    {(playPath[playIdx]?.t ?? 0).toFixed(2)}s{isNum(playPath[playIdx]?.n) ? ` · ${playPath[playIdx].n} refs` : ""}
                    {!solvedPath && " · preview, not stabilized"}
                  </span>
                  {playPose && !fixOn && <button className="btn sm" onClick={exitPlayback} title="Back to the marked frame at its placement pose">↺</button>}
                  {/* ⚓ / 🎛 / ⬇ all operate ON a solved path — their panels are
                      gated on one, so in preview mode the buttons would toggle
                      and open nothing. Hide them until there is something to
                      correct, smooth or render. */}
                  {solvedPath && <>
                  <button className="btn sm" style={fixOn ? { borderColor: "var(--amber)", color: "var(--amber)" } : undefined}
                    onClick={() => {
                      playingRef.current = false; setPlaying(false);
                      setSmoothOpen(false); setExportMenu(false);
                      if (!fixOn && !playPose) showFrame(playIdx); // entering fix mode needs a frame on screen at its path pose
                      setFixOn((o) => !o);
                    }}
                    title="Fix frames: scrub to where the auto-stabilize lost the world lock, drag the photo back onto the true horizon/terrain (two-finger twist = tilt), then ⚓ Anchor it. Corrections blend smoothly between anchors and the object trajectory follows.">⚓</button>
                  <button className="btn sm" onClick={() => { setSmoothOpen((o) => !o); setExportMenu(false); setFixOn(false); }}
                    title="Smoothing — camera steadiness + object-track smoothing, re-applied non-destructively from the raw solve">🎛</button>
                  <button className="btn sm teal" onClick={() => { if (exporting) exportStabilized(); else setExportMenu((m) => !m); }}
                    title="Export the world-locked clip as a video file: every frame rendered at its own solved pose from a fixed camera, with the az/el grid burned in. Tap again to cancel.">
                    {exporting ? `${Math.round(exporting * 100)}%` : "⬇"}
                  </button>
                  </>}
                </div>
              </div>
            )}
            {fixOn && !calibOn && pMode !== "place" && !sizeOn && !cmpOn && source?.mediaKind === "video" && Array.isArray(source?.posePath) && source.posePath.length > 1 && (
              <div style={{ display: "grid", gap: 4, marginBottom: 8, background: "rgba(15,23,42,.65)", border: "1px solid var(--amber)", borderRadius: 10, padding: "6px 8px" }}>
                <div style={{ display: "flex", alignItems: "center", gap: 6, flexWrap: "wrap" }}>
                  <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--amber)", fontWeight: 800 }}>⚓</span>
                  {(() => {
                    /* no per-anchor chips (they crowded the panel — scrub/step to
                       a frame instead); the readout carries the anchor count and
                       whether THIS frame is one, and ✕⚓ removes it in place */
                    const curFix = playPose && isNum(playPose.t) ? fixesNow.find((f) => Math.abs(+f.t - +playPose.t) < 1e-3) : null;
                    return (
                      <>
                        {playPose && isNum(playPose.t) && (
                          <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: fixPending ? "var(--amber)" : "var(--dim)" }}>
                            {(+playPose.t).toFixed(2)}s{fixPending ? " · adjusted" : curFix ? " · anchored" : ""}{fixesNow.length ? ` · ${fixesNow.length}⚓` : ""}
                          </span>
                        )}
                        <span style={{ flex: 1 }} />
                        <button className="btn sm amber" disabled={!playPose || !isNum(playPose?.t)}
                          title="Save this frame's pose as an anchor — corrections blend between anchors and hold past the outermost ones. Anchoring an untouched frame pins it as correct so a neighbouring correction can't bleed into it."
                          onClick={setFixAnchor}>{fixPending ? "⚓ Anchor" : "⚓ Pin"}</button>
                        {curFix && !fixPending && <button className="btn sm" title="Remove this frame's anchor" onClick={() => dropFixAnchor(+curFix.t)}>✕⚓</button>}
                        {fixPending && <button className="btn sm" title="Discard the adjustment on this frame" onClick={revertFixFrame}>↺</button>}
                      </>
                    );
                  })()}
                </div>
                {/* nudge row is ALWAYS on in fix mode (field ask — no toggle) */}
                {playPose && (
                  <div style={{ display: "flex", gap: 3, alignItems: "center", flexWrap: "wrap" }}>
                    {/* arrow + roll sense inverted per field test — every tap moves
                        the photo the way it reads on screen. − ＋ scale the FRAME
                        POSE (its FOV — how much sky the photo spans), part of the
                        anchor like az/el/roll. No labels: 8 buttons must fit one
                        phone line. */}
                    <button className="btn sm" style={{ padding: "6px 9px" }} title="Photo left (azimuth +0.1°)" onClick={() => nudgeFix(0.1, 0, 0)}>‹</button>
                    <button className="btn sm" style={{ padding: "6px 9px" }} title="Photo right (azimuth −0.1°)" onClick={() => nudgeFix(-0.1, 0, 0)}>›</button>
                    <button className="btn sm" style={{ padding: "6px 9px", marginLeft: 5 }} title="Photo down (elevation +0.1°)" onClick={() => nudgeFix(0, 0.1, 0)}>▼</button>
                    <button className="btn sm" style={{ padding: "6px 9px" }} title="Photo up (elevation −0.1°)" onClick={() => nudgeFix(0, -0.1, 0)}>▲</button>
                    <button className="btn sm" style={{ padding: "6px 9px", marginLeft: 5 }} title="Tilt (roll +0.2°)" onClick={() => nudgeFix(0, 0, 0.2)}>⟲</button>
                    <button className="btn sm" style={{ padding: "6px 9px" }} title="Tilt (roll −0.2°)" onClick={() => nudgeFix(0, 0, -0.2)}>⟳</button>
                    <button className="btn sm" style={{ padding: "6px 9px", marginLeft: 5 }} title="Photo smaller — spans less sky (FOV −0.3°)" onClick={() => nudgeFix(0, 0, 0, -0.3)}>−</button>
                    <button className="btn sm" style={{ padding: "6px 9px" }} title="Photo bigger — spans more sky (FOV +0.3°)" onClick={() => nudgeFix(0, 0, 0, 0.3)}>＋</button>
                  </div>
                )}
                {/* ✓ Done lives bottom-right beside the hint — in the top row it
                    wrapped onto its own orphan line on narrow phones */}
                <div style={{ display: "flex", alignItems: "flex-end", gap: 8 }}>
                  <div style={{ flex: 1, fontSize: 9.5, color: "var(--dim)", lineHeight: 1.3 }}>
                    drag photo onto the true horizon · twist = tilt · ⚓ per frame — blends between anchors
                  </div>
                  <button className="btn sm" style={{ flex: "0 0 auto" }} onClick={() => setFixOn(false)}>✓</button>
                </div>
              </div>
            )}
            {smoothOpen && !calibOn && pMode !== "place" && source?.mediaKind === "video" && Array.isArray(source?.posePath) && source.posePath.length > 1 && (
              <div style={{ display: "grid", gap: 6, marginBottom: 8, background: "rgba(15,23,42,.65)", border: "1px solid var(--line)", borderRadius: 10, padding: "8px 10px" }}>
                <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                  <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)", minWidth: 86 }}>🎥 steadiness</span>
                  <input type="range" min={0} max={1} step={0.05} value={isNum(source?.smoothCam) ? +source.smoothCam : 0.25}
                    onPointerDown={(e) => e.stopPropagation()} onTouchStart={(e) => e.stopPropagation()}
                    onChange={(e) => applySmooth("cam", +e.target.value)} style={{ flex: 1, touchAction: "auto", pointerEvents: "auto" }} />
                </div>
                {Array.isArray(source?.objPath) && source.objPath.length > 1 && (
                  <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                    <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--dim)", minWidth: 86 }}>🛸 track smooth</span>
                    <input type="range" min={0} max={1} step={0.05} value={isNum(source?.smoothObj) ? +source.smoothObj : 0.25}
                      onPointerDown={(e) => e.stopPropagation()} onTouchStart={(e) => e.stopPropagation()}
                      onChange={(e) => applySmooth("obj", +e.target.value)} style={{ flex: 1, touchAction: "auto", pointerEvents: "auto" }} />
                  </div>
                )}
                <div style={{ fontSize: 9.5, color: "var(--dim)", lineHeight: 1.4 }}>
                  Non-destructive — re-applied from the raw solve, so slide freely. Left = keep hard corners
                  (a real anomalous maneuver is never averaged away); right = smooth into a clean curve
                  (an airplane's jitter is tracker noise, not flight). Heavier track smoothing also damps
                  real fast maneuvers in the measured rates — that trade is yours to make, so it's a slider.
                </div>
              </div>
            )}
            {exportMenu && !exporting && !calibOn && pMode !== "place" && source?.mediaKind === "video" && Array.isArray(source?.posePath) && source.posePath.length > 1 && (
              <div style={{ display: "grid", gap: 6, marginBottom: 8 }}>
                {[
                  ["view", "▣ World view", "the dome framing shown in playback — grid + all visible sky layers burned in"],
                  ["full", "⛶ Max resolution", "clean footage, no overlays — native source detail, sized so zoomed frames keep every pixel"],
                  ...(source?.A?.p1 && source?.A?.p2 ? [["crop", "◎ Object close-up", "zoomed crop that follows the object, pixel-pinned per frame at export — clean, no overlays"]] : []),
                ].map(([m, t2, d2]) => (
                  <button key={m} className="btn sm" style={{ textAlign: "left", padding: "8px 10px" }}
                    onClick={() => { setExportMenu(false); exportStabilized(m); }}>
                    <span style={{ color: "var(--teal)" }}>{t2}</span>
                    <span style={{ display: "block", fontSize: 10, color: "var(--dim)", marginTop: 2 }}>{d2}</span>
                  </button>
                ))}
              </div>
            )}
          </>
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
        {!botMin && !single && pMode !== "place" && (trajOn || sizeOn || cmpOn) && (
          <div style={{ marginBottom: 8 }}>
            {trajOn && (<>
            {(source?.moments || []).filter((m) => isNum(m?.A?.az) && isNum(m?.A?.el) && isNum(m?.whenMs)).length > 0 && (
              <div style={{ fontSize: 10, color: "var(--track)", marginBottom: 5, lineHeight: 1.35 }}>
                ↳ This observer has placed photo-moments. Points you drop here are timed from this photo and <b>interleave with the moments</b> on one trajectory — fill in the gaps between shots.
              </div>
            )}
            {/* READ-ONLY for both stills and video: the trajectory is laid down
               on the measure step (Track tool) now, so there's no drop/aim here
               — that's why the world view no longer needs an aiming crosshair. */}
            <div style={{ fontSize: 10.5, color: "var(--dim)", lineHeight: 1.45, fontStyle: "italic" }}>
              Shown for reference. Lay down &amp; edit this object's trajectory on the <b style={{ color: "var(--track)" }}>measure step</b> — <b>⊕ Track points</b> (tap the path on the photo; <b>✎ Adjust</b> for timing/turn/size).
            </div>
            {sortedTrack.length > 0 && (
              <div style={{ display: "flex", gap: 4, flexWrap: "wrap", alignItems: "center", marginTop: 6 }}>
                <span style={{ fontFamily: "var(--mono)", fontSize: 10, color: "var(--track)" }}>t₀</span>
                <span style={{ fontSize: 10, color: "var(--dim)", fontStyle: "italic" }}>{source?.mediaKind === "video" ? "timing set by the video frames" : "timing set on the measure step"}</span>
                <span style={{ marginLeft: "auto", fontFamily: "var(--mono)", fontSize: 11, color: "var(--track)" }}>
                  total {trajTotal.toFixed(1)} s · {sortedTrack.length} pt{sortedTrack.length > 1 ? "s" : ""}
                </span>
              </div>
            )}
            {false && selSeg != null && sortedTrack[selSeg] && (() => {
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
            {false && selPt != null && sortedTrack[selPt] && (() => {
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
                    /* VIDEO: size + attitude are set on the MEASURE step (Track →
                       ✎ Adjust size/shape), scrubbing to each point — not here. */
                    if (source?.mediaKind === "video") return (
                      <div style={{ fontSize: 10.5, color: "var(--dim)", margin: "2px 0", lineHeight: 1.5 }}>
                        Set this object's <b style={{ color: "var(--track)" }}>size &amp; attitude</b> on the measure step — <b>Track → ✎ Adjust size/shape</b>, scrub to the point. It carries through here automatically.
                      </div>
                    );
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
                    /* a manual sky-view size overrides any measure-step pixel size —
                       drop wpx so the post-stabilize FOV re-derivation can't undo this */
                    const setAng = (a) => update({ track: sortedTrack.map((p, i) => (i === selPt ? (({ wpx, ...rest }) => ({ ...rest, ang: +clampN(a, angMin, 60).toFixed(5) }))(p) : p)) });
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
                  {source?.shapeFit && source?.mediaKind !== "video" && (
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
            </>)}
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
                <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 3 }}>Aim the crosshair, ⌖ drop the ghost there, then slide its assumed distance — or set it on a map:</div>
                {hasPos && (
                  <button className="btn sm" style={{ marginTop: 5 }} onClick={() => setMapPick({ mode: "compare" })}>📍 Set distance on a map</button>
                )}
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
                  /* cloud-base cap: below a real deck, range < base/sin(el) */
                  const deck = wxSky && wxSky.baseAGL != null && ((wxSky.low != null ? wxSky.low : wxSky.cloud) || 0) >= 40;
                  const cb = (deck && isNum(objEl) && objEl > 0.5) ? cloudRangeBound(wxSky.baseAGL, objEl, objAngW) : null;
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
                      {cb && (
                        <div style={{ fontFamily: "var(--mono)", fontSize: 11, marginTop: 3, color: objD > cb.maxRange ? "var(--red)" : "var(--teal)" }}>
                          ☁ cloud base ≈ {fmtLenShort(wxSky.baseAGL)} — if it was BELOW the deck it was within <b>{fmtLenShort(cb.maxRange)}</b>{cb.maxSize != null ? <> (≤ <b>{fmtLenShort(cb.maxSize)}</b> across)</> : null}{objD > cb.maxRange ? " · slider is past that" : ""}
                        </div>
                      )}
                      {hasPos && (
                        <button className="btn sm" style={{ marginTop: 6 }} onClick={() => setMapPick({ mode: "size" })}>📍 Set distance on a map</button>
                      )}
                    </>
                  );
                })() : (
                  <div style={{ fontSize: 11, color: "var(--dim)" }}>Mark the object's width first (fit a shape, or set the two size marks on the photo) — then slide an assumed distance to read its true size.</div>
                )}
              </div>
            )}
          </div>
        )}
        {!botMin && !fixOn && (
        <div style={{ fontSize: 9.5, lineHeight: 1.3, color: "rgba(255,255,255,.6)", marginTop: 2, marginBottom: 6 }}>
          {pMode === "place" && photoOn
            ? (panMode
              ? "✋ Pan mode — drag moves the magnified view. Tap ✋ again to slide the sky."
              : "drag = slide sky · pinch = FOV · twist = roll (first move wins the gesture) · 🎛 fine taps")
            : wizard
              ? (single
                ? "Align this moment's photo to the sky, then Continue — it becomes one direction (its time comes from the moment). Together with the other moments it builds the trajectory."
                : calibOn
                  ? "Tap a named star or planet, then aim the crosshair on it in the photo and ✓ Set. One star fixes roll + FOV, a second adds lens distortion, a third is a full plate-solve."
                  : trajOn
                    ? "Aim the crosshair where the object was at each moment and ⊕ drop points — the path can run right past the photo's edges. Tap a +Δt chip to adjust timing, or tap a numbered point to set how tight its turn was (hard corner ↔ wide arc)."
                    : sizeOn
                      ? "Slide an assumed distance — or 📍 set it on a map — to read the object's true size and altitude at that range."
                      : cmpOn
                        ? "Aim the crosshair, ⌖ drop the reference ghost there, then slide its distance — or set it on a map — to compare its apparent size to your object's."
                        : "Drag to look around · pinch to zoom. Pick a tool below: ✥ Place to align the photo, ⊕ Trajectory to trace its path, 📏 Size or ⚖ Compare to gauge distance.")
              : motionOn
                ? "Point the phone exactly where the object was, then capture."
                : "Drag to look around · pinch to zoom · put the crosshair where the object was. The Sun/Moon are drawn where they really were at the sighting time — use them to anchor your bearing."}
        </div>
        )}
        {/* the PERSISTENT bottom row — never collapsed, hosts the ⌃/⌄ toggle */}
        {!wizard && pMode !== "place" && (
        <div style={{ display: "flex", gap: 8 }}>
          <button className="btn" style={{ flex: "0 0 auto", padding: "10px 12px" }} title={botMin ? "Show the controls" : "Hide the controls for a cleaner view"} onClick={() => setBotMin((v) => !v)}>{botMin ? "⌃" : "⌄"}</button>
          <button className="btn amber" style={{ flex: 1 }} onClick={() => { onCapture("A", viewAz, viewAlt); setFlash("Moment A locked ✓ — sky view stays open; aim B or Close"); }}>Set Moment A</button>
          <button className="btn teal" style={{ flex: 1 }} onClick={() => { onCapture("B", viewAz, viewAlt); setFlash("Moment B locked ✓ — Close when done"); }}>Set Moment B</button>
        </div>
        )}
        {wizard && pMode === "place" && (
          <div style={{ display: "flex", gap: 8 }}>
            <button className="btn" style={{ flex: "0 0 auto", padding: "10px 12px" }} title={botMin ? "Show the alignment tools" : "Hide the tools — the photo and dome stay live for a clean look"} onClick={() => setBotMin((v) => !v)}>{botMin ? "⌃" : "⌄"}</button>
            <button className="btn amber" style={{ flex: 1 }} onClick={donePlace}>✓ Horizon lined up — continue</button>
          </div>
        )}
        {wizard && pMode !== "place" && (
          <div style={{ display: "flex", gap: 8 }}>
            <button className="btn" style={{ flex: "0 0 auto", padding: "10px 12px" }} title={botMin ? "Show the controls" : "Hide the controls for a cleaner view"} onClick={() => setBotMin((v) => !v)}>{botMin ? "⌃" : "⌄"}</button>
            <button className="btn" onClick={() => { if (photoOn) commitPlacement(); onWizardBack && onWizardBack(); }}>‹ Back</button>
            <button className="btn amber" style={{ flex: 1 }} onClick={() => { if (photoOn) commitPlacement(); onWizardNext && onWizardNext(); }}>Continue →</button>
          </div>
        )}
      </div>
      {mapPick && hasPos && (() => {
        const isSize = mapPick.mode === "size";
        const aim = source?.mediaAim || {};
        const azObj = isSize ? (isNum(source?.A?.az) ? +source.A.az : effAz) : (cmpPos && isNum(cmpPos.az) ? cmpPos.az : effAz);
        const elObj = isSize ? (isNum(source?.A?.el) ? +source.A.el : effAlt) : (cmpPos && isNum(cmpPos.el) ? cmpPos.el : effAlt);
        const azCenter = isNum(aim.az) ? +aim.az : azObj;
        return (
          <DistanceMapPick
            lat={LAT} lon={LNG} azCenter={azCenter} azObj={azObj} elObj={elObj} fovH={fovM}
            objAng={isSize ? objAngW : null} initDist={isSize ? objD : cmpD}
            title={isSize ? "WHERE WAS THE OBJECT?" : "WHERE WAS THE REFERENCE?"}
            onClose={() => setMapPick(null)}
            onAccept={(d) => { if (isSize) setObjD(clampN(d, 30, 80000)); else setCmpD(clampN(d, 5, 120000)); setMapPick(null); }} />
        );
      })()}
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
function PinMap({ lat, lon, origin, others, onChange, bearing, tilt, fov }) {
  const boxRef = useRef(null);
  const mapRef = useRef(null);
  const layersRef = useRef(null);     // {sat, street, trans, ref}
  const overlayRef = useRef(null);    // marker layer group
  const originLineRef = useRef(null);
  const originRef = useRef(origin);
  const onChangeRef = useRef(onChange);
  const progRef = useRef(false);      // programmatic setView — don't commit
  const coordElRef = useRef(null);
  const distElRef = useRef(null);
  const coneRef = useRef(null);       // FOV cone body polygon (geographic)
  const capRef = useRef(null);        // cap ellipse/circle outline
  const aimLineRef = useRef(null);    // centre aim polyline
  const youMarkerRef = useRef(null);  // free-look "YOU" pin (geographic)
  const aimRef = useRef({});          // latest bearing/tilt/fov for move-time redraw
  const posRef = useRef({});          // latest lat/lon for free-look anchor
  const moveModeRef = useRef(false);
  const [ready, setReady] = useState(false);
  const [baseSat, setBaseSat] = useState(true);
  /* Move-pin mode. OFF (default): pan/zoom to look around WITHOUT moving your
     position — the pin stays geographically fixed. ON: the pin rides the map
     centre and a pan commits the new position (the old "drag the ground" model). */
  const [moveMode, setMoveMode] = useState(false);
  originRef.current = origin;
  onChangeRef.current = onChange;
  aimRef.current = { bearing, tilt, fov };
  posRef.current = { lat, lon };
  moveModeRef.current = moveMode;

  const mPerDegN = 111320;
  const mPerDegE = (la) => 111320 * Math.max(0.2, Math.cos((+la || 0) * D2R));
  /* the observer point: map centre while repositioning, else the fixed position */
  const youPoint = () => {
    const map = mapRef.current;
    if (moveModeRef.current || !isNum(posRef.current.lat) || !isNum(posRef.current.lon)) return map.getCenter();
    return L.latLng(+posRef.current.lat, +posRef.current.lon);
  };

  const hud = () => {
    const map = mapRef.current; if (!map) return;
    const c = youPoint();
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

  /* viewing-direction cone drawn as REAL geography (like the size/compare
     distance picker): a ~25-mile sight-line out over the map so you can see what
     you were looking toward — city, roads and landmarks under it. It foreshortens
     with the up-angle: at the horizon it's a long flat wedge; angling up pulls the
     far end IN and rounds it into an ellipse; straight up it's a circle centred on
     you (looking into the sky all around). Anchored at your position (or the map
     centre while repositioning) and redrawn on pan. */
  const drawAim = () => {
    const map = mapRef.current; if (!map) return;
    const o = youPoint();
    const { bearing, tilt, fov } = aimRef.current;
    const clear = (r) => { if (r.current) { map.removeLayer(r.current); r.current = null; } };
    if (!isNum(bearing)) { clear(coneRef); clear(capRef); clear(aimLineRef); return; }
    const az = ((+bearing % 360) + 360) % 360;
    const elp = clampN(isNum(tilt) ? +tilt : 0, 0, 90);           // shape morph uses 0..90
    const s = Math.sin(elp * D2R), c = Math.cos(elp * D2R);
    const reach = 40234;                                          // ~25 mi slant sight-line
    const half = (clampN(isNum(fov) ? +fov : 60, 1, 170) / 2) * D2R;
    const B = clampN(reach * Math.tan(half), 10, reach);          // perp half-width = cap radius at zenith
    const d = reach * c;                                          // cap-centre ground distance (→ 0 at zenith)
    const A = B * s;                                              // along-aim cap semi-axis (0 flat at horizon → B circle up)
    const cosA = Math.cos(az * D2R), sinA = Math.sin(az * D2R);
    const pt = (x, y) => {                                        // aim frame (x forward, y right) → [lat,lon]
      const offN = x * cosA - y * sinA, offE = x * sinA + y * cosA;
      return [o.lat + offN / mPerDegN, o.lng + offE / mPerDegE(o.lat)];
    };
    const NE = 36, ell = [];
    for (let i = 0; i < NE; i++) { const th = (i / NE) * 2 * Math.PI; ell.push([d + A * Math.cos(th), B * Math.sin(th)]); }
    /* body = convex hull of the eye point ∪ the cap ellipse: a triangle wedge at
       the horizon, an ice-cream cone as it rounds, and (eye inside the ellipse) the
       bare circle at the zenith — all handled by the hull, no special-casing. */
    const cross = (O, P, Q) => (P[0] - O[0]) * (Q[1] - O[1]) - (P[1] - O[1]) * (Q[0] - O[0]);
    const sp = ell.concat([[0, 0]]).sort((p, q) => p[0] - q[0] || p[1] - q[1]);
    const lo = []; for (const p of sp) { while (lo.length >= 2 && cross(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
    const up = []; for (let i = sp.length - 1; i >= 0; i--) { const p = sp[i]; while (up.length >= 2 && cross(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
    lo.pop(); up.pop();
    const hullPts = lo.concat(up).map((p) => pt(p[0], p[1]));
    const capPts = ell.map((p) => pt(p[0], p[1]));
    const aimPts = [pt(0, 0), pt(d, 0)];
    if (!coneRef.current) coneRef.current = L.polygon(hullPts, { color: "#5FD3BC", weight: 1, opacity: 0.5, fillColor: "#5FD3BC", fillOpacity: 0.12, interactive: false }).addTo(map);
    else coneRef.current.setLatLngs(hullPts);
    if (!capRef.current) capRef.current = L.polygon(capPts, { color: "#5FD3BC", weight: 1.6, opacity: 0.85, fill: false, interactive: false }).addTo(map);
    else capRef.current.setLatLngs(capPts);
    if (!aimLineRef.current) aimLineRef.current = L.polyline(aimPts, { color: "#5FD3BC", weight: 2.5, opacity: 0.9, interactive: false }).addTo(map);
    else aimLineRef.current.setLatLngs(aimPts);
  };

  useEffect(() => {
    const el = boxRef.current; if (!el || mapRef.current) return;
    const map = L.map(el, {
      center: [+lat, +lon], zoom: 17, zoomControl: false,
      attributionControl: true, doubleClickZoom: false,
      touchZoom: "center", scrollWheelZoom: "center", bounceAtZoomLimits: false,
    });
    map.attributionControl.setPrefix(false);
    const sat = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
      maxZoom: 21, maxNativeZoom: 19, attribution: "© Esri, Maxar, Earthstar Geographics",
    });
    const street = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 21, maxNativeZoom: 19, attribution: "© OpenStreetMap contributors", className: "pinmap-street-tiles",
    });
    /* Esri reference overlays over the imagery — roads + boundaries/place labels,
       so a satellite view still reads towns, highways and landmarks under the
       viewing cone (same layers the report basemap and distance picker use). */
    const trans = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Transportation/MapServer/tile/{z}/{y}/{x}", { maxZoom: 21, maxNativeZoom: 19, opacity: 0.9 });
    const ref = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}", { maxZoom: 21, maxNativeZoom: 19 });
    sat.addTo(map); trans.addTo(map); ref.addTo(map);
    map.on("move", () => { hud(); drawAim(); });
    map.on("moveend", () => {
      if (progRef.current) { progRef.current = false; return; }
      if (!moveModeRef.current) return;   // free-look pan — don't move the position
      const c = map.getCenter();
      onChangeRef.current(c.lat, c.lng);
    });
    mapRef.current = map; layersRef.current = { sat, street, trans, ref };
    hud(); drawAim();
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
    }
    hud(); drawAim();
  }, [lat, lon]);

  useEffect(() => {
    const map = mapRef.current; if (!map || !ready) return;
    const { sat, street, trans, ref } = layersRef.current;
    if (baseSat) { map.removeLayer(street); sat.addTo(map); trans.addTo(map); ref.addTo(map); }
    else { map.removeLayer(sat); map.removeLayer(trans); map.removeLayer(ref); street.addTo(map); }
  }, [baseSat, ready]);

  /* redraw the viewing cone when the aim (bearing/tilt/fov) changes */
  useEffect(() => {
    if (!ready) return;
    drawAim();
  }, [bearing, tilt, fov, ready]);

  /* entering move mode: recentre on the position so the crosshair sits on it.
     Either mode: refresh the HUD/cone + the free-look pin marker. */
  useEffect(() => {
    const map = mapRef.current; if (!map || !ready) return;
    if (moveMode && isNum(lat) && isNum(lon)) {
      const cc = map.getCenter();
      if (Math.abs(cc.lat - +lat) > 2e-6 || Math.abs(cc.lng - +lon) > 2e-6) { progRef.current = true; map.setView([+lat, +lon], map.getZoom(), { animate: false }); }
    }
    if (youMarkerRef.current) { map.removeLayer(youMarkerRef.current); youMarkerRef.current = null; }
    if (!moveMode && isNum(lat) && isNum(lon)) {
      youMarkerRef.current = L.marker([+lat, +lon], { interactive: false, keyboard: false,
        icon: L.divIcon({ className: "", iconSize: [0, 0], html: `<div class="lmk lmk-fix">⊕<span>YOU</span></div>` }) }).addTo(map);
    }
    hud(); drawAim();
  }, [moveMode, lat, lon, ready]);

  /* photo-GPS origin + fellow observers */
  useEffect(() => {
    const map = mapRef.current; if (!map || !ready) return;
    if (overlayRef.current) { map.removeLayer(overlayRef.current); overlayRef.current = null; }
    originLineRef.current = null;
    const g = L.layerGroup();
    if (origin && isNum(origin.lat)) {
      const c = youPoint();
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
    hud();
  }, [others, origin, ready]);

  return (
    <div style={{ marginTop: 10 }}>
      <div style={{ display: "flex", justifyContent: "space-between", alignItems: "center", marginBottom: 4 }}>
        <ML style={{ marginBottom: 0 }}>{moveMode ? "Move pin — drag the map to reposition" : "Look around — pan/zoom freely"}</ML>
        <div style={{ display: "flex", gap: 4 }}>
          <button className={"btn sm" + (moveMode ? " amber" : "")} onClick={() => setMoveMode((v) => !v)} title={moveMode ? "Repositioning — the pin follows the map centre; tap to lock it and look around freely" : "Pan/zoom moves the view only; tap to move your pin"}>✥ Move</button>
          <button className="btn sm" onClick={() => setBaseSat((s) => !s)}>{baseSat ? "🗺 street" : "🛰 sat"}</button>
          <button className="btn sm" onClick={() => mapRef.current && mapRef.current.zoomOut()}>−</button>
          <button className="btn sm" onClick={() => mapRef.current && mapRef.current.zoomIn()}>+</button>
        </div>
      </div>
      <div className="pinmapwrap">
        <div ref={boxRef} style={{ position: "absolute", inset: 0 }} />
        <div className="map-north">N ↑</div>
        {moveMode && (
          <>
            <svg className="pinmap-cross" viewBox="-14 -14 28 28" width="28" height="28">
              <circle cx="0" cy="0" r="7" fill="none" stroke="#5FD3BC" strokeWidth="2" />
              <path d="M-12 0H12M0 -12V12" stroke="#5FD3BC" strokeWidth="2" />
            </svg>
            <div className="pinmap-you">YOU</div>
          </>
        )}
        <div className="pinmap-hud">
          <div ref={distElRef} style={{ color: "var(--amber)" }} />
          <div ref={coordElRef} />
        </div>
      </div>
    </div>
  );
}

/* ============================================================
   DISTANCE MAP PICKER — set the assumed distance for the size/compare tools
   by placing where the object was on a map. Shows the camera at the photo's
   GPS, a wedge for its field of view + the object's sight-line, and a draggable
   "object here" pin. The straight-line ground distance (converted to a slant
   range through the sight-line elevation, so size AND altitude stay consistent)
   feeds the tool's distance. Lives inside the open SkyAimer, so its map box is
   class "mappick" — whitelisted in the aimer's touchmove scroll-lock so Leaflet
   pan/drag actually work.
   ============================================================ */
function DistanceMapPick({ lat, lon, azCenter, azObj, elObj, fovH, objAng, initDist, title, onAccept, onClose }) {
  const boxRef = useRef(null);
  const mapRef = useRef(null);
  const objRef = useRef(null);
  const layersRef = useRef(null);
  const [baseSat, setBaseSat] = useState(true);
  const [gDist, setGDist] = useState(null);   // straight-line ground distance to the pin, m

  const mPerDegN = 111320;
  const mPerDegE = (la) => 111320 * Math.max(0.2, Math.cos((+la || 0) * D2R));
  const destPoint = (az, d) => [lat + (d * Math.cos(az * D2R)) / mPerDegN, lon + (d * Math.sin(az * D2R)) / mPerDegE(lat)];
  const groundDist = (la, lo) => Math.hypot((lo - lon) * mPerDegE(lat), (la - lat) * mPerDegN);
  const el0 = clampN(isNum(elObj) ? +elObj : 0, 0, 89);
  const cosEl = Math.max(0.05, Math.cos(el0 * D2R));
  const slantOf = (g) => g / cosEl;                             // ground → line-of-sight range
  const initGround = Math.max(30, (isNum(initDist) ? +initDist : 1000) * cosEl);

  useEffect(() => {
    const el = boxRef.current; if (!el || mapRef.current || typeof L === "undefined") return;
    const start = destPoint(azObj, initGround);
    const map = L.map(el, { center: [lat, lon], zoom: 13, zoomControl: false, attributionControl: true, doubleClickZoom: false });
    map.attributionControl.setPrefix(false);
    const sat = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", { maxZoom: 21, maxNativeZoom: 19, attribution: "© Esri, Maxar, Earthstar Geographics" });
    /* transparent Esri reference overlays — roads + boundaries/place labels —
       laid over the imagery so a satellite view still reads towns, highways and
       ridgelines (the same layers the report basemap uses) */
    const trans = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Transportation/MapServer/tile/{z}/{y}/{x}", { maxZoom: 21, maxNativeZoom: 19, opacity: 0.9 });
    const ref = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}", { maxZoom: 21, maxNativeZoom: 19 });
    const street = L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", { maxZoom: 21, maxNativeZoom: 19, attribution: "© OpenStreetMap contributors", className: "pinmap-street-tiles" });
    sat.addTo(map); trans.addTo(map); ref.addTo(map);
    layersRef.current = { sat, street, trans, ref };
    mapRef.current = map;
    /* the modal's flex map box can lay out a frame late — recalc size, THEN
       frame both the camera and the initial pin (so a 25-mile sight-line and a
       near guess both fit) */
    requestAnimationFrame(() => {
      try {
        map.invalidateSize(false);
        /* frame the camera plus a good stretch of the sight-line (≥ ~12 km, more
           if the current guess is already far) so a distant object is reachable
           without hunting — not just the near default pin */
        const frameEnd = destPoint(azObj, clampN(initGround * 1.6, 12000, 42000));
        map.fitBounds(L.latLngBounds([[lat, lon], frameEnd]), { padding: [50, 50], maxZoom: 14 });
      } catch (e) { }
    });

    /* FOV sector + the object's sight-line + the camera position. Rays run out
       to ~25 miles so the wedge still guides you when the object was far off. */
    const wedgeR = 40234;
    if (isNum(azCenter) && isNum(fovH)) {
      const arc = [[lat, lon]];
      const n = 24;
      for (let i = 0; i <= n; i++) arc.push(destPoint(azCenter - fovH / 2 + (fovH * i) / n, wedgeR));
      L.polygon(arc, { color: "#5fd0ff", weight: 1, opacity: 0.7, fillColor: "#5fd0ff", fillOpacity: 0.1, interactive: false }).addTo(map);
    }
    if (isNum(azObj)) L.polyline([[lat, lon], destPoint(azObj, wedgeR)], { color: "#F5A93F", weight: 2, dashArray: "7 5", opacity: 0.9, interactive: false }).addTo(map);
    L.marker([lat, lon], { interactive: false, icon: L.divIcon({ className: "", iconSize: [0, 0], html: `<div class="lmk lmk-cam">◉<span>camera</span></div>` }) }).addTo(map);

    const om = L.marker(start, { draggable: true, autoPan: true, icon: L.divIcon({ className: "", iconSize: [0, 0], html: `<div class="lmk lmk-obj">✦<span>object was over here</span></div>` }) }).addTo(map);
    objRef.current = om;
    const upd = (ll) => setGDist(groundDist(ll.lat, ll.lng));
    om.on("drag", (e) => upd(e.target.getLatLng()));
    om.on("dragend", (e) => upd(e.target.getLatLng()));
    map.on("click", (e) => { om.setLatLng(e.latlng); upd(e.latlng); });
    upd({ lat: start[0], lng: start[1] });
    return () => { map.remove(); mapRef.current = null; };
  }, []); // eslint-disable-line

  useEffect(() => {
    const map = mapRef.current, ls = layersRef.current; if (!map || !ls) return;
    if (baseSat) { map.removeLayer(ls.street); ls.sat.addTo(map); ls.trans.addTo(map); ls.ref.addTo(map); }
    else { map.removeLayer(ls.sat); map.removeLayer(ls.trans); map.removeLayer(ls.ref); ls.street.addTo(map); }
  }, [baseSat]);

  const slant = gDist != null ? slantOf(gDist) : null;
  const size = slant != null && isNum(objAng) ? 2 * slant * Math.tan(objAng * D2R / 2) : null;
  const alt = slant != null && el0 > 0 ? slant * Math.sin(el0 * D2R) : null;

  return (
    <div className="mappick-modal">
      <div style={{ display: "flex", alignItems: "center", gap: 8, padding: "10px 12px", borderBottom: "1px solid var(--line)" }}>
        <div style={{ flex: 1, fontFamily: "var(--mono)", fontWeight: 800, letterSpacing: ".1em", fontSize: 12 }}>{title || "WHERE WAS IT?"}</div>
        <button className="btn sm" onClick={() => setBaseSat((s) => !s)}>{baseSat ? "🗺 street" : "🛰 sat"}</button>
        <button className="btn sm" onClick={() => mapRef.current && mapRef.current.zoomOut()}>−</button>
        <button className="btn sm" onClick={() => mapRef.current && mapRef.current.zoomIn()}>+</button>
      </div>
      <div ref={boxRef} className="mappick" style={{ flex: 1, minHeight: 0 }} />
      <div style={{ padding: "8px 12px 12px", borderTop: "1px solid var(--line)" }}>
        <div style={{ fontSize: 11, color: "var(--dim)", marginBottom: 6, lineHeight: 1.4 }}>
          Tap the map (or drag the ✦) where the object was — inside the blue field-of-view wedge, along the amber sight-line. The distance from the camera sets the tool.
        </div>
        <div style={{ fontFamily: "var(--mono)", fontSize: 12, color: "var(--teal)", minHeight: 18 }}>
          {gDist != null ? (
            <>{fmtLenShort(gDist)} away{el0 > 0.5 ? <> · <span style={{ color: "var(--amber)" }}>{fmtLenShort(slant)}</span> line-of-sight ({el0.toFixed(0)}° up)</> : null}
              {size != null && <> → <b>{fmtLenShort(size)}</b> across</>}{alt != null && <> · {fmtLenShort(alt)} up</>}</>
          ) : "place the object…"}
        </div>
        <div style={{ display: "flex", gap: 8, marginTop: 8 }}>
          <button className="btn" style={{ flex: 1 }} onClick={onClose}>Cancel</button>
          <button className="btn amber" style={{ flex: 2 }} disabled={slant == null} onClick={() => slant != null && onAccept(Math.round(slant))}>✓ Use this distance</button>
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
  const [tiltHint, setTiltHint] = useState(false); // side-view angle popup while sliding
  const tiltHintRef = useRef(0);
  const pokeTiltHint = () => { setTiltHint(true); clearTimeout(tiltHintRef.current); tiltHintRef.current = setTimeout(() => setTiltHint(false), 1100); };
  const [camHint, setCamHint] = useState(false); // side-view height popup while sliding
  const camHintRef = useRef(0);
  const pokeCamHint = () => { setCamHint(true); clearTimeout(camHintRef.current); camHintRef.current = setTimeout(() => setCamHint(false), 1100); };
  const posDone = isNum(src.lat) && isNum(src.lon);
  /* camera height above ground — the metadata can't be trusted for this (phone
     GPS altitude ≈ terrain even from an upstairs window), so it's a manual
     override for the building layer. Unset ⇒ the sky view still auto-estimates
     from GPS-altitude−terrain and falls back to 1.6 m eye height. */
  const camH = isNum(src.camH) ? +src.camH : 1.6;
  const setCamH = (m) => update({ camH: +clampN(+m, 0, 120).toFixed(1) });
  /* Viewing direction = the placement-center azimuth, held in mediaAim.az —
     the SAME field the sky view uses for photo placement. Both screens read
     and write it, so they always mirror: change the ray here and the sky view
     opens aimed there; rotate the placement in the sky view and (on commit)
     the ray here follows. Auto-seeded from the EXIF compass at load. */
  const bearing = isNum(src.mediaAim?.az) ? +src.mediaAim.az
    : (isNum(src.meta?.azTrue) ? +src.meta.azTrue
      : (isNum(src.meta?.az) ? +src.meta.az
        : (isNum(src.A?.az) ? +src.A.az
          : 0)));   // no compass anywhere → still DRAW the cone (due north) so there's something to grab — it was invisible until the slider was first touched
  const setBearing = (deg) => {
    const b = ((+deg % 360) + 360) % 360;
    const old = isNum(src.mediaAim?.az) ? +src.mediaAim.az : b;
    const d = b - old; // repointing rotates the whole placement in azimuth — the sight-lines ride along
    const rot = (a) => ((((+a + d) % 360) + 360) % 360);
    const patch = { mediaAim: { az: +b.toFixed(2), el: src.mediaAim?.el ?? 15, roll: src.mediaAim?.roll ?? 0 } };
    if (isNum(src.A?.az)) patch.A = { ...src.A, az: rot(src.A.az).toFixed(1) };
    if (isNum(src.B?.az)) patch.B = { ...src.B, az: rot(src.B.az).toFixed(1) };
    /* a pure yaw of the placement: the solved camera path + object track are
       anchored to it, so they yaw by the same delta (a rotation about world-up
       leaves el/roll untouched) — same re-anchoring the sky view does */
    if (Math.abs(d) > 0.02 && Array.isArray(src.posePath) && src.posePath.length) {
      const yaw = (arr) => arr.map((p) => ({ ...p, az: +rot(p.az).toFixed(3) }));
      patch.posePath = yaw(src.posePath);
      if (Array.isArray(src.posePathRaw) && src.posePathRaw.length) patch.posePathRaw = yaw(src.posePathRaw);
      if (Array.isArray(src.objPath) && src.objPath.length) patch.objPath = yaw(src.objPath);
      if (Array.isArray(src.objPathRaw) && src.objPathRaw.length) patch.objPathRaw = yaw(src.objPathRaw);
      if (Array.isArray(src.poseFixes) && src.poseFixes.length) patch.poseFixes = yaw(src.poseFixes); // ⚓ Fix-frames anchors are absolute poses — they yaw along
    }
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
  /* horizontal field of view — draws the aim as a CONE on the map (not just an
     arrow). When the lens metadata gives it (`meta.fovH`) it's locked/authoritative;
     otherwise it's a user estimate set with the slider here (same `fovH` field the
     measure step uses, so the two mirror). */
  const hasFovMeta = isNum(src.meta?.fovH);
  const fovH = isNum(src.fovH) ? +src.fovH : (hasFovMeta ? +src.meta.fovH : 68);
  const setFovH = (v) => update({ fovH: +clampN(+v, 8, 160).toFixed(1) });
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
      setDemMsg(`✓ terrain elevation ${fmtLenShort(h)} — steadier than phone GPS altitude (±5 m wobble)`);
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
              <button className="btn sm teal" onClick={searchPlace} disabled={findBusy || !q.trim()}>{findBusy ? <Spin /> : "🔎 Search"}</button>
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
            {/* compact manual coordinate entry — the MAP below is the focus;
                these are the type/paste fallback + fine-tune */}
            <div className="grid3" style={{ marginTop: 6, gap: 6 }}>
              <Num compact label="Latitude" value={src.lat} onChange={(v) => {
                const m = String(v).match(/(-?\d+(?:\.\d+)?)[,\s]+(-?\d+(?:\.\d+)?)/);
                if (m) update({ lat: m[1], lon: m[2] }); else update({ lat: v });
              }} ph="e.g. 42.1638" />
              <Num compact label="Longitude" value={src.lon} onChange={(v) => {
                const m = String(v).match(/(-?\d+(?:\.\d+)?)[,\s]+(-?\d+(?:\.\d+)?)/);
                if (m) update({ lat: m[1], lon: m[2] }); else update({ lon: v });
              }} ph="e.g. −123.6480" />
              <Num compact label="Elev" unit="m, opt" value={src.alt} onChange={(v) => update({ alt: v })} ph="0" />
            </div>
            <div style={{ marginTop: 6, display: "flex", gap: 8, alignItems: "center", flexWrap: "wrap" }}>
              {ENABLE_GPS_BUTTON && (
                <button className="btn sm teal" onClick={grabLocation}>{geoBusy ? <><Spin style={{ marginRight: 6 }} />Locating</> : "📍 Use my GPS"}</button>
              )}
              {!posDone && src.meta && isNum(src.meta.lat) && (
                <button className="btn sm amber" onClick={() => update({ lat: String(src.meta.lat), lon: String(src.meta.lon), ...(isNum(src.meta.alt) ? { alt: String(src.meta.alt) } : {}) })}>
                  📎 Use the photo's GPS
                </button>
              )}
              {posDone && (
                <button className="btn sm" onClick={grabDem} disabled={demBusy}>{demBusy ? <Spin /> : "⛰ Use terrain elevation"}</button>
              )}
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
                  others={others} bearing={bearing} tilt={tilt} fov={fovH}
                  onChange={(la, lo) => update({ lat: la.toFixed(6), lon: lo.toFixed(6) })} />
                {/* aim sliders live UNDER the map so you set the ray you can see */}
                <div style={{ marginTop: 6 }}>
                  <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
                    <ML style={{ marginBottom: 1 }}>Viewing direction</ML>
                    <span style={{ color: "var(--teal)", fontFamily: "var(--mono)", fontSize: 11 }}>{isNum(bearing) ? `${Math.round(bearing)}° ${compass8(bearing)}` : "drag to set"}</span>
                  </div>
                  <input type="range" min={0} max={359} step={1} value={isNum(bearing) ? bearing : 0} onChange={(e) => setBearing(+e.target.value)} />
                </div>
                {/* field of view — a cone on the map. From the lens when the photo
                    carries it; otherwise a slider so you can widen/narrow the wedge
                    to match the shot. */}
                <div style={{ marginTop: 4 }}>
                  <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
                    <ML style={{ marginBottom: 1 }}>Field of view</ML>
                    <span style={{ color: hasFovMeta ? "var(--teal)" : "var(--amber)", fontFamily: "var(--mono)", fontSize: 11 }}>
                      {`${Math.round(fovH)}°`}{hasFovMeta ? " · from lens ✓" : " · estimate"}
                    </span>
                  </div>
                  {hasFovMeta
                    ? <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 1 }}>The lens metadata sets the cone width — the wedge shows how much sky the photo spans.</div>
                    : <input type="range" min={12} max={130} step={1} value={fovH} onChange={(e) => setFovH(+e.target.value)} />}
                </div>
                <div style={{ marginTop: 2, position: "relative" }}>
                  <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
                    <ML style={{ marginBottom: 1 }}>How high you looked</ML>
                    <span style={{ color: "var(--teal)", fontFamily: "var(--mono)", fontSize: 11 }}>{Math.round(tilt)}° {tilt >= 75 ? "straight up" : tilt <= 5 ? "horizon" : "up"}</span>
                  </div>
                  <input type="range" min={-20} max={90} step={1} value={tilt}
                    onPointerDown={pokeTiltHint} onChange={(e) => { setTilt(+e.target.value); pokeTiltHint(); }} />
                  {/* momentary side-view of the sight-line angle while sliding */}
                  <div style={{ position: "absolute", right: 0, bottom: 26, width: 116, height: 92, background: "rgba(10,15,28,.96)", border: "1px solid var(--line)", borderRadius: 10, boxShadow: "0 6px 18px rgba(0,0,0,.5)", opacity: tiltHint ? 1 : 0, transform: tiltHint ? "translateY(0)" : "translateY(4px)", transition: "opacity .18s, transform .18s", pointerEvents: "none", zIndex: 5 }}>
                    {(() => {
                      const ox = 20, oy = 70, L = 58, th = tilt * D2R; // observer bottom-left, ray at the tilt angle
                      const rx = ox + Math.cos(th) * L, ry = oy - Math.sin(th) * L;
                      const hx = ox + 82; // horizon reference length
                      const aR = 20; // angle-arc radius
                      const ax = ox + Math.cos(th) * aR, ay = oy - Math.sin(th) * aR;
                      const sweep = tilt < 0 ? 1 : 0; // SVG y-down: CCW (up) arc = sweep 0
                      return (
                        <svg viewBox="0 0 116 92" width="116" height="92">
                          <line x1={ox} y1={oy} x2={hx} y2={oy} stroke="#3a4a5c" strokeWidth="1.4" strokeDasharray="3 3" />
                          <text x={hx - 32} y={oy + 12} fill="var(--dim)" fontFamily="var(--mono)" fontSize="8">horizon</text>
                          <path d={`M ${ox + aR} ${oy} A ${aR} ${aR} 0 0 ${sweep} ${ax} ${ay}`} fill="none" stroke="var(--amber)" strokeWidth="1.2" opacity="0.8" />
                          <line x1={ox} y1={oy} x2={rx} y2={ry} stroke="var(--teal)" strokeWidth="2.2" strokeLinecap="round" />
                          <polygon points={`${rx},${ry} ${rx - Math.cos(th - 0.5) * 8},${ry + Math.sin(th - 0.5) * 8} ${rx - Math.cos(th + 0.5) * 8},${ry + Math.sin(th + 0.5) * 8}`} fill="var(--teal)" />
                          <circle cx={ox} cy={oy - 5} r="3.2" fill="#dfe8ff" />
                          <line x1={ox} y1={oy - 2} x2={ox} y2={oy} stroke="#dfe8ff" strokeWidth="2.4" strokeLinecap="round" />
                          <text x={ox + 26} y={oy - 8} fill="var(--amber)" fontFamily="var(--mono)" fontSize="12" fontWeight="700">{Math.round(tilt)}°</text>
                        </svg>
                      );
                    })()}
                  </div>
                  <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 1 }}>Photo metadata has no up/down angle — set 90° for straight up; fine-tune later in Place mode.</div>
                </div>
                <div style={{ marginTop: 6, position: "relative" }}>
                  <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
                    <ML style={{ marginBottom: 1 }}>Camera height off the ground</ML>
                    <span style={{ color: "var(--teal)", fontFamily: "var(--mono)", fontSize: 11 }}>{fmtLenShort(camH)} · {camH <= 2 ? "standing" : `≈${storeys(camH)} floor${storeys(camH) > 1 ? "s" : ""}`}</span>
                  </div>
                  <input type="range" min={0} max={60} step={0.5} value={camH}
                    onPointerDown={pokeCamHint} onChange={(e) => { setCamH(+e.target.value); pokeCamHint(); }} />
                  {/* momentary side-view of the camera height while sliding */}
                  <div style={{ position: "absolute", right: 0, bottom: 26, width: 116, height: 92, background: "rgba(10,15,28,.96)", border: "1px solid var(--line)", borderRadius: 10, boxShadow: "0 6px 18px rgba(0,0,0,.5)", opacity: camHint ? 1 : 0, transform: camHint ? "translateY(0)" : "translateY(4px)", transition: "opacity .18s, transform .18s", pointerEvents: "none", zIndex: 5 }}>
                    {(() => {
                      const gy = 74, px = 28, pw = 28; // ground y, structure x + width
                      const pixH = clampN(camH * 3.2, 3, 58); // scale metres → px (cap so tall shots still fit)
                      const top = gy - pixH;
                      const eyeH = clampN(1.6 * 3.2, 4, 58); // reference person for scale
                      const floorPx = 3.3 * 3.2; // storey height in px, matching storeys()
                      const nFloors = storeys(camH);
                      const floorLines = [];
                      if (camH > 3.5) for (let k = 1; k < nFloors && k * floorPx < pixH - 2; k++) floorLines.push(gy - k * floorPx);
                      return (
                        <svg viewBox="0 0 116 92" width="116" height="92">
                          <line x1="6" y1={gy} x2="110" y2={gy} stroke="#3a4a5c" strokeWidth="1.4" />
                          <text x="6" y={gy + 10} fill="var(--dim)" fontFamily="var(--mono)" fontSize="8">ground</text>
                          {/* the structure the camera sits on, divided into storeys */}
                          <rect x={px} y={top} width={pw} height={pixH} fill="rgba(255,178,74,0.12)" stroke="rgba(255,178,74,0.8)" strokeWidth="1.2" />
                          {floorLines.map((fy, i) => <line key={i} x1={px} y1={fy} x2={px + pw} y2={fy} stroke="rgba(255,178,74,0.45)" strokeWidth="0.8" />)}
                          {/* camera eye at the top */}
                          <circle cx={px + pw / 2} cy={top - 3} r="3.2" fill="var(--teal)" />
                          <line x1={px + pw / 2} y1={top} x2={px + pw / 2} y2={top - 1} stroke="var(--teal)" strokeWidth="2" />
                          {/* reference person at ground for scale */}
                          <circle cx="13" cy={gy - eyeH - 2} r="2.2" fill="#dfe8ff" />
                          <line x1="13" y1={gy - eyeH} x2="13" y2={gy} stroke="#dfe8ff" strokeWidth="1.8" strokeLinecap="round" />
                          {/* height dimension + storeys */}
                          <line x1={px + pw + 7} y1={gy} x2={px + pw + 7} y2={top} stroke="var(--amber)" strokeWidth="1" strokeDasharray="2 2" />
                          <text x={px + pw + 11} y={(gy + top) / 2 + 1} fill="var(--amber)" fontFamily="var(--mono)" fontSize="10.5" fontWeight="700">{fmtLenShort(camH)}</text>
                          <text x={px + pw + 11} y={(gy + top) / 2 + 12} fill="var(--dim)" fontFamily="var(--mono)" fontSize="8.5">{camH <= 2 ? "standing" : `≈${nFloors} fl`}</text>
                        </svg>
                      );
                    })()}
                  </div>
                  <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 1 }}>Only matters for the 🏙 building layer — raise it if you shot from an upstairs window or balcony (metadata usually can't tell). Leave at ground for a normal shot.</div>
                </div>
              </div>
            )}
    </>
  );
}

/* ============================================================
   SENSOR CAPTURE — the in-app camera that records what EXIF drops (ENABLE_CAPTURE)

   getUserMedia live view + DeviceMotion/DeviceOrientation. At the shutter it
   samples the phone's gravity vector and compass and turns them into the SAME
   {az, el, roll} the placement uses — so a Phodar-captured shot arrives with its
   up/down angle and roll already solved (the exact things standard photo EXIF
   omits), plus GPS with real accuracy. The pose is a SEED: the sky view's
   snap-to-ridges / star-align still refine it, and the report keeps the raw
   sensor block. No App Store — a hosted PWA reaches all of this on iOS Safari.
   ============================================================ */
function SensorCapture({ onCapture, onClose }) {
  const videoRef = useRef(null), streamRef = useRef(null);
  const gRef = useRef(null), headRef = useRef(null), rawRef = useRef({});
  const [started, setStarted] = useState(false);
  const [camErr, setCamErr] = useState("");
  const [pose, setPose] = useState(null);
  const [compassAcc, setCompassAcc] = useState(null);
  const [gps, setGps] = useState(null);
  const gpsRef = useRef(null), watchRef = useRef(null);
  const [gpsBusy, setGpsBusy] = useState(false);
  /* CLEAN VIEW (👁). This screen is a viewfinder pointed at something that is
     happening NOW, and the guidance stacked under it — readout tiles, GPS line,
     raw sensors, two explanation paragraphs, the secondary shutters — leaves
     little sky to actually look at. The toggle strips it to what you cannot
     shoot without: the crosshair, a one-line pose readout, the full-res shutter,
     the record button, and the way out.
     What survives on PURPOSE even in the clean view: the camera error, the
     "no motion data" box and the missing-position warning. Those change what the
     capture IS WORTH, and a tidier screen is not worth letting someone shoot a
     whole session believing it carried attitude when it didn't. */
  const [lean, setLean] = useState(() => { try { return localStorage.getItem("phodar-capclean") === "1"; } catch (e) { return false; } });
  const toggleLean = () => setLean((v) => { const n = !v; try { localStorage.setItem("phodar-capclean", n ? "1" : "0"); } catch (e) { } return n; });
  /* NO-MOTION GUARD: DeviceOrientation/Motion simply never fire in some
     browsers (a Chrome tab on iOS, an in-app webview, desktop). Without this
     the capture looks alive — camera running, shutter working — while tilt,
     roll and bearing sit at "—" and the recorded clip carries an empty log.
     Cost a full field session to diagnose, so say it plainly. */
  const [noMotion, setNoMotion] = useState(false);
  useEffect(() => {
    if (!started) return;
    const id = setTimeout(() => { if (!gRef.current && headRef.current == null) setNoMotion(true); }, 2500);
    return () => clearTimeout(id);
  }, [started]);
  const askGps = () => {
    if (!navigator.geolocation) { setCamErr("This browser can't provide a position — you'll set it by hand on the next step."); return; }
    setGpsBusy(true);
    navigator.geolocation.getCurrentPosition(
      (p) => { const g = { lat: p.coords.latitude, lon: p.coords.longitude, alt: p.coords.altitude, acc: p.coords.accuracy, altAcc: p.coords.altitudeAccuracy }; gpsRef.current = g; setGps(g); setGpsBusy(false); },
      () => { setGpsBusy(false); setCamErr("Location is blocked for this site — allow it in Settings › Safari (or the share menu), or set your position by hand on the next step."); },
      { enableHighAccuracy: true, timeout: 10000 });
  };
  /* ⇅ flip controls the ELEVATION sense only (decoupled from bearing/roll).
     +1 default = field-correct on iOS (truth +21.9° read +20 raw); −1 inverts.
     Key bumped (…2) so devices that toggled during the inverted-default build
     reset to the corrected default. */
  const [elSign, setElSign] = useState(() => { try { return localStorage.getItem("phodar-elsign2") === "-1" ? -1 : 1; } catch (e) { return 1; } });
  const elSignRef = useRef(elSign); elSignRef.current = elSign;
  /* WHICH SENSOR STORY THIS DEVICE TELLS. iOS hands over a tilt-compensated
     camera heading (webkitCompassHeading) and an accelerometer pointing along
     the pull; nothing else does either. `modeRef` records which path is live so
     the readout can say so and the pose math can branch:
       "ios"    — webkitCompassHeading present; the field-calibrated path, untouched
       "orient" — an ABSOLUTE alpha/beta/gamma (Android): the camera pose comes
                  straight out of the rotation matrix, no accelerometer needed
       null     — no compass reference at all; tilt/roll only, bearing unknown
     gSignRef is the accelerometer's sign convention, detected at runtime by
     comparing it against the orientation angles rather than sniffing the UA.
     Last non-zero answer wins (a swinging phone can't answer). */
  const modeRef = useRef(null), gSignRef = useRef(1), betaRef = useRef(null);
  const [sensorMode, setSensorMode] = useState(null);
  const onOrient = useCallback((e) => {
    const abs = e.absolute === true;
    if (isNum(e.beta) && isNum(e.gamma)) betaRef.current = { beta: e.beta, gamma: e.gamma };
    if (isNum(e.webkitCompassHeading)) {
      headRef.current = e.webkitCompassHeading;
      if (isNum(e.webkitCompassAccuracy)) setCompassAcc(Math.abs(e.webkitCompassAccuracy));
      if (modeRef.current !== "ios") { modeRef.current = "ios"; setSensorMode("ios"); }
    } else if (abs && isNum(e.alpha) && isNum(e.beta) && isNum(e.gamma)) {
      /* Android: alpha is rotation about the device's OWN axis, NOT a camera
         heading — resolve the real one through the rotation matrix, and take
         the gravity direction from the same angles (already OS-fused, and it
         sidesteps the accelerometer sign question entirely). */
      const p = poseFromOrientation(e.alpha, e.beta, e.gamma);
      const u = upFromOrientation(e.beta, e.gamma);
      if (p && u) {
        headRef.current = p.az;
        gRef.current = { x: -u.x * 9.80665, y: -u.y * 9.80665, z: -u.z * 9.80665 };  // in the iOS convention
        if (modeRef.current !== "orient") { modeRef.current = "orient"; setSensorMode("orient"); }
      }
    }
    /* raw sensor block — kept so the landscape-compass behaviour is DIAGNOSABLE
       from a field screenshot instead of guessed at (α/β/γ + the untouched
       webkitCompassHeading, before any correction). */
    rawRef.current = { hdg: isNum(e.webkitCompassHeading) ? e.webkitCompassHeading : null, alpha: isNum(e.alpha) ? e.alpha : null, beta: isNum(e.beta) ? e.beta : null, gamma: isNum(e.gamma) ? e.gamma : null, abs, mode: modeRef.current, gSign: gSignRef.current };
  }, []);
  const onMotion = useCallback((e) => {
    const g = e.accelerationIncludingGravity;
    if (!(g && isNum(g.x) && isNum(g.y) && isNum(g.z))) return;
    const b = betaRef.current;
    if (b) { const s = gravitySign(g, b.beta, b.gamma); if (s) gSignRef.current = s; }
    /* the orientation path already wrote a fused, sign-free gravity vector */
    if (modeRef.current === "orient") return;
    const k = gSignRef.current;
    gRef.current = { x: g.x * k, y: g.y * k, z: g.z * k };
  }, []);
  /* Everything kicks off from the ▶ Start tap (iOS gates getUserMedia AND
     requestPermission behind a user gesture). ORDER MATTERS: request the
     motion/orientation permission FIRST, before awaiting getUserMedia — awaiting
     the camera first consumes the gesture, so the permission request silently
     fails and NO deviceorientation/devicemotion events ever arrive (field bug:
     GPS filled but tilt/roll/bearing stuck on "—"). */
  const start = async () => {
    setCamErr("");
    let motionGranted = true;
    try {
      if (typeof DeviceMotionEvent !== "undefined" && typeof DeviceMotionEvent.requestPermission === "function") {
        motionGranted = (await DeviceMotionEvent.requestPermission()) === "granted";
      }
      if (typeof DeviceOrientationEvent !== "undefined" && typeof DeviceOrientationEvent.requestPermission === "function") {
        const r = await DeviceOrientationEvent.requestPermission();
        motionGranted = motionGranted && r === "granted";
      }
    } catch (e) { motionGranted = false; }
    if (motionGranted) {
      /* BOTH names: iOS only ever fires "deviceorientation" (carrying
         webkitCompassHeading), while Chrome on Android puts the
         compass-referenced angles on "deviceorientationabsolute" and leaves
         plain "deviceorientation" RELATIVE to wherever the phone happened to be
         — an alpha from that event is not a bearing at all. Which one a given
         event is gets decided by its own `absolute` flag, not by its name, so
         Firefox (absolute on the plain event) works too. */
      window.addEventListener("deviceorientation", onOrient, true);
      if ("ondeviceorientationabsolute" in window) window.addEventListener("deviceorientationabsolute", onOrient, true);
      window.addEventListener("devicemotion", onMotion, true);
    } else {
      setCamErr("Motion access was denied. On iPhone: Settings › Safari › Motion & Orientation Access. On Android: allow motion sensors for this site. Then reopen this capture.");
    }
    try {
      const s = await navigator.mediaDevices.getUserMedia({ video: { facingMode: { ideal: "environment" }, width: { ideal: 1920 }, height: { ideal: 1080 } }, audio: false });
      streamRef.current = s;
      if (videoRef.current) { videoRef.current.srcObject = s; videoRef.current.play().catch(() => { }); }
    } catch (e) { setCamErr((c) => c || ("Camera blocked — allow camera access for this site (and use https). " + (e?.message || ""))); }
    /* GPS: WATCH it, don't sample it once. A recorded clip carries no EXIF, so
       the sensor fix is the ONLY position it will ever have — and a one-shot
       read can arrive after the shutter (or not at all), which left the
       position step empty and un-advanceable after an instrumented recording
       (field report). The ref also dodges the stale-closure trap: MediaRecorder
       callbacks are bound once, at record start. */
    const gotGps = (p) => {
      const g = { lat: p.coords.latitude, lon: p.coords.longitude, alt: p.coords.altitude, acc: p.coords.accuracy, altAcc: p.coords.altitudeAccuracy };
      gpsRef.current = g; setGps(g);
    };
    if (navigator.geolocation) {
      navigator.geolocation.getCurrentPosition(gotGps, () => { }, { enableHighAccuracy: true, timeout: 8000 });
      try { watchRef.current = navigator.geolocation.watchPosition(gotGps, () => { }, { enableHighAccuracy: true, maximumAge: 4000 }); } catch (e) { }
    }
    setStarted(true);
  };
  /* SMOOTHING — the raw magnetometer/accelerometer jitters frame to frame,
     worst in portrait aimed at the horizon (the compass reference, the phone's
     top edge, points near-vertical there — its noisiest pose). Exponentially
     average the gravity vector and the heading (as a unit vector, so it wraps
     cleanly) before deriving the pose, for both the live readout and the shutter
     snapshot — a steadier readout and a more reliable seed. */
  const gEmaRef = useRef(null), hEmaRef = useRef(null);
  const smoothedHeading = () => { const h = hEmaRef.current; return h && (Math.abs(h.c) + Math.abs(h.s)) > 1e-3 ? ((Math.atan2(h.s, h.c) * R2D) % 360 + 360) % 360 : headRef.current; };
  /* interface orientation (screen angle) — iOS reports webkitCompassHeading in a
     frame that flips 180° in landscape, so the pose math needs to know the hold.
     screen.orientation.angle is the modern API; window.orientation the iOS one. */
  const screenAngle = () => {
    try { const a = window.screen && window.screen.orientation && window.screen.orientation.angle; if (isNum(a)) return a; } catch (e) { }
    return isNum(window.orientation) ? window.orientation : 0;
  };
  useEffect(() => {
    let raf;
    const A = 0.18;
    const tick = () => {
      const g = gRef.current;
      if (g) {
        const e = gEmaRef.current;
        gEmaRef.current = e ? { x: e.x + (g.x - e.x) * A, y: e.y + (g.y - e.y) * A, z: e.z + (g.z - e.z) * A } : { x: g.x, y: g.y, z: g.z };
        const h = headRef.current;
        if (isNum(h)) {
          const c = Math.cos(h * D2R), s = Math.sin(h * D2R), he = hEmaRef.current;
          hEmaRef.current = he ? { c: he.c + (c - he.c) * A, s: he.s + (s - he.s) * A } : { c, s };
        }
        const pv = poseFromGravity(gEmaRef.current, smoothedHeading(), { elSign: elSignRef.current, orient: screenAngle(), headingIsCamera: modeRef.current === "orient" });
        setPose(pv);
        /* INSTRUMENTED VIDEO: log the same solved attitude the readout shows,
           on the recorder's clock. rAF paces this at the display rate (~60 Hz),
           which is far denser than the 0.25 s the stabilizer solves at — and
           the samples are already gravity-smoothed, so the log carries motion
           rather than accelerometer noise. */
        if (recordingRef.current && pv) {
          const L = logRef.current;
          const tt = (performance.now() - recT0Ref.current) / 1000;
          /* ~25 Hz is plenty — six times denser than the 0.25 s the stabilizer
             solves at — and keeps a long clip's log small enough to autosave
             (localStorage caps around 5 MB and the whole sighting shares it) */
          if (L.length < 30000 && (!L.length || tt - L[L.length - 1].t >= 0.04)) L.push({
            t: +tt.toFixed(3),
            az: +pv.az.toFixed(2), el: +pv.el.toFixed(2), roll: +(pv.roll || 0).toFixed(2),
          });
        }
      }
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, []);
  useEffect(() => () => {
    window.removeEventListener("deviceorientation", onOrient, true);
    if ("ondeviceorientationabsolute" in window) window.removeEventListener("deviceorientationabsolute", onOrient, true);
    window.removeEventListener("devicemotion", onMotion, true);
    if (watchRef.current != null) { try { navigator.geolocation.clearWatch(watchRef.current); } catch (e) { } watchRef.current = null; }
    if (streamRef.current) streamRef.current.getTracks().forEach((t) => t.stop());
  }, [onOrient, onMotion]);
  /* the capture is a full-screen camera — suppress the app's portrait lock so it
     works held in landscape (natural for a wide scene / the horizon). */
  useEffect(() => {
    document.documentElement.classList.add("capturing");
    return () => document.documentElement.classList.remove("capturing");
  }, []);
  const flip = () => setElSign((s) => { const n = s === 1 ? -1 : 1; try { localStorage.setItem("phodar-elsign2", String(n)); } catch (e) { } return n; });
  /* snapshot the SMOOTHED sensor reading — az/el/roll + provenance */
  const snapPose = () => { const g = gEmaRef.current || gRef.current, hd = smoothedHeading(), orient = screenAngle(), camHdg = modeRef.current === "orient"; return { pose: g ? poseFromGravity(g, hd, { elSign: elSignRef.current, orient, headingIsCamera: camHdg }) : null, heading: hd, compassAcc, gps: gpsRef.current || gps, gravity: g, elSign: elSignRef.current, orient, raw: { ...rawRef.current }, whenMs: Date.now() }; };
  /* QUICK: an instant getUserMedia frame — lower-res but the pose is exactly
     synced to the pixels. Good for a near/large object. */
  const shoot = () => {
    const v = videoRef.current; if (!v || !v.videoWidth) return;
    const cv = document.createElement("canvas"); cv.width = v.videoWidth; cv.height = v.videoHeight;
    cv.getContext("2d").drawImage(v, 0, 0, cv.width, cv.height);
    onCapture({ dataUrl: cv.toDataURL("image/jpeg", 0.95), w: cv.width, h: cv.height, ...snapPose() });
  };
  /* FULL-RES: freeze the pose, then hand off to the NATIVE camera for a true
     full-megapixel still (getUserMedia only yields a ~1080p video frame). The
     photo maps to the frozen pose — hold the aim while the camera opens. */
  const nativeRef = useRef(null), libRef = useRef(null), shotPoseRef = useRef(null);
  const takeFullRes = () => { shotPoseRef.current = snapPose(); if (nativeRef.current) nativeRef.current.click(); };
  const onNativeFile = (e) => { const f = e.target.files && e.target.files[0]; e.target.value = ""; if (f) onCapture({ file: f, ...(shotPoseRef.current || snapPose()) }); };
  /* ============================================================
     INSTRUMENTED VIDEO — record a clip WITH a continuous attitude log.

     The stabilizer's weakness is structural: its only absolute reference is
     the frame you aligned, so between re-anchors it is an incremental chain
     that drifts (~1.4° measured over 22 s) and freezes on frames with nothing
     to track. A phone's own attitude has the opposite profile — gravity gives
     pitch and roll absolutely, in EVERY frame, and cannot drift. Logging it
     alongside the clip lets the two cover for each other (src/video/
     sensorpath.js does the fusion; vision keeps the absolute frame, the
     sensors supply motion).

     Honest trade-off, stated in the UI: getUserMedia records at ~1080p with
     no lens switching and no optical zoom, so this is a MEASUREMENT mode, not
     the way to shoot your best-looking evidence. The log's clock is anchored
     at recorder start; the residual constant offset to the encoded timeline is
     recovered later by syncSensor, so it does not need to be exact here. */
  const recRef = useRef(null), chunksRef = useRef([]), logRef = useRef([]), recT0Ref = useRef(0);
  const [recording, setRecording] = useState(false);
  const [recSecs, setRecSecs] = useState(0);
  const recordingRef = useRef(false); recordingRef.current = recording;
  useEffect(() => {
    if (!recording) return;
    const id = setInterval(() => setRecSecs(Math.max(0, (performance.now() - recT0Ref.current) / 1000)), 250);
    return () => clearInterval(id);
  }, [recording]);
  const startRec = () => {
    const st = streamRef.current;
    if (!st) { setCamErr("Camera isn't running yet — tap ▶ Start first."); return; }
    if (typeof MediaRecorder === "undefined") { setCamErr("This browser can't record video (no MediaRecorder)."); return; }
    const mime = ["video/mp4", "video/webm;codecs=vp9", "video/webm"].find((m) => { try { return MediaRecorder.isTypeSupported(m); } catch (e) { return false; } }) || "";
    let rec;
    try { rec = new MediaRecorder(st, mime ? { mimeType: mime } : undefined); }
    catch (e) { setCamErr("Couldn't start recording: " + (e?.message || e)); return; }
    chunksRef.current = []; logRef.current = [];
    rec.ondataavailable = (e) => { if (e.data && e.data.size) chunksRef.current.push(e.data); };
    rec.onstop = async () => {
      const blob = new Blob(chunksRef.current, { type: mime || "video/mp4" });
      const ext = /mp4/.test(mime) ? "mp4" : "webm";
      const file = new File([blob], `phodar-instrumented.${ext}`, { type: blob.type });
      const log = logRef.current.slice();
      setRecording(false);
      if (!log.length) { setCamErr("Recorded, but no motion data arrived — check Settings › Safari › Motion & Orientation Access."); }
      /* LAST CHANCE at a position: a recorded clip has no EXIF, so if the watch
         hasn't produced a fix yet this is the final opportunity to get one —
         without it the sighting opens at step 2 with nothing to continue from. */
      let snap = snapPose();
      if (!(snap.gps && isNum(snap.gps.lat)) && navigator.geolocation) {
        const g = await new Promise((res) => {
          let done = false;
          const fin = (v) => { if (!done) { done = true; res(v); } };
          try {
            navigator.geolocation.getCurrentPosition(
              (p) => fin({ lat: p.coords.latitude, lon: p.coords.longitude, alt: p.coords.altitude, acc: p.coords.accuracy, altAcc: p.coords.altitudeAccuracy }),
              () => fin(null), { enableHighAccuracy: true, timeout: 3500 });
          } catch (e) { fin(null); }
          setTimeout(() => fin(null), 4000);   // never leave the user staring at a stopped recording
        });
        if (g) { gpsRef.current = g; setGps(g); snap = { ...snap, gps: g }; }
        else setCamErr("Recorded, but no GPS fix — you'll need to set your position by hand on the next step.");
      }
      onCapture({ file, sensorPath: log, ...snap });
    };
    recT0Ref.current = performance.now();
    try { rec.start(250); } catch (e) { setCamErr("Couldn't start recording: " + (e?.message || e)); return; }
    recRef.current = rec;
    setRecSecs(0); setRecording(true);
  };
  const stopRec = () => { const r = recRef.current; recRef.current = null; if (r && r.state !== "inactive") { try { r.stop(); } catch (e) { setRecording(false); } } else setRecording(false); };
  /* NIGHT / LONG-EXPOSURE workflow: iOS gives web apps only a stripped-down
     capture sheet — Night mode & long exposure are exclusive to the native
     Camera app. So: freeze the aim HERE (sensor pose captured now), switch to
     the Camera app for the long shot, come back, attach it from the library —
     the photo carries the frozen pose. */
  const [nightPose, setNightPose] = useState(null);
  const freezeForNight = () => { const p = snapPose(); shotPoseRef.current = p; setNightPose(p); };
  const q = poseQuality(compassAcc, true, sensorMode);
  const readVal = (v, suf) => isNum(v) ? Math.round(v) + (suf || "") : "—";
  return (
    <div style={{ position: "fixed", inset: 0, zIndex: 10000, background: "#000", display: "flex", flexDirection: "column" }}>
      <video ref={videoRef} muted playsInline autoPlay style={{ position: "absolute", inset: 0, width: "100%", height: "100%", objectFit: "cover", background: "#000" }} />
      {/* crosshair */}
      {started && (
        <div style={{ position: "absolute", left: "50%", top: "50%", transform: "translate(-50%,-50%)", pointerEvents: "none", color: "rgba(64,199,178,.9)" }}>
          <div style={{ width: 34, height: 1, background: "currentColor", position: "absolute", left: -17, top: 0 }} />
          <div style={{ width: 1, height: 34, background: "currentColor", position: "absolute", left: 0, top: -17 }} />
        </div>
      )}
      {/* header */}
      <div style={{ position: "relative", zIndex: 2, display: "flex", alignItems: "center", justifyContent: "space-between", padding: "calc(8px + env(safe-area-inset-top)) 12px 8px" }}>
        {lean
          ? <span />
          : <span style={{ color: "#fff", fontFamily: "var(--mono)", fontSize: 12, background: "rgba(0,0,0,.4)", padding: "3px 8px", borderRadius: 8 }}>📷 Sensor capture <b style={{ color: "var(--amber)" }}>beta</b></span>}
        <div style={{ display: "flex", gap: 8 }}>
          {started && (
            <button onClick={toggleLean} title={lean ? "show the guidance" : "clean view — hide everything but the essentials"}
              style={{ background: lean ? "rgba(64,199,178,.25)" : "rgba(0,0,0,.5)", color: lean ? "var(--teal)" : "#fff", border: `1px solid ${lean ? "var(--teal)" : "rgba(255,255,255,.3)"}`, borderRadius: 18, width: 36, height: 36, fontSize: 16 }}>👁</button>
          )}
          <button onClick={onClose} style={{ background: "rgba(0,0,0,.5)", color: "#fff", border: "1px solid rgba(255,255,255,.3)", borderRadius: 18, width: 36, height: 36, fontSize: 18 }}>✕</button>
        </div>
      </div>
      <div style={{ flex: 1 }} />
      {/* start gate OR live readout + shutter */}
      {!started ? (
        <div style={{ position: "relative", zIndex: 2, padding: "0 20px calc(40px + env(safe-area-inset-bottom))", textAlign: "center", color: "#fff" }}>
          <div style={{ fontSize: 13, lineHeight: 1.6, marginBottom: 16, background: "rgba(0,0,0,.5)", padding: 14, borderRadius: 12 }}>
            Shoot the sighting <b>in Phodar</b> and it records the camera's up/down angle, roll and heading — the pose EXIF leaves out — so the sky view opens already pointed. The photo itself is taken at <b>full resolution</b> by the phone camera (crucial for distant objects); the sensors only add what the metadata drops. Tap Start to allow the camera and motion sensors.
          </div>
          <button className="btn amber" style={{ padding: "14px 26px", fontSize: 15 }} onClick={start}>▶ Start camera &amp; sensors</button>
          {camErr && <div style={{ color: "var(--red)", fontSize: 12, marginTop: 10 }}>{camErr}</div>}
        </div>
      ) : (
        <div style={{ position: "relative", zIndex: 2, padding: "0 14px calc(20px + env(safe-area-inset-bottom))" }}>
          {camErr && <div style={{ color: "var(--amber)", fontSize: 11.5, marginBottom: 8, background: "rgba(0,0,0,.55)", padding: "6px 10px", borderRadius: 8 }}>{camErr}</div>}
          {/* live pose readout — the one thing the clean view keeps, because
              recording a pose you can't see is the whole failure mode this
              screen exists to avoid. One line instead of three tiles. */}
          {lean ? (
            <div style={{ fontFamily: "var(--mono)", fontSize: 12.5, background: "rgba(0,0,0,.5)", borderRadius: 9, padding: "6px 10px", marginBottom: 10, textAlign: "center", color: "var(--teal)" }}>
              {pose
                ? <>{readVal(pose.el, "°")} up · {readVal(pose.roll, "°")} roll · {isNum(headRef.current) ? readVal(pose.az, "° " + compass8(pose.az)) : "no compass"}</>
                : <span style={{ color: "var(--amber)" }}>waiting for motion sensors…</span>}
              {!gps && <span onClick={askGps} style={{ color: "var(--amber)", fontWeight: 800, cursor: "pointer" }}> · 📍 no fix</span>}
            </div>
          ) : (
            <div style={{ display: "flex", gap: 8, marginBottom: 10, fontFamily: "var(--mono)" }}>
              {[["tilt", pose ? readVal(pose.el, "°") : "—", "up/down"], ["roll", pose ? readVal(pose.roll, "°") : "—", "level"], ["bearing", pose && isNum(headRef.current) ? readVal(pose.az, "° " + compass8(pose.az)) : "—", "compass"]].map(([k, v, sub]) => (
                <div key={k} style={{ flex: 1, background: "rgba(0,0,0,.55)", borderRadius: 10, padding: "7px 6px", textAlign: "center" }}>
                  <div style={{ fontSize: 10, color: "var(--dim)", textTransform: "uppercase", letterSpacing: 0.5 }}>{k}</div>
                  <div style={{ fontSize: 17, fontWeight: 700, color: "var(--teal)" }}>{v}</div>
                  <div style={{ fontSize: 9, color: "var(--dim)" }}>{sub}</div>
                </div>
              ))}
            </div>
          )}
          {!lean && <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between", gap: 10, background: "rgba(0,0,0,.5)", borderRadius: 10, padding: "6px 10px", marginBottom: 10, fontSize: 10.5, color: "#cfe" }}>
            {/* A RECORDED clip carries no EXIF, so this fix is the only position
                it will ever have — make its absence loud and RETRYABLE. Tapping
                re-requests from a fresh user gesture, which is what actually
                makes iOS show the prompt when it was dismissed or never asked. */}
            <span onClick={askGps} style={{ cursor: "pointer", color: gps ? undefined : "var(--amber)", fontWeight: gps ? undefined : 800 }}>
              {gps ? `GPS ${gps.lat.toFixed(5)}, ${gps.lon.toFixed(5)}${isNum(gps.acc) ? ` ±${Math.round(gps.acc)}m` : ""}`
                : (gpsBusy ? "GPS — finding…" : "📍 No position — tap to allow")}
            </span>
            <button onClick={flip} style={{ background: "transparent", border: "1px solid rgba(255,255,255,.35)", color: "#fff", borderRadius: 8, padding: "3px 8px", fontSize: 10.5 }}>⇅ flip tilt</button>
          </div>}
          {/* raw-sensor diagnostic — so a landscape screenshot shows the actual
             numbers (untouched compass + α/β/γ + screen angle) and the compass
             frame can be solved from data, not guessed. */}
          {!lean && <div style={{ background: "rgba(0,0,0,.5)", borderRadius: 8, padding: "4px 10px", marginBottom: 10, fontFamily: "var(--mono)", fontSize: 9.5, color: "#8ab", textAlign: "center", letterSpacing: 0.2 }}>
            raw hdg {isNum(headRef.current) ? Math.round(headRef.current) + "°" : "—"} · scr {Math.round(screenAngle())}° · α{isNum(rawRef.current.alpha) ? Math.round(rawRef.current.alpha) : "—"} β{isNum(rawRef.current.beta) ? Math.round(rawRef.current.beta) : "—"} γ{isNum(rawRef.current.gamma) ? Math.round(rawRef.current.gamma) : "—"} · {sensorMode || "no-compass"}{gSignRef.current === -1 ? " g−" : ""}
          </div>}
          {!lean && (!pose
            ? <div style={{ fontSize: 10.5, color: "var(--amber)", textAlign: "center", marginBottom: 6, textShadow: "0 1px 3px #000", lineHeight: 1.5 }}>Waiting for motion sensors — move the phone slightly. If tilt/roll stay blank: on iPhone turn ON <b>Settings › Safari › Motion &amp; Orientation Access</b>, on Android allow motion sensors for this site, then reopen.</div>
            : <div style={{ fontSize: 10, color: q.headingOk ? "var(--teal)" : "var(--amber)", textAlign: "center", marginBottom: 6, textShadow: "0 1px 3px #000" }}>{q.note}</div>)}
          {!lean && <div style={{ fontSize: 9.5, color: "#9ab", textAlign: "center", marginBottom: 10, textShadow: "0 1px 3px #000" }}>Aim at the horizon → tilt should read ≈ 0°; straight up → ≈ 90°. If it's inverted, tap ⇅ flip.</div>}
          {/* hidden native camera for the full-resolution still + a library
              picker for the Night-mode workflow (no `capture` attr) */}
          <input ref={nativeRef} type="file" accept="image/*" capture="environment" onChange={onNativeFile} style={{ display: "none" }} />
          <input ref={libRef} type="file" accept="image/*" onChange={(e) => { setNightPose(null); onNativeFile(e); }} style={{ display: "none" }} />
          {/* PRIMARY: full-resolution native photo (far objects need every pixel).
              SECONDARY: instant lower-res frame with an exactly-synced pose.
              NIGHT: freeze the aim → shoot in the Camera app → attach. */}
          <div style={{ display: "flex", flexDirection: "column", alignItems: "center", gap: 8 }}>
            <button onClick={takeFullRes} className="btn amber" style={{ padding: "13px 22px", fontSize: 15, borderRadius: 30 }}>📸 Full-resolution photo</button>
            {!lean && <div style={{ fontSize: 9, color: "#9ab", textAlign: "center", textShadow: "0 1px 3px #000", maxWidth: 280 }}>Opens the phone camera at full megapixels and keeps this aim — hold steady and shoot right away. Best for distant objects.</div>}
            {noMotion && (
              <div style={{ background: "rgba(229,72,77,.18)", border: "1px solid #e5484d", color: "#ffd7d9", borderRadius: 10, padding: "8px 10px", fontSize: 11, maxWidth: 320, textAlign: "center", lineHeight: 1.45 }}>
                <b>No motion data.</b> On iPhone, use the home-screen app rather
                than a browser tab — iOS only gives motion access there — or
                allow Settings › Safari › Motion &amp; Orientation Access. On
                Android, allow motion sensors for this site (and use https).
                Recording still works, but the clip won't carry attitude.
              </div>
            )}
            {/* INSTRUMENTED VIDEO — records the attitude log with the clip */}
            <button onClick={recording ? stopRec : startRec}
              style={{ background: recording ? "rgba(229,72,77,.85)" : "rgba(64,199,178,.22)", color: recording ? "#fff" : "var(--teal)", border: `1px solid ${recording ? "#e5484d" : "var(--teal)"}`, borderRadius: 30, padding: "11px 20px", fontSize: 14, fontWeight: 700 }}>
              {recording ? `⏹ Stop  ${recSecs.toFixed(1)}s` : "🎬 Record with motion data"}
            </button>
            {/* the explanation goes in the clean view; the missing-position
                warning does NOT — a clip with no fix and no EXIF is a sighting
                with no observer, which is worth interrupting a tidy screen for */}
            {(!lean || !gps) && (
              <div style={{ fontSize: 9, color: "#9ab", textAlign: "center", textShadow: "0 1px 3px #000", maxWidth: 300 }}>
                {!lean && <>
                  Logs the phone's tilt, roll and bearing continuously — the stabilizer then has a
                  drift-free reference in every frame, and frames with nothing to track stop freezing.
                  <b style={{ color: "#cde" }}> Records at ~1080p with no zoom</b>, so use it when the
                  measurement matters more than the footage.
                </>}
                {!gps && <b onClick={askGps} style={{ color: "var(--amber)", display: "block", marginTop: lean ? 0 : 3, fontSize: lean ? 10 : 9, cursor: "pointer" }}>
                  No position yet — a recorded clip carries no EXIF, so tap here for a fix
                  or you'll have to type your location on the next step.
                </b>}
              </div>
            )}
            {!lean && <div style={{ display: "flex", gap: 8, alignItems: "center" }}>
              <button onClick={shoot} style={{ background: "rgba(0,0,0,.5)", color: "#fff", border: "1px solid rgba(255,255,255,.3)", borderRadius: 20, padding: "6px 14px", fontSize: 12 }}>⚡ Quick frame (lower-res)</button>
              <button onClick={nightPose ? () => libRef.current && libRef.current.click() : freezeForNight}
                style={{ background: nightPose ? "rgba(64,199,178,.25)" : "rgba(0,0,0,.5)", color: nightPose ? "var(--teal)" : "#fff", border: `1px solid ${nightPose ? "var(--teal)" : "rgba(255,255,255,.3)"}`, borderRadius: 20, padding: "6px 14px", fontSize: 12 }}>
                {nightPose ? "📁 Attach the shot" : "🌙 Night / long exposure"}
              </button>
            </div>}
            {!lean && nightPose && nightPose.pose && (
              <div style={{ fontSize: 9.5, color: "var(--teal)", textAlign: "center", textShadow: "0 1px 3px #000", maxWidth: 300, lineHeight: 1.5 }}>
                Aim frozen ✓ ({Math.round(nightPose.pose.az)}° {compass8(nightPose.pose.az)} · {Math.round(nightPose.pose.el)}° up). iOS only gives web apps a basic camera — Night mode lives in the Camera app. Switch there, shoot from THIS spot at THIS aim, come back and 📁 attach it.
              </div>
            )}
          </div>
        </div>
      )}
    </div>
  );
}

/* ============================================================
   AERIAL MEASURE — the looking-DOWN workflow (appMode === "aerial")

   A downward-looking sensor of KNOWN position + altitude geolocates a ground
   target from a single frame (the ground is a known surface — the dual of sky
   triangulation, which needs two observers because the target's range is
   unknown). The user gives the platform's lat/lon (pin), MSL altitude, and the
   sensor pose (look azimuth + depression angle + FOV); tapping the target in the
   image casts its pixel sight-line to the ground plane via geolocate.js and
   reads back real lat/lon, slant range, and ground-sample distance. Two marks
   bracket a true ground size. Everything is stored on `src.plat` / `src.sensor`
   / `src.aTarget` / `src.aSpan`, kept out of the sky-mode fields.

   Telemetry-free (redacted-footage) GCP-homography geolocation is the next
   step; this is the ray-cast (known-pose) path.
   ============================================================ */
const M_PER_FT = 0.3048;
function AerialMeasure({ src, update, unitsImp }) {
  const mediaRef = useRef(null);
  const [mode, setMode] = useState("target");     // 'target' | 'size'
  const [demBusy, setDemBusy] = useState(false);
  const [copied, setCopied] = useState(false);
  const natW = isNum(src.natW) ? +src.natW : 0, natH = isNum(src.natH) ? +src.natH : 0;
  const isVideo = src.mediaKind === "video";

  /* platform (sensor) position + MSL altitude — kept in src.plat so it never
     collides with the sky-mode observer fields (src.lat/lon/alt). */
  /* seed the platform from the frame's own GPS metadata when present — drone /
     military FMV often embeds the sensor's lat/lon/altitude. Honest: it's the
     file's telemetry, not a fabricated value. Falls back to unset (user pins). */
  const plat = src.plat || {};
  const platLat = isNum(plat.lat) ? +plat.lat : (isNum(src.meta?.lat) ? +src.meta.lat : null);
  const platLon = isNum(plat.lon) ? +plat.lon : (isNum(src.meta?.lon) ? +src.meta.lon : null);
  const platAlt = isNum(plat.alt) ? +plat.alt : (isNum(src.meta?.alt) ? +src.meta.alt : 1000);    // metres MSL
  const groundAlt = isNum(src.groundAlt) ? +src.groundAlt : 0;
  /* target HEIGHT ABOVE GROUND (m). 0 = the target sits on the ground (a
     vehicle, a building). >0 = an AIRBORNE object (a drone, an orb) — its
     sight-line must be stopped at its own altitude plane, not the ground, or it
     geolocates to the ground point beneath it (its shadow). With one platform
     this height is a FREE unknown, exactly like assumed distance in sky mode. */
  const objH = isNum(src.objH) ? +src.objH : 0;
  const setPlat = (p) => update({ plat: { ...plat, ...p } });

  /* sensor pose: look azimuth (true°), depression (el is NEGATIVE), FOV (°).
     az seeds from the EXIF platform heading when present; el defaults to a
     typical oblique −30°; fov reuses the measure-step src.fovH (EXIF lens). */
  const sensor = src.sensor || {};
  const az = isNum(sensor.az) ? +sensor.az : (isNum(src.meta?.azTrue) ? +src.meta.azTrue : (isNum(src.meta?.az) ? +src.meta.az : 0));
  const el = isNum(sensor.el) ? +sensor.el : -30;
  const fov = isNum(src.fovH) ? +src.fovH : (isNum(src.meta?.fovH) ? +src.meta.fovH : 68);
  const setSensor = (p) => update({ sensor: { ...sensor, az, el, ...p } });

  const cam = { natW, natH, az, el, roll: 0, fov, k: 0 };
  const platform = { lat: platLat, lon: platLon, alt: platAlt };
  const telemetryOk = isNum(platLat) && isNum(platLon) && platAlt > groundAlt && natW > 0 && el < 0;

  /* METHOD: 'telemetry' (ray-cast off a known platform pose) or 'gcp' (fit a
     planar image→ground homography from ≥4 ground control points — no platform
     position, altitude, or gimbal angle needed). The GCP path is the redacted-
     footage answer: identify features whose real lat/lon you can read off a map. */
  const method = src.aMethod === "gcp" ? "gcp" : "telemetry";
  const setMethod = (m) => update({ aMethod: m });
  const gcps = Array.isArray(src.gcps) ? src.gcps : [];
  const gcpsReady = gcps.filter((g) => isNum(g.px) && isNum(g.lat) && isNum(g.lon));
  const [gsel, setGsel] = useState(0);

  const target = src.aTarget && isNum(src.aTarget.x) ? src.aTarget : null;
  const span = Array.isArray(src.aSpan) ? src.aSpan.filter((p) => p && isNum(p.x)) : [];
  /* moving-target track: the SAME target marked at several video timestamps.
     Each geolocates through the (assumed constant) pose/homography; the ground
     path drives groundKinematics for real speed + heading. */
  const track = Array.isArray(src.aTrack) ? src.aTrack.filter((p) => p && isNum(p.x) && isNum(p.t)) : [];
  const [vt, setVt] = useState(0);   // current video mark time (s)
  const [dur, setDur] = useState(0);

  /* fit the GCP homography (least squares; rms = ground reprojection error, m) */
  const geoH = method === "gcp" ? groundHomography(gcpsReady) : null;
  const solveOk = method === "telemetry" ? telemetryOk : !!(geoH && natW > 0);

  /* pixel→ground on an arbitrary MSL plane. GCP homography is fixed to the
     ground plane, so it can't lift off it — an airborne target there resolves to
     its ground shadow (flagged in the UI). Ray-cast honours any plane, so an
     airborne object is intersected at groundAlt + objH. */
  const objAlt = groundAlt + objH;
  const airborne = objH > 0;
  const locateAt = (x, y, alt) => method === "gcp"
    ? (geoH ? pixelToGroundH(x, y, geoH) : null)
    : (telemetryOk ? pixelToGround(x, y, cam, platform, alt) : null);
  const locate = (x, y) => locateAt(x, y, groundAlt);        // ground (footprint)
  const locateObj = (x, y) => locateAt(x, y, objAlt);        // the object's own plane (target/size/track)

  /* geolocation results (live) */
  const tGround = target && solveOk ? locateObj(target.x, target.y) : null;
  const gsd = telemetryOk ? centerGSD(cam, platform, groundAlt) : null;   // slant-based; telemetry only
  const spanM = span.length === 2 && solveOk
    ? (method === "gcp" ? groundSpanH(span[0], span[1], geoH) : groundSpanM(span[0], span[1], cam, platform, objAlt))
    : null;
  const tRange = tGround && method === "telemetry" ? haversineM(platform, tGround) : null;
  const tBearing = tGround && method === "telemetry" ? bearingDegGround(platform, tGround) : null;

  /* ground footprint: the four image corners cast to the ground (null for any
     corner whose ray misses on the oblique telemetry path). */
  const footprint = solveOk
    ? [[0, 0], [natW, 0], [natW, natH], [0, natH]].map(([x, y]) => locate(x, y))
    : [];

  /* moving-target ground kinematics (fixed-pose assumption — the platform is
     hovering / the GCPs hold across the marked frames). Each track mark
     geolocates on the object's plane, then groundKinematics gives distance-true
     speed + heading (horizontal, at the assumed height for an airborne object). */
  const trackGeo = solveOk && track.length >= 2
    ? track.map((p) => { const g = locateObj(p.x, p.y); return g && isNum(g.lat) ? { t: p.t, lat: g.lat, lon: g.lon } : null; }).filter(Boolean)
    : [];
  const kin = trackGeo.length >= 2 ? groundKinematics(trackGeo) : null;

  /* map a pointer event on the media to natural pixel coords (rect read LIVE —
     iOS layout shifts make any cached rect stale within a gesture). */
  const evToNat = (e) => {
    const el2 = mediaRef.current; if (!el2 || !(natW > 0)) return null;
    const r = el2.getBoundingClientRect();
    const x = ((e.clientX - r.left) / r.width) * natW;
    const y = ((e.clientY - r.top) / r.height) * natH;
    if (x < 0 || y < 0 || x > natW || y > natH) return null;
    return { x: +x.toFixed(1), y: +y.toFixed(1) };
  };
  const onTap = (e) => {
    const p = evToNat(e); if (!p) return;
    if (mode === "gcp" && method === "gcp") {
      const g = { px: p.x, py: p.y, lat: null, lon: null };
      const next = [...gcps, g];
      update({ gcps: next });
      setGsel(next.length - 1);          // select the fresh point so the map/inputs target it
    } else if (mode === "size") {
      const next = span.length >= 2 ? [p] : [...span, p];   // 3rd tap restarts the pair
      update({ aSpan: next });
    } else if (mode === "track") {
      /* one mark per timestamp: replace any existing point within 1/60 s of the
         current time, else add — then keep the track time-sorted. */
      const t = +vt.toFixed(3);
      const rest = track.filter((q) => Math.abs(q.t - t) > 0.016);
      update({ aTrack: [...rest, { t, x: p.x, y: p.y }].sort((a, b) => a.t - b.t) });
    } else update({ aTarget: p });       // default (incl. a stale 'gcp' mode in telemetry)
  };
  const setGcp = (i, patch) => update({ gcps: gcps.map((g, j) => (j === i ? { ...g, ...patch } : g)) });
  const delGcp = (i) => { update({ gcps: gcps.filter((_, j) => j !== i) }); setGsel((s) => clampN(s > i ? s - 1 : s, 0, Math.max(0, gcps.length - 2))); };
  const seekVideo = (t) => { setVt(t); const v = mediaRef.current; if (v && isVideo) { try { v.currentTime = t; } catch (e) { } } };

  /* seed the target ground elevation from terrain under the platform (a decent
     flat-ground proxy; the target's own DEM refinement is a later step). */
  const grabGroundDem = async () => {
    if (!isNum(platLat) || !isNum(platLon)) return;
    setDemBusy(true);
    try { const h = await demElevation(platLat, platLon); update({ groundAlt: +h.toFixed(0) }); } catch (e) { }
    setDemBusy(false);
  };

  const toDisp = (m) => (unitsImp ? m / M_PER_FT : m);
  const fromDisp = (v) => (unitsImp ? v * M_PER_FT : v);
  const altUnit = unitsImp ? "ft" : "m";

  const dot = (p, col, key, lbl) => {
    if (!p || !(natW > 0)) return null;
    return (
      <div key={key} style={{ position: "absolute", left: `${(p.x / natW) * 100}%`, top: `${(p.y / natH) * 100}%`, transform: "translate(-50%,-50%)", pointerEvents: "none" }}>
        <div style={{ width: 14, height: 14, borderRadius: 8, border: `2px solid ${col}`, boxShadow: "0 0 0 1px rgba(0,0,0,.6)" }} />
        {lbl && <div style={{ position: "absolute", left: 16, top: -2, color: col, fontFamily: "var(--mono)", fontSize: 10, textShadow: "0 0 3px #000" }}>{lbl}</div>}
      </div>
    );
  };

  useEffect(() => {
    if (isVideo && mediaRef.current) {
      const v = mediaRef.current;
      const onMeta = () => {
        if (v.duration && isFinite(v.duration)) setDur(v.duration);
        /* backfill natW/natH from the clip itself — the measure step normally
           sets them, but jumping straight here (or a reloaded session) can leave
           them unset, and every tap-to-mark needs them. */
        if (!(natW > 0) && v.videoWidth) update({ natW: v.videoWidth, natH: v.videoHeight });
        /* seek to the marked frame (or nudge frame 0) — iOS Safari leaves a fresh
           <video> blank until it decodes a frame, and marking needs it visible. */
        if (isNum(src.A?.videoTime)) { try { v.currentTime = +src.A.videoTime; setVt(+src.A.videoTime); } catch (e) { } }
        else { try { v.currentTime = 0.03; } catch (e) { } }
      };
      if (v.readyState >= 1) onMeta(); else v.addEventListener("loadedmetadata", onMeta, { once: true });
    }
  }, [isVideo, src.mediaUrl, src.A?.videoTime, natW]);

  const gcpDot = (g, i) => {
    if (!(natW > 0)) return null;
    const placed = isNum(g.lat) && isNum(g.lon), sel = i === gsel;
    const col = placed ? "#7ee0a0" : "#ffb24a";
    return (
      <div key={"g" + i} style={{ position: "absolute", left: `${(g.px / natW) * 100}%`, top: `${(g.py / natH) * 100}%`, transform: "translate(-50%,-50%)", pointerEvents: "none" }}>
        <div style={{ width: sel ? 18 : 14, height: sel ? 18 : 14, borderRadius: 10, border: `2px solid ${col}`, background: sel ? "rgba(126,224,160,.25)" : "transparent", boxShadow: "0 0 0 1px rgba(0,0,0,.6)" }} />
        <div style={{ position: "absolute", left: sel ? 18 : 15, top: -2, color: col, fontFamily: "var(--mono)", fontSize: 10, fontWeight: 700, textShadow: "0 0 3px #000" }}>{i + 1}{placed ? "" : "?"}</div>
      </div>
    );
  };
  return (
    <div>
      <div style={{ fontSize: 12, color: "var(--dim)", padding: "0 2px 8px", lineHeight: 1.5 }}>
        A <b style={{ color: "var(--teal)" }}>downward-looking</b> sensor geolocates the ground under it — no second observer. {method === "gcp"
          ? <>No telemetry needed: mark <b style={{ color: "var(--ink)" }}>≥4 ground features</b> whose real lat/lon you can read off a map, then tap the target.</>
          : <>Give the platform's spot, height, and where the sensor pointed, then tap the target.</>}
      </div>

      {/* method: known platform pose (ray-cast) vs ground control points (redacted) */}
      <div style={{ display: "inline-flex", border: "1px solid var(--line)", borderRadius: 8, overflow: "hidden", marginBottom: 8 }}>
        {[["telemetry", "📡 Known platform"], ["gcp", "🗺 Ground points (no telemetry)"]].map(([m, lbl]) => (
          <button key={m} onClick={() => setMethod(m)}
            style={{ border: "none", padding: "6px 11px", fontSize: 11.5, cursor: "pointer", background: method === m ? "var(--teal)" : "transparent", color: method === m ? "var(--bg)" : "var(--dim)", fontWeight: method === m ? 700 : 400 }}>{lbl}</button>
        ))}
      </div>

      {/* ── the frame, with tap-to-mark ── */}
      {src.mediaUrl ? (
        <div style={{ position: "relative", width: "100%", borderRadius: 10, overflow: "hidden", border: "1px solid var(--line)", background: "#000" }}
          onPointerDown={onTap}>
          {isVideo
            ? <video ref={mediaRef} src={src.mediaUrl} muted playsInline preload="auto" style={{ display: "block", width: "100%" }} />
            : <img ref={mediaRef} src={src.mediaUrl} alt="frame" draggable={false} style={{ display: "block", width: "100%" }} />}
          {method === "gcp" && gcps.map((g, i) => gcpDot(g, i))}
          {track.length > 1 && (
            <svg viewBox={`0 0 ${natW} ${natH}`} preserveAspectRatio="none" style={{ position: "absolute", inset: 0, width: "100%", height: "100%", pointerEvents: "none" }}>
              <polyline points={track.map((p) => `${p.x},${p.y}`).join(" ")} fill="none" stroke="#8FB4FF" strokeWidth={Math.max(1, natW / 500)} opacity="0.8" />
            </svg>
          )}
          {track.map((p, i) => dot(p, Math.abs(p.t - vt) < 0.05 ? "#c9d8ff" : "#8FB4FF", "trk" + i, String(i + 1)))}
          {dot(target, "var(--teal)", "t", "target")}
          {span.map((p, i) => dot(p, "var(--amber)", "s" + i, i === 0 ? "size ①" : "size ②"))}
        </div>
      ) : (
        <div style={{ padding: 20, textAlign: "center", color: "var(--dim)", border: "1px dashed var(--line)", borderRadius: 10 }}>
          Add the aerial frame on the previous step first.
        </div>
      )}

      {/* video scrubber — seek to a frame, then mark the target there (Track mode) */}
      {src.mediaUrl && isVideo && dur > 0 && (
        <div style={{ display: "flex", alignItems: "center", gap: 8, marginTop: 6 }}>
          <span style={{ fontFamily: "var(--mono)", fontSize: 11, color: "var(--dim)", minWidth: 78 }}>t {vt.toFixed(2)}s</span>
          <input type="range" min={0} max={dur} step={Math.max(0.01, dur / 600)} value={vt} onChange={(e) => seekVideo(+e.target.value)} style={{ flex: 1 }} />
        </div>
      )}

      {/* mark-mode toggle */}
      <div style={{ display: "flex", gap: 6, marginTop: 8, flexWrap: "wrap" }}>
        {method === "gcp" && (
          <button className="btn sm" onClick={() => setMode("gcp")}
            style={{ flex: 1, background: mode === "gcp" ? "var(--teal)" : "", color: mode === "gcp" ? "var(--bg)" : "" }}>🗺 Add ground point</button>
        )}
        {[["target", "🎯 Mark target"], ["size", "📏 Bracket size"]].map(([m, lbl]) => (
          <button key={m} className="btn sm" onClick={() => setMode(m)}
            style={{ flex: 1, background: mode === m ? "var(--teal)" : "", color: mode === m ? "var(--bg)" : "" }}>{lbl}</button>
        ))}
        {isVideo && (
          <button className="btn sm" onClick={() => setMode("track")}
            style={{ flex: 1, background: mode === "track" ? "var(--teal)" : "", color: mode === "track" ? "var(--bg)" : "" }}>🛰 Track motion</button>
        )}
        {(target || span.length) ? <button className="btn sm ghost" style={{ color: "var(--red)" }} onClick={() => update({ aTarget: null, aSpan: [] })}>Clear marks</button> : null}
      </div>

      {/* ── moving-target track (video) ── */}
      {isVideo && (mode === "track" || track.length > 0) && (
        <div style={{ marginTop: 12 }}>
          <ML>Motion track — scrub, then tap the target on each frame (🛰 mode)</ML>
          {track.length === 0 && <div style={{ fontSize: 12, color: "var(--amber)", padding: "2px 2px" }}>Pick 🛰 Track motion, scrub to a moment, and tap the target. Repeat at ≥2 times to get speed + heading.</div>}
          {track.length > 0 && (
            <div style={{ display: "flex", flexWrap: "wrap", gap: 5, marginTop: 4 }}>
              {track.map((p, i) => (
                <span key={i} onClick={() => seekVideo(p.t)}
                  style={{ display: "inline-flex", alignItems: "center", gap: 4, fontFamily: "var(--mono)", fontSize: 11, padding: "3px 7px", borderRadius: 12, cursor: "pointer", border: `1px solid ${Math.abs(p.t - vt) < 0.05 ? "var(--teal)" : "var(--line)"}`, color: "#8FB4FF" }}>
                  {i + 1}·{p.t.toFixed(2)}s
                  <b style={{ color: "var(--red)", cursor: "pointer" }} onClick={(e) => { e.stopPropagation(); update({ aTrack: track.filter((_, j) => j !== i) }); }}>✕</b>
                </span>
              ))}
            </div>
          )}
          {kin && (
            <div style={{ fontFamily: "var(--mono)", fontSize: 12, lineHeight: 1.7, marginTop: 6, color: "var(--dim)" }}>
              speed avg <b style={{ color: "var(--teal)" }}>{fmtSpeed(kin.avgSpeedMS)}</b> · peak <b style={{ color: "var(--teal)" }}>{fmtSpeed(kin.peakSpeedMS)}</b><br />
              heading <b style={{ color: "var(--ink)" }}>{Math.round(kin.headingDeg)}° {compass8(kin.headingDeg)}</b> · path <b style={{ color: "var(--ink)" }}>{fmtLenShort(kin.distM)}</b> over <b style={{ color: "var(--ink)" }}>{kin.durationS.toFixed(1)}s</b>
            </div>
          )}
          {track.length >= 2 && !kin && <div style={{ fontSize: 11, color: "var(--amber)", marginTop: 4 }}>Solve the pose/GCPs above so the track points geolocate.</div>}
          {track.length > 0 && <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 4 }}>Assumes the sensor pose is fixed across these frames (hovering platform / stationary ground points). A slewing/moving platform needs per-frame telemetry — on the roadmap.</div>}
        </div>
      )}

      {/* ── GCP list + placement map (gcp method) ── */}
      {method === "gcp" && (
        <div style={{ marginTop: 14 }}>
          <ML>Ground control points — tap the frame (🗺 mode), then pin or type each one's real spot</ML>
          {gcps.length === 0 && <div style={{ fontSize: 12, color: "var(--amber)", padding: "4px 2px" }}>Pick 🗺 Add ground point, then tap a recognisable feature in the frame (a road junction, a building corner). Repeat for ≥4.</div>}
          {gcps.map((g, i) => {
            const placed = isNum(g.lat) && isNum(g.lon), sel = i === gsel;
            return (
              <div key={i} onClick={() => setGsel(i)}
                style={{ display: "flex", alignItems: "center", gap: 6, padding: "5px 6px", marginTop: 4, borderRadius: 8, cursor: "pointer", border: `1px solid ${sel ? "var(--teal)" : "var(--line)"}`, background: sel ? "rgba(60,200,180,.08)" : "transparent" }}>
                <span style={{ fontFamily: "var(--mono)", fontSize: 12, color: placed ? "#7ee0a0" : "#ffb24a", fontWeight: 700, width: 16 }}>{i + 1}</span>
                <input value={isNum(g.lat) ? g.lat : ""} placeholder="lat" inputMode="decimal" onClick={(e) => e.stopPropagation()}
                  onChange={(e) => setGcp(i, { lat: e.target.value === "" ? null : +e.target.value })} style={{ flex: 1, minWidth: 0, fontSize: 12, padding: "4px 6px" }} />
                <input value={isNum(g.lon) ? g.lon : ""} placeholder="lon" inputMode="decimal" onClick={(e) => e.stopPropagation()}
                  onChange={(e) => setGcp(i, { lon: e.target.value === "" ? null : +e.target.value })} style={{ flex: 1, minWidth: 0, fontSize: 12, padding: "4px 6px" }} />
                <button className="btn sm ghost" style={{ color: "var(--red)", padding: "3px 7px" }} onClick={(e) => { e.stopPropagation(); delGcp(i); }}>✕</button>
              </div>
            );
          })}
          {gcps.length > 0 && (
            <>
              <div style={{ fontSize: 10.5, color: "var(--dim)", margin: "6px 2px 4px" }}>Drag the map so the crosshair sits on point {gsel + 1}'s real location — it fills that point's lat/lon.</div>
              <PinMap lat={isNum(gcps[gsel]?.lat) ? +gcps[gsel].lat : null} lon={isNum(gcps[gsel]?.lon) ? +gcps[gsel].lon : null}
                onChange={(la, lo) => setGcp(gsel, { lat: +la.toFixed(6), lon: +lo.toFixed(6) })} />
            </>
          )}
          <div style={{ fontSize: 11, fontFamily: "var(--mono)", marginTop: 6, color: gcpsReady.length >= 4 ? "var(--teal)" : "var(--amber)" }}>
            {gcpsReady.length} / {Math.max(4, gcps.length)} placed{gcpsReady.length < 4 ? ` · need ${4 - gcpsReady.length} more` : geoH ? ` · fit rms ${fmtLenShort(geoH.rms)}` : ""}
          </div>
        </div>
      )}

      {/* ── platform position (pin) + sensor pose (telemetry method) ── */}
      {method === "telemetry" && (<>
      <ML style={{ marginTop: 14 }}>Platform position — where the sensor is</ML>
      <PinMap lat={platLat} lon={platLon} onChange={(la, lo) => setPlat({ lat: +la.toFixed(6), lon: +lo.toFixed(6) })} bearing={az} />
      <div style={{ display: "flex", gap: 6, marginTop: 6 }}>
        <input value={isNum(platLat) ? platLat : ""} placeholder="lat" inputMode="decimal"
          onChange={(e) => setPlat({ lat: e.target.value === "" ? "" : +e.target.value })}
          style={{ flex: 1, minWidth: 0 }} />
        <input value={isNum(platLon) ? platLon : ""} placeholder="lon" inputMode="decimal"
          onChange={(e) => setPlat({ lon: e.target.value === "" ? "" : +e.target.value })}
          style={{ flex: 1, minWidth: 0 }} />
      </div>

      {/* platform altitude */}
      <div style={{ display: "flex", gap: 10, marginTop: 12 }}>
        <div style={{ flex: 1 }}>
          <ML>Platform altitude (MSL)</ML>
          <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
            <input value={+toDisp(platAlt).toFixed(0)} inputMode="numeric"
              onChange={(e) => setPlat({ alt: +fromDisp(+e.target.value).toFixed(1) })} style={{ width: 90 }} />
            <span style={{ color: "var(--dim)", fontSize: 12 }}>{altUnit}</span>
          </div>
        </div>
        <div style={{ flex: 1 }}>
          <ML>Ground elevation (MSL)</ML>
          <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
            <input value={+toDisp(groundAlt).toFixed(0)} inputMode="numeric"
              onChange={(e) => update({ groundAlt: +fromDisp(+e.target.value).toFixed(1) })} style={{ width: 78 }} />
            <span style={{ color: "var(--dim)", fontSize: 12 }}>{altUnit}</span>
            <button className="btn sm ghost" disabled={demBusy || !isNum(platLat)} onClick={grabGroundDem}>{demBusy ? "…" : "⛰ terrain"}</button>
          </div>
        </div>
      </div>
      <div style={{ fontSize: 10.5, color: "var(--teal)", fontFamily: "var(--mono)", marginTop: 3 }}>
        height above ground: {isNum(platLat) ? fmtLenShort(platAlt - groundAlt) : "—"}
      </div>

      {/* target height above ground — 0 for a ground target, >0 for an AIRBORNE
          object (drone / orb) so its ray stops at its own altitude, not the ground */}
      <div style={{ marginTop: 12 }}>
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
          <ML style={{ marginBottom: 1 }}>Target height above ground</ML>
          <span style={{ color: airborne ? "var(--amber)" : "var(--teal)", fontFamily: "var(--mono)", fontSize: 11 }}>{airborne ? fmtLenShort(objH) : "on the ground"}</span>
        </div>
        <div style={{ display: "flex", alignItems: "center", gap: 6 }}>
          <input value={+toDisp(objH).toFixed(0)} inputMode="numeric"
            onChange={(e) => update({ objH: Math.max(0, +fromDisp(+e.target.value)) })} style={{ width: 90 }} />
          <span style={{ color: "var(--dim)", fontSize: 12 }}>{altUnit} {objH > 0 ? `· ${fmtLenShort(platAlt - objAlt)} below platform` : ""}</span>
        </div>
        <div style={{ fontSize: 10, color: airborne ? "var(--amber)" : "var(--dim)", marginTop: 2, lineHeight: 1.5 }}>
          {airborne
            ? <>Airborne target: with one platform its height is an assumption — the position below <b>scales with it</b> (like distance for a single sky witness). Sweep it, or fix it with a second observer.</>
            : <>0 for something on the ground (vehicle, structure). Raise it for an airborne object (drone, orb) so its sight-line stops at its own altitude, not the ground beneath it.</>}
        </div>
      </div>

      {/* sensor look azimuth */}
      <div style={{ marginTop: 12 }}>
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
          <ML style={{ marginBottom: 1 }}>Look azimuth (compass bearing)</ML>
          <span style={{ color: "var(--teal)", fontFamily: "var(--mono)", fontSize: 11 }}>{Math.round(((az % 360) + 360) % 360)}° {compass8(az)}</span>
        </div>
        <input type="range" min={0} max={359} step={1} value={((az % 360) + 360) % 360} onChange={(e) => setSensor({ az: +e.target.value })} />
      </div>

      {/* sensor depression */}
      <div style={{ marginTop: 6 }}>
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
          <ML style={{ marginBottom: 1 }}>Depression below horizontal</ML>
          <span style={{ color: "var(--teal)", fontFamily: "var(--mono)", fontSize: 11 }}>{Math.round(-el)}° {(-el) >= 80 ? "≈ straight down" : (-el) <= 10 ? "shallow / oblique" : "down"}</span>
        </div>
        <input type="range" min={1} max={90} step={1} value={-el} onChange={(e) => setSensor({ el: -(+e.target.value) })} />
      </div>

      {/* FOV */}
      <div style={{ marginTop: 6 }}>
        <div style={{ display: "flex", justifyContent: "space-between", alignItems: "baseline" }}>
          <ML style={{ marginBottom: 1 }}>Field of view</ML>
          <span style={{ color: isNum(src.meta?.fovH) ? "var(--teal)" : "var(--amber)", fontFamily: "var(--mono)", fontSize: 11 }}>{Math.round(fov)}°{isNum(src.meta?.fovH) ? " · from lens ✓" : " · estimate"}</span>
        </div>
        {isNum(src.meta?.fovH)
          ? <div style={{ fontSize: 10, color: "var(--dim)", marginTop: 1 }}>Sensor FOV comes from the frame's lens metadata.</div>
          : <input type="range" min={2} max={130} step={0.5} value={fov} onChange={(e) => update({ fovH: +(+e.target.value).toFixed(1) })} />}
      </div>
      </>)}

      {/* ── results ── */}
      <div className="card" style={{ marginTop: 14 }}>
        <ML>Geolocation</ML>
        {!solveOk ? (
          <div style={{ fontSize: 12, color: "var(--amber)", lineHeight: 1.5 }}>
            {method === "gcp"
              ? (!(natW > 0) ? "Load the frame on the previous step." : `Place ${Math.max(0, 4 - gcpsReady.length)} more ground control point${4 - gcpsReady.length === 1 ? "" : "s"} (≥4 needed to fit).`)
              : (!(isNum(platLat) && isNum(platLon)) ? "Pin the platform position to solve."
                : !(platAlt > groundAlt) ? "Platform altitude must be above the ground elevation."
                  : !(natW > 0) ? "Load the frame on the previous step."
                    : "Set a downward depression angle.")}
          </div>
        ) : (
          <>
            <div style={{ fontFamily: "var(--mono)", fontSize: 12, lineHeight: 1.7 }}>
              {tGround ? (
                <>
                  <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                    <span style={{ color: "var(--teal)" }}>{airborne && method === "telemetry" ? "object" : "target"}&nbsp;</span>
                    <b>{tGround.lat.toFixed(6)}, {tGround.lon.toFixed(6)}</b>
                    <button className="btn sm ghost" style={{ padding: "2px 7px", fontSize: 10 }}
                      onClick={() => { try { navigator.clipboard.writeText(`${tGround.lat.toFixed(6)}, ${tGround.lon.toFixed(6)}`); setCopied(true); setTimeout(() => setCopied(false), 1200); } catch (e) { } }}>{copied ? "✓" : "copy"}</button>
                  </div>
                  {method === "telemetry"
                    ? <div style={{ color: "var(--dim)" }}>slant range <b style={{ color: "var(--ink)" }}>{fmtLenShort(tGround.slant)}</b> · ground dist <b style={{ color: "var(--ink)" }}>{fmtLenShort(tRange)}</b> · bearing <b style={{ color: "var(--ink)" }}>{Math.round(tBearing)}° {compass8(tBearing)}</b>{airborne ? <> · at <b style={{ color: "var(--amber)" }}>{fmtLenShort(objH)}</b> AGL</> : ""}</div>
                    : <div style={{ color: "var(--dim)" }}>from {gcpsReady.length} ground points · fit rms <b style={{ color: "var(--ink)" }}>{fmtLenShort(geoH.rms)}</b></div>}
                </>
              ) : <div style={{ color: "var(--amber)" }}>Tap the target — {method === "gcp" ? "it geolocates through the ground homography." : "its sight-line clears the ground plane at this pose."}</div>}
              {method === "telemetry" && <div style={{ color: "var(--dim)", marginTop: 2 }}>resolution ≈ <b style={{ color: "var(--ink)" }}>{gsd ? (unitsImp ? (gsd / M_PER_FT).toFixed(2) + " ft/px" : gsd.toFixed(2) + " m/px") : "—"}</b> at frame centre</div>}
              {span.length === 2 && <div style={{ color: "var(--amber)", marginTop: 2 }}>bracketed size <b style={{ color: "var(--ink)" }}>{spanM != null ? fmtLenShort(spanM) : "—"}</b>{airborne && method === "telemetry" ? " (at assumed height)" : ""}</div>}
              {span.length === 1 && <div style={{ color: "var(--dim)", marginTop: 2 }}>size: tap the second edge…</div>}
              {/* airborne honesty: a ground homography can't leave its plane */}
              {method === "gcp" && tGround && <div style={{ color: "var(--amber)", marginTop: 5, fontSize: 11, lineHeight: 1.5 }}>⚠ This is the point on the GROUND under the target's line of sight. For an <b>airborne</b> object (drone, orb) that's its shadow, not its position — a ground homography can't leave the ground plane. Its true position needs the platform's pose (📡 tab) or a second observer.</div>}
            </div>
            {(() => {
              const spanGeo = span.length === 2 && solveOk ? span.map((p) => locateObj(p.x, p.y)) : null;
              return (<>
                <AerialGroundMap footprint={footprint} target={tGround} span={spanGeo} track={trackGeo} platform={method === "telemetry" ? { lat: platLat, lon: platLon } : null} />
                <AerialFootprint footprint={footprint} target={tGround} span={spanGeo} showNadir={method === "telemetry"} />
              </>);
            })()}
          </>
        )}
      </div>
    </div>
  );
}

/* compact top-down schematic: the sensor footprint quad, the geolocated target,
   and (telemetry only) the platform nadir — a quick sanity read of the geometry
   before the full satellite ground view. Everything arrives already in one ENU
   (e,n) metre frame from `locate`, so there's no per-method conversion here.
   Pure canvas, auto-scaled to the points' bounds. */
function AerialFootprint({ footprint, target, span, showNadir }) {
  const ref = useRef(null);
  useEffect(() => {
    const cv = ref.current; if (!cv) return;
    const dpr = window.devicePixelRatio || 1, W = cv.clientWidth || 300, H = 180;
    cv.width = W * dpr; cv.height = H * dpr;
    const ctx = cv.getContext("2d"); ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, W, H);
    const pts = [];
    if (showNadir) pts.push([0, 0]);
    const foot = (footprint || []).filter(Boolean);
    foot.forEach((g) => pts.push([g.e, g.n]));
    const tEN = target && isNum(target.e) ? [target.e, target.n] : null;
    if (tEN) pts.push(tEN);
    const sEN = (span && span[0] && span[1] && isNum(span[0].e)) ? span.map((g) => [g.e, g.n]) : null;
    if (sEN) sEN.forEach((p) => pts.push(p));
    if (pts.length < 2) return;
    let minX = Math.min(...pts.map((p) => p[0])), maxX = Math.max(...pts.map((p) => p[0]));
    let minY = Math.min(...pts.map((p) => p[1])), maxY = Math.max(...pts.map((p) => p[1]));
    const pad = 18, spanX = Math.max(1, maxX - minX), spanY = Math.max(1, maxY - minY);
    const sc = Math.min((W - 2 * pad) / spanX, (H - 2 * pad) / spanY);
    const X = (e) => pad + (e - minX) * sc, Y = (n) => H - pad - (n - minY) * sc;  // north up
    // footprint quad
    if (foot.length >= 3) {
      ctx.beginPath();
      foot.forEach((g, i) => { const x = X(g.e), y = Y(g.n); i ? ctx.lineTo(x, y) : ctx.moveTo(x, y); });
      ctx.closePath();
      ctx.fillStyle = "rgba(60,200,180,.10)"; ctx.fill();
      ctx.strokeStyle = "rgba(60,200,180,.55)"; ctx.lineWidth = 1.4; ctx.stroke();
    }
    // platform nadir (telemetry only — GCP mode has no platform)
    ctx.font = "10px monospace";
    if (showNadir) {
      ctx.fillStyle = "#dfe8ff";
      ctx.beginPath(); ctx.arc(X(0), Y(0), 4, 0, 7); ctx.fill();
      ctx.fillStyle = "#8ea3bf"; ctx.fillText("nadir", X(0) + 6, Y(0) - 4);
    }
    // size bracket
    if (sEN) {
      ctx.strokeStyle = "#ffb24a"; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(X(sEN[0][0]), Y(sEN[0][1])); ctx.lineTo(X(sEN[1][0]), Y(sEN[1][1])); ctx.stroke();
    }
    // target
    if (tEN) {
      ctx.fillStyle = "#3ce0c0";
      ctx.beginPath(); ctx.arc(X(tEN[0]), Y(tEN[1]), 5, 0, 7); ctx.fill();
      ctx.strokeStyle = "#0a0f1c"; ctx.lineWidth = 1.5; ctx.stroke();
    }
    // north arrow
    ctx.strokeStyle = "#5a6b82"; ctx.fillStyle = "#5a6b82"; ctx.lineWidth = 1.2;
    ctx.beginPath(); ctx.moveTo(W - 14, 30); ctx.lineTo(W - 14, 12); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(W - 14, 10); ctx.lineTo(W - 17, 16); ctx.lineTo(W - 11, 16); ctx.closePath(); ctx.fill();
    ctx.fillText("N", W - 18, 40);
  });
  return <canvas ref={ref} style={{ width: "100%", height: 180, marginTop: 10, display: "block" }} />;
}

/* the aerial ground view: the geolocated target, the sensor footprint quad and
   any size bracket drawn on real satellite imagery (Esri World Imagery, same
   base as PlotBoard/PinMap). Everything is passed as {lat,lon} — the geo the
   two geolocation methods already produce — so this is method-agnostic. */
function AerialGroundMap({ footprint, target, span, track, platform }) {
  const boxRef = useRef(null), mapRef = useRef(null), layerRef = useRef(null);
  const foot = (footprint || []).filter((g) => g && isNum(g.lat));
  const trk = (track || []).filter((g) => g && isNum(g.lat));
  const key = JSON.stringify({ f: foot.map((g) => [g.lat, g.lon]), t: target && [target.lat, target.lon], s: (span || []).map((g) => g && [g.lat, g.lon]), k: trk.map((g) => [g.lat, g.lon]), p: platform && [platform.lat, platform.lon] });
  useEffect(() => {
    const el = boxRef.current; if (!el || mapRef.current) return;
    const map = L.map(el, { attributionControl: true, zoomControl: false });
    map.attributionControl.setPrefix(false);
    L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
      maxZoom: 21, maxNativeZoom: 19, attribution: "© Esri, Maxar, Earthstar Geographics",
    }).addTo(map);
    L.control.scale({ imperial: true }).addTo(map);
    mapRef.current = map;
    return () => { map.remove(); mapRef.current = null; };
  }, []);
  useEffect(() => {
    const map = mapRef.current; if (!map) return;
    if (layerRef.current) { map.removeLayer(layerRef.current); layerRef.current = null; }
    const g = L.layerGroup();
    const bounds = [];
    if (foot.length >= 3) {
      const ring = foot.map((p) => [p.lat, p.lon]);
      L.polygon(ring, { color: "#3ce0c0", weight: 1.4, opacity: 0.75, fillColor: "#3ce0c0", fillOpacity: 0.08, interactive: false }).addTo(g);
      ring.forEach((p) => bounds.push(p));
    }
    if (platform && isNum(platform.lat)) {
      const pn = [platform.lat, platform.lon];
      L.marker(pn, { interactive: false, icon: L.divIcon({ className: "", iconSize: [0, 0], html: `<div class="lmk lmk-tri">◇<span>nadir</span></div>` }) }).addTo(g);
      bounds.push(pn);
    }
    if (span && span[0] && span[1] && isNum(span[0].lat)) {
      const sl = [[span[0].lat, span[0].lon], [span[1].lat, span[1].lon]];
      L.polyline(sl, { color: "#ffb24a", weight: 3, opacity: 0.95, interactive: false }).addTo(g);
      sl.forEach((p) => bounds.push(p));
    }
    if (trk.length >= 2) {
      const tl = trk.map((p) => [p.lat, p.lon]);
      L.polyline(tl, { color: "#8FB4FF", weight: 2.5, opacity: 0.95, interactive: false }).addTo(g);
      tl.forEach((p, i) => { bounds.push(p); L.circleMarker(p, { radius: 3, color: "#8FB4FF", fillColor: "#8FB4FF", fillOpacity: 1, weight: 1, interactive: false }).addTo(g); });
    }
    if (target && isNum(target.lat)) {
      const tn = [target.lat, target.lon];
      L.marker(tn, { interactive: false, icon: L.divIcon({ className: "", iconSize: [0, 0], html: `<div class="lmk lmk-fix">⊕<span>target</span></div>` }) }).addTo(g);
      bounds.push(tn);
    }
    g.addTo(map); layerRef.current = g;
    if (bounds.length) { try { map.fitBounds(L.latLngBounds(bounds).pad(0.35), { maxZoom: 18, animate: false }); } catch (e) { } }
  }, [key]);
  if (!foot.length && !(target && isNum(target.lat)) && trk.length < 2) return null;
  return <div className="plotwrap" style={{ marginTop: 10 }}><div ref={boxRef} style={{ position: "absolute", inset: 0 }} /><div className="map-north">N ↑</div></div>;
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
        {busy ? <><Spin style={{ marginRight: 6 }} />Querying traffic</> : ageMin > 15 ? "🛰 Check archived traffic at the sighting time" : "🛰 Check live aircraft now"}
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
                    {c.flight || c.reg || c.hex}{c.t ? ` · ${c.t}` : ""}{c.span != null ? ` · ${fmtLenShort(c.span)} span` : ""}
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
                  {c.altM != null ? `FL ${(c.altM / FT_M / 100).toFixed(0)} · ` : ""}{c.gs != null ? fmtSpeedShort(c.gs) : ""}{c.track != null ? ` · trk ${c.track.toFixed(0)}°` : ""}
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
            <ML>Object altitude (above observers)</ML>
            <div className="readout">{fmtLenShort(r.solA.X[2])}</div>
            <div className="readsub">{fmtLenAlt(r.solA.X[2])}</div>
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
          <div className="readsub">{fmtLenAlt(r.sizeAvg)} across (longest marked dimension)</div>
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
              <div className="readsub">Δalt {fmtLenShort(r.motion.XB[2] - r.solA.X[2])}</div>
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
/* --- shared screen wake lock: hold while ANY long job runs ----------------
   Refcounted so overlapping jobs (a stabilize + a report build) compose;
   iOS silently releases the lock when the page hides, so it re-arms on
   every return to visible while jobs are outstanding. No-op without the
   API. Callers: wakeHold() … try { work } finally { wakeRelease() }. */
let _wakeJobs = 0, _wakeLock = null, _wakeVisArmed = false;
async function _wakeGrab() {
  try {
    if (_wakeJobs > 0 && !_wakeLock && typeof navigator !== "undefined" && navigator.wakeLock) {
      const l = await navigator.wakeLock.request("screen");
      if (_wakeJobs > 0) { _wakeLock = l; l.addEventListener?.("release", () => { if (_wakeLock === l) _wakeLock = null; }); }
      else { try { l.release(); } catch (e) { } }
    }
  } catch (e) { }
}
function wakeHold() {
  _wakeJobs++;
  if (!_wakeVisArmed && typeof document !== "undefined") {
    _wakeVisArmed = true;
    document.addEventListener("visibilitychange", () => { if (document.visibilityState === "visible") _wakeGrab(); });
  }
  _wakeGrab();
}
function wakeRelease() {
  _wakeJobs = Math.max(0, _wakeJobs - 1);
  if (!_wakeJobs && _wakeLock) { try { _wakeLock.release(); } catch (e) { } _wakeLock = null; }
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

/* Pack ONE moment: strip the live media handles, keep the measurements, and
   bundle a ≤1000px JPEG thumbnail (+ rescaled marks) as report evidence. Marks
   rescale by the same k so an object overlay drawn against natW/natH lines up;
   the trajectory math never touches these (it uses az/el + pixel-ratio-invariant
   angular size). Returns a lean plain object safe to embed/share. */
async function packMoment(m) {
  const { mediaUrl, mediaKind, mediaNorm, track, ...r } = m;
  if (mediaUrl && mediaKind === "image" && r.natW) {
    try {
      const im = await new Promise((res, rej) => { const i = new Image(); i.onload = () => res(i); i.onerror = rej; i.src = mediaUrl; });
      const k = Math.min(1000, r.natW) / r.natW;
      const cv = document.createElement("canvas");
      cv.width = Math.round(r.natW * k); cv.height = Math.round(r.natH * k);
      cv.getContext("2d").drawImage(im, 0, 0, cv.width, cv.height);
      const sp = (p) => (p ? { ...p, x: p.x * k, y: p.y * k } : p);
      r.natW = cv.width; r.natH = cv.height;
      r.A = { ...r.A, p1: sp(r.A?.p1), p2: sp(r.A?.p2) };
      if (r.B?.pb) r.B = { ...r.B, pb: sp(r.B.pb) };
      if (r.shapeFit) r.shapeFit = { ...r.shapeFit, cx: r.shapeFit.cx * k, cy: r.shapeFit.cy * k, sizeNat: r.shapeFit.sizeNat * k };
      r.mediaJpeg = cv.toDataURL("image/jpeg", 0.72);
    } catch (e) { /* embed the moment without its thumbnail */ }
  }
  return r;
}

/* Bundle each observer with a 1600px copy of their photo, rescaling ALL
   pixel-space data to match so the export is self-consistent (angles are
   pixel-ratio invariant). Analysis always runs on the full-res originals. */
async function packSources(sources) {
  const act = sources.filter((s) => !isEmptySource(s));
  const out = [];
  for (const s of act) {
    /* preStab is the undo snapshot for the last stabilize run — private working
       state, and a full duplicate of both paths. It has no meaning to a
       recipient and would roughly double the video payload of every share. */
    const { mediaUrl, mediaKind, mediaNorm, open, preStab, ...r } = s;
    /* moments carry their own (heavy) photos — keep the measurements (whenMs +
       A.az/el + marks + natW/H/fovH) so an imported sighting still reconstructs
       the multi-photo trajectory via sourceTrack, and bundle a modest thumbnail
       (≤1000 px) as the report's trajectory evidence. Marks rescale with the
       thumbnail so the object overlay lines up; angular size is pixel-ratio
       invariant so the trajectory math is unaffected either way. */
    if (Array.isArray(r.moments) && r.moments.length) {
      r.moments = await Promise.all(r.moments.map((m) => packMoment(m)));
    }
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
            r.detailJpeg = dc.toDataURL("image/jpeg", 0.92);
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
  if (bm) g += tx(W - 6, H - 6, "© Esri, Maxar, Earthstar Geographics", { size: 9 }).replace("<text ", '<text text-anchor="end" opacity=".9" ');
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
<text x="${L}" y="${T - 3}" font-size="10" fill="#0e7d6f">speed (peak ${fmtSpeedShort(k.peakSpeed)})</text>
${lod ? `<text x="${W - Rm}" y="${T - 3}" font-size="10" fill="#C77B14" text-anchor="end">felt load (peak ${k.peakLoad?.toFixed(1)} g, dashed)</text>` : ""}
<text x="${W / 2}" y="${H - 8}" font-size="10" fill="#888" text-anchor="middle">${(t1 - t0).toFixed(1)} s</text>
</svg>`;
}

/* wind-rose for the report's Wind check: a top-down compass with a drift arrow
   per altitude (length = wind speed, colour blue→red by speed), and the
   object's OWN motion drawn as a bold black arrow on the same scale — so a
   reader can SEE whether the object moved like the wind at any layer (a balloon
   would line up with one). Self-contained SVG; the level nearest the object's
   altitude is bolded. */
function reportWindSvg(prof, objSpeed, objHeading, nearestM) {
  if (!prof || !prof.levels || !prof.levels.length) return "";
  const W = 340, H = 340, cx = W / 2, cy = H / 2 + 8, R = 120;
  const levels = prof.levels;
  const vmax = Math.max(objSpeed || 0, ...levels.map((L) => L.speedMs), 1);
  const speedColor = (v) => `hsl(${(210 - 210 * Math.min(1, v / vmax)).toFixed(0)},80%,45%)`;
  const vec = (brg, len) => [cx + len * Math.sin(brg * D2R), cy - len * Math.cos(brg * D2R)];
  const arrow = (brg, len, col, w, op) => {
    const [x2, y2] = vec(brg, len), a = brg * D2R, dx = Math.sin(a), dy = -Math.cos(a), px = -dy, py = dx, ah = 7;
    const bx = x2 - dx * ah, by = y2 - dy * ah;
    return `<line x1="${cx}" y1="${cy}" x2="${x2.toFixed(1)}" y2="${y2.toFixed(1)}" stroke="${col}" stroke-width="${w}" opacity="${op}"/>` +
      `<path d="M${x2.toFixed(1)},${y2.toFixed(1)} L${(bx + px * 3.6).toFixed(1)},${(by + py * 3.6).toFixed(1)} L${(bx - px * 3.6).toFixed(1)},${(by - py * 3.6).toFixed(1)} z" fill="${col}" opacity="${op}"/>`;
  };
  let g = `<circle cx="${cx}" cy="${cy}" r="${R}" fill="#fff" stroke="#ddd"/><circle cx="${cx}" cy="${cy}" r="${(R * 0.5).toFixed(1)}" fill="none" stroke="#eee"/>`;
  [["N", 0], ["E", 90], ["S", 180], ["W", 270]].forEach(([lbl, b]) => { const p = vec(b, R + 12); g += `<text x="${p[0].toFixed(1)}" y="${(p[1] + 4).toFixed(1)}" font-size="11" font-weight="700" fill="#888" text-anchor="middle">${lbl}</text>`; });
  levels.forEach((L) => {
    const near = nearestM != null && L.levelM === nearestM;
    const len = R * 0.92 * Math.min(1, L.speedMs / vmax);
    g += arrow(L.driftDeg, len, speedColor(L.speedMs), near ? 3.2 : 1.5, near ? 1 : 0.7);
    const lp = vec(L.driftDeg, len + 12);
    g += `<text x="${lp[0].toFixed(1)}" y="${(lp[1] + 3).toFixed(1)}" font-size="8.5" fill="${near ? "#000" : "#aaa"}" font-weight="${near ? 700 : 400}" text-anchor="middle">${fmtLenShort(L.levelM)}</text>`;
  });
  if (objSpeed != null && objSpeed > 0 && objHeading != null) {
    const len = R * 0.92 * Math.min(1, objSpeed / vmax);
    g += arrow(objHeading, len, "#111", 3.6, 1);
    const lp = vec(objHeading, len * 0.55);
    g += `<text x="${(lp[0] + 6).toFixed(1)}" y="${lp[1].toFixed(1)}" font-size="10" font-weight="700" fill="#111">object</text>`;
  }
  g += `<circle cx="${cx}" cy="${cy}" r="2.5" fill="#333"/>`;
  return `<svg viewBox="0 0 ${W} ${H}" style="max-width:340px;width:100%;border:1px solid #ddd;border-radius:6px;background:#fafafa"><text x="10" y="18" font-size="12" font-weight="700" fill="#333">Winds aloft vs object motion</text>${g}</svg>` +
    `<p class="cap">Each coloured arrow is the wind's drift direction at one altitude (length = speed, blue→red = slow→fast); the bold black arrow is the object's own motion. A balloon rides the wind, so its motion would line up with the layer at its height (bold label).</p>`;
}

/* Overlay SVG (in a placed photo's own natW×natH pixel space) drawing an arrow
   at the marked object showing which way the wind at the OBJECT'S altitude would
   carry it across THIS photo — projected through the photo's pose. Returns "" if
   the photo isn't placed / has no object direction. windObj = {driftDeg, levelM}. */
function windArrowOverlay(s, windObj) {
  if (!windObj || !isNum(s.fovH) || !s.natW || !s.natH || !isNum(s.A?.az) || !isNum(s.A?.el)) return "";
  const ma = s.mediaAim || {};
  const caz = isNum(ma.az) ? +ma.az : +s.A.az, cel = isNum(ma.el) ? +ma.el : +s.A.el;
  const roll = isNum(ma.roll) ? +ma.roll : 0, k = isNum(ma.dist) ? +ma.dist : 0;
  const d0 = dirFromAzEl(+s.A.az, +s.A.el);
  const v = [Math.sin(windObj.driftDeg * D2R), Math.cos(windObj.driftDeg * D2R), 0]; // drift horizontal (ENU)
  const d1 = unit([d0[0] + 0.06 * v[0], d0[1] + 0.06 * v[1], d0[2] + 0.06 * v[2]]);  // small drift step
  const p0 = dirToPixK(d0, s.natW, s.natH, caz, cel, roll, +s.fovH, k);
  const p1 = dirToPixK(d1, s.natW, s.natH, caz, cel, roll, +s.fovH, k);
  if (!p0 || !p1) return "";
  const dx = p1.px - p0.px, dy = p1.py - p0.py, L = Math.hypot(dx, dy) || 1, ux = dx / L, uy = dy / L;
  const len = s.natW * 0.16, sw = Math.max(2, s.natW / 360), ah = len * 0.26;
  const ex = p0.px + ux * len, ey = p0.py + uy * len, bx = ex - ux * ah, by = ey - uy * ah, pxp = -uy, pyp = ux;
  const col = "#38bdf8";
  const head = `<path d="M${ex.toFixed(1)},${ey.toFixed(1)} L${(bx + pxp * ah * 0.5).toFixed(1)},${(by + pyp * ah * 0.5).toFixed(1)} L${(bx - pxp * ah * 0.5).toFixed(1)},${(by - pyp * ah * 0.5).toFixed(1)} z" fill="${col}"/>`;
  const fs = Math.max(12, s.natW / 62);
  return `<svg viewBox="0 0 ${s.natW} ${s.natH}" style="position:absolute;left:0;top:0;width:100%;height:100%">` +
    `<line x1="${p0.px.toFixed(1)}" y1="${p0.py.toFixed(1)}" x2="${ex.toFixed(1)}" y2="${ey.toFixed(1)}" stroke="${col}" stroke-width="${sw.toFixed(1)}" opacity="0.92"/>${head}` +
    `<text x="${(p0.px + ux * len * 0.5 + pxp * fs * 1.3).toFixed(1)}" y="${(p0.py + uy * len * 0.5 + pyp * fs * 1.3).toFixed(1)}" font-size="${fs.toFixed(0)}" fill="${col}" stroke="#000" stroke-width="${Math.max(1, s.natW / 1200).toFixed(1)}" paint-order="stroke" font-weight="700" text-anchor="middle">wind @ ${fmtLenShort(windObj.levelM)}</text></svg>`;
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
  prop: [["Top", "x", "y", "length", "wingspan"], ["Side", "x", "z", "length", "height"], ["Front", "y", "z", "wingspan", "height"]],
  heli: [["Top", "x", "y", "length", "rotor span"], ["Side", "x", "z", "length", "height"], ["Front", "y", "z", "rotor span", "height"]],
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
  /* weather + cloud base at the sighting — one fetch, shared by the single-
     witness size chart (cloud-base range cap) and the conditions section.
     Silently omitted if the proxy / Open-Meteo is unreachable. */
  let wx = null;
  {
    const wref = origAct.find((s) => isNum(s.lat) && isNum(s.lon) && isNum(s.whenMs));
    const wla = wref ? +wref.lat : (fix.ok ? fix.ref.lat : null);
    const wlo = wref ? +wref.lon : (fix.ok ? fix.ref.lon : null);
    const wms = wref ? +wref.whenMs : null;
    if (wla != null && wlo != null && wms != null) {
      try { wx = await fetchWeatherAt(wla, wlo, wms); } catch (e) { wx = null; }
    }
  }
  /* Is there an actual LOW cloud DECK? The estimated base (Espy LCL, from
     surface temp/dew) is a LOW-cloud base, so the range/size cap is only
     meaningful when there's real low/total cover — not on a clear night or
     under high cirrus only. */
  const wxDeck = wx && wx.baseAGL != null && ((wx.low != null ? wx.low : wx.cloud) || 0) >= 40;
  /* winds-aloft profile, fetched ONCE here (fix only) — reused by the photo-
     exhibit wind arrows AND the Wind check section. windObj = the layer at the
     object's altitude (what a balloon at that height would ride). */
  let windProfR = null, windObj = null;
  if (fix.ok) {
    try {
      const wwhen = +(sources.find((s) => isNum(s.whenMs))?.whenMs || Date.now());
      windProfR = await fetchWindProfile(fix.ref.lat, fix.ref.lon, wwhen);
      const altMSL = fix.solA.X[2] + (fix.ref.alt || 0);
      const nrst = windProfR.levels.reduce((b, L) => Math.abs(L.levelM - altMSL) < Math.abs(b.levelM - altMSL) ? L : b, windProfR.levels[0]);
      windObj = { driftDeg: nrst.driftDeg, speedMs: nrst.speedMs, levelM: nrst.levelM };
    } catch (e) { windProfR = null; }
  }
  const e2 = (s) => String(s ?? "").replace(/&/g, "&amp;").replace(/</g, "&lt;");
  const dv = (s) => (s === "" || s == null ? "—" : e2(s));
  const ft = (m) => Math.round(m * 3.28084);
  /* length as "primary (complementary)" respecting the user's unit choice, so
     an imperial report reads "20 ft (6 m)" not the redundant "20 ft (20 ft)" */
  const lp = (m) => `${fmtLenShort(m)}${isImperialUnits() ? ` (${n1(m)} m)` : ` (${ft(m)} ft)`}`;
  /* speed in the user's primary system with the other as a cross-check */
  const sp = (ms) => isImperialUnits() ? `${Math.round(ms * 2.23694)} mph (${Math.round(ms * 3.6)} km/h)` : `${Math.round(ms)} m/s (${Math.round(ms * 2.23694)} mph)`;
  const row = (k, v) => `<tr><td>${k}</td><td>${v ?? "—"}</td></tr>`;
  const obsRows = packed.map((s, i) =>
    `<tr><td>${e2(s.name || "Observer " + (i + 1))}</td><td>${dv(s.lat)}, ${dv(s.lon)} · ${fmtLenShort(+(s.alt) || 0)}</td><td>${s.whenMs ? new Date(+s.whenMs).toLocaleString() : "—"}</td><td>${dv(s.A?.az)}° / ${dv(s.A?.el)}°</td><td>${isNum(s.fovH) ? (+s.fovH).toFixed(1) + "°" : "—"}</td><td>${(s.track || []).length}</td></tr>`
  ).join("");

  /* --- is an object (az,el) inside a photo's frame? Projects it through the
     photo's own pose (mediaAim center + FOV + lens distortion) and checks the
     pixel lands in-bounds. Used to only report cross-check features that are
     actually IN the picture, not everything above the horizon somewhere. --- */
  const objInFrame = (s, oaz, oel) => {
    if (!s || !isNum(s.fovH) || !s.natW || !s.natH) return false;
    const ma = s.mediaAim || {};
    const caz = isNum(ma.az) ? +ma.az : (isNum(s.A?.az) ? +s.A.az : null);
    const cel = isNum(ma.el) ? +ma.el : (isNum(s.A?.el) ? +s.A.el : null);
    if (caz == null || cel == null) return false;
    const p = dirToPixK(dirFromAzEl(oaz, oel), s.natW, s.natH, caz, cel, isNum(ma.roll) ? +ma.roll : 0, +s.fovH, isNum(ma.dist) ? +ma.dist : 0);
    if (!p) return false;
    const mx = s.natW * 0.03, my = s.natH * 0.03; // small margin past the edge
    return p.px > -mx && p.px < s.natW + mx && p.py > -my && p.py < s.natH + my;
  };
  const inAnyFrame = (oaz, oel) => origAct.some((s) => objInFrame(s, oaz, oel));
  const brg = (la1, lo1, la2, lo2) => { const p1 = la1 * D2R, p2 = la2 * D2R, dL = (lo2 - lo1) * D2R; return ((Math.atan2(Math.sin(dL) * Math.cos(p2), Math.cos(p1) * Math.sin(p2) - Math.sin(p1) * Math.cos(p2) * Math.cos(dL)) * R2D) + 360) % 360; };

  /* --- how each photo was aligned onto the sky — the references that turn
     pixels into true bearings (persisted as source.calib). Always reported. --- */
  const alignText = (s) => {
    const c = s.calib;
    if (!c || c.method === "manual") return { how: "placed by eye — dragged onto the sky/terrain (no star or terrain solve)", cls: "cap" };
    if (c.method === "stars") {
      const via = c.mode === "auto" ? "automatic plate-solve of the star field"
        : `tapped reference stars${c.refs && c.refs.length ? " (" + c.refs.map(e2).join(", ") + ")" : ""}`;
      const q = isNum(c.rms) ? ` — ${isNum(c.n) ? c.n + " stars, " : ""}${c.rms}° fit` : "";
      return { how: `aligned to the stars via ${via}${q}${c.loose ? " · loose fit, verify" : ""}`, cls: c.loose ? "cap" : "" };
    }
    if (c.method === "terrain") return { how: `aligned to the DEM terrain skyline (snap to ridges)${isNum(c.rms) ? ` — ${c.n} points, ${c.rms}° fit` : ""}`, cls: "" };
    return { how: "placed by eye", cls: "cap" };
  };
  const alignRows = packed.filter((s) => s.mediaAim || s.calib);
  const alignHtml = alignRows.length ? `<h2>Image alignment</h2>
<p class="cap">How each photo was oriented on the sky — the reference used to turn its pixels into true bearings. Everything downstream depends on this.</p>
<table><tr><th>Observer</th><th>Aligned via</th><th>FOV · lens</th></tr>${alignRows.map((s, i) => {
    const a = alignText(s); const k = s.mediaAim && isNum(s.mediaAim.dist) ? +s.mediaAim.dist : 0;
    return `<tr><td>${e2(s.name || "Observer " + (i + 1))}</td><td class="${a.cls}">${a.how}</td><td>${isNum(s.fovH) ? (+s.fovH).toFixed(1) + "°" : "—"}${Math.abs(k) > 1e-4 ? ` · lens k ${k >= 0 ? "+" : ""}${k.toFixed(3)}` : ""}</td></tr>`;
  }).join("")}</table>` : "";

  /* collapse a "<h2>Title</h2>rest" section into an expandable <details> */
  const collapsible = (html, open) => {
    if (!html) return "";
    const m = html.match(/^\s*<h2>([\s\S]*?)<\/h2>([\s\S]*)$/);
    return m ? `<details class="sec"${open ? " open" : ""}><summary>${m[1]}</summary>${m[2]}</details>` : html;
  };
  let fixHtml;
  if (fix.ok) {
    const mslA = fix.solA.X[2] + (fix.ref.alt || 0);
    const geomTbl = `<table><tr><th>Observer</th><th>Range</th><th>Angular size</th><th>→ True size</th></tr>` +
      fix.perSource.map((p) => `<tr><td>${e2(p.name || "—")}</td><td>${fmtLenShort(p.dist)}</td><td>${p.ang != null ? fmtDeg(p.ang) : "—"}</td><td>${p.size != null ? lp(p.size) : "—"}</td></tr>`).join("") +
      `</table>`;
    /* uncertainty ELLIPSE: re-solve with each witness's az/el nudged ±1° and
       fit a covariance ellipse to the resulting ground points — captures that
       the error is long along the near-parallel baseline direction, not a
       single scalar. */
    const ell = (() => {
      if (!fix.ok) return null;
      const wit = sources.filter((s) => isNum(s.A?.az) && isNum(s.A?.el));
      if (wit.length < 2) return null;
      const pts = [];
      for (const wsel of wit) for (const daz of [-1, 0, 1]) for (const del of [-1, 0, 1]) {
        if (daz === 0 && del === 0) continue;
        const mod = sources.map((s) => s === wsel ? { ...s, A: { ...s.A, az: +s.A.az + daz, el: +s.A.el + del } } : s);
        const f = analyze(mod);
        if (f.ok) pts.push([f.solA.X[0], f.solA.X[1]]);
      }
      return covEllipse(pts);
    })();
    const qualTbl = `<table>` +
      row("Baseline (observer separation)", fmtLenShort(fix.baseline)) +
      row("Ray convergence angle", fix.conv.toFixed(1) + "°") +
      row("Ray miss distance (RMS)", `${fmtLenShort(fix.solA.rmsMiss)} (${(fix.missRatio * 100).toFixed(1)}% of range)`) +
      row("Range / baseline ratio", `${(fix.meanDist / Math.max(1, fix.baseline)).toFixed(1)} : 1`) +
      (ell ? row("Position uncertainty (1σ ellipse)", `${fmtLenShort(ell.major)} × ${fmtLenShort(ell.minor)} — long axis bears ${Math.round(ell.bearing)}° / ${Math.round((ell.bearing + 180) % 360)}° <span class="cap">(from ±1° pointing; weakest across the baseline)</span>`) : row("Position uncertainty", `± ${fmtLenShort(fix.posErr)} (from a ±1° pointing error)`)) +
      row("Quality rating", fix.rating + (fix.behind ? " — rays cross BEHIND an observer; treat as unreliable (see caveats)" : "")) +
      `</table>`;
    fixHtml = `<table>` +
      row("Object ground position", `${fix.geoA.lat.toFixed(5)}, ${fix.geoA.lon.toFixed(5)} (± ${fmtLenShort(fix.posErr)})`) +
      row("Altitude (MSL)", lp(mslA)) +
      row("Altitude above the observers", lp(fix.solA.X[2])) +
      row("Range from each observer", fix.perSource.map((p, i) => `${e2(p.name || "Observer " + (i + 1))}: ${fmtLenShort(p.dist)}`).join(" · ")) +
      (fix.sizeAvg != null ? row("Object size (avg)", lp(fix.sizeAvg)) : "") +
      ((asp) => asp ? row("Aspect-corrected span (if elongated)", `${asp.map((x) => `${fmtLenShort(x.S)} @ long-axis ${Math.round(x.psi)}°`).join(" or ")} (${asp[0].n} views${asp[0].rms != null ? `, fit rms ${fmtLenShort(asp[0].rms)}` : ""})`) : "")(aspectSpan(fix)) +
      (fix.motion?.speed != null ? row("Speed A→B", `${sp(fix.motion.speed)}, heading ${Math.round(fix.motion.heading)}° ${compass8(fix.motion.heading)}${isNum(fix.motion.vRate) ? `, vertical ${fix.motion.vRate >= 0 ? "climb" : "descent"} ${fmtSpeedShort(Math.abs(fix.motion.vRate))}` : ""}`) : "") +
      (fix.motion?.disp != null ? row("Displacement A→B", `${fmtLenShort(fix.motion.disp)}${fix.motion.dt != null ? ` over ${fix.motion.dt.toFixed(2)} s` : ""} (Δalt ${fmtLenShort(fix.motion.XB[2] - fix.solA.X[2])})`) : "") +
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
      /* cloud-base cap: if the object was below the deck, its range < base/sin(el),
         so everything to the RIGHT of this line is ruled out for a below-cloud object */
      const cb = (wxDeck && drawAlt) ? cloudRangeBound(wx.baseAGL, el, wAng) : null;
      const cloudCut = (cb && cb.maxRange >= D0 && cb.maxRange <= D1) ? (() => {
        const xc = X(cb.maxRange);
        return `<rect x="${xc.toFixed(1)}" y="${T}" width="${(W - Rm - xc).toFixed(1)}" height="${H - T - B}" fill="rgba(120,120,120,.10)"/>` +
          `<line x1="${xc.toFixed(1)}" y1="${T}" x2="${xc.toFixed(1)}" y2="${H - B}" stroke="#7a7a7a" stroke-width="1.5" stroke-dasharray="5 3"/>` +
          `<text x="${(xc - 5).toFixed(1)}" y="${T + 12}" font-size="10" fill="#555" text-anchor="end">cloud base ≈ ${fmtLenShort(cb.maxRange)} if below</text>`;
      })() : "";
      fixHtml += `<svg viewBox="0 0 ${W} ${H}" style="max-width:100%;border:1px solid #ddd;border-radius:6px;background:#fff">
<text x="${L}" y="20" font-size="12" font-weight="700" fill="#333">Assumed distance ⇄ implied size${drawAlt ? " &amp; altitude" : ""} (${wAng.toFixed(2)}° wide${drawAlt ? `, ${el.toFixed(0)}° up` : ""})</text>
<text x="${W - Rm}" y="13" font-size="10" fill="#0e7d6f" text-anchor="end">■ size</text>${drawAlt ? `<text x="${W - Rm}" y="26" font-size="10" fill="#2563c9" text-anchor="end">■ altitude above you</text>` : ""}
${xTicks}${yTicks}${cloudCut}${refs}${altLine}
<line x1="${X(D0)}" y1="${Y(s0).toFixed(1)}" x2="${X(D1)}" y2="${Y(s1).toFixed(1)}" stroke="#0e7d6f" stroke-width="2.5"/>
<text x="${W / 2}" y="${H - 8}" font-size="10" fill="#888" text-anchor="middle">assumed distance →</text>
<text x="14" y="${H / 2}" font-size="10" fill="#888" transform="rotate(-90 14 ${H / 2})" text-anchor="middle">size / altitude (m) →</text>
</svg>
<p class="cap">One witness can't fix the distance — but every assumed distance implies both a size and ${drawAlt ? `an altitude above you (from the ${el.toFixed(0)}° sight-line). Amber dots mark common objects at that size; blue dots mark notable altitudes.` : "a size. Dots mark where common objects would sit on this sight-line. (Add the object's elevation to also read altitude.)"}${cb && cb.maxRange >= D0 && cb.maxRange <= D1 ? ` <b>If the object was below the cloud deck</b> (base est. ${fmtLenShort(wx.baseAGL)} AGL), it was within <b>${fmtLenShort(cb.maxRange)}</b>${cb.maxSize != null ? ` and no larger than <b>${fmtLenShort(cb.maxSize)}</b>` : ""} — everything right of the grey line is ruled out. Above the deck flips this into a floor.` : ""}</p>`;
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
      const dimRow = (lbl, u) => row(lbl, mum != null ? lp(u * mum) : `${(u / Math.max(ext.x, ext.y, ext.z, 1e-6)).toFixed(2)}× (relative — no absolute scale)`);
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
    row("Avg / peak speed", `${fmtSpeedShort(tr.stereo.k.avgSpeed)} / ${fmtSpeedShort(tr.stereo.k.peakSpeed)} peak`) +
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
      /* only aircraft that actually fell inside a photo frame — an airliner
         50° off the pointing is not "in the picture" and just clutters the report */
      const framed = cands.filter((c) => c.per && c.per[0] && inAnyFrame(c.per[0].az, c.per[0].el));
      const rows = framed.slice(0, 8).map((c) => {
        const p = c.per[0];
        return `<tr><td>${e2(c.flight || c.reg || c.hex)}${c.t ? ` · ${e2(c.t)}` : ""}</td><td>${c.span != null ? fmtLenShort(c.span) : "—"}</td><td>${c.sepMax.toFixed(1)}°</td><td>${p.az.toFixed(0)}° / ${p.el.toFixed(0)}°</td><td>${fmtLenShort(p.rangeM)}</td><td>${p.predAng != null ? p.predAng.toFixed(2) + "°" : "—"}${measA != null ? ` vs ${measA.toFixed(2)}°` : ""}</td><td>${c.altM != null ? Math.round(c.altM * 3.28084).toLocaleString() + " ft" : "—"}${c.gs != null ? ` · ${fmtSpeedShort(c.gs)}` : ""}</td></tr>`;
      }).join("");
      const best = framed[0];
      /* route enrichment for the best in-frame match only (one adsbdb call) —
         turns a hex/callsign into "SFO → SEA, B738" */
      let routeTxt = "";
      if (best && (best.hex || best.flight)) {
        try {
          const info = await fetchAcInfo(best.hex, best.flight);
          const fr = info && info.route;
          const nm = (a) => a && (a.municipality || a.iata_code || a.icao_code) || "";
          const code = (a) => a && (a.iata_code || a.icao_code) || "";
          if (fr && fr.origin && fr.destination) {
            routeTxt = ` Route: <b>${e2(nm(fr.origin))}${code(fr.origin) ? ` (${e2(code(fr.origin))})` : ""} → ${e2(nm(fr.destination))}${code(fr.destination) ? ` (${e2(code(fr.destination))})` : ""}</b>${info.aircraft && info.aircraft.type ? `, ${e2(info.aircraft.type)}` : ""} <span class="cap">(adsbdb)</span>.`;
          }
        } catch (e) { /* adsbdb offline — omit route */ }
      }
      const verdict = !cands.length
        ? `No airborne transponder aircraft were within ${snap.nm} nm at check time. ADS-B absence rules out airliners and most GA — not military or non-transponder traffic.`
        : !framed.length
          ? `No transponder aircraft fell inside any photo frame (${cands.length} were airborne nearby; closest was ${cands[0].sepMax.toFixed(1)}° outside the frame).`
          : best.sepMax < 2.5
            ? `<b>${e2(best.flight || best.reg || best.hex)}</b> was in frame, within ${best.sepMax.toFixed(1)}° of every witness sight-line — a strong mundane candidate; compare its predicted angular size against the measurement above.`
            : `${framed.length} aircraft fell in frame; the nearest to the marked object (${e2(best.flight || best.reg || best.hex)}) was ${best.sepMax.toFixed(1)}° off.`;
      /* one-line thesis with a likelihood the object was a plane — separation
         from the sight-line + whether the predicted angular size matches. */
      const bp = best ? best.per[0] : null;
      const sizeRatio = (bp && bp.predAng != null && measA != null && measA > 0) ? bp.predAng / measA : null;
      const sizeOk = sizeRatio != null ? (sizeRatio > 0.4 && sizeRatio < 2.5) : null;
      const acAssess = !cands.length
        ? `<b>Assessment — aircraft: ruled out.</b> No transponder traffic was in range (military / non-transponder craft can't be excluded this way).`
        : !framed.length
          ? `<b>Assessment — aircraft: unlikely.</b> ${cands.length} were airborne nearby, but none fell inside the photo frame.`
          : (best.sepMax < 1.2 && sizeOk !== false)
            ? `<b>Assessment — aircraft: very likely.</b> A transponder aircraft sat just ${best.sepMax.toFixed(1)}° from the marked object${sizeOk ? " and its predicted angular size matches the measurement" : ""}.`
            : best.sepMax < 2.5
              ? `<b>Assessment — aircraft: likely.</b> The nearest match was ${best.sepMax.toFixed(1)}° off the sight-line${sizeOk === false ? ", though its predicted size doesn't match well — check the numbers" : ""}.`
              : best.sepMax < 6
                ? `<b>Assessment — aircraft: possible.</b> The nearest in-frame aircraft was ${best.sepMax.toFixed(1)}° off the marked object.`
                : `<b>Assessment — aircraft: unlikely.</b> The nearest in-frame aircraft was ${best.sepMax.toFixed(1)}° from the object.`;
      adsbHtml = `<h2>Aircraft check (ADS-B)</h2>
<p class="lead">${acAssess}</p>
<p class="cap">Transponder aircraft that fell <b>inside the photo frame</b>. Source: ${e2(snap.src)} · ${gapTxt || `captured ${new Date(snap.fetchedAt).toLocaleString()}`}</p>
${framed.length ? `<table><tr><th>Flight</th><th>Span</th><th>Off sight-line (worst witness)</th><th>Seen at az/el</th><th>Range</th><th>Would appear vs measured</th><th>Alt · speed</th></tr>${rows}</table>` : ""}
<p>${verdict}${routeTxt}</p>`;
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
    /* moment strip — the additional timestamped photos (Moment 2, 3, …) that
       build this observer's trajectory, each with its object mark + placed
       direction, so the path in the trajectory chart has visible provenance */
    let momStrip = "";
    const moms = (s.moments || []).filter((m) => m.mediaJpeg);
    if (moms.length) {
      const objOv = (m) => {
        if (m.shapeFit) {
          const col = `hsl(${m.shapeFit.hue ?? 36},85%,42%)`;
          const paths = shapeProjNat(m.shapeFit).curves.map((c) =>
            `<polyline fill="none" stroke="${col}" stroke-width="${Math.max(1, m.natW / 700)}" opacity="0.6" points="${c.map((p) => p.x.toFixed(1) + "," + p.y.toFixed(1)).join(" ")}"/>`).join("");
          return `<svg viewBox="0 0 ${m.natW} ${m.natH}" style="position:absolute;left:0;top:0;width:100%;height:100%">${paths}</svg>`;
        }
        if (m.A?.p1 && m.A?.p2) {
          const sw = Math.max(1.5, m.natW / 400);
          return `<svg viewBox="0 0 ${m.natW} ${m.natH}" style="position:absolute;left:0;top:0;width:100%;height:100%"><line x1="${m.A.p1.x}" y1="${m.A.p1.y}" x2="${m.A.p2.x}" y2="${m.A.p2.y}" stroke="#C77B14" stroke-width="${sw}"/><circle cx="${m.A.p1.x}" cy="${m.A.p1.y}" r="${Math.max(5, m.natW / 130)}" fill="none" stroke="#C77B14" stroke-width="${sw}"/><circle cx="${m.A.p2.x}" cy="${m.A.p2.y}" r="${Math.max(5, m.natW / 130)}" fill="none" stroke="#C77B14" stroke-width="${sw}"/></svg>`;
        }
        return "";
      };
      const cards = moms.map((m, mi) => {
        const mAdjSty = imgAdjFilter(m.imgAdj) === "none" ? "" : `filter:${imgAdjFilter(m.imgAdj)};`;
        const when = m.whenMs ? new Date(+m.whenMs).toLocaleTimeString() : "time unset";
        const dir = isNum(m.A?.az) && isNum(m.A?.el) ? `${(+m.A.az).toFixed(1)}° az / ${(+m.A.el).toFixed(1)}° el` : "not placed";
        return `<div style="flex:1 1 200px;min-width:150px;max-width:280px"><div style="position:relative;display:block"><img src="${m.mediaJpeg}" style="width:100%;display:block;border:1px solid #ccc;border-radius:4px;${mAdjSty}"/>${objOv(m)}</div><div class="cap">Moment ${mi + 2} · ${when} · ${dir}</div></div>`;
      }).join("");
      momStrip = `<div class="cap" style="margin-top:8px">Moments — additional photos building ${e2(s.name || "Observer " + (i + 1))}'s trajectory (direction over time):</div>
<div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:4px">${cards}</div>`;
    }
    return `<h2>Exhibit — ${e2(s.name || "Observer " + (i + 1))}</h2>
<div style="position:relative;display:inline-block;max-width:100%"><img src="${imgSrc}" style="max-width:100%;display:block;${adjSty}"/>${overlay}</div>
<div class="cap">${s.meta?.model ? e2(s.meta.model) + " · " : ""}${s.whenMs ? new Date(+s.whenMs).toLocaleString() : ""}${s.mediaAim ? ` · placed ${(+s.mediaAim.az).toFixed(1)}° az / ${(+s.mediaAim.el).toFixed(1)}° el` : ""}${s.shapeFit ? ` · ${e2(s.shapeFit.kind)} fit` : ""}${moms.length ? ` · Moment 1 of ${moms.length + 1}` : ""}${adjCap}</div>
${detailBlock}
${momStrip}`;
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
  /* --- object photometry: colour + brightness from the photo's own pixels,
     and a ROUGH apparent magnitude when a catalogued star shares the frame
     (phone tone-mapping is nonlinear → order-of-magnitude, labelled). --- */
  let photomHtml = "";
  {
    const phCands = origAct.filter((x) => x.mediaUrl && x.mediaKind !== "video" && x.natW && x.natH && x.A?.p1 && x.A?.p2 && isNum(x.fovH));
    if (phCands.length && typeof document !== "undefined") {
      const results = [];
      for (const s of phCands) {
        try {
          const im = await new Promise((res, rej) => { const i = new Image(); i.onload = () => res(i); i.onerror = rej; i.src = s.mediaUrl; });
          const W = s.natW, H = s.natH, cap = 2000, sc = Math.min(1, cap / Math.max(W, H));
          const cw = Math.round(W * sc), ch = Math.round(H * sc);
          const cv = document.createElement("canvas"); cv.width = cw; cv.height = ch;
          const ctx = cv.getContext("2d"); ctx.drawImage(im, 0, 0, cw, ch);
          const D = ctx.getImageData(0, 0, cw, ch).data;
          const ocx = (s.A.p1.x + s.A.p2.x) / 2 * sc, ocy = (s.A.p1.y + s.A.p2.y) / 2 * sc;
          const orad = clampN(Math.hypot(s.A.p1.x - s.A.p2.x, s.A.p1.y - s.A.p2.y) / 2 * sc, 3, Math.min(cw, ch) * 0.25);
          const obj = aperture(D, cw, ch, ocx, ocy, orad);
          if (!obj) continue;
          const col = colorDesc(obj.r, obj.g, obj.b);
          const ma = s.mediaAim || {};
          const caz = isNum(ma.az) ? +ma.az : (isNum(s.A?.az) ? +s.A.az : null);
          const cel = isNum(ma.el) ? +ma.el : (isNum(s.A?.el) ? +s.A.el : null);
          const when = isNum(s.whenMs) ? +s.whenMs : Date.now();
          let refMag = null, refName = null, refFlux = null;
          if (caz != null && cel != null && isNum(s.lat) && isNum(s.lon)) {
            const cands = [];
            for (const [ra, dec, mag, name] of STARS) {
              if (mag > 2.2 || !name) continue;
              const p = raDecToAzEl(ra, dec, when, +s.lat, +s.lon); if (p.alt < 2) continue;
              const px = dirToPixK(dirFromAzEl(p.az, p.alt), W, H, caz, cel, isNum(ma.roll) ? +ma.roll : 0, +s.fovH, isNum(ma.dist) ? +ma.dist : 0);
              if (!px) continue;
              const sx = px.px * sc, sy = px.py * sc;
              if (sx < 6 || sy < 6 || sx > cw - 6 || sy > ch - 6) continue;
              if (Math.hypot(sx - ocx, sy - ocy) < orad * 1.5) continue; // not the object itself
              const ap = aperture(D, cw, ch, sx, sy, 5);
              if (ap && ap.flux > 0 && ap.satFrac < 0.4) cands.push({ mag, name, flux: ap.flux });
            }
            cands.sort((a, b) => a.mag - b.mag);
            if (cands.length) { refMag = cands[0].mag; refName = cands[0].name; refFlux = cands[0].flux; }
          }
          results.push({ name: s.name, col, satHi: obj.satFrac > 0.15, peak: obj.peak, m: refFlux ? relMag(obj.flux, refFlux, refMag) : null, refName, refMag });
        } catch (e) { /* skip this observer's photo */ }
      }
      if (results.length) {
        const r0 = results[0];
        const photAssess = /red|orange/.test(r0.col)
          ? "a reddish light — an aircraft's red beacon/nav light, a distant sodium lamp, or Mars"
          : /green/.test(r0.col)
            ? "a greenish light — an aircraft's green nav light, or an atmospheric/lens tint"
            : /blue/.test(r0.col)
              ? "a blue-white light — an LED, a xenon strobe, or a hot star"
              : r0.satHi
                ? "a very bright, saturated point — consistent with a landing light, a bright planet, or a specular flare"
                : "unremarkable in colour and brightness — not diagnostic on its own";
        const anyMag = results.some((r) => r.m != null);
        const rows = results.map((r, i) => `<tr><td>${e2(r.name || "Observer " + (i + 1))}</td><td>${e2(r.col)}</td><td>${r.satHi ? "saturated core" : `peak ${Math.round(100 * r.peak / 255)}%`}</td><td>${r.m != null ? `${r.m.toFixed(1)}${r.satHi ? " (floor)" : ""} <span class="cap">vs ${e2(r.refName)} (${r.refMag})</span>` : "—"}</td></tr>`).join("");
        photomHtml = `<h2>Object photometry</h2><p class="lead"><b>Assessment — appearance:</b> ${photAssess}.</p>
<table><tr><th>Observer</th><th>Colour</th><th>Brightness</th><th>Apparent mag (rough)</th></tr>${rows}</table>
<p class="cap">Measured from each photo's own pixels (aperture on the marked object, sky background subtracted). ${anyMag ? "Where a catalogued star shared the frame, the magnitude is calibrated against it — phone HDR/tone-mapping is nonlinear, so treat it as order-of-magnitude (±~1 mag); a saturated core makes it a floor." : "No catalogued star was cleanly in any frame to anchor an absolute magnitude, so only colour and relative brightness are given."} A steady red/green pair points to aircraft navigation lights; a saturated warm-white point is typical of a landing light or a bright planet.</p>`;
      }
    }
  }
  /* --- VIDEO ANALYSIS: for each stabilized + object-tracked video, the dense
     per-frame ANGULAR trajectory (measured, distance-free) → angular rate,
     total sweep, angular-size profile → what it all implies at a ladder of
     candidate distances (or the actual stereo-fix distance), plus a strip of
     keyframes with the tracked object marked and captioned. This is the
     scientific-analysis surface for footage. --- */
  let videoHtml = "";
  {
    const vids = origAct.filter((s) => s.mediaKind === "video" && s.mediaUrl && Array.isArray(s.objPath) && s.objPath.length >= 3 && Array.isArray(s.posePath) && s.posePath.length >= 2 && s.natW && s.natH);
    if (vids.length && typeof document !== "undefined") {
      const spd = (mps) => isImperialUnits() ? `${Math.round(mps * 2.23694)} mph` : `${mps < 1 ? mps.toFixed(1) : Math.round(mps)} m/s`;
      /* bake keyframes: seek an offscreen video to each time, draw to a capped
         canvas, mark the tracked object (world dir → this frame's solved pose →
         pixel), return {t, jpeg, mark, ...}. Sequential single-in-flight seeks. */
      const bakeKeyframes = (s, vk, times) => new Promise((resolve) => {
        const v = document.createElement("video");
        v.muted = true; v.playsInline = true; v.preload = "auto";
        const objAt = (t) => {
          const op = s.objPath;
          let lo = 0, hi = op.length - 1;
          if (t <= op[0].t) hi = 0; else if (t >= op[op.length - 1].t) lo = op.length - 1;
          else while (hi - lo > 1) { const m = (lo + hi) >> 1; if (op[m].t <= t) lo = m; else hi = m; }
          const a = op[lo], b = op[hi], u = hi === lo ? 0 : (t - a.t) / Math.max(1e-9, b.t - a.t);
          const dAz = ((b.az - a.az + 540) % 360) - 180;
          return { az: (((a.az + dAz * u) % 360) + 360) % 360, el: a.el + (b.el - a.el) * u };
        };
        const out = [];
        let idx = 0;
        const done = () => { try { v.removeAttribute("src"); v.load(); } catch (e) { } resolve(out); };
        const step = () => {
          if (idx >= times.length) return done();
          const t = times[idx];
          let fired = false;
          const wd = setTimeout(() => { if (!fired) { fired = true; out.push(null); idx++; step(); } }, 3000);
          v.onseeked = () => {
            if (fired) return; fired = true; clearTimeout(wd); v.onseeked = null;
            try {
              const cap = 900, sc = Math.min(1, cap / Math.max(v.videoWidth, v.videoHeight));
              const cw = Math.round(v.videoWidth * sc), ch = Math.round(v.videoHeight * sc);
              const cv = document.createElement("canvas"); cv.width = cw; cv.height = ch;
              const ctx = cv.getContext("2d");
              if (imgAdjFilter(s.imgAdj) !== "none") ctx.filter = imgAdjFilter(s.imgAdj);
              ctx.drawImage(v, 0, 0, cw, ch); ctx.filter = "none";
              const pp = posePathAt(s.posePath, t);
              const od = objAt(t);
              let mark = null;
              if (pp) { const px = dirToPixK(dirFromAzEl(od.az, od.el), s.natW, s.natH, pp.az, pp.el, pp.roll || 0, pp.fov, pp.k || 0); if (px) mark = { x: px.px * sc, y: px.py * sc }; }
              // angular size at t (interpolated), and instantaneous rate
              const smp = vk.samples.reduce((best, x) => Math.abs(x.t - t) < Math.abs(best.t - t) ? x : best, vk.samples[0]);
              const rr = vk.rate.reduce((best, x) => Math.abs(x.t - t) < Math.abs(best.t - t) ? x : best, vk.rate[0]);
              out.push({ t, jpeg: cv.toDataURL("image/jpeg", 0.82), w: cw, h: ch, mark, az: od.az, el: od.el, ang: smp.ang, omega: rr ? rr.omega : null });
            } catch (e) { out.push(null); }
            idx++; step();
          };
          try { v.currentTime = Math.min(t, (v.duration || t + 1) - 0.05); } catch (e) { if (!fired) { fired = true; clearTimeout(wd); out.push(null); idx++; step(); } }
        };
        v.onloadeddata = () => step();
        v.onerror = () => done();
        v.src = s.mediaUrl;
        try { v.load(); } catch (e) { }
      });
      const blocks = [];
      for (let vi = 0; vi < vids.length; vi++) {
        const s = vids[vi];
        const vk = videoKinematics(s);
        if (!vk) continue;
        /* absolute distance from THIS observer, if a stereo fix nailed it */
        let fixDist = null;
        if (fix.ok && isNum(s.lat) && isNum(s.lon)) {
          const Po = [(+s.lon - fix.ref.lon) * 111320 * Math.cos(fix.ref.lat * D2R), (+s.lat - fix.ref.lat) * 111320, (isNum(s.alt) ? +s.alt : 0) - fix.ref.alt];
          fixDist = Math.hypot(fix.solA.X[0] - Po[0], fix.solA.X[1] - Po[1], fix.solA.X[2] - Po[2]);
        }
        /* candidate distance ladder (+ the fix distance if we have one) */
        const ladder = [100, 300, 1000, 3000, 10000];
        const dists = fixDist ? [fixDist] : ladder;
        const distRows = dists.map((D) => {
          const a = vk.atDistance(D);
          if (!a) return "";
          return `<tr${fixDist ? ' style="background:#eef7ff"' : ""}><td>${fmtLenShort(D)}${fixDist ? " <b>(triangulated)</b>" : ""}</td><td>${a.sizeM != null ? fmtLenShort(a.sizeM) : "<span class='cap'>size not marked</span>"}</td><td>${spd(a.avgSpeed)}</td><td>${spd(a.peakSpeed)}</td><td>${fmtLenShort(a.path)}</td></tr>`;
        }).join("");
        /* angular-rate mini plot (ω vs t), self-contained SVG */
        const rateSvg = (() => {
          const W = 480, H = 120, m = 26;
          const ts = vk.rate.map((r) => r.t), ws = vk.rate.map((r) => r.omega);
          const t0 = ts[0], t1 = ts[ts.length - 1], wMax = Math.max(...ws, 0.1);
          const X = (t) => m + (t1 > t0 ? (t - t0) / (t1 - t0) : 0) * (W - m - 8);
          const Y = (w) => H - m - (w / wMax) * (H - m - 10);
          const pts = vk.rate.map((r) => `${X(r.t).toFixed(1)},${Y(r.omega).toFixed(1)}`).join(" ");
          return `<svg viewBox="0 0 ${W} ${H}" style="width:100%;max-width:${W}px;height:auto;border:1px solid #e2e2e2;border-radius:6px;background:#fff">
<line x1="${m}" y1="${H - m}" x2="${W - 8}" y2="${H - m}" stroke="#bbb"/><line x1="${m}" y1="10" x2="${m}" y2="${H - m}" stroke="#bbb"/>
<polyline fill="none" stroke="#1188aa" stroke-width="1.8" points="${pts}"/>
<text x="${m}" y="9" fill="#666" font-size="10">${wMax.toFixed(1)}°/s</text>
<text x="${W - 8}" y="${H - 9}" font-size="10" fill="#666" text-anchor="end">${(t1 - t0).toFixed(1)} s</text>
<text x="4" y="${(H) / 2}" font-size="10" fill="#666" transform="rotate(-90 10 ${H / 2})">angular rate</text></svg>`;
        })();
        // keyframes: evenly spaced across the tracked span + the marked frame
        const span = vk.samples;
        const t0 = span[0].t, t1 = span[span.length - 1].t;
        const nKF = clampN(Math.round((t1 - t0) / 1.2) + 2, 4, 8);
        const kfTimes = Array.from({ length: nKF }, (_, i) => +(t0 + (t1 - t0) * i / (nKF - 1)).toFixed(3));
        const kf = await bakeKeyframes(s, vk, kfTimes);
        const kfCards = kf.filter(Boolean).map((f) => {
          const col = s.shapeFit ? `hsl(${s.shapeFit.hue ?? 36},85%,45%)` : "#e23";
          const markSvg = f.mark ? `<svg viewBox="0 0 ${f.w} ${f.h}" style="position:absolute;left:0;top:0;width:100%;height:100%"><circle cx="${f.mark.x.toFixed(1)}" cy="${f.mark.y.toFixed(1)}" r="${Math.max(8, f.w / 34).toFixed(1)}" fill="none" stroke="${col}" stroke-width="${Math.max(1.5, f.w / 320).toFixed(2)}"/><line x1="${f.mark.x.toFixed(1)}" y1="${(f.mark.y - f.w / 22).toFixed(1)}" x2="${f.mark.x.toFixed(1)}" y2="${(f.mark.y + f.w / 22).toFixed(1)}" stroke="${col}" stroke-width="${Math.max(0.8, f.w / 500).toFixed(2)}"/><line x1="${(f.mark.x - f.w / 22).toFixed(1)}" y1="${f.mark.y.toFixed(1)}" x2="${(f.mark.x + f.w / 22).toFixed(1)}" y2="${f.mark.y.toFixed(1)}" stroke="${col}" stroke-width="${Math.max(0.8, f.w / 500).toFixed(2)}"/></svg>` : "";
          const adjSty = imgAdjFilter(s.imgAdj) === "none" ? "" : `filter:${imgAdjFilter(s.imgAdj)};`;
          return `<div style="flex:1 1 210px;min-width:170px;max-width:300px"><div style="position:relative"><img src="${f.jpeg}" style="width:100%;display:block;border:1px solid #ccc;border-radius:4px;${adjSty}"/>${markSvg}</div><div class="cap">t ${f.t.toFixed(2)} s · ${f.az.toFixed(1)}° az / ${f.el.toFixed(1)}° el${f.ang != null ? ` · ${f.ang.toFixed(2)}° wide` : ""}${f.omega != null ? ` · ${f.omega.toFixed(1)}°/s` : ""}</div></div>`;
        }).join("");
        const held = vk.samples.filter((x) => (x.q || 0) < 0.15).length;
        const angLine = vk.angMin != null
          ? `Its apparent width ranged ${vk.angMin.toFixed(2)}°–${vk.angMax.toFixed(2)}°${vk.rangeRatio && Math.abs(vk.rangeRatio - 1) > 0.05 ? ` — a <b>${vk.rangeRatio.toFixed(2)}× range change</b> (it moved ${vk.rangeRatio > 1 ? "closer then" : ""} ${vk.angMax > vk.angMin ? "nearer" : "farther"} over the clip)` : " (near-constant — little toward/away motion)"}.`
          : `Angular size was not marked frame-to-frame, so only transverse (across-sky) motion is measured — mark the object's width on a few frames (measure step) to recover toward/away motion.`;
        blocks.push(`${vids.length > 1 ? `<h3>${e2(s.name || "Observer " + (vi + 1))}</h3>` : ""}
<p class="lead"><b>Measured angular motion:</b> the object swept <b>${vk.sweep.toFixed(1)}°</b> of sky over <b>${vk.dur.toFixed(1)} s</b> (${vk.n} tracked frames), averaging <b>${vk.avgOmega.toFixed(2)}°/s</b> and peaking at <b>${vk.peakOmega.toFixed(2)}°/s</b>. ${angLine}${held ? ` <span class="cap">(${held} frame${held > 1 ? "s" : ""} held on the guide, not pixel-locked.)</span>` : ""}</p>
${rateSvg}
<p class="cap" style="margin-top:8px">Angular position &amp; rate are measured directly from the world-locked track — no distance needed. Linear size and speed below follow only once a distance is assumed${fixDist ? " (here fixed by triangulation)" : ""}:</p>
<table><tr><th>Assumed distance</th><th>True size</th><th>Avg speed</th><th>Peak speed</th><th>Path length</th></tr>${distRows}</table>
${fixDist ? "" : `<p class="cap">Single viewpoint — distance is unknown, so the row you believe fixes everything else. A second observer's video triangulates the true distance and collapses this to one row.</p>`}
${kfCards ? `<p class="cap" style="margin-top:10px"><b>Keyframes</b> — the tracked object marked (${s.shapeFit ? "shape colour" : "red"} reticle) at sampled moments:</p><div style="display:flex;gap:8px;flex-wrap:wrap;margin-top:4px">${kfCards}</div>` : ""}`);
      }
      if (blocks.length) videoHtml = `<h2>Video analysis</h2>${blocks.join('<hr style="border:none;border-top:1px solid #eee;margin:18px 0"/>')}`;
    }
  }
  /* --- TWO-VIDEO TRAJECTORY: ≥2 stabilized+tracked clips of the same object,
     time-synced and triangulated per frame into a dense absolute 3D path. This
     is the accurate answer — true distance, size, speed, g-load over the whole
     clip — that a single video can only give conditionally. --- */
  let vstereoHtml = "";
  {
    const vs = stereoVideo(origAct);
    if (vs && vs.ok && vs.n >= 3) {
      const spd = (m) => fmtSpeedShort(m);
      const syncTxt = vs.syncConf > 0.4 ? "well-constrained" : vs.syncConf > 0.12 ? "moderate" : "soft (a far/slow object barely pins the clocks — treat absolute speed as approximate)";
      const offEach = origAct.filter((s) => s.mediaKind === "video" && Array.isArray(s.objPath));
      /* true SIZE over the path when an observer sized the object across frames:
         angular size (interpolated to each instant) × that observer's range */
      let sizeHtml = "";
      {
        const sizer = offEach.find((s) => (s.track || []).some((p) => isNum(p.t) && isNum(p.ang) && +p.ang > 0));
        const pm = sizer ? vs.perObs.find((o) => o.name === sizer.name) : null;
        if (sizer && pm) {
          const sized = (sizer.track || []).filter((p) => isNum(p.t) && isNum(p.ang) && +p.ang > 0).map((p) => ({ t: +p.t, ang: +p.ang })).sort((a, b) => a.t - b.t);
          const base = (isNum(sizer.whenMs) ? +sizer.whenMs / 1000 : 0) + (isNum(sizer.syncOffset) ? +sizer.syncOffset : 0);
          const angAt = (tv) => { if (tv <= sized[0].t) return sized[0].ang; if (tv >= sized[sized.length - 1].t) return sized[sized.length - 1].ang; let i = 0; while (i < sized.length - 1 && sized[i + 1].t < tv) i++; const a = sized[i], b = sized[i + 1]; return a.ang + (b.ang - a.ang) * ((tv - a.t) / Math.max(1e-9, b.t - a.t)); };
          const sizes = vs.times.map((t, i) => { const vt = t - base; const range = mag(sub(vs.pos[i], pm.P)); return 2 * range * Math.tan((angAt(vt) * D2R) / 2); });
          const smin = Math.min(...sizes), smax = Math.max(...sizes), savg = sizes.reduce((a, b) => a + b, 0) / sizes.length;
          sizeHtml = `<p class="lead" style="margin-top:8px"><b>True size (triangulated):</b> ${fmtLenShort(savg)} across on average${smax - smin > savg * 0.1 ? ` (ranging ${fmtLenShort(smin)}–${fmtLenShort(smax)} as attitude/aspect changed)` : ""} — angular size measured on ${e2(sizer.name || "one clip")} × its triangulated range.</p>`;
        }
      }
      // top-down plot (reuse the satellite-basemap plotter with a synthetic fix)
      let plot = "";
      try {
        const mid = vs.pos[vs.pos.length >> 1];
        const vfix = { ref: vs.ref, obs: vs.perObs.map((o) => ({ P: o.P, s: { name: o.name }, dA: unit(sub(mid, o.P)) })), solA: { X: mid, ts: vs.perObs.map((o) => mag(sub(mid, o.P))), rmsMiss: vs.meanMiss } };
        plot = await reportPlotSvg(vfix, vs.pos);
      } catch (e) { plot = ""; }
      const kh = vs.k ? `<table>` +
        row("Common instants / duration", `${vs.n} frames · ${(vs.window[1] - vs.window[0]).toFixed(1)} s`) +
        row("Path length", fmtLenShort(vs.k.path)) +
        row("Avg / peak speed", `${spd(vs.k.avgSpeed)} / ${spd(vs.k.peakSpeed)} peak`) +
        (vs.k.peakA != null ? row("Peak acceleration", vs.k.peakA.toFixed(1) + " m/s²") : "") +
        (vs.k.peakLoad != null ? row("Peak felt load", vs.k.peakLoad.toFixed(2) + " g") : "") +
        (vs.k.peakTurn != null ? row("Peak turn rate", vs.k.peakTurn.toFixed(1) + " °/s") : "") +
        `</table>` + reportTrajSvg(vs.k) : "";
      vstereoHtml = `<h2>Two-video trajectory (dense stereo)</h2>
<p class="lead"><b>Triangulated from ${vs.nObs} clips</b> frame-by-frame: ${vs.n} common instants over ${(vs.window[1] - vs.window[0]).toFixed(1)} s. The clips were auto-synchronised (${vs.offset >= 0 ? "+" : ""}${vs.offset.toFixed(2)} s relative offset, ${syncTxt})${vs.dropped ? `, and ${vs.dropped} mistracked frame${vs.dropped > 1 ? "s were" : " was"} rejected` : ""}.</p>
<table><tr><th>Fix geometry</th><th></th></tr>
${row("Baseline (observer spacing)", fmtLenShort(vs.baseline))}
${row("Mean convergence angle", `${vs.conv.toFixed(1)}°${vs.conv < 6 ? " — shallow; depth (range/speed) less certain" : ""}`)}
${row("Mean ray miss", fmtLenShort(vs.meanMiss))}
${vs.perObs.map((o) => row(`Range from ${e2(o.name || "observer")}`, fmtLenShort(o.meanRange))).join("")}
</table>
${sizeHtml}
${kh}
${plot ? `<div style="margin-top:10px">${plot}</div><p class="cap">Top-down: the two sight-line fans and the triangulated 3D path (blue), over satellite imagery.</p>` : ""}
<p class="cap">Each frame's object direction comes from the stabilized, world-locked track (camera motion removed), so this is a direct per-instant intersection of two real sight-lines — no assumed distance. Accuracy is bounded by the ${fmtLenShort(vs.baseline)} baseline vs the ${fmtLenShort(vs.perObs[0]?.meanRange || 0)} range (convergence ${vs.conv.toFixed(1)}°), the ${vs.bothWhen ? "EXIF-seeded" : "manually-set"} time sync, and each clip's compass/sky alignment.</p>`;
    }
  }
  /* --- VIDEO + STILL: a dense clip anchored to absolute scale by one photo's
     sight-line — a full absolute trajectory from a mixed pair. Only when there
     ISN'T already a two-video fix (that's stronger). --- */
  let mixedHtml = "";
  {
    const mx = (!vstereoHtml) ? mixedStereo(origAct) : null;
    if (mx && mx.ok && mx.n >= 3) {
      const spd = (m) => fmtSpeedShort(m);
      let plot = "";
      try {
        const mid = mx.pos[mx.pos.length >> 1];
        const vfix = { ref: mx.ref, obs: [{ P: mx.Pv, s: { name: mx.names[0] }, dA: unit(sub(mid, mx.Pv)) }, { P: mx.Ps, s: { name: mx.names[1] }, dA: dirFromAzEl(0, 0) }], solA: { X: mx.anchor.X, ts: [mag(sub(mx.anchor.X, mx.Pv)), mag(sub(mx.anchor.X, mx.Ps))], rmsMiss: mx.anchor.rmsMiss } };
        plot = await reportPlotSvg(vfix, mx.pos);
      } catch (e) { plot = ""; }
      const kh = mx.k ? `<table>` +
        row("Frames / duration", `${mx.n} · ${(mx.times[mx.times.length - 1] - mx.times[0]).toFixed(1)} s`) +
        row("Path length", fmtLenShort(mx.k.path)) +
        row("Avg / peak speed", `${spd(mx.k.avgSpeed)} / ${spd(mx.k.peakSpeed)} peak`) +
        (mx.k.peakA != null ? row("Peak acceleration", mx.k.peakA.toFixed(1) + " m/s²") : "") +
        (mx.k.peakLoad != null ? row("Peak felt load", mx.k.peakLoad.toFixed(2) + " g") : "") +
        `</table>` + reportTrajSvg(mx.k) : "";
      mixedHtml = `<h2>Video + photo trajectory</h2>
<p class="lead"><b>${e2(mx.names[0] || "the clip")}</b> (a stabilized, object-tracked video) anchored to absolute scale by <b>${e2(mx.names[1] || "the photo")}</b>'s sight-line. The two rays meet best ${mx.anchor.vt.toFixed(2)} s into the clip — the object's true position at that instant — fixing its range at <b>${fmtLenShort(mx.anchor.dist)}</b>${mx.anchor.sizeM != null ? ` and true size at <b>${fmtLenShort(mx.anchor.sizeM)}</b>` : ""}; the clip's own ${mx.sized ? "size profile" : "constant range"} then scales that across every frame into a full 3D path.</p>
<table><tr><th>Anchor fix</th><th></th></tr>
${row("Baseline (video↔photo)", fmtLenShort(mx.baseline))}
${row("Convergence at the anchor", `${mx.conv.toFixed(1)}°${mx.conv < 6 ? " — shallow; range less certain" : ""}`)}
${row("Ray miss at the anchor", fmtLenShort(mx.anchor.rmsMiss))}
${row("Range at the anchor", fmtLenShort(mx.anchor.dist))}
${mx.sizeMin != null ? row("True size (min–max)", `${fmtLenShort(mx.sizeMin)} – ${fmtLenShort(mx.sizeMax)}`) : ""}
</table>
${kh}
${plot ? `<div style="margin-top:10px">${plot}</div><p class="cap">Top-down: the video's sight-line fan and the photo's single ray, the anchor fix, and the scaled 3D path (blue).</p>` : ""}
<p class="cap">${mx.sized ? "The object was sized across the clip, so its range (and true size) vary frame-to-frame; the photo pins the absolute scale." : "The object wasn't sized frame-to-frame, so range is held at the anchor value — absolute distance and tangential speed are recovered, but toward/away motion isn't. Size the object on a few frames (measure step) to capture it."} A second full video, or a photo taken closer to the object's mid-flight, tightens this further.</p>`;
    }
  }
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
      const wxRows = wx ? (() => {
        const cover = wx.cloud != null ? `${Math.round(wx.cloud)}%${[wx.low != null ? `low ${Math.round(wx.low)}%` : "", wx.mid != null ? `mid ${Math.round(wx.mid)}%` : "", wx.high != null ? `high ${Math.round(wx.high)}%` : ""].filter(Boolean).length ? ` (${[wx.low != null ? `low ${Math.round(wx.low)}%` : "", wx.mid != null ? `mid ${Math.round(wx.mid)}%` : "", wx.high != null ? `high ${Math.round(wx.high)}%` : ""].filter(Boolean).join(" · ")})` : ""}` : null;
        return (cover != null ? row("Cloud cover", cover) : "") +
          (wxDeck ? row("Cloud base (est.)", `${fmtLenShort(wx.baseAGL)} above ground <span class="cap">— from temp/dew-point (Espy); an object below the deck was closer than base ÷ sin(elevation)</span>`) : "") +
          (wx.visM != null ? row("Visibility", wx.visM >= 20000 ? "≥ 20 km (clear)" : fmtLenShort(wx.visM)) : "") +
          (wx.tempC != null ? row("Air", `${Math.round(wx.tempC)}°C${wx.dewC != null ? ` · dew point ${Math.round(wx.dewC)}°C` : ""}${wx.rh != null ? ` · ${Math.round(wx.rh)}% RH` : ""}`) : "");
      })() : row("Cloud / weather", `<span class="cap">unavailable — Open-Meteo returned no data for this time &amp; place (the cloud-base range cap is skipped)</span>`);
      const condAssess = wxDeck
        ? `<b>Assessment — conditions:</b> a low cloud deck (base ≈ ${fmtLenShort(wx.baseAGL)} AGL) was present — it caps a below-cloud object's range and size (see the size chart), and can itself catch and scatter ground light.`
        : (wx && wx.cloud != null && wx.cloud < 25)
          ? `<b>Assessment — conditions:</b> mostly clear skies — no cloud ceiling to bound the object's range.`
          : `<b>Assessment — conditions:</b> Sun/Moon geometry below is exact; use it to sanity-check the reported time and rule the Sun/Moon in or out as the light source.`;
      condHtml = `<h2>Sighting conditions</h2><p class="lead">${condAssess}</p><table>` +
        row("Local sky", `${tw} — Sun ${Math.abs(sun.alt).toFixed(1)}° ${sun.alt >= 0 ? "above" : "below"} the horizon at az ${Math.round(sun.az)}° ${compass8(sun.az)}`) +
        row("Moon", `${ill}% illuminated · ${moon.alt > 0 ? `${moon.alt.toFixed(0)}° up at az ${Math.round(moon.az)}° ${compass8(moon.az)}` : "below the horizon"}`) +
        (dec != null ? row("Magnetic declination", `${dec >= 0 ? "+" : ""}${dec.toFixed(1)}° (WMM2025 — added to any magnetic compass bearing to get true)`) : "") +
        wxRows +
        `</table>` +
        `<p class="cap">At the sighting location &amp; time (${new Date(Tw).toLocaleString()}); the Sun/Moon and declination are effectively identical for co-located observers. Sun/Moon geometry is exact — use it to sanity-check the reported time, and to rule the Sun/Moon in or out as glare or the light source.${wx ? ` Weather from ${wx.src}; cloud base is an estimate.` : ""}</p>`;
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
          if (!objInFrame(w, c.az, c.alt)) continue; // only what's actually in this photo's frame
          const sep = Math.acos(Math.min(1, Math.max(-1,
            d[0] * dirFromAzEl(c.az, c.alt)[0] + d[1] * dirFromAzEl(c.az, c.alt)[1] + d[2] * dirFromAzEl(c.az, c.alt)[2]))) * R2D;
          hits.push({ wit: w.name, ...c, sep });
        }
      }
      hits.sort((a, b) => a.sep - b.sep);
      const venusHit = hits.find((h) => h.label.includes("Venus"));
      const satHit = hits.find((h) => h.label.startsWith("🛰"));
      const satStale = satHit && satHit.stale > 5 ? ` (TLE epoch ≈ ${Math.round(satHit.stale)} d from the sighting — position approximate)` : "";
      const near = hits[0], nearName = near ? near.label.replace(/^\S+\s/, "") : "";
      const skyAssess = !hits.length
        ? `<b>Assessment — astronomical object: unlikely.</b> No bright planet, star, satellite, Sun or Moon fell in the frame at that time.`
        : near.sep < 2
          ? `<b>Assessment — likely ${e2(nearName)}.</b> It sat just ${near.sep.toFixed(1)}° from the marked object — a strong match for a known ${near.label.startsWith("🛰") ? "satellite" : "sky object"}.`
          : near.sep < 5
            ? `<b>Assessment — possibly ${e2(nearName)}.</b> The nearest bright body was ${near.sep.toFixed(1)}° from the object.`
            : `<b>Assessment — astronomical object: unlikely.</b> The nearest bright body in frame was ${near.sep.toFixed(1)}° from the object.`;
      skyHtml = `<h2>Sky-object check</h2><p class="lead">${skyAssess}</p><p class="cap">Bright planets, stars, satellites, the Sun &amp; Moon that fell <b>inside the photo frame</b> at the sighting time. "Off sight-line" is how far each sat from the marked object.</p>` + (hits.length
        ? `<table><tr><th>Object (in frame)</th><th>Witness</th><th>Off sight-line</th><th>At az/el</th></tr>` +
        hits.map((h) => `<tr><td>${e2(h.label)}</td><td>${e2(h.wit)}</td><td>${h.sep.toFixed(1)}°</td><td>${h.az.toFixed(1)}° / ${h.alt.toFixed(1)}°</td></tr>`).join("") +
        `</table>` + (venusHit ? `<p>⚠ <b>Venus was in frame, ${venusHit.sep.toFixed(1)}° from the marked object</b> — Venus is the single most-reported "UFO"; a stationary, slowly-setting brilliant light is its signature.</p>` : "")
        + (satHit ? `<p>🛰 <b>${e2(satHit.label.slice(2).trim())} was in frame, ${satHit.sep.toFixed(1)}° from the marked object</b> and sunlit${satStale} — a steady point gliding across the sky in minutes is a satellite's signature.</p>` : "")
        : `<p class="cap">No bright planet, star, satellite, Sun or Moon fell within the photo frame at the sighting time.</p>`);
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
    {
      try {
        const when = +(origAct.find((s) => isNum(s.whenMs))?.whenMs || Date.now());
        const altMSL = fix.solA.X[2] + (fix.ref.alt || 0);
        const prof = windProfR || await fetchWindProfile(fix.ref.lat, fix.ref.lon, when);
        const haveMotion = objSpeed != null && objSpeed > 0.2;
        // the level nearest the object's triangulated altitude (its balloon level)
        const nearest = prof.levels.reduce((b, L) => Math.abs(L.levelM - altMSL) < Math.abs(b.levelM - altMSL) ? L : b, prof.levels[0]);
        const nv = haveMotion ? balloonVerdict(objSpeed, objHeading, nearest) : null;
        const consistent = haveMotion ? prof.levels.filter((L) => balloonVerdict(objSpeed, objHeading, L).verdict === "balloon-consistent") : [];
        // full profile table, high altitude → surface, matching levels highlighted
        const rows = prof.levels.slice().reverse().map((L) => {
          const v = haveMotion ? balloonVerdict(objSpeed, objHeading, L) : null;
          const bg = v?.verdict === "balloon-consistent" ? " style=\"background:rgba(95,211,188,.18)\"" : v?.verdict === "partially wind-like" ? " style=\"background:rgba(245,169,63,.14)\"" : "";
          const cmp = v ? `${Math.round(v.dHead)}° · ${isFinite(v.ratio) ? v.ratio.toFixed(1) + "×" : "≫"} · ${v.verdict === "balloon-consistent" ? "✓ match" : v.verdict === "partially wind-like" ? "~ partial" : "✗"}` : "—";
          return `<tr${bg}><td>${fmtLenShort(L.levelM)}${L === nearest ? " <b>← object</b>" : ""}</td><td>${fmtSpeedShort(L.speedMs)} from ${Math.round(L.fromDeg)}° → drift ${Math.round(L.driftDeg)}°</td><td>${cmp}</td></tr>`;
        }).join("");
        const verdictLine = haveMotion
          ? `<p class="${nv.verdict === "balloon-consistent" ? "" : "cap"}">At the object's altitude (${fmtLenShort(altMSL)} MSL, ≈ ${nearest.hPa} hPa) the wind drifts <b>${fmtSpeedShort(nearest.speedMs)} toward ${Math.round(nearest.driftDeg)}°</b>; the object (${motionSrc}) moved <b>${fmtSpeedShort(objSpeed)} toward ${Math.round(objHeading)}°</b> — heading off ${Math.round(nv.dHead)}°, speed ${isFinite(nv.ratio) ? nv.ratio.toFixed(1) + "×" : "≫"}. <b>${nv.verdict === "balloon-consistent" ? "⚠ Consistent with a wind-borne object (balloon signature)." : nv.verdict === "partially wind-like" ? "Partially wind-like — not conclusive." : "Not wind-borne at its altitude: a balloon cannot do this."}</b>${nv.verdict !== "balloon-consistent" && consistent.length ? ` <span class="cap">The wind DOES match the motion at ${consistent.map((L) => fmtLenShort(L.levelM)).join(", ")}, but the object was triangulated at ${fmtLenShort(altMSL)} — so a balloon is ruled out unless that altitude is wrong.</span>` : ""}</p>`
          : `<p class="cap">No usable object motion to compare against — the winds-aloft profile is shown for reference.</p>`;
        const balloonAssess = !haveMotion
          ? `<b>Assessment — balloon: undetermined.</b> No object motion was captured to compare against the wind.`
          : nv.verdict === "balloon-consistent"
            ? `<b>Assessment — balloon: consistent.</b> The object's motion matches the wind at its altitude (a free balloon's signature).`
            : nv.verdict === "partially wind-like"
              ? `<b>Assessment — balloon: partially consistent.</b> Some wind-like motion, but not a clean match.`
              : `<b>Assessment — balloon: ruled out.</b> The wind at the object's altitude can't produce this motion.`;
        /* the object-altitude wind arrow drawn on each placed photo — kept HERE
           (not on the primary exhibits, where it read like the object's path) */
        const windObjHere = { driftDeg: nearest.driftDeg, speedMs: nearest.speedMs, levelM: nearest.levelM };
        const windPhotos = packed.map((s2, i2) => {
          const ov = windArrowOverlay(s2, windObjHere);
          if (!ov || !s2.mediaJpeg) return "";
          const f = imgAdjFilter(s2.imgAdj);
          return `<div style="position:relative;display:inline-block;max-width:280px;width:100%;vertical-align:top"><img src="${s2.mediaJpeg}" style="width:100%;display:block;border:1px solid #ccc;border-radius:4px;${f === "none" ? "" : `filter:${f};`}"/>${ov}<div class="cap">${e2(s2.name || "Observer " + (i2 + 1))}</div></div>`;
        }).filter(Boolean).join("");
        const windPhotoBlock = windPhotos
          ? `<div style="display:flex;gap:10px;flex-wrap:wrap;margin-top:12px">${windPhotos}</div><p class="cap">Blue arrow on each photo = the wind's drift direction at the object's altitude (${fmtLenShort(nearest.levelM)}), projected into that frame. Compare it to the object's apparent motion in the picture: a balloon drifts along the arrow.</p>`
          : "";
        windHtml = `<h2>Wind check (balloon test)</h2><p class="lead">${balloonAssess}</p>${verdictLine}
${reportWindSvg(prof, haveMotion ? objSpeed : null, haveMotion ? objHeading : null, nearest.levelM)}
${windPhotoBlock}
<table class="tbl"><thead><tr><th>Altitude (MSL)</th><th>Wind → drift</th><th>Δhdg · speed · match</th></tr></thead><tbody>${rows}</tbody></table>
<p class="cap">Winds aloft from ${prof.src}. A free balloon rides the wind at its altitude — matching heading (within ±25°) and speed (0.5–1.6×). Highlighted rows match the object's motion.</p>`;
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
    /* a placed photo lets us drop events whose direction isn't in frame (a
       fireball on the far horizon isn't in a straight-up shot); no frame → keep */
    const frameSrc = origAct.find((s) => isNum(s.fovH) && s.natW && (s.mediaAim || isNum(s.A?.az)) && isNum(s.lat));
    const evVisible = (la, lo, el) => (la == null || lo == null || !frameSrc) ? true : objInFrame(frameSrc, brg(+frameSrc.lat, +frameSrc.lon, la, lo), el);
    if (obs0) {
      const fmtDt = (h) => Math.abs(h) < 48 ? `${h >= 0 ? "+" : ""}${h.toFixed(1)} h` : `${h >= 0 ? "+" : ""}${(h / 24).toFixed(1)} d`;
      try {
        const L = (await fetchLaunches(+obs0.lat, +obs0.lon, when)).filter((x) => Math.abs(x.dtHours) <= 14 * 24 && evVisible(x.lat, x.lon, 2)).slice(0, 8);
        if (L.length) {
        const l0 = L[0], nearStar = L.find((x) => x.starlink && Math.abs(x.dtHours) < 72);
        const launchAssess = nearStar
          ? `<b>Assessment — rocket / Starlink: plausible.</b> A Starlink batch launched ${fmtDt(nearStar.dtHours)} — a fresh train of dots stays visible for days and is a very common "fleet of lights" report.`
          : Math.abs(l0.dtHours) < 3
            ? `<b>Assessment — rocket launch: plausible.</b> A launch ${fmtDt(l0.dtHours)}${l0.distKm != null ? ` within ${Math.round(l0.distKm)} km` : ""} — a twilight plume/upper stage can look anomalous for hundreds of km.`
            : `<b>Assessment — rocket launch: context only.</b> Launches occurred near the date; a match needs the timing to line up.`;
        launchHtml = `<h2>Launch check (rocket launches)</h2><p class="lead">${launchAssess}</p>
<p class="cap">Rocket launches near the sighting (Launch Library 2). A fresh Starlink batch is a moving &ldquo;train&rdquo; of dots for days after launch; a twilight launch plume is visible for hundreds of km.</p>
<table><tr><th>When (Δ)</th><th>Rocket / mission</th><th>Pad</th><th>Range</th></tr>${L.map((x) => `<tr><td>${new Date(x.net).toLocaleString()}<br><span class="cap">${fmtDt(x.dtHours)}</span></td><td>${e2(x.rocket || x.name)}${x.starlink ? " · <b>🛰 STARLINK</b>" : ""}<br><span class="cap">${e2(x.mission || "")}</span></td><td>${e2(x.padName || "")}</td><td>${x.distKm != null ? Math.round(x.distKm) + " km" : "—"}</td></tr>`).join("")}</table>`;
        }
      } catch (e) { /* offline / rate-limited — omit */ }
      try {
        const F = (await fetchFireballs(+obs0.lat, +obs0.lon, when)).filter((x) => Math.abs(x.dtHours) <= 24 && evVisible(x.lat, x.lon, x.altKm != null && x.distKm ? clampN(Math.atan2(x.altKm, x.distKm) * R2D, 0, 85) : 5)).slice(0, 6);
        if (F.length) {
        const f0 = F[0], strong = Math.abs(f0.dtHours) < 0.5 && (f0.distKm == null || f0.distKm < 500);
        const fireAssess = strong
          ? `<b>Assessment — fireball / meteor: strong.</b> A bolide was logged ${fmtDt(f0.dtHours)}${f0.distKm != null ? ` and ${Math.round(f0.distKm)} km away` : ""} — a close time-and-place match is a strong meteor explanation.`
          : `<b>Assessment — fireball / meteor: possible.</b> A bolide was logged ${fmtDt(f0.dtHours)} — check whether the timing matches your sighting.`;
        fireballHtml = `<h2>Fireball check (NASA CNEOS)</h2><p class="lead">${fireAssess}</p>
<p class="cap">Bright bolides logged by US Government sensors near the sighting time. A match within minutes and a few hundred km is a strong meteor explanation.</p>
<table><tr><th>When (Δ)</th><th>Energy (kt TNT)</th><th>Alt / speed</th><th>Range</th></tr>${F.map((x) => `<tr><td>${new Date(x.t).toLocaleString()}<br><span class="cap">${fmtDt(x.dtHours)}</span></td><td>${x.energyKt != null ? x.energyKt : "—"}${x.impactKt != null ? ` <span class="cap">(${x.impactKt} total)</span>` : ""}</td><td>${x.altKm != null ? x.altKm + " km" : "—"}${x.velKmS != null ? ` · ${x.velKmS} km/s` : ""}</td><td>${x.distKm != null ? Math.round(x.distKm) + " km" : "—"}</td></tr>`).join("")}</table>`;
        }
      } catch (e) { /* omit */ }
    }
  }
  /* meteor-shower check — static radiants, no network. Shows the annual showers
     active on the sighting date, each radiant's az/el at the time & place, and
     (if the photo is placed) how far the radiant sat from the sight-line: a
     streak coming FROM near a radiant is a strong meteor explanation. */
  let meteorHtml = "";
  {
    const obs0 = origAct.find((s) => isNum(s.lat) && isNum(s.lon) && isNum(s.whenMs));
    if (obs0) {
      const la = +obs0.lat, lo = +obs0.lon, when = +obs0.whenMs;
      const showers = activeShowers(when);
      const sightW = origAct.find((s) => isNum(s.A?.az) && isNum(s.A?.el));
      const sd = sightW ? dirFromAzEl(+sightW.A.az, +sightW.A.el) : null;
      if (showers.length) {
        let upCount = 0;
        const rows = showers.map((sh) => {
          const p = raDecToAzEl(sh.ra, sh.dec, when, la, lo);
          const up = p.alt > 0;
          if (up) upCount++;
          let sepTxt = "—";
          if (up && sd) {
            const rd = dirFromAzEl(p.az, p.alt);
            const sep = Math.acos(Math.min(1, Math.max(-1, sd[0] * rd[0] + sd[1] * rd[1] + sd[2] * rd[2]))) * R2D;
            sepTxt = `${Math.round(sep)}°`;
          }
          const pk = sh.daysFromPeak === 0 ? "at peak" : `${Math.abs(sh.daysFromPeak)} d ${sh.daysFromPeak < 0 ? "before" : "after"} peak`;
          return `<tr><td>${e2(sh.name)}${sh.fireball ? ' · <b>fireball-rich</b>' : ""}<br><span class="cap">${pk}</span></td><td>${up ? `${Math.round(p.az)}° ${compass8(p.az)} · ${Math.round(p.alt)}° up` : "below horizon"}</td><td>${sepTxt}</td><td>ZHR ${sh.zhr} · ${sh.v} km/s</td></tr>`;
        }).join("");
        const meteorAssess = upCount
          ? `<b>Assessment — meteor: possible only for a fast streak.</b> ${upCount} shower radiant${upCount > 1 ? "s are" : " is"} above the horizon, so a quick streak lasting a second or two could be a meteor — but a slow, steady, or hovering light is not.`
          : `<b>Assessment — meteor: unlikely.</b> Showers are active on the date, but their radiants are below the horizon here — few meteors.`;
        meteorHtml = `<h2>Meteor-shower check</h2><p class="lead">${meteorAssess}</p>
<p class="cap">Annual showers active on the sighting date. Meteors radiate OUTWARD from the radiant, so a fast streak pointing back to an above-horizon radiant is likely a shower meteor; the Taurids produce slow, bright fireballs. "Off sight-line" is the radiant's angle from the marked object.</p>
<table><tr><th>Shower</th><th>Radiant now</th><th>Off sight-line</th><th>Rate · speed</th></tr>${rows}</table>`;
      }
    }
  }
  /* nearby airfields — approach/departure corridors concentrate the low, slow
     traffic that gets reported; context for the aircraft explanation. */
  let airportsHtml = "";
  {
    const w0 = origAct.find((s) => isNum(s.lat) && isNum(s.lon));
    const rla = fix.ok ? fix.ref.lat : (w0 ? +w0.lat : null);
    const rlo = fix.ok ? fix.ref.lon : (w0 ? +w0.lon : null);
    if (rla != null && rlo != null) {
      try {
        const aps = (await fetchAirports(rla, rlo, 45000)).slice(0, 4);
        if (aps.length) {
          const rows = aps.map((a) => `<tr><td>${e2(a.name)}${a.iata || a.icao ? ` (${e2(a.iata || a.icao)})` : ""}${a.kind ? ` · <span class="cap">${e2(a.kind)}</span>` : ""}</td><td>${fmtLenShort(a.distM)} ${compass8(a.bearing)} (${Math.round(a.bearing)}°)</td></tr>`).join("");
          const near = aps[0];
          const apAssess = near.distM < 8000
            ? `<b>Assessment — traffic context: high.</b> ${e2(near.name)} is only ${fmtLenShort(near.distM)} ${compass8(near.bearing)} — its approach/departure corridors put low, slow aircraft nearby, so weigh the aircraft explanation heavily.`
            : near.distM < 25000
              ? `<b>Assessment — traffic context: moderate.</b> The nearest field (${e2(near.name)}) is ${fmtLenShort(near.distM)} away — aircraft in transit are plausible.`
              : `<b>Assessment — traffic context: low.</b> The nearest field is ${fmtLenShort(near.distM)} away — concentrated approach traffic is less likely here.`;
          airportsHtml = `<h2>Nearby airfields</h2><p class="lead">${apAssess}</p>
<p class="cap">Aerodromes near ${fix.ok ? "the object's ground position" : "the observer"} (OpenStreetMap). Approach and departure corridors concentrate low, slow, light-carrying traffic — worth weighing alongside the ADS-B check.</p>
<table><tr><th>Airfield</th><th>Distance · bearing</th></tr>${rows}</table>`;
        }
      } catch (e) { /* overpass busy / offline — omit */ }
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
.cap{color:#666;font-size:12px}
.lead{background:#f4f6fb;border-left:3px solid #2563c9;border-radius:0 6px 6px 0;padding:8px 12px;margin:2px 0 12px;font-size:13px}
blockquote.stmt{margin:4px 0 0;padding:8px 12px;background:#faf7f0;border-left:3px solid #C77B14;border-radius:0 6px 6px 0;font-size:13px;line-height:1.55;white-space:pre-wrap}
details.sec{border-top:1px solid #e4e4e4;margin-top:14px}
details.sec>summary{font:700 12px ui-monospace,monospace;letter-spacing:.16em;text-transform:uppercase;color:#555;padding:12px 0;cursor:pointer;list-style:none;display:flex;align-items:center;gap:9px}
details.sec>summary::-webkit-details-marker{display:none}
details.sec>summary::before{content:"\\25B8";color:#C77B14;font-size:12px}
details.sec[open]>summary::before{content:"\\25BE"}
details.sec>*:last-child{margin-bottom:14px}
@media print{.noprint{display:none}details.sec{border-top:none;margin-top:18px}details.sec>summary{cursor:auto}details.sec>summary::before{content:""}}
@media(max-width:640px){body{margin:16px auto;padding:0 12px}table{table-layout:fixed;font-size:12px}}
</style></head><body>
<div style="display:flex;align-items:center;gap:14px;flex-wrap:wrap;margin-bottom:2px"><img src="data:image/svg+xml,${encodeURIComponent(phodarLogoRaw)}" alt="PHODAR" style="height:48px;width:auto;border-radius:8px;display:block"/><span style="font:700 13px ui-monospace,Menlo,monospace;letter-spacing:.16em;color:#555">SIGHTING REPORT</span></div>
<div class="cap">Generated ${new Date().toLocaleString()} · photogrammetric detection &amp; ranging · phodar v1</div>
<h2>Observers (${packed.length})</h2>
<table><tr><th>Name</th><th>Position</th><th>Time</th><th>Bearing az/el</th><th>FOV</th><th>Traj pts</th></tr>${obsRows}</table>
<h2>Result</h2>${fixHtml}
${(() => { const ws = origAct.filter((s) => s.statement && String(s.statement).trim()); return ws.length ? `<h2>Witness accounts</h2>` + ws.map((s, i) => `<div style="margin:0 0 12px"><b>${e2(s.name || "Observer " + (i + 1))}</b>${s.whenMs ? ` <span class="cap">· ${new Date(+s.whenMs).toLocaleString()}</span>` : ""}<blockquote class="stmt">${e2(String(s.statement).trim())}</blockquote></div>`).join("") : ""; })()}
${dimsHtml}
${kin ? `<h2>Trajectory kinematics (stereo)</h2>${kin}` : soloKin}
${collapsible(vstereoHtml, true)}
${collapsible(mixedHtml, true)}
${collapsible(videoHtml, false)}
${collapsible(alignHtml, true)}
${collapsible(adsbHtml, false)}
${collapsible(airportsHtml, false)}
${collapsible(photomHtml, false)}
${collapsible(condHtml, false)}
${collapsible(skyHtml, false)}
${collapsible(windHtml, false)}
${collapsible(launchHtml, false)}
${collapsible(fireballHtml, false)}
${collapsible(meteorHtml, false)}
${exhibits}
${collapsible(`<h2>Method</h2><p>Each photo is pixel-normalized and its lens field of view read from EXIF. The object's sky direction is fixed by aligning the photo on an astronomically anchored alt-azimuth grid (Sun/Moon computed for the reported time and place). With two or more observers, sight-lines are intersected by least squares in a local ENU frame; ray convergence and rms miss distance grade the fix. Object size = measured angular size × range. Trajectories interpolate each witness's directions to common instants before triangulating each instant; speeds, accelerations and felt g-loads follow by finite differences with 3-point smoothing.</p>`, false)}
${collapsible(`<h2>Caveats</h2><p>${fix.ok ? `Quality <b>${fix.rating}</b>: baseline ${fmtLenShort(fix.baseline)}, convergence ${fix.conv.toFixed(1)}°, rms ray miss ${fmtLenShort(fix.solA.rmsMiss)}; a ±1° bearing error implies ≈ ${fmtLenShort(fix.posErr)} of position uncertainty.` : `Single-perspective data — directions and angular sizes are honest; absolute range, size and speed require a second viewpoint.`} Compass bearings may be magnetic rather than true; EXIF times are device-local.</p>`, false)}
${diagHtml}
<p class="cap"> ${opts.exhibits === "full" || opts.exhibits === "files" ? "Exhibit photos are full resolution; the embedded share data carries 1600 px working copies." : "Bundled photos are 1600 px working copies; analysis used the originals."}</p>
<p class="cap">Cross-check data: aircraft from the ADS-B community networks (airplanes.live, adsb.lol, adsb.fi, OpenSky); satellites from CelesTrak; terrain from Terrarium/AWS Open Data; peaks, buildings, aerodromes and street maps © OpenStreetMap contributors (ODbL); satellite imagery © Esri, Maxar, Earthstar Geographics; winds from Open-Meteo; magnetic declination from NOAA WMM2025. Every one of them is a free public source — see docs/DATA-SOURCES.md in the Phodar repository.</p>
<p class="noprint"><b>Add your perspective:</b> open Phodar → Import and choose this very file — the sighting data and photos are embedded below.</p>
<script type="application/json" id="phodar-data">${data}</script>
<script>(function(){function set(open){document.querySelectorAll('details.sec').forEach(function(d){if(open){if(!d.open){d.dataset.was='0';d.open=true}}else if(d.dataset.was==='0'){d.open=false;delete d.dataset.was}})}window.addEventListener('beforeprint',function(){set(true)});window.addEventListener('afterprint',function(){set(false)});if(window.matchMedia){var mq=window.matchMedia('print');if(mq.addListener)mq.addListener(function(e){set(e.matches)})}})();</script>
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

function WizStep({ n, title, children, onBack, onNext, nextLabel, nextDisabled, disabledLabel, help }) {
  return (
    <div style={{ padding: "14px 12px 96px" }}>
      <div style={{ display: "flex", alignItems: "center", gap: 10, marginBottom: 10 }}>
        <button className="btn sm" onClick={onBack}>‹</button>
        <div>
          <div style={{ fontFamily: "var(--mono)", fontWeight: 800, letterSpacing: ".12em", fontSize: 14 }}>{title}</div>
          <WizDots n={n} style={{ marginTop: 4 }} />
        </div>
        {help && <HelpButton section={help} style={{ marginLeft: "auto" }} />}
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

/* capture-time editor for a moment — auto-filled from EXIF when present, but
   always adjustable (shared/re-encoded photos often lose their timestamp, and
   the inter-moment gap is what turns the angular path into a real speed). */
function MomentTimeCtl({ m, onChange }) {
  const toLocal = (ms) => {
    const d = new Date(isNum(ms) ? +ms : Date.now());
    const p = (n) => String(n).padStart(2, "0");
    return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())}T${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
  };
  const fromExif = isNum(m.meta?.timeMs);
  return (
    <div className="card" style={{ margin: "10px 0 0" }}>
      <ML>When was this moment taken?</ML>
      <div style={{ fontSize: 11, color: "var(--dim)", margin: "2px 0 6px", lineHeight: 1.4 }}>
        {fromExif
          ? "✓ read from the photo — adjust only if it's wrong."
          : "This photo carried no timestamp. Set it as exactly as you can — the seconds between moments set the object's speed."}
      </div>
      <input type="datetime-local" step="1" value={toLocal(m.whenMs)}
        onChange={(e) => { const t = e.target.value ? Date.parse(e.target.value) : NaN; if (isNum(t)) onChange({ whenMs: t, tSource: "manual" }); }}
        style={{ width: "100%" }} />
    </div>
  );
}

function WizHome({ sources, est, onNew, onAddWitness, onResume, onRemove, onImport, onReport, onAddMoment, onOpenMoment, onRemoveMoment, unitsImp, onToggleUnits, appMode, onSetMode }) {
  const fileRef = useRef(null);
  const [impMsg, setImpMsg] = useState("");
  const real = sources.filter((s) => !isEmptySource(s));
  const fix = analyze(sources);
  const dot = (on, k, title) => <span key={k} title={title} style={{ display: "inline-block", width: 7, height: 7, borderRadius: 4, background: on ? "var(--teal)" : "var(--line)", marginRight: 4 }} />;
  return (
    <div style={{ padding: "26px 14px 40px", position: "relative" }}>
      <HelpButton section="start" style={{ position: "absolute", top: "calc(10px + env(safe-area-inset-top))", right: 14, zIndex: 30 }} />
      {/* the shim fell back to memory (private browsing / site data blocked):
          everything still works, but nothing survives a reload — say so once,
          here, rather than letting someone lose an hour of marking to it */}
      {typeof window !== "undefined" && window.storageVolatile && (
        <div className="warn" style={{ margin: "0 0 10px", fontSize: 11.5 }}>
          ⚠ This browser is blocking site storage (private browsing, or site data turned off). Phodar works, but <b>nothing will survive a reload</b> — export the report or share file before you close the tab.
        </div>
      )}
      <div style={{ textAlign: "center", marginTop: 16 }}>
        <img src={phodarLogo} alt="PHODAR" style={{ display: "block", width: "min(460px, 94%)", margin: "0 auto", borderRadius: 12 }} />
        <div className="microlabel" style={{ marginTop: 6 }}>Photogrammetric detection &amp; ranging</div>
        <div style={{ color: "var(--dim)", fontSize: 12, marginTop: 10, lineHeight: 1.5 }}>
          {appMode === "aerial"
            ? <>Geolocate a target from a <b style={{ color: "var(--teal)" }}>downward-looking</b> sensor — a plane, drone, or mast of known position. One frame is enough: the ground is a known surface.</>
            : <>Turn a sighting photo into real numbers — direction, size, altitude, speed. Two witnesses make it true triangulation.</>}
        </div>
        {/* SKY vs AERIAL — the same measurement pipeline pointed up (triangulate an
            unknown-range object from ≥2 observers) or down (geolocate a target off
            one platform of known position). Hidden behind AERIAL_ENABLED while the
            sky app is refined; the aerial code stays intact for when it's flipped. */}
        {AERIAL_ENABLED && (
          <div style={{ display: "inline-flex", marginTop: 12, border: "1px solid var(--line)", borderRadius: 9, overflow: "hidden" }}>
            {[["sky", "🔭 Sky (looking up)"], ["aerial", "🛰 Aerial (looking down)"]].map(([m, lbl]) => (
              <button key={m} onClick={() => onSetMode(m)}
                style={{
                  border: "none", padding: "8px 14px", fontSize: 12.5, cursor: "pointer",
                  background: appMode === m ? "var(--teal)" : "transparent",
                  color: appMode === m ? "var(--bg)" : "var(--dim)",
                  fontWeight: appMode === m ? 700 : 400,
                }}>{lbl}</button>
            ))}
          </div>
        )}
        <div>
          <button className="chip" style={{ marginTop: 10 }} onClick={onToggleUnits}>
            units: <b style={{ color: "var(--amber)" }}>{unitsImp ? "ft · mi · mph" : "m · km · m/s"}</b> — tap to switch
          </button>
        </div>
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
          {sources.map((s, i) => {
            const moments = s.moments || [];
            /* trajectory exists when ≥2 placed shots (primary + moments) OR a
               drawn manual track — either way the observer has a path */
            const placedShots = [s, ...moments].filter((m) => isNum(m.A?.az) && isNum(m.A?.el) && isNum(m.whenMs)).length;
            const hasTraj = placedShots >= 2 || (s.track || []).length > 1;
            const timeLbl = (ms) => (isNum(ms) ? new Date(+ms).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit", second: "2-digit" }) : "no time");
            return (
              <div key={s.id} style={{ padding: "7px 0", borderBottom: "1px solid var(--line)" }}>
                <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
                  <div style={{ flex: 1, fontSize: 13 }}>
                    {s.name || `Observer ${i + 1}`}
                    <div style={{ marginTop: 3 }}>
                      {/* photo · position · direction are the completable facets;
                         a trajectory dot only appears (and is always green) when a
                         path exists — it's optional, so it never blocks "complete" */}
                      {dot(!!s.mediaUrl, "m", "photo")}{dot(isNum(s.lat) && isNum(s.lon), "p", "position")}{dot(isNum(s.A?.az) && isNum(s.A?.el), "d", "direction")}{hasTraj ? dot(true, "t", "trajectory") : null}
                    </div>
                  </div>
                  <button className="btn sm" onClick={() => onResume(s.id)}>Open ▸</button>
                  <button className="btn sm ghost" style={{ color: "var(--red)", padding: "6px 8px" }}
                    onClick={() => {
                      if (window.confirm(`Remove ${s.name || `Observer ${i + 1}`} from this sighting? Their photo and measurements go with them.`)) onRemove(s.id);
                    }}>✕</button>
                </div>
                {/* moment tree: the primary photo is the first moment; each extra
                   photo of the same object (at another time) adds a trajectory point */}
                {(!!s.mediaUrl || moments.length > 0) && (
                  <div style={{ marginTop: 6, marginLeft: 4, paddingLeft: 8, borderLeft: "2px solid var(--line)" }}>
                    <div style={{ display: "flex", alignItems: "center", gap: 6, fontSize: 11.5, color: "var(--dim)", padding: "2px 0" }}>
                      <span style={{ color: "var(--teal)" }}>◷</span>
                      <span style={{ flex: 1 }}>📷 Moment 1 · {timeLbl(s.whenMs)}{isNum(s.A?.az) ? "" : " · not placed"}</span>
                    </div>
                    {moments.map((m, mi) => (
                      <div key={m.id} style={{ display: "flex", alignItems: "center", gap: 6, fontSize: 11.5, color: "var(--dim)", padding: "2px 0" }}>
                        <span style={{ color: isNum(m.A?.az) ? "var(--teal)" : "var(--line)" }}>◷</span>
                        <span style={{ flex: 1 }}>📷 Moment {mi + 2} · {timeLbl(m.whenMs)}{isNum(m.A?.az) ? "" : " · needs placing"}</span>
                        <button className="btn sm ghost" style={{ padding: "3px 7px", fontSize: 11 }} onClick={() => onOpenMoment(s.id, m.id)}>{m.mediaUrl ? "Edit" : "Add photo"}</button>
                        <button className="btn sm ghost" style={{ color: "var(--red)", padding: "3px 6px", fontSize: 11 }}
                          onClick={() => { if (window.confirm(`Remove Moment ${mi + 2} from ${s.name || `Observer ${i + 1}`}?`)) onRemoveMoment(s.id, m.id); }}>✕</button>
                      </div>
                    ))}
                    {!!s.mediaUrl && (
                      <button className="btn sm ghost" style={{ marginTop: 4, fontSize: 11.5, padding: "4px 8px" }} onClick={() => onAddMoment(s.id)}>＋ Add moment</button>
                    )}
                    {placedShots >= 2 && (
                      <div style={{ fontSize: 10.5, color: "var(--track)", marginTop: 4 }}>↳ trajectory from {placedShots} placed photos</div>
                    )}
                  </div>
                )}
              </div>
            );
          })}
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
        <HelpButton section="results" style={{ marginLeft: "auto" }} />
      </div>
      <div className="card" style={{ margin: 0 }}>
        {fix.ok ? (
          <>
            <ML>Triangulated fix — {fix.obs.length} observers</ML>
            <div style={{ fontFamily: "var(--mono)", fontSize: 13, lineHeight: 1.9 }}>
              altitude <b style={{ color: "var(--teal)" }}>{fmtLenShort(fix.solA.X[2])}</b> ({fmtLenAlt(fix.solA.X[2])})<br />
              {fix.sizeAvg != null && <>size <b style={{ color: "var(--teal)" }}>{fmtLenShort(fix.sizeAvg)}</b> ({fmtLenAlt(fix.sizeAvg)})<br /></>}
              {fix.motion?.speed != null && <>speed <b style={{ color: "var(--teal)" }}>{fmtSpeedShort(fix.motion.speed)}</b><br /></>}
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
  /* the report is ALWAYS regenerated from the current sources by reportHtml, so
     importing an older sighting produces a fresh report with the current
     checks. Safeguard: if the sources change (e.g. an import) while a preview is
     cached, drop it so the next view rebuilds. */
  useEffect(() => { setPrevHtml(null); setManual(null); }, [sources]);
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
  /* the full bundle as a single .zip: the report, the importable data, the
     FULL-res photos, and each observer's VIDEOS — the original clip plus the
     world-locked stabilized render when one has been exported (the
     before/after pair). A download, importable back into Phodar. */
  const downloadBundle = async () => {
    setMsg("packing bundle…");
    wakeHold(); // keyframe baking + video packing runs long — don't let the phone doze
    try {
    const html = await reportHtml(sources, est, { exhibits: "files" });
    const json = await buildShareJson(sources, est);
    const act = sources.filter((s) => !isEmptySource(s));
    const files = [
      { name: strU8("report.html"), data: strU8(html) },
      { name: strU8("sighting.phodar.json"), data: strU8(json) },
    ];
    const extOf = (t) => /quicktime/.test(t) ? "mov" : /webm/.test(t) ? "webm" : "mp4";
    let vidN = 0;
    for (let i = 0; i < act.length; i++) {
      const s = act[i];
      if (s.mediaUrl && s.mediaKind === "image") {
        try { files.push({ name: strU8(`photos/observer-${i + 1}.jpg`), data: dataUrlU8(s.mediaUrl) }); } catch (e) { }
      }
      if (s.mediaKind === "video") {
        try {
          const rec = await mediaGet(s.id);
          let vb = rec && rec.kind === "video" && rec.data ? rec.data : null;
          if (!vb && s.mediaUrl) vb = await fetch(s.mediaUrl).then((r) => r.blob()).catch(() => null);
          if (vb) { files.push({ name: strU8(`videos/observer-${i + 1}-original.${extOf(vb.type || "")}`), data: new Uint8Array(await vb.arrayBuffer()) }); vidN++; }
          const st = await mediaGet(s.id + ":stab");
          if (st && st.data) { files.push({ name: strU8(`videos/observer-${i + 1}-stabilized.${extOf(st.data.type || "")}`), data: new Uint8Array(await st.data.arrayBuffer()) }); vidN++; }
        } catch (e) { }
      }
    }
    const blob = makeZip(files);
    if (download("phodar-sighting.zip", blob, "application/zip"))
      setMsg(`✓ downloading bundle — ${(blob.size / 1048576).toFixed(1)} MB · report + data + full-res photos${vidN ? ` + ${vidN} video${vidN > 1 ? "s" : ""}` : ""}`);
    else setMsg("Bundle download needs the deployed app — this preview can't save binaries.");
    } finally { wakeRelease(); }
  };
  /* share the viewable report page itself via the OS share sheet (text,
     email, AirDrop…); falls back to a download/copy where share is unsupported */
  const shareReportHtml = async () => {
    let html = prevHtml;
    if (!html) { wakeHold(); try { html = await reportHtml(sources, est, { exhibits: "full" }); } finally { wakeRelease(); } }
    try {
      const file = new File([html], "phodar-report.html", { type: "text/html" });
      if (navigator.canShare && navigator.canShare({ files: [file] })) {
        await navigator.share({ files: [file], title: "PHODAR sighting report" });
        return;
      }
    } catch (e) { if (e && e.name === "AbortError") return; }
    deliver("phodar-report.html", html, "text/html");
  };
  const openReport = async () => { setMsg("packing…"); wakeHold(); try { setPrevHtml(await reportHtml(sources, est, { exhibits: "full" })); } finally { wakeRelease(); } setMsg(""); };
  return (
    <div style={{ padding: "14px 12px 40px" }}>
      <div style={{ display: "flex", alignItems: "center", gap: 10, marginBottom: 10 }}>
        <button className="btn sm" onClick={onBack}>‹</button>
        <div style={{ fontFamily: "var(--mono)", fontWeight: 800, letterSpacing: ".12em", fontSize: 14 }}>REPORT &amp; SHARE</div>
        <HelpButton section="report" style={{ marginLeft: "auto" }} />
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
          <button className="btn" style={{ padding: 12 }} onClick={downloadBundle}>⬇ Download bundle (.zip — report + photos + videos + data)</button>
        </div>
        {msg && <div style={{ fontSize: 12, color: "var(--teal)", marginTop: 8 }}>{msg}</div>}
        <div style={{ fontSize: 11, color: "var(--dim)", marginTop: 8 }}>
          The bundle re-imports into Phodar. Open the report to read it and share the page itself (text / email) from the top.
        </div>
      </div>

      {/* A fixed inset:0 overlay in the INSTALLED PWA starts at the physical
          top of the screen — the status bar is translucent — so a plain padding
          put this header under the notch and its buttons out of reach (field
          report). Every other full-screen overlay already pads by the safe
          area; this one and the copy box below were the misses. Bottom inset
          too, or the report's last lines sit under the home indicator. */}
      {prevHtml && (
        <div style={{ position: "fixed", inset: 0, zIndex: 300, background: "#0009", display: "flex", flexDirection: "column", paddingBottom: "env(safe-area-inset-bottom)" }}>
          <div style={{ display: "flex", gap: 6, padding: "calc(10px + env(safe-area-inset-top)) calc(12px + env(safe-area-inset-right)) 10px calc(12px + env(safe-area-inset-left))", background: "var(--bg)", alignItems: "center", flexWrap: "wrap" }}>
            <span style={{ fontFamily: "var(--mono)", fontSize: 12, fontWeight: 800, flex: 1, minWidth: 60 }}>REPORT</span>
            <button className="btn sm amber" onClick={shareReportHtml}>📤 Share/Download page</button>
            <button className="btn sm" onClick={() => setPrevHtml(null)}>✕ Close</button>
          </div>
          {msg && <div style={{ padding: "0 12px 8px", background: "var(--bg)", fontSize: 11, color: "var(--teal)" }}>{msg}</div>}
          <iframe title="report" srcDoc={prevHtml} style={{ flex: 1, border: 0, background: "#fff" }} />
        </div>
      )}

      {manual && (
        <div style={{ position: "fixed", inset: 0, zIndex: 300, background: "#000c", display: "flex", flexDirection: "column", padding: "calc(12px + env(safe-area-inset-top)) calc(12px + env(safe-area-inset-right)) calc(12px + env(safe-area-inset-bottom)) calc(12px + env(safe-area-inset-left))" }}>
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
  /* SKY (looking up) vs AERIAL (looking down) mode. Sky triangulates an unknown-
     range object from ≥2 ground observers; aerial geolocates a target off a
     downward sensor of known position/altitude (single frame is enough — the
     ground is a known surface). Persisted like units so a reload keeps the mode.
     The whole app branches on this: measure/place UI, results, and reports. */
  const [appModeRaw, setAppMode] = useState(() => {
    try { return localStorage.getItem("phodar-mode") === "aerial" ? "aerial" : "sky"; } catch (e) { return "sky"; }
  });
  /* aerial is behind AERIAL_ENABLED — while it's off, force sky everywhere so a
     persisted "aerial" (from earlier testing) can never route into hidden UI. */
  const appMode = AERIAL_ENABLED ? appModeRaw : "sky";
  const setMode = (m) => {
    setAppMode(m);
    try { localStorage.setItem("phodar-mode", m); } catch (e) { }
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
          /* re-attach media from IndexedDB (autosave strips mediaUrl) — for the
             observer's primary photo AND each of its moments (keyed by id) */
          const ids = d.sources.flatMap((s) => [s.id, ...(s.moments || []).map((m) => m.id)]);
          Promise.all(ids.map(async (id) => ({ id, rec: await mediaGet(id) }))).then((rs) => {
            const recFor = (id) => rs.find((r) => r.id === id)?.rec;
            const attach = (o) => {
              const hit = recFor(o.id);
              if (!hit || o.mediaUrl) return o;
              const url = hit.kind === "video" ? URL.createObjectURL(hit.data) : hit.data;
              return { ...o, mediaUrl: url, mediaKind: hit.kind, mediaNorm: hit.kind === "image" };
            };
            setSources((ss) => ss.map((s) => {
              const s2 = attach(s);
              return (s2.moments || []).length ? { ...s2, moments: s2.moments.map(attach) } : s2;
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
      try {
        /* strip heavy media URLs — the observer's own AND each moment's (data
           URLs are MBs; localStorage caps ~5 MB). The pixels live in IndexedDB
           and are re-attached on boot; only measurements/points persist here. */
        const stripMedia = ({ mediaUrl, mediaKind, mediaNorm, ...rest }) => rest;
        const lean = sources.map((s) => {
          const s2 = stripMedia(s);
          return (s2.moments || []).length ? { ...s2, moments: s2.moments.map(stripMedia) } : s2;
        });
        window.storage.set("phodar-v1", JSON.stringify({ sources: lean, est }));
      } catch (e) { }
    }, 800);
    return () => clearTimeout(id);
  }, [sources, est]);

  const updateSource = (id, patch) =>
    setSources((ss) => ss.map((s) => (s.id === id ? { ...s, ...patch } : s)));
  const removeSource = (id) => {
    const s = sources.find((x) => x.id === id);
    mediaDel(id); mediaDel(id + ":stab");
    (s?.moments || []).forEach((m) => mediaDel(m.id));
    setSources((ss) => ss.filter((s) => s.id !== id));
  };
  /* ——— moments: additional timestamped photos under ONE observer ——— */
  const updateMoment = (srcId, momId, patch) =>
    setSources((ss) => ss.map((s) => (s.id !== srcId ? s
      : { ...s, moments: (s.moments || []).map((m) => (m.id === momId ? { ...m, ...patch } : m)) })));
  const addMoment = (srcId) => {
    const parent = sources.find((s) => s.id === srcId);
    if (!parent) return;
    const m = makeMoment(parent.fovH);
    setSources((ss) => ss.map((s) => (s.id === srcId ? { ...s, moments: [...(s.moments || []), m] } : s)));
    setUi({ view: "m1", srcId, momId: m.id });
  };
  const removeMoment = (srcId, momId) => {
    mediaDel(momId);
    setSources((ss) => ss.map((s) => (s.id !== srcId ? s
      : { ...s, moments: (s.moments || []).filter((m) => m.id !== momId) })));
  };
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
    wakeHold(); // media packing (thumbnails/crops) can run long on big sightings
    let j; try { j = await buildShareJson(sources, est); } finally { wakeRelease(); }
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
    const wmom = wsrc && ui.momId ? (wsrc.moments || []).find((m) => m.id === ui.momId) : null;
    const momIdx = wmom ? (wsrc.moments || []).findIndex((m) => m.id === wmom.id) : -1;
    let page = null;
    if (ui.view === "report") {
      page = <ReportView sources={sources} est={est} onBack={() => goView("home")} />;
    } else if (ui.view === "s4") {
      page = <WizFinish sources={sources} est={est} onAdd={addWitness} onReport={() => goView("report")} onShare={shareJsonNow} onHome={() => goView("home")} onFixAlt={(id, alt) => updateSource(id, { alt })} />;
    } else if (ui.view !== "home" && wsrc) {
      if (ui.view === "s1") {
        page = (
          <WizStep n={1} title={appMode === "aerial" ? "THE AERIAL FRAME" : "THE PHOTO"} help="photo" onBack={() => goView("home")} onNext={() => goView("s2")}
            nextLabel={appMode === "aerial" ? "Next · platform & geolocate →" : (wsrc.mediaUrl ? "Next · where were you? →" : "Skip media — enter data by hand →")}>
            <MediaMeasure wizard src={wsrc} update={(p) => updateSource(wsrc.id, p)} />
          </WizStep>
        );
      } else if (ui.view === "s2" && appMode === "aerial") {
        /* AERIAL (looking down): platform pose + single-frame ground geolocation.
           Results are inline; no sky triangulation step, so Next returns home. */
        page = (
          <WizStep n={2} title="PLATFORM & TARGET" help="photo" onBack={() => goView("s1")} onNext={() => goView("home")}
            nextLabel="Done · back to sightings →">
            <AerialMeasure src={wsrc} update={(p) => updateSource(wsrc.id, p)} unitsImp={unitsImp} />
          </WizStep>
        );
      } else if (ui.view === "s2") {
        page = (
          <WizStep n={2} title="YOUR POSITION" help="position" onBack={() => goView("s1")} onNext={() => goView("s3")}
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
      } else if (ui.view === "m1" && wmom) {
        page = (
          <WizStep n={2} title={`MOMENT ${momIdx + 2} · PHOTO`} help="photo"
            onBack={() => { if (!wmom.mediaUrl) removeMoment(wsrc.id, wmom.id); goView("home"); }} onNext={() => goView("m2")}
            nextDisabled={!wmom.mediaUrl} nextLabel="Next · place it in the sky →"
            disabledLabel="Add this moment's photo to continue">
            <div style={{ fontSize: 12, color: "var(--dim)", padding: "0 2px 8px", lineHeight: 1.5 }}>
              Another photo of the <b style={{ color: "var(--ink)" }}>same object</b> from <b style={{ color: "var(--ink)" }}>{wsrc.name}</b>'s spot, at a different time. Mark where the object sits — its direction plus this moment's time add a point to the trajectory.
            </div>
            <MediaMeasure wizard src={wmom} update={(p) => updateMoment(wsrc.id, wmom.id, p)} />
            {wmom.mediaUrl && <MomentTimeCtl m={wmom} onChange={(p) => updateMoment(wsrc.id, wmom.id, p)} />}
          </WizStep>
        );
      } else if (ui.view === "m2" && wmom) {
        page = (
          <SkyAimer open wizard single source={wmom} update={(p) => updateMoment(wsrc.id, wmom.id, p)}
            onClose={() => goView("home")} onWizardBack={() => goView("m1")} onWizardNext={() => goView("home")}
            lat={isNum(wsrc.lat) ? +wsrc.lat : 42.16} lng={isNum(wsrc.lon) ? +wsrc.lon : -123.66}
            whenMs={isNum(wmom.whenMs) ? +wmom.whenMs : Date.now()}
            initAz={isNum(wmom.A?.az) ? +wmom.A.az : (wmom.mediaAim && isNum(wmom.mediaAim.az) ? +wmom.mediaAim.az : (isNum(wsrc.A?.az) ? +wsrc.A.az : 180))}
            initAlt={isNum(wmom.A?.el) ? +wmom.A.el : (wmom.mediaAim && isNum(wmom.mediaAim.el) ? +wmom.mediaAim.el : (isNum(wsrc.A?.el) ? +wsrc.A.el : 20))}
            marks={[]} which="A"
            onCapture={(wh, az, el) => updateMoment(wsrc.id, wmom.id, { A: { ...wmom.A, az: az.toFixed(2), el: el.toFixed(2) } })} />
        );
      }
    }
    if (!page) page = <WizHome sources={sources} est={est} onNew={newSighting} onAddWitness={addWitness} onResume={(id) => setUi({ view: "s1", srcId: id })} onRemove={removeSource} onImport={importShared} onReport={() => goView("report")}
      onAddMoment={addMoment} onOpenMoment={(sid, mid) => setUi({ view: "m1", srcId: sid, momId: mid })} onRemoveMoment={removeMoment}
      unitsImp={unitsImp} onToggleUnits={toggleUnits} appMode={appMode} onSetMode={setMode} />;
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
