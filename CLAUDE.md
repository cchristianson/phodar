# CLAUDE.md — Phodar

PHOtogrammetric Detection And Ranging: a mobile-first React app that turns
UFO/UAP sighting photos into triangulated **position, true size, altitude,
speed and heading** — one observer gives honest angular data; two or more give
a real 3D fix. Built and field-validated against ground truth (see
`docs/FIELD-TESTS.md`: a rooftop weathervane resolved to within 1 inch of its
true span and inside the owner's height estimate).

## Commands
- `npm run dev` — Vite dev server (mobile testing: open the LAN URL on a phone)
- `npm run build` / `npm run preview`
- `npm test` — runs `scripts/mathcheck.js` + `scripts/trajcheck.js`.
  **These must pass after ANY change to the math core.** As of the module
  split they **import the real `src/math/*` modules** (they used to re-implement
  the math, so a bug in the shipped code couldn't fail them). They assert exact
  triangulation recovery, ENU round-trips, angular-size → span, the
  elevation-convergence fix, and full trajectory-kinematics recovery of a
  simulated 3.5 g maneuver by driving `analyzeTracks` directly.

## Layout
The **math core is now extracted** into `src/math/` (pure ES modules, no React,
imported by both `phodar.jsx` and the test scripts):
- `geodesy.js` — constants (`D2R/R2D/RE/RAD`), vector ops, `enuFromGeo`/
  `geoFromEnu`, `dirFromAzEl`/`dirToAzEl`, `clampN`.
- `format.js` — `isNum`, `fmtLen`/`fmtSpeed`/`fmtDeg`, `compass8`.
- `projection.js` — `focalPx`, `photoBasis`, `angSizeFromPoints`,
  `pixelDirFromAnchor`, `homography`/`solveN`, `matrix3dFromH`. The pixel/pose
  fns take a `pose` sample (not a per-source constant) — a video provision.
- `triangulate.js` — `solve3`, `intersectLines`, `analyze`,
  `arbitrateBearings`, `aspectSpan`.
- `kinematics.js` — `trackDirections`, `kinematics`, `analyzeTracks`.
- `astro.js` — `sunPos`/`moonPos`/`moonFrac` (SunCalc-derived).

Everything else still lives in `src/phodar.jsx` (~3,750 lines) **on purpose** —
it was developed as a single Claude artifact. It is organized by banner
comments, in this order:

1. **EXIF / QuickTime parser** — hand-rolled TIFF walk: GPS, time, bearing
   (`GPSImgDirection` + true/mag ref), 35 mm focal → FOV, orientation.
2. **3D shape system** — `SHAPES`, `shapeWire` (orb/saucer/capsule/tri/plane/
   bird wireframes), rotation mats, `shapeProjNat` (orthographic project +
   silhouette extremes → auto-writes `A.p1/p2`).
3. **Components** — `MediaMeasure` (upload → canvas normalize → shape fitting,
   pinch-zoom, loupe, auto-horizon), `PositionEditor` + `PinMap`,
   `SkyAimer` (Place/Look modes, canvas mesh warp, wizard trajectory + Δt
   chips, compare ghosts), `PlotBoard`, charts, `ResultsPanel`, Solo/Guide.
4. **Wizard + reports** — `WizHome/WizStep/WizFinish/ReportView`,
   `packSources`, `reportHtml` (self-contained HTML w/ embedded data + photo
   exhibits + detail crops), `buildShareJson`, zip writer, import.
5. **App shell** — wizard (default) / Lab (power users) branch.

## Non-negotiable invariants (each one was a multi-hour bug hunt)
1. **Never render user images through CSS `matrix3d`.** iOS Safari composes
   its hidden EXIF-orientation transform with author matrices unpredictably.
   All images are **canvas-normalized once at load** (≤4300 px — iOS canvas
   area ceiling — EXIF baked out, `mediaNorm` flag), and the sky-view photo
   warp is a hand-rolled canvas **triangle mesh through `projectD`**.
2. **Projection aspect is tangent-scaled:** `tanV = tanH · (h/w)`. Degree
   scaling is wrong and was the original geometry bug.
3. **`pixelDirFromAnchor` handles azimuth convergence** (1/cos el). Never
   revert to `dAz = atan(dx/f)` — 41 % error at 45° elevation.
4. **Place mode renders the photo as ONE fused element** (image + marks +
   frame in a single DOM box, `rotate(−roll)`). Proven exactly equal to the
   projective quad on-axis; splitting layers reintroduces divergence.
5. **Touch stack:** deferred gesture commit (7 px slop / 260 ms hold / tap on
   up), second finger always cancels into pinch, native `touchstart`
   preventDefault on marking canvases (iOS ignores `touch-action` for
   multi-touch), document-level lock while touching, blur/visibilitychange
   hard-reset. `pointercancel` must never place a point.
6. **Loupe keeps its devicePixelRatio backing store** or it renders soft.
7. **Drags are rAF-coalesced** (`queuePose`) — 120 Hz phones flood React
   otherwise. Warp texture is pre-downsampled to 1280 px, mesh 7 columns,
   warp DPR capped at 2.
8. **`window.storage`** is the persistence API (artifact heritage);
   `src/storageShim.js` maps it to localStorage. Keep the contract.

## Field findings baked into the UX (don't "simplify" these away)
- Phone compass: **sub-degree on foot**, **14–66° wrong** near metal (inside a
  car; under a steel-roofed pergola). Hence `arbitrateBearings` (size-ratio
  compass arbitration), per-ray-miss diagnostics in reports, and warnings.
- Phone GPS **altitude** wobbles ±5 m → short-baseline alt-spread warning;
  the fix is "set observer elevations equal on level ground".
- Mountain/tree skylines are not a true horizon → auto-level warns when
  |pitch| > 6°.
- FOV pinch during sky placement overrides lens truth — the readout shows it;
  Reset restores lens FOV.

## Priority backlog (rough order)
1. **Module split** (mechanical; run `npm test` after). **Math core: DONE** —
   extracted to `src/math/{geodesy,format,projection,triangulate,kinematics,
   astro}.js`, with the test scripts rewritten to import the real modules.
   **Remaining seams** (the banner comments): `src/exif.js`, `src/shapes.js`,
   `src/components/{MediaMeasure,SkyAimer,PinMap,...}.jsx`,
   `src/report/{html,share,zip}.js`, `src/wizard/*.jsx`. Keep behavior
   identical.
2. **Leaflet map: DONE** — `PinMap` now renders Leaflet tiles (Esri World
   Imagery default, OSM street toggle with a dark-inversion filter), same
   props and same drag-ground-under-fixed-pin interaction: the crosshair is a
   fixed DOM overlay, `moveend` commits the center via `onChange`, and a
   `progRef` guard + 2e-6° epsilon stop commit echoes from looping. Markers
   are `divIcon`s (no PNG assets — they break under bundlers).
3. **ADS-B check — LIVE VERSION DONE** (`src/checks/adsb.js` + `AdsbCheck` in
   ResultsPanel): queries live aircraft around the observers, ranks by
   worst-witness angular separation from the sight-line (a real match must
   satisfy EVERY witness), type→wingspan (~140 types + emitter-category
   fallback) → predicted angular size vs measured, curvature+refraction
   corrected el, ground-state aircraft filtered, search radius derived from
   the sight-line elevation (45 kft ceiling). Ranking math is asserted in
   `scripts/mathcheck.js`. CORS reality (probed 2026-07): **airplanes.live
   sends ACAO:\*** (primary, browser-direct); **adsb.lol has NO CORS** (kept
   as fallback; needs the Railway proxy to matter in-browser).
   **Remaining**: historical replay (adsb.lol daily dumps or OpenSky ≤1 h —
   needs the proxy), report-table integration (the one-table-of-mundane-
   explanations design), adsbdb.com hex→route enrichment, track-time
   matching (compare aircraft position history against the witness track,
   not just Moment A).
4. **Terrain skyline calibration** — the strongest calibration source in
   hills, where auto-horizon fails. Data: AWS Open Data Terrain Tiles
   (Terrarium PNG, free, no key, CORS-open, 3DEP/NED 10 m in the US):
   `https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png`,
   decode `h = R*256 + G + B/256 − 32768` (attribution required). Fetch
   ~z13 3×3 near + z11 ring to ~45 km around the observer (≈1–2 MB), decode
   via canvas into typed-array heightfields. Compute the predicted skyline:
   for az 0→360° (0.4° step), ray-march log-spaced distances, elevation
   angle = atan((h(d) − h0 − 1.6 − d²(1−k)/2R)/d) with refraction k≈0.13,
   keep the running max. Render it as a dashed TERRAIN polyline in the sky
   view (like the grid) — dragging the photo until its ridges sit on the line
   calibrates az + pitch + roll simultaneously, day or night, no compass.
   v2: extend the auto-horizon detector to a full photo-skyline polyline and
   cross-correlate against the DEM skyline over azimuth shift → one-tap snap.
   Side benefit: `h0 = DEM(observer)` gives true ground elevation — offer
   "use DEM elevation for all observers", structurally fixing the GPS-altitude
   -wobble problem. Point-query fallbacks: OpenTopoData public API
   (`api.opentopodata.org/v1/ned10m`, 100 pts/call, 1000 calls/day) and USGS
   EPQS; bulk GeoTIFF via OpenTopography API (free key) if precomputing
   server-side.
5. Re-enable device sensors: flip `ENABLE_SENSORS` (point-with-phone + camera
   AR were parked because the artifact iframe blocked permissions). The 📍
   use-my-GPS button is likewise parked behind `ENABLE_GPS_BUTTON` (didn't
   work reliably in the field; the Leaflet pin map covers positioning).
6. PWA manifest + `capture` camera input; real file downloads already work
   outside the artifact (zip/report/share buttons are wired).
7. Multi-witness 3D **pose reconciliation** from `shapeFit.rotM` — two
   observers' capsule poses should agree on one world axis; a falsifiable
   consistency test.
8. Report charts (trajectory/plot as embedded images), print CSS polish.
9. **Reference-locked video** (after the still reference stack — video composes
   the same primitives). Phase A: pose(t) timeline — per-frame Δpose from
   horizon/ridge tracking (pitch+roll; detector exists) + distant-feature
   horizontal drift (yaw), anchored absolutely at sparse keyframes via
   skyline match / star solve / sun; assume pure rotation (valid for distant
   scenes), fixed zoom (warn if skyline scale drifts), note rolling shutter.
   Phase B: measurement-grade stabilization — feed pose(t) into the EXISTING
   canvas mesh warp with a fixed virtual camera; object pixel motion becomes
   true angular motion; auto-track the object → dense time-stamped az/el
   records into the existing track pipeline. Export via canvas.captureStream
   + MediaRecorder; frame-stack stabilized frames for faint objects; a
   grid-burned stabilized clip is a report exhibit. Provisions to make NOW
   during the module split: calibration fns take (frame, pose); treat
   mediaAim as a pose sample, not a per-source constant.

## Calibration & cross-check source roadmap (all free)
Beyond terrain (item 4) and ADS-B (item 3), verified candidates:
- **Astrometry.net** (free key, self-hostable): plate-solve night photos →
  exact pointing + rotation + pixel scale (= measured FOV per device/lens).
- **Star/planet catalog** (static, no API): ~300 brightest stars as JSON +
  compact planetary math; render on the sky grid like the Sun chip.
- **CelesTrak TLEs + satellite.js** (no key, client-side): ISS/Starlink/sat
  check against the sight-line at the sighting time — the night ADS-B.
- **Open-Meteo** (no key, historical): winds aloft at the fix altitude →
  balloon signature test (track speed/heading ≈ wind); cloud-base bounds.
- **NOAA WMM** (client-side JS): magnetic→true declination when azRef="M".
- **Launch Library 2** + **NASA CNEOS fireball API**: known-events correlator
  (rocket launches, bolides) near the sighting time/place.
- **adsbdb.com**: aircraft hex/callsign → type, registration, route.
- **Esri World Imagery / OSM tiles**: satellite basemap for the Leaflet pin.
- **OSM Overpass**: named peaks + towers near the observer → labeled DEM
  skyline and landmark azimuth anchors. **Nominatim**: readable place names
  in reports.
Design principle: each check outputs the same shape — a candidate with
predicted az/el/angular-size/motion and an angular separation from the
witness sight-line — so the report can rank ALL mundane explanations in one
table.

## Video: reference-locked pipeline (phase 2 — after the stills stack matures)
Goal: world-locked video — solve each frame's absolute pose so the sky/grid/
terrain stay frozen and only the object moves. Build ladder:
1. Per-frame pose: pure-rotation (3-DOF) solve from sparse optical-flow tracks
   of static background features, anchored to reference KEYFRAMES (terrain
   skyline / star plate-solve / sun). Distant scenes ⇒ translation negligible.
2. Track points convert through their own frame's pose (removes the current
   "camera never moved" assumption — the biggest hidden video error today).
3. World-stabilized playback/exhibit render = existing mesh warp, per frame.
4. Auto object tracking (template correlation seeded by first tap) → dense
   trajectories → real g-load curves.
5. Rolling-shutter per-row pose correction; OIS/EIS = slowly-varying FOV term
   in the solve. Endgame: PWA/native capture logging gyro @100+ Hz, fused
   with absolute references (complementary filter).
First video ground truth: tripod clip of an airliner + ADS-B replay.
Prerequisite discipline: keep pose/projection math pure & frame-agnostic
through the module split; allow track points an optional per-frame pose field.

## Style
Functional React, hooks only, no state libraries. The entire aesthetic (night
instrument: `--bg #070B14`, amber inputs, teal computed values, mono readouts)
lives in the `css` template string — keep it. No heavy dependencies without
discussion; the hand-rolled EXIF parser, zip writer, and 3D projector are
features, not oversights. Preserve honest epistemics everywhere: warnings over
silent guesses, caveats in reports, "quality: poor" when it's poor.
