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
   pinch-zoom, loupe, brightness/contrast), `PositionEditor` + `PinMap` (Leaflet),
   `SkyAimer` (Place/Look modes, canvas mesh warp, wizard trajectory + Δt
   chips, compare ghosts, aircraft/star/terrain layers), `PlotBoard`,
   charts, `ResultsPanel` (rendered inside `WizFinish`), `AdsbCheck`.
4. **Wizard + reports** — `WizHome/WizStep/WizFinish/ReportView`,
   `packSources`, `reportHtml` (self-contained HTML w/ embedded data + photo
   exhibits + detail crops), `buildShareJson`, zip writer, import.
5. **App shell** — the guided wizard is the ONLY workflow. The Lab/advanced
   mode, Solo mode, the demo loader, the auto-level-from-horizon detector,
   and the manual "A/B from marks" buttons were all removed 2026-07
   (user decision: one workflow to maintain). `commitPlacement` now derives
   BOTH A and B sight-lines automatically from marked points when the
   photo placement commits.

## Non-negotiable invariants (each one was a multi-hour bug hunt)
1. **Never render user images through CSS `matrix3d`.** iOS Safari composes
   its hidden EXIF-orientation transform with author matrices unpredictably.
   All images are **canvas-normalized once at load** (scaled by AREA to iOS
   Safari's ~16.7 Mpx canvas ceiling — kept at 16.0 Mpx with a 4600 px side
   guard, so a 12 MP photo passes through untouched and 24/48 MP shots keep far
   more detail than the old flat 4300 px cap; near-lossless JPEG 0.98; EXIF
   baked out, `mediaNorm` flag), and the sky-view photo warp is a hand-rolled
   canvas **triangle mesh through `projectD`**. The warp texture is ALWAYS a
   static image (≤1600 px): for video, the marked frame
   (`A.videoTime`) is baked to a canvas off the render path — the warp never
   draws a live `<video>` (per-render video→canvas draws were slow, janky, and
   crashed iOS on memory). This holds for STABILIZED PLAYBACK too: it is a
   single-in-flight SEEK loop on an offscreen video — each step bakes THAT
   frame to the texture and sets the frame's solved pose (`playPose`, a
   display-only override; placement state is never touched, so
   commitPlacement can't absorb a mid-video pose). Analysis (marks/shape)
   stays single-frame on the marked frame set on the measure step.
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
   **Corollary (cost: several broken releases):** the aimer's scroll-lock
   preventDefaults ALL document touchmoves while open, which silently kills
   native drags on anything inside it — sliders and draggable elements DON'T
   WORK in the sky view unless whitelisted in that handler (range inputs now
   are). Prefer buttons/taps; if you must add a slider, extend the whitelist
   and test on a real phone.
6. **Loupe keeps its devicePixelRatio backing store** or it renders soft.
7. **Drags are rAF-coalesced** (`queuePose`) — 120 Hz phones flood React
   otherwise. Warp texture is pre-downsampled to 1600 px, mesh 7 columns,
   warp DPR capped at 2.
8. **`window.storage`** is the persistence API (artifact heritage);
   `src/storageShim.js` maps it to localStorage. Keep the contract.
   Media does NOT go through it: autosave strips `mediaUrl` (data URLs are
   MBs; localStorage caps ~5 MB). Images/videos persist in IndexedDB via
   `src/mediaStore.js` keyed by source id, re-attached on boot; removal,
   new-sighting and reset clear their entries.

## Image brightness/contrast (display-only, non-destructive)
`source.imgAdj = {bri, con}` (percentages, 100 = neutral) is a DISPLAY aid set on
the first image step (MediaMeasure sliders) and carried through the sky view and
report. It NEVER modifies the original pixels: measurement paths (star detection
/ `autoStarAlign`, snap-to-ridges, marks) always read the raw `mediaUrl`. Applied
two ways that are kept in exact visual sync: CSS `filter: brightness() contrast()`
on `<img>` surfaces (measure, place mode, report exhibits) and a pixel pass
(`applyImgAdj`, same math order — brightness then contrast) baked into canvas
surfaces where `ctx.filter` is unreliable on iOS (the sky-view warp texture, the
measure loupe). Persists via autosave + share JSON; report captions note when a
photo was adjusted.

## Field findings baked into the UX (don't "simplify" these away)
- Phone compass: **sub-degree on foot**, **14–66° wrong** near metal (inside a
  car; under a steel-roofed pergola). Hence `arbitrateBearings` (size-ratio
  compass arbitration), per-ray-miss diagnostics in reports, and warnings.
- Phone GPS **altitude** wobbles ±5 m → short-baseline alt-spread warning;
  the fix is "set observer elevations equal on level ground".
- Mountain/tree skylines are not a true horizon — the DEM terrain skyline
  in the sky view is the calibration answer (the old auto-level-from-horizon
  detector was removed with the Lab).
- FOV pinch during sky placement overrides lens truth — the readout shows it;
  Reset restores lens FOV.

## Priority backlog (rough order)
1. **Module split** (mechanical; run `npm test` after). **Math core: DONE** —
   extracted to `src/math/{geodesy,format,projection,triangulate,kinematics,
   astro}.js`, with the test scripts rewritten to import the real modules.
   **exif.js and shapes.js: DONE** (pure modules; parseMediaMeta public,
   shape system exports its solids/rotations/projector). **Remaining
   seams**: `src/components/{MediaMeasure,SkyAimer,PinMap,...}.jsx`,
   `src/report/{html,share,zip}.js`, `src/wizard/*.jsx`. Keep behavior
   identical — do these as the video work begins (video lives in SkyAimer).
2. **Leaflet map: DONE** — `PinMap` now renders Leaflet tiles (Esri World
   Imagery default, OSM street toggle with a dark-inversion filter), same
   props and same drag-ground-under-fixed-pin interaction: the crosshair is a
   fixed DOM overlay, `moveend` commits the center via `onChange`, and a
   `progRef` guard + 2e-6° epsilon stop commit echoes from looping. Markers
   are `divIcon`s (no PNG assets — they break under bundlers). The position
   step also draws a **viewing-direction ray** from the pin (fixed DOM SVG,
   north-up map so screen rotation = true bearing): auto from the photo's
   EXIF compass, or set via a bearing slider. It is the placement-center
   azimuth `mediaAim.az` — the SAME field the sky view uses — so the two
   screens always mirror: set the ray → sky opens aimed there; rotate the
   placement in the sky view → on commit the ray follows. Repointing the ray
   rotates the observer's sight-lines (`A.az`/`B.az`) by the same delta.
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
   **Also done**: live traffic drawn on the SkyAimer dome (✈ chips at true
   az/el, heading-rotated, 20 s refresh, header toggle, amber staleness
   warning), snapshot persisted to `source.adsb` on aimer close/unmount
   (wizard UNMOUNTS the aimer — the unmount drain matters), and an
   "Aircraft check (ADS-B)" report section ranking the snapshot against the
   final sight-lines with a fetch-vs-sighting time-gap caveat.
   **Historical replay: DONE** — `server/index.mjs` (dependency-free node,
   also serves `dist/`; Railway `npm start`) exposes `/api/hist?lat&lon&nm&t`
   backed by the tar1090 **globe_history archives** (globe.airplanes.live
   primary, globe.adsbexchange.com fallback — both serve ~2 years back AND
   today progressively, no key, no CORS → hence server-side). Data path:
   30-min binary heatmap slice (10–25 MB, LRU-cached) → 60×u32 offset table
   → 30 s sub-slices, 16-byte entries {u32 hex, i32 lat×1e6, i32 lon×1e6,
   i16 alt/25ft (−123 = ground sentinel), i16 gs×10kt}, marker hex
   0xe7f7c9d carries ms timestamp in lat/lon, INFO entries (|lat|>90°)
   carry the callsign; nearest ~32 hexes refined via
   `traces/{xx}/trace_full_{hex}.json` → interpolated exact state +
   registration + type. Format validated against trace ground truth.
   Pick deep (32): near airports many picks are on the ground AT t and get
   dropped. Client: sighting >15 min old → `fetchAircraftAt` (archive,
   "✈ N @ sighting" chip, teal provenance line) with live fallback (amber
   warning); fresh → live with 20 s refresh. Dev: vite proxies `/api`→8787;
   run `node server/index.mjs` beside `npm run dev`.
   **Remaining**: adsbdb.com hex→route enrichment, track-time matching
   (compare aircraft position history against the witness track, not just
   Moment A). The user is wizard-first: the Lab stays but new check UI
   belongs in the sky view + report, not Lab-only panels.
4. **Terrain skyline calibration — v1 DONE** (`src/terrain.js` + SkyAimer):
   Terrarium tiles (CORS-open, probed) → z13 3×3 + z11 5×5 heightfields →
   `skylineFromSampler` ray-march (az 0→360° @0.4°, log distances to 35 km,
   k≈0.13 refraction) → dashed green TERRAIN polyline on the dome with a
   ⛰ header toggle. The pure core takes an injected sampler and is asserted
   in mathcheck (synthetic cone). **Validated against independent ground
   truth**: observer DEM 408.6 m vs USGS NED 408.3 m; skyline at az 250°
   7.12° vs 7.00° from an OpenTopoData ray-march. PositionEditor gained
   "⛰ Use terrain elevation" (per-observer; sets alt from `demElevation`) —
   the structural fix for GPS-altitude wobble.
   **v2 SNAP: DONE** — detectSkyline (72-column sky/ground edge polyline,
   windowed-median outlier rejection) + matchSkyline (azimuth scan with
   per-candidate least-squares pitch intercept + roll slope; asserted in
   mathcheck against synthetic truth) + a two-pass "⛰ Snap to ridges"
   button in place mode. End-to-end verified against a synthetic photo
   rendered from the REAL Rogue Valley DEM at a known pose: one tap took
   az 258.4/el 6.2/roll 0 to exactly 250.0/9.9/1.5 (truth 250/10/1.5).
   detectSkyline picks the LOWEST sky/ground boundary with sustained ground
   below (per-column adaptive threshold), NOT the strongest edge — a tree
   canopy out-gradients a hazy distant ridge and used to capture the
   detector (field report: snap failed with rms 7.57°); it now sees through
   foreground foliage that has sky beneath it. Asserted in mathcheck on
   synthetic sky/ridge/field images with and without side-column canopy
   (both recover the horizon to ~2 px). Also: a "colour" hue slider under the
   sky-view mode row recolors ALL photo overlays together — the crosshair
   (`accentCol`), the object outline, and the terrain ridge/peak lines
   (`ridgeCol`) — so they stand out against the artist's photo (state
   `ridgeHue`, persisted in localStorage `phodar:uiHue`, default 40 ≈ app amber).
   Roll sign is empirical — matchSkyline returns the error IN the
   measurements; az/el add, roll subtracts. "Set every observer's
   elevation from terrain" one-tap lives in WizFinish's altitude-spread
   warning. **Layered ridges: DONE** — `skylineFromSampler` also returns
   `ridges`: interior visible crest lines below the silhouette (a crest
   is emitted only when unoccluded by nearer terrain AND the ground
   behind drops ≥0.25° below it — occluded stretches emit nothing, so
   layering is in the data), stitched into polylines by depth+elevation
   continuity, drawn on the dome in the same green, thinner and faded
   with distance; the silhouette stays the bright line and the snap
   still matches against it only. Asserted in mathcheck (near cone in
   front of a tall far wall; a fully-hidden middle cone must not leak).
   **Sea-level clamp**: DEM tiles carry bathymetry (negative sea-floor depth)
   over oceans, but the visible surface is 0 m — sampling raw made the
   running-max skyline a bumpy, too-low fake ridge over open water (field
   report: coastal observer looking out to sea). `skylineFromSampler` now
   clamps samples (and the eye height) to ≥ 0, so water reads as the flat sea
   horizon; asserted in mathcheck. **Foreground skip**: the far-field clamp
   wasn't enough — the real culprit at the coast was near-field NOISE: the
   coarse tile (~19 m/px z13) put a spurious 7 m "berm" 40–85 m from a 5.6 m
   eye → a fake 2.7° ridge that won the running-max over open ocean. The
   march now starts at 200 m (a few coarse-grid pixels), below which samples
   are resolution noise, not the horizon this calibrates against. It only
   changes the skyline when the near field spikes ABOVE eye at close range
   (the coast case); for normal terrain the far horizon already wins. Both
   asserted in mathcheck (a near berm over sea must stay flat). **Still open**: labeled peaks via OSM Overpass. Point-query fallbacks if
   Terrarium dies: OpenTopoData (`/v1/ned10m`, 100 pts/call, 1000/day),
   USGS EPQS.
5. Re-enable device sensors: flip `ENABLE_SENSORS` (point-with-phone + camera
   AR were parked because the artifact iframe blocked permissions). The 📍
   use-my-GPS button is likewise parked behind `ENABLE_GPS_BUTTON` (didn't
   work reliably in the field; the Leaflet pin map covers positioning).
6. PWA manifest + `capture` camera input; real file downloads already work
   outside the artifact (zip/report/share buttons are wired).
7. Multi-witness 3D **pose reconciliation** from `shapeFit.rotM` — two
   observers' capsule poses should agree on one world axis; a falsifiable
   consistency test.
8. **Report charts: DONE** — reportPlotSvg (top-down observers/rays/fix/
   trajectory, scale bar) + reportTrajSvg (speed + felt-load strip) as
   self-contained SVG; plus the single-witness distance⇄size chart. The
   top-down plot now composites an **Esri World Imagery basemap + map-data
   overlay** (satellite + roads + place-names/boundaries): Web-Mercator tiles
   → canvas → data URI baked into the report, so it stays self-contained/
   offline. Tiles are fetched via a **same-origin proxy**
   (`/api/tile/{layer}/{z}/{y}/{x}`, layer ∈ img|trans|ref, in
   `server/index.mjs`) so `toDataURL` isn't CORS-tainted; any failure (no
   server, upstream down) falls back to the plain background — never blocks
   the report. Report HTML also carries a
   viewport meta + responsive rules so it fits mobile width. Print CSS
   polish still open.
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
- **Star/planet catalog: DONE** — `src/math/starcat.js` (327 stars to mag
  3.6, 235 named, Yale BSC via d3-celestial) + `src/math/planets.js`
  (Schlyter low-precision ephemeris; **validated vs JPL Horizons to
  ~0.01°**, frozen as mathcheck regression values) + `raDecToAzEl` in
  astro.js (star-transit geometry asserted). Sky view renders the REAL
  night sky (the old random dots are gone): mag-scaled dots, labels for
  bright named stars, glowing labeled planet markers, tap-to-recenter
  planet chips. Reports gain a "Sky-object check": Sun/Moon/planets/
  bright stars within 5° of any sight-line, with a Venus-specific warning
  (the most-reported "UFO" there is).
  **Auto star-align: DONE** — `src/checks/platesolve.js` is a LOCAL plate solve
  (not blind astrometry): `detectStars` finds compact bright blobs in the photo
  (connected-components; over-size diffuse blobs = clouds are dropped), and
  `autoStarAlign` coarse-searches small roll/az/el/FOV offsets around the manual
  placement for the most catalog matches, then ICP-refines with shrinking
  tolerance via `solvePoseAnchors` → exact az/el/roll/FOV/lens-k. Seeded by the
  current placement (mirrors "Snap to ridges" but against stars).
  **Wide "straight-up" solve (gridStarAlign + src/math/starcatDeep.js): DONE** —
  the reliable path when EXIF gives the FOV and you know roughly how HIGH you
  looked (near zenith) but NOT which way you were rotated. Locks FOV to a narrow
  band around the EXIF value + elevation near the prior, then scans the ROTATION
  (az+roll) against a DEEP mag-5 catalog (1627 stars — a wide phone frame
  captures far more than the mag-3.6 display set), verified by a spatial-hash
  blob lookup, and polishes IN PLACE (no coarse re-search — that drifts on a
  dense catalog). Validated on a real main-camera zenith photo: 72 stars at
  0.04° rms. autoAlign runs grid FIRST when fovH + tilt exist, then blind, then
  seeded. NB: an EXIF-STRIPPED shared copy (no FOV) makes the solver drift to a
  false wide-angle pose — the FOV constraint is what makes the zenith case work.
  One-tap "✦ Auto star-align" button in SkyAimer place mode. **Robust to outliers**:
  matches catalog→nearest blob (so a UFO / satellite / plane / faint non-catalog
  blob is simply never matched), and each ICP iteration trims correspondences
  whose angular residual is far from the consensus (median-based) before the
  least-squares solve, so a cloud-hidden star mis-matched to a nearby blob can't
  bias the pose. Clouds hiding stars just lower the inlier count — it solves on
  whatever remains (≥5). Asserted in mathcheck (blob detection + pose recovery
  from a perturbed seed WITH two cloud-hidden stars, a UFO blob, and clutter —
  UFO/clutter excluded, only the visible stars matched).
  **Seedless (blind) solve: DONE** — `blindStarAlign` needs NO manual placement:
  it matches the ASTERISM (star-to-star angular distances are pose-invariant;
  FOV known from EXIF, visible catalog known from location+time). Two matched
  stars fix the centre; roll is ill-constrained near the centre so it's swept
  (not derived), pruned by a roll-invariant radial pre-score, verified by inlier
  count (RANSAC-style), then handed to `autoStarAlign` to polish + fit k. SkyAimer
  "✦ Auto star-align" now tries blind FIRST (fallback: seeded from the current
  placement). Asserted in mathcheck: full pose recovered from NO seed with the FOV
  guess 10 % wrong and clutter present. ~1 s desktop; the real bright-star catalog
  in one field is small, so it's quick on-device.
- **CelesTrak satellites: DONE** (src/checks/satellites.js — visual-group
  TLEs, satellite.js v5 SGP4 (v7 is WASM-first and breaks vite), Earth-shadow
  lit test, dome markers + pass trails, sky-object-check integration with
  TLE-staleness honesty; ISS validated vs wheretheiss.at to ~61 km).
  The aircraft SKY-TRACK pattern it reuses: `/api/hist` returns
  `trail` ([[dtSec, lat, lon, altM], …] ±4 min from the full-day trace);
  the dome draws faint dashed polylines for craft within 25° of the current
  view direction OR the sight-line (bright/solid when selected) — keying only
  off the sight-line meant zero tracks before Moment A is set, so the look
  direction is the fallback reference; live mode accumulates its own trail
  from the 20 s polls. Satellites should emit the same trail shape.
- **Open-Meteo winds: DONE** (src/checks/winds.js + report "Wind check"):
  pressure-level wind at the fix altitude, forecast API ≤~3 months back,
  ERA5 archive beyond; balloonVerdict compares heading + speed ratio.
  Cloud-base bounds still open.
- **NOAA WMM: DONE** (src/math/geomag.js — embedded WMM2025 coefficients,
  NOAA geomagc algorithm, validated against ALL 100 official test vectors
  to 0.005°; EXIF magnetic bearings auto-correct to true on load).
- **Launch Library 2** + **NASA CNEOS fireball API**: known-events correlator
  (rocket launches, bolides) near the sighting time/place.
- **adsbdb.com**: aircraft hex/callsign → type, registration, route.
- **Esri World Imagery / OSM tiles**: satellite basemap for the Leaflet pin.
- **OSM Overpass**: named peaks + towers near the observer → labeled DEM
  skyline and landmark azimuth anchors. **Forward geocoding is DONE** —
  PositionEditor's "find your spot by name" search runs the **US Census
  geocoder** (onelineaddress, TIGER/parcel — resolves to the actual house;
  Nominatim only knows the road, so rural street addresses landed at the
  start of the highway) for queries containing a digit, then **Nominatim**
  (jsonv2, landmarks + non-US); both CORS-open, no key, browser-direct, no
  server proxy. Census failures (CORS/non-US) fall through to Nominatim.
  Results are labeled exact-address vs road/area (drag the pin). Readable
  place names in reports still open.
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
   **DONE (rungs A+B)** — `src/video/postrack.js` (pure, mathcheck-asserted):
   `detectBgFeatures` (day corners / night star-blobs), `trackFeatures` (NCC
   + sub-pixel), `poseFromTracks` (solvePoseAnchors + the autoStarAlign
   median trim; FOV/k locked via `seed.lockFov/lockK`), `initTracker`/
   `stepTracker` (predict→track→solve→re-acquire, walks OUTWARD from the
   marked frame). SkyAimer "🎞 Stabilize video" (place tools, video only)
   seeks an offscreen video every 0.25 s (≤140 samples, tracked at ≤768 px —
   pose is resolution-independent) and writes `source.posePath =
   [{t,az,el,roll,fov,k,n}]` (persists through autosave/pack; only media
   handles are stripped). Frames with <6 background refs HOLD the previous
   pose and are counted honestly. **Zoom is tracked** (UFO clips almost
   always zoom): rotation preserves pairwise pixel distances between the
   tracked features, zoom scales them — stepTracker's median distance-ratio
   `s` detects it, re-tries failed edge features under the scale-corrected
   FOV (they overshoot the NCC search during a zoom), and seeds the solve
   with that FOV — freed for polish at ≥8 anchors, else locked at the
   estimate (a sparse frame never wanders FOV on its own; lockFov stays on
   when s≈1, so fixed-lens clips stay drift-free). Playback applies the
   per-frame FOV automatically. **FAST zoom** (real witnesses zoom hard):
   when a step's tracking COLLAPSES (predictions overshoot the search AND
   the features' appearance rescales, so NCC finds nothing), stepTracker
   sweeps candidate zoom factors 0.35–2.6× — `trackFeatures(opts.tScale)`
   bilinear-resamples the templates to each candidate's scale and positions
   re-predict under its FOV — adopts the factor that makes the background
   reappear, refines it from the matches' pairwise distances, and re-tracks
   everything under it (asserted: 60→40° in ONE step recovers to <1.5°).
   On top, the stabilize walk BISECTS in time on a failed step (tracker
   snapshot → rewind → midpoint frame, 2 levels → ~0.06 s), halving the
   per-step change exactly where the motion is fastest. **GLOBAL
   REGISTRATION IS THE PRIMARY (the fix that finally beat real foliage
   clips)**: differential tracking cannot survive self-similar scenes
   under zoom — every feature finds a lookalike near its prediction, the
   zoom is masked (s≈1, all features "ok"), and the pose goes self-
   consistently wrong (field-observed twice, 36-42 refs mid-zoom). So
   every step first registers the WHOLE coarse frame (96 px gray) against
   the REFERENCE frame across an explicit scale ladder (0.72–3.65×,
   sub-rung parabola; `registerToRef`) — whole-frame structure can't
   alias like local patches, and the result is ABSOLUTE (ref-anchored):
   zoom-proof and drift-proof by construction. The global pose seeds the
   sparse layer's predictions/tScale; the sparse solve then polishes
   (FOV freed only with ≥10 well-spread anchors — rung quantization is
   ~±0.5° so the polish matters); if sparse fails the global coarse pose
   is adopted rather than holding (absolute > held). The differential
   machinery (scale probe by radial-fit coherence, collapse ladder
   rescue) remains as the FALLBACK for frames that pan off the reference
   coverage (registerToRef → null). The feature-precise re-anchor still
   runs near reference scale, but its FOV vote is gated to near-native
   template scale (0.77–1.3) — heavy resampling biases it worse than the
   sparse polish it would override. Top-up only on solved/global steps
   (re-seeding on a held pose bakes its error into new features' world
   dirs — the old poisoning). Asserted: foliage-like self-similar field
   AND sparse-blob field through full 60→41→60 zoom cycles track FOV
   to <1° with az locked. Field-clip hardening (validated offline against
   the real 5× zoom clip): scale ladder reaches ~5×; the registration
   template is a CENTRAL 80% CROP so s≈1 keeps translation freedom (a
   full-frame template made handheld pan bias the scale ~5%); point-
   content (starfields) gates global OFF (area NCC aliases dots-on-dots
   — the differential chain owns those); a CONTINUITY GATE rejects
   global fixes >8° az / >10° el from the chain (deep-zoom templates
   are tiny and self-similar content offers wrong placements — one held
   a 30° az excursion for 1.5 s mid-zoom and an 11° one at a zoom-out
   landing; costs are asymmetric — a rejected true fix only delays
   re-anchoring, an accepted false one poisons samples — so the az gate
   is tight, while el stays 10° because coarse vertical placement is
   noisier); a PHYSICAL FOV CAP (`opts.fovMax`, threaded from stabilize:
   lens EXIF ×1.06, or placement ×1.3 without metadata) bars every
   estimator — ladder rungs, probes, rescue, solves — from reporting a
   frame WIDER than the lens's widest, because the s<1 rungs have the
   SMALLEST templates, which decorrelate least under handheld mismatch
   and would otherwise win impossible 110–127° FOVs exactly at the
   zoom-out landing (field-observed on the second real clip); a wild
   single-step roll (>10° on <12 anchors) is rejected as a blurred-frame
   garbage solve; and the walk
   runs `despikePath` (neighbour-interpolation despike, ramps preserved,
   weak frames yield sooner) before saving, reported as "N glitches
   smoothed". **Roll-hinted registration**: the whole-frame NCC assumes
   the frame sits at the reference's roll — a handheld tilt of ~5°+
   decorrelates it (field-observed: the clip's rolled tail, horizon
   tilted ~12°, lost every global lock and froze the pose while the
   camera kept moving). `registerToRef` also tries the coarse frame
   DE-ROTATED by the chain's roll estimate (`opts.rollHint`, threaded by
   stepTracker) and keeps the better score; the matched center is
   rotation-invariant so the az/el/fov mapping is unchanged, and roll
   itself stays owned by the sparse solve. **Held-frame bridging**: a
   frame that neither solved nor globally locked repeats the previous
   pose (stepTracker returns `held`), which despike can never repair (a
   repeat always deviates less than the neighbours disagree). The walk
   drops interior held runs spanning ≤0.55 s so posePathAt interpolates
   across them ("N weak frames bridged"); LONGER runs stay frozen —
   interpolating a 1 s+ gap fabricates motion (on the real clip it would
   ramp a zoom in 0.75 s early, up to 26° of invented FOV). After despike
   the walk runs `smoothPath`: an evidence-weighted 3-tap pull toward the
   neighbours' time-interpolation (strong solves barely move, weak ones
   lean harder; az wrap-aware; 2 passes) — sub-degree solve noise reads
   as background jitter in the world-locked render, while real motion is
   a ramp across samples and passes through (a perfect linear pan is
   untouched, asserted in mathcheck). **Drift &
   re-anchoring**: incremental drift comes from feature TURNOVER
   (replacements inherit the current pose estimate's error into their
   world dir — a zoom episode churns many). Fix: the tracker keeps the
   reference frame forever; whenever the zoom is within ~2/3–3/2 of the
   reference scale, the PRISTINE reference features are matched directly
   into the frame and the pose re-solved absolutely (stepTracker 4b,
   `anchored`/`drift` in the result; non-prime features' g refreshed
   under the fix). The stabilize walk then `smearDrift`s each anchor's
   correction back across the un-anchored span, linear in time — the
   incremental chain and the absolute fix MEET IN THE MIDDLE instead of
   snapping (asserted: a tracker contaminated with 0.4° turnover drift
   recovers truth to 0.1° and reports the removed drift). `detectBgFeatures` 'auto' mode COMBINES
   star blobs + gradient corners (a dusk clip uses stars AND the tree
   line; either alone was wasteful). Asserted in mathcheck (60→46° zoom
   sweep recovered to ~1°, az locked; tree + stars both contribute).
2. Track points convert through their own frame's pose (removes the current
   "camera never moved" assumption — the biggest hidden video error today).
3. World-stabilized playback/exhibit render = existing mesh warp, per frame.
   **DONE (rung B)** — ▶ + scrubber in look mode when a posePath exists:
   single-in-flight seek → bake frame → set `playPose` (a DISPLAY-ONLY pose
   override; `poseNow = playPose || placement`, so commitPlacement never
   absorbs a mid-video pose). Entering place/any tool mode exits playback;
   exit re-bakes the marked frame before dropping the override so texture
   and pose always agree. **⬇ video EXPORT: DONE** — three framings from
   one renderer: "view" (fixed virtual camera fit from the union of all
   frames' corners, ≤1920 px), "full" (same framing, output sized by the
   tan-space ratio camFov/minPathFov so the most-zoomed frames keep native
   pixel density — ≤4096 with a WebCodecs step-down ladder
   3840/2560/1920; the MediaRecorder fallback keeps ≤2560 on iOS: a 3840
   canvas + 4K realtime encode crashed Safari on an iPhone 14), and "crop"
   (camera centered on the marked object's world dir + the B sight-line,
   FOV = max(12× object size, 1.9× A–B separation) with a 1.6° floor —
   PROPORTIONAL zoom, so a far/small object crops much tighter than a
   near/big one — ≤3840; finer 16-col mesh with off-canvas cell culling,
   grid pitch scales with FOV down to 0.5°). OVERLAYS ONLY ON "view"
   (grid + readout + all visible sky layers + wireframe): "full" and
   "crop" are CLEAN evidence renders, nothing burned in (user decision).
   The warp texture is NATIVE for full/crop, guarded only by the iOS
   canvas ceiling (4600 px side / 16 Mpx) — the old 2048 texture cap
   silently halved 4K sources before the warp ever saw them; "view"
   keeps the 1600 px playback texture. **Encoder: WebCodecs VideoEncoder is the PRIMARY
   path** — offline frame-by-frame encode with explicit timestamps and
   queue backpressure, muxed by the hand-rolled single-track H.264 muxer
   `src/video/mp4mux.js` (validated by full ffmpeg decode of a muxed real
   x264 stream; byte-layout asserted in mathcheck; assumes PTS==DTS,
   which `latencyMode:"realtime"` guarantees). This exists because
   MediaRecorder is a REALTIME API: a phone encoder that can't sustain
   30 fps at high resolution silently drops/queues frames until the
   output truncates (field-observed twice on an iPhone 14 — at 3840 AND
   2560 wide), so no realtime cap is ever safe. With offline encode,
   full 3840 is allowed everywhere WebCodecs supports it. MediaRecorder
   (canvas.captureStream, mp4 → webm) remains the no-WebCodecs fallback
   with the conservative iOS 2560 cap and wall-clock pacing BOTH WAYS
   (skip ahead when seeks lag, WAIT when ahead — advancing a fixed 1/30
   min regardless compressed an 18.6 s clip to 14 s playing 1.3× fast).
   Seeks are clamped below the media duration (near-end seeks stall the
   decoder — field-observed as an export truncated 1.1 s early), and in
   the fallback a seek running >300 ms PAUSES the recorder with the span
   excluded from the pacing clock; the WebCodecs path just re-encodes
   the previous texture at the correct timestamp (brief freeze, exact
   duration). A WebCodecs RUNTIME failure steps down the size ladder
   (3840→2560→1920) before falling back to MediaRecorder —
   isConfigSupported can accept a size the hardware then refuses
   (field-observed) — and export errors surface the actual message,
   not a generic "failed".
   The render is persisted to IndexedDB (`mediaStore`, key
   `sourceId + ":stab"`; re-stabilizing deletes it as stale, source
   removal cleans it) and the report .zip bundle packs each observer's
   original clip AND the stabilized render (before/after pair) beside
   the photos.
4. Auto object tracking (template correlation seeded by first tap) → dense
   trajectories → real g-load curves. **v2 DONE (stepObject/snapToObject in
   postrack)** — validated END-TO-END: a synthetic clip built from a real
   field frame (moving crop = camera shake, compositing a 12 px dot flying
   a known path) is driven through the REAL app UI by Playwright (upload →
   fit shape → wizard → place → stabilize) and the saved objPath matches
   ground truth; the offline module harness tracks the same clip to 0.34°
   mean. Every mechanism below exists because the naive version failed on
   that ground truth:
   - Rides the stabilize walk with its OWN native-res buffers (≤1600):
     at the camera solve's 768 px + video compression, a small object
     washes out to a few grey levels and NO matcher can hold it.
   - Seed SNAPPED onto the object (snapToObject: multi-scale
     center-surround contrast peak) — marks are never pixel-centred, and
     a template cut half-off a small object is lost immediately.
   - CONSTANT-VELOCITY prediction (gPrev chain): a world-stationary
     prediction falls behind a real mover and a static lookalike near the
     stale prediction latches forever. Prediction rides the object's own
     angular velocity, so locality preference helps instead of hurting.
   - NEAR vs WIDE search arbitration: take the far match only when
     clearly stronger (Δncc>0.12) — twin-star ties stay near, background
     latches lose to the real object.
   - CENTER-WEIGHTED NCC (trackFeatures opts.centerW, σ=patch/3.4) +
     patch hard-capped at 13: an unweighted background-majority template
     correlates ~0.9+ with "the same grass" after the object leaves.
   - The MARKS stamp their own frame: syncShape writes A.videoTime at fit
     time (video), so marks and marked-frame can never disagree (the old
     "✓ Use this frame"-only flow let the object template seed off the
     wrong frame entirely).
   Misses hold the velocity prediction and count honestly; the track is
   kept only if ≥30% matched (else "object lost", outline stays put).
   `source.objPath = [{t,az,el,q}]` persists; playback rotates the object
   marks onto the tracked dir (Rodrigues d0→dT; B sight-line and track
   points stay pinned — their own observations), and the crop export's
   camera FOLLOWS the track.
   **HYBRID GUIDED TRACKING + SECOND PASS**: the object pass now runs
   AFTER the camera walk completes, against the despiked+smoothed path —
   so every conversion uses the final poses, and measure-step TRACK
   points ({t,x,y}: the object tapped on its own frame, tool now enabled
   in the wizard for videos) convert through their own frame's solved
   pose, staying mutually consistent whatever frames they were marked
   on. With ≥2 timed waypoints they become the GUIDE: stepObject
   opts.guide owns the prediction and a match >2° (guideGate) off the
   human's path is REJECTED as a lookalike — the manual trajectory
   outranks pixels, the matcher only fine-tunes (asserted in mathcheck;
   verified end-to-end through the real wizard UI: 3 px waypoint taps,
   guided track matches ground truth). With <2 waypoints the guide is
   off and misses ride the velocity prediction. Guided misses carry
   q=0.25 (the guide dir), honest in the readout.
   **ALIGN FRAME ≠ OBJECT FRAME (source.alignT)**: the MEASURE step's
   "⛰ Align on this frame" button (wizard video controls, below the
   frame slider; teal ▾ slider tick) picks WHICH frame the world
   alignment is done on (clearest horizon/stars) — decoupled from
   A.videoTime (where the object was fitted; defaults keep them
   coupled). A sky-view scrubber used to do this but was REMOVED:
   every tick re-baked the warp texture and re-rendered the whole
   dome, making it crawl — the measure step's plain `<video>` scrub is
   cheap, so selection lives there and the sky view just shows a
   static "aligning on X.XXs" line. A teal callout on the measure
   step nudges the user to pick one when unset. The sky view
   bakes alignT; stabilize anchors its walk there (refT=alignT, the
   placement pose describes that frame); the object pass then seeds on
   the MARKED frame with its pose from posePathAt. pixDirMarked and the
   export wireframe use the marks-frame solved pose whenever the frames
   differ and a path exists. The placement stamps calib.vt = alignT and
   the sky view warns when the align frame later moves — a pose
   describes ONE frame. Verified e2e coupled (defaults, byte-identical)
   AND decoupled (align 6.0 s, object 0.5 s, track intact).
   Stabilize progress lives in the BUTTON ("🎞 n/total…"), not
   the flash (2.2 s auto-hide made it blink on long clips). The fitted
   3D WIREFRAME rides the track: in the dome (shapeProjNat curves →
   placement-pose dirs → Rodrigues onto the tracked dir, same pipeline
   as the marks) and burned into ALL export framings — gated by the 🛸
   header toggle (objOn), which controls the dome overlay AND the
   export burn-in. playPose CARRIES t — the dome follow keys off
   `playPose.t` (it shipped without t once and the overlay froze at
   the marked spot all playback while the export tracked fine). The
   look-mode dome shows ONLY the wireframe (the two mark rings are a
   no-shape fallback; track dots + B circle were dropped as clutter,
   and the numbered trajectory chips render only while the ⊕
   Trajectory tool is active). The dome wire renders at TRUE fitted
   angular size, scaling with dome zoom through the projection — a
   short-lived "legibility floor" that magnified small wires was
   REMOVED on user request (the set size must be honoured; a distant
   object really is tiny — zoom in). The world-locked EXPORT burns in
   every dome layer that is visible, honouring the same toggles:
   terrain skyline + ridges + named peaks, building boxes (skipped in
   the moving-camera crop mode — the occlusion pass is too heavy per
   frame), stars/planets/Sun/Moon, satellites + Starlink with trails,
   aircraft chips + sky-tracks, compass letters — all drawn from
   world az/el data through the export camera each frame
   (drawSkyLayers, wrapped so a layer error can never kill an
   export). The grid is drawn OVER the frame now, like the dome. The
   schematic overlays (winds-aloft stack, cloud veil) stay dome-only:
   their heights are made up, and burning them into a world-locked
   exhibit would lie. Measure-step track markers fade away from their own
   frame (full ≤0.3 s, gone by 1.1 s, 0.15 floor while the Track tool
   is active; the route polyline stays faint). Verified visually: the
   wireframe sits on the ground-truth dot across an in-app world-view
   export. Manual per-frame correction of a solved track: not yet.
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
