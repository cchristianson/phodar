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
- `npm run helpcheck` — guards the in-app "?" manual (`HELP_SECTIONS`), which is
  the ONLY documentation a user gets and drifts silently: a feature ships, the
  manual never mentions it, nothing fails. It sweeps every distinctive glyph
  that appears on a button, a list of named features keyed on the code that
  implements them, and the shape picker, and fails on anything undocumented.
  A control that needs no entry (close ✕, nudge arrow) goes in the script's
  `STRUCTURAL` list **with a reason**, so skipping the docs is a decision.
  **Run it after adding any control.** It found five undocumented features the
  first time it ran that a by-hand audit had already missed.

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
- `kinematics.js` — `trackDirections`, `kinematics`, `analyzeTracks`,
  `videoKinematics` (dense objPath angular kinematics), `stereoVideo`
  (two-video dense triangulation with auto time-sync), `mixedStereo`
  (video + still: anchor the dense clip to absolute scale via one photo's
  sight-line). **VISIBILITY SEGMENTS** (`trackSegments`/`interSegments`,
  field case: a drone visible only in sections of each of two videos): a
  witness's track has holes where they lost the object, and interpolating a
  direction across a hole fabricates a ray nobody observed — both stereo
  paths triangulate ONLY instants inside every witness's visible segments
  (gap > ~4× the track's own cadence = break), stereoVideo drops q<0.3
  held/guided samples first (predictions, not observations), and the
  UI/report state seconds used vs ignored. Asserted in mathcheck with a
  truth path that turns sharply inside one witness's blind stretch.
  **GEOMETRIC CLOCK SYNC** (field case: a video's capture time ~20 min
  wrong in-app, and still ~41 s off after a hand-fix — proven against the
  drone log): analyzeTracks anchors each track at whenMs + video t (A.t
  counts as a common-clock anchor ONLY when it differs from the source's
  own videoTime — field data carried "0.00" and A.t==videoTime
  placeholders), then for two witnesses searches ±45 s for the offset
  where sight-lines sharply intersect (±30 min coarse RESCUE when tracks
  don't overlap at all), adopting it only on a DECISIVE minimum — a
  hovering object fits every offset, and a flat minimum must never invent
  a shift. Applied shifts are declared in UI + report. This fixes clocks
  RELATIVELY; `witnessClockCheck` (flightlog.js) pins them absolutely
  against a drone log (one sight-line vs the whole flight; exposed the
  real 41 s residual at 0.13° sharp). All mathcheck-asserted (25 s and
  20 min recovery, hover refusal, aligned pass-through).
- `astro.js` — `sunPos`/`moonPos`/`moonFrac` (SunCalc-derived).

Everything else still lives in `src/phodar.jsx` (~3,750 lines) **on purpose** —
it was developed as a single Claude artifact. It is organized by banner
comments, in this order:

1. **EXIF / QuickTime parser** — hand-rolled TIFF walk: GPS, time, bearing
   (`GPSImgDirection` + true/mag ref), 35 mm focal → FOV, orientation.
   **QuickTime is scanned at BOTH ends**: an iPhone writes `moov` (which
   carries the ISO-6709 location and `mvhd` creation time) at the END of the
   file, not the start, so a head-only scan silently lost GPS + timestamp on
   every clip bigger than the window — measured on a 27 MB field recording
   whose location string sat in the last 200 bytes, which is why video
   sightings kept needing the position typed in by hand. Head window catches
   faststart/streaming files; both cases asserted in mathcheck.
   `scripts/metacheck.mjs <file>` runs this parser over any candidate and
   prints what the app will actually see (position / bearing / FOV) plus
   Overpass + F4map links for the spot — use it before building a test around
   a photo, since most web images have EXIF stripped.
2. **3D shape system** — `SHAPES`, `shapeWire` (orb/saucer/capsule/tri/cube/
   plane/bird wireframes), rotation mats, `shapeProjNat` (orthographic project +
   silhouette extremes → auto-writes `A.p1/p2`).
   Per-shape params ride on the `shapeFit` object and reach `shapeWire` as its
   `opts` (capsule `aspect`, bird `wing`/`wingA`, jelly `tent`, cube `squash`,
   vee `sweep`/`notch`, balloon `cord`),
   so a new one needs no plumbing — but a new KIND must be registered in four
   places or it half-works: `SHAPES` (picker), `SHAPE_R0` (default 3/4 pose),
   `shapeWire` (geometry) and `SHAPE_VIEWS` in phodar.jsx (the report's
   dimensioned 3-view, which silently renders nothing without an entry).
   Mathcheck now walks `SHAPES` and asserts all four for every kind, so that
   rule is enforced rather than remembered. A fifth, softer one: the spin
   slider's gate in phodar.jsx lists the kinds that are NOT solids of
   revolution — an orb, tic-tac, balloon or jellyfish is left out because
   spinning it about its own axis would change nothing.
   **Cube ↔ diamond**: `squash` ∈ [0,1] tapers the top and bottom faces toward
   the axis while the waist stays put — 0 is a cube, 1 is a square bipyramid
   (the "diamond"), and the middle is a truncated gem. **`stretch`** ∈ [0.25,3]
   then scales the HEIGHT against a fixed footprint, and the two are
   deliberately independent — squash picks the profile's SHAPE, stretch its
   PROPORTION — so a box, a column, a slab, a tall gem and a flat lozenge are
   all reachable. The **pyramid** (`pyr`) shares `stretch` as its height
   (0.25 = shallow cap, 3 = spire) over a unit square base; base + four slant
   edges only, since any horizontal ring would be a line that is not an edge
   (the same reason the cube's waist is suppressed at squash 0). Note for
   testing: in the 3/4 default pose a shallow pyramid's apex sits INSIDE the
   base square's projected span, so its drawn bounding box does not change
   below ~1× even though the solid does — assert model extents, not screen
   height. The waist ring is what
   makes it a diamond rather than a pair of frustums, and it's suppressed at
   squash 0 (it would band a plain cube with a non-edge) as are the collapsed
   caps at squash 1 (they'd leave a dot at each apex). Asserted in mathcheck by
   corner set + extents + monotone taper, not by eye.
   **V / delta** (`vee`) is the boomerang, and it carries the two things
   witnesses actually vary independently: `sweep` ∈ [0.12,0.9] is how far the
   tips trail behind the apex, and `notch` ∈ [0,1] morphs the trailing edge
   from dead straight (a solid delta) to a deep centre notch (two thin arms).
   Tip-to-tip span stays 1 throughout — it is the measured dimension. Note
   when testing: `notch` moves an INTERIOR vertex, so the drawn geometry
   changes while the outline's bounding box and the measured angular size do
   not; `sweep` is the param that moves the measurement.
   **Stealth jet** (`stealth`, F-117-like) is deliberately spare — planform,
   one dorsal ridge, two folds a side, canted V-tails, cockpit facet. At the
   size these wires are actually drawn (tens of pixels) every extra facet line
   reads as noise; an earlier version with inboard facet edges was unreadable.
   **Balloon** (`balloon`) is the envelope of revolution plus a dangling
   string, `cord` ∈ [0,2] scaling the string and 0 removing it. It exists
   because it is the most-mistaken-for-a-UFO object there is and the taper
   plus the string are what let a witness tell it from an orb in the photo.
   **Bird wing ANGLE, not position** (field report: "the wing position is kind
   of silly"): `wingA` ∈ [−25°,55°] swings the wings aft (a stoop) or forward
   (a soar) — it REPLACED `wingX`, which slid the whole wing fore/aft along
   the body, a thing no wing does. Implemented as a shear on |y| rather than a
   rotation about the shoulder, so the root edge stays welded to the body and
   the tip-to-tip span stays owned by `wing` alone: the two sliders answer
   separate questions (how WIDE, at what ANGLE) instead of fighting. A stored
   `wingX` fit is carried over as the angle that puts the tip in the same
   place, so an old bird keeps its silhouette (and its measurement) rather
   than silently snapping back to neutral — asserted in mathcheck. Note when
   testing: near 0° the drawn bounding box barely moves (the head still sets
   the fore extent), so assert model extents or the geometry hash, not screen
   width.
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
9. **Never `position: fixed` for chrome inside a scrolling page — use
   `position: sticky`.** On iOS a fixed element resolves against the LAYOUT
   viewport, so the moment the visual viewport diverges from it — a stray
   two-finger pinch (Safari has ignored `user-scalable=no` since iOS 10, so the
   page CAN be zoomed), the URL bar collapsing, a focused input shifting the
   page — it renders at a stale, arbitrary spot. Field report: "the next button
   ends up all over the screen instead of at the bottom where it belongs".
   `WizStep`'s footer is sticky and therefore laid out by the same scrollport as
   the content, so it can only ever be at the bottom; a zoom scales it WITH the
   page. Fixed is still correct for `inset: 0` FULL-SCREEN overlays (the sky
   view, modals) — those cover the viewport and lock scrolling, so there is no
   stale offset to be wrong about. Asserted in a scratchpad harness that
   reproduces the stranding with a control fixed bar under a transformed
   ancestor.

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
   sends ACAO:\*** (browser-direct fallback); **adsb.lol has NO CORS**.
   **MULTI-FEED MERGE: DONE** — `/api/live?lat&lon&nm` (server/index.mjs) fans
   out to airplanes.live + adsb.lol + adsb.fi (ADSBx-v2) **and OpenSky**
   (state-vector shape; adds MLAT / Mode-S targets pure ADS-B misses),
   `Promise.allSettled`, unions by ICAO hex with per-field back-fill (a type
   designator from one feed, a fresh position from another — ADSBx type/reg is
   never lost to an OpenSky dup). Each network has different receiver coverage,
   so the union catches craft any single feed misses; a feed that errors or
   rate-limits (OpenSky anon ~400/day per IP) is dropped, the rest still answer.
   `fetchAircraft` calls `/api/live` first and falls back to browser-direct
   airplanes.live when the server is absent. Merge/normalize logic verified with
   a synthetic harness (dedup, back-fill, m→ft / m·s⁻¹→kt, category idx→ADSBx).
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
   **TWO-VIDEO STEREO: DONE** — `stereoVideo` (math/kinematics.js, pure,
   mathcheck-asserted) triangulates ≥2 stabilized+object-tracked clips of
   the same object. Each clip's dense objPath is already WORLD az/el per
   frame (camera motion removed), so every sample is a sight-line — no
   pose conversion. It (1) puts each clip on an absolute clock
   (whenMs + video t + optional syncOffset), (2) AUTO-SYNCS by searching
   the relative time offset that MINIMISES mean ray-miss (device clocks
   drift; the object's own motion is the shared signal — a far/slow object
   gives a shallow minimum, reported as low sync confidence), (3)
   triangulates each common instant with median/MAD outlier rejection (a
   blurred/mistracked frame is a fat miss, dropped), (4) runs kinematics on
   the dense 3D path. Returns fix + trajectory + recovered offset +
   confidence + residuals. Report "Two-video trajectory (dense stereo)"
   section: sync offset + confidence, fix geometry (baseline, convergence,
   ray-miss, per-observer range), true size when sized, kinematics
   (reportTrajSvg) + top-down plot (reportPlotSvg). Verified e2e (injected
   two synthetic clips: recovered a 0.6 s clock error, 351 km/h truth
   speed, rejected outlier frames). Still open: a manual sync-offset nudge
   for when EXIF clocks are missing AND the object is too slow to auto-lock.
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
   polish still open. **Video analysis section: DONE** — for any
   stabilized + object-tracked clip the report adds a "Video analysis"
   section built on `videoKinematics` (math/kinematics.js, pure,
   mathcheck-asserted): the DENSE per-frame objPath → angular rate ω(t)
   in °/s (peak/avg + a self-contained SVG plot), total sky sweep and
   duration — all distance-free — then `atDistance(D)` turns any assumed
   distance into true size + avg/peak tangential speed + path length,
   rendered as a ladder table (or a single row highlighted when a stereo
   fix nails the distance). If the object was sized across frames the
   angular-size range + range-ratio (toward/away motion) is reported.
   Ends with a KEYFRAME strip: N frames baked from the video (offscreen
   seek → canvas → JPEG data URI, capped 900 px), each with the tracked
   object marked (world dir → that frame's solved pose → pixel via
   dirToPixK) and captioned (t, az/el, angular size, rate). All async +
   document-gated; verified end-to-end through the wizard UI to the
   report (8 keyframes, rate plot, table).
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
- **Drone flight-log ground truth: DONE** (`src/checks/flightlog.js` +
  `FlightLogCheck` in ResultsPanel + a report section; protocol in
  `docs/DRONE-TEST.md`): upload the user's own drone log (Airdata CSV /
  decoded DJI Fly CSV / DJI .SRT captions — the raw DJI Fly `.txt` is
  encrypted, the UI says so — binary rejection verified against a REAL Mini
  FlightRecord: v13+ payloads are AES-encrypted, only DJI's key service
  decodes them, which is what Airdata/PhantomHelp do) and grade direction,
  fix position, size, alt, speed against the craft's GPS. DELIBERATELY
  HIDDEN (user decision): it is a test harness, not a sighting feature, so
  ResultsPanel shows only a dim "🛩 calibration" footer link until tapped —
  but the full section stands open whenever a log is already loaded, so an
  in-progress calibration (or an imported one) stays manageable.
  Design decisions that matter: the time
  match scans the WHOLE log for the minimum worst-witness separation (a
  local-time export parsed in the wrong timezone is the common failure —
  a windowed search around the stated time would just miss), and altitude
  datums are explicit (`droneAltM`: abs MSL → takeoff+homeElev → observer
  elevation ASSUMED and flagged). Log persists downsampled (≤900 pts) on
  the first witness as `source.flightLog`, so it rides autosave/share/report
  for free. Pure + mathcheck-asserted against a synthetic flight built from
  exact ENU truth (parse → interpolate → predict → 7.5 s clock-skew
  recovery → analyze() fix graded "excellent" on perfect data).
  **FIELD-VALIDATED** (docs/FIELD-TESTS.md Case 5): a real Mavic Mini flight
  triangulated to 2.19 m of its logged position (2.2% of range) on the
  production PWA. That flight's photos were 122 s apart, so
  calibrationSummary also grades each witness at its OWN photo time
  (per.ownSep; a hover makes the joint-instant match undersell a good
  sight-line — obs 1 was 0.19° at its own time vs 1.24° joint).
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
   **Close-up PIXEL PIN — v3 (`pinStep`, postrack.js, pure + mathcheck-
   asserted).** The crop camera re-locks on the object's ACTUAL pixels
   every frame (pinFind contrast sweep), because the solved track carries
   sub-degree error that the crop zoom magnifies into gross wander. The
   v2 lesson, learned from a real field close-up (object swung ±20% of
   the frame and LEFT it): never gate the locked chain against the TRACK
   — the track is the thing the pin exists to correct, so with ~1° of
   track error the gate rejected the pin's own correct finds and the
   miss path eased the camera back onto the bad track. v3 policy: while
   locked, the tight search window IS the gate (a 4× backstop bounds a
   runaway chain); ACQUIRE still gates against the track (a bird must
   not capture the camera — human outranks pixels); brief fades
   WORLD-HOLD the last lock; a released lock GLIDES back to the track,
   never snaps. Reproduced + fixed in a harness driving the real
   pinFind: a 1.2°-wander track went from 380 px rms object wander (with
   losses) to 41 px, identical to a good track.
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
   - SEED DRIFT ANCHOR (stepObject opts.seed): the pixel template comes
     from the PREVIOUS frame, so each frame's small localization error
     seeds the next frame's template and the track slowly walks off the
     object (field report: "beginning tracks then gets squirrelly and
     drifts off"). Every step ALSO matches the PRISTINE seed template
     (the object patch from the marked frame, which can never drift)
     near the same prediction, and PREFERS it whenever it still
     correlates within 0.06 ncc of the frame-to-frame match — so a
     stable-looking object stays locked to its original appearance, and
     only a real appearance change (zoom/rotation dropping the seed's
     ncc well below the adaptive match) lets the frame-to-frame match
     win. Asserted in mathcheck (a distinct bright object swept past a
     dimmer background grid stays <0.35° over 14 frames).
   Misses hold the velocity prediction and count honestly; the track is
   kept only if ≥30% matched (else "object lost", outline stays put).
   The finished objPath is run through `smoothObjPath` (postrack, pure,
   mathcheck-asserted): an evidence-weighted despike+smooth keyed on `q`
   (the match confidence) — a background-lookalike LATCH (a lone big jump
   on a low-q frame) is snapped back onto the neighbours' time-
   interpolation, per-frame matcher jitter is damped by a light 3-tap
   pull (strong pixel locks barely move, misses lean on their
   neighbours), and a real smooth object sweep (low-frequency across
   samples) passes through untouched. This is what makes the outline
   GLIDE on the tracked object instead of stuttering; the flash reports
   "N jumps smoothed".
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
   Trajectory tool is active). **KEYFRAMED size + attitude**: the
   displayed model (dome playback + world-view export) interpolates the
   SIZE (`wpx` marks, measure step) and ROTATION (`rotM` marks, sky-view
   trajectory rotMode) the user set along the track — `sampleShapeAt`
   (shapes.js, pure, mathcheck-asserted) linearly ramps apparent width
   and quaternion-SLERPs attitude between keyframes (clamped past the
   ends, fitted shape where none), and `shapeAt` normalises the target
   apparent width to a real sizeNat through the rotated shape's own
   projection. Two size marks ⇒ a smooth grow/shrink between them; N
   marks ⇒ piecewise; same for attitude via SLERP. The dome wire renders
   at TRUE fitted angular size, scaling with dome zoom through the
   projection — a
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
   export. **Manual pose correction (⚓ Fix frames): DONE** — playback-row
   ⚓ opens a mode where you scrub to a frame that lost the world lock,
   drag the photo back onto the true horizon/terrain (one finger = az/el,
   two-finger twist = roll; two-finger drag pans the view, pinch zooms),
   and ⚓ Anchor it. Anchors are ABSOLUTE poses in `source.poseFixes`;
   `applyPoseFixes` (postrack, pure, mathcheck-asserted) turns them into
   a delta field over the smoothed base path — exact at anchors, linear
   between, HELD beyond the outermost (so drift heals to the ends; a
   zero-delta "Pin as correct" anchor bounds a correction region) — and
   `applyDirFixes` shifts the object track through the same field. The
   stored posePath/objPath are re-derived (rederivePaths) on every
   anchor/smoothing change from the raws, so anchors and the smoothing
   sliders compose non-destructively; waypoints re-convert through the
   fixed path automatically. Live feedback: the pending (un-anchored)
   drag lives ONLY in playPose (display override — scrubbing away
   discards it, commitPlacement can't absorb it), and the trajectory
   overlay renders in fix mode (gated by 🛸) from a preview path with
   the pending pose applied as one more anchor, so the trajectory
   slides live under your finger. Re-stabilizing clears poseFixes
   (corrections of the old solve); re-aligning the placement rotates
   them (reanchorPose), the PinMap bearing ray yaws them.
   **⛰ AUTO-ANCHOR TO TERRAIN: TRIED AND REVERTED** — do not rebuild it
   this way. The diagnosis behind it still stands and is worth keeping:
   MEASURED on a real field clip (22.8 s, 1.35x zoom, handheld, ridge
   visible throughout) the stabilizer removes the SHAKE well — residual
   frame-to-frame background motion 0.26 px at 540-wide, HF jitter
   0.08 px/frame vs 0.11 in the original — but the lock SLIDES, ~60 px
   cumulative excursion at 540-wide (~120 px at output res), accumulating
   mostly over the first 10 s, and visible in the world-view export as the
   photo's ridge sitting ON the burned-in DEM terrain line at t=8 s and
   ~1.4 deg BELOW it at t=16 s. Cause: the only absolute reference is the
   ONE marked frame, so everything between re-anchors is an incremental
   chain, and the coarse global register (96 px, integer NCC peak
   ~0.4 deg/px, 11.5% scale rungs) can't hold it better than ~1-2 deg.
   The attempted cure ran detectSkyline + matchSkyline on ~30 sampled
   frames and wrote each solved pose as a poseFixes anchor. IT FAILED
   BADLY IN THE FIELD: `matchSkyline` scans the FULL 360 deg of azimuth,
   so an edge that is not really the terrain horizon — a nearby TREE LINE
   across a flat meadow, which is the common case — fits best at a
   completely wrong bearing. The anchors disagreed frame to frame, the
   world-view export's virtual camera grew to contain them, and the photo
   rendered as a tiny tilted sliver in a mostly-empty frame. (Beware: a
   frame-to-frame background-motion metric IMPROVES in that state, because
   there are barely any photo pixels left to measure — always look at a
   frame, not just the number.) A cross-frame median filter on the
   corrections was added and still did not save it. If this is retried,
   the azimuth scan must be BOUNDED to a few degrees around the tracked
   pose, the detected edge must be verified as terrain (not canopy), and
   the result must be judged on a rendered frame.
      **Tracker-precision experiments that did NOT ship** (measured; kept
   here so they aren't retried blind): sub-pixel parabolic peak on the
   global NCC surface, refilling the feature herd toward maxN instead of
   minMatch+4, widening the absolute re-anchor past `ref.feats.slice(0,20)`,
   and a fine second scale ladder. Together they halved the field clip's
   azimuth error (rms 0.82°→0.44°, FOV range 25%→17%), but each of the
   last three degrades the synthetic 2× zoom-cycle regression (`textured
   world`): widening the re-anchor breaks it outright (heavy template
   resampling biases it — the very thing its scale gate exists for), and
   the fine ladder pushed `maxA` from 0.11° to 0.29° against a 0.3°
   limit. Steady-state precision trades against deep-zoom robustness
   there; the absolute terrain anchor sidesteps the trade entirely.
   **INSTRUMENTED CAPTURE (sensor attitude log): v1 DONE** — the answer to
   the drift measured above, from the other direction. `SensorCapture` gains
   "🎬 Record with motion data": getUserMedia + MediaRecorder while the
   existing gravity/compass pose loop logs `{t, az, el, roll}` at ~25 Hz
   (rAF-paced, gravity-smoothed, throttled so a long clip still autosaves)
   into `source.sensorPath`. `src/video/sensorpath.js` (pure,
   mathcheck-asserted) then does the work:
   `syncSensor` recovers the constant offset between the log's clock and the
   encoded timeline — the gap between "start recording" and the first frame
   is device-specific — by minimising disagreement on the SHAPE of the
   motion with each axis's mean removed, so the compass bias (field-measured
   14-66 deg near metal) cannot drag the answer; convention is
   `sensorAt(log, videoT + offset)`. `fuseSensorVisual` then walks the solved
   path: a well-solved frame is taken as-is and becomes an anchor, a weak or
   held frame is carried from the last anchor by the SENSOR'S OWN DELTA
   (real motion instead of a frozen repeat), and when vision recovers the
   carried run is smeared onto it so the two meet in the middle. A stretch
   longer than `maxCarry` (2.5 s) is left held and flagged rather than
   fabricated. Each entry gains `src`: v/s/b/h, counted in the stabilize
   flash. Applied in `rederivePaths`, so smoothing sliders and Fix-frames
   anchors compose with it.
   **INLIER COUNT IS NOT A TRUTH SIGNAL** (field case, first instrumented
   clip): a tracker that loses the scene and re-acquires on whatever drifted
   into frame reports a CONFIDENT solve that barely moves — the phone swept
   95 deg of azimuth (gravity concurring with 27 deg of elevation) while the
   visual solve reported 11.5 deg at 34-46 inliers per frame. Every frame
   therefore passed `strong()` and the sensors were never consulted at all.
   So `motionDisagreement` compares the PATH LENGTH each source travelled,
   and when the log says the camera moved and the vision says it did not
   (sen > 15 deg and vis/sen < 0.45), `sensorOnlyPath` rebuilds the whole
   path from the log — motion from the sensors, ABSOLUTE frame from the
   placement (each sample is the placement pose plus the sensor's delta
   from the alignment frame, so the compass bias cancels exactly and a
   star/terrain calibration is preserved), FOV carried from the placement
   because an in-app recording has no optical zoom. Reported honestly in
   the flash. A railed or low-confidence sync (conf < 0.45 or |offset| >
   1.9 s) is now rejected rather than fused on — the first field clip
   produced offset -2.0 s at rms 20 deg, i.e. the search hit its own limit.
   DIVISION OF LABOUR (do not blur it): gravity is absolute and drift-free
   so the sensors own MOTION; the compass is biased so they never own
   absolute pointing — vision keeps the absolute frame. Asserted: sync
   recovers a 0.42 s offset through a 37 deg compass bias, a frozen run is
   carried to within 0.25 deg of truth (vs 10.4 deg frozen), solved frames
   come through bit-identical (no bias leak), and an over-long gap stays
   held. TRADE-OFF stated in the UI: getUserMedia records ~1080p with no
   lens switching or optical zoom, so this is a MEASUREMENT mode, not the
   way to shoot the best-looking evidence. UNTESTED ON A DEVICE — the
   sensor half cannot be exercised in CI.
   **💾 SAVE TO CAMERA ROLL: DONE (as far as the platform allows)** — an in-app
   recording lives ONLY in the app's IndexedDB, so clearing site data destroys
   the footage. No browser can write to the Photos library, so `saveToRoll`
   hands the File to `navigator.share({files})` — on iOS the sheet's "Save
   Video" lands it in the camera roll — and falls back to a download (Files ›
   Share › Save Video) where file sharing is unsupported. NOT automatic and
   can't be: share() needs a live user gesture and the recording finalises
   asynchronously after the Stop tap, so the gesture is already spent. The
   primary path passes `fileRef.current` with NO await before share(), which
   is what keeps the gesture valid; only a post-reload source reads the blob
   back first. Marked `source.rollSaved` on a non-aborted share; an unsaved
   in-app clip is nagged in amber. Format honesty: Photos accepts mp4/mov and
   refuses webm — Safari's recorder picks mp4 first so an iPhone clip is fine,
   but a Chrome/Firefox recording is webm and the message says so instead of
   telling the user to tap a save that would fail.

**✂ NON-DESTRUCTIVE CLIP TRIM (`source.trim = {t0,t1}`): DONE** — measure-step
   handles (drag either end like the native iPhone editor; ⟨ start / end ⟩ set
   the edge at the playhead; ⤢ full restores). The pixels are NEVER re-encoded
   — `trimOf(src, dur)` returns the kept span and every consumer restricts what
   it SAMPLES: the frame scrubber's min/max and its ticks, the stabilize walk's
   `times` (and its cadence divisor), the preview path, the export (it follows
   the path), the report (which states the analysed span). So a trim is free,
   reversible, and cannot lose data. Both reference frames are clamped INTO the
   span (a pose on a discarded frame describes nothing) and waypoints outside it
   are ignored by the guide, followPath, the trajectory overlay and the
   waypoint snap. This is a real quality lever, not tidiness: the whip-pan while
   the phone comes up and the blurred tail are exactly the frames that break
   the tracker.
   **TRIM UI v2 (field asks + a diagnosed "glitch")**: ✂ is a MODE toggled from
   the media row (the bar costs vertical space); pressing ANYWHERE on the bar
   pulls the NEAREST end to your finger — including the dimmed trimmed-away
   region, which is how a clipped end comes back — and the handles are 44 px
   hit zones (the 30 px ones were genuinely un-grabbable on a phone, which is
   why drags appeared to do nothing). All five media controls (Replace · 💾 ·
   ⟳ · 📷 · ✂) share ONE nowrap row; the tap-mode selector was moved to its
   own line because with nowrap it overflowed.
   **TRIM DRAG SMOOTHNESS (invariant #7, violated then fixed)**: the first
   version called `setTrim()` — a source write + autosave + a video seek — on
   EVERY pointermove, i.e. ~120 of each per second on a 120 Hz phone, which is
   exactly the flood invariant #7 exists to prevent ("the trim sliders don't
   really work, or at least not smoothly"). Now a gesture moves only
   `dragTrim` (display state, rAF-coalesced), seeks are paced at ~90 ms, and
   the source is written ONCE on release. Measured: 40 pointermoves → 0 writes
   during, 1 on release, 8 seeks. `onPointerLeave` was ALSO removed from the
   end-of-gesture handlers — the bar is 38 px tall, a finger strays off it
   constantly, and aborting there was the other half of the jank; pointerup /
   pointercancel are the real end conditions.
   **"The trim won't toggle off"**: it did close — but the ✂ button stayed
   amber with a ✓ whenever a trim existed, so open and closed looked
   IDENTICAL.
   **"The clip toggle is inverted" → ✂ IS A PLAIN TOGGLE. ONE signal, the
   highlight: lit while the trim editor is open, unlit while it's shut, in every
   trim state.** No glyph badges, no second colour. Two rejected attempts, both
   worth not repeating: (a) a "✕" for *tap to close* (an ACTION) sharing the
   label slot with a "✓" for *a trim exists* (a STATE) — sharing a slot they can
   only contradict, so ✓ showed ONLY while closed and ✕ ONLY while open, exactly
   backwards from every other use of those glyphs, and a real trim went
   invisible behind the ✕; (b) dropping the ✕ but keeping the ✓ plus a teal
   outline for trimmed-but-closed — still wrong, because that is a HIGHLIGHT on
   the OFF state. A mode button answers one question (is the mode on) and the
   highlight is the whole answer; the kept span belongs in the tooltip and the
   bar's own readout, not smuggled into the toggle. Asserted by MEANING in the
   harness — no ✕ in any state, a bare "✂" label, and exactly TWO distinct looks
   across all four (open/closed × trimmed/untrimmed) — rather than against
   whatever label happens to be in the source.
   **"I lost some of the first bit I didn't clip off" — DIAGNOSED, NOT A DATA
   BUG**: a SOLVED path describes the span it was solved over, and `solvedPath`
   fed playback/export straight from `source.posePath`. So widening a trim after
   stabilizing could not extend playback (dragging the handle out "didn't
   recover" anything) and narrowing it still played discarded frames. Fixes:
   playback/export now RESTRICT the solved path to the kept span (the stored
   path keeps every sample — the trim stays non-destructive), and when the solve
   is NARROWER than the kept span both the measure step and the playback row say
   so and point at re-stabilize, instead of leaving it to read as corruption.
   **✕ CLEAR STABILIZATION + ⟳ RE-ATTACH: DONE** — `clearStab` drops every
   solved artefact (posePath/objPath/raws/poseFixes/sensorSync/preStab + the
   cached render) and returns to the original clip with all MEASUREMENTS intact;
   ↶ Undo remains the one-level "back to the previous solve". `ingestFile`
   gained `opts.keep`: re-attach the same clip (⟳ button) and the object
   placement, waypoints, alignment frame, sky placement and motion log all
   survive while only the solved paths are dropped — which is also the automatic
   behaviour when media arrives for a source that already HAS work and no
   `mediaUrl` (an imported sighting: the share file carries every measurement
   but never the video, and the old code wiped the marks on upload).
   **Playback row is ONE-MODE-AT-A-TIME** — ⚓ / 🎛 / ⬇ each open a panel below
   the row, so each now closes the other two, and ⬇ sits at the FAR RIGHT
   (field asks). ⬇ works on any playable path; ⚓ / 🎛 need a solved one.
5. Rolling-shutter per-row pose correction; OIS/EIS = slowly-varying FOV term
   in the solve. Endgame: PWA/native capture logging gyro @100+ Hz, fused
   with absolute references (complementary filter).
First video ground truth: tripod clip of an airliner + ADS-B replay.
Prerequisite discipline: keep pose/projection math pure & frame-agnostic
through the module split; allow track points an optional per-frame pose field.

## Cross-platform: the two things iOS does differently
The app is developed on an iPhone 14 and that path is FIELD-CALIBRATED — do not
touch it. But two of its assumptions are iOS-only, and each now has a tested
alternative rather than a silent wrong answer:

1. **Attitude.** iOS gives `webkitCompassHeading` (already tilt-compensated to
   the CAMERA in portrait) and an `accelerationIncludingGravity` that points
   ALONG the pull (flat, screen up → z ≈ −9.8). Android gives NEITHER: the
   compass-referenced angle arrives on **`deviceorientationabsolute`** (plain
   `deviceorientation` there is RELATIVE to wherever the phone happened to be —
   its alpha is not a bearing at all), that alpha is rotation about the DEVICE's
   own Z axis rather than a camera heading, and the accelerometer follows the
   W3C convention, which is the exact OPPOSITE sign (z ≈ +9.8) — so feeding it
   to the iOS math returns a NEGATED elevation and a 180°-rotated roll.
   - `poseFromOrientation(α,β,γ)` (capture/pose.js) solves the camera pose from
     the rotation matrix R = Rz(α)Rx(β)Ry(γ): camera axis = −(third column),
     bearing = its horizontal angle CW from north. No accelerometer involved.
     Asserted against an explicit matrix multiply over 2000 random holds.
   - `gravitySign(g, β, γ)` detects the accelerometer convention AT RUNTIME by
     comparing the reading against the orientation angles (which mean the same
     thing everywhere) — **never** by sniffing the user agent, which is a guess
     about a physical convention. Returns 0 while the phone is being swung;
     the caller keeps the last non-zero answer.
   - `poseFromGravity` gained `opts.headingIsCamera` so the non-iOS path can say
     "this heading is already the camera's" and skip the landscape regime
     correction, which exists ONLY to undo iOS's own tilt compensation.
   - SensorCapture branches on `modeRef` (`"ios"` / `"orient"` / null) and both
     branches are driven END-TO-END by `npm run capcheck`, which now runs twice:
     the iOS stub must still come out on the iOS path, and the Android stub must
     recover exactly the pose its angles imply (verified: az 324.9 / el +10 /
     roll 0.9, with the raw block recording `mode:"orient", gSign:−1`).
   - HONEST CAVEAT, deliberately not "fixed": the Android bearing may be
     MAGNETIC. iOS's is true north; the orientation sensor elsewhere may or may
     not have declination applied and there is no way to ask. Auto-correcting
     would double-correct on devices that already did it, so `poseQuality` says
     so and the sky view calibration is the answer. The pose was only ever a seed.
2. **Storage.** `localStorage` is not always there: Safari private browsing has
   thrown on write, and Firefox with site data off (plus some webviews) throws
   on the property ACCESS. Unguarded that is a BLANK APP, not a degraded one —
   the exception fires during module init before anything renders.
   `storageShim.js` probes once and falls back to an in-memory map, setting
   `window.storageVolatile` so WizHome can warn that nothing survives a reload.
   **Every reference to window.localStorage must stay INSIDE that try/catch** —
   the first version touched it once more outside, to compare, which threw in
   exactly the case it existed to handle and left `window.storage` undefined.
   `npm run storecheck` exists because of that and would have caught it.

Everything else was already feature-detected with a real fallback (WebCodecs →
MediaRecorder, `requestVideoFrameCallback` → timeout, Wake Lock / Web Share →
nothing). The one UA sniff in the codebase is the iOS export-size cap, which is
a genuine hardware limit. Media decode is the remaining unavoidable gap: HEIC
and HEVC are Safari-only, so an iPhone original handed to an Android or desktop
witness simply won't decode — `onMediaError` now NAMES that instead of blaming
the file generically.

## Style
Functional React, hooks only, no state libraries. The entire aesthetic (night
instrument: `--bg #070B14`, amber inputs, teal computed values, mono readouts)
lives in the `css` template string — keep it. No heavy dependencies without
discussion; the hand-rolled EXIF parser, zip writer, and 3D projector are
features, not oversights. Preserve honest epistemics everywhere: warnings over
silent guesses, caveats in reports, "quality: poor" when it's poor.
