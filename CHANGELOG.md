# Changelog

Notable changes to Phodar. Format loosely follows
[Keep a Changelog](https://keepachangelog.com/); versions are the ones in
`package.json`.

## [Unreleased]

### Added
- **Headless analysis engine + API access** (`src/analyze/engine.js`,
  `scripts/analyze.mjs`, `POST /api/analyze` — see `docs/API.md`): the full
  results pipeline with no UI. Feed it a session's measurements (the app's
  own .phodar.json share format) plus optionally a drone flight-log CSV and
  get one machine-readable verdict: fix, visibility- and clock-aware stereo,
  dense two-video stereo, flight-log calibration grades, per-witness clock
  checks, and every honesty warning as data. Key-gated via PHODAR_API_KEYS
  (endpoint disabled until set). Deterministic and mathcheck-asserted end to
  end; raw-media ingestion and an agentic analyst layer are the documented
  next phases.
- **MCP server (`/mcp/<key>`)**: the engine as Model Context Protocol tools
  (`analyze_session`, `parse_flight_log`), so users analyze sightings with
  their OWN AI subscription — Claude, ChatGPT (custom connectors / Agents
  SDK), Gemini and most agent frameworks all speak MCP's Streamable-HTTP
  transport. Hand-rolled stateless JSON-RPC (no dependencies), key in the
  URL path because every client can paste a URL while header auth varies.
  Full client lifecycle (initialize → tools/list → tools/call, auth and
  error paths) verified against the real two-video drone session.

### Added (quality of life)
- **Cube depth slider — the monolith axis** (field ask): the ⬛ cube gains a
  third independent proportion, `depth` (0.1–3×), thinning the footprint's
  second axis against its fixed width — so a true rectangular slab is now
  reachable, not just square boxes and columns. A live mono readout beside
  the slider shows the solid's proportions (thinnest side = 1) so a specific
  ratio can be dialled exactly; the classic 2001 monolith (1 : 4 : 9) is
  depth 0.25 + stretch 2.25. Composes cleanly with squash (thin diamonds
  work too); existing fits are unchanged (default 1). Mathcheck-asserted;
  the report's dimensioned 3-view picks the new extents up automatically.
- **Trajectory playback on the top-down plot**: when the stereo trajectory
  solves, the satellite view gains ▶ + a scrubber — a marker rides the
  triangulated path in real time with a growing progress trail, and the
  readout shows elapsed/clock time, altitude and speed at the scrubbed
  instant.
- **Sighting name**: an optional name field on the home screen that becomes
  the report's title/header and the filename of every export — report .html,
  .phodar.json share file, and the .zip bundle.
- **Video-analysis section: witness estimate + real charts** (field ask):
  the assumed-distance ladder gains the witness's own estimate as an
  emphasized amber row slotted in order ("120 m (witness estimate)" with its
  implied size/speed/path in bold), called out in the caption. The
  angular-rate plot grew from a bare sparkline to a real chart — labeled
  time and °/s grid, the peak flagged at its moment, the average as a
  dashed line — and the stereo speed + felt-load strip got the same
  treatment (labeled time axis, speed ticks left, g ticks right, peaks
  marked). All verified by rendering the real report in a browser against
  the Germany session.
- **Size ⇄ distance chart overhaul** (field ask): bigger (760×400), with a
  full labeled log grid on both axes (decades plus 2×/5× minors), so values
  can be read off it rather than eyeballed. And the assumed distance set in
  the sky view's 📏 size tool (slider or 📍 map pick) now persists on the
  observer as their own range estimate — the report chart draws it as an
  emphasized amber vertical with dots where it crosses the size and altitude
  lines and the implied numbers spelled out ("witness estimate — 394 ft →
  5.1 ft across · 225 ft above you"), plus a caption sentence. Verified by
  rendering the real report in a browser against the Germany field session.

### Added (quality of life)
- **🎥 Track-quality rating** (field ask): a camera-motion risk rating
  (excellent/good/fair/poor) rendered in the reporting surfaces — above the
  results screen's trajectory section and at the top of the report's video
  analysis (kept out of the sky view itself, which it cluttered) — naming
  its reasons: what share of the clip sits in hard zooms /
  anchor-starved stretches where camera motion can read as object motion
  (already excluded from reported peaks), and whether the tracker-noise
  floor rivals the measured motion. Silence means a clean track. The
  Germany field clip rates "fair — 39% masked". Pure + mathcheck-asserted
  (`trackQuality` in the math core).

### Fixed
- **The phantom "sharp turn" during a zoom is now masked and named** (field
  report: "the sharp turn never happens in the video — it glides like a
  balloon"): the residual 4.4°/s peak sat exactly inside a 3.3× zoom-in
  where the camera solve ran on 6-8 background anchors and held the
  camera's pointing frozen while only FOV updated — but an operator zooming
  in always tilts to re-center the subject, and that unmodeled tilt was
  booked as the object diving. That is a sustained BIAS no smoothing can
  remove, so it gets a reliability mask instead: spans where the solve had
  <9 anchors or the FOV slewed >5°/s are excluded from the reported peak
  rate and peak speed, shaded on the rate chart, and the report says what
  was excluded and why (the excursion stays visible as peakOmegaAll —
  masked, not hidden). Also: the range profile between sparse size
  keyframes now interpolates at constant radial velocity (linear range in
  time) — interpolating the ANGLE linearly implied an accelerating
  recession (dR/dt ∝ 1/ang²) and manufactured a ~170 mph end-of-gap spike.
  On the field clip the reported peak fell 4.4→2.9°/s and the implied peak
  at 120 m 910→139 mph across the whole fix series, with the remainder
  traceable to the measured tail recession. Mathcheck-asserted: the
  phantom zoom-span ramp is excluded, a real maneuver on a clean solve
  keeps its true peak, and the excursion is still returned honestly.
- **Angular rate and implied speeds no longer inflated by tracker noise**
  (field report: a balloon-smooth object showed a 9°/s spike and a
  ~900 mph implied peak). Two causes, both fixed in the math core:
  (1) the rate was a raw adjacent-frame difference, which multiplies every
  per-frame pose/tracker error by 1/dt — and both trackers are noisiest
  exactly while the camera pans or zooms. ω now comes from a weighted
  windowed fit: samples are down-weighted by camera angular speed, zoom
  rate (from the solved posePath) and low match confidence, the window
  widens where the data is noisy, and the sweep is the integral of the
  fitted rate so jitter can't random-walk it upward. A real sustained
  maneuver is a ramp across many samples and passes through; a one-frame
  excursion dies. (2) Two size keyframes a fraction of a second apart (a
  sizing repeat — the field file had 0.957° and 0.714° just 0.11 s apart)
  differentiated into ~300 m/s of phantom radial speed; near-coincident
  keyframes are now merged and the range profile smoothed before
  differencing. The report also states the estimated tracker-noise floor
  ("rate variations below ≈0.4°/s are within tracker noise"). On the real
  clip: peak 7.3→4.5°/s, implied peak at 120 m 910→169 mph. All
  mathcheck-asserted: zoom-window noise rejected, a sustained maneuver
  preserved to its true value, the sizing repeat neutralized, clean
  constant-rate clips bit-exact as before.
- **Range ratio no longer inflated by stale size stamps** (found auditing the
  same field file): sizing a track point stamps its angular size through the
  BASE FOV, and the stamp was only re-derived on placement commit — so a
  point re-sized on a zoomed frame after the last commit kept a value wrong
  by the zoom ratio, and the report's "how much closer did it get" math read
  it (the Germany clip reported a 15.25× range change; the true figure is
  5.3×). The math core now re-derives every sized point's angular size from
  its wpx through the solved per-frame FOV at the mouth of each consumer
  (sourceTrack / videoKinematics / mixedStereo), and the size panel stamps
  through the frame's own FOV in the first place. Un-stabilized and tripod
  sessions are unchanged. Mathcheck-asserted with the field file's numbers.
- **World-view wireframe scaled wrong while scrubbing a zoomed clip** (field
  report, Germany sighting): per-point size keyframes store the object's
  width in PIXELS on their own frame — and on a zooming clip (this one swept
  46°→5° FOV, 9.25×) the same angular size is ~9× more pixels zoomed in.
  Playback and the world-locked export interpolated those raw pixels and
  projected them through the marked frame's lens, so the 3D model ballooned
  up to ~8× too big around zoomed-in keyframes. Sizes are now normalized to
  one pixel scale through each keyframe's own solved FOV (angular size is
  the invariant) before interpolating — in the dome, the export burn-in and
  the measure-step ghosts alike. Tripod/fixed-lens clips are byte-identical.
  Mathcheck-asserted, including a regression straight from the field clip's
  numbers.
- **Videos vanishing from the installed app while points survive** (field
  report: "the PWA goes stale — no video loading from memory even though
  track points and other data are there"): the sighting metadata lives in
  localStorage (tiny, survives), but the clips live in IndexedDB — which,
  as best-effort storage, is exactly what iOS reclaims under disk pressure,
  hundreds-of-MB video blobs first. Three defenses now: (1) the app
  requests **persistent storage** on boot (`navigator.storage.persist()` —
  granted automatically for a home-screen PWA), so the media store stops
  being first in line for eviction; (2) a **refused write is reported at
  upload time** (storage full / private mode) instead of surfacing days
  later as silent data loss; (3) when a clip IS found missing on boot, the
  measure step **says exactly what happened** — and that re-attaching the
  same file keeps every point, mark and placement (the keep-your-work
  re-attach already existed; now you're told about it). The IndexedDB open
  is also watchdogged and retried, since iOS has been seen hanging it
  after a cold relaunch. Boot flow verified in a real browser (missing
  record → notice + flag persists across reloads; record present → clip
  re-attaches, flag clears).
- **API/MCP: lean sessions no longer crash the solver** (found by a smoke
  test of the API stack): the app always creates `A`/`B` objects on every
  source, so `analyze()` read `s.A.az` / `s.B.az` unguarded — an external
  API or MCP caller sending a minimal session without those objects got
  "analysis failed: Cannot read properties of undefined" instead of a
  verdict. Both reads are now optional-chained; a source with no B still
  solves the fix, a source with no A filters out with a named gap.
  Mathcheck-asserted with lean sessions.
- **The loupe now frames the whole shape with breathing room** (field
  report): the magnifier's zoom was derived from the shape's nominal size,
  but a stretched monolith, a balloon's string or a tilted attitude projects
  well past that (and can sit off-centre of the anchor point), so at certain
  proportions the wireframe fell off the glass. The zoom is now computed
  from the real projected extent about the loupe centre, fit to 80% of the
  glass radius — the outline always lands inside with a visible margin, so
  you can tell where it sits as it scales.
- **Adjust-mode size/tilt controls moved up beside the image** (field ask):
  the per-point size slider and attitude buttons sat at the bottom of the
  Track panel, below the mode toggle and colour rows — so on a phone,
  adjusting meant scrolling away from the very image you're matching the
  outline against. The panel now renders directly under the image and frame
  row (its own card), so the outline, the frame steppers and the size/tilt
  controls share one screen. The Track panel keeps the place/adjust toggle,
  colour and point management.
- **Track-point size ghost now scales about the point's centre** (field
  report): the wireframe drawn while sizing a track point was pinned by the
  midpoint of its widest silhouette chord, which is NOT the visual centre
  for asymmetric shapes or tilted attitudes (a saucer's dome, a balloon's
  string all hang off that chord) — so the outline slid off the point as it
  grew. It is now pinned by its APPARENT centre (the bounding box of the
  drawn curves), on the photo and in the loupe alike, so scaling grows the
  outline symmetrically about the placed point. Mathcheck-asserted.
- **Sizing/tilting a track point snaps the video to that point's frame**
  (field ask): the size + tilt controls target the nearest placed point in
  time, but the scrubber could be parked on any frame — so the outline was
  being matched against the wrong frame's pixels. First touch of either
  control now jumps the video to the targeted point's own frame (with a
  brief note saying so), the same guard philosophy the 3D object's frame
  already has.
- **All videos rendering black on the installed app (iPad field report)**:
  iOS keeps a live decoder pipeline for every <video> that ever loaded a
  source and caps those per page — a long multi-observer session saturates
  the cap, after which every clip (even a fresh upload) renders black until
  the web process restarts (Chrome has no such cap, hence "works on
  Chrome"). The offscreen videos already released their pipelines; the
  measure step's rendered player now does too, explicitly, when the clip
  changes or the step closes. If the app is currently in the all-black
  state, force-quit it (or restart the device) once — the fix prevents
  recurrence.
- **A freshly uploaded clip no longer auto-plays** (field ask): the
  first-frame paint kick now pauses at the first PRESENTED frame
  (requestVideoFrameCallback) instead of play-then-pause, which visibly ran
  the clip on some devices. A ▶ / ⏸ button joins the frame controls so the
  clip can be previewed without dragging the scrubber; a user-started play
  is never touched by the paint kick.
- **Adjusting the placed 3D object can no longer corrupt its frame** (field
  ask): the object belongs to the frame it was fitted on, but every shape
  touch re-stamped that frame to wherever the video was scrubbed — one stray
  tap while browsing frames silently moved the measurement. Now adjusting an
  existing fit while off its frame snaps the video back to it (on tool
  re-entry and as a gesture guard); deliberately moving the object to the
  current frame is the explicit 📌 "Object → this frame" button, which keeps
  the fit's size/rotation and warns to re-run Stabilize when the auto-track
  was seeded on the old frame. First placement is unchanged — any frame.
  Verified by driving the real app in a browser.
- **The measure-step loupe now follows brightness/contrast live** (field
  report: "the loupe shows the version before adjustment"). The loupe is a
  canvas, so nothing repainted it when the B/C sliders moved while it was on
  screen. It now re-pops on every B/C change while a small object is fitted
  — exactly the moment the magnified view matters, since the sliders exist
  to make a dim object visible. Reproduced and verified fixed by driving the
  real app in a browser (a gray frame under 200% brightness must read
  doubled in the loupe's own pixels).
- **A panning camera no longer fabricates object speed**: pixel waypoints on
  a stabilized clip now convert through each frame's OWN solved pose instead
  of assuming the camera never moved (a tripod clip without a solved path
  keeps the static assumption — which is then actually true). Measured
  against a drone's flight log with one handheld and one tripod camera: the
  handheld pan added ~10 mph of phantom speed (32 measured vs 21 logged);
  with per-frame poses the speed profile tracks the log (peak 26.0 vs
  logged 28.4 mph, per-instant ratio 0.85), the ray miss tightened
  0.29 → 0.19 m and the absolute clock residual fell to 1.3 s. Kinematics
  are also gap-aware: path, average speed and acceleration use measured
  segments only — never the straight-line jump across a visibility hole
  (which used to masquerade as a slow "average").
- **Witness clocks are now aligned by the object's own motion** (field case:
  one video's capture time was recorded ~20 min wrong in-app; hand-corrected
  to the minute it still sat ~41 s off — proven against the drone's flight
  log). Track stereo now (1) anchors each witness's track on its capture
  time (whenMs + video t) instead of silently aligning recording starts,
  (2) searches ±45 s for the relative offset where the sight-lines sharply
  intersect and adopts it only when the minimum is decisive — a hovering
  object (flat minimum) never gets a fabricated shift — and (3) when the
  tracks don't overlap at all, runs a wide ±30 min rescue sweep that
  recovers the 20-minute class of error. Applied shifts are declared in the
  trajectory section and the report. On the real two-video session this
  recovered a 12 s relative error and dropped the ray miss to 0.29 m. The
  drone flight-log check gains a per-witness ⏱ clock check (one sight-line
  against the whole flight) that pins clocks absolutely — it exposed the
  41 s residual at 0.13° sharp.
- **Intermittent visibility no longer poisons triangulation** (field case: a
  drone visible only in sections of each of two videos, path captured where
  possible). Interpolating a witness's direction across a visibility hole
  fabricates a ray nobody observed, and the stereo triangulation consumed it
  wherever the other witness's real data fell inside the hole. Both stereo
  pipelines (waypoint tracks and dense two-video) now build per-witness
  visibility segments (a gap ≳4× the track's own cadence is a break), drop
  low-confidence tracker samples (held/guided predictions) first, triangulate
  only instants inside every witness's segments, and report how many seconds
  of shared visibility were used vs ignored. Disjoint visibility is named
  ("never both see the object at the same moment") instead of erroring
  ambiguously. Asserted in mathcheck with a truth path that turns sharply
  inside one witness's blind stretch.
- **Twitchy object close-up exports**: the close-up's per-frame pixel pin
  gated its own correct finds against the solved track — the very thing the
  pin exists to correct — so ~1° of track error made it reject the lock and
  ease the camera back onto the bad track (field clip: the object wandered
  ±20% of the frame and left it entirely). The pin policy (now `pinStep`,
  pure and regression-tested against synthetic frames driven through the
  real detector) trusts the locked pixel chain, world-holds brief fades, and
  glides — never snaps — back to the track after a loss. In the harness a
  poor track went from 380 px rms object wander with losses to 41 px, the
  same as a good track.
- **Bundle import now brings the videos back** (field report: a 160 MB
  two-video bundle opened on another device with no videos). The bundle
  always carried each observer's clips; the importer only ever extracted the
  measurement JSON. Importing a .zip now re-attaches each observer's original
  clip (and any stabilized render) to the imported sources.
- **"Single viewpoint" with two witnesses on screen now explains itself**
  (field report): the fix status, report screen and drone-calibration check
  name which observer is incomplete and what's missing — its position
  (step 2) or its sky placement (open the sky view and tap ✓ Set A) — instead
  of just counting viewpoints. The calibration check also renders that
  guidance instead of silently showing nothing when no observer is complete.
- **Bundle save "sometimes works" in the installed app (iPad field report)**:
  packing the zip consumes the tap's user activation, and a late automatic
  `navigator.share` sometimes opened and sometimes failed indistinguishably
  from a user cancel. The installed app no longer gambles: pack finishes,
  then the 💾 Save bundle button always appears — its tap is a fresh gesture,
  so the share sheet opens reliably every time.
- **Stray "text" file beside share-sheet saves**: passing a `title` alongside
  `files` to `navigator.share` makes iOS "Save to Files" write the title out
  as a separate 22-byte text file (field report). All file shares now pass
  files only.
- **Bundle download in the installed PWA**: a home-screen web app has no
  browser download manager, so the .zip's `<a download>` click was a silent
  no-op — the app said "✓ downloading bundle" while nothing was saved (field
  report). In standalone mode the bundle now goes to the OS share sheet
  ("Save to Files" / AirDrop); if the zip build consumed the tap's user
  activation and share is refused, a 💾 Save bundle button holds the packed
  zip for a fresh gesture instead of pretending it saved.

### Added
- **Drone flight-log check (calibration)**: a ground-truth test harness,
  deliberately unobtrusive — it lives behind a small 🛩 calibration link at
  the foot of the results step. Upload your own drone's flight
  record (Airdata CSV export, decoded DJI Fly CSV, or DJI
  video .SRT captions) and every output — direction, fix position, altitude,
  true size, speed, heading — is graded against the craft's logged GPS truth.
  Whole-log time sync (clock/timezone mismatches are found by geometry and
  reported, never hidden), honest altitude datums for height-above-takeoff
  logs, DJI Mavic Mini / Neo span presets, a "Drone flight-log ground truth"
  report section, and `docs/DRONE-TEST.md` — the field protocol for flying a
  calibration test. Parsing, prediction, sync and grading are pure
  (`src/checks/flightlog.js`) and asserted in mathcheck against a synthetic
  flight with exact ground truth. Field-validated on a real Mavic Mini
  flight: fix within 2.19 m of the logged position at 57–100 m ranges
  (docs/FIELD-TESTS.md Case 5). Witnesses whose photos were taken at
  different moments are additionally graded against the drone's position at
  each photo's own time (that flight's photos were 122 s apart).

## [0.9.0] — 2026-07-26

First public release. Everything below is what the app can do at its debut, not
a list of changes against a previous version.

### Measurement
- EXIF and QuickTime metadata parsing (hand-rolled): GPS, time, compass bearing,
  35 mm focal → field of view, orientation. QuickTime is scanned at both ends of
  the file, since iPhones write the `moov` atom last.
- Magnetic bearings corrected to true via embedded NOAA WMM2025, validated
  against all 100 official test vectors.
- 3D wireframe shape fitting (orb, saucer, tic-tac, triangle, plane, bird) that
  records the object's pose; angular size from marked edges.
- Least-squares triangulation in a local ENU frame with quality grading, ray-miss
  diagnostics, size-ratio bearing arbitration and altitude-spread warnings.
- N-view foreshortening solve for the true span and long-axis heading of
  elongated objects.
- Trajectory kinematics: speeds, accelerations, felt g-loads, turn rates.
- Single-witness size↔distance analysis.

### Calibration
- Sky view rendering the real sky: stars to mag 5, planets (validated against
  JPL Horizons to ~0.01°), Sun and Moon, over the photo.
- DEM terrain skyline with layered interior ridges, sea-level clamping and
  one-tap **snap to ridges**.
- **Auto star-align**: local plate solve, seedless when the EXIF field of view
  survived, robust to clouds and non-catalog blobs.
- OSM named peaks and 3D building boxes as alignment landmarks.

### Video
- Per-frame camera pose solving: whole-frame registration against a reference
  frame, sparse feature tracking, zoom detection, absolute re-anchoring, despike
  and evidence-weighted smoothing.
- Automatic object tracking into a dense time-stamped angular path, guided by
  hand-tapped waypoints when present.
- Manual pose correction (**⚓ Fix frames**) with anchor interpolation, composing
  non-destructively with the smoothing controls.
- World-locked video export in three framings (annotated view, clean
  max-resolution, object close-up), via WebCodecs with a hand-rolled MP4 muxer
  and a MediaRecorder fallback.
- Two-video dense stereo triangulation with automatic clock sync; mixed
  video-plus-still triangulation.
- Instrumented capture: in-app recording with a synchronized device-attitude log,
  fused with the visual solve.

### Cross-checks
- ADS-B aircraft, live (four networks merged) and historical (tar1090 archives,
  ~2 years back), with sky tracks and type→wingspan angular-size prediction.
- Satellites and Starlink from CelesTrak TLEs with Earth-shadow lit tests.
- Sun, Moon, planets and bright stars near the sight-line, with a Venus warning.
- Winds aloft at the fix altitude with a balloon verdict.
- Rocket launches, CNEOS fireballs, meteor showers, nearby aerodromes.

### Reporting
- Self-contained HTML white paper with embedded photo exhibits and detail crops,
  a top-down plot on satellite imagery, speed and felt-g charts, video
  kinematics with keyframe strips, and every cross-check with its caveats.
- `.phodar.json` share files and `.zip` bundles; a second witness imports either
  and adds their perspective.

### Validation
- Ground truth: a rooftop weathervane resolved to within an inch of its true span
  from two phone photos 14 m apart. See `docs/FIELD-TESTS.md`.
- `npm test` asserts the math core against exact synthetic truth, importing the
  shipped modules.
