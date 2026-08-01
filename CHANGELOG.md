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
- **Trajectory playback on the top-down plot**: when the stereo trajectory
  solves, the satellite view gains ▶ + a scrubber — a marker rides the
  triangulated path in real time with a growing progress trail, and the
  readout shows elapsed/clock time, altitude and speed at the scrubbed
  instant.
- **Sighting name**: an optional name field on the home screen that becomes
  the report's title/header and the filename of every export — report .html,
  .phodar.json share file, and the .zip bundle.

### Fixed
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
