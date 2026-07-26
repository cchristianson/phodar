# Changelog

Notable changes to Phodar. Format loosely follows
[Keep a Changelog](https://keepachangelog.com/); versions are the ones in
`package.json`.

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
