# Architecture

A map of the codebase for someone who has just cloned it. `CLAUDE.md` is the
deep log — invariants, field findings, and every approach that was tried and
reverted, with the measurements that killed it. Read this first, then that.

## Shape of the thing

A React SPA built by Vite, plus a small dependency-free Node proxy. No state
library, no CSS framework, no UI kit. Runtime dependencies are `react`,
`react-dom`, `leaflet` and `satellite.js` — that's the whole list. The EXIF
parser, zip writer, MP4 muxer, geodesy, 3D projector, plate solver and video
pose tracker are all hand-rolled, which is why `npm test` needs no dependencies
at all and the bundle stays small.

```
src/
  phodar.jsx     ~11k lines — the app: all React components, the wizard,
                 the report generator. Organized by banner comments.
  math/          pure ES modules, no React, imported by the app AND the tests
  checks/        one module per mundane-explanation cross-check
  video/         per-frame pose solving, manual correction, MP4 muxing
  capture/       device-sensor attitude (gravity + compass)
  exif.js  shapes.js  terrain.js  buildings.js
  mediaStore.js  storageShim.js
server/index.mjs  static host + CORS proxy (see docs/SERVER.md)
scripts/          test + verification harnesses
```

### Why `phodar.jsx` is one enormous file

It was developed as a single Claude artifact, and the extraction is deliberate
and incremental: the **math core came out first** (it's what the tests need to
import), then the EXIF parser and shape system. What remains is React —
components with heavy shared state. The remaining seams are listed in
`CONTRIBUTING.md`; they're mechanical, and the rule is behavior-identical, one
seam per pull request.

Inside the file, find things by banner comment:

| Banner | What lives there |
| --- | --- |
| UI ATOMS | The shared primitives and the `css` template string (the entire look) |
| HELP / GUIDE | The in-app manual |
| MEDIA MEASURE | Upload → canvas normalize → tap-to-mark → shape fitting, pinch-zoom, loupe, brightness/contrast, video frame selection and track waypoints |
| SKY AIMER | The sky view: place/look modes, the canvas mesh warp, every sky layer, video stabilize/playback/export, ⚓ fix-frames |
| PIN MAP / POSITION EDITOR | The Leaflet map step, bearing ray, geocoding |
| SENSOR CAPTURE | The in-app instrumented camera |
| AERIAL MEASURE | The looking-down workflow |
| PLOT BOARD / ADS-B CHECK / RESULTS PANEL | Result presentation |
| WIZARD | `WizHome/WizStep/WizFinish`, `reportHtml`, share JSON, zip writer, import |

## The math core (`src/math/`)

Pure, frame-agnostic, and **imported by the test scripts** — the tests used to
re-implement the math, which meant a bug in the shipped code couldn't fail
them. That's fixed and must stay fixed.

| Module | Contents |
| --- | --- |
| `geodesy.js` | Constants, vector ops, ENU ↔ geodetic, az/el ↔ direction vector |
| `projection.js` | Focal length in pixels, photo basis, angular size from marked points, pixel → direction, homography. **The pixel/pose functions take a pose *sample*, not a per-source constant** — that's what makes video possible |
| `triangulate.js` | `solve3`, `intersectLines`, `analyze`, `arbitrateBearings`, `aspectSpan` |
| `kinematics.js` | Track directions, speeds/accelerations/felt-g, `videoKinematics`, `stereoVideo`, `mixedStereo` |
| `astro.js` | Sun/Moon (SunCalc-derived), `raDecToAzEl` |
| `starcat.js` / `starcatDeep.js` / `planets.js` | Baked catalogs and ephemeris — no network at runtime |
| `geomag.js` | WMM2025, magnetic → true bearing |
| `format.js` | Number and unit formatting |
| `geolocate.js` | Ground-plane geolocation for the aerial workflow |

## Data flow

### One photo

```
file → parseMediaMeta (exif.js)        GPS, time, compass, lens FOV, orientation
     → canvas normalize                EXIF baked out, capped to iOS's canvas ceiling
     → MediaMeasure                    mark the object, fit a 3D shape → angular size + pose
     → PositionEditor                  confirm position, set the viewing-direction ray
     → SkyAimer                        align the photo against sky + terrain
     → commitPlacement                 marks + placement pose → world sight-lines A/B
```

`commitPlacement` is the hinge: everything upstream is about establishing where
the camera was pointing, everything downstream consumes the resulting
sight-lines.

### Several photos

`analyze` (triangulate.js) intersects all observers' sight-lines by least
squares in a local ENU frame. Convergence angle and rms ray-miss grade the fix;
`arbitrateBearings` uses the size ratio between views to name which compass is
the liar rather than averaging the error. Trajectories interpolate each
witness's directions to common instants, then triangulate each instant.

### Video

Video is the same pipeline with **a pose per frame** instead of one pose:

```
stabilize walk (video/postrack.js)
  reference frame ← the alignment frame's committed placement pose
  per step: registerToRef (whole-frame, absolute)
            → sparse feature track + solve (precision)
            → re-anchor against pristine reference features (kills drift)
  → despike → smooth → optional sensor fusion → source.posePath

object pass (second walk, over the finished poses)
  stepObject: predicted position + pristine-seed template match,
              guided by hand-tapped waypoints when there are ≥2
  → smoothObjPath → source.objPath = [{t, az, el, q}]

manual corrections
  source.poseFixes → applyPoseFixes (delta field over the base path)
                   → applyDirFixes (the object track follows exactly)
```

`rederivePaths` is the **single derivation point**: raw solves → smoothing →
sensor fusion → pose fixes → waypoint snapping. Every control (smoothing
sliders, anchors, re-alignment) re-runs it from the raws, so nothing is
destructive and nothing composes wrongly.

Two clips of the same object go to `stereoVideo`, which puts both on an
absolute clock, searches the offset that minimises ray-miss, and triangulates
each common instant with outlier rejection.

## Cross-checks (`src/checks/`)

Every check answers the same question in the same shape: *given the observer's
position and the sighting time, what known object would have appeared at what
direction, angular size and motion — and how far is that from the witness's
sight-line?* That uniformity is what lets the report rank aircraft, satellites,
planets, balloons, launches and fireballs in one table.

`adsb.js`, `satellites.js`, `winds.js`, `launches.js`, `fireballs.js`,
`meteorshowers.js`, `airports.js`, `peaks.js`, `photometry.js`, `weather.js`,
and `platesolve.js` (which is a calibration tool rather than a check — it
solves the camera pose from stars).

Adding one: produce that shape, register it in the results panel and the
report, and give it an honest caveat about its own staleness or coverage.

## Persistence

- `window.storage` (via `src/storageShim.js` → localStorage) holds the
  sighting. Artifact heritage; keep the contract.
- **Media never goes through it.** Autosave strips `mediaUrl`; images, videos
  and stabilized renders live in IndexedDB via `src/mediaStore.js`, keyed by
  source id and re-attached on boot.
- The `.phodar.json` share file and the report's embedded data block are the
  interchange format — a second witness imports either one and adds their
  perspective.

## Tests

`npm test` = `scripts/mathcheck.js` + `scripts/trajcheck.js`, dependency-free,
run in CI. They assert against exact synthetic truth: triangulation recovery,
ENU round-trips, angular size → span, plate-solve pose recovery through cloud
occlusion and clutter, terrain skyline geometry, video pose tracking through
zoom cycles, sensor sync and fusion, MP4 box layout, and full trajectory
kinematics of a simulated 3.5 g maneuver.

`skycheck` / `capcheck` (Playwright, not in CI) drive the real app for the
classes of bug unit tests structurally cannot see. `metacheck` runs the app's
own metadata parser over a candidate file.

## Constraints that shape everything

The app is used on an iPhone in a field at dusk. That produces most of the
non-obvious patterns in the code — canvas normalization instead of CSS
transforms, a hand-rolled mesh warp, memory caps that look arbitrary and aren't,
a touch stack with deferred gesture commit. **Every one of them is written down
in `CLAUDE.md` with the bug that caused it.** Check there before simplifying
something that looks over-built.
