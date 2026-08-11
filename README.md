# PHODAR

[![CI](https://github.com/cchristianson/phodar/actions/workflows/ci.yml/badge.svg)](https://github.com/cchristianson/phodar/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

**PHO**togrammetric **D**etection **A**nd **R**anging — turn sighting photos
and video into numbers. One witness yields honest angular data (direction,
angular size, angular motion). Two or more yield a true triangulated fix:
**position, altitude, real size, speed and heading** — with quality grading,
error diagnostics, and a shareable white-paper report that other witnesses can
import to add their own perspective.

Built mobile-first for real sighting conditions. It runs entirely in your
browser; your photos never leave your device.

## Try it

### ▶ [phodar.app](https://phodar.app/)

No account, nothing to install — open it in a phone browser. Add it to your
home screen if you want the motion-sensor capture mode; on iPhone that is the
only way the sensors are allowed to work.

[![Watch the demo](https://img.youtube.com/vi/p1pjgN36lDA/hqdefault.jpg)](https://youtu.be/p1pjgN36lDA)

**[Watch the demo](https://youtu.be/p1pjgN36lDA)** — a drone flown to a known
position, photographed from several angles, measured in the app, then checked
against the truth.

## Does it actually work?

It was validated on ground truth. Two phone photos of a rooftop weathervane
from 14 m apart:

| Quantity | Phodar | Ground truth |
|---|---|---|
| Height AGL | 7.0 m (23 ft) | 20–25 ft (owner estimate) |
| Span (aspect-solved) | **63 cm / 25″** | ~24″ |
| Compass accuracy on foot | −0.5° / +0.9° | — |

The same testing quantified the failure modes the app now warns about:
phone compasses read **14–66° wrong near metal** (inside a car, under a steel
roof), and phone GPS **altitude** wobbles ±5 m — Phodar detects both
(size-ratio bearing arbitration, per-ray miss outlier naming, altitude-spread
warnings) instead of silently producing garbage. See
[`docs/FIELD-TESTS.md`](docs/FIELD-TESTS.md).

## What it does

**Measure.** EXIF auto-fill (GPS, time, lens FOV, compass — magnetic bearings
corrected to true via NOAA WMM2025). 3D wireframe shape fitting (orb, saucer,
tic-tac, triangle, plane, bird) that records the object's **pose**, not just
its outline. Angular size from marked edges, per-segment trajectory timing, and
single-view size↔distance analysis when you're the only witness.

**Calibrate.** An astronomically anchored sky view draws the real night sky
(stars to mag 5, planets, Sun, Moon) and a DEM terrain skyline over your photo,
so you can check — and fix — where the camera was actually pointing. One tap
**snaps to ridges** by matching the photo's horizon against the terrain model;
one tap **auto star-aligns** by plate-solving against the catalog, seedless when
the EXIF field of view survived.

**Video.** Handled the same way, and then some. Phodar solves where the camera
was pointing for **every frame** — whole-frame registration against a reference
frame, sparse feature tracking, zoom detection, absolute re-anchoring — so the
sky, terrain and grid stay frozen and only the object moves. It then tracks the
object across the clip into a dense time-stamped angular path, and exports a
world-locked stabilized video in three framings (annotated view, clean
max-resolution, and a close-up crop that follows the object). Frames that lost
the lock can be corrected by hand, and the correction interpolates across its
neighbours. Two clips of the same object triangulate against each other with
automatic clock sync.

**Cross-check.** Every mundane explanation, ranked in one table: live and
**historical** ADS-B aircraft (four networks merged; archives reach ~2 years
back), satellites and Starlink from CelesTrak TLEs, Sun/Moon/planets/bright
stars near the sight-line (with a Venus-specific warning — it's the most
reported "UFO" there is), winds aloft at the fix altitude for the balloon
hypothesis, rocket launches, and CNEOS fireballs. Each check outputs the same
shape: a candidate with predicted direction, angular size and motion, plus its
angular separation from the witness sight-line.

**Report.** A self-contained HTML white paper — the fix and its error budget,
photo exhibits with detail crops, a top-down plot on satellite imagery, speed
and felt-g strip charts, video kinematics with keyframe strips, and every
cross-check with its caveats. One file, shareable, importable by the next
witness.

## Quick start

```bash
npm install
npm run dev        # Vite — open the LAN URL on your phone
npm test           # math regression suite — must pass
```

For the cross-checks that need a proxy (historical aircraft, Overpass peaks and
buildings, report basemaps), run the API beside it:

```bash
node server/index.mjs      # :8787 — vite already proxies /api here
```

Without it the app still works: the checks that depend on it degrade with a
stated caveat rather than failing. [`docs/SERVER.md`](docs/SERVER.md) has the
endpoint table.

## Deploy

```bash
npm run build
npm start          # serves dist/ AND /api from one process on $PORT
```

`railway.toml` does exactly that — push to GitHub, point a Railway project at
the repo. There are **no API keys and no environment variables** other than
`PORT`; any host running Node 18+ works the same way. Read
[the hosting notes](docs/SERVER.md#before-you-host-this-publicly) before putting
an instance on the public internet — the proxy is unthrottled and forwards to
volunteer-funded services.

## How the math works (short version)

Each photo is pixel-normalized (EXIF baked out — iOS lies otherwise) and its
FOV taken from lens metadata. The object's sky direction is fixed by aligning
the photo on an alt-azimuth grid anchored to computed Sun/Moon positions, the
star catalog, or the DEM terrain skyline. Sight-lines from all observers are
intersected by least squares in a local ENU frame; ray convergence and rms
miss grade the fix. Size = angular size × range; an N-view foreshortening
solve recovers the **true span and long-axis heading** of elongated objects
that single views understate. Trajectories interpolate each witness's
directions to common instants before triangulating each instant — speeds,
accelerations, felt g-loads and turn rates follow. For video, every frame
carries its own solved pose, so pixel motion becomes true angular motion.

Longer version: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

## Honest epistemics

This is the rule the whole project is built on: **a warning beats a silent
guess.** Phodar says "quality: poor" when the geometry is poor, names which
witness's compass looks wrong instead of averaging the error away, reports how
stale a satellite TLE was, and flags when a video frame's world lock was
carried rather than solved. [`docs/LIMITATIONS.md`](docs/LIMITATIONS.md) is the
list of things it does badly, with measured numbers.

It is a measurement tool, not an authority. Nothing it produces is
evidence-grade for aviation safety reporting, and no result is an
identification.

## Privacy

Photos and video are decoded and measured in your browser and never uploaded.
The cross-checks send a coordinate and a time to public APIs — nothing else,
and only when you use them. Details, including what a self-hosted instance
logs: [`PRIVACY.md`](PRIVACY.md).

## Contributing

Start with [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md), then `CLAUDE.md` —
the deep engineering log of invariants, field findings, and every "don't
rebuild it this way" — then [`CONTRIBUTING.md`](CONTRIBUTING.md). Please be a
good neighbor per the [Code of Conduct](CODE_OF_CONDUCT.md). Open a bug,
feature, or **field-data** issue from the templates; CI (`npm test` +
`npm run build`) runs on every pull request. Security issues go through
[private reporting](SECURITY.md), not the public tracker.

**Ground-truth datasets are as valuable as code.** A known object, shot from
two angles, EXIF intact, with the real measurements written down, is how every
accuracy claim above got made.

## Credits

Free public data throughout — no keys, no accounts, no paid tiers: ADS-B from
the community networks, satellites from CelesTrak, terrain from Terrarium/AWS
Open Data, maps from OpenStreetMap and Esri, weather from Open-Meteo,
magnetics from NOAA. Full list with licences and limits in
[`docs/DATA-SOURCES.md`](docs/DATA-SOURCES.md).

Born on r/UFOs energy; built for anyone who thinks sightings deserve better
data. MIT licensed.
