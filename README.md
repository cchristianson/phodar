# PHODAR

**PHO**togrammetric **D**etection **A**nd **R**anging — turn sighting photos
into numbers. One witness photo yields honest angular data (direction, angular
size, angular motion). Two or more witnesses yield a true triangulated fix:
**position, altitude, real size, speed and heading** — with quality grading,
error diagnostics, and a shareable white-paper report that other witnesses can
import to add their own perspective.

Built mobile-first for real sighting conditions: EXIF auto-fill (GPS, time,
lens FOV, compass), an astronomically anchored sky view for calibrating each
photo against the Sun/Moon and horizon, 3D wireframe shape fitting (orb,
saucer, tic-tac, triangle, plane, bird) that records the object's **pose**,
per-segment trajectory timing, and single-view size↔distance analysis when
you're the only witness.

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
warnings) instead of silently producing garbage. See `docs/FIELD-TESTS.md`.

## Quick start

```bash
npm install
npm run dev        # open the LAN URL on your phone
npm test           # math regression suite — must pass
```

## Deploy (Railway)

Push to GitHub → new Railway project from repo. `railway.toml` builds with
Vite and serves the static bundle. That's it.

## How the math works (short version)

Each photo is pixel-normalized (EXIF baked out — iOS lies otherwise) and its
FOV taken from lens metadata. The object's sky direction is fixed by aligning
the photo on an alt-azimuth grid anchored to computed Sun/Moon positions, or
via automatic horizon detection. Sight-lines from all observers are
intersected by least squares in a local ENU frame; ray convergence and rms
miss grade the fix. Size = angular size × range; an N-view foreshortening
solve recovers the **true span and long-axis heading** of elongated objects
that single views understate. Trajectories interpolate each witness's
directions to common instants before triangulating each instant — speeds,
accelerations, felt g-loads and turn rates follow.

## Contributing

Start with `CLAUDE.md` (architecture + invariants) and
`CONTRIBUTING.md`. The backlog's headline items: module split, Leaflet map,
and the **ADS-B cross-check** (every transponder-equipped aircraft is a free
calibration target with published position, altitude and type).

Born on r/UFOs energy; built for anyone who thinks sightings deserve better
data. MIT licensed.
