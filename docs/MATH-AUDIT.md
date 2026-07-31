# Math accuracy audit

An end-to-end audit of the measurement chain, run 2026-07-30 against
`main` @ `1ef542e`. Every number here is reproduced by `npm run mathaudit`,
which drives the **shipped** functions with exact ground truth or an
independent reference — nothing below is an estimate.

Phodar's stated product is honest uncertainty. That makes systematic error the
thing that matters most: random error is already reported (ray-miss, convergence,
the ±1° ellipse), but **a bias that repeats every time is invisible to all of it**.
Every finding here is of that kind.

> **The single most important observation:** none of these raise the ray-miss
> residual. All the rays are distorted together, so they still intersect
> cleanly and the fix is still graded `excellent`. The quality grade measures
> *self-consistency*, not *correctness*, and it cannot see any of this.

None of it invalidates the field results — the weathervane case is 0.44 m of
model error at 120 m, well inside the reported agreement. The issues grow with
baseline, range and latitude.

## Findings

| # | Finding | Typical | Worst measured |
| --- | --- | --- | --- |
| 1 | ~~ENU frame built on a sphere, not the ellipsoid~~ | ~~0.1–0.3%~~ | **FIXED** |
| 2 | ~~Local-vertical convergence between observers ignored~~ | ~~0.045° at 5 km~~ | **FIXED** |
| 3 | Star catalog is J2000, used as if of-date | 0.29° | 0.37° |
| 4 | Stars/Sun and planets drawn in different equinoxes | 0.46° | 0.46° |
| 5 | Moon ephemeris truncated to one periodic term | 0.81° | 1.19° |
| 6 | Refraction applied to the Moon only | 0.02° | 0.30° |
| 7 | Angular size ignores the lens term the app itself fitted | 0.4–3% | 8.1% |
| 8 | Sight-lines unrefracted while cross-checks are refracted | 0.09° at 10° el | 0.30° at 2° el |

## 1 + 2 · Geodesy

`analyze()` driven with exact WGS84 truth — an object at a known geodetic
position, each observer's az/el computed from its **own** true local vertical,
i.e. the angles a perfect instrument would read on site:

| scenario | baseline | range | 3-D position error | altitude error | grade given |
| --- | --- | --- | --- | --- | --- |
| weathervane (the field-validated case) | 60 m | 120 m | 0.44 m | −0.0 m | excellent |
| drone over a field | 400 m | 1.2 km | 4.29 m | −0.9 m | excellent |
| aircraft, two witnesses | 5 km | 12 km | 28.1 m | **−16.2 m** | excellent |
| high object | 20 km | 40 km | 149.8 m | **−119.2 m** | excellent |
| high object | 50 km | 90 km | 1071 m | **−590 m** | excellent |
| 5 km baseline at the equator | 5 km | 12 km | 81.8 m | −16.7 m | excellent |

Two independent causes.

**Cause 1 — the frame is a sphere.** `enuFromGeo` scales both axes by one mean
radius `RE = 6371 km`. The correct local scales are the ellipsoid's radii of
curvature — *N* (prime vertical) east, *M* (meridional) north — which differ
from `RE` **and from each other**:

| lat | north scale error | east scale error | resulting bearing skew |
| --- | --- | --- | --- |
| 0° | −0.558% | +0.112% | 0.192° |
| 30° | −0.308% | +0.196% | 0.144° |
| 42.3° | −0.104% | +0.264% | 0.106° |
| 60° | +0.195% | +0.364% | 0.048° |

The observer baseline is built with these, so the scale error propagates
directly into range, altitude and true size. Because the two axes are scaled
*differently*, the frame is also sheared, which skews bearings by up to 0.19°.

**Cause 2 — the verticals are parallel.** Every observer's az/el is measured
against its own local vertical, but `analyze()` reads them all in the reference
observer's flat tangent frame. Over a baseline *b* the verticals diverge by
*b/R*: 0.009° at 1 km, 0.045° at 5 km, 0.18° at 20 km, 0.45° at 50 km. This is a
pure elevation bias on every non-reference observer, which is why the altitude
error above grows with baseline faster than the horizontal error does.

**FIXED** (2026-07-30). `enuFromGeo`/`geoFromEnu` are now built exactly on the
ellipsoid through ECEF, and `dirFromAzElAt()` rotates each observer's az/el from
its own local basis into the reference frame (returning `dirFromAzEl` unchanged
when the observer *is* the reference, so every single-observer path is
bit-identical). Every scenario in the table above now recovers truth to **0.00 m**
— the 50 km case went from 1071 m to sub-millimetre. Locked in by exact-truth
assertions in `npm test`.

One consequence worth recording: the curvature drop is now **inherent** in the z
that `enuFromGeo` returns. `adsb.js` had been subtracting its own `d²(1−k)/2R`
term on top of a flat frame; that would have double-counted (≈27 m at 20 km), so
it now adds back refraction alone (`+k·d²/2R`). Any future caller doing its own
curvature arithmetic must make the same adjustment.

## 3 + 4 + 5 + 6 · Astronomy

**The star catalog is J2000** (`starcat.js` says so) and `raDecToAzEl` treats
those coordinates as of-date. 26.6 years of precession is ~0.37° of sky. Over 67
catalog stars above 12°: mean offset 0.288°, max 0.371°.

The part that matters is what happens next. A camera pose has three rotational
degrees of freedom, and precession is very nearly a **rigid rotation** of the
sky — so the plate solve absorbs it:

```
residual the solver reports : 0.0350° rms   ← looks excellent
rotation it silently absorbs: 0.3724°       ← lands on the object's direction
```

The app's own field result — *"72 stars at 0.04° rms"* — is that first number.
A good residual is being read as evidence of a good pose, but here it is
evidence of a *self-consistent* one. At 10 km that bias is 65 m.

**The app carries two solar ephemerides in different frames.** No external
reference is needed to see this — `astro.js` fixes the perihelion longitude at
its J2000 value (102.9372°, no rate) while the Schlyter elements in `planets.js`
all carry secular rates (equinox of date). At the same instant they disagree by
**0.457°**. Both are self-consistent; they are drawn on the same dome, so the
star field sits ~0.4° from the planet markers. Cross-checked externally: the
shipped solar declination crosses zero 10.6 h after the published 2026 March
equinox (a reference implementation lands within 14 min).

**The Moon keeps only the equation of the centre.** Evection (1.274°), the
variation (0.658°) and the annual equation (0.186°) are all omitted. Against a
truncated ELP over one lunation: **mean 0.819°, worst 1.190°**. The Moon's disc
is 0.52° wide, so the error exceeds the anchor being used. The help calls the
Moon *"the strongest quick check that your bearing is right."*

**Refraction is applied to the Moon and nothing else.** `moonPos()` adds it;
`raDecToAzEl()` — stars, planets, the Sun — does not, so the layers on one dome
are mutually inconsistent by ~0.02° at 15° up and more near the horizon.

## 7 · Angular size

`angSizeFromPoints()` builds both rays through a pinhole (FOV only). But
`solvePoseAnchors` fits a radial distortion term *k*, stores it, and `pixToDirK`
uses it — so the app knows the lens is not a pinhole and then measures the
object as if it were. Since true size is `2·d·tan(θ/2)`, the size error equals
the angular error exactly:

| object position in frame | k=−0.10 | k=−0.05 | k=+0.05 |
| --- | --- | --- | --- |
| dead centre | 0.00% | 0.00% | 0.00% |
| 1/4 out | 0.85% | 0.42% | −0.42% |
| half way to the edge | 3.29% | 1.62% | −1.56% |
| near the corner | **8.10%** | 3.86% | −3.51% |

This only bites once *k* has actually been fitted — which is the star-align
path, the one users are told is the most accurate.

## 8 · Measurement vs cross-check

`adsb.js` drops predicted aircraft by earth curvature with standard refraction
(k≈0.13), and `terrain.js` ray-marches with the same. `triangulate.js` — the
path the witness's own sight-line takes — models neither. The two are then
differenced to rank candidates, so the comparison is between a curved,
refracting world and a flat, vacuum one. Refraction alone is 0.09° at 10°
elevation (31 m of cross-range at 20 km) and 0.30° at 2°; the curvature drop
over 20 km is 27 m (0.078°).

## What this does not cover

Verified sound and not listed above: the triangulation least-squares itself, the
tangent-scaled projection aspect, `pixelDirFromAnchor`'s azimuth convergence,
the WMM geomagnetic model (validated to 0.005° against all 100 official test
vectors), the planet ephemeris in its own frame, and the trajectory kinematics
(`npm test` recovers a simulated 3.5 g manoeuvre exactly).

Not audited: SGP4 satellite propagation beyond the existing TLE-staleness
honesty, the video pose tracker's empirical accuracy (measured separately in
CLAUDE.md against a real field clip), and photometry.

## Reproducing

```bash
npm run mathaudit
```

Dependency-free Node, like `npm test`. It is a **report, not a gate** — it
prints measured numbers so the effect of any fix can be measured rather than
asserted. If a fix lands, the corresponding rows should collapse toward zero.
