# Field validation log

## Case 1 — rooftop weathervane (Cave Junction, OR · July 2026)
Two iPhone photos on foot, 14.2 m apart, near-perpendicular bearings, target
= eagle weathervane on a house peak.

- Compass (EXIF `GPSImgDirection`, true ref) vs derived: **−0.52° / +0.88°**.
- Fix after equalizing GPS altitudes (phone alt wobbled 5.2 m across 14 m):
  range 11–13 m, **height 7.0 m AGL (23 ft)** vs owner estimate 20–25 ft,
  ray miss 0.59 m.
- Per-view projected sizes 53/44 cm; **two-view foreshortening solve → 63 cm
  span at long-axis 129°**, vs owner estimate ~24″ (61 cm). Each projection
  reproduced within 5 %.
- A later 4-view run added two contaminated shots: one under a steel-roofed
  pergola (**compass +28°, elevation −13°**), one near wire fencing
  (**compass +14°**). The per-ray-miss diagnostic names such outliers.

**Lessons encoded in the app:** equalize observer elevations on level ground;
magnetometers are sub-degree in the open and wildly wrong near metal; a
planar/elongated object's single-view size understates truth — the N-view
aspect solve recovers it.

## Case 2 — cell tower from a moving car (near Redding, CA · July 2026)
Two through-glass photos 29 s apart while driving. Raw bearings were
mutually impossible (intersection behind an observer): in-car compass error
measured **~65°** on one shot. Size-ratio arbitration identified the
self-consistent bearing and predicted the tower's location; the app now runs
this arbitration automatically whenever rays fail to converge, including on
`behind` fixes.

**Lesson:** never trust EXIF compass from inside a vehicle; calibrate in the
sky view against the Sun/shadows or a mapped landmark.

## Case 3 — instrumented capture: the sensors beat the tracker (Rogue Valley · July 2026)
First two recordings made with **🎬 Record with motion data** (in-app
getUserMedia video + a ~20 Hz gravity/compass attitude log).

**Clip 1** (5.9 s, a ~95° hand sweep). The visual stabilizer produced a
*confidently wrong* solve: it reported **34–46 inliers on every frame** while
following only **11.5° of azimuth** and freezing entirely after t=2 s. The
sensor log — which gravity independently corroborates, showing 27° of
elevation change alongside — recorded **94.9°**. Because the fusion decided
what to trust by inlier count, all 24 frames were graded "strong" and the log
was never consulted. The clip did not world-lock.

**Lesson:** *inlier count is not a truth signal.* A tracker that loses the
scene and re-acquires on whatever drifted into frame is confidently wrong,
and no amount of per-frame confidence can reveal that from the inside. Only a
second, independent witness can. Fusion now compares the **path length** each
source travelled; when the log says the camera moved and the vision says it
did not (sensors > 15°, vision/sensors < 0.45), the path is rebuilt from the
log — motion from the sensors, **absolute frame from the placement**, so the
compass bias cancels exactly and a star/terrain calibration survives.

**Clip 2** (5.8 s, ~105° sweep) after that fix: the check fired as intended
(**vision 13.4° vs sensors 220.2°** of travelled path), all 27 samples were
rebuilt from the log (`mode: "sensor"`), and the recovered path swept 104.1°
against the log's 105.5°, anchored on the placement at az 93° / el 13.3°.
Field verdict: *"worked great."*

**Open:** the visual tracker failed on BOTH in-app recordings, so the
gap-filling fusion path (vision leading, sensors carrying weak frames) has
still never been exercised on real hardware — only its sensor-only fallback
has. In-app capture is ~1080p with rolling shutter and no zoom; a
native-camera clip with a motion log would be the way to test the other half,
and is not currently possible (iOS gives no attitude to the system camera).

## Case 4 — Little Grayback Lookout, 4–6 miles out (Applegate Valley, OR · July 2026)
The first **long-range** ground truth: a target with a published elevation and
known dimensions, far enough away that the geometry is genuinely hard.

Target: **Little Grayback Lookout** (Oregon Dept. of Forestry, ~9 mi E of Cave
Junction) — **5,149 ft**, 14×14 ft cab. Three iPhone 14 stills from the valley
floor, baseline **1.48 mi**, all three photos aligned **by eye** (no star or
terrain solve).

An independent ray-march through the same DEM the app uses for its skyline put
the summit at 42.1984, −123.4936 / **5,151 ft** — 2 ft from the published
figure, so the reference itself is trustworthy.

| Observer | az error | el error | range error |
| --- | --- | --- | --- |
| 1 | −0.25° | −0.18° | +0.50 mi |
| 2 | −0.80° | −0.03° | +0.49 mi |
| 3 | −0.73° | −0.31° | +0.49 mi |

Fix: **0.45 mi long** along the sight line, altitude **5,364 ft vs 5,149 ft
(+215 ft, +4.2 %)**, size **29.3 ft** (26.9 corrected for the range error)
against a 14 ft cab. Convergence 1.46°, range/baseline 3.7:1, rated *fair*.

**The error ellipse held.** The app predicted 1σ = 3,202 ft × 142 ft with the
long axis on bearing 52°. The actual error was **0.45 mi (2,380 ft) on bearing
45°** — inside 1σ, along the axis it named. This is the first check of the
uncertainty machinery against a target of known position, and it passed:
the fix was wrong by about what the app said it would be, in the direction it
said, and it declined to call the result better than *fair*.

**Elevation source was not the limiter.** Re-running with **⛰ Use terrain
elevation** on all three observers moved the fix **7 m** and left the altitude
error unchanged: the phones' GPS altitudes were already within 3.6 m of the
DEM. Observer altitude propagates ~1:1 into object altitude and barely at all
into horizontal position, so its ceiling here was metres against a 65 m error.

**Lessons.** The limiter was *differential* azimuth: the three bearings are
biased the same way (~0.6°) but differ from each other by up to **0.55°**, and
at 1.46° convergence that differential is what set the range. Hand placement
to 0.25–0.80° is good work (0.8° ≈ 46 px on a 4032 px frame at 69.4°) and
still not enough at 3.7:1. Two things beat it, in order: **break the
collinearity** — all three observers sat on one east–west line, so every ray
was within 5° of the others — and **⛰ Snap to ridges**, which is the ideal
tool when the object sits *on* the skyline, as this one does.

Size is the weakest output and honestly so: the structure spans ~3 px at
4032 px, where ±1 px is ±33 %, and the per-observer estimates (20.8 / 30.8 /
36.4 ft) disagree by 75 % — the spread IS the uncertainty, visible in the
report without anyone having to compute it.
