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
