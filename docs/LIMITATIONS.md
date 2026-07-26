# Limitations

Phodar's design rule is that a warning beats a silent guess. This page applies
that to the app itself: what it does badly, with numbers where they were
measured. Most of these are surfaced in the app or the report as caveats; they
are collected here so nobody has to discover them the hard way.

## Measurement

**A single viewpoint cannot give you distance.** One photo yields direction,
angular size and angular motion — all honest, none of them a range. Everything
downstream of range (true size, altitude, speed) requires a second perspective
or an assumed distance, and the app says so rather than picking a number.

**Fix quality is dominated by geometry, not by care.** A short baseline with a
small convergence angle produces a fix that is precise-looking and wrong. The
report grades this (baseline, convergence angle, rms ray-miss) and translates a
±1° pointing error into position uncertainty. Read that number before quoting
any of the others.

**Phone compasses are the weakest link.** Field-measured: sub-degree accuracy
on foot, but **14–66° wrong near metal** — inside a car, under a steel-roofed
pergola. Phodar arbitrates between witnesses by size ratio and names the
suspect bearing, but with only two observers it cannot always tell which one is
lying. Calibrate against the Sun, a star field, or a terrain ridge whenever the
photo allows it — that is what the sky view is for.

**Phone GPS altitude wobbles ±5 m,** which is significant against a short
baseline. If everyone stood on level ground, set their elevations equal, or use
the one-tap DEM elevation.

**EXIF times are device-local and unverified.** Two witnesses' clocks routinely
disagree by seconds. For two-video work, the sync is recovered from the
object's own motion — but a slow, distant object gives a shallow minimum and a
correspondingly weak lock, which the report states as low sync confidence.

## Video

**The world lock slides.** Measured on a real 22.8 s handheld field clip with a
1.35× zoom: the stabilizer removes the *shake* well (residual frame-to-frame
background motion 0.26 px at 540-wide, high-frequency jitter 0.08 px/frame
against 0.11 in the original), but cumulative excursion reaches ~60 px at
540-wide — visible as the photo's ridge sitting on the burned-in terrain line
at t=8 s and ~1.4° below it at t=16 s.

The cause is structural: the only absolute reference is the one aligned frame,
so everything between re-anchors is an incremental chain, and the coarse global
registration can't hold better than ~1–2°. The **⚓ Fix frames** mode exists
for this — anchor a frame you can see is wrong and the correction interpolates
across its neighbours. An automatic terrain-based version was built, measured
and reverted; `CLAUDE.md` records exactly why and what a retry would have to do
differently.

**Frames that lose the lock are held, not invented.** A run of weak frames
shorter than 0.55 s is bridged by interpolation; longer runs stay frozen and
are reported, because interpolating a one-second gap fabricates motion. The
stabilize summary counts solved, carried, bridged and held frames separately —
that count is the honest quality metric for a clip.

**Inlier count is not a truth signal.** A tracker that loses the scene and
re-acquires on whatever drifted into frame reports a *confident* solve that
barely moves. Field case: the phone swept 95° of azimuth while the visual solve
reported 11.5° at 34–46 inliers per frame. Phodar now compares path length
against the motion log to catch this, but only when a motion log exists.

**Optical zoom is tracked, not known.** The solver infers field of view per
frame from the background's scale change, bounded by the lens's physical
widest. It is good but not exact, and a clip that zooms hard while the
background is self-similar (foliage, water) is the hardest case in the app.

## Instrumented capture

The in-app "record with motion data" mode logs device attitude at ~25 Hz
alongside the video. Real caveats:

- It records through `getUserMedia` at roughly 1080p, with **no lens switching
  and no optical zoom**. It is a measurement mode, not the way to shoot the
  best-looking footage.
- Motion sensors require the **installed PWA** on iOS. In a plain Safari or
  Chrome tab the permission prompt never appears and the log comes back empty.
- **On Android the bearing may be magnetic rather than true.** iOS's compass
  heading is referenced to true north; the orientation sensor other platforms
  expose may or may not have declination applied, and there is no way to ask
  which. Phodar says so instead of guessing — declination reaches 20° in places,
  and the sky view's terrain or star calibration is the fix. Elevation and roll
  are unaffected.
- Gravity is drift-free, so the sensors own *motion*; the compass is biased, so
  they never own *absolute pointing* — vision keeps the absolute frame. Don't
  blur that division.
- The vision-leading fusion path (a good visual solve with sensors filling weak
  frames) is asserted against synthetic data but **has not yet run on real
  hardware** — both field clips so far triggered the sensor-only fallback
  instead.
- An instrumented clip still needs a stabilize pass before world-locked
  playback.

## Cross-checks

Each check answers "could this have been X?" — never "this was X." They rank
candidates by angular separation from the sight-line, and a close match is a
hypothesis worth taking seriously, not an identification.

- **ADS-B coverage is not complete.** Four networks are merged precisely
  because each has different receiver coverage, but low-altitude, military and
  non-transponding traffic is simply absent. An empty aircraft check is weak
  evidence.
- **Historical archives lag.** The replay data is a few minutes behind real
  time; a sighting fresher than that falls back to live traffic with a warning
  about the time gap.
- **TLEs age.** Satellite positions degrade with TLE age; the report states how
  stale the elements were.
- **Winds aloft are model output,** not a measurement at your location, and
  cloud-base bounds aren't implemented.
- **Terrain is a coarse DEM** (~19 m/px at the working zoom). It is excellent
  for distant ridgelines and unreliable within a couple of hundred metres —
  the ray-march deliberately starts at 200 m because closer samples are
  resolution noise.

## Operational

- **Single-process server.** Caches are in-memory and per-process; two
  instances double the upstream load and share nothing.
- **No accounts, no sync, no backup.** Everything lives in your browser's
  storage. Clearing site data deletes the sighting. Export the share file or
  the report if it matters.
- **iOS memory ceilings are real.** Canvases are capped, undo stacks are
  bounded, and export resolutions step down on failure. Those caps are not
  arbitrary — see `CLAUDE.md`.
- **Not evidence-grade.** Nothing here is suitable for aviation safety
  reporting, and no output should be presented as an identification of anything.
