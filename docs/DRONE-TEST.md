# Drone calibration flight — field protocol

A drone you own is the best ground truth available: unlike a weathervane or a
cell tower it *moves*, and unlike an airliner you control where it goes and you
hold its full GPS/barometer log. One flight exercises the entire pipeline —
direction, triangulated position, altitude, true size, speed, heading — and the
app's **🛩 Drone flight-log check** (results step) grades every number against
the log automatically.

This protocol is written for a gen-1 DJI Mini (Mavic Mini) or DJI Neo; any
drone whose flight can be exported works the same way.

## The target

| Craft | Span to enter | Notes |
| --- | --- | --- |
| DJI Mavic Mini | 0.202 m (preset) | 160×202 mm body unfolded, 213 mm diagonal wheelbase. Spinning props blur out to ~0.33 m but photograph as almost nothing — mark the **body**. |
| DJI Neo | 0.157 m (preset) | 130×157 mm including the built-in prop guards — the guards *are* visible, so the preset span is what you'll actually mark. |

These are small targets. At 100 m the Mini subtends ~0.12° — about the Moon's
diameter ÷ 4. That is measurable, but it means:

- **Keep ranges modest: 50–150 m.** Beyond ~200 m a 20 cm drone is a dot and
  the size measurement (not the direction) degrades first.
- Shoot at your phone's longest **optical** lens (3× tele if you have it) —
  EXIF carries the focal length so the math follows automatically.
- Against clear sky, sun behind the observers. Lock/drop exposure a touch so
  the grey craft doesn't blow out against bright sky.

## Geometry (the part that decides the grade)

- **Two observers, 30–60 m apart** for ranges of 100–150 m. The app wants a
  baseline of at least ~1/10 of the range; more is better.
- Fly the drone broadly **perpendicular to the baseline**, not along it —
  that's what gives the sight-lines a healthy convergence angle.
- Elevation angles of 20–45° are comfortable to mark and keep the craft clear
  of the terrain skyline.
- Stand in the open. The field cases in `FIELD-TESTS.md` measured phone
  compasses 14–66° wrong near cars and steel roofs — and sub-degree on foot in
  the open. Don't shoot from inside/near a vehicle.
- Note or photograph the **takeoff spot**. If your log exports only
  height-above-takeoff (most DJI decodes do), the check will ask for the
  takeoff ground elevation — the position step's "⛰ Use terrain elevation"
  value for that spot is the honest answer.

## Clocks

Phone photo times are NTP-good; the drone log clock usually is too, but
timezone handling in third-party exports is a known mess. **You don't need to
fix this in the field** — the check scans the whole log for the instant whose
position best fits every sight-line and reports the offset from your stated
time. Just don't *trust* an offset of hours: that's the check telling you the
export's timezone is wrong, and it will say so.

## Flight plan (10 minutes)

1. Both observers take position, drop pins later on the map (or shoot with GPS
   on — EXIF position is read automatically). Measure nothing by hand.
2. Take off, note the spot, climb to **~40 m**, fly out to ~100 m range,
   broadly perpendicular to the baseline.
3. **Hover #1 — Moment A.** Count down out loud; both observers photograph the
   drone within a second or two of each other. Hold the hover ~10 s.
4. **Steady leg.** Fly a straight line at a constant, remembered speed
   (e.g. 5 m/s for 10 s — the log records the truth anyway).
5. **Hover #2 — Moment B.** Second synchronized photo pair. This is what turns
   the test from position-only into speed + heading.
6. Optional but valuable: one observer shoots **video** of the whole leg
   (stabilize + track later → dense trajectory vs the log), and repeat at a
   second range (e.g. 60 m and 140 m) to probe how error grows.
7. Land. Before leaving, each observer takes one wide shot of the horizon from
   their spot — it helps the terrain-skyline calibration later.

Fly legal and boring: VLOS, under 400 ft AGL, away from people — a calibration
flight needs nothing fancy.

## Getting the log

- **Airdata (recommended):** DJI Fly syncs flight records to your DJI account;
  connect airdata.com (free tier is fine) → open the flight → **Export → CSV**.
  Airdata CSVs carry UTC datetimes *and* MSL altitude — the best case.
- **PhantomHelp / flight-reader:** upload the flight record `.txt` from the
  phone (`DJI Fly/FlightRecords/`), download the decoded CSV. (The raw `.txt`
  is encrypted — the app can't read it directly, and will say so.)
- **Video .SRT captions:** if the drone wrote subtitle telemetry next to its
  own video, that file works too (per-frame GPS). The drone's own video isn't
  needed for the analysis — the log is.

## Analysis

1. Run the wizard normally for both observers: photo → mark the drone's body
   edges → position pins → sky placement (calibrate against terrain/Sun — this
   is a *pointing* test, so calibrate like you mean it).
2. Set Moment B from the second photo pair for speed.
3. Results step → **🛩 Drone flight-log check** → load the CSV/SRT → pick the
   preset → read the grade. If the log is height-above-takeoff only, enter the
   takeoff elevation.
4. Generate the report — it now opens with a **"Drone flight-log ground
   truth"** section: the answer key, in writing, embedded in a self-contained
   file.
5. Add the results to `docs/FIELD-TESTS.md` (what was measured, what the truth
   was, what lesson got encoded).

## What to expect

With a 40 m baseline at ~120 m range, calibrated pointing, and honest ±1°
bearings: direction within ~1°, position error a few metres (2–5% of range),
altitude within a couple of metres, size within ~25% (marking a 0.1° target is
the hard part — this is the number most likely to be humbling), speed within
~10%. If direction grades excellent and position doesn't, the geometry
(baseline/convergence) is the suspect, not the compass.
