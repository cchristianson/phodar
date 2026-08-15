# Changelog

Notable changes to Phodar. Format loosely follows
[Keep a Changelog](https://keepachangelog.com/); versions are the ones in
`package.json`.

## [Unreleased]

### Added
- **📍 Find my spot — skyline geolocation** (position step). For media
  with stripped location data: give it a rough search area and it
  sweeps candidate positions, matching the DEM terrain skyline against
  EVERY usable frame at once (one camera can't point everywhere — a
  candidate must explain all frames within one ±40° pointing window),
  and ranks where the shot fits best with the implied facing direction.
  Optionally, tap a frame and pin a structure the photo shows (💧 water
  tank / 📡 mast): pins are matched against OSM twins
  (`/api/landmarks`) and candidates that also place the pinned
  structure in the right direction are badged and ranked first —
  field-proven the discriminating constraint (2° deviation for the
  true area vs 60–160° for rivals on the Himalaya test clip, where
  silhouette shape alone was NOT decisive across a 28 km box in four
  successive harness rounds). That honesty is structural: a flat score
  spread says "terrain alone can't decide — treat these as ranked
  suggestions", never a confident wrong pin. The final check stays
  human: tap a candidate, eyeball the satellite imagery, drag ⌖ onto
  your actual spot, adopt (sets position + viewing direction).
  Deliberately zero ambient UI: one button on the position step, a
  self-contained overlay, no sky-view changes. Under the hood: a
  haze-proof far-skyline detector (atmospheric scattering makes distant
  ridges literally sky-blue — luminance deficit from the top-sky
  reference sees through it, and a sky-quality gate drops frames
  without clean sky instead of letting them flatten the joint score);
  one shared z11+z8 heightfield region (~26 MB) for the whole sweep
  instead of a 15 MB demSampler entry per candidate (which would OOM an
  iPad); FOV swept per frame (witnesses zoom) unless the lens is known.
  Pure core in `src/geoloc.js`, 15 mathcheck assertions (synthetic
  sweep recovers a known cell exactly with per-frame pointing, far
  detector to sub-pixel, decisive-vs-flat verdict calibrated to the
  field data), 22-assertion browser e2e driving the real UI with the
  real 91 s Himalaya clip end to end. First field round on an iPhone
  found two gaps, both fixed same-day: the pin view's portrait frame
  overflowed its flex box and pushed the kind chips off-screen (the
  image is now sized to a measured fit box, and the e2e asserts the
  chips land fully on-screen), and the overlay gained its own 🔎 place
  search that takes captions-grade queries ("India lower Himalaya
  range") via Nominatim, framing the map to the result's bounding box
  so an area answer reads as an area, not a fake point.

### Added (since the geolocation stack)
- **🎬 Scene-cut detection — a compilation clip is not one camera.**
  Field case (the second "still no better" report): the clip hard-cut
  from a dusk city segment to unrelated daytime-sky footage at 23.6 s —
  a social-media splice, subtle enough that even ffmpeg's scene detector
  scores it below its default threshold because both sides are mostly
  featureless sky. The stabilizer solved the splice as camera motion, so
  every pose after the cut was fiction, and the panorama dutifully
  painted two unrelated scenes into one canvas (the registration then
  "locked" sky onto sky at 0.97 confidence — featureless content
  correlates with anything). No stitcher fix could help; the fix is
  upstream and honest: `stepTracker` now returns scene-cut evidence when
  nothing placed a frame — either a healthy template herd where not ONE
  template re-found its target while the whole-frame register only
  "matched" as an impossible teleport (same-scene frames at ≤¼ s always
  re-find some), or a chromaticity jump (dusk warm-gray → daylight blue
  have near-identical LUMINANCE, which is why gray correlation alone is
  blind to exactly this splice; chromaticity also normalizes out
  auto-exposure swings). The walk confirms by its existing time
  bisection — same-scene blur resolves at finer spacing, a real cut
  never does — then STOPS at the cut instead of solving fiction,
  records `source.sceneCuts`, and says so: in the solve flash, in the
  playback row ("solve stops at the 23.6s scene cut"), and on the
  measure step next to the ✂ trim that isolates either scene. The
  object pass stays inside the solved span too — no more tracking "the
  object" into different footage through a held pose. Asserted in
  mathcheck four ways (noise-textured cut flagged, palette flip caught,
  featureless frame stays an honest hold, ordinary step never flagged);
  verified end-to-end on the user's actual spliced clip. Stated
  limitation: two spliced scenes of near-identical smooth content AND
  palette can still cross-match — measured on synthetic smooth fields —
  so a missed cut is possible; the trim remains the manual override.
- **🖼 Panorama v3 — adaptive-resolution registration (the "ran it 3
  times and no better" fix).** The v2 re-registration ran every frame
  against ONE fixed 2 px/° coarse twin of the whole panorama — and a
  deeply zoomed frame (~10° FOV) spans only ~20 px there, below what
  any correlation can lock, so the zoom stretches that needed
  correction most got none (field-measured: 2–3 zoom-scale corrections
  across ~85 frames, composite unchanged after three runs). Each frame
  now registers inside its OWN local window (`regWindow`) at a
  resolution where it spans ~100–200 px whatever its FOV, with the
  base cropped from the FULL-RES composite so the detail is really
  there — two levels: a placement lock reaching ~±2° at any zoom
  depth, then the FOV-scale ladder at ~2× the density (a zoom error is
  a scale error, and scale needs pixels — exactly what the fixed twin
  denied the zoomed frames). Wide frames keep the old coarse floor, so
  their cost and behavior barely change; correction quantization on a
  zoomed frame drops from 0.5° to ~0.05°. Proven in mathcheck (window
  sizing/bounds/mapping) and the browser harness: a fov-8 frame lying
  by +0.7° az and 15% zoom — the exact starved case — recovers to
  0.01° az and 1.2% FOV residual through the real render path.

### Fixed
- **iOS text autosizing inflated the sky-view status text** ("why is
  the text under the re-stabilize button so big?"): Safari's font
  boosting re-sized text-dense blocks the styles never asked it to,
  which also widened rows until buttons clipped off the right edge of
  the screen. The stylesheet now pins `text-size-adjust: 100%`, the 📐
  panorama bar wraps with a real minimum text basis, and the playback
  row wraps as a backstop — the ⬇ %-progress button can no longer be
  pushed off-screen even under large accessibility text.

- **📐 Panorama corrections → the camera path.** The composite measures
  where each frame REALLY belongs — and now those measurements can flow
  back into the solve. The stitcher carries the zoom-scale correction
  forward frame to frame (a mid-zoom FOV error is smooth in time, so
  each frame is seeded with the trend and the ladder only hunts the
  residual — large cumulative errors stay reachable rung by rung), and
  strong registrations (score ≥ 0.5, ≤4°, ladder-bounded FOV) become
  candidate ⚓ anchors. An amber bar offers "📐 Apply to path": they
  merge into poseFixes through the existing anchor machinery, so
  playback, the trajectory, exports and the live-frame-over-panorama
  all follow — previously the live frame drew at the uncorrected solved
  pose and visibly disagreed with the composite behind it (field
  report). Deliberately OPT-IN with hand-placed anchors outranking
  (the reverted terrain auto-anchor is the cautionary tale);
  re-stabilizing clears them like any anchor.
- **🖼 Panorama: zoom-scale registration (the two-trees fix).** Field
  case: the final zoomed frame of a clip landed at the wrong SIZE — the
  same tree twice, small in the current frame and large in the previous
  one. A shift can never fix that: a zoom error is a SCALE error, and
  the solve's FOV estimate mid-zoom carries a few percent of error.
  `registerFrame` now searches a small FOV-scale ladder alongside the
  shift — the frame's coarse footprint is rendered at candidate zoom
  corrections and the correlation picks the one that locks onto the
  composite. Scale 1 is privileged (a rung must beat it by a real
  margin, so non-zooming frames never jitter their FOV), and the ladder
  only runs mid-zoom or when the plain shift can't lock, so steady pans
  cost nothing extra. Proven both ways: mathcheck corrects a synthetic
  15% zoom bias to 1.2% residual and keeps clean frames at scale 1; the
  browser harness pulls a claimed 46° FOV back to 40.5° against a true
  40° through the real render path. The flash reports how many frames
  were zoom-corrected.
- **🖼 Panorama v2 — re-registered stitching + the dome layer.** The
  first field render placed tiles visibly wrong: the solve's weak
  stretches (few anchors, chained drift, whip-pans) were trusted
  blindly. Two fixes: `panoPick` gates frames (held frames never
  qualify; starved solves and >25°/s whip-pans are dropped when enough
  strong frames remain), and every frame is now RE-REGISTERED against
  the growing composite before painting — in equirect space a pixel
  shift IS an angular shift, so alignment is a plain masked NCC on the
  coarse grayscale (`bestShift`; the solved pose seeds the placement,
  the pixels finish it; the sharp-repaint pass reuses the corrected
  poses). And the panorama now lives ON THE DOME: a 🖼 sky-layer toggle
  (builds the stitch on first tap, session-cached, invalidated by any
  re-solve) lays the whole pan frozen under the live frame, whose
  bright accent border shows exactly where the current moment sits —
  scrubbing moves the live frame across the composite. Sign convention
  proven end-to-end in the browser harness: a deliberate +2° pose bias
  is measured as −4 px and exactly undone.
- **🖼 Panorama (still) — the pan, registered in hindsight.** A fourth
  option in the stabilized-clip export menu: every frame projected into
  ONE equirectangular image at its solved direction (`src/video/
  panorama.js`). An iPhone panorama registers frames as you sweep; the
  stabilize walk already did that retrospectively — so messy handheld
  motion, reversals and zooms all stitch. Frames paint chronologically
  with feathered edges (exposure differences blend instead of seaming),
  then the sharpest third (narrowest FOV) repaints on top — a zoom pass
  becomes a high-resolution inset instead of being buried under later
  wide shots. Azimuth unwraps across the 0/360 seam, roll rides the
  per-cell mesh, held frames are skipped, and the canvas respects the
  iOS guards (≤4600 px side, ≤16 Mpx). A still render: no encoder, no
  realtime constraints — seeks into one canvas, saved as JPEG. A moving
  object can appear more than once; the flash says that's its real
  path, not a glitch. Pure geometry (unwrap, tan-true layout, equirect
  mapping, zoom-last ordering) is mathcheck-asserted; the compositor is
  verified in a browser harness (marker dots land at their predicted
  directions through a rolled frame, overlaps blend, uncovered canvas
  stays background).

### Changed
- **Find my spot: ±1 km fine mode + coordinate paste.** The
  walk-the-last-kilometer tool: once anything (a coarse search, a tip, a
  report) has you within a neighborhood, paste coordinates like
  "30.379, 78.104" into the 🔎 search to jump straight there, pick
  ±1 km, and the sweep runs ~200 m cells with 150 m structure rings.
  The results header says plainly what changes at that scale: the ridge
  shape barely moves between cells, so the pinned structures and the
  near-ridge layer do the ranking — pin everything the frames show.
  Twin structures are fetched over at least 3 km regardless of the
  radius (the anchor tank may sit just outside a tight circle), and the
  result-thinning distance scales with the grid so fine candidates
  aren't collapsed away.
- **Find my spot: 🌦 weather cross-check.** The clip SHOWS its sky, and
  the claimed date + a candidate area imply one: the search now reads
  the sky from its own sampled frames (saturated blue = clear,
  white-gray dome = overcast, hazy in-between = mixed and honestly
  no-call) and compares it against the reanalysis archive's cloud cover
  for the searched area on the stated date (reusing `fetchWeatherAt` /
  the `/api/winds` proxy, ERA5 for older dates). Agreement is mild
  support; a clean contradiction ("clip is overcast, archive says 8%
  cloud") says to question the date or the area. Structural honesty:
  cloud data is ~25 km coarse, so the check judges the AREA+DATE
  pairing and can never separate candidates inside one search — the
  line says so.
- **Find my spot: setting filters on real land-use + the near-ridge
  depth layer.** Field report: "in a town" passed a spot with zero
  buildings — a place NODE marks a town's *center*, and its type radius
  covered a bare field 1 km out. The filters now run on OSM built-up
  LAND-USE polygons (residential/retail/commercial/industrial, fetched
  as bounding boxes in a second Overpass query): "in a town" means
  standing on mapped built-up land, "outside" means clearly off it, and
  the view test ray-marches into other built-up patches (still
  excluding the one you stand in). Place nodes remain the fallback
  where land-use isn't mapped; no data still never filters. And a new
  SECOND matching signal: the detected ridge line below the far wall
  (the darker, nearer crest) is scored against the DEM's interior
  visible crests — a pan has no parallax, but where the near crest sits
  against the far wall changes fast with position. Scored JOINTLY with
  the far skyline at every candidate pointing (evaluating it only at
  the far layer's chosen azimuth let a wrong azimuth win first and then
  punished the truth — measured, fixed): on a synthetic wall+near-ridge
  world where far-only actually prefers a displaced spot (sep 0.81),
  the joint score picks the truth with margin (1.37×) and punishes a
  2 km displacement ~9.5×. Results say "⛰ Two ridge layers matched"
  when the depth term is active.
- **Find my spot: setting context + stabilized-pan lock + pin
  housekeeping.** Two new chip rows tell the search what the witness
  already knows: where you STOOD (🏘 in a town / 🌾 outside one) and
  what the view crosses (🏙 a town / 🌲 open country) — matched against
  OSM place nodes with per-type built-up radii (city 6 km … hamlet
  0.6 km), the stood-in place excluded from the look test so "in city
  aimed out of city" works, and no place data means no filtering on a
  guess. Position filters run BEFORE scoring (they also save compute);
  view filters after (they need the solved pointing); the results state
  how many candidates the settings ruled out. If the clip was
  🎞 stabilized first, the search now locks every frame's direction and
  zoom together via the posePath — the whole pan scores as one rigid
  sample set over a single global rotation, which on the synthetic
  ground-truth world sharpened the best/median spread from 0.478 to
  0.055 (~9×). Mistaken pins are now easy to remove: tap the pin on the
  frame (bigger hit target) or its ✕ chip in the new pin list under the
  frame strip.
- **Find my spot: seven pin kinds, multiple pins.** The structure pins
  grew from water tank + mast to ⚡ power pylon, 🏭 chimney, 🌬 wind
  turbine, 🗼 lighthouse and 🏔 named peak (each mapped to its OSM
  selector in `/api/landmarks`, only the pinned kinds are queried, and
  twins are kept nearest-first so a dense pylon grid can't eat the
  cap). Pins now ACCUMULATE — any number per frame, across frames —
  because two structures in one frame constrain the position far harder
  than one. The consistency test moved into a pure, mathcheck-asserted
  core (`pinsDeviation`): every candidate is scored against ALL pins,
  worst pin governs, each kind carries a plausible camera-to-structure
  range gate (1.5 km for a water tank, 40 km for a peak pinned off the
  skyline) — without which a town with 31 mapped towers would let every
  cell "pass" by accident — and a pinned structure that isn't on the
  map is skipped, never held against a spot. Peaks never spawn
  ring candidates (you pin a peak from afar, you don't stand on it).
- **Position-step name search survives caption-grade queries.**
  Nominatim is exact-name matching with zero fuzz — measured: "lower
  himalayan range" hits the real mountain range, "lower himalaya range"
  returns NOTHING — so a failed phrase now gets a bounded relaxation
  ladder: the query with sighting-caption noise words stripped
  (UAP/over/seen/…), then each remaining content word alone, with
  single-word hits kept only when they are AREAS (wide bounding box or
  a place/boundary/natural class — keeps "Himalayas", drops a
  restaurant named Himalaya). An area-sized result is tagged with its
  width ("~2341 km area"), frames the whole area on the pin map
  (programmatic fitBounds, guarded so no position commit fires), sets
  the pin at its centre with an honest note, and points at 📍 Find my
  spot when media is attached. Browser e2e: the full caption "UAP over
  the lower Himalaya range India" resolves in ≤5 requests.
- **🔏 C2PA cryptographic verification** (user-approved dependency —
  Adobe's open-source c2pa-js SDK). The byte-scan's "a Content
  Credentials manifest exists" note now upgrades to a checked verdict:
  the SDK validates the X.509 signature chain and recomputes the signed
  content hashes against the exact pixels in hand. A valid camera
  signature becomes strong positive evidence ("Signed by Leica Camera AG
  … pixels unmodified since signing"); a manifest that FAILS
  verification is a tamper ALARM (the pixels were altered after
  signing — hard cryptographic evidence); a validly-signed
  trained-algorithmic-media assertion is a conclusive AI alarm (the
  generator itself attesting the image is synthetic); a verified edit
  chain naming an editor warns with the disclosure. LAZY by design: the
  ~6 MB WASM + SDK load only when the upload scan actually finds a
  manifest marker — ordinary files never fetch a byte (proven in e2e:
  a plain JPEG triggers zero SDK requests, a marked file loads the
  chunks, and a bogus manifest degrades gracefully to the presence note
  with no errors). Interpretation is a pure function with 5 mathcheck
  assertion groups (valid capture / tamper / AI / edit-chain / no
  manifest); absence of credentials still proves nothing and the UI
  still says so.
- **🌙 Moon terminator forensic — the composite-killer** (last item on
  the list). When the moon is IN a photo, its lit limb must point at the
  sun: pure celestial mechanics that a pasted-in moon routinely gets
  wrong and no metadata stripping can repair. `src/checks/moonlimb.js`
  (pure, 18 mathcheck assertions): the predicted limb direction is a sky
  point stepped from the moon toward the sun and projected through the
  photo's REAL pose (roll and lens distortion included — no hand-derived
  angle conventions to get wrong); the measured direction is the lit
  region's MINOR principal axis (every phase shape is mirror-symmetric
  about the limb axis), signed by the width taper (the terminator end is
  wide, the limb end tapers like a rim) — center-error-free by
  construction, recovering the limb direction to <2° on synthetic
  crescent, half AND gibbous discs. Verdicts have an inconclusive gray
  zone and hard gates: full/new moons, weak asymmetry, or a
  non-moon-sized blob yield NO verdict — this forensic never guesses.
  Agreement lands as positive evidence in the Authenticity section
  ("its lit limb points within N° of where the sun requires"); a big
  mismatch warns with the honest triage order ("a wrong stated
  time/place or a mis-set placement also shifts it; re-check those
  first"). End-to-end proven both ways on synthetic zoomed moon photos
  built from the real ephemeris: the correctly-lit disc attests, the
  140°-rotated composite warns and auto-opens the section (5-assertion
  browser e2e).
- **Aurora, contrail and sky-lantern context checks** (the small trio
  from the list). ✨ AURORA: for night sightings at auroral-capable
  latitudes the report pulls the GFZ Kp geomagnetic index for the
  sighting instant (full history, CC-BY, via a new `/api/kp` proxy —
  GFZ sends no CORS) and compares it against the observer's GEOMAGNETIC
  latitude (centered-dipole transform in `src/checks/aurora.js`):
  "Aurora possible on the poleward horizon — Kp 6.3 puts the oval's
  edge near geomagnetic 54°" through to overhead-oval for real storms.
  ✈ CONTRAILS: the conditions table gains flight-level humidity
  (Open-Meteo 250/300 hPa, reaches ~3 months back) with the useful
  reading being the NEGATIVE — "dry aloft: a long-lasting white trail
  that day was probably NOT a contrail" — alongside the
  persistent-contrails-expected positive. 🏮 LANTERNS: a pure calendar
  check (no network) flags night sightings on July 4, New Year's Eve
  and Lunar New Year (embedded 2015–2035 dates): sky lanterns are
  drifting orange points that flicker and fade, riding the surface
  wind — the note points at the wind check for the comparison.
  17 mathcheck assertions (dipole latitudes vs known cities, verdict
  bands, Kp binning, date windows incl. small-hours spillover, RH
  bands) + 5-assertion browser e2e on a mocked Kp-6.3 July 4 night.
- **🪖 Military airspace check — FAA special-use airspace vs the observer
  and the sight-line** (next on the list). A new report section answers
  whether the observer stood inside — or was looking into — a Military
  Operations Area, Restricted, Prohibited, Alert or Warning area (FAA
  AIS ArcGIS open data via a cached `/api/airspace` proxy; US airspace
  only, stated). Pure geometry in `src/checks/airspace.js`:
  ray-cast point-in-polygon, floor/ceiling parsing (SFC → 0, FL → ×100
  ft), a sampled march along the sight-line azimuth that reports where
  the ray enters and leaves each zone ("sight-line enters it 20–58 km
  out"), edge-true nearest distance, and a deliberately light schedule
  read that flags when the sighting falls inside a zone's published
  window — returning honest "unknown" for anything it can't parse,
  since schedules change by NOTAM. Assessment weights the military
  explanation heavily when inside or looking into a zone, and the
  caption notes the sharp edge: military aircraft in these blocks often
  fly WITHOUT ADS-B — exactly when the aircraft check goes blind.
  Probed live (the Rogue Valley's real Dolphin North/South MOAs return
  correctly). 11 mathcheck assertions + 7-assertion browser e2e.
- **🎈 Weather-balloon check (radiosonde) — real data for the #1 mundane
  explanation** (next item on the user's go-ahead list). Two independent
  layers in a new report section. LAUNCH SCHEDULE: the worldwide SondeHub
  launch-site catalog (~900 stations with positions, synoptic launch
  times, measured ascent rates and burst altitudes; `/api/sondesites`
  proxy, week-cached) answers for ANY sighting age whether a scheduled
  balloon was actually AIRBORNE at the stated time — and where in its
  flight: "airborne — ascending, ≈ 9.4 km up (launched 4:00 PM)".
  RECEIVED TRACKS: actual radiosonde telemetry near the observer
  (`/api/sondes` proxy — live near-point query for fresh sightings, the
  global telemetry archive distance-filtered server-side for older ones)
  ranked against every sight-line exactly like aircraft: range, altitude,
  predicted angular size from the envelope's altitude-grown diameter
  (~1.5 m at release → ~7.5 m at burst), and the full 🎯 trajectory
  match when the witness has a timed track — reusing the same pure
  track-time geometry the aircraft check ships. Field-validated against
  the real network: the actual Aug 11 Medford 00Z sonde comes back as a
  full ascending track from the launch point 9 km from the user's test
  area. Honest both ways: no received telemetry "rules out received
  sondes, not balloons" (volunteer receiver coverage), and the schedule
  layer still answers when no receiver heard anything. 10 mathcheck
  assertions (site parsing, launch-window physics incl. descent phase,
  candidate ranking) + 6-assertion browser e2e with a synthetic balloon
  flight driven through the real report.
- **✈ ADS-B track-time matching — trajectory against trajectory, not one
  instant** (roadmap item, user go-ahead). With a timed witness track (a
  tracked video's dense path, or timed waypoints) and archived traffic,
  each aircraft's whole ±4-min flight path is interpolated to every
  witness sample AT THE SAME WALL-CLOCK TIMES and scored by angular
  separation (worst witness governs; never extrapolates beyond the
  recorded trail; fewer than 3 overlapping samples → no verdict).
  A single-instant separation can be coincidence — a whole path matching
  at the right times essentially IS the aircraft, and a divergent one
  genuinely rules that aircraft out. The results panel badges each
  candidate (🎯 tracks the witness path / ◎ roughly parallels / ✗ path
  diverges), re-ranks by trajectory when it ran, and adds a conclusive
  assessment line on a real match; the report's aircraft section prints
  the same verdict. Proven end-to-end with two mocked aircraft placed
  IDENTICALLY at Moment A: the one whose archive trail flew the
  witness's path scored mean 0.0° and 🎯, the 30-second time-shifted
  decoy scored 21° and ✗ — a discrimination no instant ranking can make.
  8 mathcheck assertions against synthetic exact truth + 5-assertion
  browser e2e.
- **Six new validation checks** (user ask: "the easy ones" — each reuses
  existing infrastructure, all pure-math cores mathcheck-asserted, key UI
  paths browser-verified 13/13 + shadow/auth regressions green):
  1. **EXIF close-subject tells.** The parser now reads `SubjectDistance`,
     `Flash` and `SubjectDistanceRange`; the capture step's 🔬 panel warns
     at upload when the camera itself says its subject was close — focus
     locked metres away, a macro/close focus range, or a flash whose
     strobe-return bits say its light bounced back (a phone flash reaches
     a few metres). Devastating and honest against a "distant craft"
     claim; rides authDerived into the report automatically.
  2. **🕐 Sundial inversion.** The ⚑ shadow gadget runs backwards: a dial
     rotates a teal ghost shadow, and phodar prints the time(s) of day
     the sun actually casts that way at that place — compare with the
     stated time (recovered a test capture time to <1 min; a direction
     the sun never produces is itself a finding). Ghost length uses the
     matched instant's real sun altitude.
  3. **⛰ Terrain line-of-sight.** When a fix solves, each witness's ray
     to it is marched through the DEM (same model as the skyline):
     a ray passing below a ridge means the geometry is impossible as
     stated — surfaced as a red warning in results and the report, with
     a quiet all-clear line when it passes. Noise-guarded so a grazing
     ridge is never declared a wall.
  4. **🛣 Vehicle-light check.** For night sightings the report shows
     where the sight-line crosses mapped roads (name, distance, the road
     point's own elevation angle) — a car cresting a rise reads as a
     hovering light, and now it has a name and a distance.
  5. **📡 Tower & mast check.** New `/api/masts` Overpass proxy + pure
     ranking: masts, towers, chimneys and lighthouses within 25 km and a
     few degrees of the sight-line, with height, distance, and the top's
     elevation over the terrain model — obstruction strobes are the
     classic pulsing "hovering light". States that OSM is incomplete.
  6. **✨ Satellite flare geometry.** Every satellite pass now carries the
     specular-glint angle (sun reflected off an earth-facing panel toward
     the observer) and phase angle; the dome chip hints "✨ flare?" and
     the report's sky-object check explains when a brilliant
     swell-and-vanish flare was geometrically possible — stated as
     possible, never predicted (attitude unknown). Asserted offline via
     the exact sub-satellite-point invariant (glint = phase at zenith).
- **⚑ Shadow check v2: tilt-to-place, moonlight shadows, and a report
  compass diagram** (user asks). The flagpole now stands where the
  CENTER OF YOUR VIEW meets the ground — tilt down and it comes closer,
  tilt up and it moves out (6 m floor, 300 m cap), so standing it right
  on a shadow you want to compare is a tilt, not a setting; the status
  line shows the live standoff. After dark the check switches to the
  MOON when it's up: a bright moon draws a faint blue moonlight shadow
  with its % lit stated ("93%-lit moon ⇒ faint shadow toward NW, needs
  dark skies"), a thin moon (<30% lit) is honestly "too dim to cast
  usable shadows", and with neither sun nor moon up the line says ANY
  visible shadow in the photo contradicts the stated time and place —
  a shadow the light couldn't cast is never drawn. The REPORT's
  Authenticity section gains a "Shadow geometry" block per observer: a
  self-contained compass-rose SVG (arrow = shadow direction, ☀/🌙 glyph
  at the source azimuth) plus the exact direction and the
  length-per-object-height ratio — checkable against the photo,
  unfakeable by stripping metadata. 8 new mathcheck assertions
  (gaze-to-ground standoff mapping, sun/moon/none arbitration against
  real ephemeris, ratio and direction math) + a 15-assertion browser
  e2e across day, bright-moon and dark-night cases including the
  report block.
- **⚑ Shadow check — a sun-shadow flagpole gadget on the sky view** (user
  ask). A new header toggle plants a schematic 5 m flagpole on level
  ground at the center of your view (it follows as you pan, and sits
  under the photo in place mode) and draws where the sun at the sighting
  time would throw its shadow — dark band on the ground, dashed
  centerline, ⚑ marker at the pole top. It is a physics cross-check
  against shadows visible in the photo: metadata can be stripped, but
  shadow direction at a stated time and place cannot be faked. The
  status line states the sun's az/alt, the shadow's length and compass
  direction, and the honest caveat (direction exact; length assumes
  flat ground). Honours the 📷 camera height (the pole moves out to
  stay in view from rooftops), and when the sun is below the horizon it
  says so — visible shadows would then contradict the claimed time —
  drawing the pole but never a fabricated shadow. Pure geometry in
  `src/shadow.js` (15 mathcheck assertions: vertical-pole azimuth
  invariance, H/tan(alt) length, away-from-sun bearing, exact tip
  position, grazing-sun cap honesty, no-shadow-at-night). Dome-only by
  design — the pole is fictitious, so it is never burned into
  world-locked video exports (same rule as the winds stack). Verified
  in-browser day and night (12 assertions).
- **Authenticity checks: upload forensics, physics consistency, and a
  report section with a loud manipulation banner** (user ask; signals
  fire the moment their inputs exist rather than all at the end).
  `src/checks/authenticity.js` (pure, mathcheck-asserted) scans the
  ORIGINAL uploaded bytes — before any canvas normalization — for
  editing-software fingerprints: AI-generator metadata (Stable
  Diffusion / ComfyUI / Midjourney / DALL·E / Firefly and friends,
  including the `Steps/Sampler` parameter block PNG generators embed),
  C2PA content credentials (a `trainedAlgorithmicMedia` assertion is an
  alarm; ordinary provenance is a note), Photoshop/GIMP/editor traces,
  PNG text chunks, JPEG structure notes (Ducky/Save-for-Web,
  progressive re-encode, APP14), and for video the Lavf/ffmpeg
  re-encode and ReplayKit screen-recording marks. Findings are ranked
  alarm → warn → note → info and shown in a 🔬 File authenticity panel
  on the capture step AT UPLOAD (red border when an alarm fires).
  Derived checks join as their inputs arrive: once position + time
  exist, the position step compares the photo's measured brightness
  against the computed sun elevation (a bright scene at astronomical
  night, or a black frame at midday, is a physics inconsistency no
  metadata stripping can hide) and flags file-time-vs-stated-time and
  EXIF-GPS-vs-pin mismatches; a star or terrain calibration registers
  as positive evidence the sky matches the claimed time and place. The
  report gains an "Authenticity checks" section listing every finding
  per observer plus what was NOT tested (pixel-level splice forensics,
  AI-detector models, reverse-image search), and any alarm puts a
  ⛔ MANIPULATION INDICATORS DETECTED banner above every other section —
  the report then explicitly describes the FILE, not necessarily a real
  event. Honest epistemics both ways: a clean scan proves nothing (any
  fingerprint can be stripped), so absence of findings is never sold as
  authenticity. 17 mathcheck assertions on the pure module + a
  10-assertion end-to-end browser run (AI-marked PNG alarms at upload,
  night-brightness check fires on step 2, banner sits above every
  report section, plain camera JPEG trips nothing).

### Fixed
- **Help-menu audit: coverage verified complete, six new drift guards
  added** (user ask). A browser sweep enumerated all 126 interactive
  controls across every screen (home, capture step empty/photo/video,
  position, sky view + place mode, results, report) and matched each
  against the manual — zero gaps at both lenient and strict word-match
  thresholds (the only unmatched string is the home tagline, which is a
  subtitle, not a control). What was missing was future-proofing:
  recently shipped features were documented but UNGUARDED, so their
  manual entries could be deleted without anything failing. helpcheck's
  NAMED list gains six: the roads overlay, the bring-your-own-AI card,
  the PWA install hint, the report-link import, the keep-the-metadata
  how-to, and the desktop gesture wording (27 named features guarded,
  up from 21).
- **Report audit: every recent feature now discloses itself in the final
  report** (user ask: make sure new functionality is accounted for).
  Verified already present: track-quality grading with the
  camera-workload caveat, geometric clock-sync shifts, visibility
  seconds used vs ignored, the analysed trim span, flight-log
  calibration honesty, and the calibration method. Four gaps found and
  fixed: (1) **⚓ manual pose anchors** were invisible — a
  human-corrected camera path is different evidence, so the video
  section now states how many frames were re-anchored and that
  corrections blend between anchors; (2) **sensor-fused camera paths**
  were invisible — the report now says when frames were carried by the
  phone's motion sensors, and when the whole path is sensor-built
  (visual solve couldn't follow); (3) **report-link provenance** — a
  sighting filled from a public report page now cites that page's URL
  beside the witness accounts; (4) the **video-analysis section
  vanished entirely for a video whose clip is gone** (evicted media, or
  any imported share — share files never carry the video) even though
  every measurement was present: the gate keyed on an attached
  mediaUrl, and the boot deliberately drops mediaKind for lost media.
  It now keys on mediaKind OR mediaLost === "video", renders the full
  kinematics/rate-plot/distance-ladder from the measurements, skips
  only the keyframe strip, and says plainly that the clip isn't
  attached. Also: **track quality "excellent" is now stated instead of
  silent** — a reviewer should never infer quality from a line's
  absence. Browser-verified (7 assertions on a seeded sighting
  exercising all of it) plus the 45-assertion export round-trip
  regression.
- **Export audit: two losses found and fixed, everything else verified
  intact** (user ask: make sure the exported file captures all useful
  data). The share pipeline is structurally sound — `packSources` strips
  a NAMED list (media handles + working state) and passes everything
  else through, so new fields flow into exports automatically. The
  audit seeded a source carrying every field any feature writes,
  exported through the real UI, and diffed both the share file and a
  re-import into a fresh profile (45 assertions). Found: (1)
  `track[].wpx` — a PIXEL width — was not rescaled when a big photo is
  packed down to its 1600 px working copy, so an imported still drew
  its sized track ghosts 1/k too large; it now rescales with every
  other pixel-space field (`ang`, being angular, correctly never did).
  (2) `mediaKind` was stripped, so a JSON-only import of a video
  sighting forgot it ever was one; the one-string kind now survives
  pack AND import (media itself stays stripped, as designed).
  Also documented in-code that `packMoment` drops a moment's stray
  `track` deliberately (a moment's trajectory contribution is its
  placed A direction; a duplicate track would double-count). Verified
  unchanged through the full round-trip: marks, shape fit (rescaled
  consistently), solved camera/object paths + raws, pose anchors,
  sensor log + sync, camera refs, EXIF meta, brightness/contrast, trim,
  align frame, camera height, ADS-B snapshot, flight log, report-link
  provenance, statement, position/time, and the est block; report.html
  embeds the same packed JSON, so the report import path inherits all
  of it.
- **Road ribbons: camera height factored in properly + near-field ground
  lock** (field ask: "match true perspective even better; make sure
  camera elevation/height off ground is factored in"). Three fixes:
  (1) the 📷 camera-height nudge row only appeared with the buildings
  layer — it now shows whenever roads OR buildings are on (one eye
  height scales both), with a finer 0.2 m step and the floor lowered
  from 1.6 m to 1.0 m, because a windshield shot sits ≈1.2 m and at
  ribbon scale the difference is visible in the near road (the hint now
  says so). The horizon strip's two eye clamps got the same 1.0 m floor.
  (2) NEAR-FIELD GROUND LOCK: the DEM is a ~20 m grid, and one cell of
  wobble at 50 m is metres of height — visibly floating or sinking the
  very road the observer stands on. That road IS the observer's ground,
  so near-field elevations lock to h0 and blend into the real DEM by
  ~380 m, where a metre is sub-line-width. (3) The occlusion march sees
  the same lock but only ever LOWERED — a noise berm above eye level
  would otherwise wall off everything behind it (the terrain.js
  foreground-berm lesson), while raising samples would fabricate an
  apron that erases a genuinely visible valley road seen from a ridge
  road; min() kills berms and keeps valleys. Near segments also densify
  finer (~6 m inside 400 m, run-in to ~1.5 m) so the ribbon reaches the
  bottom of the frame. All mathcheck-asserted (near wobble locked to
  ground, real DEM beyond the ramp, berm-immune occlusion with the hill
  case intact); browser regression passes.

### Added
- **🛣 Roads are RIBBONS now, not threads** (field ask on the first
  render: "the road you are on should look big — accurate width, follow
  terrain, match reality"). Each road carries its real roadway width
  (explicit OSM `width` tag → `lanes` × 3.4 m + shoulders → per-class
  default, 19 m motorway down to 4 m service lane) and renders as left/
  right EDGE lines offset perpendicular to the centerline — so the
  highway you stand on fills the frame and converges to its true
  vanishing point exactly like the photo — plus a subtle asphalt fill
  (per-segment quads, immune to horizon/behind-camera cuts) and a
  dashed amber CENTER line on major roads, the way a real highway reads.
  Near-field segments are densified (~10 m steps inside 300 m) because
  perspective magnifies close range — a 40 m Overpass chord is visibly
  angular at 30 m out. Same treatment in all three renderers (dome,
  world-locked export, horizon strip). Mathcheck-asserted: ribbon width
  at the feet = 2·atan(w/2 ÷ d) (28.1° for a 10 m primary at 20 m),
  edges converge >8× from near to far. Browser-verified: edge pair +
  fill quads + dashed center all present on the mocked highway.
- **🛣 Roads overlay — OSM road centerlines in true perspective** (field
  ask: a highway shot straight down the centerline had nothing to align
  against; a road is the best azimuth anchor FLAT terrain has, where no
  ridge exists to snap to). New pure module `src/roads.js` + Overpass
  proxy `/api/roads` (mirror-raced + cached like /api/buildings):
  centerlines within ~2.6 km are clipped, simplified and
  nearest-first-budgeted, then each vertex takes its REAL ground
  elevation from the same cached DEM tiles the terrain skyline uses
  (same curvature+refraction constant, same sea-level clamp) and is
  ray-marched against the DEM so hills genuinely hide the stretches
  behind them (approximate: computed at derive time with the default
  eye). Drawn in three places from one derivation: the sky-view dome
  (new 🛣 header toggle; slate-white polylines, major roads brighter,
  distance-faded like the ridge haze; the camera-height nudge moves
  roads and rooftops together), the world-locked video export (same
  layer, honoring the toggle), and the position step's horizon strip
  (both profile and 3D-vista modes — aim the viewing-direction ray down
  the photo's own road). DEM unreachable → flat-ground elevations at
  the observer's height, flagged honestly in the status line. Pure
  parse/projection/occlusion mathcheck-asserted against synthetic truth
  (ENU conversion, flat-plane el = −atan(eye/d) with curvature, a hill
  occluding the far stretch while the near stretch survives);
  browser-verified end-to-end with a mocked Overpass answer (chip
  count, status line, dome polylines). Manual updated (sky-layers 🛣
  item + position-step ⛰ item).
- **📲 PWA install hint on the home screen** (Apple devices only —
  Android/desktop Chrome surface their own install prompt; Apple never
  does). iPhone/iPad get the Share → Add to Home Screen steps, Mac
  Safari gets File → Add to Dock. The load-bearing reason is stated
  honestly: Safari can erase a website's saved data after ~7 days of
  disuse (the "my videos disappeared" field failure) and an INSTALLED
  app is exempt — plus full screen without browser bars fighting the
  marking gestures, and 📷 Capture with sensors in a clean camera view.
  Hidden when already running installed (display-mode standalone /
  navigator.standalone) and dismissable with ✕ (persisted). The
  manifest + icons already shipped, so installs were always possible —
  just never suggested. Manual entry added; browser-verified across
  iOS/Mac/non-Apple UAs including dismissal persistence.
- **Step 1 is now "THE CAPTURE"** (it takes photo or video), and the empty
  step got useful instead of noisy (field asks): before anything is
  loaded, a note says GPS/time/bearing/lens metadata auto-fill when the
  file carries them — use the original, not a re-send — with an
  "ⓘ keeping the metadata" popover explaining how to move a capture
  between devices without stripping it (AirDrop with All Photos Data ON,
  Files/Drive instead of chat apps on Android, HEIC → JPEG). The camera
  FOV fields and the measured-angular-size readout are hidden until
  media is loaded — they're properties of the capture, and the
  skip-media path never needs them here. One exception kept: a
  view-only sighting whose media was never re-attached still shows its
  FOV readout (that number came from the original file). Manual updated
  (section renamed "Step 1 — The capture", ⓘ documented); browser-
  verified before/after upload.
- **"🤖 Bring your own AI" card on the home screen** (user ask: a section
  that explains the API with a full prompt to copy into your AI of
  choice). Collapsible card at the bottom of the sighting list: explains
  the MCP server in plain language (your AI is the eyes and
  context-gatherer, Phodar does the math, you own the review), an
  optional API-key field (persisted locally; keys stated as free and
  handed out personally — Phodar will never charge), and "📋 Copy the
  AI prompt" — a complete
  ready-to-paste prompt carrying the MCP URL (key filled in when
  entered), the five-step workflow (fetch_report → ingest_media with
  trim guidance → keyframe look + inspect_frame confirmation →
  auto_measure with every real fact → job_status + bundle link), and
  the honesty rules (never invent values, declare guesses, placement
  approximate until refined in the app). Clipboard-blocked browsers fall
  back to a selectable prompt; the full text also renders in the card.
  Browser-verified: placeholder ↔ key completion, persistence across
  reload, workflow text present.

### Changed
- **Report-link import parked behind `ENABLE_REPORT_LINK` (default off)**
  after the first production attempt confirmed ufosighting.report's
  Cloudflare bot protection 403s the server fetch (and a reader-proxy
  fallback gets the same challenge page — no clean workaround that isn't
  bot-protection evasion). Notably the site's own robots.txt ALLOWS
  general crawlers (`User-agent: * → Allow: /`; only the big AI-training
  bots are disallowed, PhodarBot is not among them), so the block is
  Cloudflare acting against the site's stated policy — the ask to
  allowlist PhodarBot stands on solid ground. The `/api/report` endpoint
  stays live so the allowlist can be verified without a redeploy; flip
  the flag when it lands.

### Added
- **Report-link import** (user ask: drop a ufosighting.report link into
  Phodar and it extracts the coordinates, metadata and witness statement;
  the user downloads the video themselves). Home screen gains "🔗 Fill
  from a report link": paste a report page URL (or drag the link onto the
  screen on a desktop) and a new observer is pre-filled from what the page
  states — position, date/time, the witness statement, and the sighting
  name from the page title. Server side is a new un-keyed `GET
  /api/report?url=` that wraps the existing phase-2 `fetchReport` scraper
  (og:/twitter: metas, JSON-LD, datetime/geo hints, media links);
  because it is un-keyed it is HOST-ALLOWLISTED (default
  ufosighting.report, extend via `PHODAR_REPORT_HOSTS`) so it can't be
  used as an open scrape/SSRF proxy. Interpretation is deterministic and
  JSON-LD-first (structured data beats loose page-text hints), and every
  value lands in normal wizard fields for review — the import message
  says what was extracted and warns that a report's location is often
  the town, not the exact spot. The media is DELIBERATELY not
  downloaded (another site's video is not ours to hot-pull, and browser
  CORS blocks it anyway): step 1 shows a callout linking back to the
  report page and its media files — save them there, then load with the
  normal picker, and every imported measurement stays. Verified offline
  end-to-end against a mock report site through the real server + app
  (9 assertions: position/time/statement/name/media links all land,
  step 2 unlocks). Honest caveat: ufosighting.report currently 403s
  non-browser clients (Cloudflare bot protection — confirmed again from
  this sandbox), so until PhodarBot is allowlisted there the import may
  fail with a clear message telling the user to copy the details by
  hand; the flow degrades, it doesn't break.
- **Modality-aware gesture hints** (user ask: every screen/mode should hint
  mouse actions on desktop and finger gestures on mobile). A module-level
  switch (`FINE_PTR`, from the `(hover: hover) and (pointer: fine)` media
  query, evaluated once at load) picks the WORDING of every gesture hint —
  the in-app manual and all inline hint lines. Touch devices read
  pinch / twist / second finger exactly as before; a mouse-driven page
  reads scroll / Shift+drag / Alt+drag / click. Covered: step 1's shape
  readout, adjust-mode line, bottom marking hints and the manual's
  photo-step items; the sky view's place/look banners, fix-frames banner,
  🎛 and ⚓ tooltips and the manual's sky items; the view-only manual
  entry. The "On a DESKTOP" summary tips now render only on desktop (they
  were noise on a phone). Gestures themselves are unchanged — both input
  paths always work; only the teaching text adapts. helpcheck reads the
  source, where both wordings remain visible, so coverage still asserts.
  Browser-verified: the same build shows mouse wording in a desktop
  context and touch wording in an emulated-mobile context across step 1,
  the manual and the sky view (12 assertions).

### Fixed
- **Desktop scrollbar no longer collapses on step 1** (field report: "when
  holding shift the main page's scroll bar disappears which shifts
  everything over a little"). Any press on the marking canvas engages the
  body scroll-lock (`overflow: hidden`) that exists to stop iOS page
  scroll / pull-to-refresh during a touch drag. iOS scrollbars are
  overlays with no layout width, so touch never saw a side effect — but a
  desktop scrollbar has real width, so every mouse press shifted the whole
  centered column sideways. The lock is now touch-only (gesture pointerType
  tracked in a ref): a mouse drag can't scroll the page, so it never needed
  the lock. Browser-verified: mouse press/drag leaves body overflow
  untouched; a touch press still locks and releases on lift.
- **Place-mode desktop follow-ups** (field report). Three fixes:
  (1) Shift+scroll zoomed one direction no matter which way the wheel
  turned — with Shift held, browsers deliver the wheel delta on `deltaX`
  (the horizontal-scroll convention), so the `deltaY > 0` test was stuck
  false; the handler now reads whichever axis moved. (2) No modifier is
  needed anymore: in place mode the scroll wheel resizes the photo's field
  of view directly (the two-finger pinch) — the look-mode view zoom is
  meaningless while placing since the view is slaved to the photo, so
  plain scroll and Shift+scroll both do the pinch. (3) "Still can't zoom
  out enough to see the whole image": the placed photo's width floor was
  0.5 of the viewport — tuned for portrait phones, but on a LANDSCAPE
  window the visible band is short relative to the width, the height-fit
  fraction for a portrait photo computes ~0.2–0.3, and clamping it up to
  0.5 left the photo taller than the band with no way out (display zoom
  can't go below 1×). The floor is now 0.15 on landscape viewports
  (portrait unchanged), with a second floor at tan(fovM/2)/tan(67.5°) so
  the derived view FOV never hits its 135° cap and un-locks the sky
  overlays from the pinned photo. Also helps iPad landscape. Browser-
  verified at 1440×810 with a portrait photo: whole photo inside the
  band, wheel resizes FOV both directions with and without Shift, look
  mode still zooms the view.

### Added
- **Desktop audit — the rest of the app** (user ask: "I have really only
  used the app on my iPhone and iPad up until now — audit for anything else
  that needs to get optimized for desktop"). Audited every screen at a
  desktop viewport; the gaps were all interaction paths that only existed
  as two-finger gestures or touch idioms, fixed with mouse/keyboard
  equivalents gated so the field-calibrated touch stack is byte-identical:
  - **Measure step (the big one — the marking screen had NO mouse zoom):**
    scroll-wheel zooms the photo toward the pointer (the pinch, same
    clamps), Shift+drag pans while zoomed (the two-finger drag; a viewing
    gesture, so it works in view-only mode and never places a mark), and
    Alt+drag rolls the fitted shape or the selected track point's model
    about the view axis (the two-finger twist). All pointerType-"mouse"
    gated.
  - **← / → step one frame** on a loaded video (the keyboard −1/+1 fr) —
    skipped whenever a form control has focus, so typing in the statement
    box or nudging the frame slider's own native arrows never double-acts.
  - **Escape closes things**: the ? manual and the 📍 distance-map modal
    (capture-phase, so only the top overlay closes), and the sky view —
    through the SAME path as the visible ‹ Back / ✕ button, committing the
    placement exactly like a tap.
  - **Drag-and-drop files**: a photo/video dropped anywhere on the measure
    step loads it (same `ingestFile` path as the picker, with a drop
    highlight on the Load button); a share file (.phodar.json / report
    .html / sighting .zip) dropped on the home screen imports it.
  - Checked and already fine: Leaflet maps (wheel zoom + drag native),
    trim bar and all sliders (pointer events + cursors), the portrait
    rotate-lock (gated `pointer: coarse` + small screen — desktops can't
    match), the 520 px centered column, report/results scrolling, and the
    storage/attitude fallbacks documented in CLAUDE.md.
  Browser-verified at 1440×810 through the real built app (14 assertions:
  zoom/pan/roll each move the render and place no marks, drop replaces
  media, arrows step 0.05s→0.15s and ignore focused inputs, Escape closes
  the manual and exits the sky view to the wizard). Manual updated
  (photo-step + sky-view desktop tips, import item); helpcheck passes.
- **Desktop (mouse) support in the sky view** (field report: "you can't zoom
  out enough to show the full perimeter of the image, and there's no way to
  drag/rotate like the mobile gestures"). Two causes, two fixes. The
  zoom-out cap was 90° of HORIZONTAL field — tuned on portrait phones, where
  the vertical then exceeds 120°; on a landscape desktop window the same cap
  leaves only ~55–60° of vertical, less than a portrait photo spans, so the
  perimeter could never fit. The cap is now aspect-aware (the short axis can
  reach 90°, tangent-scaled like the projection itself, max 130°), applied to
  the scroll-wheel, the +/− buttons, pinch and the auto-fit — portrait
  viewports unchanged. And the two-finger gestures got mouse equivalents,
  gated to pointerType "mouse" so the field-calibrated touch stack is
  untouched: Shift+drag rolls the photo (place and ⚓ fix modes — the twist),
  Shift+scroll resizes the photo's FOV in place mode (the pinch); plain drag
  (look/place/fix/trackball) and plain scroll-zoom already worked. Manual
  updated. Browser-verified at 1440×810: zoom-out reaches 121°, Shift+drag
  persists roll −30° through commit, Shift+scroll takes fovH 41.6°→55.8°.
- **Phase 2 field-test fixes** (first live run against a real
  ufosighting.report record, driven by an independent AI client):
  ffmpeg now installs via `nixpacks.toml` (the `railway.toml` nixpacksPlan
  request didn't take on Railway; `/api/health` now reports `ingest:
  true/false` so a deploy can be verified remotely, and an ffmpeg probe miss
  is retried after 60 s instead of disabling ingestion for the process
  lifetime). **Trim-at-ingest**: `ingest_media`/`POST /api/ingest` accept
  `trim {t0,t1}` (span ≤150 s) and remux-copy just that span straight off the
  URL via range requests — the route for 4K phone clips (both SeaTac
  originals were 590–750 MB, over the 300 MB download cap). The media fetcher
  and report scraper send a browser-class user-agent carrying an honest
  `PhodarBot/1 (+https://phodar.app)` token (the bare bot UA got 403'd);
  when a site still blocks the scraper, the AI reads the page itself and
  passes the media URL — that path worked in the field test. analyze_session's
  tool description now states the minimal session shape (the tester had to
  reverse-engineer it).
- **Phase 2 — raw-media ingestion: hand an AI a video/photo URL (or a report
  page) and get back a reviewable phodar sighting.** The server now runs the
  real measurement pipeline on raw media: EXIF/QuickTime metadata, object
  snap + sizing, the full video stabilizer (per-frame camera pose, same walk
  as the app: bisection, held-run bridging, despike, smoothing) and the
  guided object auto-tracker — then returns a draft `.phodar.json` plus an
  importable bundle (.zip with the original media) for HUMAN review in the
  app. The division of labour is explicit: the AI is the eyes and the
  context-gatherer — `ingest_media` returns keyframes as images the AI looks
  at, `inspect_frame` returns a ×3 crosshair crop to confirm the object
  mark, and the AI passes position/time/bearing/witness text gleaned from
  the report — while phodar is the instrument, and every defaulted value is
  listed in `source.ingest.guessed` for the human to review. The sky
  placement is carried as approximate until refined in the app (star-align /
  terrain snap / by hand).

  New: `src/ingest/{media,auto,serve,worker}.mjs` (ffmpeg decode; the
  pipeline; jobs + report-page scraping; a forked worker so minutes of solve
  never block the dyno serving the app), HTTP `/api/ingest`, `/api/measure`,
  `/api/job/<id>[/bundle.zip|/session.json]`, MCP tools `ingest_media`,
  `inspect_frame`, `auto_measure`, `job_status`, `fetch_report` (best-effort
  page scrape — og/JSON-LD/media links — written blind, needs field
  testing), and ffmpeg in the Railway image. The app's zip writer moved to
  `src/report/zip.js`, shared by both sides and mathcheck-asserted.

  Verified end to end on the real Germany clip driven through the actual MCP
  conversation — keyframes seen, object confirmed at the crosshair, job
  polled to done: auto sight-line 359.54°/35.13° vs the human session's
  359.85°/34.91°, an 86-point camera path tracking the tilt to ~57°, an
  84-point object track, and the produced bundle imported through the real
  app UI with every measurement intact.
- **👁 View-only mode — a master Edit/View toggle on the home screen** (field
  ask: "a view-only mode for people who just want to load a finished
  file/bundle and see the steps and everything without making any changes").
  It sits above Import, because that is the order a review actually happens in:
  set the mode, then load the file you were sent.
  It governs the whole app and is remembered between sessions. Every editing
  tool is hidden throughout — the media row, tap-mode selector and shape
  controls on step 1; the search, coordinate inputs and pin-move on step 2;
  ✥ Place, 🎞 Stabilize, ⚓ Fix frames and 🎛 Smoothing in the sky view;
  add-witness, new-sighting and the sighting name — while everything that only
  READS stays live: pinch-zoom the photo, scrub and play the clip, look around
  the dome with every sky layer, the full results panel, the report, the
  bundle and the share file. Import stays enabled in both modes, since that is
  how a reviewer loads the sighting in the first place.

  Enforced at the DATA layer, not by hiding buttons: `updateSource` /
  `updateMoment` are the only way any measurement changes, so review mode
  closes them and a blocked write says so in a toast. Hiding the controls is
  the honesty layer on top — a mode that silently swallowed input would be
  worse than no mode. A 👁 VIEW ONLY badge sits in every step header (and the
  sky view's HUD) so a missing control is never a mystery. Verified
  end-to-end in a real browser: 34 assertions across the full wizard walk,
  including that the stored sighting is byte-identical after reviewing every
  screen.
- **Brightness/contrast stays usable in view-only mode**, and every control
  that WOULDN'T work there is now hidden (field asks). B/C is a display aid
  that never touches the original pixels, so a reviewer can brighten a dark
  clip to see the object; it is allowlisted by patch key, alongside the fields
  the app re-derives from the media file on every load (natural size, the
  canvas-normalized URL) — blocking those protected nothing and risked leaving
  an imported sighting without the dimensions it needs to draw. The FOV
  default-guess that used to ride along with them is suppressed instead, since
  that IS a measurement input. The dead-control audit was done by enumerating
  every visible button and input per screen in a browser: gone in review mode
  are ⛰ Align on this frame, the FOV preset + custom field and the witness
  statement box (step 1); all four aim sliders — bearing, FOV, up-angle,
  camera height (step 2, replaced by a readout); the home screen's remove-
  observer ✕, ＋ Add moment and name field (now shown as the title it is); and
  the flight-log loader plus the 🛩 calibration link when no log is present.
  Verified: 17 assertions, and a keyed diff proving no MEASUREMENT changed
  across a full review walk.
- **In-app manual brought back up to date**, and `helpcheck` extended so these
  can't drift again (21 named features guarded, up from 14). New entries: the
  Edit/View master toggle; ‹ / 🏠 navigation on the last two steps; what the
  stabilizer can track (a cloud-only sky, and the frame-to-frame lock that
  carries a sweep off the reference frame, reported per-clip); the 🎥 track
  quality rating and what lowers it; why the maneuver-load g is an upper bound
  on a heavily-stabilized clip; and the aircraft sky-tracks drawn on the dome.
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
- **Stabilizer tracks clouds** (field ask: "most UFO videos have nothing to
  reference except clouds"): a soft cloud edge ramps brightness over 10–20 px,
  so its per-pixel gradient sat below the corner detector's gate and whole
  cloud banks contributed zero references — the camera solve starved on
  exactly the clips that need it most. When the strict pass finds too few
  references (<16), a soft-sky fallback now re-detects on a 3×-downsampled
  frame (gradients concentrate ×3, so soft structure becomes corner-sharp)
  and those features track with a larger template patch and a relaxed
  flat-patch gate. Clips with real corners are byte-identical — the fallback
  only fires when starved. Cloud drift is inconsequential at these
  timescales (a 2 km cloud in 20 km/h wind moves ~0.3° over a 21 s clip,
  dwarfed by multi-degree camera motion) and the pose solve's median trim
  absorbs the residual. Mathcheck-asserted: the strict detector starves on a
  synthetic cloud sky, the fallback recovers 50+ references, and a 10-frame
  pan is held to 0.07° on clouds alone.
- **Stabilizer chain register — sweeps across the dome are no longer lost**
  (field case, the Germany sighting: the camera tilts from 30° elevation to
  near-zenith on a cloud-only sky, and the solve froze at 42° — the object
  trajectory piled into a knot instead of climbing the dome). Once the view
  pans or zooms off the marked reference frame, the absolute whole-frame
  registration has nothing to lock to, and the sparse feature layer alone
  can freeze on self-similar clouds: every patch finds a lookalike near its
  stale prediction and the solve confirms near-zero motion with a
  confident-looking inlier count, while the whole frame visibly slides.
  Now the same whole-frame registration runs against the PREVIOUS frame
  whenever the reference can't lock — a motion floor that seeds the feature
  predictions where the content actually went, so the sparse layer matches
  truth again. Frame-to-frame lock counts are stated in the stabilize
  summary (it measures real motion but can drift slowly, unlike a
  reference lock). Mathcheck-asserted (a fast cloud tilt off the reference
  loses >8° without it, tracks to 0.28° with it) and verified offline on
  the actual field clip: the solved elevation now climbs 30°→61° through
  the formerly frozen span with zero held frames. Re-run 🎞 Stabilize on
  affected sightings to pick it up.
- **Strong g-figure caveat when the camera solve was worked hard** (field
  ask: a maneuver-load figure computed through heavy stabilization is not a
  measurement). `trackQuality` now also measures the CAMERA's workload over
  the tracked span — total sweep, zoom ratio, and the fraction of frames on
  the frame-to-frame lock — and flags `camHeavy` when any is substantial,
  capping the quality grade at fair (poor when the chain carried most of the
  clip). The single-observer trajectory section renders a hard warning
  directly under the maneuver-load readout: acceleration
  double-differentiates a track measured through the camera solve, so
  residual stabilization wobble shows up as g the object never pulled —
  treat the figure as an upper bound and distrust any maneuver the rate
  plot doesn't show as a sustained ramp. The same reason line flows into
  the results-panel quality banner and the report's video-analysis lead.
  Mathcheck-asserted (a Germany-like sweep+zoom+chain clip rates poor with
  the upper-bound caveat; a steady tripod solve stays unflagged).

### Fixed (this batch)
- **The in-app manual was painted over by the sky view's bottom controls**
  (field report). The ? button lives inside the sky view's HUD, which is a
  positioned, z-indexed element — a CSS stacking context — so the overlay's own
  z-index was scoped inside it and the bottom bar (same z-index, later in the
  DOM) drew straight over the manual. The overlay now renders through a portal
  to `<body>`, which takes it out of every ancestor context and holds wherever
  a help button is placed. Pointer events are stopped at its backdrop too: a
  portal escapes the DOM tree but NOT React's synthetic event tree, so without
  that a tap on the open manual would have rotated the dome behind it.
  (Affected edit mode as much as review mode.)
- **🎯 Solve from marks was not a toggle**: every tap re-opened its smoothing
  panel, so tapping the button again could never close it — the only ways out
  were ✓ Done or switching to another tool (field report). It now behaves like
  every other mode button: tap to open and solve, tap again to close,
  highlighted while open.
- **🎯 Solve from marks is hidden in view-only mode** — it writes a solved
  camera path, so it had no business being reachable while reviewing. It only
  appears when a clip carries hand-marked camera refs, which is why the first
  dead-control sweep missed it.

### Changed
- **The wizard's last two steps step BACK one page, and gained a 🏠 button**
  (field ask). Their ‹ used to jump all the way to the sighting list, so
  correcting a sky placement after reading the results meant walking the whole
  wizard again: the results step's ‹ now returns to the sky view that produced
  those numbers, and the report step's ‹ returns to the results step (or to
  the list when the report was opened straight from there — no forward jump
  into a step the user never visited). 🏠 on both is the unconditional way out.
- **💾 Share file (.phodar.json) moved to the report step**, beside the report
  and bundle downloads where the other two exports live. It also goes through
  that page's own delivery chain now (download → clipboard → a select-all copy
  box) instead of an alert that could leave the user with nothing when the
  download was blocked. The results step's remaining button reads
  "📄 Generate report & share". Nav + button placement verified end-to-end in
  a real browser.

### Added (quality of life)
- **Cube depth slider — the monolith axis** (field ask): the ⬛ cube gains a
  third independent proportion, `depth` (0.1–3×), thinning the footprint's
  second axis against its fixed width — so a true rectangular slab is now
  reachable, not just square boxes and columns. A live mono readout beside
  the slider shows the solid's proportions (thinnest side = 1) so a specific
  ratio can be dialled exactly; the classic 2001 monolith (1 : 4 : 9) is
  depth 0.25 + stretch 2.25. Composes cleanly with squash (thin diamonds
  work too); existing fits are unchanged (default 1). Mathcheck-asserted;
  the report's dimensioned 3-view picks the new extents up automatically.
- **Trajectory playback on the top-down plot**: when the stereo trajectory
  solves, the satellite view gains ▶ + a scrubber — a marker rides the
  triangulated path in real time with a growing progress trail, and the
  readout shows elapsed/clock time, altitude and speed at the scrubbed
  instant.
- **Sighting name**: an optional name field on the home screen that becomes
  the report's title/header and the filename of every export — report .html,
  .phodar.json share file, and the .zip bundle.
- **Video-analysis section: witness estimate + real charts** (field ask):
  the assumed-distance ladder gains the witness's own estimate as an
  emphasized amber row slotted in order ("120 m (witness estimate)" with its
  implied size/speed/path in bold), called out in the caption. The
  angular-rate plot grew from a bare sparkline to a real chart — labeled
  time and °/s grid, the peak flagged at its moment, the average as a
  dashed line — and the stereo speed + felt-load strip got the same
  treatment (labeled time axis, speed ticks left, g ticks right, peaks
  marked). All verified by rendering the real report in a browser against
  the Germany session.
- **Size ⇄ distance chart overhaul** (field ask): bigger (760×400), with a
  full labeled log grid on both axes (decades plus 2×/5× minors), so values
  can be read off it rather than eyeballed. And the assumed distance set in
  the sky view's 📏 size tool (slider or 📍 map pick) now persists on the
  observer as their own range estimate — the report chart draws it as an
  emphasized amber vertical with dots where it crosses the size and altitude
  lines and the implied numbers spelled out ("witness estimate — 394 ft →
  5.1 ft across · 225 ft above you"), plus a caption sentence. Verified by
  rendering the real report in a browser against the Germany field session.
- **🎥 Track-quality rating** (field ask): a camera-motion risk rating
  (excellent/good/fair/poor) rendered in the reporting surfaces — above the
  results screen's trajectory section and at the top of the report's video
  analysis (kept out of the sky view itself, which it cluttered) — naming
  its reasons: what share of the clip sits in hard zooms /
  anchor-starved stretches where camera motion can read as object motion
  (already excluded from reported peaks), and whether the tracker-noise
  floor rivals the measured motion. Silence means a clean track. The
  Germany field clip rates "fair — 39% masked". Pure + mathcheck-asserted
  (`trackQuality` in the math core).

### Fixed
- **Trajectory view: playhead highlight + dots toggle** (field ask): while
  scrubbing stabilized playback with ⊕ Trajectory open, the point nearest
  the playhead now lights up amber with a ring, so you can see where along
  the path the clip is. And a ◆ shapes / ● dots toggle in the panel swaps
  the 3D model drawn at every point for plain dots — 47 wireframes on a
  dense path was unreadable.
- **The phantom "sharp turn" during a zoom is now masked and named** (field
  report: "the sharp turn never happens in the video — it glides like a
  balloon"): the residual 4.4°/s peak sat exactly inside a 3.3× zoom-in
  where the camera solve ran on 6-8 background anchors and held the
  camera's pointing frozen while only FOV updated — but an operator zooming
  in always tilts to re-center the subject, and that unmodeled tilt was
  booked as the object diving. That is a sustained BIAS no smoothing can
  remove, so it gets a reliability mask instead: spans where the solve had
  <9 anchors or the FOV slewed >5°/s are excluded from the reported peak
  rate and peak speed, shaded on the rate chart, and the report says what
  was excluded and why (the excursion stays visible as peakOmegaAll —
  masked, not hidden). Also: the range profile between sparse size
  keyframes now interpolates at constant radial velocity (linear range in
  time) — interpolating the ANGLE linearly implied an accelerating
  recession (dR/dt ∝ 1/ang²) and manufactured a ~170 mph end-of-gap spike.
  On the field clip the reported peak fell 4.4→2.9°/s and the implied peak
  at 120 m 910→139 mph across the whole fix series, with the remainder
  traceable to the measured tail recession. Mathcheck-asserted: the
  phantom zoom-span ramp is excluded, a real maneuver on a clean solve
  keeps its true peak, and the excursion is still returned honestly.
- **Angular rate and implied speeds no longer inflated by tracker noise**
  (field report: a balloon-smooth object showed a 9°/s spike and a
  ~900 mph implied peak). Two causes, both fixed in the math core:
  (1) the rate was a raw adjacent-frame difference, which multiplies every
  per-frame pose/tracker error by 1/dt — and both trackers are noisiest
  exactly while the camera pans or zooms. ω now comes from a weighted
  windowed fit: samples are down-weighted by camera angular speed, zoom
  rate (from the solved posePath) and low match confidence, the window
  widens where the data is noisy, and the sweep is the integral of the
  fitted rate so jitter can't random-walk it upward. A real sustained
  maneuver is a ramp across many samples and passes through; a one-frame
  excursion dies. (2) Two size keyframes a fraction of a second apart (a
  sizing repeat — the field file had 0.957° and 0.714° just 0.11 s apart)
  differentiated into ~300 m/s of phantom radial speed; near-coincident
  keyframes are now merged and the range profile smoothed before
  differencing. The report also states the estimated tracker-noise floor
  ("rate variations below ≈0.4°/s are within tracker noise"). On the real
  clip: peak 7.3→4.5°/s, implied peak at 120 m 910→169 mph. All
  mathcheck-asserted: zoom-window noise rejected, a sustained maneuver
  preserved to its true value, the sizing repeat neutralized, clean
  constant-rate clips bit-exact as before.
- **Range ratio no longer inflated by stale size stamps** (found auditing the
  same field file): sizing a track point stamps its angular size through the
  BASE FOV, and the stamp was only re-derived on placement commit — so a
  point re-sized on a zoomed frame after the last commit kept a value wrong
  by the zoom ratio, and the report's "how much closer did it get" math read
  it (the Germany clip reported a 15.25× range change; the true figure is
  5.3×). The math core now re-derives every sized point's angular size from
  its wpx through the solved per-frame FOV at the mouth of each consumer
  (sourceTrack / videoKinematics / mixedStereo), and the size panel stamps
  through the frame's own FOV in the first place. Un-stabilized and tripod
  sessions are unchanged. Mathcheck-asserted with the field file's numbers.
- **World-view wireframe scaled wrong while scrubbing a zoomed clip** (field
  report, Germany sighting): per-point size keyframes store the object's
  width in PIXELS on their own frame — and on a zooming clip (this one swept
  46°→5° FOV, 9.25×) the same angular size is ~9× more pixels zoomed in.
  Playback and the world-locked export interpolated those raw pixels and
  projected them through the marked frame's lens, so the 3D model ballooned
  up to ~8× too big around zoomed-in keyframes. Sizes are now normalized to
  one pixel scale through each keyframe's own solved FOV (angular size is
  the invariant) before interpolating — in the dome, the export burn-in and
  the measure-step ghosts alike. Tripod/fixed-lens clips are byte-identical.
  Mathcheck-asserted, including a regression straight from the field clip's
  numbers.
- **Videos vanishing from the installed app while points survive** (field
  report: "the PWA goes stale — no video loading from memory even though
  track points and other data are there"): the sighting metadata lives in
  localStorage (tiny, survives), but the clips live in IndexedDB — which,
  as best-effort storage, is exactly what iOS reclaims under disk pressure,
  hundreds-of-MB video blobs first. Three defenses now: (1) the app
  requests **persistent storage** on boot (`navigator.storage.persist()` —
  granted automatically for a home-screen PWA), so the media store stops
  being first in line for eviction; (2) a **refused write is reported at
  upload time** (storage full / private mode) instead of surfacing days
  later as silent data loss; (3) when a clip IS found missing on boot, the
  measure step **says exactly what happened** — and that re-attaching the
  same file keeps every point, mark and placement (the keep-your-work
  re-attach already existed; now you're told about it). The IndexedDB open
  is also watchdogged and retried, since iOS has been seen hanging it
  after a cold relaunch. Boot flow verified in a real browser (missing
  record → notice + flag persists across reloads; record present → clip
  re-attaches, flag clears).
- **API/MCP: lean sessions no longer crash the solver** (found by a smoke
  test of the API stack): the app always creates `A`/`B` objects on every
  source, so `analyze()` read `s.A.az` / `s.B.az` unguarded — an external
  API or MCP caller sending a minimal session without those objects got
  "analysis failed: Cannot read properties of undefined" instead of a
  verdict. Both reads are now optional-chained; a source with no B still
  solves the fix, a source with no A filters out with a named gap.
  Mathcheck-asserted with lean sessions.
- **The loupe now frames the whole shape with breathing room** (field
  report): the magnifier's zoom was derived from the shape's nominal size,
  but a stretched monolith, a balloon's string or a tilted attitude projects
  well past that (and can sit off-centre of the anchor point), so at certain
  proportions the wireframe fell off the glass. The zoom is now computed
  from the real projected extent about the loupe centre, fit to 80% of the
  glass radius — the outline always lands inside with a visible margin, so
  you can tell where it sits as it scales.
- **Adjust-mode size/tilt controls moved up beside the image** (field ask):
  the per-point size slider and attitude buttons sat at the bottom of the
  Track panel, below the mode toggle and colour rows — so on a phone,
  adjusting meant scrolling away from the very image you're matching the
  outline against. The panel now renders directly under the image and frame
  row (its own card), so the outline, the frame steppers and the size/tilt
  controls share one screen. The Track panel keeps the place/adjust toggle,
  colour and point management.
- **Track-point size ghost now scales about the point's centre** (field
  report): the wireframe drawn while sizing a track point was pinned by the
  midpoint of its widest silhouette chord, which is NOT the visual centre
  for asymmetric shapes or tilted attitudes (a saucer's dome, a balloon's
  string all hang off that chord) — so the outline slid off the point as it
  grew. It is now pinned by its APPARENT centre (the bounding box of the
  drawn curves), on the photo and in the loupe alike, so scaling grows the
  outline symmetrically about the placed point. Mathcheck-asserted.
- **Sizing/tilting a track point snaps the video to that point's frame**
  (field ask): the size + tilt controls target the nearest placed point in
  time, but the scrubber could be parked on any frame — so the outline was
  being matched against the wrong frame's pixels. First touch of either
  control now jumps the video to the targeted point's own frame (with a
  brief note saying so), the same guard philosophy the 3D object's frame
  already has.
- **All videos rendering black on the installed app (iPad field report)**:
  iOS keeps a live decoder pipeline for every <video> that ever loaded a
  source and caps those per page — a long multi-observer session saturates
  the cap, after which every clip (even a fresh upload) renders black until
  the web process restarts (Chrome has no such cap, hence "works on
  Chrome"). The offscreen videos already released their pipelines; the
  measure step's rendered player now does too, explicitly, when the clip
  changes or the step closes. If the app is currently in the all-black
  state, force-quit it (or restart the device) once — the fix prevents
  recurrence.
- **A freshly uploaded clip no longer auto-plays** (field ask): the
  first-frame paint kick now pauses at the first PRESENTED frame
  (requestVideoFrameCallback) instead of play-then-pause, which visibly ran
  the clip on some devices. A ▶ / ⏸ button joins the frame controls so the
  clip can be previewed without dragging the scrubber; a user-started play
  is never touched by the paint kick.
- **Adjusting the placed 3D object can no longer corrupt its frame** (field
  ask): the object belongs to the frame it was fitted on, but every shape
  touch re-stamped that frame to wherever the video was scrubbed — one stray
  tap while browsing frames silently moved the measurement. Now adjusting an
  existing fit while off its frame snaps the video back to it (on tool
  re-entry and as a gesture guard); deliberately moving the object to the
  current frame is the explicit 📌 "Object → this frame" button, which keeps
  the fit's size/rotation and warns to re-run Stabilize when the auto-track
  was seeded on the old frame. First placement is unchanged — any frame.
  Verified by driving the real app in a browser.
- **The measure-step loupe now follows brightness/contrast live** (field
  report: "the loupe shows the version before adjustment"). The loupe is a
  canvas, so nothing repainted it when the B/C sliders moved while it was on
  screen. It now re-pops on every B/C change while a small object is fitted
  — exactly the moment the magnified view matters, since the sliders exist
  to make a dim object visible. Reproduced and verified fixed by driving the
  real app in a browser (a gray frame under 200% brightness must read
  doubled in the loupe's own pixels).
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
