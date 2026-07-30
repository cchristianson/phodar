# Contributing to Phodar

Thanks for helping make sighting reports mean something.

## Ground rules

1. **`npm test` must pass** before and after your change. The scripts verify
   the triangulation, projection, kinematics, plate-solve, terrain, video-pose
   and sensor-fusion math against exact synthetic truth, importing the real
   `src/` modules — if you touched the math and they fail, the math is wrong,
   not the tests.
2. Read the **Non-negotiable invariants** in `CLAUDE.md` first. Most of them
   encode iOS Safari bugs that took hours to hunt; "cleaner" versions
   reintroduce them. The same file records several approaches that were tried,
   measured and **reverted** — check it before rebuilding one of them.
3. Test on a real phone (`npm run dev`, open the LAN URL). The marking canvas,
   loupe, and sky view are touch instruments — the desktop pointer hides
   whole bug classes.
4. Honest epistemics is the product. Prefer a warning over a silent guess;
   prefer "quality: poor" over an impressive-looking wrong answer.

## Running the whole thing

```bash
npm install
npm run dev                 # Vite, :5173
node server/index.mjs       # the API proxy, :8787 — vite proxies /api here
```

The second process is optional, but most cross-checks are dark without it. See
`docs/SERVER.md`.

## Tests

| Command | Needs | Runs in CI |
| --- | --- | --- |
| `npm test` | nothing — pure Node | yes, on every PR |
| `npm run helpcheck` | nothing — pure Node | no |
| `npm run skycheck <url>` | Playwright + a built preview | no |
| `npm run capcheck <url>` | Playwright + a built preview | no |
| `npm run storecheck <url>` | Playwright + a built preview | no |

`npm test` (`scripts/mathcheck.js` + `scripts/trajcheck.js`) is dependency-free
on purpose, so it can never be the reason a contribution stalls.

`npm run helpcheck` guards the in-app manual. The "?" overlay (`HELP_SECTIONS`)
is the only documentation a user ever gets, and it drifts silently — a feature
ships, the manual doesn't mention it, and nothing fails. The check sweeps every
distinctive glyph that appears on a button, plus a list of named features keyed
on the code that implements them, plus the shape picker, and fails if any of
them is undocumented. If you add a control that genuinely needs no manual entry
(a close ✕, a nudge arrow), list its glyph in `STRUCTURAL` **with a reason** —
so skipping the docs is a decision rather than an oversight.

The browser harnesses drive the real app and catch what unit tests can't.
`skycheck` walks the wizard until the sky view mounts — it exists because a
module-scope ordering bug once shipped a black screen that no unit test could
see. `capcheck` stubs camera and motion hardware to exercise the
instrumented-capture path end to end, **twice**: once as an iPhone and once as
an Android device, which report attitude in genuinely different ways.
`storecheck` boots with `localStorage` blocked, the way strict privacy settings
do. Playwright is deliberately **not** a declared dependency; install it when
you need them:

```bash
npm i -D playwright && npx playwright install chromium
npm run build && npm run preview -- --port 4173 &
npm run skycheck http://localhost:4173
```

`scripts/metacheck.mjs <file>` is a useful third one: it runs the app's own
EXIF/QuickTime parser over a photo or clip and prints what Phodar will actually
see. Most images off the web have their metadata stripped — check before
building a test around one.

## Where to start

The backlog with full context lives in `CLAUDE.md` ("Priority backlog"). Things
that are self-contained and well-guarded by tests:

- **Module split.** `src/phodar.jsx` is ~10k lines organized by banner
  comments; the math core, EXIF parser and shape system have already been
  extracted. The remaining seams are `src/components/{MediaMeasure,SkyAimer,
  PinMap}.jsx`, `src/report/{html,share,zip}.js` and `src/wizard/*.jsx`.
  Mechanical, behavior-identical, one seam per pull request.
- **Report print CSS.** The report reads well on screen and is only roughly
  paginated.
- **Two-video sync nudge.** Automatic clock sync works; there is no manual
  offset control for when EXIF times are missing *and* the object moves too
  slowly to lock.
- **`adsbdb` route enrichment.** Hex/callsign → origin and destination, to put
  a plausible aircraft match in context.
- **Another cross-check.** Anything that can predict a direction, an angular
  size and a motion for a known object drops straight into the existing table —
  see the design principle in `CLAUDE.md`.

## Field data welcome

Ground-truth datasets (known objects, multiple angles, EXIF intact) are as
valuable as code — open an issue with a `.phodar.json` and the true
measurements. Every accuracy number in the README came from one.
