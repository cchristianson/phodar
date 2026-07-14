# Contributing to Phodar

Thanks for helping make sighting reports mean something.

## Ground rules
1. **`npm test` must pass** before and after your change. The scripts verify
   the triangulation, projection, and kinematics math against exact synthetic
   truth — if you touched the math core and they fail, the math is wrong, not
   the tests.
2. Read the **Non-negotiable invariants** in `CLAUDE.md` first. Most of them
   encode iOS Safari bugs that took hours to hunt; "cleaner" versions
   reintroduce them.
3. Test on a real phone (`npm run dev`, open the LAN URL). The marking canvas,
   loupe, and sky view are touch instruments — the desktop pointer hides
   whole bug classes.
4. Honest epistemics is the product. Prefer a warning over a silent guess;
   prefer "quality: poor" over an impressive-looking wrong answer.

## Good first issues
- Module split along the banner-comment seams (mechanical, well-guarded by
  tests).
- Leaflet swap for the PinMap canvas.
- Aircraft type → wingspan lookup table (feeds the ADS-B checker).
- Report print CSS polish.

## Field data welcome
Ground-truth datasets (known objects, multiple angles, EXIF intact) are as
valuable as code — open an issue with a `.phodar.json` and the true
measurements.
