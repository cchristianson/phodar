<!-- Thanks for contributing to Phodar! Keep changes surgical and honest. -->

## What & why
<!-- What does this change and what problem does it solve? Link any issue: Fixes #123 -->

## How it was verified
<!-- Phodar is a touch instrument and a measurement tool. Say how you checked it. -->
- [ ] `npm test` passes (triangulation / projection / kinematics regression)
- [ ] `npm run build` succeeds
- [ ] Tested on a real phone (iOS Safari especially) if it touches UI / touch / canvas
- [ ] For math changes: added or updated an assertion in `scripts/mathcheck.js` / `scripts/trajcheck.js`

## Checklist
- [ ] I read the **Non-negotiable invariants** in `CLAUDE.md` and didn't reintroduce a documented iOS/Safari bug
- [ ] No new heavy dependency without discussion (the hand-rolled EXIF parser, zip writer, and 3D projector are features, not oversights)
- [ ] Honest epistemics preserved — a warning over a silent guess; "quality: poor" when it's poor
- [ ] The diff is minimal and focused (no drive-by refactors of unrelated code)

## Notes for reviewers
<!-- Anything non-obvious: coordinate-system assumptions, a field report this addresses, screenshots for UI. -->
