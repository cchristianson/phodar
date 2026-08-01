# Phodar analysis API

Headless access to the full analysis pipeline: POST a session's measurements
(the app's own share format) plus optionally a drone flight-log, get back one
machine-readable verdict with every solver's result — the Moment-A fix, the
visibility- and clock-aware track stereo, dense two-video stereo, and the
flight-log ground-truth calibration with per-witness clock checks.

The engine consumes **measurements**, not raw media: the `.phodar.json` a
session exports already carries positions, sight-lines, tracks and solved
pose/object paths. Raw-media ingestion (auto shape fitting and tracking from
a bare video) is a planned later phase — see the roadmap at the bottom.

## Enabling

The endpoint is **disabled until keys exist**. On the server (Railway →
service → Variables):

```
PHODAR_API_KEYS=key-for-you,key-for-a-friend
```

Any comma-separated opaque strings; treat them like passwords. Requests
without a valid `X-API-Key` get 401; with no keys configured the endpoint
returns 503.

## Endpoint

```
POST /api/analyze
X-API-Key: <key>
Content-Type: application/json
```

Body (≤32 MB):

```jsonc
{
  "session": { /* a .phodar.json share file, verbatim */ },
  // or "sources": [ ... ] directly
  "flightLogText": "time(millisecond),datetime(utc),...",  // optional: Airdata CSV / decoded DJI CSV / DJI .SRT text
  "flightLogName": "flight.csv",   // optional, helps format detection
  "spanM": 0.202,                  // optional: true span of the craft (m)
  "droneId": "mini1",              // optional preset: mini1 | neo
  "homeElevM": 485                 // optional: takeoff elevation for rel-alt logs
}
```

Response: the verdict object (`engine: "phodar-analyze/1"`) —
per-source completeness (with named gaps), `fix` (rating, position, ranges,
size, motion), `trackStereo` (instants, ray miss, shared vs ignored
visibility, geometric clock sync), `videoStereo` (dense two-clip solve when
objPaths exist), `flightLog` (calibration grade, fix-vs-log error, size
ratio, per-witness ⏱ clock offsets with sharpness), and `warnings` — every
honesty caveat the app would show, as strings.

### Example

```bash
curl -s https://<your-app>.up.railway.app/api/analyze \
  -H "X-API-Key: $KEY" -H "Content-Type: application/json" \
  -d @- << EOF | jq .flightLog.calibration
{ "session": $(cat sighting.phodar.json),
  "flightLogText": $(jq -Rs . < flight.csv),
  "spanM": 0.202 }
EOF
```

## CLI (same engine, no server)

```bash
node scripts/analyze.mjs sighting.phodar.json \
  --log flight.csv --span 0.202 --out verdict.json
```

Prints a human summary; `--out` writes the full verdict JSON.

## Notes & limits

- **Privacy**: sessions carry observers' GPS positions and photo thumbnails.
  The server does not store request bodies, but treat keys and payloads
  accordingly.
- **Rate limiting**: shares the server's standard per-IP query limiter.
- **Determinism**: same input → same verdict; the engine is pure and covered
  by `npm test` (a synthetic session with exact ground truth must grade
  "excellent" end to end, including clock-error recovery).

## Roadmap

1. **Media ingestion** (phase 2): accept raw photos/videos; EXIF/QuickTime
   metadata, auto object detection (contrast-blob seeding), stabilization and
   object tracking server-side (needs ffmpeg in the deploy image), automatic
   calibration where the frame allows it (star plate-solve at night, terrain
   skyline snap when ridges are visible) — with the same honesty when it
   doesn't ("relative geometry only").
2. **Agentic layer** (phase 3): a Claude-driven analyst that inspects rendered
   frames and residuals, retries with different align frames/trims/seeds, and
   writes the human evaluation — wrapping this engine as its toolset.
