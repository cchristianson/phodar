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
curl -s https://phodar.app/api/analyze \
  -H "X-API-Key: $KEY" -H "Content-Type: application/json" \
  -d @- << EOF | jq .flightLog.calibration
{ "session": $(cat sighting.phodar.json),
  "flightLogText": $(jq -Rs . < flight.csv),
  "spanM": 0.202 }
EOF
```

## Phase 2 — raw-media ingestion (auto-measurement)

Hand the server a sighting **video or photo by URL** and it runs the real
measurement pipeline — EXIF/QuickTime metadata, object snap + sizing, the full
video stabilizer (per-frame camera pose) and guided object auto-track — and
returns a draft `.phodar.json` plus an importable bundle (.zip with the
original media) for **human review in the app**. The caller supplies the
judgment the app's user normally supplies: where the object is (a mark on a
keyframe), and whatever context exists (position, time, camera bearing and
up-angle, witness text). Every defaulted value is listed in
`source.ingest.guessed`; the sky placement is carried as approximate until
refined in the app.

Requires ffmpeg on the server (nixpacks.toml installs it; fallback: set the
service variable `NIXPACKS_APT_PKGS=ffmpeg`). `GET /api/health` reports
`ingest: true/false` so you can verify a deploy. Without ffmpeg these
endpoints answer with a named error. Media is held ~2 hours; jobs are
in-memory on the single dyno (a redeploy forgets them — re-run).

```
POST /api/ingest        {url, filename?, trim?:{t0,t1}} JSON — or raw bytes
                        → { mediaId, probe, meta, keyframeTimes }
                        trim fetches ONLY that span off the URL (range
                        requests, no re-encode; span ≤150 s) — the route for
                        4K phone clips over the 300 MB download cap
POST /api/measure       {mediaId, context, object:{t,fx,fy,wfrac?}, track?}
                        → { jobId, poll }
GET  /api/job/<id>                 → job status (stage, %, then summary)
GET  /api/job/<id>/bundle.zip      → the importable bundle
GET  /api/job/<id>/session.json    → the .phodar.json alone
```

All key-gated like `/api/analyze`; the GET artifact routes also accept
`?key=` so a human can click the link. `context.trim {t0,t1}` is required
for clips over 90 s and wise regardless — trim to the sighting.

The MCP tools (below) wrap the same pipeline **with vision**: `ingest_media`
returns keyframes as images the AI looks at, `inspect_frame` returns a zoomed
crosshair crop to confirm the object mark, `auto_measure`/`job_status` run
and poll the job, and `fetch_report` scrapes a report page (og/JSON-LD/media
links) so an AI can go from a sighting-report URL to a reviewed draft in one
conversation. The AI is the eyes and context-gatherer; phodar is the
instrument; the human owns the review.

## MCP server (bring your own AI)

The same engine speaks the **Model Context Protocol**, so users drive phodar
with their own AI subscription — no AI costs on phodar's side. MCP is an
open standard: Claude, ChatGPT (custom connectors, developer mode, and the
OpenAI Agents SDK), Gemini's SDKs, LangChain and most agent frameworks all
consume remote MCP servers over the same Streamable-HTTP transport this
serves. Stateless JSON responses, no OAuth required.

```
URL:   https://phodar.app/mcp/<api-key>
```

The key rides the URL because every client can paste a URL, while header
support varies (Authorization: Bearer and X-API-Key also work). A key in a
URL can land in logs — hand out per-person keys and rotate freely.

**Connect from Claude**: Settings → Connectors → Add custom connector →
paste the URL. Then in any chat: attach your `.phodar.json` (and flight-log
CSV) and ask for an analysis.

**Connect from ChatGPT**: Settings → Connectors (developer mode) → Add →
MCP server, paste the URL, no auth. Or in code, the Agents SDK / Responses
API `mcp` tool with `server_url`.

**Tools exposed**:
- `analyze_session` — the full verdict from a session's measurements
  (+ optional flight log): fix, stereo, clock sync, calibration grades,
  warnings. Returns a text summary and the structured verdict.
- `parse_flight_log` — inspect a drone log on its own (samples, span,
  clock/altitude datums, first/mid/last states).

The `initialize` response carries orientation instructions, so a connected
AI knows the workflow (measure in the app → export share file → analyze
here) without any prompt engineering by the user.

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
