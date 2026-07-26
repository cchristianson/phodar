# Privacy

Short version: **your photos and videos never leave your device.** All the
measurement happens in your browser. There is no account, no login, no
telemetry, no analytics, no cookies, no ad network, and no server that stores
anything about you.

This page is about the parts that are *not* obvious.

## What stays on your device

- **Photos and videos.** They are decoded, normalized and measured in the
  browser. Images and clips live in your browser's IndexedDB; the rest of the
  sighting lives in localStorage. Nothing is uploaded.
- **Your position, times, marks, shape fits, tracks and results.** Same
  storage, same rule.
- **Reports, `.phodar.json` share files and `.zip` bundles.** Generated
  locally, saved by you, sent by you — to whomever you choose. The report is a
  single self-contained HTML file with the photos embedded, so it works offline
  and needs no server.

Clearing your browser data, or the app's own "reset", deletes all of it. There
is no copy anywhere else.

## What is sent off the device

The cross-checks compare your sighting against public data, and that means
asking public services about a place and a time. What goes out, and where:

| What is sent | To | Why |
| --- | --- | --- |
| Observer latitude/longitude (rounded to the query radius) and the sighting time | ADS-B feeds and archives, Open-Meteo, Launch Library, NASA CNEOS | To ask "what was flying / blowing / launching near here, then?" |
| Approximate coordinates | OSM Overpass, Terrarium elevation tiles, Esri and OSM map tiles | Terrain, peaks, buildings, aerodromes and the map you're looking at |
| Text you type into the place search | US Census geocoder, then Nominatim | Forward geocoding |

No image data, no video, no report content and no identifier is ever included
in any of those requests. They are ordinary, unauthenticated public API calls —
the same ones any map or weather page makes. Each one is still a request from
your IP address to a third party, whose own logging and privacy policy applies;
`docs/DATA-SOURCES.md` links every one of them.

**Cross-checks only run when you use them.** If you never open the sky view,
never generate a report and never search for a place, the app makes no outbound
requests at all beyond loading itself. It works fully offline in that mode —
triangulation, sizing, kinematics and the report itself need no network.

## If you self-host

`server/index.mjs` proxies some of those requests so the browser can reach
sources that send no CORS headers. It **forwards**; it does not store. It keeps
an in-memory cache of upstream *responses* (aircraft archive slices, Overpass
answers) that is dropped on restart, and it writes no logs beyond whatever your
host captures by default.

If you run a public instance, note that your server's access log — and your
host's — will contain the coordinates your users query. That is a real record
of where people are reporting sightings from. Decide deliberately what you
retain, and say so on your instance. See `docs/SERVER.md`.

## Sharing is your call

The report and the share file are designed to be handed to other witnesses,
investigators or the internet. They contain **exactly** what you put in: your
photos, your position, your times, your name if you entered one. Nothing is
stripped or obscured automatically. Look at what you're sending before you send
it — a sighting report from your back yard contains your back yard's
coordinates.
