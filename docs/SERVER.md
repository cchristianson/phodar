# The Phodar server

`server/index.mjs` is a small, **dependency-free** Node HTTP server (Node 18+,
uses only `node:http`, `node:fs`, `node:zlib` and global `fetch`). It does two
jobs:

1. serves the built `dist/` bundle, and
2. proxies the handful of upstream data sources that a browser **cannot** reach
   directly.

Phodar is a client-side app: all the photogrammetry runs in your browser and
your photos never leave the device. The server exists only because some of the
cross-check sources send no CORS headers, and one of them (the ADS-B history
archive) needs a 10–25 MB binary decoded before it's useful on a phone.

## Running it

```bash
# development — two processes
npm run dev                 # Vite on :5173, proxies /api → :8787
node server/index.mjs       # the API, on :8787

# production / preview — one process serves both
npm run build
npm start                   # dist/ + /api on $PORT (default 8787)
```

`vite.config.js` already proxies `/api` to `localhost:8787`, so the only thing
you have to remember in development is to start the second process.

**If you skip it, the app still works** — it degrades, quietly and by design.
Nothing hard-fails, but most of the mundane-explanation cross-checks go dark.
See the table below.

## Endpoints

| Endpoint | Query | Upstream | Without it |
| --- | --- | --- | --- |
| `/api/live` | `lat`, `lon`, `nm` | airplanes.live + adsb.lol + adsb.fi + OpenSky, merged by ICAO hex | Falls back to browser-direct airplanes.live (the only feed with `ACAO: *`) — fewer aircraft, no MLAT/Mode-S targets |
| `/api/hist` | `lat`, `lon`, `nm`, `t` (ms), `win` (min) | tar1090 `globe_history` archives — globe.airplanes.live, globe.adsbexchange.com fallback | No historical aircraft check; sightings older than 15 min fall back to *live* traffic with an amber warning |
| `/api/tile/{img\|trans\|ref}/{z}/{y}/{x}` | — | Esri ArcGIS World Imagery + reference overlays | The report's top-down plot draws on a plain background instead of satellite imagery |
| `/api/peaks` | `lat`, `lon`, `r` | OSM Overpass | No named summits on the terrain skyline |
| `/api/buildings` | `lat`, `lon`, `r` | OSM Overpass | No 3D building boxes in the sky view |
| `/api/airports` | `lat`, `lon`, `r` | OSM Overpass | No nearby-airport context in the report |
| `/api/winds` | Open-Meteo query string | Open-Meteo forecast + ERA5 archive | Wind check absent; balloon verdict unavailable |
| `/api/launches` | `net0`, `net1` (ISO) | Launch Library 2 | No rocket-launch correlation |
| `/api/fireballs` | `date-min`, `date-max` | NASA JPL CNEOS fireball API | No bolide correlation |
| `/api/health` | — | — | — |

Sources the browser reaches **directly**, with no proxy involved (they are
CORS-open): CelesTrak TLEs, Terrarium elevation tiles, Open-Meteo (also
proxied, for consistency), adsbdb, Nominatim, the US Census geocoder, Esri and
OSM map tiles in the Leaflet views. Those keep working with the server down.

## Caching and footprint

- ADS-B history slices are cached in an in-memory LRU capped at **80 MB**;
  aircraft traces at **30 MB**. `/api/health` reports the current slice-cache
  size.
- Overpass answers are cached in-process: peaks 6 h, buildings and airports
  24 h.
- All of it is **per-process and non-persistent**. Restarting drops the cache;
  running two instances doubles the upstream traffic. There is no shared store
  and no database.

## Deploying

`railway.toml` builds with Vite and starts `npm start`, i.e. **the server**,
not a static host — the API is part of the deployment. Any host that can run
`node server/index.mjs` with `PORT` set works the same way; there are no other
environment variables and no API keys anywhere in the project.

`npm run start:static` exists for the static-only case (Vite preview, no API).

## Before you host this publicly

The proxy is deliberately narrow — every handler builds its own upstream URL
from validated numeric parameters, and there is no user-supplied host or path
anywhere, so it can't be turned into an open relay. But it is **unauthenticated
and unthrottled**, and it forwards to volunteer-funded infrastructure
(Overpass, the tar1090 archives). If you run a public instance:

- put a rate limit in front of it, per IP;
- keep the `user-agent` identifying your deployment, not `phodar/1`, so the
  upstreams can reach you if your instance misbehaves;
- read `docs/DATA-SOURCES.md` for each source's terms and limits.
