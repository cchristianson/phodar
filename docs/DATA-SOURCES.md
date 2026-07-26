# Data sources

Phodar has no API keys, no accounts and no paid tiers. Every cross-check runs
against public data — which means every one of them is somebody else's
infrastructure, most of it donated. This page lists what we use, under what
terms, and what breaks if a source goes away.

Attribution requirements are met in-app (Leaflet attribution controls on every
map, and a credit line on the report's satellite plot). If you fork Phodar or
re-use its output, those credits have to travel with it.

## Aircraft

| Source | Used for | Terms / limits |
| --- | --- | --- |
| [airplanes.live](https://airplanes.live/) | Live traffic; also the browser-direct fallback when the proxy is absent (it is the only feed sending `ACAO: *`) | Free, no key. Community-funded feeders — don't poll faster than the app's 20 s. |
| [adsb.lol](https://adsb.lol/) | Live traffic (merged) | Free, no key, ODbL. **No CORS** — proxy only. |
| [adsb.fi](https://adsb.fi/) | Live traffic (merged) | Free, no key. |
| [OpenSky Network](https://opensky-network.org/) | Live traffic (merged) — adds MLAT / Mode-S targets pure ADS-B feeds miss | Anonymous access ~400 requests/day per IP. Non-commercial use; research-funded. |
| [tar1090 `globe_history`](https://globe.airplanes.live/) archives (airplanes.live, adsbexchange fallback) | Historical replay for sightings older than 15 min | Free, no key. Each query pulls a **10–25 MB** binary slice — cache it (the server does) and don't loop over it. |
| [adsbdb](https://www.adsbdb.com/) | hex/callsign → type, registration, route | Free, no key. |

Four independent networks are merged because each has different receiver
coverage; a feed that errors or rate-limits is dropped and the rest still
answer.

## Terrain and maps

| Source | Used for | Terms / limits |
| --- | --- | --- |
| [Terrarium elevation tiles](https://registry.opendata.aws/terrain-tiles/) (AWS Open Data) | DEM heightfields → skyline ray-march, observer elevation | Public, CORS-open, no key. Mapzen/Tilezen data; see the registry page for the per-dataset credits (SRTM, NED, and others). |
| [Esri World Imagery](https://www.arcgis.com/home/item.html?id=10df2279f9684e4a9f6a7f08febac2a9) + reference overlays | Satellite basemap for the pin map, distance picker and report plot | Free for use within the Esri map ecosystem; **attribution required** — "© Esri, Maxar, Earthstar Geographics" is shown on every map and on the report plot. No bulk downloading or re-hosting of tiles. |
| [OpenStreetMap tiles](https://www.openstreetmap.org/) | Street-map toggle on the pin map | ODbL. Attribution required and shown. The OSMF [tile usage policy](https://operations.osmfoundation.org/policies/tiles/) forbids heavy or automated use — a public Phodar instance should point at its own tile source. |
| [OSM Overpass](https://overpass-api.de/) (four mirrors, tried in order) | Named peaks, building footprints, nearby airports | ODbL. Volunteer-funded, strict fair use. Answers are cached 6–24 h in-process; keep it that way. |
| [Nominatim](https://nominatim.org/) | Place-name search (non-US, landmarks) | ODbL. [Usage policy](https://operations.osmfoundation.org/policies/nominatim/): max 1 request/second, identifiable client, no bulk geocoding. Phodar only queries on an explicit search tap. |
| [US Census geocoder](https://geocoding.geo.census.gov/) | US street addresses (resolves to the parcel; Nominatim only knows the road) | Public domain, no key. |

## Sky

| Source | Used for | Terms / limits |
| --- | --- | --- |
| [CelesTrak](https://celestrak.org/) | Visual-group and Starlink TLEs → satellite passes | Free, no key. TLEs are updated a few times a day; Phodar reports their age rather than pretending they're fresh. Don't re-fetch per view. |
| Yale Bright Star Catalog via [d3-celestial](https://github.com/ofrohn/d3-celestial) | The 327-star display catalog and the 1627-star deep catalog used by the plate solver | BSD-3 (Olaf Frohn). Baked into `src/math/starcat*.js` — no network at runtime. |
| [SunCalc](https://github.com/mourner/suncalc) algorithms | Sun and Moon position, Moon phase | BSD-2 (Vladimir Agafonkin). Inlined in `src/math/astro.js`. |
| Schlyter low-precision ephemeris | Planet positions (validated against JPL Horizons to ~0.01°) | Public method, implemented in `src/math/planets.js`. |
| [NOAA/NCEI WMM2025](https://www.ncei.noaa.gov/products/world-magnetic-model) | Magnetic → true bearing correction for EXIF compass data | Public domain. Coefficients embedded; **valid through 2029** — they must be replaced then. |

## Weather and events

| Source | Used for | Terms / limits |
| --- | --- | --- |
| [Open-Meteo](https://open-meteo.com/) forecast + ERA5 archive | Winds aloft at the fix altitude (balloon check) | Free for non-commercial use, no key. Attribution appreciated; CC-BY for the underlying ERA5. |
| [Launch Library 2](https://thespacedevs.com/llapi) | Rocket-launch correlation | Free tier, no key, rate-limited per IP. |
| [NASA JPL CNEOS fireball API](https://ssd-api.jpl.nasa.gov/doc/fireball.html) | Bolide correlation | Public domain. |

## Runtime dependencies

`leaflet` (BSD-2), `react` / `react-dom` (MIT), `satellite.js` (MIT). That's the
whole list — the EXIF parser, zip writer, MP4 muxer, 3D projector, geodesy and
plate solver are all hand-rolled, which is why the bundle stays small and why
`npm test` needs no dependencies at all.

## If a source dies

Nothing here is load-bearing for the core measurement. Triangulation, sizing,
kinematics and the report all run offline from the photos alone; every source
above feeds a *cross-check*, and each one fails soft with a stated caveat
rather than a wrong answer. Known fallbacks:

- Elevation: OpenTopoData (`/v1/ned10m`, 100 points/call, 1000/day) or USGS
  EPQS as point queries if Terrarium goes away.
- Aircraft: any single ADS-B feed is enough; the merge is an improvement, not a
  requirement.
- Basemaps: the report plot draws on a plain background when tiles fail.
