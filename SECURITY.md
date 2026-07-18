# Security Policy

## Reporting a vulnerability

**Please do not open a public issue for security problems.**

Report privately through GitHub's built-in flow:

1. Go to the repository's **Security** tab → **Report a vulnerability**
   (GitHub Private Vulnerability Reporting).
2. Describe the issue, the impact, and steps to reproduce.

We'll acknowledge the report, investigate, and coordinate a fix and disclosure
timeline with you.

## Scope

Phodar is a client-side web app with a small dependency-free Node proxy
(`server/index.mjs`) that forwards requests to public, keyless data sources
(aircraft, terrain, weather, map tiles). Things especially worth reporting:

- Ways the proxy could be turned into an open relay or made to reach
  unintended hosts (SSRF).
- Exposure of any secret or user data (the app is designed to keep photos and
  location on-device — a path that leaks them off-device is in scope).
- Cross-site scripting via imported `.phodar.json` / shared report data.

## Not in scope

- Rate limits or availability of third-party public APIs (Overpass,
  Open-Meteo, ADS-B archives, tile servers).
- The inherent accuracy limits of consumer phone sensors (compass/GPS) — these
  are documented and surfaced as warnings by design.
