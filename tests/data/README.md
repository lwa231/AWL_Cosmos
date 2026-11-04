# tests/data

`horizons_j2000_plus_1yr.h` holds the reference vectors for the "Real solar system (M8)" test
(`tests/test_solar_system.cpp`). They are stored, never fetched at test time.

## Source

NASA/JPL Horizons on-line ephemeris system, API `https://ssd.jpl.nasa.gov/api/horizons.api`, retrieved 2026-11-04
(Git Bash `curl`, one request per body, `format=text`):

    COMMAND='<id>'   (body ids 10, 199, 299, 399, 301, 499, 599, 699, 799, 899, 999; not the system barycentres)
    OBJ_DATA='YES'  MAKE_EPHEM='YES'  EPHEM_TYPE='VECTORS'
    CENTER='500@0'  (solar-system barycentre)
    REF_PLANE='ECLIPTIC'  REF_SYSTEM='ICRF'  (ecliptic and equinox of J2000, X axis along ICRF)
    OUT_UNITS='KM-S'  VEC_TABLE='2'  TIME_TYPE='TDB'  CSV_FORMAT='NO'
    TLIST='2451545.0 2451910.25'

* T0 = JD 2451545.0 TDB = 2000-Jan-01 12:00:00 TDB (J2000). These vectors are the initial state in
  `scenes/solar_system.json` (km and km/s converted to m and m/s).
* T0 + 1 yr = JD 2451910.25 TDB (365.25 days later) = 2000-Dec-31 18:00:00 TDB. These are in the header here.
* Ephemerides named by Horizons: DE441 (Sun, Mercury, Venus, Earth, Moon), mar099 (Mars), jup365_merged, sat441l,
  ura184_merged, nep098_merged, plu060_merged.
* `mu` (GM, km^3/s^2) and the volumetric mean radius in the scene are the values printed in each body's Horizons
  physical-data header (body GMs, not system GMs). Spin periods are the sidereal rotation periods from the same headers.

The file is generated from the raw responses by a throwaway script; to refresh, repeat the requests above and replace the
numbers. Horizons data is public-domain NASA/JPL output.
