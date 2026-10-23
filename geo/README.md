# geo/ — frozen geodata for the Gazebo world

Fetched once and committed so every participant session uses identical data.
Regenerate only deliberately (it changes the study environment).

| Path | Source | Tool |
|---|---|---|
| `naip/*.jpg` | USDA NAIP orthoimagery via the USGS National Map ImageServer — https://imagery.nationalmap.gov/arcgis/rest/services/USGSNAIPImagery/ImageServer | `tools/fetch_geo.py` |
| `osm.json` | OpenStreetMap buildings and trees via the Overpass API — https://overpass-api.de | `tools/fetch_geo.py` |
| `geo_meta.json` | Area, tile bounds, pixel sizes, fetch time | `tools/fetch_geo.py` |
| `lidar_meta.json` | 3DEP lidar project and tiles chosen (point clouds in `lidar/` are not committed) | `tools/fetch_lidar.py` |

Area: home (scenario.json) ± 1500 m. Imagery tiles are 1 km × 1 km, EPSG:4326, about 0.49 m per pixel.

## Attribution

- Building and tree data © OpenStreetMap contributors, available under the Open Database License (ODbL): https://www.openstreetmap.org/copyright
- Imagery: USDA National Agriculture Imagery Program (NAIP), served by the USGS National Map: https://www.usgs.gov/centers/eros/science/usgs-eros-archive-aerial-photography-national-agriculture-imagery-program-naip
- Lidar: USGS 3D Elevation Program (3DEP), project MI_WayneCounty_2017_A17: https://www.usgs.gov/3d-elevation-program
