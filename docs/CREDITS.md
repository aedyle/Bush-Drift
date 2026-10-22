# Credits

## The LandCruiser 79

The car is cut down from **"Toyota Land Cruiser J79 Pickup Off- Road"** by **DR1KING100K**,
https://sketchfab.com/3d-models/toyota-land-cruiser-j79-pickup-off-road-353616b065af4059b6a3cb84f964a270
(author: https://sketchfab.com/DR1KING100K), licensed under **CC BY 4.0**
(http://creativecommons.org/licenses/by/4.0/).

Changes: the cabin interior, underside and small parts removed; rescaled to the real 3,180 mm wheelbase;
the body rebuilt as 7 cm surface voxels in eight flat colours; the wheels replaced with simple 12-sided
tyres and two-tone rims at the model's wheel positions and size. Made by `tools/make_79.py` (Blender, headless).

**The ute in the game is a modified version**: a low-detail voxel model made from DR1KING100K's model, not the
original. The downloaded model (`src-models/`) is never published. The voxel game file made from it
(`assets-built/lc79.car`) is built locally and ships inside the browser and Android builds, under the same
CC BY 4.0 licence, with this credit.

## Map data

**Road line, hut outline, ford and bridge: © OpenStreetMap contributors**, https://www.openstreetmap.org/copyright.
OpenStreetMap data is licensed under the **Open Database License (ODbL) 1.0**. The game's road (`route.csv`
and `meta.txt` in the builds) is derived from it: the OSM line, moved onto the visible track and smoothed. That
road data is a derivative database and is itself **available under the ODbL 1.0**
(https://opendatacommons.org/licenses/odbl/1-0/). It can be rebuilt from OSM with `scripts/` (see
`scripts/README.md`).

**Elevation: NSW 5 m Digital Elevation Model, © State of New South Wales (Spatial Services, a business unit of
the Department of Customer Service NSW)**, licensed under **CC BY 4.0** (https://creativecommons.org/licenses/by/4.0/).
Changed: sampled onto a 5 m grid round the route and a 20 m grid out to 3 km, and smoothed under the road.

**Imagery: NSW Imagery, © State of New South Wales (Spatial Services)**, CC BY 4.0; the service also credits
captures by AAM, Jacobs/Ausimage and Landsat. The imagery is used to find the track and to estimate how wooded
each 5 m or 20 m cell is. **No imagery pixels are in this repository or in the game**: only the derived road
line and the per-cell tree-cover numbers (`canopy.bin`, `far.bin`). Diagnostics drawn over the imagery go to
the gitignored `shots/` folder.

## Plants (session 3)

Every plant is made in code by `tools/make_plants.py` (Blender, headless); no plant models or textures are
downloaded. Which species grow where comes from public data, used as reference only
(`docs/forest/species.md`): the NSW State Vegetation Type Map and BioNet Vegetation Classification
(© State of NSW, Department of Climate Change, Energy, the Environment and Water), PlantNET / Flora of NSW
(Royal Botanic Gardens Sydney), and Atlas of Living Australia occurrence records.

## Deer and the edge's scrub (build B)

Made in code by `tools/make_world.py` (Blender, headless), like the plants: low-poly sambar deer (four poses) and the
dense scrub on the map's edge. No models or textures downloaded. Why sambar: `docs/forest/deer.md`.

## Alexanders Hut (session 3)

The hut and the campground props are made in code by `tools/make_hut.py`. Its plan comes from the OSM
building outline (© OpenStreetMap contributors, ODbL); its look from photos used as reference only and never
copied into the game: the author's own, and NPWS (John Spencer and Lawani Colley, © DPIE/OEH), Julian Robinson
(Flickr, all rights reserved) and stock-photo previews. They are kept in `refs/hut/` (gitignored, private) and
listed in `docs/hut/reference.md`.

## Sound (session 3b)

All recorded sound is from Freesound (https://freesound.org), released under **CC0 1.0** (public domain
dedication); each clip's licence was checked on its Freesound page on 20 Oct 2026. Credit isn't required, but
here it is. The loops are cut, levelled and made seamless by `tools/make_sfx.py` into `assets/sfx/`.

| file | recording | by |
|---|---|---|
| `engine_idle.ogg` | Truck Engine (Diesel), a Ford F250 V8 diesel idling, https://freesound.org/s/611760/ | Spencer_Eisenhauer |
| `engine_load.ogg` | Car driving in exterior, Toyota Landcruiser 4x4, dirt road, https://freesound.org/s/668552/ | felix.blume |
| `gravel_roll.ogg` | Slowly driving on gravel road, mic close to wheels, https://freesound.org/s/577504/ | TRP |
| `gravel_slide.ogg` | Truck Pulling Away On Gravel Then Doing Donuts, https://freesound.org/s/725428/ | mwchristian95 |
| `bush.ogg` | Birdsong In Northern NSW Hinterland (Mebbin), https://freesound.org/s/812609/ | Alex_hears_things |
| `kookaburra.ogg` | Kookaburras in the Victorian Bush, https://freesound.org/s/427327/ | Sassaby |
| `rocket_whistle.ogg` | rocket-whistle.wav, https://freesound.org/s/186933/ (CC0, checked 22 Oct 2026; trimmed to 1.32 s, mono) | ReadeOnly |

The score and gear-change notes are generated in code.

## Music

| file | song | by |
|---|---|---|
| `assets/music/the-late-cake.ogg` | The Late Cake (a song in Explore) | willfarrell85, made with Suno |

Used with the permission of its owner, who holds commercial rights to it (a Suno premium subscription); it is not
covered by this repository's licence and may not be reused without his permission.
