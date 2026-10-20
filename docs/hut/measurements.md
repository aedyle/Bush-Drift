# Alexanders Hut: measurements (session 3b, 20 Oct 2026)

Session 3b re-measured the hut and its surroundings from two aerials and five ground photos, then matched the game against each photo from the same camera. This file holds the numbers. `reference.md` still holds the materials, history and sources, but **its section 1 (orientation) and the plan in section 2 are wrong** (see "What session 3 had wrong").

Frame: metres **E, N of the hut's OSM centroid** (-36.7208104, 149.4627460), heights from the ground there. Bearings are compass degrees.

## Sources

| # | image | used for |
|---|---|---|
| 1 | Google satellite close-up (the author) | today's track, the bollard rows, the car-park loop. Georeferenced by its two lat/lon pins (about 15.15 px/m, north up), then shifted (+1.8 E, -1.0 N) so its roof sits on the OSM outline |
| 2 | NSW SIX Maps aerial (the author) | The same 2011 50 cm NSW capture as the game's z18 mosaic (`data/imagery_z18.png`). The screenshot is cropped, so its scale bar doesn't show; the georeferenced mosaic was measured instead |
| 3 | ground photo: the gable with the 12-pane window | the front, the verandah, the squat tank, the hitching rail, the sign, the plough, the gate |
| 4 | Alamy photo: the long side | the south-west wall, the chimney's end |
| 5 | NPWS car-park photo | **the start view** |
| + | own photo 06 (front and verandah), 12 (car park), 10 (the wing's gable end), 03 and 07 (from the picnic spot) | cross-checks |

## What session 3 had wrong

- Session 3 read the ridge as running NNE–SSW (30°/210°), with the 12-pane gable facing SSW and the chimney on the "west long wall". **The ridge actually runs WNW–ESE (121°/301°), along the OSM outline's long south-west side.**
- The photos settle it:
  - In photo 4 and own photo 12 the chimney's face has rakes, so it is a gable end. The long side with the slim tank and the small window adjoins it.
  - In photo 3 and own photo 06 the 12-pane gable has that same long side on its left.
  - Own photo 10 shows a third gable with a second 12-pane window, a tall stone plinth and the stone steps (OSM puts the steps at the NNE corner). That is the end of a wing.
  - Seen from outside, the OSM outline's notch is the verandah.
- The 2011 aerial agrees: a rusty main roof along the south-west side and the L to the NNE.

## The hut

| part | measure | how |
|---|---|---|
| main block | 8.6 m long (C → B, along 121°) × 4.4 m | OSM edge B–C; width from photo 3 (gable scaled off the squat tank) and OSM A–B 4.6 |
| ridge | 121°/301°; main pitch 42°, ridge 4.73 m above the hut's ground | photo 3 (gable triangle about 2.2 m over a 2.2 m half-width) |
| front gable | faces **ESE (121°)**, the approach. Vertical grey slabs, horizontal boards in the triangle, the 12-pane window near the centre | photo 3, own photo 06 |
| chimney gable | faces **WNW (301°)**, the car park. The chimney is centred on it: a slab firebox about 1.7 m wide and 1 m deep with a rusty hood, and a tapering iron flue to **about 6.8 m** | photo 5, own photo 12 and 03; the camera solve puts the flue top within 5 px |
| south-west wall | from the chimney end: red-brown horizontal boards about 4.4 m long with a small window (centre about 3.4 m along), then vertical grey slabs to the front corner. The slim tank stands against it about 0.6 m from the chimney corner | photos 4, 5, own photo 06 and 12 |
| wing | off the NNE side of the back 5.4 m, 4.5 m deep (OSM E–D 5.5, C–D 8.9 m). Gable roof, ridge 31°/211°, pitch 33°, ridge about 4.5 m. Its NNE gable end has dark brown boards, a 12-pane window and a tall stone plinth where the ground falls away | OSM; own photo 10 |
| verandah | in the corner: 3.2 m (along the main block) × 3.0 m. Open to the ESE, with three posts (one red). Its back wall is the wing's red-iron ESE wall. A flatter skillion carries the wing's ESE slope down to the front | photo 3, own photo 06 |
| squat tank | on its stone plinth in front of the verandah, centre about (4.6, -1.4) | photo 3, own photo 06 |
| eaves | 2.75 m above the ground (floor 0.35 m on the plinth); overhangs 0.3 m at the eaves, 0.2 m at the gables | |

### Camera solves

Points marked on each photo (hut corners at the ground and the eaves, gable apex, flue top, window), solved for position, heading, tilt and lens (`tools/photomatch/solve_cam.py`):

| photo | camera (E, N) | from the hut | range | lens (vertical) | rms error |
|---|---|---|---|---|---|
| 5 car park | -25.4, -11.0 | 247° | 27.6 m | 48° | 2.8 px |
| 3 drive in | 75.3, -28.7 | 111° | 81 m | 23° (telephoto) | 4.2 px |
| 4 Alamy | -1.5, -25.9 | 183° | 26 m | 61° | 6.4 px |
| own photo 06 | 11.2, -17.5 | 147° | 21 m | 36° | 5.9 px |
| own photo 12 | -32.9, -16.7 | 243° | 37 m | 20° | 9.6 px |

Range and lens trade off against each other when the hut is the only thing marked. Photo 5's solve includes the photographer's eye height (1.6 m over the DEM) to pin it. Photo 3 is telephoto, so its ground positions aren't usable. own photo 12 fits worst: its far corner is about 15 px off, either because the lens is wider than the solve allows or because the wing's side wall isn't quite flush with the chimney gable.

## Around the hut

| feature | where (E, N) | source |
|---|---|---|
| track, today | about 5.5 m south of the 2011 line the game used, all the way to the loop entry. Centreline about (-3, -14.5) south of the hut, bearing 114°; it turns into the loop under the big blackwood and rejoins the old line round the loop (the hairpin apex agrees to under 2 m) | Google, the NSW mosaic, checked with top-down renders (`tools/photomatch/aerial.py`). **The game road moves 5.5 m right up to s = 40, easing back by s = 72** (`scripts/build_route.py` MOVE_*). Route now 7798.7 m (ford s 6627) |
| car park | a dirt turning loop west-south-west of the hut, about E -25 to -6, N -7 to -27. Game: a bare disc centred (-14.5, -17.0), 9 m radius, at its own level (DEM mean, +1.3 m on the old pad) | Google; photo 5 (grass under the camera, dirt to the right) |
| start | the 79 parked in the car park just before the road at (-15.3, -9.3), facing down it (115°), with the hut ahead on the left. The author chose this from the top-down render (20 Oct); it replaced the first version, parked facing the hut to copy photo 5 | `tuning-bush.txt` start_* |
| bollards | the north edge of the track from the car park past the hut (Google); (A row along the car park's north edge, read off photo 5, was removed: the author says those posts don't exist.) The south side of the track from s = 16 | |
| fire ring, woodpile, log bench | seen in photo 5 on the lawn south-west of the hut; placed, then **removed at the author's request** (20 Oct). The OSM fire pit by the picnic table stays | photo 5 |
| picnic table and fire pit | OSM spots, about 13 m north-west. The big blackwood over them is at (-9, 14) | OSM; aerial crown; photo 5 bearing |
| blackwoods (dense, round, low crowns; new "hutwood" plant) | (0.5, 9.5), (6, 5.8), (9.5, 11.5) by the wing and verandah; (-9, 14) over the picnic spot; (-14.5, 12) and (-20, 9.5) towards the forest; (27.9, -21.1) on the north side of the track at the loop entry, its crown over the road edge | NSW aerial crowns; photos 3, 5 |
| forest edge | 35–45 m west and north-west of the hut. Session 3's frost hollow thinned every forest within about 90 m to 15% gums; it now thins only open grass, and forest the imagery shows 35 m or more out stays forest | aerials; photo 3 (tall forest right behind the hut) |
| hitching rail | north edge of the track at s = 22 | photo 3 |
| NPWS sign | north edge at s = 34 (unchanged) | photo 3 |
| plough | just behind the sign, s = 36, 3.8 m out | photo 3 |
| farm gate | by the big blackwood, s = 44 | photo 3 |
| toilet | OSM spot, 37 m west, at the forest edge with its path. The aerial shows the small roof there | OSM; NSW aerial |

## Still uncertain

- Whether the wing's north-west side wall is flush with the chimney gable (OSM says yes; own photo 12 fits a little better if it's set back).
- The small window's exact place on the south-west wall (3.0–4.0 m from the chimney end).
- The squat tank's exact spot, and whether the verandah is open at its NNE end.
- Where the plough faces, and where exactly the gate stands.
- The toilet's look: no photo yet.
