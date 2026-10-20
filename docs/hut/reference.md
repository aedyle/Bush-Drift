# Alexanders Hut — modelling reference

> **Session 3b correction (20 Oct 2026):** sections 1 and 2 below have the hut turned 90°. The ridge runs WNW–ESE, the 12-pane
> front gable faces ESE (the approach), the chimney is centred on the WNW gable (the car park), and a gabled wing runs off the
> back half to the NNE, with the verandah in the corner between them. In the sections below, read "front (SSW)" as ESE,
> "west long wall" as the south-west wall (minus the chimney) and "rear gable" as the wing's NNE gable end.
> The numbers are in `measurements.md`.

Alexanders Hut, Tantawangalo section of South East Forest National Park NSW, at the end of Robinsons Track
beside Alexanders Hut campground (one site). Slab hut built in 1922 by Alexander Robinson, who took over the
summer grazing lease, "from local timber, hessian and corrugated iron" (4x4 Australia, Ron Moon, 2019). Today
NPWS keeps it for day use and as an emergency shelter.

Reference photos are in `refs/hut/` (gitignored, private, reference only, never shipped). Sources and credits are at the end.

Confidence tags: **[sure]** visible in several photos · **[likely]** one photo or inferred · **[guess]** estimate, model to taste.

---

## 1. Orientation (read this first)

Combining the OSM footprint, the 0.6 m NSW aerial tile (`data/raw/imagery/z18/159855_239907.jpg`) and the photos:

- The **front** (the gable end with the big 12-pane window plus the verandah wing) faces roughly **SSW, bearing ~210°**,
  towards the end of Robinsons Track and the camping area. **[likely]**
- The **main block's ridge runs front-to-back, along bearing ~30°/210°** (NNE–SSW), so the front is a gable end. **[sure that the front is a gable; bearing likely]**
- The **verandah wing** sits on the **east** half of the front (right-hand side when you face the front). **[sure: right of the gable in every front photo]**
- The **chimney** is on the **west long wall** (left side when you face the front). **[likely]** Evidence: it shows behind and to the left
  of the front gable in both front photos. Inside, the 12-pane front window is to the *left* of the fireplace, which means you face west to see the fireplace.
  The picnic and fire area appear to the left (north) in the chimney-side photo, which matches OSM placing them NW of the hut.
- Forest: tall wet eucalypt forest wraps the **west and north-west**, its edge about **30–40 m** west of the hut. The toilet sits just inside that edge.
  Big individual trees (black wattle or eucalypt, about 10–12 m tall) stand **5–10 m north and north-east** of the hut. Open, gently sloping natural grassland
  (a frost-hollow plain) runs **east, south-east and south**, with forest again on the far side of the valley, 150 m or more away.

## 2. Footprint and size

From the OSM building way 1347517164 (`building=cabin`, `historic=yes`, `name=Alexander's Hut`), converted to local metres around its centroid
(-36.7208104, 149.4627460). It is an **L-shape of about 63 m²**:

| edge | length | bearing | meaning (my interpretation) |
|---|---|---|---|
| B→C | 8.6 m | 301° | **front line** (main gable front + verandah wing front, flush) |
| C→D | 8.9 m | 30° | **west long wall** of the main block (chimney wall) |
| D→E | 5.5 m | 119° | **rear gable end** of the main block |
| E→F | 4.65 m | 211° | rear half of the main block's **east wall** (exposed) |
| F→A | 3.2 m | 118° | **back wall of the verandah wing** |
| A→B | 4.6 m | 210° | **east end wall of the wing** |

So:
- **Main block: ~5.5 m wide × ~8.9 m long**, gable roof, ridge along the long axis. **[OSM trace; ±1 m]**
- **Verandah wing: ~3.2 m wide × ~4.6 m deep**, on the east side of the front half. **[OSM trace; ±1 m]**
- Overall about 8.6 m across the front × 8.9 m deep.

Cross-checks: the wing front has two verandah bays between three posts (about 1.5 m each), which fits a 3.2 m wide wing. The aerial shows a
rusty roof about 9–12 m across, with a lighter, newer-iron patch at the SE corner, which fits the wing with its silver re-sheeting. One caveat: scaling from the
12-pane window, assuming a sash of about 1 m, gives a narrower front gable of about 3.5–4 m. The squat tank scaling gives about 5 m. OSM traces of small huts are coarse.
**Suggested model: main block 5.0 × 8.5 m, wing 3.2 × 4.5 m.**

Heights **[guess, scaled off people and tanks]**: wall plate (eave) about **2.2 m** above ground. Floor about **0.3–0.4 m** above ground on a stone
plinth and stumps. Main ridge about **4.0–4.3 m**. Gable pitch about **30–35°**.

## 3. Rooms

- **Main room** **[sure]**: board floor, fireplace on the west long wall, the 12-pane window in the front (south) gable wall, and a ledged-and-braced
  timber door (dark red-brown) beside the fireplace. Ceiling of **sagging hessian** (natural brown). Walls lined inside and painted
  **cream or off-white**, with dark-red painted trim and window frame.
- Possibly a **second room** in the rear half of the main block **[guess]**: the 8.9 m length is long for one room, and an inside door sits right of the fireplace.
- **Wing**: an open **verandah** across the front with an **enclosed back wall** of **red-brown painted or rusted corrugated iron**. There is a **stone-and-slab bench with an old
  enamel/cast sink** on the verandah, and an old stove or safe box at its east end **[sure: Flickr verandah photo; NPWS 02]**. Whether there is an enclosed room
  behind the verandah is unclear. Model the verandah about 1.6–1.8 m deep with a closed iron wall behind it.
- A door from the verandah into the main room, in the **east wall of the main block right by the verandah corner** (red-brown door) **[sure: NPWS 02]**.

## 4. Walls

- **Front gable end** (with the window): **vertical** split slabs, wide (25–35 cm) and irregular, with cover strips. Gable triangle above in
  **horizontal** weatherboard-like slabs. Colour: **silver-grey, heavily weathered**, slightly blue-grey in shade. **[sure]**
- **West long wall** (chimney side): **horizontal** slabs, dark brown to grey-brown. The southern part beside the tank is warmer **reddish-brown**
  (old paint or redgum), with one **small window** (about 0.6 × 0.8 m, two panes or a casement) between the tank and the front corner. The last 1.5 m at the SW
  corner is **vertical grey slabs**. **[sure: Flickr chimney photo, NPWS 05]**
- **Verandah back wall**: **corrugated iron, rust-red/brown** (reads as a painted red oxide). **[sure]**
- Corner posts are round or adzed timber. Bottom plates sit on a **dry-stone plinth** (grey granite cobbles), visible under the verandah and the tank. **[sure]**
- The hessian is **inside** (ceiling and probably wall lining). No hessian shows outside. **[likely]**

## 5. Roof

- **Main block: gable**, ridge front-to-back (bearing ~30°), **pitch about 30–35°**. Corrugated iron with the corrugations running down the slope. **[sure: front gable]**
- **Overhangs**: small. About 0.2–0.3 m at the eaves, and about 0.15 m at the gable (thin barge board). Quad-profile galvanised **gutters** on the long sides,
  with a downpipe to each tank. **[sure]**
- **Colour**: mottled **rust orange-brown (#8a4a2a to #a85a32)** with **grey-silver patches and streaks** where the zinc survives, plus a few
  newer **bright silver sheets** (a strip along the east slope, ridge capping, and the wing roof's streaky re-sheeting). Silver ridge capping. **[sure]**
- **Verandah wing**: a lower roof, read as a **hip or skillion falling to the front (south)**, top tucked under the main roof's east slope, with a hipped
  east end. Rusty with **vertical silver streaks** (patched sheets). Verandah eave about 2.1 m. **[likely]**
- Uncertain: in both chimney-side photos the far end of the roof reads like a **hip**. That is probably wide-angle distortion of the gable barge, but the rear
  (north) end could be hipped. A gable is the safe choice.

## 6. Chimney (the landmark)

- On the **west long wall**, roughly **mid-length** (perhaps a little towards the front). **[likely]**
- **Firebox enclosure**: a box about **1.6–1.8 m wide, about 0.8–1.0 m deep, about 2.0–2.2 m tall**, clad in **weathered vertical grey timber slabs**, sitting on
  stone. Its top is a **rusty corrugated or flat iron sloped "shoulder" hood** that funnels into the flue. **[sure]**
- **Flue**: a **tall, tapering, square-ish sheet-iron stack**, about 0.6 m at the base narrowing to about 0.35–0.4 m, with a **dented, slightly leaning top section**.
  Colour **deep rust brown, darker at the top**. The top stands about **5.5–6 m above ground**, so **1.5–2 m above the ridge**. **[sure for form; heights guessed]**
- Inside: an open fireplace about 1.4 m wide, **brick-lined** (red-brown, sooty), with a timber mantel shelf. **[sure]**

## 7. Doors and windows

- **Front gable**: one **12-pane window** (3 wide × 4 high, white or cream glazing bars, about 1.0 × 1.3 m), **centred slightly left**, sill about 0.8 m above floor. No door in the
  front gable. **[sure]**
- **East wall of the main block, front half** (inside the verandah): a **door**, about 0.8 × 1.9 m, red-brown painted boards. **[sure]**
- **West wall**: a small window south of the chimney (see §4). Possibly a second door north of the fireplace (inside view). **[likely / guess]**
- The verandah is open to the south: **3 posts** (two bays), square-sawn or round timber, about 0.12 m, one painted **red** (Flickr). **[sure]**

## 8. Water tanks (two)

1. **Front tank** **[sure]**: squat **galvanised corrugated** tank, **about 2.0–2.2 m diameter × 1.5 m tall**, silver-grey with dull streaks (newer than
   the hut), on a **circular dry-stone plinth about 0.5–0.6 m high**. It stands **at the front, in the inside corner** where the gable block meets the verandah wing,
   in front of the verandah's west bay, slightly forward of the front line. Fed by a downpipe from the main roof's east gutter.
2. **Side tank** **[sure]**: **slim, tall galvanised corrugated** tank, about **1.0–1.1 m diameter × 1.8–1.9 m tall**, on flat stones at ground level,
   against the **west wall just south of the chimney**, with a downpipe from the west gutter.

## 9. Ground, floor, yard

- **Floor**: timber boards, raised. The hut sits on a low **dry-stone plinth** of grey granite rocks, most visible under the verandah and front. **[sure]**
- Mown or grazed short **grass** right up to the walls. Patches of bare dirt at the fire area. **[sure]**
- **Bollards**: **round timber bollards** about **0.6–0.8 m tall, 0.25–0.3 m diameter**, grey and weathered, flat-topped, **about 2–3 m apart**. They line both sides
  of the gravel track where it runs past the front of the hut and the turnaround, and they ring the day-use area on the hut's west side. **[sure]**
- **Old split-post fences**: tall (1.2–1.5 m), lichen-covered grey split posts carrying 3–4 plain wires. One line runs from the hut's **east side across the paddock**
  (seen running right from the verandah in the plough photos). There are remnants elsewhere. **[sure]**
- **Entrance**: a **log rail** (one horizontal log on two posts, about 1.1 m high) beside the track at the approach. An **old rusty iron farm gate** leans in the scrub
  on the right of the approach. **[sure: NPWS 04]**
- **Sign**: an NPWS **green-and-white routed sign "Alexanders Hut"** on two short posts beside the track at the approach, low (about 0.6 m). **[sure: NPWS 04]**

## 10. Visitor additions

- **Fire ring and bench** **[sure]**: on the **west/north-west side** of the hut, a steel fire ring at ground level, a **plank-on-two-posts bench**, and a **woodpile**
  of split logs. OSM `leisure=firepit` sits about 12 m NW of the hut centre.
- **Picnic / BBQ area** **[sure]**: timber **picnic tables with bench seats** (grey-brown timber) to the **north-west**, ringed by bollards. OSM `leisure=picnic_table`
  is about 13 m NW. NPWS lists "barbecue facilities, fire rings (bring your own firewood), picnic tables".
- **Toilet**: OSM `amenity=toilets`, **pit latrine, seated, unisex**, about **37 m due west**, at the forest edge, reached by a short unpaved footway. NPWS says
  "non-flush toilets". **No photo found.** The aerial shows a small light-grey roof there. For the model, use the standard NPWS **single-cubicle composting/pit
  toilet**: about 1.5 × 2 m, **khaki-green or grey Colorbond** (or timber-clad) walls, **skillion Colorbond roof** in grey or green, a vent pipe, and a small ramp or step. **[guess]**
- A small **ring of stones** (old fire ring or rock pile, about 1.2 m across) sits in the grass in front of the hut. **[sure: NPWS 01]**

## 11. The plough

- A **horse-drawn wheeled ("sulky") plough or disc implement**, fully rusted orange-brown. It has **two large spoked iron wheels** (about 0.9 m and 0.6 m), a frame
  with **curved mouldboard/disc pans**, a seat-stalk stub, and a **long drawbar/pole about 2.5 m** trailing downhill. **[sure for look; exact type a guess,
  probably a stump-jump mouldboard or disc plough]**
- **Where**: in the open paddock **in front and to the right (south-east) of the hut, about 20–30 m away**, on the grass slope just beyond the fence posts, pole
  pointing down-slope (towards the camera in the photos). **[sure: NPWS 01, Flickr plough photo]**

## 12. OSM data within about 80 m (from `data/raw/osm-hut-context.json`)

Local offsets are metres east (E) / north (N) of the hut centroid (-36.7208104, 149.4627460).

| feature | OSM | lat, lon | offset | tags |
|---|---|---|---|---|
| **Hut** | way 1347517164 (6 nodes) | centroid -36.72081, 149.46275 | 0, 0 | `building=cabin`, `historic=yes`, `name=Alexander's Hut` |
| **Toilets** | node 5172627553 | -36.7208106, 149.4623282 | 37.3 W, 0.0 | `amenity=toilets`, `access=yes`, `fee=no`, `toilets:disposal=pitlatrine`, `toilets:position=seated`, `unisex=yes` |
| **Picnic table** | node 12464790785 | -36.7207278, 149.4626410 | 9.4 W, 9.1 N | `leisure=picnic_table` |
| **Fire pit** | node 12464790786 | -36.7207259, 149.4626569 | 7.9 W, 9.3 N | `leisure=firepit` |
| **Campground** | way 1347517121 (closed, 6 vertices) | see below | hut on its NW edge | `tourism=camp_site`, `name=Alexander's Hut Campground`, `tents=yes`, `caravans=yes`, `reservation=required`, `fee=yes`, `note=no camping fee, however booking fee payable` |
| **Robinsons Track** | way 533120940 (32 nodes) | ends at -36.7208655, 149.4626325 | ends 10 W, 6 S | `highway=track`, `tracktype=grade3`, `name=Robinsons Track`, `source=Survey` |
| footway to toilet | way 1347517098 | -36.7208326, 149.4625073 → -36.7208009, 149.4623490 | 21 W → 35 W | `highway=footway`, `surface=unpaved` |
| Wilkinson's Walking Track | way 1347517120 | -36.7209874, 149.4629791 → -36.7208450, 149.4629183 → -36.7207638, 149.4627748 → -36.7207520, 149.4627817 | 21 E, 20 S → 15 E, 4 S → 3 E, 5 N | `highway=path`, `foot=designated`, `surface=unpaved` |
| Wilkinson's — stone steps | way 1347517119 | -36.7207520, 149.4627817 → -36.7207316, 149.4627937 | 3 E, 6.5 N → 4 E, 9 N | `highway=steps`, `material=stone`, `step_count=4`, `incline=down` |
| Wilkinson's Walking Track | way 1347517118 (10 nodes) | -36.7207316, 149.4627937 → … → -36.7196954, 149.4623470 (heads N, 125 m) | 9.7 m nearest | `highway=path`, `foot=designated`, `surface=unpaved` |
| Wilkinson's Walking Track | way 1347517178 (156 nodes) | nearest about 48 E, 37 S | 60 m nearest | `highway=path`, `foot=designated`, `surface=unpaved` |

(Other Wilkinson's segments, a timber bridge and timber steps, are 120–140 m north, outside the radius.)

Hut polygon vertices (lat, lon → E, N m):
- A -36.7208278, 149.4627842 → 3.4, -1.9
- B -36.7208638, 149.4627580 → 1.1, -5.9
- C -36.7208240, 149.4626755 → -6.3, -1.5
- D -36.7207541, 149.4627258 → -1.8, 6.2
- E -36.7207783, 149.4627795 → 3.0, 3.55
- F -36.7208145, 149.4627529 → 0.6, -0.45

Campground polygon (lat, lon → E, N m): (-36.7208998, 149.4631161 → 33.0, -9.9), (-36.7210148, 149.4630443 → 26.6, -22.6),
(-36.7211696, 149.4629391 → 17.2, -39.7), (-36.7209707, 149.4624241 → -28.7, -17.7), (-36.7208326, 149.4625073 → -21.3, -2.5),
(-36.7206859, 149.4625957 → -13.4, 13.8). It is a grassy area mostly **south and south-west** of the hut, about 60 × 40 m.

Notes on the OSM: the picnic table and fire pit are only 1.4 m apart (one day-use spot). Wilkinson's Walking Track (from Nunnock)
reaches the hut from the SE and leaves north up 4 stone steps right beside the hut's NE corner. The track's last stretch comes from the ESE along the south
side of the hut and ends 10 m WSW of it. The aerial shows that turnaround at the hut's SW.

## 13. Sources and image credits

All saved images are © their owners and are kept **privately for reference only**. Nothing here is licensed for redistribution.

| file | shows | source page | credit / licence |
|---|---|---|---|
| `plough-hut-forest-npws.jpg` | front gable + verandah + tank from the SSW, plough, fence posts, forest | https://www.nationalparks.nsw.gov.au/camping-and-accommodation/campgrounds/alexanders-hut-campground (image 01) | John Spencer © DPIE |
| `tank-closeup-npws.jpg` | front tank on stone plinth, gable window, verandah, door, iron wall | same page (02) | John Spencer/OEH |
| `camping-area-npws.jpg` | hut across the gravel track, bollard row, plough in distance | same page (03) | Lawani Colley/OEH |
| `approach-road-npws.jpg` | approach track, bollards, log rail, sign, gate, front of hut, forest behind | same page (04) | John Spencer/OEH |
| `woodpile-bbq-npws.jpg` | west (chimney) side, slim tank, fire ring and bench, woodpile, picnic area | same page (05) | John Spencer/OEH |
| `interior-fireplace-npws.jpg` | interior: hessian ceiling, painted lining, brick fireplace, 12-pane window, door, board floor | same page (06) | John Spencer/OEH |
| `chimney-tank-side-flickr-ozjulian.jpg` | best chimney and roof view, slim tank, people for scale | https://www.flickr.com/photos/ozjulian/52686329588/ | Julian Robinson, All rights reserved |
| `plough-hut-flickr-ozjulian.jpg` | front + plough, sunny, higher resolution | https://www.flickr.com/photos/ozjulian/52686114329/ | Julian Robinson, All rights reserved |
| `verandah-sink-flickr-ozjulian.jpg` | verandah interior: sink bench, iron wall, red post, view east over the paddock | https://www.flickr.com/photos/ozjulian/52685319707/ | Julian Robinson, All rights reserved |

Text sources:
- NPWS campground pages (overview and visitor info): one site, day-use hut and emergency shelter, non-flush toilets, fire rings, picnic tables, barbecue facilities.
  https://www.nationalparks.nsw.gov.au/camping-and-accommodation/campgrounds/alexanders-hut-campground/visitor-info
- 4x4 Australia, "Exploring NSW South-East part 2" (Ron Moon, 29 Jun 2019): 1890s first hut; 1922 Alexander Robinson's slab hut of timber, hessian and iron.
  https://www.4x4australia.com.au/explore/nsw/exploring-nsw-south-east-part-2-4x4-adventure-series
  (Careful: the hut in that article's lead photo and the "few km south of Cathcart, on forestry land, prefabricated in the USA" text are about the
  **Woolingubrah Inn**, not Alexanders Hut. Some search summaries mix them up.)
- Sapphire Coast / VisitNSW listing: "set at the forest edge on a plain of naturally treeless frost hollows".
- NSW Spatial Services imagery tile (data/raw), used for the forest edge, tree positions and roof colour.

Searched without useful hits: Wikimedia Commons (nothing of the hut), Hemax Camper article (blocked).
