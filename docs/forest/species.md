# Bush Drift — plants along the route

Research notes for modelling low-poly plants along the Alexanders Hut → Tantawangalo bridge route (South East Forest NP, Glenbog area). Compiled 20 Oct 2026.

**How to read this:**
- **[S]** means the point is sourced. Sources are listed at the bottom.
- **[I]** means it is my inference or general botanical knowledge and was not checked against a source for this site.
- Heights and bark descriptions come from PlantNET (Flora of NSW) unless marked [I].
- Trunk diameters and crown-start heights are all [I]. They are typical values for mature trees in tall forest, chosen for modelling, not measurements.

## Plant community types (PCTs) on the route

These come from the NSW State Vegetation Type Map, extant PCT layer. I queried its REST service at 19 points along `data/route_refined.json`, plus 5×5 to 7×7 point grids around the hut, the ford and the bridge.

- **Road corridor:** many points on the centreline return **"Not classified"**. That is the cleared road and its verges.
- **Assignment:** each PCT below is the one mapped right beside the track.

| Where (route index, lat/lon) | PCT | Name | Formation |
|---|---|---|---|
| Hut and its forest edge (0–300; -36.7209, 149.4626) | **3220** | Southeast Tableland Ranges Brown Barrel Fern Forest | Wet sclerophyll (shrubby) |
| First km, heading S/SW (977; -36.7242, 149.4572) | **3221** | Southern Escarpment Messmate Forest | Wet sclerophyll (shrubby) |
| Plateau, west side (1954; -36.7320, 149.4562) | **3300** | Southeast Escarpment Peaks Dry Shrub Forest | Dry sclerophyll (shrubby) — a short patch only |
| Plateau (2500, 4400–6300) | **3221** | Southern Escarpment Messmate Forest | the main plateau type |
| Plateau (2931–3400; -36.738, 149.463–149.467) | **3220** | Brown Barrel Fern Forest | |
| Plateau gully crossing (3908; -36.7432, 149.4708) | **3217** | Southeast Escarpment Wet Layered Forest | Wet sclerophyll (shrubby) |
| Descent and Solomons Creek ford (6655–7000; -36.7402, 149.4974) | **3451** and **3221**, with **3190** nearby | 3451 Far South Sheltered Monkey Gum Forest; 3190 South Coast Hinterland Monkey Gum Wet Fern Forest | 3451 dry sclerophyll (shrub/grass); 3190 wet fern forest |
| Tantawangalo Creek bridge (7816; -36.7424, 149.5070) | **3217**, with **3451** on the slopes | Southeast Escarpment Wet Layered Forest | Wet sclerophyll |

**The hut's frost hollow is not mapped as grassland.** The SVTM marks it "Not classified" and puts PCT 3220 all around it, so the grassland is too small or too modified to be mapped.
- NPWS describes the site as "a plain of naturally treeless frost hollows". [S]
- Its Nunnock walks go "from grassland to snow gum woodland". [S]

**What the PCT descriptions say.** The species lists below come from these BioNet descriptions [S]:
- **3220 Brown Barrel Fern Forest:** a very tall canopy of *E. fastigata*, sometimes with *E. radiata* or *E. viminalis*.
  - Sparse small trees: *Acacia melanoxylon*.
  - Shrubs: *Leucopogon lanceolatus*, *Acacia dealbata*, *Coprosma quadrifida*.
  - Ground: dense bracken (*Pteridium esculentum*) and the tussock grass *Poa meionectes*, plus *Lomandra longifolia* and *Dianella tasmanica*.
  - BioNet names Glenbog as part of its range.
- **3221 Messmate Forest:** found on gentle upper slopes and crests.
  - Canopy: *E. obliqua*, very often with *E. cypellocarpa* and occasionally *E. sieberi*.
  - Shrubs (sparse): *Leucopogon lanceolatus*, *A. dealbata*.
  - Ground: bracken, *Lomandra longifolia* and *Poa meionectes*.
- **3217 Wet Layered Forest:** found in deep gullies and sheltered slopes.
  - Canopy: *E. fastigata* with *E. cypellocarpa*.
  - Small trees: *Pomaderris aspera*, *Bedfordia arborescens*, *Olearia argophylla*, and occasionally *A. melanoxylon* or *A. dealbata*.
  - Occasional tree ferns: *Dicksonia antarctica* or *Cyathea australis*.
  - Ground: ferns such as *Blechnum nudum*, bracken and *Polystichum proliferum*, plus vines.
- **3451 Monkey Gum Forest:** canopy of *E. cypellocarpa* ("monkey gum" = mountain grey gum), with scattered *E. globoidea*. The ground has *Poa meionectes*, *Lomandra* and bracken.
  - BioNet names "Upper Tantawangalo Creek catchment" in its range.
  - ALA has only one *E. globoidea* record within 4 km, so I have left it out.
- **3300 Dry Shrub Forest:** *E. dalrympleana* with *E. radiata*.

## Species to model (16)

**Zones:**
- **A** = frost-hollow grassland around the hut and its hard forest edge.
- **B** = tall wet/damp forest on the plateau.
- **C** = damper, fernier gullies at Solomons Creek and the Tantawangalo bridge.

### Canopy eucalypts

| Species | Zone | Height / trunk | Bark | Crown / foliage |
|---|---|---|---|---|
| **Brown barrel**, *Eucalyptus fastigata* | B (3220), C (3217); dominant near the hut edge | Usually to 50 m [S]. Trunk 1–2 m diameter [I] | Red-brown to grey-brown, short-fibrous to stringy on the trunk and big branches. Smooth white to grey above, shedding in long ribbons [S] — model a rough brown trunk with a pale upper crown and hanging ribbons | Tall straight bole, crown starting at about half to two-thirds of the height [I]. Glossy green leaves [S] |
| **Messmate**, *E. obliqua* | B (3221, the main plateau type); C occasionally | Usually to 50 m, sometimes 90 [S]. Trunk 1–2 m [I] | Grey to red-brown stringybark to the small branches [S] — fully rough and fibrous, the darkest and shaggiest trunk | Broad crown, glossy green leaves 10–15 cm long [S] |
| **Mountain grey gum / monkey gum**, *E. cypellocarpa* | B (3221), C (3217, 3451, 3190) — the descent and ford | To 50 m, sometimes 65 [S]. Trunk 0.8–1.5 m [I] | Smooth white, grey or yellow, shedding in long ribbons [S] — the pale, patchy, ghostly trunk on the descent | Dull to semi-glossy green, long leaves 12–20 cm [S]. Often a short rough "stocking" at the base [I] |
| **Narrow-leaved peppermint**, *E. radiata* | B (an occasional tree in 3220; part of 3300) | To 30 m, rarely 50 [S]. Trunk 0.5–1 m [I] | Grey to grey-brown, short-fibrous "peppermint" bark on the trunk, smooth grey above [S] | Fine, narrow, dark-green leaves; a denser, lower crown than the giants [I] |
| **Snow gum**, *E. pauciflora* | A — the edge of the frost hollow | To 20 m [S], often 6–15 m and multi-stemmed in hollows [I]. Stems 0.3–0.8 m [I] | Smooth white, grey or yellow with insect scribbles, shedding in ribbons [S] — streaky, often with olive or red patches when wet [I] | Twisted, low-branching, spreading crown; glossy green leaves [S]. ALA has 7 records within 1.5 km of the hut [S] |
| **Black sallee**, *E. stellulata* | A — frost-hollow edge, mixed with snow gum | To 15 m [S], usually smaller and multi-stemmed [I] | Grey-black, compact and fibrous on the lower trunk; smooth grey or green above [S] — a dark stocking under a greenish upper trunk | Small rounded crown, short glossy leaves 5–9 cm [S]. ALA has 4 records within 1.5 km of the hut [S]. Placing it on the hollow edge is [I] |

Also present but not given their own row (if you want a seventh gum, use ribbon gum):
- **Ribbon gum**, *E. viminalis*: an occasional tree in 3220. To 30 m, sometimes 50. Smooth white with a rough lower trunk and long ribbons [S].
- ***E. dalrympleana*** (3300 patch).
- ***E. sieberi*** (occasional in 3221).

### Small trees / midstorey

| Species | Zone | Notes |
|---|---|---|
| **Blackwood**, *Acacia melanoxylon* | B, C (plus 8 ALA records near the hut) | Tree 6–30 m; deeply fissured, dark grey-black bark [S]. Dense, dark, dull-green crown of flat phyllodes 6–14 cm long [S]. Model at 8–15 m in the forest [I] |
| **Silver wattle**, *A. dealbata* | B, C — scattered; common on track verges and disturbed ground [I] | Shrub or tree to 30 m; grey to grey-green bark, smooth when young [S]. Feathery, silvery blue-grey foliage [S] — a useful contrast colour. Model at 5–12 m [I] |
| **Hazel pomaderris**, *Pomaderris aspera* | C (3217, 3190); ALA has 5 records near the creeks [S] | Shrub to 4 m [S] (taller in gullies [I]). Large, wrinkled, dark-green leaves with whitish undersides [S]. Use the same mesh for musk daisy-bush, *Olearia argophylla* (to 9 m, silver-backed leaves) and blanket leaf, *Bedfordia arborescens* (to 5 m, white-felted leaf undersides) [S] — silver flashes in the gully understorey |

### Shrub

| Species | Zone | Notes |
|---|---|---|
| **Lance beard-heath**, *Leucopogon lanceolatus* | B, C — the shrub most consistently named in 3220, 3221 and 3451 [S] | Bushy, erect shrub 0.7–3 m; small green leaves; white flower spikes and red fruit [S]. One low-poly bush for the generic understorey filler. *Coprosma quadrifida* (prickly currant bush) is a similar-sized alternative [S] |

### Ferns

| Species | Zone | Notes |
|---|---|---|
| **Soft tree fern**, *Dicksonia antarctica* | C mainly; occasional in B gullies (3217) | Trunk usually to 4.5 m high and ~30 cm thick, covered in red-brown hairs towards the crown and brown fibrous roots lower down; fronds 1–3 m, spreading, dark glossy green [S]. ALA has 5 records within 1.5 km of the ford and bridge and 42 within 4 km of the corridor [S] |
| **Rough tree fern**, *Cyathea australis* | C; occasional in B gullies | Trunk 2.5–20 m high and 16–40 cm thick, covered in old stipe bases (knobbly) [S]. Fronds 2–3 m, dark green, on stipes with spiny dark bases [S]. Model it taller and slimmer than *Dicksonia*, with a more open crown [I]. ALA has 3 records near the creeks and 40 in the corridor [S] |
| **Bracken**, *Pteridium esculentum* | A (edge), B, C — dominant ground cover in every PCT on the route [S] | Fronds erect and stiff, 0.6–1.5 m high, broadly triangular, dark green [S]. Use one dense, waist-high instanced clump everywhere under the forest. Add the clumping fern *Polystichum proliferum* (fronds 50–80 cm [S]) in damp gullies, C |

### Ground layer

| Species | Zone | Notes |
|---|---|---|
| **Tussock grass**, *Poa meionectes* / *P. sieberiana* | A, B, C | *P. meionectes* is a dominant ground-layer species in 3220, 3221 and 3451, and *P. sieberiana* in 3300 [S]. ALA records both within 600 m of the hut [S]. Fine-leaved tussocks about 0.3–0.6 m, grey-green to green [I] |
| **Kangaroo grass**, *Themeda triandra* | A — the hut grassland | Densely tufted, to about 1.2 m when flowering [S]. ALA has 8 records within 1.5 km of the hut [S]. Turns red-bronze or straw-coloured in summer and autumn [I] — a good colour for the open hollow |
| **Spiny-headed mat-rush**, *Lomandra longifolia* | B, C, and the forest edge of A | Tufted, with strap leaves 50–100 cm long [S]. Present in every forest PCT on the route [S]. A bright-green fountain clump [I] |

## Zone notes

**A — Hut frost hollow and its edge.**
- Open grassland of kangaroo grass and *Poa* tussocks, with forbs. ALA within 600 m of the hut shows *Themeda*, *Poa sieberiana*, *P. meionectes* and *Sorghum leiocladum*. [S]
- It also shows exotic pasture grasses from the cattle era, *Holcus lanatus* and *Anthoxanthum odoratum* [S], so a slightly patchy, pasture-like green is realistic.
- The edge is abrupt: brown barrel forest (PCT 3220) right up to the clearing. [S] — the map shows that.
- Snow gum and black sallee as a fringe at the cold rim of the hollow are an inference from the ALA records and the NPWS "grassland to snow gum woodland" wording. [I]
- *Poa labillardierei*: no ALA records within 4 km of the corridor, so I would not feature it.

**B — Plateau.**
- Mostly messmate forest (3221), with brown barrel stretches (3220) and a short drier patch (3300).
- Trees: tall stringybark and brown-barrel trunks, scattered blackwood and silver wattle.
- Understorey: sparse shrubs over a dense, near-continuous layer of bracken, *Poa* and *Lomandra*.

**C — Ford and bridge gullies.**
- The Solomons Creek ford sits in monkey gum forest (3451, with 3190 nearby); the Tantawangalo bridge in wet layered forest (3217).
- Trees: pale mountain grey gum and brown barrel over a layered midstorey of *Pomaderris*, *Olearia* and *Bedfordia*.
- Tree ferns are confirmed in the area by ALA records near both crossings.
- The PCT descriptions rate tree ferns only "occasional", so cluster them along the creek lines rather than carpeting the slopes. [I]
- Ground: *Polystichum* and *Blechnum* ferns plus vines.

## Uncertainty

- The SVTM is modelled mapping at 5 m. It is not a field survey of this track.
- The ford coordinates are approximate.
- Grid sampling found 3190 near the ford, but no centreline point fell in it.
- ALA counts are presence records within a radius, not abundance. They include some observations that may be imprecisely located.
- All trunk diameters, crown-start heights and seasonal colours are [I].

## Sources

- **NSW SVTM extant PCT map (REST identify, layer 3):** https://mapprod3.environment.nsw.gov.au/arcgis/rest/services/VIS/SVTM_NSW_Extant_PCT/MapServer
- **BioNet Vegetation Classification PCT definitions** (OData; PCTID 3220, 3221, 3217, 3300, 3451, 3190): https://data.bionet.nsw.gov.au/biosvcapp/odata/VegetationClassification_PCTDefinition?$filter=PCTID%20eq%203220
- **PlantNET, Flora of NSW species pages** (pattern): https://plantnet.rbgsyd.nsw.gov.au/cgi-bin/NSWfl.pl?page=nswfl&lvl=sp&name=Eucalyptus~fastigata
  - Used for: *E. fastigata*, *obliqua*, *cypellocarpa*, *radiata*, *viminalis*, *pauciflora*, *stellulata*, *Acacia melanoxylon*, *A. dealbata*, *Pomaderris aspera*, *Bedfordia arborescens*, *Olearia argophylla*, *Leucopogon lanceolatus*, *Dicksonia antarctica*, *Cyathea australis*, *Pteridium esculentum*, *Polystichum proliferum*, *Themeda triandra*, *Lomandra longifolia*.
- **Atlas of Living Australia occurrence search** (radius queries around the hut, the creeks and the corridor midpoint): https://biocache-ws.ala.org.au/ws/occurrences/search
- **NPWS, Alexanders Hut campground:** https://www.nationalparks.nsw.gov.au/camping-and-accommodation/campgrounds/alexanders-hut-campground
- **NPWS, Nunnock Swamp and grasslands walking tracks:** https://www.nationalparks.nsw.gov.au/things-to-do/walking-tracks/nunnock-swamp-and-grasslands-walking-tracks
