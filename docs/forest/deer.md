# The deer (build B)

Twenty static deer stand in the bush along the route; drive into one to find it.

## Which deer

**Sambar (*Rusa unicolor*).** No source found names the deer species for the South East Forest National Park or
Tantawangalo itself, so this is a judgement from what is published nearby:

- NSW has five feral deer species; fallow, red, sambar and rusa are widespread
  ([NSW DPI, feral deer](https://www.dpi.nsw.gov.au/biosecurity/vertebrate-pests/pest-animals-in-nsw/feral-deer/feral-deer)).
- On the Far South Coast, South East Local Land Services is trapping and collaring **fallow and sambar** deer between
  Kiah and Moruya under the Far South Coast Wild Deer Management Plan
  ([Bega District News](https://www.begadistrictnews.com.au/story/6826857/wild-deer-between-kiah-and-moruya-to-be-trapped-collared-and-released/);
  [Far South Coast Conservation Management Network](https://fsccmn.com/?p=2636)).
- Sambar are the deer of the **tall wet eucalypt forests** of south-eastern Australia
  ([Australian Zoologist 41, 2021: "Modelling the factors influencing Sambar Deer (*Rusa unicolor*) occurrence in the wet
  eucalypt forests of south-eastern Australia"](https://www.nespthreatenedspecies.edu.au/media/rcfhm3ll/modelling-the-factors-influencing-sambar-deer-rusa-unicolor-occurrence-in-the-wet-eucalypt-forests-of-south-eastern-australia.pdf)).
  The route is tall wet forest at 650 to 950 m (`species.md`), so sambar fits it better than fallow, which favours more
  open, grassy country.

## The model

`tools/make_world.py`: a big, dark brown, shaggy-necked deer with large rounded ears and a dark tail with a chestnut
patch under it. Hinds about 1.2 m at the shoulder; stags bigger, with three-tined antlers (sambar antlers have three
tines). Four poses: hind standing, hind grazing, stag standing with its head turned, stag grazing. About 220 to 280
triangles near, 80 to 150 far. The coat is a little lighter than a real sambar's so they read under the canopy.

## Where they stand

`src/world.c` places them from a seed (`deer_seed`, default 1), so they are in the same spots every run: three round
the hut, seven on the plateau, six on the descent, two at Solomons Creek ford and two near the bridge. Seven stand 9 to
16 m from the road with a clear view to it; thirteen are 20 to 42 m out, behind scrub. Each spot is checked to be
reachable from the road in the ute: a search on a 1 m grid round the trunks and logs, and no slope over 28 %.
`deer_random = 1` puts them somewhere new each run instead.

