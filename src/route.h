/* Bush Drift's road: the real route from Alexanders Hut to the Tantawangalo Creek bridge, read from the
 * files scripts/ builds (data/built/: route.csv, meta.txt, terrain.bin, canopy.bin). Holds the centreline (every 2 m), the ground height anywhere near it (road,
 * verge, ditch, then the DEM), the trees, and the chunk geometry, which a worker thread builds as the car
 * nears it (the game takes at most one finished chunk a frame and
 * never waits). Physics axes as the car's: x = east, z = north, y = up (metres; heights relative to the
 * start pad). No drawing calls in here: chunks come out as plain vertex and colour arrays. */
#ifndef ROUTE_H
#define ROUTE_H
#include <stdint.h>

#define RT_DS 2.0f          /* metres between centreline samples */
#define RT_CHUNK 60         /* samples per chunk: 120 m */
#define RT_MAX_CHUNKS 128

enum { SURF_ROAD, SURF_VERGE, SURF_DITCH, SURF_ROUGH, SURF_DECK, SURF_BED, SURF_WATER, SURF_ROCK };   /* build C: the creek way's gravel bed and its water; build D: its rocks (rock gardens, rocky climbs) */

/* Everything standing beside the road (session 3): a copy of one of the plant or prop templates (tools/make_plants.py,
 * tools/make_hut.py). kind: 0 = a trunk or post (the car's run ends on it, r = its radius), 2 = soft (shrubs, ferns,
 * grass, litter, logs: scenery only), 3 = solid but not drawn (the hut's walls), 4 = drawn, not solid (the hut and
 * props, which have kind 3 rings for collisions). t0/t1: the near and far template (-1 = none); lat: metres off
 * the centreline, which decides which of a chunk's parts the plant goes in. */
typedef struct { float x, z, r, h, yaw, scale, lat; uint32_t seed; unsigned char kind, wood; short t0, t1; } RtTree;   /* build C: wood = 1 on a firewood log and its posts (the only logs in Explorer) */

/* Plant and prop templates, read from assets-built/plants.txt and hut.txt. Triangles in the game's axes about the
 * plant's base; kind per triangle: 'b' bark/wood, 'l' leaves, 'g' ground cover. */
typedef struct { char name[16], zone[4]; int variant, lod, ntri; float h, r; char *kind; float *rgb, *p; } RtTemplate;
int route_templates_load(const char *path);     /* adds a file's templates; returns how many (0 = not found) */
int route_template_find(const char *name, int variant, int lod);
int route_template_count(void);
const RtTemplate *route_template(int i);

typedef struct {
    /* centreline */
    int n;
    float *x, *z, *h, *hdem, *head, *curv; /* h: road surface (smoothed), relative to elev0; head: radians, 0 = east, anticlockwise */
    float length, elev0;
    /* meta */
    float lane_w, lane_lo, lane_hi;
    float start_x, start_z, start_yaw;      /* yaw in the car's convention: 0 = +z (north), positive towards +x (east) */
    float pad_x, pad_z, pad_r, pad_h;
    float ford_s, bridge_s0, bridge_s1, hut_x, hut_z, clearing_r;
    float ford_lo, ford_hi, ford_water;     /* the ford's water: along the road from lo to hi (m), at this height */
    /* terrain grid (5 m) */
    int gx, gy; float gx0, gz0, cell;
    float *dem;                             /* relative to elev0, gaps filled */
    unsigned char *canopy;                  /* 0 open .. 255 tree cover; NULL = not available */
    short *owner;                           /* chunk owning each cell (by its centre's nearest point), -1 = too far */
    float *vh;                              /* ground height at each grid vertex ((gx+1) x (gy+1)), NAN where not needed */
    int *cell_first, *cell_list;            /* cells of each chunk */
    /* nearest-point index: buckets of samples */
    float bx0, bz0, bcell; int bnx, bnz; int *bfirst, *blist;
    /* trees, sorted by chunk; and an index by grid cell for collisions */
    RtTree *tree; int ntree; int tree_first[RT_MAX_CHUNKS + 1];
    int nsolid;                             /* trunks, posts and hut walls (kinds 0 and 3) */
    int *tcell_first, *tcell_list;
    int nchunks;
    /* build B: the map's edge. hut_d: metres from each grid point to the hut's open frost hollow (0 inside it);
     * chunk_r: how far each chunk's ground reaches from its centre (the hollow makes chunk 0 bigger) */
    float *hut_d; unsigned char *known; int hollow_cells; float *hdem_far_tmp;
    float chunk_r[RT_MAX_CHUNKS];
    /* the junction (follow-up): Cattlemans Link Trail leaving the route at junction_s; a stub of it every 2 m
     * (spur_x/z/h, spur_n points) up to a gate across it at gate_t metres */
    float junction_s, jx, jz; int spur_n; float spur_x[64], spur_z[64], spur_h[64]; float gate_t, gate_x, gate_z, gate_yaw;
    /* build C: Explorer. explore = the world as Explorer has it (only the firewood logs; the creek way open); the chunks are
     * rebuilt when it changes. nwood: the firewood logs, by where they lie */
    int explore;
    int nwood, wood_clearing, wood_road, wood_creek, wood_bridge;
    /* build C: the creek way (creek.csv, creek_dem.bin; scripts/build_creek.py): a second line, down Solomons Creek from the
     * hut's clearing to the ford, with its own strip of ground and bush in chunks nchunks .. nchunks + ck_nch - 1. Open only in
     * Explorer (its edge joins the world's); its chunks are streamed only while creek_stream is set (the car near or in it). */
    int ck_n, ck_nch; float *ck_x, *ck_z, *ck_bed, *ck_head; float ck_len;
    float ck_exit_s;                        /* where it leaves the clearing: the way in */
    float ckb_x0, ckb_z0, ckb_cell; int ckb_nx, ckb_nz; int *ckb_first, *ckb_list; unsigned char *ckb_near;
    float ck_jam_s[4]; int ck_jam_side[4], ck_njam; float ck_fall_s; int ck_fall_side;
    int creek_stream;
    float tell_x, tell_z, tell_yaw, post_x, post_z, post_yaw;   /* the way in's blazed tree and fallen post (drawn by the clearing's chunk) */
    /* build D: the harder creek. Rock gardens (ck_rock_s0..s1: humped rocks across the bed, Low Range country), climb-outs
     * (ck_byp: a raised rocky bench on one side round a log jam, a fallen tree right across, or the waterfall), fallen trees
     * (ck_df: 0 part way across, 1 right across, 2 along the bank), the old stock route's markers, and the cattleman's camp */
    int ck_nrock; float ck_rock_s0[6], ck_rock_s1[6];
    int ck_nhump; float *ck_hump;           /* rocks under the rock gardens: x, z, radius, height (4 floats each), sorted by s */
    int *ck_hump_first;                     /* per 2 m creek sample: the first hump at or after it (ck_n + 1) */
    int ck_nbyp; float ck_byp_s[8], ck_byp_h[8]; int ck_byp_side[8], ck_byp_kind[8];   /* kind 0 log jam, 1 fallen tree, 2 the waterfall */
    int ck_ndf; float ck_df_s[24]; int ck_df_kind[24], ck_df_side[24];
    int ck_nmark; float ck_mark_x[12], ck_mark_z[12], ck_mark_yaw[12], ck_mark_s[12]; int ck_mark_v[12];
    float camp_s, camp_x, camp_z, camp_yaw, trunk_x, trunk_z, trunk_yaw; int camp_on, camp_side;
    /* build D, part 2: the creek way's drive-over things. Each is a bump in Explorer's ground, shaped as a 0.45 m tyre rolling
     * over it: stones (domes), stumps (rounded blocks), branches, small logs and mossy fallen trees (ridges along yaw, from half
     * length l each side), boulders (broad domes). Indexed on a 4 m grid. Wet logs are listed separately (they move: main.c). */
    int ck_nbit; struct RtBit { float x, z, yaw, l, r, h; unsigned char type, var; } *ck_bit;
    float ckg_x0, ckg_z0; int ckg_nx, ckg_nz; int *ckg_first, *ckg_list;
    int ck_nwet; struct RtWet { float x, z, yaw; int var; } ck_wet[48];
    int ck_nmoss; float ck_moss_s[32];   /* where the mossy fallen trees cross (the crawl test, the bot's slow places) */
    /* build D: the stags, standing dead gums (Explorer only; not in the tree list, so Drift never meets them). Each falls
     * along fall (radians, map convention) and lies as three firewood logs at log_x/z (yaw log_yaw) */
    int nstag; struct RtStag { float x, z, r, h, fall, log_x[3], log_z[3], log_yaw; int variant, where; } stag[48];   /* where: 0 road, 1 clearing, 2 creek */
    int loaded;
} Route;

typedef struct { float s, lat, dist; int i; } RtNear;

/* Reads data/built (dir). Returns 1 on success; prints what is missing otherwise. */
int route_load(Route *R, const char *dir);
/* Nearest point of the centreline: s along the road, signed lateral (+ = left of the direction of travel). */
RtNear route_near(const Route *R, float x, float z);
float route_road_h(const Route *R, float s);
float route_dem(const Route *R, float x, float z);
/* Ground height and the surface kind at a point (the profile the meshes are built from). */
float route_height(const Route *R, float x, float z, int *surf);
float route_heading_at(const Route *R, float s);         /* radians, map convention */
void route_point(const Route *R, float s, float *x, float *z);
/* Half the drivable width and the other cross-section sizes, from the tuning values. */
float route_half_width(void);
/* Build B, the map's edge: how far a point is into the edge (metres; below 0 = inside the world). The ground rises
 * into a bank from 0 to edge_w, dense trees stand past it, and the car is held back from edge_stop. */
float route_edge(const Route *R, RtNear n, float x, float z);
float route_edge_at(const Route *R, float x, float z);
/* In the hut's open frost hollow (or the car park): counts as road for the off-road rule and the ditch rule. */
int route_in_clearing(const Route *R, float x, float z);
/* The side road (Cattlemans Link Trail's first metres): distance from its middle and how far along it (m); 1e9 if far. */
float route_spur_dist(const Route *R, float x, float z, float *t);
int route_on_spur(const Route *R, float x, float z);   /* on its gravel or verge, short of its end */
/* Build C, the creek way: the nearest point of its line (s from upstream, lat + = left going downstream; dist 1e9 if there's
 * none), a point and the bed's height along it, how wide its strip is there, and whether a point is in its strip (open or not). */
RtNear route_creek_near(const Route *R, float x, float z);
void route_creek_point(const Route *R, float s, float *x, float *z);
float route_creek_bed(const Route *R, float s);
float route_creek_bedw(const Route *R, float s);
float route_creek_half(const Route *R, float s);
int route_in_creek(const Route *R, float x, float z);
/* Build D: the bot's line down the creek way at s: metres to the left of the line to hold (0 on the bed), and 1 in a place
 * to go slowly (a climb-out, a rock garden) */
float route_creek_line(const Route *R, float s, int *slow);
enum { BIT_STONE, BIT_STUMP, BIT_BRANCH, BIT_SMALLLOG, BIT_MOSSY, BIT_BOULDER, BIT_TWIGS };
float route_creek_bump(const Route *R, float x, float z);   /* build D, part 2: the height the creek's things add here (Explorer), as a 0.45 m tyre feels it */
float route_creek_bump_r(const Route *R, float x, float z, float tyre_r);   /* the same for a rounder or sharper feeler (the belly: their real shape) */
int route_creek_rocky(const Route *R, float s);   /* in a rock garden */
void route_place_stags(Route *R);   /* build D: after the trees and the creek way's features */
/* A trunk the car's outline touches (centre pos, yaw and the outline's sizes); returns its index or -1. */
int route_tree_hit(const Route *R, float cx, float cz, float yaw, float half_w, float front, float rear);
/* Build C: the ute's shape for small posts, n slices along it (z0..z1 metres, half width hw); 0 = the box as for trunks. */
void route_set_post_outline(int n, const float *z0, const float *z1, const float *hw);
/* Least radius of the centreline within `ahead` metres after s. */
float route_min_radius_ahead(const Route *R, float s, float ahead);
float route_speed_ahead(const Route *R, float s, float ahead, float alat, float abrake); /* m/s */

/* ---- chunk geometry ---- */
/* A chunk comes in parts (session 3) so the game can draw detail only where it shows:
 *   GROUND  road, verge, ditch, terrain, water, bridge;
 *   NEAR    plants within RT_NEAR_LAT of the centreline in full detail, and all the ground cover;
 *   NEARLO  the same plants' far versions (drawn instead of NEAR when the chunk is far from the camera);
 *   BACK    plants further out, always in their far versions, and the hut and props.
 * Shadows are cast by NEARLO and BACK. Vertex colour: rgb = the colour fully lit; alpha = 2 * round(127 * share of
 * the light that comes from the sun) + 1 if the spot is in the sun by the baked dapple (used beyond the shadow map).
 * Texture coordinates say what the surface is: (s, 100 + lat) on the road, (0, 1) terrain, (0, 3) far land, else (0, 0). */
enum { RT_PART_GROUND, RT_PART_NEAR, RT_PART_NEARLO, RT_PART_BACK, RT_PART_EDGE, RT_NPARTS };   /* build B: EDGE = plants 40 m or more from the road (the edge's bank), drawn far, no shadows */
#define RT_NEAR_LAT 32.0f
typedef struct { int nverts, cap; float *v, *t; unsigned char *c; } RtPart;  /* triangles = nverts / 3 */
typedef struct {
    int index;
    float base_x, base_z;       /* positions are relative to this */
    RtPart part[RT_NPARTS];     /* xyz (drawing axes: z mirrored), uv and rgba */
    double build_ms;
    int ntri_road, ntri_terrain, ntri_trees, ntri_scrub, ntri_other;
} RtGeom;
/* Places the trees again (the cross-section tuning changed). */
void route_replace_trees(Route *R);
/* Follow-up: logs break when hit. route_log_break takes the hit post's index and returns the log's chunk (-1: not a log);
 * route_logs_restore puts every broken log back (changed is told each chunk to rebuild). */
int route_log_break(Route *R, int post);
void route_logs_restore(Route *R, void (*changed)(int chunk));
/* Build C: the log a post belongs to (its index), and breaking the log lying at x, z (Explorer's saved ones); -1 if none. */
int route_log_index(const Route *R, int post);
int route_log_break_at(Route *R, float x, float z);
void route_build_chunk(const Route *R, int index, RtGeom *g);
void route_geom_free(RtGeom *g);
/* Build B: one template (a deer) set on the ground and lit, into g's GROUND part. */
void route_build_model(const Route *R, int tpl, float x, float z, float yaw, float scale, RtGeom *g);
void route_chunk_centre(const Route *R, int index, float *x, float *z, float *radius);
float route_chunk_line_r(const Route *R, int index);

/* The far land (session 3): ground and forest canopy out to a few km from data/built/far.bin, so the horizon is hills
 * and trees. Built once into tiles (each an RtGeom with only its GROUND part); 0 tiles if the file is missing. */
#define RT_FAR_MAX_TILES 128
int route_far_build(const Route *R, const char *path, RtGeom *tiles, int max_tiles);
/* The sun (physics axes), shared by the baked light and the game's shadows and sky. */
void route_sun_dir(float *x, float *y, float *z);

/* Streaming on a worker thread. Each frame: route_stream_want(car position), then take at most one
 * finished chunk with route_stream_take (the caller uploads it and keeps it until route_stream_drop says
 * it is no longer wanted). */
void route_stream_start(const Route *R);
void route_stream_stop(void);
void route_stream_want(float x, float z, float radius);
int route_stream_take(RtGeom *out);             /* 1 = a chunk was handed over (the caller owns its arrays) */
int route_stream_wanted(int index);
void route_stream_invalidate(int index);          /* build this chunk again (a log broke); the old one stays drawn till then */
int route_stream_pending(void);                 /* wanted chunks not yet handed over */
typedef struct { long built; double ms_sum, ms_max; } RtStreamStats;
RtStreamStats route_stream_stats(void);
#endif
