/* Build B (21 Oct): the finds and the progress kept on the device. Twenty sambar deer standing in the bush
 * (placed once from a seed, so they are in the same spots every run and can be hunted down over many runs), the
 * hidden achievements, and the file that remembers both. The rules are fed one step at a time from a WorldStep, so
 * the game and the headless bots share them. No drawing in here. */
#ifndef WORLD_H
#define WORLD_H
#include <stdint.h>
#include "route.h"

#define WORLD_DEER 20
#define WORLD_EXTRA 5   /* build C follow-up: five more, spotted by Quayle once he has his rifle back */
typedef struct { float x, z, yaw, s, lat; int variant, hidden; } Deer;

enum {
    ACH_CARPARK_DONUT, ACH_OFF_TRACK, ACH_YODEL, ACH_FULL_YODEL, ACH_FORDING,
    ACH_DEER_1, ACH_DEER_10, ACH_DEER_20, ACH_BRIDGE, ACH_NO_BRAKES, ACH_BIG_POT, ACH_CLEAN,
    ACH_JUNCTION_DONUT, ACH_ARMY_MAN, ACH_GONE_WOODIN,
    ACH_FIRE_1, ACH_FIREWORKS, ACH_CHARCUTERIE, ACH_HUT_NIGHT, ACH_BACK_ON_DUTY,   /* build C, Explorer */
    ACH_CAMP_OVEN,
    ACH_STOCK_ROUTE,   /* build D: the old stock route's ten markers, then its camp */
    ACH_RAZOR, ACH_CLEAN_CUTS, ACH_MIDNIGHT, ACH_SPOTLIGHT, ACH_WET_FEET, ACH_LOW_SLOW, ACH_MECHANIC,   /* build E */
    ACH_TRIALS, ACH_CLEAN_RUN, ACH_TYRE,   /* build E: Creek Trials, a clean run, a tyre changed */
    ACH_FULL_HOUSE,
    ACH_N
};
enum { MODE_DRIFT, MODE_EXPLORE };
enum { IN_DRIFT = 1, IN_EXPLORE = 2, IN_BOTH = 3 };
typedef struct { const char *key, *name, *how; int shown, modes; const char *clue; int section; } AchInfo;   /* build E: a clue for the hidden ones; the section it's listed in */
enum { SEC_DRIFT, SEC_HUT, SEC_BUSH, SEC_CREEK, SEC_QUAYLE, SEC_FIREWOOD, SEC_N };   /* shown: named before it's earned (the rest read ???); modes: where it can be earned */
extern const AchInfo ACH[ACH_N];

/* Build C, Explorer: the hamper hunt. Twelve things from a Hut-trip board hidden round the hut's clearing. */
#define HAMPER_N 12
enum { HM_BRIE, HM_REDWAX, HM_CHEDDAR, HM_CHORIZO, HM_OLIVES, HM_JALAPENO, HM_PEPPERS, HM_DIPS, HM_FETA, HM_CHIPS, HM_BAGUETTE, HM_WINE };
extern const char *const HAMPER_NAME[HAMPER_N];
typedef struct { float x, z, yaw; int placed; } HamperSpot;
#define LOGS_GONE_MAX 128

typedef struct {
    Deer deer[WORLD_DEER + WORLD_EXTRA]; int ndeer, nextra;   /* the extra five follow the twenty (deer[20..24]) */
    uint32_t found;                 /* deer found on this device (bit per deer) */
    /* the follow-up: one toy army man in the bush, aiming at a deer; found like a deer */
    float army_x, army_z, army_yaw; int army_deer, army_placed, army_found;
    float jx, jz; int junction;     /* the Cattlemans junction, for its donut */
    int earned[ACH_N]; char date[ACH_N][12];
    int run_found;                  /* deer found this run */
    /* this run, for the achievements */
    int braked, ditched, yodel_ok, yodel_armed; float yodel_t, prev_s;
    int newly[ACH_N], nnew;         /* earned and not yet shown (world_pop) */
    int dirty;
    /* build C: Explorer, all kept in the progress file */
    int mode;                       /* the mode last chosen on the title (MODE_DRIFT, MODE_EXPLORE) */
    int tray_wood;                  /* split logs in the tray (a load is wood_full) */
    int fire_level;                 /* 0 none, 1..3: the fires lit by the chimney end of the hut */
    int woodpile;                   /* split logs stacked against the hut's west wall (15 a load) */
    float night;                    /* the time of day it is heading for: 0 golden hour, 0.5 dusk, 1 night */
    uint32_t hamper;                /* the hamper's things in the esky (bit per HAMPER item) */
    int board;                      /* the board laid out on the picnic table */
    int hut_night;                  /* the finale has played */
    int rifle;                      /* Quayle's rifle: 0 lost in the bush, 1 in the tray, 2 back with him */
    int quayle;                     /* where Quayle is: 0 at his post, 1 in the tray (rifle back, the extra deer out), 2 in the cab */
    int tray_deer;                  /* a deer in the tray (its index + 1; 0 none) */
    int stew;                       /* the camp oven of venison stew on the table */
    int nlogs_gone; float logs_gone[LOGS_GONE_MAX][2];   /* logs splintered in Explorer (they come back with a new day) */
    /* build D: the creek way's old stock route */
    uint32_t markers;               /* its markers passed (bit per marker; kept for good) */
    int trunk;                      /* the camp's trunk opened today (a load of firewood; closed again with a new day) */
    int camp_seen;                  /* this session: the camp's been reached (its message once) */
    int nmark; float mark_x[12], mark_z[12]; int mark_v[12];   /* where they are (placed at load, not saved) */
    float camp_x, camp_z, trunk_x, trunk_z, trunk_yaw; int camp_on;
    int nwet; float wet_x[48], wet_z[48], wet_yaw[48]; int wet_state[48];   /* build D, part 2: the creek's wet logs: 0 where they lay, 1 shoved (x, z, yaw), 2 smashed; back with a new day */
    uint64_t stag_down[2], stag_logs[2];
    /* build E: Will's chainsaw. saw: sharpness 0..100; razor: logs left at half wear after a razor sharpening; streak: logs
     * split in one pass, running; sharpened: times today (Bush mechanic) */
    float saw; int razor, streak, sharpened;
    int tracked[3];   /* build E: the achievements tracked on the HUD (index + 1; 0 none) */
    float wet_m;      /* build E: metres driven in the creek way this session (Wet feet) */
    int trials_best;  /* build E: the best Creek Trials score banked */
    int have_saw, have_ute;   /* (reading the file) */
    int day;   /* build E: days started (the trial gates are placed fresh for each) */
    float tyre_wear[4], body_dmg; int punct, spare;   /* build E: the ute's condition: tyre wear 0..1 (1 = punctured), a flat (wheel + 1), body knocks 0..1, a spare in the tray */   /* build D: the dead trees knocked down today (bit per stag) and their logs taken (3 bits a stag); back with a new day */
    /* where things are (placed at load, not saved) */
    HamperSpot item[HAMPER_N];
    float fire_x, fire_z;           /* the campfire's middle, off the chimney gable */
    float hut_x, hut_z;             /* the hut's middle (the arrivals: within hut_reach of it) */
    float picnic_x, picnic_z, picnic_yaw;   /* the picnic table */
    float rifle_x, rifle_z, rifle_yaw; int rifle_placed;
    /* this session (not saved): the way down Solomons Creek, for Off the beaten track */
    int creek_from_hut, creek_seen;
} World;
extern World W;

/* One step of the run, as the rules see it. */
typedef struct {
    float dt; int running, finished;
    float x, z, s, lat, drift_deg, speed;
    int on_road;                    /* the car's middle on the gravel or the verge */
    int brake;                      /* the brake pedal (not the handbrake) */
    int ditch_rule;                 /* the ditch rule fired this step (two wheels deep), or the car went off the road */
    float offroad_m;                /* metres driven off the road this run */
    int donut_carpark, donut_road;  /* a donut this step (x, z: where the car is) */
    int yodel_on, yodel_started; float yodel_left;
    float pot; int banked; float banked_value; int lost;
    float ford_s;
    int wood;                       /* logs run over this run */
    int explore;                    /* build C: Explorer mode (the Drift-only achievements can't be earned) */
    int in_clearing, in_creek;      /* in the hut's clearing; in the creek way's strip (Explorer) */
    float creek_s, creek_len, creek_gap_s;   /* how far down the creek way (m), its length, where it leaves the clearing */
} WorldStep;

/* Places the deer (seed: the same seed, the same spots). Returns how many found a spot (WORLD_DEER normally). */
int world_place_deer(const Route *R, uint32_t seed);
void world_run_start(void);
void world_step(const WorldStep *st);
/* A deer touched by the car's outline (centre, yaw, half width, front, rear; metres): its index, or -1. Marks it found. */
int world_deer_hit(float cx, float cz, float yaw, float half_w, float front, float rear);
int world_found_count(void);
/* The army man touched by the outline (marks him found): 1, else 0. */
int world_army_hit(float cx, float cz, float yaw, float half_w, float front, float rear);
/* The next achievement earned and not yet shown, or -1. */
int world_pop(void);
void world_earn(int a);
/* The progress file: deer found and achievements, per device. */
void world_load(const char *path);
void world_save(void);
void world_reset_progress(void);   /* everything found and earned gone (Drift's records kept) */
int world_selftest(const char *dir);
int world_extra_on(void);   /* the extra five deer are out (Quayle has his rifle back) */

/* ---- build C: Explorer ---- */
/* Places the hamper's things, the campfire's spot and Quayle's rifle (the rifle needs the creek way: rx, rz, ryaw from the caller). */
void world_place_explore(const Route *R);
/* The hamper thing touched by the outline (now in the esky), or -1. */
int world_hamper_hit(float cx, float cz, float yaw, float half_w, float front, float rear);
int world_hamper_count(void);
/* The rifle touched (now in the tray): 1. */
int world_rifle_hit(float cx, float cz, float yaw, float half_w, float front, float rear);
/* Quayle touched while the rifle is in the tray: he takes it back (2). Returns 1 when he did. */
int world_rifle_return(float cx, float cz, float yaw, float half_w, float front, float rear);
/* Stopped by the chimney end with a full tray: the wood goes on the fire, the next level (1..3) is returned, else 0. */
int world_deliver(float cx, float cz, float speed);
/* Stopped by the picnic table with all twelve: the board is laid out (1). */
int world_board(float cx, float cz, float speed);
/* Near the fire spot (for the hint) and near the table: metres from the car's middle. */
float world_fire_dist(float cx, float cz);
float world_hut_dist(float cx, float cz);   /* the play test: the hut's things happen within hut_reach of it */
float world_table_dist(float cx, float cz);
/* Stopped by the table with a deer in the tray: with a fire at level 2 or more the stew goes on (1); not hot enough: 2; else 0. */
int world_stew(float cx, float cz, float speed);
/* Start a new day: the fire out, the light back to golden hour, the logs back, the tray emptied. */
void world_new_day(void);
/* A log splintered in Explorer (kept gone till a new day). */
void world_log_gone(float x, float z);
/* Build D: the stock route. Places the markers and the camp from the route (Explorer's creek way). */
void world_place_creek_d(const Route *R);
/* A marker within marker_reach of the car's middle, not passed before: its index (now counted), else -1. */
int world_marker_check(float cx, float cz);
int world_marker_count(void);
int world_ach_progress(int a, float *have, float *need);   /* build E: a counted achievement's progress (0: not a counted one) */
/* At the camp (within camp_reach): 1 the first time this session (2 if that also earned Droving days), else 0. */
int world_camp_check(float cx, float cz);
/* The car's outline at the trunk, closed, with room in the tray: it opens and the tray fills (1); a full tray: 2; else 0. */
int world_trunk_open(float cx, float cz, float yaw, float half_w, float front, float rear);
#endif
