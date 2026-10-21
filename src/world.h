/* Build B (21 Oct): the finds and the progress kept on the device. Twenty sambar deer standing in the bush
 * (placed once from a seed, so they are in the same spots every run and can be hunted down over many runs), the
 * hidden achievements, and the file that remembers both. The rules are fed one step at a time from a WorldStep, so
 * the game and the headless bots share them. No drawing in here. */
#ifndef WORLD_H
#define WORLD_H
#include <stdint.h>
#include "route.h"

#define WORLD_DEER 20
typedef struct { float x, z, yaw, s, lat; int variant, hidden; } Deer;

enum {
    ACH_CARPARK_DONUT, ACH_OFF_TRACK, ACH_YODEL, ACH_FULL_YODEL, ACH_FORDING,
    ACH_DEER_1, ACH_DEER_10, ACH_DEER_20, ACH_BRIDGE, ACH_NO_BRAKES, ACH_BIG_POT, ACH_CLEAN,
    ACH_JUNCTION_DONUT, ACH_ARMY_MAN, ACH_GONE_WOODIN, ACH_N
};
typedef struct { const char *key, *name, *how; int shown; } AchInfo;   /* shown: named before it's earned (the rest read ???) */
extern const AchInfo ACH[ACH_N];

typedef struct {
    Deer deer[WORLD_DEER]; int ndeer;
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
int world_selftest(const char *dir);
#endif
