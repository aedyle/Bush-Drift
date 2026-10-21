/* Build B: the deer, the achievements and the progress file (world.h). */
#include "world.h"
#include "records.h"
#include "tuning.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

World W;
static char path_[300];

const AchInfo ACH[ACH_N] = {
    { "carpark_donut", "Donut in the car park", "a donut in the car park (practice counts)", 0 },
    { "off_track", "Off the beaten track", "more than 1 km off the road in one run", 0 },
    { "yodel", "Drift Yodel", "set off Drift Yodel mode", 0 },
    { "full_yodel", "Full Yodel", "keep one pot going through the whole minute of Drift Yodel, then bank it", 0 },
    { "fording", "Fording", "cross Solomons Creek ford sideways, 30 degrees or more", 0 },
    { "deer_1", "Deer hunter", "find a deer", 1 },
    { "deer_10", "Deer stalker", "find 10 deer", 1 },
    { "deer_20", "Herd", "find all 20 deer", 1 },
    { "bridge", "Down the mountain", "reach the bridge", 1 },
    { "no_brakes", "No brakes, no worries", "reach the bridge without touching the brake", 0 },
    { "big_pot", "Big pot", "bank a single pot over 20,000", 0 },
    { "clean", "Hut to bridge, clean", "reach the bridge without going in the ditch", 0 },
    { "junction_donut", "Cattlemans corner", "a donut at the Cattlemans junction", 0 },
    { "army_man", "Toy soldier", "find the army man in the bush", 0 },
    { "gone_woodin", "GONE WOODIN'", "fill the tray: 15 logs in one run", 0 },
};

/* ------------------------------------------------------------------ the deer */

static uint32_t hash32(uint32_t x) { x ^= x >> 16; x *= 0x7feb352dU; x ^= x >> 15; x *= 0x846ca68bU; x ^= x >> 16; return x; }
static float rnd(uint32_t *s) { *s = hash32(*s + 0x9e3779b9U); return (float)(*s >> 8) / 16777216.0f; }

/* Where along the road each deer goes and whether it is easy to see from the road (V) or properly hidden (H):
 * a few round the hut, most on the plateau and the descent, a couple at the ford, two near the bridge. */
static const struct { float s0, s1; int hidden; } SLOT[WORLD_DEER] = {
    { 20, 200, 0 }, { 60, 350, 1 }, { 150, 420, 1 },
    { 500, 900, 0 }, { 900, 1400, 1 }, { 1400, 1900, 1 }, { 1900, 2300, 0 }, { 2300, 2900, 1 }, { 2900, 3500, 1 }, { 3500, 4100, 0 },
    { 4200, 4600, 1 }, { 4600, 5000, 0 }, { 5000, 5400, 1 }, { 5400, 5800, 1 }, { 5800, 6150, 0 }, { 6150, 6450, 1 },
    { 6450, 6610, 0 }, { 6645, 6800, 1 },
    { 6900, 7300, 1 }, { 7300, 7740, 1 },
};

static int solid_near(const Route *R, float x, float z, float r) { return route_tree_hit(R, x, z, 0, r, r, -r) >= 0; }

/* Soft plants a metre or more tall within r: the cover a hidden deer stands behind. */
static int cover_near(const Route *R, float x, float z, float s, float r)
{
    int c = (int)(s / (RT_DS * RT_CHUNK)), n = 0;
    for (int ch = c - 1; ch <= c + 1; ch++) {
        if (ch < 0 || ch >= R->nchunks) continue;
        for (int k = R->tree_first[ch]; k < R->tree_first[ch + 1]; k++) {
            const RtTree *t = &R->tree[k];
            if (t->t0 < 0 || (t->kind != 2 && t->kind != 0) || t->h < 0.8f) continue;
            if (hypotf(t->x - x, t->z - z) < r) n++;
        }
    }
    return n;
}

/* Can the ute get from the road to the deer (and back) without touching a trunk, a log or the edge, or a slope over 28%? A search on a 1 m grid
 * round both, every blocked cell being one a 2.2 m box (about the ute's width) would touch. */
static int reachable(const Route *R, float x0, float z0, float x1, float z1)
{
    float lo_x = fminf(x0, x1) - 14, lo_z = fminf(z0, z1) - 14, hi_x = fmaxf(x0, x1) + 14, hi_z = fmaxf(z0, z1) + 14;
    int nx = (int)(hi_x - lo_x) + 1, nz = (int)(hi_z - lo_z) + 1;
    if (nx * nz > 200000) return 0;
    unsigned char *st = calloc((size_t)nx * nz, 1);   /* 0 unknown, 1 open, 2 blocked, 3 seen */
    int *q = malloc(sizeof(int) * (size_t)nx * nz), qh = 0, qt = 0, ok = 0;
    int si = (int)(x0 - lo_x), sj = (int)(z0 - lo_z);
    q[qt++] = sj * nx + si; st[sj * nx + si] = 3;
    while (qh < qt && !ok) {
        int k = q[qh++], i = k % nx, j = k / nx;
        float x = lo_x + (float)i + 0.5f, z = lo_z + (float)j + 0.5f;
        if (hypotf(x - x1, z - z1) < 2.0f) { ok = 1; break; }
        const int di[4] = { 1, -1, 0, 0 }, dj[4] = { 0, 0, 1, -1 };
        for (int d = 0; d < 4; d++) {
            int a = i + di[d], b = j + dj[d];
            if (a < 0 || b < 0 || a >= nx || b >= nz) continue;
            int m = b * nx + a;
            if (st[m]) continue;
            float xx = lo_x + (float)a + 0.5f, zz = lo_z + (float)b + 0.5f;
            float gx = route_height(R, xx + 1.0f, zz, NULL) - route_height(R, xx - 1.0f, zz, NULL), gz = route_height(R, xx, zz + 1.0f, NULL) - route_height(R, xx, zz - 1.0f, NULL);
            int blocked = route_tree_hit(R, xx, zz, 0, 1.1f, 1.1f, -1.1f) >= 0 || route_edge_at(R, xx, zz) > T.edge_stop - 1.0f
                          || hypotf(gx, gz) * 0.5f > 0.28f;   /* steeper than 28%: the ute won't climb back out */
            st[m] = blocked ? 2 : 3;
            if (!blocked) q[qt++] = m;
        }
    }
    free(st); free(q);
    return ok;
}

int world_place_deer(const Route *R, uint32_t seed)
{
    W.ndeer = 0;
    for (int d = 0; d < WORLD_DEER; d++) {
        uint32_t sd = hash32(seed * 7919u + (uint32_t)d * 104729u + 17u);
        int placed = 0;
        for (int tries = 0; tries < 1500 && !placed; tries++) {
            int strict = tries < 600;   /* the first tries insist on cover (hidden) or a clear view (easy) */
            float s = SLOT[d].s0 + (SLOT[d].s1 - SLOT[d].s0) * rnd(&sd);
            float side = rnd(&sd) < 0.5f ? -1.0f : 1.0f;
            float lat = SLOT[d].hidden ? 20.0f + 22.0f * rnd(&sd) : 9.0f + 7.0f * rnd(&sd);
            float rx, rz; route_point(R, s, &rx, &rz);
            float hd = route_heading_at(R, s), x = rx - sinf(hd) * lat * side, z = rz + cosf(hd) * lat * side;
            RtNear n = route_near(R, x, z);
            if (fabsf(n.s - s) > 30.0f || fabsf(n.lat) < 8.0f) continue;            /* nearer another stretch of road, or on it */
            if (route_edge(R, n, x, z) > -4.0f) continue;                           /* well inside the edge */
            if (R->ford_s > 0 && fabsf(n.s - R->ford_s) < 16.0f && fabsf(n.lat) < 14.0f) continue;   /* not standing in the creek */
            if (R->bridge_s0 > 0 && n.s > R->bridge_s0 - 20.0f) continue;
            float h0 = route_height(R, x - 1.5f, z, NULL), h1 = route_height(R, x + 1.5f, z, NULL), h2 = route_height(R, x, z - 1.5f, NULL), h3 = route_height(R, x, z + 1.5f, NULL);
            if (hypotf(h1 - h0, h3 - h2) / 3.0f > 0.22f) continue;                  /* not on a steep face */
            if (solid_near(R, x, z, 2.6f)) continue;
            int crowd = 0;
            for (int e = 0; e < W.ndeer; e++) if (hypotf(W.deer[e].x - x, W.deer[e].z - z) < 25.0f) crowd = 1;
            if (crowd) continue;
            if (strict && SLOT[d].hidden && cover_near(R, x, z, n.s, 4.5f) < 3) continue;
            if (strict && !SLOT[d].hidden) {   /* nothing solid between the road and it */
                int blocked = 0;
                for (float f = 0.15f; f < 0.9f && !blocked; f += 0.1f) blocked = solid_near(R, rx + (x - rx) * f, rz + (z - rz) * f, 0.6f);
                if (blocked) continue;
            }
            if (!reachable(R, rx, rz, x, z)) continue;
            Deer *dd = &W.deer[W.ndeer++];
            dd->x = x; dd->z = z; dd->s = n.s; dd->lat = n.lat; dd->hidden = SLOT[d].hidden;
            dd->variant = (int)(rnd(&sd) * 4.0f) & 3;
            dd->yaw = SLOT[d].hidden ? rnd(&sd) * 6.2831853f : -hd + (side > 0 ? 0.0f : 3.14159f) + (rnd(&sd) - 0.5f) * 1.2f;   /* the easy ones stand side-on to the road */
            placed = 1;
        }
        if (!placed) fprintf(stderr, "world: no spot for deer %d (km %.2f to %.2f)\n", d + 1, SLOT[d].s0 / 1000, SLOT[d].s1 / 1000);
    }
    /* the army man: 9 to 13 m from a hidden deer on the plateau, further from the road than it, aiming at it */
    W.junction = R->spur_n > 1; W.jx = R->jx; W.jz = R->jz;
    W.army_placed = 0;
    uint32_t sd = hash32(seed * 31u + 991u);
    for (int tries = 0; tries < 3000 && !W.army_placed; tries++) {
        int d = tries < 1500 ? 8 : (int)(rnd(&sd) * (float)W.ndeer);   /* deer 9 (km 3.2, behind scrub) first */
        if (d >= W.ndeer) continue;
        const Deer *dr = &W.deer[d];
        float a = rnd(&sd) * 6.2831853f, r = 9.0f + 4.0f * rnd(&sd), x = dr->x + cosf(a) * r, z = dr->z + sinf(a) * r;
        RtNear n = route_near(R, x, z);
        if (fabsf(n.lat) < fabsf(dr->lat) + 2.0f || fabsf(n.s - dr->s) > 40.0f) continue;   /* deeper in the bush than the deer */
        if (route_edge(R, n, x, z) > -4.0f || solid_near(R, x, z, 2.2f)) continue;
        float h0 = route_height(R, x - 1.5f, z, NULL), h1 = route_height(R, x + 1.5f, z, NULL), h2 = route_height(R, x, z - 1.5f, NULL), h3 = route_height(R, x, z + 1.5f, NULL);
        if (hypotf(h1 - h0, h3 - h2) / 3.0f > 0.22f) continue;
        int clear = 1;   /* a line of sight to the deer: no trunk in the way */
        for (float f = 0.15f; f < 0.85f && clear; f += 0.1f) clear = !solid_near(R, x + (dr->x - x) * f, z + (dr->z - z) * f, 0.5f);
        if (!clear) continue;
        float rx, rz; route_point(R, n.s, &rx, &rz);
        if (!reachable(R, rx, rz, x, z)) continue;
        W.army_x = x; W.army_z = z; W.army_deer = d; W.army_placed = 1;
        W.army_yaw = atan2f(dr->z - z, dr->x - x);   /* the model aims along its x: turned to the deer */
    }
    if (!W.army_placed) fprintf(stderr, "world: no spot for the army man\n");
    return W.ndeer;
}

int world_army_hit(float cx, float cz, float yaw, float half_w, float front, float rear)
{
    if (!W.army_placed || W.army_found) return 0;
    float fx = sinf(yaw), fz = cosf(yaw), rx = cosf(yaw), rz = -sinf(yaw), dx = W.army_x - cx, dz = W.army_z - cz;
    float lf = dx * fx + dz * fz, lr = dx * rx + dz * rz;
    float of = lf > front ? lf - front : (lf < rear ? rear - lf : 0), orr = fmaxf(fabsf(lr) - half_w, 0);
    if (of * of + orr * orr >= 0.9f * 0.9f) return 0;
    W.army_found = 1; W.dirty = 1;
    world_earn(ACH_ARMY_MAN);
    return 1;
}

int world_found_count(void)
{
    int n = 0;
    for (int d = 0; d < W.ndeer; d++) n += (W.found >> d) & 1;
    return n;
}

int world_deer_hit(float cx, float cz, float yaw, float half_w, float front, float rear)
{
    float fx = sinf(yaw), fz = cosf(yaw), rx = cosf(yaw), rz = -sinf(yaw);
    for (int d = 0; d < W.ndeer; d++) {
        if ((W.found >> d) & 1) continue;
        float dx = W.deer[d].x - cx, dz = W.deer[d].z - cz;
        if (dx * dx + dz * dz > 100.0f) continue;
        float lf = dx * fx + dz * fz, lr = dx * rx + dz * rz;
        float of = lf > front ? lf - front : (lf < rear ? rear - lf : 0), orr = fmaxf(fabsf(lr) - half_w, 0);
        if (of * of + orr * orr < 1.1f * 1.1f) {   /* the deer's body, about a metre round its middle */
            W.found |= 1u << d; W.run_found++; W.dirty = 1;
            int n = world_found_count();
            if (n >= 1) world_earn(ACH_DEER_1);
            if (n >= 10) world_earn(ACH_DEER_10);
            if (n >= WORLD_DEER) world_earn(ACH_DEER_20);
            return d;
        }
    }
    return -1;
}

/* ------------------------------------------------------------- achievements */

void world_earn(int a)
{
    if (a < 0 || a >= ACH_N || W.earned[a]) return;
    W.earned[a] = 1; records_today(W.date[a]); W.dirty = 1;
    if (W.nnew < ACH_N) W.newly[W.nnew++] = a;
}

int world_pop(void)
{
    if (!W.nnew) return -1;
    int a = W.newly[0];
    memmove(W.newly, W.newly + 1, sizeof(int) * (size_t)(--W.nnew));
    return a;
}

void world_run_start(void)
{
    W.run_found = 0; W.braked = 0; W.ditched = 0; W.yodel_ok = 0; W.yodel_armed = 0; W.yodel_t = 0; W.prev_s = -1e9f;
}

void world_step(const WorldStep *st)
{
    static int yodel_was;
    if (st->running) {
        if (st->brake) W.braked = 1;
        if (st->ditch_rule) W.ditched = 1;
        if (st->offroad_m > 1000.0f) world_earn(ACH_OFF_TRACK);
        if (st->wood >= (int)T.wood_full) world_earn(ACH_GONE_WOODIN);
        if (st->donut_carpark) world_earn(ACH_CARPARK_DONUT);
        if (st->donut_road && W.junction && hypotf(st->x - W.jx, st->z - W.jz) < 18.0f) world_earn(ACH_JUNCTION_DONUT);
        if (W.yodel_armed) {   /* a pot that lasted a whole minute of the mode: banked later (even in the next mode) it counts */
            if (st->banked) { world_earn(ACH_FULL_YODEL); W.yodel_armed = 0; }
            else if (st->lost) W.yodel_armed = 0;
        }
        if (st->yodel_started) { world_earn(ACH_YODEL); W.yodel_ok = 1; W.yodel_t = 0; }
        if (st->yodel_on) {   /* Full Yodel: a pot from the first 5 s of the mode, never banked or lost till it ends */
            W.yodel_t += st->dt;
            if (st->lost || st->banked || (W.yodel_t > 5.0f && st->pot <= 0)) W.yodel_ok = 0;
        } else if (yodel_was) {
            W.yodel_armed = W.yodel_ok && W.yodel_t >= 59.0f && st->pot > 0;
            W.yodel_ok = 0;
        }
        if (st->ford_s > 0 && st->on_road && W.prev_s < st->ford_s && st->s >= st->ford_s && st->s - W.prev_s < 5.0f && st->drift_deg >= 30.0f) world_earn(ACH_FORDING);
        W.prev_s = st->s;
        if (st->banked && st->banked_value > 20000.0f) world_earn(ACH_BIG_POT);
    }
    if (st->finished) {
        if (W.yodel_armed && st->banked) world_earn(ACH_FULL_YODEL);
        if (st->banked && st->banked_value > 20000.0f) world_earn(ACH_BIG_POT);
        world_earn(ACH_BRIDGE);
        if (!W.braked) world_earn(ACH_NO_BRAKES);
        if (!W.ditched) world_earn(ACH_CLEAN);
    }
    yodel_was = st->yodel_on && st->running;
}

/* ------------------------------------------------------------- the progress file */
/* bush-drift progress 1 / deer <one 0 or 1 per deer> / ach <key> <date> ... / end. Written whole (records.c). */

static int read_progress(const char *path)
{
    FILE *f = fopen(path, "r");
    if (!f) return 0;
    char line[200], key[64], date[32];
    int head = 0, end = 0, army = 0; uint32_t found = 0; int earned[ACH_N] = { 0 }; char dates[ACH_N][12];
    memset(dates, 0, sizeof(dates));
    while (fgets(line, sizeof(line), f)) {
        if (!head) { if (strncmp(line, "bush-drift progress 1", 21)) break; head = 1; continue; }
        if (!strncmp(line, "deer ", 5)) {
            for (int d = 0; d < WORLD_DEER && line[5 + d] && line[5 + d] != '\n'; d++) if (line[5 + d] == '1') found |= 1u << d;
        } else if (!strncmp(line, "army 1", 6)) { army = 1;
        } else if (sscanf(line, "ach %63s %31s", key, date) == 2) {
            for (int a = 0; a < ACH_N; a++) if (!strcmp(key, ACH[a].key)) { earned[a] = 1; records_set_date(dates[a], date); }
        } else if (!strncmp(line, "end", 3)) { end = 1; break; }
    }
    fclose(f);
    if (!head || !end) return 0;
    W.found = found; W.army_found = army;
    memcpy(W.earned, earned, sizeof(earned)); memcpy(W.date, dates, sizeof(dates));
    return 1;
}

void world_load(const char *path)
{
    snprintf(path_, sizeof(path_), "%s", path);
    W.found = 0; W.army_found = 0; memset(W.earned, 0, sizeof(W.earned)); memset(W.date, 0, sizeof(W.date)); W.nnew = 0; W.dirty = 0;
    if (read_progress(path)) return;
    char tmp[320]; snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    if (read_progress(tmp)) fprintf(stderr, "progress: %s unreadable, finished save used\n", path);
}

void world_save(void)
{
    if (!path_[0]) return;
    char text[2048]; int k = snprintf(text, sizeof(text), "bush-drift progress 1\ndeer ");
    for (int d = 0; d < WORLD_DEER; d++) text[k++] = (W.found >> d) & 1 ? '1' : '0';
    text[k++] = '\n';
    if (W.army_found) k += snprintf(text + k, sizeof(text) - (size_t)k, "army 1\n");
    for (int a = 0; a < ACH_N; a++) if (W.earned[a]) k += snprintf(text + k, sizeof(text) - (size_t)k, "ach %s %s\n", ACH[a].key, W.date[a][0] ? W.date[a] : "-");
    snprintf(text + k, sizeof(text) - (size_t)k, "end\n");
    if (records_write_whole(path_, text)) W.dirty = 0;
    else fprintf(stderr, "progress: could not save %s\n", path_);
}

/* --------------------------------------------------------------- selftest */

static int check(int ok, const char *what) { printf("  %s  %s\n", ok ? "ok  " : "FAIL", what); return ok; }

int world_selftest(const char *dir)
{
    char path[300]; snprintf(path, sizeof(path), "%s/progress-test.txt", dir);
    remove(path);
    int ok = 1;
    World keep = W;
    world_load(path);
    ok &= check(W.found == 0 && !W.earned[ACH_BRIDGE], "no file: nothing found, nothing earned");
    W.ndeer = WORLD_DEER;
    for (int d = 0; d < WORLD_DEER; d++) { W.deer[d].x = (float)d * 100.0f; W.deer[d].z = 0; }
    world_run_start();
    ok &= check(world_deer_hit(3.0f, 0, 0, 1.0f, 2.5f, -2.5f) == -1, "a deer 3 m to the side of the outline is not found");
    ok &= check(world_deer_hit(2.0f, 0, 0, 1.0f, 2.5f, -2.5f) == 0 && W.earned[ACH_DEER_1] && world_pop() == ACH_DEER_1, "one touched: found, Deer hunter earned and shown");
    ok &= check(world_deer_hit(2.0f, 0, 0, 1.0f, 2.5f, -2.5f) == -1, "a found deer can't be found twice");
    for (int d = 1; d < 9; d++) world_deer_hit((float)d * 100.0f, 0, 0, 1.0f, 2.5f, -2.5f);
    ok &= check(world_found_count() == 9 && !W.earned[ACH_DEER_10], "nine: no Deer stalker yet");
    world_deer_hit(900.0f, 0, 0, 1.0f, 2.5f, -2.5f);
    ok &= check(W.earned[ACH_DEER_10] && !W.earned[ACH_DEER_20] && W.run_found == 10, "ten: Deer stalker");
    for (int d = 10; d < WORLD_DEER; d++) world_deer_hit((float)d * 100.0f, 0, 0, 1.0f, 2.5f, -2.5f);
    ok &= check(W.earned[ACH_DEER_20], "twenty: Herd");
    /* a run, step by step */
    WorldStep st; memset(&st, 0, sizeof(st)); st.dt = 0.1f; st.running = 1; st.ford_s = 500; st.on_road = 1;
    world_run_start();
    st.s = 498; world_step(&st); st.s = 501; st.drift_deg = 20; world_step(&st);
    ok &= check(!W.earned[ACH_FORDING], "the ford at 20 degrees: no Fording");
    world_run_start(); st.s = 498; world_step(&st); st.s = 501; st.drift_deg = 33; world_step(&st);
    ok &= check(W.earned[ACH_FORDING], "the ford at 33 degrees: Fording");
    st.drift_deg = 0;
    st.offroad_m = 999; world_step(&st); ok &= check(!W.earned[ACH_OFF_TRACK], "999 m off the road: not yet");
    st.offroad_m = 1001; world_step(&st); ok &= check(W.earned[ACH_OFF_TRACK], "1001 m: Off the beaten track");
    st.offroad_m = 0;
    st.banked = 1; st.banked_value = 19999; world_step(&st); ok &= check(!W.earned[ACH_BIG_POT], "a 19,999 pot: no Big pot");
    st.banked_value = 20001; world_step(&st); ok &= check(W.earned[ACH_BIG_POT], "20,001: Big pot");
    st.banked = 0; st.banked_value = 0;
    st.donut_road = 1; world_step(&st); ok &= check(!W.earned[ACH_CARPARK_DONUT], "a donut on the road: not the car park one");
    st.donut_road = 0; st.donut_carpark = 1; world_step(&st); ok &= check(W.earned[ACH_CARPARK_DONUT], "a donut in the car park");
    st.donut_carpark = 0;
    /* Full Yodel: broken by a bank in the middle, then done properly */
    world_run_start();
    st.yodel_started = 1; st.yodel_on = 1; st.pot = 100; world_step(&st); st.yodel_started = 0;
    ok &= check(W.earned[ACH_YODEL], "Drift Yodel");
    for (int i = 0; i < 300; i++) world_step(&st);
    st.banked = 1; world_step(&st); st.banked = 0; st.pot = 50;
    for (int i = 0; i < 300; i++) world_step(&st);
    st.yodel_on = 0; world_step(&st); st.banked = 1; world_step(&st); st.banked = 0;
    ok &= check(!W.earned[ACH_FULL_YODEL], "banked half way through the minute: no Full Yodel");
    st.yodel_started = 1; st.yodel_on = 1; st.pot = 10; world_step(&st); st.yodel_started = 0;
    for (int i = 0; i < 600; i++) { st.pot += 10; world_step(&st); }
    st.yodel_on = 0; world_step(&st);
    ok &= check(!W.earned[ACH_FULL_YODEL] && W.yodel_armed, "the minute through with the pot going: armed, not yet banked");
    st.banked = 1; st.banked_value = st.pot; world_step(&st); st.banked = 0;
    ok &= check(W.earned[ACH_FULL_YODEL], "then banked: Full Yodel");
    /* the bridge */
    world_run_start(); memset(&st, 0, sizeof(st)); st.dt = 0.1f; st.running = 1;
    st.brake = 1; world_step(&st); st.brake = 0; st.running = 0; st.finished = 1; world_step(&st);
    ok &= check(W.earned[ACH_BRIDGE] && !W.earned[ACH_NO_BRAKES] && W.earned[ACH_CLEAN], "the bridge, braked once, never in the ditch: Down the mountain and clean, not No brakes");
    W.earned[ACH_CLEAN] = 0;
    world_run_start(); st.running = 1; st.finished = 0; st.ditch_rule = 1; world_step(&st); st.ditch_rule = 0; st.running = 0; st.finished = 1; world_step(&st);
    ok &= check(W.earned[ACH_NO_BRAKES] && !W.earned[ACH_CLEAN], "no brakes but the ditch once: No brakes, not clean");
    /* the junction donut, and the army man */
    W.junction = 1; W.jx = 100; W.jz = 100;
    world_run_start(); memset(&st, 0, sizeof(st)); st.dt = 0.1f; st.running = 1;
    st.donut_road = 1; st.x = 130; st.z = 100; world_step(&st);
    ok &= check(!W.earned[ACH_JUNCTION_DONUT], "a donut 30 m from the junction: not Cattlemans corner");
    st.x = 110; world_step(&st); st.donut_road = 0;
    ok &= check(W.earned[ACH_JUNCTION_DONUT], "a donut 10 m from it: Cattlemans corner");
    W.army_placed = 1; W.army_found = 0; W.army_x = 50; W.army_z = 0;
    ok &= check(!world_army_hit(52.5f, 0, 0, 1.0f, 2.5f, -2.5f) && world_army_hit(51.5f, 0, 0, 1.0f, 2.5f, -2.5f) && W.earned[ACH_ARMY_MAN], "the army man: missed at 1.5 m, found touching, Toy soldier");
    world_run_start(); memset(&st, 0, sizeof(st)); st.dt = 0.1f; st.running = 1;
    st.wood = 14; world_step(&st); ok &= check(!W.earned[ACH_GONE_WOODIN], "14 logs: the tray isn't full");
    st.wood = 15; world_step(&st); ok &= check(W.earned[ACH_GONE_WOODIN], "15 logs: GONE WOODIN'");
    /* saved and read back */
    uint32_t f0 = W.found; int e0[ACH_N]; memcpy(e0, W.earned, sizeof(e0));
    world_save();
    W.found = 0; W.army_found = 0; memset(W.earned, 0, sizeof(W.earned));
    world_load(path);
    ok &= check(W.found == f0 && !memcmp(e0, W.earned, sizeof(e0)) && W.army_found == 1, "saved and read back the same (the army man too)");
    FILE *f = fopen(path, "w"); fprintf(f, "bush-drift progress 1\ndeer 1111"); fclose(f);
    world_load(path);
    ok &= check(W.found == 0, "a damaged file reads as nothing (not as part of one)");
    remove(path);
    W = keep;
    printf(ok ? "worldtest finished\n" : "worldtest FAILED\n");
    return ok ? 0 : 1;
}
