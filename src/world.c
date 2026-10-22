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
    { "carpark_donut", "Donut in the car park", "a donut in the car park (practice counts)", 0, IN_BOTH, "Round and round, parked", SEC_DRIFT },
    { "off_track", "Off the beaten track", "find the secret way, and drive it from the hut down to the ford", 0, IN_EXPLORE, "Follow the water", SEC_CREEK },
    { "yodel", "Drift Yodel", "set off Drift Yodel mode", 0, IN_BOTH, "Spin to win", SEC_DRIFT },
    { "full_yodel", "Full Yodel", "keep one pot going through the whole minute of Drift Yodel, then bank it", 0, IN_DRIFT, "Hold it for the whole song", SEC_DRIFT },
    { "fording", "Fording", "cross Solomons Creek ford sideways, 30 degrees or more", 0, IN_BOTH, "Wet, and sideways", SEC_DRIFT },
    { "deer_1", "Deer hunter", "find a deer", 1, IN_BOTH, NULL, SEC_BUSH },
    { "deer_10", "Deer stalker", "find 10 deer", 1, IN_BOTH, NULL, SEC_BUSH },
    { "deer_20", "Herd", "find all 20 deer", 1, IN_BOTH, NULL, SEC_BUSH },
    { "bridge", "Down the mountain", "reach the bridge", 1, IN_DRIFT, NULL, SEC_DRIFT },
    { "no_brakes", "No brakes, no worries", "reach the bridge without touching the brake", 0, IN_DRIFT, "Who needs a pedal?", SEC_DRIFT },
    { "big_pot", "Big pot", "bank a single pot over 20,000", 0, IN_DRIFT, "Greed is good", SEC_DRIFT },
    { "clean", "Hut to bridge, clean", "reach the bridge without going in the ditch", 0, IN_DRIFT, "Keep it between the ditches", SEC_DRIFT },
    { "junction_donut", "Cattlemans corner", "a donut at the Cattlemans junction", 0, IN_BOTH, "A spin at the junction", SEC_BUSH },
    { "army_man", "Toy soldier", "find Quayle in the bush", 0, IN_BOTH, "Someone's watching the deer", SEC_QUAYLE },
    { "gone_woodin", "GONE WOODIN'", "fill the tray: 15 logs", 0, IN_BOTH, "Fill her up", SEC_FIREWOOD },
    { "fire_1", "Bush telly", "light a fire at the hut", 0, IN_EXPLORE, "Something to watch tonight", SEC_HUT },
    { "fireworks", "Cracker night", "build the fire big enough for the fireworks", 0, IN_EXPLORE, "The chimney has a secret", SEC_HUT },
    { "charcuterie", "Charcuterie", "lay out the board on the picnic table", 0, IN_EXPLORE, "Something for the table", SEC_HUT },
    { "hut_night", "A night at the Hut", "the big fire, the fireworks and the board", 0, IN_EXPLORE, "The perfect evening", SEC_HUT },
    { "back_on_duty", "Back on duty", "find Quayle's rifle and give it back to him", 0, IN_EXPLORE, "He's lost something", SEC_QUAYLE },
    { "camp_oven", "Camp oven", "venison stew on a good fire", 0, IN_EXPLORE, "What's for dinner?", SEC_QUAYLE },
    { "stock_route", "Droving days", "follow the old stock route's ten markers down to its camp", 0, IN_EXPLORE, "Old cattlemen passed this way", SEC_CREEK },
    { "razor", "Razor sharp", "a razor-sharp result filing Will's chainsaw", 0, IN_EXPLORE, "A file and a steady hand", SEC_FIREWOOD },
    { "clean_cuts", "Clean cuts", "10 logs in a row, each cut in one pass", 0, IN_EXPLORE, "One clean hit", SEC_FIREWOOD },
    { "midnight", "Midnight run", "reach the bridge at night", 0, IN_EXPLORE, "After dark, all the way down", SEC_BUSH },
    { "spotlight", "Spotlighting", "find a deer at night, by headlight", 0, IN_EXPLORE, "Eyes in the dark", SEC_BUSH },
    { "wet_feet", "Wet feet", "drive 1 km of the creek way", 0, IN_EXPLORE, "A long way down the water", SEC_CREEK },
    { "low_slow", "Low and slow", "climb back up the waterfall in Low Range", 0, IN_EXPLORE, "Up where the water comes down", SEC_CREEK },
    { "mechanic", "Bush mechanic", "sharpen the saw three times in one day", 0, IN_EXPLORE, "Keep that chain keen", SEC_FIREWOOD },
    { "trials", "Creek Trials", "bank 5,000 Creek Trials points in one run", 0, IN_EXPLORE, "Clean lines score", SEC_CREEK },
    { "clean_run", "Not a scratch", "the creek way from the hut to the ford with no damage at all", 0, IN_EXPLORE, "Gentle on the ute", SEC_CREEK },
    { "tyre", "Tyre change", "change a flat tyre in the bush", 0, IN_EXPLORE, "Jack, nuts, spare", SEC_CREEK },
    { "full_house", "Full house", "every other achievement", 1, IN_BOTH, NULL, SEC_BUSH },
};

const char *const HAMPER_NAME[HAMPER_N] = { "brie", "red-wax cheese", "cheddar", "chorizo", "green olives", "stuffed jalapenos",
                                            "red peppers", "ZZZZEASONING!!!", "whipped feta", "corn chips", "baguette", "two reds" };

/* ------------------------------------------------------------------ the deer */

static uint32_t hash32(uint32_t x) { x ^= x >> 16; x *= 0x7feb352dU; x ^= x >> 15; x *= 0x846ca68bU; x ^= x >> 16; return x; }
static float rnd(uint32_t *s) { *s = hash32(*s + 0x9e3779b9U); return (float)(*s >> 8) / 16777216.0f; }

/* Where along the road each deer goes and whether it is easy to see from the road (V) or properly hidden (H):
 * a few round the hut, most on the plateau and the descent, a couple at the ford, two near the bridge. */
static const struct { float s0, s1; int hidden; } SLOT[WORLD_DEER + WORLD_EXTRA] = {
    { 20, 200, 0 }, { 60, 350, 1 }, { 150, 420, 1 },
    { 500, 900, 0 }, { 900, 1400, 1 }, { 1400, 1900, 1 }, { 1900, 2300, 0 }, { 2300, 2900, 1 }, { 2900, 3500, 1 }, { 3500, 4100, 0 },
    { 4200, 4600, 1 }, { 4600, 5000, 0 }, { 5000, 5400, 1 }, { 5400, 5800, 1 }, { 5800, 6150, 0 }, { 6150, 6450, 1 },
    { 6450, 6610, 0 }, { 6645, 6800, 1 },
    { 6900, 7300, 1 }, { 7300, 7740, 1 },
    /* build C follow-up: the five Quayle spots once he's back on duty, near the road and easy to see */
    { 1000, 1500, 0 }, { 2400, 2900, 0 }, { 3700, 4200, 0 }, { 5200, 5700, 0 }, { 6900, 7500, 0 },
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
    W.ndeer = 0; W.nextra = 0;
    for (int d = 0; d < WORLD_DEER + WORLD_EXTRA; d++) {
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
            for (int e = 0; e < W.nextra; e++) if (hypotf(W.deer[WORLD_DEER + e].x - x, W.deer[WORLD_DEER + e].z - z) < 25.0f) crowd = 1;
            if (crowd) continue;
            if (strict && SLOT[d].hidden && cover_near(R, x, z, n.s, 4.5f) < 3) continue;
            if (strict && !SLOT[d].hidden) {   /* nothing solid between the road and it */
                int blocked = 0;
                for (float f = 0.15f; f < 0.9f && !blocked; f += 0.1f) blocked = solid_near(R, rx + (x - rx) * f, rz + (z - rz) * f, 0.6f);
                if (blocked) continue;
            }
            if (!reachable(R, rx, rz, x, z)) continue;
            Deer *dd = d < WORLD_DEER ? &W.deer[W.ndeer++] : &W.deer[WORLD_DEER + W.nextra++];
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

int world_extra_on(void) { return W.quayle >= 1; }
int world_deer_hit(float cx, float cz, float yaw, float half_w, float front, float rear)
{
    float fx = sinf(yaw), fz = cosf(yaw), rx = cosf(yaw), rz = -sinf(yaw);
    for (int dd = 0; dd < W.ndeer + (world_extra_on() ? W.nextra : 0); dd++) {
        int d = dd < W.ndeer ? dd : WORLD_DEER + (dd - W.ndeer);
        if ((W.found >> d) & 1) continue;
        float dx = W.deer[d].x - cx, dz = W.deer[d].z - cz;
        if (dx * dx + dz * dz > 100.0f) continue;
        float lf = dx * fx + dz * fz, lr = dx * rx + dz * rz;
        float of = lf > front ? lf - front : (lf < rear ? rear - lf : 0), orr = fmaxf(fabsf(lr) - half_w, 0);
        if (of * of + orr * orr < 1.1f * 1.1f) {   /* the deer's body, about a metre round its middle */
            W.found |= 1u << d; W.run_found++; W.dirty = 1;
            if (W.night >= 0.8f) world_earn(ACH_SPOTLIGHT);   /* build E: by headlight (Explorer's night) */
            if (W.quayle == 1 && !W.tray_deer && !W.stew) { W.tray_deer = d + 1; W.quayle = 2; }   /* Quayle's shot: it goes in the tray, he gets in the cab */
            int n = world_found_count();
            if (n >= 1) world_earn(ACH_DEER_1);
            if (n >= 10) world_earn(ACH_DEER_10);
            if (n >= WORLD_DEER) world_earn(ACH_DEER_20);
            return d;
        }
    }
    return -1;
}

/* ------------------------------------------------------------- build C: Explorer */
/* The hamper hunt: twelve things hidden round the hut's clearing, at landmarks found in the route's own props and plants
 * (so they follow the hut if it moves): the fire ring, the toilet, the plough, the tanks, the old iron gate, the fence line,
 * snow gums out in the clearing, the creek, and two in long grass. Every spot is checked to be in the clearing, clear of
 * anything solid, and reachable from the car park round the trunks. */
static int find_prop(const Route *R, const char *name, float *x, float *z, float *yaw)
{
    int t = route_template_find(name, 0, 0);
    for (int k = 0; k < R->ntree && t >= 0; k++) if (R->tree[k].t0 == t) { *x = R->tree[k].x; *z = R->tree[k].z; if (yaw) *yaw = R->tree[k].yaw; return 1; }
    return 0;
}
static int spot_ok(const Route *R, float x, float z, int need_clearing)
{
    if (need_clearing && !route_in_clearing(R, x, z)) return 0;
    if (route_tree_hit(R, x, z, 0, 1.0f, 1.0f, -1.0f) >= 0) return 0;
    RtNear n = route_near(R, x, z);
    if (n.s > 0 && fabsf(n.lat) < route_half_width() + T.verge_w) return 0;   /* not on the gravel */
    return reachable(R, R->start_x, R->start_z, x, z);
}
/* From (x, z) out in rings until a spot passes (the first try is the spot itself). */
static int settle(const Route *R, float *x, float *z, int need_clearing)
{
    for (float r = 0; r < 12.0f; r += 0.75f)
        for (int a = 0; a < (r == 0 ? 1 : 12); a++) {
            float tx = *x + cosf((float)a * 0.5236f) * r, tz = *z + sinf((float)a * 0.5236f) * r;
            if (spot_ok(R, tx, tz, need_clearing)) { *x = tx; *z = tz; return 1; }
        }
    return 0;
}
/* The snow gum nearest a point (in the clearing): the spot goes on its far side from the hut, 2.4 m from the trunk. */
static int under_gum(const Route *R, float px, float pz, float *x, float *z)
{
    int t = -1; float best = 1e9f, sx = 0, sz = 0;
    for (int k = 0; k < R->ntree; k++) {
        const RtTree *g = &R->tree[k];
        if (g->kind != 0 || g->t0 < 0 || strcmp(route_template(g->t0)->name, "snowgum") || !route_in_clearing(R, g->x, g->z)) continue;
        float d = hypotf(g->x - px, g->z - pz);
        if (d < best) { best = d; t = k; sx = g->x; sz = g->z; }
    }
    if (t < 0) return 0;
    float ax = sx - R->hut_x, az = sz - R->hut_z, al = fmaxf(hypotf(ax, az), 1.0f);
    *x = sx + ax / al * 2.4f; *z = sz + az / al * 2.4f;
    return 1;
}

void world_place_explore(const Route *R)
{
    float hx = R->hut_x, hz = R->hut_z, px, pz, pyaw;
    /* the campfire: off the chimney gable (the hut's WNW end), on the grass (tuning fire_e, fire_n: metres from the hut) */
    W.fire_x = hx + T.fire_e; W.fire_z = hz + T.fire_n; W.hut_x = hx; W.hut_z = hz;
    W.picnic_x = hx - 10.5f; W.picnic_z = hz + 9.5f; W.picnic_yaw = 0.5f;
    if (find_prop(R, "picnic", &px, &pz, &pyaw)) { W.picnic_x = px; W.picnic_z = pz; W.picnic_yaw = pyaw; }
    struct { float x, z; int clearing; } want[HAMPER_N];
    for (int i = 0; i < HAMPER_N; i++) { want[i].x = hx; want[i].z = hz; want[i].clearing = 1; }
    /* easy: the brie by the fire ring, the baguette on the hitching rail end */
    if (find_prop(R, "firering", &px, &pz, NULL)) { want[HM_BRIE].x = px + 2.2f; want[HM_BRIE].z = pz + 1.4f; }
    if (find_prop(R, "rail", &px, &pz, &pyaw)) { RtNear n = route_near(R, px, pz); float hd = route_heading_at(R, n.s), sd = n.lat > 0 ? 1.0f : -1.0f; want[HM_BAGUETTE].x = px - sinf(hd) * sd * 1.6f + cosf(pyaw) * 1.8f; want[HM_BAGUETTE].z = pz + cosf(hd) * sd * 1.6f + sinf(pyaw) * 1.8f; }
    /* behind things, from the track */
    if (find_prop(R, "toilet", &px, &pz, NULL)) { want[HM_REDWAX].x = px - 2.6f; want[HM_REDWAX].z = pz + 0.6f; }
    if (find_prop(R, "plough", &px, &pz, NULL)) { RtNear n = route_near(R, px, pz); float hd = route_heading_at(R, n.s), sd = n.lat > 0 ? 1.0f : -1.0f; want[HM_CHEDDAR].x = px - sinf(hd) * sd * 2.6f; want[HM_CHEDDAR].z = pz + cosf(hd) * sd * 2.6f; }
    want[HM_OLIVES].x = hx - 7.9f; want[HM_OLIVES].z = hz - 3.4f;   /* behind the slim tank at the chimney end */
    if (find_prop(R, "gate", &px, &pz, &pyaw)) { RtNear n = route_near(R, px, pz); float hd = route_heading_at(R, n.s), sd = n.lat > 0 ? 1.0f : -1.0f; want[HM_FETA].x = px - sinf(hd) * sd * 2.2f; want[HM_FETA].z = pz + cosf(hd) * sd * 2.2f; }
    { float best = -1; int got = 0; float fx = 0, fz = 0;   /* the jalapenos by the fence line's far end */
      int tf = route_template_find("fencepost", 0, 0);
      for (int k = 0; k < R->ntree && tf >= 0; k++) { const RtTree *t = &R->tree[k]; if (t->t0 != tf || hypotf(t->x - hx, t->z - hz) > 80.0f) continue;
          float d = t->x - hx; if (d > best) { best = d; fx = t->x; fz = t->z; got = 1; } }
      if (got) { want[HM_JALAPENO].x = fx + 1.2f; want[HM_JALAPENO].z = fz - 1.8f; } }
    /* out in the clearing: under snow gums in the north arm and the east arm, by the creek, in long grass */
    if (under_gum(R, hx - 20.0f, hz + 210.0f, &px, &pz)) { want[HM_CHORIZO].x = px; want[HM_CHORIZO].z = pz; }
    if (under_gum(R, hx + 260.0f, hz - 40.0f, &px, &pz)) { want[HM_DIPS].x = px; want[HM_DIPS].z = pz; }
    want[HM_CHIPS].x = hx + 15.0f; want[HM_CHIPS].z = hz + 112.0f;     /* on the bank of Solomons Creek where it crosses the clearing */
    if (R->ck_n > 1) { float cs = R->ck_exit_s - 45.0f, cx, cz; route_creek_point(R, cs, &cx, &cz); RtNear cn = route_creek_near(R, hx, hz); float side = cn.lat > 0 ? 1.0f : -1.0f;
        int i = (int)(cs / RT_DS); float hd = atan2f(R->ck_z[i + 1] - R->ck_z[i], R->ck_x[i + 1] - R->ck_x[i]), off = side * (route_creek_bedw(R, cs) + 2.5f);
        want[HM_CHIPS].x = cx - sinf(hd) * off; want[HM_CHIPS].z = cz + cosf(hd) * off; }
    want[HM_PEPPERS].x = hx + 150.0f; want[HM_PEPPERS].z = hz - 150.0f; /* long grass, the south-east */
    want[HM_WINE].x = hx + 40.0f; want[HM_WINE].z = hz - 200.0f;       /* long grass, the far south */
    /* Quayle's rifle: down the creek way near its end, on a gravel bar at the edge of the water (Explorer only, like the way) */
    W.rifle_placed = 0;
    if (R->ck_n > 1) {
        float rs = R->ck_len - 380.0f, cx, cz; route_creek_point(R, rs, &cx, &cz);
        int i = (int)(rs / RT_DS); float hd = atan2f(R->ck_z[i + 1] - R->ck_z[i], R->ck_x[i + 1] - R->ck_x[i]), off = route_creek_bedw(R, rs) * 0.55f;   /* on the open gravel, between the water and the ferns */
        W.rifle_x = cx - sinf(hd) * off; W.rifle_z = cz + cosf(hd) * off; W.rifle_yaw = -hd + 0.4f; W.rifle_placed = 1;
    }
    for (int i = 0; i < HAMPER_N; i++) {
        HamperSpot *h = &W.item[i];
        h->x = want[i].x; h->z = want[i].z; h->placed = settle(R, &h->x, &h->z, want[i].clearing);
        h->yaw = (float)i * 2.39996f;
        if (!h->placed) fprintf(stderr, "world: no spot for the %s (wanted %.0f, %.0f m from the hut: clearing %d, solid %d, reachable %d)\n", HAMPER_NAME[i], want[i].x - hx, want[i].z - hz,
                                route_in_clearing(R, want[i].x, want[i].z), route_tree_hit(R, want[i].x, want[i].z, 0, 1.0f, 1.0f, -1.0f) >= 0, reachable(R, R->start_x, R->start_z, want[i].x, want[i].z));
        else if (getenv("WORLD_DEBUG")) fprintf(stderr, "world: the %s at %.1f, %.1f m from the hut (%.1f m from where it was wanted)\n", HAMPER_NAME[i], h->x - hx, h->z - hz, hypotf(h->x - want[i].x, h->z - want[i].z));
    }
}

static int touches(float px, float pz, float r, float cx, float cz, float yaw, float half_w, float front, float rear)
{
    float fx = sinf(yaw), fz = cosf(yaw), rx = cosf(yaw), rz = -sinf(yaw), dx = px - cx, dz = pz - cz;
    if (dx * dx + dz * dz > 100.0f) return 0;
    float lf = dx * fx + dz * fz, lr = dx * rx + dz * rz;
    float of = lf > front ? lf - front : (lf < rear ? rear - lf : 0), orr = fmaxf(fabsf(lr) - half_w, 0);
    return of * of + orr * orr < r * r;
}
int world_hamper_count(void) { int n = 0; for (int i = 0; i < HAMPER_N; i++) n += (W.hamper >> i) & 1; return n; }
int world_hamper_hit(float cx, float cz, float yaw, float half_w, float front, float rear)
{
    if (W.board) return -1;
    for (int i = 0; i < HAMPER_N; i++) {
        if (((W.hamper >> i) & 1) || !W.item[i].placed) continue;
        if (touches(W.item[i].x, W.item[i].z, 0.8f, cx, cz, yaw, half_w, front, rear)) { W.hamper |= 1u << i; W.dirty = 1; return i; }
    }
    return -1;
}
int world_rifle_hit(float cx, float cz, float yaw, float half_w, float front, float rear)
{
    if (W.rifle != 0 || !W.rifle_placed || !touches(W.rifle_x, W.rifle_z, 0.8f, cx, cz, yaw, half_w, front, rear)) return 0;
    W.rifle = 1; W.dirty = 1;
    return 1;
}
int world_rifle_return(float cx, float cz, float yaw, float half_w, float front, float rear)
{
    if (W.rifle != 1 || !W.army_placed || !touches(W.army_x, W.army_z, 1.6f, cx, cz, yaw, half_w, front, rear)) return 0;
    W.rifle = 2; W.quayle = 1; W.dirty = 1;   /* he hops in the tray; the extra deer are out */
    if (!W.army_found) { W.army_found = 1; world_earn(ACH_ARMY_MAN); }
    world_earn(ACH_BACK_ON_DUTY);
    return 1;
}
float world_fire_dist(float cx, float cz) { return hypotf(cx - W.fire_x, cz - W.fire_z); }
float world_table_dist(float cx, float cz) { return hypotf(cx - W.picnic_x, cz - W.picnic_z); }
float world_hut_dist(float cx, float cz) { return hypotf(cx - W.hut_x, cz - W.hut_z); }
/* (speed is no longer asked: driving in close is enough) */
int world_deliver(float cx, float cz, float speed)
{
    (void)speed;
    if (W.fire_level >= 3 || W.tray_wood < (int)T.wood_full || world_hut_dist(cx, cz) > T.hut_reach) return 0;
    W.tray_wood = 0; W.fire_level++; W.woodpile = W.fire_level * 15; W.dirty = 1;
    W.night = W.fire_level == 1 ? 0.5f : W.fire_level == 2 ? 0.9f : 1.0f;
    world_earn(ACH_FIRE_1);
    if (W.fire_level >= 3) world_earn(ACH_FIREWORKS);
    return W.fire_level;
}
int world_stew(float cx, float cz, float speed)
{
    (void)speed;
    if (!W.tray_deer || W.stew || world_hut_dist(cx, cz) > T.hut_reach) return 0;
    if (W.fire_level < 2) return 2;
    W.tray_deer = 0; W.stew = 1; W.dirty = 1;
    world_earn(ACH_CAMP_OVEN);
    return 1;
}
int world_board(float cx, float cz, float speed)
{
    (void)speed;
    if (W.board || world_hamper_count() < HAMPER_N || world_hut_dist(cx, cz) > T.hut_reach) return 0;
    W.board = 1; W.dirty = 1;
    world_earn(ACH_CHARCUTERIE);
    return 1;
}
void world_new_day(void)
{
    W.fire_level = 0; W.night = 0; W.tray_wood = 0; W.nlogs_gone = 0; W.woodpile = 0; W.dirty = 1;
    W.day++; W.trunk = 0; W.stag_down[0] = W.stag_down[1] = 0; W.stag_logs[0] = W.stag_logs[1] = 0; memset(W.wet_state, 0, sizeof(W.wet_state)); W.saw = 100; W.razor = 0; W.sharpened = 0; memset(W.tyre_wear, 0, sizeof(W.tyre_wear)); W.body_dmg = 0; W.punct = 0; W.spare = 1;   /* build D: the trunk shut again (more wood in it), the dead trees standing */
}

/* ---- build D: the old stock route down the creek way ---- */
void world_place_creek_d(const Route *R)
{
    W.nmark = R->ck_nmark < 12 ? R->ck_nmark : 12;
    for (int m = 0; m < W.nmark; m++) { W.mark_x[m] = R->ck_mark_x[m]; W.mark_z[m] = R->ck_mark_z[m]; W.mark_v[m] = R->ck_mark_v[m]; }
    W.camp_on = R->camp_on; W.camp_x = R->camp_x; W.camp_z = R->camp_z; W.trunk_x = R->trunk_x; W.trunk_z = R->trunk_z; W.trunk_yaw = R->trunk_yaw;
}
int world_marker_count(void) { int n = 0; for (int m = 0; m < W.nmark; m++) n += (W.markers >> m) & 1; return n; }
int world_marker_check(float cx, float cz)
{
    for (int m = 0; m < W.nmark; m++) {
        if ((W.markers >> m) & 1) continue;
        if (hypotf(cx - W.mark_x[m], cz - W.mark_z[m]) < T.marker_reach) { W.markers |= 1u << m; W.dirty = 1; return m; }
    }
    return -1;
}
int world_camp_check(float cx, float cz)
{
    if (!W.camp_on || hypotf(cx - W.camp_x, cz - W.camp_z) > T.camp_reach) return 0;
    if (W.nmark > 0 && world_marker_count() >= W.nmark && !W.earned[ACH_STOCK_ROUTE]) { W.camp_seen = 1; world_earn(ACH_STOCK_ROUTE); return 2; }
    if (W.camp_seen) return 0;
    W.camp_seen = 1;
    return 1;
}
int world_trunk_open(float cx, float cz, float yaw, float half_w, float front, float rear)
{
    if (!W.camp_on || W.trunk || !touches(W.trunk_x, W.trunk_z, 1.2f, cx, cz, yaw, half_w, front, rear)) return 0;
    if (W.tray_wood >= (int)T.wood_full) return 2;
    W.trunk = 1; W.tray_wood = (int)T.wood_full; W.dirty = 1;
    return 1;
}
void world_log_gone(float x, float z)
{
    if (W.nlogs_gone < LOGS_GONE_MAX) { W.logs_gone[W.nlogs_gone][0] = x; W.logs_gone[W.nlogs_gone][1] = z; W.nlogs_gone++; W.dirty = 1; }
}

/* ------------------------------------------------------------- achievements */

void world_earn(int a)
{
    if (a < 0 || a >= ACH_N || W.earned[a]) return;
    if (!(ACH[a].modes & (W.mode == MODE_EXPLORE ? IN_EXPLORE : IN_DRIFT))) return;   /* build C: not in this mode */
    W.earned[a] = 1; records_today(W.date[a]); W.dirty = 1;
    if (W.nnew < ACH_N) W.newly[W.nnew++] = a;
    if (a != ACH_FULL_HOUSE && !W.earned[ACH_FULL_HOUSE]) {   /* build E: every other one */
        int all = 1; for (int b = 0; b < ACH_N; b++) if (b != ACH_FULL_HOUSE && !W.earned[b]) all = 0;
        if (all) { W.earned[ACH_FULL_HOUSE] = 1; records_today(W.date[ACH_FULL_HOUSE]); if (W.nnew < ACH_N) W.newly[W.nnew++] = ACH_FULL_HOUSE; }
    }
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
        if (st->wood >= (int)T.wood_full) world_earn(ACH_GONE_WOODIN);
        if (st->explore) {   /* build C: Off the beaten track, the creek way from the hut's clearing all the way down to the ford */
            static float since_clearing = 1e9f;
            since_clearing = st->in_clearing ? 0.0f : since_clearing + st->dt;
            if (st->in_creek && st->creek_s < st->creek_gap_s + 80.0f && since_clearing < 30.0f) W.creek_from_hut = 1;
            if (!st->in_creek && !st->in_clearing && st->creek_s < st->creek_len - 300.0f) W.creek_from_hut = 0;   /* back out onto the road */
            if (W.creek_from_hut && st->in_creek && st->creek_len > 0 && st->creek_s >= st->creek_len - 40.0f) world_earn(ACH_OFF_TRACK);
        }
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
    static World ex; memset(&ex, 0, sizeof(ex));   /* build C: the Explorer lines, kept only if the whole file reads */
    memset(dates, 0, sizeof(dates));
    while (fgets(line, sizeof(line), f)) {
        if (!head) { if (strncmp(line, "bush-drift progress 1", 21)) break; head = 1; continue; }
        if (!strncmp(line, "deer ", 5)) {
            for (int d = 0; d < WORLD_DEER + WORLD_EXTRA && line[5 + d] && line[5 + d] != '\n'; d++) if (line[5 + d] == '1') found |= 1u << d;
        } else if (!strncmp(line, "army 1", 6)) { army = 1;
        } else if (!strncmp(line, "mode explore", 12)) { ex.mode = MODE_EXPLORE;
        } else if (sscanf(line, "tray %d", &ex.tray_wood) == 1) {
        } else if (sscanf(line, "fire %d", &ex.fire_level) == 1) {
        } else if (sscanf(line, "pile %d", &ex.woodpile) == 1) {
        } else if (sscanf(line, "night %f", &ex.night) == 1) {
        } else if (!strncmp(line, "hamper ", 7)) {
            for (int h = 0; h < HAMPER_N && line[7 + h] && line[7 + h] != '\n'; h++) if (line[7 + h] == '1') ex.hamper |= 1u << h;
        } else if (!strncmp(line, "board 1", 7)) { ex.board = 1;
        } else if (!strncmp(line, "hutnight 1", 10)) { ex.hut_night = 1;
        } else if (sscanf(line, "rifle %d", &ex.rifle) == 1) {
        } else if (sscanf(line, "quayle %d", &ex.quayle) == 1) {
        } else if (sscanf(line, "traydeer %d", &ex.tray_deer) == 1) {
        } else if (!strncmp(line, "stew 1", 6)) { ex.stew = 1;
        } else if (!strncmp(line, "markers ", 8)) {   /* build D */
            for (int m = 0; m < 12 && line[8 + m] && line[8 + m] != '\n'; m++) if (line[8 + m] == '1') ex.markers |= 1u << m;
        } else if (!strncmp(line, "trunk 1", 7)) { ex.trunk = 1;
        } else if (!strncmp(line, "stags ", 6)) {
            for (int m = 0; m < 128 && line[6 + m] && line[6 + m] != '\n'; m++) if (line[6 + m] == '1') ex.stag_down[m >> 6] |= 1ull << (m & 63);
        } else if (sscanf(line, "saw %f %d %d %d", &ex.saw, &ex.razor, &ex.streak, &ex.sharpened) >= 1) {   /* build E */
            ex.have_saw = 1;
        } else if (sscanf(line, "tracked %d %d %d", &ex.tracked[0], &ex.tracked[1], &ex.tracked[2]) >= 1) {
        } else if (sscanf(line, "trials %d %f %d", &ex.trials_best, &ex.wet_m, &ex.day) >= 1) {
        } else if (sscanf(line, "ute %f %f %f %f %f %d %d", &ex.tyre_wear[0], &ex.tyre_wear[1], &ex.tyre_wear[2], &ex.tyre_wear[3], &ex.body_dmg, &ex.punct, &ex.spare) == 7) {
            ex.have_ute = 1;
        } else if (!strncmp(line, "wet ", 4)) {   /* build D, part 2: a wet log moved or smashed */
            int k, st; float x, z, y;
            if (sscanf(line, "wet %d %d %f %f %f", &k, &st, &x, &z, &y) == 5 && k >= 0 && k < 48) { ex.wet_state[k] = st; ex.wet_x[k] = x; ex.wet_z[k] = z; ex.wet_yaw[k] = y; }
        } else if (!strncmp(line, "staglogs ", 9)) {
            for (int m = 0; m < 128 && line[9 + m] && line[9 + m] != '\n'; m++) if (line[9 + m] == '1') ex.stag_logs[m >> 6] |= 1ull << (m & 63);
        } else if (ex.nlogs_gone < LOGS_GONE_MAX && sscanf(line, "log %f %f", &ex.logs_gone[ex.nlogs_gone][0], &ex.logs_gone[ex.nlogs_gone][1]) == 2) { ex.nlogs_gone++;
        } else if (sscanf(line, "ach %63s %31s", key, date) == 2) {
            for (int a = 0; a < ACH_N; a++) if (!strcmp(key, ACH[a].key)) { earned[a] = 1; records_set_date(dates[a], date); }
        } else if (!strncmp(line, "end", 3)) { end = 1; break; }
    }
    fclose(f);
    if (!head || !end) return 0;
    W.found = found; W.army_found = army;
    memcpy(W.earned, earned, sizeof(earned)); memcpy(W.date, dates, sizeof(dates));
    W.mode = ex.mode; W.tray_wood = ex.tray_wood < 0 ? 0 : ex.tray_wood > (int)T.wood_full ? (int)T.wood_full : ex.tray_wood;
    W.fire_level = ex.fire_level < 0 ? 0 : ex.fire_level > 3 ? 3 : ex.fire_level; W.night = ex.night < 0 ? 0 : ex.night > 1 ? 1 : ex.night;
    W.woodpile = ex.woodpile < 0 ? 0 : ex.woodpile > 45 ? 45 : ex.woodpile;
    W.hamper = ex.hamper; W.board = ex.board; W.hut_night = ex.hut_night; W.rifle = ex.rifle < 0 || ex.rifle > 2 ? 0 : ex.rifle;
    W.nlogs_gone = ex.nlogs_gone; memcpy(W.logs_gone, ex.logs_gone, sizeof(W.logs_gone));
    W.quayle = ex.quayle < 0 || ex.quayle > 2 ? 0 : ex.quayle; W.tray_deer = ex.tray_deer < 0 || ex.tray_deer > WORLD_DEER + WORLD_EXTRA ? 0 : ex.tray_deer; W.stew = ex.stew;
    if (W.rifle == 2 && W.quayle == 0) W.quayle = 1;   /* a file from before he rode in the tray */
    for (int k = 0; k < 48; k++) if (ex.wet_state[k]) { W.wet_state[k] = ex.wet_state[k]; W.wet_x[k] = ex.wet_x[k]; W.wet_z[k] = ex.wet_z[k]; W.wet_yaw[k] = ex.wet_yaw[k]; }
    if (ex.have_saw) { W.saw = ex.saw < 0 ? 0 : ex.saw > 100 ? 100 : ex.saw; W.razor = ex.razor; W.streak = ex.streak; W.sharpened = ex.sharpened; }
    for (int q = 0; q < 3; q++) W.tracked[q] = ex.tracked[q] >= 0 && ex.tracked[q] <= ACH_N ? ex.tracked[q] : 0;
    W.trials_best = ex.trials_best; W.wet_m = ex.wet_m > 0 ? ex.wet_m : 0; W.day = ex.day;
    if (ex.have_ute) { memcpy(W.tyre_wear, ex.tyre_wear, sizeof(W.tyre_wear)); W.body_dmg = ex.body_dmg; W.punct = ex.punct; W.spare = ex.spare; }
    W.markers = ex.markers; W.trunk = ex.trunk; W.stag_down[0] = ex.stag_down[0]; W.stag_down[1] = ex.stag_down[1]; W.stag_logs[0] = ex.stag_logs[0]; W.stag_logs[1] = ex.stag_logs[1];
    return 1;
}

void world_load(const char *path)
{
    snprintf(path_, sizeof(path_), "%s", path);
    W.found = 0; W.army_found = 0; memset(W.earned, 0, sizeof(W.earned)); memset(W.date, 0, sizeof(W.date)); W.nnew = 0; W.dirty = 0;
    W.mode = MODE_DRIFT; W.tray_wood = 0; W.fire_level = 0; W.night = 0; W.hamper = 0; W.board = 0; W.hut_night = 0; W.rifle = 0; W.nlogs_gone = 0; W.quayle = 0; W.tray_deer = 0; W.stew = 0; W.woodpile = 0;
    W.markers = 0; W.trunk = 0; W.stag_down[0] = W.stag_down[1] = 0; W.stag_logs[0] = W.stag_logs[1] = 0; memset(W.wet_state, 0, sizeof(W.wet_state)); W.saw = 100; W.razor = 0; W.sharpened = 0; memset(W.tyre_wear, 0, sizeof(W.tyre_wear)); W.body_dmg = 0; W.punct = 0; W.spare = 1;
    if (read_progress(path)) return;
    char tmp[320]; snprintf(tmp, sizeof(tmp), "%s.tmp", path);
    if (read_progress(tmp)) fprintf(stderr, "progress: %s unreadable, finished save used\n", path);
}

/* Play test (22 Oct): Reset progress. The deer, the achievements and Explorer's state all go (the mode chosen stays, and
 * Drift's records are kept: they're a separate file); saved straight away. */
void world_reset_progress(void)
{
    int mode = W.mode;
    W.found = 0; W.army_found = 0; memset(W.earned, 0, sizeof(W.earned)); memset(W.date, 0, sizeof(W.date)); W.nnew = 0;
    W.tray_wood = 0; W.fire_level = 0; W.night = 0; W.hamper = 0; W.board = 0; W.hut_night = 0; W.rifle = 0; W.nlogs_gone = 0;
    W.quayle = 0; W.tray_deer = 0; W.stew = 0; W.woodpile = 0; W.creek_seen = 0; W.creek_from_hut = 0; W.run_found = 0;
    W.markers = 0; W.trunk = 0; W.camp_seen = 0; W.stag_down[0] = W.stag_down[1] = 0; W.stag_logs[0] = W.stag_logs[1] = 0; memset(W.wet_state, 0, sizeof(W.wet_state)); W.saw = 100; W.razor = 0; W.sharpened = 0; memset(W.tyre_wear, 0, sizeof(W.tyre_wear)); W.body_dmg = 0; W.punct = 0; W.spare = 1;
    W.mode = mode; W.dirty = 1;
    world_save();
}

void world_save(void)
{
    if (!path_[0]) return;
    static char text[8192]; int k = snprintf(text, sizeof(text), "bush-drift progress 1\ndeer ");
    for (int d = 0; d < WORLD_DEER + WORLD_EXTRA; d++) text[k++] = (W.found >> d) & 1 ? '1' : '0';
    text[k++] = '\n';
    if (W.army_found) k += snprintf(text + k, sizeof(text) - (size_t)k, "army 1\n");
    /* build C: Explorer (an older game reads past these lines) */
    k += snprintf(text + k, sizeof(text) - (size_t)k, "mode %s\ntray %d\nfire %d\npile %d\nnight %.3f\nhamper ", W.mode == MODE_EXPLORE ? "explore" : "drift", W.tray_wood, W.fire_level, W.woodpile, W.night);
    for (int h = 0; h < HAMPER_N; h++) text[k++] = (W.hamper >> h) & 1 ? '1' : '0';
    text[k++] = '\n';
    k += snprintf(text + k, sizeof(text) - (size_t)k, "board %d\nhutnight %d\nrifle %d\nquayle %d\ntraydeer %d\nstew %d\n", W.board, W.hut_night, W.rifle, W.quayle, W.tray_deer, W.stew);
    for (int g = 0; g < W.nlogs_gone; g++) k += snprintf(text + k, sizeof(text) - (size_t)k, "log %.1f %.1f\n", W.logs_gone[g][0], W.logs_gone[g][1]);
    if (W.nmark > 0 || W.markers) {   /* build D */
        k += snprintf(text + k, sizeof(text) - (size_t)k, "markers ");
        for (int m = 0; m < (W.nmark > 0 ? W.nmark : 10); m++) text[k++] = (W.markers >> m) & 1 ? '1' : '0';
        text[k++] = '\n';
    }
    if (W.trunk) k += snprintf(text + k, sizeof(text) - (size_t)k, "trunk 1\n");
    k += snprintf(text + k, sizeof(text) - (size_t)k, "saw %.1f %d %d %d\ntracked %d %d %d\ntrials %d %.0f %d\nute %.3f %.3f %.3f %.3f %.3f %d %d\n", W.saw, W.razor, W.streak, W.sharpened,
                  W.tracked[0], W.tracked[1], W.tracked[2], W.trials_best, W.wet_m, W.day, W.tyre_wear[0], W.tyre_wear[1], W.tyre_wear[2], W.tyre_wear[3], W.body_dmg, W.punct, W.spare);   /* build E */
    for (int q = 0; q < 48; q++) if (W.wet_state[q]) k += snprintf(text + k, sizeof(text) - (size_t)k, "wet %d %d %.2f %.2f %.3f\n", q, W.wet_state[q], W.wet_x[q], W.wet_z[q], W.wet_yaw[q]);
    if (W.stag_down[0] || W.stag_down[1]) {
        k += snprintf(text + k, sizeof(text) - (size_t)k, "stags ");
        int last = 0; for (int m = 0; m < 128; m++) if ((W.stag_down[m >> 6] >> (m & 63)) & 1) last = m;
        for (int m = 0; m <= last; m++) text[k++] = (W.stag_down[m >> 6] >> (m & 63)) & 1 ? '1' : '0';
        text[k++] = '\n';
    }
    if (W.stag_logs[0] || W.stag_logs[1]) {
        k += snprintf(text + k, sizeof(text) - (size_t)k, "staglogs ");
        int last = 0; for (int m = 0; m < 128; m++) if ((W.stag_logs[m >> 6] >> (m & 63)) & 1) last = m;
        for (int m = 0; m <= last; m++) text[k++] = (W.stag_logs[m >> 6] >> (m & 63)) & 1 ? '1' : '0';
        text[k++] = '\n';
    }
    for (int a = 0; a < ACH_N; a++) if (W.earned[a]) k += snprintf(text + k, sizeof(text) - (size_t)k, "ach %s %s\n", ACH[a].key, W.date[a][0] ? W.date[a] : "-");
    snprintf(text + k, sizeof(text) - (size_t)k, "end\n");
    if (records_write_whole(path_, text)) W.dirty = 0;
    else fprintf(stderr, "progress: could not save %s\n", path_);
}

/* build E: how far along a counted achievement is (the screen's bars and the tracked list); 0 if it isn't a counted one */
int world_ach_progress(int a, float *have, float *need)
{
    int n = 0; float h = 0, q = 0;
    switch (a) {
    case ACH_DEER_1: h = (float)world_found_count(); q = 1; n = 1; break;
    case ACH_DEER_10: h = (float)world_found_count(); q = 10; n = 1; break;
    case ACH_DEER_20: h = (float)world_found_count(); q = (float)WORLD_DEER; n = 1; break;
    case ACH_STOCK_ROUTE: h = (float)world_marker_count(); q = (float)(W.nmark ? W.nmark : 10); n = 1; break;
    case ACH_CLEAN_CUTS: h = (float)W.streak; q = 10; n = 1; break;
    case ACH_CHARCUTERIE: h = (float)world_hamper_count(); q = (float)HAMPER_N; n = 1; break;
    case ACH_WET_FEET: h = W.wet_m; q = 1000; n = 1; break;
    case ACH_GONE_WOODIN: h = (float)W.tray_wood; q = T.wood_full; n = 1; break;
    case ACH_MECHANIC: h = (float)W.sharpened; q = 3; n = 1; break;
    case ACH_TRIALS: h = (float)W.trials_best; q = 5000; n = 1; break;
    case ACH_FULL_HOUSE: for (int k = 0; k < ACH_N; k++) if (k != ACH_FULL_HOUSE) { h += (float)W.earned[k]; q += 1; } n = 1; break;
    default: break;
    }
    if (h > q) h = q;
    if (have) *have = h;
    if (need) *need = q;
    return n;
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
    W.mode = MODE_DRIFT;
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
    st.offroad_m = 5000; world_step(&st); ok &= check(!W.earned[ACH_OFF_TRACK], "build C: 5 km off the road in Drift isn't Off the beaten track any more (it's the creek way, in Explorer)");
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
    /* build C: Explorer. The modes: a Drift-only one can't be earned in Explorer, and the other way round */
    W.mode = MODE_EXPLORE; W.earned[ACH_BRIDGE] = 0; world_earn(ACH_BRIDGE); ok &= check(!W.earned[ACH_BRIDGE], "Explorer: Down the mountain can't be earned (Drift only)");
    W.mode = MODE_DRIFT; world_earn(ACH_FIRE_1); ok &= check(!W.earned[ACH_FIRE_1], "Drift: Bush telly can't be earned (Explorer only)");
    W.mode = MODE_EXPLORE;
    /* the fires: a full tray, driven in close to the hut (no need to stop) */
    W.fire_x = 0; W.fire_z = 0; W.hut_x = 0; W.hut_z = 0; W.tray_wood = 14; W.fire_level = 0; W.night = 0;
    ok &= check(world_deliver(2, 0, 0) == 0 && W.fire_level == 0, "14 logs by the fire: not a load");
    W.tray_wood = 15;
    ok &= check(world_deliver(T.hut_reach + 1.0f, 0, 5.0f) == 0, "a metre further out than hut_reach: nothing");
    ok &= check(world_deliver(T.hut_reach - 1.0f, 0, 5.0f) == 1 && W.tray_wood == 0 && W.night == 0.5f && W.earned[ACH_FIRE_1], "a full tray, driving in near the hut at 18 km/h: level 1, dusk, Bush telly");
    W.tray_wood = 15; world_deliver(2, 0, 0);
    ok &= check(W.fire_level == 2 && W.night > 0.8f && !W.earned[ACH_FIREWORKS], "the second load: level 2, night, no fireworks yet");
    W.tray_wood = 15; world_deliver(2, 0, 0);
    ok &= check(W.fire_level == 3 && W.earned[ACH_FIREWORKS], "the third: level 3, Cracker night");
    W.tray_wood = 15; ok &= check(world_deliver(2, 0, 0) == 0 && W.fire_level == 3, "a fourth load: the fire stays at 3");
    /* the hamper and the board */
    for (int i = 0; i < HAMPER_N; i++) { W.item[i].x = 1000.0f + 50.0f * (float)i; W.item[i].z = 0; W.item[i].placed = 1; }
    W.hamper = 0; W.board = 0; W.picnic_x = 500; W.picnic_z = 500; W.hut_x = 510; W.hut_z = 500;
    ok &= check(world_hamper_hit(1003.0f, 0, 0, 1.0f, 2.5f, -2.5f) == -1, "a hamper thing 3 m to the side: not picked up");
    ok &= check(world_hamper_hit(1001.5f, 0, 0, 1.0f, 2.5f, -2.5f) == 0 && world_hamper_count() == 1, "touched: in the esky (1 of 12)");
    for (int i = 1; i < HAMPER_N - 1; i++) world_hamper_hit(1000.0f + 50.0f * (float)i, 0, 0, 1.0f, 2.5f, -2.5f);
    ok &= check(world_board(500, 502, 0) == 0 && !W.board, "eleven, at the table: no board");
    world_hamper_hit(1000.0f + 50.0f * (HAMPER_N - 1), 0, 0, 1.0f, 2.5f, -2.5f);
    ok &= check(world_board(540, 500, 0) == 0, "all twelve, 30 m from the hut: no board");
    ok &= check(world_board(500, 502, 4.0f) == 1 && W.board && W.earned[ACH_CHARCUTERIE], "all twelve, driving in by the hut: the board, Charcuterie");
    /* the rifle and Quayle */
    W.rifle = 0; W.rifle_placed = 1; W.rifle_x = 200; W.rifle_z = 0; W.army_placed = 1; W.army_x = 300; W.army_z = 0; W.earned[ACH_BACK_ON_DUTY] = 0;
    ok &= check(!world_rifle_return(300.5f, 0, 0, 1.0f, 2.5f, -2.5f) && W.rifle == 0, "Quayle touched without the rifle: nothing");
    ok &= check(world_rifle_hit(201.0f, 0, 0, 1.0f, 2.5f, -2.5f) && W.rifle == 1, "the rifle touched: in the tray");
    ok &= check(world_rifle_return(300.5f, 0, 0, 1.0f, 2.5f, -2.5f) && W.rifle == 2 && W.earned[ACH_BACK_ON_DUTY], "Quayle touched with it: back on duty");
    ok &= check(W.quayle == 1 && world_extra_on(), "he rides in the tray; the extra five deer are out");
    /* a deer found with Quayle in the tray: into the tray, he gets in the cab; then the stew at the table */
    W.ndeer = 1; W.nextra = 1; W.deer[0].x = 4000; W.deer[0].z = 0; W.deer[WORLD_DEER].x = 4100; W.deer[WORLD_DEER].z = 0; W.found = 0; W.stew = 0; W.tray_deer = 0;
    ok &= check(world_deer_hit(4100.5f, 0, 0, 1.0f, 2.5f, -2.5f) == WORLD_DEER && W.tray_deer == WORLD_DEER + 1 && W.quayle == 2 && world_found_count() == 0,
                "an extra deer found: in the tray, Quayle in the cab, not counted in the twenty");
    ok &= check(world_deer_hit(4000.5f, 0, 0, 1.0f, 2.5f, -2.5f) == 0 && W.tray_deer == WORLD_DEER + 1, "the next deer: found as ever, the tray keeps the first");
    W.fire_level = 1;
    ok &= check(world_stew(500, 502, 0) == 2 && W.tray_deer && !W.stew, "at the table on a level 1 fire: not hot enough");
    W.fire_level = 2;
    ok &= check(world_stew(540, 500, 0) == 0, "30 m from the hut: nothing");
    ok &= check(world_stew(500, 502, 3.0f) == 1 && !W.tray_deer && W.stew && W.earned[ACH_CAMP_OVEN], "driven in on a level 2 fire: the camp oven, Camp oven");
    W.fire_level = 3;
    /* the creek way, for Off the beaten track: from the clearing into it, then all the way down */
    W.earned[ACH_OFF_TRACK] = 0; W.creek_from_hut = 0;
    world_run_start(); memset(&st, 0, sizeof(st)); st.dt = 0.1f; st.running = 1; st.explore = 1; st.creek_len = 6000; st.creek_gap_s = 400;
    st.in_creek = 1; st.creek_s = 5980; world_step(&st);
    ok &= check(!W.earned[ACH_OFF_TRACK], "at the ford end of the creek way, not from the hut: not yet");
    st.in_creek = 0; st.creek_s = 300; st.in_clearing = 1; world_step(&st);
    st.in_clearing = 0; st.in_creek = 1; st.creek_s = 420; world_step(&st);
    st.in_creek = 0; st.creek_s = 2000; world_step(&st);
    st.in_creek = 1; st.creek_s = 5980; world_step(&st);
    ok &= check(!W.earned[ACH_OFF_TRACK], "from the clearing, but out of it to the road half way: not yet");
    st.in_creek = 0; st.creek_s = 300; st.in_clearing = 1; world_step(&st);
    st.in_clearing = 0; st.in_creek = 1;
    for (float cs = 420; cs < 5990; cs += 50) { st.creek_s = cs; world_step(&st); }
    ok &= check(W.earned[ACH_OFF_TRACK], "the clearing to the ford down the creek way: Off the beaten track");
    /* build D: the old stock route: ten markers, the camp, the trunk */
    W.nmark = 10; W.markers = 0; W.camp_seen = 0; W.camp_on = 1; W.camp_x = 8000; W.camp_z = 0; W.trunk_x = 8004; W.trunk_z = 0; W.trunk = 0; W.earned[ACH_STOCK_ROUTE] = 0;
    for (int m = 0; m < 10; m++) { W.mark_x[m] = 7000.0f + 50.0f * (float)m; W.mark_z[m] = 6.0f; W.mark_v[m] = m % 5; }
    ok &= check(world_marker_check(7000, 6.0f + T.marker_reach + 0.5f) == -1, "a marker a little over marker_reach away: not counted");
    ok &= check(world_marker_check(7000, 0) == 0 && world_marker_count() == 1 && world_marker_check(7000, 0) == -1, "driven past within a few metres: MARKER 1/10, and only once");
    for (int m = 1; m < 9; m++) world_marker_check(7000.0f + 50.0f * (float)m, 0);
    ok &= check(world_camp_check(8000, 5) == 1 && !W.earned[ACH_STOCK_ROUTE], "the camp with 9 of 10: its message, no achievement");
    ok &= check(world_camp_check(8000, 5) == 0, "and the message only once a session");
    world_marker_check(7450, 0);
    ok &= check(world_marker_count() == 10 && world_camp_check(8000, 5) == 2 && W.earned[ACH_STOCK_ROUTE], "back at the camp with all ten: Droving days");
    W.tray_wood = 15; ok &= check(world_trunk_open(8003, 0, 0, 1.0f, 2.5f, -2.5f) == 2 && !W.trunk, "the trunk with a full tray: stays shut");
    W.tray_wood = 3; ok &= check(world_trunk_open(8010, 0, 0, 1.0f, 2.5f, -2.5f) == 0, "the trunk 6 m off: nothing");
    ok &= check(world_trunk_open(8003, 0, 0, 1.0f, 2.5f, -2.5f) == 1 && W.trunk && W.tray_wood == (int)T.wood_full, "driven up to it: open, and a full tray of firewood");
    W.tray_wood = 0; ok &= check(world_trunk_open(8003, 0, 0, 1.0f, 2.5f, -2.5f) == 0, "open already: once a day");
    W.stag_down[0] = 0x5ull; W.stag_down[1] = 1ull << 3;
    /* a new day */
    W.tray_wood = 7; world_new_day();
    ok &= check(!W.trunk && !W.stag_down[0] && !W.stag_down[1] && world_marker_count() == 10, "build D, a new day: the trunk shut (more wood), the dead trees standing; the markers stay found");
    W.trunk = 1; W.stag_down[0] = 0x5ull; W.stag_down[1] = 1ull << 3;
    W.wet_state[3] = 1; W.wet_x[3] = 123.5f; W.wet_z[3] = -45.25f; W.wet_yaw[3] = 1.25f; W.wet_state[7] = 2;
    ok &= check(W.fire_level == 0 && W.night == 0 && W.tray_wood == 0 && W.board == 1 && W.rifle == 2, "a new day: the fire out, golden hour, the tray empty; the board and the rifle stay");
    W.fire_level = 2; W.night = 0.9f; W.tray_wood = 4; world_log_gone(12.5f, -7.5f); world_log_gone(100.0f, 3.0f);
    /* saved and read back */
    uint32_t f0 = W.found; int e0[ACH_N]; memcpy(e0, W.earned, sizeof(e0));
    world_save();
    W.found = 0; W.army_found = 0; memset(W.earned, 0, sizeof(W.earned));
    W.mode = 0; W.fire_level = 0; W.night = 0; W.tray_wood = 0; W.hamper = 0; W.board = 0; W.rifle = 0; W.nlogs_gone = 0; W.quayle = 0; W.stew = 0;
    W.markers = 0; W.trunk = 0; W.stag_down[0] = W.stag_down[1] = 0; W.stag_logs[0] = W.stag_logs[1] = 0; memset(W.wet_state, 0, sizeof(W.wet_state)); W.saw = 100; W.razor = 0; W.sharpened = 0; memset(W.tyre_wear, 0, sizeof(W.tyre_wear)); W.body_dmg = 0; W.punct = 0; W.spare = 1;
    world_load(path);
    ok &= check(world_marker_count() == 10 && W.trunk == 1 && W.stag_down[0] == 0x5ull && W.stag_down[1] == (1ull << 3), "build D read back: the markers, the trunk, the dead trees down");
    ok &= check(W.wet_state[3] == 1 && fabsf(W.wet_x[3] - 123.5f) < 0.01f && fabsf(W.wet_yaw[3] - 1.25f) < 0.01f && W.wet_state[7] == 2 && W.wet_state[0] == 0, "part 2 read back: a wet log shoved (where to) and one smashed");
    ok &= check(W.found == f0 && !memcmp(e0, W.earned, sizeof(e0)) && W.army_found == 1, "saved and read back the same (the army man too)");
    ok &= check(W.mode == MODE_EXPLORE && W.fire_level == 2 && fabsf(W.night - 0.9f) < 0.01f && W.tray_wood == 4 && world_hamper_count() == HAMPER_N && W.board && W.rifle == 2
                && W.nlogs_gone == 2 && fabsf(W.logs_gone[1][0] - 100.0f) < 0.1f && W.quayle == 2 && W.stew, "Explorer's state read back: the mode, the fire, the night, the tray, the hamper, the board, the rifle, the logs gone");
    world_new_day(); ok &= check(!W.wet_state[3] && !W.wet_state[7], "part 2, a new day: the wet logs back where they lay");
    FILE *f = fopen(path, "w"); fprintf(f, "bush-drift progress 1\ndeer 1111"); fclose(f);
    world_load(path);
    ok &= check(W.found == 0, "a damaged file reads as nothing (not as part of one)");
    remove(path);
    W = keep;
    printf(ok ? "worldtest finished\n" : "worldtest FAILED\n");
    return ok ? 0 : 1;
}
