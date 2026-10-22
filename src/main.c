/* Bush Drift: a drifting game on a real dirt road, Alexanders Hut to the Tantawangalo Creek bridge
 * (src/route.c), in a LandCruiser 79 ute (src/car.c). Platform layer (raylib): window, input, chase camera, the low-resolution pixel look,
 * readout. Physics axes are x right (east), y up, z forward (north); drawing mirrors z (see RV). */
#include "car.h"
#include "tuning.h"
#include "bench.h"
#include "records.h"
#include "route.h"
#include "erecords.h"
#include "world.h"
#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>

#define GAME_VERSION "play test 3"
#define VIEW_W 640
#define VIEW_H 480

/* the phone builds (Android, and iOS from session 06): the whole screen, touch buttons, GLSL ES 3.00, no Quit */
#if defined(__ANDROID__) || defined(BD_IOS)
#define BD_PHONE 1
#endif
#ifdef BD_IOS
#include "../ios/bd_ios.h"
#endif

#ifdef HANDHELD /* what raylib's loader found on the device, for the bench report */
extern int GLAD_GL_EXT_discard_framebuffer;
extern const unsigned char *(*glad_glGetString)(unsigned int name);
#endif

static Vector3 RV(V3 v) { Vector3 r = { v.x, v.y, -v.z }; return r; }

/* Drawing happens relative to a floating origin (v2.0.1, 16 Oct). The simulation keeps its world
 * coordinates, so every road drives as before; only what goes to the graphics chip is made small.
 * Far from the start (4 km out at 10 km driven) the handheld's vertex maths lost precision and the
 * walls wobbled. ro, the drawing origin, follows the car every frame; the camera and every matrix
 * are computed from differences with it on the processor, in full precision. Buffers that are
 * written once hold positions relative to an origin of their own: each road chunk its first
 * centreline point, the posts and dots the drawing origin when they were last written whole, the
 * tyre marks a base moved every 500 m (marks_rebase). */
static V3 ro;
static Route RT;      /* the road (route.c), read at start */
static V3 anchors[4]; /* the handling's wheel positions (set by select_model) */
static Vector3 RO(V3 v) { return RV(vsub(v, ro)); }
static Matrix at_base(V3 base) { return MatrixTranslate(base.x - ro.x, 0, -(base.z - ro.z)); } /* a buffer written relative to base */

/* ------------------------------------------------------------- car model */

typedef struct { V3 p[3]; unsigned char r, g, b, lamp; } Tri;   /* build C: lamp = glows at night */
static Tri *tris;
static int ntris;
static const char *model_note = "box (no car file)";

static void push_tri(V3 a, V3 b, V3 c, float r, float g, float bl)
{
    tris = realloc(tris, sizeof(Tri) * (size_t)(ntris + 1));
    V3 n = vnorm(vcross(vsub(b, a), vsub(c, a)));
    float shade = 0.62f + 0.45f * maxf(vdot(n, vnorm(v3(0.55f, 0.65f, 0.5f))), 0); /* bright enough: the palette step darkens mid greys */
    Tri *t = &tris[ntris++];
    t->p[0] = a; t->p[1] = b; t->p[2] = c; t->lamp = 0;
    t->r = (unsigned char)(clamp01(r * shade) * 255);
    t->g = (unsigned char)(clamp01(g * shade) * 255);
    t->b = (unsigned char)(clamp01(bl * shade) * 255);
}

/* A small flat-coloured box added to the car model, faces wound outwards like the body's. */
static void push_light(V3 c, float h, int r, int g, int b)
{
    V3 p[8];
    for (int i = 0; i < 8; i++) p[i] = v3(c.x + ((i & 1) ? h : -h), c.y + ((i & 2) ? h : -h), c.z + ((i & 4) ? h : -h));
    static const int q[6][4] = { {0,1,3,2}, {4,6,7,5}, {0,4,5,1}, {2,3,7,6}, {0,2,6,4}, {1,5,7,3} };
    for (int f = 0; f < 6; f++) {
        for (int k = 0; k < 2; k++) {
            V3 a = p[q[f][0]], b2 = p[q[f][k + 1]], c2 = p[q[f][k + 2]];
            V3 n = vcross(vsub(b2, a), vsub(c2, a));
            V3 mid = vscale(vadd(vadd(a, b2), c2), 1.0f / 3.0f);
            if (vdot(n, vsub(mid, c)) < 0) { V3 t2 = b2; b2 = c2; c2 = t2; }
            push_tri(a, b2, c2, 1, 1, 1);
            tris[ntris - 1].r = (unsigned char)r; tris[ntris - 1].g = (unsigned char)g; tris[ntris - 1].b = (unsigned char)b;
        }
    }
}

/* Build C: a box between two corners, faces wound outwards. */
static void push_box(V3 lo, V3 hi, float r, float g, float b)
{
    V3 p[8], c = vscale(vadd(lo, hi), 0.5f);
    for (int i = 0; i < 8; i++) p[i] = v3((i & 1) ? hi.x : lo.x, (i & 2) ? hi.y : lo.y, (i & 4) ? hi.z : lo.z);
    static const int q[6][4] = { {0,1,3,2}, {4,6,7,5}, {0,4,5,1}, {2,3,7,6}, {0,2,6,4}, {1,5,7,3} };
    for (int f = 0; f < 6; f++)
        for (int k = 0; k < 2; k++) {
            V3 a = p[q[f][0]], b2 = p[q[f][k + 1]], c2 = p[q[f][k + 2]];
            V3 mid = vscale(vadd(vadd(a, b2), c2), 1.0f / 3.0f);
            if (vdot(vcross(vsub(b2, a), vsub(c2, a)), vsub(mid, c)) < 0) { V3 t2 = b2; b2 = c2; c2 = t2; }
            push_tri(a, b2, c2, r, g, b);
        }
}

static void make_box_model(void)
{
    float x = 0.9f, y0 = -0.35f, y1 = 0.6f, z0 = -2.2f, z1 = 2.0f;
    V3 c[8];
    for (int i = 0; i < 8; i++) c[i] = v3((i & 1) ? x : -x, (i & 2) ? y1 : y0, (i & 4) ? z1 : z0);
    static const int q[6][4] = { {0,1,3,2}, {4,6,7,5}, {0,4,5,1}, {2,3,7,6}, {0,2,6,4}, {1,5,7,3} };
    for (int f = 0; f < 6; f++) {
        float r = 0.85f, g = (f == 1) ? 0.3f : 0.85f, b = (f == 1) ? 0.3f : 0.85f; /* front face red */
        push_tri(c[q[f][0]], c[q[f][1]], c[q[f][2]], r, g, b);
        push_tri(c[q[f][0]], c[q[f][2]], c[q[f][3]], r, g, b);
    }
}

/* The car body (tools/make_79.py makes it). The file's "wheel" lines are where its wheels are: the handling,
 * the drawn wheels and the tyre marks all use them. "outline" is the body's box for trunks and posts and
 * the shadow; "lamps own" means the model has its own lamp colours ("lamp" lines are
 * triangles drawn at full brightness), so no light boxes are added. */
typedef struct {
    const char *name, *path, *note;
    int loaded, first, count; /* its triangles in tris[] */
    V3 own[4], phys[4];
    int has_phys, has_outline, has_ball, own_lamps;
    float half_w, front, rear, ball_x, ball_z, ball_r;
    Mesh mesh, shadow;
    /* ours: a body may carry its own wheels ("wtri" lines, each wheel about its own centre, x along the
     * axle) and their size ("wheelsize radius width"); drawn steered and turning. Otherwise the plain drum. */
    int wfirst[4], wcount[4], has_wsize;
    float wheel_r, wheel_w;
    Mesh wmesh[4];
} CarModel;
#define NMODELS 1
static CarModel models[NMODELS] = {
    { .name = "LC79", .path = "assets-built/lc79.car", .note = "LandCruiser 79" },
};
static Tri *wtris; /* the bodies' own wheels */
static int nwtris;
static const int cur_model = 0;
static V3 wheel_draw_off[4]; /* from the handling's wheel to where this body's wheel is drawn, in the car's frame */

static int load_car_file(CarModel *m)
{
    FILE *f = fopen(m->path, "r");
    if (!f) return 0;
    char line[512], id[8];
    int wheels = 0, pwheels = 0, before = ntris;
    while (fgets(line, sizeof(line), f)) {
        float v[12];
        if (sscanf(line, "wheel %7s %f %f %f", id, &v[0], &v[1], &v[2]) == 4) {
            int i = !strcmp(id, "FL") ? FL : !strcmp(id, "FR") ? FR : !strcmp(id, "RL") ? RL : RR;
            m->own[i] = v3(v[0], v[1], v[2]);
            wheels++;
        } else if (sscanf(line, "physwheel %7s %f %f %f", id, &v[0], &v[1], &v[2]) == 4) {
            int i = !strcmp(id, "FL") ? FL : !strcmp(id, "FR") ? FR : !strcmp(id, "RL") ? RL : RR;
            m->phys[i] = v3(v[0], v[1], v[2]);
            pwheels++;
        } else if (sscanf(line, "outline %f %f %f", &v[0], &v[1], &v[2]) == 3) {
            m->half_w = v[0]; m->front = v[1]; m->rear = v[2]; m->has_outline = 1;
        } else if (sscanf(line, "ball %f %f %f", &v[0], &v[1], &v[2]) == 3) {
            m->ball_x = v[0]; m->ball_z = v[1]; m->ball_r = v[2]; m->has_ball = 1;
        } else if (sscanf(line, "wheelsize %f %f", &v[0], &v[1]) == 2) {
            m->wheel_r = v[0]; m->wheel_w = v[1]; m->has_wsize = 1;
        } else if (sscanf(line, "wtri %7s %f %f %f %f %f %f %f %f %f %f %f %f", id, &v[0], &v[1], &v[2], &v[3], &v[4], &v[5],
                          &v[6], &v[7], &v[8], &v[9], &v[10], &v[11]) == 13) {
            int i = !strcmp(id, "FL") ? FL : !strcmp(id, "FR") ? FR : !strcmp(id, "RL") ? RL : RR;
            /* kept in file order per wheel: the file lists FL, FR, RL, RR in turn */
            if (m->wcount[i] == 0) m->wfirst[i] = nwtris;
            Tri *t0 = tris; int n0 = ntris; /* shade with push_tri, then move the triangle across */
            push_tri(v3(v[3], v[4], v[5]), v3(v[6], v[7], v[8]), v3(v[9], v[10], v[11]), v[0], v[1], v[2]);
            wtris = realloc(wtris, sizeof(Tri) * (size_t)(nwtris + 1));
            wtris[nwtris++] = tris[ntris - 1];
            ntris = n0; (void)t0;
            m->wcount[i]++;
        } else if (!strncmp(line, "lamps own", 9)) {
            m->own_lamps = 1;
        } else if (sscanf(line, "tri %f %f %f %f %f %f %f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5],
                          &v[6], &v[7], &v[8], &v[9], &v[10], &v[11]) == 12) {
            push_tri(v3(v[3], v[4], v[5]), v3(v[6], v[7], v[8]), v3(v[9], v[10], v[11]), v[0], v[1], v[2]);
        } else if (sscanf(line, "lamp %f %f %f %f %f %f %f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5],
                          &v[6], &v[7], &v[8], &v[9], &v[10], &v[11]) == 12) {
            push_tri(v3(v[3], v[4], v[5]), v3(v[6], v[7], v[8]), v3(v[9], v[10], v[11]), 1, 1, 1);
            tris[ntris - 1].r = (unsigned char)(clamp01(v[0]) * 255); tris[ntris - 1].g = (unsigned char)(clamp01(v[1]) * 255); tris[ntris - 1].b = (unsigned char)(clamp01(v[2]) * 255);
            tris[ntris - 1].lamp = 1;
        }
    }
    fclose(f);
    m->has_phys = pwheels == 4;
    if (wheels != 4 || ntris == before) { ntris = before; return 0; }
    return 1;
}

static struct { int own; float half_w, front, rear; } car_outline;   /* the body's box for trunks and posts, from the model file */

/* --------------------------------------------------------------- scoring */
/* Bush Drift's own rules (session 3b; build A, 21 Oct: any slide scores, no time-out, a bank button).
 * The drift angle is the rear axle's slide angle (the rear tyres' slip angle): it is zero driving straight
 * and stays small in a gripping bend, where the body's own angle at its middle would not. Past score_dead_deg
 * it fills the pot at score_rate * speed * ((angle - dead) / score_ref_deg) ^ score_curve points a second,
 * the angle capped at score_cap_deg. The multiplier climbs one step per score_mult_hold seconds of sliding,
 * counted at full rate from score_mult_deg of angle, pro rata down to nothing at score_mult_from_deg, up to score_mult_max.
 * The pot is only yours when banked: the bank button (B, Right Shift, L2, or BANK on a phone) or the bridge.
 * A crash loses it; so does the ditch (ditch_rule 0; 1 banks it instead). score_autobank 1 brings back the
 * old rules: a gap over score_gap in the slide restarts the multiplier's count, and score_bank_after seconds
 * below score_min_deg banks the pot. */

static float score_boost = 1.0f;   /* build A: pot points and donut bonuses are multiplied by this (Drift Yodel mode) */
typedef struct {
    float pot, total, held, since_slide; int mult; int drifting; int ev_mult_up, ev_banked;
    float last_banked, best_banked; int best_mult; /* for the records: the last and the biggest combo banked, the highest multiplier */
    float angle, rate;     /* this frame's drift angle (deg) and pot points a second, for the readout */
    float ditch_t;         /* seconds the car has been in the ditch (see ditch_wheels) */
    float flash_t, flash_v; int flash_kind;   /* the HUD's "BANKED" / "LOST" moment: seconds left, points, 1 banked 2 lost */
    int ev_lost;           /* the pot was lost this frame (crash or ditch): the soft "lost" notes */
    /* donuts (build A): the turn so far in one direction, the ground it covers, and the count this run */
    float donut_turn, donut_off, dn_x0, dn_x1, dn_z0, dn_z1, donut_pts; int donuts, ev_donut;   /* ev_donut: 1 scored, 2 practice (the car park) */
    /* build B: off the road (past the ditch, not in the hut's clearing): no pot builds; the distance driven so */
    int offroad; float offroad_m;
} Score;

static float score_combo(const Score *s) { return s->pot * (float)s->mult; }   /* unrounded (the HUD rounds), so small banks count */

static void score_expire(Score *s)   /* bank */
{
    s->last_banked = score_combo(s);
    if (s->last_banked > s->best_banked) s->best_banked = s->last_banked;
    s->total += s->last_banked;
    if (s->last_banked >= 10) { s->flash_t = 2.0f; s->flash_v = s->last_banked; s->flash_kind = 1; s->ev_banked = 1; }   /* crumbs bank quietly */
    s->pot = 0; s->held = 0; s->mult = 1;
}

static void score_lose(Score *s)
{
    float v = score_combo(s);
    if (v >= 10) { s->flash_t = 2.0f; s->flash_v = v; s->flash_kind = 2; s->ev_lost = 1; }
    s->pot = 0; s->held = 0; s->mult = 1;
}

/* The drift angle: the rear tyres' mean slip angle in degrees, going forwards (0 otherwise). */
static float drift_angle(const Car *c)
{
    if (car_local_velocity(c).z < 1.0f) return 0;
    return 0.5f * (fabsf(c->wheel[RL].slip_angle) + fabsf(c->wheel[RR].slip_angle)) * RAD2DEGF;
}

/* Pot points a second at this angle and speed (m/s). */
static float score_rate_at(float angle, float speed)
{
    float a = fminf(angle, T.score_cap_deg) - T.score_dead_deg;
    if (a <= 0 || speed <= 0) return 0;
    return T.score_rate * speed * powf(a / fmaxf(T.score_ref_deg, 1.0f), T.score_curve);
}

/* deep: wheels past the verge by ditch_margin or more (rt_physics); offroad: the car's middle past the ditch
 * (build B: crossing the ditch costs the pot once, as the ditch does; off the road nothing builds a pot) */
static void score_update(Score *s, const Car *c, float dt, int allowed, int deep, int offroad)
{
    if (s->flash_t > 0) s->flash_t -= dt;
    float ang = drift_angle(c), sp = vlen(v3(c->body.vel.x, 0, c->body.vel.z));
    s->angle = ang;
    if (allowed && offroad && !s->offroad && (s->pot > 0 || s->mult > 1)) { if (T.ditch_rule != 0) score_expire(s); else score_lose(s); }
    s->offroad = offroad;
    if (offroad) {
        if (allowed) s->offroad_m += sp * dt;
        s->rate = 0; s->drifting = 0; s->ditch_t = 0;
        return;
    }
    s->rate = allowed ? score_rate_at(ang, sp) : 0;
    s->drifting = s->rate > 0;
    /* the ditch: ends the multiplier and loses (or banks) the pot */
    s->ditch_t = allowed && deep >= (int)T.ditch_wheels ? s->ditch_t + dt : 0;
    if (s->ditch_t > T.ditch_time && (s->pot > 0 || s->mult > 1)) {
        if (T.ditch_rule != 0) score_expire(s); else score_lose(s);
        return;
    }
    if (T.score_autobank != 0) {   /* the old rules */
        if (allowed && ang > T.score_min_deg) s->since_slide = 0;
        else {
            s->since_slide += dt;
            if (s->since_slide > T.score_gap) s->held = 0;
            if (s->since_slide > T.score_bank_after && s->pot > 0) score_expire(s);
        }
    }
    if (s->drifting) {
        s->pot += s->rate * score_boost * dt;
        s->held += dt * clamp01((ang - T.score_mult_from_deg) / fmaxf(T.score_mult_deg - T.score_mult_from_deg, 0.1f));
        if (s->held >= T.score_mult_hold && s->mult < (int)T.score_mult_max) { s->mult++; s->held = 0; s->ev_mult_up = 1; }
        if (s->mult > s->best_mult) s->best_mult = s->mult;
    }
}

/* The rules before build A, kept only so --drivetest can score the same drive both ways: over 3 deg of body
 * slide and 8 km/h, 10 points a second per m/s sideways plus 0.2 of the forward speed; a gap over 0.6 s restarts
 * the multiplier's count, 1.8 s straight banks (to the nearest ten), a crash loses the pot. */
typedef struct { float pot, total, held, since; int mult, best_mult; } OldScore;
static void old_score_update(OldScore *s, const Car *c, float dt, int allowed)
{
    V3 lv = car_local_velocity(c);
    if (allowed && lv.z * 3.6f > 8.0f && c->slide_deg > 3.0f) {
        s->pot += 10.0f * (fabsf(lv.x) + 0.2f * fabsf(lv.z)) * dt;
        s->held += dt; s->since = 0;
        if (s->held >= 1.2f && s->mult < 9) { s->mult++; s->held = 0; }
        if (s->mult > s->best_mult) s->best_mult = s->mult;
    } else {
        s->since += dt;
        if (s->since > 0.6f) s->held = 0;
        if (s->since > 1.8f && s->pot > 0) { s->total += roundf(s->pot * (float)s->mult / 10.0f) * 10.0f; s->pot = 0; s->held = 0; s->mult = 1; }
    }
}

/* A whole number with thousands commas. */
static const char *commas(int v)
{
    static char buf[4][24];
    static int turn;
    char *o = buf[turn++ & 3], tmp[16];
    int n = snprintf(tmp, sizeof(tmp), "%d", v < 0 ? -v : v), k = 0;
    if (v < 0) o[k++] = '-';
    for (int i = 0; i < n; i++) { o[k++] = tmp[i]; if ((n - 1 - i) % 3 == 0 && i != n - 1) o[k++] = ','; }
    o[k] = 0;
    return o;
}

/* --------------------------------------------------------------- the run */
/* Bush Drift's own run rules: a countdown, then the clock runs until the bridge, a hard hit (tree, post,
 * hut) or driving the wrong way for wrong_way_time seconds. */
enum { RUN_COUNTDOWN, RUN_RUNNING, RUN_FINISHED, RUN_WRONG_WAY_OVER, RUN_WALL_OVER };
enum { DIR_CORRECT, DIR_WRONG };
typedef struct { int state; float countdown, time; int dir, wrong_shown; float wrong_left, ev_hit_speed;
                 int next_cp, ncp, missed, ev_cp; float prev_s, back_m;
                 float offroad_t; int wood; float ev_thud; } Run;   /* build C: ev_thud = the speed of a hit that only stopped the ute (Explorer) */   /* follow-up: seconds off the road (15 puts you back on the track); logs run over */   /* build B: checkpoints passed in order; metres driven backwards */
static int explore;              /* build C: Explorer mode (free roaming; see the "Explorer" section) */
static void run_reset(Run *r, int with_countdown)
{
    memset(r, 0, sizeof(*r));
    r->state = with_countdown ? RUN_COUNTDOWN : RUN_RUNNING;
    r->countdown = with_countdown && !explore ? 3.8f : 0.0f;   /* build C: Explorer only waits for the road to be there */
    r->dir = DIR_CORRECT; r->wrong_left = T.wrong_way_time;
    r->prev_s = -1e9f;
}

/* ------------------------------------------------------------ tyre marks */

#ifndef HUD_COPIES
#define HUD_COPIES 3
#endif
#define NMARKS 6000
#ifndef MARK_COPIES
#define MARK_COPIES 3
#endif
/* NMARKS quads held on the graphics chip. A new mark overwrites the oldest quad and only
 * the new part is sent. There are MARK_COPIES identical buffers used in turn, one a frame,
 * each brought up to date just before its turn: the handheld's driver waits a whole screen
 * refresh if asked to change a buffer that the frame before is still being drawn from. */
static Mesh mark_mesh;                 /* the copy to draw this frame; vertices and colours are the one master copy */
static Mesh mark_gpu[MARK_COPIES];
static long mark_synced[MARK_COPIES];  /* how many marks had been made when each copy was last brought up to date */
static long mark_total;
static Material flat_mat, mark_mat;
static int mark_ready, mark_next, mark_added, mark_fades, mark_time_loc;
static double mark_clock; /* seconds of game time, for the fade */
#define MARK_TICKS 20.0 /* a mark's time of birth is kept in its colour, in twentieths of a second, 16 bits */

/* Tyre marks fade by age without their buffers ever being rewritten: each mark carries when it was
 * made (in the red and green bytes of its colour) and the graphics chip works out how faded it is
 * from the time now. A mark past its life is moved off the screen, so it costs nothing to fill. */
#ifdef HANDHELD
/* raylib's own shader for OpenGL ES 2 declares mediump in the vertex stage: positions and the camera
 * matrix then lose precision on this chip. Ours is the same shader with highp in the vertex stage. */
static const char *flat_vs = "#version 100\nprecision highp float;\nattribute vec3 vertexPosition; attribute vec2 vertexTexCoord; attribute vec4 vertexColor;\n"
    "varying vec2 fragTexCoord; varying vec4 fragColor; uniform mat4 mvp;\n"
    "void main(){ fragTexCoord = vertexTexCoord; fragColor = vertexColor; gl_Position = mvp*vec4(vertexPosition, 1.0); }\n";
static const char *flat_fs = "#version 100\nprecision mediump float;\nvarying vec2 fragTexCoord; varying vec4 fragColor; uniform sampler2D texture0; uniform vec4 colDiffuse;\n"
    "void main(){ gl_FragColor = texture2D(texture0, fragTexCoord)*colDiffuse*fragColor; }\n";
static const char *mark_vs = "precision highp float;\nattribute vec3 vertexPosition; attribute vec4 vertexColor; uniform mat4 mvp; uniform vec3 markTime; varying float fragAlpha;\n"
    "void main(){ float birth = floor(vertexColor.r*255.0+0.5)*256.0 + floor(vertexColor.g*255.0+0.5);\n"
    " float age = mod(markTime.x - birth + 65536.0, 65536.0); float k = clamp((markTime.y - age)/markTime.z, 0.0, 1.0);\n"
    " fragAlpha = vertexColor.a*k; gl_Position = (k > 0.0) ? mvp*vec4(vertexPosition, 1.0) : vec4(2.0, 2.0, 2.0, 1.0); }\n";
static const char *mark_fs = "precision mediump float; varying float fragAlpha; void main(){ gl_FragColor = vec4(0.098, 0.098, 0.098, fragAlpha); }\n";
#else
#if defined(__EMSCRIPTEN__) || defined(BD_PHONE)
#define MARK_GLSL "#version 300 es\nprecision highp float;\n"
#else
#define MARK_GLSL "#version 330\n"
#endif
static const char *mark_vs = MARK_GLSL "in vec3 vertexPosition; in vec4 vertexColor; uniform mat4 mvp; uniform vec3 markTime; out float fragAlpha;\n"
    "void main(){ float birth = floor(vertexColor.r*255.0+0.5)*256.0 + floor(vertexColor.g*255.0+0.5);\n"
    " float age = mod(markTime.x - birth + 65536.0, 65536.0); float k = clamp((markTime.y - age)/markTime.z, 0.0, 1.0);\n"
    " fragAlpha = vertexColor.a*k; gl_Position = (k > 0.0) ? mvp*vec4(vertexPosition, 1.0) : vec4(2.0, 2.0, 2.0, 1.0); }\n";
static const char *mark_fs = MARK_GLSL "in float fragAlpha; out vec4 finalColor; void main(){ finalColor = vec4(0.098, 0.098, 0.098, fragAlpha); }\n";
#endif
static V3 mark_prev_l[4], mark_prev_r[4];
static int mark_has_prev[4];
static V3 mark_base; /* the marks' vertices are relative to this (see ro) */

static void marks_init(void)
{
    mark_mesh.vertexCount = NMARKS * 6;
    mark_mesh.triangleCount = NMARKS * 2;
    mark_mesh.vertices = calloc((size_t)NMARKS * 18, sizeof(float));
    mark_mesh.colors = calloc((size_t)NMARKS * 24, 1);
    for (int k = 0; k < MARK_COPIES; k++) {
        mark_gpu[k] = mark_mesh;
        UploadMesh(&mark_gpu[k], true);
    }
    mark_mesh = mark_gpu[0];
    flat_mat = LoadMaterialDefault();
    mark_mat = LoadMaterialDefault();
#ifdef HANDHELD
    Shader fl = LoadShaderFromMemory(flat_vs, flat_fs);
    if (fl.id != flat_mat.shader.id && fl.id > 0) flat_mat.shader = fl;
    else fprintf(stderr, "highp shader not available: the default one is used\n");
#endif
    Shader sh = LoadShaderFromMemory(mark_vs, mark_fs);
    mark_time_loc = GetShaderLocation(sh, "markTime");
    if (sh.id != mark_mat.shader.id && mark_time_loc >= 0) { mark_mat.shader = sh; mark_fades = 1; }
    else fprintf(stderr, "tyre-mark fade shader not available: marks will not fade\n");
    mark_ready = 1;
}

static void marks_clear(void)
{
    mark_next = mark_added = 0;
    mark_base = v3(0, 0, 0); /* every road starts at the world's origin */
    memset(mark_has_prev, 0, sizeof(mark_has_prev));
    if (!mark_ready) return;
    memset(mark_mesh.vertices, 0, (size_t)NMARKS * 18 * sizeof(float));
    memset(mark_mesh.colors, 0, (size_t)NMARKS * 24);
    for (int k = 0; k < MARK_COPIES; k++) {
        UpdateMeshBuffer(mark_gpu[k], 0, mark_mesh.vertices, NMARKS * 18 * (int)sizeof(float), 0);
        UpdateMeshBuffer(mark_gpu[k], 3, mark_mesh.colors, NMARKS * 24, 0);
        mark_synced[k] = mark_total;
    }
}

static void marks_step(const Car *c)
{
    if (!mark_ready) return;
    for (int i = 0; i < 4; i++) {
        const Wheel *w = &c->wheel[i];
        if (!w->on_ground || w->slip_velo <= T.mark_slip_from) { mark_has_prev[i] = 0; continue; }
        V3 half = vscale(v3(w->right_dir.x, 0, w->right_dir.z), T.wheel_width * 0.5f);
        V3 at = vadd(w->hit_point, qrot(c->body.rot, wheel_draw_off[i])); /* under the wheel as drawn */
        V3 l = vsub(at, half), r = vadd(at, half);
        l.y = route_height(&RT, l.x, l.z, NULL) + 0.03f; /* on the ground, which is not flat any more */
        r.y = route_height(&RT, r.x, r.z, NULL) + 0.03f;
        if (mark_has_prev[i]) {
            V3 q[6] = { mark_prev_l[i], mark_prev_r[i], l, mark_prev_r[i], r, l };
            unsigned char a = (unsigned char)(clampf((w->slip_velo - T.mark_slip_from) / 15.0f, 0.25f, 1.0f) * 170.0f);
            unsigned born = (unsigned)fmod(mark_clock * MARK_TICKS, 65536.0);
            float *vp = mark_mesh.vertices + (size_t)mark_next * 18;
            unsigned char *cp = mark_mesh.colors + (size_t)mark_next * 24;
            for (int k = 0; k < 6; k++) {
                Vector3 m = RV(vsub(q[k], mark_base));
                vp[k * 3] = m.x; vp[k * 3 + 1] = m.y; vp[k * 3 + 2] = m.z;
                cp[k * 4] = (unsigned char)(born >> 8); cp[k * 4 + 1] = (unsigned char)(born & 255); cp[k * 4 + 2] = 25; cp[k * 4 + 3] = a;
            }
            mark_next = (mark_next + 1) % NMARKS;
            mark_added++;
            mark_total++;
        }
        mark_prev_l[i] = l; mark_prev_r[i] = r; mark_has_prev[i] = 1;
    }
}

/* Once the car is 500 m from the marks' base, the base moves to it: every mark is shifted on the
 * processor (a few thousand additions) and each of the three copies is sent whole on its own turn,
 * as a mark's ordinary update is, so no buffer is rewritten while the frame before draws from it. */
static int mark_rebases;
static void marks_rebase(V3 to)
{
    if (!mark_ready) return;
    float dx = to.x - mark_base.x, dz = to.z - mark_base.z;
    if (dx * dx + dz * dz < 500.0f * 500.0f) return;
    for (int i = 0; i < NMARKS * 6; i++) { mark_mesh.vertices[i * 3] -= dx; mark_mesh.vertices[i * 3 + 2] += dz; } /* z is stored mirrored */
    mark_base = v3(to.x, 0, to.z);
    for (int k = 0; k < MARK_COPIES; k++) mark_synced[k] = mark_total - NMARKS - 1; /* behind by more than a ring: sent whole */
    mark_rebases++;
}

/* Pick this frame's copy and send it the quads made since its last turn (two pieces if the ring wrapped). */
static void marks_upload(int frame)
{
    if (!mark_ready) return;
    int k = frame % MARK_COPIES;
    mark_mesh = mark_gpu[k];
    mark_added = 0;
    long behind = mark_total - mark_synced[k];
    if (behind == 0) return;
    if (behind > NMARKS) { /* after a rebase (or a very long frame): the whole copy in one piece */
        UpdateMeshBuffer(mark_mesh, 0, mark_mesh.vertices, NMARKS * 18 * (int)sizeof(float), 0);
        UpdateMeshBuffer(mark_mesh, 3, mark_mesh.colors, NMARKS * 24, 0);
        mark_synced[k] = mark_total;
        return;
    }
    int n = (int)behind;
    int first = (mark_next - n + NMARKS) % NMARKS;
    int run = (first + n <= NMARKS) ? n : NMARKS - first;
    UpdateMeshBuffer(mark_mesh, 0, mark_mesh.vertices + (size_t)first * 18, run * 18 * (int)sizeof(float), first * 18 * (int)sizeof(float));
    UpdateMeshBuffer(mark_mesh, 3, mark_mesh.colors + (size_t)first * 24, run * 24, first * 24);
    if (run < n) {
        UpdateMeshBuffer(mark_mesh, 0, mark_mesh.vertices, (n - run) * 18 * (int)sizeof(float), 0);
        UpdateMeshBuffer(mark_mesh, 3, mark_mesh.colors, (n - run) * 24, 0);
    }
    mark_synced[k] = mark_total;
}

/* Each car body as a mesh uploaded once; drawn each frame with a transform. The scene shader reads the colour's alpha
 * as the share of light from the sun (session 3): about half for the car, whose faces turn. */
#define CAR_ALPHA (64 * 2 + 1)
static void car_mesh_init(CarModel *m)
{
    Mesh *cm = &m->mesh;
    cm->vertexCount = m->count * 3;
    cm->triangleCount = m->count;
    cm->vertices = malloc((size_t)m->count * 9 * sizeof(float));
    cm->colors = malloc((size_t)m->count * 12);
    for (int k = 0; k < m->count; k++) {
        const Tri *t = &tris[m->first + k];
        for (int j = 0; j < 3; j++) {
            V3 v = t->p[2 - j]; /* reversed: mirroring z flips the winding */
            float *vp = cm->vertices + (size_t)(k * 3 + j) * 3;
            unsigned char *cp = cm->colors + (size_t)(k * 3 + j) * 4;
            vp[0] = v.x; vp[1] = v.y; vp[2] = -v.z;
            cp[0] = t->r; cp[1] = t->g; cp[2] = t->b; cp[3] = t->lamp ? 255 : CAR_ALPHA;   /* build C: 255 = a lamp (the scene shader keeps it lit at night) */
        }
    }
    UploadMesh(cm, false);
    for (int w = 0; w < 4; w++) { /* the body's own wheels, in the wheel's frame (not mirrored: drawn with basis_matrix) */
        if (!m->wcount[w]) continue;
        Mesh *wm = &m->wmesh[w];
        wm->vertexCount = m->wcount[w] * 3;
        wm->triangleCount = m->wcount[w];
        wm->vertices = malloc((size_t)m->wcount[w] * 9 * sizeof(float));
        wm->colors = malloc((size_t)m->wcount[w] * 12);
        for (int k = 0; k < m->wcount[w]; k++) {
            const Tri *t = &wtris[m->wfirst[w] + k];
            for (int j = 0; j < 3; j++) {
                float *vp = wm->vertices + (size_t)(k * 3 + j) * 3;
                unsigned char *cp = wm->colors + (size_t)(k * 3 + j) * 4;
                vp[0] = t->p[j].x; vp[1] = t->p[j].y; vp[2] = t->p[j].z;
                cp[0] = t->r; cp[1] = t->g; cp[2] = t->b; cp[3] = CAR_ALPHA;
            }
        }
        UploadMesh(wm, false);
    }
}

/* Everything else in the scene is also a mesh built once. On the handheld each batch of
 * hand-fed triangles costs milliseconds in the driver, so nothing 3D is fed by hand any more. */
typedef struct { float *v; unsigned char *c; int n, cap; } MeshBuilder;
static void mb_vert(MeshBuilder *b, V3 p, Color col, int mirror)
{
    if (b->n == b->cap) {
        b->cap = b->cap ? b->cap * 2 : 256;
        b->v = realloc(b->v, (size_t)b->cap * 3 * sizeof(float));
        b->c = realloc(b->c, (size_t)b->cap * 4);
    }
    b->v[b->n * 3] = p.x; b->v[b->n * 3 + 1] = p.y; b->v[b->n * 3 + 2] = mirror ? -p.z : p.z;
    b->c[b->n * 4] = col.r; b->c[b->n * 4 + 1] = col.g; b->c[b->n * 4 + 2] = col.b; b->c[b->n * 4 + 3] = col.a;
    b->n++;
}
static void mb_quad(MeshBuilder *b, V3 p0, V3 p1, V3 p2, V3 p3, Color col, int mirror)
{
    mb_vert(b, p0, col, mirror); mb_vert(b, p1, col, mirror); mb_vert(b, p2, col, mirror);
    mb_vert(b, p0, col, mirror); mb_vert(b, p2, col, mirror); mb_vert(b, p3, col, mirror);
}
static Mesh mb_finish(MeshBuilder *b)
{
    Mesh m = { 0 };
    m.vertexCount = b->n;
    m.triangleCount = b->n / 3;
    m.vertices = b->v;
    m.colors = b->c;
    UploadMesh(&m, false);
    return m;
}
static Color shaded(Color base, V3 normal)
{
    float s = 0.5f + 0.5f * maxf(vdot(vnorm(normal), vnorm(v3(0.55f, 0.65f, 0.5f))), 0);
    return (Color){ (unsigned char)(base.r * s), (unsigned char)(base.g * s), (unsigned char)(base.b * s), base.a };
}

static Mesh wheel_mesh;
static void scene_meshes_init(void)
{
    MeshBuilder w = { 0 }, s = { 0 };
    for (int i = 0; i < 10; i++) { /* wheel: unit cylinder along x, scaled when drawn */
        float a0 = (float)i * 0.6283185f, a1 = (float)(i + 1) * 0.6283185f, am = (a0 + a1) * 0.5f;
        V3 p0 = v3(0, cosf(a0), sinf(a0)), p1 = v3(0, cosf(a1), sinf(a1));
        Color side = shaded((Color){ 60, 60, 64, CAR_ALPHA }, v3(0, cosf(am), sinf(am)));
        mb_quad(&w, v3(-1, p0.y, p0.z), v3(1, p0.y, p0.z), v3(1, p1.y, p1.z), v3(-1, p1.y, p1.z), side, 0);
        for (int e = -1; e <= 1; e += 2) {
            Color cap = { 44, 44, 48, CAR_ALPHA };
            mb_vert(&w, v3((float)e, 0, 0), cap, 0); mb_vert(&w, v3((float)e, p0.y, p0.z), cap, 0); mb_vert(&w, v3((float)e, p1.y, p1.z), cap, 0);
        }
    }
    wheel_mesh = mb_finish(&w);
    for (int i = 0; i < NMODELS; i++) { /* the body's shadow: its outline and a margin */
        CarModel *m = &models[i];
        if (!m->loaded) continue;
        float hw = m->has_outline ? m->half_w + 0.25f : 1.25f, z0 = m->has_outline ? m->rear - 0.2f : -3.3f, z1 = m->has_outline ? m->front + 0.2f : 2.5f;
        memset(&s, 0, sizeof(s));
        mb_quad(&s, v3(-hw, 0, z0), v3(hw, 0, z0), v3(hw, 0, z1), v3(-hw, 0, z1), (Color){ 0, 0, 0, 70 }, 0);
        m->shadow = mb_finish(&s);
    }
}

/* ------------------------------------------------------------------ road */
/* The real route (src/route.c): read once, its chunks built on a worker thread as the car nears them and
 * uploaded here, at most one a frame. The handling still runs on flat ground; the road only lifts and
 * tilts the car when drawn, adds the pull of the slope, drags wheels that are off the gravel, and ends the
 * run at a tree or at the bridge. */

static Run run;
static int rt_have;
static const char *rt_dir = "data/built";
static const char *erec_path = "data/records-bush-drift.txt"; /* the top ten's "seed" field holds the time to the bridge, in hundredths (0 = not reached) */
static int rt_runs;
static float rt_max_s;          /* furthest along the road this run, m */
static float rt_bot_alat;       /* --bot A: scripted runs use the road bot at A m/s2 sideways */
static float rt_finish_time;
static V3 shot_plant_at; static int shot_plant_on; /* SHOT_PLANT pictures: the camera looks at this */
#define RT_SLOTS 32
static struct { int index, tris[RT_NPARTS]; Mesh mesh[RT_NPARTS]; V3 base, centre; float radius, line_r; } rt_slot[RT_SLOTS];
static Mesh far_mesh[RT_FAR_MAX_TILES]; static V3 far_base[RT_FAR_MAX_TILES]; static int far_n, far_tris; static double far_build_ms;
static double rt_upload_ms_max;
static long rt_uploads;
static int rt_tris_drawn, rt_draw_calls, rt_shadow_tris, rt_tris_max, rt_calls_max, rt_shadow_max; static double rt_gpu_mb, rt_gpu_mb_max;
#define TERRAIN_REACH_DRAW 64.0f /* route.c's terrain reach: a chunk's plants are within this of its line */

typedef struct { int ditch, rough, rock, deep, tree; float tree_speed; int wet, wet_smash; int saw_ev; float saw_x, saw_z; int stag, staglog; float stag_speed, stag_vx, stag_vz; float grade; float edge; int offroad, held, carpark, log_broken; float log_x, log_z, log_cx, log_cz; } RtPhys;   /* carpark: off the road but no tag or timer; log_broken: a log splintered this step, here */   /* build B: how far into the map's edge; off the road; held back by the edge */   /* deep: wheels ditch_margin or more past the verge (scoring) */
static RtPhys rt_last;          /* this frame's last physics step, for the readout */
static int bot_ghost;           /* build B, the shortcut test (BOT_GHOST=1): trunks don't stop the car, so only the ground and the edge do */
static int logs_break = 1;      /* build C: 0 = logs are hard (Explorer, with the tray already full: the wood isn't wasted) */

/* ---- build B: the deer (src/world.c places them and keeps the finds); here they are drawn and found ---- */
static Mesh deer_mesh[WORLD_DEER + WORLD_EXTRA][2]; static V3 deer_base[WORLD_DEER + WORLD_EXTRA]; static int deer_tris[WORLD_DEER + WORLD_EXTRA][2], deer_meshes;
/* a deer still to find and out in the bush now (the extra five only once Quayle has his rifle back) */
static int deer_out(int d) { return !((W.found >> d) & 1) && (d < WORLD_DEER || world_extra_on()); }
static Mesh army_mesh[2]; static V3 army_base; static int army_tris[2], army_pose;   /* the follow-up: the toy army man (build C: Quayle) */
static uint32_t deer_seed_now;
static void deer_place(int fresh_seed)
{
    deer_seed_now = fresh_seed ? (uint32_t)time(NULL) * 2654435761u + (uint32_t)rt_runs : (uint32_t)T.deer_seed;
    world_place_deer(&RT, deer_seed_now);
}
static Mesh rt_upload_part(RtPart *P);
static void deer_meshes_free(void)
{
    for (int d = 0; d < WORLD_DEER + WORLD_EXTRA; d++) for (int l = 0; l < 2; l++) if (deer_mesh[d][l].vertexCount) UnloadMesh(deer_mesh[d][l]);
    memset(deer_mesh, 0, sizeof(deer_mesh)); deer_meshes = 0;
}
static void deer_meshes_init(void)   /* each deer as two small meshes (near and far), built where it stands */
{
    deer_meshes_free();
    for (int d = 0; d < WORLD_DEER + W.nextra; d++)
        for (int l = 0; l < 2; l++) {
            if (d >= W.ndeer && d < WORLD_DEER) continue;
            RtGeom g;
            route_build_model(&RT, route_template_find("sambar", W.deer[d].variant, l), W.deer[d].x, W.deer[d].z, W.deer[d].yaw, 1.0f, &g);
            deer_mesh[d][l] = rt_upload_part(&g.part[RT_PART_GROUND]); deer_tris[d][l] = g.part[RT_PART_GROUND].nverts / 3;
            deer_base[d] = v3(g.base_x, 0, g.base_z);
            route_geom_free(&g);
        }
    deer_meshes = WORLD_DEER + W.nextra;
    for (int l = 0; l < 2; l++) {
        if (army_mesh[l].vertexCount) UnloadMesh(army_mesh[l]);
        memset(&army_mesh[l], 0, sizeof(Mesh)); army_tris[l] = 0;
        if (!W.army_placed) continue;
        RtGeom g;
        int pose = W.rifle == 2 ? 1 : 0;   /* build C: looking about for his rifle, or back on duty */
        int tp = route_template_find("armyman", pose, l); if (tp < 0) tp = route_template_find("armyman", 0, l);
        route_build_model(&RT, tp, W.army_x, W.army_z, W.army_yaw + (pose ? 0.0f : 0.9f), 1.0f, &g);
        army_pose = pose;
        army_mesh[l] = rt_upload_part(&g.part[RT_PART_GROUND]); army_tris[l] = g.part[RT_PART_GROUND].nverts / 3;
        army_base = v3(g.base_x, 0, g.base_z);
        route_geom_free(&g);
    }
}
static int army_check(const Car *c)
{
    float hw = car_outline.own ? car_outline.half_w : T.body_half_w, fr = car_outline.own ? car_outline.front : T.body_front, re = car_outline.own ? car_outline.rear : T.body_rear;
    return world_army_hit(c->body.pos.x, c->body.pos.z, car_yaw(c), hw, fr, re);
}
/* The car's outline against the deer: the one it touched (now found), or -1. */
static int deer_check(const Car *c)
{
    float hw = car_outline.own ? car_outline.half_w : T.body_half_w, fr = car_outline.own ? car_outline.front : T.body_front, re = car_outline.own ? car_outline.rear : T.body_rear;
    return world_deer_hit(c->body.pos.x, c->body.pos.z, car_yaw(c), hw, fr, re);
}
/* ---- build C: Explorer's things, drawn like the deer: a mesh each, built where it stands ---- */
static float night_now = -1;      /* the darkness drawn now (0 golden hour .. 1 night), heading for W.night */
static float fire_grow[4];        /* each fire level's size as drawn (0..1), so a new one builds up */
static float fireworks_t = -1;    /* seconds since the fireworks began (-1: none this session) */
static float throw_t = -1;        /* the tray's logs flying onto the fire (seconds since) */
static int throw_level;
static float hutnight_t = -1;     /* the finale's camera orbit (seconds since it began; -1 off) */
/* play test (22 Oct): the finale is about 45 s, to Will's song (it plays on after, while you drive): one slow pan along
 * the table (the board and the camp oven on it), round the fire, the fireworks over the chimney with the moon beside them,
 * then down to the hut, the 79 and the fire */
static int hn_move;       /* the 79 to the car park for the finale (and its replay) */
#define HN_FIRE 12.0f     /* round the fire */
#define HN_UP 22.0f       /* the fireworks, the moon beside them */
#define HN_FW 20.0f       /* the fireworks begin */
#define HN_DOWN 32.0f     /* the hut, the 79 and the fire (play test: no shot of the sky alone first) */
#define HN_END 44.0f
/* build C follow-up: the wood cutscene. A load delivered cuts to a camera west of the hut, the ute parked at the start of
 * the track; the split logs stack up one by one against the hut's west wall, then the fire catches and the light goes. */
static float cut_t = -1; static int cut_level, cut_shown, cut_move_car, in_window;
#define CUT_LOG0 1.0f     /* the first log lands */
#define CUT_LOG_DT 0.28f  /* and one every this */
#define CUT_FIRE 6.0f     /* the fire catches */
static float cut_end(void) { return cut_level >= 3 ? 17.0f : 13.0f; }
static int cut_logs_now(void) { return cut_t < 0 ? W.woodpile : cut_shown + (int)clampf(floorf((cut_t - CUT_LOG0) / CUT_LOG_DT) + 1.0f, 0.0f, 15.0f); }
/* play test (22 Oct): the charcuterie cutscene. In close to the hut with all twelve: a camera by the picnic table, the esky on
 * the grass beside it; the twelve things hop out one by one onto the bare plank (the two reds last), then the board as laid
 * out, a sparkle, and a slow pan along it. */
static float bcut_t = -1; static int bcut_landed, bcut_move;
#define BCUT_ITEM0 2.0f    /* the first thing out of the esky */
#define BCUT_DT 1.15f      /* and one every this */
#define BCUT_FLY 0.9f      /* each one's hop */
#define BCUT_DONE (BCUT_ITEM0 + (HAMPER_N - 1) * BCUT_DT + BCUT_FLY + 0.6f)   /* the board as laid out */
#define BCUT_END (BCUT_DONE + 7.0f)
static Mesh cut_item_mesh[HAMPER_N], cut_esky_mesh;
/* play test: the stew's cutscene. A look into the tray at the deer, then a cut to the table: the camp oven comes down on it,
 * steaming in the lantern's light, and the camera goes slowly round it. */
static float scut_t = -1; static int scut_move;
#define SCUT_CUT 2.6f    /* from the tray to the table */
#define SCUT_END 13.0f
static int cutscene_on(void) { return cut_t >= 0 || bcut_t >= 0 || scut_t >= 0; }
static float smooth_step01(float t) { t = clamp01(t); return t * t * (3.0f - 2.0f * t); }   /* life size, in their own frames (made with the tray's) */
/* a point in the picnic table's frame: along the board, across it (+ towards the hut), up from the table top */
static V3 bcut_at(float along, float across, float up)
{
    float ax = cosf(W.picnic_yaw), az = sinf(W.picnic_yaw), nx = -az, nz = ax;
    float sd = (W.hut_x - W.picnic_x) * nx + (W.hut_z - W.picnic_z) * nz >= 0 ? 1.0f : -1.0f;
    float gy = route_height(&RT, W.picnic_x, W.picnic_z, NULL);
    return v3(W.picnic_x + ax * along + nx * across * sd, gy + 0.785f + up, W.picnic_z + az * along + nz * across * sd);
}
static V3 bcut_target(int i) { return bcut_at(-0.82f + 1.64f * (float)i / (float)(HAMPER_N - 1), 0.0f, 0.03f); }
static V3 bcut_esky(void) { return bcut_at(-0.55f, 1.05f, -0.785f); }
/* thing i now: 0 still in the esky, else 1 with where it is (in the air, or landed) and its spin */
static int bcut_item(int i, V3 *p, float *spin)
{
    float t0 = BCUT_ITEM0 + (float)i * BCUT_DT;
    if (bcut_t < t0) return 0;
    float u = clamp01((bcut_t - t0) / BCUT_FLY);
    V3 a = vadd(bcut_esky(), v3(0, 0.42f, 0)), b = bcut_target(i);
    *p = vlerp(a, b, u); p->y += sinf(u * 3.14159265f) * 0.7f; *spin = (1.0f - u) * 7.0f;
    return 1;
}
static Mesh pile_mesh[45]; static V3 pile_base[45]; static float pile_y[45]; static int pile_tris[45];
/* build D: the stags' state (more below, with their physics) */
#define STAG_FALL 1.8f
static float stag_t[48];   /* seconds since it began to fall; -1 standing; 99 down before now */
static int stag_down(int k) { return (int)((W.stag_down[k >> 6] >> (k & 63)) & 1); }
static int stag_log_taken(int k, int q) { int b = k * 3 + q; return (int)((W.stag_logs[b >> 6] >> (b & 63)) & 1); }
static Mesh hamper_mesh[HAMPER_N], fire_mesh[3], board_mesh, rifle_mesh, oven_mesh, plank_mesh, lamp_mesh; static V3 hamper_base[HAMPER_N], fire_base, board_base, rifle_base, oven_base, plank_base, lamp_base;
static int hamper_tris[HAMPER_N], fire_tris[3], board_tris, rifle_tris, oven_tris, plank_tris, lamp_tris; static V3 oven_at, lamp_at;
static Mesh trunk_mesh[2]; static V3 trunk_base[2]; static int trunk_tris[2];   /* build D: the camp's trunk, shut and open */
static void wet_pos(int k, float *x, float *z, float *yaw);
static Mesh wet_mesh[48]; static V3 wet_base[48]; static float wet_gy0[48];   /* part 2: the wet logs, built where they lie first, drawn where they are now */
static Mesh stag_mesh[48][2], staglog_mesh[48][3]; static V3 stag_base[48], staglog_base[48][3]; static float stag_gy[48]; static int stag_tris[48][2];   /* build D: the stags */
static Mesh model_at(const char *name, int variant, int lod, float x, float z, float yaw, float scale, V3 *base, int *tris)
{
    RtGeom g; Mesh m = { 0 };
    int t = route_template_find(name, variant, lod);
    if (t < 0) { *tris = 0; return m; }
    route_build_model(&RT, t, x, z, yaw, scale, &g);
    m = rt_upload_part(&g.part[RT_PART_GROUND]); *tris = g.part[RT_PART_GROUND].nverts / 3; *base = v3(g.base_x, 0, g.base_z);
    route_geom_free(&g);
    return m;
}
static void explore_meshes_init(void)
{
    for (int i = 0; i < HAMPER_N; i++) if (W.item[i].placed) hamper_mesh[i] = model_at("hamper", i, 0, W.item[i].x, W.item[i].z, W.item[i].yaw, 1.0f, &hamper_base[i], &hamper_tris[i]);
    float hy = 0;   /* the fires turn to face the chimney */
    for (int l = 0; l < 3; l++) fire_mesh[l] = model_at("campfire", l, 0, W.fire_x, W.fire_z, hy, 1.0f, &fire_base, &fire_tris[l]);
    board_mesh = model_at("board", 0, 0, W.picnic_x, W.picnic_z, W.picnic_yaw, 1.0f, &board_base, &board_tris);
    plank_mesh = model_at("board", 1, 0, W.picnic_x, W.picnic_z, W.picnic_yaw, 1.0f, &plank_base, &plank_tris);   /* the bare plank, for the cutscene */
    oven_mesh = model_at("campoven", 0, 0, W.picnic_x, W.picnic_z, W.picnic_yaw, 1.0f, &oven_base, &oven_tris);   /* in the board's frame */
    {   /* where its steam comes from: the pot's middle in the board's frame (tools/make_world.py campoven: 0.22, -0.30) */
        float cy = cosf(W.picnic_yaw), sy = sinf(W.picnic_yaw), px = 0.22f, pz = -0.30f;
        float ox = W.picnic_x + px * cy - pz * sy, oz = W.picnic_z + px * sy + pz * cy;
        oven_at = v3(ox, route_height(&RT, W.picnic_x, W.picnic_z, NULL) + 0.785f + 0.16f, oz);
        px = -0.72f; pz = -0.28f;   /* and the lantern's globe (tools/make_world.py lantern) */
        lamp_at = v3(W.picnic_x + px * cy - pz * sy, route_height(&RT, W.picnic_x, W.picnic_z, NULL) + 0.785f + 0.15f, W.picnic_z + px * sy + pz * cy);
    }
    lamp_mesh = model_at("lantern", 0, 0, W.picnic_x, W.picnic_z, W.picnic_yaw, 1.0f, &lamp_base, &lamp_tris);   /* play test: a lantern on the table */
    {   /* the woodpile: against the hut's west (chimney) gable, north of the chimney, logs end-on to the wall, ten to a layer */
        const float Cx = -6.3f, Cz = -1.5f, Ux = 0.859f, Uz = -0.511f, Vx = 0.511f, Vz = 0.859f;   /* the hut's own frame (tools/make_hut.py) */
        float yaw = atan2f(Uz, Ux);
        for (int k = 0; k < 45; k++) {
            int layer = k / 5, i = k % 5;
            float u = -0.5f - 0.03f * (float)(k % 3), v = 3.15f + 0.36f * (float)i + (layer & 1 ? 0.15f : 0.0f), jy = 0.05f * (float)((k * 7) % 5 - 2) / 2.0f;
            float x = RT.hut_x + Cx + Ux * u + Vx * v, z = RT.hut_z + Cz + Uz * u + Vz * v;
            pile_mesh[k] = model_at("splitlog", k % 3, 0, x, z, yaw + jy, 1.5f, &pile_base[k], &pile_tris[k]);
            pile_y[k] = 0.155f * (float)layer;
        }
    }
    if (W.rifle_placed) {   /* on the creek way's ground as Explorer has it (in Drift's the bank is up there) */
        int keep = RT.explore; RT.explore = 1;
        rifle_mesh = model_at("rifle", 0, 0, W.rifle_x, W.rifle_z, W.rifle_yaw, 1.0f, &rifle_base, &rifle_tris);
        RT.explore = keep;
    }
    {   /* build D: the stags standing (near and far) and their logs, on Explorer's ground */
        int keep = RT.explore; RT.explore = 1;
        for (int k = 0; k < RT.nstag; k++) {
            const struct RtStag *g = &RT.stag[k];
            float sc = g->h / (7.5f + 2.5f * (float)g->variant);   /* the model is 7.5 / 10 / 12.5 m with its splinters */
            for (int l = 0; l < 2; l++) stag_mesh[k][l] = model_at("stag", g->variant, l, g->x, g->z, (float)k * 2.3f, sc, &stag_base[k], &stag_tris[k][l]);
            stag_gy[k] = route_height(&RT, g->x, g->z, NULL);
            int tl = 0;
            for (int q = 0; q < 3; q++) { staglog_mesh[k][q] = model_at("log", 0, 0, g->log_x[q], g->log_z[q], g->log_yaw, 0.68f, &staglog_base[k][q], &tl); }
        }
        RT.explore = keep;
    }
    {   /* part 2: the wet logs, on Explorer's ground */
        int keep = RT.explore, tl = 0; RT.explore = 1;
        for (int k = 0; k < RT.ck_nwet; k++) { wet_mesh[k] = model_at("wetlog", RT.ck_wet[k].var, 0, RT.ck_wet[k].x, RT.ck_wet[k].z, RT.ck_wet[k].yaw, 1.0f, &wet_base[k], &tl); wet_gy0[k] = route_height(&RT, RT.ck_wet[k].x, RT.ck_wet[k].z, NULL); }
        RT.explore = keep;
    }
    if (W.camp_on) {   /* build D: the trunk at the cattleman's camp, likewise on Explorer's ground */
        int keep = RT.explore; RT.explore = 1;
        for (int v = 0; v < 2; v++) trunk_mesh[v] = model_at("camp", 1 + v, 0, W.trunk_x, W.trunk_z, W.trunk_yaw, 1.0f, &trunk_base[v], &trunk_tris[v]);
        RT.explore = keep;
    }
}
/* Draws them (shadow = 1: into the shadow map, within r of cw). */
static Material scene_mat;
static void explore_draw(V3 cw, int shadow, float r)
{
    Material m = shadow ? flat_mat : scene_mat;
    for (int i = 0; i < HAMPER_N; i++)
        if (hamper_mesh[i].vertexCount && !((W.hamper >> i) & 1) && !W.board && hypotf(W.item[i].x - cw.x, W.item[i].z - cw.z) < r) { DrawMesh(hamper_mesh[i], m, at_base(hamper_base[i])); if (!shadow) { rt_tris_drawn += hamper_tris[i]; rt_draw_calls++; } }
    if (!explore) return;
    int lv = W.fire_level - (throw_t >= 0 || (cut_t >= 0 && cut_t < CUT_FIRE) ? 1 : 0);   /* the new level shows once the wood is in and the fire catches */
    for (int k = 0, n = cut_logs_now(); k < n && k < 45; k++)
        if (pile_mesh[k].vertexCount && hypotf(pile_base[k].x - cw.x, pile_base[k].z - cw.z) < r) DrawMesh(pile_mesh[k], m, MatrixMultiply(at_base(pile_base[k]), MatrixTranslate(0, pile_y[k], 0)));
    if (lv > 0 && fire_mesh[lv - 1].vertexCount && hypotf(W.fire_x - cw.x, W.fire_z - cw.z) < r) DrawMesh(fire_mesh[lv - 1], m, at_base(fire_base));
    int laying = bcut_t >= 0 && bcut_t < BCUT_DONE;
    if (W.board && !laying && board_mesh.vertexCount && hypotf(W.picnic_x - cw.x, W.picnic_z - cw.z) < r) DrawMesh(board_mesh, m, MatrixMultiply(at_base(board_base), MatrixTranslate(0, 0.785f, 0)));
    if (laying && hypotf(W.picnic_x - cw.x, W.picnic_z - cw.z) < r) {   /* the charcuterie cutscene: the plank, the esky, the things hopping across */
        if (plank_mesh.vertexCount) DrawMesh(plank_mesh, m, MatrixMultiply(at_base(plank_base), MatrixTranslate(0, 0.785f, 0)));
        V3 e = bcut_esky();
        if (cut_esky_mesh.vertexCount) DrawMesh(cut_esky_mesh, m, MatrixMultiply(MatrixRotateY(W.picnic_yaw), MatrixTranslate(e.x - ro.x, e.y, -(e.z - ro.z))));
        for (int i = 0; i < HAMPER_N; i++) {
            V3 p; float sp;
            if (!cut_item_mesh[i].vertexCount || !bcut_item(i, &p, &sp)) continue;
            DrawMesh(cut_item_mesh[i], m, MatrixMultiply(MatrixRotateY(W.picnic_yaw + sp), MatrixTranslate(p.x - ro.x, p.y, -(p.z - ro.z))));
        }
    }
    if (W.rifle == 0 && rifle_mesh.vertexCount && hypotf(W.rifle_x - cw.x, W.rifle_z - cw.z) < r) {   /* part 2: it floats above its beacon, turning slowly, so it reads as a rifle */
        int keep = RT.explore; RT.explore = 1; float gy = route_height(&RT, W.rifle_x, W.rifle_z, NULL); RT.explore = keep;
        float tm = (float)GetTime(); V3 p = v3(W.rifle_x - ro.x, gy, -(W.rifle_z - ro.z));
        Matrix spin = MatrixMultiply(MatrixMultiply(MatrixMultiply(MatrixTranslate(-p.x, -p.y, -p.z), MatrixScale(1.8f, 1.8f, 1.8f)), MatrixRotateY(tm * 0.8f)), MatrixTranslate(p.x, p.y + 1.45f + 0.1f * sinf(tm * 2.0f), p.z));   /* larger than life up there */
        DrawMesh(rifle_mesh, m, MatrixMultiply(at_base(rifle_base), spin));
    }
    if (W.camp_on && trunk_mesh[W.trunk ? 1 : 0].vertexCount && RT.creek_stream && hypotf(W.trunk_x - cw.x, W.trunk_z - cw.z) < r) DrawMesh(trunk_mesh[W.trunk ? 1 : 0], m, at_base(trunk_base[W.trunk ? 1 : 0]));
    if (RT.creek_stream) for (int k = 0; k < RT.ck_nwet; k++) {   /* part 2: the wet logs where they are now (moved: turned about where they lay, and carried) */
        if (W.wet_state[k] == 2 || !wet_mesh[k].vertexCount) continue;
        float lx, lz, ly; wet_pos(k, &lx, &lz, &ly);
        if (hypotf(lx - cw.x, lz - cw.z) > r) continue;
        const struct RtWet *h0 = &RT.ck_wet[k];
        float gy = route_height(&RT, lx, lz, NULL) - wet_gy0[k];
        V3 o = v3(h0->x - ro.x, wet_gy0[k], -(h0->z - ro.z));
        Matrix mv = MatrixMultiply(MatrixMultiply(MatrixTranslate(-o.x, -o.y, -o.z), MatrixRotateY(ly - h0->yaw)), MatrixTranslate(lx - ro.x, wet_gy0[k] + gy, -(lz - ro.z)));
        DrawMesh(wet_mesh[k], m, MatrixMultiply(at_base(wet_base[k]), mv));
    }
    for (int k = 0; k < RT.nstag; k++) {   /* build D: the stags: standing, falling (turned about the foot), or down as logs */
        const struct RtStag *g = &RT.stag[k];
        float d = hypotf(g->x - cw.x, g->z - cw.z);
        if (d > r || (shadow && d > r)) continue;
        if (g->where == 2 && !RT.creek_stream) continue;   /* the creek way's, only while it's there */
        if (!stag_down(k)) {
            int l = d < 140.0f ? 0 : 1;
            if (stag_mesh[k][l].vertexCount) { DrawMesh(stag_mesh[k][l], m, at_base(stag_base[k])); if (!shadow) { rt_tris_drawn += stag_tris[k][l]; rt_draw_calls++; } }
        } else if (stag_t[k] >= 0 && stag_t[k] < STAG_FALL) {
            float u = stag_t[k] / STAG_FALL, ang = 1.53f * u * u;   /* slow to start, then fast */
            V3 f = v3(g->x - ro.x, stag_gy[k], -(g->z - ro.z)), ax = v3(-sinf(g->fall), 0, -cosf(g->fall));
            Matrix rot = MatrixMultiply(MatrixMultiply(MatrixTranslate(-f.x, -f.y, -f.z), MatrixRotate((Vector3){ ax.x, ax.y, ax.z }, -ang)), MatrixTranslate(f.x, f.y, f.z));
            if (stag_mesh[k][0].vertexCount) DrawMesh(stag_mesh[k][0], m, MatrixMultiply(at_base(stag_base[k]), rot));
        } else for (int q = 0; q < 3; q++)
            if (!stag_log_taken(k, q) && staglog_mesh[k][q].vertexCount) DrawMesh(staglog_mesh[k][q], m, at_base(staglog_base[k][q]));
    }
    if (lamp_mesh.vertexCount && hypotf(W.picnic_x - cw.x, W.picnic_z - cw.z) < r) DrawMesh(lamp_mesh, m, MatrixMultiply(at_base(lamp_base), MatrixTranslate(0, 0.785f, 0)));
    if ((W.stew || getenv("SHOT_STEW")) && oven_mesh.vertexCount && !(scut_t >= 0 && scut_t < SCUT_CUT) && hypotf(W.picnic_x - cw.x, W.picnic_z - cw.z) < r) {
        float drop = scut_t >= 0 ? 0.9f * (1.0f - smooth_step01((scut_t - SCUT_CUT - 0.3f) / 0.7f)) : 0.0f;   /* the stew's cutscene: it comes down onto the table */
        DrawMesh(oven_mesh, m, MatrixMultiply(at_base(oven_base), MatrixTranslate(0, 0.785f + drop, 0)));
    }
}
static const Color sky_colour = { 227, 199, 160, 255 };

static void rt_slot_free(int i)
{
    if (rt_slot[i].index >= 0) for (int p = 0; p < RT_NPARTS; p++) if (rt_slot[i].mesh[p].vertexCount > 0) UnloadMesh(rt_slot[i].mesh[p]);
    memset(&rt_slot[i], 0, sizeof(rt_slot[i]));
    rt_slot[i].index = -1;
}
static void rt_slots_clear(void) { for (int i = 0; i < RT_SLOTS; i++) rt_slot_free(i); }

/* One part of a chunk onto the graphics chip (the arrays stay with the caller, who frees them). */
static Mesh rt_upload_part(RtPart *P)
{
    Mesh m = { 0 };
    if (P->nverts == 0) return m;
    m.vertexCount = P->nverts; m.triangleCount = P->nverts / 3;
    m.vertices = P->v; m.texcoords = P->t; m.colors = P->c;
    UploadMesh(&m, false);
    m.vertices = NULL; m.texcoords = NULL; m.colors = NULL; /* on the graphics chip now */
    return m;
}

/* Each frame: say where the car is, drop chunks no longer wanted, take at most one finished one. */
static void rt_frame(V3 at)
{
    route_stream_want(at.x, at.z, T.view_radius);
    for (int i = 0; i < RT_SLOTS; i++)
        if (rt_slot[i].index >= 0 && !route_stream_wanted(rt_slot[i].index)) rt_slot_free(i);
    RtGeom g;
    if (!route_stream_take(&g)) return;
    double t0 = GetTime();
    int free_slot = -1;
    for (int i = 0; i < RT_SLOTS; i++) {
        if (rt_slot[i].index == g.index) rt_slot_free(i);
        if (rt_slot[i].index < 0 && free_slot < 0) free_slot = i;
    }
    if (free_slot < 0) { route_geom_free(&g); return; }
    for (int p = 0; p < RT_NPARTS; p++) { rt_slot[free_slot].mesh[p] = rt_upload_part(&g.part[p]); rt_slot[free_slot].tris[p] = g.part[p].nverts / 3; }
    route_geom_free(&g);
    rt_slot[free_slot].index = g.index;
    rt_slot[free_slot].base = v3(g.base_x, 0, g.base_z);
    float cx, cz, r; route_chunk_centre(&RT, g.index, &cx, &cz, &r);
    rt_slot[free_slot].centre = v3(cx, 0, cz); rt_slot[free_slot].radius = r; rt_slot[free_slot].line_r = route_chunk_line_r(&RT, g.index);
    double ms = (GetTime() - t0) * 1000.0;
    if (ms > rt_upload_ms_max) rt_upload_ms_max = ms;
    rt_uploads++;
}

/* The far land: built once when the window opens (route.c), a mesh per tile. */
static void far_init(void)
{
    if (!rt_have || T.far_land == 0) return;
    static RtGeom tiles[RT_FAR_MAX_TILES];
    double t0 = GetTime();
    int n = route_far_build(&RT, TextFormat("%s/far.bin", rt_dir), tiles, RT_FAR_MAX_TILES);
    for (int i = 0; i < n; i++) {
        far_mesh[i] = rt_upload_part(&tiles[i].part[RT_PART_GROUND]);
        far_base[i] = v3(tiles[i].base_x, 0, tiles[i].base_z);
        far_tris += tiles[i].part[RT_PART_GROUND].nverts / 3;
        route_geom_free(&tiles[i]);
    }
    far_n = n;
    far_build_ms = (GetTime() - t0) * 1000.0;
    fprintf(stderr, "far land: %d tiles, %d triangles, built in %.0f ms\n", far_n, far_tris, far_build_ms);
}


/* ---- build E: Will's chainsaw (Explorer). Each firewood log has a cut meter: every pass over it adds (speed factor x
 * sharpness factor), and at full it splits. The sweet spot is 15-35 km/h; slower is a nudge, faster cuts less and wears the
 * chain more. Sharp (60%+) splits in one pass, 30-60% takes two, under 30% three; at 0 it won't cut. Each pass wears the
 * chain (more in creek gravel and on the dead trees' logs; half after a razor sharpening). Drift's logs are as they were. ---- */
static int saw_log = -1; static float saw_log_t = 1;
static int saw_key[512], saw_nkey; static float saw_cutv[512];
static float *saw_cut_of(int key)
{
    for (int i = 0; i < saw_nkey; i++) if (saw_key[i] == key) return &saw_cutv[i];
    if (saw_nkey == 512) { saw_nkey = 0; }
    saw_key[saw_nkey] = key; saw_cutv[saw_nkey] = 0; return &saw_cutv[saw_nkey++];
}
static float saw_speed_factor(float v)
{
    float k = v * 3.6f;
    if (k < 10.0f) return 0;
    if (k < 15.0f) return (k - 10.0f) / 5.0f;
    if (k <= 35.0f) return 1.0f;
    if (k <= 50.0f) return 0.6f;
    return 0.25f;
}
static float saw_sharp_factor(void) { return W.saw <= 0 ? 0.0f : W.saw >= 60.0f ? 1.0f : W.saw >= 30.0f ? 0.5f : 0.34f; }
/* a pass over a log; 1 = it split (ev 1: in one pass, 2: after more); 0 = not yet (ev 3 cut some, 4 the saw won't cut, 5 too slow) */
static int saw_pass(int key, float v, int dirty, int *ev)
{
    float *cut = saw_cut_of(key), sf = saw_speed_factor(v), add = sf * saw_sharp_factor(), kmh = v * 3.6f;
    float wear = T.saw_wear * (1.0f + fmaxf(kmh - 35.0f, 0.0f) / 20.0f) * (dirty ? 1.5f : 1.0f) * (W.razor > 0 ? 0.5f : 1.0f);
    if (sf > 0 && W.saw > 0) W.saw = fmaxf(W.saw - wear, 0.0f), W.dirty = 1;
    int first = *cut <= 0.0f;
    *cut += add;
    if (*cut >= 0.999f) {
        if (first) { W.streak++; if (W.streak >= 10) world_earn(ACH_CLEAN_CUTS); } else W.streak = 0;
        if (W.razor > 0) W.razor--;
        *cut = -1e9f;   /* gone */
        *ev = first ? 1 : 2; return 1;
    }
    if (sf > 0 && add > 0) W.streak = 0;
    *ev = W.saw <= 0 ? 4 : sf <= 0 ? 5 : 3;
    return 0;
}
/* ---- build D: the stags, standing dead gums in Explorer. Hit one at stag_speed or more and it creaks, topples the way it
 * was placed to fall (away from the road or the creek's line), lands with a crash and lies as three firewood logs; slower, a
 * thud as a trunk. Down till a new day (W.stag_down; their logs taken, W.stag_logs). Not in Drift at all. ---- */
static int stag_box_touch(float px, float pz, float r, float cx, float cz, float yaw, float hw, float fr, float re)
{
    float fx = sinf(yaw), fz = cosf(yaw), rx = cosf(yaw), rz = -sinf(yaw), dx = px - cx, dz = pz - cz;
    float lf = dx * fx + dz * fz, lr = dx * rx + dz * rz;
    float of = lf > fr ? lf - fr : (lf < re ? re - lf : 0), orr = fmaxf(fabsf(lr) - hw, 0);
    return of * of + orr * orr < r * r;
}
static void stag_physics(Car *c, RtPhys *ph, float hw, float fr, float re)
{
    float x = c->body.pos.x, z = c->body.pos.z, yaw = car_yaw(c);
    for (int k = 0; k < RT.nstag; k++) {
        const struct RtStag *g = &RT.stag[k];
        float dx = x - g->x, dz = z - g->z;
        if (dx * dx + dz * dz > 400.0f) continue;
        if (stag_down(k)) {   /* its logs, once it's down */
            if (stag_t[k] >= 0 && stag_t[k] < STAG_FALL) continue;
            float ax = cosf(g->log_yaw), az = sinf(g->log_yaw);
            for (int q = 0; q < 3 && ph->staglog < 0; q++) {
                if (stag_log_taken(k, q)) continue;
                for (float f = -1.1f; f <= 1.11f; f += 1.1f) if (stag_box_touch(g->log_x[q] + ax * f, g->log_z[q] + az * f, 0.35f, x, z, yaw, hw, fr, re)) { ph->staglog = k * 3 + q; break; }
            }
            continue;
        }
        if (!stag_box_touch(g->x, g->z, g->r, x, z, yaw, hw, fr, re)) continue;
        float d = fmaxf(hypotf(dx, dz), 1e-3f), into = -(c->body.vel.x * dx + c->body.vel.z * dz) / d;
        ph->stag = k; ph->stag_speed = vlen(c->body.vel); ph->stag_vx = c->body.vel.x; ph->stag_vz = c->body.vel.z;
        if (into > 0) { c->body.vel.x += into * dx / d; c->body.vel.z += into * dz / d; }
        c->body.pos.x += 0.03f * dx / d; c->body.pos.z += 0.03f * dz / d;
        return;
    }
}

/* ---- build D, part 2: the creek's wet logs. Lying in the water; the ute shoves one aside when it's slower than wet_smash and
 * smashes it faster. Never firewood ("WET LOG"). Where they've gone (W.wet_*) lasts till a new day. ---- */
static float wet_vx[48], wet_vz[48], wet_w[48], wet_push[48]; static int wet_seen[48];
static float wet_len(int k) { return RT.ck_wet[k].var ? 1.3f : 1.6f; }   /* half lengths (wetlog v0 3.2 m, v1 2.6 m) */
static void wet_pos(int k, float *x, float *z, float *yaw)
{
    if (W.wet_state[k] == 1) { *x = W.wet_x[k]; *z = W.wet_z[k]; *yaw = W.wet_yaw[k]; }
    else { *x = RT.ck_wet[k].x; *z = RT.ck_wet[k].z; *yaw = RT.ck_wet[k].yaw; }
}
static void wet_physics(Car *c, RtPhys *ph, float hw, float fr, float re)
{
    ph->wet = -1; ph->wet_smash = 0;
    float x = c->body.pos.x, z = c->body.pos.z, yaw = car_yaw(c);
    for (int k = 0; k < RT.ck_nwet; k++) {
        if (W.wet_state[k] == 2) continue;
        float lx, lz, ly; wet_pos(k, &lx, &lz, &ly);
        /* the log slides on and slows (gravel and water), turning as it goes */
        if (wet_vx[k] != 0 || wet_vz[k] != 0 || wet_w[k] != 0) {
            float sp = hypotf(wet_vx[k], wet_vz[k]), dv = 4.0f * FIXED_DT, keep = sp > dv ? (sp - dv) / sp : 0.0f;
            wet_vx[k] *= keep; wet_vz[k] *= keep; wet_w[k] *= 1.0f - clamp01(3.0f * FIXED_DT);
            W.wet_x[k] = lx + wet_vx[k] * FIXED_DT; W.wet_z[k] = lz + wet_vz[k] * FIXED_DT; W.wet_yaw[k] = ly + wet_w[k] * FIXED_DT; W.wet_state[k] = 1; W.dirty = 1;
            if (fabsf(wet_vx[k]) + fabsf(wet_vz[k]) < 0.02f && fabsf(wet_w[k]) < 0.02f) wet_vx[k] = wet_vz[k] = wet_w[k] = 0;
            wet_pos(k, &lx, &lz, &ly);
        }
        float dx = lx - x, dz = lz - z;
        if (dx * dx + dz * dz > 100.0f) continue;
        float ax = cosf(ly), az = sinf(ly), L = wet_len(k); int hit = -99;
        for (int q = -2; q <= 2 && hit == -99; q++) { float f = L * (float)q / 2.0f; if (stag_box_touch(lx + ax * f, lz + az * f, 0.24f, x, z, yaw, hw, fr, re)) hit = q; }
        if (hit == -99) continue;
        float sp = vlen(c->body.vel);
        ph->wet = k;
        if (sp >= T.wet_smash) { W.wet_state[k] = 2; W.dirty = 1; ph->wet_smash = 1; c->body.vel = vscale(c->body.vel, 0.85f); return; }
        /* shoved: the log takes the ute's speed where it's touched, the ute loses a little; off-centre, it turns */
        float f = L * (float)hit / 2.0f, px = lx + ax * f, pz = lz + az * f;
        /* thrown ahead and aside, away from the ute's middle line, so it clears the way rather than riding on the bull bar */
        float fx = sinf(yaw), fz = cosf(yaw), side = (px - x) * cosf(yaw) - (pz - z) * sinf(yaw), sgn = side >= 0 ? 1.0f : -1.0f, push = fmaxf(sp, 1.5f);
        wet_vx[k] = fx * push * 1.3f + cosf(yaw) * sgn * 2.0f; wet_vz[k] = fz * push * 1.3f - sinf(yaw) * sgn * 2.0f;
        wet_push[k] += FIXED_DT;
        if (wet_push[k] > 1.0f) {   /* kept on shoving (it's jammed against something): it rolls off to the side, out of the way */
            float off = (c->body.vel.x * cosf(yaw) - c->body.vel.z * sinf(yaw)) >= 0 ? -1.0f : 1.0f; off = side >= 0 ? 1.0f : -1.0f;
            W.wet_x[k] = lx + cosf(yaw) * off * 3.0f; W.wet_z[k] = lz - sinf(yaw) * off * 3.0f; W.wet_yaw[k] = yaw; W.wet_state[k] = 1; W.dirty = 1;
            wet_vx[k] = wet_vz[k] = wet_w[k] = 0; wet_push[k] = 0;
        }
        wet_w[k] = clampf(f * ((-az) * c->body.vel.x + ax * c->body.vel.z) * 0.6f / fmaxf(L * L, 0.5f), -3.0f, 3.0f);
        c->body.vel = vscale(c->body.vel, 1.0f - 1.0f * FIXED_DT);
        return;
    }
}
/* each physics step: the falls go on (sound: game), and the state follows W (a new day, a reset, a file read) */
static void stag_crash(int k);
static void stag_step(float dt, int sound)
{
    for (int k = 0; k < RT.nstag; k++) {
        if (!stag_down(k)) { stag_t[k] = -1; continue; }
        if (stag_t[k] < 0) { stag_t[k] = 99.0f; continue; }
        if (stag_t[k] >= 90.0f) continue;
        float t0 = stag_t[k]; stag_t[k] += dt;
        if (sound && t0 < STAG_FALL && stag_t[k] >= STAG_FALL) stag_crash(k);
    }
}
/* what a step's stag contact means: 1 it topples, 2 a thud, 3 a log into the tray (0 nothing) */
static int stag_event(Car *c, const RtPhys *ph)
{
    if (ph->stag >= 0) {
        int k = ph->stag;
        if (ph->stag_speed >= T.stag_speed) {
            W.stag_down[k >> 6] |= 1ull << (k & 63); stag_t[k] = 0; W.dirty = 1;
            c->body.vel = v3(ph->stag_vx * T.stag_keep, 0, ph->stag_vz * T.stag_keep);   /* it gives way: the ute carries on, slower */
            return 1;
        }
        if (ph->stag_speed >= T.tree_min_speed) {   /* a thud: a small push back, the spin taken out */
            float dx = c->body.pos.x - RT.stag[k].x, dz = c->body.pos.z - RT.stag[k].z, d = fmaxf(hypotf(dx, dz), 1e-3f);
            c->body.vel = v3(dx / d * 0.6f, 0, dz / d * 0.6f); c->body.angvel = v3(0, 0, 0);
            return 2;
        }
        return 0;
    }
    if (ph->staglog >= 0 && W.tray_wood < (int)T.wood_full) {
        int b = ph->staglog;
        if (T.saw_on != 0) {   /* build E: the saw for these too (dirty: they've been on the ground) */
            if (1000000 + b == saw_log) { saw_log_t = 0; return 0; }
            saw_log = 1000000 + b; saw_log_t = 0; int ev = 0;
            if (!saw_pass(1000000 + b, vlen(c->body.vel), 1, &ev)) { c->body.vel = vscale(c->body.vel, ev == 5 ? 0.85f : 0.65f); ((RtPhys *)ph)->saw_ev = ev; return 6; }
            ((RtPhys *)ph)->saw_ev = ev;
        }
        W.stag_logs[b >> 6] |= 1ull << (b & 63); W.tray_wood++; W.dirty = 1;
        c->body.vel = vscale(c->body.vel, T.log_keep);
        return 3;
    }
    return 0;
}

/* ---- build D: the sprung body (body_physics). The handling is still the flat-plane car (src/car.c); over it, a body on four
 * springs: its height, pitch and roll, each wheel feeling the ground under it. A wheel's load scales its grip (car.grip_lost),
 * so a light wheel slides and a wheel in the air has none; the ground pushes back at each wheel along its slope (a log's face
 * slows you, a crest throws you); the belly and the bumpers ground out on big things (stuck), and too much roll or pitch is a
 * rollover. Off (body_physics = 0), everything is exactly as before. ---- */
typedef struct {
    int init; float y, vy, pitch, pv, roll, rv;   /* body origin height (world); pitch nose up +; roll right side up + (radians) */
    float s_prev[4], F[4], wheel_y[4], bs_prev[5], g_prev[4], bg_prev[5]; int contact[4], ncontact, belly, have_g;
    float last_t; V3 last_pos;
    float air_t, air_peak, air_from_x, air_from_z, rolled_t; int rolled;
    float last_air_h, last_air_d, last_air_t;   /* the last jump: height, distance, time in the air */
    float knock[4], air_vy, land_v; int land_ev;   /* build E: the hardest knock at each corner this step (m/s into the bump stop); the speed down at the last landing */
} BodyState;
static BodyState bp;
#define BP_DROOP 0.22f
#define BP_BUMP 0.24f   /* a 79's travel up (plus the tyre's give) */
static void body_init(const Car *c)
{
    float h = 0;
    for (int i = 0; i < 4; i++) { V3 w = vadd(c->body.pos, qrot(c->body.rot, anchors[i])); bp.wheel_y[i] = route_height(&RT, w.x, w.z, NULL); h += 0.25f * bp.wheel_y[i]; }
    memset(&bp, 0, sizeof(bp));
    bp.init = 1; bp.y = h + 0.75f; bp.last_t = c->time; bp.last_pos = c->body.pos;
    for (int i = 0; i < 4; i++) { V3 w = vadd(c->body.pos, qrot(c->body.rot, anchors[i])); bp.wheel_y[i] = route_height(&RT, w.x, w.z, NULL); bp.contact[i] = 1; }
    bp.ncontact = 4;
}
static void body_step(Car *c, RtPhys *ph)
{
    (void)ph;
    if (!bp.init || c->time < bp.last_t - 1e-4f || hypotf(c->body.pos.x - bp.last_pos.x, c->body.pos.z - bp.last_pos.z) > 4.0f) body_init(c);
    float yaw = car_yaw(c), sy = sinf(yaw), cy = cosf(yaw);
    V3 f = v3(sy, 0, cy), r = v3(cy, 0, -sy);
    float m = T.car_mass, gr = 9.81f;
    float zf = 0.5f * (anchors[FL].z + anchors[FR].z), zr = 0.5f * (anchors[RL].z + anchors[RR].z), zc = zr + T.car_front_share * (zf - zr);
    float ax[4], az[4], g[4], gx[4], gz[4], W[4];
    const float e = 0.4f;
    for (int i = 0; i < 4; i++) {
        ax[i] = anchors[i].x; az[i] = anchors[i].z - zc;
        float x = c->body.pos.x + r.x * anchors[i].x + f.x * anchors[i].z, z = c->body.pos.z + r.z * anchors[i].x + f.z * anchors[i].z;
        g[i] = route_height(&RT, x, z, NULL);
        gx[i] = clampf((route_height(&RT, x + e, z, NULL) - route_height(&RT, x - e, z, NULL)) / (2 * e), -1.5f, 1.5f);
        gz[i] = clampf((route_height(&RT, x, z + e, NULL) - route_height(&RT, x, z - e, NULL)) / (2 * e), -1.5f, 1.5f);
        W[i] = m * gr * 0.5f * (i < 2 ? T.car_front_share : 1.0f - T.car_front_share);
    }
    /* the belly (three points along its middle) and the bumpers: under the body origin by this much */
    float fr0 = car_outline.own ? car_outline.front : T.body_front, re0 = car_outline.own ? car_outline.rear : T.body_rear;
    const float bzp[5] = { -1.2f, 0.0f, 1.2f, fr0 - 0.15f, re0 + 0.15f }, bh[5] = { T.belly_h, T.belly_h, T.belly_h, T.bumper_h, T.bumper_h };
    float bg[5], bgx[5], bgz[5];
    for (int j = 0; j < 5; j++) {
        float x = c->body.pos.x + f.x * bzp[j], z = c->body.pos.z + f.z * bzp[j];
        bg[j] = route_height(&RT, x, z, NULL);
        if (explore && RT.ck_nbit) bg[j] += route_creek_bump_r(&RT, x, z, 0.05f) - route_creek_bump(&RT, x, z);   /* part 2: the belly feels the logs' and rocks' real shapes, not a tyre's */
        bgx[j] = clampf((route_height(&RT, x + e, z, NULL) - route_height(&RT, x - e, z, NULL)) / (2 * e), -2.0f, 2.0f);
        bgz[j] = clampf((route_height(&RT, x, z + e, NULL) - route_height(&RT, x, z - e, NULL)) / (2 * e), -2.0f, 2.0f);
    }
    const int ns = 8; float h = FIXED_DT / (float)ns;
    for (int i = 0; i < 4; i++) bp.knock[i] = 0;   /* build E */
    float Fsum[4] = { 0 }, Fb[5] = { 0 };
    if (!bp.have_g) { for (int i = 0; i < 4; i++) { bp.g_prev[i] = g[i]; bp.s_prev[i] = (g[i] + 0.75f) - (bp.y + sinf(bp.pitch) * az[i] + sinf(bp.roll) * ax[i]); } for (int j = 0; j < 5; j++) bp.bg_prev[j] = bg[j]; bp.have_g = 1; }
    for (int st = 0; st < ns; st++) {
        float sp = sinf(bp.pitch), sr = sinf(bp.roll), Ft = 0, Mp = 0, Mr = 0, u = (float)(st + 1) / (float)ns;   /* the ground moves under the wheel through the step, not all at once */
        for (int i = 0; i < 4; i++) {
            float gi = bp.g_prev[i] + (g[i] - bp.g_prev[i]) * u;
            float corner = bp.y + sp * az[i] + sr * ax[i], s_ = (gi + 0.75f) - corner, ds = (s_ - bp.s_prev[i]) / h, F = 0;
            bp.s_prev[i] = s_;
            if (s_ > -BP_DROOP) {
                float damp = clampf(T.susp_c * ds, -1.5f * W[i], 2.5f * W[i]);   /* dampers blow off: no huge kick from a sharp edge */
                F = fmaxf(W[i] + T.susp_k * s_ + damp, 0.0f);
                if (s_ > BP_BUMP) F += T.susp_k * 5.0f * (s_ - BP_BUMP) + T.susp_c * 2.0f * fmaxf(ds, 0.0f);   /* the bump stop: progressive rubber */
                F = fminf(F, 4.5f * W[i]);   /* a tyre and its bump stop can only take so much: past it, the wheel rides up the rock */
            }
            bp.F[i] = F; Fsum[i] += F / (float)ns;
            Ft += F; Mp += F * az[i]; Mr += F * ax[i];
        }
        for (int j = 0; j < 5; j++) {
            float under = bp.y + sp * (bzp[j] - zc) - bh[j], pen = bp.bg_prev[j] + (bg[j] - bp.bg_prev[j]) * u - under, rate = (pen - bp.bs_prev[j]) / h;
            bp.bs_prev[j] = pen;
            float Fj = pen > 0 ? fmaxf(T.belly_k * pen + T.belly_k * 0.05f * fmaxf(rate, 0.0f), 0.0f) : 0.0f;
            Fb[j] = Fj; Ft += Fj; Mp += Fj * (bzp[j] - zc);
        }
        float ay = Ft / m - gr;
        bp.vy += ay * h; bp.y += bp.vy * h;
        bp.pv += (Mp / T.body_pitch_i - 1.5f * bp.pv) * h; bp.pitch += bp.pv * h;
        bp.rv += (Mr / T.body_roll_i - 4.0f * bp.rv) * h; bp.roll += bp.rv * h;
        /* hard contact: past full travel the wheel can't go further up into the body. The corner takes the ground's rise (an
         * inelastic knock), and is put back at full travel: hitting a log or a rock fast throws the body up (a launch) */
        for (int i = 0; i < 4; i++) {
            float gi = bp.g_prev[i] + (g[i] - bp.g_prev[i]) * u, corner = bp.y + sinf(bp.pitch) * az[i] + sinf(bp.roll) * ax[i], s_ = (gi + 0.75f) - corner, smax = BP_BUMP + 0.08f;
            if (s_ <= smax) continue;
            bp.knock[i] = fmaxf(bp.knock[i], fmaxf((s_ - bp.s_prev[i]) / h, 0.0f));   /* build E: recorded only */
            float close = fminf(fmaxf((s_ - bp.s_prev[i]) / h, 0.0f) + (s_ - smax) / h * 0.5f, T.knock_max);
            float meff = 1.0f / (1.0f / m + az[i] * az[i] / T.body_pitch_i + ax[i] * ax[i] / T.body_roll_i), J = meff * close;
            bp.vy += J / m; bp.pv += J * az[i] / T.body_pitch_i; bp.rv += J * ax[i] / T.body_roll_i;
            float dpos = (s_ - smax) * 0.5f; bp.y += dpos; bp.s_prev[i] = s_ - dpos;
        }
    }
    if (getenv("BOT_KNOCKS")) { for (int i = 0; i < 4; i++) if (bp.knock[i] > 1.0f) printf("   knock %d %.2f m/s at %.0f km/h\n", i, bp.knock[i], vlen(c->body.vel) * 3.6f); if (bp.land_ev) { printf("   landing %.2f m/s\n", bp.land_v); bp.land_ev = 0; } }
    for (int i = 0; i < 4; i++) bp.g_prev[i] = g[i];
    for (int j = 0; j < 5; j++) bp.bg_prev[j] = bg[j];
    /* the ground's push at each wheel and under the belly, along its slope (on even ground: the old slope pull) */
    float ahx = 0, ahz = 0, belly = 0;
    bp.ncontact = 0;
    for (int i = 0; i < 4; i++) {
        bp.contact[i] = Fsum[i] > 1.0f; bp.ncontact += bp.contact[i];
        ahx -= Fsum[i] / m * gx[i] * T.grade_gravity; ahz -= Fsum[i] / m * gz[i] * T.grade_gravity;
        float load = Fsum[i] / fmaxf(W[i], 1.0f);
        c->wheel[i].grip_lost = 1.0f - clamp01(load / fmaxf(T.grip_light, 0.01f));   /* grip fades only as a wheel goes light (in the air: none); the handling's own load transfer does the rest */
        float corner = bp.y + sinf(bp.pitch) * az[i] + sinf(bp.roll) * ax[i];
        bp.wheel_y[i] = fmaxf(g[i], corner - 0.75f - BP_DROOP);   /* the wheel's foot: on the ground, or hanging */
    }
    for (int j = 0; j < 5; j++) { ahx -= Fb[j] / m * bgx[j]; ahz -= Fb[j] / m * bgz[j]; belly += Fb[j]; }
    c->body.vel.x += ahx * FIXED_DT; c->body.vel.z += ahz * FIXED_DT;
    bp.belly = belly > 1.0f;
    if (bp.belly) {   /* grounded on its belly: it drags (stuck, if the wheels can't push it off) */
        float sp = hypotf(c->body.vel.x, c->body.vel.z), dv = T.belly_mu * belly / m * FIXED_DT;
        if (sp > 1e-3f) { float keep = sp > dv ? (sp - dv) / sp : 0.0f; c->body.vel.x *= keep; c->body.vel.z *= keep; }
        c->body.angvel.y *= 0.9f;
    }
    /* in the air: how high and how far */
    if (bp.ncontact == 0) {
        if (bp.air_t == 0) { bp.air_from_x = c->body.pos.x; bp.air_from_z = c->body.pos.z; bp.air_peak = 0; bp.air_vy = 0; }
        bp.air_vy = fminf(bp.air_vy, bp.vy);   /* build E */
        bp.air_t += FIXED_DT;
        float lo = 1e9f; for (int i = 0; i < 4; i++) lo = fminf(lo, bp.y + sinf(bp.pitch) * az[i] + sinf(bp.roll) * ax[i] - 0.75f - BP_DROOP - g[i]);
        bp.air_peak = fmaxf(bp.air_peak, lo);
    } else if (bp.air_t > 0) {
        if (bp.air_t > 0.15f) { bp.land_v = -bp.air_vy; bp.land_ev = 1; bp.last_air_t = bp.air_t; bp.last_air_h = bp.air_peak; bp.last_air_d = hypotf(c->body.pos.x - bp.air_from_x, c->body.pos.z - bp.air_from_z); }
        bp.air_t = 0;
    }
    /* over: past this much roll or pitch it goes (it settles on its side or roof; the game puts it back) */
    if (!bp.rolled && (fabsf(bp.roll) > T.roll_over || fabsf(bp.pitch) > T.roll_over)) { bp.rolled = 1; bp.rolled_t = 0; }
    if (bp.rolled) {
        bp.rolled_t += FIXED_DT;
        float want = bp.roll >= 0 ? 1.75f : -1.75f;
        bp.roll += (want - bp.roll) * clamp01(FIXED_DT * 3.0f); bp.rv = 0; bp.pv *= 0.9f;
        c->body.vel = vscale(c->body.vel, 0.9f); c->body.angvel = vscale(c->body.angvel, 0.85f);
        for (int i = 0; i < 4; i++) c->wheel[i].grip_lost = 1.0f;
    }
    if (getenv("BOT_BODY") && (bp.ncontact < 4 || bp.belly || (getenv("BOT_BODY")[0] == 'a' && c->time > 9.0f && c->time < 11.6f))) { RtNear n = route_near(&RT, c->body.pos.x, c->body.pos.z); printf("   body t %.2f km %.3f lat %+.1f v %.1f contact %d belly %d y-g %.2f pitch %.1f roll %.1f vy %.2f F %.0f %.0f %.0f %.0f\n", c->time, n.s / 1000, n.lat, vlen(c->body.vel), bp.ncontact, bp.belly, bp.y - 0.25f * (g[0] + g[1] + g[2] + g[3]), bp.pitch * RAD2DEGF, bp.roll * RAD2DEGF, bp.vy, Fsum[0], Fsum[1], Fsum[2], Fsum[3]); }
    bp.last_t = c->time; bp.last_pos = c->body.pos;
}

/* Build D: where to put the ute back after a rollover, or when R (Recover) is pressed in Explorer: a trail of the last safe
 * places it was upright with all four wheels down on open ground, one a second. Recover uses the newest that's at least
 * 2.5 s old (not the spot it went over at); with none, the nearest point of the creek way's line or the road. */
static struct { float x, z, yaw, t; } trail[16]; static int trail_n; static float trail_clock, trail_last;
static void trail_clear(void) { trail_n = 0; trail_last = trail_clock - 10.0f; }
static void trail_step(const Car *c)
{
    trail_clock += FIXED_DT;
    if (trail_clock - trail_last < 1.0f) return;
    if (T.body_physics != 0 && (bp.ncontact < 4 || bp.rolled || bp.belly || fabsf(bp.roll) > 0.25f || fabsf(bp.pitch) > 0.3f)) return;
    if (route_edge_at(&RT, c->body.pos.x, c->body.pos.z) > T.edge_stop - 2.0f) return;
    if (trail_n == 16) { memmove(&trail[0], &trail[1], sizeof(trail[0]) * 15); trail_n = 15; }
    trail[trail_n].x = c->body.pos.x; trail[trail_n].z = c->body.pos.z; trail[trail_n].yaw = car_yaw(c); trail[trail_n].t = trail_clock; trail_n++;
    trail_last = trail_clock;
}
static void recover(Car *c)
{
    int k = -1;
    for (int i = trail_n - 1; i >= 0; i--) if (trail[i].t <= trail_clock - 2.5f) { k = i; break; }
    if (k < 0 && trail_n > 0) k = 0;
    float x, z, yaw;
    if (k >= 0) { x = trail[k].x; z = trail[k].z; yaw = trail[k].yaw; trail_n = k + 1; }
    else {
        RtNear cn = route_creek_near(&RT, c->body.pos.x, c->body.pos.z), n = route_near(&RT, c->body.pos.x, c->body.pos.z);
        if (explore && route_in_creek(&RT, c->body.pos.x, c->body.pos.z)) { route_creek_point(&RT, cn.s, &x, &z); int i = (int)(cn.s / RT_DS); if (i > RT.ck_n - 2) i = RT.ck_n - 2; yaw = (float)(M_PI / 2) - atan2f(RT.ck_z[i + 1] - RT.ck_z[i], RT.ck_x[i + 1] - RT.ck_x[i]); }
        else { float s0 = clampf(n.s, 4.0f, RT.length - 15.0f); route_point(&RT, s0, &x, &z); yaw = (float)(M_PI / 2) - route_heading_at(&RT, s0); }
    }
    int low = c->low;
    car_reset(c, v3(x, 0.75f, z), yaw);
    if (low) car_set_low(c, 1);
    bp.init = 0;
}


/* One physics step of the ground (after car_step): the slope's pull, drag on wheels off the gravel, and
 * trunks. Shared by the game and the headless drives. */
static void rt_physics(Car *c, RtPhys *ph)
{
    memset(ph, 0, sizeof(*ph));
    ph->tree = -1;
    float x = c->body.pos.x, z = c->body.pos.z, e = 0.75f;
    float gx = (route_height(&RT, x + e, z, NULL) - route_height(&RT, x - e, z, NULL)) / (2 * e);
    float gz = (route_height(&RT, x, z + e, NULL) - route_height(&RT, x, z - e, NULL)) / (2 * e);
    float gl = hypotf(gx, gz);
    if (gl > 0.6f) { gx *= 0.6f / gl; gz *= 0.6f / gl; }
    float g = 9.81f * T.grade_gravity;
    if (T.body_physics != 0) body_step(c, ph);   /* build D: the sprung body (its own pull along the ground, wheel by wheel) */
    else {
    c->body.vel.x -= g * gx * FIXED_DT;
    c->body.vel.z -= g * gz * FIXED_DT;
    }
    if (explore && T.trials_on != 0 && W.punct >= 1 && W.punct <= 4) {   /* build E: a flat: less grip at that wheel, drag, and a pull towards its side */
        int p = W.punct - 1; float v = vlen(c->body.vel);
        c->wheel[p].grip_lost = fmaxf(c->wheel[p].grip_lost, 1.0f - T.flat_grip);
        if (v > 0.3f) c->body.vel = vscale(c->body.vel, fmaxf(v - T.flat_drag * FIXED_DT, 0.0f) / v);
        float side = (p == 0 || p == 2) ? -1.0f : 1.0f, a = side * T.flat_pull * fminf(v / 10.0f, 1.5f) * FIXED_DT;
        if (v > 0.5f) { Q r = qaxis(v3(0, 1, 0), a); c->body.rot = qnorm(qmul(r, c->body.rot)); c->body.vel = qrot(r, c->body.vel); }
    }
    V3 f = qrot(c->body.rot, v3(0, 0, 1));
    ph->grade = gx * f.x + gz * f.z;
    /* wheels past the verge: the ditch, and beyond it rough ground */
    for (int i = 0; i < 4; i++) {
        int s;
        route_height(&RT, c->wheel[i].hit_point.x, c->wheel[i].hit_point.z, &s);
        if (s == SURF_DITCH) ph->ditch++;
        else if (s == SURF_ROUGH) ph->rough++;
        else if (s == SURF_ROCK) ph->rock++;   /* build D: the creek way's rocks (Explorer's strip only) */
        if (s == SURF_DITCH || s == SURF_ROUGH) {   /* the verge is safe; this far into the ditch, or past it, is not */
            RtNear wn = route_near(&RT, c->wheel[i].hit_point.x, c->wheel[i].hit_point.z);
            int carpark = RT.pad_r > 0 && hypotf(c->wheel[i].hit_point.x - RT.pad_x, c->wheel[i].hit_point.z - RT.pad_z) < RT.pad_r + 6.0f;   /* the car park is open ground: donuts */
            if (!carpark && fabsf(wn.lat) >= route_half_width() + T.verge_w + T.ditch_margin) ph->deep++;
        }
    }
    if (ph->ditch || ph->rough) {
        V3 vh = v3(c->body.vel.x, 0, c->body.vel.z);
        float sp = vlen(vh);
        float k = ((float)ph->ditch * T.ditch_drag + (float)ph->rough * T.rough_drag) * 0.25f;
        float dv = (k * sp + (float)ph->ditch * 0.25f * T.ditch_decel) * FIXED_DT;
        if (sp > 1e-3f) { float keep = sp > dv ? (sp - dv) / sp : 0.0f; c->body.vel.x *= keep; c->body.vel.z *= keep; }
        c->body.angvel.y *= 1.0f - clamp01(k * 0.5f * FIXED_DT);
    }
    if (ph->rock) {   /* build D: rocks: no carrying speed over them (crawl in Low Range) */
        V3 vh = v3(c->body.vel.x, 0, c->body.vel.z);
        float sp = vlen(vh), k = (float)ph->rock * 0.25f;
        float dv = (k * T.rock_drag * sp + k * T.rock_decel) * FIXED_DT;
        if (sp > 1e-3f) { float keep = sp > dv ? (sp - dv) / sp : 0.0f; c->body.vel.x *= keep; c->body.vel.z *= keep; }
    }
    /* build B: off the road, and the map's edge: up the bank the car is eased back, and held at edge_stop */
    {
        RtNear cn = route_near(&RT, x, z);
        float rd = cn.s < 0 ? hypotf(cn.s, cn.lat) : cn.s > RT.length ? hypotf(cn.s - RT.length, cn.lat) : fabsf(cn.lat);
        ph->carpark = RT.pad_r > 0 && hypotf(x - RT.pad_x, z - RT.pad_z) < RT.pad_r + 6.0f;
        /* off the road: past the ditch (the hut's clearing too: it no longer counts as road), the side road aside; the car park
         * scores nothing either, but shows no tag and runs no timer */
        ph->offroad = (rd > route_half_width() + T.verge_w + T.ditch_w + T.offroad_from && !route_on_spur(&RT, x, z)) || ph->carpark;
        V3 f2 = qrot(c->body.rot, v3(0, 0, 1)), r2 = qrot(c->body.rot, v3(1, 0, 0));
        float fr0 = car_outline.own ? car_outline.front : T.body_front, re0 = car_outline.own ? car_outline.rear : T.body_rear, hw0 = car_outline.own ? car_outline.half_w : T.body_half_w;
        float e = route_edge(&RT, cn, x, z);
        for (int k = 0; k < 4; k++) {   /* the outline's corners */
            float lf = k < 2 ? fr0 : re0, lr = (k & 1) ? hw0 : -hw0;
            e = fmaxf(e, route_edge_at(&RT, x + f2.x * lf + r2.x * lr, z + f2.z * lf + r2.z * lr));
        }
        ph->edge = e;
        float from = T.edge_stop - 2.0f;
        if (e > from) {
            const float q = 1.0f;
            float gx2 = route_edge_at(&RT, x + q, z) - route_edge_at(&RT, x - q, z), gz2 = route_edge_at(&RT, x, z + q) - route_edge_at(&RT, x, z - q), gl2 = hypotf(gx2, gz2);
            if (gl2 > 1e-4f) {
                gx2 /= gl2; gz2 /= gl2;   /* outwards */
                float k = clamp01((e - from) / 2.0f), out = c->body.vel.x * gx2 + c->body.vel.z * gz2;
                if (out > 0) { c->body.vel.x -= gx2 * out * k; c->body.vel.z -= gz2 * out * k; }
                float keep = 1.0f - clamp01(1.5f * k * FIXED_DT);
                c->body.vel.x *= keep; c->body.vel.z *= keep;
                if (e > T.edge_stop) { c->body.pos.x -= gx2 * (e - T.edge_stop); c->body.pos.z -= gz2 * (e - T.edge_stop); ph->held = 1; }
            }
        }
    }
    /* trunks */
    float hw = car_outline.own ? car_outline.half_w : T.body_half_w, fr = car_outline.own ? car_outline.front : T.body_front, re = car_outline.own ? car_outline.rear : T.body_rear;
    int t = route_tree_hit(&RT, x, z, car_yaw(c), hw, fr, re);
    if (bot_ghost && t >= 0 && RT.tree[t].kind != 5) t = -1;   /* trunks off for the tests (logs still break) */
    saw_log_t += FIXED_DT; if (saw_log_t > 0.4f) saw_log = -1;   /* build E: off the last log: the next touch is a new pass */
    if (t >= 0 && RT.tree[t].kind == 5 && logs_break && explore && T.saw_on != 0) {   /* build E: Will's chainsaw */
        int li = route_log_index(&RT, t);
        if (li == saw_log) { saw_log_t = 0; t = -1; }   /* still over it, the same pass */
        else {
            saw_log = li; saw_log_t = 0;
            RtNear cq = route_creek_near(&RT, RT.tree[t].x, RT.tree[t].z); int dirty = cq.dist < 15.0f && cq.s > RT.ck_exit_s, ev = 0;
            if (!saw_pass(li, vlen(c->body.vel), dirty, &ev)) {   /* not through yet: a jolt, and over it (the next pass cuts more) */
                ph->saw_ev = ev; ph->saw_x = RT.tree[t].x; ph->saw_z = RT.tree[t].z;
                c->body.vel = vscale(c->body.vel, ev == 5 ? 0.85f : 0.65f);
                t = -1;
            } else ph->saw_ev = ev;   /* split: firewood, below */
        }
    }
    if (t >= 0 && RT.tree[t].kind == 5 && logs_break) {   /* follow-up: a log splinters (firewood) and the ute carries on, a little slower */
        int li = route_log_index(&RT, t);
        ph->log_x = RT.tree[t].x; ph->log_z = RT.tree[t].z;
        if (li >= 0) { ph->log_cx = RT.tree[li].x; ph->log_cz = RT.tree[li].z; }
        int ch = route_log_break(&RT, t);
        if (ch >= 0) { route_stream_invalidate(ch); ph->log_broken = 1; c->body.vel = vscale(c->body.vel, T.log_keep); }
        t = -1;
    }
    if (t >= 0) {
        const RtTree *tr = &RT.tree[t];
        float dx = x - tr->x, dz = z - tr->z, d = fmaxf(hypotf(dx, dz), 1e-3f);
        float into = -(c->body.vel.x * dx + c->body.vel.z * dz) / d; /* speed towards the trunk */
        ph->tree = t; ph->tree_speed = vlen(c->body.vel);
        if (into > 0) { c->body.vel.x += into * dx / d; c->body.vel.z += into * dz / d; }
        c->body.pos.x += 0.03f * dx / d; c->body.pos.z += 0.03f * dz / d;
    }
    ph->stag = ph->staglog = -1;
    if (explore && RT.nstag > 0) stag_physics(c, ph, hw, fr, re);   /* build D: the dead trees (Explorer only) */
    ph->wet = -1; ph->wet_smash = 0;
    if (explore && RT.ck_nwet > 0) wet_physics(c, ph, hw, fr, re);   /* part 2: the wet logs */
}

/* The run's rules, one physics step (after rt_physics): the hold at the start until the road is there,
 * going forward, the wrong-way count, a tree hit, and the bridge. */
static const char *rt_hit_what = "TREE";
static void rt_rules(Run *r, const Car *c, const RtPhys *ph, float *max_s, int road_ready)
{
    if (r->state == RUN_COUNTDOWN) {
        r->countdown -= FIXED_DT;
        if (r->countdown <= 0 && road_ready) r->state = RUN_RUNNING;
        return;
    }
    if (r->state != RUN_RUNNING) return;
    r->time += FIXED_DT;
    r->ev_thud = 0;
    if (explore) {   /* build C: no checkpoints, no wrong way, no bridge, no game over: a hard hit only stops the ute */
        RtNear en = route_near(&RT, c->body.pos.x, c->body.pos.z);
        if (en.s > *max_s && fabsf(en.lat) < 15.0f) *max_s = en.s;
        if (ph->tree >= 0 && ph->tree_speed >= T.tree_min_speed) {
            const RtTree *t = &RT.tree[ph->tree];
            rt_hit_what = t->kind == 0 ? "TREE" : t->kind == 5 ? "LOG" : t->r <= 0.2f ? "POST" : "HUT";
            r->ev_thud = ph->tree_speed;
        }
        return;
    }
    RtNear n = route_near(&RT, c->body.pos.x, c->body.pos.z);
    /* build B: checkpoints every checkpoint_every metres, passed in order within 25 m of the road (no jumping
     * across from another stretch); the distance counts only up to the next one, and the bridge needs them all */
    float every = fmaxf(T.checkpoint_every, 50.0f);
    r->ncp = RT.bridge_s0 > 0 ? (int)((RT.bridge_s0 - 30.0f) / every) : 0;
    r->ev_cp = 0;
    int near_road = fabsf(n.lat) < T.edge_dist && n.s > -5.0f;   /* the gate spans the whole strip to the bank: exploring beside the road still passes it */
    while (r->next_cp < r->ncp && near_road && n.s - r->prev_s < 10.0f && r->prev_s < (float)(r->next_cp + 1) * every && n.s >= (float)(r->next_cp + 1) * every) { r->next_cp++; r->ev_cp = 1; }   /* a real crossing: from short of it to past it in one step */
    if (near_road) r->prev_s = n.s;
    float cap = r->next_cp < r->ncp ? (float)(r->next_cp + 1) * every : 1e9f;
    if (n.s > *max_s && fabsf(n.lat) < 15.0f) *max_s = fminf(n.s, cap);
    r->missed = fabsf(n.lat) < 15.0f && n.s > cap + 40.0f;
    /* wrong way (build B): driving back along the road, 30 m of it, not where you are; off the road it doesn't count */
    {
        float hd = route_heading_at(&RT, fmaxf(n.s, 0.0f)), along = c->body.vel.x * cosf(hd) + c->body.vel.z * sinf(hd);
        if (ph->offroad || fabsf(n.lat) > 15.0f) r->back_m = 0;
        else r->back_m = fmaxf(r->back_m - along * FIXED_DT * (along > 0 ? 3.0f : 1.0f), 0.0f);
    }
    /* build C: no run end for it any more (the checkpoints already stop shortcuts); Drift shows a small hint, and nothing
     * scores while it's up, so a bend can't be farmed back and forth */
    if (r->back_m < 30.0f) {
        if (r->dir != DIR_CORRECT) { r->dir = DIR_CORRECT; r->wrong_shown = 0; }
    } else if (r->dir == DIR_CORRECT) { r->dir = DIR_WRONG; r->wrong_shown = 1; }
    if (T.body_physics != 0 && bp.rolled && bp.rolled_t > 0.6f) { rt_hit_what = "ROLLED"; r->state = RUN_WALL_OVER; r->ev_hit_speed = 0; return; }   /* build D: over it went */
    if (ph->tree >= 0 && ph->tree_speed >= T.tree_min_speed) {
        const RtTree *t = &RT.tree[ph->tree];   /* session 3b: what it was, for the message (hidden posts: small = a post, big = the hut and its kin) */
        rt_hit_what = t->kind == 0 ? "TREE" : t->kind == 5 ? "LOG" : t->r <= 0.2f ? "POST" : "HUT";
        if (RT.gate_t > 0 && hypotf(t->x - RT.gate_x, t->z - RT.gate_z) < 14.0f) rt_hit_what = "GATE";   /* the boom gate and its fence */
        r->state = RUN_WALL_OVER; r->ev_hit_speed = ph->tree_speed; return;
    }
    if (RT.bridge_s1 > RT.bridge_s0 && n.s >= RT.bridge_s0 + 1.0f && fabsf(n.lat) < route_half_width() + 1.5f && r->next_cp >= r->ncp) r->state = RUN_FINISHED;
}

/* Follow-up: off the road for offroad_reset seconds (the car park aside) puts the ute back on the track, stopped and
 * facing the right way, at the nearest point of the road but never past the next checkpoint. Returns 1 when it did. */
static int rt_offroad_timer(Run *r, Car *c, const RtPhys *ph)
{
    if (r->state != RUN_RUNNING || explore) { r->offroad_t = 0; return 0; }   /* build C: Explorer never puts you back */
    r->offroad_t = ph->offroad && !ph->carpark ? r->offroad_t + FIXED_DT : 0;
    if (T.offroad_reset <= 0 || r->offroad_t < T.offroad_reset) return 0;
    RtNear n = route_near(&RT, c->body.pos.x, c->body.pos.z);
    float every = fmaxf(T.checkpoint_every, 50.0f), cap = r->next_cp < r->ncp ? (float)(r->next_cp + 1) * every - 2.0f : RT.length - 15.0f;
    float s = clampf(n.s, 4.0f, fminf(cap, RT.length - 15.0f)), x, z;
    route_point(&RT, s, &x, &z);
    car_reset(c, v3(x, 0.75f, z), (float)(M_PI / 2) - route_heading_at(&RT, s));
    r->offroad_t = 0; r->prev_s = s; r->back_m = 0;
    return 1;
}

/* The road bot (--drivetest, and --bot for benchmarks and pictures): aims at the centreline 8 m + look
 * seconds ahead and takes each bend at the speed its radius allows at alat m/s2 sideways. */
static int bot_brake_ahead = 1;
static void rt_bot(const Car *car, float alat, float look_s, float target, CarInput *in)
{
    RtNear n = route_near(&RT, car->body.pos.x, car->body.pos.z);
    float v = vlen(car->body.vel), look = 8.0f + look_s * v;
    float tx, tz;
    route_point(&RT, fmaxf(n.s, 0.0f) + look, &tx, &tz);
    V3 l = qrot(qconj(car->body.rot), v3(tx - car->body.pos.x, 0, tz - car->body.pos.z));
    float ang = atan2f(l.x, l.z);
    float rr = route_min_radius_ahead(&RT, n.s, 20.0f + v * 1.6f), tgt = fminf(target, sqrtf(alat * rr));
    if (bot_brake_ahead) { /* session 2: brake in time on gravel, allowing for the steepest downhill in the next 150 m */
        float s0 = fmaxf(n.s, 0.0f), down = 0;
        for (float d = 0; d < 150.0f; d += 10.0f) down = fmaxf(down, (route_road_h(&RT, s0 + d) - route_road_h(&RT, s0 + d + 10.0f)) / 10.0f);
        float ab = fmaxf(alat * 0.7f - 9.81f * down * T.grade_gravity, 0.4f);
        tgt = fminf(tgt, route_speed_ahead(&RT, s0, 250.0f, alat, ab));
    }
    memset(in, 0, sizeof(*in));
    in->steer = clampf(ang * 2.5f, -1.0f, 1.0f);
    in->throttle = v < tgt ? 1.0f : 0.0f;
    if (v > tgt + 2.0f) in->brake = clamp01((v - tgt - 2.0f) / 4.0f);
}

/* The shortcut bot (build B, --drivetest with BOT_SHORTCUT): drives cross-country at BOT_SC_KMH (default 50).
 * "beeline" aims straight at the bridge; "cut" aims at the road BOT_SC_AHEAD metres (default 400) past the nearest
 * point, so it cuts every bend through the bush. With BOT_GHOST=1 trunks don't stop it, which tests only the ground. */
static int bot_shortcut; static float bot_sc_ahead = 400.0f, bot_sc_speed = 50.0f / 3.6f;
static float route_heading_at_creek_pub(float s) { int i = (int)clampf(s / RT_DS, 0, (float)(RT.ck_n - 2)); return atan2f(RT.ck_z[i + 1] - RT.ck_z[i], RT.ck_x[i + 1] - RT.ck_x[i]); }
static void rt_shortcut_bot(const Car *car, CarInput *in)
{
    RtNear n = route_near(&RT, car->body.pos.x, car->body.pos.z);
    float tx, tz;
    if (bot_shortcut == 3) {   /* up the side road, aiming 8 m ahead along it, into the gate */
        float t2; route_spur_dist(&RT, car->body.pos.x, car->body.pos.z, &t2);
        int k = (int)clampf((fminf(t2, 60.0f) + 8.0f) / 2.0f, 1, (float)(RT.spur_n - 1)); tx = RT.spur_x[k]; tz = RT.spur_z[k];
    }
    else if (bot_shortcut == 1) route_point(&RT, RT.bridge_s0 + 4.0f, &tx, &tz);
    else route_point(&RT, fminf(fmaxf(n.s, 0.0f) + bot_sc_ahead, RT.bridge_s0 + 4.0f), &tx, &tz);
    V3 l = qrot(qconj(car->body.rot), v3(tx - car->body.pos.x, 0, tz - car->body.pos.z));
    float v = vlen(car->body.vel), ang = atan2f(l.x, l.z);
    memset(in, 0, sizeof(*in));
    in->steer = clampf(ang * 2.0f, -1.0f, 1.0f);
    in->throttle = v < bot_sc_speed ? 1.0f : 0.0f;
    if (v > bot_sc_speed + 3.0f) in->brake = 0.5f;
}

/* The deer bot (build B, --drivetest with BOT_DEER=1, trunks off): the road bot, but at each deer not yet found it
 * leaves the road, drives into it at 20 km/h and comes back. It proves every deer can be found and that finds count;
 * that each can be reached round the trunks is checked where they are placed (src/world.c). */
static int bot_deer, bot_donut, bot_nobrake, bot_logs; static CarInput bot_in;
static void rt_deer_bot(const Car *car, CarInput *in)
{
    RtNear n = route_near(&RT, car->body.pos.x, car->body.pos.z);
    int want = -1; float best = 1e9f;
    if (bot_logs) {   /* follow-up, BOT_LOGS=1: the logs within 20 m of the road instead of the deer */
        float lx = 0, lz = 0, ls = 1e9f;
        for (int k = 0; k < RT.ntree; k++) {
            const RtTree *t = &RT.tree[k];
            if (t->t0 < 0 || t->lat > 20.0f || strcmp(route_template(t->t0)->name, "log")) continue;
            RtNear ln = route_near(&RT, t->x, t->z);
            if (ln.s > n.s - 10.0f && ln.s < ls) { ls = ln.s; lx = t->x; lz = t->z; }
        }
        if (ls < n.s + 30.0f) {
            V3 l = qrot(qconj(car->body.rot), v3(lx - car->body.pos.x, 0, lz - car->body.pos.z));
            float v = vlen(car->body.vel);
            memset(in, 0, sizeof(*in));
            in->steer = clampf(atan2f(l.x, l.z) * 2.0f, -1, 1);
            in->throttle = v < 7.0f ? 1.0f : 0.0f;
            return;
        }
        rt_bot(car, 3.0f, 0.55f, 18.0f, in);
        return;
    }
    for (int d = 0; d < W.ndeer; d++) if (!((W.found >> d) & 1) && W.deer[d].s > n.s - 60.0f && W.deer[d].s < best) { best = W.deer[d].s; want = d; }
    float ws = want >= 0 ? W.deer[want].s : 1e9f, tx = want >= 0 ? W.deer[want].x : 0, tz = want >= 0 ? W.deer[want].z : 0;
    if (W.army_placed && !W.army_found) {   /* the army man counts as one more find */
        RtNear an = route_near(&RT, W.army_x, W.army_z);
        if (an.s > n.s - 60.0f && an.s < ws) { ws = an.s; tx = W.army_x; tz = W.army_z; want = 99; }
    }
    if (want >= 0 && n.s > ws - 25.0f) {
        V3 l = qrot(qconj(car->body.rot), v3(tx - car->body.pos.x, 0, tz - car->body.pos.z));
        float v = vlen(car->body.vel);
        memset(in, 0, sizeof(*in));
        in->steer = clampf(atan2f(l.x, l.z) * 2.0f, -1, 1);
        float dist = hypotf(tx - car->body.pos.x, tz - car->body.pos.z), vt = fminf(5.5f, 1.0f + 0.4f * dist);   /* slow right down near it: the ute turns wide */
        in->throttle = v < vt ? 1.0f : 0.0f;
        if (v > vt + 2.0f) in->brake = 0.6f;
        return;
    }
    rt_bot(car, 3.0f, 0.55f, 18.0f, in);
}

/* Donuts (build A, 21 Oct): a full turn of the nose in one direction with the tail out at least
 * donut_min_deg (the rear tyres' slip angle), under donut_max_kmh, while the car's middle stays inside a box
 * donut_box metres across. Each one on the road is worth donut_bonus straight onto the score (not into the pot);
 * in the car park it's practice and scores nothing, unless donut_carpark is 1. */
static void donut_update(Score *s, const Car *c, float dt, int running)
{
    if (!running) { s->donut_turn = 0; return; }
    V3 lv = car_local_velocity(c);
    float sp = vlen(v3(c->body.vel.x, 0, c->body.vel.z)), wy = c->body.angvel.y;
    float rear = 0.5f * (fabsf(c->wheel[RL].slip_angle) + fabsf(c->wheel[RR].slip_angle)) * RAD2DEGF;
    int ok = lv.z > -0.5f && sp > 0.8f && sp * 3.6f < T.donut_max_kmh && rear >= T.donut_min_deg && fabsf(wy) > 0.5f
             && (s->donut_turn == 0 || s->donut_turn * wy > 0);
    float x = c->body.pos.x, z = c->body.pos.z;
    if (!ok) {
        s->donut_off += dt;
        if (s->donut_off > 0.4f) s->donut_turn = 0;
        return;
    }
    s->donut_off = 0;
    if (s->donut_turn == 0) { s->dn_x0 = s->dn_x1 = x; s->dn_z0 = s->dn_z1 = z; }
    s->dn_x0 = fminf(s->dn_x0, x); s->dn_x1 = fmaxf(s->dn_x1, x); s->dn_z0 = fminf(s->dn_z0, z); s->dn_z1 = fmaxf(s->dn_z1, z);
    if (s->dn_x1 - s->dn_x0 > T.donut_box || s->dn_z1 - s->dn_z0 > T.donut_box) { s->donut_turn = 0; return; }   /* wandering off: not a donut */
    s->donut_turn += wy * dt;
    if (fabsf(s->donut_turn) >= T.donut_turn_deg * DEG2RAD) {
        float cx = 0.5f * (s->dn_x0 + s->dn_x1), cz = 0.5f * (s->dn_z0 + s->dn_z1);
        int carpark = RT.pad_r > 0 && hypotf(cx - RT.pad_x, cz - RT.pad_z) < RT.pad_r + 6.0f;
        if (explore) { s->ev_donut = carpark ? 2 : 1; s->donuts++; s->flash_v = 0; s->flash_kind = 5; }   /* build C: no points in Explorer */
        else if (!carpark || T.donut_carpark != 0) { float b = T.donut_bonus * score_boost; s->total += b; s->donut_pts += b; s->donuts++; s->ev_donut = 1; s->flash_v = b; s->flash_kind = 3; }
        else { s->ev_donut = 2; s->flash_v = 0; s->flash_kind = 4; }
        s->flash_t = 2.0f;
        s->donut_turn = 0;
    }
}

/* The "drift everything" bot (build A, --drivetest with BOT_DRIFT=degrees): the road bot's steering (it aims the
 * nose at the centreline ahead, which countersteers by itself once the tail swings), but in every bend tighter
 * than 120 m it kicks the tail out on the way in (a brake stab, or the handbrake with BOT_HB=1) and then holds
 * the slide with the throttle: more for more angle, less for less, within a speed limit for the bend. */
static float bot_drift_deg, bot_hb_left, bot_kick_wait;
static int bot_use_hb;
static void rt_drift_bot(const Car *car, float alat, CarInput *in)
{
    rt_bot(car, alat, 0.55f, 32.0f, in);   /* the road bot's line and speed */
    RtNear n = route_near(&RT, car->body.pos.x, car->body.pos.z);
    float v = vlen(car->body.vel), s0 = fmaxf(n.s, 0.0f);
    V3 lv = car_local_velocity(car);
    float r = car->body.angvel.y;
    float beta = (lv.z > 1.0f) ? atan2f(lv.x + r * anchors[RL].z, lv.z) * RAD2DEGF : 0;   /* at the rear axle; travel right of the nose = + */
    float rr = route_min_radius_ahead(&RT, s0, 10.0f + v * 0.8f);
    float h1 = route_heading_at(&RT, s0 + 2.0f), h2 = route_heading_at(&RT, s0 + 8.0f + v * 0.8f);
    float dh = -atan2f(sinf(h2 - h1), cosf(h2 - h1));   /* + = the road turns left (the map's angles run the other way) */
    float want = 0;   /* signed: + = a left-hand slide (travel right of the nose) */
    if (rr < 120.0f && fabsf(dh) > 0.06f) want = (dh > 0 ? 1.0f : -1.0f) * bot_drift_deg;
    float got = want > 0 ? beta : -beta, err = fabsf(want) - got;
    if (bot_kick_wait > 0) bot_kick_wait -= FIXED_DT;
    if (want != 0) {
        float vmax = sqrtf(alat * 1.6f * rr) + 2.0f;
        if (v < vmax) in->throttle = clamp01(0.35f + 0.08f * err);
        else in->throttle = clamp01(0.2f + 0.04f * err);
        if (v > vmax + 4.0f) in->brake = fmaxf(in->brake, clamp01((v - vmax - 4.0f) / 6.0f));
        else in->brake = 0;
        if (err > 0.6f * fabsf(want) && v > 7.0f && bot_kick_wait <= 0 && bot_hb_left <= 0) { bot_hb_left = 0.3f; bot_kick_wait = 1.5f; }
    }
    if (getenv("BOT_TRACE") && ((int)(car->time / FIXED_DT + 0.5f)) % 10 == 0)
        printf("   t %5.1f s %6.1f  lat %+5.1f  v %4.1f  rr %5.0f dh %+.2f  want %+5.1f beta %+5.1f  r %+.2f  steer %+.2f thr %.2f brk %.2f hb %.2f\n",
               car->time, s0, n.lat, v, rr, dh, want, beta, r, in->steer, in->throttle, in->brake, bot_hb_left);
    if (bot_hb_left > 0) {
        bot_hb_left -= FIXED_DT;
        if (bot_use_hb) in->handbrake = 1; else { in->brake = 1; in->throttle = 0; }
        in->steer = clampf(in->steer + (want > 0 ? -0.5f : 0.5f), -1, 1);   /* turned in: left for a left bend */
    }
}

/* The car as drawn: lifted onto the ground under its wheels and tilted to it (smoothed a little). */
typedef struct { float h, pitch, roll; float wh[4]; int init; } CarGround;
static CarGround cg;
static void car_ground_update(const Car *c, float dt, int snap)
{
    float h[4];
    for (int i = 0; i < 4; i++) { V3 w = vadd(c->body.pos, qrot(c->body.rot, anchors[i])); h[i] = route_height(&RT, w.x, w.z, NULL); } /* from the pose: wheel contacts are stale while the car is held still */
    float wb = fmaxf(anchors[FL].z - anchors[RL].z, 1.0f), tr = fmaxf(anchors[FR].x - anchors[FL].x, 0.8f);
    float hc = 0.25f * (h[0] + h[1] + h[2] + h[3]);
    float pitch = 0.5f * ((h[FL] + h[FR]) - (h[RL] + h[RR])) / wb;    /* rise forward */
    float roll = 0.5f * ((h[FR] + h[RR]) - (h[FL] + h[RL])) / tr;     /* rise to the right */
    float k = snap || !cg.init ? 1.0f : clamp01(dt * 14.0f);
    cg.h += (hc - cg.h) * k; cg.pitch += (pitch - cg.pitch) * k; cg.roll += (roll - cg.roll) * k;
    for (int i = 0; i < 4; i++) cg.wh[i] += (h[i] - cg.wh[i]) * k;
    cg.init = 1;
}
static float car_lean_roll, car_lean_pitch; /* degrees, for the readout */

/* ---------------------------------------------------------------- dust (ours, session 2) */
/* Gravel dust instead of tyre smoke: pale, warm puffs thrown up behind sliding wheels (and a little at
 * any speed), drifting with what the car gave them, growing and hanging for a few seconds. Camera-facing
 * squares in one batch, drawn into the small picture so the palette and dither make them pixel dust. */
#define DUST_MAX 400
static struct { V3 p, v; float age, life, size, a0; } dust[DUST_MAX];
static int dust_n;
static unsigned dust_seed = 12345u;
static float dust_rnd(void) { dust_seed = dust_seed * 1664525u + 1013904223u; return (float)(dust_seed >> 8) / 16777216.0f; }
static void dust_clear(void) { dust_n = 0; }
static void dust_update(const Car *c, float dt, int running)
{
    for (int i = 0; i < dust_n; ) { /* age, drift and slow */
        dust[i].age += dt;
        if (dust[i].age >= dust[i].life) { dust[i] = dust[--dust_n]; continue; }
        float drag = expf(-1.6f * dt);
        dust[i].v = vscale(dust[i].v, drag);
        dust[i].v.y += 0.25f * dt; /* warm air: lifts a little */
        dust[i].p = vadd(dust[i].p, vscale(dust[i].v, dt));
        i++;
    }
    if (!running || dt <= 0) return;
    float sp = vlen(c->body.vel);
    static float owed[4];
    for (int w = 0; w < 4; w++) {
        const Wheel *wh = &c->wheel[w];
        if (!wh->on_ground) continue;
        float slide = clamp01((fabsf(wh->slip_velo) - 1.0f) / 10.0f);
        float rate = T.dust_rate * (slide + 0.12f * clamp01((sp - 6.0f) / 20.0f)) * (w >= RL ? 1.0f : 0.6f);
        owed[w] += rate * dt;
        while (owed[w] >= 1.0f && dust_n < DUST_MAX) {
            owed[w] -= 1.0f;
            V3 at = wh->hit_point;
            at.y = route_height(&RT, at.x, at.z, NULL) + 0.25f;
            dust[dust_n].p = vadd(at, v3((dust_rnd() - 0.5f) * 0.6f, 0, (dust_rnd() - 0.5f) * 0.6f));
            dust[dust_n].v = vadd(vscale(c->body.vel, 0.25f + 0.15f * dust_rnd()), v3((dust_rnd() - 0.5f) * 2.0f, 0.6f + 1.2f * dust_rnd(), (dust_rnd() - 0.5f) * 2.0f));
            dust[dust_n].age = 0;
            dust[dust_n].life = T.dust_life * (0.6f + 0.6f * dust_rnd());
            dust[dust_n].size = T.dust_size * (0.7f + 0.6f * dust_rnd());
            dust[dust_n].a0 = 0.55f + 0.40f * slide;
            dust_n++;
        }
        if (owed[w] > 1.0f) owed[w] = 1.0f;
    }
}
static float dust_dim = 1.0f;   /* build C: dust is dim at night */
static void dust_draw(Camera3D cam)
{
    if (!dust_n) return;
    float dim = dust_dim;
    Vector3 f = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
    Vector3 r = Vector3Normalize(Vector3CrossProduct(f, cam.up)), u = Vector3CrossProduct(r, f);
    static const float ca[7] = { 1, 0.5f, -0.5f, -1, -0.5f, 0.5f, 1 }, sa[7] = { 0, 0.866f, 0.866f, 0, -0.866f, -0.866f, 0 };
    rlDisableBackfaceCulling();   /* the scene turns culling back on for the car: without this the dust never showed */
    rlSetTexture(0);
    rlBegin(RL_TRIANGLES);
    for (int i = 0; i < dust_n; i++) { /* a soft hexagon each */
        float t = dust[i].age / dust[i].life;
        float h = dust[i].size * (0.35f + 0.65f * sqrtf(t)) * 0.5f;
        float a = dust[i].a0 * (t < 0.1f ? t / 0.1f : 1.0f - (t - 0.1f) / 0.9f);
        Vector3 c = RO(dust[i].p);
        float dc = Vector3Distance(c, cam.position);
        a *= clamp01((dc - 1.5f) / 2.5f); /* thin out what the camera drives through */
        if (a <= 0.01f) continue;
        unsigned char al = (unsigned char)(clamp01(a) * 255);
        for (int k = 0; k < 6; k++) {
            Vector3 p0 = Vector3Add(c, Vector3Add(Vector3Scale(r, h * ca[k]), Vector3Scale(u, h * sa[k])));
            Vector3 p1 = Vector3Add(c, Vector3Add(Vector3Scale(r, h * ca[k + 1]), Vector3Scale(u, h * sa[k + 1])));
            rlColor4ub((unsigned char)(232 * dim), (unsigned char)(212 * dim), (unsigned char)(170 * dim), al); rlVertex3f(c.x, c.y, c.z);
            rlColor4ub((unsigned char)(226 * dim), (unsigned char)(204 * dim), (unsigned char)(160 * dim), al / 2); rlVertex3f(p1.x, p1.y, p1.z);
            rlColor4ub((unsigned char)(226 * dim), (unsigned char)(204 * dim), (unsigned char)(160 * dim), al / 2); rlVertex3f(p0.x, p0.y, p0.z);
        }
    }
    rlEnd();
    rlDrawRenderBatchActive();
    rlEnableBackfaceCulling();
}
/* ---------------------------------------------------------- splinters (follow-up: logs splinter into firewood) */
/* Short wooden shards thrown out of a broken log: they tumble, fall, bounce once or twice and fade. */
#define SPLINTER_MAX 160
typedef struct { V3 p, v, ax; float age, life, len, spin, ang; unsigned char r, g, b; } Splinter;
static Splinter splinter[SPLINTER_MAX];
static int splinter_n;
static float wood_flash_t;
static void splinters_burst(V3 at, V3 vel)
{
    for (int k = 0; k < 50 && splinter_n < SPLINTER_MAX; k++) {
        Splinter *s = &splinter[splinter_n++];
        s->p = vadd(at, v3((dust_rnd() - 0.5f) * 1.6f, dust_rnd() * 0.4f, (dust_rnd() - 0.5f) * 1.6f));
        s->v = vadd(vscale(vel, 0.35f + 0.3f * dust_rnd()), v3((dust_rnd() - 0.5f) * 7.0f, 2.0f + 4.5f * dust_rnd(), (dust_rnd() - 0.5f) * 7.0f));
        s->ax = vnorm(v3(dust_rnd() - 0.5f, dust_rnd() - 0.5f, dust_rnd() - 0.5f));
        s->age = 0; s->life = 1.8f + 1.4f * dust_rnd(); s->len = 0.3f + 0.6f * dust_rnd(); s->spin = (dust_rnd() - 0.5f) * 30.0f; s->ang = dust_rnd() * 6.28f;
        int light = dust_rnd() < 0.55f;   /* pale split wood or dark bark */
        s->r = light ? 214 : 96; s->g = light ? 176 : 72; s->b = light ? 120 : 50;
    }
}
static void splinters_update(float dt)
{
    for (int i = 0; i < splinter_n; ) {
        Splinter *s = &splinter[i];
        s->age += dt;
        if (s->age >= s->life) { splinter[i] = splinter[--splinter_n]; continue; }
        s->v.y -= 9.8f * dt;
        s->p = vadd(s->p, vscale(s->v, dt));
        s->ang += s->spin * dt;
        float g = route_height(&RT, s->p.x, s->p.z, NULL) + 0.03f;
        if (s->p.y < g) { s->p.y = g; s->v = v3(s->v.x * 0.4f, fabsf(s->v.y) * 0.3f, s->v.z * 0.4f); s->spin *= 0.5f; }
        i++;
    }
}
static void splinters_draw(Camera3D cam)
{
    if (!splinter_n) return;
    (void)cam;
    rlDisableBackfaceCulling();
    rlSetTexture(0);
    rlBegin(RL_TRIANGLES);
    for (int i = 0; i < splinter_n; i++) {
        Splinter *s = &splinter[i];
        float a = clamp01((s->life - s->age) / 0.5f);
        V3 d = qrot(qaxis(s->ax, s->ang), v3(1, 0, 0)), w = qrot(qaxis(s->ax, s->ang), v3(0, 0.08f, 0));
        V3 p0 = vsub(s->p, vscale(d, s->len * 0.5f)), p1 = vadd(s->p, vscale(d, s->len * 0.5f));
        Vector3 q0 = RO(vsub(p0, w)), q1 = RO(vadd(p0, w)), q2 = RO(vadd(p1, w)), q3 = RO(vsub(p1, w));
        rlColor4ub(s->r, s->g, s->b, (unsigned char)(255 * a));
        rlVertex3f(q0.x, q0.y, q0.z); rlVertex3f(q1.x, q1.y, q1.z); rlVertex3f(q2.x, q2.y, q2.z);
        rlVertex3f(q0.x, q0.y, q0.z); rlVertex3f(q2.x, q2.y, q2.z); rlVertex3f(q3.x, q3.y, q3.z);
    }
    rlEnd();
    rlDrawRenderBatchActive();
    rlEnableBackfaceCulling();
}

/* Firewood in the tray (follow-up): split logs stacked three layers of five in the tub (its floor is 0.48 m up, 1.7 m
 * long behind the cab); wood_mesh[k] holds the first k of them, in the car's own frame like its body. */
static Mesh wood_mesh[16];
static void wood_meshes_init(void)
{
    int start = ntris, per = 0;
    uint32_t sd = 777u;
    for (int k = 0; k < 15; k++) {
        int layer = k / 5, i = k % 5;
        float x = -0.6f + 0.3f * (float)i + (layer == 1 ? 0.08f : 0.0f), y = 0.48f + 0.09f + 0.165f * (float)layer;
        sd = sd * 1664525u + 1013904223u; float j = (float)(sd >> 8) / 16777216.0f;
        float z0 = -2.98f + 0.1f * j, z1 = -1.86f - 0.06f * j, r = 0.085f + 0.015f * j;   /* build C: shorter, at the back, the esky in front */
        V3 a[6], b[6];
        for (int q = 0; q < 6; q++) { float an = (float)q * 1.0472f + j; a[q] = v3(x + cosf(an) * r, y + sinf(an) * r, z0); b[q] = v3(x + cosf(an) * r, y + sinf(an) * r, z1); }
        float br = 0.36f + 0.08f * j, bg = 0.26f + 0.05f * j, bb = 0.17f + 0.03f * j;
        for (int q = 0; q < 6; q++) {
            int q2 = (q + 1) % 6;
            push_tri(a[q], b[q], b[q2], br, bg, bb); push_tri(a[q], b[q2], a[q2], br, bg, bb);
            push_tri(v3(x, y, z0), a[q2], a[q], 0.78f, 0.62f, 0.42f);   /* the sawn ends */
            push_tri(v3(x, y, z1), b[q], b[q2], 0.78f, 0.62f, 0.42f);
        }
        if (k == 0) per = ntris - start;
    }
    for (int n = 1; n <= 15; n++) {
        Mesh *m = &wood_mesh[n];
        int count = per * n;
        m->vertexCount = count * 3; m->triangleCount = count;
        m->vertices = malloc((size_t)count * 9 * sizeof(float)); m->colors = malloc((size_t)count * 12);
        for (int k = 0; k < count; k++) {
            const Tri *t = &tris[start + k];
            for (int q = 0; q < 3; q++) {
                V3 v = t->p[2 - q];
                float *vp = m->vertices + (size_t)(k * 3 + q) * 3; unsigned char *cp = m->colors + (size_t)(k * 3 + q) * 4;
                vp[0] = v.x; vp[1] = v.y; vp[2] = -v.z;
                cp[0] = t->r; cp[1] = t->g; cp[2] = t->b; cp[3] = CAR_ALPHA;
            }
        }
        UploadMesh(m, false);
    }
    ntris = start;
}

/* Build C: things in the tray, in the car's own frame: the esky (front of the tray, against the cab), what's been found
 * peeking over its rim, and Quayle's rifle lying across the wood. Made from the templates like the wood is. */
static Mesh esky_mesh, tray_rifle_mesh, peek_mesh, tray_quayle_mesh, tray_deer_mesh;
/* lying = 1: tipped on its side first (the deer in the tray), its long way along the tray */
static Mesh tray_mesh_lie(const char *name, int variant, V3 at, float yaw, float scale, int lying)
{
    Mesh m = { 0 };
    int ti = route_template_find(name, variant, 0);
    if (ti < 0) return m;
    const RtTemplate *t = route_template(ti);
    int start = ntris; float cy = cosf(yaw), sy = sinf(yaw);
    float lo[3] = { 1e9f, 1e9f, 1e9f }, hi[3] = { -1e9f, -1e9f, -1e9f };
    for (int k = 0; k < t->ntri * 9; k++) { int a = k % 3; lo[a] = fminf(lo[a], t->p[k]); hi[a] = fmaxf(hi[a], t->p[k]); }
    int along_x = hi[0] - lo[0] > hi[2] - lo[2];
    for (int k = 0; k < t->ntri; k++) {
        V3 q[3];
        for (int j = 0; j < 3; j++) {
            float px = t->p[9 * k + 3 * j], py = t->p[9 * k + 3 * j + 1], pz = t->p[9 * k + 3 * j + 2];
            if (lying) {   /* on its side, legs out to the right, the long way along z, centred, resting on y = 0 */
                float L = along_x ? px - 0.5f * (lo[0] + hi[0]) : pz - 0.5f * (lo[2] + hi[2]), A = along_x ? pz - 0.5f * (lo[2] + hi[2]) : px - 0.5f * (lo[0] + hi[0]);
                px = py - lo[1] - 0.5f * (hi[1] - lo[1]); py = A + 0.5f * (along_x ? hi[2] - lo[2] : hi[0] - lo[0]); pz = L;
            }
            px *= scale; py *= scale; pz *= scale;
            q[j] = v3(at.x + px * cy - pz * sy, at.y + py, at.z + px * sy + pz * cy);
        }
        push_tri(q[0], q[1], q[2], t->rgb[3 * k], t->rgb[3 * k + 1], t->rgb[3 * k + 2]);
        if (vdot(vcross(vsub(q[1], q[0]), vsub(q[2], q[0])), v3(0, 1, 0)) < -1e9f) { }
    }
    int count = ntris - start;
    m.vertexCount = count * 3; m.triangleCount = count;
    m.vertices = malloc((size_t)count * 9 * sizeof(float)); m.colors = malloc((size_t)count * 12);
    for (int k = 0; k < count; k++) {
        const Tri *tt = &tris[start + k];
        for (int q = 0; q < 3; q++) {
            V3 v = tt->p[2 - q];
            float *vp = m.vertices + (size_t)(k * 3 + q) * 3; unsigned char *cp = m.colors + (size_t)(k * 3 + q) * 4;
            vp[0] = v.x; vp[1] = v.y; vp[2] = -v.z;
            cp[0] = tt->r; cp[1] = tt->g; cp[2] = tt->b; cp[3] = CAR_ALPHA;
        }
    }
    UploadMesh(&m, false);
    ntris = start;
    return m;
}
static Mesh tray_mesh_from(const char *name, int variant, V3 at, float yaw, float scale) { return tray_mesh_lie(name, variant, at, yaw, scale, 0); }
static void tray_meshes_init(void)
{
    tray_quayle_mesh = tray_mesh_from("armyman", 1, v3(-0.36f, 0.48f, -1.55f), -1.5708f, 1.0f);   /* build C follow-up: standing in the tray facing forward, the rifle over the cab */
    tray_deer_mesh = tray_mesh_lie("sambar", 2, v3(0.0f, 0.5f, -1.9f), 0.0f, 0.72f, 1);
    for (int i = 0; i < HAMPER_N; i++) cut_item_mesh[i] = tray_mesh_from("hamper", i, v3(0, 0, 0), 0.0f, 1.0f / 2.2f);   /* life size (the bush's are 2.2 times) */
    cut_esky_mesh = tray_mesh_from("esky", 0, v3(0, 0, 0), 0.0f, 0.95f);
    esky_mesh = tray_mesh_from("esky", 0, v3(0.0f, 0.48f, -1.58f), 0.0f, 0.95f);
    tray_rifle_mesh = tray_mesh_from("rifle", 0, v3(0.05f, 0.62f, -2.2f), 1.45f, 0.95f);
    MeshBuilder b = { 0 };   /* one small block per thing found, peeking out of the esky (coloured as it's drawn) */
    V3 c[8]; for (int i = 0; i < 8; i++) c[i] = v3((i & 1) ? 0.05f : -0.05f, (i & 2) ? 0.1f : 0.0f, (i & 4) ? 0.05f : -0.05f);
    static const int q[6][4] = { {0,1,3,2}, {4,6,7,5}, {0,4,5,1}, {2,3,7,6}, {0,2,6,4}, {1,5,7,3} };
    for (int f = 0; f < 6; f++) { Color col = shaded((Color){ 255, 255, 255, CAR_ALPHA }, v3(f == 4 ? -1.0f : f == 5 ? 1.0f : 0, f == 3 ? 1.0f : f == 2 ? -1.0f : 0, f == 1 ? 1.0f : f == 0 ? -1.0f : 0)); mb_quad(&b, c[q[f][0]], c[q[f][1]], c[q[f][2]], c[q[f][3]], col, 1); }
    peek_mesh = mb_finish(&b);
}
static const unsigned char hamper_col[HAMPER_N][3] = { { 242, 236, 220 }, { 200, 30, 25 }, { 242, 205, 110 }, { 140, 32, 22 }, { 110, 140, 52 }, { 80, 140, 46 },
                                                      { 205, 38, 24 }, { 142, 172, 88 }, { 248, 248, 242 }, { 250, 200, 52 }, { 205, 145, 72 }, { 60, 20, 30 } };

static Q car_tilt(Q rot)
{
    V3 f = qrot(rot, v3(0, 0, 1)), r = qrot(rot, v3(1, 0, 0));
    V3 fh = vnorm(v3(f.x, 0, f.z)), rh = vnorm(v3(r.x, 0, r.z));
    V3 up = vnorm(vcross(vadd(fh, v3(0, cg.pitch, 0)), vadd(rh, v3(0, cg.roll, 0))));
    /* the shortest turn from straight up to the ground's up */
    V3 ax = vcross(v3(0, 1, 0), up);
    float s = vlen(ax), cth = up.y;
    if (s < 1e-6f) return rot;
    Q t = qaxis(vscale(ax, 1.0f / s), atan2f(s, cth));
    return qmul(t, rot);
}

/* A drawing matrix from a position and three axis vectors given in physics axes. */
static Matrix basis_matrix(V3 p, V3 bx, V3 by, V3 bz)
{
    Matrix m = { 0 };
    m.m0 = bx.x; m.m1 = bx.y; m.m2 = -bx.z;
    m.m4 = by.x; m.m5 = by.y; m.m6 = -by.z;
    m.m8 = bz.x; m.m9 = bz.y; m.m10 = -bz.z;
    m.m12 = p.x; m.m13 = p.y; m.m14 = -p.z; m.m15 = 1.0f;
    return m;
}

/* Physics pose to a drawing matrix, with the z mirror applied on both sides. */
static Matrix pose_matrix(V3 pos, Q rot)
{
    V3 ex = qrot(rot, v3(1, 0, 0)), ey = qrot(rot, v3(0, 1, 0)), ez = qrot(rot, v3(0, 0, 1));
    Matrix m = { 0 };
    m.m0 = ex.x;  m.m1 = ex.y;  m.m2 = -ex.z;
    m.m4 = ey.x;  m.m5 = ey.y;  m.m6 = -ey.z;
    m.m8 = -ez.x; m.m9 = -ez.y; m.m10 = ez.z;
    m.m12 = pos.x; m.m13 = pos.y; m.m14 = -pos.z; m.m15 = 1.0f;
    return m;
}

/* ------------------------------------------------------- generated sound */
/* Old-console style sound made in code, no files: a pulse-wave engine that follows
 * revs, clocked noise for the tyres, and short note runs for score and gear events. */

static struct {
    AudioStream stream;
    int ready;
    volatile float rpm_norm, throttle, skid, volume, lv_engine, lv_tyre, lv_blip;
    volatile int blip_id, blip_kind;
    volatile float blip_pitch, blip_gain;
    float cur_pitch, cur_gain;
    float ph1, ph2, noise_ph, noise_val, freq, thr, sk;
    unsigned lfsr;
    int blip_seen, kind;
    float blip_t, blip_ph;
    /* build C: the fire's crackle (0..1) and the fireworks' bangs (a burst of low noise each time bang_id moves on) */
    volatile float crackle; volatile int bang_id; volatile float bang_gain;
    int bang_seen; float bang_env, bang_lp, pop_env, pop_lp, hiss_lp;
    /* play test (22 Oct): the fireworks' own sounds: a sharp crack in each bang, the stars crackling after (the whistle is recorded) */
    float crack_env, crack_lp, spark_env, spark_pop, spark_lp;
    volatile float fw_level;   /* the fire and the fireworks' level over the rest (up under the finale's song: play test) */
    volatile float whine, whine_f; float whine_lv, whine_fs, whine_ph;   /* build D: Low Range's transmission whine (level 0..1, pitch Hz) */
} syn;

static volatile long synth_ns; /* time spent generating sound since the bench last looked */
static void synth_callback(void *buffer, unsigned int frames)
{
    struct timespec ts0, ts1;
    clock_gettime(CLOCK_MONOTONIC, &ts0);
    short *out = (short *)buffer;
    const float sr = 22050.0f;
    /* 1 multiplier up, 2 banked (a rising major arpeggio), 3 gear change, 4 wall knock, 5 drift-line dot or menu move (pitch set by the caller),
     * 6 lap or menu choice, 7 clutch dump, 8 a wall banked the pot (a knock, then the banking notes), 9 a wall lost the pot (a knock, then the
     * lost notes), 10 the pot lost (build A: a soft low two-note fall, triangle wave, quiet; was a square-wave buzz), 11 a donut,
     * 12 a deer found (build B: a soft rising chime), 13 an achievement (a short square fanfare) */
    static const float notes[15][4] = { { 0 }, { 660, 880, 1320 }, { 523, 659, 784, 1047 }, { 190, 120 }, { 95, 60, 45 }, { 990 }, { 660, 990 }, { 70, 150, 310 },
                                        { 95, 523, 784, 1047 }, { 70, 196, 147 }, { 196, 147 }, { 1047, 784, 1047, 1319 },   /* 11: a donut */
                                        { 587, 880, 1175 }, { 784, 784, 1047, 1568 },
                                        { 96, 88, 81, 74 } };   /* 14: build D, a dead tree creaking over (a slow wavering fall) */
    static const float note_len[15] = { 0, 0.06f, 0.07f, 0.03f, 0.05f, 0.05f, 0.09f, 0.025f, 0.07f, 0.13f, 0.14f, 0.06f, 0.11f, 0.08f, 0.32f };
    static const int note_n[15] = { 0, 3, 4, 2, 3, 1, 2, 3, 4, 3, 2, 4, 3, 4, 4 };
    static const float note_gain[15] = { 0, 1, 0.9f, 1, 1, 1, 1, 1, 0.9f, 0.55f, 0.55f, 0.9f, 0.8f, 0.75f, 0.8f };
    static const int note_soft[15] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1 };   /* triangle wave with a decay on each note, not a square (the lost notes) */
    for (unsigned int i = 0; i < frames; i++) {
        syn.freq += ((55.0f + 500.0f * syn.rpm_norm) - syn.freq) * 0.004f;
        syn.thr += (syn.throttle - syn.thr) * 0.004f;
        syn.sk += (syn.skid - syn.sk) * 0.004f;
        syn.ph1 += syn.freq / sr; if (syn.ph1 >= 1.0f) syn.ph1 -= 1.0f;
        syn.ph2 += syn.freq * 0.5f / sr; if (syn.ph2 >= 1.0f) syn.ph2 -= 1.0f;
        float duty = 0.25f + 0.2f * syn.thr;
        float eng = (syn.ph1 < duty ? 0.55f : -0.55f) + (syn.ph2 < 0.5f ? 0.45f : -0.45f);
        eng *= (0.3f + 0.7f * syn.thr) * (0.45f + 0.55f * syn.rpm_norm);
        syn.noise_ph += (1800.0f + 2600.0f * syn.sk) / sr;
        if (syn.noise_ph >= 1.0f) {
            syn.noise_ph -= 1.0f;
            unsigned bit = ((syn.lfsr >> 0) ^ (syn.lfsr >> 1)) & 1u;
            syn.lfsr = (syn.lfsr >> 1) | (bit << 14);
            syn.noise_val = (syn.lfsr & 1u) ? 1.0f : -1.0f;
        }
        float tyre = syn.noise_val * syn.sk * syn.lv_tyre;
        float blip = 0;
        if (syn.blip_seen != syn.blip_id) { syn.blip_seen = syn.blip_id; syn.kind = syn.blip_kind; syn.cur_pitch = syn.blip_pitch; syn.cur_gain = syn.blip_gain; syn.blip_t = 0; syn.blip_ph = 0; }
        if (syn.kind > 0) {
            int k = syn.kind, n = (int)(syn.blip_t / note_len[k]);
            if (n >= note_n[k]) syn.kind = 0;
            else {
                syn.blip_ph += notes[k][n] * syn.cur_pitch / sr; if (syn.blip_ph >= 1.0f) syn.blip_ph -= 1.0f;
                float w;
                if (note_soft[k] && notes[k][n] > 80.0f) {   /* triangle, fading through the note (the low knock stays square) */
                    float in_note = (syn.blip_t - (float)n * note_len[k]) / note_len[k];
                    w = (syn.blip_ph < 0.5f ? 4.0f * syn.blip_ph - 1.0f : 3.0f - 4.0f * syn.blip_ph) * 0.6f * (1.0f - 0.7f * in_note);
                } else w = syn.blip_ph < 0.5f ? 0.5f : -0.5f;
                blip = w * syn.cur_gain * note_gain[k];
                syn.blip_t += 1.0f / sr;
            }
        }
        float fire = 0;   /* build C */
        if (syn.bang_seen != syn.bang_id) { syn.bang_seen = syn.bang_id; syn.bang_env = syn.bang_gain; syn.crack_env = syn.bang_gain; syn.spark_env = syn.bang_gain * 0.7f; }
        if (syn.crack_env > 0.001f || syn.spark_env > 0.001f) {   /* the crack (bright, short) and the crackle after it (sparse sharp pops) */
            syn.lfsr = syn.lfsr * 1103515245u + 12345u; float n = (float)((syn.lfsr >> 9) & 0xffff) / 32768.0f - 1.0f;
            syn.crack_lp += (n - syn.crack_lp) * 0.45f; fire += syn.crack_lp * syn.crack_env * 1.4f; syn.crack_env *= 0.9965f;
            if ((float)((syn.lfsr >> 3) & 0xfff) < 3.0f * clamp01(syn.spark_env * 2.0f)) syn.spark_pop = 0.7f + 0.3f * (float)((syn.lfsr >> 17) & 0xff) / 255.0f;
            syn.spark_lp += (n - syn.spark_lp) * 0.6f; fire += (n - syn.spark_lp) * syn.spark_pop * syn.spark_env * 0.9f; syn.spark_pop *= 0.985f;
            syn.spark_env *= 0.99993f; if (syn.spark_env < 0.002f) syn.spark_env = 0;
        }
        if (syn.bang_env > 0.001f || syn.crackle > 0.001f) {
            syn.lfsr = syn.lfsr * 1103515245u + 12345u; float n = (float)((syn.lfsr >> 9) & 0xffff) / 32768.0f - 1.0f;
            if (syn.bang_env > 0.001f) { syn.bang_lp += (n - syn.bang_lp) * 0.06f; fire += syn.bang_lp * syn.bang_env * 2.2f; syn.bang_env *= 0.99985f; if (syn.bang_env < 0.02f) syn.bang_env *= 0.9995f; }
            if (syn.crackle > 0.001f) {
                if ((float)((syn.lfsr >> 4) & 0xfff) < 2.2f * syn.crackle) syn.pop_env = 0.6f + 0.4f * (float)((syn.lfsr >> 16) & 0xff) / 255.0f;
                syn.pop_lp += (n - syn.pop_lp) * 0.5f; syn.hiss_lp += (n - syn.hiss_lp) * 0.03f;
                fire += (syn.pop_lp * syn.pop_env + syn.hiss_lp * 0.25f) * syn.crackle * 0.5f; syn.pop_env *= 0.993f;
            }
        }
        float wh = 0;   /* build D: the transfer case in low: a soft gear whine that follows the prop shaft */
        syn.whine_lv += (syn.whine - syn.whine_lv) * 0.002f;
        if (syn.whine_lv > 0.001f) {
            syn.whine_fs += (syn.whine_f - syn.whine_fs) * 0.003f;
            syn.whine_ph += syn.whine_fs / sr; if (syn.whine_ph >= 1.0f) syn.whine_ph -= 1.0f;
            float tri = syn.whine_ph < 0.5f ? 4.0f * syn.whine_ph - 1.0f : 3.0f - 4.0f * syn.whine_ph;
            wh = (tri + 0.3f * sinf(6.2831853f * syn.whine_ph * 2.0f)) * syn.whine_lv * 0.11f;
        }
        float s = (eng * syn.lv_engine + tyre + blip * syn.lv_blip + wh + fire * (syn.fw_level > 0 ? syn.fw_level : 1.0f)) * syn.volume;
        s = roundf(s * 16.0f) / 16.0f; /* coarse steps for the old-console grit */
        out[i] = (short)(clampf(s, -1.0f, 1.0f) * 30000.0f);
    }
    clock_gettime(CLOCK_MONOTONIC, &ts1);
    synth_ns += (ts1.tv_sec - ts0.tv_sec) * 1000000000L + (ts1.tv_nsec - ts0.tv_nsec);
}

static void synth_init(void)
{
    if (!IsAudioDeviceReady()) return;
    syn.lfsr = 0x4a35u;
    syn.freq = 80.0f;
    SetAudioStreamBufferSizeDefault(1024);
    syn.stream = LoadAudioStream(22050, 16, 1);
    SetAudioStreamCallback(syn.stream, synth_callback);
    PlayAudioStream(syn.stream);
    syn.ready = 1;
}

static void synth_blip2(int kind, float pitch, float gain) { syn.blip_kind = kind; syn.blip_pitch = pitch; syn.blip_gain = gain; syn.blip_id++; }
static void synth_blip(int kind) { synth_blip2(kind, 1.0f, 1.0f); }

/* --soundcheck: run the generator with no sound device and print how loud each part is. */
static float synth_rms(float rpm, float thr, float skid, int blip, float le, float lt, float lb)
{
    static short buf[22050];
    memset(&syn, 0, sizeof(syn));
    syn.lfsr = 0x4a35u; syn.freq = 55.0f + 500.0f * rpm; syn.thr = thr; syn.sk = skid;
    syn.rpm_norm = rpm; syn.throttle = thr; syn.skid = skid; syn.volume = T.synth_volume;
    syn.lv_engine = le; syn.lv_tyre = lt; syn.lv_blip = lb;
    if (blip) synth_blip(blip);
    int n = blip ? 3000 : 22050;
    synth_callback(buf, (unsigned)n);
    double sum = 0;
    for (int i = 0; i < n; i++) sum += (double)buf[i] * buf[i];
    return (float)sqrt(sum / n) / 30000.0f;
}
static int soundcheck(void)
{
    float e = T.synth_engine, t = T.synth_tyre, b = T.synth_blip;
    printf("generated sound levels (RMS, 1.0 = full scale), synth_volume %.2f\n", T.synth_volume);
    printf("  engine  cruising (half revs, half throttle) %.3f   flat out near the limiter %.3f   idle %.3f\n",
           synth_rms(0.5f, 0.5f, 0, 0, e, 0, 0), synth_rms(0.95f, 1, 0, 0, e, 0, 0), synth_rms(0.1f, 0, 0, 0, e, 0, 0));
    printf("  tyres   light slide %.3f   hard slide %.3f   (at the old fixed 0.5: %.3f and %.3f)\n",
           synth_rms(0, 0, 0.4f, 0, 0, t, 0), synth_rms(0, 0, 1, 0, 0, t, 0), synth_rms(0, 0, 0.4f, 0, 0, 0.5f, 0), synth_rms(0, 0, 1, 0, 0, 0.5f, 0));
    printf("  notes   multiplier up %.3f   banked %.3f   gear change %.3f   pot lost %.3f\n",
           synth_rms(0, 0, 0, 1, 0, 0, b), synth_rms(0, 0, 0, 2, 0, 0, b), synth_rms(0, 0, 0, 3, 0, 0, b), synth_rms(0, 0, 0, 10, 0, 0, b));
    printf("  all at once (flat out, hard slide, note): %.3f\n", synth_rms(0.95f, 1, 1, 1, e, t, b));
    return 0;
}

/* ----------------------------------------------------------------- sound */
/* Recorded sound (session 3b, CC0 recordings from Freesound: docs/CREDITS.md, cut by tools/make_sfx.py into
 * assets/sfx/): two engine loops cross-faded and pitched by revs (a V8 diesel idling, a LandCruiser under
 * load), gravel under the tyres (rolling, by speed; sliding, by how fast the contact patches slide), the
 * bush (birds and insects) and now and then a kookaburra. The synth above plays the score's notes. */

static struct {
    int ready;
    Music loop[5]; int has[5];      /* engine idle, engine load, gravel roll, gravel slide, bush */
    Sound kook; int has_kook; float kook_in;
    Music yodel; int has_yodel; float yodel_fade;   /* build A: Drift Yodel, only while the mode is on */
    Music finale; int has_finale, finale_on;        /* play test (22 Oct): the Hut night's song, once through from the finale */
    Sound whistle[4]; int has_whistle, whistle_next;  /* a rocket's whistle going up (four, so they overlap) */
} snd;
enum { SND_IDLE, SND_LOAD, SND_ROLL, SND_SLIDE, SND_BUSH };
static float snd_duck = 1.0f;   /* build A: everything but the song, turned down while Drift Yodel plays (yodel_duck) */

static void sound_init(void)
{
    InitAudioDevice();
    snd.ready = IsAudioDeviceReady();
    if (!snd.ready) return;
    static const char *files[5] = { "assets/sfx/engine_idle.ogg", "assets/sfx/engine_load.ogg", "assets/sfx/gravel_roll.ogg",
                                    "assets/sfx/gravel_slide.ogg", "assets/sfx/bush.ogg" };
    for (int i = 0; i < 5; i++) {
        snd.loop[i] = LoadMusicStream(files[i]);
        if (snd.loop[i].frameCount == 0) continue;
        snd.loop[i].looping = true; SetMusicVolume(snd.loop[i], 0); PlayMusicStream(snd.loop[i]);
        snd.has[i] = 1;
    }
    snd.kook = LoadSound("assets/sfx/kookaburra.ogg"); snd.has_kook = snd.kook.frameCount > 0;   /* no FileExists: on Android the files are inside the APK */
    snd.kook_in = 25.0f;
    snd.whistle[0] = LoadSound("assets/sfx/rocket_whistle.ogg"); snd.has_whistle = snd.whistle[0].frameCount > 0;
    if (snd.has_whistle) for (int i = 1; i < 4; i++) snd.whistle[i] = LoadSoundAlias(snd.whistle[0]);
    snd.yodel = LoadMusicStream("assets/sfx/drift_yodel.ogg"); snd.has_yodel = snd.yodel.frameCount > 0;
    if (snd.has_yodel) snd.yodel.looping = true;
    /* the finale's song: The Late Cake by willfarrell85, used with his permission (docs/CREDITS.md) */
    snd.finale = LoadMusicStream("assets/music/the-late-cake.ogg"); snd.has_finale = snd.finale.frameCount > 0;
    if (snd.has_finale) { snd.finale.looping = false; fprintf(stderr, "sound: the finale's song, %.0f s\n", GetMusicTimeLength(snd.finale)); }
}

static void snd_set(int i, float vol, float pitch)
{
    if (!snd.has[i]) return;
    SetMusicVolume(snd.loop[i], clamp01(vol * T.snd_volume * snd_duck));
    SetMusicPitch(snd.loop[i], clampf(pitch, 0.3f, 3.0f));
    UpdateMusicStream(snd.loop[i]);
}

static void sound_bush(float dt, int on)   /* the bush: only while driving (build A: not on the title, records or results) */
{
    snd_set(SND_BUSH, on ? T.snd_bush : 0.0f, 1.0f);
    if (!on) { if (snd.has_kook && IsSoundPlaying(snd.kook)) StopSound(snd.kook); return; }
    if (snd.has_kook && (snd.kook_in -= dt) <= 0) {
        SetSoundVolume(snd.kook, clamp01(T.snd_kookaburra * T.snd_volume * snd_duck * (0.6f + 0.4f * (float)GetRandomValue(0, 100) / 100.0f)));
        SetSoundPan(snd.kook, (float)GetRandomValue(15, 85) / 100.0f);
        PlaySound(snd.kook);
        snd.kook_in = (float)GetRandomValue(45, 110);
    }
}

static void sound_update(const Car *c, int playing)
{
    if (syn.ready) { syn.volume = T.synth_volume * snd_duck; syn.lv_engine = 0; syn.lv_tyre = 0; syn.lv_blip = T.synth_blip; syn.rpm_norm = 0; syn.throttle = 0; syn.skid = 0; }
    if (syn.ready) {   /* build D: Low Range's whine, from the prop shaft's speed (a 23-tooth mesh), louder with speed and throttle */
        float wr = 0; for (int i = 0; i < 4; i++) wr += fabsf(c->wheel[i].ang_vel) * 0.25f;
        float shaft = wr * T.car_final_drive * T.car_low_ratio / 6.2831853f;
        syn.whine = c->low && playing ? clamp01(shaft / 6.0f) * (0.45f + 0.55f * clamp01(c->throttle)) : 0.0f;
        syn.whine_f = 120.0f + shaft * 23.0f;
    }
    if (!snd.ready) return;
    float dt = GetFrameTime();
    float rpm = c->rpm, thr = clamp01(c->throttle);
    float x = clamp01((rpm - T.snd_cross_from_rpm) / fmaxf(T.snd_cross_to_rpm - T.snd_cross_from_rpm, 1.0f));  
    x = x * x * (3.0f - 2.0f * x);   /* 0 idle loop .. 1 load loop, eased */
    snd_set(SND_IDLE, T.snd_engine * (1.0f - x) * (0.75f + 0.25f * thr), rpm / T.snd_idle_rpm);
    snd_set(SND_LOAD, T.snd_engine * x * (0.55f + 0.45f * thr), rpm / T.snd_load_rpm);
    float v = vlen(v3(c->body.vel.x, 0, c->body.vel.z)), slip = 0;
    for (int i = 0; i < 4; i++) slip = fmaxf(slip, c->wheel[i].slip_velo);
    snd_set(SND_ROLL, T.snd_gravel * clamp01(v / 18.0f), 0.75f + clampf(v / 40.0f, 0, 0.6f));
    float sl = clamp01((slip - T.skid_sound_from) / 9.0f);
    snd_set(SND_SLIDE, T.snd_slide * sl, 0.85f + clampf(slip / 30.0f, 0, 0.45f));
    sound_bush(dt, playing);
}

/* Menus: engine, tyres and the bush silent; the short notes still sound. */
static void sound_menu(void)
{
    if (syn.ready) { syn.rpm_norm = 0; syn.throttle = 0; syn.skid = 0; syn.volume = T.synth_volume; syn.lv_engine = 0; syn.lv_tyre = 0; syn.lv_blip = T.synth_blip; syn.whine = 0; }
    if (!snd.ready) return;
    for (int i = 0; i < 4; i++) snd_set(i, 0, 1);
    sound_bush(GetFrameTime(), 0);
}

/* Drift Yodel mode (build A, 21 Oct): any donut, even practice in the car park, starts it. For yodel_time
 * seconds the song plays, the handling is the drift preset, flames come out of the exhaust, scoring is doubled
 * (yodel_score) and a pixel sign of the horns sits on the screen. Then it all ends, and the next donut starts it again. */
static struct { int active; float left, banner; int saved_preset; } yodel;
static void yodel_music(int start)
{
    if (!snd.ready || !snd.has_yodel) return;
    if (start) { StopMusicStream(snd.yodel); PlayMusicStream(snd.yodel); snd.yodel_fade = 1.0f; }
}
static void sound_whistle(float gain)   /* a rocket going up: the recorded whistle, a little higher or lower each time */
{
    if (!snd.ready || !snd.has_whistle || T.fw_whistle <= 0) return;
    Sound w = snd.whistle[snd.whistle_next++ & 3];
    SetSoundVolume(w, clamp01(gain * T.fw_whistle * (hutnight_t >= 0 ? T.fw_finale : 1.0f)));
    SetSoundPitch(w, 0.9f + 0.2f * dust_rnd());
    PlaySound(w);
}
static void finale_music(int on)   /* the Hut night's song: start it from the top, or stop it */
{
    if (!snd.ready || !snd.has_finale) return;
    StopMusicStream(snd.finale); snd.finale_on = 0;
    if (on) { SetMusicVolume(snd.finale, clamp01(T.snd_music)); PlayMusicStream(snd.finale); snd.finale_on = 1; }
}
static void sound_finale(void)   /* every frame: the song plays on through to its end, the game's sounds turned down under it */
{
    if (!snd.finale_on) return;
    if (!IsMusicStreamPlaying(snd.finale) || yodel.active) { finale_music(0); return; }   /* done (or Drift Yodel's song takes over) */
    UpdateMusicStream(snd.finale);   /* (its volume: set in the frame, by the finale's part) */
    if (snd_duck > T.yodel_duck) snd_duck = T.yodel_duck;
}
static void sound_yodel(float dt)   /* every frame: the song while the mode is on, a short fade when it ends */
{
    if (!snd.ready || !snd.has_yodel) return;
    if (!yodel.active && snd.yodel_fade > 0) { snd.yodel_fade -= dt / 0.8f; if (snd.yodel_fade <= 0) { snd.yodel_fade = 0; StopMusicStream(snd.yodel); } }
    snd_duck = 1.0f - (1.0f - T.yodel_duck) * snd.yodel_fade;
    if (!IsMusicStreamPlaying(snd.yodel)) { snd_duck = 1.0f; sound_finale(); return; }
    SetMusicVolume(snd.yodel, clamp01(T.snd_music * snd.yodel_fade));
    UpdateMusicStream(snd.yodel);
}

/* exhaust flames: short-lived hot puffs from the tailpipe (left rear, under the tray) while the throttle is open */
#define FLAME_MAX 160
static struct { V3 p, v; float age, life, size; } flame[FLAME_MAX];
static int flame_n; static float flame_owed;
static V3 flame_tip; static float flame_glow;   /* play test: where Drift Yodel's fire comes out, and how bright its light (0..1) */
static void flames_update(const Car *c, float dt, int on)
{
    for (int i = 0; i < flame_n; ) {
        flame[i].age += dt;
        if (flame[i].age >= flame[i].life) { flame[i] = flame[--flame_n]; continue; }
        flame[i].p = vadd(flame[i].p, vscale(flame[i].v, dt));
        flame[i].v.y += 1.5f * dt;
        i++;
    }
    flame_glow += clampf((on ? 1.0f : 0.0f) - flame_glow, -dt * 5.0f, dt * 8.0f);
    if (!on || dt <= 0) { flame_owed = 0; return; }
    float rate = 25.0f + 70.0f * clamp01(c->throttle) + (c->changing_gear ? 120.0f : 0.0f);
    flame_owed += rate * dt;
    float rear = car_outline.own ? car_outline.rear : T.body_rear;
    V3 tip = vadd(c->body.pos, qrot(c->body.rot, v3(-0.35f, 0, rear - 0.1f)));
    tip.y = route_height(&RT, tip.x, tip.z, NULL) + 0.55f; flame_tip = vadd(tip, vscale(qrot(c->body.rot, v3(0, 0, -1)), 0.6f));
    V3 back = qrot(c->body.rot, v3(0, 0, -1));
    while (flame_owed >= 1.0f && flame_n < FLAME_MAX) {
        flame_owed -= 1.0f;
        float sp = 3.0f + 5.0f * dust_rnd();
        flame[flame_n].p = tip;
        flame[flame_n].v = vadd(vscale(c->body.vel, 0.8f), vadd(vscale(back, sp), v3((dust_rnd() - 0.5f) * 1.2f, 0.3f + 0.8f * dust_rnd(), (dust_rnd() - 0.5f) * 1.2f)));
        flame[flame_n].age = 0; flame[flame_n].life = 0.18f + 0.25f * dust_rnd(); flame[flame_n].size = 0.55f + 0.45f * dust_rnd();
        flame_n++;
    }
    if (flame_owed > 1.0f) flame_owed = 1.0f;
}
static void flames_draw(Camera3D cam)
{
    if (!flame_n) return;
    Vector3 f = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
    Vector3 r = Vector3Normalize(Vector3CrossProduct(f, cam.up)), u = Vector3CrossProduct(r, f);
    static const float ca[7] = { 1, 0.5f, -0.5f, -1, -0.5f, 0.5f, 1 }, sa[7] = { 0, 0.866f, 0.866f, 0, -0.866f, -0.866f, 0 };
    rlDisableBackfaceCulling();   /* the scene turns culling back on for the car */
    rlSetTexture(0);
    rlBegin(RL_TRIANGLES);
    for (int i = 0; i < flame_n; i++) {
        float t = flame[i].age / flame[i].life, h = flame[i].size * (1.0f - 0.6f * t);
        /* white-yellow at the pipe, through orange to a dark red edge */
        unsigned char cr = 255, cg = (unsigned char)(240 - 170 * t), cb = (unsigned char)(160 * (1.0f - t) * (1.0f - t)), al = (unsigned char)(255 * (1.0f - t * t));
        Vector3 c = RO(flame[i].p);
        for (int k = 0; k < 6; k++) {
            Vector3 p0 = Vector3Add(c, Vector3Add(Vector3Scale(r, h * ca[k]), Vector3Scale(u, h * sa[k])));
            Vector3 p1 = Vector3Add(c, Vector3Add(Vector3Scale(r, h * ca[k + 1]), Vector3Scale(u, h * sa[k + 1])));
            rlColor4ub(cr, cg, cb, al); rlVertex3f(c.x, c.y, c.z);
            rlColor4ub(230, 70, 20, al / 2); rlVertex3f(p1.x, p1.y, p1.z);
            rlColor4ub(230, 70, 20, al / 2); rlVertex3f(p0.x, p0.y, p0.z);
        }
    }
    rlEnd();
    rlDrawRenderBatchActive();
    rlEnableBackfaceCulling();
}

/* the sign of the horns, a pixel at a time: o outline, x skin, k knuckle line */
static void draw_horns(int x, int y, int px)
{
    static const char *art[14] = {
        " oo     oo ",
        "oxxo   oxxo",
        "oxxo   oxxo",
        "oxxo   oxxo",
        "oxxo   oxxo",
        "oxxoooooxxo",
        "oxxxkxkxxxo",
        "oxxxxxxxxxoo",
        "oxxxxxxxxxxo",
        "oxxkkkkkxxxo",
        "oxxxxxxxxxo",
        " oxxxxxxxo",
        " oxxxxxxxo",
        "  ooooooo" };
    const Color ol = { 30, 20, 20, 255 }, sk = { 245, 196, 140, 255 }, kn = { 196, 140, 96, 255 };
    for (int j = 0; j < 14; j++)
        for (int i = 0; art[j][i]; i++) {
            char ch = art[j][i];
            if (ch == ' ') continue;
            DrawRectangle(x + i * px, y + j * px, px, px, ch == 'o' ? ol : ch == 'k' ? kn : sk);
        }
}

/* ------------------------------------------------------------------ misc */

static const char *tuning_path = "data/tuning-live.txt"; /* optional, for tuning while driving (F5 reads, F6 writes); the built-in values otherwise */

/* The handling presets (ours, session 2): a few values over tuning-79.txt. H (or Menu+Down) cycles them
 * while driving; feel can't be measured, so the player picks. The skidpad, slide and road-bot numbers for each
 * are in the session 2 log. */
/* The handling presets (H): grip-only overrides from the handling itself (car_preset in src/car.c). */
#define NPRESETS CAR_NPRESETS
static int preset = 1, preset_given, want_preset;
static const char *preset_name(int p) { const char *n = "?"; car_preset(p, &n); return n; }
static void preset_apply(int p) { preset = (p % NPRESETS + NPRESETS) % NPRESETS; tuning_apply_text(car_preset(preset, NULL)); }

/* Drift Yodel mode on and off (build A): the drift preset for the minute, the player's own preset back after */
static void yodel_start(Car *c)
{
    if (yodel.active) return;
    yodel.active = 1; yodel.left = T.yodel_time; yodel.banner = 3.0f; yodel.saved_preset = preset;
    preset_apply(NPRESETS - 1); car_setup(c, anchors);   /* the last preset is drift */
    score_boost = T.yodel_score;
    yodel_music(1);
    if (getenv("SHOT_SEQ")) fprintf(stderr, "yodel on, %s handling (was %s)\n", preset_name(preset), preset_name(yodel.saved_preset));
}
static void yodel_stop(Car *c)
{
    if (!yodel.active) return;
    yodel.active = 0; yodel.banner = 0;
    preset_apply(yodel.saved_preset); car_setup(c, anchors);
    score_boost = 1.0f;
    if (getenv("SHOT_SEQ")) fprintf(stderr, "yodel off, %s handling\n", preset_name(preset));
}

/* Make a body the one in use: where the handling's wheels are, where they are drawn, and the
 * outline the walls use. The handling only ever sees anchors[]. */
static void select_model(int i)
{
    CarModel *m = &models[i];
    model_note = m->note;
    for (int w = 0; w < 4; w++) {
        anchors[w] = m->own[w];
        wheel_draw_off[w] = vsub(m->own[w], anchors[w]);
    }
    car_outline.own = m->has_outline;
    car_outline.half_w = m->half_w; car_outline.front = m->front; car_outline.rear = m->rear;
}

static int cam_snap;
static float crash_lost;   /* build A: what the crash took from the pot, for the results */
/* build B: the deer found flash and the achievement banners (one at a time, under the pot, not over the road ahead) */
static float deer_flash_t, army_flash_t; static int deer_flash_n;
static struct { int ach; float t; } ach_pop;
static void spawn_ex(Car *c, Score *s, int with_countdown)
{
    cam_snap = 1;
    rt_runs++; rt_max_s = 0; rt_finish_time = 0;
    run_reset(&run, with_countdown);
    car_reset(c, v3(RT.start_x, 0.75f, RT.start_z), RT.start_yaw); /* parked on the bare patch west of the hut, facing the track */
    cg.init = 0;
    marks_clear();
    if (s) memset(s, 0, sizeof(*s)), s->mult = 1;
    world_run_start();
    if (!explore) route_logs_restore(&RT, route_stream_invalidate);   /* follow-up: the logs broken last run are back (build C: in Drift only) */
    splinter_n = 0;
    wood_flash_t = deer_flash_t = army_flash_t = 0;
    if (T.deer_random != 0 && rt_runs > 1) { deer_place(1); W.found = 0; deer_meshes_init(); }   /* new spots every run: the count is this run's */
}
static void spawn(Car *c, Score *s) { spawn_ex(c, s, 0); }

/* ------------------------------------------------------------- Explorer (build C) */
/* Explorer mode: free roaming. No score, pot, checkpoints, reset or game over; a hard hit stops the ute with a thud and
 * you reverse out. The road both ways, the hut's clearing, and (from the clearing) the creek way. What it keeps is in the
 * progress file with the deer and the achievements (world.c): the firewood in the tray, the fires and the time of day,
 * the hamper, Quayle's rifle, the logs splintered. */
static void web_storage_flush(void);
static float shake_t;   /* the camera's shake after a thud (seconds left) */
static int stream_on;

/* The fire, the fireworks, the wood thrown on, the deer's eyes in the headlights: one pool of bright particles, drawn unlit
 * (they make their own light) after the scene. */
#define FX_MAX 1400
enum { FX_FLAME, FX_SPARK, FX_ROCKET, FX_STAR, FX_LOG, FX_GLOW };
typedef struct { V3 p, v; float age, life, size; unsigned char r, g, b, kind; } Fx;
static Fx fx[FX_MAX]; static int fx_n;
static float fx_owed[3], fw_next, fw_small_next, fw_flash; static V3 fw_flash_at; static Color fw_flash_c;
static struct { float t; float gain; } bang_q[16]; static int bang_n;
static V3 chimney_top(void)
{
    float x = RT.hut_x - 5.64f, z = RT.hut_z + 0.74f;
    return v3(x, route_height(&RT, RT.hut_x, RT.hut_z, NULL) + 6.9f, z);
}
static V3 fire_at(void) { return v3(W.fire_x, route_height(&RT, W.fire_x, W.fire_z, NULL), W.fire_z); }
/* play test: where the moon is (a direction, world axes): from the finale's fireworks camera (south-east of the chimney,
 * 25 m out, 4.7 m up) about 44 degrees up and 17 m to the right of the bursts, where the sky is clear of the crowns, so that shot has it in the sky over them */
static V3 moon_dir(void)
{
    static V3 md; static int have;
    if (!have && rt_have) {
        V3 ct = chimney_top(); float an = -0.9f, gy = route_height(&RT, ct.x, ct.z, NULL);
        V3 cp = v3(ct.x + cosf(an) * 25.0f, 0, ct.z + sinf(an) * 25.0f); cp.y = route_height(&RT, cp.x, cp.z, NULL) + 4.7f;
        V3 fwd = vnorm(v3(ct.x - cp.x, 0, ct.z - cp.z)), side = v3(-fwd.z, 0, fwd.x);
        V3 p = vadd(v3(ct.x, gy, ct.z), vadd(vscale(side, -17.0f), v3(0, 4.7f + 25.0f * tanf(0.76f), 0)));
        md = vnorm(vsub(p, cp)); have = 1;
    }
    return have ? md : vnorm(v3(0.4f, 0.7f, -0.6f));
}
static Fx *fx_new(int kind)
{
    if (fx_n >= FX_MAX) return NULL;
    Fx *f = &fx[fx_n++]; memset(f, 0, sizeof(*f)); f->kind = (unsigned char)kind;
    return f;
}
static const unsigned char fw_cols[6][3] = { { 255, 80, 60 }, { 90, 230, 120 }, { 100, 200, 255 }, { 240, 80, 200 }, { 255, 214, 120 }, { 255, 250, 235 } };
static void fw_burst(V3 at, int n, float speed, int col)
{
    for (int k = 0; k < n; k++) {
        Fx *f = fx_new(FX_STAR); if (!f) break;
        float u = dust_rnd() * 2.0f - 1.0f, a = dust_rnd() * 6.2831853f, r = sqrtf(fmaxf(1.0f - u * u, 0.0f)), sp = speed * (0.75f + 0.25f * dust_rnd());
        f->p = at; f->v = v3(cosf(a) * r * sp, u * sp, sinf(a) * r * sp);
        f->life = 1.5f + 0.9f * dust_rnd(); f->size = 0.9f + 0.4f * dust_rnd();
        int c = (dust_rnd() < 0.2f) ? 5 : col; f->r = fw_cols[c][0]; f->g = fw_cols[c][1]; f->b = fw_cols[c][2];
    }
    fw_flash = 1.0f; fw_flash_at = at; fw_flash_c = (Color){ fw_cols[col][0], fw_cols[col][1], fw_cols[col][2], 255 };
}
static void fx_update(const Car *c, Camera3D *cam_unused, float dt)
{
    (void)cam_unused;
    for (int i = 0; i < fx_n; ) {
        Fx *f = &fx[i];
        f->age += dt;
        if (f->age >= f->life) {
            if (f->kind == FX_ROCKET) { V3 at = f->p; int col = (int)(dust_rnd() * 5.0f) % 5; fx[i] = fx[--fx_n]; fw_burst(at, (int)((55.0f + dust_rnd() * 30.0f) * T.fx_density), 12.0f + 5.0f * dust_rnd(), col);
                float dd = vlen(vsub(at, c->body.pos)); if (bang_n < 16) { bang_q[bang_n].t = dd / 340.0f; bang_q[bang_n].gain = clampf(140.0f / (dd + 40.0f), 0.15f, 1.0f); bang_n++; } continue; }
            fx[i] = fx[--fx_n]; continue;
        }
        if (f->kind == FX_FLAME) { f->v.y += 1.2f * dt; f->v = vscale(f->v, expf(-1.2f * dt)); }
        else if (f->kind == FX_SPARK) { f->v.y -= 1.5f * dt; f->v.x += (dust_rnd() - 0.5f) * 3.0f * dt; f->v.z += (dust_rnd() - 0.5f) * 3.0f * dt; }
        else if (f->kind == FX_ROCKET) { f->v.y -= 6.0f * dt; if (dust_rnd() < 0.35f) { Fx *t = fx_new(FX_SPARK); if (t) { t->p = f->p; t->v = v3((dust_rnd() - 0.5f), -1.0f, (dust_rnd() - 0.5f)); t->life = 0.45f; t->size = 0.22f; t->r = 255; t->g = 210; t->b = 140; } } }
        else if (f->kind == FX_STAR) { f->v.y -= 3.0f * dt; f->v = vscale(f->v, expf(-1.5f * dt)); }
        else if (f->kind == FX_LOG) { f->v.y -= 9.8f * dt; }
        f->p = vadd(f->p, vscale(f->v, dt));
        i++;
    }
    if (fw_flash > 0) fw_flash = fmaxf(fw_flash - dt * 2.5f, 0.0f);
    for (int b = 0; b < bang_n; ) { bang_q[b].t -= dt; if (bang_q[b].t <= 0) { syn.bang_gain = bang_q[b].gain; syn.bang_id++; bang_q[b] = bang_q[--bang_n]; continue; } b++; }
    if (!explore || dt <= 0) { syn.crackle = 0; return; }
    /* the wood thrown on: the tray's logs arc onto the fire over a second and a half */
    if (throw_t >= 0) {
        if (throw_t == 0) {
            V3 dst = fire_at();
            for (int k = 0; k < (int)T.wood_full; k++) {
                Fx *f = fx_new(FX_LOG); if (!f) break;
                V3 from = vadd(c->body.pos, qrot(c->body.rot, v3(-0.5f + 0.25f * (float)(k % 5), 0.9f + 0.15f * (float)(k / 5), -2.3f)));
                from.y = route_height(&RT, from.x, from.z, NULL) + 0.9f + 0.15f * (float)(k / 5);
                float T0 = 0.7f + 0.05f * (float)k + 0.2f * dust_rnd();
                V3 to = vadd(dst, v3((dust_rnd() - 0.5f) * 1.2f, 0.4f, (dust_rnd() - 0.5f) * 1.2f));
                f->p = from; f->v = v3((to.x - from.x) / T0, (to.y - from.y) / T0 + 0.5f * 9.8f * T0, (to.z - from.z) / T0);
                f->life = T0; f->size = 0.9f; f->r = 150; f->g = 110; f->b = 70;
            }
        }
        throw_t += dt;
        if (throw_t > 1.9f) { throw_t = -1; if (throw_level >= 3) fireworks_t = 0; }   /* the third load lands: the fireworks */
    }
    int lv = W.fire_level - (throw_t >= 0 || (cut_t >= 0 && cut_t < CUT_FIRE) ? 1 : 0);
    for (int l = 0; l < 3; l++) fire_grow[l] = lv > l ? fminf(fire_grow[l] + dt / 4.0f, 1.0f) : 0.0f;
    if (cut_t >= 7.0f && cut_level >= 3 && fireworks_t < 0) fireworks_t = 0;   /* the third load: the fireworks, while the cutscene holds */
    V3 fa = fire_at();
    float dfire = vlen(vsub(fa, c->body.pos));
    syn.crackle = lv > 0 ? clamp01((0.5f + 0.25f * (float)lv) * 30.0f / (dfire + 12.0f)) : 0.0f;
    if ((W.stew || getenv("SHOT_STEW")) && dust_n < DUST_MAX && dust_rnd() < dt * 5.0f && vlen(vsub(oven_at, c->body.pos)) < 120.0f) {   /* the stew steaming */
        dust[dust_n].p = vadd(oven_at, v3((dust_rnd() - 0.5f) * 0.15f, 0, (dust_rnd() - 0.5f) * 0.15f));
        dust[dust_n].v = v3((dust_rnd() - 0.5f) * 0.1f, 0.35f + 0.2f * dust_rnd(), (dust_rnd() - 0.5f) * 0.1f);
        dust[dust_n].age = 0; dust[dust_n].life = 2.2f + dust_rnd(); dust[dust_n].size = 0.35f + 0.2f * dust_rnd(); dust[dust_n].a0 = 0.45f;
        dust_n++;
    }
    if (lv > 0) {   /* flames and sparks */
        float g = fire_grow[lv - 1], R = lv == 1 ? 0.4f : lv == 2 ? 0.75f : 1.3f;
        fx_owed[0] += (lv == 1 ? 45.0f : lv == 2 ? 80.0f : 140.0f) * (0.4f + 0.6f * g) * dt * T.fx_density;
        fx_owed[1] += (4.0f + 6.0f * (float)lv) * dt * T.fx_density;
        while (fx_owed[0] >= 1.0f) { fx_owed[0] -= 1.0f; Fx *f = fx_new(FX_FLAME); if (!f) break;
            float a = dust_rnd() * 6.2831853f, r = R * sqrtf(dust_rnd()) * 0.75f;
            f->p = v3(fa.x + cosf(a) * r, fa.y + 0.15f, fa.z + sinf(a) * r);
            f->v = v3((dust_rnd() - 0.5f) * 0.4f, (1.2f + 1.6f * dust_rnd()) * (0.8f + 0.35f * (float)lv) * (0.5f + 0.5f * g), (dust_rnd() - 0.5f) * 0.4f);
            f->life = (0.45f + 0.35f * dust_rnd()) * (0.9f + 0.25f * (float)lv); f->size = (0.45f + 0.35f * dust_rnd()) * (0.7f + 0.35f * (float)lv); }
        while (fx_owed[1] >= 1.0f) { fx_owed[1] -= 1.0f; Fx *f = fx_new(FX_SPARK); if (!f) break;
            f->p = v3(fa.x + (dust_rnd() - 0.5f) * R, fa.y + 0.6f, fa.z + (dust_rnd() - 0.5f) * R);
            f->v = v3((dust_rnd() - 0.5f) * 1.2f, 2.5f + 3.5f * dust_rnd() * (float)lv * 0.6f, (dust_rnd() - 0.5f) * 1.2f);
            f->life = 1.2f + 1.4f * dust_rnd(); f->size = 0.22f; f->r = 255; f->g = 190 + (unsigned char)(50 * dust_rnd()); f->b = 90; }
    }
    if (fireworks_t >= 0) {   /* the fireworks out of the chimney: about 30 s of bursts, then a slower sparkle that keeps going */
        fireworks_t += dt;
        V3 ct = chimney_top();
        if (fireworks_t < 30.0f) {
            fw_next -= dt;
            if (fw_next <= 0) { { float dd = vlen(vsub(ct, c->body.pos)); sound_whistle(clampf(120.0f / (dd + 40.0f), 0.12f, 1.0f)); }   /* its whistle */
                                Fx *f = fx_new(FX_ROCKET); if (f) { f->p = ct; f->v = v3((dust_rnd() - 0.5f) * 7.0f, 21.0f + 7.0f * dust_rnd(), (dust_rnd() - 0.5f) * 7.0f); f->life = 1.2f + 0.5f * dust_rnd(); f->size = 0.6f; f->r = 255; f->g = 230; f->b = 180; }
                                fw_next = 0.45f + 0.7f * dust_rnd(); if (fireworks_t > 24.0f) fw_next *= 0.5f; }   /* a quicker finale */
        } else {
            fx_owed[2] += 18.0f * dt * T.fx_density;
            while (fx_owed[2] >= 1.0f) { fx_owed[2] -= 1.0f; Fx *f = fx_new(FX_SPARK); if (!f) break;
                f->p = ct; f->v = v3((dust_rnd() - 0.5f) * 2.5f, 4.0f + 3.0f * dust_rnd(), (dust_rnd() - 0.5f) * 2.5f); f->life = 1.6f + dust_rnd(); f->size = 0.3f;
                int k = (int)(dust_rnd() * 3.0f); f->r = k == 0 ? 255 : k == 1 ? 255 : 160; f->g = k == 0 ? 214 : k == 1 ? 160 : 220; f->b = k == 2 ? 255 : 120; }
            fw_small_next -= dt;
            if (fw_small_next <= 0) { fw_burst(vadd(ct, v3((dust_rnd() - 0.5f) * 8.0f, 22.0f + 10.0f * dust_rnd(), (dust_rnd() - 0.5f) * 8.0f)), 28, 7.0f, (int)(dust_rnd() * 5.0f) % 5); fw_small_next = 4.0f + 3.0f * dust_rnd();
                float dd = vlen(vsub(ct, c->body.pos)); if (bang_n < 16) { bang_q[bang_n].t = dd / 340.0f; bang_q[bang_n].gain = clampf(60.0f / (dd + 40.0f), 0.1f, 0.5f); bang_n++; } }
        }
    }
}
typedef struct { float x, z; char t[32]; Color c; } Marker;   /* pictures: a labelled spot */
static float night_dim(void) { return 1.0f - 0.8f * clamp01(night_now); }
static float creek_sink_now;   /* how far the far land sinks along the creek way (0 unless you're in it) */
/* The haze by the time of day: golden hour, a mauve dusk, a blue-black night. */
static void haze_colour(float k, float *hc, float *hsun)
{
    static const float day[3] = { 227 / 255.0f, 199 / 255.0f, 160 / 255.0f }, dusk[3] = { 0.42f, 0.30f, 0.40f }, nite[3] = { 0.025f, 0.03f, 0.06f };
    static const float dsun[3] = { 246 / 255.0f, 196 / 255.0f, 140 / 255.0f }, ssun[3] = { 0.80f, 0.42f, 0.30f };
    for (int i = 0; i < 3; i++) {
        hc[i] = k < 0.5f ? day[i] + (dusk[i] - day[i]) * k * 2.0f : dusk[i] + (nite[i] - dusk[i]) * (k - 0.5f) * 2.0f;
        if (hsun) hsun[i] = k < 0.5f ? dsun[i] + (ssun[i] - dsun[i]) * k * 2.0f : ssun[i] + (nite[i] - ssun[i]) * (k - 0.5f) * 2.0f;
    }
}
static void fx_draw(Camera3D cam, const Car *c, float night)
{
    Vector3 f = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
    Vector3 r = Vector3Normalize(Vector3CrossProduct(f, cam.up)), u = Vector3CrossProduct(r, f);
    rlDisableBackfaceCulling(); rlSetTexture(0);
    rlBegin(RL_TRIANGLES);
    for (int i = 0; i < fx_n; i++) {
        const Fx *p = &fx[i];
        float t = p->age / p->life;
        Vector3 cc = RO(p->p);
        if (p->kind == FX_LOG) {   /* a split log tumbling through the air */
            float h = 0.35f, w = 0.09f, an = p->age * 9.0f + (float)i;
            Vector3 d = Vector3Add(Vector3Scale(r, cosf(an) * h), Vector3Scale(u, sinf(an) * h)), e = Vector3Add(Vector3Scale(r, -sinf(an) * w), Vector3Scale(u, cosf(an) * w));
            unsigned char lr = (unsigned char)(p->r * (0.3f + 0.7f * night_dim())), lg = (unsigned char)(p->g * (0.3f + 0.7f * night_dim())), lb = (unsigned char)(p->b * (0.3f + 0.7f * night_dim()));
            Vector3 q0 = Vector3Subtract(Vector3Subtract(cc, d), e), q1 = Vector3Add(Vector3Subtract(cc, d), e), q2 = Vector3Add(Vector3Add(cc, d), e), q3 = Vector3Subtract(Vector3Add(cc, d), e);
            rlColor4ub(lr, lg, lb, 255); rlVertex3f(q0.x, q0.y, q0.z); rlVertex3f(q1.x, q1.y, q1.z); rlVertex3f(q2.x, q2.y, q2.z);
            rlVertex3f(q0.x, q0.y, q0.z); rlVertex3f(q2.x, q2.y, q2.z); rlVertex3f(q3.x, q3.y, q3.z);
            continue;
        }
        float h = p->size * 0.5f, a; unsigned char cr, cg, cb;
        if (p->kind == FX_FLAME) { h *= 1.0f - 0.55f * t; cr = 255; cg = (unsigned char)(235 - 175 * t); cb = (unsigned char)(150 * (1.0f - t) * (1.0f - t)); a = 1.0f - t * t; }
        else if (p->kind == FX_STAR) { cr = p->r; cg = p->g; cb = p->b; a = t < 0.7f ? 1.0f : (1.0f - t) / 0.3f; if (t > 0.6f && ((i + (int)(p->age * 20.0f)) & 1)) a *= 0.4f; }
        else { cr = p->r; cg = p->g; cb = p->b; a = 1.0f - t; }
        Vector3 rr = Vector3Scale(r, h), uu = Vector3Scale(u, h);
        unsigned char al = (unsigned char)(255 * clamp01(a));
        if (p->kind == FX_STAR || p->kind == FX_ROCKET) {   /* a trail behind it: two fainter, smaller sparks back along its path */
            for (int tk = 1; tk <= 2; tk++) {
                Vector3 tc = RO(vsub(p->p, vscale(p->v, 0.055f * (float)tk))); float th = h * (1.0f - 0.3f * (float)tk);
                Vector3 r2 = Vector3Scale(r, th), u2 = Vector3Scale(u, th); unsigned char a2 = (unsigned char)(al / (tk + 1));
                rlColor4ub(cr, cg, cb, a2); rlVertex3f(tc.x - r2.x - u2.x, tc.y - r2.y - u2.y, tc.z - r2.z - u2.z); rlVertex3f(tc.x + r2.x - u2.x, tc.y + r2.y - u2.y, tc.z + r2.z - u2.z); rlVertex3f(tc.x + r2.x + u2.x, tc.y + r2.y + u2.y, tc.z + r2.z + u2.z);
                rlVertex3f(tc.x - r2.x - u2.x, tc.y - r2.y - u2.y, tc.z - r2.z - u2.z); rlVertex3f(tc.x + r2.x + u2.x, tc.y + r2.y + u2.y, tc.z + r2.z + u2.z); rlVertex3f(tc.x - r2.x + u2.x, tc.y - r2.y + u2.y, tc.z - r2.z + u2.z);
            }
        }
        rlColor4ub(cr, cg, cb, al); rlVertex3f(cc.x - rr.x - uu.x, cc.y - rr.y - uu.y, cc.z - rr.z - uu.z); rlVertex3f(cc.x + rr.x - uu.x, cc.y + rr.y - uu.y, cc.z + rr.z - uu.z); rlVertex3f(cc.x + rr.x + uu.x, cc.y + rr.y + uu.y, cc.z + rr.z + uu.z);
        rlVertex3f(cc.x - rr.x - uu.x, cc.y - rr.y - uu.y, cc.z - rr.z - uu.z); rlVertex3f(cc.x + rr.x + uu.x, cc.y + rr.y + uu.y, cc.z + rr.z + uu.z); rlVertex3f(cc.x - rr.x + uu.x, cc.y - rr.y + uu.y, cc.z - rr.z + uu.z);
    }
    if (explore && lamp_mesh.vertexCount && night > 0.15f) {   /* play test: the lantern's glow, a soft halo round the globe */
        float on = smooth_step01((night - 0.15f) / 0.35f);
        Vector3 lc = RO(lamp_at);
        for (int k = 0; k < 2; k++) {   /* a bright core and a wide faint halo, each a disc fading to its rim */
            float h = k ? 0.75f : 0.14f; unsigned char al = (unsigned char)((k ? 90 : 240) * on);
            for (int j = 0; j < 12; j++) {
                float a0 = (float)j * 0.5235988f, a1 = a0 + 0.5235988f;
                Vector3 p0 = Vector3Add(lc, Vector3Add(Vector3Scale(r, cosf(a0) * h), Vector3Scale(u, sinf(a0) * h))), p1 = Vector3Add(lc, Vector3Add(Vector3Scale(r, cosf(a1) * h), Vector3Scale(u, sinf(a1) * h)));
                rlColor4ub(255, 214, 140, al); rlVertex3f(lc.x, lc.y, lc.z);
                rlColor4ub(255, 200, 120, 0); rlVertex3f(p0.x, p0.y, p0.z); rlVertex3f(p1.x, p1.y, p1.z);
            }
        }
    }
    /* build C follow-up: the things to collect glow: a soft column of light and a few sparkles circling each, pulsing,
     * so they read as pick-ups from a way off (the hamper's twelve and Quayle's rifle) */
    if (explore && T.find_glow != 0) {
        float tm = (float)GetTime();
        /* dim in daylight (a quarter at golden hour), full from dusk: the hamper's easier found once the fire's going */
        float nn = night_now < 0 ? W.night : night_now, day = 0.25f + 0.75f * clamp01(nn / 0.5f);
        for (int i = 0; i <= HAMPER_N; i++) {
            float gx, gz;
            if (i < HAMPER_N) { if (((W.hamper >> i) & 1) || W.board || !W.item[i].placed) continue; gx = W.item[i].x; gz = W.item[i].z; }
            else { if (W.rifle != 0 || !W.rifle_placed) continue; gx = W.rifle_x; gz = W.rifle_z; }
            V3 b = v3(gx, 0, gz);
            float dc = hypotf(gx - (cam.position.x + ro.x), gz - (ro.z - cam.position.z));
            if (dc > 160.0f) continue;
            int keep = RT.explore; RT.explore = 1; b.y = route_height(&RT, gx, gz, NULL); RT.explore = keep;
            float pulse = 0.75f + 0.25f * sinf(tm * 3.0f + (float)i), near = clamp01((dc - 2.5f) / 4.0f);   /* fades as you drive into it */
            float dayk = i == HAMPER_N ? 1.0f : day, top = i == HAMPER_N ? 1.3f : 2.8f;   /* the rifle: a short bright pool of light under it (it floats above) */   /* build D: Quayle's rifle full strength by day, and taller (it's down a shady creek) */
            /* the column: a camera-facing strip, brightest at the foot, fading out 2.8 m up (the rifle's 4 m) */
            Vector3 c0 = RO(v3(b.x, b.y + 0.05f, b.z)), c1 = RO(v3(b.x, b.y + top, b.z));
            float w = (0.32f + 0.0045f * dc) * (i == HAMPER_N ? 2.2f : 1.0f);   /* the rifle's wider */
            Vector3 rw = Vector3Scale(r, w);
            unsigned char a0 = (unsigned char)((i == HAMPER_N ? 255.0f * (0.85f + 0.15f * pulse) : 215.0f * pulse) * near * dayk);
            unsigned char gg = i == HAMPER_N ? 170 : 228, gb = i == HAMPER_N ? 50 : 130;   /* the rifle's a warmer orange: it shows against the pale gravel by day */
            rlColor4ub(255, gg, gb, a0); rlVertex3f(c0.x - rw.x, c0.y, c0.z - rw.z); rlVertex3f(c0.x + rw.x, c0.y, c0.z + rw.z);
            rlColor4ub(255, gg, gb, 0); rlVertex3f(c1.x + rw.x, c1.y, c1.z + rw.z);
            rlColor4ub(255, gg, gb, a0); rlVertex3f(c0.x - rw.x, c0.y, c0.z - rw.z);
            rlColor4ub(255, gg, gb, 0); rlVertex3f(c1.x + rw.x, c1.y, c1.z + rw.z); rlVertex3f(c1.x - rw.x, c1.y, c1.z - rw.z);
            for (int k = 0; k < 5; k++) {   /* sparkles circling and rising */
                float ph = tm * 1.3f + (float)k * 1.257f, rise = fmodf(tm * 0.6f + (float)k * 0.37f, 1.0f);
                V3 sp = v3(b.x + cosf(ph) * 0.65f, b.y + 0.25f + 1.3f * rise, b.z + sinf(ph) * 0.65f);
                Vector3 sc = RO(sp); float h = (0.06f + 0.0012f * dc) * (i == HAMPER_N ? 2.5f : 1.0f);
                unsigned char al = (unsigned char)(255 * (1.0f - rise) * near * dayk);
                rlColor4ub(255, 250, 210, al);
                rlVertex3f(sc.x - r.x * h, sc.y - h, sc.z - r.z * h); rlVertex3f(sc.x + r.x * h, sc.y - h, sc.z + r.z * h); rlVertex3f(sc.x + r.x * h, sc.y + h, sc.z + r.z * h);
                rlVertex3f(sc.x - r.x * h, sc.y - h, sc.z - r.z * h); rlVertex3f(sc.x + r.x * h, sc.y + h, sc.z + r.z * h); rlVertex3f(sc.x - r.x * h, sc.y + h, sc.z - r.z * h);
            }
        }
    }
    /* the deer's eyes, shining green-white in the headlights at night */
    if (night > 0.45f && c) {
        V3 cf = qrot(c->body.rot, v3(0, 0, 1)), cp = vadd(c->body.pos, vscale(cf, 2.2f));
        for (int d = 0; d < WORLD_DEER + W.nextra; d++) {
            if (d >= W.ndeer && d < WORLD_DEER) continue;
            if (!deer_out(d)) continue;
            const Deer *dr = &W.deer[d];
            int stag = dr->variant >= 2, graze = dr->variant == 1 || dr->variant == 3; float k = stag ? 1.12f : 1.0f;
            float ex = graze ? 1.25f * k : 1.22f * k, ey = graze ? 0.42f : 1.86f * k;
            float hy = route_height(&RT, dr->x, dr->z, NULL), cy = cosf(dr->yaw), sy = sinf(dr->yaw);
            for (int e = -1; e <= 1; e += 2) {
                float px = ex, pz = 0.07f * (float)e;
                V3 eye = v3(dr->x + px * cy - pz * sy, hy + ey, dr->z + px * sy + pz * cy);
                V3 to = vsub(eye, cp); float dd = vlen(to);
                if (dd > 70.0f || dd < 2.0f || vdot(vscale(to, 1.0f / dd), cf) < 0.9f) continue;
                Vector3 ec = RO(eye); float hh = 0.07f + dd * 0.0025f;
                unsigned char al = (unsigned char)(255 * clamp01((night - 0.45f) * 3.0f) * clamp01((70.0f - dd) / 20.0f));
                rlColor4ub(210, 255, 200, al);
                rlVertex3f(ec.x - r.x * hh, ec.y - hh, ec.z - r.z * hh); rlVertex3f(ec.x + r.x * hh, ec.y - hh, ec.z + r.z * hh); rlVertex3f(ec.x + r.x * hh, ec.y + hh, ec.z + r.z * hh);
                rlVertex3f(ec.x - r.x * hh, ec.y - hh, ec.z - r.z * hh); rlVertex3f(ec.x + r.x * hh, ec.y + hh, ec.z + r.z * hh); rlVertex3f(ec.x - r.x * hh, ec.y + hh, ec.z - r.z * hh);
            }
        }
    }
    rlEnd(); rlDrawRenderBatchActive(); rlEnableBackfaceCulling();
}

static void mode_enter(int ex)
{
    int changed = RT.explore != ex || !stream_on;
    explore = ex; W.mode = ex ? MODE_EXPLORE : MODE_DRIFT; RT.explore = ex;
    route_logs_restore(&RT, NULL);
    if (ex) for (int g = 0; g < W.nlogs_gone; g++) route_log_break_at(&RT, W.logs_gone[g][0], W.logs_gone[g][1]);   /* still gone */
    if (ex && W.fire_level >= 3 && fireworks_t < 0) fireworks_t = 30.5f;   /* the evening going on: the chimney's slow sparkle */
    if (changed && rt_have) { route_stream_stop(); rt_slots_clear(); route_stream_start(&RT); stream_on = 1; }   /* the chunks differ by mode */
    else if (rt_have) for (int c = 0; c < RT.nchunks; c++) route_stream_invalidate(c);
    W.dirty = 1;
}
/* A hit that only stops the ute (Explorer): a small push back off what it hit, the spin taken out. */
static void explore_thud(Car *c, const RtPhys *ph)
{
    if (ph->tree < 0) return;
    const RtTree *t = &RT.tree[ph->tree];
    float dx = c->body.pos.x - t->x, dz = c->body.pos.z - t->z, d = fmaxf(hypotf(dx, dz), 1e-3f);
    c->body.vel = v3(dx / d * 0.6f, 0, dz / d * 0.6f);
    c->body.angvel = v3(0, 0, 0);
}
static void puff(V3 at, int n, float spread, float up)   /* a burst of dust: a find, a thud */
{
    for (int k = 0; k < n && dust_n < DUST_MAX; k++) {
        dust[dust_n].p = vadd(at, v3((dust_rnd() - 0.5f) * spread, dust_rnd() * up, (dust_rnd() - 0.5f) * spread));
        dust[dust_n].v = v3((dust_rnd() - 0.5f) * 5.0f, 1.0f + 2.5f * dust_rnd(), (dust_rnd() - 0.5f) * 5.0f);
        dust[dust_n].age = 0; dust[dust_n].life = 0.9f + 0.7f * dust_rnd(); dust[dust_n].size = 1.4f + 1.2f * dust_rnd(); dust[dust_n].a0 = 0.9f;
        dust_n++;
    }
}
/* build D: a stag lands: a crash and a cloud of dust along it */
static void stag_crash(int k)
{
    const struct RtStag *g = &RT.stag[k];
    for (int q = 0; q < 3; q++) puff(v3(g->log_x[q], route_height(&RT, g->log_x[q], g->log_z[q], NULL) + 0.4f, g->log_z[q]), 18, 2.0f, 1.2f);
    syn.bang_gain = 0.55f; syn.bang_id++;
    synth_blip2(4, 0.7f, 1.0f);
}
/* Explorer's events this step: the hamper, the rifle, the fire, the board. Each one flashes a line, sounds and is saved. */
static float ex_flash_t, ex_flash_len = 2.4f; static char ex_flash[64], ex_flash_sub[96]; static Color ex_flash_c;
/* The creek way, for the rules (Off the beaten track) and for what's streamed and drawn: in its strip, how far down it. */
static void explore_creek_step(WorldStep *ws, float x, float z)
{
    ws->in_creek = 0; ws->creek_s = 0; ws->creek_len = RT.ck_len; ws->creek_gap_s = RT.ck_exit_s;
    if (!explore || RT.ck_n < 2) return;
    RtNear cn = route_creek_near(&RT, x, z);
    ws->creek_s = cn.s; ws->in_creek = route_in_creek(&RT, x, z);
}
/* Each frame: the creek way's chunks are wanted only near its way in or in it (Explorer), and the far land along it sinks
 * out of the way while you're down there. */
static void creek_frame(V3 at, float dt)
{
    int want = 0, in = 0;
    if (explore && RT.ck_n > 1) {
        RtNear cn = route_creek_near(&RT, at.x, at.z);
        in = route_in_creek(&RT, at.x, at.z) && cn.s > RT.ck_exit_s - 10.0f;
        want = cn.dist < 160.0f && cn.s > RT.ck_exit_s - 220.0f;
        if (in) W.creek_seen = 1;
    }
    RT.creek_stream = want || getenv("BD_CREEK_ALL") != NULL;
    creek_sink_now += clampf((in ? 1.0f : 0.0f) - creek_sink_now, -dt * 1.5f, dt * 1.5f);
}
static void ex_say(const char *t, Color c) { snprintf(ex_flash, sizeof(ex_flash), "%s", t); ex_flash_sub[0] = 0; ex_flash_t = ex_flash_len = 3.5f; ex_flash_c = c; }
/* a message for after this one (and after any cutscene) */
static char ex_next[64], ex_next_sub[96]; static Color ex_next_c;
static void ex_then(const char *t, const char *sub, Color c) { snprintf(ex_next, sizeof(ex_next), "%s", t); snprintf(ex_next_sub, sizeof(ex_next_sub), "%s", sub); ex_next_c = c; }
/* a find and a line under it (what Quayle says, what to do next): held longer so it can be read */
static void ex_say2(const char *t, const char *sub, Color c) { ex_say(t, c); snprintf(ex_flash_sub, sizeof(ex_flash_sub), "%s", sub); ex_flash_t = ex_flash_len = 6.0f; }
/* ---- build E: Creek Trials, the trial gates and the ute's damage (Explorer). Over each of the creek way's obstacles (the
 * stumps, small logs, boulders and mossy fallen trees), slowly or in Low Range and with no knock, scores into a pot; clean
 * ones in a row raise its multiplier. A knock, a hard landing or a rollover loses the pot. It banks at the camp and at the
 * ford; the run's banked total is Creek Trials' score (W.trials_best, the best run). The gates, flagged stake pairs set across
 * awkward lines over obstacles (fresh each day), give a bonus through cleanly; clipping a stake bends it and costs points.
 * Knocks and hard landings wear the tyre at that corner and the body; a tyre worn through is a flat (it pulls and drags) till
 * the spare goes on (pause menu); the hut fixes everything and puts a spare in the tray, as does a new day. ---- */
#define GATE_MAX 16
typedef struct { float x[2], z[2], cx, cz, tx, tz, s, lane; int bent[2], done; float prev_along; } Gate;   /* lane: the doable line, metres left of the middle */
static Gate gate[GATE_MAX]; static int ngate = 0, gate_day = -1;
static int *obs_idx, nobs; static float *obs_s; static unsigned char *obs_done;
typedef struct { float pot, banked, fault_t, msg_t, fault_msg_t; int mult, streak, in_ck, nclean, from_hut, clean_run, no_spare_said; } Trials;
static Trials TR;
static int obs_base(int type) { return type == BIT_MOSSY ? 60 : type == BIT_BOULDER ? 30 : type == BIT_SMALLLOG ? 20 : type == BIT_STUMP ? 15 : 0; }
static void obs_init(void)
{
    if (obs_idx || RT.ck_nbit <= 0) return;
    obs_idx = malloc(sizeof(int) * (size_t)RT.ck_nbit); obs_s = malloc(sizeof(float) * (size_t)RT.ck_nbit); obs_done = calloc((size_t)RT.ck_nbit, 1);
    for (int b = 0; b < RT.ck_nbit; b++) if (obs_base(RT.ck_bit[b].type) > 0) { RtNear cn = route_creek_near(&RT, RT.ck_bit[b].x, RT.ck_bit[b].z); obs_idx[nobs] = b; obs_s[nobs++] = cn.s; }
    for (int i = 1; i < nobs; i++) { int bi = obs_idx[i]; float si = obs_s[i]; int j = i - 1; while (j >= 0 && obs_s[j] > si) { obs_idx[j + 1] = obs_idx[j]; obs_s[j + 1] = obs_s[j]; j--; } obs_idx[j + 1] = bi; obs_s[j + 1] = si; }
}
/* the gates for today: one in each of gates_n stretches of the creek way, across an obstacle, seeded by the day */
static void gates_place(void)
{
    obs_init(); gate_day = W.day; ngate = 0;
    if (RT.ck_n < 2 || nobs == 0) return;
    int n = (int)T.gates_n; if (n > GATE_MAX) n = GATE_MAX;
    float s0 = RT.ck_exit_s + 250.0f, s1 = RT.ck_len - 60.0f;
    uint32_t sd = 0x5eed1u + (uint32_t)W.day * 7919u;
    for (int g = 0; g < n; g++) {
        float a = s0 + (s1 - s0) * (float)g / (float)n, b = s0 + (s1 - s0) * (float)(g + 1) / (float)n;
        int lo = 0; while (lo < nobs && obs_s[lo] < a) lo++;
        int hi = lo; while (hi < nobs && obs_s[hi] < b) hi++;
        if (hi <= lo) continue;
        for (int t = 0; t < 24; t++) {
            sd = sd * 1664525u + 1013904223u; int k = lo + (int)((sd >> 8) % (uint32_t)(hi - lo));
            const struct RtBit *q = &RT.ck_bit[obs_idx[k]];
            if (q->type == BIT_MOSSY) continue;   /* right across the bed: no way past with two wheels on the ground */
            float hd = route_heading_at_creek_pub(obs_s[k]), tx = cosf(hd), tz = sinf(hd), px = -tz, pz = tx, half = 2.1f, bed = route_creek_bed(&RT, obs_s[k]);
            /* the play test (Dylan): every gate must be doable. Shift the gate sideways off the obstacle till there's a line
             * through it with one side's wheels on clear ground and the other's up on the obstacle (no higher than a tyre
             * climbs), and room under the belly; no line, no gate here */
            float goff = 0, glane = 0; int found = 0;
            static const float offs[5] = { 0.0f, 0.6f, -0.6f, 1.1f, -1.1f };
            for (int oi = 0; oi < 5 && !found; oi++) {
                float gx = q->x + px * offs[oi], gzz = q->z + pz * offs[oi];
                for (float o = -0.7f; o <= 0.71f && !found; o += 0.35f) {   /* a ute-width line with room to the stakes */
                    float hmax[3] = { 0, 0, 0 };
                    for (int tr = 0; tr < 3; tr++) { float l = o + (float)(tr - 1) * 0.8f;
                        for (float a = -4.0f; a <= 4.01f; a += 1.0f) { float xx = gx + px * l + tx * a, zz = gzz + pz * l + tz * a; hmax[tr] = fmaxf(hmax[tr], route_height(&RT, xx, zz, NULL) - bed); } }
                    float lo = fminf(hmax[0], hmax[2]), hi = fmaxf(hmax[0], hmax[2]);
                    if (lo <= 0.15f && hi <= 0.45f && hi >= 0.12f && hmax[1] <= 0.3f) { found = 1; goff = offs[oi]; glane = o; }
                }
            }
            if (!found) continue;
            float gcx = q->x + px * goff, gcz = q->z + pz * goff;
            Gate G; memset(&G, 0, sizeof(G)); G.lane = glane; G.cx = gcx; G.cz = gcz; G.tx = tx; G.tz = tz; G.s = obs_s[k];
            G.x[0] = gcx - px * half; G.z[0] = gcz - pz * half; G.x[1] = gcx + px * half; G.z[1] = gcz + pz * half;
            int ok = 1;
            for (int e = 0; e < 2 && ok; e++) ok = route_in_creek(&RT, G.x[e], G.z[e]) && route_edge_at(&RT, G.x[e], G.z[e]) < T.edge_stop - 2.0f && route_tree_hit(&RT, G.x[e], G.z[e], 0, 0.6f, 0.6f, -0.6f) < 0;
            for (int h = 0; h < ngate && ok; h++) ok = fabsf(gate[h].s - G.s) > 40.0f;
            if (ok) { gate[ngate++] = G; if (getenv("RT_GATES")) fprintf(stderr, "GATE %d at %.0f m down the creek way, over a %s\n", ngate - 1, G.s, q->type == BIT_MOSSY ? "mossy fallen tree" : q->type == BIT_BOULDER ? "boulder" : q->type == BIT_SMALLLOG ? "small log" : "stump"); break; }
        }
    }
}
static void trials_reset_run(void) { TR.clean_run = 1; TR.pot = 0; TR.banked = 0; TR.mult = 1; TR.streak = 0; TR.fault_t = 9; if (obs_done) memset(obs_done, 0, (size_t)RT.ck_nbit); for (int g = 0; g < ngate; g++) gate[g].done = 0, gate[g].prev_along = 0; }
static void trials_fault(const char *why)
{
    if (TR.pot > 0) { ex_say2("POT LOST", TextFormat("%s: %s gone", why, commas((int)TR.pot)), (Color){ 255, 120, 90, 255 }); synth_blip(9); }
    else if (TR.fault_msg_t <= 0 && TR.in_ck) ex_say(why, (Color){ 255, 150, 110, 255 });
    TR.pot = 0; TR.mult = 1; TR.streak = 0; TR.fault_t = 0; TR.fault_msg_t = 2.0f;
}
static void ute_damage(int corner, float amt_tyre, float amt_body)
{
    if (corner < 0) { for (int i = 0; i < 4; i++) W.tyre_wear[i] = fminf(W.tyre_wear[i] + amt_tyre, W.punct ? 0.99f : 1.0f); }
    else W.tyre_wear[corner] = fminf(W.tyre_wear[corner] + amt_tyre, W.punct && W.punct != corner + 1 ? 0.99f : 1.0f);
    W.body_dmg = fminf(W.body_dmg + amt_body, 1.0f); W.dirty = 1; TR.clean_run = 0;
    for (int i = 0; i < 4 && !W.punct; i++) if (W.tyre_wear[i] >= 1.0f) {
        W.punct = i + 1;
        ex_say2("A FLAT TYRE!", W.spare ? "Pause: Change the tyre (the spare's in the tray)" : "No spare: limp home to the hut", (Color){ 255, 120, 90, 255 }); synth_blip2(10, 0.6f, 0.9f);
        if (getenv("BOT_EVENTS")) printf("   a flat tyre (wheel %d)\n", i);
    }
}
/* one step: damage from the body's knocks and landings, the obstacles and gates, the banks, the hut's repairs */
static int trials_step(const Car *c, float hw, float fr, float re)
{
    if (T.trials_on == 0) { bp.land_ev = 0; return 0; }
    int ev = 0; float x = c->body.pos.x, z = c->body.pos.z, kmh = vlen(c->body.vel) * 3.6f, yaw = car_yaw(c);
    if (gate_day != W.day) gates_place();
    TR.fault_t += FIXED_DT; if (TR.fault_msg_t > 0) TR.fault_msg_t -= FIXED_DT;
    /* damage (anywhere in Explorer) */
    int kc = -1; float kv = 0; for (int i = 0; i < 4; i++) if (bp.knock[i] > kv) { kv = bp.knock[i]; kc = i; }
    if (T.body_physics != 0) {
        if (kv >= T.trials_knock) { ute_damage(kc, T.dmg_tyre * (kv - 1.5f), T.dmg_body * (kv - 1.5f)); trials_fault("A KNOCK"); if (getenv("BOT_EVENTS")) printf("   a knock, %.1f m/s, wheel %d (tyre %.2f, body %.2f)\n", kv, kc, W.tyre_wear[kc], W.body_dmg); }
        if (bp.land_ev) { bp.land_ev = 0; if (bp.land_v >= T.trials_land) { ute_damage(-1, 0.06f * (bp.land_v - 3.0f), 0.06f * (bp.land_v - 3.0f)); trials_fault("A HARD LANDING"); if (getenv("BOT_EVENTS")) printf("   a hard landing, %.1f m/s\n", bp.land_v); } }
        static int was_rolled; if (bp.rolled && !was_rolled) { ute_damage(-1, 0.1f, 0.35f); trials_fault("OVER SHE GOES"); } was_rolled = bp.rolled;
    }
    if (W.punct && !W.spare && !TR.no_spare_said) { TR.no_spare_said = 1; }
    if (!W.punct) TR.no_spare_said = 0;
    /* the hut fixes the ute and puts a spare in the tray */
    if (world_hut_dist(x, z) < T.hut_reach) {
        float worst = W.body_dmg; for (int i = 0; i < 4; i++) worst = fmaxf(worst, W.tyre_wear[i]);
        if (W.punct || W.spare < 1 || worst > 0.02f) {
            memset(W.tyre_wear, 0, sizeof(W.tyre_wear)); W.body_dmg = 0; W.punct = 0; W.spare = 1; W.dirty = 1; ev = 1;
            ex_say2("FIXED UP AT THE HUT", "Tyres, panels, and a spare in the tray", (Color){ 160, 230, 150, 255 }); synth_blip(2);
            if (getenv("BOT_EVENTS")) printf("   the ute fixed at the hut\n");
        }
    }
    if (RT.ck_n < 2) return ev;
    /* the creek way: in it past the gap? */
    RtNear cn = route_creek_near(&RT, x, z);
    int in_ck = route_in_creek(&RT, x, z) && cn.s > RT.ck_exit_s;
    if (in_ck && !TR.in_ck && cn.s < RT.ck_exit_s + 80.0f) trials_reset_run();   /* in by the way in: a new run */
    if (!in_ck && TR.in_ck && TR.pot > 0) { TR.pot = 0; TR.mult = 1; TR.streak = 0; }   /* out of the creek way unbanked: the pot's gone */
    TR.in_ck = in_ck;
    if (!in_ck) return ev;
    obs_init();
    /* the obstacles near: each once a run */
    int lo = 0, hi = nobs; while (lo < hi) { int m = (lo + hi) / 2; if (obs_s[m] < cn.s - 15.0f) lo = m + 1; else hi = m; }
    for (int k = lo; k < nobs && obs_s[k] < cn.s + 15.0f; k++) {
        int b = obs_idx[k]; if (obs_done[b]) continue;
        const struct RtBit *q = &RT.ck_bit[b];
        float ax = cosf(q->yaw), az = sinf(q->yaw), dx = x - q->x, dz = z - q->z, along = clampf(dx * ax + dz * az, -q->l, q->l);
        float d = hypotf(dx - ax * along, dz - az * along);
        if (d > q->r + 0.9f) continue;
        obs_done[b] = 1;
        int clean = (c->low || kmh <= T.trials_slow) && TR.fault_t > 1.5f;
        if (clean) {
            TR.streak++; TR.mult = 1 + TR.streak / 6; if (TR.mult > 4) TR.mult = 4;
            TR.pot += (float)(obs_base(q->type) * TR.mult); TR.nclean++; TR.msg_t = 1.2f;
        } else if (TR.fault_t > 1.5f) {
            if (TR.streak >= 3 && TR.fault_msg_t <= 0) ex_say("TOO QUICK: NO POINTS", (Color){ 230, 220, 200, 255 });
            TR.streak = 0; TR.mult = 1;
        }
    }
    /* the gates: through between the stakes cleanly for a bonus; a stake clipped bends and costs */
    for (int g = 0; g < ngate; g++) {
        Gate *G = &gate[g];
        if (fabsf(G->s - cn.s) > 30.0f) { G->prev_along = 0; continue; }
        for (int e = 0; e < 2; e++) if (!G->bent[e] && stag_box_touch(G->x[e], G->z[e], 0.12f, x, z, yaw, hw, fr, re)) {
            G->bent[e] = 1; TR.pot = fmaxf(TR.pot - 100.0f, 0.0f); synth_blip2(5, 2.4f, 0.8f); ev = 1;
            ex_say("TING! A STAKE CLIPPED: -100", (Color){ 255, 200, 120, 255 });
            if (getenv("BOT_EVENTS")) printf("   gate %d: a stake clipped\n", g);
        }
        float dx = x - G->cx, dz = z - G->cz, along = dx * G->tx + dz * G->tz, lat = -dx * G->tz + dz * G->tx;
        if (G->prev_along != 0 && (G->prev_along < 0) != (along < 0) && fabsf(lat) < 2.1f && !G->done) {
            G->done = 1;
            if ((c->low || kmh <= T.trials_slow) && TR.fault_t > 1.0f && !G->bent[0] && !G->bent[1]) {
                TR.pot += 150.0f * (float)TR.mult; synth_blip(1); ev = 1;
                ex_say(TextFormat("GATE, CLEAN: +%d", 150 * TR.mult), (Color){ 150, 230, 140, 255 });
                if (getenv("BOT_EVENTS")) printf("   gate %d: through cleanly\n", g);
            } else if (getenv("BOT_EVENTS")) printf("   gate %d: through, not clean\n", g);
        }
        G->prev_along = along == 0 ? 1e-4f : along;
    }
    /* the banks: the camp and the ford */
    int at_camp = W.camp_on && hypotf(x - W.camp_x, z - W.camp_z) < T.camp_reach + 8.0f, at_ford = cn.s > RT.ck_len - 40.0f;
    if ((at_camp || at_ford) && TR.pot > 0) {
        TR.banked += TR.pot;
        ex_say2(TextFormat("BANKED %s", commas((int)TR.pot)), TextFormat("Creek Trials this run: %s", commas((int)TR.banked)), (Color){ 255, 235, 120, 255 }); synth_blip(2);
        if (getenv("BOT_EVENTS")) printf("   trials banked %.0f (run %.0f; %d clean so far)\n", TR.pot, TR.banked, TR.nclean);
        TR.pot = 0; ev = 1;
        if ((int)TR.banked > W.trials_best) { W.trials_best = (int)TR.banked; W.dirty = 1; }
        if (TR.banked >= 5000.0f) world_earn(ACH_TRIALS);
    }
    if (at_ford && W.creek_from_hut && TR.clean_run) world_earn(ACH_CLEAN_RUN);   /* the creek way from the hut's clearing (as Off the beaten track) with no damage */
    return ev;
}
static void trials_hud(void)
{
    if (T.trials_on == 0 || !TR.in_ck) return;
    const Color ink = { 255, 248, 230, 255 }, gold = { 255, 235, 120, 255 };
    const char *t = commas((int)TR.pot);
    DrawRectangle(320 - 70, 4, 140, 44, (Color){ 20, 16, 30, 110 });
    DrawText("CREEK TRIALS", 320 - MeasureText("CREEK TRIALS", 10) / 2, 7, 10, (Color){ 200, 220, 255, 230 });
    DrawText(t, 320 - MeasureText(t, 20) / 2 - (TR.mult > 1 ? 14 : 0), 18, 20, TR.msg_t > 0 ? gold : ink);
    if (TR.mult > 1) DrawText(TextFormat("x%d", TR.mult), 320 + MeasureText(t, 20) / 2 - 8, 24, 10, gold);
    const char *b = TextFormat("banked %s  best %s", commas((int)TR.banked), commas(W.trials_best));
    DrawText(b, 320 - MeasureText(b, 10) / 2, 38, 10, (Color){ 210, 205, 215, 220 });
    if (TR.msg_t > 0) TR.msg_t -= GetFrameTime();
}
/* the stakes: a white post with red bands and an orange flag, 1.3 m; drawn straight or bent over */
static Mesh stake_mesh;
static void stake_mesh_init(void)
{
    MeshBuilder b = { 0 }; const float w = 0.05f;
    for (int band = 0; band < 5; band++) {
        float y0 = 0.26f * (float)band, y1 = y0 + 0.26f; Color c = band & 1 ? (Color){ 210, 40, 30, 200 } : (Color){ 240, 240, 235, 200 };
        for (int f = 0; f < 4; f++) {
            float a0 = 1.5708f * (float)f, a1 = a0 + 1.5708f; V3 p0 = v3(cosf(a0) * w, 0, sinf(a0) * w), p1 = v3(cosf(a1) * w, 0, sinf(a1) * w);
            Color sc = shaded(c, v3(cosf(a0 + 0.785f), 0, sinf(a0 + 0.785f)));
            mb_quad(&b, v3(p0.x, y0, p0.z), v3(p0.x, y1, p0.z), v3(p1.x, y1, p1.z), v3(p1.x, y0, p1.z), sc, 0);
            mb_quad(&b, v3(p1.x, y0, p1.z), v3(p1.x, y1, p1.z), v3(p0.x, y1, p0.z), v3(p0.x, y0, p0.z), sc, 0);   /* both windings: drawn mirrored */
        }
    }
    Color fl = { 255, 140, 30, 255 };   /* the flag (both faces), lit like a lamp so it shows at dusk */
    mb_vert(&b, v3(0, 1.3f, 0), fl, 0); mb_vert(&b, v3(0, 0.94f, 0), fl, 0); mb_vert(&b, v3(0.5f, 1.12f, 0), fl, 0);
    mb_vert(&b, v3(0, 1.3f, 0), fl, 0); mb_vert(&b, v3(0.5f, 1.12f, 0), fl, 0); mb_vert(&b, v3(0, 0.94f, 0), fl, 0);
    stake_mesh = mb_finish(&b);
}
static void gates_draw(V3 cw, Material m, float r)
{
    if (T.trials_on == 0 || !explore) return;
    if (gate_day != W.day) gates_place();
    if (!stake_mesh.vertexCount) stake_mesh_init();
    for (int g = 0; g < ngate; g++) for (int e = 0; e < 2; e++) {
        float x = gate[g].x[e], z = gate[g].z[e];
        if (hypotf(x - cw.x, z - cw.z) > r) continue;
        Q rot = qaxis(v3(0, 1, 0), -atan2f(gate[g].tz, gate[g].tx) + (e ? 3.1416f : 0.0f));   /* the flags fly downstream */
        if (gate[g].bent[e]) rot = qmul(rot, qaxis(v3(0, 0, 1), 0.9f));
        DrawMesh(stake_mesh, m, MatrixMultiply(MatrixScale(2.0f, 2.0f, 2.0f), pose_matrix(vsub(v3(x, route_height(&RT, x, z, NULL) - 0.05f, z), ro), rot)));   /* drawn twice life size, to show at a distance */
    }
}

static int explore_step(const Car *c, int saving)
{
    float hw = car_outline.own ? car_outline.half_w : T.body_half_w, fr = car_outline.own ? car_outline.front : T.body_front, re = car_outline.own ? car_outline.rear : T.body_rear;
    float x = c->body.pos.x, z = c->body.pos.z, yaw = car_yaw(c), sp = vlen(c->body.vel);
    int ev = 0;
    int h = world_hamper_hit(x, z, yaw, hw, fr, re);
    if (h >= 0) {
        puff(v3(W.item[h].x, route_height(&RT, W.item[h].x, W.item[h].z, NULL) + 0.4f, W.item[h].z), 24, 0.8f, 0.8f);
        static const char *const shown[HAMPER_N] = { "THE BRIE", "THE RED-WAX CHEESE", "THE CHEDDAR", "THE CHORIZO", "GREEN OLIVES", "STUFFED JALAPENOS",
                                                     "RED PEPPERS", "ZZZZEASONING!!!", "WHIPPED FETA", "CORN CHIPS", "A BAGUETTE", "TWO BOTTLES OF RED" };
        int nh = world_hamper_count();
        ex_say2(shown[h], nh >= HAMPER_N ? "Charcuterie: all 12! Take them to the hut" : TextFormat("Charcuterie: %d of %d", nh, HAMPER_N), (Color){ 140, 200, 255, 255 });   /* build E: no "esky" in the words */
        synth_blip2(12, 1.33f, 0.9f); ev = 1;
        if (getenv("BOT_EVENTS") || getenv("SHOT_SEQ")) fprintf(stderr, "   hamper: the %s (%d of %d)\n", HAMPER_NAME[h], world_hamper_count(), HAMPER_N);
    }
    if (world_rifle_hit(x, z, yaw, hw, fr, re)) {
        ex_say2("QUAYLE'S RIFLE", W.army_found ? "Take it back to Quayle" : "Find Quayle and give it back to him", (Color){ 150, 210, 110, 255 }); synth_blip(12); ev = 1;
        if (getenv("BOT_EVENTS") || getenv("SHOT_SEQ")) fprintf(stderr, "   the rifle picked up\n");
    }
    if (world_rifle_return(x, z, yaw, hw, fr, re)) {
        ex_say2("QUAYLE'S BACK ON DUTY", "Quayle: \"I'll ride in the back. Let's get a deer!\"", (Color){ 150, 210, 110, 255 }); synth_blip(13); ev = 1;
        ex_then("FIVE MORE DEER", "Quayle: \"I've marked where I saw them on your map.\"", (Color){ 255, 200, 90, 255 });   /* (pause: Show map) */
        if (getenv("BOT_EVENTS") || getenv("SHOT_SEQ")) fprintf(stderr, "   the rifle given back\n");
    }
    {   /* build D: the old stock route down the creek way: its markers, its camp, and the trunk there */
        int mk = world_marker_check(x, z);
        if (mk >= 0) {
            static const char *const what[5] = { "An old blaze on a stump", "A horseshoe nailed to a stump", "A rusted stock route sign", "An old cattle-yard post", "A stone cairn" };
            int nm = world_marker_count();
            ex_say2(TextFormat("MARKER %d/%d", nm, W.nmark), nm >= W.nmark ? "The old stock route: all ten. Follow it down to the camp" : what[W.mark_v[mk] % 5], (Color){ 230, 200, 140, 255 });
            synth_blip2(12, 0.9f, 0.8f); ev = 1;
            if (getenv("BOT_EVENTS") || getenv("SHOT_SEQ")) fprintf(stderr, "   marker %d (%d of %d)\n", mk + 1, nm, W.nmark);
        }
        int cp = world_camp_check(x, z);
        if (cp) {
            ex_say2("THE CATTLEMAN'S CAMP", cp == 2 ? "The old stock route, followed all the way" : world_marker_count() < W.nmark ? TextFormat("Stock route markers passed: %d of %d", world_marker_count(), W.nmark) : "A stone fireplace, a billy, a rusted trunk", (Color){ 230, 200, 140, 255 });
            synth_blip(cp == 2 ? 13 : 12); ev = 1;
            if (getenv("BOT_EVENTS") || getenv("SHOT_SEQ")) fprintf(stderr, "   the camp%s\n", cp == 2 ? ", with every marker" : "");
        }
        int tk = world_trunk_open(x, z, yaw, hw, fr, re);
        if (tk == 1) {
            ex_say2("A FULL LOAD OF FIREWOOD", "Dry split wood, kept in the old trunk", (Color){ 255, 170, 80, 255 });
            puff(v3(W.trunk_x, route_height(&RT, W.trunk_x, W.trunk_z, NULL) + 0.5f, W.trunk_z), 24, 0.8f, 0.8f);
            synth_blip(4); ev = 1;
            if (getenv("BOT_EVENTS") || getenv("SHOT_SEQ")) fprintf(stderr, "   the trunk opened: a full tray\n");
        } else if (tk == 2 && ex_flash_t <= 0) ex_say("THE TRAY'S ALREADY FULL", (Color){ 255, 200, 120, 255 });
    }
    int lv = bcut_t < 0 && scut_t < 0 ? world_deliver(x, z, sp) : 0;
    if (lv > 0) {
        if (in_window) { cut_t = 0; cut_level = lv; cut_shown = W.woodpile - 15; cut_move_car = 1; }   /* the cutscene (the bots in --drivetest go straight on) */
        ex_say(lv == 1 ? "THE FIRE'S LIT" : lv == 2 ? "MORE WOOD ON" : "A PROPER BONFIRE", (Color){ 255, 170, 80, 255 });
        synth_blip(6); ev = 1;
        if (getenv("BOT_EVENTS") || getenv("SHOT_SEQ")) { fflush(stdout); fprintf(stderr, "   the fire: level %d\n", lv); }
        if (lv == 2 && !W.stew && !W.tray_deer) ex_then("WHAT'S FOR DINNER?", "Did Quayle get a deer?", (Color){ 255, 190, 120, 255 });   /* play test: a nudge to the stew */
    }
    int st = !in_window || !cutscene_on() ? world_stew(x, z, sp) : 0;
    if (st == 1) {
        if (in_window) { scut_t = 0; scut_move = 1; }
        ex_say2("THE CAMP OVEN'S ON", "Venison stew, on the table", (Color){ 255, 190, 120, 255 }); synth_blip(2); ev = 1;
        puff(v3(W.picnic_x, route_height(&RT, W.picnic_x, W.picnic_z, NULL) + 1.0f, W.picnic_z), 20, 0.6f, 0.8f);
        if (getenv("BOT_EVENTS") || getenv("SHOT_SEQ")) fprintf(stderr, "   the stew on\n");
    }   /* (not hot enough: the hint by the hut says so) */
    if ((!in_window || !cutscene_on()) && world_board(x, z, sp)) {   /* (after the wood's cutscene, if both come at once) */
        if (in_window) { bcut_t = 0; bcut_landed = 0; bcut_move = 1; }
        ex_say("THE BOARD IS SET", (Color){ 255, 220, 140, 255 }); synth_blip(2); ev = 1;
        if (getenv("BOT_EVENTS") || getenv("SHOT_SEQ")) fprintf(stderr, "   the board laid out\n");
    }
    if (W.fire_level >= 3 && W.board && !W.hut_night && hutnight_t < 0 && throw_t < 0 && !cutscene_on()) {   /* both done: the finale */
        hutnight_t = 0; W.hut_night = 1; W.dirty = 1;
        fireworks_t = -1;   /* the fireworks again, for the scene (from HN_FW) */
        finale_music(1); hn_move = 1;   /* the 79 parked in the car park, for the last shot */
        world_earn(ACH_HUT_NIGHT); ev = 1;
        if (getenv("BOT_EVENTS") || getenv("SHOT_SEQ")) fprintf(stderr, "   hut night\n");
    }
    ev |= trials_step(c, hw, fr, re);   /* build E: Creek Trials, the gates, the ute's damage */
    {   /* build E: Midnight run (the bridge at night), Wet feet (1 km of the creek way), Low and slow (up past the waterfall in Low Range) */
        static float px_, pz_; static int have_p, ls_armed;
        float step = have_p ? hypotf(x - px_, z - pz_) : 0.0f; px_ = x; pz_ = z; have_p = 1;
        if (step > 3.0f) step = 0;   /* put back somewhere: not driven */
        RtNear rn = route_near(&RT, x, z);
        if (W.night >= 0.8f && RT.bridge_s0 > 0 && rn.s >= RT.bridge_s0 && rn.dist < 12.0f && !W.earned[ACH_MIDNIGHT]) { world_earn(ACH_MIDNIGHT); ev = 1; }
        if (RT.ck_n > 1) {
            RtNear cn = route_creek_near(&RT, x, z);
            int in_ck = route_in_creek(&RT, x, z) && cn.s > RT.ck_exit_s;
            if (in_ck && step > 0) { int was = (int)(W.wet_m / 100.0f); W.wet_m += step; if ((int)(W.wet_m / 100.0f) != was) W.dirty = 1; if (W.wet_m >= 1000.0f && !W.earned[ACH_WET_FEET]) { world_earn(ACH_WET_FEET); ev = 1; } }
            if (RT.ck_fall_s > 0) {
                float fs = RT.ck_fall_s;
                if (!in_ck || !c->low || cn.s > fs + 60.0f) ls_armed = 0;   /* out of the creek, out of Low Range, or well below: start over */
                if (in_ck && c->low && cn.s > fs + 20.0f && cn.s < fs + 60.0f) ls_armed = 1;   /* below the fall, in Low Range */
                if (ls_armed && cn.s < fs - 20.0f) { ls_armed = 0; if (!W.earned[ACH_LOW_SLOW]) { world_earn(ACH_LOW_SLOW); ev = 1; } }
            }
        }
    }
    if (ev && saving) world_save(), web_storage_flush();
    return ev;
}

/* --routetest: no window. The route as the game reads it: length, heights, grades and bends from its
 * own centreline; whether the ground function agrees with the road along the middle; trees; and the
 * time and size of every chunk's geometry. */
static int RW_EXPLORE_SET(const Route *R) { int k = R->explore; ((Route *)R)->explore = 1; return k; }
static void RW_EXPLORE_RESTORE(const Route *R, int k) { ((Route *)R)->explore = k; }
static int routetest(void)
{
    const Route *R = &RT;
    printf("ROUTE %d samples every %.0f m, %.0f m long, %d chunks of %.0f m; heights relative to the start pad (%.1f m above sea level)\n",
           R->n, RT_DS, R->length, R->nchunks, RT_CHUNK * RT_DS, R->elev0);
    double climb = 0, desc = 0; float hmin = 1e9f, hmax = -1e9f, smin = 0, smax = 0;
    for (int i = 0; i < R->n; i++) {
        if (R->h[i] < hmin) { hmin = R->h[i]; smin = (float)i * RT_DS; }
        if (R->h[i] > hmax) { hmax = R->h[i]; smax = (float)i * RT_DS; }
        if (i) { float d = R->h[i] - R->h[i - 1]; if (d > 0) climb += d; else desc -= d; }
    }
    printf("ROUTE road surface: start %.0f m, highest %.0f m at km %.2f, lowest %.0f m at km %.2f, end %.0f m; climb %.0f m, descent %.0f m (above sea level)\n",
           R->h[0] + R->elev0, hmax + R->elev0, smax / 1000, hmin + R->elev0, smin / 1000, R->h[R->n - 1] + R->elev0, climb, desc);
    int w = (int)(100.0f / RT_DS); float gup = -1e9f, gdn = 1e9f, sup = 0, sdn = 0;
    for (int i = 0; i + w < R->n; i++) {
        float g = (R->h[i + w] - R->h[i]) / 100.0f;
        if (g > gup) { gup = g; sup = (float)i * RT_DS; }
        if (g < gdn) { gdn = g; sdn = (float)i * RT_DS; }
    }
    printf("ROUTE steepest 100 m: up %.1f%% at km %.2f, down %.1f%% at km %.2f (smoothed road surface)\n", gup * 100, sup / 1000, gdn * 100, sdn / 1000);
    printf("ROUTE ford at km %.2f (water from %.0f to %.0f m, road %.2f m below the water at its lowest), bridge km %.3f to %.3f, start pad %.0f m across\n",
           R->ford_s / 1000, R->ford_lo, R->ford_hi, R->ford_water - route_road_h(R, R->ford_s), R->bridge_s0 / 1000, R->bridge_s1 / 1000, R->pad_r * 2);
    printf("ROUTE lane width found %.1f m (%.1f to %.1f); drivable width in play %.1f m, verge %.1f, ditch %.1f m wide and %.2f deep\n",
           R->lane_w, R->lane_lo, R->lane_hi, T.road_width, T.verge_w, T.ditch_w, T.ditch_depth);
    /* the ground function along the middle and across */
    float worst = 0; int bad = 0, surf_bad = 0;
    for (int i = 0; i < R->n; i += 3) {
        float s = (float)i * RT_DS;
        int sf;
        float h = route_height(R, R->x[i], R->z[i], &sf);
        float d = fabsf(h - route_road_h(R, s) - 0.06f);
        float dp = hypotf(R->x[i] - R->pad_x, R->z[i] - R->pad_z);
        if (dp < R->pad_r + 6.0f) continue;
        if (sf == SURF_DECK) d = fabsf(h - route_road_h(R, s));
        if (d > worst) worst = d;
        if (d > 0.05f) bad++;
        if (sf != SURF_ROAD && sf != SURF_DECK) surf_bad++;
    }
    printf("ROUTE ground on the centreline: worst %.3f m off the road surface, %d points over 5 cm, %d not reading as road\n", worst, bad, surf_bad);
    int ntree = 0, nscrub = 0, nprop = 0;
    for (int k = 0; k < R->ntree; k++) { if (R->tree[k].kind == 2) nscrub++; else if (R->tree[k].kind == 0) ntree++; else nprop++; }
    printf("ROUTE trunks and posts %d, soft plants %d, hut and props %d (%.0f trunks per km)\n", ntree, nscrub, nprop, ntree / (R->length / 1000));
    { /* how many of each template */
        int *cnt = calloc((size_t)route_template_count(), sizeof(int));
        for (int k = 0; k < R->ntree; k++) if (R->tree[k].t0 >= 0) cnt[R->tree[k].t0]++;
        char line[2048] = ""; const char *last = "";
        int sum = 0;
        for (int i = 0; i <= route_template_count(); i++) {
            const char *nm = i < route_template_count() ? route_template(i)->name : "";
            if (strcmp(nm, last) && last[0]) { snprintf(line + strlen(line), sizeof(line) - strlen(line), "%s %d, ", last, sum); sum = 0; }
            if (i < route_template_count()) { sum += cnt[i]; last = nm; }
        }
        printf("ROUTE plants: %s\n", line);
        free(cnt);
    }
    int near_road = 0;
    for (int k = 0; k < R->ntree; k++) {
        if (R->tree[k].kind != 0 && R->tree[k].kind != 3 && R->tree[k].kind != 5) continue;
        RtNear n = route_near(R, R->tree[k].x, R->tree[k].z);
        if (R->tree[k].kind == 3 && n.s < 12.0f) continue; /* the hut's own walls: the track runs past them at the start, outside the verge */
        if (n.s > 0 && n.s < R->length && fabsf(n.lat) - R->tree[k].r < route_half_width() + T.verge_w + T.ditch_w + 0.4f) {
            if (near_road < 40 && n.s > 12.0f) printf("ROUTE   solid %s at km %.3f, %+.1f m off the middle, radius %.2f\n", R->tree[k].t0 >= 0 ? route_template(R->tree[k].t0)->name : R->tree[k].kind == 5 ? "log" : "hut wall", n.s / 1000, n.lat, R->tree[k].r);
            near_road++;
        }
    }
    printf("ROUTE trunks, posts and walls reaching inside the ditch's outer edge: %d (should be 0)\n", near_road);
    {   /* build B: the map's edge and the deer */
        int bad_e = 0; float lo = 1e9f;
        for (int i = 0; i < R->n; i += 5) {   /* the edge never comes nearer the road than edge_dist (round the hut, further out) */
            for (int sd = -1; sd <= 1; sd += 2) {
                float hd = route_heading_at(R, (float)i * RT_DS), x = R->x[i] - sinf(hd) * sd * (T.edge_dist - 0.5f), z = R->z[i] + cosf(hd) * sd * (T.edge_dist - 0.5f);
                RtNear nn = route_near(R, x, z);
                if (fabsf(nn.s - (float)i * RT_DS) > 5.0f) continue;   /* nearer another stretch of road */
                float e = route_edge(R, nn, x, z); if (e > 0) bad_e++; lo = fminf(lo, -e);
            }
        }
        int ntri_edge = 0;
        for (int k = 0; k < R->ntree; k++) if (R->tree[k].t0 >= 0 && !strcmp(route_template(R->tree[k].t0)->name, "edgescrub")) ntri_edge++;
        if (R->spur_n > 1) printf("ROUTE the junction: Cattlemans Link Trail leaves at km %.3f (%.0f, %.0f), a stub of %d points; the gate %.0f m up it at (%.1f, %.1f)\n", R->junction_s / 1000, R->jx, R->jz, R->spur_n, R->gate_t, R->gate_x, R->gate_z);
        if (W.rifle_placed) printf("ROUTE Quayle's rifle at (%.1f, %.1f), yaw %.2f\n", W.rifle_x, W.rifle_z, W.rifle_yaw);
        if (getenv("RT_DUMP_FINDS")) {   /* every find's place, for maps: kind number x z */
            FILE *f = fopen(getenv("RT_DUMP_FINDS"), "w");
            for (int d = 0; f && d < W.ndeer; d++) fprintf(f, "deer %d %.1f %.1f %d\n", d + 1, W.deer[d].x, W.deer[d].z, W.deer[d].hidden);
            for (int d = WORLD_DEER; f && d < WORLD_DEER + W.nextra; d++) fprintf(f, "extra %d %.1f %.1f 0\n", d - WORLD_DEER + 1, W.deer[d].x, W.deer[d].z);   /* Quayle's five */
            if (f && W.army_placed) fprintf(f, "quayle 0 %.1f %.1f 0\n", W.army_x, W.army_z);
            if (f && W.rifle_placed) fprintf(f, "rifle 0 %.1f %.1f 0\n", W.rifle_x, W.rifle_z);
            for (int i = 0; f && i < HAMPER_N; i++) fprintf(f, "hamper %d %.1f %.1f 0\n", i + 1, W.item[i].x, W.item[i].z);
            if (f) fprintf(f, "hut 0 %.1f %.1f 0\n", R->hut_x, R->hut_z);
            if (f) fclose(f);
        }
        if (W.army_placed) { RtNear an = route_near(R, W.army_x, W.army_z); printf("ROUTE the army man: km %.2f, %+.1f m off the road at (%.1f, %.1f), deer at (%.1f, %.1f), %.1f m from deer %d (reachable)\n", an.s / 1000, an.lat, W.army_x, W.army_z, W.deer[W.army_deer].x, W.deer[W.army_deer].z, hypotf(W.army_x - W.deer[W.army_deer].x, W.army_z - W.deer[W.army_deer].z), W.army_deer + 1); }
        printf("ROUTE the edge: a bank from %.0f m off the road, %.0f m up to %.1f m high; the car is held %.1f m up it; hut hollow %d grid points (%.1f ha); points just inside the edge reading as past it: %d (should be 0); edge scrub %d\n",
               T.edge_dist, T.edge_w, T.edge_h, T.edge_stop, R->hollow_cells, R->hollow_cells * R->cell * R->cell / 10000.0f, bad_e, ntri_edge);
        if (getenv("RT_DUMP_EDGE")) {   /* the edge round the hut, 2 m steps over 1.4 km square: float32, row 0 = south, for tools that overlay it */
            FILE *f = fopen(getenv("RT_DUMP_EDGE"), "wb");
            for (int j = 0; j < 700 && f; j++) for (int i = 0; i < 700; i++) { float v = route_edge_at(R, R->hut_x - 700.0f + 2.0f * (float)i, R->hut_z - 700.0f + 2.0f * (float)j); fwrite(&v, 4, 1, f); }
            if (f) fclose(f);
        }
        for (int d = 0; d < W.ndeer; d++)
            printf("ROUTE deer %2d: km %.2f, %+5.1f m off the road, %s, pose %d\n", d + 1, W.deer[d].s / 1000, W.deer[d].lat, W.deer[d].hidden ? "hidden" : "easy  ", W.deer[d].variant);
        if (getenv("RT_DUMP_CLEAR")) {   /* build C: in the hut's clearing (1) or not (0), 2 m steps over 1.6 km square round the hut, row 0 = south */
            FILE *f = fopen(getenv("RT_DUMP_CLEAR"), "wb");
            for (int j = 0; j < 800 && f; j++) for (int i = 0; i < 800; i++) { unsigned char v = (unsigned char)route_in_clearing(R, R->hut_x - 800.0f + 2.0f * (float)i, R->hut_z - 800.0f + 2.0f * (float)j); fwrite(&v, 1, 1, f); }
            if (f) fclose(f);
        }
        if (getenv("RT_DUMP_PROPS")) {   /* build C: every prop, post and solid within 120 m of the hut, metres east/north of it, for plotting */
            FILE *f = fopen(getenv("RT_DUMP_PROPS"), "w");
            for (int k = 0; k < R->ntree && f; k++) {
                const RtTree *t = &R->tree[k];
                if (hypotf(t->x - R->hut_x, t->z - R->hut_z) > (getenv("RT_DUMP_FAR") ? 700.0f : 120.0f) || (t->kind == 2 && t->lat < 98.0f && !getenv("RT_DUMP_SOFT"))) continue;
                fprintf(f, "%d %s %.2f %.2f %.2f %.2f\n", t->kind, t->t0 >= 0 ? route_template(t->t0)->name : "-", t->x - R->hut_x, t->z - R->hut_z, t->r, t->yaw);
            }
            if (f) fclose(f);
        }
    }
    if (getenv("RT_MARKERS")) for (int m = 0; m < R->ck_nmark; m++) { RtNear c = route_creek_near(R, R->ck_mark_x[m], R->ck_mark_z[m]); int sl; float bl = route_creek_line(R, c.s, &sl); printf("MARKER %d at %.0f m, %+.1f m off the line, bed half %.1f, bot line %+.1f, variant %d, at (%.1f, %.1f)\n", m + 1, c.s, c.lat, route_creek_bedw(R, c.s), bl, R->ck_mark_v[m], R->ck_mark_x[m], R->ck_mark_z[m]); }
    if (getenv("RT_MARKERS")) { printf("CAMP (%.1f, %.1f) trunk (%.1f, %.1f)\n", R->camp_x, R->camp_z, R->trunk_x, R->trunk_z); for (int d = 0; d < R->ck_ndf; d++) { float x, z; route_creek_point(R, R->ck_df_s[d], &x, &z); printf("DEADFALL %d kind %d at %.0f m (%.1f, %.1f)\n", d, R->ck_df_kind[d], R->ck_df_s[d], x, z); } }
    if (getenv("RT_STAGS")) for (int k = 0; k < R->nstag; k++) { RtNear n = route_near(R, R->stag[k].x, R->stag[k].z); printf("STAG %d %s at (%.1f, %.1f), road km %.2f %+.1f m, %.1f m tall, falls to %.0f deg\n", k, R->stag[k].where == 0 ? "road strip" : R->stag[k].where == 1 ? "clearing edge" : "creek way", R->stag[k].x, R->stag[k].z, n.s / 1000, n.lat, R->stag[k].h, R->stag[k].fall * RAD2DEGF); }
    if (getenv("RT_XSEC")) {   /* part 2: the creek's cross-sections from S0 to S1 every 4 m (heights relative to the bed) */
        float s0 = 0, s1 = 0; sscanf(getenv("RT_XSEC"), "%f,%f", &s0, &s1); int keep = RW_EXPLORE_SET(R);
        for (float cs = s0; cs <= s1; cs += 4.0f) { int i = (int)(cs / RT_DS); float hd = atan2f(R->ck_z[i + 1] - R->ck_z[i], R->ck_x[i + 1] - R->ck_x[i]), x, z; route_creek_point(R, cs, &x, &z); float b0 = route_creek_bed(R, cs);
            printf("XSEC %5.0f bw %.1f fallside %d:", cs, route_creek_bedw(R, cs), R->ck_fall_side); for (float lat = -10; lat <= 10.01f; lat += 1.0f) printf(" %5.2f", route_height(R, x - sinf(hd) * lat, z + cosf(hd) * lat, NULL) - b0); printf("\n"); }
        RW_EXPLORE_RESTORE(R, keep);
    }
    if (getenv("RT_BITS")) {   /* part 2: the creek's things within 12 m of S metres down it */
        float cs = (float)atof(getenv("RT_BITS")), x, z; route_creek_point(R, cs, &x, &z);
        for (int b = 0; b < R->ck_nbit; b++) { const struct RtBit *q = &R->ck_bit[b]; if (hypotf(q->x - x, q->z - z) > 12.0f) continue; RtNear n = route_creek_near(R, q->x, q->z); printf("BIT type %d at %.1f m %+.1f, l %.2f r %.2f h %.2f yaw %.2f\n", q->type, n.s, n.lat, q->l, q->r, q->h, q->yaw); }
        int keep = RW_EXPLORE_SET(R); for (float d = -8; d <= 8; d += 1.0f) { float px, pz; route_creek_point(R, cs + d, &px, &pz); printf("LINE %.0f h %.2f bump %.2f\n", cs + d, route_height(R, px, pz, NULL), route_creek_bump(R, px, pz)); } RW_EXPLORE_RESTORE(R, keep);
    }
    if (getenv("RT_PROBEA")) { float px = 0, pz = 0; sscanf(getenv("RT_PROBEA"), "%f,%f", &px, &pz); for (float dz = -3; dz <= 3; dz += 0.5f) { printf("PA %+.1f:", dz); for (float dx = -3; dx <= 3; dx += 0.5f) { int sf; printf(" %6.2f", route_height(R, px + dx, pz + dz, &sf)); } printf("\n"); } printf("hut at %.1f %.1f\n", R->hut_x, R->hut_z); }
    if (getenv("RT_PROBE")) {   /* build D: heights round a point (metres E/N of the hut), for comparing builds */
        float px = 0, pz = 0; sscanf(getenv("RT_PROBE"), "%f,%f", &px, &pz);
        for (float dz = -4; dz <= 4; dz += 2) for (float dx = -4; dx <= 4; dx += 2) { int sf; float h = route_height(R, R->hut_x + px + dx, R->hut_z + pz + dz, &sf); printf("PROBE %+.0f %+.0f %.5f %d dem %.5f\n", px + dx, pz + dz, h, sf, route_dem(R, R->hut_x + px + dx, R->hut_z + pz + dz)); }
    }
    if (R->ck_n > 1) {   /* build C: the creek way. Can the ute get from the clearing to the ford along it? A search on a 1 m grid
                          * (Explorer's world: its strip open), each cell blocked if a 2.2 m box there touches anything solid, is up its
                          * bank past the hold, or is steeper than 35 %. Reports the furthest it got if not. */
        Route *RW = (Route *)R; int keep = RW->explore; RW->explore = 1;
        /* build D: twice. With the high-range limit (35 %: the rear-drive ute stalls past about that at walking pace) and with
         * Low Range's (the slope's cap: 4WD in low holds it); then, along the low-range path, the steep places only low gets up */
        float far_hi = 0; int ok_hi = 0, ok = 0, open_cells = 0; float far_s = 0;
        for (int pass = 0; pass < 2; pass++) {
            float steep_max = pass == 0 ? 0.35f : 0.62f;
            float x0 = 1e9f, x1 = -1e9f, z0 = 1e9f, z1 = -1e9f;
            for (int i = 0; i < R->ck_n; i++) { x0 = fminf(x0, R->ck_x[i]); x1 = fmaxf(x1, R->ck_x[i]); z0 = fminf(z0, R->ck_z[i]); z1 = fmaxf(z1, R->ck_z[i]); }
            x0 -= 40; z0 -= 40; x1 += 40; z1 += 40;
            int nx = (int)(x1 - x0) + 1, nz = (int)(z1 - z0) + 1;
            /* a 0-1 search: a steep cell (over 35 %) costs 1, so the low-range path climbs only where it has to */
            unsigned char *st = calloc((size_t)nx * nz, 1); int QN = 8000000, *q = malloc(sizeof(int) * (size_t)QN), qh = QN / 2, qt = QN / 2, *par = malloc(sizeof(int) * (size_t)nx * nz);
            float *slope = malloc(sizeof(float) * (size_t)nx * nz); int *cost = malloc(sizeof(int) * (size_t)nx * nz);
            for (int k = 0; k < nx * nz; k++) cost[k] = 1 << 30;
            float sx, sz; route_creek_point(R, R->ck_exit_s - 40.0f, &sx, &sz);
            float ex, ez; route_creek_point(R, R->ck_len - 15.0f, &ex, &ez);
            int si = (int)(sx - x0), sj = (int)(sz - z0), goal = -1; q[qt++] = sj * nx + si; st[sj * nx + si] = 3; par[sj * nx + si] = -1; slope[sj * nx + si] = 0; cost[sj * nx + si] = 0;
            float far = 0; int okp = 0, oc = 0;
            while (qh < qt && !okp) {
                int k = q[qh++], i = k % nx, j = k / nx;
                if (st[k] == 4) continue;   /* settled */
                st[k] = 4;
                float x = x0 + (float)i + 0.5f, z = z0 + (float)j + 0.5f;
                RtNear cn = route_creek_near(R, x, z); if (cn.s > far && cn.dist < 30.0f) far = cn.s;
                if (hypotf(x - ex, z - ez) < 3.0f) { okp = 1; goal = k; break; }
                const int di[4] = { 1, -1, 0, 0 }, dj[4] = { 0, 0, 1, -1 };
                for (int d = 0; d < 4; d++) {
                    int a = i + di[d], b = j + dj[d]; if (a < 0 || b < 0 || a >= nx || b >= nz) continue;
                    int m = b * nx + a; if (st[m] == 2 || st[m] == 4) continue;
                    if (st[m] == 3) {   /* seen: a cheaper way to it? */
                        int c1 = cost[k] + (slope[m] > 0.35f);
                        if (c1 < cost[m]) { cost[m] = c1; par[m] = k; if (slope[m] > 0.35f) { if (qt < QN) q[qt++] = m; } else if (qh > 0) q[--qh] = m; }
                        continue;
                    }
                    float xx = x0 + (float)a + 0.5f, zz = z0 + (float)b + 0.5f;
                    RtNear c2 = route_creek_near(R, xx, zz);
                    int blocked = c2.dist > 45.0f; float sl = 0;
                    if (!blocked) {
                        float gx = route_height(R, xx + 1.0f, zz, NULL) - route_height(R, xx - 1.0f, zz, NULL), gz = route_height(R, xx, zz + 1.0f, NULL) - route_height(R, xx, zz - 1.0f, NULL);
                        sl = hypotf(gx, gz) * 0.5f;
                        blocked = route_tree_hit(R, xx, zz, 0, 1.1f, 1.1f, -1.1f) >= 0 || route_edge_at(R, xx, zz) > T.edge_stop - 1.0f || sl > steep_max;
                    }
                    st[m] = blocked ? 2 : 3;
                    if (!blocked) { oc++; par[m] = k; slope[m] = sl; cost[m] = cost[k] + (sl > 0.35f); if (sl > 0.35f) { if (qt < QN) q[qt++] = m; } else if (qh > 0) q[--qh] = m; }
                }
            }
            if (pass == 0) { ok_hi = okp; far_hi = far; }
            else {
                ok = okp; far_s = far; open_cells = oc;
                if (okp) {   /* the low-range path's steep places: runs of cells over the high-range limit, by where they are down the creek */
                    float spot_s[64], spot_g[64]; int ns = 0;
                    for (int k = goal; k >= 0; k = par[k]) {
                        if (slope[k] <= 0.35f) continue;
                        RtNear cn = route_creek_near(R, x0 + (float)(k % nx) + 0.5f, z0 + (float)(k / nx) + 0.5f);
                        int f = -1; for (int u = 0; u < ns; u++) if (fabsf(spot_s[u] - cn.s) < 25.0f) f = u;
                        if (f < 0 && ns < 64) { f = ns++; spot_s[f] = cn.s; spot_g[f] = 0; }
                        if (f >= 0) spot_g[f] = fmaxf(spot_g[f], slope[k]);
                    }
                    for (int a = 0; a < ns; a++) for (int b = a + 1; b < ns; b++) if (spot_s[b] < spot_s[a]) { float t = spot_s[a]; spot_s[a] = spot_s[b]; spot_s[b] = t; t = spot_g[a]; spot_g[a] = spot_g[b]; spot_g[b] = t; }
                    printf("ROUTE the creek way's steep spots (over 35%%, Low Range country), on the low-range path: %d:", ns);
                    for (int a = 0; a < ns; a++) printf(" %.0f m (%.0f%%)", spot_s[a], spot_g[a] * 100);
                    printf("\n");
                    if (getenv("RT_DUMP_SPOTS")) { FILE *f = fopen(getenv("RT_DUMP_SPOTS"), "w"); for (int a = 0; f && a < ns; a++) fprintf(f, "%.1f %.3f\n", spot_s[a], spot_g[a]); if (f) fclose(f); }
                }
            }
            free(st); free(q); free(par); free(slope); free(cost);
        }
        printf("ROUTE the creek way: %.0f m, leaves the clearing at %.0f m; falls at %.0f m; log jams at %.0f and %.0f m; from the clearing to the ford for the ute in Low Range: %s (searched %d open square metres%s); in high range: %s\n",
               R->ck_len, R->ck_exit_s, R->ck_fall_s, R->ck_njam > 0 ? R->ck_jam_s[0] : 0.0f, R->ck_njam > 1 ? R->ck_jam_s[1] : 0.0f, ok ? "YES" : "NO", open_cells, ok ? "" : TextFormat("; got no further than %.0f m down", far_s),
               ok_hi ? "YES" : TextFormat("NO (stopped by a steep climb %.0f m down)", far_hi));
        if (getenv("CREEK_DEBUG")) {   /* a map round S metres down: each cell open (.), a solid (T), past the hold (E), too steep (S) */
            float cs = (float)atof(getenv("CREEK_DEBUG"));
            for (float lat = 24; lat >= -24; lat -= 1.5f) {
                printf("%+5.1f ", lat);
                for (float ss = cs - 45; ss <= cs + 45; ss += 1.5f) {
                    float x, z; route_creek_point(R, ss, &x, &z); int i = (int)(ss / RT_DS); float hd = atan2f(R->ck_z[i + 1] - R->ck_z[i], R->ck_x[i + 1] - R->ck_x[i]);
                    float xx = x - sinf(hd) * lat, zz = z + cosf(hd) * lat;
                    float gx = route_height(R, xx + 1.0f, zz, NULL) - route_height(R, xx - 1.0f, zz, NULL), gz = route_height(R, xx, zz + 1.0f, NULL) - route_height(R, xx, zz - 1.0f, NULL);
                    char ch = route_tree_hit(R, xx, zz, 0, 1.1f, 1.1f, -1.1f) >= 0 ? 'T' : route_edge_at(R, xx, zz) > T.edge_stop - 1.0f ? 'E' : hypotf(gx, gz) * 0.5f > 0.35f ? 'S' : '.';
                    putchar(ch);
                }
                putchar('\n');
            }
        }
        RW->explore = keep;
        float cms = 0, cmax = 0; long ctri = 0;
        for (int c = R->nchunks; c < R->nchunks + R->ck_nch; c++) { RtGeom g; route_build_chunk(R, c, &g); cms += (float)g.build_ms; cmax = fmaxf(cmax, (float)g.build_ms); for (int p = 0; p < RT_NPARTS; p++) ctri += g.part[p].nverts / 3; route_geom_free(&g); }
        printf("ROUTE the creek way's %d chunks: build %.1f ms mean, %.1f worst; %ld triangles in all (%ld a chunk)\n", R->ck_nch, cms / (float)R->ck_nch, cmax, ctri, ctri / R->ck_nch);
    }
    double ms_sum = 0, ms_max = 0; long verts = 0, vmax = 0, part_tris[RT_NPARTS] = { 0 }; int vmax_i = 0;
    long road = 0, terr = 0, trees = 0, scrub = 0, other = 0;
    for (int c = 0; c < R->nchunks; c++) {
        RtGeom g;
        route_build_chunk(R, c, &g);
        ms_sum += g.build_ms; if (g.build_ms > ms_max) ms_max = g.build_ms;
        long nv = 0; for (int p = 0; p < RT_NPARTS; p++) { nv += g.part[p].nverts; part_tris[p] += g.part[p].nverts / 3; }
        verts += nv; if (nv > vmax) { vmax = nv; vmax_i = c; }
        road += g.ntri_road; terr += g.ntri_terrain; trees += g.ntri_trees; scrub += g.ntri_scrub; other += g.ntri_other;
        route_geom_free(&g);
    }
    printf("ROUTE chunks: build %.1f ms mean, %.1f ms worst (worker thread); %ld triangles in all (road %ld, terrain %ld, trees %ld, shrubs %ld, other %ld), %ld a chunk on average, most %ld (chunk %d); %.1f MB for the whole route on the graphics chip\n",
           ms_sum / R->nchunks, ms_max, verts / 3, road, terr, trees, scrub, other, verts / 3 / R->nchunks, vmax / 3, vmax_i, verts * 24.0 / 1048576.0);
    printf("ROUTE parts: ground %ld, near (full detail) %ld, near far-versions %ld, back %ld, edge (no shadows) %ld triangles; %d templates\n",
           part_tris[RT_PART_GROUND], part_tris[RT_PART_NEAR], part_tris[RT_PART_NEARLO], part_tris[RT_PART_BACK], part_tris[RT_PART_EDGE], route_template_count());
    return 0;
}

/* --drivetest: no window. The road bot from the start pad to the bridge at a range of cornering limits
 * (m/s2 sideways), automatic gearbox, scored as the game scores. A careful bot should reach the bridge
 * without touching a tree; pushed harder it should go off and die at a tree. */
typedef struct { int state; float time, dist, top, mean, ditch_s, rough_s, score; float end_s, end_lat, hit_speed; int best_x; float old_score; int old_x, lost_ditch, lost_crash; float lost_pts, max_lat; int cp, ncp; float offroad_m; int deer, finished_seen, resets, wood; unsigned long long ach; int thuds, stags, rolls, wet_smashed, wet_shoved; float creek_t0, creek_t1; } RtRun;

/* Build C, the creek bot (--drivetest with BOT_CREEK=1, Explorer): a path for the ute from the car park across the clearing,
 * through the way in and down the creek way to the ford, found on a 1 m grid as the route test's search does (and kept
 * 1.5 m clear of anything solid), then driven with the real handling: steering to a point 7 m along the path, slower
 * where it bends or climbs out round the jams and the waterfall, backing off a little if it stalls against something. */
static float *bot_path; static int bot_path_n;
static int creek_path(void)
{
    const Route *R = &RT;
    float x0 = 1e9f, x1 = -1e9f, z0 = 1e9f, z1 = -1e9f;
    for (int i = 0; i < R->ck_n; i++) { x0 = fminf(x0, R->ck_x[i]); x1 = fmaxf(x1, R->ck_x[i]); z0 = fminf(z0, R->ck_z[i]); z1 = fmaxf(z1, R->ck_z[i]); }
    x0 = fminf(x0, R->start_x) - 40; z0 = fminf(z0, R->start_z) - 40; x1 = fmaxf(x1, R->start_x) + 40; z1 = fmaxf(z1, R->start_z) + 40;
    int nx = (int)(x1 - x0) + 1, nz = (int)(z1 - z0) + 1;
    int *par = malloc(sizeof(int) * (size_t)nx * nz); unsigned char *st = calloc((size_t)nx * nz, 1); int *q = malloc(sizeof(int) * (size_t)nx * nz), qh = 0, qt = 0;
    float ex, ez; route_creek_point(R, R->ck_exit_s - 60.0f, &ex, &ez);   /* to the creek in the clearing; down it on its line */
    int si = (int)(R->start_x - x0), sj = (int)(R->start_z - z0), goal = -1;
    q[qt++] = sj * nx + si; st[sj * nx + si] = 3; par[sj * nx + si] = -1;
    while (qh < qt) {
        int k = q[qh++], i = k % nx, j = k / nx;
        if (hypotf(x0 + (float)i + 0.5f - ex, z0 + (float)j + 0.5f - ez) < 2.0f) { goal = k; break; }
        const int di[4] = { 1, -1, 0, 0 }, dj[4] = { 0, 0, 1, -1 };
        for (int d = 0; d < 4; d++) {
            int a = i + di[d], b = j + dj[d]; if (a < 0 || b < 0 || a >= nx || b >= nz) continue;
            int m = b * nx + a; if (st[m]) continue;
            float xx = x0 + (float)a + 0.5f, zz = z0 + (float)b + 0.5f;
            RtNear cn = route_creek_near(R, xx, zz);
            int blocked = cn.dist > 45.0f && !route_in_clearing(R, xx, zz);
            if (!blocked) {
                float gx = route_height(R, xx + 1.0f, zz, NULL) - route_height(R, xx - 1.0f, zz, NULL), gz = route_height(R, xx, zz + 1.0f, NULL) - route_height(R, xx, zz - 1.0f, NULL);
                blocked = route_tree_hit(R, xx, zz, 0, 1.6f, 1.6f, -1.6f) >= 0 || route_edge_at(R, xx, zz) > T.edge_stop - 1.5f || hypotf(gx, gz) * 0.5f > 0.30f;
            }
            st[m] = blocked ? 2 : 3;
            if (!blocked) { par[m] = k; q[qt++] = m; }
        }
    }
    bot_path_n = 0; free(bot_path); bot_path = NULL;
    if (goal >= 0) {
        int n = 0; for (int k = goal; k >= 0; k = par[k]) n++;
        bot_path = malloc(sizeof(float) * 2 * (size_t)n); bot_path_n = n;
        int i2 = n - 1; for (int k = goal; k >= 0; k = par[k], i2--) { bot_path[2 * i2] = x0 + (float)(k % nx) + 0.5f; bot_path[2 * i2 + 1] = z0 + (float)(k / nx) + 0.5f; }
        /* smooth it (a grid path zig-zags) */
        for (int pass = 0; pass < 6; pass++) for (int i = 1; i + 1 < n; i++) for (int c = 0; c < 2; c++) bot_path[2 * i + c] = 0.25f * bot_path[2 * (i - 1) + c] + 0.5f * bot_path[2 * i + c] + 0.25f * bot_path[2 * (i + 1) + c];
    }
    free(par); free(st); free(q);
    return goal >= 0;
}
static int bot_path_at;
static float bot_stall, bot_back; static int saw_evn[6];
/* Along the creek way's own line (kept clear of trunks), up or down it (dir), to s_to: on the bed, out onto the bypass's
 * side round each log jam and the waterfall. Returns 1 when there. */
static int creek_follow(const Car *car, CarInput *in, float s_to, float vmax)
{
    float x = car->body.pos.x, z = car->body.pos.z, v = vlen(car->body.vel);
    RtNear cn = route_creek_near(&RT, x, z);
    float dir = s_to > cn.s ? 1.0f : -1.0f, look = 7.0f + 0.6f * v, ts = clampf(cn.s + dir * look, 0, RT.ck_len), lat = 0;
    if ((s_to - cn.s) * dir < 3.0f) return 1;
    int slow = 0; lat = route_creek_line(&RT, ts, &slow);   /* build D: round the climb-outs, slowly through the rock gardens */
    { int sl2 = 0; route_creek_line(&RT, cn.s, &sl2); slow |= sl2; }
    float tx, tz; route_creek_point(&RT, ts, &tx, &tz);
    int i = (int)(ts / RT_DS); if (i > RT.ck_n - 2) i = RT.ck_n - 2;
    float hd = atan2f(RT.ck_z[i + 1] - RT.ck_z[i], RT.ck_x[i + 1] - RT.ck_x[i]);
    tx -= sinf(hd) * lat; tz += cosf(hd) * lat;
    int at_gate = 0;
    if (getenv("BOT_GATES")) for (int g = 0; g < ngate; g++) { float ds = (gate[g].s - cn.s) * dir; if (ds > -3.0f && ds < 24.0f) { float k = ds > 7.0f ? -6.0f : 5.0f; tx = gate[g].cx - gate[g].tz * gate[g].lane + gate[g].tx * dir * k; tz = gate[g].cz + gate[g].tx * gate[g].lane + gate[g].tz * dir * k; at_gate = 1; } }   /* build E: square on to the gate, 6 m out, then through its middle */
    V3 l = qrot(qconj(car->body.rot), v3(tx - x, 0, tz - z));
    float ang = atan2f(l.x, l.z);
    float h2 = route_heading_at_creek_pub(ts + dir * 14.0f), bend = fabsf(atan2f(sinf(h2 - hd), cosf(h2 - hd)));
    float vt = clampf(vmax - 10.0f * bend, 2.0f, vmax);
    if (lat != 0 || fabsf(cn.lat - lat) > 2.5f) vt = fminf(vt, 3.2f);
    if (at_gate) vt = fminf(vt, 2.2f);
    if (slow) vt = fminf(vt, 2.2f);
    if (getenv("BOT_TRACE_CK")) { float a0 = 0, a1 = 0; sscanf(getenv("BOT_TRACE_CK"), "%f,%f", &a0, &a1); static float tk; tk += FIXED_DT; if (cn.s > a0 && cn.s < a1 && tk > 0.5f) { tk = 0; printf("   ck s %.1f lat %+.1f want %+.1f v %.1f pitch %.0f roll %.0f wheels %d belly %d back %.1f\n", cn.s, cn.lat, lat, v, bp.pitch * RAD2DEGF, bp.roll * RAD2DEGF, bp.ncontact, bp.belly, bot_back); } }
    if (bot_back > 0) { bot_back -= FIXED_DT; in->brake = 1; in->steer = -clampf(ang * 2.0f, -1, 1); return 0; }
    bot_stall = v < 0.4f ? bot_stall + FIXED_DT : 0;
    if (bot_stall > 2.5f) { bot_back = 1.5f; bot_stall = 0; }
    in->steer = clampf(ang * 2.4f, -1.0f, 1.0f);
    in->throttle = v < vt ? clamp01(0.5f + (vt - v) * 0.3f) : 0.0f;
    if (v > vt + 1.2f) in->brake = clamp01((v - vt - 1.2f) / 3.0f);
    return 0;
}
static void rt_creek_bot(const Car *car, CarInput *in)
{
    memset(in, 0, sizeof(*in));
    RtNear cn = route_creek_near(&RT, car->body.pos.x, car->body.pos.z);
    if (cn.dist < 8.0f && cn.s > RT.ck_exit_s - 70.0f) { creek_follow(car, in, RT.ck_len, getenv("BOT_CREEK_V") ? (float)atof(getenv("BOT_CREEK_V")) : 8.0f); return; }   /* on the creek: down it */
    if (bot_path_n < 2) return;   /* across the clearing to it, on the path */
    float x = car->body.pos.x, z = car->body.pos.z, best = 1e9f; int bi = bot_path_at;
    for (int i = bot_path_at; i < bot_path_n && i < bot_path_at + 60; i++) { float d = hypotf(bot_path[2 * i] - x, bot_path[2 * i + 1] - z); if (d < best) { best = d; bi = i; } }
    bot_path_at = bi;
    int ti = bi + 7 < bot_path_n ? bi + 7 : bot_path_n - 1;
    V3 l = qrot(qconj(car->body.rot), v3(bot_path[2 * ti] - x, 0, bot_path[2 * ti + 1] - z));
    float ang = atan2f(l.x, l.z), v = vlen(car->body.vel);
    if (bot_back > 0) { bot_back -= FIXED_DT; in->brake = 1; in->steer = -clampf(ang * 2.0f, -1, 1); return; }
    bot_stall = v < 0.4f ? bot_stall + FIXED_DT : 0;
    if (bot_stall > 2.5f) { bot_back = 1.5f; bot_stall = 0; }
    in->steer = clampf(ang * 2.2f, -1.0f, 1.0f);
    in->throttle = v < 5.0f ? 0.7f : 0.0f;
}

static int bot_creek, bot_explore;
static int bgoal_n, bgoal_at;   /* the Explorer bot's list of places (below) */
static void explore_bot_setup(void);
static void rt_explore_bot(const Car *car, CarInput *in, Run *r);
static float car_end_x, car_end_z;     /* where the last test drive ended */
static int bot_banks = 1;                 /* BOT_BANK=0: the bot never presses bank (only the bridge banks) */
static float hist[16], hist_pts[16];      /* BOT_HIST: seconds and pot points per 2-degree band of drift angle */
static RtRun rt_drive(float alat, float target, float look, int rows)
{
    Car car;
    memset(&car, 0, sizeof(car));
    car_setup(&car, anchors);
    T.manual_gears = 0;
    car_reset(&car, v3(RT.start_x, 0.75f, RT.start_z), RT.start_yaw);
    Run r;
    run_reset(&r, 0);
    float max_s0 = 0;
    if (getenv("BOT_AT")) {   /* build B: start this far down the road, on its middle, rolling at BOT_AT_KMH (default 40); checkpoints behind count as passed */
        float s0 = (float)atof(getenv("BOT_AT")), x, z, hd = route_heading_at(&RT, s0), v0 = getenv("BOT_AT_KMH") ? (float)atof(getenv("BOT_AT_KMH")) / 3.6f : 11.0f;
        route_point(&RT, s0, &x, &z);
        car_reset(&car, v3(x, 0.75f, z), (float)(M_PI / 2) - hd);
        car.body.vel = v3(cosf(hd) * v0, 0, sinf(hd) * v0);
        r.next_cp = (int)(s0 / fmaxf(T.checkpoint_every, 50.0f)); r.prev_s = s0; max_s0 = s0;
    }
    world_run_start();
    route_logs_restore(&RT, NULL);
    Score sc; memset(&sc, 0, sizeof(sc)); sc.mult = 1;
    OldScore os; memset(&os, 0, sizeof(os)); os.mult = 1;
    float max_s = max_s0, sp_sum = 0;
    RtRun res = { 0 };
    int steps = 0;
    float limit_s = getenv("BOT_SECONDS") ? (float)atof(getenv("BOT_SECONDS")) : 1200.0f;
    while (r.state == RUN_RUNNING && steps < (int)(limit_s / FIXED_DT)) {
        CarInput in;
        if (bot_donut) { memset(&in, 0, sizeof(in)); if (r.time > 0.3f) { in.steer = -(float)bot_donut; in.throttle = 1; } else in.brake = 1; }
        else if (bot_creek) rt_creek_bot(&car, &in);
        else if (bot_explore) rt_explore_bot(&car, &in, &r);
        else if (bot_deer) rt_deer_bot(&car, &in);
        else if (bot_shortcut) rt_shortcut_bot(&car, &in);
        else if (bot_drift_deg > 0) rt_drift_bot(&car, alat, &in);
        else rt_bot(&car, alat, look, target, &in);
        if (bot_nobrake) { if (bot_nobrake == 2 && in.brake > 0.3f) in.handbrake = 1; in.brake = 0; }   /* 2: the handbrake instead (the pedal is what the achievement counts) */
        if (getenv("BOT_KICK")) {   /* build B, the ford test: "metres before the ford, seconds, steer": handbrake and steer there, then hold the slide with throttle */
            float kd = 12, kt = 0.5f, ks = 1; sscanf(getenv("BOT_KICK"), "%f,%f,%f", &kd, &kt, &ks);
            RtNear kn = route_near(&RT, car.body.pos.x, car.body.pos.z);
            static float kick_t; if (r.time < FIXED_DT * 1.5f) kick_t = -1;
            if (kick_t < 0 && kn.s > RT.ford_s - kd) kick_t = 0;
            if (kick_t >= 0 && kick_t < kt) { in.handbrake = 1; in.steer = ks; in.throttle = 0.4f; in.brake = 0; kick_t += FIXED_DT; }
            else if (kick_t >= kt && kn.s < RT.ford_s + 5.0f) { in.steer = -0.3f * ks; in.throttle = 0.8f; in.brake = 0; kick_t += FIXED_DT; }
        }
        bot_in = in;
        if (explore && getenv("BOT_LOW") && !car.low && vlen(car.body.vel) < T.low_max_kmh / 3.6f) car_set_low(&car, 1);   /* build D: the bot in Low Range (Explorer) */
        car_step(&car, &in);
        RtPhys ph;
        logs_break = !explore || W.tray_wood < (int)T.wood_full;
        rt_physics(&car, &ph);
        rt_rules(&r, &car, &ph, &max_s, 1);
        if (bot_explore && (bgoal_at >= bgoal_n || (bot_explore == 1 && W.fire_level >= 3))) { if (getenv("BOT_EVENTS")) printf("   the Explorer bot is done at %.1f s\n", r.time); break; }
        if (r.ev_thud > 0) { explore_thud(&car, &ph); res.thuds++; if (getenv("BOT_EVENTS")) printf("   thud: %s at %.0f km/h, %.1f s\n", rt_hit_what, r.ev_thud * 3.6f, r.time); r.ev_thud = 0; }
        if (getenv("BOT_TRACE_PH")) { float a0 = 0, a1 = 0; sscanf(getenv("BOT_TRACE_PH"), "%f,%f", &a0, &a1); RtNear cq = route_creek_near(&RT, car.body.pos.x, car.body.pos.z); static float tq; tq += FIXED_DT;
            if (cq.s > a0 && cq.s < a1 && tq > 0.25f) { tq = 0; printf("   ph hdg %+.0f s %.1f lat %+.1f v %.1f wet %d stag %d tree %d edge %.2f held %d rock %d grade %+.2f F %.0f %.0f %.0f %.0f thr %.2f gear %d low %d rpm %.0f\n", (car_yaw(&car) - ((float)(M_PI / 2) - route_heading_at_creek_pub(cq.s))) * RAD2DEGF, cq.s, cq.lat, vlen(car.body.vel), ph.wet, ph.stag, ph.tree, ph.edge, ph.held, ph.rock, ph.grade, bp.F[0], bp.F[1], bp.F[2], bp.F[3], bot_in.throttle, car.gear, car.low, car.rpm); } }
        if (explore) {   /* build D: the stags */
            int se = stag_event(&car, &ph);
            if (se == 1) { res.stags++; if (getenv("BOT_EVENTS")) printf("   a dead tree down at %.0f km/h, %.1f s (stag %d)\n", ph.stag_speed * 3.6f, r.time, ph.stag); }
            else if (se == 2) { res.thuds++; if (getenv("BOT_EVENTS")) printf("   thud: DEAD TREE at %.0f km/h, %.1f s\n", ph.stag_speed * 3.6f, r.time); }
            else if (se == 3) { r.wood++; if (getenv("BOT_EVENTS")) printf("   firewood %d at %.1f s, from a dead tree (tray %d)\n", r.wood, r.time, W.tray_wood); }
            stag_step(FIXED_DT, 0);
            if (ph.wet >= 0 && ph.wet_smash) { res.wet_smashed++; if (getenv("BOT_EVENTS")) printf("   a wet log smashed at %.0f km/h, %.1f s\n", vlen(car.body.vel) * 3.6f / 0.85f, r.time); }
            else if (ph.wet >= 0 && !wet_seen[ph.wet]) { res.wet_shoved++; wet_seen[ph.wet] = 1; if (getenv("BOT_EVENTS")) printf("   a wet log shoved, %.1f s\n", r.time); }
            trail_step(&car);
            if (T.body_physics != 0 && bp.rolled && bp.rolled_t > 2.2f) { res.rolls++; if (getenv("BOT_EVENTS")) printf("   rolled over at %.1f s; recovered\n", r.time); recover(&car); }
        }
        if (T.body_physics != 0 && bp.last_air_t > 0.3f && getenv("BOT_EVENTS")) { printf("   airborne %.2f s, %.2f m up, %.1f m along, at %.1f s\n", bp.last_air_t, bp.last_air_h, bp.last_air_d, r.time); bp.last_air_t = 0; }
        if (explore && r.state == RUN_RUNNING) {   /* build C: Explorer's finds and the creek way's timing */
            explore_step(&car, 0);
            if (RT.ck_n > 1) { RtNear cn = route_creek_near(&RT, car.body.pos.x, car.body.pos.z);
                if (res.creek_t0 == 0 && route_in_creek(&RT, car.body.pos.x, car.body.pos.z) && cn.s > RT.ck_exit_s) { res.creek_t0 = r.time; if (getenv("BOT_EVENTS")) printf("   into the creek way at %.1f s\n", r.time); }
                if (res.creek_t0 > 0 && res.creek_t1 == 0 && cn.dist < 30.0f && cn.s > RT.ck_len - 25.0f) { res.creek_t1 = r.time; if (getenv("BOT_EVENTS")) printf("   at the ford at %.1f s\n", r.time); if (bot_creek) break; }
                static float next_mark; if (r.time < FIXED_DT * 1.5f) next_mark = 500.0f;
                if (cn.dist < 30.0f && cn.s > next_mark && getenv("BOT_EVENTS")) { printf("   creek %4.0f m at %.1f s\n", cn.s, r.time); next_mark += 500.0f; } }
        }
        if (rt_offroad_timer(&r, &car, &ph)) { res.resets++; if (getenv("BOT_EVENTS")) printf("   back on the track after %.0f s off it, at %.1f s, km %.2f\n", T.offroad_reset, r.time, route_near(&RT, car.body.pos.x, car.body.pos.z).s / 1000); }
        if (ph.saw_ev) { saw_evn[ph.saw_ev]++; if (getenv("BOT_SAW")) printf("   saw pass %d at %.0f km/h, saw %.0f%%, %.1f s\n", ph.saw_ev, vlen(car.body.vel) * 3.6f, W.saw, r.time); ph.saw_ev = 0; }   /* build E */
        if (ph.log_broken && r.state == RUN_RUNNING) { r.wood++; if (explore) { W.tray_wood++; world_log_gone(ph.log_cx, ph.log_cz); } if (getenv("BOT_EVENTS")) printf("   firewood %d at %.1f s, km %.2f%s\n", r.wood, r.time, max_s / 1000, explore ? TextFormat(" (tray %d)", W.tray_wood) : ""); }
        score_update(&sc, &car, FIXED_DT, r.state == RUN_RUNNING && r.dir == DIR_CORRECT, ph.deep, ph.offroad);
        old_score_update(&os, &car, FIXED_DT, r.state == RUN_RUNNING && r.dir == DIR_CORRECT);
        if (bot_donut) donut_update(&sc, &car, FIXED_DT, r.state == RUN_RUNNING);
        if (r.state == RUN_RUNNING && army_check(&car) && getenv("BOT_EVENTS")) printf("   the army man found at %.1f s, km %.2f\n", r.time, max_s / 1000);
        if (r.state == RUN_RUNNING && deer_check(&car) >= 0) { res.deer++; if (getenv("BOT_EVENTS")) printf("   deer found at %.1f s, km %.2f (%d of %d on this device)\n", r.time, max_s / 1000, world_found_count(), W.ndeer); }
        if (r.state == RUN_FINISHED && sc.pot > 0) score_expire(&sc);   /* the bridge banks the pot (as in the game) */
        {
            RtNear wn = route_near(&RT, car.body.pos.x, car.body.pos.z);
            WorldStep ws = { 0 };
            ws.dt = FIXED_DT; ws.running = r.state == RUN_RUNNING; ws.finished = r.state == RUN_FINISHED && !res.finished_seen;
            ws.x = car.body.pos.x; ws.z = car.body.pos.z; ws.s = wn.s; ws.lat = wn.lat; ws.drift_deg = sc.angle; ws.speed = vlen(car.body.vel);
            ws.on_road = fabsf(wn.lat) < route_half_width() + T.verge_w; ws.brake = bot_in.brake > 0; ws.ditch_rule = sc.ditch_t > T.ditch_time || ph.offroad;
            ws.offroad_m = sc.offroad_m; ws.donut_carpark = sc.ev_donut == 2; ws.donut_road = sc.ev_donut == 1;
            ws.pot = sc.pot; ws.banked = sc.ev_banked; ws.banked_value = sc.last_banked; ws.lost = sc.ev_lost; ws.ford_s = RT.ford_s; ws.wood = explore ? W.tray_wood : r.wood;
            ws.explore = explore; ws.in_clearing = route_in_clearing(&RT, car.body.pos.x, car.body.pos.z); explore_creek_step(&ws, car.body.pos.x, car.body.pos.z);
            { static float ps = -1e9f; if (r.time < FIXED_DT * 1.5f) ps = -1e9f;
              if (getenv("BOT_EVENTS") && ps < RT.ford_s && wn.s >= RT.ford_s) printf("   the ford at %.0f km/h, rear slide %.0f deg, %+.1f m off the middle\n", ws.speed * 3.6f, sc.angle, wn.lat);
              ps = wn.s; }
            world_step(&ws);
            if (ws.finished) res.finished_seen = 1;
            for (int a; (a = world_pop()) >= 0; ) { res.ach |= 1ull << a; if (getenv("BOT_EVENTS")) printf("   achievement: %s at %.1f s\n", ACH[a].name, r.time); }
            sc.ev_banked = sc.ev_donut = 0;
        }
        if (sc.ev_lost) { sc.ev_lost = 0; res.lost_ditch++; res.lost_pts += sc.flash_v; sc.ev_lost = 0; if (getenv("BOT_EVENTS")) printf("   lost %.0f in the ditch at %.1f s, km %.2f\n", sc.flash_v, r.time, max_s / 1000); }
        if (r.state == RUN_WALL_OVER) { if (sc.pot > 0) { res.lost_crash++; res.lost_pts += score_combo(&sc); } score_lose(&sc); os.pot = 0; }
        float sp = vlen(car.body.vel);
        sp_sum += sp; if (sp > res.top) res.top = sp;
        if (ph.ditch) res.ditch_s += FIXED_DT;
        if (ph.rough) res.rough_s += FIXED_DT;
        if (getenv("BOT_POS") && steps % (getenv("BOT_POS")[0] == 'a' ? 1 : 50) == 0) printf("   pos t %.2f %.4f %.4f %.4f st %.3f th %.2f g %.4f d%d r%d k%d tr%d\n", r.time, car.body.pos.x - RT.hut_x, car.body.pos.z - RT.hut_z, vlen(car.body.vel), bot_in.steer, bot_in.throttle, ph.grade, ph.ditch, ph.rough, ph.rock, ph.tree);
        if (rows > 1 && steps % 250 == 0) printf("   t %5.1f s  km %.2f  %3.0f km/h  lat %5.1f  grade %5.1f%%  ditch %d rough %d\n", r.time, max_s / 1000, sp * 3.6f, route_near(&RT, car.body.pos.x, car.body.pos.z).lat, ph.grade * 100, ph.ditch, ph.rough);
        res.hit_speed = ph.tree_speed;
        { float la = fabsf(route_near(&RT, car.body.pos.x, car.body.pos.z).lat); if (la > res.max_lat) res.max_lat = la; }
        { /* the scripted player banks once it has been driving straight for a second (a careful player's habit) */
            static float straight; straight = sc.angle < 2.0f ? straight + FIXED_DT : 0;
            float bank_at = getenv("BOT_BANK_AT") ? (float)atof(getenv("BOT_BANK_AT")) : 0;   /* build B: bank only once the pot is worth this */
            if (T.score_bank_button != 0 && bot_banks && straight > 1.0f && sc.pot > 0 && score_combo(&sc) >= bank_at && r.state == RUN_RUNNING) score_expire(&sc);
            int b = (int)fminf(sc.angle / 2.0f, 15.0f); hist[b] += FIXED_DT; hist_pts[b] += sc.rate * FIXED_DT;
        }
        steps++;
    }
    if (sc.pot > 0 && r.state != RUN_WALL_OVER) score_expire(&sc);
    if (os.pot > 0 && r.state != RUN_WALL_OVER) os.total += roundf(os.pot * (float)os.mult / 10.0f) * 10.0f;
    res.old_score = os.total; res.old_x = os.best_mult;
    RtNear n = route_near(&RT, car.body.pos.x, car.body.pos.z);
    car_end_x = car.body.pos.x; car_end_z = car.body.pos.z;
    res.state = r.state; res.time = r.time; res.dist = max_s; res.mean = steps ? sp_sum / steps : 0;
    res.score = sc.total; res.best_x = sc.best_mult; res.end_s = n.s; res.end_lat = n.lat;
    res.cp = r.next_cp; res.ncp = r.ncp; res.offroad_m = sc.offroad_m; res.wood = r.wood;
    if (r.state == RUN_FINISHED && !res.finished_seen) {   /* the last step finished the run: the bridge's achievements */
        WorldStep ws = { 0 }; ws.finished = 1; ws.banked = sc.ev_banked; ws.banked_value = sc.last_banked; world_step(&ws);
        for (int a; (a = world_pop()) >= 0; ) res.ach |= 1ull << a;
    }
    return res;
}

/* Build C, the bollards beside the track (--drivetest with BOT_POSTS=1): for a pair of neighbouring bollards in each row,
 * the ute starts on the road's middle, aimed square at the gap between them, and drives out through it at walking pace,
 * straight and then turned a few degrees. Prints whether it touched a post. */
static int posts_test(void)
{
    int tb = route_template_find("bollard", 0, 0), npass = 0, nclip = 0, nrun = 0;
    /* each try: degrees turned off square, and metres off the gap's middle (a line through it, or one aimed at it from the road's middle) */
    const float angles[] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 3.0f, -3.0f, 1.5f }, offs[] = { 0.0f, 0.1f, -0.15f, 0.2f, -0.3f, 0.0f, 0.0f, 0.0f };
    const int aimed[] = { 0, 0, 0, 0, 0, 0, 0, 1 };
    float hw = car_outline.own ? car_outline.half_w : T.body_half_w, fr = car_outline.own ? car_outline.front : T.body_front, re = car_outline.own ? car_outline.rear : T.body_rear;
    printf("POSTS bollards %.2f m apart face to face (%.2f m between centres); the ute's outline is %.2f m wide at its mirrors\n", T.bollard_gap, T.bollard_gap + 0.32f, 2 * hw);
    for (int k = 0; k + 1 < RT.ntree && nrun < 6; k++) {
        const RtTree *a = &RT.tree[k];
        if (a->t0 != tb) continue;
        int m = -1;   /* its neighbour in the same row: the nearest other bollard, under 3.5 m */
        for (int q = 0; q < RT.ntree; q++) if (q != k && RT.tree[q].t0 == tb && hypotf(RT.tree[q].x - a->x, RT.tree[q].z - a->z) < 3.5f && (m < 0 || hypotf(RT.tree[q].x - a->x, RT.tree[q].z - a->z) < hypotf(RT.tree[m].x - a->x, RT.tree[m].z - a->z))) m = q;
        if (m < k) continue;
        float mx = 0.5f * (a->x + RT.tree[m].x), mz = 0.5f * (a->z + RT.tree[m].z);
        RtNear n = route_near(&RT, mx, mz);
        if (n.s < 20.0f || n.s > 80.0f || (nrun % 2 == 0 && n.lat > 0) || (nrun % 2 == 1 && n.lat < 0)) continue;   /* one pair from each side in turn */
        int clear_behind = 1;   /* nothing else solid in the way out (the plough, the gate, the rail) */
        for (float f = 0.0f; f < 7.0f; f += 0.5f) { float bx = mx + (mx - RT.x[n.i]) / fabsf(n.lat) * f, bz = mz + (mz - RT.z[n.i]) / fabsf(n.lat) * f; int t = route_tree_hit(&RT, bx, bz, 0, 1.3f, 1.3f, -1.3f); if (t >= 0 && t != k + 1 && t != m + 1) clear_behind = 0; }
        if (!clear_behind) continue;
        nrun++;
        float rx, rz; route_point(&RT, n.s, &rx, &rz);
        float out_yaw = atan2f(mx - rx, mz - rz);   /* the car's yaw: 0 = +z, towards +x positive */
        printf("POSTS pair at km %.3f, %s side, %.2f m apart centre to centre:", n.s / 1000, n.lat > 0 ? "left" : "right", hypotf(RT.tree[m].x - a->x, RT.tree[m].z - a->z));
        for (unsigned ai = 0; ai < sizeof(angles) / sizeof(angles[0]); ai++) {
            Car car; memset(&car, 0, sizeof(car)); car_setup(&car, anchors);
            float yaw = out_yaw + angles[ai] * DEG2RADF, back = 10.0f, ox = mx + cosf(out_yaw) * offs[ai], oz = mz - sinf(out_yaw) * offs[ai];
            if (aimed[ai]) car_reset(&car, v3(mx - sinf(out_yaw) * back, 0.75f, mz - cosf(out_yaw) * back), yaw);   /* from the road's middle, a degree or two off */
            else car_reset(&car, v3(ox - sinf(yaw) * back, 0.75f, oz - cosf(yaw) * back), yaw);   /* on a line through the gap (or beside its middle), at the angle */
            int hit = 0;
            for (int st = 0; st < (int)(9.0f / FIXED_DT) && !hit; st++) {
                CarInput in = { 0 }; float v = vlen(car.body.vel);
                in.throttle = v < 2.5f ? 0.6f : 0.0f;   /* walking pace, wheel straight */
                car_step(&car, &in);
                int t = route_tree_hit(&RT, car.body.pos.x, car.body.pos.z, car_yaw(&car), hw, fr, re);
                if (t >= 0 && RT.tree[t].kind == 3 && hypotf(RT.tree[t].x - mx, RT.tree[t].z - mz) < 3.0f) hit = 1;
                if ((car.body.pos.x - mx) * sinf(out_yaw) + (car.body.pos.z - mz) * cosf(out_yaw) > 6.0f) break;   /* through */
            }
            if (aimed[ai]) printf("  | aimed at it from the road %+.1f deg: %s", angles[ai], hit ? "CLIPS" : "through");
            else printf("  | %+.1f deg %+.2f m: %s", angles[ai], offs[ai], hit ? "CLIPS" : "through");
            if (ai == 0) { if (hit) nclip++; else npass++; }
        }
        printf("\n");
    }
    printf("POSTS straight through: %d of %d pairs\n", npass, npass + nclip);
    return nclip > 0 || npass == 0;
}

/* Build C, the Explorer bot (--drivetest with BOT_EXPLORE=1 firewood, 2 the hamper, 3 the rifle, 4 all three): it knows where
 * everything is. A list of places to go, each "touch it" or "stop there"; the long legs along the road (either way) with the
 * road bot's line, the last metres (and anything in the clearing or the creek way) on a path found on a 1 m grid round the
 * trunks, then driven like the creek bot. */
typedef struct { float x, z; int stop; const char *what; } BotGoal;
static BotGoal bgoal[256];
static float *bpath; static int bpath_n, bpath_at, bpath_goal = -1, bot_phase, bot_to_fire; static float bot_wait, bot_leg_t, bot_leg_max = 400.0f, bot_replan, bot_vend; static int bot_runup;
static int plan_path(float fx, float fz, float tx, float tz, float margin)
{
    float x0 = fminf(fx, tx) - margin, x1 = fmaxf(fx, tx) + margin, z0 = fminf(fz, tz) - margin, z1 = fmaxf(fz, tz) + margin;
    int nx = (int)(x1 - x0) + 1, nz = (int)(z1 - z0) + 1;
    if ((long)nx * nz > 9000000L) return 0;
    int *par = malloc(sizeof(int) * (size_t)nx * nz); unsigned char *st = calloc((size_t)nx * nz, 1); int *q = malloc(sizeof(int) * (size_t)nx * nz), qh = 0, qt = 0, goal = -1;
    int si = (int)clampf(fx - x0, 0, (float)(nx - 1)), sj = (int)clampf(fz - z0, 0, (float)(nz - 1));
    q[qt++] = sj * nx + si; st[sj * nx + si] = 3; par[sj * nx + si] = -1;
    while (qh < qt) {
        int k = q[qh++], i = k % nx, j = k / nx;
        if (hypotf(x0 + (float)i + 0.5f - tx, z0 + (float)j + 0.5f - tz) < 2.6f) { goal = k; break; }
        const int di[4] = { 1, -1, 0, 0 }, dj[4] = { 0, 0, 1, -1 };
        for (int d = 0; d < 4; d++) {
            int a = i + di[d], b = j + dj[d]; if (a < 0 || b < 0 || a >= nx || b >= nz) continue;
            int m = b * nx + a; if (st[m]) continue;
            float xx = x0 + (float)a + 0.5f, zz = z0 + (float)b + 0.5f;
            float gx = route_height(&RT, xx + 1.0f, zz, NULL) - route_height(&RT, xx - 1.0f, zz, NULL), gz = route_height(&RT, xx, zz + 1.0f, NULL) - route_height(&RT, xx, zz - 1.0f, NULL);
            int tb = route_tree_hit(&RT, xx, zz, 0, 1.75f, 1.75f, -1.75f);   /* trunks with room for the ute to swing; posts and walls closer */
            int blocked = (tb >= 0 && (RT.tree[tb].kind == 0 || route_tree_hit(&RT, xx, zz, 0, 1.25f, 1.25f, -1.25f) >= 0)) || route_edge_at(&RT, xx, zz) > T.edge_stop - 1.5f || hypotf(gx, gz) * 0.5f > 0.30f;
            if (blocked && (hypotf(xx - tx, zz - tz) < 3.0f || hypotf(xx - fx, zz - fz) < 3.5f)) blocked = 0;   /* the log itself is the goal; and the ute may be wedged where it starts */
            st[m] = blocked ? 2 : 3;
            if (!blocked) { par[m] = k; q[qt++] = m; }
        }
    }
    free(bpath); bpath = NULL; bpath_n = 0; bpath_at = 0;
    if (goal >= 0) {
        int n = 0; for (int k = goal; k >= 0; k = par[k]) n++;
        bpath = malloc(sizeof(float) * 2 * (size_t)(n + 1)); bpath_n = n;
        int i2 = n - 1; for (int k = goal; k >= 0; k = par[k], i2--) { bpath[2 * i2] = x0 + (float)(k % nx) + 0.5f; bpath[2 * i2 + 1] = z0 + (float)(k / nx) + 0.5f; }
        for (int pass = 0; pass < 5; pass++) for (int i = 1; i + 1 < n; i++) for (int c = 0; c < 2; c++) bpath[2 * i + c] = 0.25f * bpath[2 * (i - 1) + c] + 0.5f * bpath[2 * i + c] + 0.25f * bpath[2 * (i + 1) + c];
        bpath[2 * n] = tx; bpath[2 * n + 1] = tz; bpath_n = n + 1;   /* and on into it */
    }
    if (goal < 0 && getenv("BOT_TRACE_EX")) printf("   (no path from (%.0f, %.0f) to (%.0f, %.0f), margin %.0f: %d cells searched)\n", fx, fz, tx, tz, margin, qt);
    free(par); free(st); free(q);
    return goal >= 0;
}
static void goal_add(float x, float z, int stop, const char *what) { if (bgoal_n < 256) bgoal[bgoal_n++] = (BotGoal){ x, z, stop, what }; }
static void explore_bot_setup(void)
{
    bgoal_n = bgoal_at = 0; bpath_goal = -1;
    if (RT.ck_n > 1 && bot_explore != 2) creek_path();   /* the creek way's own path, for the legs down it */
    W.tray_wood = 0; W.fire_level = 0; W.night = 0; W.hamper = 0; W.board = 0; W.hut_night = 0; W.rifle = 0; W.nlogs_gone = 0; W.saw = 100; W.razor = 0; W.streak = 0; W.sharpened = 0;   /* build E: a sharp saw to start */
    if (getenv("BD_FIRE")) W.fire_level = atoi(getenv("BD_FIRE"));   /* a quest started part way: the fires already lit, the rifle already in the tray */
    if (getenv("BD_FLAT")) { W.punct = atoi(getenv("BD_FLAT")); W.tyre_wear[W.punct - 1] = 1; W.spare = 0; W.body_dmg = 0.5f; }   /* build E: a flat and no spare, to limp home */
    if (getenv("BD_RIFLE")) W.rifle = atoi(getenv("BD_RIFLE"));
    route_logs_restore(&RT, NULL);
    if (bot_explore == 1 || bot_explore == 4) {   /* firewood: the clearing's and the road's in order out from the hut, then the bridge's, then the creek's */
        static float lg[160][3]; int n = 0, ncreek = 0;
        for (int k = 0; k + 1 < RT.ntree && n < 160; k++) {
            const RtTree *t = &RT.tree[k];
            if (!t->wood || t->kind == 5 || t->t0 < 0 || strcmp(route_template(t->t0)->name, "log")) continue;
            RtNear rn = route_near(&RT, t->x, t->z), cn = route_creek_near(&RT, t->x, t->z);
            int creek = cn.dist < 40.0f && cn.s > RT.ck_exit_s && fabsf(rn.lat) > 60.0f;
            lg[n][0] = t->x; lg[n][1] = t->z; lg[n][2] = creek ? 100000.0f + cn.s : rn.s; ncreek += creek; n++;
        }
        int nlogs = n, nst = 0;
        static float stg[48][3];
        if (!getenv("BOT_STAGS") || atoi(getenv("BOT_STAGS"))) for (int k = 0; k < RT.nstag; k++) {   /* build D: the stags, in the same order (their logs follow when each is down); BOT_STAGS=2: the road strip's only */
            if (getenv("BOT_STAGS") && atoi(getenv("BOT_STAGS")) == 2 && RT.stag[k].where != 0) continue;
            RtNear rn = route_near(&RT, RT.stag[k].x, RT.stag[k].z), cn = route_creek_near(&RT, RT.stag[k].x, RT.stag[k].z);
            stg[nst][0] = RT.stag[k].x; stg[nst][1] = RT.stag[k].z; stg[nst][2] = RT.stag[k].where == 2 ? 100000.0f + cn.s : rn.s; nst++;   /* the clearing's by the road's nearest point too */
        }
        static float all[208][4]; int na = 0;
        for (int i = 0; i < n && na < 208; i++) { all[na][0] = lg[i][0]; all[na][1] = lg[i][1]; all[na][2] = lg[i][2]; all[na][3] = 0; na++; }
        for (int i = 0; i < nst && na < 208; i++) { all[na][0] = stg[i][0]; all[na][1] = stg[i][1]; all[na][2] = stg[i][2]; all[na][3] = 1; na++; }
        for (int a = 0; a < na; a++) for (int b = a + 1; b < na; b++) if (all[b][2] < all[a][2]) { float t[4]; memcpy(t, all[a], sizeof(t)); memcpy(all[a], all[b], sizeof(t)); memcpy(all[b], t, sizeof(t)); }
        (void)nlogs;
        int per = (int)T.wood_full, taken = 0, loads = 0; (void)loads;
        for (int i = 0; i < na; i++) {
            if (all[i][3] > 0.5f) { goal_add(all[i][0], all[i][1], 0, "a dead tree"); continue; }
            goal_add(all[i][0], all[i][1], 0, all[i][2] > 99999.0f ? "a log down the creek" : "a log");
            if (++taken == per) { taken = 0; loads++; }   /* the fire is visited when the tray fills (rt_explore_bot); every log is listed, in case some can't be reached */
        }
        printf("DRIVE the Explorer bot: %d firewood logs known (%d down the creek way), and %d dead trees (%d logs in them); three loads of %d\n", n, ncreek, nst, 3 * nst, per);
    }
    if (bot_explore == 2 || bot_explore == 4) {   /* the hamper, nearest first, then the table */
        int done[HAMPER_N] = { 0 }; float px = RT.start_x, pz = RT.start_z;
        for (int k = 0; k < HAMPER_N; k++) {
            int b = -1; float bd = 1e9f;
            for (int i = 0; i < HAMPER_N; i++) if (!done[i] && W.item[i].placed && hypotf(W.item[i].x - px, W.item[i].z - pz) < bd) { bd = hypotf(W.item[i].x - px, W.item[i].z - pz); b = i; }
            if (b < 0) break;
            done[b] = 1; goal_add(W.item[b].x, W.item[b].z, 0, HAMPER_NAME[b]); px = W.item[b].x; pz = W.item[b].z;
        }
        float hd = atan2f(W.picnic_z - W.fire_z, W.picnic_x - W.fire_x);
        goal_add(W.picnic_x - cosf(hd) * 3.5f, W.picnic_z - sinf(hd) * 3.5f, 1, "the picnic table");
    }
    if (bot_explore == 3 || bot_explore == 4) { if (W.rifle < 1) goal_add(W.rifle_x, W.rifle_z, 0, "Quayle's rifle"); goal_add(W.army_x, W.army_z, 0, "Quayle"); }
    if (bot_explore == 5) {   /* follow-up: Quayle back on duty and in the tray; the nearest deer (any of the 25) into the tray, then the camp oven at the table */
        W.rifle = 2; W.quayle = 1; W.tray_deer = 0; W.stew = 0; W.found = 0; W.earned[ACH_CAMP_OVEN] = 0;
        int b = -1; float bd = 1e9f;
        for (int d = 0; d < WORLD_DEER + W.nextra; d++) { if (d >= W.ndeer && d < WORLD_DEER) continue; float dd = hypotf(W.deer[d].x - RT.start_x, W.deer[d].z - RT.start_z); if (dd < bd) { bd = dd; b = d; } }
        if (b >= 0) goal_add(W.deer[b].x, W.deer[b].z, 0, "a deer for the tray");
        float hd = atan2f(W.picnic_z - W.fire_z, W.picnic_x - W.fire_x);
        goal_add(W.picnic_x - cosf(hd) * 3.5f, W.picnic_z - sinf(hd) * 3.5f, 1, "the camp oven");
        printf("DRIVE the Explorer bot: Quayle in the tray, deer %d (%.0f m off) for the tray, then the table on a level %d fire\n", b + 1, bd, W.fire_level);
    }
}
/* stuck: less than a metre in the last three seconds (pushing against a trunk jitters, so speed alone doesn't tell) */
static int bot_stuck(const Car *car)
{
    static float t, px, pz;
    t += FIXED_DT;
    if (t < 3.0f) return 0;
    int stuck = hypotf(car->body.pos.x - px, car->body.pos.z - pz) < 1.0f;
    t = 0; px = car->body.pos.x; pz = car->body.pos.z;
    return stuck;
}
/* follow bpath with pure pursuit; returns 1 when at its end */
static int follow(const Car *car, CarInput *in, float vmax)
{
    float x = car->body.pos.x, z = car->body.pos.z, best = 1e9f; int bi = bpath_at;
    for (int i = bpath_at; i < bpath_n && i < bpath_at + 60; i++) { float d = hypotf(bpath[2 * i] - x, bpath[2 * i + 1] - z); if (d < best) { best = d; bi = i; } }
    bpath_at = bi;
    int ti = bi + 6 < bpath_n ? bi + 6 : bpath_n - 1, fi = bi + 16 < bpath_n ? bi + 16 : bpath_n - 1;
    V3 l = qrot(qconj(car->body.rot), v3(bpath[2 * ti] - x, 0, bpath[2 * ti + 1] - z));
    float ang = atan2f(l.x, l.z), v = vlen(car->body.vel);
    float h1 = atan2f(bpath[2 * ti + 1] - z, bpath[2 * ti] - x), h2 = atan2f(bpath[2 * fi + 1] - bpath[2 * ti + 1], bpath[2 * fi] - bpath[2 * ti]), bend = fabsf(atan2f(sinf(h2 - h1), cosf(h2 - h1)));
    float left = (float)(bpath_n - 1 - bi), vt = clampf(fminf(vmax - 8.0f * bend, 1.0f + 0.5f * left), 1.5f, vmax);
    if (bot_vend > 0 && left < 26.0f) vt = clampf(bot_vend - 4.0f * bend, 4.3f, bot_vend);   /* build E: over a log at the saw's speed, not creeping up to it */
    if (bot_back > 0) { bot_back -= FIXED_DT; in->brake = 1; in->steer = -clampf(ang * 2.0f, -1, 1); return 0; }
    if (bot_stuck(car)) { bot_back = 1.4f; return 0; }
    in->steer = clampf(ang * 2.2f, -1.0f, 1.0f);
    in->throttle = v < vt ? clamp01(0.5f + (vt - v) * 0.3f) : 0.0f;
    if (v > vt + 1.5f) in->brake = clamp01((v - vt - 1.5f) / 3.0f);
    return bi >= bpath_n - 2;
}
static void rt_explore_bot(const Car *car, CarInput *in, Run *r)
{
    memset(in, 0, sizeof(*in));
    (void)r;
    if (W.tray_wood >= (int)T.wood_full && W.fire_level < 3 && !bot_to_fire && bgoal_at < bgoal_n && bgoal_n < 255) {   /* a full tray: to the fire first */
        memmove(&bgoal[bgoal_at + 1], &bgoal[bgoal_at], sizeof(BotGoal) * (size_t)(bgoal_n - bgoal_at)); bgoal_n++;
        bgoal[bgoal_at] = (BotGoal){ W.fire_x - 3.0f, W.fire_z - 4.0f, 1, "the fire" }; bot_to_fire = 1; bpath_goal = -1; bot_leg_t = 0; bot_phase = 0;
    }
    if (T.saw_on != 0 && W.saw < 12.0f && bgoal_at < bgoal_n && strcmp(bgoal[bgoal_at].what, "the saw") && bgoal_n < 255) {   /* build E: blunt: to the hut to sharpen */
        memmove(&bgoal[bgoal_at + 1], &bgoal[bgoal_at], sizeof(BotGoal) * (size_t)(bgoal_n - bgoal_at)); bgoal_n++;
        bgoal[bgoal_at] = (BotGoal){ W.fire_x - 3.0f, W.fire_z - 4.0f, 1, "the saw" }; bpath_goal = -1; bot_leg_t = 0; bot_phase = 0;
    }
    if (bgoal_at >= bgoal_n || (bot_explore == 1 && W.fire_level >= 3)) { in->handbrake = 1; return; }   /* done: parked (the brake at a standstill would be reverse) */
    BotGoal *g = &bgoal[bgoal_at];
    if (!strcmp(g->what, "the fire") && W.tray_wood < (int)T.wood_full) { bgoal_at++; bot_to_fire = 0; bot_phase = 0; bpath_goal = -1; bot_leg_t = 0; bot_wait = 0; return; }   /* the wood's on */
    float x = car->body.pos.x, z = car->body.pos.z, dg = hypotf(g->x - x, g->z - z), v = vlen(car->body.vel);
    bot_leg_t += FIXED_DT;
    int done = 0;
    /* each goal is done when what it's for has happened: the log gone, the thing found, the wood on, the board out */
    if (g->stop && !strcmp(g->what, "the saw") && dg < 3.5f && (v < 0.3f || bot_wait > 0)) {   /* build E: the bot files it, a good job (20 s) */
        bot_wait += FIXED_DT; in->handbrake = 1;
        if (bot_wait > 20.0f) { W.saw = 100.0f; W.razor = 0; W.sharpened++; if (W.sharpened >= 3) world_earn(ACH_MECHANIC); if (getenv("BOT_EVENTS")) printf("   the saw sharpened (%d today), %.1f s\n", W.sharpened, r ? r->time : 0.0f); }
        if (W.saw >= 99.0f) { bgoal_at++; bot_phase = 0; bpath_goal = -1; bot_leg_t = 0; bot_wait = 0; }
        return;
    }
    if (g->stop) { if (dg < 3.0f) { in->brake = v > 0.3f ? 1.0f : 0.0f; in->handbrake = 1; if (v < 0.3f) bot_wait += FIXED_DT; if (bot_wait > 0.5f) done = !strcmp(g->what, "the picnic table") ? W.board : !strcmp(g->what, "the camp oven") ? W.stew : !strcmp(g->what, "the saw") ? W.saw >= 99.0f : 1; if (bot_wait > 3.0f && !done) { bot_wait = 0; g->x += (W.picnic_x - g->x) * 0.4f; g->z += (W.picnic_z - g->z) * 0.4f; bpath_goal = -1; bot_phase = 0; } } }
    else if (!strcmp(g->what, "Quayle's rifle")) done = W.rifle >= 1;
    else if (!strcmp(g->what, "Quayle")) done = W.rifle == 2;
    else if (!strcmp(g->what, "a deer for the tray")) done = W.tray_deer != 0;
    else if (!strncmp(g->what, "a log", 5)) { int t = route_tree_hit(&RT, g->x, g->z, 0, 0.3f, 0.3f, -0.3f); done = t < 0 || RT.tree[t].kind != 5; }
    else if (!strcmp(g->what, "a dead tree")) {   /* build D: down, then its three logs next */
        int k = -1; for (int q = 0; q < RT.nstag; q++) if (fabsf(RT.stag[q].x - g->x) < 0.01f && fabsf(RT.stag[q].z - g->z) < 0.01f) k = q;
        done = k < 0 || stag_down(k);
        if (done && k >= 0 && bgoal_n + 3 < 256) {
            memmove(&bgoal[bgoal_at + 4], &bgoal[bgoal_at + 1], sizeof(BotGoal) * (size_t)(bgoal_n - bgoal_at - 1)); bgoal_n += 3;
            for (int q = 0; q < 3; q++) bgoal[bgoal_at + 1 + q] = (BotGoal){ RT.stag[k].log_x[q], RT.stag[k].log_z[q], 0, "a dead tree's log" };
        }
    }
    else if (!strcmp(g->what, "a dead tree's log")) {
        done = 1; for (int k = 0; k < RT.nstag; k++) for (int q = 0; q < 3; q++) if (fabsf(RT.stag[k].log_x[q] - g->x) < 0.01f && fabsf(RT.stag[k].log_z[q] - g->z) < 0.01f) done = stag_log_taken(k, q);
    }
    else { for (int i = 0; i < HAMPER_N; i++) if (!strcmp(g->what, HAMPER_NAME[i])) done = (W.hamper >> i) & 1; }
    if (bot_leg_t == FIXED_DT) bot_leg_max = 150.0f + dg / 2.5f;
    int must_goal = !strcmp(g->what, "the fire") || !strcmp(g->what, "the picnic table") || !strcmp(g->what, "Quayle") || !strcmp(g->what, "the camp oven") || !strcmp(g->what, "the saw");
    if (!done && bot_leg_t > bot_leg_max && must_goal) { bot_leg_t = FIXED_DT; bot_phase = 0; bpath_goal = -1; }   /* start that leg over, never skip it */
    if (done || bot_leg_t > bot_leg_max) {
        if (!done && getenv("BOT_EVENTS")) printf("   gave up on %s (%.0f, %.0f)\n", g->what, g->x, g->z);
        bgoal_at++; bpath_goal = -1; bot_wait = 0; bot_leg_t = 0; bot_phase = 0; bot_runup = 0;
        return;
    }
    if (g->stop && dg < 3.0f) return;
    if (getenv("BOT_TRACE_EX") && ((int)(bot_leg_t / FIXED_DT + 0.5f)) % (int)(2.0f / FIXED_DT) == 0) {
        RtNear t1 = route_creek_near(&RT, x, z), t2 = route_creek_near(&RT, g->x, g->z);
        printf("   [%s %d] at (%.0f, %.0f) goal (%.0f, %.0f) %.0f m away, %.1f m/s, leg %d, path %d/%d; creek %.0f %+.1f -> %.0f %+.1f\n", g->what, bgoal_at, x, z, g->x, g->z, dg, v, bot_phase, bpath_at, bpath_n, t1.s, t1.lat, t2.s, t2.lat);
    }
    RtNear cn = route_near(&RT, x, z), gn = route_near(&RT, g->x, g->z);
    if (cn.s < 0) cn.lat = copysignf(hypotf(cn.s, cn.lat), cn.lat);   /* behind the road's start (the car park end): the real distance to it */
    if (gn.s < 0) gn.lat = copysignf(hypotf(gn.s, gn.lat), gn.lat);
    RtNear gc = route_creek_near(&RT, g->x, g->z), cc = route_creek_near(&RT, x, z);
    int goal_creek = gc.dist < 40.0f && gc.s > RT.ck_exit_s && (fabsf(gn.lat) > 60.0f || route_in_creek(&RT, g->x, g->z)), car_creek = cc.dist < 40.0f && cc.s > RT.ck_exit_s && (fabsf(cn.lat) > 60.0f || route_in_creek(&RT, x, z));   /* part 2: the creek's own strip near the ford too (it's narrow now) */
    int must = !strcmp(g->what, "the fire") || !strcmp(g->what, "the picnic table") || !strcmp(g->what, "Quayle") || !strcmp(g->what, "the saw");   /* never skipped */
    float gs_road = route_in_clearing(&RT, g->x, g->z) ? 30.0f : gn.s;
    /* the leg, a step at a time: 0 decide, 1 back to the road, 2 along it, 3 in to the goal, 4 the creek way */
    if (bot_phase == 0) {
        if (goal_creek || car_creek) bot_phase = 4;
        else if (fabsf(gs_road - cn.s) > 90.0f && dg > 60.0f) bot_phase = fabsf(cn.lat) > 5.0f ? 1 : 2;
        else bot_phase = 3;
        bpath_goal = -1;
    }
    if (bot_phase == 4) {
        float cs = car_creek ? cc.s : RT.ck_exit_s - 40.0f, gs = goal_creek ? gc.s : (cs > 0.5f * RT.ck_len ? RT.ck_len : RT.ck_exit_s - 40.0f);   /* out by its nearer end: the ford or the clearing */
        if (fabsf(gs - cs) < 25.0f && (car_creek || dg < 60.0f)) { bot_phase = 3; bpath_goal = -1; return; }
        if (cc.dist > 8.0f || cc.s < RT.ck_exit_s - 70.0f) {   /* first to the creek in the clearing (or back to its line) */
            float ex, ez; route_creek_point(&RT, cc.dist > 8.0f && cc.s > RT.ck_exit_s - 70.0f ? cc.s : RT.ck_exit_s - 60.0f, &ex, &ez);
            if (bpath_goal != 2000) { if (!plan_path(x, z, ex, ez, 60.0f) && !plan_path(x, z, ex, ez, 140.0f)) { bot_phase = 1; return; } bpath_goal = 2000; }
            if (follow(car, in, 5.0f)) bpath_goal = -1;
            return;
        }
        bpath_goal = -1;
        if (creek_follow(car, in, gs, 6.5f)) { bot_phase = goal_creek ? 3 : 0; if (!goal_creek) bot_phase = 1; }
        return;
    }
    if (bot_phase == 1) {   /* back to the road: a path to its nearest point */
        if (fabsf(cn.lat) < 3.0f) { bot_phase = 2; bpath_goal = -1; return; }
        if (bpath_goal != 3000) {
            float rx, rz; route_point(&RT, clampf(cn.s, 4.0f, RT.length - 4.0f), &rx, &rz);
            if (!plan_path(x, z, rx, rz, 30.0f) && !plan_path(x, z, rx, rz, 100.0f)) { bot_phase = 2; return; }
            bpath_goal = 3000; bot_replan = 0;
        }
        bot_replan += FIXED_DT;
        if (follow(car, in, 4.0f) || bot_replan > 25.0f) bpath_goal = -1;
        return;
    }
    if (bot_phase == 2) {   /* along the road, up or down it */
        if (fabsf(gs_road - cn.s) < 60.0f) { bot_phase = 3; bpath_goal = -1; return; }
        if (fabsf(cn.lat) > 8.0f) { bot_phase = 1; bpath_goal = -1; return; }
        float dir = gs_road > cn.s ? 1.0f : -1.0f, look = 9.0f + 0.5f * v, tx, tz;
        route_point(&RT, clampf(cn.s + dir * look, 0, RT.length), &tx, &tz);
        V3 l = qrot(qconj(car->body.rot), v3(tx - x, 0, tz - z));
        float ang = atan2f(l.x, l.z), rr = route_min_radius_ahead(&RT, dir > 0 ? cn.s : fmaxf(cn.s - 60.0f, 0), 60.0f), vt = fminf(14.0f, sqrtf(3.5f * rr));
        if (fabsf(gs_road - cn.s) < 150.0f) vt = fminf(vt, 6.0f);
        in->steer = clampf(ang * 2.5f, -1, 1); in->throttle = v < vt ? 1.0f : 0.0f; if (v > vt + 2.0f) in->brake = clamp01((v - vt - 2.0f) / 4.0f);
        if (bot_back > 0) { bot_back -= FIXED_DT; memset(in, 0, sizeof(*in)); in->brake = 1; }
        else if (bot_stuck(car)) bot_back = 1.4f;
        return;
    }
    /* 3: in to the goal. Build D: a dead tree is driven at straight from 25 m, at 30 km/h, so it goes over */
    if (!strcmp(g->what, "a dead tree") && dg < 25.0f) {
        V3 l = qrot(qconj(car->body.rot), v3(g->x - x, 0, g->z - z));
        float ang = atan2f(l.x, l.z);
        if (bot_back > 0) { bot_back -= FIXED_DT; in->brake = 1; in->steer = -clampf(ang * 2.0f, -1, 1); return; }
        bot_stall = v < 0.4f ? bot_stall + FIXED_DT : 0;
        if (bot_stall > 1.5f) { bot_back = 4.0f; bot_stall = 0; }
        in->steer = clampf(ang * 2.4f, -1, 1); in->throttle = v < 8.5f ? 1.0f : 0.0f;
        return;
    }
    int log_goal = T.saw_on != 0 && (!strncmp(g->what, "a log", 5) || !strcmp(g->what, "a dead tree's log"));
    bot_vend = 0;
    if (log_goal && (bot_runup || (bpath_goal != 4000 && dg < 12.0f))) {   /* build E: too close for the saw's speed: back off for a run-up first */
        if (bpath_goal != 4100) {
            float ux = x - g->x, uz = z - g->z, ul = fmaxf(hypotf(ux, uz), 0.1f); int ok = 0;
            for (int a = 0; a < 8 && !ok; a++) { float an = 0.785f * (float)((a + 1) / 2) * (a & 1 ? 1.0f : -1.0f), cx = (ux * cosf(an) - uz * sinf(an)) / ul, cz = (ux * sinf(an) + uz * cosf(an)) / ul;
                ok = route_tree_hit(&RT, g->x + cx * 20.0f, g->z + cz * 20.0f, 0, 1.75f, 1.75f, -1.75f) < 0 && plan_path(x, z, g->x + cx * 20.0f, g->z + cz * 20.0f, 30.0f); }
            if (!ok) { bot_runup = 0; bpath_goal = -1; }
            else { bot_runup = 1; bpath_goal = 4100; }
        }
        if (bpath_goal == 4100 && follow(car, in, 4.5f)) { bot_runup = 0; bpath_goal = -1; }
        if (bot_runup) return;
    }
    if (bpath_goal != 4000) {
        if (!plan_path(x, z, g->x, g->z, 50.0f) && !plan_path(x, z, g->x, g->z, 120.0f)) {
            if (must) { bot_phase = fabsf(cn.lat) > 5.0f ? 1 : 2; return; }   /* the fire, the table, Quayle: round by the road again */
            if (getenv("BOT_EVENTS")) printf("   no path to %s\n", g->what);
            bgoal_at++; bot_leg_t = 0; bot_phase = 0; return;
        }
        bpath_goal = 4000; bot_replan = 0;
        if (log_goal && bpath_n >= 2) {   /* build E: on past the log, so the run carries over it */
            float dx = bpath[2 * bpath_n - 2] - bpath[2 * bpath_n - 6 > 0 ? 2 * bpath_n - 6 : 0], dz = bpath[2 * bpath_n - 1] - bpath[2 * bpath_n - 5 > 1 ? 2 * bpath_n - 5 : 1], dl = fmaxf(hypotf(dx, dz), 0.1f);
            bpath = realloc(bpath, sizeof(float) * 2 * (size_t)(bpath_n + 10));
            for (int k = 1; k <= 10; k++) { bpath[2 * bpath_n] = g->x + dx / dl * (float)k; bpath[2 * bpath_n + 1] = g->z + dz / dl * (float)k; bpath_n++; }
        }
    }
    bot_replan += FIXED_DT;
    if (log_goal) bot_vend = 6.0f;
    if (follow(car, in, 4.5f) && log_goal) { bot_runup = 1; bpath_goal = -1; }   /* build E: over it and not through: round again */
    else if ((bpath_n && bpath_at >= bpath_n - 2 && dg > 4.0f) || bot_replan > 40.0f) bpath_goal = -1;   /* at its end but not there, or slow: plan again */
}

/* Build D, the crawl test (--drivetest with BOT_CRAWL=1, Explorer): each hard place on the creek way (the climb-outs, the
 * waterfall's step, the rock gardens, and any steep spot the route test's search lists in RT_SPOTS), driven by the creek
 * bot from a standstill 30 m before it to 40 m past it, in high range and then in Low Range, two minutes each at most. */
static int crawl_once(float s_from, float s_to, int low, float *took, float *got_to, int *thuds)
{
    Car car; memset(&car, 0, sizeof(car)); car_setup(&car, anchors); T.manual_gears = 0;
    float lat0 = route_creek_line(&RT, s_from, NULL), x, z; route_creek_point(&RT, s_from, &x, &z);
    int i = (int)(s_from / RT_DS); float hd = atan2f(RT.ck_z[i + 1] - RT.ck_z[i], RT.ck_x[i + 1] - RT.ck_x[i]);
    x -= sinf(hd) * lat0; z += cosf(hd) * lat0;
    car_reset(&car, v3(x, 0.75f, z), (float)(M_PI / 2) - hd);
    if (low) car_set_low(&car, 1);
    memset(W.tyre_wear, 0, sizeof(W.tyre_wear)); W.punct = 0; W.body_dmg = 0;   /* build E: each from a sound ute */
    Run r; run_reset(&r, 0); r.state = RUN_RUNNING;
    float max_s = 0, best = s_from; *thuds = 0; bot_stall = bot_back = 0;
    for (int st = 0; st < 120 * 50; st++) {
        CarInput in; memset(&in, 0, sizeof(in));
        if (creek_follow(&car, &in, s_to, 6.0f)) { *took = (float)st * FIXED_DT; *got_to = s_to; return 1; }
        car_step(&car, &in);
        RtPhys ph; logs_break = 0; rt_physics(&car, &ph); rt_rules(&r, &car, &ph, &max_s, 1);
        if (r.ev_thud > 0) { explore_thud(&car, &ph); (*thuds)++; r.ev_thud = 0; }
        explore_step(&car, 0);   /* build E: Low and slow, Wet feet */
        RtNear cn = route_creek_near(&RT, car.body.pos.x, car.body.pos.z);
        if (cn.dist < 20.0f) best = fmaxf(best, cn.s);
    }
    *took = 120.0f; *got_to = best;
    return 0;
}
/* build E, the flat test (--drivetest with BOT_FLAT=1, Explorer): on a straight bit of road at 40 km/h, wheel held straight,
 * half throttle for 6 s: how far left (+) it ends up and how much speed it keeps, with no flat and with each wheel flat */
static int flat_test(void)
{
    explore = 1; RT.explore = 1; W.mode = MODE_EXPLORE;
    float s0 = getenv("BOT_AT") ? (float)atof(getenv("BOT_AT")) : 1200.0f;
    const char *nm[5] = { "no flat", "front left", "front right", "rear left", "rear right" };
    for (int f = 0; f <= 4; f++) {
        Car car; memset(&car, 0, sizeof(car)); car_setup(&car, anchors); bp.init = 0;
        float x, z, hd = route_heading_at(&RT, s0); route_point(&RT, s0, &x, &z);
        car_reset(&car, v3(x, route_height(&RT, x, z, NULL) + 0.75f, z), (float)(M_PI / 2) - hd); car.body.vel = v3(cosf(hd) * 11.0f, 0, sinf(hd) * 11.0f);
        W.punct = f; Run r; run_reset(&r, 0); r.state = RUN_RUNNING; float ms = 0;
        for (int st = 0; st < 6 * 50; st++) { CarInput in; memset(&in, 0, sizeof(in)); in.throttle = 0.5f; car_step(&car, &in); RtPhys ph; rt_physics(&car, &ph); rt_rules(&r, &car, &ph, &ms, 1); }
        float px = car.body.pos.x - x, pz = car.body.pos.z - z, left = -px * sinf(hd) + pz * cosf(hd);
        printf("FLAT %-12s after 6 s: %+.1f m left of the line it started on, %.0f km/h\n", nm[f], left, vlen(car.body.vel) * 3.6f);
    }
    W.punct = 0;
    return 0;
}
/* build E, the gate test (--drivetest with BOT_GATETEST=1, Explorer): each of today's gates, from a standstill 25 m up the
 * creek, in Low Range, the bot lined up on the doable line the placement found: through, clean, a stake clipped, stuck? */
static int gate_test(void)
{
    explore = 1; RT.explore = 1; W.mode = MODE_EXPLORE; route_logs_restore(&RT, NULL); setenv("BOT_GATES", "1", 1);
    if (getenv("BD_DAY")) W.day = atoi(getenv("BD_DAY"));
    gates_place(); int clean = 0;
    for (int g = 0; g < ngate; g++) {
        float t; float got; int th; gate[g].done = 0; gate[g].bent[0] = gate[g].bent[1] = 0; gate[g].prev_along = 0; TR.fault_t = 9;
        int ok = crawl_once(gate[g].s - 25.0f, gate[g].s + 15.0f, 1, &t, &got, &th);
        int c = gate[g].done && !gate[g].bent[0] && !gate[g].bent[1] && TR.fault_t > 0.5f; clean += c;
        printf("GATE %d at %.0f m: %s, %s%s\n", g, gate[g].s, ok ? TextFormat("through in %.0f s", t) : "STUCK", gate[g].done ? "between the stakes" : "MISSED the gate", gate[g].bent[0] || gate[g].bent[1] ? ", a stake clipped" : c ? ", clean" : "");
    }
    printf("GATE %d of %d driven through cleanly\n", clean, ngate);
    return 0;
}
static int crawl_test(void)
{
    explore = 1; RT.explore = 1; W.mode = MODE_EXPLORE; route_logs_restore(&RT, NULL);
    float sp[48]; const char *what[48]; int n = 0;
    for (int b = 0; b < RT.ck_nbyp && n < 40; b++) { sp[n] = RT.ck_byp_s[b] - (RT.ck_byp_kind[b] == 2 ? 30.0f : 22.0f); what[n++] = RT.ck_byp_kind[b] == 0 ? "log jam's climb-out" : RT.ck_byp_kind[b] == 1 ? "fallen tree's climb-out" : "down the waterfall"; }
    for (int g = 0; g < RT.ck_nrock && n < 40; g++) { sp[n] = RT.ck_rock_s0[g]; what[n++] = "rock garden"; }
    for (int g = 0; g < RT.ck_nmoss && n < 40; g++) { int dup = 0; for (int k = 0; k < n; k++) if (fabsf(sp[k] - RT.ck_moss_s[g]) < 30.0f) dup = 1; if (!dup) { sp[n] = RT.ck_moss_s[g]; what[n++] = "mossy fallen tree"; } }
    if (getenv("RT_SPOTS")) { FILE *f = fopen(getenv("RT_SPOTS"), "r"); float a, gr; while (f && n < 48 && fscanf(f, "%f %f", &a, &gr) == 2) { int dup = 0; for (int k = 0; k < n; k++) if (fabsf(sp[k] - a) < 30.0f) dup = 1; if (!dup) { sp[n] = a; what[n++] = "steep spot (route test)"; } } if (f) fclose(f); }
    for (int a = 0; a < n; a++) for (int b = a + 1; b < n; b++) if (sp[b] < sp[a]) { float t = sp[a]; sp[a] = sp[b]; sp[b] = t; const char *w = what[a]; what[a] = what[b]; what[b] = w; }
    printf("CRAWL each hard place on the creek way, from a standstill 30 m before it to 40 m past it (creek bot, two minutes at most)\n");
    int cleared_low = 0, stuck_high = 0;
    for (int k = 0; k < n; k++) {
        float t0, g0, t1, g1; int th0, th1;
        int ok0 = crawl_once(sp[k] - 30.0f, sp[k] + 40.0f, 0, &t0, &g0, &th0), ok1 = crawl_once(sp[k] - 30.0f, sp[k] + 40.0f, 1, &t1, &g1, &th1);
        printf("CRAWL %5.0f m  %-24s high range: %-34s Low Range: %s\n", sp[k], what[k],
               ok0 ? TextFormat("through in %.0f s (%d thuds)", t0, th0) : TextFormat("STUCK %.0f m in (%d thuds)", g0 - (sp[k] - 30.0f), th0),
               ok1 ? TextFormat("through in %.0f s (%d thuds)", t1, th1) : TextFormat("STUCK %.0f m in (%d thuds)", g1 - (sp[k] - 30.0f), th1));
        cleared_low += ok1; stuck_high += !ok0;
    }
    printf("CRAWL %d places: Low Range through %d; high range stuck at %d\n", n, cleared_low, stuck_high);
    if (RT.ck_fall_s > 0) {   /* part 2: back UP the waterfall's chute, from a standstill below it */
        float t0, g0, t1, g1; int th0, th1;
        W.earned[ACH_LOW_SLOW] = 0; int ok0 = crawl_once(RT.ck_fall_s + 30.0f, RT.ck_fall_s - 40.0f, 0, &t0, &g0, &th0), ls0 = W.earned[ACH_LOW_SLOW]; W.earned[ACH_LOW_SLOW] = 0;
        int ok1 = crawl_once(RT.ck_fall_s + 30.0f, RT.ck_fall_s - 40.0f, 1, &t1, &g1, &th1);
        printf("CRAWL up the waterfall: Low and slow earned in high range %s, in Low Range %s\n", ls0 ? "YES (wrong)" : "no (right)", W.earned[ACH_LOW_SLOW] ? "YES" : "NO");
        printf("CRAWL %5.0f m  %-24s high range: %-34s Low Range: %s\n", RT.ck_fall_s, "up the waterfall", ok0 ? TextFormat("through in %.0f s", t0) : TextFormat("STUCK %.0f m below the top", g0 - (RT.ck_fall_s - 40.0f)), ok1 ? TextFormat("through in %.0f s", t1) : TextFormat("STUCK %.0f m below the top", g1 - (RT.ck_fall_s - 40.0f)));
    }
    if (T.body_physics != 0) for (int g = 0; g < RT.ck_nmoss && g < 6; g++) {   /* part 2: a launch: flat out at a mossy fallen tree from 70 m back, high range */
        Car car; memset(&car, 0, sizeof(car)); car_setup(&car, anchors);
        float s0 = RT.ck_moss_s[g] - 70.0f, x, z; route_creek_point(&RT, s0, &x, &z); int i = (int)(s0 / RT_DS); float hd = atan2f(RT.ck_z[i + 1] - RT.ck_z[i], RT.ck_x[i + 1] - RT.ck_x[i]);
        car_reset(&car, v3(x, 0.75f, z), (float)(M_PI / 2) - hd); bp.init = 0;
        Run r; run_reset(&r, 0); r.state = RUN_RUNNING; float ms = 0, best_h = 0, best_d = 0, best_t = 0, hit_v = 0; int rolled = 0;
        for (int st = 0; st < 25 * 50; st++) {
            CarInput in; memset(&in, 0, sizeof(in));
            RtNear cn = route_creek_near(&RT, car.body.pos.x, car.body.pos.z);
            if (cn.s > RT.ck_moss_s[g] + 25.0f) break;
            creek_follow(&car, &in, RT.ck_moss_s[g] + 40.0f, 16.0f); in.throttle = 1; in.brake = 0;
            car_step(&car, &in); RtPhys ph; logs_break = 0; rt_physics(&car, &ph); rt_rules(&r, &car, &ph, &ms, 1);
            if (r.ev_thud > 0) { explore_thud(&car, &ph); r.ev_thud = 0; }
            if (fabsf(cn.s - RT.ck_moss_s[g]) < 3.0f && hit_v == 0) hit_v = vlen(car.body.vel);
            if (bp.last_air_t > best_t) { best_t = bp.last_air_t; best_h = bp.last_air_h; best_d = bp.last_air_d; }
            if (bp.rolled) rolled = 1;
        }
        printf("LAUNCH at the mossy fallen tree %.0f m down, flat out in high range: hit at %.0f km/h; %s%s\n", RT.ck_moss_s[g], hit_v * 3.6f,
               best_t > 0 ? TextFormat("in the air %.2f s, %.2f m up, %.1f m along", best_t, best_h, best_d) : "never off the ground", rolled ? "; ROLLED" : "");
    }
    if (W.camp_on) {   /* build D: the camp's trunk, driven up to from 30 m up the creek, with an empty tray */
        Car car; memset(&car, 0, sizeof(car)); car_setup(&car, anchors);
        float x, z; route_creek_point(&RT, RT.camp_s - 30.0f, &x, &z); int i = (int)((RT.camp_s - 30.0f) / RT_DS); float hd = atan2f(RT.ck_z[i + 1] - RT.ck_z[i], RT.ck_x[i + 1] - RT.ck_x[i]);
        car_reset(&car, v3(x, 0.75f, z), (float)(M_PI / 2) - hd); car_set_low(&car, 1);
        W.tray_wood = 0; W.trunk = 0; Run r; run_reset(&r, 0); r.state = RUN_RUNNING; float ms = 0, t = 0; int got = 0;
        float hw = car_outline.own ? car_outline.half_w : T.body_half_w, fr = car_outline.own ? car_outline.front : T.body_front, re = car_outline.own ? car_outline.rear : T.body_rear;
        for (int st = 0; st < 60 * 50 && !got; st++) {
            CarInput in; memset(&in, 0, sizeof(in));
            V3 l = qrot(qconj(car.body.rot), v3(W.trunk_x - car.body.pos.x, 0, W.trunk_z - car.body.pos.z));
            in.steer = clampf(atan2f(l.x, l.z) * 2.4f, -1, 1); in.throttle = vlen(car.body.vel) < 2.5f ? 0.6f : 0.0f;
            car_step(&car, &in); RtPhys ph; logs_break = 0; rt_physics(&car, &ph); rt_rules(&r, &car, &ph, &ms, 1);
            if (r.ev_thud > 0) { explore_thud(&car, &ph); r.ev_thud = 0; }
            got = world_trunk_open(car.body.pos.x, car.body.pos.z, car_yaw(&car), hw, fr, re) == 1; t = (float)st * FIXED_DT;
        }
        printf("CRAWL the camp's trunk, driven up to from 30 m up the creek: %s\n", got ? TextFormat("opened after %.0f s, the tray full (%d logs)", t, W.tray_wood) : "NOT REACHED");
    }
    return cleared_low == n ? 0 : 1;
}

/* Build D, the hidden-way test (--drivetest with BOT_HIDDEN=1, Explorer). Only the gap should lead into the creek way.
 * 1. A search on a 1 m grid from the car park over everything the ute fits (a 2.1 m box, nothing solid, not up a bank past
 *    the hold), with the gap's own cells closed: does any cell of the creek way past the bend become reachable?
 * 2. Real drives in Low Range: from a grid of start points round the thicket on both banks, aimed at points down the creek
 *    way, 40 s each, backing off and trying again when stopped. Each ends in the creek way through the gap, in it some other
 *    way (a failure), or held outside. */
static int in_gap(float x, float z) { RtNear c = route_creek_near(&RT, x, z); return c.s > RT.ck_exit_s - T.creek_wall_from - 16.0f && c.s < RT.ck_exit_s + 32.0f && c.dist < 3.6f; }
static int past_bend(float x, float z) { RtNear c = route_creek_near(&RT, x, z); return route_in_creek(&RT, x, z) && c.s > RT.ck_exit_s + 45.0f && c.dist < 25.0f; }
static int hidden_test(void)
{
    explore = 1; RT.explore = 1; W.mode = MODE_EXPLORE; route_logs_restore(&RT, NULL);
    const Route *R = &RT;
    float cx, cz; route_creek_point(R, R->ck_exit_s, &cx, &cz);
    float x0 = fminf(cx, R->start_x) - 260, z0 = fminf(cz, R->start_z) - 260, x1 = fmaxf(cx, R->start_x) + 260, z1 = fmaxf(cz, R->start_z) + 260;
    int nx = (int)(x1 - x0) + 1, nz = (int)(z1 - z0) + 1, results[2] = { 0, 0 };
    for (int pass = 0; pass < 2; pass++) {   /* 0: the gap closed, 1: open */
        unsigned char *st = calloc((size_t)nx * nz, 1); int *q = malloc(sizeof(int) * (size_t)nx * nz), qh = 0, qt = 0, open = 0, got = 0;
        int si = (int)(R->start_x - x0), sj = (int)(R->start_z - z0); q[qt++] = sj * nx + si; st[sj * nx + si] = 3;
        while (qh < qt) {
            int k = q[qh++], i = k % nx, j = k / nx;
            const int di[4] = { 1, -1, 0, 0 }, dj[4] = { 0, 0, 1, -1 };
            for (int d = 0; d < 4; d++) {
                int a = i + di[d], b = j + dj[d]; if (a < 0 || b < 0 || a >= nx || b >= nz) continue;
                int m = b * nx + a; if (st[m]) continue;
                float xx = x0 + (float)a + 0.5f, zz = z0 + (float)b + 0.5f;
                int blocked = route_tree_hit(R, xx, zz, 0, 1.05f, 1.05f, -1.05f) >= 0 || route_edge_at(R, xx, zz) > T.edge_stop - 0.5f || (pass == 0 && in_gap(xx, zz));
                if (!blocked && route_near(R, xx, zz).s > 600.0f) blocked = 1;   /* the search stays round the clearing (the road on is a long way round) */
                st[m] = blocked ? 2 : 3;
                if (!blocked) { q[qt++] = m; open++; if (past_bend(xx, zz)) got++; }
            }
        }
        results[pass] = got;
        printf("HIDDEN grid search from the car park, gap %s: %d square metres reachable, %d of them in the creek way past the bend\n", pass ? "open" : "closed", open, got);
        free(st); free(q);
    }
    /* the drives */
    int n = 0, via_gap = 0, round_side = 0, held = 0;
    for (int side = -1; side <= 1; side += 2)
        for (int ia = 0; ia < 4; ia++)
            for (int il = 0; il < 5; il++) {
                float s0 = R->ck_exit_s - 50.0f + 13.0f * (float)ia, lat = (float)side * (6.0f + 7.0f * (float)il);
                float hd = route_heading_at_creek_pub(s0), px, pz; route_creek_point(R, s0, &px, &pz);
                px -= sinf(hd) * lat; pz += cosf(hd) * lat;
                if (route_edge_at(R, px, pz) > -0.5f || route_tree_hit(R, px, pz, 0, 1.3f, 1.3f, -1.3f) >= 0) continue;   /* not open ground */
                for (int ih = 0; ih < 5; ih++) {
                    float ts = R->ck_exit_s + 40.0f + 40.0f * (float)ih, tl = (ih % 2 ? 1.0f : -1.0f) * 6.0f * (float)(ih / 2), tx, tz;
                    route_creek_point(R, ts, &tx, &tz); float th = route_heading_at_creek_pub(ts); tx -= sinf(th) * tl; tz += cosf(th) * tl;
                    Car car; memset(&car, 0, sizeof(car)); car_setup(&car, anchors); T.manual_gears = 0;
                    car_reset(&car, v3(px, 0.75f, pz), atan2f(tx - px, tz - pz)); car_set_low(&car, 1);
                    Run r; run_reset(&r, 0); r.state = RUN_RUNNING; float ms = 0; float back = 0, still = 0; int gap_seen = 0, inside = 0;
                    for (int st = 0; st < 40 * 50; st++) {
                        CarInput in; memset(&in, 0, sizeof(in));
                        V3 l = qrot(qconj(car.body.rot), v3(tx - car.body.pos.x, 0, tz - car.body.pos.z));
                        float ang = atan2f(l.x, l.z), v = vlen(car.body.vel);
                        if (back > 0) { back -= FIXED_DT; in.brake = 1; in.steer = -clampf(ang * 2.0f, -1, 1); }
                        else { in.steer = clampf(ang * 2.0f + 0.6f * sinf((float)st * 0.013f), -1, 1); in.throttle = 1; still = v < 0.5f ? still + FIXED_DT : 0; if (still > 1.5f) { back = 1.2f; still = 0; } }
                        car_step(&car, &in);
                        RtPhys ph; logs_break = 0; rt_physics(&car, &ph); rt_rules(&r, &car, &ph, &ms, 1);
                        if (r.ev_thud > 0) { explore_thud(&car, &ph); r.ev_thud = 0; }
                        if (in_gap(car.body.pos.x, car.body.pos.z)) gap_seen = 1;
                        if (past_bend(car.body.pos.x, car.body.pos.z)) { inside = 1; break; }
                    }
                    n++;
                    if (inside && gap_seen) via_gap++; else if (inside) { round_side++; printf("HIDDEN   got in round the side: from %.0f m, %+.0f m off the creek, aimed %.0f m down\n", s0, lat, ts); }
                    else held++;
                }
            }
    printf("HIDDEN %d drives in Low Range from round the thicket on both banks: %d into the creek way through the gap, %d round the side, %d held outside\n", n, via_gap, round_side, held);
    printf("HIDDEN %s\n", results[0] == 0 && round_side == 0 && results[1] > 0 ? "only the gap leads in" : "NOT HIDDEN: another way in");
    return results[0] == 0 && round_side == 0 ? 0 : 1;
}

static int drivetest(int rows)
{
    if (getenv("BOT_POSTS")) return posts_test();
    if (getenv("BOT_HIDDEN")) return hidden_test();
    if (getenv("BOT_CRAWL")) return crawl_test();
    if (getenv("BOT_FLAT")) return flat_test();
    if (getenv("BOT_GATETEST")) return gate_test();
    float alats[] = { 3, 4, 5, 6, 7, 8, 10, 12, 15, 20, 30 }; /* m/s2; gravel lives at the low end (session 2) */
    float look = getenv("BOT_LOOK") ? (float)atof(getenv("BOT_LOOK")) : 0.55f, target = getenv("BOT_TARGET") ? (float)atof(getenv("BOT_TARGET")) : 32.0f;
    printf("DRIVE road bot from the start pad, target %.0f km/h, look %.2f s ahead, automatic; road %.1f m wide\n", target * 3.6f, look, T.road_width);
    int ok_any = 0, n = (int)(sizeof(alats) / sizeof(alats[0]));
    if (getenv("BOT_ALATS")) { /* session 3b: just these limits, "3,4" */
        n = 0; for (char *q = getenv("BOT_ALATS"); *q && n < 11; ) { alats[n++] = strtof(q, &q); while (*q == ',' || *q == ' ') q++; }
    }
    if (getenv("BOT_BANK")) bot_banks = atoi(getenv("BOT_BANK"));
    if (getenv("BOT_DRIFT")) { bot_drift_deg = (float)atof(getenv("BOT_DRIFT")); printf("DRIVE the drift bot: %.0f deg in every bend%s\n", bot_drift_deg, getenv("BOT_HB") && atoi(getenv("BOT_HB")) ? ", handbrake starts" : ", brake-stab starts"); }
    bot_use_hb = getenv("BOT_HB") && atoi(getenv("BOT_HB"));
    if (getenv("BOT_LOGS")) { bot_deer = 1; bot_logs = 1; bot_ghost = 1; n = 1; printf("DRIVE the log bot: the road, and over every log within 20 m of it (trunks off)\n"); }
    if (getenv("BOT_DEER")) { bot_deer = 1; bot_ghost = 1; n = 1; W.found = 0; printf("DRIVE the deer bot: the road, and into every deer on the way (trunks off)\n"); }
    if (getenv("BOT_CREEK")) {   /* build C: Explorer, the creek way */
        explore = 1; RT.explore = 1; W.mode = MODE_EXPLORE; n = 1;
        int ok = creek_path();
        printf("DRIVE the creek bot (Explorer): from the car park across the clearing, through the way in and down Solomons Creek to the ford: a path %s (%d points)\n", ok ? "found" : "NOT found", bot_path_n);
        if (!ok) return 1;
        bot_creek = 1; bot_path_at = 0;
    }
    if (getenv("BOT_EXPLORE")) { explore = 1; RT.explore = 1; W.mode = MODE_EXPLORE; n = 1; bot_explore = atoi(getenv("BOT_EXPLORE")); explore_bot_setup(); }
    if (getenv("BOT_DONUT")) { bot_donut = atoi(getenv("BOT_DONUT")) < 0 ? -1 : 1; n = 1; printf("DRIVE donuts on the spot in the car park, full lock and throttle\n"); }
    if (getenv("BOT_NOBRAKE")) { bot_nobrake = atoi(getenv("BOT_NOBRAKE")); printf("DRIVE the brake pedal is never touched (the bot lifts instead)\n"); }
    if (getenv("BOT_SHORTCUT")) {   /* build B: the shortcut test, one run */
        bot_shortcut = !strcmp(getenv("BOT_SHORTCUT"), "beeline") ? 1 : !strcmp(getenv("BOT_SHORTCUT"), "gate") ? 3 : 2;
        if (getenv("BOT_SC_AHEAD")) bot_sc_ahead = (float)atof(getenv("BOT_SC_AHEAD"));
        if (getenv("BOT_SC_KMH")) bot_sc_speed = (float)atof(getenv("BOT_SC_KMH")) / 3.6f;
        bot_ghost = getenv("BOT_GHOST") && atoi(getenv("BOT_GHOST"));
        printf("DRIVE the shortcut bot: %s at %.0f km/h%s\n", bot_shortcut == 1 ? "straight for the bridge" : TextFormat("aiming %.0f m ahead on the road", bot_sc_ahead), bot_sc_speed * 3.6f, bot_ghost ? ", trunks off" : "");
        n = 1;
    }
    for (int k = 0; k < n; k++) {
        if (rows > 1) printf("--- %.0f m/s2\n", alats[k]);
        memset(hist, 0, sizeof(hist)); memset(hist_pts, 0, sizeof(hist_pts));
        RtRun r = rt_drive(alats[k], target, look, rows);
        if (getenv("BOT_HIST")) {
            printf("  drift angle, seconds (pot points):");
            for (int b = 0; b < 16; b++) if (hist[b] > 0) printf("  %d-%d%s %.0f (%.0f)", 2 * b, 2 * b + 2, b == 15 ? "+" : "", hist[b], hist_pts[b]);
            printf("\n");
        }
        const char *how = r.state == RUN_FINISHED ? "BRIDGE" : r.state == RUN_WALL_OVER ? rt_hit_what : r.state == RUN_WRONG_WAY_OVER ? "wrong way" : "timed out";
        printf("DRIVE corners at %4.1f m/s2: %-9s after %6.1f s, %5.0f m (end at km %.2f, %+.1f m off the middle)  mean %3.0f top %3.0f km/h  ditch %4.1f s  rough %4.1f s  score %7.0f best x%d%s\n",
               alats[k], how, r.time, r.dist, r.end_s / 1000, r.end_lat, r.mean * 3.6f, r.top * 3.6f, r.ditch_s, r.rough_s, r.score, r.best_x,
               r.state == RUN_WALL_OVER ? TextFormat("  (hit at %.0f km/h)", r.hit_speed * 3.6f) : "");
        if (bot_shortcut) printf("      shortcut test: furthest from the road %.0f m, checkpoints %d of %d, %.0f m off the road; ended %.0f m from the bridge's start\n", r.max_lat, r.cp, r.ncp, r.offroad_m, hypotf(car_end_x - RT.x[(int)(RT.bridge_s0 / RT_DS)], car_end_z - RT.z[(int)(RT.bridge_s0 / RT_DS)]));
        if (r.resets || r.wood) printf("      put back on the track %d times; firewood %d\n", r.resets, r.wood);
        if (saw_evn[1] + saw_evn[2] + saw_evn[3] + saw_evn[4] + saw_evn[5]) printf("      the saw: %d split in one pass, %d after more, %d not through, %d blunt, %d too slow; sharpened %d times\n", saw_evn[1], saw_evn[2], saw_evn[3], saw_evn[4], saw_evn[5], W.sharpened);
        if (bot_creek) printf("      the creek way: into it at %.1f s, at the ford at %.1f s: %s; %d thuds%s\n", r.creek_t0, r.creek_t1, r.creek_t1 > 0 ? TextFormat("%.0f s from the way in to the ford (%d:%02d)", r.creek_t1 - r.creek_t0, (int)((r.creek_t1 - r.creek_t0) / 60), (int)(r.creek_t1 - r.creek_t0) % 60) : "DID NOT GET THERE", r.thuds,
                              r.creek_t1 > 0 ? "" : TextFormat(" (ended %.0f m down the creek, %.1f m off it)", route_creek_near(&RT, car_end_x, car_end_z).s, route_creek_near(&RT, car_end_x, car_end_z).dist));
        if (r.deer || r.ach) {
            printf("      deer found %d (%d of %d on the device); achievements earned:", r.deer, world_found_count(), W.ndeer);
            for (int a = 0; a < ACH_N; a++) if ((r.ach >> a) & 1ull) printf(" [%s]", ACH[a].name);
            printf("\n");
        }
        if (getenv("BOT_ACH")) { memset(W.earned, 0, sizeof(W.earned)); W.found = 0; }   /* each run earns afresh */
        printf("      old rules on the same drive: score %7.0f best x%d;  new rules: pot lost %d times in the ditch, %d in the crash (%.0f points)\n",
               r.old_score, r.old_x, r.lost_ditch, r.lost_crash, r.lost_pts);
        if (r.state == RUN_FINISHED) ok_any = 1;
    }
    return ok_any ? 0 : 1;
}

/* ------------------------------------------------------- title and records */
/* Both screens are still pictures made of text only, drawn through the same three-buffer text
 * path as the readout. Nothing moves, so a frame here is a clear and a few dozen letters. */

enum { SCR_TITLE, SCR_RECORDS, SCR_ACH, SCR_GAME };
enum { MENU_DRIFT, MENU_EXPLORE, MENU_RECORDS, MENU_ACH, MENU_RESET, MENU_QUIT, MENU_N };   /* play test (22 Oct): Reset progress */   /* build C: Drift / Explore */
#if defined(__EMSCRIPTEN__) || defined(BD_PHONE)
#define MENU_SHOWN 5   /* the browser and phone builds: no Quit */
#else
#define MENU_SHOWN MENU_N
#endif
static const char *test_label; /* --label TEXT: shown in red under the version on the title screen */
static const Color menu_ink = { 24, 28, 32, 255 }, menu_red = { 225, 45, 45, 255 }, menu_dim = { 70, 90, 100, 255 };

static void text_centred(const char *t, int y, int size, Color c) { DrawText(t, (VIEW_W - MeasureText(t, size)) / 2, y, size, c); }
static void text_right(const char *t, int x_right, int y, int size, Color c) { DrawText(t, x_right - MeasureText(t, size), y, size, c); }

#define MENU_Y0 196
#define MENU_DY 34
static int reset_ask, reset_yes;   /* Reset progress: asked to make sure (reset_yes: Yes picked) */
static int ach_pct(void) { int got = 0; for (int a = 0; a < ACH_N; a++) got += W.earned[a]; return (got * 100 + ACH_N / 2) / ACH_N; }
static void draw_title(int sel)
{
    if (getenv("SHOT_RESETASK")) reset_ask = 1;   /* pictures: the are-you-sure */
    const char *item[MENU_N] = { "Drift", "Explore", "Records", "Achievements", "Reset progress", "Quit" };
    text_centred("BUSH DRIFT", 64, 60, menu_ink);
    DrawRectangle(92, 130, 456, 4, menu_red);
    text_centred(GAME_VERSION, 144, 20, menu_red);
    /* a test build says what it is (--label), so it can be told from the plain game */
    if (test_label) text_centred(TextFormat("TEST: %s", test_label), 170, 10, menu_red);
    for (int i = 0; i < MENU_SHOWN; i++) {
        int y = MENU_Y0 + MENU_DY * i, off = i <= MENU_EXPLORE && !rt_have;
        if (i == sel) { DrawRectangle(176, y - 5, 312, 36, menu_ink); DrawText(">", 188, y, 30, menu_red); }
        DrawText(item[i], 236, y, 30, i == sel ? RAYWHITE : (off ? menu_dim : menu_ink));
        if (i == MENU_DRIFT) DrawText("for a score", 372, y + 11, 10, i == sel ? (Color){ 200, 200, 200, 255 } : menu_dim);
        if (i == MENU_EXPLORE) DrawText("roam, no score", 372, y + 11, 10, i == sel ? (Color){ 200, 200, 200, 255 } : menu_dim);
        if (i == MENU_ACH) DrawText(TextFormat("%d%% complete", ach_pct()), 236 + MeasureText(item[i], 30) + 12, y + 11, 10, i == sel ? (Color){ 200, 200, 200, 255 } : menu_dim);   /* build E */
    }
    if (reset_ask) {   /* are you sure? */
        DrawRectangle(110, 170, 420, 150, (Color){ 20, 16, 30, 240 });
        text_centred("Reset all progress?", 184, 20, RAYWHITE);
        text_centred("The deer, the achievements and everything done in Explorer.", 214, 10, (Color){ 210, 205, 215, 255 });
        text_centred("Drift's records are kept.", 228, 10, (Color){ 210, 205, 215, 255 });
        for (int i = 0; i < 2; i++) {
            int x = i ? 330 : 170, on = (i == 0) == (reset_yes != 0);
            if (on) DrawRectangle(x, 258, 140, 34, (Color){ 255, 235, 120, 255 });
            const char *t = i ? "No" : "Yes, reset";
            DrawText(t, x + 70 - MeasureText(t, 20) / 2, 265, 20, on ? (Color){ 24, 28, 32, 255 } : RAYWHITE);
        }
        return;
    }
    text_centred(sel == MENU_EXPLORE ? "Free roaming, both ways along the road.  Firewood, deer and things hidden round the hut."
                                     : "Alexanders Hut to the Tantawangalo Creek bridge.  A tree ends the run; the ditch slows you.", 400, 10, menu_dim);
    text_centred(TextFormat("Deer found %d of %d", world_found_count(), W.ndeer ? W.ndeer : WORLD_DEER), 418, 10, menu_dim);
#if defined(BD_PHONE)
    DrawText("Tap Drive to start.  In the game: steer on the left; gas, brake and handbrake on the right", 20, VIEW_H - 22, 10, menu_dim);
#elif defined(__EMSCRIPTEN__)
    DrawText("Up/Down: move     Enter: choose     Esc: back     In the game: arrows drive, Space handbrake", 20, VIEW_H - 22, 10, menu_dim);
#else
    DrawText("D-pad: move     A: choose     B: back     Start: go", 20, VIEW_H - 22, 10, menu_dim);
#endif
}

/* "test" after a date: carried in from a test entry's own records (round 3's handling entries, 16 Oct) */
static void draw_date(const char *date, int test, int x, int y)
{
    DrawText(records_date_text(date), x, y, 20, menu_ink);
    if (test) DrawText("test", x + MeasureText(records_date_text(date), 20) + 6, y + 5, 10, menu_dim);
}

static void draw_records(void)
{
    text_centred("RECORDS", 22, 30, menu_ink);
    text_centred("Alexanders Hut to the bridge", 58, 20, menu_red);
    DrawText("score", 150, 92, 10, menu_dim); DrawText("distance", 236, 92, 10, menu_dim); DrawText("best x", 330, 92, 10, menu_dim);
    DrawText("date", 392, 92, 10, menu_dim); DrawText("time", 552, 92, 10, menu_dim);
    DrawRectangle(40, 104, 560, 2, menu_dim);
    for (int i = 0; i < EREC_MAX; i++) {
        int y = 112 + 25 * i;
        text_right(TextFormat("%d", i + 1), 70, y, 20, i < EREC.n ? menu_red : menu_dim);
        if (i >= EREC.n) { DrawText("-", 150, y, 20, menu_dim); continue; }
        const ERecEntry *e = &EREC.top[i];
        text_right(commas(e->score), 220, y, 20, menu_ink);
        text_right(TextFormat("%s m", commas((int)e->dist)), 316, y, 20, menu_ink);
        DrawText(e->mult > 0 ? TextFormat("x%d", e->mult) : "--", 330, y, 20, menu_ink);
        draw_date(e->date, e->test, 392, y);
        DrawText(e->seed ? TextFormat("%d:%04.1f", (int)(e->seed / 6000), (float)(e->seed % 6000) / 100.0f) : "--", 552, y + 5, 10, menu_dim);
    }
    DrawRectangle(40, 368, 560, 2, menu_dim);
    const ERecBest *b[3] = { &EREC.furthest, &EREC.combo, &EREC.mult };
    const char *name[3] = { "Furthest", "Best combo", "Highest x" };
    for (int i = 0; i < 3; i++) {
        int y = 376 + 24 * i;
        DrawText(name[i], 60, y, 20, menu_red);
        if (b[i]->value <= 0) { DrawText("--", 236, y, 20, menu_ink); continue; }
        DrawText(i == 0 ? TextFormat("%s m", commas((int)b[i]->value)) : i == 1 ? commas((int)b[i]->value) : TextFormat("x%d", (int)b[i]->value), 236, y, 20, menu_ink);
        draw_date(b[i]->date, b[i]->test, 392, y);
        
    }
    DrawText("B: back", 20, VIEW_H - 22, 10, menu_dim);
    text_right("time = to the bridge", VIEW_W - 20, VIEW_H - 22, 10, menu_dim);
}

/* Build E: the achievements screen. A scrolling list in sections (each with its count), filtered All / Drift / Explore /
 * Tracked / Done. Counted ones show a bar; hidden ones read ??? with a short clue; earned ones their name, what was done and
 * the date. Up to three can be tracked (OK, or a tap on the row): they show top right while driving, in the mode they count in. */
enum { AF_ALL, AF_DRIFT, AF_EXPLORE, AF_TRACKED, AF_DONE, AF_N };
static int ach_filter, ach_sel = -1; static float ach_scroll, ach_say_t; static const char *ach_say;
static const char *const SEC_NAME[SEC_N] = { "Drift", "The Hut", "The bush", "The creek", "Quayle", "Firewood" };
static const char *const AF_NAME[AF_N] = { "All", "Drift", "Explore", "Tracked", "Done" };
static int ach_tracked(int a) { for (int q = 0; q < 3; q++) if (W.tracked[q] == a + 1) return 1; return 0; }
static int ach_track_toggle(int a)   /* 1 tracked, 0 untracked, -1 full */
{
    for (int q = 0; q < 3; q++) if (W.tracked[q] == a + 1) { W.tracked[q] = 0; W.dirty = 1; return 0; }
    for (int q = 0; q < 3; q++) if (!W.tracked[q]) { W.tracked[q] = a + 1; W.dirty = 1; return 1; }
    return -1;
}
static int ach_pass(int a, int f)
{
    return f == AF_ALL || (f == AF_DRIFT && (ACH[a].modes & IN_DRIFT)) || (f == AF_EXPLORE && (ACH[a].modes & IN_EXPLORE)) || (f == AF_TRACKED && ach_tracked(a)) || (f == AF_DONE && W.earned[a]);
}
typedef struct { int a, sec, y, h; } AchRow;   /* a < 0: a section's heading */
#define ACH_Y0 98
#define ACH_Y1 444
static int ach_rows(AchRow *row, int *total)
{
    int n = 0, y = 0;
    for (int sec = 0; sec < SEC_N; sec++) {
        int cnt = 0; for (int a = 0; a < ACH_N; a++) if (ACH[a].section == sec && ach_pass(a, ach_filter)) cnt++;
        if (!cnt) continue;
        row[n++] = (AchRow){ -1, sec, y, 24 }; y += 24;
        for (int a = 0; a < ACH_N; a++) if (ACH[a].section == sec && ach_pass(a, ach_filter)) { row[n++] = (AchRow){ a, sec, y, 34 }; y += 34; }
    }
    *total = y;
    return n;
}
static void draw_achievements(void)
{
    int got = 0; for (int a = 0; a < ACH_N; a++) got += W.earned[a];
    int ntr = 0; for (int q = 0; q < 3; q++) ntr += W.tracked[q] != 0;
    text_centred("ACHIEVEMENTS", 12, 30, menu_ink);
    text_centred(TextFormat("%d of %d, %d%% complete     tracking %d of 3", got, ACH_N, ach_pct(), ntr), 46, 10, menu_red);
    for (int f = 0; f < AF_N; f++) {   /* the filters */
        int x = 40 + 114 * f, on = f == ach_filter;
        if (on) DrawRectangle(x, 62, 104, 24, menu_ink); else DrawRectangleLines(x, 62, 104, 24, menu_dim);
        DrawText(AF_NAME[f], x + 52 - MeasureText(AF_NAME[f], 20) / 2, 65, 20, on ? RAYWHITE : menu_ink);
    }
    AchRow row[ACH_N + SEC_N]; int total, n = ach_rows(row, &total);
    BeginScissorMode(0, ACH_Y0 - 4, VIEW_W, ACH_Y1 - ACH_Y0 + 6);
    const Color drift_c = { 190, 70, 50, 255 }, expl_c = { 50, 130, 70, 255 };
    if (n == 0) text_centred(ach_filter == AF_TRACKED ? "Nothing tracked yet: pick up to three on All (OK, or tap one)" : "None yet", ACH_Y0 + 40, 10, menu_dim);
    for (int k = 0; k < n; k++) {
        int y = ACH_Y0 + row[k].y - (int)ach_scroll;
        if (y + row[k].h < ACH_Y0 - 4 || y > ACH_Y1) continue;
        if (row[k].a < 0) {   /* the section and its count */
            int c = 0, t = 0; for (int a = 0; a < ACH_N; a++) if (ACH[a].section == row[k].sec) { t++; c += W.earned[a]; }
            DrawText(SEC_NAME[row[k].sec], 24, y + 4, 20, menu_red);
            DrawText(TextFormat("%d/%d", c, t), 30 + MeasureText(SEC_NAME[row[k].sec], 20), y + 10, 10, menu_red);
            DrawRectangle(24, y + 22, 592, 1, menu_dim);
            continue;
        }
        int a = row[k].a, tr = ach_tracked(a), hid = !W.earned[a] && !ACH[a].shown;
        if (k == ach_sel) DrawRectangle(20, y + 1, 600, 32, (Color){ 255, 255, 255, 90 });
        /* the track pin */
        if (tr) { DrawCircle(32, y + 12, 6, menu_red); DrawRectangle(31, y + 17, 2, 7, menu_red); }
        else DrawCircleLines(32, y + 12, 5, menu_dim);
        DrawText(hid ? "???" : ACH[a].name, 46, y + 4, 20, W.earned[a] ? menu_ink : menu_dim);
        const char *sub = hid ? (ACH[a].clue ? TextFormat("Clue: %s", ACH[a].clue) : "") : ACH[a].how;
        DrawText(sub, 56, y + 22, 10, hid ? (Color){ 120, 100, 140, 255 } : menu_dim);
        const char *tag = ACH[a].modes == IN_BOTH ? "DRIFT + EXPLORE" : ACH[a].modes == IN_DRIFT ? "DRIFT" : "EXPLORE";
        Color tc = ACH[a].modes == IN_BOTH ? menu_dim : ACH[a].modes == IN_DRIFT ? drift_c : expl_c;
        if (W.earned[a]) { const char *dt = records_date_text(W.date[a]); DrawText(dt, 612 - MeasureText(dt, 10), y + 6, 10, menu_red); DrawText(tag, 612 - MeasureText(tag, 10), y + 22, 10, tc); }
        else {
            float hv, nd;
            if (world_ach_progress(a, &hv, &nd) && nd > 0) {
                const char *pt = a == ACH_WET_FEET ? TextFormat("%.0f/%.0f m", hv, nd) : TextFormat("%s/%s", commas((int)hv), commas((int)nd));
                DrawText(pt, 612 - MeasureText(pt, 10), y + 4, 10, menu_ink);
                DrawRectangle(502, y + 16, 110, 5, (Color){ 0, 0, 0, 50 }); DrawRectangle(502, y + 16, (int)(110.0f * hv / nd), 5, menu_red);
                DrawText(tag, 612 - MeasureText(tag, 10), y + 24, 10, tc);
            } else DrawText(tag, 612 - MeasureText(tag, 10), y + 22, 10, tc);
        }
    }
    EndScissorMode();
    if (ach_scroll > 1) DrawTriangle((Vector2){ 628, ACH_Y0 }, (Vector2){ 622, ACH_Y0 + 8 }, (Vector2){ 634, ACH_Y0 + 8 }, menu_ink);
    if (ach_scroll < (float)(total - (ACH_Y1 - ACH_Y0)) - 1) DrawTriangle((Vector2){ 622, ACH_Y1 - 8 }, (Vector2){ 628, ACH_Y1 }, (Vector2){ 634, ACH_Y1 - 8 }, menu_ink);
    if (ach_say_t > 0 && ach_say) { int w = MeasureText(ach_say, 20); DrawRectangle(320 - w / 2 - 12, 230, w + 24, 34, (Color){ 20, 16, 30, 230 }); DrawText(ach_say, 320 - w / 2, 237, 20, RAYWHITE); }
#if defined(BD_PHONE)
    DrawText("Tap a filter, or an achievement to track it (up to 3).  Drag to scroll.  Back: the title", 20, VIEW_H - 16, 10, menu_dim);
#else
    DrawText("Up/Down: move   Left/Right: filter   OK: track (up to 3, shown while driving)   Back: the title", 20, VIEW_H - 16, 10, menu_dim);
#endif
}
/* one frame of the screen's input; returns 1 to leave it */
static int ach_input(int up, int down, int left, int right, int ok, int back, float dt, int tap, Vector2 tap_at, float drag, float wheel)
{
    if (back) return 1;
    if (ach_say_t > 0) ach_say_t -= dt;
    if (left || right) { ach_filter = (ach_filter + (right ? 1 : AF_N - 1)) % AF_N; ach_sel = -1; ach_scroll = 0; synth_blip2(5, 1.0f, 0.6f); }
    AchRow row[ACH_N + SEC_N]; int total, n = ach_rows(row, &total);
    float maxs = fmaxf((float)(total - (ACH_Y1 - ACH_Y0)), 0.0f);
    if (ach_sel < 0 || ach_sel >= n || row[ach_sel].a < 0) { ach_sel = -1; for (int k = 0; k < n; k++) if (row[k].a >= 0) { ach_sel = k; break; } }
    if (up && ach_sel >= 0) { for (int k = ach_sel - 1; k >= 0; k--) if (row[k].a >= 0) { ach_sel = k; break; } synth_blip2(5, 1.0f, 0.6f); }
    if (down && ach_sel >= 0) { for (int k = ach_sel + 1; k < n; k++) if (row[k].a >= 0) { ach_sel = k; break; } synth_blip2(5, 1.0f, 0.6f); }
    if ((up || down) && ach_sel >= 0) {   /* keep it in view (and its heading, at the top) */
        float top = (float)row[ach_sel].y - (ach_sel > 0 && row[ach_sel - 1].a < 0 ? 24.0f : 0.0f), bot = (float)(row[ach_sel].y + row[ach_sel].h);
        if (top < ach_scroll) ach_scroll = top;
        if (bot > ach_scroll + (float)(ACH_Y1 - ACH_Y0)) ach_scroll = bot - (float)(ACH_Y1 - ACH_Y0);
    }
    ach_scroll = clampf(ach_scroll - drag - wheel * 34.0f, 0.0f, maxs);
    int pick = -1;
    if (tap) {
        if (tap_at.y > 60 && tap_at.y < 88) { int f = (int)((tap_at.x - 40.0f) / 114.0f); if (tap_at.x >= 40 && f >= 0 && f < AF_N) { ach_filter = f; ach_sel = -1; ach_scroll = 0; synth_blip2(5, 1.0f, 0.6f); } return 0; }
        if (tap_at.y > ACH_Y0 && tap_at.y < ACH_Y1) for (int k = 0; k < n; k++) { float y = (float)(ACH_Y0 + row[k].y) - ach_scroll; if (row[k].a >= 0 && tap_at.y >= y && tap_at.y < y + (float)row[k].h) { ach_sel = k; pick = row[k].a; } }
    }
    if (ok && ach_sel >= 0) pick = row[ach_sel].a;
    if (pick >= 0) {
        int r = ach_track_toggle(pick);
        ach_say = r < 0 ? "Tracking three already: untrack one first" : r ? "Tracked: it shows while you drive" : "Not tracked"; ach_say_t = 1.4f;
        synth_blip2(r < 0 ? 10 : 5, r ? 1.2f : 0.8f, 0.6f);
        if (r >= 0) world_save(), web_storage_flush();
    }
    return 0;
}
/* build E: the tracked achievements, top right while driving: only those that count in this mode; one flashes as it moves */
static void draw_tracked(int mode_bits, float dt, int y0)
{
    static float last[3] = { -1, -1, -1 }, flash[3];
    int y = y0;
    for (int q = 0; q < 3; q++) {
        int a = W.tracked[q] - 1;
        if (a < 0 || a >= ACH_N || !(ACH[a].modes & mode_bits)) { last[q] = -1; continue; }
        float hv = 0, nd = 0; int counted = world_ach_progress(a, &hv, &nd);
        float v = W.earned[a] ? 1e9f : hv;
        if (last[q] >= 0 && v != last[q]) flash[q] = 1.5f;
        last[q] = v; if (flash[q] > 0) flash[q] -= dt;
        const char *nm = W.earned[a] || ACH[a].shown ? ACH[a].name : "???";
        const char *pr = W.earned[a] ? "DONE" : counted ? (a == ACH_WET_FEET ? TextFormat("%.0f/%.0f m", hv, nd) : TextFormat("%s/%s", commas((int)hv), commas((int)nd))) : "";
        int wn = MeasureText(nm, 10), wp = pr[0] ? MeasureText(pr, 10) + 8 : 0, w = wn + wp + 16;
        int on = flash[q] > 0 && ((int)(flash[q] * 6.0f) & 1);
        DrawRectangle(634 - w, y, w, 14, on ? (Color){ 255, 235, 120, 200 } : (Color){ 20, 16, 30, 120 });
        DrawRectangle(634 - w, y, 3, 14, W.earned[a] ? (Color){ 140, 220, 130, 255 } : (Color){ 255, 200, 90, 255 });
        DrawText(nm, 634 - w + 8, y + 2, 10, on ? (Color){ 24, 28, 32, 255 } : (Color){ 255, 248, 230, 230 });
        if (pr[0]) DrawText(pr, 634 - wp + 2, y + 2, 10, on ? (Color){ 24, 28, 32, 255 } : (Color){ 255, 235, 120, 255 });
        if (!W.earned[a] && !ACH[a].shown && ACH[a].clue) { y += 14; const char *cl = ACH[a].clue; int wc = MeasureText(cl, 10) + 12; DrawRectangle(634 - wc, y, wc, 12, (Color){ 20, 16, 30, 100 }); DrawText(cl, 634 - wc + 8, y + 1, 10, (Color){ 210, 200, 230, 220 }); }
        y += 16;
    }
}

/* Build B: the whole route from above, fitted in a box: the road, how far this run got (gold), the hut, the ford and
 * the bridge, and the deer found on this device (orange dots). Lines only, so it costs next to nothing. */
static int map_creek_all;   /* play test (22 Oct): the pause menu's map shows the creek, all of it, as just a creek */
static void draw_route_map(int bx, int by, int bw, int bh, float got_s, int car_dot, V3 car_at, int labels)
{
    float x0 = 1e9f, x1 = -1e9f, z0 = 1e9f, z1 = -1e9f;
    for (int i = 0; i < RT.n; i += 10) { x0 = fminf(x0, RT.x[i]); x1 = fmaxf(x1, RT.x[i]); z0 = fminf(z0, RT.z[i]); z1 = fmaxf(z1, RT.z[i]); }
    float sc = fminf((float)(bw - 8) / fmaxf(x1 - x0, 1.0f), (float)(bh - 8) / fmaxf(z1 - z0, 1.0f));
    float ox = (float)bx + 0.5f * ((float)bw - (x1 - x0) * sc), oy = (float)by + 0.5f * ((float)bh + (z1 - z0) * sc);
#define MAPX(x) (ox + ((x) - x0) * sc)
#define MAPY(z) (oy - ((z) - z0) * sc)
    DrawRectangle(bx, by, bw, bh, (Color){ 30, 26, 36, 190 });
    for (int i = 10; i < RT.n; i += 10) {
        Color c = (float)i * RT_DS <= got_s ? (Color){ 255, 235, 120, 255 } : (Color){ 170, 160, 140, 255 };
        DrawLineEx((Vector2){ MAPX(RT.x[i - 10]), MAPY(RT.z[i - 10]) }, (Vector2){ MAPX(RT.x[i]), MAPY(RT.z[i]) }, (float)i * RT_DS <= got_s ? 3.0f : 2.0f, c);
    }
    if (explore && (W.creek_seen || map_creek_all) && RT.ck_n > 1)   /* build C: the creek way, once you've been down it (or the whole creek, on the big map) */
        for (int i = 20; i < RT.ck_n; i += 20) if (map_creek_all || (float)i * RT_DS > RT.ck_exit_s - 40.0f) DrawLineEx((Vector2){ MAPX(RT.ck_x[i - 20]), MAPY(RT.ck_z[i - 20]) }, (Vector2){ MAPX(RT.ck_x[i]), MAPY(RT.ck_z[i]) }, 2.0f, (Color){ 110, 170, 230, 255 });
    for (int d = 0; d < W.ndeer; d++) if ((W.found >> d) & 1) DrawCircleV((Vector2){ MAPX(W.deer[d].x), MAPY(W.deer[d].z) }, 3.0f, (Color){ 255, 140, 40, 255 });
    if (world_extra_on())   /* build C follow-up: Quayle's five, rings till they're found */
        for (int d = WORLD_DEER; d < WORLD_DEER + W.nextra; d++) {
            Vector2 at = { MAPX(W.deer[d].x), MAPY(W.deer[d].z) };
            if ((W.found >> d) & 1) DrawCircleV(at, 3.0f, (Color){ 255, 140, 40, 255 }); else DrawRing(at, 3.0f, 5.0f, 0, 360, 12, (Color){ 255, 200, 90, (unsigned char)(170 + 85 * (((int)(GetTime() * 2)) & 1)) });
        }
    if (W.army_placed && W.army_found && W.quayle == 0) DrawCircleV((Vector2){ MAPX(W.army_x), MAPY(W.army_z) }, 3.0f, (Color){ 150, 210, 110, 255 });   /* Quayle */
    if (explore && W.fire_level > 0) DrawCircleV((Vector2){ MAPX(W.fire_x), MAPY(W.fire_z) }, 3.5f + (float)W.fire_level, (Color){ 255, 120, 40, 220 });   /* the fire */
    if (explore) {   /* build D: the stock route's markers passed (small tan diamonds), and its camp once reached */
        for (int m = 0; m < W.nmark; m++) if ((W.markers >> m) & 1) {
            Vector2 c = { MAPX(W.mark_x[m]), MAPY(W.mark_z[m]) };
            DrawTriangle((Vector2){ c.x, c.y - 3.5f }, (Vector2){ c.x - 3.0f, c.y }, (Vector2){ c.x + 3.0f, c.y }, (Color){ 230, 200, 140, 255 });
            DrawTriangle((Vector2){ c.x - 3.0f, c.y }, (Vector2){ c.x, c.y + 3.5f }, (Vector2){ c.x + 3.0f, c.y }, (Color){ 230, 200, 140, 255 });
        }
        if (W.camp_on && (W.camp_seen || W.earned[ACH_STOCK_ROUTE] || W.trunk)) DrawRectangleV((Vector2){ MAPX(W.camp_x) - 3.0f, MAPY(W.camp_z) - 3.0f }, (Vector2){ 6.0f, 6.0f }, (Color){ 200, 140, 90, 255 });
    }
    float fx, fz; route_point(&RT, RT.ford_s, &fx, &fz);
    DrawCircleV((Vector2){ MAPX(fx), MAPY(fz) }, 2.0f, (Color){ 120, 190, 255, 255 });
    if (car_dot) DrawCircleV((Vector2){ MAPX(car_at.x), MAPY(car_at.z) }, 2.5f, RAYWHITE);   /* (the hut and bridge dots went: play test, they hid the fire) */
    if (map_creek_all) {   /* play test: the big map's key, bottom left (only what's on it now) */
        typedef struct { int kind; Color c; const char *t; } KeyRow; KeyRow key[10]; int nk = 0;
        key[nk++] = (KeyRow){ 0, RAYWHITE, "the 79" };
        key[nk++] = (KeyRow){ 3, (Color){ 255, 235, 120, 255 }, "the road (gold: as far as this run got)" };
        key[nk++] = (KeyRow){ 3, (Color){ 110, 170, 230, 255 }, "the creek" };
        key[nk++] = (KeyRow){ 0, (Color){ 255, 140, 40, 255 }, "a deer found" };
        if (world_extra_on()) key[nk++] = (KeyRow){ 1, (Color){ 255, 200, 90, 255 }, "a deer Quayle saw (still to find)" };
        if (W.army_placed && W.army_found && W.quayle == 0) key[nk++] = (KeyRow){ 0, (Color){ 150, 210, 110, 255 }, "Quayle" };
        if (explore && W.fire_level > 0) key[nk++] = (KeyRow){ 2, (Color){ 255, 120, 40, 200 }, "the campfire" };
        if (explore && W.markers) key[nk++] = (KeyRow){ 4, (Color){ 230, 200, 140, 255 }, "an old stock route marker" };
        if (explore && W.camp_on && (W.camp_seen || W.earned[ACH_STOCK_ROUTE] || W.trunk) && nk < 10) key[nk++] = (KeyRow){ 5, (Color){ 200, 140, 90, 255 }, "the cattleman's camp" };
        int kw = 0;
        for (int k = 0; k < nk; k++) kw = MeasureText(key[k].t, 10) > kw ? MeasureText(key[k].t, 10) : kw;
        int kx = bx + bw - kw - 38, ky = by + 14;   /* top right: the route never goes there (it runs west then south-east) */
        DrawRectangle(kx - 4, ky - 6, kw + 34, 14 * nk + 6, (Color){ 20, 16, 30, 220 });
        for (int k = 0; k < nk; k++) {
            Vector2 at = { (float)kx + 8, (float)(ky + 14 * k + 1) };
            if (key[k].kind == 0) DrawCircleV(at, 3.0f, key[k].c);
            else if (key[k].kind == 1) DrawRing(at, 3.0f, 5.0f, 0, 360, 12, key[k].c);
            else if (key[k].kind == 2) DrawCircleV(at, 4.5f, key[k].c);
            else if (key[k].kind == 4) { DrawTriangle((Vector2){ at.x, at.y - 3.5f }, (Vector2){ at.x - 3.0f, at.y }, (Vector2){ at.x + 3.0f, at.y }, key[k].c); DrawTriangle((Vector2){ at.x - 3.0f, at.y }, (Vector2){ at.x, at.y + 3.5f }, (Vector2){ at.x + 3.0f, at.y }, key[k].c); }
            else if (key[k].kind == 5) DrawRectangleV((Vector2){ at.x - 3.0f, at.y - 3.0f }, (Vector2){ 6.0f, 6.0f }, key[k].c);
            else DrawLineEx((Vector2){ at.x - 7, at.y }, (Vector2){ at.x + 7, at.y }, 2.5f, key[k].c);
            DrawText(key[k].t, kx + 22, ky + 14 * k - 4, 10, (Color){ 230, 225, 235, 255 });
        }
    }
    if (labels) {
        DrawText("hut", (int)MAPX(RT.x[0]) + 5, (int)MAPY(RT.z[0]) - 4, 10, (Color){ 200, 230, 200, 255 });
        DrawText("ford", (int)MAPX(fx) - 12, (int)MAPY(fz) + 4, 10, (Color){ 180, 210, 255, 255 });
        DrawText("bridge", (int)MAPX(RT.x[RT.n - 1]) - 40, (int)MAPY(RT.z[RT.n - 1]) + 4, 10, (Color){ 255, 190, 170, 255 });
    }
#undef MAPX
#undef MAPY
}

/* Laptops report touchpads and wireless keyboards as joysticks. Order the devices:
 * names that look like a controller first, then unknown ones, then the plainly wrong
 * ones. The first is used; F3 moves on to the next. */
static int pad_skip;
static int pad_rank(int i)
{
    const char *n = GetGamepadName(i);
    if (!n) return 2;
    static const char *good[] = { "8BitDo", "8Bitdo", "SN30", "Xbox", "X-Box", "XBOX", "Controller", "Gamepad", "Nintendo", "DualS", "Joy" };
    static const char *bad[] = { "ouchpad", "eyboard", "ouse", "rackPoint", "TrackPad", "Consumer", "System Control", "Ducky" };
    for (unsigned k = 0; k < sizeof(good) / sizeof(good[0]); k++) if (strstr(n, good[k])) return 0;
    for (unsigned k = 0; k < sizeof(bad) / sizeof(bad[0]); k++) if (strstr(n, bad[k])) return 2;
    return 1;
}
static int pick_pad(void)
{
    int found[8], n = 0;
    for (int rank = 0; rank <= 2; rank++)
        for (int i = 0; i < 8; i++)
            if (IsGamepadAvailable(i) && pad_rank(i) == rank) found[n++] = i;
    if (!n) return -1;
    int pick = found[pad_skip % n];
    return (pad_skip % n == 0 && pad_rank(pick) == 2) ? -1 : pick; /* never default to a touchpad or keyboard */
}

/* ------------------------------------------------------------- the look */
/* The 3D scene is drawn small (480 to 160 pixels wide) and blown up with hard pixels. Session 3: a sky with clouds
 * is drawn first; every scene mesh then goes through one shader that lights it (sun or shade, from a shadow map round
 * the car or, further out, the baked dapple), adds drawn detail to the gravel and the ground, and fades it into
 * layered golden haze; the blow-up goes through another that snaps each pixel to a fixed warm palette (with an
 * ordered dither). */
#define NPIX 5
static const int pix_w[NPIX] = { 480, 400, 320, 240, 160 };
static RenderTexture2D low_rt;
static int low_mode = -1;
static Shader pal_sh, sky_sh;
static int loc_cam, loc_haze, loc_hcol, loc_pal, loc_low, loc_dith;
static int loc_hcol2, loc_sun, loc_lvp, loc_smap, loc_shon, loc_woff, loc_rdet, loc_roadw;
static int sk_res, sk_f, sk_r, sk_u, sk_tan, sk_sun, sk_camw, sk_clouds, sk_haze, sk_night, sk_moon;
static int loc_night, loc_fire, loc_flash, loc_flashc, loc_headp, loc_headd, loc_barp, loc_bard, loc_sink, loc_lamp, loc_rear;   /* build C */
static Mesh ridge_mesh;
#define NPAL 64
static const unsigned char palette[NPAL][3] = {
    { 246, 217, 160 }, { 242, 184, 128 }, { 232, 160, 110 }, { 201, 169, 166 }, { 166, 165, 184 }, { 142, 155, 181 }, { 111, 134, 168 }, /* sky and haze */
    { 46, 59, 42 }, { 62, 82, 51 }, { 85, 107, 58 }, { 116, 133, 63 }, { 154, 160, 76 }, { 194, 179, 90 }, { 138, 143, 94 },        /* bush */
    { 90, 70, 50 }, { 126, 99, 69 }, { 163, 132, 92 }, { 201, 167, 122 }, { 227, 201, 154 },                                       /* dirt */
    { 217, 208, 192 }, { 179, 168, 148 }, { 140, 130, 115 },                                                                      /* trunks, stone */
    { 42, 38, 53 }, { 63, 58, 76 }, { 87, 80, 102 },                                                                              /* shade */
    { 240, 240, 236 }, { 168, 173, 179 }, { 22, 22, 26 }, { 200, 55, 45 }, { 247, 226, 122 },                                    /* car, lamps */
    { 94, 124, 142 }, { 147, 180, 191 },                                                                                          /* water */
    /* session 3 */
    { 138, 62, 38 }, { 168, 84, 48 }, { 108, 52, 36 },                                                                            /* rusty iron */
    { 34, 46, 32 }, { 52, 70, 40 }, { 128, 150, 120 },                                                                            /* deep forest, wattle silver */
    { 186, 196, 214 }, { 214, 200, 210 }, { 252, 238, 214 },                                                                      /* sky, clouds */
    { 110, 108, 104 }, { 198, 198, 194 },                                                                                         /* galvanised, slabs */
    { 60, 46, 36 }, { 150, 112, 80 },                                                                                             /* bark */
    { 244, 150, 90 }, { 70, 62, 80 }, { 156, 126, 140 },                                                                          /* sun glow, deep shade, mauve haze */
    { 88, 102, 90 }, { 120, 134, 118 },                                                                                           /* grey-green leaves (wattle, gully trees) */
    { 118, 122, 132 }, { 150, 154, 162 },                                                                                         /* the silver ute in shade */
    /* build C: night, fire and fireworks */
    { 6, 8, 16 }, { 14, 18, 34 }, { 26, 32, 56 }, { 42, 50, 82 }, { 66, 74, 112 },                                               /* night blues */
    { 255, 214, 120 }, { 255, 140, 40 }, { 180, 56, 22 },                                                                         /* firelight, embers */
    { 240, 80, 200 }, { 90, 230, 120 }, { 100, 200, 255 }, { 255, 250, 235 },                                                     /* fireworks */
};

/* GLSL written once, for desktop GL 3.3 or GL ES 2 (the phone build). */
#if defined(__EMSCRIPTEN__) || defined(BD_PHONE)   /* the browser (WebGL 2) and Android (GLES 3) builds: GLSL ES 3.00, to match raylib's own default shaders */
#define GLSL_VS "#version 300 es\nprecision highp float;\n#define IN in\n#define OUT out\n"
#define GLSL_FS "#version 300 es\nprecision highp float;\n#define IN in\n#define TEX texture\nout vec4 finalColor;\n#define FRAG finalColor\n"
#elif defined(GRAPHICS_API_OPENGL_ES2)
#define GLSL_VS "#version 100\nprecision highp float;\n#define IN attribute\n#define OUT varying\n"
#define GLSL_FS "#version 100\nprecision highp float;\n#define IN varying\n#define TEX texture2D\n#define FRAG gl_FragColor\n"
#else
#define GLSL_VS "#version 330\n#define IN in\n#define OUT out\n"
#define GLSL_FS "#version 330\n#define IN in\n#define TEX texture\nout vec4 finalColor;\n#define FRAG finalColor\n"
#endif
/* build C: a face's normal from the screen-space slope of its position (flat shading for the night lights); none on GL ES 2 */
#if defined(GRAPHICS_API_OPENGL_ES2) && !defined(__EMSCRIPTEN__) && !defined(BD_PHONE)
#define GLSL_NORMAL "#define NORMAL_OF(p) vec3(0.0, 1.0, 0.0)\n"
#else
#define GLSL_NORMAL "#define NORMAL_OF(p) normalize(cross(dFdx(p), dFdy(p)))\n"
#endif
#define GLSL_NOISE \
    "float h21(vec2 p){ p = fract(p*vec2(123.34, 456.21)); p += dot(p, p + 45.32); return fract(p.x*p.y); }\n" \
    "float vn(vec2 p){ vec2 i = floor(p), f = fract(p); f = f*f*(3.0 - 2.0*f);\n" \
    " return mix(mix(h21(i), h21(i + vec2(1.0, 0.0)), f.x), mix(h21(i + vec2(0.0, 1.0)), h21(i + vec2(1.0, 1.0)), f.x), f.y); }\n"

static const char *scene_vs = GLSL_VS
    "IN vec3 vertexPosition; IN vec2 vertexTexCoord; IN vec4 vertexColor;\n"
    "uniform mat4 mvp; uniform mat4 matModel; uniform mat4 lightVP; uniform float farSink;\n"
    "OUT vec4 fragColor; OUT vec3 wpos; OUT vec2 uv; OUT vec4 lpos;\n"
    "void main(){ vec3 vp = vertexPosition; if (vertexTexCoord.y > 2.5 && vertexTexCoord.y < 3.5) vp.y -= vertexTexCoord.x*farSink;   /* build C: the far land sinks along the creek way */\n"
    " vec4 wp = matModel*vec4(vp, 1.0); wpos = wp.xyz; uv = vertexTexCoord; fragColor = vertexColor;\n"
    " lpos = lightVP*wp; gl_Position = mvp*vec4(vp, 1.0); }\n";
static const char *scene_fs = GLSL_FS
    "IN vec4 fragColor; IN vec3 wpos; IN vec2 uv; IN vec4 lpos;\n"
    "uniform vec3 camPos; uniform vec4 haze; uniform vec3 hazeCol; uniform vec3 hazeSun; uniform vec3 sunDir; uniform vec4 colDiffuse;\n"
    "uniform sampler2D shadowMap; uniform vec4 shadowOn; uniform vec3 worldOff; uniform float roadDetail; uniform float roadW;\n"
    "uniform vec4 nightP; uniform vec4 fireP; uniform vec4 flashP; uniform vec3 flashC; uniform vec3 headP; uniform vec3 headD; uniform vec3 barP; uniform vec3 barD;\n"
    "uniform vec4 lampP; uniform vec4 rearP;\n"
    GLSL_NOISE
    GLSL_NORMAL
    "void main(){\n"
    " vec3 c = fragColor.rgb*colDiffuse.rgb;\n"
    " float a = floor(fragColor.a*255.0 + 0.5); float bit = mod(a, 2.0); float s = floor(a/2.0)/127.0;\n"
    " vec3 w = wpos + worldOff;\n"
    " if (uv.y > 50.0 && roadDetail > 0.5) {   /* the gravel: corrugations, ruts, stones, potholes and puddles */\n"
    "  float rs = uv.x, lat = uv.y - 100.0, al = abs(lat);\n"
    "  float corr = smoothstep(0.45, 0.8, vn(vec2(rs/38.0, 3.1))); float rip = sin(rs*8.727);\n"
    "  c *= 1.0 + 0.08*corr*sign(rip)*smoothstep(0.15, 0.75, abs(rip))*(1.0 - smoothstep(roadW - 0.9, roadW, al));\n"
    "  c *= 1.0 - 0.10*(1.0 - smoothstep(0.0, 0.18, abs(al - 0.8)));\n"
    "  vec2 g = vec2(rs, lat)/0.32; vec2 gi = floor(g); float edge = smoothstep(1.2, roadW, al);\n"
    "  if (h21(gi) > 0.95 - 0.10*edge) { vec2 f = fract(g) - 0.5; if (dot(f, f) < 0.1) c = mix(c, h21(gi + 7.0) > 0.45 ? vec3(0.86, 0.81, 0.72) : vec3(0.42, 0.38, 0.33), 0.75); }\n"
    "  vec2 hp = vec2(rs/2.4, lat/1.5 + 40.0); float ph = vn(hp)*(0.55 + 0.45*vn(vec2(rs/90.0, 7.0)));\n"
    "  float hole = smoothstep(0.84, 0.87, ph)*(1.0 - smoothstep(roadW - 0.7, roadW - 0.2, al));\n"
    "  if (hole > 0.0) { float puddle = step(0.74, vn(vec2(rs/60.0, 1.7)));\n"
    "   vec3 hc2 = puddle > 0.5 ? mix(vec3(0.40, 0.38, 0.37), vec3(0.78, 0.72, 0.64), step(0.92, vn(hp*3.0))) : c*0.66;\n"
    "   c = mix(c, hc2, hole); if (puddle > 0.5) { s *= 1.0 - hole; } }\n"
    " } else if (uv.y > 0.5 && uv.y < 1.5) {   /* ground: grass and litter mottled finer than the 5 m cells */\n"
    "  c *= 0.90 + 0.20*vn(w.xz/2.3) + 0.08*(h21(floor(w.xz/0.5)) - 0.5);\n"
    " } else if (uv.y > 2.5 && uv.y < 3.5) {   /* far land: crowns */\n"
    "  c *= 0.78 + 0.44*vn(w.xz/9.0);\n"
    " }\n"
    " float vis = bit;\n"
    " if (shadowOn.x > 0.5) {\n"
    "  vec3 pr = lpos.xyz/lpos.w*0.5 + 0.5; float edge = max(abs(pr.x - 0.5), abs(pr.y - 0.5))*2.0;\n"
    "  if (edge < 1.0 && pr.z < 1.0) { float t = shadowOn.z, b = shadowOn.y, sv = 0.0;\n"
    "   sv += step(pr.z - b, TEX(shadowMap, pr.xy + vec2(-0.5, -0.5)*t).r); sv += step(pr.z - b, TEX(shadowMap, pr.xy + vec2(0.5, -0.5)*t).r);\n"
    "   sv += step(pr.z - b, TEX(shadowMap, pr.xy + vec2(-0.5, 0.5)*t).r); sv += step(pr.z - b, TEX(shadowMap, pr.xy + vec2(0.5, 0.5)*t).r);\n"
    "   vis = mix(sv*0.25, bit, smoothstep(0.78, 0.98, edge)); }\n"
    " }\n"
    " vec3 lit = c*mix(vec3(0.95, 0.97, 1.05)*(1.0 - 0.55*s), vec3(1.0), vis);\n"
    " if (nightP.x > 0.001) {   /* build C: dusk and night. The sun goes; what's left is a little sky light, the fire, the ute's lights */\n"
    "  float k = nightP.x; vec3 sh = c*(1.0 - 0.55*s);\n"
    "  vec3 dusk = sh*vec3(0.60, 0.47, 0.58) + lit*0.18, dark = sh*vec3(0.075, 0.09, 0.17);\n"
    "  vec3 base = k < 0.5 ? mix(lit, dusk, k*2.0) : mix(dusk, dark, (k - 0.5)*2.0);\n"
    "  if (a > 250.5) base = c;   /* lamps glow */\n"
    "  vec3 nrm = NORMAL_OF(wpos); vec3 add = vec3(0.0); float lk = smoothstep(0.15, 0.6, k);\n"
    "  if (fireP.w > 0.0) { vec3 L = fireP.xyz - wpos; float d2 = dot(L, L); float f = fireP.w*(0.82 + 0.1*sin(nightP.y*13.0) + 0.08*sin(nightP.y*31.0 + 1.3));\n"
    "   float ndl = 0.3 + 0.7*max(dot(nrm, L*inversesqrt(d2 + 0.01)), 0.0); add += vec3(1.0, 0.52, 0.22)*f*ndl/(1.0 + d2/(fireP.w*fireP.w*30.0)); }\n"
    "  if (nightP.z > 0.5) { vec3 L = wpos - headP; float dh = length(L) + 0.001; vec3 l = L/dh;\n"
    "   float cone = smoothstep(0.88, 0.975, dot(l, headD)), att = (1.0 - smoothstep(48.0, 70.0, dh))/(1.0 + dh*dh/220.0);\n"
    "   add += vec3(1.0, 0.94, 0.80)*3.4*cone*att*(0.35 + 0.65*max(-dot(nrm, l), 0.0));\n"
    "   vec3 L2 = wpos - barP; float db = length(L2) + 0.001; vec3 l2 = L2/db;\n"
    "   float cone2 = smoothstep(0.60, 0.82, dot(l2, barD)), att2 = (1.0 - smoothstep(26.0, 40.0, db))/(1.0 + db*db/120.0);\n"
    "   add += vec3(0.92, 0.95, 1.0)*1.6*cone2*att2*(0.35 + 0.65*max(-dot(nrm, l2), 0.0)); }\n"
    "  if (flashP.w > 0.0) { vec3 L = flashP.xyz - wpos; add += flashC*flashP.w/(1.0 + dot(L, L)/2500.0); }\n"
    "  lit = base + c*add*lk;\n"
    " }\n"
    " if (lampP.w > 0.0 || rearP.w > 0.0) {   /* play test: the lantern on the picnic table, and Drift Yodel's fire out the back */\n"
    "  vec3 nrm = NORMAL_OF(wpos); vec3 add = vec3(0.0);\n"
    "  if (lampP.w > 0.0) { vec3 L = lampP.xyz - wpos; float d2 = dot(L, L);\n"
    "   add += vec3(1.0, 0.84, 0.55)*lampP.w*(0.35 + 0.65*max(dot(nrm, L*inversesqrt(d2 + 0.01)), 0.0))/(1.0 + d2/5.0); }\n"
    "  if (rearP.w > 0.0) { vec3 L = rearP.xyz - wpos; float d2 = dot(L, L);\n"
    "   add += vec3(1.0, 0.50, 0.18)*rearP.w*(0.3 + 0.7*max(dot(nrm, L*inversesqrt(d2 + 0.01)), 0.0))/(1.0 + d2/14.0); }\n"
    "  lit += c*add;\n"
    " }\n"
    " vec3 dv = wpos - camPos; float d = length(dv);\n"
    " float fog = 1.0 - exp(-max(d - haze.x, 0.0)/haze.y);\n"
    " fog = min(fog*mix(1.0, 1.45, clamp((haze.w - w.y)/160.0, 0.0, 1.0)), haze.z);   /* thicker in the valleys */\n"
    " vec3 hc = mix(hazeCol, hazeSun, pow(max(dot(dv/max(d, 0.001), sunDir), 0.0), 4.0));\n"
    " FRAG = vec4(mix(lit, hc, fog), 1.0); }\n";
static const char *sky_fs = GLSL_FS
    "IN vec2 fragTexCoord; IN vec4 fragColor;\n"
    "uniform vec2 res; uniform vec3 camF; uniform vec3 camR; uniform vec3 camU; uniform vec2 tanA; uniform vec3 sunDir; uniform vec3 camW;\n"
    "uniform float clouds; uniform vec3 hazeCol; uniform float night; uniform vec3 moonDir;\n"
    GLSL_NOISE
    "void main(){\n"
    " vec2 ndc = gl_FragCoord.xy/res*2.0 - 1.0; vec3 d = normalize(camF + camR*ndc.x*tanA.x + camU*ndc.y*tanA.y); float h = d.y;\n"
    " vec3 c = mix(vec3(0.95, 0.80, 0.60), vec3(0.80, 0.74, 0.74), smoothstep(-0.02, 0.18, h)); c = mix(c, vec3(0.47, 0.55, 0.70), smoothstep(0.15, 0.75, h));\n"
    " float sd = max(dot(d, sunDir), 0.0);\n"
    " c += vec3(1.0, 0.70, 0.40)*pow(sd, 6.0)*0.35; c = mix(c, vec3(1.0, 0.95, 0.80), smoothstep(0.9975, 0.999, sd)*(1.0 - smoothstep(0.2, 0.45, night)));\n"
    " if (night > 0.001) {   /* build C: dusk (a red-orange horizon under violet) then night (blue-black, stars) */\n"
    "  vec3 dusk = mix(vec3(0.92, 0.48, 0.30), vec3(0.42, 0.28, 0.44), smoothstep(-0.02, 0.2, h)); dusk = mix(dusk, vec3(0.13, 0.13, 0.28), smoothstep(0.18, 0.7, h)); dusk += vec3(0.6, 0.3, 0.1)*pow(sd, 3.0)*0.5;\n"
    "  vec3 dark = mix(vec3(0.07, 0.08, 0.15), vec3(0.012, 0.018, 0.05), smoothstep(0.0, 0.5, h));\n"
    "  c = night < 0.5 ? mix(c, dusk, night*2.0) : mix(dusk, dark, (night - 0.5)*2.0);\n"
    "  if (h > 0.03) { vec2 g = floor(d.xz/(h + 0.3)*220.0); float st = h21(g); float tw = 0.75 + 0.25*sin(st*500.0 + camW.x*0.0);\n"
    "   if (st > 0.9955) c = mix(c, vec3(0.88, 0.9, 1.0)*tw, smoothstep(0.45, 0.9, night)*smoothstep(0.03, 0.12, h)); }\n"
    " }\n"
    " if (night > 0.45) {   /* play test: a smiling full moon */\n"
    "  vec3 md = moonDir;   /* placed so the finale's fireworks shot has it in the sky above them (moon_dir) */\n"
    "  vec3 mr = normalize(cross(md, vec3(0.0, 1.0, 0.0))), mu = cross(mr, md);\n"
    "  float R = 0.035; vec2 q = vec2(dot(d, mr), dot(d, mu))/R; float md2 = dot(d, md), rr = length(q);\n"
    "  float on = smoothstep(0.45, 0.8, night);\n"
    "  if (md2 > 0.0) {\n"
    "   c += vec3(0.55, 0.56, 0.58)*on*0.35*exp(-max(rr - 1.0, 0.0)*1.3)*step(1.0, rr);   /* its glow */\n"
    "   if (rr < 1.0) {\n"
    "    vec3 m = vec3(0.96, 0.96, 0.94)*(0.86 + 0.12*q.y - 0.06*q.x) + vec3(0.06)*(1.0 - rr*rr);   /* pale grey-white, lit from the top left */\n"
    "    float cr = 0.0;   /* soft craters */\n"
    "    cr += 1.0 - smoothstep(0.15, 0.19, length(q - vec2(0.32, 0.62))); cr += 1.0 - smoothstep(0.09, 0.12, length(q - vec2(-0.58, 0.40)));\n"
    "    cr += 1.0 - smoothstep(0.08, 0.11, length(q - vec2(0.66, -0.12))); cr += 1.0 - smoothstep(0.11, 0.14, length(q - vec2(-0.34, -0.74)));\n"
    "    cr += 1.0 - smoothstep(0.06, 0.08, length(q - vec2(0.05, 0.86))); cr += 1.0 - smoothstep(0.07, 0.09, length(q - vec2(0.55, 0.30)));\n"
    "    m *= 1.0 - 0.09*clamp(cr, 0.0, 1.0);\n"
    "    for (int e = 0; e < 2; e++) {   /* the eyes: white, a dark iris, a glint, a lid line over them */\n"
    "     vec2 ec = vec2(e == 0 ? -0.35 : 0.35, 0.10); vec2 eq = (q - ec)/vec2(0.18, 0.16);\n"
    "     float lid = length((q - ec - vec2(0.0, 0.02))/vec2(0.21, 0.20));\n"
    "     if (lid < 1.0 && lid > 0.86 && q.y > ec.y + 0.05) m = vec3(0.55, 0.54, 0.53);\n"
    "     if (dot(eq, eq) < 1.0) { m = vec3(0.97, 0.97, 0.96);\n"
    "      vec2 iq = q - ec - vec2(0.02, -0.02); if (dot(iq, iq) < 0.011) m = vec3(0.13, 0.11, 0.11);\n"
    "      vec2 gq = q - ec - vec2(0.06, 0.03); if (dot(gq, gq) < 0.0012) m = vec3(1.0); }\n"
    "    }\n"
    "    vec2 nq = (q - vec2(0.0, -0.20))/vec2(0.13, 0.08);   /* the nose: a little lit bump with a shadow under it */\n"
    "    if (dot(nq, nq) < 1.0) m *= nq.y > 0.0 ? 1.06 : 0.84;\n"
    "    float sm = length(q - vec2(0.0, 0.02));   /* a gentle smile */\n"
    "    if (abs(sm - 0.50) < 0.028 && q.y < -0.36 && abs(q.x) < 0.36) m = vec3(0.45, 0.43, 0.42);\n"
    "    m *= 0.86 + 0.14*smoothstep(1.0, 0.7, rr);   /* darker round the edge */\n"
    "    c = mix(c, m, on*smoothstep(1.0, 0.92, rr));\n"
    "   }\n"
    "  }\n"
    " }\n"
    " if (clouds > 0.5 && h > 0.0) {\n"
    "  vec2 p = (camW.xz + d.xz/max(h, 0.035)*900.0)/1500.0;\n"
    "  float n = 0.55*vn(p) + 0.28*vn(p*2.13 + 5.2) + 0.17*vn(p*4.7 + 1.3);\n"
    "  float cov = smoothstep(0.52, 0.74, n)*smoothstep(0.01, 0.10, h)*(1.0 - 0.6*smoothstep(0.5, 1.0, night));\n"
    "  vec3 cc = mix(vec3(0.78, 0.66, 0.72), vec3(1.0, 0.87, 0.70), smoothstep(0.58, 0.80, n)) + vec3(0.25, 0.15, 0.05)*pow(sd, 3.0);\n"
    "  cc = night < 0.5 ? mix(cc, vec3(0.70, 0.38, 0.40), night*2.0) : mix(vec3(0.70, 0.38, 0.40), vec3(0.05, 0.06, 0.11), (night - 0.5)*2.0);\n"
    "  c = mix(c, cc, cov*0.9);\n"
    " }\n"
    " c = mix(c, hazeCol, (1.0 - smoothstep(-0.02, 0.07, h))*0.7);\n"
    " FRAG = vec4(c, 1.0); }\n";
static const char *pal_fs = GLSL_FS
    "IN vec2 fragTexCoord; IN vec4 fragColor; uniform sampler2D texture0; uniform vec2 lowSize; uniform vec3 pal[64]; uniform float dith;\n"
    "float b2(vec2 p){ return 2.0*p.x + 3.0*p.y - 4.0*p.x*p.y; }\n"
    "void main(){ vec3 c = TEX(texture0, fragTexCoord).rgb; vec2 px = floor(fragTexCoord*lowSize);\n"
    " float b = 4.0*b2(mod(px, 2.0)) + b2(mod(floor(px/2.0), 2.0)); c += (b/16.0 - 0.47)*dith;\n"
    " vec3 best = pal[0]; float bd = 1e9; for (int i = 0; i < 64; i++){ vec3 d = c - pal[i]; float e = dot(d*d, vec3(0.32, 0.5, 0.18)); if (e < bd){ bd = e; best = pal[i]; } }\n"
    " FRAG = vec4(best, 1.0); }\n";

/* The shadow map: a depth-only target the scene's casters are drawn into from the sun each frame. */
static RenderTexture2D shadow_rt;
static int shadow_res_now;
static void shadow_init(int res)
{
    if (shadow_res_now == res) return;
    if (shadow_res_now) { rlUnloadTexture(shadow_rt.depth.id); rlUnloadFramebuffer(shadow_rt.id); }
    memset(&shadow_rt, 0, sizeof(shadow_rt));
    shadow_rt.id = rlLoadFramebuffer();
    shadow_rt.texture.width = shadow_rt.texture.height = res;
    rlEnableFramebuffer(shadow_rt.id);
    shadow_rt.depth.id = rlLoadTextureDepth(res, res, false);
    shadow_rt.depth.width = shadow_rt.depth.height = res; shadow_rt.depth.mipmaps = 1; shadow_rt.depth.format = 19;
    rlFramebufferAttach(shadow_rt.id, shadow_rt.depth.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
    if (!rlFramebufferComplete(shadow_rt.id)) fprintf(stderr, "shadow map: framebuffer not complete\n");
    rlDisableFramebuffer();
    shadow_res_now = res;
}

static void look_init(void)
{
    scene_mat = LoadMaterialDefault();
    Shader sh = LoadShaderFromMemory(scene_vs, scene_fs);
    if (sh.id > 0 && sh.id != scene_mat.shader.id) scene_mat.shader = sh;
    else fprintf(stderr, "scene shader not available\n");
    Shader *S = &scene_mat.shader;
    loc_cam = GetShaderLocation(*S, "camPos"); loc_haze = GetShaderLocation(*S, "haze"); loc_hcol = GetShaderLocation(*S, "hazeCol");
    loc_hcol2 = GetShaderLocation(*S, "hazeSun"); loc_sun = GetShaderLocation(*S, "sunDir"); loc_lvp = GetShaderLocation(*S, "lightVP");
    loc_smap = GetShaderLocation(*S, "shadowMap"); loc_shon = GetShaderLocation(*S, "shadowOn"); loc_woff = GetShaderLocation(*S, "worldOff");
    loc_rdet = GetShaderLocation(*S, "roadDetail"); loc_roadw = GetShaderLocation(*S, "roadW");
    loc_night = GetShaderLocation(*S, "nightP"); loc_fire = GetShaderLocation(*S, "fireP"); loc_flash = GetShaderLocation(*S, "flashP"); loc_flashc = GetShaderLocation(*S, "flashC");
    loc_headp = GetShaderLocation(*S, "headP"); loc_headd = GetShaderLocation(*S, "headD"); loc_barp = GetShaderLocation(*S, "barP"); loc_bard = GetShaderLocation(*S, "barD"); loc_sink = GetShaderLocation(*S, "farSink");
    loc_lamp = GetShaderLocation(*S, "lampP"); loc_rear = GetShaderLocation(*S, "rearP");
    sky_sh = LoadShaderFromMemory(NULL, sky_fs);
    sk_res = GetShaderLocation(sky_sh, "res"); sk_f = GetShaderLocation(sky_sh, "camF"); sk_r = GetShaderLocation(sky_sh, "camR"); sk_u = GetShaderLocation(sky_sh, "camU");
    sk_tan = GetShaderLocation(sky_sh, "tanA"); sk_sun = GetShaderLocation(sky_sh, "sunDir"); sk_camw = GetShaderLocation(sky_sh, "camW");
    sk_clouds = GetShaderLocation(sky_sh, "clouds"); sk_haze = GetShaderLocation(sky_sh, "hazeCol"); sk_night = GetShaderLocation(sky_sh, "night"); sk_moon = GetShaderLocation(sky_sh, "moonDir");
    pal_sh = LoadShaderFromMemory(NULL, pal_fs);
    loc_pal = GetShaderLocation(pal_sh, "pal");
    loc_low = GetShaderLocation(pal_sh, "lowSize");
    loc_dith = GetShaderLocation(pal_sh, "dith");
    float p[NPAL * 3];
    for (int i = 0; i < NPAL * 3; i++) p[i] = (float)palette[i / 3][i % 3] / 255.0f;
    SetShaderValueV(pal_sh, loc_pal, p, SHADER_UNIFORM_VEC3, NPAL);
    /* far ridges: two rings of ranges well beyond the far land, in the haze's colours, so the edge of the world is never bare */
    MeshBuilder b = { 0 };
    for (int layer = 0; layer < 2; layer++) {
        float R = layer ? 9000.0f : 6500.0f;
        Color c = layer ? (Color){ 200, 176, 172, 255 } : (Color){ 168, 156, 172, 255 };
        int N = 160;
        for (int i = 0; i < N; i++) {
            float a0 = (float)i * 6.2831853f / (float)N, a1 = (float)(i + 1) * 6.2831853f / (float)N;
#define RIDGE(a) (4.0f * (35.0f + 28.0f * sinf((a) * 3.0f + (float)layer * 1.7f) + 16.0f * sinf((a) * 7.0f + 0.5f) + 9.0f * sinf((a) * 13.0f + (float)layer) + (layer ? 30.0f : 0.0f)))
            V3 p0 = v3(R * sinf(a0), -900.0f, R * cosf(a0)), p1 = v3(R * sinf(a1), -900.0f, R * cosf(a1));
            V3 q0 = v3(p0.x, RIDGE(a0) - 250.0f, p0.z), q1 = v3(p1.x, RIDGE(a1) - 250.0f, p1.z);
#undef RIDGE
            mb_quad(&b, p0, p1, q1, q0, c, 1);
        }
    }
    ridge_mesh = mb_finish(&b);
}

static void look_set_mode(int mode)
{
    mode = ((mode % NPIX) + NPIX) % NPIX;
    if (mode == low_mode) return;
    if (low_mode >= 0) UnloadRenderTexture(low_rt);
    low_rt = LoadRenderTexture(pix_w[mode], pix_w[mode] * 3 / 4);
    SetTextureFilter(low_rt.texture, TEXTURE_FILTER_POINT);
    low_mode = mode;
    T.pixel_mode = (float)mode;
}

/* ------------------------------------------------------------------ main */

/* ------------------------------------------------------------ touch controls */
/* On-screen buttons for phones (session 3b; after raylib's core_input_virtual_controls and core_input_multitouch
 * examples, and Android's 48 dp touch-target guidance). The game's 4:3 picture sits in the middle of a wide phone
 * screen, so the buttons go in the bars either side: steering on the left, gas, brake and handbrake on the right,
 * pause and restart in the top corners. Any finger on a button holds it; several at once work. */
typedef struct { float x, y, r; int down, was; } TButton;
enum { TB_LEFT, TB_RIGHT, TB_GAS, TB_BRAKE, TB_HAND, TB_PAUSE, TB_RESTART, TB_BANK, TB_N };
static TButton tb[TB_N];
static int touch_on, touch_tap; static Vector2 touch_tap_at, touch_start; static int touch_prev_n; static float touch_moved;   /* build E: how far the finger has gone since it went down (a drag, not a tap) */
static Rectangle touch_game;   /* where the picture is on the screen */
static float safe_l, safe_r, safe_t, safe_b;   /* the screen edges to keep buttons off (iOS: the notch and the home bar) */

static void touch_update(void)
{
    float sw = (float)GetScreenWidth(), sh = (float)GetScreenHeight();
    float sc = minf(sw / VIEW_W, sh / VIEW_H);
    touch_game = (Rectangle){ (sw - VIEW_W * sc) * 0.5f, (sh - VIEW_H * sc) * 0.5f, VIEW_W * sc, VIEW_H * sc };
    /* button size from the screen height (about 9 mm on a phone), then each cluster centred in its side bar,
     * or pulled in from the edge far enough that its buttons don't overlap when the bars are narrow */
#ifdef BD_IOS
    bd_ios_safe_area(&safe_l, &safe_r, &safe_t, &safe_b);
#endif
    float r = minf(sh * 0.11f, sw * 0.075f);
    float bar_l = maxf(touch_game.x - safe_l, 0), bar_r = maxf(touch_game.x - safe_r, 0);   /* the side bars, less the safe-area insets */
    float cl = safe_l + maxf(bar_l * 0.5f, 2.4f * r), cr = sw - safe_r - maxf(bar_r * 0.5f, 2.6f * r);
    tb[TB_LEFT] = (TButton){ cl - 1.15f * r, sh * 0.74f, r, 0, tb[TB_LEFT].down };
    tb[TB_RIGHT] = (TButton){ cl + 1.15f * r, sh * 0.74f, r, 0, tb[TB_RIGHT].down };
    tb[TB_GAS] = (TButton){ cr + 1.0f * r, sh * 0.70f, r * 1.2f, 0, tb[TB_GAS].down };
    tb[TB_BRAKE] = (TButton){ cr - 1.35f * r, sh * 0.80f, r, 0, tb[TB_BRAKE].down };
    tb[TB_HAND] = (TButton){ cr + 0.2f * r, sh * 0.40f, r * 0.9f, 0, tb[TB_HAND].down };
    tb[TB_PAUSE] = (TButton){ r * 0.75f, r * 0.75f, r * 0.55f, 0, tb[TB_PAUSE].down };
    tb[TB_RESTART] = (TButton){ sw - r * 0.75f, r * 0.75f, r * 0.55f, 0, tb[TB_RESTART].down };
    tb[TB_BANK] = (TButton){ cl, sh * 0.42f, r * 0.85f, 0, tb[TB_BANK].down };   /* build A: above the steering, for a quick left-thumb tap */
    for (int i = 0; i < TB_N; i++) {   /* keep every button wholly on the screen */
        tb[i].x = clampf(tb[i].x, safe_l + tb[i].r * 1.05f, sw - safe_r - tb[i].r * 1.05f);
        tb[i].y = clampf(tb[i].y, safe_t + tb[i].r * 1.05f, sh - safe_b - tb[i].r * 1.05f);
    }
    int n = GetTouchPointCount();
    for (int k = 0; k < n; k++) {
        Vector2 p = GetTouchPosition(k);
        for (int i = 0; i < TB_N; i++) { float dx = p.x - tb[i].x, dy = p.y - tb[i].y; if (dx * dx + dy * dy < tb[i].r * tb[i].r * 1.4f) tb[i].down = 1; }
    }
    /* a tap: one finger down and up again without moving far (for the menus and the end of a run) */
    touch_tap = 0;
    if (n > 0 && touch_prev_n == 0) { touch_start = GetTouchPosition(0); touch_moved = 0; }
    if (n > 0) { Vector2 p0 = GetTouchPosition(0); touch_moved = fmaxf(touch_moved, hypotf(p0.x - touch_start.x, p0.y - touch_start.y)); }
    if (n == 0 && touch_prev_n > 0) { touch_tap = 1; touch_tap_at = touch_start; }
    touch_prev_n = n;
}
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
EMSCRIPTEN_KEEPALIVE void bd_touch_seen(void) { touch_on = 1; }   /* the page calls this on a real finger touch (not a mouse click) */
/* Build B: the browser keeps the records and the progress file in its own storage (IndexedDB), so they survive a reload */
static void web_storage_open(void)
{
    emscripten_run_script("try { FS.mkdir('/keep'); FS.mount(IDBFS, {}, '/keep'); Module.bdSynced = 0; FS.syncfs(true, function (e) { Module.bdSynced = 1; }); } catch (e) { Module.bdSynced = 1; }");
    for (int i = 0; i < 300 && !emscripten_run_script_int("Module.bdSynced ? 1 : 0"); i++) emscripten_sleep(10);
}
static void web_storage_flush(void) { emscripten_run_script("try { FS.syncfs(false, function (e) { }); } catch (e) { }"); }
#else
static void web_storage_flush(void) { }
#endif
static int tb_pressed(int i) { return tb[i].down && !tb[i].was; }
static Vector2 touch_to_view(Vector2 p) { return (Vector2){ (p.x - touch_game.x) / touch_game.width * VIEW_W, (p.y - touch_game.y) / touch_game.height * VIEW_H }; }

static int low_shown;   /* build D: Low Range is on (the LOW button lit) */
static void touch_draw(int in_game)
{
    if (!touch_on || !in_game) return;   /* the menus are tapped directly */
    for (int i = 0; i < TB_N; i++) {
        if (i == TB_BANK && T.score_bank_button == 0 && !explore) continue;   /* follow-up: no bank button (leaving the road banks); build D: Explorer's LOW */
        TButton *b = &tb[i];
        Color fill = b->down ? (Color){ 255, 255, 255, 110 } : (Color){ 255, 255, 255, 45 }, ink = (Color){ 255, 255, 255, 200 };
        if (i == TB_GAS) fill = b->down ? (Color){ 120, 220, 120, 140 } : (Color){ 120, 220, 120, 55 };
        if (i == TB_BRAKE || i == TB_HAND) fill = b->down ? (Color){ 240, 110, 90, 140 } : (Color){ 240, 110, 90, 55 };
        if (i == TB_BANK) fill = b->down ? (Color){ 255, 220, 90, 160 } : (Color){ 255, 220, 90, 60 };
        if (i == TB_BANK && explore) fill = b->down || low_shown ? (Color){ 255, 150, 60, 170 } : (Color){ 255, 150, 60, 55 };
        DrawCircleV((Vector2){ b->x, b->y }, b->r, fill);
        DrawRing((Vector2){ b->x, b->y }, b->r - 3, b->r, 0, 360, 48, ink);
        float a = b->r * 0.42f;
        if (i == TB_LEFT) DrawTriangle((Vector2){ b->x - a, b->y }, (Vector2){ b->x + a * 0.7f, b->y + a }, (Vector2){ b->x + a * 0.7f, b->y - a }, ink);
        else if (i == TB_RIGHT) DrawTriangle((Vector2){ b->x + a, b->y }, (Vector2){ b->x - a * 0.7f, b->y - a }, (Vector2){ b->x - a * 0.7f, b->y + a }, ink);
        else if (i == TB_PAUSE) { DrawRectangleRec((Rectangle){ b->x - a * 0.7f, b->y - a, a * 0.5f, a * 2 }, ink); DrawRectangleRec((Rectangle){ b->x + a * 0.2f, b->y - a, a * 0.5f, a * 2 }, ink); }
        else {
            const char *t = i == TB_GAS ? "GAS" : i == TB_BRAKE ? "BRAKE" : i == TB_HAND ? "HAND" : i == TB_BANK ? (explore ? "LOW" : "BANK") : "R";
            int fs = (int)(b->r * (i == TB_RESTART ? 0.8f : 0.36f));
            DrawText(t, (int)(b->x - MeasureText(t, fs) * 0.5f), (int)(b->y - fs * 0.5f), fs, ink);
        }
    }
}

/* Build C: Explorer's readout, top left: the tray, the esky, the deer, and the hints by the fire and the table. */
static void draw_explore_hud(const Car *c, int frame)
{
    const Color lab = { 255, 248, 230, 220 }, val = { 255, 235, 120, 255 };
    int sawrow = T.saw_on != 0;   /* build E: Will's chainsaw, a bar under the deer */
    int uterow = T.trials_on != 0;   /* build E: the ute's condition, under the saw */
    DrawRectangle(6, 6, 146, (W.rifle == 1 || W.tray_deer || W.quayle == 1 ? 58 : 46) + (sawrow ? 12 : 0) + (uterow ? 12 : 0), (Color){ 20, 16, 30, 110 });
    if (uterow) {
        int y = (W.rifle == 1 || W.tray_deer || W.quayle == 1 ? 58 : 46) + (sawrow ? 12 : 0);
        float worst = W.body_dmg; for (int i = 0; i < 4; i++) worst = fmaxf(worst, W.tyre_wear[i]);
        DrawText("UTE", 12, y, 10, (Color){ 255, 248, 230, 220 });
        if (W.punct) DrawText(W.spare ? "FLAT: CHANGE IT" : "FLAT, NO SPARE", 62, y, 10, (frame / 20) % 2 ? (Color){ 255, 110, 80, 255 } : (Color){ 255, 200, 160, 255 });
        else {
            Color uc = worst < 0.4f ? (Color){ 140, 220, 130, 255 } : worst < 0.7f ? (Color){ 255, 200, 90, 255 } : (Color){ 255, 110, 80, 255 };
            DrawRectangle(100, y + 2, 44, 6, (Color){ 60, 56, 70, 255 }); DrawRectangle(100, y + 2, (int)(44.0f * (1.0f - worst)), 6, uc);
            if (!W.spare) DrawText("no spare", 44, y, 10, (Color){ 255, 200, 160, 220 });
        }
    }
    if (sawrow) {
        int y = W.rifle == 1 || W.tray_deer || W.quayle == 1 ? 58 : 46;
        DrawText("SAW", 12, y, 10, lab);
        Color sc = W.saw >= 60 ? (Color){ 140, 220, 130, 255 } : W.saw >= 30 ? (Color){ 255, 200, 90, 255 } : (Color){ 255, 110, 80, 255 };
        DrawRectangle(100, y + 2, 44, 6, (Color){ 60, 56, 70, 255 }); DrawRectangle(100, y + 2, (int)(44.0f * W.saw / 100.0f), 6, sc);
        if (W.razor > 0) DrawText("*", 146, y - 1, 10, (Color){ 255, 240, 160, 255 });
    }
    DrawText("FIREWOOD", 12, 10, 10, lab); DrawText(TextFormat("%d/%d", W.tray_wood, (int)T.wood_full), 100, 10, 10, W.tray_wood >= (int)T.wood_full ? (Color){ 255, 170, 80, 255 } : val);
    DrawText("CHARCUTERIE", 12, 22, 10, lab); DrawText(W.board ? "on the table" : TextFormat("%d/%d", world_hamper_count(), HAMPER_N), 100, 22, 10, val);
    DrawText("DEER", 12, 34, 10, lab);
    if (world_extra_on()) { int nx = 0; for (int e = WORLD_DEER; e < WORLD_DEER + W.nextra; e++) nx += (W.found >> e) & 1; DrawText(TextFormat("%d/%d +%d/%d", world_found_count(), W.ndeer, nx, W.nextra), 100, 34, 10, val); }
    else DrawText(TextFormat("%d/%d", world_found_count(), W.ndeer), 100, 34, 10, val);
    if (W.rifle == 1) DrawText("QUAYLE'S RIFLE", 12, 46, 10, (Color){ 150, 210, 110, 255 });
    else if (W.tray_deer) DrawText("A DEER IN THE TRAY", 12, 46, 10, (Color){ 255, 170, 70, 255 });
    else if (W.quayle == 1) DrawText("QUAYLE IN THE TRAY", 12, 46, 10, (Color){ 150, 210, 110, 255 });
    if (T.minimap == 0) DrawText("M: map", 12, VIEW_H - 16, 10, (Color){ 255, 248, 230, 150 });
    if (T.saw_on != 0 && W.tray_wood < (int)T.wood_full) {   /* build E: a firewood log just ahead: your speed against the saw's sweet spot */
        static int widx[400], nw = -1;
        if (nw < 0) { nw = 0; for (int k = 0; k + 1 < RT.ntree && nw < 400; k++) { const RtTree *t = &RT.tree[k]; if (t->wood && t->kind != 5 && t->kind != 6 && t->t0 >= 0 && !strcmp(route_template(t->t0)->name, "log")) widx[nw++] = k; } }
        V3 f = qrot(c->body.rot, v3(0, 0, 1)); float best = 14.0f; int found = 0;
        for (int q = 0; q < nw; q++) { const RtTree *t = &RT.tree[widx[q]]; if (t->t0 < 0) continue; float dx = t->x - c->body.pos.x, dz = t->z - c->body.pos.z, d = hypotf(dx, dz); if (d < best && dx * f.x + dz * f.z > 0.3f * d) { best = d; found = 1; } }
        for (int k = 0; k < RT.nstag && !found; k++) { if (!stag_down(k)) continue; for (int q = 0; q < 3; q++) { if (stag_log_taken(k, q)) continue; float dx = RT.stag[k].log_x[q] - c->body.pos.x, dz = RT.stag[k].log_z[q] - c->body.pos.z, d = hypotf(dx, dz); if (d < best && dx * f.x + dz * f.z > 0.3f * d) { best = d; found = 1; } } }
        if (found) {
            float kmh = vlen(c->body.vel) * 3.6f;
            const char *t = kmh < 15.0f ? "LOG: FASTER" : kmh <= 35.0f ? "LOG: SPOT ON" : "LOG: EASE OFF";
            Color lc = kmh < 15.0f ? (Color){ 200, 200, 210, 255 } : kmh <= 35.0f ? (Color){ 140, 230, 130, 255 } : (Color){ 255, 170, 80, 255 };
            const char *tt = TextFormat("%s  %.0f km/h", t, kmh);
            DrawRectangle(320 - MeasureText(tt, 10) / 2 - 6, VIEW_H - 52, MeasureText(tt, 10) + 12, 16, (Color){ 20, 16, 30, 140 });
            DrawText(tt, 320 - MeasureText(tt, 10) / 2, VIEW_H - 49, 10, lc);
        }
    }
    if (c->low) {   /* build D: LOW RANGE and a small 4WD sign (the chassis from above: both axles driven, a prop shaft to each) */
        int x0 = 8, y0 = VIEW_H - 46; Color oc = { 255, 160, 60, 255 }, dk = { 20, 16, 30, 150 };
        DrawRectangle(x0 - 2, y0 - 2, 100, 26, dk);
        DrawRectangle(x0 + 2, y0 + 1, 4, 7, oc); DrawRectangle(x0 + 14, y0 + 1, 4, 7, oc);     /* front wheels */
        DrawRectangle(x0 + 2, y0 + 14, 4, 7, oc); DrawRectangle(x0 + 14, y0 + 14, 4, 7, oc);   /* rear wheels */
        DrawRectangle(x0 + 5, y0 + 4, 10, 1, oc); DrawRectangle(x0 + 5, y0 + 17, 10, 1, oc);   /* axles */
        DrawRectangle(x0 + 9, y0 + 4, 2, 14, oc);                                              /* prop shafts */
        DrawText("4WD", x0 + 24, y0 + 1, 10, oc);
        DrawText("LOW RANGE", x0 + 24, y0 + 12, 10, (Color){ 255, 235, 200, 255 });
    }
    float sp = vlen(c->body.vel), x = c->body.pos.x, z = c->body.pos.z;
    const char *hint = NULL;
    (void)sp;
    int at_hut = world_hut_dist(x, z) < T.hut_reach + 10.0f;   /* the hut's things happen on arrival: these say what's still wanted */
    if (at_hut && W.fire_level < 3 && W.tray_wood > 0 && W.tray_wood < (int)T.wood_full)
        hint = TextFormat("A FULL TRAY FOR THE FIRE: %d/%d", W.tray_wood, (int)T.wood_full);
    else if (at_hut && W.tray_deer && W.fire_level < 2)
        hint = "THE STEW NEEDS A BIGGER FIRE: TWO LOADS OF WOOD";
    else if (at_hut && !W.board && world_hamper_count() > 0 && world_hamper_count() < HAMPER_N)
        hint = TextFormat("THE BOARD NEEDS ALL %d: CHARCUTERIE %d/%d", HAMPER_N, world_hamper_count(), HAMPER_N);
    if (hint) DrawText(hint, 320 - MeasureText(hint, 10) / 2, 92, 10, (Color){ 255, 220, 150, (unsigned char)(200 + 55 * ((frame / 30) % 2)) });
}
/* Build C: Explorer's pause menu. */
/* the pause menu's items; Replay the Hut night only once it's been had (play test, 22 Oct) */
enum { PA_KEEP, PA_MAP, PA_HUT, PA_DAY, PA_REPLAY, PA_QUIT, PA_SAW, PA_TYRE };
static int near_hut_now;   /* build E: the ute is at the hut (the saw can be sharpened) */
static int pause_items(int *act)
{
    int n = 0;
    act[n++] = PA_KEEP;
    if (W.punct && W.spare > 0) act[n++] = PA_TYRE;   /* build E: a flat: change it */
    if (near_hut_now && T.saw_on != 0) act[n++] = PA_SAW;   /* build E: Will's chainsaw, at the hut */
    act[n++] = PA_MAP; act[n++] = PA_HUT; act[n++] = PA_DAY;
    if (W.hut_night) act[n++] = PA_REPLAY;
    act[n++] = PA_QUIT;
    return n;
}
static void draw_pause(int sel)
{
    static const char *name[8] = { "Keep exploring", "Show map", "Back to the hut", "Start a new day", "Replay the Hut night", "Quit to title", "Sharpen the saw", "Change the tyre" };
    int act[8], n = pause_items(act), y0 = 240 - (64 + 34 * n) / 2;
    DrawRectangle(160, y0, 320, 64 + 34 * n, (Color){ 20, 16, 30, 225 });
    DrawText("PAUSED", 320 - MeasureText("PAUSED", 20) / 2, y0 + 12, 20, RAYWHITE);
    for (int i = 0; i < n; i++) {
        int y = y0 + 52 + 34 * i;
        if (i == sel) DrawRectangle(180, y - 5, 280, 28, (Color){ 255, 235, 120, 255 });
        DrawText(name[act[i]], 200, y, 20, i == sel ? (Color){ 24, 28, 32, 255 } : RAYWHITE);
    }
    int a = sel >= 0 && sel < n ? act[sel] : PA_KEEP;
    const char *why = a == PA_DAY ? "the fire out, back to golden hour, the logs back, the ute fixed" : a == PA_HUT ? "the ute back in the car park" : a == PA_REPLAY ? "the finale again, with the song" :
                      a == PA_SAW ? TextFormat("Will's chainsaw: %.0f%% sharp", W.saw) : a == PA_TYRE ? "the spare from the tray" : "";
    DrawText(why, 320 - MeasureText(why, 10) / 2, y0 + 52 + 34 * n - 4, 10, (Color){ 200, 200, 210, 255 });
}

/* ---- build E: the skill games. Sharpening Will's chainsaw: a round file held at 30 degrees, a stroke a cutter, every
 * other cutter down the left side, then the right. A marker sweeps across; press LEFT (then RIGHT) as it lines up with the
 * 30 degree mark. Changing a tyre: crack the nuts, jack it up, swap, then the nuts in a star pattern; press when it lines up.
 * The result: rough, good or razor (the saw); a tidy or a sloppy job (the tyre). ---- */
typedef struct { int on, kind, n, i, result; float t, score, res_t, flash; int last_ok; } SkillGame;
static SkillGame sk;
enum { SK_SAW, SK_TYRE };
static void skill_start(int kind) { memset(&sk, 0, sizeof(sk)); sk.on = 1; sk.kind = kind; sk.n = kind == SK_SAW ? 16 : 10; synth_blip(6); }
static float skill_pos(void) { return 0.5f + 0.48f * sinf(sk.t * (sk.kind == SK_SAW ? 3.6f : 3.0f) + 0.6f * (float)sk.i); }
static int skill_want(void) { return sk.kind == SK_SAW ? (sk.i < sk.n / 2 ? -1 : 1) : 0; }   /* -1 left, 1 right, 0 either */
static void skill_done(void);
/* one frame: the key presses (-1 left, 1 right, 2 the OK button); returns 1 while it runs */
static void skill_step(float dt, int key)
{
    if (!sk.on) return;
    if (sk.res_t > 0) { sk.res_t -= dt; if (sk.res_t <= 0) sk.on = 0; return; }
    sk.t += dt; if (sk.flash > 0) sk.flash -= dt;
    if (!key) return;
    int want = skill_want(); float pos = skill_pos(), acc = clamp01(1.0f - fabsf(pos - 0.5f) / 0.5f);
    int ok = want == 0 ? 1 : key == want;
    if (ok) { sk.score += acc * acc; synth_blip2(5, 0.8f + 0.5f * acc, 0.7f); } else synth_blip2(10, 1.0f, 0.5f);
    sk.last_ok = ok && acc > 0.7f; sk.flash = 0.25f; sk.i++;
    if (sk.i >= sk.n) skill_done();
}
static void skill_done(void)
{
    float q = sk.score / (float)sk.n;   /* 1 = perfect */
    sk.res_t = 2.6f;
    if (sk.kind == SK_SAW) {
        sk.result = q >= T.saw_razor_q ? 2 : q >= 0.45f ? 1 : 0;
        W.saw = sk.result ? 100.0f : 70.0f; W.razor = sk.result == 2 ? 15 : 0; W.sharpened++; W.dirty = 1;
        if (sk.result == 2) world_earn(ACH_RAZOR);
        if (W.sharpened >= 3) world_earn(ACH_MECHANIC);
        synth_blip(sk.result == 2 ? 13 : 2);
    } else {
        sk.result = q >= 0.45f ? 1 : 0;
        if (W.punct) { W.tyre_wear[W.punct - 1] = sk.result ? 0.0f : 0.35f; W.punct = 0; W.spare--; W.dirty = 1; world_earn(ACH_TYRE); }
        synth_blip(2);
    }
    if (getenv("SHOT_SEQ") || getenv("BOT_EVENTS")) fprintf(stderr, "   skill game %d: %.0f%%, result %d\n", sk.kind, q * 100, sk.result);
}
static void skill_draw(void)
{
    if (!sk.on) return;
    DrawRectangle(60, 70, 520, 330, (Color){ 20, 16, 30, 250 });
    const char *title = sk.kind == SK_SAW ? "SHARPEN WILL'S CHAINSAW" : "CHANGE THE TYRE";
    DrawText(title, 320 - MeasureText(title, 20) / 2, 86, 20, (Color){ 255, 220, 140, 255 });
    if (sk.res_t > 0) {
        const char *r = sk.kind == SK_SAW ? (sk.result == 2 ? "RAZOR SHARP!" : sk.result ? "GOOD AND SHARP" : "ROUGH, BUT IT'LL CUT") : (sk.result ? "TIDY JOB" : "IT'LL HOLD... PROBABLY");
        const char *r2 = sk.kind == SK_SAW ? (sk.result == 2 ? "100%, and it'll stay keen longer" : sk.result ? "100%" : "70%") : (sk.result ? "The spare's on: good as new" : "The spare's on, a bit wobbly");
        DrawText(r, 320 - MeasureText(r, 30) / 2, 200, 30, sk.result ? (Color){ 150, 230, 140, 255 } : (Color){ 255, 190, 120, 255 });
        DrawText(r2, 320 - MeasureText(r2, 10) / 2, 240, 10, RAYWHITE);
        return;
    }
    int want = skill_want();
    const char *how = sk.kind == SK_SAW ? (want < 0 ? "LEFT side cutters: press LEFT as the file lines up at 30 degrees" : "RIGHT side cutters: press RIGHT as the file lines up at 30 degrees")
                                        : (sk.i < 5 ? "Crack the wheel nuts: press as it lines up" : "The spare's on: nuts in a star pattern, press as it lines up");
    DrawText(how, 320 - MeasureText(how, 10) / 2, 114, 10, RAYWHITE);
    /* the chain (the saw) or the wheel's five nuts (the tyre), done ones lit */
    if (sk.kind == SK_SAW) for (int k = 0; k < sk.n; k++) { int x = 120 + k * 25; Color c = k < sk.i ? (Color){ 220, 220, 230, 255 } : (Color){ 90, 90, 100, 255 }; if (k == sk.i) c = (Color){ 255, 220, 120, 255 }; DrawRectangle(x, 206 + ((k & 1) ? 6 : 0), 14, 10, c); DrawRectangle(x - 4, 222, 22, 3, (Color){ 120, 120, 130, 255 }); }
    else { static const int star[5] = { 0, 2, 4, 1, 3 }; DrawCircle(320, 185, 34, (Color){ 60, 60, 66, 255 }); for (int k = 0; k < 5; k++) { int nut = star[k]; float a = -1.5708f + 1.2566f * (float)nut; Color c = (sk.i % 5) > k || sk.i >= 10 ? (Color){ 220, 220, 230, 255 } : (sk.i % 5) == k ? (Color){ 255, 220, 120, 255 } : (Color){ 110, 110, 120, 255 }; DrawCircle(320 + (int)(cosf(a) * 22), 185 + (int)(sinf(a) * 22), 6, c); } }
    if (sk.kind == SK_SAW) {   /* the round file over the cutter, at the angle the sweep says; the 30 degree guide behind it */
        int cx = 120 + (sk.i < sk.n ? sk.i : sk.n - 1) * 25 + 7, cy = 206;
        float g = (want < 0 ? 1.0f : -1.0f) * 30.0f * DEG2RAD, a = (want < 0 ? 1.0f : -1.0f) * (30.0f + (skill_pos() - 0.5f) * 90.0f) * DEG2RAD;
        DrawLineEx((Vector2){ (float)cx, (float)cy }, (Vector2){ cx + sinf(g) * 60.0f, cy - cosf(g) * 60.0f }, 2.0f, (Color){ 90, 160, 100, 200 });
        DrawText("30", cx + (int)(sinf(g) * 66.0f) - 6, cy - (int)(cosf(g) * 66.0f) - 10, 10, (Color){ 120, 200, 120, 255 });
        DrawLineEx((Vector2){ (float)cx, (float)cy }, (Vector2){ cx + sinf(a) * 56.0f, cy - cosf(a) * 56.0f }, 5.0f, (Color){ 170, 170, 180, 255 });   /* the file */
        DrawLineEx((Vector2){ cx + sinf(a) * 40.0f, cy - cosf(a) * 40.0f }, (Vector2){ cx + sinf(a) * 62.0f, cy - cosf(a) * 62.0f }, 8.0f, (Color){ 150, 90, 50, 255 });   /* its handle */
    }
    /* the sweep: a bar with the 30 degree mark in the middle */
    int bx = 140, bw = 360, by = 260;
    DrawRectangle(bx, by, bw, 18, (Color){ 50, 46, 60, 255 });
    DrawRectangle(bx + bw / 2 - 26, by, 52, 18, (Color){ 70, 130, 80, 255 });
    DrawRectangle(bx + bw / 2 - 8, by, 16, 18, (Color){ 120, 200, 120, 255 });
    int mx = bx + (int)(skill_pos() * (float)bw);
    DrawRectangle(mx - 2, by - 6, 4, 30, sk.flash > 0 ? (sk.last_ok ? (Color){ 150, 255, 150, 255 } : (Color){ 255, 120, 90, 255 }) : RAYWHITE);
    const char *keys = sk.kind == SK_SAW ? (want < 0 ? "LEFT  (keyboard, D-pad, or the left button)" : "RIGHT  (keyboard, D-pad, or the right button)") : "any of LEFT, RIGHT, OK";
    DrawText(keys, 320 - MeasureText(keys, 10) / 2, 292, 10, (Color){ 220, 210, 230, 255 });
    DrawText(TextFormat("%d of %d", sk.i, sk.n), 320 - MeasureText(TextFormat("%d of %d", sk.i, sk.n), 10) / 2, 310, 10, (Color){ 200, 200, 210, 255 });
    DrawText("Back: stop", 80, 380, 10, (Color){ 180, 180, 190, 255 });
}
/* build E, selftest: the saw's passes and wear, the skill games' results, the new save lines */
static int gcheck(int ok, const char *what) { printf("  %s  %s\n", ok ? "ok  " : "FAIL", what); return ok; }
static int game_selftest(const char *dir)
{
    int ok = 1, ev = 0; World keep = W;
    float sw = T.saw_wear;
    W.mode = MODE_EXPLORE; W.saw = 100; W.razor = 0; W.streak = 0; saw_nkey = 0;
    ok &= gcheck(saw_pass(1, 25.0f / 3.6f, 0, &ev) == 1 && ev == 1 && fabsf(W.saw - (100.0f - sw)) < 0.01f, "the saw: sharp, 25 km/h: split in one pass, one pass of wear");
    float s0 = W.saw;
    ok &= gcheck(saw_pass(2, 6.0f / 3.6f, 0, &ev) == 0 && ev == 5 && W.saw == s0, "6 km/h: too slow, no cut and no wear");
    ok &= gcheck(saw_pass(3, 60.0f / 3.6f, 0, &ev) == 0 && ev == 3 && fabsf(s0 - W.saw - sw * 2.25f) < 0.01f, "60 km/h: a quarter cut, 2.25 times the wear");
    s0 = W.saw; saw_pass(4, 25.0f / 3.6f, 1, &ev);
    ok &= gcheck(fabsf(s0 - W.saw - sw * 1.5f) < 0.01f, "in the creek gravel: 1.5 times the wear");
    W.saw = 45; int a = saw_pass(5, 25.0f / 3.6f, 0, &ev), e1 = ev, b = saw_pass(5, 25.0f / 3.6f, 0, &ev);
    ok &= gcheck(a == 0 && e1 == 3 && b == 1 && ev == 2 && W.streak == 0, "half sharp (45%): two passes, and the streak starts over");
    W.saw = 20; a = saw_pass(6, 25.0f / 3.6f, 0, &ev); b = saw_pass(6, 25.0f / 3.6f, 0, &ev); int c3 = saw_pass(6, 25.0f / 3.6f, 0, &ev);
    ok &= gcheck(a == 0 && b == 0 && c3 == 1, "blunt (20%): three passes");
    W.saw = 0; ok &= gcheck(saw_pass(7, 25.0f / 3.6f, 0, &ev) == 0 && ev == 4, "0%: the saw won't cut");
    W.saw = 100; W.razor = 15; s0 = W.saw; saw_pass(8, 25.0f / 3.6f, 0, &ev);
    ok &= gcheck(fabsf(s0 - W.saw - sw * 0.5f) < 0.01f && W.razor == 14, "razor sharp: half the wear, one fewer log of it left");
    W.saw = 100; W.razor = 0; W.streak = 0; memset(W.earned, 0, sizeof(W.earned)); while (world_pop() >= 0) {}
    for (int k = 0; k < 9; k++) saw_pass(100 + k, 25.0f / 3.6f, 0, &ev);
    ok &= gcheck(W.streak == 9 && !W.earned[ACH_CLEAN_CUTS], "nine clean cuts in a row: no Clean cuts yet");
    saw_pass(200, 25.0f / 3.6f, 0, &ev);
    ok &= gcheck(W.earned[ACH_CLEAN_CUTS], "ten: Clean cuts");
    /* the skill games, played by a steady hand (as SHOT_SKILL) and by a careless one */
    for (int pass = 0; pass < 3; pass++) {
        W.saw = 30; W.sharpened = pass; skill_start(SK_SAW);
        for (int f = 0; f < 60 * 120 && sk.on && sk.res_t <= 0; f++) { float pos = skill_pos(); int key = pass == 2 ? ((f % 37) == 0 ? -1 : 0) : fabsf(pos - 0.5f) < 0.03f ? skill_want() : 0; skill_step(1.0f / 60.0f, key); }
        if (pass == 0) ok &= gcheck(sk.result == 2 && W.saw == 100 && W.razor == 15 && W.earned[ACH_RAZOR], "sharpening, a steady hand: razor sharp, 100% and 15 logs of half wear, Razor sharp");
        if (pass == 2) ok &= gcheck(sk.result == 0 && W.saw == 70 && W.razor == 0 && W.sharpened == 3 && W.earned[ACH_MECHANIC], "sharpening carelessly, the third today: rough, 70%, Bush mechanic");
    }
    W.punct = 3; W.spare = 1; W.tyre_wear[2] = 1.0f; skill_start(SK_TYRE);
    for (int f = 0; f < 60 * 120 && sk.on && sk.res_t <= 0; f++) { float pos = skill_pos(); skill_step(1.0f / 60.0f, fabsf(pos - 0.5f) < 0.03f ? 2 : 0); }
    ok &= gcheck(sk.result == 1 && W.punct == 0 && W.spare == 0 && W.tyre_wear[2] == 0 && W.earned[ACH_TYRE], "the tyre changed tidily: the spare on, good as new, Tyre change");
    sk.on = 0;
    /* the new save lines */
    char path[300]; snprintf(path, sizeof(path), "%s/progress-test-e.txt", dir); remove(path);
    world_load(path);
    W.saw = 37.5f; W.razor = 4; W.streak = 6; W.sharpened = 2; W.tracked[0] = ACH_WET_FEET + 1; W.tracked[1] = ACH_DEER_20 + 1; W.tracked[2] = 0; W.trials_best = 4321;
    W.tyre_wear[0] = 0.25f; W.tyre_wear[3] = 0.5f; W.body_dmg = 0.4f; W.punct = 2; W.spare = 0; W.wet_m = 640;
    world_save(); World w1 = W; world_load(path);
    ok &= gcheck(fabsf(W.saw - 37.5f) < 0.06f && W.razor == 4 && W.streak == 6 && W.sharpened == 2, "build E read back: the saw");
    ok &= gcheck(W.tracked[0] == w1.tracked[0] && W.tracked[1] == w1.tracked[1] && W.tracked[2] == 0 && W.trials_best == 4321 && fabsf(W.wet_m - 640) < 1, "build E read back: the tracked three, the best Creek Trials, the creek metres");
    ok &= gcheck(fabsf(W.tyre_wear[3] - 0.5f) < 0.001f && fabsf(W.body_dmg - 0.4f) < 0.001f && W.punct == 2 && W.spare == 0, "build E read back: the ute's tyres, body, flat and spare");
    remove(path);
    W = keep; T.saw_wear = sw;
    printf("gametest finished\n");
    return !ok;
}
/* Replay the Hut night: the fire at 3, the night and the woodpile as they were for it, put back after */
static int hn_replay, hn_keep_fire, hn_keep_pile; static float hn_keep_night;
static void hn_replay_start(void)
{
    hn_replay = 1; hn_move = 1; hn_keep_fire = W.fire_level; hn_keep_pile = W.woodpile; hn_keep_night = W.night;
    W.fire_level = 3; W.woodpile = 45; W.night = 1.0f; night_now = 1.0f; for (int l = 0; l < 3; l++) fire_grow[l] = 1.0f;
    hutnight_t = 0; fireworks_t = -1; finale_music(1);
}
static void hn_replay_end(void)
{
    if (!hn_replay) return;
    hn_replay = 0; W.fire_level = hn_keep_fire; W.woodpile = hn_keep_pile; W.night = hn_keep_night; night_now = W.night; fireworks_t = -1;
}

#ifdef BD_IOS
#include <SDL3/SDL_main.h>   /* SDL starts the iOS app and then calls this main */
#endif
#ifdef __ANDROID__
#include <android_native_app_glue.h>
#include <sys/stat.h>
#include <unistd.h>
struct android_app *GetAndroidApp(void);
#endif
int main(int argc, char **argv)
{
#ifdef __ANDROID__
    /* the phone build: files packed in the APK are read through raylib's fopen wrapper; records and the live
     * tuning file are written to the app's own storage, which becomes the working folder */
    if (chdir(GetAndroidApp()->activity->internalDataPath) == 0) mkdir("data", 0755);
    touch_on = 1;
#endif
#ifdef __EMSCRIPTEN__
    web_storage_open();   /* build B: records and progress kept in the browser's storage */
    erec_path = "/keep/records-bush-drift.txt";
#endif
#ifdef BD_IOS
    /* iOS: the game's files are read from the app bundle (the working folder); records and the live tuning file
     * go to the app's own writable folder */
    bd_ios_setup(&erec_path, &tuning_path);
    touch_on = 1;
#endif
    int want_title = 0, want_records = 0, want_ach = 0, fake_score = 0, mute = 0;
    int want_soundcheck = 0, idle = 0, want_cartest = 0, want_selftest = 0;
    const char *progress_arg = NULL;
    int want_routetest = 0, want_drivetest = 0, show_rows = 0, shot_frames = 0, bench = 0, hide_readout = 0, novsync = 0, pixel_arg = -1;
    const char *sets[16], *look_dir = NULL, *menu_keys = NULL;
    int nsets = 0;
    float start_s = -1;
    const char *shot_path = NULL, *car_path = "assets-built/lc79.car";
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--soundcheck")) want_soundcheck = 1;
        else if (!strcmp(argv[i], "--cartest")) want_cartest = 1;
        else if (!strcmp(argv[i], "--preset") && i + 1 < argc) { preset = atoi(argv[++i]); preset_given = 1; }
        else if (!strcmp(argv[i], "--rows")) show_rows++;
        else if (!strcmp(argv[i], "--routetest")) want_routetest = 1;  /* the route as read, its ground, trees and chunks; no window */
        else if (!strcmp(argv[i], "--drivetest")) want_drivetest = 1;  /* the road bot hut to bridge at a range of cornering limits; no window */
        else if (!strcmp(argv[i], "--route") && i + 1 < argc) rt_dir = argv[++i];
        else if (!strcmp(argv[i], "--bot") && i + 1 < argc) rt_bot_alat = (float)atof(argv[++i]); /* scripted runs: the road bot at this many m/s2 sideways */
        else if (!strcmp(argv[i], "--at") && i + 1 < argc) start_s = (float)atof(argv[++i]); /* scripted runs: start this far along the road (m), on its middle */
        else if (!strcmp(argv[i], "--looks") && i + 1 < argc) look_dir = argv[++i]; /* pictures at the hut, plateau, descent and ford at each pixel size */
        else if (!strcmp(argv[i], "--pixel") && i + 1 < argc) pixel_arg = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--novsync")) novsync = 1; /* frame-time runs: draw as fast as possible */
        else if (!strcmp(argv[i], "--mute")) mute = 1;
        else if (!strcmp(argv[i], "--recordsfile") && i + 1 < argc) erec_path = argv[++i];
        else if (!strcmp(argv[i], "--set") && i + 1 < argc && nsets < 16) sets[nsets++] = argv[++i];
        else if (!strcmp(argv[i], "--idle")) idle = 1;
        else if (!strcmp(argv[i], "--shot") && i + 2 < argc) { shot_path = argv[i + 1]; shot_frames = atoi(argv[i + 2]); i += 2; }
        else if (!strcmp(argv[i], "--tuning") && i + 1 < argc) tuning_path = argv[++i];
        else if (!strcmp(argv[i], "--box")) car_path = "";
        else if (!strcmp(argv[i], "--bench") && i + 1 < argc) bench = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--hide")) hide_readout = 1;
        else if (!strcmp(argv[i], "--menukeys") && i + 1 < argc) menu_keys = argv[++i];
        else if (!strcmp(argv[i], "--title")) want_title = 1;
        else if (!strcmp(argv[i], "--records")) want_records = 1;
        else if (!strcmp(argv[i], "--achievements")) want_ach = 1;
        else if (!strcmp(argv[i], "--fakescore") && i + 1 < argc) fake_score = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--label") && i + 1 < argc) test_label = argv[++i];
        else if (!strcmp(argv[i], "--selftest")) want_selftest = 1;   /* build B: the records and progress files, the achievements' rules, the deer's spots; no window */
        else if (!strcmp(argv[i], "--progressfile") && i + 1 < argc) progress_arg = argv[++i];
    }
    int keys = tuning_load(tuning_path);
    if (keys < 0) fprintf(stderr, "tuning: no file at %s, built-in values in use\n", tuning_path);
    else fprintf(stderr, "tuning: %d value%s read from %s over the built-in ones\n", keys, keys == 1 ? "" : "s", tuning_path);
    if (tuning_missing()[0]) fprintf(stderr, "tuning: not built in (using 0): %s\n", tuning_missing());
    preset_apply(preset);
    for (int k = 0; k < nsets; k++) {
        const char *eq = strchr(sets[k], '=');
        int hit = 0;
        for (int i = 0; eq && i < tuning_count(); i++)
            if (strlen(tuning_name(i)) == (size_t)(eq - sets[k]) && !strncmp(tuning_name(i), sets[k], (size_t)(eq - sets[k]))) { *tuning_ptr(i) = strtof(eq + 1, NULL); hit = 1; }
        if (!hit) { fprintf(stderr, "--set %s: no such tuning value\n", sets[k]); return 1; }
    }

    models[0].path = car_path;
    for (int i = 0; i < NMODELS; i++) {
        CarModel *m = &models[i];
        m->first = ntris;
        m->loaded = m->path[0] && load_car_file(m);
        if (!m->loaded && i == 0) {
            m->own[FL] = v3(-0.8f, -0.2f, 1.4f); m->own[FR] = v3(0.8f, -0.2f, 1.4f);
            m->own[RL] = v3(-0.8f, -0.2f, -1.4f); m->own[RR] = v3(0.8f, -0.2f, -1.4f);
            m->note = "box (no car file)"; m->name = "box";
            make_box_model();
            m->loaded = 1;
        }
        if (m->loaded && i == 0) {   /* build C: a light bar on the cab roof, for Explorer's nights (four lamps in a dark housing) */
            float yb = 1.5f, zb = 0.3f;
            push_box(v3(-0.58f, yb, zb - 0.06f), v3(0.58f, yb + 0.09f, zb + 0.06f), 0.12f, 0.12f, 0.13f);   /* the housing */
            for (int k = 0; k < 4; k++) {   /* the lamps, on its front face */
                float x0 = -0.5f + 0.27f * (float)k, x1 = x0 + 0.2f, zf = zb + 0.061f;
                push_tri(v3(x0, yb + 0.015f, zf), v3(x1, yb + 0.015f, zf), v3(x1, yb + 0.075f, zf), 1, 1, 1);
                push_tri(v3(x0, yb + 0.015f, zf), v3(x1, yb + 0.075f, zf), v3(x0, yb + 0.075f, zf), 1, 1, 1);
                for (int j = 1; j <= 2; j++) { tris[ntris - j].r = 255; tris[ntris - j].g = 250; tris[ntris - j].b = 235; tris[ntris - j].lamp = 1; }
            }
        }
        if (m->loaded && !m->own_lamps) {
            for (int s = -1; s <= 1; s += 2) {
                push_light(v3(0.72f * (float)s, 0.12f, T.body_front + 0.08f), 0.22f, 255, 245, 170);
                push_light(v3(0.72f * (float)s, 0.15f, T.body_rear - 0.05f), 0.18f, 200, 30, 25);
            }
        }
        m->count = ntris - m->first;
    }
    select_model(0);
    {   /* build C: the body's half width in 25 cm slices along it (above the tyres' feet), for posts (route_set_post_outline) */
        const CarModel *m = &models[0];
        if (m->has_outline && m->count > 0) {
            float hw[64] = { 0 }, z0[24], z1[24], w[24]; int n = 0;
            for (int k = 0; k < m->count; k++) for (int j = 0; j < 3; j++) {
                V3 p = tris[m->first + k].p[j];
                if (p.y < -0.25f) continue;
                int b = (int)floorf((p.z - m->rear) / 0.25f); if (b < 0) b = 0; if (b > 63) b = 63;
                hw[b] = fmaxf(hw[b], fabsf(p.x));
            }
            for (int b = 0; b < 64 && m->rear + 0.25f * (float)b < m->front; b++) {
                float lo = m->rear + 0.25f * (float)b, hi = fminf(lo + 0.25f, m->front);
                if (n > 0 && fabsf(w[n - 1] - hw[b]) < 0.03f) { z1[n - 1] = hi; w[n - 1] = fmaxf(w[n - 1], hw[b]); continue; }
                if (n == 24) { z1[n - 1] = hi; w[n - 1] = fmaxf(w[n - 1], hw[b]); continue; }
                z0[n] = lo; z1[n] = hi; w[n] = hw[b]; n++;
            }
            route_set_post_outline(n, z0, z1, w);
        }
    }

    if (want_soundcheck) return soundcheck();
    if (want_cartest) return car_selftest();   /* the handling's own checks (src/car.c, docs/handling-spec.md) */
    {
        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        int ntp = route_templates_load("assets-built/plants.txt") + route_templates_load("assets-built/hut.txt");
        if (!route_templates_load("assets-built/world.txt")) fprintf(stderr, "no deer or edge scrub (blender -b --python tools/make_world.py)\n");
        if (!ntp) fprintf(stderr, "no plant templates: bare ground (blender -b --python tools/make_plants.py ...)\n");
        rt_have = route_load(&RT, rt_dir);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        if (rt_have) fprintf(stderr, "route: %.0f m, %d plants and props (%d solid), read in %.0f ms\n", RT.length, RT.ntree, RT.nsolid, (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1e6);
    }
    if (rt_have) {   /* build B: the deer, and what this device has found */
        struct timespec t0, t1; clock_gettime(CLOCK_MONOTONIC, &t0);
        deer_place(T.deer_random != 0);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        fprintf(stderr, "deer: %d placed in %.0f ms\n", W.ndeer, (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1e6);
        clock_gettime(CLOCK_MONOTONIC, &t0);
        world_place_explore(&RT);   /* build C: the hamper, the fire, the rifle */
        world_place_creek_d(&RT);   /* build D: the stock route's markers and its camp */
        clock_gettime(CLOCK_MONOTONIC, &t1);
        int nh = 0; for (int i = 0; i < HAMPER_N; i++) nh += W.item[i].placed;
        fprintf(stderr, "explorer: %d of %d hamper things placed, firewood %d logs (clearing %d, road %d, bridge %d, creek %d), in %.0f ms\n", nh, HAMPER_N, RT.nwood, RT.wood_clearing, RT.wood_road, RT.wood_bridge, RT.wood_creek,
                (t1.tv_sec - t0.tv_sec) * 1000.0 + (t1.tv_nsec - t0.tv_nsec) / 1e6);
    }
    {   /* the progress file sits beside the records (the phones' own storage too) */
        static char pp[320];
        const char *slash = strrchr(erec_path, '/');
        if (progress_arg) snprintf(pp, sizeof(pp), "%s", progress_arg);
        else if (slash) snprintf(pp, sizeof(pp), "%.*s/progress-bush-drift.txt", (int)(slash - erec_path), erec_path);
        else snprintf(pp, sizeof(pp), "progress-bush-drift.txt");
        world_load(pp);
        if (want_drivetest || want_selftest || want_routetest) W.mode = getenv("BD_EXPLORE") ? MODE_EXPLORE : MODE_DRIFT;   /* tests: the mode they ask for */
        if ((want_drivetest || want_routetest) && getenv("BD_EXPLORE")) { explore = 1; RT.explore = 1; }
    }
    if (want_selftest) {
        int bad = erecords_selftest("data") | world_selftest("data") | game_selftest("data");
        if (rt_have) {
            int hidden = 0; for (int d = 0; d < W.ndeer; d++) hidden += W.deer[d].hidden;
            printf("deer: %d of %d placed (%d hidden, %d easy to see), every one reachable from the road round the trunks\n", W.ndeer, WORLD_DEER, hidden, W.ndeer - hidden);
            bad |= W.ndeer != WORLD_DEER;
        }
        printf(bad ? "selftest FAILED\n" : "selftest: all passed\n");
        return bad;
    }
    if (want_routetest) return rt_have ? routetest() : 1;
    if (want_drivetest) return rt_have ? drivetest(show_rows) : 1;
    if (!rt_have) fprintf(stderr, "no route in %s: nothing to drive (make route)\n", rt_dir);
    for (int i = 0; i < RT_SLOTS; i++) rt_slot[i].index = -1;

    Car car;
    Score score;
    memset(&car, 0, sizeof(car));
    memset(&score, 0, sizeof(score)); score.mult = 1;
    car_setup(&car, anchors);
    int scripted = bench || shot_path || look_dir;
    if (scripted) {   /* build C, pictures and tests: Explorer's state set up front (BD_FIRE=0..3, BD_HAMPER=n found, BD_BOARD=1, BD_RIFLE=0..2, BD_TRAY=n) */
        if (getenv("BD_FIRE")) { W.fire_level = atoi(getenv("BD_FIRE")); W.night = W.fire_level == 0 ? 0 : W.fire_level == 1 ? 0.5f : W.fire_level == 2 ? 0.9f : 1.0f; }
        if (getenv("BD_HAMPER")) { int n = atoi(getenv("BD_HAMPER")); W.hamper = n >= HAMPER_N ? (1u << HAMPER_N) - 1 : (1u << n) - 1; }
        if (getenv("BD_BOARD")) W.board = atoi(getenv("BD_BOARD"));
        if (getenv("BD_RIFLE")) W.rifle = atoi(getenv("BD_RIFLE"));
        if (getenv("BD_TRAY")) W.tray_wood = atoi(getenv("BD_TRAY"));
        if (getenv("BD_HUTNIGHT")) W.hut_night = atoi(getenv("BD_HUTNIGHT"));
        if (getenv("BD_QUAYLE")) W.quayle = atoi(getenv("BD_QUAYLE"));   /* follow-up: 1 in the tray, 2 in the cab */
        if (getenv("BD_TRAYDEER")) W.tray_deer = atoi(getenv("BD_TRAYDEER"));
        if (getenv("BD_STEW")) W.stew = atoi(getenv("BD_STEW"));
    }
    if (scripted && getenv("SHOT_ACH")) {   /* pictures: a few achievements earned, to show the screen */
        const int some[] = { ACH_CARPARK_DONUT, ACH_YODEL, ACH_DEER_1, ACH_BRIDGE, ACH_CLEAN };
        for (unsigned k = 0; k < sizeof(some) / sizeof(some[0]); k++) { W.earned[some[k]] = 1; snprintf(W.date[some[k]], 12, "2026-10-21"); }
        W.found = 0x2f;
        W.earned[ACH_FIRE_1] = 1; snprintf(W.date[ACH_FIRE_1], 12, "2026-10-22"); W.earned[ACH_STOCK_ROUTE] = 1; snprintf(W.date[ACH_STOCK_ROUTE], 12, "2026-10-22");   /* build E: more of it */
        W.tracked[0] = ACH_DEER_10 + 1; W.tracked[1] = ACH_WET_FEET + 1; W.tracked[2] = ACH_LOW_SLOW + 1; W.wet_m = 420; W.hamper = 0x1b; W.streak = 4; W.sharpened = 1; W.markers = 0x3ff;
        if (getenv("SHOT_ACHF")) { float sc = 0; sscanf(getenv("SHOT_ACHF"), "%d,%f,%d", &ach_filter, &sc, &ach_sel); ach_scroll = sc; }
    }
    erecords_load(erec_path);
    if (scripted && want_records)
        for (int i = EREC.n; i < EREC_MAX; i++) erecords_add_run(1234560 - i * 98760, 28 - i, 7800.0f - (float)i * 987.0f, 31234u + (uint32_t)i * 777u, "2026-10-28");
    int screen = want_title ? SCR_TITLE : want_records ? SCR_RECORDS : want_ach ? SCR_ACH : (scripted && rt_have) ? SCR_GAME : SCR_TITLE;
    int menu_sel = W.mode == MODE_EXPLORE ? MENU_EXPLORE : MENU_DRIFT, menu_block = 1;   /* build C: the last mode chosen */
    int en_rank = 0, en_new_far = 0, en_new_combo = 0, en_new_mult = 0;

#ifdef __EMSCRIPTEN__
    SetConfigFlags(novsync ? 0 : FLAG_VSYNC_HINT);   /* the browser: a fixed 4:3 canvas that the page scales (resizing it to the window squashed the picture) */
#elif defined(BD_IOS)
    SetConfigFlags(FLAG_WINDOW_HIGHDPI | FLAG_WINDOW_UNDECORATED | (novsync ? 0 : FLAG_VSYNC_HINT));   /* iOS: the whole screen at full resolution, no status bar */
#else
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | (novsync ? 0 : FLAG_VSYNC_HINT));
#endif
#ifndef __ANDROID__
    if (getenv("BD_TOUCH")) touch_on = 1;   /* tests: the touch buttons on the desktop (with BD_WINDOW=WxH for a phone shape) */
#endif
#ifdef BD_PHONE
    InitWindow(0, 0, "Bush Drift");   /* the phone: the whole display; the picture is centred and the touch buttons go either side */
#ifdef BD_IOS
    bd_ios_fit_screen();
#endif
#else
    InitWindow(VIEW_W * 2, VIEW_H * 2, "Bush Drift");
    { int ww, wh; if (getenv("BD_WINDOW") && sscanf(getenv("BD_WINDOW"), "%dx%d", &ww, &wh) == 2) SetWindowSize(ww, wh); }
#endif
    SetExitKey(KEY_NULL);
    SetTargetFPS(novsync ? 0 : 60);
    RenderTexture2D target = LoadRenderTexture(VIEW_W, VIEW_H);
    SetTextureFilter(target.texture, TEXTURE_FILTER_POINT);
    static rlRenderBatch hud_batch;
    hud_batch = rlLoadRenderBatch(HUD_COPIES, 2048);
    rlSetRenderBatchActive(&hud_batch);
    marks_init();
    for (int i = 0; i < NMODELS; i++) if (models[i].loaded) car_mesh_init(&models[i]);
    scene_meshes_init();
    wood_meshes_init();
    tray_meshes_init();
    look_init();
    far_init();
    if (rt_have) deer_meshes_init();
    if (rt_have) explore_meshes_init();
    look_set_mode(pixel_arg >= 0 ? pixel_arg : (int)T.pixel_mode);
    if (rt_have && (getenv("BD_EXPLORE") || (scripted && W.mode == MODE_EXPLORE && getenv("BD_KEEP_MODE")))) mode_enter(1);   /* tests and pictures: Explorer */
    if (rt_have && !stream_on) { route_stream_start(&RT); stream_on = 1; }
    if (rt_have) spawn_ex(&car, &score, !scripted && !explore);
    if (!mute) { sound_init(); synth_init(); }
#ifdef BD_IOS
    if (!mute) bd_ios_audio_ambient();   /* the silent switch mutes the game; other apps' music keeps playing */
#endif
    int prev_gear = car.gear;
    double st_sim = 0, st_draw = 0, st_worst = 0, st_worst_all = 0, st_t0 = GetTime();
    int st_frames = 0, st_slow = 0, st_window = 0;
    float show_worst = 0, show_sim = 0, show_draw = 0;
    double frame_ms_sum = 0; int frame_ms_n = 0; float frame_ms[8192]; /* --bench: every frame's time, for the percentiles */

    V3 prev_pos = car.body.pos;
    Q prev_rot = car.body.rot;
    float acc = 0;
#if defined(__EMSCRIPTEN__) || defined(BD_PHONE)
    hide_readout = 1;   /* the browser build starts clean; F1 shows the readout */
#endif
    int show_readout = !hide_readout, sel = 0, frame = 0, shift_req = 0, quit = 0, prev_run_state = run.state;
    int tree_flash = 0;
    char status[160] = "";
    float status_time = 0;
    for (int i = 0; i < tuning_count(); i++) if (!strcmp(tuning_name(i), "road_width")) sel = i;
    /* --looks: four places, three pixel sizes each */
    struct { const char *name; float s; } looks[4] = { { "hut", -1 }, { "plateau", 2400 }, { "descent", 5000 }, { "ford", -2 } };
    int look_i = 0, look_mode = 0, look_wait = 0;
    if (look_dir) start_s = -1;
    int freeze = 0, paused = 0, pause_sel = 0, map_view = 0;   /* build C: Explorer's pause menu (and its map) */
    V3 cam_pos = v3(0, 0, 0), cam_dir = v3(0, 0, 1); int cam_snap_was = 0;

    in_window = 1;
    while (!WindowShouldClose() && !quit) {
#ifdef BD_IOS
        bd_ios_wait_while_background();   /* no drawing while the app is in the background (iOS stops a GL app that draws there) */
#endif
        float dt = getenv("SHOT_SEQ") ? 1.0f / 60.0f : GetFrameTime();   /* recordings (session 3b): a fixed 60 fps step */
        if (frame > 30) {
            if (st_frames == 0) st_t0 = GetTime();
            st_frames++;
            if (dt > st_worst) st_worst = dt;
            if (dt > st_worst_all) st_worst_all = dt;
            if (dt > 0.020f) st_slow++;
            if (++st_window >= 120) { show_worst = (float)st_worst * 1000.0f; st_worst = 0; st_window = 0; }
            if (bench && frame_ms_n < 8192) frame_ms[frame_ms_n++] = dt * 1000.0f;
            frame_ms_sum += dt * 1000.0;
        }
        if (dt > 0.1f) dt = 0.1f;
        if (bench && novsync) dt = 1.0f / 60.0f; /* frame-time runs: the game moves on a 60th a frame however fast frames come */
        frame++;

        /* ---- input ---- */
        int pad = pick_pad();
        CarInput in = { 0 };
        int leave = 0, bank_req = IsKeyPressed(KEY_B) || IsKeyPressed(KEY_RIGHT_SHIFT);   /* build A: Left Shift stays the clutch */
        int low_req = IsKeyPressed(KEY_L), hut_req = 0;   /* build D: Low Range (Explorer): L, the pad's D-pad Up, LOW on a phone */
        int in_game = screen == SCR_GAME, over = in_game && run.state >= RUN_FINISHED;
        int m_up = IsKeyPressed(KEY_UP), m_down = IsKeyPressed(KEY_DOWN), m_ok = IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_X);
        int m_back = IsKeyPressed(KEY_ESCAPE) || IsKeyPressed(KEY_Z) || IsKeyPressed(KEY_BACKSPACE);
        int m_held = IsKeyDown(KEY_ENTER) || IsKeyDown(KEY_X) || IsKeyDown(KEY_ESCAPE) || IsKeyDown(KEY_Z);
        if (IsKeyPressed(KEY_ESCAPE)) leave = 1;
        if (menu_keys) {
            char k = (frame % 20 == 10 && frame / 20 < (int)strlen(menu_keys)) ? menu_keys[frame / 20] : '.';
            m_up = k == 'u'; m_down = k == 'd'; m_ok = k == 'a'; m_back = k == 'b'; leave = k == 'l'; m_held = 0;
        }
        if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_LEFT_ALT)) in.clutch = 1;
        if (IsKeyDown(KEY_LEFT)) in.steer -= 1;
        if (IsKeyDown(KEY_RIGHT)) in.steer += 1;
        if (IsKeyDown(KEY_UP) || IsKeyDown(KEY_X)) in.throttle = 1;
        if (IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_Z)) in.brake = 1;
        if (IsKeyDown(KEY_SPACE) || IsKeyDown(KEY_C)) in.handbrake = 1;
        int reset = IsKeyPressed(KEY_R) || IsKeyPressed(KEY_ENTER);
        touch_update();
        if (IsKeyPressed(KEY_BACK)) { leave = 1; m_back = 1; }   /* Android's back button */
        if (touch_on && in_game) {
            if (tb[TB_LEFT].down) in.steer -= 1;
            if (tb[TB_RIGHT].down) in.steer += 1;
            if (tb[TB_GAS].down) in.throttle = 1;
            if (tb[TB_BRAKE].down) in.brake = 1;
            if (tb[TB_HAND].down) in.handbrake = 1;
            if (tb_pressed(TB_RESTART)) reset = 1;
            if (tb_pressed(TB_BANK)) { if (explore) low_req = 1; else bank_req = 1; }   /* build D: in Explorer the button is LOW */
            if (tb_pressed(TB_PAUSE)) leave = 1;
            if (over && touch_tap && CheckCollisionPointRec(touch_tap_at, touch_game)) reset = 1;   /* run over: tap to go again */
        }
        if (touch_on && !in_game && touch_tap) {   /* menus: tap an item */
            Vector2 v = touch_to_view(touch_tap_at);
            if (screen == SCR_TITLE && reset_ask) { }   /* the Yes / No buttons: handled with the menu below */
            else if (screen == SCR_TITLE) {
                for (int i = 0; i < MENU_SHOWN; i++) if (v.x > 170 && v.x < 495 && v.y > MENU_Y0 - 3 + MENU_DY * i && v.y < MENU_Y0 + 31 + MENU_DY * i) { menu_sel = i; m_ok = 1; }
            } else m_back = 1;
        }
        int reload = IsKeyPressed(KEY_F5);
        if (IsKeyPressed(KEY_H)) want_preset = 1;
        if (want_preset && yodel.active) {   /* the mode keeps drift; H picks what comes back after it */
            want_preset = 0;
            yodel.saved_preset = (yodel.saved_preset + 1) % NPRESETS;
            snprintf(status, sizeof(status), "handling after drift yodel: %s", preset_name(yodel.saved_preset));
            status_time = 3;
        }
        if (want_preset) {
            want_preset = 0;
            preset_apply(preset + 1);
            car_setup(&car, anchors);
            snprintf(status, sizeof(status), "handling: %s", preset_name(preset));
            status_time = 3;
        }
        if (IsKeyPressed(KEY_M)) { T.minimap = T.minimap != 0 ? 0.0f : 1.0f; snprintf(status, sizeof(status), "map while driving: %s", T.minimap != 0 ? "on" : "off"); status_time = 2; }
        if (IsKeyPressed(KEY_P)) { look_set_mode(low_mode + 1); snprintf(status, sizeof(status), "pixels: %d wide", pix_w[low_mode]); status_time = 2; }
        if (IsKeyPressed(KEY_S) || IsKeyPressed(KEY_PAGE_UP)) shift_req = 1;
        if (IsKeyPressed(KEY_A) || IsKeyPressed(KEY_PAGE_DOWN)) shift_req = -1;
        if (pad >= 0) {
            int east = IsGamepadButtonDown(pad, GAMEPAD_BUTTON_RIGHT_FACE_RIGHT);
            int south = IsGamepadButtonDown(pad, GAMEPAD_BUTTON_RIGHT_FACE_DOWN);
            if (T.pad_swap_ab != 0) { int t = east; east = south; south = t; }
            if (east) in.throttle = 1;
            if (south) in.brake = 1;
            if (IsGamepadButtonDown(pad, GAMEPAD_BUTTON_LEFT_FACE_LEFT)) in.steer -= 1;
            if (IsGamepadButtonDown(pad, GAMEPAD_BUTTON_LEFT_FACE_RIGHT)) in.steer += 1;
            if (IsGamepadButtonDown(pad, GAMEPAD_BUTTON_RIGHT_TRIGGER_1)) in.handbrake = 1;
            if (IsGamepadButtonDown(pad, GAMEPAD_BUTTON_LEFT_TRIGGER_1) && !IsGamepadButtonDown(pad, GAMEPAD_BUTTON_MIDDLE)) in.clutch = 1;
            if (IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_MIDDLE_LEFT)) {
                if (IsGamepadButtonDown(pad, GAMEPAD_BUTTON_MIDDLE)) reload = 1;
                else reset = 1;
            }
            {
                int ok = IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_RIGHT_FACE_RIGHT), back = IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_RIGHT_FACE_DOWN);
                if (T.pad_swap_ab != 0) { int t = ok; ok = back; back = t; }
                m_ok |= ok || IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_MIDDLE_RIGHT);
                m_back |= back;
                m_up |= IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_LEFT_FACE_UP);
                m_down |= IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_LEFT_FACE_DOWN);
                m_held |= east || south || IsGamepadButtonDown(pad, GAMEPAD_BUTTON_MIDDLE_RIGHT) || IsGamepadButtonDown(pad, GAMEPAD_BUTTON_MIDDLE_LEFT);
            }
            if (in_game && IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_MIDDLE_RIGHT)) { if (over) leave = 1; else show_readout = !show_readout; }
            if (IsGamepadButtonDown(pad, GAMEPAD_BUTTON_MIDDLE_RIGHT) && IsGamepadButtonDown(pad, GAMEPAD_BUTTON_MIDDLE_LEFT)) leave = 1;
            if (IsGamepadButtonDown(pad, GAMEPAD_BUTTON_MIDDLE) && IsGamepadButtonDown(pad, GAMEPAD_BUTTON_MIDDLE_RIGHT)) leave = 1;
            if (in_game && IsGamepadButtonDown(pad, GAMEPAD_BUTTON_MIDDLE)) { /* Menu held: L1 = steering-twist build-up, R1 = gearbox, Up = pixel size, Down = handling preset */
                if (IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_RIGHT_TRIGGER_1)) T.manual_gears = T.manual_gears != 0 ? 0.0f : 1.0f;
                if (IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_LEFT_FACE_UP)) look_set_mode(low_mode + 1);
                if (IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_LEFT_FACE_DOWN)) want_preset = 1;
            }
            {
                int top = IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_RIGHT_FACE_UP), left = IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_RIGHT_FACE_LEFT);
                if (T.pad_swap_ab != 0) { int t = top; top = left; left = t; }
                if (top || IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_RIGHT_TRIGGER_2)) shift_req = 1;
                if (left) shift_req = -1;
                if (IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_LEFT_TRIGGER_2) && !IsGamepadButtonDown(pad, GAMEPAD_BUTTON_MIDDLE)) bank_req = 1;
                if (in_game && IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_LEFT_FACE_UP) && !IsGamepadButtonDown(pad, GAMEPAD_BUTTON_MIDDLE)) low_req = 1;   /* build D: D-pad Up */
            }
            if (T.use_stick != 0) {
                float sx = GetGamepadAxisMovement(pad, GAMEPAD_AXIS_LEFT_X);
                if (fabsf(sx) > 0.1f) in.steer += sx;
            }
        }
        in.steer = clampf(in.steer, -1, 1);
        if (bench && idle) {   /* build C: a frame-time run parked where it starts (BOT_CREEK_BENCH: driving down the creek way) */
            if (getenv("BOT_CREEK_BENCH") && RT.ck_n > 1) creek_follow(&car, &in, RT.ck_len, 6.0f);
        } else if (shot_path && getenv("SHOT_LAUNCH") && explore && RT.ck_n > 1) {   /* part 2: flat out down the creek at a place (SHOT_LAUNCH=S, from 60 m back) */
            float ls = 0, back = 60.0f; sscanf(getenv("SHOT_LAUNCH"), "%f,%f", &ls, &back);
            if (frame == 1) { float x, z; route_creek_point(&RT, ls - back, &x, &z); int i = (int)((ls - back) / RT_DS); float hd = atan2f(RT.ck_z[i + 1] - RT.ck_z[i], RT.ck_x[i + 1] - RT.ck_x[i]); car_reset(&car, v3(x, 0.75f, z), (float)(M_PI / 2) - hd); bp.init = 0; prev_pos = car.body.pos; prev_rot = car.body.rot; cam_snap = 1; }
            memset(&in, 0, sizeof(in)); creek_follow(&car, &in, ls + 60.0f, 16.0f); in.throttle = 1; in.brake = 0;
            if (getenv("SHOT_SEQ") && frame % 10 == 0) fprintf(stderr, "launch frame %d: %.0f m down, %.0f km/h, wheels down %d\n", frame, route_creek_near(&RT, car.body.pos.x, car.body.pos.z).s, vlen(car.body.vel) * 3.6f, bp.ncontact);
        } else if (shot_path && getenv("SHOT_STAG") && explore && RT.nstag > 0) {   /* build D: flat out at the stag */
            int k = atoi(getenv("SHOT_STAG")) % RT.nstag;
            V3 l = qrot(qconj(car.body.rot), v3(RT.stag[k].x - car.body.pos.x, 0, RT.stag[k].z - car.body.pos.z));
            memset(&in, 0, sizeof(in));
            if (!stag_down(k)) { in.steer = clampf(atan2f(l.x, l.z) * 2.4f, -1, 1); in.throttle = 1; } else in.brake = 1;
        } else if (shot_path && idle) {
            if (frame > 200 && run.state < RUN_FINISHED && !explore) run.state = RUN_WALL_OVER;
        } else if ((shot_path || bench) && rt_have) { /* scripted runs: the road bot */
            T.manual_gears = 0;
            if (getenv("BOT_DONUT")) {   /* build A: donuts on the spot, full lock and throttle (BOT_DONUT=1 left, -1 right) */
                static float stopped_for;
                memset(&in, 0, sizeof(in));
                stopped_for = vlen(car.body.vel) < 0.5f ? stopped_for + dt : stopped_for;
                if (stopped_for < 0.3f) in.brake = 1;   /* stop first (after --at the car is rolling) */
                else { in.steer = -(float)atof(getenv("BOT_DONUT")); in.throttle = 1; }
                if (run.state >= RUN_FINISHED) stopped_for = 0;
                if (getenv("SHOT_BANK_AT") && frame == atoi(getenv("SHOT_BANK_AT"))) { bank_req = 1; memset(&in, 0, sizeof(in)); }   /* build B: the Full Yodel test */
                if (getenv("SHOT_BANK_AT") && frame > atoi(getenv("SHOT_BANK_AT"))) { memset(&in, 0, sizeof(in)); in.brake = 1; }
            } else if (getenv("BOT_LOGS")) {   /* follow-up: the log bot in the game (pictures; BOT_GHOST=1 keeps it out of the trunks) */
                bot_logs = 1; bot_ghost = getenv("BOT_GHOST") != NULL; rt_deer_bot(&car, &in);
            } else if (getenv("BOT_DRIFT")) {   /* build A: the drift bot, banking after a second straight like the test drives */
                static float straight;
                bot_drift_deg = (float)atof(getenv("BOT_DRIFT")); bot_use_hb = getenv("BOT_HB") && atoi(getenv("BOT_HB"));
                rt_drift_bot(&car, rt_bot_alat > 0 ? rt_bot_alat : 3.0f, &in);
                straight = score.angle < 2.0f ? straight + dt : 0;
                if (straight > 1.0f && score.pot > 0 && !getenv("BOT_NOBANK")) bank_req = 1;
            } else rt_bot(&car, rt_bot_alat > 0 ? rt_bot_alat : 6.0f, 0.55f, 32.0f, &in);
            if (run.state >= RUN_FINISHED && !getenv("SHOT_AT_END")) reset = 1;
        }
        if (bench && frame >= bench) quit = 1;

        if (IsKeyPressed(KEY_F1)) show_readout = !show_readout;
        if (IsKeyPressed(KEY_F2)) T.use_stick = T.use_stick != 0 ? 0.0f : 1.0f;
        if (IsKeyPressed(KEY_F3)) pad_skip++;
        if (IsKeyPressed(KEY_F4)) T.manual_gears = T.manual_gears != 0 ? 0.0f : 1.0f;
        float road_before[7] = { T.road_width, T.verge_w, T.ditch_w, T.ditch_depth, T.bank_w, T.tree_density, T.scrub_density };
        if (reload) {
            int n = tuning_load(tuning_path);
            preset_apply(preset);
            select_model(cur_model);
            car_setup(&car, anchors);
            snprintf(status, sizeof(status), n < 0 ? "could not read %s" : "reloaded %s", tuning_path);
            status_time = 3;
        }
        if (IsKeyPressed(KEY_F6)) {
            snprintf(status, sizeof(status), tuning_save(tuning_path) == 0 ? "saved %s" : "could not save %s", tuning_path);
            status_time = 3;
        }
        if (IsKeyPressed(KEY_LEFT_BRACKET)) sel = (sel + tuning_count() - 1) % tuning_count();
        if (IsKeyPressed(KEY_RIGHT_BRACKET)) sel = (sel + 1) % tuning_count();
        int dn = IsKeyPressed(KEY_MINUS) || IsKeyPressedRepeat(KEY_MINUS), up = IsKeyPressed(KEY_EQUAL) || IsKeyPressedRepeat(KEY_EQUAL);
        if (dn || up) {
            float *p = tuning_ptr(sel);
            float step = fabsf(*p) > 1e-6f ? fabsf(*p) * 0.05f : 0.05f;
            *p += up ? step : -step;
            car_setup(&car, anchors);
        }
        { /* the road's shape changed (tuning): trees placed again and every chunk rebuilt */
            float now[7] = { T.road_width, T.verge_w, T.ditch_w, T.ditch_depth, T.bank_w, T.tree_density, T.scrub_density };
            if (rt_have && memcmp(now, road_before, sizeof(now))) {
                route_stream_stop(); rt_slots_clear(); route_replace_trees(&RT); route_stream_start(&RT);
            }
        }

        /* ---- title and records screens ---- */
        double t_sim0 = GetTime(), t_sim1 = t_sim0;
        int steps = 0, start_event = 0;
        near_hut_now = explore && in_game && world_hut_dist(car.body.pos.x, car.body.pos.z) < T.hut_reach && vlen(car.body.vel) < 2.0f;   /* build E */
        if (getenv("SHOT_SKILL") && in_game && !sk.on) {   /* pictures: SHOT_SKILL=saw or tyre starts one after a second */
            static int shot_sk_t, shot_sk_done; if (!shot_sk_done && ++shot_sk_t > 60) { shot_sk_done = 1; if (getenv("SHOT_SKILL")[0] == 't') { W.punct = 2; W.spare = 1; W.tyre_wear[1] = 1.0f; skill_start(SK_TYRE); } else { W.saw = 22; skill_start(SK_SAW); } }
        }
        if (sk.on && in_game) {   /* build E: a skill game: LEFT / RIGHT / OK; Back stops it */
            int kl = IsKeyPressed(KEY_LEFT) || (pad >= 0 && IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_LEFT_FACE_LEFT)) || (touch_on && tb_pressed(TB_LEFT));
            int kr = IsKeyPressed(KEY_RIGHT) || (pad >= 0 && IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_LEFT_FACE_RIGHT)) || (touch_on && tb_pressed(TB_RIGHT));
            int key = kl ? -1 : kr ? 1 : (m_ok || (touch_on && tb_pressed(TB_GAS))) ? 2 : 0;
            if (getenv("SHOT_SKILL")) { float pos = skill_pos(); key = fabsf(pos - 0.5f) < 0.03f ? (skill_want() ? skill_want() : 2) : 0; }   /* pictures and tests: a steady hand */
            if (key == 2 && skill_want() != 0) key = 0;   /* a side is wanted: OK does nothing */
            if ((m_back || leave) && sk.res_t <= 0) { sk.on = 0; leave = 0; m_back = 0; }
            else skill_step(dt, key);
            reset = 0; leave = 0;
        }
        if (in_game && explore && (!scripted || menu_keys)) {   /* build C: Explorer's pause menu (Esc, Start, the pause button) */
            if (paused && map_view) {   /* the map: any button back to the menu */
                reset = 0;
                if (m_ok || m_back || leave || (touch_on && touch_tap)) { map_view = 0; leave = 0; synth_blip2(5, 0.7f, 0.6f); }
            } else if (paused) {
                reset = 0;
                int pact[8], pn = pause_items(pact), py0 = 240 - (64 + 34 * pn) / 2;
                if (pause_sel >= pn) pause_sel = pn - 1;
                if (m_up) { pause_sel = (pause_sel + pn - 1) % pn; synth_blip2(5, 1.0f, 0.6f); }
                if (m_down) { pause_sel = (pause_sel + 1) % pn; synth_blip2(5, 1.0f, 0.6f); }
                if (touch_on && touch_tap) { Vector2 v = touch_to_view(touch_tap_at); for (int i = 0; i < pn; i++) if (v.x > 180 && v.x < 460 && v.y > py0 + 48 + 34 * i && v.y < py0 + 78 + 34 * i) { pause_sel = i; m_ok = 1; } }
                int pa = pact[pause_sel];
                if (m_ok && pa == PA_MAP) { map_view = 1; synth_blip(6); leave = 0; }   /* the map, still paused */
                else if (m_ok) {
                    synth_blip(6); paused = 0;
                    if (pa == PA_HUT) { reset = 1; hut_req = 1; }   /* back to the hut */
                    else if (pa == PA_SAW) skill_start(SK_SAW);   /* build E */
                    else if (pa == PA_TYRE) skill_start(SK_TYRE);
                    else if (pa == PA_REPLAY) hn_replay_start();
                    else if (pa == PA_DAY) {       /* a new day: the fire out, golden hour again, the logs back, the tray empty */
                        world_new_day(); mode_enter(1); night_now = 0; memset(fire_grow, 0, sizeof(fire_grow)); fireworks_t = -1; throw_t = -1; hutnight_t = -1; finale_music(0);
                        if (!scripted) world_save(), web_storage_flush();
                        snprintf(status, sizeof(status), "a new day at the hut"); status_time = 3;
                    } else if (pa == PA_QUIT) { if (!scripted) world_save(), web_storage_flush(); leave = 1; finale_music(0); }
                    if (pa != PA_QUIT) leave = 0;
                } else if (m_back || leave) { paused = 0; leave = 0; }
            } else if (leave && hutnight_t < 0) { paused = 1; pause_sel = 0; map_view = 0; leave = 0; synth_blip2(5, 0.7f, 0.6f); }
            if (reset && hutnight_t >= 0) reset = 0;
        }
        if (in_game && !scripted && run.state == RUN_RUNNING && (leave || reset) && !explore) {
            char today[12];
            records_today(today);
            int b = erecords_best(&EREC.furthest, rt_max_s, 0, today);
            b |= erecords_best(&EREC.combo, score.best_banked, 0, today);
            b |= erecords_best(&EREC.mult, (float)score.best_mult, 0, today);
            if (b) erecords_save(), web_storage_flush();
        }
        if (in_game && leave) yodel_stop(&car);
        if (in_game && leave) {
            if (scripted && !menu_keys) quit = 1;
            else { screen = SCR_TITLE; menu_block = 1; menu_sel = explore ? MENU_EXPLORE : MENU_DRIFT; }
        }
        if (screen != SCR_GAME) {
            reset = 0; shift_req = 0;
            if (menu_block) { if (!m_held) menu_block = 0; m_ok = m_back = 0; }
            if (scripted && !menu_keys) m_ok = m_back = m_up = m_down = 0;
            if (screen == SCR_ACH) {   /* build E: the list: move, filter, track */
                int kl = IsKeyPressed(KEY_LEFT) || (pad >= 0 && IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_LEFT_FACE_LEFT)), kr = IsKeyPressed(KEY_RIGHT) || (pad >= 0 && IsGamepadButtonPressed(pad, GAMEPAD_BUTTON_LEFT_FACE_RIGHT));
                Vector2 tv = touch_on && touch_tap && touch_moved < 12.0f ? touch_to_view(touch_tap_at) : (Vector2){ 0, 0 };
                float drag = 0; static float drag_y = -1; if (touch_on && GetTouchPointCount() > 0) { float yv = touch_to_view(GetTouchPosition(0)).y; if (drag_y >= 0) drag = yv - drag_y; drag_y = yv; } else drag_y = -1;
                if (ach_input(m_up, m_down, kl, kr, m_ok, m_back, dt, touch_on && touch_tap && touch_moved < 12.0f, tv, drag, GetMouseWheelMove())) { screen = SCR_TITLE; menu_block = 1; synth_blip2(5, 0.7f, 0.6f); }
            } else if (screen == SCR_RECORDS) {
                if (m_back || m_ok) { screen = SCR_TITLE; menu_block = 1; synth_blip2(5, 0.7f, 0.6f); }
            } else if (reset_ask) {   /* Reset progress: Yes or No (left/right or up/down; back is No) */
                if (m_up || m_down || IsKeyPressed(KEY_LEFT) || IsKeyPressed(KEY_RIGHT)) { reset_yes = !reset_yes; synth_blip2(5, 1.0f, 0.6f); }
                if (touch_on && touch_tap) { Vector2 v = touch_to_view(touch_tap_at); if (v.y > 255 && v.y < 295) { if (v.x > 170 && v.x < 310) { reset_yes = 1; m_ok = 1; } else if (v.x > 330 && v.x < 470) { reset_yes = 0; m_ok = 1; } } }
                if (m_back) { reset_ask = 0; menu_block = 1; synth_blip2(5, 0.7f, 0.6f); }
                else if (m_ok) {
                    if (reset_yes) {
                        world_reset_progress(); if (rt_have) route_logs_restore(&RT, NULL); deer_meshes_init(); web_storage_flush();
                        snprintf(status, sizeof(status), "progress reset: a fresh start"); status_time = 3; synth_blip(6);
                    } else synth_blip2(5, 0.7f, 0.6f);
                    reset_ask = 0; menu_block = 1;
                }
            } else {
                if (m_up) { menu_sel = (menu_sel + MENU_SHOWN - 1) % MENU_SHOWN; synth_blip2(5, 1.0f, 0.6f); }
                if (m_down) { menu_sel = (menu_sel + 1) % MENU_SHOWN; synth_blip2(5, 1.0f, 0.6f); }
                if (m_back && MENU_SHOWN > MENU_QUIT) menu_sel = MENU_QUIT;
                if (m_ok) {
                    if (menu_sel == MENU_QUIT) quit = 1;
                    else if (menu_sel == MENU_RECORDS) { screen = SCR_RECORDS; menu_block = 1; synth_blip(6); }
                    else if (menu_sel == MENU_ACH) { screen = SCR_ACH; menu_block = 1; synth_blip(6); }
                    else if (menu_sel == MENU_RESET) { reset_ask = 1; reset_yes = 0; menu_block = 1; synth_blip(6); }
                    else if (rt_have) { mode_enter(menu_sel == MENU_EXPLORE); screen = SCR_GAME; reset = 1; start_event = 1; synth_blip(6); paused = 0; }
                    else { snprintf(status, sizeof(status), "no route in %s: nothing to drive", rt_dir); status_time = 3; }
                }
            }
        }
        if (screen != SCR_GAME) {
            sound_menu();
            sound_yodel(dt); syn.fw_level = 1.0f;
            BeginTextureMode(target);
            ClearBackground(sky_colour);
            hud_batch.currentBuffer = frame % HUD_COPIES;
            if (screen == SCR_TITLE) draw_title(menu_sel);
            else if (screen == SCR_ACH) draw_achievements();
            else draw_records();
            if (status_time > 0) { text_centred(status, VIEW_H - 44, 10, MAROON); status_time -= dt; }
            goto present;
        }
        if (in_game && hutnight_t > 0.5f && (m_ok || m_back || leave || (touch_on && touch_tap))) { hutnight_t = -1; leave = m_ok = m_back = 0; reset = 0; }   /* the finale skipped */
        if (in_game && cut_t > 0.5f && (m_ok || m_back || leave || (touch_on && touch_tap))) {   /* the wood cutscene skipped: straight to its end */
            if (cut_level >= 3 && fireworks_t < 0) fireworks_t = 0;
            cut_t = -1; cam_snap = 1; leave = m_ok = m_back = 0; reset = 0;
        }
        if (in_game && hn_move) { car_reset(&car, v3(RT.start_x, 0.75f, RT.start_z), RT.start_yaw); prev_pos = car.body.pos; prev_rot = car.body.rot; cg.init = 0; hn_move = 0; }   /* the replay: parked at the hut */
        if (hn_replay && hutnight_t < 0) hn_replay_end();   /* the replay over (or skipped): the fire and the night as they were */
        if (in_game && scut_t > 0.5f && (m_ok || m_back || leave || (touch_on && touch_tap))) { scut_t = -1; cam_snap = 1; leave = m_ok = m_back = 0; reset = 0; }   /* the stew's cutscene skipped */
        if (in_game && scut_t >= 0) {
            if (scut_move) { car_reset(&car, v3(RT.start_x, 0.75f, RT.start_z), RT.start_yaw); prev_pos = car.body.pos; prev_rot = car.body.rot; cg.init = 0; scut_move = 0; }
            if (scut_t < SCUT_CUT + 1.0f && scut_t + dt >= SCUT_CUT + 1.0f) { synth_blip2(4, 0.9f, 1.0f); puff(vadd(oven_at, v3(0, -0.1f, 0)), 4, 0.3f, 0.2f); }   /* the pot down on the table */
            scut_t += dt;
            if (scut_t > SCUT_END) { scut_t = -1; cam_snap = 1; }
        }
        if (in_game && bcut_t > 0.5f && (m_ok || m_back || leave || (touch_on && touch_tap))) { bcut_t = -1; cam_snap = 1; leave = m_ok = m_back = 0; reset = 0; }   /* the board's cutscene skipped */
        if (in_game && bcut_t >= 0) {
            if (bcut_move) { car_reset(&car, v3(RT.start_x, 0.75f, RT.start_z), RT.start_yaw); prev_pos = car.body.pos; prev_rot = car.body.rot; cg.init = 0; bcut_move = 0; }
            int n = 0; for (int i = 0; i < HAMPER_N; i++) n += bcut_t >= BCUT_ITEM0 + (float)i * BCUT_DT + BCUT_FLY;
            while (bcut_landed < n) { synth_blip2(12, 1.1f + 0.06f * (float)bcut_landed, 0.6f); bcut_landed++; }   /* each one lands */
            if (bcut_t < BCUT_DONE && bcut_t + dt >= BCUT_DONE) {   /* the board as laid out: a little gold sparkle over it */
                for (int k = 0; k < 36; k++) { Fx *f = fx_new(FX_STAR); if (!f) break;
                    f->p = bcut_at(-0.85f + 1.7f * dust_rnd(), (dust_rnd() - 0.5f) * 0.3f, 0.15f); f->v = v3((dust_rnd() - 0.5f) * 0.6f, 0.8f + 1.0f * dust_rnd(), (dust_rnd() - 0.5f) * 0.6f);
                    f->life = 1.2f + 0.8f * dust_rnd(); f->size = 0.05f + 0.04f * dust_rnd(); f->r = 255; f->g = 226; f->b = 130; }
                synth_blip(2);
            }
            bcut_t += dt;
            if (bcut_t > BCUT_END) { bcut_t = -1; cam_snap = 1; }
        }
        if (in_game && cut_t >= 0 && cut_move_car) {   /* parked at the start of the track for the cutscene */
            car_reset(&car, v3(RT.start_x, 0.75f, RT.start_z), RT.start_yaw); prev_pos = car.body.pos; prev_rot = car.body.rot; cg.init = 0; cut_move_car = 0;
        }
        if (in_game && cut_t >= 0) {   /* the logs land one by one, each with a knock and a little dust */
            static int heard = -1; int n = cut_logs_now();
            if (heard < 0 || cut_t < CUT_LOG0) heard = cut_shown;
            while (heard < n && heard < 45) { V3 b = pile_base[heard]; puff(v3(b.x, route_height(&RT, b.x, b.z, NULL) + pile_y[heard] + 0.15f, b.z), 5, 0.4f, 0.3f); synth_blip2(4, 1.4f + 0.05f * (float)(heard % 5), 0.7f); heard++; }
            if (fabsf(cut_t - CUT_FIRE) < dt) synth_blip(6);
            cut_t += dt;
            if (cut_t > cut_end()) { cut_t = -1; cam_snap = 1; heard = -1; }
        }
        if (reset && explore && in_game && !hut_req && T.body_physics != 0 && run.state == RUN_RUNNING) {   /* build D: R in Explorer is Recover (the pause menu still has Back to the hut) */
            recover(&car); prev_pos = car.body.pos; prev_rot = car.body.rot; cam_snap = 1; acc = 0;
            ex_say("RECOVERED", (Color){ 200, 230, 255, 255 });
            reset = 0;
        }
        hut_req = 0;
        if (reset) {
            yodel_stop(&car);
            trail_clear();
            spawn_ex(&car, &score, start_event || explore);
            crash_lost = 0;
            prev_run_state = run.state;
            en_rank = en_new_far = en_new_combo = en_new_mult = 0;
            prev_pos = car.body.pos; prev_rot = car.body.rot; acc = 0;
            tree_flash = 0;
        }
        if (frame == 1 && getenv("SHOT_PLANT") && rt_have) { /* pictures (session 3): park 14 m short of the first of a species 6-12 m off the road, after --at */
            const char *want = getenv("SHOT_PLANT"); float from = start_s > 0 ? start_s : 100.0f; start_s = -1;
            for (int k = 0; k < RT.ntree; k++) {
                const RtTree *t = &RT.tree[k];
                if (t->t0 < 0 || strcmp(route_template(t->t0)->name, want)) continue;
                RtNear nn = route_near(&RT, t->x, t->z);
                if (nn.s < from || fabsf(nn.lat) < 6.0f || fabsf(nn.lat) > 12.0f) continue;
                start_s = nn.s - 16.0f; shot_plant_at = v3(t->x, route_height(&RT, t->x, t->z, NULL) + fminf(t->h * 0.3f, 5.0f), t->z); shot_plant_on = 1; break;
            }
            if (start_s >= 0) fprintf(stderr, "shot: %s at km %.2f\n", want, (start_s + 16.0f) / 1000.0f);
        }
        if (frame == 1 && getenv("SHOT_DEER") && rt_have) {   /* pictures (build B): park on the road short of deer N (1-20), the camera on it */
            int d = atoi(getenv("SHOT_DEER")) - 1;
            if (d >= 0 && d < W.ndeer) {
                float back = getenv("SHOT_DEER_BACK") ? (float)atof(getenv("SHOT_DEER_BACK")) : 18.0f;
                start_s = W.deer[d].s - back; shot_plant_on = 1;
                shot_plant_at = v3(W.deer[d].x, route_height(&RT, W.deer[d].x, W.deer[d].z, NULL) + 1.0f, W.deer[d].z);
                fprintf(stderr, "shot: deer %d at km %.2f, %+.0f m off the road (%s)\n", d + 1, W.deer[d].s / 1000, W.deer[d].lat, W.deer[d].hidden ? "hidden" : "easy");
            }
        }
        if (frame == 1 && getenv("SHOT_CUT") && explore) {   /* pictures (build C follow-up): load N's cutscene from its start */
            int n = atoi(getenv("SHOT_CUT")); W.fire_level = n; W.woodpile = 15 * n; W.night = n == 1 ? 0.5f : n == 2 ? 0.9f : 1.0f;
            night_now = n == 1 ? 0.0f : n == 2 ? 0.5f : 0.9f; cut_t = 0; cut_level = n; cut_shown = 15 * (n - 1); cut_move_car = 1;
        }
        if (frame == 1 && getenv("SHOT_MAPVIEW") && explore) paused = 1;
        if (frame == 1 && getenv("SHOT_REPLAY") && explore && W.hut_night) hn_replay_start();   /* pictures and tests: the menu's replay */
        if (frame == 1 && getenv("SHOT_PAUSE") && explore) { paused = 1; pause_sel = atoi(getenv("SHOT_PAUSE")); }   /* pictures: the pause menu, item N lit */
        if (frame == 1 && getenv("SHOT_HN") && explore) { hutnight_t = (float)atof(getenv("SHOT_HN")); if (hutnight_t >= HN_FW) { fireworks_t = hutnight_t - HN_FW; fw_next = 0; } }   /* pictures: the finale from T s */
        if (frame == 1 && getenv("SHOT_SCUT") && explore) { W.stew = 1; W.tray_deer = 0; scut_t = (float)atof(getenv("SHOT_SCUT")); scut_move = 1; }   /* pictures: the stew's cutscene from T s */
        if (frame == 1 && getenv("SHOT_SAY") && explore) ex_say2("THE BRIE", "Charcuterie: 3 of 12", (Color){ 140, 200, 255, 255 });   /* pictures: a two-line message */   /* pictures: the pause menu's map */
        if (frame == 1 && getenv("SHOT_BCUT") && explore) { W.board = 1; W.hamper = (1u << HAMPER_N) - 1; bcut_t = (float)atof(getenv("SHOT_BCUT")); bcut_landed = 0; bcut_move = 1; }   /* pictures: the board's cutscene from T s */
        if (frame == 1 && getenv("SHOT_FIREWORKS")) { fireworks_t = (float)atof(getenv("SHOT_FIREWORKS")); fw_next = 0; }   /* pictures (build C): the fireworks from this many seconds in */
        if (frame == 1 && getenv("SHOT_CREEK") && rt_have && RT.ck_n > 1) {   /* pictures (build C): parked in the creek way, S metres down it (SHOT_CREEK=S[,LAT]) facing downstream */
            float cs = 0, cl = 0; sscanf(getenv("SHOT_CREEK"), "%f,%f", &cs, &cl);
            float x, z; route_creek_point(&RT, cs, &x, &z); int i = (int)(cs / RT_DS); float hd = atan2f(RT.ck_z[i + 1] - RT.ck_z[i], RT.ck_x[i + 1] - RT.ck_x[i]);
            x -= sinf(hd) * cl; z += cosf(hd) * cl;
            car_reset(&car, v3(x, 0.75f, z), (float)(M_PI / 2) - hd); car.body.vel = v3(0, 0, 0);
            prev_pos = car.body.pos; prev_rot = car.body.rot; cam_snap = 1;
        }
        if (frame == 2 && getenv("SHOT_LOW") && explore) car_set_low(&car, 1);   /* pictures (build D): Low Range on */
        if (getenv("SHOT_STAG") && rt_have && explore && RT.nstag > 0) {   /* pictures (build D): driving at stag K from D m (SHOT_STAG=K[,D[,SIDE]]), flat out */
            int k = 0; float dd = 22.0f, side = 0; sscanf(getenv("SHOT_STAG"), "%d,%f,%f", &k, &dd, &side); k = k % RT.nstag;
            const struct RtStag *g = &RT.stag[k];
            float a = g->fall + (float)M_PI + side;   /* from the side it falls away from */
            if (frame == 1) { car_reset(&car, v3(g->x + cosf(a) * dd, 0.75f, g->z + sinf(a) * dd), atan2f(-cosf(a), -sinf(a))); prev_pos = car.body.pos; prev_rot = car.body.rot; cam_snap = 1; }
        }
        if (getenv("SHOT_GATE") && frame == 1 && rt_have && explore) {   /* build E pictures: SHOT_GATE=k,dist: the ute this far before gate k, facing down the creek */
            int k = 0; float dd = 12.0f; sscanf(getenv("SHOT_GATE"), "%d,%f", &k, &dd); if (gate_day != W.day) gates_place();
            if (ngate > 0) { k %= ngate; float x = gate[k].cx - gate[k].tx * dd, z = gate[k].cz - gate[k].tz * dd;
                car_reset(&car, v3(x, 0.75f, z), (float)(M_PI / 2) - atan2f(gate[k].tz, gate[k].tx)); prev_pos = car.body.pos; prev_rot = car.body.rot; cam_snap = 1;
                if (getenv("SHOT_BENT")) gate[k].bent[1] = 1; }
        }
        if (getenv("SHOT_TRIALS") && frame == 3) { TR.pot = 1240; TR.mult = 3; TR.streak = 11; TR.banked = 2150; W.trials_best = 4380; TR.msg_t = 9; }   /* build E pictures: a pot going */
        if (getenv("SHOT_FLAT") && frame == 1) { W.punct = 2; W.tyre_wear[1] = 1; W.body_dmg = 0.4f; }
        if (getenv("SHOT_LOG") && frame == 1 && rt_have) {   /* build E pictures: SHOT_LOG=n,dist,kmh: rolling at the n-th firewood log from dist m */
            int n = 0, seen = 0; float dd = 12.0f, kmh = 25.0f; sscanf(getenv("SHOT_LOG"), "%d,%f,%f", &n, &dd, &kmh);
            for (int k = 0; k + 1 < RT.ntree; k++) { const RtTree *t = &RT.tree[k]; if (!(t->wood && t->kind != 5 && t->kind != 6 && t->t0 >= 0 && !strcmp(route_template(t->t0)->name, "log"))) continue;
                if (seen++ != n) continue;
                RtNear rn = route_near(&RT, t->x, t->z); float hd = route_heading_at(&RT, fmaxf(rn.s, 0.0f)), x = t->x - cosf(hd) * dd, z = t->z - sinf(hd) * dd;
                car_reset(&car, v3(x, 0.75f, z), (float)(M_PI / 2) - hd); car.body.vel = v3(cosf(hd) * kmh / 3.6f, 0, sinf(hd) * kmh / 3.6f);
                prev_pos = car.body.pos; prev_rot = car.body.rot; cam_snap = 1; break; }
        }
        if (frame == 1 && start_s >= 0 && rt_have) { /* --at: on the road's middle, facing along it, rolling at 40 km/h */
            float x, z; route_point(&RT, start_s, &x, &z);
            float hd = route_heading_at(&RT, start_s);
            if (getenv("SHOT_LAT")) { float lt = (float)atof(getenv("SHOT_LAT")); x -= sinf(hd) * lt; z += cosf(hd) * lt; }   /* build B: metres to the left of the road */
            float turn = getenv("SHOT_TURN") ? (float)atof(getenv("SHOT_TURN")) * DEG2RADF : 0.0f;   /* degrees to the left of the road's way */
            car_reset(&car, v3(x, 0.75f, z), (float)(M_PI / 2) - route_heading_at(&RT, start_s) - turn);
            hd += turn;
            car.body.vel = (shot_plant_on || getenv("SHOT_LAT")) ? v3(0, 0, 0) : v3(cosf(hd) * 11.0f, 0, sinf(hd) * 11.0f);
            rt_max_s = start_s; prev_pos = car.body.pos; prev_rot = car.body.rot; cam_snap = 1;
            run.next_cp = (int)(start_s / fmaxf(T.checkpoint_every, 50.0f)); run.prev_s = start_s;   /* build B: the checkpoints behind count as passed */
        }
        if (look_dir && rt_have) { /* --looks: park at each place, wait for the road, snap a picture per pixel size */
            freeze = 1;
            if (look_wait == 0) {
                float s = looks[look_i].s;
                if (s == -1) car_reset(&car, v3(RT.start_x, 0.75f, RT.start_z), RT.start_yaw);
                else {
                    if (s == -2) s = RT.ford_s - 22.0f;
                    float x, z; route_point(&RT, s, &x, &z);
                    car_reset(&car, v3(x, 0.75f, z), (float)(M_PI / 2) - route_heading_at(&RT, s));
                }
                prev_pos = car.body.pos; prev_rot = car.body.rot; cam_snap = 1;
                look_set_mode(look_mode);
                run.state = RUN_RUNNING; run.time = 0;
            }
            look_wait++;
        }

        /* build D: Low Range. Explorer only (Drift ignores the button and never has it on); the 79 shifts into 4L slowly rolling */
        if (car.low && (!explore || yodel.active)) car_set_low(&car, 0);
        if (low_req && in_game && !paused && run.state == RUN_RUNNING) {
            if (!explore) { snprintf(status, sizeof(status), "Low Range: Explore only"); status_time = 2; }
            else if (yodel.active) { }
            else if (!car.low && vlen(car.body.vel) > T.low_max_kmh / 3.6f) ex_say("SLOW DOWN FOR LOW RANGE", (Color){ 255, 200, 120, 255 });
            else { car_set_low(&car, !car.low); synth_blip2(3, car.low ? 0.8f : 1.2f, 1.0f); if (getenv("SHOT_SEQ")) fprintf(stderr, "low range %s\n", car.low ? "on" : "off"); }
        }
        low_shown = car.low;

        /* ---- fixed-step simulation ---- */
        t_sim0 = GetTime();
        if (rt_have) { creek_frame(car.body.pos, dt); rt_frame(car.body.pos); }
        acc += dt;
        mark_clock += dt;
        int road_ready = rt_have && route_stream_pending() == 0;
        while (acc >= FIXED_DT && steps < 5) {
            prev_pos = car.body.pos;
            prev_rot = car.body.rot;
            CarInput sin = in;
            sin.shift = shift_req;
            shift_req = 0;
            if (run.state >= RUN_FINISHED || cutscene_on() || hutnight_t >= 0) sin = (CarInput){ .handbrake = 1, .brake = vlen(car.body.vel) > 2.0f ? 1.0f : 0.0f };
            RtPhys ph = { 0 };
            ph.tree = -1; ph.wet = -1; ph.stag = ph.staglog = -1;   /* build E: nothing touched till the physics says so (the countdown read as wet log 0) */
            logs_break = !explore || W.tray_wood < (int)T.wood_full;   /* build C: a full tray leaves the logs where they lie */
            if (run.state != RUN_COUNTDOWN && !freeze && !paused && !sk.on) {
                car_step(&car, &sin);
                rt_physics(&car, &ph);
                marks_step(&car);
                if (run.state == RUN_RUNNING && in_game && army_check(&car)) {   /* Quayle touched: found (build C: he stays, looking about for his rifle) */
                    if (!explore) army_flash_t = 3.5f;
                    else if (W.rifle == 0) ex_say2("QUAYLE FOUND!", "Quayle: \"Lost me rifle out here. Help us find it?\"", (Color){ 150, 210, 110, 255 });
                    /* with the rifle in the tray, explore_step gives it back just after: its message instead */
                    puff(v3(W.army_x, route_height(&RT, W.army_x, W.army_z, NULL) + 0.8f, W.army_z), 40, 1.2f, 1.6f);
                    synth_blip(12);
                    if (!scripted) world_save(), web_storage_flush();
                    if (getenv("SHOT_SEQ")) fprintf(stderr, "army man found\n");
                }
                if (run.state == RUN_RUNNING && in_game) {   /* build B: a deer touched: found, gone in a puff */
                    int d = deer_check(&car);
                    if (d >= 0 && explore && W.tray_deer == d + 1) {   /* build C follow-up: Quayle's shot: into the tray, and he's off into the cab */
                        ex_say2("DEER IN THE TRAY", "Quayle: \"I'll hop in the cab. Take it back to the hut.\"", (Color){ 255, 170, 70, 255 });
                        synth_blip(13);
                        if (!scripted) world_save(), web_storage_flush();
                        if (getenv("BOT_EVENTS") || getenv("SHOT_SEQ")) fprintf(stderr, "   deer %d into the tray\n", d + 1);
                    } else if (d >= WORLD_DEER) {   /* one of Quayle's five */
                        int nx = 0; for (int e = WORLD_DEER; e < WORLD_DEER + W.nextra; e++) nx += (W.found >> e) & 1;
                        ex_say(TextFormat("QUAYLE'S DEER %d/%d", nx, W.nextra), (Color){ 255, 170, 70, 255 });
                        puff(v3(W.deer[d].x, route_height(&RT, W.deer[d].x, W.deer[d].z, NULL) + 0.6f, W.deer[d].z), 40, 1.6f, 1.4f);
                        synth_blip(12);
                        if (!scripted) world_save(), web_storage_flush();
                    } else if (d >= 0) {
                        deer_flash_t = 3.5f; deer_flash_n = world_found_count();
                        puff(v3(W.deer[d].x, route_height(&RT, W.deer[d].x, W.deer[d].z, NULL) + 0.6f, W.deer[d].z), 40, 1.6f, 1.4f);
                        synth_blip(12);
                        if (!scripted) world_save(), web_storage_flush();
                        if (getenv("SHOT_SEQ")) fprintf(stderr, "deer %d found (%d of %d)\n", d + 1, deer_flash_n, W.ndeer);
                    }
                }
                if (run.state == RUN_RUNNING && in_game && explore) explore_step(&car, !scripted);   /* build C */
            }
            int before = run.state;
            if (!freeze && !paused && !sk.on) rt_rules(&run, &car, &ph, &rt_max_s, road_ready);
            if (run.ev_thud > 0) {   /* build C, Explorer: a thud, a shake and a puff; reverse out */
                explore_thud(&car, &ph); shake_t = 0.35f; synth_blip2(4, 0.8f, 1.0f);
                V3 hp = vadd(car.body.pos, vscale(vnorm(v3(RT.tree[ph.tree].x - car.body.pos.x, 0, RT.tree[ph.tree].z - car.body.pos.z)), 2.0f));
                puff(v3(hp.x, route_height(&RT, hp.x, hp.z, NULL) + 0.5f, hp.z), 14, 1.0f, 0.8f);
                if (getenv("SHOT_SEQ")) fprintf(stderr, "thud: %s at %.0f km/h\n", rt_hit_what, run.ev_thud * 3.6f);
                run.ev_thud = 0;
            }
            if (explore && !freeze && !paused) {   /* build D: the stags */
                int se = stag_event(&car, &ph);
                if (se == 1) {
                    synth_blip(14); shake_t = 0.2f; ex_say("TIMBER!", (Color){ 255, 220, 160, 255 });
                    puff(v3(RT.stag[ph.stag].x, route_height(&RT, RT.stag[ph.stag].x, RT.stag[ph.stag].z, NULL) + 1.0f, RT.stag[ph.stag].z), 16, 1.0f, 1.2f);
                    if (!scripted) world_save(), web_storage_flush();
                    if (getenv("SHOT_SEQ")) fprintf(stderr, "stag %d down at %.0f km/h\n", ph.stag, ph.stag_speed * 3.6f);
                } else if (se == 2) {
                    shake_t = 0.35f; synth_blip2(4, 0.8f, 1.0f);
                    puff(v3(RT.stag[ph.stag].x, route_height(&RT, RT.stag[ph.stag].x, RT.stag[ph.stag].z, NULL) + 0.5f, RT.stag[ph.stag].z), 14, 1.0f, 0.8f);
                } else if (se == 3) {
                    int k = ph.staglog / 3, q = ph.staglog % 3;
                    run.wood++; wood_flash_t = 3.0f; synth_blip(4);
                    splinters_burst(v3(RT.stag[k].log_x[q], route_height(&RT, RT.stag[k].log_x[q], RT.stag[k].log_z[q], NULL) + 0.4f, RT.stag[k].log_z[q]), car.body.vel);
                    if (!scripted) world_save(), web_storage_flush();
                }
                stag_step(FIXED_DT, 1);
                if (ph.saw_ev) {   /* build E: Will's chainsaw */
                    static float saw_prev = 100;
                    if (ph.saw_ev == 3) { splinters_burst(v3(ph.saw_x, route_height(&RT, ph.saw_x, ph.saw_z, NULL) + 0.3f, ph.saw_z), car.body.vel); synth_blip2(4, 1.1f, 0.7f); if (ex_flash_t <= 0) ex_say("NOT THROUGH YET: AGAIN", (Color){ 255, 210, 150, 255 }); }
                    else if (ph.saw_ev == 4) { if (ex_flash_t <= 0) ex_say2("THE SAW WON'T CUT", "Will's chainsaw needs sharpening: back to the hut", (Color){ 255, 150, 110, 255 }); synth_blip2(4, 0.7f, 0.8f); }
                    else if (ph.saw_ev == 5) { if (ex_flash_t <= 0) ex_say("TOO SLOW TO CUT", (Color){ 220, 220, 230, 255 }); }
                    else if (ph.saw_ev == 1 && W.streak >= 3) { if (ex_flash_t <= 0) ex_say(TextFormat("CLEAN CUT x%d", W.streak), (Color){ 160, 230, 150, 255 }); }
                    if ((saw_prev > 50 && W.saw <= 50) || (saw_prev > 25 && W.saw <= 25)) ex_say2(W.saw <= 25 ? "THE SAW'S NEARLY BLUNT" : "THE SAW'S GETTING BLUNT", "Sharpen it at the hut (the pause menu there)", (Color){ 255, 190, 120, 255 });
                    saw_prev = W.saw;
                    if (!scripted) world_save(), web_storage_flush();
                }
                if (ph.wet >= 0) {   /* part 2: a wet log: smashed, or the first shove says what it is */
                    float lx, lz, ly; wet_pos(ph.wet, &lx, &lz, &ly);
                    if (ph.wet_smash) {
                        splinters_burst(v3(lx, route_height(&RT, lx, lz, NULL) + 0.3f, lz), car.body.vel); puff(v3(lx, route_height(&RT, lx, lz, NULL) + 0.4f, lz), 20, 1.6f, 1.0f);
                        synth_blip2(4, 0.9f, 1.0f); shake_t = 0.2f;
                        ex_say2("SMASHED A WET LOG", "Too soggy to burn: not firewood", (Color){ 150, 200, 230, 255 });
                        if (!scripted) world_save(), web_storage_flush();
                    } else if (!wet_seen[ph.wet]) { ex_say2("WET LOG", "Too soggy to burn: shove it aside", (Color){ 150, 200, 230, 255 }); synth_blip2(4, 1.2f, 0.6f); }
                    wet_seen[ph.wet] = 1;
                }
            }
            if (!freeze && !paused && run.state == RUN_RUNNING) {   /* build D: the trail of safe spots, and a rollover in Explorer */
                trail_step(&car);
                if (T.body_physics != 0 && bp.rolled && explore) {
                    if (bp.rolled_t < FIXED_DT * 1.5f) { ex_say("OVER SHE GOES!", (Color){ 255, 180, 120, 255 }); shake_t = 0.35f; synth_blip2(4, 0.6f, 1.0f); }
                    if (bp.rolled_t > 2.2f) { recover(&car); prev_pos = car.body.pos; prev_rot = car.body.rot; cam_snap = 1; ex_say("BACK ON ITS WHEELS", (Color){ 200, 230, 255, 255 }); }
                }
            }
            if (!freeze && rt_offroad_timer(&run, &car, &ph)) {   /* follow-up: back on the track */
                prev_pos = car.body.pos; prev_rot = car.body.rot; cam_snap = 1; cg.init = 0;
                snprintf(status, sizeof(status), "back on the track"); status_time = 2;
                if (getenv("SHOT_SEQ")) fprintf(stderr, "reset onto the track at frame %d\n", frame);
            }
            if (ph.log_broken && run.state == RUN_RUNNING) {   /* follow-up: firewood */
                run.wood++; wood_flash_t = 3.0f;
                if (explore) { W.tray_wood++; world_log_gone(ph.log_cx, ph.log_cz); if (!scripted) world_save(), web_storage_flush(); }   /* build C: kept, and gone till a new day */ splinters_burst(v3(ph.log_x, route_height(&RT, ph.log_x, ph.log_z, NULL) + 0.4f, ph.log_z), car.body.vel);
                synth_blip(4);
                if (getenv("SHOT_SEQ")) fprintf(stderr, "firewood %d at frame %d\n", run.wood, frame);
            }
            if (run.state == RUN_WALL_OVER && before == RUN_RUNNING) { car.body.vel = v3(0, 0, 0); car.body.angvel = v3(0, 0, 0); tree_flash = 1; }
            if (run.state == RUN_FINISHED && before == RUN_RUNNING) rt_finish_time = run.time;
            rt_last = ph;
            acc -= FIXED_DT;
            steps++;
        }
        if (steps == 5) acc = 0;
        int level_note = 0, event_ended = 0, lost_this_frame = 0;
        {
            float lost = -1;
            if (run.state == RUN_WALL_OVER && prev_run_state != RUN_WALL_OVER) { lost = score_combo(&score); crash_lost = lost; score_lose(&score); score.ev_lost = 0; lost_this_frame = 1; }
            else if (bank_req && T.score_bank_button != 0 && in_game && run.state == RUN_RUNNING && score.pot > 0) score_expire(&score);
            event_ended = run.state != prev_run_state && run.state >= RUN_FINISHED;
            if (event_ended && run.state != RUN_WALL_OVER && score.pot > 0) score_expire(&score);   /* the bridge or wrong way: bank what's in the pot */
            prev_run_state = run.state;
            if (fake_score && scripted && event_ended) { score.total = (float)fake_score; score.best_mult = 17; score.best_banked = (float)(fake_score / 3 / 10 * 10); rt_max_s = 3456.0f; }
            if (lost >= 0) { synth_blip(lost > 0 ? 9 : 4); level_note = 1; if (!bench) printf("%s at km %.2f, %.0f km/h: lost the pot (%.0f points)\n", rt_hit_what, rt_max_s / 1000, run.ev_hit_speed * 3.6f, lost); }
            else if (event_ended && run.state == RUN_FINISHED) { synth_blip(6); level_note = 1; }
        }
        score_update(&score, &car, dt, run.state == RUN_RUNNING && run.dir == DIR_CORRECT && !explore, rt_last.deep, rt_last.offroad);
        donut_update(&score, &car, dt, run.state == RUN_RUNNING);
        if (score.ev_donut && run.state == RUN_RUNNING && !yodel.active && !snd.finale_on) yodel_start(&car);   /* (not over the Hut night's song) */
        if (yodel.active) {
            yodel.left -= dt; if (yodel.banner > 0) yodel.banner -= dt;
            if (run.state != RUN_RUNNING) yodel_stop(&car);
            else if (yodel.left <= 0) { yodel_stop(&car); snprintf(status, sizeof(status), "drift yodel over: a donut starts it again"); status_time = 3; }
        }
        if (event_ended) {
            printf("BUSH run over (%s): %.0f m, %.1f s, score %d, best x%d, best combo %.0f\n",
                   run.state == RUN_FINISHED ? "bridge" : run.state == RUN_WALL_OVER ? rt_hit_what : "wrong way", rt_max_s, run.time,
                   (int)(score.total + 0.5f), score.best_mult, score.best_banked);
            if (!scripted || fake_score) {
                char today[12];
                records_today(today);
                uint32_t t100 = run.state == RUN_FINISHED ? (uint32_t)(rt_finish_time * 100.0f + 0.5f) : 0;
                en_rank = erecords_add_run((int)(score.total + 0.5f), score.best_mult, rt_max_s, t100, today);
                en_new_far = erecords_best(&EREC.furthest, rt_max_s, t100, today);
                en_new_combo = erecords_best(&EREC.combo, score.best_banked, t100, today);
                en_new_mult = erecords_best(&EREC.mult, (float)score.best_mult, t100, today);
                if (!scripted && (en_rank || en_new_far || en_new_combo || en_new_mult)) erecords_save(), web_storage_flush();
            }
        }
        sound_update(&car, run.state < RUN_FINISHED);
        sound_yodel(dt);
        syn.fw_level = (snd.finale_on ? 1.0f / fmaxf(snd_duck, 0.2f) : 1.0f) * (hutnight_t >= 0 ? T.fw_finale : 1.0f);   /* the fireworks heard under the song (not turned down with the rest) */
        if (snd.finale_on) SetMusicVolume(snd.finale, clamp01(T.snd_music));   /* the song is the main thing: full */
        if (level_note) { }
        else if (score.ev_mult_up) synth_blip(1);
        else if (score.ev_donut == 1) synth_blip(11);
        else if (score.ev_donut == 2) synth_blip2(11, 0.75f, 0.6f);
        else if (score.ev_banked) synth_blip(2);
        else if (score.ev_lost) synth_blip(10);
        if ((score.ev_banked || score.ev_lost || score.ev_donut) && getenv("SHOT_SEQ"))
            fprintf(stderr, "event %d %s %.0f\n", frame, score.ev_donut == 1 ? "donut" : score.ev_donut == 2 ? "donut-practice" : score.ev_banked ? "banked" : "lost", score.flash_v);
        else if (car.gear != prev_gear && car.gear > 0) synth_blip(3);
        prev_gear = car.gear;
        if (in_game) {   /* build B: the achievements' rules, one step a frame */
            static int yodel_prev;
            RtNear wn = route_near(&RT, car.body.pos.x, car.body.pos.z);
            WorldStep ws = { 0 };
            ws.dt = dt; ws.running = run.state == RUN_RUNNING; ws.finished = event_ended && run.state == RUN_FINISHED;
            ws.x = car.body.pos.x; ws.z = car.body.pos.z; ws.s = wn.s; ws.lat = wn.lat; ws.drift_deg = score.angle; ws.speed = vlen(car.body.vel);
            ws.on_road = fabsf(wn.lat) < route_half_width() + T.verge_w; ws.brake = in.brake > 0;
            ws.ditch_rule = score.ditch_t > T.ditch_time || rt_last.offroad;
            ws.offroad_m = score.offroad_m; ws.donut_carpark = score.ev_donut == 2; ws.donut_road = score.ev_donut == 1;
            ws.yodel_on = yodel.active; ws.yodel_started = yodel.active && !yodel_prev; ws.yodel_left = yodel.left;
            ws.pot = score.pot; ws.banked = score.ev_banked; ws.banked_value = score.last_banked; ws.lost = score.ev_lost || lost_this_frame; ws.ford_s = RT.ford_s; ws.wood = explore ? W.tray_wood : run.wood;
            ws.explore = explore; ws.in_clearing = route_in_clearing(&RT, car.body.pos.x, car.body.pos.z); explore_creek_step(&ws, car.body.pos.x, car.body.pos.z);
            world_step(&ws);
            yodel_prev = yodel.active;
            if (ach_pop.t <= 0) { int a = world_pop(); if (a >= 0) { ach_pop.ach = a; ach_pop.t = 5.0f; synth_blip(13); if (!scripted) world_save(), web_storage_flush(); if (getenv("SHOT_SEQ")) fprintf(stderr, "achievement %s\n", ACH[a].name); } }
            if (event_ended && !scripted && W.dirty) world_save(), web_storage_flush();
        }
        score.ev_mult_up = score.ev_banked = score.ev_lost = score.ev_donut = 0;
        t_sim1 = GetTime();
        marks_rebase(car.body.pos);
        marks_upload(frame);
        if (!isfinite(car.body.pos.x + car.body.pos.y + car.body.pos.z)) {
            spawn(&car, &score);
            snprintf(status, sizeof(status), "physics blew up; car reset");
            status_time = 5;
        }

        /* ---- the car on the ground, and the chase camera ---- */
        float alpha = acc / FIXED_DT;
        V3 rpos = vlerp(prev_pos, car.body.pos, alpha);
        Q rrot = qnlerp(prev_rot, car.body.rot, alpha);
        car_ground_update(&car, dt, cam_snap);
        Q drot = car_tilt(rrot);
        if (T.body_physics != 0 && bp.init) {   /* build D: the sprung body as simulated: its height, pitch and roll; the wheels on the ground or hanging */
            cg.h = bp.y - 0.75f; for (int i = 0; i < 4; i++) cg.wh[i] = bp.wheel_y[i];
            drot = qmul(rrot, qmul(qaxis(v3(0, 0, 1), bp.roll), qaxis(v3(1, 0, 0), -bp.pitch)));
        }
        { /* ours (session 2): a little drawn lean on the flat-plane handling, so the tall ute rolls out of a
           * slide and squats and dives on throttle and brake. A sprung, lightly damped follow of the car's
           * own acceleration; the wheels stay where the handling has them. */
            static V3 v_prev; static float lr, lrv, lp, lpv;
            if (T.body_physics != 0) lp = lpv = 0;   /* build D: the body's own pitch and roll are drawn; the lean adds only the roll out of a slide */
            if (cam_snap || dt <= 0) { v_prev = car.body.vel; lr = lrv = lp = lpv = 0; }
            float h = fminf(dt, 0.05f);
            V3 la = h > 0 ? qrot(qconj(car.body.rot), vscale(vsub(car.body.vel, v_prev), 1.0f / h)) : v3(0, 0, 0); /* the car's acceleration, in its own axes */
            v_prev = car.body.vel;
            float w = 2.0f * PI * T.lean_hz, k = w * w, c = 2.0f * T.lean_damp * w;
            /* turning right (la.x > 0) the top goes left, out of the turn; pulling away (la.z > 0) the nose lifts */
            float tr = clampf(T.lean_roll * la.x, -7, 7), tp = clampf(-T.lean_pitch * la.z, -4, 4);
            lrv += (k * (tr - lr) - c * lrv) * h; lr += lrv * h;
            lpv += (k * (tp - lp) - c * lpv) * h; lp += lpv * h;
            if (T.body_physics != 0) lp = lpv = 0;
            car_lean_roll = lr; car_lean_pitch = lp;
            drot = qmul(drot, qmul(qaxis(v3(0, 0, 1), lr * DEG2RADF), qaxis(v3(1, 0, 0), lp * DEG2RADF)));
        }
        V3 dpos = v3(rpos.x, rpos.y + cg.h, rpos.z);
        if (cam_snap) { dust_clear(); flame_n = 0; }
        dust_update(&car, fminf(dt, 0.05f), in_game && run.state == RUN_RUNNING);
        flames_update(&car, fminf(dt, 0.05f), (in_game && run.state == RUN_RUNNING && yodel.active && car.throttle > 0.3f) || getenv("SHOT_YODEL") != NULL);   /* SHOT_YODEL: pictures, the fire on */
        {
            V3 f = qrot(rrot, v3(0, 0, 1)), fh = vnorm(v3(f.x, 0, f.z));
            V3 vh = v3(car.body.vel.x, 0, car.body.vel.z);
            float sp = vlen(vh);
            V3 want_dir = sp > 2.0f ? vnorm(vadd(fh, vscale(vh, 1.4f * clamp01(sp / 10.0f) / sp))) : fh; /* behind the direction of travel as much as the nose */
            if (cam_snap) cam_dir = want_dir;
            else {
                cam_dir = vnorm(vlerp(cam_dir, want_dir, clamp01(dt * T.chase_turn)));
                if (vlen(cam_dir) < 0.5f) cam_dir = want_dir;
            }
            V3 want = vadd(dpos, v3(-cam_dir.x * T.chase_dist, T.chase_height, -cam_dir.z * T.chase_dist));
            float hg = route_height(&RT, want.x, want.z, NULL) + 0.75f + 1.2f;
            if (want.y < hg) want.y = hg;
            cam_pos = cam_snap ? want : vlerp(cam_pos, want, clamp01(1.0f - expf(-dt * T.chase_follow)));
            if (cam_pos.y < hg) cam_pos.y = hg;
            cam_snap_was = cam_snap; cam_snap = 0;
        }
        V3 look_at = vadd(dpos, v3(cam_dir.x * T.chase_look, 0.9f, cam_dir.z * T.chase_look));
        {   /* build C: the time of day heads for where the fires have taken it (dusk over ten seconds); Drift stays at golden hour */
            float want = explore ? W.night : 0.0f;
            if (getenv("SHOT_NIGHT")) want = (float)atof(getenv("SHOT_NIGHT"));
            if (night_now < 0 || !in_game || (cam_snap_was && cut_t < 0)) night_now = want;
            else if (cut_t >= 0 && cut_t < CUT_FIRE) { }   /* the light holds till the fire catches */
            else { float rate = cut_t >= 0 ? 0.1f : 0.05f; night_now += clampf(want - night_now, -rate * dt, rate * dt); }
            if (getenv("SHOT_NIGHT")) night_now = want;
        }
        if (in_game && explore && scut_t >= 0) {   /* the stew's cutscene: into the tray at the deer, then round the camp oven on the table */
            if (scut_t < SCUT_CUT) { cam_pos = vadd(dpos, qrot(drot, v3(1.5f, 2.7f, -4.6f))); look_at = vadd(dpos, qrot(drot, v3(0, 0.6f, -1.8f))); }
            else {
                float a0 = atan2f(W.hut_z - oven_at.z, W.hut_x - oven_at.x), an = a0 - 0.6f + 1.3f * smooth_step01((scut_t - SCUT_CUT) / (SCUT_END - SCUT_CUT));
                float rad = 1.7f - 0.4f * smooth_step01((scut_t - SCUT_CUT) / 6.0f);
                cam_pos = v3(oven_at.x + cosf(an) * rad, oven_at.y + 0.55f, oven_at.z + sinf(an) * rad); look_at = vadd(oven_at, v3(0, -0.08f, 0));
            }
        } else if (in_game && explore && bcut_t >= 0) {   /* the charcuterie cutscene: from the hut's side of the table, then a slow pan along the board */
            V3 c1 = bcut_at(-0.6f + 0.04f * fminf(bcut_t, BCUT_DONE), 3.0f, 1.0f), t1 = bcut_at(-0.1f, 0.35f, -0.1f);
            if (bcut_t < BCUT_DONE) { cam_pos = c1; look_at = t1; }
            else {
                float u = smooth_step01((bcut_t - BCUT_DONE) / 7.0f), b = smooth_step01((bcut_t - BCUT_DONE) / 1.5f);
                cam_pos = vlerp(c1, bcut_at(-1.1f + 2.0f * u, 1.7f, 0.75f), b); look_at = vlerp(t1, bcut_at(-0.7f + 1.7f * u, 0.0f, 0.05f), b);
            }
        } else if (in_game && explore && cut_t >= 0) {   /* the wood cutscene: west of the hut, looking back east over it to the car park and the track */
            float k = cut_level >= 3 ? smooth_step01((cut_t - 8.0f) / 6.0f) : 0.0f;
            /* close on the wall while the logs stack, then back to the wide shot (the hut, the ute, the fire) as it catches */
            float pull = smooth_step01((cut_t - 4.9f) / 2.4f), gy = route_height(&RT, RT.hut_x, RT.hut_z, NULL);
            V3 close = v3(RT.hut_x - 14.5f, 0, RT.hut_z + 1.3f), wide = v3(RT.hut_x - 25.0f + 0.12f * cut_t, 0, RT.hut_z - 6.0f);
            V3 cp = vlerp(close, wide, pull); cp.y = route_height(&RT, cp.x, cp.z, NULL) + 2.0f + 1.4f * pull;
            V3 ct = chimney_top(), pile = v3(RT.hut_x - 6.3f + 0.511f * 4.0f - 0.859f * 0.6f, gy + 0.6f, RT.hut_z - 1.5f + 0.859f * 4.0f + 0.511f * 0.6f);
            V3 tg = vlerp(pile, v3(RT.hut_x - 7.0f, gy + 1.0f, RT.hut_z - 1.0f), pull);
            cam_pos = cp; look_at = vlerp(tg, v3(ct.x, ct.y + 7.0f, ct.z), k);
        } else if (in_game && explore && hutnight_t >= 0) {   /* build C: A night at the Hut (play test: a minute, to the song) */
            float t0 = hutnight_t; hutnight_t += dt;
            if (t0 < HN_FW && hutnight_t >= HN_FW) { fireworks_t = 0; fw_next = 0; }
            V3 ct = chimney_top(), fa = fire_at();
            if (hutnight_t < HN_FIRE) {   /* one slow pan along the table, close, in the lantern light: the board, and the camp oven on it */
                float u = smooth_step01(hutnight_t / HN_FIRE);
                cam_pos = bcut_at(-1.25f + 2.4f * u, 1.6f, 0.65f); look_at = bcut_at(-0.85f + 1.8f * u, -0.1f, 0.05f);
            } else if (hutnight_t < HN_UP) {   /* round the fire, the hut behind it (an arc that keeps out of the trees) */
                float t1 = HN_FIRE, an = 2.95f + 0.5f * smooth_step01((hutnight_t - t1) / (HN_UP - t1)), rad = 8.5f;
                V3 cp = v3(fa.x + cosf(an) * rad, 0, fa.z + sinf(an) * rad); cp.y = route_height(&RT, cp.x, cp.z, NULL) + 2.0f;
                cam_pos = cp; look_at = v3(fa.x, fa.y + 1.2f, fa.z);
            } else {   /* a cut to the open clearing south-east of the hut, a slow push in, a tilt up to the fireworks (the moon beside them) */
                float k = smooth_step01((hutnight_t - HN_UP) / 4.0f), push = smooth_step01((hutnight_t - HN_UP) / 10.0f);
                V3 cen = v3(ct.x, fa.y, ct.z);
                float an = -0.9f, rad = 27.0f - 3.0f * push;   /* (a sweep out from the table went through the trees) */
                V3 cp = v3(cen.x + cosf(an) * rad, 0, cen.z + sinf(an) * rad); cp.y = route_height(&RT, cp.x, cp.z, NULL) + 4.7f;
                V3 fw = v3(cen.x, fa.y + 0.7f + 16.0f * k, cen.z);
                cam_pos = cp; look_at = fw;
                if (hutnight_t >= HN_DOWN) {   /* last: a cut west of the hut (the wood cutscene's wide shot): a short tilt down from the fireworks over the chimney to the hut, the 79 in the car park and the fire */
                    float dn = 0.55f + 0.45f * smooth_step01((hutnight_t - HN_DOWN) / 3.0f), gy = route_height(&RT, RT.hut_x, RT.hut_z, NULL);
                    V3 wp = v3(RT.hut_x - 25.0f + 0.1f * (hutnight_t - HN_DOWN), 0, RT.hut_z - 6.0f); wp.y = route_height(&RT, wp.x, wp.z, NULL) + 3.4f;
                    cam_pos = wp; look_at = vlerp(v3(ct.x, ct.y + 24.0f, ct.z), v3(RT.hut_x - 7.0f, gy + 1.0f, RT.hut_z - 1.0f), dn);
                }
            }
            if (hutnight_t > HN_END) hutnight_t = -1;
        }
        Camera3D cam = { 0 };
        ro = v3(rpos.x, 0, rpos.z);
        cam.position = RO(cam_pos);
        cam.target = RO(look_at);
        if (shot_plant_on) cam.target = RO(shot_plant_at);
        if (getenv("SHOT_CAM")) { /* pictures (session 3): "e,n,up,te,tn,tup" metres from the hut, camera and target */
            float q[6] = { 0 };
            if (sscanf(getenv("SHOT_CAM"), "%f,%f,%f,%f,%f,%f", q, q + 1, q + 2, q + 3, q + 4, q + 5) == 6) {
                /* SHOT_CAMABS (session 3b, photo matching): heights from the ground at the hut, not under each point */
                float hg0 = route_height(&RT, RT.hut_x, RT.hut_z, NULL); int abs_h = getenv("SHOT_CAMABS") != NULL;
                V3 cpw = v3(RT.hut_x + q[0], (abs_h ? hg0 : route_height(&RT, RT.hut_x + q[0], RT.hut_z + q[1], NULL)) + q[2], RT.hut_z + q[1]);
                V3 ctw = v3(RT.hut_x + q[3], (abs_h ? hg0 : route_height(&RT, RT.hut_x + q[3], RT.hut_z + q[4], NULL)) + q[5], RT.hut_z + q[4]);
                cam.position = RO(cpw); cam.target = RO(ctw); cam_pos = cpw;
            }
        }
        if (getenv("SHOT_CAMS")) { /* pictures (build B): "s,lat,up,ts,tlat,tup": camera and target by distance along and off the road (+ = left), heights over the ground */
            float q[6] = { 0 };
            if (sscanf(getenv("SHOT_CAMS"), "%f,%f,%f,%f,%f,%f", q, q + 1, q + 2, q + 3, q + 4, q + 5) == 6) {
                V3 pw[2];
                for (int k = 0; k < 2; k++) {
                    float x, z, hd = route_heading_at(&RT, q[3 * k]); route_point(&RT, q[3 * k], &x, &z);
                    x -= sinf(hd) * q[3 * k + 1]; z += cosf(hd) * q[3 * k + 1];
                    pw[k] = v3(x, route_height(&RT, x, z, NULL) + q[3 * k + 2], z);
                }
                cam.position = RO(pw[0]); cam.target = RO(pw[1]); cam_pos = pw[0];
            }
        }
        if (getenv("SHOT_CAMW")) { /* pictures: "x,z,up,tx,tz,tup" in the game's own metres (the route report prints them) */
            float q[6];
            if (sscanf(getenv("SHOT_CAMW"), "%f,%f,%f,%f,%f,%f", q, q + 1, q + 2, q + 3, q + 4, q + 5) == 6) {
                V3 c0 = v3(q[0], route_height(&RT, q[0], q[1], NULL) + q[2], q[1]), c1 = v3(q[3], route_height(&RT, q[3], q[4], NULL) + q[5], q[4]);
                cam.position = RO(c0); cam.target = RO(c1); cam_pos = c0;
            }
        }
        if (shake_t > 0) {   /* build C: the thud's shake */
            float k = shake_t / 0.35f, ph = (float)GetTime() * 47.0f;
            cam.position.x += sinf(ph) * 0.18f * k; cam.position.y += sinf(ph * 1.3f + 1.0f) * 0.12f * k; cam.target.x += sinf(ph * 0.9f + 2.0f) * 0.1f * k;
            shake_t -= dt;
        }
        cam.up = (Vector3){ 0, 1, 0 };
        cam.fovy = getenv("SHOT_FOV") ? (float)atof(getenv("SHOT_FOV")) : T.chase_fov;   /* photo matching (session 3b) */
        cam.projection = CAMERA_PERSPECTIVE;

        /* ---- the scene, small ---- */
        V3 sunp; route_sun_dir(&sunp.x, &sunp.y, &sunp.z);
        Vector3 sun_d = { sunp.x, sunp.y, -sunp.z };                 /* drawing axes mirror z */
        if (getenv("SHOT_MOON")) {   /* pictures: the camera turned to the moon */
            V3 md = moon_dir();
            cam.target = (Vector3){ cam.position.x + md.x * 100.0f, cam.position.y + md.y * 100.0f, cam.position.z - md.z * 100.0f };
        }
        Matrix light_vp = MatrixIdentity();
        int shadows_on = T.shadows != 0 && rt_have && night_now < 0.6f;   /* build C: no sun shadows after dusk */
        if (in_game) fx_update(&car, &cam, fminf(dt, 0.05f));
        rt_tris_drawn = 0; rt_draw_calls = 0; rt_shadow_tris = 0;
        if (shadows_on) { /* the sun's view: an orthographic box ahead of the car, snapped to whole texels so it doesn't crawl */
            int res = (int)clampf(T.shadow_res, 256, 4096);
            shadow_init(res);
            V3 f = qrot(rrot, v3(0, 0, 1));
            float box = fmaxf(T.shadow_box, 20.0f), texel = box / (float)res;
            Vector3 C = { rpos.x + f.x * box * 0.22f, cg.h, -(rpos.z + f.z * box * 0.22f) };   /* absolute drawing axes */
            Vector3 L = sun_d, Rt = Vector3Normalize(Vector3CrossProduct((Vector3){ 0, 1, 0 }, L)), Up = Vector3CrossProduct(L, Rt);
            float ca = Vector3DotProduct(C, Rt), cb = Vector3DotProduct(C, Up), cl = Vector3DotProduct(C, L);
            ca -= fmodf(ca, texel); cb -= fmodf(cb, texel);
            Vector3 Cs = Vector3Add(Vector3Add(Vector3Scale(Rt, ca), Vector3Scale(Up, cb)), Vector3Scale(L, cl));
            Vector3 rel = { Cs.x - ro.x, Cs.y, Cs.z + ro.z };
            Camera3D lc = { 0 };
            lc.position = Vector3Add(rel, Vector3Scale(L, 420.0f)); lc.target = rel; lc.up = (Vector3){ 0, 1, 0 };
            lc.fovy = box; lc.projection = CAMERA_ORTHOGRAPHIC;
            BeginTextureMode(shadow_rt);
            rlClearScreenBuffers();
            rlSetClipPlanes(1.0, 900.0);
            BeginMode3D(lc);
            light_vp = MatrixMultiply(rlGetMatrixModelview(), rlGetMatrixProjection());
            rlDisableBackfaceCulling();
            V3 cw = v3(Cs.x, 0, -Cs.z);
            for (int i = 0; i < RT_SLOTS; i++) {
                if (rt_slot[i].index < 0) continue;
                float dx = rt_slot[i].centre.x - cw.x, dz = rt_slot[i].centre.z - cw.z;
                if (hypotf(dx, dz) > box * 0.75f + rt_slot[i].radius + 60.0f) continue;
                for (int p = RT_PART_NEARLO; p <= RT_PART_BACK; p++)
                    if (rt_slot[i].mesh[p].vertexCount) { DrawMesh(rt_slot[i].mesh[p], flat_mat, at_base(rt_slot[i].base)); rt_draw_calls++; rt_shadow_tris += rt_slot[i].tris[p]; }
            }
            for (int d = 0; d < deer_meshes; d++)   /* build B: the deer cast shadows too */
                if (deer_out(d) && deer_mesh[d][1].vertexCount && hypotf(W.deer[d].x - cw.x, W.deer[d].z - cw.z) < box) { DrawMesh(deer_mesh[d][1], flat_mat, at_base(deer_base[d])); rt_shadow_tris += deer_tris[d][1]; }
            if (W.army_placed && W.quayle == 0 && army_mesh[1].vertexCount && hypotf(W.army_x - cw.x, W.army_z - cw.z) < box) { DrawMesh(army_mesh[1], flat_mat, at_base(army_base)); rt_shadow_tris += army_tris[1]; }
            explore_draw(cw, 1, box);   /* build C: the hamper, the fire, the board, the rifle cast shadows too */
            DrawMesh(models[cur_model].mesh, flat_mat, pose_matrix(vsub(v3(rpos.x, rpos.y + cg.h, rpos.z), ro), drot));
            EndMode3D();
            EndTextureMode();
            rlEnableBackfaceCulling();
        }
        BeginTextureMode(low_rt);
        ClearBackground(sky_colour);
        {   /* the sky: gradient, sun glow and drifting clouds, from the camera's own view */
            int lw = low_rt.texture.width, lh = low_rt.texture.height;
            Vector3 F = Vector3Normalize(Vector3Subtract(cam.target, cam.position)), Rr = Vector3Normalize(Vector3CrossProduct(F, cam.up)), U = Vector3CrossProduct(Rr, F);
            float th = tanf(cam.fovy * 0.5f * DEG2RAD), res2[2] = { (float)lw, (float)lh }, tan2[2] = { th * (float)lw / (float)lh, th };
            Vector3 cw = { cam.position.x + ro.x, cam.position.y, cam.position.z - ro.z };
            float cl = T.clouds, hz3[3]; haze_colour(night_now, hz3, NULL);
            SetShaderValue(sky_sh, sk_res, res2, SHADER_UNIFORM_VEC2); SetShaderValue(sky_sh, sk_tan, tan2, SHADER_UNIFORM_VEC2);
            SetShaderValue(sky_sh, sk_f, &F, SHADER_UNIFORM_VEC3); SetShaderValue(sky_sh, sk_r, &Rr, SHADER_UNIFORM_VEC3); SetShaderValue(sky_sh, sk_u, &U, SHADER_UNIFORM_VEC3);
            SetShaderValue(sky_sh, sk_sun, &sun_d, SHADER_UNIFORM_VEC3); SetShaderValue(sky_sh, sk_camw, &cw, SHADER_UNIFORM_VEC3);
            SetShaderValue(sky_sh, sk_clouds, &cl, SHADER_UNIFORM_FLOAT); SetShaderValue(sky_sh, sk_haze, hz3, SHADER_UNIFORM_VEC3);
            float nn = night_now; SetShaderValue(sky_sh, sk_night, &nn, SHADER_UNIFORM_FLOAT);
            { V3 m = moon_dir(); Vector3 md = { m.x, m.y, -m.z }; SetShaderValue(sky_sh, sk_moon, &md, SHADER_UNIFORM_VEC3); }   /* drawing axes mirror z */
            BeginShaderMode(sky_sh);
            DrawRectangle(0, 0, lw, lh, WHITE);
            EndShaderMode();
        }
        rlDrawRenderBatchActive();
        rlSetClipPlanes(0.3f, 12000.0f);
        BeginMode3D(cam);
        rlDisableBackfaceCulling();
        {
            float hz[4] = { T.haze_near, fmaxf(T.haze_len, 1.0f), T.haze_max, 940.0f - RT.elev0 }, hc[3], hc2[3];
            haze_colour(night_now, hc, hc2);
            {   /* build C: the night's lights */
                float np[4] = { night_now, (float)GetTime(), night_now > 0.3f && in_game && T.night_lights != 0 ? 1.0f : 0.0f, 0 };
                V3 fa = fire_at(); int lv = W.fire_level - (throw_t >= 0 || (cut_t >= 0 && cut_t < CUT_FIRE) ? 1 : 0);
                float fp[4] = { 0, 0, 0, 0 };
                if (explore && lv > 0 && T.night_lights != 0) { Vector3 q = RO(vadd(fa, v3(0, 0.9f, 0))); fp[0] = q.x; fp[1] = q.y; fp[2] = q.z; fp[3] = (lv == 1 ? 1.1f : lv == 2 ? 1.6f : 2.4f) * (0.3f + 0.7f * fire_grow[lv - 1]); }
                float flp[4] = { 0, 0, 0, 0 }, flc[3] = { fw_flash_c.r / 255.0f, fw_flash_c.g / 255.0f, fw_flash_c.b / 255.0f };
                if (fw_flash > 0) { Vector3 q = RO(fw_flash_at); flp[0] = q.x; flp[1] = q.y; flp[2] = q.z; flp[3] = fw_flash * 1.4f; }
                V3 hpw = vadd(dpos, qrot(drot, v3(0, 0.6f, 2.35f))), hdw = vnorm(qrot(drot, v3(0, -0.075f, 1))), bpw = vadd(dpos, qrot(drot, v3(0, 1.58f, 0.42f))), bdw = vnorm(qrot(drot, v3(0, -0.2f, 1)));
                Vector3 hp = RO(hpw), hd = { hdw.x, hdw.y, -hdw.z }, bp = RO(bpw), bd = { bdw.x, bdw.y, -bdw.z };
                Shader S2 = scene_mat.shader;
                SetShaderValue(S2, loc_night, np, SHADER_UNIFORM_VEC4); SetShaderValue(S2, loc_fire, fp, SHADER_UNIFORM_VEC4);
                SetShaderValue(S2, loc_flash, flp, SHADER_UNIFORM_VEC4); SetShaderValue(S2, loc_flashc, flc, SHADER_UNIFORM_VEC3);
                SetShaderValue(S2, loc_headp, &hp, SHADER_UNIFORM_VEC3); SetShaderValue(S2, loc_headd, &hd, SHADER_UNIFORM_VEC3);
                SetShaderValue(S2, loc_barp, &bp, SHADER_UNIFORM_VEC3); SetShaderValue(S2, loc_bard, &bd, SHADER_UNIFORM_VEC3);
                float sink = creek_sink_now; SetShaderValue(S2, loc_sink, &sink, SHADER_UNIFORM_FLOAT);
                float lp[4] = { 0, 0, 0, 0 }, rp[4] = { 0, 0, 0, 0 };
                if (explore && lamp_mesh.vertexCount && T.night_lights != 0) { Vector3 q = RO(lamp_at); float on = smooth_step01((night_now - 0.15f) / 0.35f); lp[0] = q.x; lp[1] = q.y; lp[2] = q.z; lp[3] = 1.5f * on * (0.94f + 0.06f * sinf((float)GetTime() * 7.0f)); }
                if (flame_glow > 0.01f) { Vector3 q = RO(vadd(flame_tip, v3(0, 0.25f, 0))); rp[0] = q.x; rp[1] = q.y; rp[2] = q.z; rp[3] = flame_glow * (2.2f + 1.0f * night_now) * (0.85f + 0.15f * sinf((float)GetTime() * 23.0f) * sinf((float)GetTime() * 9.0f)); }
                SetShaderValue(S2, loc_lamp, lp, SHADER_UNIFORM_VEC4); SetShaderValue(S2, loc_rear, rp, SHADER_UNIFORM_VEC4);
            }
            Vector3 cp = cam.position, woff = { ro.x, 0, -ro.z };
            float so[4] = { shadows_on ? 1.0f : 0.0f, T.shadow_bias / 899.0f, 1.0f / fmaxf(T.shadow_res, 1.0f), 0 }, rd = T.road_detail, rw = route_half_width();
            Shader S = scene_mat.shader;
            SetShaderValue(S, loc_cam, &cp, SHADER_UNIFORM_VEC3);
            SetShaderValue(S, loc_haze, hz, SHADER_UNIFORM_VEC4);
            SetShaderValue(S, loc_hcol, hc, SHADER_UNIFORM_VEC3); SetShaderValue(S, loc_hcol2, hc2, SHADER_UNIFORM_VEC3);
            SetShaderValue(S, loc_sun, &sun_d, SHADER_UNIFORM_VEC3); SetShaderValue(S, loc_woff, &woff, SHADER_UNIFORM_VEC3);
            SetShaderValue(S, loc_shon, so, SHADER_UNIFORM_VEC4); SetShaderValue(S, loc_rdet, &rd, SHADER_UNIFORM_FLOAT); SetShaderValue(S, loc_roadw, &rw, SHADER_UNIFORM_FLOAT);
            SetShaderValueMatrix(S, loc_lvp, light_vp);
            if (shadows_on) { rlEnableShader(S.id); int slot = 10; rlActiveTextureSlot(10); rlEnableTexture(shadow_rt.depth.id); rlSetUniform(loc_smap, &slot, SHADER_UNIFORM_INT, 1); rlActiveTextureSlot(0); }
        }
        rlDisableDepthMask();
        { float nd = night_dim(); flat_mat.maps[MATERIAL_MAP_DIFFUSE].color = (Color){ (unsigned char)(255 * (0.08f + 0.92f * nd)), (unsigned char)(255 * (0.09f + 0.91f * nd)), (unsigned char)(255 * (0.16f + 0.84f * nd)), 255 };
          DrawMesh(ridge_mesh, flat_mat, MatrixTranslate(cam.position.x, cam.position.y - 60.0f, cam.position.z));
          flat_mat.maps[MATERIAL_MAP_DIFFUSE].color = WHITE; }
        rlEnableDepthMask();
        V3 camw = v3(cam_pos.x, 0, cam_pos.z);
        for (int i = 0; i < far_n; i++) {   /* the far land: tiles within reach */
            float dx = far_base[i].x + 640.0f - camw.x, dz = far_base[i].z + 640.0f - camw.z;
            if (hypotf(dx, dz) > 5200.0f) continue;
            DrawMesh(far_mesh[i], scene_mat, at_base(far_base[i])); rt_tris_drawn += far_mesh[i].triangleCount; rt_draw_calls++;
        }
        for (int i = 0; i < RT_SLOTS; i++) {
            if (rt_slot[i].index < 0) continue;
            float dc = hypotf(rt_slot[i].centre.x - camw.x, rt_slot[i].centre.z - camw.z) - rt_slot[i].line_r + TERRAIN_REACH_DRAW;
            int near = dc < T.near_dist;
            int parts[4] = { RT_PART_GROUND, near ? RT_PART_NEAR : RT_PART_NEARLO, RT_PART_BACK, RT_PART_EDGE };
            for (int k = 0; k < 4; k++) {
                int p = parts[k];
                if (!rt_slot[i].mesh[p].vertexCount) continue;
                DrawMesh(rt_slot[i].mesh[p], scene_mat, at_base(rt_slot[i].base)); rt_tris_drawn += rt_slot[i].tris[p]; rt_draw_calls++;
            }
        }
        for (int d = 0; d < deer_meshes; d++) {   /* build B: the deer not yet found; near ones in full, far ones simpler, none past 320 m */
            if (!deer_out(d)) continue;
            float dd = hypotf(W.deer[d].x - camw.x, W.deer[d].z - camw.z);
            if (dd > 320.0f) continue;
            int l = dd < 70.0f ? 0 : 1;
            if (!deer_mesh[d][l].vertexCount) continue;
            DrawMesh(deer_mesh[d][l], scene_mat, at_base(deer_base[d])); rt_tris_drawn += deer_tris[d][l]; rt_draw_calls++;
        }
        if (W.army_placed && W.quayle == 0) {   /* the army man (build C: Quayle stays once found, till he rides in the ute) */
            float dd = hypotf(W.army_x - camw.x, W.army_z - camw.z);
            int l = dd < 70.0f ? 0 : 1;
            if (dd < 320.0f && army_mesh[l].vertexCount) { DrawMesh(army_mesh[l], scene_mat, at_base(army_base)); rt_tris_drawn += army_tris[l]; rt_draw_calls++; }
        }
        explore_draw(camw, 0, 320.0f);   /* build C */
        gates_draw(camw, scene_mat, 220.0f);   /* build E */
        { /* for the phone estimate: the most drawn in a frame, and what's on the graphics chip (24 bytes a vertex; the shadow map 4 bytes a texel) */
            double mb = (double)far_tris * 72.0;
            for (int i = 0; i < RT_SLOTS; i++) if (rt_slot[i].index >= 0) for (int p = 0; p < RT_NPARTS; p++) mb += (double)rt_slot[i].tris[p] * 72.0;
            mb = mb / 1048576.0 + (shadows_on ? (double)shadow_res_now * shadow_res_now * 4.0 / 1048576.0 : 0.0);
            rt_gpu_mb = mb;
            if (frame > 60) {
                rt_tris_max = rt_tris_drawn > rt_tris_max ? rt_tris_drawn : rt_tris_max;
                rt_calls_max = rt_draw_calls > rt_calls_max ? rt_draw_calls : rt_calls_max;
                rt_shadow_max = rt_shadow_tris > rt_shadow_max ? rt_shadow_tris : rt_shadow_max;
                rt_gpu_mb_max = fmax(mb, rt_gpu_mb_max);
            }
        }
        rlDisableDepthMask();
        DrawMesh(mark_mesh, mark_fades ? mark_mat : flat_mat, at_base(mark_base));
        if (mark_fades) {
            float life = T.mark_life > 0 ? T.mark_life * (float)MARK_TICKS : 1e9f;
            float mt[3] = { (float)fmod(mark_clock * MARK_TICKS, 65536.0), life, maxf(minf(T.mark_fade, T.mark_life > 0 ? T.mark_life : T.mark_fade) * (float)MARK_TICKS, 1.0f) };
            SetShaderValue(mark_mat.shader, mark_time_loc, mt, SHADER_UNIFORM_VEC3);
        }
        V3 tright = qrot(drot, v3(1, 0, 0)), tup = qrot(drot, v3(0, 1, 0)), tfwd = qrot(drot, v3(0, 0, 1));
        if (!shadows_on) { /* shadow: the car's outline on the ground, nudged away from the low sun */
            V3 sp = vsub(v3(rpos.x + 0.45f, cg.h + 0.05f, rpos.z + 0.25f), ro);
            DrawMesh(models[cur_model].shadow, flat_mat, basis_matrix(sp, tright, tup, tfwd));
        }
        rlEnableDepthMask();
        int hide_car = getenv("SHOT_NOCAR") != NULL;   /* photo matching (session 3b): the scene without the ute */
        for (int i = 0; i < 4 && !hide_car; i++) {
            Wheel *w = &car.wheel[i];
            V3 centre = vadd(vadd(w->centre, vsub(rpos, car.body.pos)), qrot(rrot, wheel_draw_off[i]));
            centre.y += cg.wh[i];
            V3 ax = vnorm(w->right_dir);
            if (T.body_physics != 0 && bp.init) {   /* part 2: the wheels on the sprung body: they move with it (in the air they hang at full droop; over it rolls, they go with it) */
                float d = i < 2 ? car.steer_angle : 0.0f, travel = clampf(bp.s_prev[i], -BP_DROOP, BP_BUMP + 0.08f);
                V3 local = vadd(vadd(anchors[i], wheel_draw_off[i]), v3(0, travel, 0));
                centre = vadd(dpos, qrot(drot, local));
                ax = vnorm(qrot(drot, v3(cosf(d), 0, -sinf(d))));
            }
            V3 wf = vnorm(vcross(ax, tup));
            const CarModel *cm = &models[cur_model];
            if (cm->wcount[i]) { /* its own wheel: steered (ax) and turned about the axle */
                float cs = cosf(w->rotation), sn = sinf(w->rotation);
                V3 wu = vadd(vscale(tup, cs), vscale(wf, sn)), wz = vsub(vscale(wf, cs), vscale(tup, sn));
                DrawMesh(cm->wmesh[i], scene_mat, basis_matrix(vsub(centre, ro), ax, wu, wz));
            } else
                DrawMesh(wheel_mesh, scene_mat, basis_matrix(vsub(centre, ro), vscale(ax, T.wheel_width * 0.5f), vscale(tup, T.wheel_radius), vscale(wf, T.wheel_radius)));
        }
        rlEnableBackfaceCulling();
        if (!hide_car) DrawMesh(models[cur_model].mesh, scene_mat, pose_matrix(vsub(dpos, ro), drot));
        { int wn = explore ? W.tray_wood : run.wood; wn = wn < 15 ? wn : 15; if (getenv("SHOT_WOOD")) wn = atoi(getenv("SHOT_WOOD")); if (!hide_car && wn > 0 && wn <= 15) DrawMesh(wood_mesh[wn], scene_mat, pose_matrix(vsub(dpos, ro), drot)); }   /* follow-up: firewood in the tray */
        if (!hide_car && explore) {   /* build C: the esky (once there's something in it) and Quayle's rifle */
            Matrix cm = pose_matrix(vsub(dpos, ro), drot);
            int wn2 = W.tray_wood < 15 ? W.tray_wood : 15; if (getenv("SHOT_WOOD")) wn2 = atoi(getenv("SHOT_WOOD")); float pile_top = wn2 > 0 ? 0.57f + 0.165f * (float)((wn2 - 1) / 5) + 0.09f : 0.0f;   /* part 2: the rifle lies on top of any firewood */
            int nh = world_hamper_count();
            int qin = W.quayle == 1 || getenv("SHOT_QUAYLE_TRAY") != NULL;
            if (qin) DrawMesh(tray_quayle_mesh, scene_mat, cm);
            if (W.tray_deer || (scut_t >= 0 && scut_t < SCUT_CUT) || getenv("SHOT_TRAY_DEER")) DrawMesh(tray_deer_mesh, scene_mat, cm);
            if (qin) cm = MatrixMultiply(MatrixTranslate(0.34f, 0, 0), cm);   /* the esky shifts over for him */
            if (((nh > 0 && !W.board) || getenv("SHOT_ESKY")) && !(W.tray_deer || (scut_t >= 0 && scut_t < SCUT_CUT) || getenv("SHOT_TRAY_DEER"))) {
                DrawMesh(esky_mesh, scene_mat, cm);
                Color keep = scene_mat.maps[MATERIAL_MAP_DIFFUSE].color; int k = 0;
                for (int i = 0; i < HAMPER_N; i++) {
                    if (!((W.hamper >> i) & 1) && !getenv("SHOT_ESKY")) continue;
                    scene_mat.maps[MATERIAL_MAP_DIFFUSE].color = (Color){ hamper_col[i][0], hamper_col[i][1], hamper_col[i][2], 255 };
                    float px = -0.21f + 0.14f * (float)(k % 4), pz = -1.66f + 0.13f * (float)(k / 4), py = 0.48f + 0.27f + 0.02f * (float)(k % 3);
                    DrawMesh(peek_mesh, scene_mat, MatrixMultiply(MatrixTranslate(px, py, -pz), cm)); k++;
                }
                scene_mat.maps[MATERIAL_MAP_DIFFUSE].color = keep;
            }
            if (W.rifle == 1 || getenv("SHOT_TRAY_RIFLE")) DrawMesh(tray_rifle_mesh, scene_mat, MatrixMultiply(MatrixTranslate(0, fmaxf(pile_top - 0.55f, 0.0f), 0), cm));
        }
        rlDisableDepthMask();
        dust_dim = night_dim();
        dust_draw(cam);
        splinters_update(fminf(dt, 0.05f));
        splinters_draw(cam);
        flames_draw(cam);
        if (in_game) fx_draw(cam, &car, night_now);   /* build C */
        rlEnableDepthMask();
        EndMode3D();
        EndTextureMode();

        /* ---- blown up to 640x480 through the palette, then the readout ---- */
        BeginTextureMode(target);
        ClearBackground(BLACK);
        {
            float ls[2] = { (float)low_rt.texture.width, (float)low_rt.texture.height }, di = T.dither != 0 ? 0.055f : 0.0f;
            SetShaderValue(pal_sh, loc_low, ls, SHADER_UNIFORM_VEC2);
            SetShaderValue(pal_sh, loc_dith, &di, SHADER_UNIFORM_FLOAT);
            BeginShaderMode(pal_sh);
            DrawTexturePro(low_rt.texture, (Rectangle){ 0, 0, ls[0], -ls[1] }, (Rectangle){ 0, 0, VIEW_W, VIEW_H }, (Vector2){ 0, 0 }, 0, WHITE);
            EndShaderMode();
        }
        hud_batch.currentBuffer = frame % HUD_COPIES;
        if (in_game && explore && cutscene_on() && !getenv("SHOT_NOHUD")) DrawText("any button: skip", VIEW_W - 110, VIEW_H - 16, 10, (Color){ 255, 248, 230, 140 });
        if (in_game && explore && cutscene_on()) goto present;   /* the cutscene: no readout */
        if (in_game && explore && hutnight_t >= 0) {   /* build C: A night at the Hut */
            float a = clamp01((hutnight_t - 1.0f) / 1.0f) * (1.0f - clamp01((hutnight_t - 7.5f) / 1.5f));
            if (a > 0) { const char *t = "A NIGHT AT THE HUT"; DrawText(t, 320 - MeasureText(t, 40) / 2, 70, 40, (Color){ 255, 214, 140, (unsigned char)(255 * a) }); }
            if (hutnight_t > 1.0f && !getenv("SHOT_NOHUD")) DrawText("any button: skip", VIEW_W - 110, VIEW_H - 16, 10, (Color){ 255, 248, 230, 140 });
            goto present;   /* (the play test: no readout over the finale) */
        }
        if (getenv("SHOT_MARKERS")) {   /* pictures (build C follow-up): where things are, labelled on the scene (1 the hamper, 2 + the deer, Quayle, the rifle) */
            int what = atoi(getenv("SHOT_MARKERS"));
            Marker mk[64]; int nm = 0;
            #define MK(X, Z, C, ...) do { mk[nm].x = (X); mk[nm].z = (Z); mk[nm].c = (C); snprintf(mk[nm].t, sizeof(mk[nm].t), __VA_ARGS__); nm++; } while (0)
            if (what == 1) {
                for (int i = 0; i < HAMPER_N; i++) if (W.item[i].placed) MK(W.item[i].x, W.item[i].z, ((Color){ 255, 228, 120, 255 }), "%d", i + 1);
                MK(W.fire_x, W.fire_z, ((Color){ 255, 140, 60, 255 }), "F");
                MK(W.picnic_x, W.picnic_z, ((Color){ 200, 230, 255, 255 }), "T");
                if (RT.ck_n > 1) { float wx, wz; route_creek_point(&RT, RT.ck_exit_s, &wx, &wz); MK(wx, wz, ((Color){ 140, 200, 255, 255 }), "W"); }
                DrawRectangle(4, 4, 150, 14 + 11 * (HAMPER_N + 3), (Color){ 20, 16, 30, 200 });
                for (int i = 0; i < HAMPER_N; i++) DrawText(TextFormat("%2d  %s", i + 1, HAMPER_NAME[i]), 10, 10 + 11 * i, 10, (Color){ 255, 228, 120, 255 });
                DrawText(" F  the fire", 10, 10 + 11 * HAMPER_N, 10, (Color){ 255, 140, 60, 255 });
                DrawText(" T  the picnic table", 10, 21 + 11 * HAMPER_N, 10, (Color){ 200, 230, 255, 255 });
                DrawText(" W  the way in (creek)", 10, 32 + 11 * HAMPER_N, 10, (Color){ 140, 200, 255, 255 });
            } else {
                for (int d = 0; d < W.ndeer; d++) MK(W.deer[d].x, W.deer[d].z, ((Color){ 255, 160, 70, 255 }), "%d", d + 1);
                if (W.army_placed) MK(W.army_x, W.army_z, ((Color){ 150, 230, 110, 255 }), "Quayle");
                if (W.rifle_placed) MK(W.rifle_x, W.rifle_z, ((Color){ 150, 230, 110, 255 }), "rifle");
            }
            #undef MK
            for (int k = 0; k < nm; k++) {
                int keep = RT.explore; RT.explore = 1; float gy = route_height(&RT, mk[k].x, mk[k].z, NULL); RT.explore = keep;
                Vector2 sp = GetWorldToScreenEx(RO(v3(mk[k].x, gy, mk[k].z)), cam, VIEW_W, VIEW_H);
                if (sp.x < -50 || sp.y < -20 || sp.x > VIEW_W + 50 || sp.y > VIEW_H + 20) continue;
                DrawCircleV(sp, 4, BLACK); DrawCircleV(sp, 3, mk[k].c);
                DrawText(mk[k].t, (int)sp.x + 6, (int)sp.y - 5, 10, BLACK); DrawText(mk[k].t, (int)sp.x + 5, (int)sp.y - 6, 10, mk[k].c);
            }
        }
        if (getenv("SHOT_NOHUD")) goto present;   /* pictures (build B): the scene alone (the route map in docs/route) */
        float kmh = vlen(car.body.vel) * 3.6f;
        const Color ink = { 255, 248, 230, 255 }, gold = { 255, 235, 120, 255 };
        if (!explore) DrawText(commas((int)(score.total + 0.5f)), 300, 6, 20, ink);
        if (explore) draw_explore_hud(&car, frame);   /* build C */
        if (explore) trials_hud();   /* build E */
        if (!sk.on && !paused) draw_tracked(explore ? IN_EXPLORE : IN_DRIFT, GetFrameTime(), explore ? 58 : 84);   /* build E */
        if (!explore && (score.pot >= 5 || score.mult > 1)) {   /* build A: the unbanked pot, its multiplier and the count to the next step */
            int danger = score.ditch_t > 0;
            Color pc = danger ? ((frame / 6) % 2 ? (Color){ 255, 120, 60, 255 } : (Color){ 255, 200, 120, 255 }) : gold;
            const char *pt = TextFormat("+%s  x%d", commas((int)roundf(score.pot / 10.0f) * 10), score.mult);
            int pw = MeasureText(pt, 20), px = 320 - pw / 2;
            DrawText(pt, px, 28, 20, pc);
            if (score.mult < (int)T.score_mult_max) {
                DrawRectangle(px, 50, pw, 2, (Color){ 40, 36, 50, 160 });
                DrawRectangle(px, 50, (int)((float)pw * clamp01(score.held / fmaxf(T.score_mult_hold, 0.1f))), 2, pc);
            }
            if (T.score_autobank == 0) {
                const char *ht = danger ? "DITCH!" : T.score_bank_button == 0 ? "leave the road to bank it" : touch_on ? "unbanked: tap BANK" : "unbanked: B to bank";
                DrawText(ht, 320 - MeasureText(ht, 10) / 2, 55, 10, danger ? pc : (Color){ 255, 248, 230, 190 });
            }
        }
        if (rt_last.offroad && !rt_last.carpark && run.state == RUN_RUNNING && !explore) {   /* build B: why nothing is scoring; follow-up: and the countdown back to the track */
            const char *ot = T.offroad_reset > 0 ? TextFormat("OFF-ROAD  %d", (int)ceilf(fmaxf(T.offroad_reset - run.offroad_t, 0.0f))) : "OFF-ROAD";
            DrawRectangle(320 - MeasureText(ot, 20) / 2 - 6, 30, MeasureText(ot, 20) + 12, 22, (Color){ 40, 30, 20, 150 });
            DrawText(ot, 320 - MeasureText(ot, 20) / 2, 32, 20, (Color){ 230, 190, 120, 255 });
        }
        if (run.missed && run.state == RUN_RUNNING) {
            const char *mt = "CHECKPOINT MISSED: BACK UP THE ROAD";
            DrawText(mt, 320 - MeasureText(mt, 20) / 2, 66, 20, (Color){ 255, 120, 90, 255 });
        } else if (run.wrong_shown && run.state == RUN_RUNNING && !explore) {   /* build C: a hint only */
            const char *wt = "WRONG WAY";
            DrawText(wt, 320 - MeasureText(wt, 20) / 2, 66, 20, (Color){ 230, 60, 40, (unsigned char)(170 + 85 * ((frame / 20) % 2)) });
        }
        if (deer_flash_t > 0 && run.state == RUN_RUNNING) {   /* build B: a deer found */
            float a = clamp01(deer_flash_t / 1.0f), rise = fminf(3.5f - deer_flash_t, 2.4f) * 6.0f;
            const char *dt2 = TextFormat("DEER FOUND %d/%d", deer_flash_n, W.ndeer);
            DrawText(dt2, 320 - MeasureText(dt2, 30) / 2, (int)(128 - rise), 30, (Color){ 255, 170, 70, (unsigned char)(255 * a) });
            deer_flash_t -= dt;
        }
        if (army_flash_t > 0 && run.state == RUN_RUNNING) {
            float a = clamp01(army_flash_t / 1.0f), rise = fminf(3.5f - army_flash_t, 2.4f) * 6.0f;
            const char *t2 = "QUAYLE FOUND!";
            DrawText(t2, 320 - MeasureText(t2, 30) / 2, (int)(128 - rise), 30, (Color){ 150, 210, 110, (unsigned char)(255 * a) });
            army_flash_t -= dt;
        }
        if (wood_flash_t > 0 && run.state == RUN_RUNNING) {   /* follow-up: firewood */
            float a = clamp01(wood_flash_t / 0.8f), rise = fminf(3.0f - wood_flash_t, 2.2f) * 6.0f;
            int wn = explore ? W.tray_wood : run.wood;
            const char *t3 = wn >= (int)T.wood_full ? "TRAY FULL!" : TextFormat("FIREWOOD %d/%d", wn, (int)T.wood_full);
            DrawText(t3, 320 - MeasureText(t3, 30) / 2, (int)(128 - rise), 30, (Color){ 230, 170, 100, (unsigned char)(255 * a) });
            wood_flash_t -= dt;
        }
        if (ex_flash_t <= 0 && ex_next[0] && run.state == RUN_RUNNING) { ex_say2(ex_next, ex_next_sub, ex_next_c); ex_next[0] = 0; }   /* the one waiting */
        if (ex_flash_t > 0 && run.state == RUN_RUNNING) {   /* build C: Explorer's finds */
            float a = clamp01(ex_flash_t / 1.0f), rise = fminf(ex_flash_len - ex_flash_t, 2.4f) * 6.0f;
            Color c2 = ex_flash_c; c2.a = (unsigned char)(255 * a);
            DrawText(ex_flash, 320 - MeasureText(ex_flash, 30) / 2, (int)(128 - rise), 30, c2);
            if (ex_flash_sub[0]) {
                int sw = MeasureText(ex_flash_sub, 20);
                DrawRectangle(320 - sw / 2 - 8, (int)(162 - rise) - 4, sw + 16, 28, (Color){ 20, 16, 30, (unsigned char)(150 * a) });
                DrawText(ex_flash_sub, 320 - sw / 2, (int)(162 - rise), 20, (Color){ 255, 248, 230, (unsigned char)(255 * a) });
            }
            ex_flash_t -= dt;
        }
        if (ach_pop.t > 0) {   /* build B: an achievement, a short banner low on the screen */
            float a = clamp01(ach_pop.t / 0.8f) * clamp01((5.0f - ach_pop.t) / 0.2f);
            const char *at = TextFormat("ACHIEVEMENT:  %s", ACH[ach_pop.ach].name);
            int w2 = MeasureText(at, 20) + 24;
            DrawRectangle(320 - w2 / 2, 404, w2, 30, (Color){ 20, 16, 30, (unsigned char)(200 * a) });
            DrawRectangle(320 - w2 / 2, 404, w2, 2, (Color){ 255, 200, 90, (unsigned char)(255 * a) });
            DrawText(at, 320 - w2 / 2 + 12, 410, 20, (Color){ 255, 220, 140, (unsigned char)(255 * a) });
            ach_pop.t -= dt;
        }
        if (T.minimap != 0 && run.state < RUN_FINISHED) draw_route_map(8, 64, 120, 90, rt_max_s, 1, car.body.pos, 0);   /* build B: the optional map while driving (M) */
        if (yodel.active) {   /* Drift Yodel: the horns, the minute counting down, the doubled scoring */
            int bob = (int)(2.0f * sinf((float)GetTime() * 9.0f));
            draw_horns(10, 120 + bob, 3);
            DrawText(TextFormat("DRIFT YODEL  %d", (int)ceilf(fmaxf(yodel.left, 0))), 50, 126, 10, (Color){ 255, 150, 60, 255 });
            if (!explore) DrawText(TextFormat("x%g SCORE", T.yodel_score), 50, 140, 10, (Color){ 255, 235, 120, 255 });
            if (yodel.banner > 0) {
                const char *bt = "DRIFT YODEL MODE ENGAGED";
                Color bc = ((int)(GetTime() * 8) % 2) ? (Color){ 255, 120, 40, 255 } : (Color){ 255, 235, 120, 255 };
                DrawRectangle(0, 186, VIEW_W, 46, (Color){ 20, 10, 10, 170 });
                DrawText(bt, 320 - MeasureText(bt, 30) / 2, 194, 30, bc);
            }
        }
        if (score.flash_t > 0 && run.state == RUN_RUNNING) {   /* the moment it is banked or lost */
            float a = clamp01(score.flash_t / 0.6f), rise = (2.0f - score.flash_t) * 8.0f;
            const char *ft = score.flash_kind == 1 ? TextFormat("BANKED +%s", commas((int)(score.flash_v + 0.5f)))
                           : score.flash_kind == 3 ? TextFormat("DONUT +%s", commas((int)(score.flash_v + 0.5f)))
                           : score.flash_kind == 4 ? "DONUT  (car park: practice)" : score.flash_kind == 5 ? "DONUT!" : TextFormat("LOST %s", commas((int)(score.flash_v + 0.5f)));
            Color fc = score.flash_kind == 1 || score.flash_kind == 3 ? (Color){ 255, 235, 120, (unsigned char)(255 * a) }
                     : score.flash_kind == 4 || score.flash_kind == 5 ? (Color){ 200, 230, 255, (unsigned char)(255 * a) } : (Color){ 235, 120, 110, (unsigned char)(255 * a) };
            DrawText(ft, 320 - MeasureText(ft, 30) / 2, (int)(92 - rise), 30, fc);
        }
        {
            int g = car.gear - car.neutral;
            const char *gs = g < 0 ? "R" : g == 0 ? "N" : TextFormat("%d", g);
            DrawText(gs, VIEW_W - 34, 6, 30, car.rpm > T.car_limit_rpm * 0.93f ? (Color){ 230, 60, 40, 255 } : ink);
            DrawText(TextFormat("%3.0f", kmh), VIEW_W - 78, 10, 20, ink);
            DrawText(TextFormat("%s  %s", T.manual_gears != 0 ? "manual" : "auto", preset_name(preset)), VIEW_W - 78, 36, 10, ink);
            DrawRectangle(VIEW_W - 78, 48, 70, 4, (Color){ 60, 60, 60, 255 });
            DrawRectangle(VIEW_W - 78, 48, (int)(70.0f * clamp01(car.rpm / T.car_limit_rpm)), 4, car.rpm > T.car_limit_rpm * 0.93f ? (Color){ 230, 60, 40, 255 } : (Color){ 240, 240, 240, 255 });
        }
        if (in.clutch != 0) {
            DrawRectangle(VIEW_W - 78, 83, 70, 13, in.clutch != 0 ? (Color){ 240, 150, 30, 255 } : (Color){ 220, 50, 35, 255 });
            DrawText("CLUTCH IN", VIEW_W - 74, 85, 10, (Color){ 20, 20, 20, 255 });
        }
        if (rt_last.ditch && run.state == RUN_RUNNING) DrawText("DITCH", VIEW_W - 78, 100, 10, (Color){ 255, 170, 60, 255 });
        {
            if (!explore) {
                DrawText(TextFormat("%.2f / %.1f km", rt_max_s / 1000.0f, RT.bridge_s0 / 1000.0f), VIEW_W - 100, 58, 10, ink);
                DrawText(TextFormat("%d:%04.1f", (int)(run.time / 60.0f), fmodf(run.time, 60.0f)), VIEW_W - 78, 70, 10, ink);
                DrawRectangle(8, VIEW_H - 8, VIEW_W - 16, 3, (Color){ 40, 36, 50, 180 });
                DrawRectangle(8, VIEW_H - 8, (int)((float)(VIEW_W - 16) * clamp01(rt_max_s / fmaxf(RT.bridge_s0, 1.0f))), 3, gold);
            }
            if (run.state == RUN_COUNTDOWN && explore) { }
            else if (run.state == RUN_COUNTDOWN && run.countdown <= 3.0f)
                DrawText(TextFormat("%d", (int)ceilf(run.countdown)), 300, 150, 80, ink);
            else if (run.state == RUN_COUNTDOWN)
                DrawText("ALEXANDERS HUT", 205, 160, 20, ink);
            else if (run.state == RUN_RUNNING && run.time < 0.8f && !look_dir && !explore)
                DrawText("GO", 270, 150, 80, ink);
            else if (run.state >= RUN_FINISHED) {
                const Color hot = { 255, 110, 90, 255 };
                DrawRectangle(120, 100, 400, 250, (Color){ 30, 26, 36, 215 });
                /* the follow-up to build B: GAME OVER and what you hit */
                const char *what = !strcmp(rt_hit_what, "TREE") ? "a tree" : !strcmp(rt_hit_what, "LOG") ? "a log" : !strcmp(rt_hit_what, "POST") ? "a post"
                                 : !strcmp(rt_hit_what, "GATE") ? "the gate" : !strcmp(rt_hit_what, "HUT") ? "the hut" : "something hard";
                DrawText(run.state == RUN_FINISHED ? "THE BRIDGE" : "GAME OVER", 140, 108, 30, ink);
                DrawText(run.state == RUN_FINISHED ? "You made it down the mountain!" : run.state == RUN_WALL_OVER ? TextFormat("You crashed into %s!", what) : "You drove the wrong way!",
                         140, 140, 20, run.state == RUN_FINISHED ? gold : hot);
                DrawText(TextFormat("score  %s", commas((int)(score.total + 0.5f))), 140, 168, 20, gold);
                if (score.best_mult > 1) text_right(TextFormat("best x%d", score.best_mult), 500, 168, 20, gold);
                if (en_rank == 1) DrawText("NEW BEST SCORE   #1 in the top 10", 140, 192, 20, hot);
                else if (en_rank > 1) DrawText(TextFormat("NEW RECORD   #%d in the top 10", en_rank), 140, 192, 20, hot);
                if (run.state == RUN_FINISHED) DrawText(TextFormat("time        %d:%04.1f", (int)(rt_finish_time / 60.0f), fmodf(rt_finish_time, 60.0f)), 140, 216, 20, ink);
                else DrawText(TextFormat("distance    %s m of %s", commas((int)rt_max_s), commas((int)RT.bridge_s0)), 140, 216, 20, ink);
                if (en_new_far && run.state != RUN_FINISHED) text_right("FURTHEST YET", 500, 221, 10, hot);
                int bc = (int)(score.best_banked + 0.5f);
                DrawText(TextFormat("best combo  %s", bc > 0 ? commas(bc) : "--"), 140, 240, 20, ink);
                if (en_new_combo) text_right("NEW BEST COMBO", 500, 245, 10, hot);
                if (en_new_mult) text_right("HIGHEST MULTIPLIER YET", 500, 268, 10, hot);
                if (score.donuts > 0) text_right(TextFormat("donuts %d  (+%s)", score.donuts, commas((int)(score.donut_pts))), 500, 290, 10, (Color){ 255, 235, 120, 255 });
                if (run.state == RUN_WALL_OVER && crash_lost >= 5) DrawText(TextFormat("unbanked pot lost  %s", commas((int)(crash_lost + 0.5f))), 140, 266, 10, (Color){ 235, 120, 110, 255 });
                if (EREC.n > 0) DrawText(TextFormat("top score  %s   furthest  %s m", commas(EREC.top[0].score), commas((int)EREC.furthest.value)), 140, 290, 10, LIGHTGRAY);
                DrawText(TextFormat("deer this run: %d  (total %d/%d)     firewood: %d%s", W.run_found, world_found_count(), W.ndeer, run.wood, run.wood >= (int)T.wood_full ? " (tray full)" : ""), 140, 306, 10, (Color){ 255, 170, 70, 255 });
                DrawText("Select / R: again from the hut        Start / Esc: title", 140, 330, 10, LIGHTGRAY);
                draw_route_map(120, 354, 400, 116, rt_max_s, run.state != RUN_FINISHED, car.body.pos, 1);   /* build B: how far it got, and the deer found */
            }
        }
        if (tree_flash) tree_flash = 0;
        if (paused && (map_view || getenv("SHOT_MAPVIEW"))) {   /* play test: the whole map, the creek on it as just a creek */
            DrawRectangle(0, 0, VIEW_W, VIEW_H, (Color){ 20, 16, 30, 240 });
            DrawText("MAP", 320 - MeasureText("MAP", 20) / 2, 14, 20, RAYWHITE);
            map_creek_all = 1; draw_route_map(30, 40, 580, 410, rt_max_s, 1, car.body.pos, 1); map_creek_all = 0;
            DrawText("any button: back", VIEW_W - 110, VIEW_H - 16, 10, (Color){ 255, 248, 230, 160 });
        } else if (paused) draw_pause(pause_sel);   /* build C */
        skill_draw();   /* build E */
        if (show_readout && !look_dir) {
            int y = 6;
            const Color col = { 255, 248, 230, 255 };
            V3 lv = car_local_velocity(&car);
            float body_slip = (fabsf(lv.z) + fabsf(lv.x) > 1.0f) ? atan2f(lv.x, fabsf(lv.z)) * RAD2DEGF : 0;
            RtNear nn = route_near(&RT, car.body.pos.x, car.body.pos.z);
            DrawText(TextFormat("%3.0f km/h  gear %d  %4.0f rpm  slip %5.1f  sideways %4.1f m/s%s", kmh, car.gear - car.neutral, car.rpm, body_slip, lv.x, score.drifting ? "  DRIFT" : ""), 6, y, 10, col); y += 11;
            DrawText(TextFormat("road km %.3f  %+.1f m from the middle  grade %+.1f%%  height %.0f m  ditch %d rough %d", nn.s / 1000, nn.lat, rt_last.grade * 100, cg.h + RT.elev0, rt_last.ditch, rt_last.rough), 6, y, 10, col); y += 11;
            DrawText(TextFormat("pot %d  x%d  total %d  drift angle %4.1f  %3.0f a second  ditch %.2f s%s", (int)score.pot, score.mult, (int)score.total, score.angle, score.rate, score.ditch_t,
                                T.score_autobank != 0 ? TextFormat("  banks in %3.1f s", score.pot > 0 ? maxf(T.score_bank_after - score.since_slide, 0) : 0.0f) : ""), 6, y, 10, col); y += 11;
            DrawText(pad >= 0 ? TextFormat("pad: %s", GetGamepadName(pad)) : "pad: none found, keyboard only (F3 = try next device)", 6, y, 10, col); y += 11;
            RtStreamStats ss = route_stream_stats();
            int nslot = 0; for (int i = 0; i < RT_SLOTS; i++) nslot += rt_slot[i].index >= 0;
            DrawText(TextFormat("pixels %d x %d (P)  %d fps  chunks %d in play, %d triangles, %d draws  built %ld (worst %.1f ms)  upload worst %.2f ms", low_rt.texture.width, low_rt.texture.height, GetFPS(), nslot, rt_tris_drawn, rt_draw_calls, ss.built, ss.ms_max, rt_upload_ms_max), 6, y, 10, col); y += 11;
            DrawText(TextFormat("worst frame %4.1f ms  sim %4.2f ms  drawing %4.2f ms  slow frames %d", show_worst, show_sim, show_draw, st_slow), 6, y, 10, col); y += 11;
            DrawText(TextFormat("tune [ ] : %s = %g   (- = change, F5 reload, F6 save)", tuning_name(sel), *tuning_ptr(sel)), 6, VIEW_H - 28, 10, col);
            DrawText("go X/A  brake Z/B  handbrake Space/R1  clutch LShift/L1  gears S,A  bank B/RShift/L2  P pixels  H handling  F1 readout  R reset  Esc title", 6, VIEW_H - 18, 10, col);
        }
        if (status_time > 0) { DrawText(status, 6, VIEW_H - 44, 10, (Color){ 255, 200, 120, 255 }); status_time -= dt; }
    present:
        rlDrawRenderBatchActive();
        st_sim += t_sim1 - t_sim0;
        st_draw += GetTime() - t_sim1;
        if (st_window == 0 && st_frames > 0) { show_sim = (float)(st_sim / st_frames) * 1000.0f; show_draw = (float)(st_draw / st_frames) * 1000.0f; }
        EndTextureMode();

        if (look_dir && look_wait > 20 && route_stream_pending() == 0 && screen == SCR_GAME) {
            Image img = LoadImageFromTexture(target.texture);
            ImageFlipVertical(&img);
            ExportImage(img, TextFormat("%s/look-%d-%s-%d.png", look_dir, look_i + 1, looks[look_i].name, pix_w[low_mode]));
            UnloadImage(img);
            look_wait = 0;
            if (++look_mode == NPIX) { look_mode = 0; if (++look_i == 4) quit = 1; }
        }
        /* SHOT_SEQ=dir (session 3b): a recording. Every frame to dir/NNNNN.png ("-" = log only), slide and speed to
         * stderr, stop after SHOT_SEQ_N frames (default 1200, 20 s). Use with --shot and a large frame count. */
        if (getenv("SHOT_SEQ")) {
            const char *dir = getenv("SHOT_SEQ"); int nmax = getenv("SHOT_SEQ_N") ? atoi(getenv("SHOT_SEQ_N")) : 1200;
            if (strcmp(dir, "-")) {
                Image img = LoadImageFromTexture(target.texture); ImageFlipVertical(&img);
                ExportImage(img, TextFormat("%s/%05d.png", dir, frame)); UnloadImage(img);
            }
            if (frame % 6 == 0) { RtNear nn = route_near(&RT, car.body.pos.x, car.body.pos.z);
                fprintf(stderr, "seq %5d t %5.1f s %6.1f lat %+5.1f v %5.1f km/h slide %4.0f state %d flames %d yodel %d thr %.2f\n", frame, frame / 60.0f, nn.s, nn.lat, vlen(car.body.vel) * 3.6f, car.slide_deg, run.state, flame_n, yodel.active, car.throttle); }
            if (frame >= nmax) break;
        }
        if (getenv("SHOT_AT_END") && screen == SCR_GAME) {   /* pictures (build B): the results, half a second after the run ends */
            static int over_frames;
            over_frames = run.state >= RUN_FINISHED ? over_frames + 1 : 0;
            if (over_frames == 30) { Image img = LoadImageFromTexture(target.texture); ImageFlipVertical(&img); ExportImage(img, getenv("SHOT_AT_END")); UnloadImage(img); break; }
        }
        if (getenv("SHOT_EXTRA")) {   /* build D: more pictures on the way, "frame:path,frame:path" */
            char buf[1024]; snprintf(buf, sizeof(buf), "%s", getenv("SHOT_EXTRA"));
            for (char *tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
                char *c = strchr(tok, ':'); if (!c) continue; *c = 0;
                if (atoi(tok) == frame) { Image img = LoadImageFromTexture(target.texture); ImageFlipVertical(&img); ExportImage(img, c + 1); UnloadImage(img); }
            }
        }
        /* SHOT_SLIDE=deg (ours, session 2): wait past the frame count until the car is sliding at least this much */
        if (shot_path && frame >= shot_frames && (!getenv("SHOT_SLIDE") || car.slide_deg >= strtof(getenv("SHOT_SLIDE"), NULL) || frame > shot_frames + 3000)) {
            Image img = LoadImageFromTexture(target.texture);
            ImageFlipVertical(&img);
            ExportImage(img, shot_path);
            if (getenv("SHOT_SLIDE")) { V3 lv = car_local_velocity(&car); fprintf(stderr, "shot: slide %.0f deg, %d dust, lean roll %.1f pitch %.1f deg, sideways velocity %.1f, yaw rate %.2f\n", car.slide_deg, dust_n, car_lean_roll, car_lean_pitch, lv.x, car.body.angvel.y); }
            UnloadImage(img);
            break;
        }
        BeginDrawing();
#ifdef BD_IOS
        bd_ios_screen_viewport();
#endif
        ClearBackground(BLACK);
        float sw = (float)GetScreenWidth(), sh = (float)GetScreenHeight();
        float sc = minf(sw / VIEW_W, sh / VIEW_H);
        Rectangle dst = { (sw - VIEW_W * sc) * 0.5f, (sh - VIEW_H * sc) * 0.5f, VIEW_W * sc, VIEW_H * sc };
        DrawTexturePro(target.texture, (Rectangle){ 0, 0, VIEW_W, -VIEW_H }, dst, (Vector2){ 0, 0 }, 0, WHITE);
        touch_draw(screen == SCR_GAME);
#ifdef BD_IOS
        {   /* the iPhone play test: frame times in the corner, for testers' screenshots */
            float fm, fw, fwa; int fs;
            bd_ios_frame_stats(&fm, &fw, &fwa, &fs);
            if (fm > 0) DrawText(TextFormat("%.0f fps  worst %.0f ms  slow %d", 1000.0f / fm, fwa, fs),
                                 (int)(dst.x + 6), (int)(dst.y + 6), 10, (Color){ 255, 255, 255, 170 });
        }
#endif
        EndDrawing();
        if (getenv("SHOT_WINDOW") && frame == atoi(getenv("SHOT_WINDOW_AT") ? getenv("SHOT_WINDOW_AT") : "90")) { TakeScreenshot(getenv("SHOT_WINDOW")); quit = 1; }   /* tests: the whole window, buttons and all */
    }
    if (st_frames > 0) {
        double secs = GetTime() - st_t0;
        long hwm = 0, rss = 0;
        FILE *sf = fopen("/proc/self/status", "r");
        if (sf) { char ln[128]; while (fgets(ln, sizeof(ln), sf)) { sscanf(ln, "VmHWM: %ld", &hwm); sscanf(ln, "VmRSS: %ld", &rss); } fclose(sf); }
        printf("STATS frames %d in %.1f s (%.1f fps), mean frame %.2f ms, worst frame %.1f ms, frames over 20 ms: %d, simulation %.2f ms, drawing (processor side) %.2f ms per frame, memory now %.1f MB, peak %.1f MB\n",
               st_frames, secs, st_frames / secs, frame_ms_sum / st_frames, st_worst_all * 1000.0, st_slow, st_sim / st_frames * 1000.0, st_draw / st_frames * 1000.0, rss / 1024.0, hwm / 1024.0);
        if (frame_ms_n > 10) {
            for (int i = 0; i < frame_ms_n; i++) for (int j = i + 1; j < frame_ms_n; j++) if (frame_ms[j] < frame_ms[i]) { float t = frame_ms[i]; frame_ms[i] = frame_ms[j]; frame_ms[j] = t; }
            printf("STATS frame ms: median %.2f, 95th %.2f, 99th %.2f, worst %.2f; pixels %d wide; distance %.0f m\n", frame_ms[frame_ms_n / 2], frame_ms[frame_ms_n * 95 / 100], frame_ms[frame_ms_n * 99 / 100], frame_ms[frame_ms_n - 1], pix_w[low_mode], rt_max_s);
        }
        struct timespec cpu;
        clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &cpu);
        printf("STATS processor time used: %.1f s of %.1f s = %.0f%% of one core\n", cpu.tv_sec + cpu.tv_nsec / 1e9, secs, (cpu.tv_sec + cpu.tv_nsec / 1e9) / secs * 100.0);
        RtStreamStats ss = route_stream_stats();
        printf("STATS road: %ld chunks built on the worker (%.1f ms each, worst %.1f), upload worst %.2f ms, %d triangles drawn last frame\n",
               ss.built, ss.built ? ss.ms_sum / ss.built : 0.0, ss.ms_max, rt_upload_ms_max, rt_tris_drawn);
        printf("STATS budget: most in a frame %d triangles in the scene + %d in the shadow map (+ the car's 4,830 in each), %d draw calls; scene meshes on the graphics chip up to %.1f MB (now %.1f) incl. far land %.1f MB and shadow map %.0f MB\n",
               rt_tris_max, rt_shadow_max, rt_calls_max + 8, rt_gpu_mb_max, rt_gpu_mb, far_tris * 72.0 / 1048576.0, shadow_res_now * (double)shadow_res_now * 4.0 / 1048576.0);
    }
    route_stream_stop();
    erecords_finish();
    rlSetRenderBatchActive(NULL);
    rlUnloadRenderBatch(hud_batch);
    if (snd.ready) CloseAudioDevice();
    UnloadRenderTexture(target);
    CloseWindow();
    return 0;
}
