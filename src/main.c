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

#define GAME_VERSION "play test 2"
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

typedef struct { V3 p[3]; unsigned char r, g, b; } Tri;
static Tri *tris;
static int ntris;
static const char *model_note = "box (no car file)";

static void push_tri(V3 a, V3 b, V3 c, float r, float g, float bl)
{
    tris = realloc(tris, sizeof(Tri) * (size_t)(ntris + 1));
    V3 n = vnorm(vcross(vsub(b, a), vsub(c, a)));
    float shade = 0.62f + 0.45f * maxf(vdot(n, vnorm(v3(0.55f, 0.65f, 0.5f))), 0); /* bright enough: the palette step darkens mid greys */
    Tri *t = &tris[ntris++];
    t->p[0] = a; t->p[1] = b; t->p[2] = c;
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
                 float offroad_t; int wood; } Run;   /* follow-up: seconds off the road (15 puts you back on the track); logs run over */   /* build B: checkpoints passed in order; metres driven backwards */
static void run_reset(Run *r, int with_countdown)
{
    memset(r, 0, sizeof(*r));
    r->state = with_countdown ? RUN_COUNTDOWN : RUN_RUNNING;
    r->countdown = with_countdown ? 3.8f : 0.0f;
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
            cp[0] = t->r; cp[1] = t->g; cp[2] = t->b; cp[3] = CAR_ALPHA;
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

typedef struct { int ditch, rough, deep, tree; float tree_speed; float grade; float edge; int offroad, held, carpark, log_broken; float log_x, log_z; } RtPhys;   /* carpark: off the road but no tag or timer; log_broken: a log splintered this step, here */   /* build B: how far into the map's edge; off the road; held back by the edge */   /* deep: wheels ditch_margin or more past the verge (scoring) */
static RtPhys rt_last;          /* this frame's last physics step, for the readout */
static int bot_ghost;           /* build B, the shortcut test (BOT_GHOST=1): trunks don't stop the car, so only the ground and the edge do */

/* ---- build B: the deer (src/world.c places them and keeps the finds); here they are drawn and found ---- */
static Mesh deer_mesh[WORLD_DEER][2]; static V3 deer_base[WORLD_DEER]; static int deer_tris[WORLD_DEER][2], deer_meshes;
static Mesh army_mesh[2]; static V3 army_base; static int army_tris[2];   /* the follow-up: the toy army man */
static uint32_t deer_seed_now;
static void deer_place(int fresh_seed)
{
    deer_seed_now = fresh_seed ? (uint32_t)time(NULL) * 2654435761u + (uint32_t)rt_runs : (uint32_t)T.deer_seed;
    world_place_deer(&RT, deer_seed_now);
}
static Mesh rt_upload_part(RtPart *P);
static void deer_meshes_free(void)
{
    for (int d = 0; d < WORLD_DEER; d++) for (int l = 0; l < 2; l++) if (deer_mesh[d][l].vertexCount) UnloadMesh(deer_mesh[d][l]);
    memset(deer_mesh, 0, sizeof(deer_mesh)); deer_meshes = 0;
}
static void deer_meshes_init(void)   /* each deer as two small meshes (near and far), built where it stands */
{
    deer_meshes_free();
    for (int d = 0; d < W.ndeer; d++)
        for (int l = 0; l < 2; l++) {
            RtGeom g;
            route_build_model(&RT, route_template_find("sambar", W.deer[d].variant, l), W.deer[d].x, W.deer[d].z, W.deer[d].yaw, 1.0f, &g);
            deer_mesh[d][l] = rt_upload_part(&g.part[RT_PART_GROUND]); deer_tris[d][l] = g.part[RT_PART_GROUND].nverts / 3;
            deer_base[d] = v3(g.base_x, 0, g.base_z);
            route_geom_free(&g);
        }
    deer_meshes = W.ndeer;
    for (int l = 0; l < 2; l++) {
        if (army_mesh[l].vertexCount) UnloadMesh(army_mesh[l]);
        memset(&army_mesh[l], 0, sizeof(Mesh)); army_tris[l] = 0;
        if (!W.army_placed) continue;
        RtGeom g;
        route_build_model(&RT, route_template_find("armyman", 0, l), W.army_x, W.army_z, W.army_yaw, 1.0f, &g);
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
    c->body.vel.x -= g * gx * FIXED_DT;
    c->body.vel.z -= g * gz * FIXED_DT;
    V3 f = qrot(c->body.rot, v3(0, 0, 1));
    ph->grade = gx * f.x + gz * f.z;
    /* wheels past the verge: the ditch, and beyond it rough ground */
    for (int i = 0; i < 4; i++) {
        int s;
        route_height(&RT, c->wheel[i].hit_point.x, c->wheel[i].hit_point.z, &s);
        if (s == SURF_DITCH) ph->ditch++;
        else if (s == SURF_ROUGH) ph->rough++;
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
    if (t >= 0 && RT.tree[t].kind == 5) {   /* follow-up: a log splinters (firewood) and the ute carries on, a little slower */
        ph->log_x = RT.tree[t].x; ph->log_z = RT.tree[t].z;
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
    if (r->back_m < 30.0f) {
        if (r->dir != DIR_CORRECT) { r->dir = DIR_CORRECT; r->wrong_shown = 0; r->wrong_left = T.wrong_way_time; }
    } else if (r->dir == DIR_CORRECT) { r->dir = DIR_WRONG; r->wrong_shown = 1; }
    if (r->wrong_shown) {
        r->wrong_left -= FIXED_DT;
        if (r->wrong_left <= 1.0f) { r->state = RUN_WRONG_WAY_OVER; return; }
    }
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
    if (r->state != RUN_RUNNING) { r->offroad_t = 0; return 0; }
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
        if (!carpark || T.donut_carpark != 0) { float b = T.donut_bonus * score_boost; s->total += b; s->donut_pts += b; s->donuts++; s->ev_donut = 1; s->flash_v = b; s->flash_kind = 3; }
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
static void dust_draw(Camera3D cam)
{
    if (!dust_n) return;
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
            rlColor4ub(232, 212, 170, al); rlVertex3f(c.x, c.y, c.z);
            rlColor4ub(226, 204, 160, al / 2); rlVertex3f(p1.x, p1.y, p1.z);
            rlColor4ub(226, 204, 160, al / 2); rlVertex3f(p0.x, p0.y, p0.z);
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
        float z0 = -2.78f + 0.12f * j, z1 = -1.36f - 0.1f * j, r = 0.085f + 0.015f * j;
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
    static const float notes[14][4] = { { 0 }, { 660, 880, 1320 }, { 523, 659, 784, 1047 }, { 190, 120 }, { 95, 60, 45 }, { 990 }, { 660, 990 }, { 70, 150, 310 },
                                        { 95, 523, 784, 1047 }, { 70, 196, 147 }, { 196, 147 }, { 1047, 784, 1047, 1319 },   /* 11: a donut */
                                        { 587, 880, 1175 }, { 784, 784, 1047, 1568 } };
    static const float note_len[14] = { 0, 0.06f, 0.07f, 0.03f, 0.05f, 0.05f, 0.09f, 0.025f, 0.07f, 0.13f, 0.14f, 0.06f, 0.11f, 0.08f };
    static const int note_n[14] = { 0, 3, 4, 2, 3, 1, 2, 3, 4, 3, 2, 4, 3, 4 };
    static const float note_gain[14] = { 0, 1, 0.9f, 1, 1, 1, 1, 1, 0.9f, 0.55f, 0.55f, 0.9f, 0.8f, 0.75f };
    static const int note_soft[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0 };   /* triangle wave with a decay on each note, not a square (the lost notes) */
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
        float s = (eng * syn.lv_engine + tyre + blip * syn.lv_blip) * syn.volume;
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
    snd.yodel = LoadMusicStream("assets/sfx/drift_yodel.ogg"); snd.has_yodel = snd.yodel.frameCount > 0;
    if (snd.has_yodel) snd.yodel.looping = true;
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
    if (syn.ready) { syn.rpm_norm = 0; syn.throttle = 0; syn.skid = 0; syn.volume = T.synth_volume; syn.lv_engine = 0; syn.lv_tyre = 0; syn.lv_blip = T.synth_blip; }
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
static void sound_yodel(float dt)   /* every frame: the song while the mode is on, a short fade when it ends */
{
    if (!snd.ready || !snd.has_yodel) return;
    if (!yodel.active && snd.yodel_fade > 0) { snd.yodel_fade -= dt / 0.8f; if (snd.yodel_fade <= 0) { snd.yodel_fade = 0; StopMusicStream(snd.yodel); } }
    snd_duck = 1.0f - (1.0f - T.yodel_duck) * snd.yodel_fade;
    if (!IsMusicStreamPlaying(snd.yodel)) { snd_duck = 1.0f; return; }
    SetMusicVolume(snd.yodel, clamp01(T.snd_music * snd.yodel_fade));
    UpdateMusicStream(snd.yodel);
}

/* exhaust flames: short-lived hot puffs from the tailpipe (left rear, under the tray) while the throttle is open */
#define FLAME_MAX 160
static struct { V3 p, v; float age, life, size; } flame[FLAME_MAX];
static int flame_n; static float flame_owed;
static void flames_update(const Car *c, float dt, int on)
{
    for (int i = 0; i < flame_n; ) {
        flame[i].age += dt;
        if (flame[i].age >= flame[i].life) { flame[i] = flame[--flame_n]; continue; }
        flame[i].p = vadd(flame[i].p, vscale(flame[i].v, dt));
        flame[i].v.y += 1.5f * dt;
        i++;
    }
    if (!on || dt <= 0) { flame_owed = 0; return; }
    float rate = 25.0f + 70.0f * clamp01(c->throttle) + (c->changing_gear ? 120.0f : 0.0f);
    flame_owed += rate * dt;
    float rear = car_outline.own ? car_outline.rear : T.body_rear;
    V3 tip = vadd(c->body.pos, qrot(c->body.rot, v3(-0.35f, 0, rear - 0.1f)));
    tip.y = route_height(&RT, tip.x, tip.z, NULL) + 0.55f;
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
    route_logs_restore(&RT, route_stream_invalidate);   /* follow-up: the logs broken last run are back */
    splinter_n = 0;
    wood_flash_t = deer_flash_t = army_flash_t = 0;
    if (T.deer_random != 0 && rt_runs > 1) { deer_place(1); W.found = 0; deer_meshes_init(); }   /* new spots every run: the count is this run's */
}
static void spawn(Car *c, Score *s) { spawn_ex(c, s, 0); }

/* --routetest: no window. The route as the game reads it: length, heights, grades and bends from its
 * own centreline; whether the ground function agrees with the road along the middle; trees; and the
 * time and size of every chunk's geometry. */
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
typedef struct { int state; float time, dist, top, mean, ditch_s, rough_s, score; float end_s, end_lat, hit_speed; int best_x; float old_score; int old_x, lost_ditch, lost_crash; float lost_pts, max_lat; int cp, ncp; float offroad_m; int deer, ach, finished_seen, resets, wood; } RtRun;
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
    while (r.state == RUN_RUNNING && steps < (int)(1200.0f / FIXED_DT)) {
        CarInput in;
        if (bot_donut) { memset(&in, 0, sizeof(in)); if (r.time > 0.3f) { in.steer = -(float)bot_donut; in.throttle = 1; } else in.brake = 1; }
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
        car_step(&car, &in);
        RtPhys ph;
        rt_physics(&car, &ph);
        rt_rules(&r, &car, &ph, &max_s, 1);
        if (rt_offroad_timer(&r, &car, &ph)) { res.resets++; if (getenv("BOT_EVENTS")) printf("   back on the track after %.0f s off it, at %.1f s, km %.2f\n", T.offroad_reset, r.time, route_near(&RT, car.body.pos.x, car.body.pos.z).s / 1000); }
        if (ph.log_broken && r.state == RUN_RUNNING) { r.wood++; if (getenv("BOT_EVENTS")) printf("   firewood %d at %.1f s, km %.2f\n", r.wood, r.time, max_s / 1000); }
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
            ws.pot = sc.pot; ws.banked = sc.ev_banked; ws.banked_value = sc.last_banked; ws.lost = sc.ev_lost; ws.ford_s = RT.ford_s; ws.wood = r.wood;
            { static float ps = -1e9f; if (r.time < FIXED_DT * 1.5f) ps = -1e9f;
              if (getenv("BOT_EVENTS") && ps < RT.ford_s && wn.s >= RT.ford_s) printf("   the ford at %.0f km/h, rear slide %.0f deg, %+.1f m off the middle\n", ws.speed * 3.6f, sc.angle, wn.lat);
              ps = wn.s; }
            world_step(&ws);
            if (ws.finished) res.finished_seen = 1;
            for (int a; (a = world_pop()) >= 0; ) { res.ach |= 1 << a; if (getenv("BOT_EVENTS")) printf("   achievement: %s at %.1f s\n", ACH[a].name, r.time); }
            sc.ev_banked = sc.ev_donut = 0;
        }
        if (sc.ev_lost) { sc.ev_lost = 0; res.lost_ditch++; res.lost_pts += sc.flash_v; sc.ev_lost = 0; if (getenv("BOT_EVENTS")) printf("   lost %.0f in the ditch at %.1f s, km %.2f\n", sc.flash_v, r.time, max_s / 1000); }
        if (r.state == RUN_WALL_OVER) { if (sc.pot > 0) { res.lost_crash++; res.lost_pts += score_combo(&sc); } score_lose(&sc); os.pot = 0; }
        float sp = vlen(car.body.vel);
        sp_sum += sp; if (sp > res.top) res.top = sp;
        if (ph.ditch) res.ditch_s += FIXED_DT;
        if (ph.rough) res.rough_s += FIXED_DT;
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
        for (int a; (a = world_pop()) >= 0; ) res.ach |= 1 << a;
    }
    return res;
}

static int drivetest(int rows)
{
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
        if (r.deer || r.ach) {
            printf("      deer found %d (%d of %d on the device); achievements earned:", r.deer, world_found_count(), W.ndeer);
            for (int a = 0; a < ACH_N; a++) if ((r.ach >> a) & 1) printf(" [%s]", ACH[a].name);
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
enum { MENU_DRIVE, MENU_RECORDS, MENU_ACH, MENU_QUIT, MENU_N };
#if defined(__EMSCRIPTEN__) || defined(BD_PHONE)
#define MENU_SHOWN 3   /* the browser and phone builds: no Quit */
#else
#define MENU_SHOWN MENU_N
#endif
static const char *test_label; /* --label TEXT: shown in red under the version on the title screen */
static const Color menu_ink = { 24, 28, 32, 255 }, menu_red = { 225, 45, 45, 255 }, menu_dim = { 70, 90, 100, 255 };

static void text_centred(const char *t, int y, int size, Color c) { DrawText(t, (VIEW_W - MeasureText(t, size)) / 2, y, size, c); }
static void text_right(const char *t, int x_right, int y, int size, Color c) { DrawText(t, x_right - MeasureText(t, size), y, size, c); }

static void draw_title(int sel)
{
    const char *item[MENU_N] = { "Drive", "Records", "Achievements", "Quit" };
    text_centred("BUSH DRIFT", 64, 60, menu_ink);
    DrawRectangle(92, 130, 456, 4, menu_red);
    text_centred(GAME_VERSION, 144, 20, menu_red);
    /* a test build says what it is (--label), so it can be told from the plain game */
    if (test_label) text_centred(TextFormat("TEST: %s", test_label), 170, 10, menu_red);
    for (int i = 0; i < MENU_SHOWN; i++) {
        int y = 200 + 44 * i, off = i == MENU_DRIVE && !rt_have;
        if (i == sel) { DrawRectangle(176, y - 5, 312, 38, menu_ink); DrawText(">", 188, y, 30, menu_red); }
        DrawText(item[i], 236, y, 30, i == sel ? RAYWHITE : (off ? menu_dim : menu_ink));
    }
    text_centred("Alexanders Hut to the Tantawangalo Creek bridge.  A tree ends the run; the ditch slows you.", 400, 10, menu_dim);
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

/* Build B: the achievements (src/world.c). Not yet earned: ??? (a few are named up front: ACH[].shown). */
static void draw_achievements(void)
{
    int got = 0; for (int a = 0; a < ACH_N; a++) got += W.earned[a];
    text_centred("ACHIEVEMENTS", 22, 30, menu_ink);
    text_centred(TextFormat("%d of %d     deer found %d of %d", got, ACH_N, world_found_count(), W.ndeer ? W.ndeer : WORLD_DEER), 58, 20, menu_red);
    DrawRectangle(40, 84, 560, 2, menu_dim);
    for (int a = 0; a < ACH_N; a++) {   /* 15 rows of 25 px: the name, the how in small under it */
        int y = 89 + 25 * a;
        if (W.earned[a] || ACH[a].shown) {
            DrawText(ACH[a].name, 60, y, 20, W.earned[a] ? menu_ink : menu_dim);
            DrawText(ACH[a].how, 72, y + 16, 10, menu_dim);
            if (W.earned[a]) text_right(records_date_text(W.date[a]), 590, y + 5, 10, menu_red);
        } else DrawText("???", 60, y + 3, 20, menu_dim);
    }
    DrawText("B: back", 20, 10, 10, menu_dim);
}

/* Build B: the whole route from above, fitted in a box: the road, how far this run got (gold), the hut, the ford and
 * the bridge, and the deer found on this device (orange dots). Lines only, so it costs next to nothing. */
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
    for (int d = 0; d < W.ndeer; d++) if ((W.found >> d) & 1) DrawCircleV((Vector2){ MAPX(W.deer[d].x), MAPY(W.deer[d].z) }, 3.0f, (Color){ 255, 140, 40, 255 });
    float fx, fz; route_point(&RT, RT.ford_s, &fx, &fz);
    DrawCircleV((Vector2){ MAPX(RT.x[0]), MAPY(RT.z[0]) }, 2.5f, (Color){ 120, 220, 120, 255 });
    DrawCircleV((Vector2){ MAPX(fx), MAPY(fz) }, 2.0f, (Color){ 120, 190, 255, 255 });
    DrawCircleV((Vector2){ MAPX(RT.x[RT.n - 1]), MAPY(RT.z[RT.n - 1]) }, 2.5f, (Color){ 235, 90, 70, 255 });
    if (car_dot) DrawCircleV((Vector2){ MAPX(car_at.x), MAPY(car_at.z) }, 2.5f, RAYWHITE);
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
static Material scene_mat;
static Shader pal_sh, sky_sh;
static int loc_cam, loc_haze, loc_hcol, loc_pal, loc_low, loc_dith;
static int loc_hcol2, loc_sun, loc_lvp, loc_smap, loc_shon, loc_woff, loc_rdet, loc_roadw;
static int sk_res, sk_f, sk_r, sk_u, sk_tan, sk_sun, sk_camw, sk_clouds, sk_haze;
static Mesh ridge_mesh;
#define NPAL 52
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
#define GLSL_NOISE \
    "float h21(vec2 p){ p = fract(p*vec2(123.34, 456.21)); p += dot(p, p + 45.32); return fract(p.x*p.y); }\n" \
    "float vn(vec2 p){ vec2 i = floor(p), f = fract(p); f = f*f*(3.0 - 2.0*f);\n" \
    " return mix(mix(h21(i), h21(i + vec2(1.0, 0.0)), f.x), mix(h21(i + vec2(0.0, 1.0)), h21(i + vec2(1.0, 1.0)), f.x), f.y); }\n"

static const char *scene_vs = GLSL_VS
    "IN vec3 vertexPosition; IN vec2 vertexTexCoord; IN vec4 vertexColor;\n"
    "uniform mat4 mvp; uniform mat4 matModel; uniform mat4 lightVP;\n"
    "OUT vec4 fragColor; OUT vec3 wpos; OUT vec2 uv; OUT vec4 lpos;\n"
    "void main(){ vec4 wp = matModel*vec4(vertexPosition, 1.0); wpos = wp.xyz; uv = vertexTexCoord; fragColor = vertexColor;\n"
    " lpos = lightVP*wp; gl_Position = mvp*vec4(vertexPosition, 1.0); }\n";
static const char *scene_fs = GLSL_FS
    "IN vec4 fragColor; IN vec3 wpos; IN vec2 uv; IN vec4 lpos;\n"
    "uniform vec3 camPos; uniform vec4 haze; uniform vec3 hazeCol; uniform vec3 hazeSun; uniform vec3 sunDir; uniform vec4 colDiffuse;\n"
    "uniform sampler2D shadowMap; uniform vec4 shadowOn; uniform vec3 worldOff; uniform float roadDetail; uniform float roadW;\n"
    GLSL_NOISE
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
    " vec3 dv = wpos - camPos; float d = length(dv);\n"
    " float fog = 1.0 - exp(-max(d - haze.x, 0.0)/haze.y);\n"
    " fog = min(fog*mix(1.0, 1.45, clamp((haze.w - w.y)/160.0, 0.0, 1.0)), haze.z);   /* thicker in the valleys */\n"
    " vec3 hc = mix(hazeCol, hazeSun, pow(max(dot(dv/max(d, 0.001), sunDir), 0.0), 4.0));\n"
    " FRAG = vec4(mix(lit, hc, fog), 1.0); }\n";
static const char *sky_fs = GLSL_FS
    "IN vec2 fragTexCoord; IN vec4 fragColor;\n"
    "uniform vec2 res; uniform vec3 camF; uniform vec3 camR; uniform vec3 camU; uniform vec2 tanA; uniform vec3 sunDir; uniform vec3 camW;\n"
    "uniform float clouds; uniform vec3 hazeCol;\n"
    GLSL_NOISE
    "void main(){\n"
    " vec2 ndc = gl_FragCoord.xy/res*2.0 - 1.0; vec3 d = normalize(camF + camR*ndc.x*tanA.x + camU*ndc.y*tanA.y); float h = d.y;\n"
    " vec3 c = mix(vec3(0.95, 0.80, 0.60), vec3(0.80, 0.74, 0.74), smoothstep(-0.02, 0.18, h)); c = mix(c, vec3(0.47, 0.55, 0.70), smoothstep(0.15, 0.75, h));\n"
    " float sd = max(dot(d, sunDir), 0.0);\n"
    " c += vec3(1.0, 0.70, 0.40)*pow(sd, 6.0)*0.35; c = mix(c, vec3(1.0, 0.95, 0.80), smoothstep(0.9975, 0.999, sd));\n"
    " if (clouds > 0.5 && h > 0.0) {\n"
    "  vec2 p = (camW.xz + d.xz/max(h, 0.035)*900.0)/1500.0;\n"
    "  float n = 0.55*vn(p) + 0.28*vn(p*2.13 + 5.2) + 0.17*vn(p*4.7 + 1.3);\n"
    "  float cov = smoothstep(0.52, 0.74, n)*smoothstep(0.01, 0.10, h);\n"
    "  vec3 cc = mix(vec3(0.78, 0.66, 0.72), vec3(1.0, 0.87, 0.70), smoothstep(0.58, 0.80, n)) + vec3(0.25, 0.15, 0.05)*pow(sd, 3.0);\n"
    "  c = mix(c, cc, cov*0.9);\n"
    " }\n"
    " c = mix(c, hazeCol, (1.0 - smoothstep(-0.02, 0.07, h))*0.7);\n"
    " FRAG = vec4(c, 1.0); }\n";
static const char *pal_fs = GLSL_FS
    "IN vec2 fragTexCoord; IN vec4 fragColor; uniform sampler2D texture0; uniform vec2 lowSize; uniform vec3 pal[52]; uniform float dith;\n"
    "float b2(vec2 p){ return 2.0*p.x + 3.0*p.y - 4.0*p.x*p.y; }\n"
    "void main(){ vec3 c = TEX(texture0, fragTexCoord).rgb; vec2 px = floor(fragTexCoord*lowSize);\n"
    " float b = 4.0*b2(mod(px, 2.0)) + b2(mod(floor(px/2.0), 2.0)); c += (b/16.0 - 0.47)*dith;\n"
    " vec3 best = pal[0]; float bd = 1e9; for (int i = 0; i < 52; i++){ vec3 d = c - pal[i]; float e = dot(d*d, vec3(0.32, 0.5, 0.18)); if (e < bd){ bd = e; best = pal[i]; } }\n"
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
    sky_sh = LoadShaderFromMemory(NULL, sky_fs);
    sk_res = GetShaderLocation(sky_sh, "res"); sk_f = GetShaderLocation(sky_sh, "camF"); sk_r = GetShaderLocation(sky_sh, "camR"); sk_u = GetShaderLocation(sky_sh, "camU");
    sk_tan = GetShaderLocation(sky_sh, "tanA"); sk_sun = GetShaderLocation(sky_sh, "sunDir"); sk_camw = GetShaderLocation(sky_sh, "camW");
    sk_clouds = GetShaderLocation(sky_sh, "clouds"); sk_haze = GetShaderLocation(sky_sh, "hazeCol");
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
static int touch_on, touch_tap; static Vector2 touch_tap_at, touch_start; static int touch_prev_n;
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
    if (n > 0 && touch_prev_n == 0) touch_start = GetTouchPosition(0);
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

static void touch_draw(int in_game)
{
    if (!touch_on || !in_game) return;   /* the menus are tapped directly */
    for (int i = 0; i < TB_N; i++) {
        if (i == TB_BANK && T.score_bank_button == 0) continue;   /* follow-up: no bank button (leaving the road banks) */
        TButton *b = &tb[i];
        Color fill = b->down ? (Color){ 255, 255, 255, 110 } : (Color){ 255, 255, 255, 45 }, ink = (Color){ 255, 255, 255, 200 };
        if (i == TB_GAS) fill = b->down ? (Color){ 120, 220, 120, 140 } : (Color){ 120, 220, 120, 55 };
        if (i == TB_BRAKE || i == TB_HAND) fill = b->down ? (Color){ 240, 110, 90, 140 } : (Color){ 240, 110, 90, 55 };
        if (i == TB_BANK) fill = b->down ? (Color){ 255, 220, 90, 160 } : (Color){ 255, 220, 90, 60 };
        DrawCircleV((Vector2){ b->x, b->y }, b->r, fill);
        DrawRing((Vector2){ b->x, b->y }, b->r - 3, b->r, 0, 360, 48, ink);
        float a = b->r * 0.42f;
        if (i == TB_LEFT) DrawTriangle((Vector2){ b->x - a, b->y }, (Vector2){ b->x + a * 0.7f, b->y + a }, (Vector2){ b->x + a * 0.7f, b->y - a }, ink);
        else if (i == TB_RIGHT) DrawTriangle((Vector2){ b->x + a, b->y }, (Vector2){ b->x - a * 0.7f, b->y - a }, (Vector2){ b->x - a * 0.7f, b->y + a }, ink);
        else if (i == TB_PAUSE) { DrawRectangleRec((Rectangle){ b->x - a * 0.7f, b->y - a, a * 0.5f, a * 2 }, ink); DrawRectangleRec((Rectangle){ b->x + a * 0.2f, b->y - a, a * 0.5f, a * 2 }, ink); }
        else {
            const char *t = i == TB_GAS ? "GAS" : i == TB_BRAKE ? "BRAKE" : i == TB_HAND ? "HAND" : i == TB_BANK ? "BANK" : "R";
            int fs = (int)(b->r * (i == TB_RESTART ? 0.8f : 0.36f));
            DrawText(t, (int)(b->x - MeasureText(t, fs) * 0.5f), (int)(b->y - fs * 0.5f), fs, ink);
        }
    }
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
        if (m->loaded && !m->own_lamps) {
            for (int s = -1; s <= 1; s += 2) {
                push_light(v3(0.72f * (float)s, 0.12f, T.body_front + 0.08f), 0.22f, 255, 245, 170);
                push_light(v3(0.72f * (float)s, 0.15f, T.body_rear - 0.05f), 0.18f, 200, 30, 25);
            }
        }
        m->count = ntris - m->first;
    }
    select_model(0);

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
    }
    {   /* the progress file sits beside the records (the phones' own storage too) */
        static char pp[320];
        const char *slash = strrchr(erec_path, '/');
        if (progress_arg) snprintf(pp, sizeof(pp), "%s", progress_arg);
        else if (slash) snprintf(pp, sizeof(pp), "%.*s/progress-bush-drift.txt", (int)(slash - erec_path), erec_path);
        else snprintf(pp, sizeof(pp), "progress-bush-drift.txt");
        world_load(pp);
    }
    if (want_selftest) {
        int bad = erecords_selftest("data") | world_selftest("data");
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
    if (scripted && getenv("SHOT_ACH")) {   /* pictures: a few achievements earned, to show the screen */
        const int some[] = { ACH_CARPARK_DONUT, ACH_YODEL, ACH_DEER_1, ACH_BRIDGE, ACH_CLEAN };
        for (unsigned k = 0; k < sizeof(some) / sizeof(some[0]); k++) { W.earned[some[k]] = 1; snprintf(W.date[some[k]], 12, "2026-10-21"); }
        W.found = 0x2f;
    }
    erecords_load(erec_path);
    if (scripted && want_records)
        for (int i = EREC.n; i < EREC_MAX; i++) erecords_add_run(1234560 - i * 98760, 28 - i, 7800.0f - (float)i * 987.0f, 31234u + (uint32_t)i * 777u, "2026-10-28");
    int screen = want_title ? SCR_TITLE : want_records ? SCR_RECORDS : want_ach ? SCR_ACH : (scripted && rt_have) ? SCR_GAME : SCR_TITLE;
    int menu_sel = MENU_DRIVE, menu_block = 1;
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
    look_init();
    far_init();
    if (rt_have) deer_meshes_init();
    look_set_mode(pixel_arg >= 0 ? pixel_arg : (int)T.pixel_mode);
    if (rt_have) route_stream_start(&RT);
    if (rt_have) spawn_ex(&car, &score, !scripted);
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
    int freeze = 0;
    V3 cam_pos = v3(0, 0, 0), cam_dir = v3(0, 0, 1);

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
            if (tb_pressed(TB_BANK)) bank_req = 1;
            if (tb_pressed(TB_PAUSE)) leave = 1;
            if (over && touch_tap && CheckCollisionPointRec(touch_tap_at, touch_game)) reset = 1;   /* run over: tap to go again */
        }
        if (touch_on && !in_game && touch_tap) {   /* menus: tap an item */
            Vector2 v = touch_to_view(touch_tap_at);
            if (screen == SCR_TITLE) {
                for (int i = 0; i < MENU_SHOWN; i++) if (v.x > 170 && v.x < 495 && v.y > 195 + 44 * i && v.y < 233 + 44 * i) { menu_sel = i; m_ok = 1; }
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
            }
            if (T.use_stick != 0) {
                float sx = GetGamepadAxisMovement(pad, GAMEPAD_AXIS_LEFT_X);
                if (fabsf(sx) > 0.1f) in.steer += sx;
            }
        }
        in.steer = clampf(in.steer, -1, 1);
        if (shot_path && idle) {
            if (frame > 200 && run.state < RUN_FINISHED) run.state = RUN_WALL_OVER;
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
        if (in_game && !scripted && run.state == RUN_RUNNING && (leave || reset)) {
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
            else { screen = SCR_TITLE; menu_block = 1; menu_sel = MENU_DRIVE; }
        }
        if (screen != SCR_GAME) {
            reset = 0; shift_req = 0;
            if (menu_block) { if (!m_held) menu_block = 0; m_ok = m_back = 0; }
            if (scripted && !menu_keys) m_ok = m_back = m_up = m_down = 0;
            if (screen == SCR_RECORDS || screen == SCR_ACH) {
                if (m_back || m_ok) { screen = SCR_TITLE; menu_block = 1; synth_blip2(5, 0.7f, 0.6f); }
            } else {
                if (m_up) { menu_sel = (menu_sel + MENU_SHOWN - 1) % MENU_SHOWN; synth_blip2(5, 1.0f, 0.6f); }
                if (m_down) { menu_sel = (menu_sel + 1) % MENU_SHOWN; synth_blip2(5, 1.0f, 0.6f); }
                if (m_back && MENU_SHOWN > MENU_QUIT) menu_sel = MENU_QUIT;
                if (m_ok) {
                    if (menu_sel == MENU_QUIT) quit = 1;
                    else if (menu_sel == MENU_RECORDS) { screen = SCR_RECORDS; menu_block = 1; synth_blip(6); }
                    else if (menu_sel == MENU_ACH) { screen = SCR_ACH; menu_block = 1; synth_blip(6); }
                    else if (rt_have) { screen = SCR_GAME; reset = 1; start_event = 1; synth_blip(6); }
                    else { snprintf(status, sizeof(status), "no route in %s: nothing to drive", rt_dir); status_time = 3; }
                }
            }
        }
        if (screen != SCR_GAME) {
            sound_menu();
            sound_yodel(dt);
            BeginTextureMode(target);
            ClearBackground(sky_colour);
            hud_batch.currentBuffer = frame % HUD_COPIES;
            if (screen == SCR_TITLE) draw_title(menu_sel);
            else if (screen == SCR_ACH) draw_achievements();
            else draw_records();
            if (status_time > 0) { text_centred(status, VIEW_H - 44, 10, MAROON); status_time -= dt; }
            goto present;
        }
        if (reset) {
            yodel_stop(&car);
            spawn_ex(&car, &score, start_event);
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

        /* ---- fixed-step simulation ---- */
        t_sim0 = GetTime();
        if (rt_have) rt_frame(car.body.pos);
        acc += dt;
        mark_clock += dt;
        int road_ready = rt_have && route_stream_pending() == 0;
        while (acc >= FIXED_DT && steps < 5) {
            prev_pos = car.body.pos;
            prev_rot = car.body.rot;
            CarInput sin = in;
            sin.shift = shift_req;
            shift_req = 0;
            if (run.state >= RUN_FINISHED) sin = (CarInput){ .handbrake = 1, .brake = vlen(car.body.vel) > 2.0f ? 1.0f : 0.0f };
            RtPhys ph = { 0 };
            ph.tree = -1;
            if (run.state != RUN_COUNTDOWN && !freeze) {
                car_step(&car, &sin);
                rt_physics(&car, &ph);
                marks_step(&car);
                if (run.state == RUN_RUNNING && in_game && army_check(&car)) {   /* the army man touched: found, gone in a puff */
                    army_flash_t = 2.4f;
                    V3 at = v3(W.army_x, route_height(&RT, W.army_x, W.army_z, NULL) + 0.8f, W.army_z);
                    for (int k = 0; k < 40 && dust_n < DUST_MAX; k++) {
                        dust[dust_n].p = vadd(at, v3((dust_rnd() - 0.5f) * 1.2f, dust_rnd() * 1.6f, (dust_rnd() - 0.5f) * 1.2f));
                        dust[dust_n].v = v3((dust_rnd() - 0.5f) * 5.0f, 1.0f + 2.5f * dust_rnd(), (dust_rnd() - 0.5f) * 5.0f);
                        dust[dust_n].age = 0; dust[dust_n].life = 0.9f + 0.7f * dust_rnd(); dust[dust_n].size = 1.4f + 1.2f * dust_rnd(); dust[dust_n].a0 = 0.9f;
                        dust_n++;
                    }
                    synth_blip(12);
                    if (!scripted) world_save(), web_storage_flush();
                    if (getenv("SHOT_SEQ")) fprintf(stderr, "army man found\n");
                }
                if (run.state == RUN_RUNNING && in_game) {   /* build B: a deer touched: found, gone in a puff */
                    int d = deer_check(&car);
                    if (d >= 0) {
                        deer_flash_t = 2.4f; deer_flash_n = world_found_count();
                        V3 at = v3(W.deer[d].x, route_height(&RT, W.deer[d].x, W.deer[d].z, NULL) + 0.6f, W.deer[d].z);
                        for (int k = 0; k < 40 && dust_n < DUST_MAX; k++) {
                            dust[dust_n].p = vadd(at, v3((dust_rnd() - 0.5f) * 1.6f, dust_rnd() * 1.4f, (dust_rnd() - 0.5f) * 1.6f));
                            dust[dust_n].v = v3((dust_rnd() - 0.5f) * 5.0f, 1.0f + 2.5f * dust_rnd(), (dust_rnd() - 0.5f) * 5.0f);
                            dust[dust_n].age = 0; dust[dust_n].life = 0.9f + 0.7f * dust_rnd(); dust[dust_n].size = 1.4f + 1.2f * dust_rnd(); dust[dust_n].a0 = 0.9f;
                            dust_n++;
                        }
                        synth_blip(12);
                        if (!scripted) world_save(), web_storage_flush();
                        if (getenv("SHOT_SEQ")) fprintf(stderr, "deer %d found (%d of %d)\n", d + 1, deer_flash_n, W.ndeer);
                    }
                }
            }
            int before = run.state;
            if (!freeze) rt_rules(&run, &car, &ph, &rt_max_s, road_ready);
            if (!freeze && rt_offroad_timer(&run, &car, &ph)) {   /* follow-up: back on the track */
                prev_pos = car.body.pos; prev_rot = car.body.rot; cam_snap = 1; cg.init = 0;
                snprintf(status, sizeof(status), "back on the track"); status_time = 2;
                if (getenv("SHOT_SEQ")) fprintf(stderr, "reset onto the track at frame %d\n", frame);
            }
            if (ph.log_broken && run.state == RUN_RUNNING) {   /* follow-up: firewood */
                run.wood++; wood_flash_t = 2.2f; splinters_burst(v3(ph.log_x, route_height(&RT, ph.log_x, ph.log_z, NULL) + 0.4f, ph.log_z), car.body.vel);
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
        score_update(&score, &car, dt, run.state == RUN_RUNNING && run.dir == DIR_CORRECT, rt_last.deep, rt_last.offroad);
        donut_update(&score, &car, dt, run.state == RUN_RUNNING);
        if (score.ev_donut && run.state == RUN_RUNNING && !yodel.active) yodel_start(&car);
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
            ws.pot = score.pot; ws.banked = score.ev_banked; ws.banked_value = score.last_banked; ws.lost = score.ev_lost || lost_this_frame; ws.ford_s = RT.ford_s; ws.wood = run.wood;
            world_step(&ws);
            yodel_prev = yodel.active;
            if (ach_pop.t <= 0) { int a = world_pop(); if (a >= 0) { ach_pop.ach = a; ach_pop.t = 3.0f; synth_blip(13); if (!scripted) world_save(), web_storage_flush(); if (getenv("SHOT_SEQ")) fprintf(stderr, "achievement %s\n", ACH[a].name); } }
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
        { /* ours (session 2): a little drawn lean on the flat-plane handling, so the tall ute rolls out of a
           * slide and squats and dives on throttle and brake. A sprung, lightly damped follow of the car's
           * own acceleration; the wheels stay where the handling has them. */
            static V3 v_prev; static float lr, lrv, lp, lpv;
            if (cam_snap || dt <= 0) { v_prev = car.body.vel; lr = lrv = lp = lpv = 0; }
            float h = fminf(dt, 0.05f);
            V3 la = h > 0 ? qrot(qconj(car.body.rot), vscale(vsub(car.body.vel, v_prev), 1.0f / h)) : v3(0, 0, 0); /* the car's acceleration, in its own axes */
            v_prev = car.body.vel;
            float w = 2.0f * PI * T.lean_hz, k = w * w, c = 2.0f * T.lean_damp * w;
            /* turning right (la.x > 0) the top goes left, out of the turn; pulling away (la.z > 0) the nose lifts */
            float tr = clampf(T.lean_roll * la.x, -7, 7), tp = clampf(-T.lean_pitch * la.z, -4, 4);
            lrv += (k * (tr - lr) - c * lrv) * h; lr += lrv * h;
            lpv += (k * (tp - lp) - c * lpv) * h; lp += lpv * h;
            car_lean_roll = lr; car_lean_pitch = lp;
            drot = qmul(drot, qmul(qaxis(v3(0, 0, 1), lr * DEG2RADF), qaxis(v3(1, 0, 0), lp * DEG2RADF)));
        }
        V3 dpos = v3(rpos.x, rpos.y + cg.h, rpos.z);
        if (cam_snap) { dust_clear(); flame_n = 0; }
        dust_update(&car, fminf(dt, 0.05f), in_game && run.state == RUN_RUNNING);
        flames_update(&car, fminf(dt, 0.05f), in_game && run.state == RUN_RUNNING && yodel.active && car.throttle > 0.3f);
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
            cam_snap = 0;
        }
        V3 look_at = vadd(dpos, v3(cam_dir.x * T.chase_look, 0.9f, cam_dir.z * T.chase_look));
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
        cam.up = (Vector3){ 0, 1, 0 };
        cam.fovy = getenv("SHOT_FOV") ? (float)atof(getenv("SHOT_FOV")) : T.chase_fov;   /* photo matching (session 3b) */
        cam.projection = CAMERA_PERSPECTIVE;

        /* ---- the scene, small ---- */
        V3 sunp; route_sun_dir(&sunp.x, &sunp.y, &sunp.z);
        Vector3 sun_d = { sunp.x, sunp.y, -sunp.z };                 /* drawing axes mirror z */
        Matrix light_vp = MatrixIdentity();
        int shadows_on = T.shadows != 0 && rt_have;
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
                if (!((W.found >> d) & 1) && deer_mesh[d][1].vertexCount && hypotf(W.deer[d].x - cw.x, W.deer[d].z - cw.z) < box) { DrawMesh(deer_mesh[d][1], flat_mat, at_base(deer_base[d])); rt_shadow_tris += deer_tris[d][1]; }
            if (W.army_placed && !W.army_found && army_mesh[1].vertexCount && hypotf(W.army_x - cw.x, W.army_z - cw.z) < box) { DrawMesh(army_mesh[1], flat_mat, at_base(army_base)); rt_shadow_tris += army_tris[1]; }
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
            float cl = T.clouds, hz3[3] = { 227 / 255.0f, 199 / 255.0f, 160 / 255.0f };
            SetShaderValue(sky_sh, sk_res, res2, SHADER_UNIFORM_VEC2); SetShaderValue(sky_sh, sk_tan, tan2, SHADER_UNIFORM_VEC2);
            SetShaderValue(sky_sh, sk_f, &F, SHADER_UNIFORM_VEC3); SetShaderValue(sky_sh, sk_r, &Rr, SHADER_UNIFORM_VEC3); SetShaderValue(sky_sh, sk_u, &U, SHADER_UNIFORM_VEC3);
            SetShaderValue(sky_sh, sk_sun, &sun_d, SHADER_UNIFORM_VEC3); SetShaderValue(sky_sh, sk_camw, &cw, SHADER_UNIFORM_VEC3);
            SetShaderValue(sky_sh, sk_clouds, &cl, SHADER_UNIFORM_FLOAT); SetShaderValue(sky_sh, sk_haze, hz3, SHADER_UNIFORM_VEC3);
            BeginShaderMode(sky_sh);
            DrawRectangle(0, 0, lw, lh, WHITE);
            EndShaderMode();
        }
        rlDrawRenderBatchActive();
        rlSetClipPlanes(0.3f, 12000.0f);
        BeginMode3D(cam);
        rlDisableBackfaceCulling();
        {
            float hz[4] = { T.haze_near, fmaxf(T.haze_len, 1.0f), T.haze_max, 940.0f - RT.elev0 }, hc[3] = { 227 / 255.0f, 199 / 255.0f, 160 / 255.0f }, hc2[3] = { 246 / 255.0f, 196 / 255.0f, 140 / 255.0f };
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
        DrawMesh(ridge_mesh, flat_mat, MatrixTranslate(cam.position.x, cam.position.y - 60.0f, cam.position.z));
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
            if ((W.found >> d) & 1) continue;
            float dd = hypotf(W.deer[d].x - camw.x, W.deer[d].z - camw.z);
            if (dd > 320.0f) continue;
            int l = dd < 70.0f ? 0 : 1;
            if (!deer_mesh[d][l].vertexCount) continue;
            DrawMesh(deer_mesh[d][l], scene_mat, at_base(deer_base[d])); rt_tris_drawn += deer_tris[d][l]; rt_draw_calls++;
        }
        if (W.army_placed && !W.army_found) {   /* the army man */
            float dd = hypotf(W.army_x - camw.x, W.army_z - camw.z);
            int l = dd < 70.0f ? 0 : 1;
            if (dd < 320.0f && army_mesh[l].vertexCount) { DrawMesh(army_mesh[l], scene_mat, at_base(army_base)); rt_tris_drawn += army_tris[l]; rt_draw_calls++; }
        }
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
        { int wn = run.wood < 15 ? run.wood : 15; if (getenv("SHOT_WOOD")) wn = atoi(getenv("SHOT_WOOD")); if (!hide_car && wn > 0 && wn <= 15) DrawMesh(wood_mesh[wn], scene_mat, pose_matrix(vsub(dpos, ro), drot)); }   /* follow-up: firewood in the tray */
        rlDisableDepthMask();
        dust_draw(cam);
        splinters_update(fminf(dt, 0.05f));
        splinters_draw(cam);
        flames_draw(cam);
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
        if (getenv("SHOT_NOHUD")) goto present;   /* pictures (build B): the scene alone (the route map in docs/route) */
        float kmh = vlen(car.body.vel) * 3.6f;
        const Color ink = { 255, 248, 230, 255 }, gold = { 255, 235, 120, 255 };
        DrawText(commas((int)(score.total + 0.5f)), 300, 6, 20, ink);
        if (score.pot >= 5 || score.mult > 1) {   /* build A: the unbanked pot, its multiplier and the count to the next step */
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
        if (rt_last.offroad && !rt_last.carpark && run.state == RUN_RUNNING) {   /* build B: why nothing is scoring; follow-up: and the countdown back to the track */
            const char *ot = T.offroad_reset > 0 ? TextFormat("OFF-ROAD  %d", (int)ceilf(fmaxf(T.offroad_reset - run.offroad_t, 0.0f))) : "OFF-ROAD";
            DrawRectangle(320 - MeasureText(ot, 20) / 2 - 6, 30, MeasureText(ot, 20) + 12, 22, (Color){ 40, 30, 20, 150 });
            DrawText(ot, 320 - MeasureText(ot, 20) / 2, 32, 20, (Color){ 230, 190, 120, 255 });
        }
        if (run.missed && run.state == RUN_RUNNING) {
            const char *mt = "CHECKPOINT MISSED: BACK UP THE ROAD";
            DrawText(mt, 320 - MeasureText(mt, 20) / 2, 66, 20, (Color){ 255, 120, 90, 255 });
        }
        if (deer_flash_t > 0 && run.state == RUN_RUNNING) {   /* build B: a deer found */
            float a = clamp01(deer_flash_t / 0.6f), rise = (2.4f - deer_flash_t) * 6.0f;
            const char *dt2 = TextFormat("DEER FOUND %d/%d", deer_flash_n, W.ndeer);
            DrawText(dt2, 320 - MeasureText(dt2, 30) / 2, (int)(128 - rise), 30, (Color){ 255, 170, 70, (unsigned char)(255 * a) });
            deer_flash_t -= dt;
        }
        if (army_flash_t > 0 && run.state == RUN_RUNNING) {
            float a = clamp01(army_flash_t / 0.6f), rise = (2.4f - army_flash_t) * 6.0f;
            const char *t2 = "ARMY MAN FOUND!";
            DrawText(t2, 320 - MeasureText(t2, 30) / 2, (int)(128 - rise), 30, (Color){ 150, 210, 110, (unsigned char)(255 * a) });
            army_flash_t -= dt;
        }
        if (wood_flash_t > 0 && run.state == RUN_RUNNING) {   /* follow-up: firewood */
            float a = clamp01(wood_flash_t / 0.5f), rise = (2.2f - wood_flash_t) * 6.0f;
            const char *t3 = run.wood >= (int)T.wood_full ? "TRAY FULL!" : TextFormat("FIREWOOD %d/%d", run.wood, (int)T.wood_full);
            DrawText(t3, 320 - MeasureText(t3, 30) / 2, (int)(128 - rise), 30, (Color){ 230, 170, 100, (unsigned char)(255 * a) });
            wood_flash_t -= dt;
        }
        if (ach_pop.t > 0) {   /* build B: an achievement, a short banner low on the screen */
            float a = clamp01(ach_pop.t / 0.5f) * clamp01((3.0f - ach_pop.t) / 0.2f);
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
            DrawText(TextFormat("x%g SCORE", T.yodel_score), 50, 140, 10, (Color){ 255, 235, 120, 255 });
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
                           : score.flash_kind == 4 ? "DONUT  (car park: practice)" : TextFormat("LOST %s", commas((int)(score.flash_v + 0.5f)));
            Color fc = score.flash_kind == 1 || score.flash_kind == 3 ? (Color){ 255, 235, 120, (unsigned char)(255 * a) }
                     : score.flash_kind == 4 ? (Color){ 200, 230, 255, (unsigned char)(255 * a) } : (Color){ 235, 120, 110, (unsigned char)(255 * a) };
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
            DrawText(TextFormat("%.2f / %.1f km", rt_max_s / 1000.0f, RT.bridge_s0 / 1000.0f), VIEW_W - 100, 58, 10, ink);
            DrawText(TextFormat("%d:%04.1f", (int)(run.time / 60.0f), fmodf(run.time, 60.0f)), VIEW_W - 78, 70, 10, ink);
            DrawRectangle(8, VIEW_H - 8, VIEW_W - 16, 3, (Color){ 40, 36, 50, 180 });
            DrawRectangle(8, VIEW_H - 8, (int)((float)(VIEW_W - 16) * clamp01(rt_max_s / fmaxf(RT.bridge_s0, 1.0f))), 3, gold);
            if (run.state == RUN_COUNTDOWN && run.countdown <= 3.0f)
                DrawText(TextFormat("%d", (int)ceilf(run.countdown)), 300, 150, 80, ink);
            else if (run.state == RUN_COUNTDOWN)
                DrawText("ALEXANDERS HUT", 205, 160, 20, ink);
            else if (run.state == RUN_RUNNING && run.time < 0.8f && !look_dir)
                DrawText("GO", 270, 150, 80, ink);
            else if (run.state == RUN_RUNNING && run.wrong_shown)
                DrawText(TextFormat("WRONG WAY  %d", (int)ceilf(run.wrong_left - 1.0f)), 190, 160, 30, (Color){ 230, 60, 40, 255 });
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
