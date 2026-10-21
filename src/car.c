/* The ute's handling, written from scratch from docs/handling-spec.md.
 *
 * A rigid body on a flat plane (x, z and yaw) carried by four tyres. Each tyre's force comes from its
 * slip (slip ratio along the wheel, slip angle across it), combined into one normalised slip so that a
 * spinning or locked wheel loses side grip (a friction ellipse). The engine drives the rear wheels
 * through a friction clutch, a five-speed gearbox and a lightly locked differential.
 *
 * Each 0.02 s step is split into small substeps. In each one the wheel and engine speeds are solved
 * implicitly (the tyre's grip, the clutch, the brakes and the diff are stiff), then the body is moved
 * by the tyre forces at the new wheel speeds. All numbers come from the tuning (tuning-car.txt). */
#include "car.h"
#include "tuning.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RPM_PER_RADS 9.5492966f
#define TWO_PI 6.2831853f

/* ---------------------------------------------------------------- presets */

/* The handling presets (H). Build A, 21 Oct: three of them. safer is as it was; best is now a little
 * closer to drift (less rear grip, rear-biased brakes, more low-down torque); drift takes more grip off the rear
 * and adds power, so a push of throttle swings the tail and holds it. Every preset states every key any of them
 * changes, so they can be cycled in any order. "best" restates the defaults (tuning-car.txt) so it can be re-applied. */
static const struct { const char *name, *text; } presets[CAR_NPRESETS] = {
    { "safer", /* most grip, rear-biased: settles quickly */
      "car_mu_front = 0.97\n"
      "car_mu_rear = 1.04\n"
      "car_long_grip = 1.0\n"
      "car_peak_slip_angle_deg = 8\n"
      "car_peak_slip_ratio = 0.15\n"
      "car_tail = 0.96\n"
      "car_tail_width = 4\n"
      "car_load_sens = 0.08\n"
      "car_brake_front = 0.65\n"
      "car_torque_nm = 500\n"
      "car_torque_idle = 280\n"
      "car_torque_from_rpm = 1600\n" },
    { "best", /* packed gravel; build A: rear grip 0.88 -> 0.85, brakes 65 -> 60% front, 500 -> 550 Nm from 1500 rpm */
      "car_mu_front = 0.85\n"
      "car_mu_rear = 0.85\n"
      "car_long_grip = 1.0\n"
      "car_peak_slip_angle_deg = 8\n"
      "car_peak_slip_ratio = 0.15\n"
      "car_tail = 0.95\n"
      "car_tail_width = 4\n"
      "car_load_sens = 0.08\n"
      "car_brake_front = 0.60\n"
      "car_torque_nm = 550\n"
      "car_torque_idle = 310\n"
      "car_torque_from_rpm = 1500\n" },
    { "drift", /* build A: best's surface, rear grip 0.80, brakes 55% front, 650 Nm from 1300 rpm */
      "car_mu_front = 0.85\n"
      "car_mu_rear = 0.80\n"
      "car_long_grip = 1.0\n"
      "car_peak_slip_angle_deg = 8\n"
      "car_peak_slip_ratio = 0.15\n"
      "car_tail = 0.95\n"
      "car_tail_width = 4\n"
      "car_load_sens = 0.08\n"
      "car_brake_front = 0.55\n"
      "car_torque_nm = 650\n"
      "car_torque_idle = 380\n"
      "car_torque_from_rpm = 1300\n" },
};

const char *car_preset(int p, const char **name)
{
    if (p < 0 || p >= CAR_NPRESETS) {
        if (name) *name = NULL;
        return NULL;
    }
    if (name) *name = presets[p].name;
    return presets[p].text;
}

/* ---------------------------------------------------------------- small helpers */

static float finite_or(float v, float d) { return isfinite(v) ? v : d; }

/* move cur towards target by at most up (rising) or down (falling) */
static float approach(float cur, float target, float up, float down)
{
    if (target > cur) return minf(target, cur + up);
    return maxf(target, cur - down);
}

static float smooth01(float t)
{
    t = clamp01(t);
    return t * t * (3.0f - 2.0f * t);
}

static float lerpf(float x, float x0, float y0, float x1, float y1)
{
    if (x1 - x0 < 1e-6f) return y1;
    return y0 + (y1 - y0) * clamp01((x - x0) / (x1 - x0));
}

/* yaw angle to the car's forward and right directions in the world */
static V3 dir_fwd(float yaw) { return v3(sinf(yaw), 0, cosf(yaw)); }
static V3 dir_right(float yaw) { return v3(cosf(yaw), 0, -sinf(yaw)); }

/* overall ratio (gear times final drive) for a gear; negative in reverse, 0 in neutral */
static float gear_ratio(int g)
{
    float r = 0;
    switch (g) {
    case -1: r = -T.car_gear_r; break;
    case 1: r = T.car_gear_1; break;
    case 2: r = T.car_gear_2; break;
    case 3: r = T.car_gear_3; break;
    case 4: r = T.car_gear_4; break;
    case 5: r = T.car_gear_5; break;
    default: r = 0;
    }
    return r * T.car_final_drive;
}

static int count_gears(void)
{
    const float g[5] = { T.car_gear_1, T.car_gear_2, T.car_gear_3, T.car_gear_4, T.car_gear_5 };
    int n = 0;
    while (n < 5 && g[n] > 0) n++;
    return n < 1 ? 1 : n;
}

/* share of the full steering lock allowed at a forward speed */
static float lock_frac(float v)
{
    float q = fabsf(v) / maxf(T.car_steer_speed_ref, 1.0f);
    return maxf(1.0f / sqrtf(1.0f + q * q), T.car_steer_min_frac);
}

/* ---------------------------------------------------------------- engine */

/* full-throttle torque (Nm) from rpm: rising to the flat peak, then following the power peak down */
static float engine_curve(float rpm)
{
    float idle = T.car_idle_rpm, lim = T.car_limit_rpm;
    float pw = T.car_power_rpm;
    float t_pw = T.car_power_kw * 1000.0f / maxf(pw / RPM_PER_RADS, 1.0f);
    if (rpm <= idle) return T.car_torque_idle;
    if (rpm < T.car_torque_from_rpm) return lerpf(rpm, idle, T.car_torque_idle, T.car_torque_from_rpm, T.car_torque_nm);
    if (rpm <= T.car_torque_to_rpm) return T.car_torque_nm;
    if (rpm < pw) return lerpf(rpm, T.car_torque_to_rpm, T.car_torque_nm, pw, t_pw);
    if (rpm < lim) return lerpf(rpm, pw, t_pw, lim, T.car_torque_at_limit);
    return lerpf(rpm, lim, T.car_torque_at_limit, lim + 600.0f, 0.0f);
}

static float engine_friction(float rpm) { return T.car_engine_fric + T.car_engine_fric_rpm * maxf(rpm, 0) * 0.001f; }

/* throttle at which the engine neither speeds up nor slows down on its own at this rpm */
static float hold_throttle(float rpm)
{
    float tc = engine_curve(rpm) * T.car_driveline_eff, tf = engine_friction(rpm);
    return tf / maxf(tc + tf, 1.0f);
}

/* net engine torque at the clutch */
static float engine_torque(float rpm, float th)
{
    return th * engine_curve(rpm) * T.car_driveline_eff - (1.0f - th) * engine_friction(rpm);
}

/* ---------------------------------------------------------------- tyre */

typedef struct { float fx, fy, sx, sa, slip_v; } TyreOut;

/* Force from one tyre. vx, vy: contact patch ground velocity along and across the wheel (m/s, y to the
 * right). w: wheel spin (rad/s). n: load (N). mu: peak friction coefficient.
 * The slip ratio and the tangent of the slip angle are each divided by their peak values and combined
 * into one slip p; the force follows a curve of p (rising to 1 at p = 1, then falling towards the tail)
 * and points against the combined slip, so wheelspin or a lock-up uses up the side grip. */
static TyreOut tyre(float vx, float vy, float w, float n, float mu)
{
    TyreOut o;
    float r = T.car_wheel_radius;
    float den = maxf(fabsf(vx), T.car_low_speed);
    float sx = (w * r - vx) / den;
    float sy = vy / den;
    float px = sx / maxf(T.car_peak_slip_ratio, 0.01f);
    float py = sy / maxf(tanf(T.car_peak_slip_angle_deg * DEG2RADF), 0.01f);
    float p = sqrtf(px * px + py * py);
    o.sx = sx;
    o.sa = atan2f(vy, maxf(fabsf(vx), 0.1f));
    o.slip_v = sqrtf((w * r - vx) * (w * r - vx) + vy * vy);
    o.fx = o.fy = 0;
    if (p < 1e-6f || n <= 0) return o;
    float f;
    if (p < 1.0f) f = 1.5f * p - 0.5f * p * p * p;
    else {
        float q = (p - 1.0f) / maxf(T.car_tail_width, 0.05f);
        f = T.car_tail + (1.0f - T.car_tail) / (1.0f + q * q);
    }
    float F = mu * n * f;
    o.fx = F * (px / p) * T.car_long_grip;
    o.fy = -F * (py / p);
    if (px > 1.0f && T.car_spin_side_loss > 0) {   /* build A: a wheel spinning past its peak on loose gravel at low speed digs in and loses
                                                    * more of its side grip (donuts); faded out by car_spin_side_to m/s along the wheel */
        float k = T.car_spin_side_loss * clamp01(1.0f - fabsf(vx) / maxf(T.car_spin_side_to, 0.1f));
        o.fy /= 1.0f + k * (px - 1.0f);
    }
    return o;
}

/* ---------------------------------------------------------------- implicit wheel/engine solver */

/* A coupling that resists a speed difference s = u.w - s0 with torque k*s, kept within [lo, hi]:
 * a clutch, a brake or the diff. While inside its limits it behaves as a stiff damper (it grips);
 * at a limit it passes a fixed torque (it slips). */
typedef struct { float u[3], s0, lo, hi; int mode; } Link;

/* Solve (M - h J) w' = M w + h (tau - J w) plus the links, for n <= 3 speeds. */
static void solve_speeds(int n, const float *m, const float *jd, const float *tau, float *w,
                         Link *lk, int nl, float h, float k)
{
    float x[3] = { 0, 0, 0 };
    for (int i = 0; i < nl; i++) lk[i].mode = 0;
    for (int iter = 0; iter < 4; iter++) {
        float a[3][4];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) a[i][j] = 0;
            a[i][i] = m[i] - h * jd[i];
            a[i][3] = m[i] * w[i] + h * (tau[i] - jd[i] * w[i]);
        }
        for (int l = 0; l < nl; l++) {
            const float *u = lk[l].u;
            if (lk[l].mode == 0) {
                for (int i = 0; i < n; i++) {
                    for (int j = 0; j < n; j++) a[i][j] += h * k * u[i] * u[j];
                    a[i][3] += h * k * lk[l].s0 * u[i];
                }
            } else {
                float t = lk[l].mode > 0 ? lk[l].hi : lk[l].lo;
                for (int i = 0; i < n; i++) a[i][3] -= h * t * u[i];
            }
        }
        /* Gaussian elimination with partial pivoting */
        for (int c = 0; c < n; c++) {
            int piv = c;
            for (int r = c + 1; r < n; r++)
                if (fabsf(a[r][c]) > fabsf(a[piv][c])) piv = r;
            if (piv != c)
                for (int j = 0; j < 4; j++) { float t = a[c][j]; a[c][j] = a[piv][j]; a[piv][j] = t; }
            if (fabsf(a[c][c]) < 1e-9f) a[c][c] = 1e-9f;
            for (int r = c + 1; r < n; r++) {
                float f = a[r][c] / a[c][c];
                for (int j = c; j < 4; j++) a[r][j] -= f * a[c][j];
            }
        }
        for (int i = n - 1; i >= 0; i--) {
            float s = a[i][3];
            for (int j = i + 1; j < n; j++) s -= a[i][j] * x[j];
            x[i] = s / a[i][i];
        }
        int changed = 0;
        for (int l = 0; l < nl; l++) {
            if (lk[l].mode != 0) continue;
            float s = -lk[l].s0;
            for (int i = 0; i < n; i++) s += lk[l].u[i] * x[i];
            float t = k * s;
            if (t > lk[l].hi) { lk[l].mode = 1; changed = 1; }
            else if (t < lk[l].lo) { lk[l].mode = -1; changed = 1; }
        }
        if (!changed) break;
    }
    for (int i = 0; i < n; i++) w[i] = finite_or(x[i], 0);
}

/* a brake on speed index idx: with ABS it only holds the wheel down to the peak slip, otherwise to 0 */
static int brake_link(Link *l, int idx, float cap, float vx, int abs_on, float abs_slip)
{
    if (cap < 1e-3f) return 0;
    memset(l, 0, sizeof(*l));
    l->u[idx] = 1;
    if (abs_on && fabsf(vx) > 1.0f) {
        float target = vx / T.car_wheel_radius * (1.0f - abs_slip);
        l->s0 = target;
        if (vx > 0) { l->lo = 0; l->hi = cap; }
        else { l->lo = -cap; l->hi = 0; }
    } else {
        l->s0 = 0;
        l->lo = -cap;
        l->hi = cap;
    }
    return 1;
}

/* ---------------------------------------------------------------- geometry */

typedef struct { V3 com; float wheelbase, track_f, track_r, zf; } Geo;

static Geo geometry(const Car *c)
{
    Geo g;
    const Wheel *w = c->wheel;
    float zf = 0.5f * (w[FL].anchor.z + w[FR].anchor.z), zr = 0.5f * (w[RL].anchor.z + w[RR].anchor.z);
    g.wheelbase = maxf(zf - zr, 0.5f);
    g.track_f = maxf(fabsf(w[FR].anchor.x - w[FL].anchor.x), 0.3f);
    g.track_r = maxf(fabsf(w[RR].anchor.x - w[RL].anchor.x), 0.3f);
    float x = 0.25f * (w[FL].anchor.x + w[FR].anchor.x + w[RL].anchor.x + w[RR].anchor.x);
    g.com = v3(x, 0, zr + T.car_front_share * (zf - zr));
    g.zf = zf;
    return g;
}

/* ---------------------------------------------------------------- interface */

float car_yaw(const Car *c) { return 2.0f * atan2f(c->body.rot.y, c->body.rot.w); }

V3 car_local_velocity(const Car *c)
{
    float yaw = car_yaw(c);
    return v3(vdot(c->body.vel, dir_right(yaw)), c->body.vel.y, vdot(c->body.vel, dir_fwd(yaw)));
}

void car_setup(Car *c, const V3 wheel_anchor[4])
{
    for (int i = 0; i < 4; i++) {
        if (wheel_anchor) c->wheel[i].anchor = wheel_anchor[i];
        else {
            int front = i < 2, left = (i & 1) == 0;
            float tr = front ? T.car_track_front : T.car_track_rear;
            c->wheel[i].anchor = v3((left ? -0.5f : 0.5f) * tr, T.car_wheel_y, (front ? 0.5f : -0.5f) * T.car_wheelbase);
        }
        c->wheel[i].on_ground = 1;
    }
    c->ngears = count_gears();
    if (c->gear > c->ngears) c->gear = c->ngears;
    c->max_steer = T.car_max_steer_deg * DEG2RADF;
    if (!c->changing_gear) c->ratio = gear_ratio(c->gear);
    /* the shared gear table: reverse, neutral, then the forward gears */
    T.gears[0] = T.car_gear_r;
    T.gears[1] = 0;
    T.gears[2] = T.car_gear_1; T.gears[3] = T.car_gear_2; T.gears[4] = T.car_gear_3;
    T.gears[5] = T.car_gear_4; T.gears[6] = T.car_gear_5;
}

static void fill_wheels(Car *c);

void car_reset(Car *c, V3 pos, float yaw_rad)
{
    c->body.pos = pos;
    c->body.rot = qaxis(v3(0, 1, 0), yaw_rad);
    c->body.vel = v3(0, 0, 0);
    c->body.angvel = v3(0, 0, 0);
    memset(&c->in, 0, sizeof(c->in));
    c->steering = c->steer_angle = 0;
    c->throttle = c->brake = c->handbrake = 0;
    c->velo = c->slide_deg = c->slide = 0;
    c->ngears = count_gears();
    c->gear = 1;
    c->neutral = 0;
    c->changing_gear = 0;
    c->shift_timer = 0;
    c->ratio = gear_ratio(1);
    c->engine_ang_vel = T.car_idle_rpm / RPM_PER_RADS;
    c->rpm = T.car_idle_rpm;
    c->clutch_position = 1;
    c->rev_limiter_triggered = 0;
    c->time = 0;
    c->max_steer = T.car_max_steer_deg * DEG2RADF;
    c->acc_fwd = c->acc_lat = 0;
    float n0 = T.car_mass * T.car_gravity * 0.25f;
    for (int i = 0; i < 4; i++) {
        Wheel *w = &c->wheel[i];
        w->rotation = w->ang_vel = 0;
        w->load = n0;
        w->slip_angle = w->slip_ratio = w->slip_velo = 0;
        w->on_ground = 1;
    }
    fill_wheels(c);
}

/* world positions and directions of the wheels, for drawing */
static void fill_wheels(Car *c)
{
    float yaw = car_yaw(c);
    V3 f = dir_fwd(yaw), r = dir_right(yaw);
    for (int i = 0; i < 4; i++) {
        Wheel *w = &c->wheel[i];
        float d = i < 2 ? c->steer_angle : 0;
        w->centre = vadd(c->body.pos, vadd(vadd(vscale(r, w->anchor.x), vscale(f, w->anchor.z)), v3(0, w->anchor.y, 0)));
        w->hit_point = vsub(w->centre, v3(0, T.car_wheel_radius, 0));
        w->right_dir = vsub(vscale(r, cosf(d)), vscale(f, sinf(d)));
        w->on_ground = 1;
    }
}

static void start_shift(Car *c, int g)
{
    if (g == c->gear) return;
    c->gear = g;
    c->ratio = 0;
    c->neutral = 1;
    if (g == 0) {               /* neutral needs no time */
        c->changing_gear = 0;
        c->shift_timer = 0;
        return;
    }
    c->changing_gear = 1;
    c->shift_timer = T.car_shift_time;
}

static void sanitise(Car *c)
{
    Body *b = &c->body;
    int bad = !isfinite(b->pos.x) || !isfinite(b->pos.z) || !isfinite(b->vel.x) || !isfinite(b->vel.z) ||
              !isfinite(b->angvel.y) || !isfinite(b->rot.y) || !isfinite(b->rot.w) || !isfinite(c->engine_ang_vel);
    for (int i = 0; i < 4; i++) bad |= !isfinite(c->wheel[i].ang_vel);
    if (bad) { /* never expected; park the car where it was rather than spread NaNs */
        V3 p = isfinite(b->pos.x) && isfinite(b->pos.z) ? b->pos : v3(0, b->pos.y, 0);
        float yaw = finite_or(car_yaw(c), 0);
        car_reset(c, p, yaw);
    }
    b->vel.y = 0;
    b->angvel.x = b->angvel.z = 0;
}

void car_step(Car *c, const CarInput *inp)
{
    const float dt = FIXED_DT;
    CarInput in = *inp;
    in.throttle = clamp01(finite_or(in.throttle, 0));
    in.brake = clamp01(finite_or(in.brake, 0));
    in.handbrake = clamp01(finite_or(in.handbrake, 0));
    in.clutch = clamp01(finite_or(in.clutch, 0));
    in.steer = clampf(finite_or(in.steer, 0), -1, 1);
    c->in = in;
    sanitise(c);

    Geo g = geometry(c);
    const float r = T.car_wheel_radius;
    float yaw = car_yaw(c);
    V3 fwd = dir_fwd(yaw), right = dir_right(yaw);
    float vf = vdot(c->body.vel, fwd), vl = vdot(c->body.vel, right);
    int manual = T.manual_gears > 0.5f;
    c->ngears = count_gears();

    /* ---- rev limiter: fuel cut above the limit until the revs fall back */
    {
        float rpm_now = c->engine_ang_vel * RPM_PER_RADS;
        if (rpm_now > T.car_limit_rpm) c->rev_limiter_triggered = 1;
        else if (rpm_now < T.car_limit_rpm - T.car_limit_hyst_rpm) c->rev_limiter_triggered = 0;
    }

    /* ---- gearbox */
    if (c->changing_gear) {
        c->shift_timer -= dt;
        if (c->shift_timer <= 0) {
            c->changing_gear = 0;
            c->shift_timer = 0;
            c->neutral = c->gear == 0;
            c->ratio = gear_ratio(c->gear);
        }
    } else if (manual) {
        if (in.shift > 0 && c->gear < c->ngears) start_shift(c, c->gear + 1);
        else if (in.shift < 0 && c->gear > -1) start_shift(c, c->gear - 1);
    } else {
        /* automatic: the brake held at a standstill selects reverse, where the pedals swap;
         * the throttle held at a standstill selects first */
        int stopped = fabsf(vf) < 0.6f;
        if (c->gear >= 0 && stopped && in.brake > 0.5f && in.throttle < 0.1f) start_shift(c, -1);
        else if (c->gear <= 0 && stopped && in.throttle > 0.5f && in.brake < 0.1f) start_shift(c, 1);
        else if (c->gear >= 1) {
            /* shift on road speed (as an automatic's output shaft sees it), so wheelspin does not
             * make it hunt; the limiter also forces an upshift if the next gear would pull */
            float rpm_road = fabsf(vf / r * gear_ratio(c->gear)) * RPM_PER_RADS;
            float rpm_next = c->gear < c->ngears ? rpm_road * gear_ratio(c->gear + 1) / gear_ratio(c->gear) : 0;
            if (c->gear < c->ngears && (rpm_road > T.car_shift_up_rpm ||
                (c->rev_limiter_triggered && rpm_next > T.car_shift_down_rpm + 200.0f)))
                start_shift(c, c->gear + 1);
            else if (c->gear > 1 && rpm_road < T.car_shift_down_rpm &&
                     rpm_road * gear_ratio(c->gear - 1) / gear_ratio(c->gear) < T.car_shift_up_rpm - 400.0f)
                start_shift(c, c->gear - 1);
        }
    }

    /* ---- pedals and steering, smoothed */
    float thr_t = in.throttle, brk_t = in.brake;
    if (!manual && c->gear == -1) { thr_t = in.brake; brk_t = in.throttle; }
    float up = dt / maxf(T.car_pedal_time, 0.001f), down = dt / maxf(T.car_pedal_release_time, 0.001f);
    c->throttle = approach(c->throttle, thr_t, up, down);
    c->brake = approach(c->brake, brk_t, up, down);
    c->handbrake = approach(c->handbrake, in.handbrake, up, down);
    {
        int returning = fabsf(in.steer) < fabsf(c->steering) || in.steer * c->steering < 0;
        float rate = (returning ? T.car_steer_return_rate : T.car_steer_rate) * dt;
        c->steering = approach(c->steering, in.steer, rate, rate);
    }

    /* slide angle of the body, and the steering assist that turns the fronts towards the travel */
    c->slide_deg = vf > 0.5f ? atan2f(fabsf(vl), vf) * RAD2DEGF : 0;
    c->slide = clamp01((c->slide_deg - T.car_slide_from_deg) / maxf(T.car_slide_full_deg - T.car_slide_from_deg, 0.1f));
    c->max_steer = T.car_max_steer_deg * DEG2RADF;
    float delta = c->steering * c->max_steer * lock_frac(vf);
    if (vf > 2.0f) {
        float beta_f = atan2f(vl + c->body.angvel.y * (g.zf - g.com.z), vf);
        delta += T.car_steer_assist * c->slide * beta_f;
    }
    delta = clampf(delta, -c->max_steer, c->max_steer);
    c->steer_angle = delta;

    /* ---- clutch: an automatic bite by engine speed (it cannot stall), times the pedal */
    float rpm0 = c->engine_ang_vel * RPM_PER_RADS;
    float bite = smooth01((rpm0 - T.car_clutch_bite_rpm) / maxf(T.car_clutch_full_rpm - T.car_clutch_bite_rpm, 1.0f));
    float engage = (c->changing_gear || c->gear == 0) ? 0 : bite * (1.0f - in.clutch);
    c->clutch_position = 1.0f - engage;
    float clutch_cap = T.car_clutch_nm * engage;
    float G = c->changing_gear ? 0 : c->ratio;
    /* during a shift the engine is blipped to the speed the new gear will want */
    float match_ratio = c->changing_gear ? gear_ratio(c->gear) : 0;

    /* ---- static loads */
    float m = maxf(T.car_mass, 1.0f), grav = T.car_gravity;
    float wf = m * grav * T.car_front_share, wr = m * grav - wf;
    float n_static = m * grav * 0.25f;
    float iz = maxf(T.car_yaw_inertia, 1.0f), iw = maxf(T.car_wheel_inertia, 0.05f), ie = maxf(T.car_engine_inertia, 0.01f);
    float kl = maxf(T.car_lock_damping, 1.0f);

    /* brakes, Nm per wheel */
    float fb = T.car_brake_nm * c->brake, bf = clamp01(T.car_brake_front);
    float cap_fb[4] = { fb * bf * 0.5f, fb * bf * 0.5f, fb * (1 - bf) * 0.5f, fb * (1 - bf) * 0.5f };
    float cap_hb = T.car_handbrake_nm * c->handbrake * 0.5f;
    int abs_on = T.car_abs > 0.5f;

    int ns = (int)clampf(T.car_substeps, 1, 32);
    float h = dt / ns;
    V3 com_w = vadd(c->body.pos, vadd(vscale(right, g.com.x), vscale(fwd, g.com.z)));
    com_w.y = c->body.pos.y;
    V3 vel = c->body.vel;
    float wy = c->body.angvel.y;
    TyreOut last[4];
    memset(last, 0, sizeof(last));

    for (int s = 0; s < ns; s++) {
        fwd = dir_fwd(yaw);
        right = dir_right(yaw);

        /* loads: static share plus transfer from the smoothed acceleration */
        float dlong = m * c->acc_fwd * T.car_com_height / g.wheelbase;
        float nf = 0.5f * (wf - dlong), nr = 0.5f * (wr + dlong);
        float dlf = m * c->acc_lat * T.car_com_height * T.car_roll_front / g.track_f;
        float dlr = m * c->acc_lat * T.car_com_height * (1.0f - T.car_roll_front) / g.track_r;
        float load[4] = { nf + dlf, nf - dlf, nr + dlr, nr - dlr };

        V3 rw[4], wfw[4], wrt[4];
        float vx[4], vy[4], mu[4], jd[4], tau[4];
        for (int i = 0; i < 4; i++) {
            Wheel *w = &c->wheel[i];
            load[i] = maxf(load[i], 0);
            float ax = w->anchor.x - g.com.x, az = w->anchor.z - g.com.z;
            rw[i] = vadd(vscale(right, ax), vscale(fwd, az));
            float d = i < 2 ? delta : 0;
            wfw[i] = vadd(vscale(fwd, cosf(d)), vscale(right, sinf(d)));
            wrt[i] = vsub(vscale(right, cosf(d)), vscale(fwd, sinf(d)));
            V3 vp = vadd(vel, v3(wy * rw[i].z, 0, -wy * rw[i].x));
            vx[i] = vdot(vp, wfw[i]);
            vy[i] = vdot(vp, wrt[i]);
            float base = i < 2 ? T.car_mu_front : T.car_mu_rear;
            mu[i] = base * maxf(1.0f - T.car_load_sens * (load[i] / n_static - 1.0f), 0.3f);
            /* tyre torque on the wheel and its slope against wheel speed (kept stabilising only) */
            TyreOut t0 = tyre(vx[i], vy[i], w->ang_vel, load[i], mu[i]);
            TyreOut t1 = tyre(vx[i], vy[i], w->ang_vel + 0.05f, load[i], mu[i]);
            jd[i] = minf(-r * (t1.fx - t0.fx) / 0.05f, 0);
            float roll = T.car_rolling * load[i] * r * tanhf(w->ang_vel * r / 0.5f);
            tau[i] = -t0.fx * r - roll;
        }

        /* front wheels: brakes only */
        for (int i = 0; i < 2; i++) {
            Link lk;
            int nl = brake_link(&lk, 0, cap_fb[i], vx[i], abs_on, T.car_abs_slip);
            float mm = iw, w1 = c->wheel[i].ang_vel;
            solve_speeds(1, &mm, &jd[i], &tau[i], &w1, &lk, nl, h, kl);
            c->wheel[i].ang_vel = w1;
        }

        /* engine and rear wheels together: clutch, diff, brakes */
        {
            float rpm = c->engine_ang_vel * RPM_PER_RADS;
            float th = c->changing_gear ? 0 : c->throttle;
            if (match_ratio != 0) {
                float want = fabsf(0.5f * (c->wheel[RL].ang_vel + c->wheel[RR].ang_vel) * match_ratio) * RPM_PER_RADS;
                if (want > T.car_idle_rpm) th = clamp01(hold_throttle(want) + (want - rpm) * T.car_idle_gain * 2.0f);
            }
            if (c->rev_limiter_triggered) th = 0;
            float gov = clamp01(hold_throttle(T.car_idle_rpm) + (T.car_idle_rpm - rpm) * T.car_idle_gain);
            th = maxf(th, gov);

            float mm[3] = { ie, iw, iw };
            float jj[3] = { 0, jd[RL], jd[RR] };
            float tt[3] = { engine_torque(rpm, th), tau[RL], tau[RR] };
            float ww[3] = { c->engine_ang_vel, c->wheel[RL].ang_vel, c->wheel[RR].ang_vel };
            Link lk[4];
            int nl = 0;
            if (clutch_cap > 1e-3f && G != 0) {
                memset(&lk[nl], 0, sizeof(Link));
                lk[nl].u[0] = 1; lk[nl].u[1] = -0.5f * G; lk[nl].u[2] = -0.5f * G;
                lk[nl].lo = -clutch_cap; lk[nl].hi = clutch_cap;
                nl++;
            }
            if (T.car_diff_lock_nm > 1e-3f) {
                memset(&lk[nl], 0, sizeof(Link));
                lk[nl].u[1] = 1; lk[nl].u[2] = -1;
                lk[nl].lo = -T.car_diff_lock_nm; lk[nl].hi = T.car_diff_lock_nm;
                nl++;
            }
            for (int i = RL; i <= RR; i++) {
                if (cap_hb > 1e-3f) nl += brake_link(&lk[nl], i - RL + 1, cap_hb + cap_fb[i], vx[i], 0, 0);
                else nl += brake_link(&lk[nl], i - RL + 1, cap_fb[i], vx[i], abs_on, T.car_abs_rear_slip);   /* build A: the rears may be let slide further */
            }
            solve_speeds(3, mm, jj, tt, ww, lk, nl, h, kl);
            c->engine_ang_vel = clampf(ww[0], 0, 1.6f * T.car_limit_rpm / RPM_PER_RADS);
            c->wheel[RL].ang_vel = ww[1];
            c->wheel[RR].ang_vel = ww[2];
        }

        /* tyre forces at the new wheel speeds move the body */
        V3 force = v3(0, 0, 0);
        float torque = 0;
        for (int i = 0; i < 4; i++) {
            Wheel *w = &c->wheel[i];
            TyreOut t = tyre(vx[i], vy[i], w->ang_vel, load[i], mu[i]);
            V3 f = vadd(vscale(wfw[i], t.fx), vscale(wrt[i], t.fy));
            force = vadd(force, f);
            torque += rw[i].z * f.x - rw[i].x * f.z;
            w->rotation = fmodf(w->rotation + w->ang_vel * h, TWO_PI);
            w->load = load[i];
            last[i] = t;
        }
        float sp = vlen(vel);
        force = vsub(force, vscale(vel, 0.5f * T.car_air_density * T.car_cda * sp));
        torque -= T.car_yaw_damp * wy;

        vel = vadd(vel, vscale(force, h / m));
        vel.y = 0;
        wy += torque / iz * h;
        com_w = vadd(com_w, vscale(vel, h));
        yaw += wy * h;

        /* smoothed acceleration in the car's frame, for the next substep's load transfer */
        float k = clamp01(h / maxf(T.car_load_lag, 0.001f));
        c->acc_fwd += (vdot(force, fwd) / m - c->acc_fwd) * k;
        c->acc_lat += (vdot(force, right) / m - c->acc_lat) * k;
    }

    /* ---- write back */
    if (yaw > 3.14159265f || yaw < -3.14159265f) yaw = atan2f(sinf(yaw), cosf(yaw));
    c->body.rot = qaxis(v3(0, 1, 0), yaw);
    c->body.vel = vel;
    c->body.angvel = v3(0, wy, 0);
    fwd = dir_fwd(yaw);
    right = dir_right(yaw);
    float py = c->body.pos.y;
    c->body.pos = vsub(com_w, vadd(vscale(right, g.com.x), vscale(fwd, g.com.z)));
    c->body.pos.y = py;
    for (int i = 0; i < 4; i++) {
        c->wheel[i].slip_ratio = last[i].sx;
        c->wheel[i].slip_angle = last[i].sa;
        c->wheel[i].slip_velo = last[i].slip_v;
    }
    c->rpm = c->engine_ang_vel * RPM_PER_RADS;
    c->velo = fabsf(vdot(vel, fwd));
    if (!c->changing_gear) c->neutral = c->gear == 0;
    c->time += dt;
    fill_wheels(c);
}

/* ================================================================ self test */

static float fwd_speed(const Car *c) { return car_local_velocity(c).z; }

static void new_car(Car *c)
{
    memset(c, 0, sizeof(*c));
    car_setup(c, NULL);
    car_reset(c, v3(0, 0.75f, 0), 0);
}

/* put the car in motion: speed (m/s), slide (deg, + = travel to the right of the nose), yaw rate */
static void set_motion(Car *c, float speed, float slide_deg, float yaw_rate)
{
    float yaw = car_yaw(c), tr = yaw + slide_deg * DEG2RADF;
    c->body.vel = vscale(dir_fwd(tr), speed);
    c->body.angvel = v3(0, yaw_rate, 0);
    float vf = fwd_speed(c);
    for (int i = 0; i < 4; i++) c->wheel[i].ang_vel = vf / T.car_wheel_radius;
    int best = 1;
    for (int gg = 1; gg <= c->ngears; gg++) {
        float rpm = vf / T.car_wheel_radius * gear_ratio(gg) * RPM_PER_RADS;
        if (rpm > 1700.0f) best = gg;
    }
    c->gear = best;
    c->ratio = gear_ratio(best);
    c->neutral = 0;
    c->changing_gear = 0;
    c->engine_ang_vel = maxf(vf / T.car_wheel_radius * c->ratio, T.car_idle_rpm / RPM_PER_RADS);
    c->rpm = c->engine_ang_vel * RPM_PER_RADS;
}

static int car_finite(const Car *c)
{
    const Body *b = &c->body;
    float v[] = { b->pos.x, b->pos.y, b->pos.z, b->rot.x, b->rot.y, b->rot.z, b->rot.w, b->vel.x, b->vel.y, b->vel.z,
                  b->angvel.y, c->steering, c->steer_angle, c->throttle, c->brake, c->handbrake, c->velo, c->slide_deg,
                  c->slide, c->ratio, c->rpm, c->engine_ang_vel, c->clutch_position, c->acc_fwd, c->acc_lat };
    for (size_t i = 0; i < sizeof(v) / sizeof(v[0]); i++)
        if (!isfinite(v[i])) return 0;
    for (int i = 0; i < 4; i++) {
        const Wheel *w = &c->wheel[i];
        float u[] = { w->centre.x, w->centre.z, w->hit_point.x, w->right_dir.x, w->right_dir.z, w->rotation, w->ang_vel,
                      w->load, w->slip_angle, w->slip_ratio, w->slip_velo };
        for (size_t k = 0; k < sizeof(u) / sizeof(u[0]); k++)
            if (!isfinite(u[k])) return 0;
    }
    return vlen(b->vel) < 150.0f && fabsf(b->angvel.y) < 50.0f;
}

/* A driver that holds a circle of radius R (centre to the car's right at the start) at a target speed:
 * pure-pursuit steering towards a point on the circle ahead, and a PI speed control on the throttle. */
typedef struct { V3 centre; float R, integ, line_i; } Circle;

static void circle_start(Car *c, Circle *d, float R, float speed)
{
    new_car(c);
    d->R = R;
    d->centre = v3(R, 0, 0);
    d->integ = 0.3f;
    d->line_i = 0;
    set_motion(c, speed, 0, speed / R);
}

static float circle_error(const Car *c, const Circle *d)
{
    float dx = c->body.pos.x - d->centre.x, dz = c->body.pos.z - d->centre.z;
    return sqrtf(dx * dx + dz * dz) - d->R;
}

static void circle_drive(Car *c, Circle *d, float target)
{
    float yaw = car_yaw(c);
    V3 f = dir_fwd(yaw), rt = dir_right(yaw);
    float vf = fwd_speed(c);
    float dx = c->body.pos.x - d->centre.x, dz = c->body.pos.z - d->centre.z;
    float phi = atan2f(dz, dx);
    float ld = clampf(0.6f * fabsf(vf) + 4.0f, 6.0f, 15.0f);
    float pt = phi - ld / d->R; /* the car travels with phi falling */
    V3 p = v3(d->centre.x + d->R * cosf(pt), 0, d->centre.z + d->R * sinf(pt));
    V3 rel = vsub(p, c->body.pos);
    rel.y = 0;
    float lx = vdot(rel, rt), lz = vdot(rel, f);
    float alpha = atan2f(lx, lz), dist = sqrtf(lx * lx + lz * lz);
    float L = T.car_wheelbase;
    float steer_angle = atan2f(2.0f * L * sinf(alpha), maxf(dist, 1.0f));
    CarInput in;
    memset(&in, 0, sizeof(in));
    /* trim on the distance off the line (proportional and integral), so the pursuit's offset goes */
    float err = sqrtf(dx * dx + dz * dz) - d->R;
    d->line_i = clampf(d->line_i + 0.02f * err * FIXED_DT, -0.2f, 0.2f);
    steer_angle += 0.02f * err + d->line_i;
    in.steer = clampf(steer_angle / (T.car_max_steer_deg * DEG2RADF * lock_frac(vf)), -1, 1);
    float e = target - vf;
    d->integ = clampf(d->integ + 0.15f * e * FIXED_DT, 0, 1);
    in.throttle = clamp01(d->integ + 0.4f * e);
    in.brake = clamp01(-0.3f * (e + 1.0f));
    car_step(c, &in);
}

/* Highest steady lateral g on a circle of radius R: the speed is raised slowly until the line
 * cannot be held (more than 1 m off); the result is at the highest speed held for the last 3 s. */
static float skidpad(float R, float *v_held)
{
    Car c;
    Circle d;
    float v = 7.0f, best_g = 0, best_v = 0, ok_since = 0;
    circle_start(&c, &d, R, v);
    for (int step = 0; step < 600 * 50; step++) {
        float t = c.time;
        if (t > 6.0f) v += 0.03f * FIXED_DT;
        circle_drive(&c, &d, v);
        float err = circle_error(&c, &d);
        if (fabsf(err) > 1.0f || c.slide_deg > 20.0f) ok_since = c.time;
        if (c.time - ok_since > 3.0f && c.time > 6.0f) {
            float vs = vlen(c.body.vel), rr = d.R + err;
            float gg = vs * vs / (rr * T.car_gravity);
            if (gg > best_g) { best_g = gg; best_v = vs; }
        }
        if (fabsf(err) > 4.0f || c.slide_deg > 60.0f) break;
    }
    if (v_held) *v_held = best_v;
    return best_g;
}

/* the most a fixed-speed circle runs off its line (m), and the largest slide angle */
static float circle_hold(float R, float speed, float *max_slide)
{
    Car c;
    Circle d;
    float worst = 0, ms = 0;
    circle_start(&c, &d, R, speed);
    for (int step = 0; step < 15 * 50; step++) {
        circle_drive(&c, &d, speed);
        if (c.time > 3.0f) worst = maxf(worst, fabsf(circle_error(&c, &d)));
        ms = maxf(ms, c.slide_deg);
    }
    if (max_slide) *max_slide = ms;
    return worst;
}

/* Slide at 60 km/h (30 degrees, still rotating into the slide), then hands and feet off:
 * seconds until the slide is under 5 degrees and the yaw rate is small, and stays so for 1 s. */
static float catch_time(void)
{
    Car c;
    CarInput in;
    memset(&in, 0, sizeof(in));
    new_car(&c);
    set_motion(&c, 60.0f / 3.6f, 30.0f, -0.4f);
    float caught_at = -1;
    for (int step = 0; step < 10 * 50; step++) {
        car_step(&c, &in);
        int ok = c.slide_deg < 5.0f && fabsf(c.body.angvel.y) < 0.2f && fwd_speed(&c) > 0;
        if (ok && caught_at < 0) caught_at = c.time;
        if (!ok) caught_at = -1;
        if (caught_at >= 0 && c.time - caught_at >= 1.0f) return caught_at;
    }
    return -1;
}

/* from rest, flat out the way a driver does a timed run: full throttle, eased only while the rear
 * wheels spin well past their grip peak. 0-100 time and top speed (km/h) */
static void accel_run(float *t100, float *top)
{
    Car c;
    CarInput in;
    memset(&in, 0, sizeof(in));
    new_car(&c);
    in.throttle = 1;
    float best = 0, gain_t = 0;
    *t100 = -1;
    for (int step = 0; step < 200 * 50; step++) {
        float sr = 0.5f * (c.wheel[RL].slip_ratio + c.wheel[RR].slip_ratio);
        in.throttle = clampf(1.0f - 2.0f * (sr - 2.0f * T.car_peak_slip_ratio), 0.3f, 1.0f);
        car_step(&c, &in);
        float kmh = fwd_speed(&c) * 3.6f;
        if (*t100 < 0 && kmh >= 100.0f) *t100 = c.time;
        if (kmh > best + 0.02f) { best = kmh; gain_t = c.time; }
        if (c.time - gain_t > 10.0f) break;
    }
    *top = best;
}

/* 80-0: time, distance and how long any wheel was locked (slip ratio under -0.5) */
static void brake_run(float *t, float *dist, float *locked)
{
    Car c;
    CarInput in;
    memset(&in, 0, sizeof(in));
    new_car(&c);
    set_motion(&c, 80.0f / 3.6f, 0, 0);
    in.brake = 1;
    V3 p0 = c.body.pos;
    *locked = 0;
    for (int step = 0; step < 30 * 50; step++) {
        car_step(&c, &in);
        for (int i = 0; i < 4; i++)
            if (c.wheel[i].slip_ratio < -0.5f) { *locked += FIXED_DT; break; }
        if (fwd_speed(&c) < 0.05f) break;
    }
    *t = c.time;
    *dist = vlen(vsub(c.body.pos, p0));
}

/* handbrake pulled at 40 km/h with half lock for 0.8 s: the largest slide angle in the next 3 s */
static float handbrake_swing(void)
{
    Car c;
    CarInput in;
    memset(&in, 0, sizeof(in));
    new_car(&c);
    set_motion(&c, 40.0f / 3.6f, 0, 0);
    float worst = 0;
    for (int step = 0; step < 4 * 50; step++) {
        in.steer = c.time < 0.8f ? 0.5f : 0;
        in.handbrake = c.time < 0.8f ? 1 : 0;
        car_step(&c, &in);
        V3 lv = car_local_velocity(&c);
        if (lv.z > 0.5f) worst = maxf(worst, atan2f(fabsf(lv.x), lv.z) * RAD2DEGF);
        else if (vlen(c.body.vel) > 0.5f) worst = maxf(worst, 90.0f);
    }
    return worst;
}

/* build A: the rear axle's slide angle (the rear tyres' mean slip angle), degrees; 90 once going backwards */
static float rear_slide(const Car *c)
{
    if (fwd_speed(c) < 0.5f) return vlen(c->body.vel) > 0.5f ? 90.0f : 0.0f;
    return 0.5f * (fabsf(c->wheel[RL].slip_angle) + fabsf(c->wheel[RR].slip_angle)) * RAD2DEGF;
}

/* Turning in at 50 km/h (0.6 of the lock for 1 s, a light throttle after), with or without a full brake stab
 * for the first 0.3 s: the largest rear slide angle in the next 3 s. */
static float turn_in_swing(int stab)
{
    Car c;
    CarInput in;
    memset(&in, 0, sizeof(in));
    new_car(&c);
    set_motion(&c, 50.0f / 3.6f, 0, 0);
    c.gear = 2; c.ratio = gear_ratio(2);
    float worst = 0;
    for (int step = 0; step < 3 * 50; step++) {
        in.steer = c.time < 1.0f ? 0.6f : 0;
        in.brake = stab && c.time < 0.3f ? 1.0f : 0;
        in.throttle = c.time >= 0.3f ? 0.3f : 0;
        car_step(&c, &in);
        worst = maxf(worst, rear_slide(&c));
    }
    return worst;
}

/* 80-0 on full brakes with a slight, steady steer (0.05): the largest rear slide angle (a stable car stays
 * near the steer's own small angle; a rear-heavy one swaps ends) */
static float brake_straight_slide(void)
{
    Car c;
    CarInput in;
    memset(&in, 0, sizeof(in));
    new_car(&c);
    set_motion(&c, 80.0f / 3.6f, 0, 0);
    in.brake = 1; in.steer = 0.05f;
    float worst = 0;
    for (int step = 0; step < 10 * 50 && fwd_speed(&c) > 1.0f; step++) {
        car_step(&c, &in);
        worst = maxf(worst, rear_slide(&c));
    }
    return worst;
}

/* A throttle push in 2nd in a bend: at kmh, 0.35 of the lock and a light throttle for 1.5 s to settle,
 * then full throttle for 2 s. The largest rear slide angle in the push; *spun = 1 if it went past 80 deg. */
static float power_push(float kmh, int *spun)
{
    Car c;
    CarInput in;
    memset(&in, 0, sizeof(in));
    new_car(&c);
    float saved = T.manual_gears;
    T.manual_gears = 1;   /* held in 2nd */
    set_motion(&c, kmh / 3.6f, 0, 0);
    c.gear = 2; c.ratio = gear_ratio(2);
    c.engine_ang_vel = fabsf(kmh / 3.6f / T.car_wheel_radius * c.ratio);
    for (int i = RL; i <= RR; i++) c.wheel[i].ang_vel = kmh / 3.6f / T.car_wheel_radius;
    float worst = 0;
    *spun = 0;
    for (int step = 0; step < 3.5f * 50; step++) {
        in.steer = 0.35f;
        in.throttle = c.time < 1.5f ? 0.25f : 1.0f;
        car_step(&c, &in);
        if (c.time >= 1.5f) worst = maxf(worst, rear_slide(&c));
    }
    if (worst >= 80.0f) *spun = 1;
    T.manual_gears = saved;
    return worst;
}

/* Donuts (build A): from rest, full lock and full throttle (a handbrake kick for the first kick_s seconds),
 * held for 12 s. Over the last 8 s: turns made, the radius of the circle the body's middle drives, the mean
 * body slide angle and speed. A proper donut keeps turning on a small circle with the tail well out. */
static void donut_run(float kick_s, float *turns, float *radius, float *slide, float *speed)
{
    Car c;
    CarInput in;
    memset(&in, 0, sizeof(in));
    new_car(&c);
    float yaw_sum = 0, prev_yaw = car_yaw(&c), sl = 0, sp = 0; int n = 0;
    float minx = 1e9f, maxx = -1e9f, minz = 1e9f, maxz = -1e9f;
    for (int step = 0; step < 12 * 50; step++) {
        in.steer = -1; in.throttle = 1;
        in.handbrake = c.time < kick_s ? 1.0f : 0.0f;
        car_step(&c, &in);
        float y = car_yaw(&c), d = atan2f(sinf(y - prev_yaw), cosf(y - prev_yaw));
        prev_yaw = y;
        if (c.time > 4.0f) {
            yaw_sum += d;
            V3 lv = car_local_velocity(&c);
            sl += vlen(c.body.vel) > 0.5f ? atan2f(fabsf(lv.x), fabsf(lv.z)) * RAD2DEGF : 0; sp += vlen(c.body.vel); n++;
            minx = minf(minx, c.body.pos.x); maxx = maxf(maxx, c.body.pos.x); minz = minf(minz, c.body.pos.z); maxz = maxf(maxz, c.body.pos.z);
        }
    }
    *turns = fabsf(yaw_sum) / TWO_PI;
    *radius = 0.25f * ((maxx - minx) + (maxz - minz));
    *slide = n ? sl / n : 0; *speed = n ? sp / n * 3.6f : 0;
}

static unsigned rng_state = 12345u;
static float rnd(void)
{
    rng_state = rng_state * 1664525u + 1013904223u;
    return (float)((rng_state >> 8) & 0xFFFFFF) / 16777216.0f;
}

/* 10 minutes of random inputs (automatic then manual), with random knocks to the velocity the way the
 * game adds slopes and ditches. Returns the step that went bad, or -1 if all stayed finite. */
static int random_run(void)
{
    Car c;
    CarInput in;
    memset(&in, 0, sizeof(in));
    new_car(&c);
    float saved_manual = T.manual_gears;
    int bad = -1;
    for (int step = 0; step < 10 * 60 * 50; step++) {
        T.manual_gears = step >= 5 * 60 * 50 ? 1.0f : 0.0f;
        if (rnd() < 0.04f) {
            in.throttle = rnd() < 0.6f ? rnd() : 0;
            in.brake = rnd() < 0.3f ? rnd() : 0;
            in.handbrake = rnd() < 0.15f ? 1 : 0;
            in.clutch = rnd() < 0.1f ? 1 : 0;
            in.steer = rnd() * 2 - 1;
        }
        in.shift = rnd() < 0.01f ? (rnd() < 0.6f ? 1 : -1) : 0;
        if (rnd() < 0.01f) c.body.vel = vadd(c.body.vel, v3(rnd() * 4 - 2, 0, rnd() * 4 - 2));
        car_step(&c, &in);
        if (!car_finite(&c)) { bad = step; break; }
    }
    T.manual_gears = saved_manual;
    return bad;
}

int car_selftest(void)
{
    static const struct { float t100, top, g, catch_s; } target[CAR_NPRESETS] = {   /* best and drift: as measured for build A */
        { 9.5f, 146, 0.85f, 1.0f }, { 10.4f, 146, 0.76f, 1.7f }, { 10.3f, 146, 0.74f, 2.7f } };
    Tuning saved = T;
    int misses = 0;
    printf("car selftest (flat gravel). targets: 0-100 / top / skidpad g / catch within 10%%, 80-0 grip-limited (2.5-5.5 s)\n");
    for (int p = 0; p < CAR_NPRESETS; p++) {
        const char *name;
        const char *text = car_preset(p, &name);
        T = saved;
        tuning_apply_text(text);
        if (getenv("CARTEST_SET")) {   /* exploring: "key = value; key = value" on top of every preset */
            char buf[1024]; snprintf(buf, sizeof(buf), "%s", getenv("CARTEST_SET"));
            for (char *q = buf; *q; q++) if (*q == ';') *q = '\n';
            tuning_apply_text(buf);
        }
        float t100, top, vh, bt, bd, locked, ms;
        accel_run(&t100, &top);
        float g = skidpad(30.0f, &vh);
        float ct = catch_time();
        brake_run(&bt, &bd, &locked);
        float swing = handbrake_swing();
        float bend = circle_hold(15.0f, 40.0f / 3.6f, &ms);
        printf("%-6s 0-100 %.1f s, top %.0f km/h, skidpad %.2f g, catch %.1f s, 80-0 %.1f s / %.0f m\n",
               name, t100, top, g, ct, bt, bd);
        printf("       (skidpad held at %.0f km/h; wheels locked %.2f s in the stop; handbrake at 40 km/h swings %.0f deg;"
               " 15 m bend at 40 km/h: %.1f m off line, slide %.0f deg)\n", vh * 3.6f, locked, swing, bend, ms);
        {   /* build A: brake balance and power oversteer, as rear slide angles (degrees) */
            int sp30, sp40, sp50;
            float ti0 = turn_in_swing(0), ti1 = turn_in_swing(1), bs = brake_straight_slide();
            float p30 = power_push(30, &sp30), p40 = power_push(40, &sp40), p50 = power_push(50, &sp50);
            float dt0, dr0, ds0, dv0, dt1, dr1, ds1, dv1;
            donut_run(0, &dt0, &dr0, &ds0, &dv0); donut_run(0.5f, &dt1, &dr1, &ds1, &dv1);
            printf("       donut, full lock and throttle from rest: %.1f turns in 8 s on a %.1f m circle, slide %.0f deg, %.0f km/h;"
                   "  with a handbrake kick: %.1f turns, %.1f m, %.0f deg, %.0f km/h\n", dt0, dr0, ds0, dv0, dt1, dr1, ds1, dv1);
            printf("       turn-in at 50 km/h: steer only %.0f deg, with a brake stab %.0f;  80-0 braking with a slight steer %.1f deg;"
                   "  2nd-gear throttle push at 30/40/50 km/h %.0f%s / %.0f%s / %.0f%s deg\n", ti0, ti1, bs,
                   p30, sp30 ? " SPUN" : "", p40, sp40 ? " SPUN" : "", p50, sp50 ? " SPUN" : "");
        }
        float got[4] = { t100, top, g, ct }, want[4] = { target[p].t100, target[p].top, target[p].g, target[p].catch_s };
        for (int k = 0; k < 4; k++)
            if (!(got[k] > 0) || fabsf(got[k] - want[k]) > 0.1f * want[k]) misses++;
        if (!(bt > 2.5f && bt <= 5.5f)) misses++; /* grip-limited, so shorter with more grip */
    }
    T = saved;
    int bad = random_run();
    if (bad < 0) printf("stability: 10 min of random inputs, every value finite and bounded\n");
    else { printf("stability: FAILED at step %d\n", bad); misses++; }
    T = saved;
    printf("%s (%d off target)\n", misses ? "selftest: some values off target" : "selftest: all on target", misses);
    return misses ? 1 : 0;
}
