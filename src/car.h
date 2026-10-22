/* The ute's handling: a planar rigid body on four tyres, an engine, a gearbox and a rear differential.
 * Written from scratch from docs/handling-spec.md (session 3b). No rendering or library calls in here. */
#ifndef CAR_H
#define CAR_H
#include "vec.h"

#define FIXED_DT 0.02f   /* seconds per physics step (50 Hz) */
enum { FL, FR, RL, RR };

typedef struct {
    float throttle, brake, steer, handbrake; /* raw driver input: 0..1, 0..1, -1 (left)..1 (right), 0..1 */
    float clutch;                             /* pedal: 0 = up, 1 = fully in */
    int shift;                                /* manual gearbox only: +1 up, -1 down, for one step */
} CarInput;

typedef struct {
    V3 pos;        /* body origin, world (y stays at the ride height) */
    Q rot;         /* about y only */
    V3 vel;        /* world, m/s */
    V3 angvel;     /* world, rad/s (y only) */
} Body;

typedef struct {
    V3 anchor;          /* wheel centre in the car's frame */
    V3 centre;          /* wheel centre, world (for drawing) */
    V3 hit_point;       /* tyre contact point, world */
    V3 right_dir;       /* axle direction (steered), world, unit */
    float rotation;     /* spin angle, radians, for drawing */
    float ang_vel;      /* spin rate, rad/s */
    float load;         /* vertical load, N */
    float slip_angle, slip_ratio;
    float slip_velo;    /* how fast the contact patch slides over the ground, m/s (marks, dust, sound) */
    int on_ground;      /* always 1 on the flat plane */
    float grip_lost;    /* build D, the sprung body (main.c, body_physics): share of the tyre's grip lost to the wheel being light
                         * (1 = in the air; below 0 = pressed down harder). 0 leaves the flat-plane handling exactly as it was */
} Wheel;

typedef struct {
    Body body;
    Wheel wheel[4];
    CarInput in;                 /* the last raw input */
    float steering;              /* smoothed, -1..1 */
    float steer_angle;           /* front wheel angle, radians */
    float throttle, brake, handbrake;   /* smoothed, 0..1 */
    float velo;                  /* |forward speed|, m/s */
    float slide_deg;             /* angle between heading and travel, degrees, going forwards (0 otherwise) */
    float slide;                 /* 0 = gripping .. 1 = fully sideways (tuning car_slide_from_deg .. car_slide_full_deg) */
    int gear;                    /* -1 reverse, 0 neutral, 1..ngears */
    int ngears, neutral;         /* neutral: 1 while no gear is engaged (also mid-shift) */
    int changing_gear;           /* 1 during a shift */
    float ratio;                 /* gear ratio times final drive (0 in neutral) */
    float rpm, engine_ang_vel;   /* engine speed */
    float clutch_position;       /* 0 = engaged .. 1 = disengaged */
    int rev_limiter_triggered;   /* 1 on the steps the limiter cuts */
    float time;                  /* seconds since car_reset */
    float max_steer;             /* radians */
    int low;                     /* build D: Low Range (4L): all four wheels driven, the gearing times car_low_ratio, the rear
                                  * diff locked. Set only through car_set_low; Explorer only, so Drift never sees it */
    /* private to car.c: state the step needs between calls */
    float shift_timer;           /* seconds left of a gear change */
    float acc_fwd, acc_lat;      /* body acceleration in the car's frame, smoothed (load transfer) */
} Car;

void car_setup(Car *c, const V3 wheel_anchor[4]);   /* (re)read the tuning; keeps motion state */
void car_reset(Car *c, V3 pos, float yaw_rad);       /* parked, engine idling, first gear */
void car_step(Car *c, const CarInput *in);           /* one FIXED_DT step */
void car_set_low(Car *c, int on);                    /* build D: Low Range on or off; the gearbox picks the gear to suit */
float car_overall_ratio(const Car *c);              /* build D: the gear's overall ratio as driven (low range included), 0 in neutral */
V3 car_local_velocity(const Car *c);                 /* x right, y up, z forward, m/s */
float car_yaw(const Car *c);                         /* radians, 0 = facing +z, towards +x positive */
int car_selftest(void);                              /* --cartest: the targets in docs/handling-spec.md */
#define CAR_NPRESETS 3
/* Handling preset p (0 safer, 1 best, 2 drift): "key = value" lines of grip values to apply on top of
 * the tuning (tuning_apply_text). Sets *name if name is not NULL. NULL for an unknown p. */
const char *car_preset(int p, const char **name);
#endif
