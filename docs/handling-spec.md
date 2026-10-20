# Bush Drift handling: a clean specification (session 3b)

This spec is for writing the car's handling **from scratch**. It holds only real-world figures, the
behaviour the game wants (measured as numbers you can test), and the interface the rest of the game calls.
It says nothing about how any other game does it. Whoever writes `src/car.c` works from this file,
`src/car.h`, `src/vec.h` and `src/tuning.h`, and from general vehicle-dynamics knowledge.

## The car

A Toyota LandCruiser 79 single-cab ute on a dirt road. Real figures:
- Mass 2,300 kg (kerb 2,130 kg + tub + driver). Wheelbase 3.18 m. Track 1.51 m front, 1.46 m rear.
  Length 5.25 m, width 1.87 m. Centre of mass about 0.75 m above the ground, a little forward of the
  wheelbase middle (front share of static load about 55%).
- Engine: 2.8 turbo diesel, 150 kW (204 PS) at 3,000 rpm, 500 Nm from 1,600 to 2,800 rpm. Idle 750 rpm,
  limiter 4,000 rpm.
- Gearbox: five forward gears, ratios 4.31, 2.33, 1.44, 1.00, 0.84; reverse 4.31; final drive 5.0;
  rear-wheel drive (the real one is part-time 4WD; the game drives the rears). Tyres about 0.45 m radius
  (35-inch mud-terrains).
- Brakes: discs front, drums rear. Handbrake on the rear wheels.
- Steering lock about 31 degrees at the front wheels.

## The game's model (keep it simple)

- **Flat plane.** The car moves on a flat plane (x east, z north, y up; metres, seconds, radians). The game
  draws hills, body lean and wheel heights itself from the road surface; the handling only needs x, z and
  yaw (with the body kept at a constant height, y = 0.75 m for the body origin, rotation about y only).
  Gravity along a slope and the ditch's drag are applied by the game to `body.vel` between steps.
- **Fixed step.** `car_step` advances exactly `FIXED_DT` = 0.02 s (50 Hz).
- **Four wheels**, each with its own load (static share + longitudinal and lateral load transfer from the
  centre-of-mass height), slip angle and slip ratio, and a tyre force curve with a peak and a falloff past
  it (gravel: low peak, broad and soft). Combine longitudinal and lateral force so that a wheel spinning or
  locked loses side grip (a friction ellipse or similar). Wheel spin is a state (angular velocity per wheel).
- **Drivetrain**: engine torque curve from the figures above, rpm from the driven wheels through the gear
  when the clutch is in, an idle governor and a rev limiter, an open or lightly locked rear differential.
  **Automatic gearbox** by default (up at about 3,600 rpm, down at about 1,600 rpm, a short shift time with
  drive cut), with the manual option (`in.shift`) when the tuning value `manual_gears` is 1. A clutch pedal
  (`in.clutch`) for clutch kicks is a nice-to-have.
- **Inputs are smoothed**: steering moves towards its target at a rate (faster back to centre), less lock
  at speed; throttle and brake ramp in over about 0.1 s.
- **Handbrake** locks the rear wheels (strong brake torque on the rears only), which is how a driver starts
  a slide on a tight bend.
- **Drift feel.** The car should be catchable: countersteer works, and lifting off lets it straighten. A
  light, honest stability help is allowed (for example a small yaw damping or a gentle steering assist that
  points the front wheels along the direction of travel when sliding), with its strength a tuning value.

## Tuning

Every number the handling uses comes from the tuning list (`src/tuning.h`, the `X(...)` list), so it can be
changed while driving. Use the `car_` prefix for new names (for example `car_mass`, `car_power_kw`,
`car_peak_mu`). Their values go in `tuning-car.txt` (to be written, committed, comments say what each one
is). Three handling presets change only the grip-related values: **safer**, **best** (default) and **wild**.

## Behaviour targets (on flat gravel, measured by `--cartest`)

| preset | 0–100 km/h | top speed | steady grip on a 30 m radius skidpad | slide at 60 km/h, then let go: caught within |
|---|---|---|---|---|
| safer | about 9.5 s | about 146 km/h | about 0.85 g | about 1.0 s |
| best | about 9.9 s | about 146 km/h | about 0.75 g | about 1.4 s |
| wild | about 10.6 s | about 146 km/h | about 0.62 g | about 2.0 s |

(Raised 20 Oct after the owner's play test: the old 0.40–0.67 g "feels like ice". Packed gravel with good
tyres gives about 0.7–0.8 g; the car should grip well until pushed, then slide progressively.)

Also:
- Braking from 80 km/h to a stop in about 4–5 s on gravel without the wheels locking for long (brake force
  limited by grip; an ABS-like cap is fine).
- At 40 km/h on a 15 m radius bend, *best* is right at the edge (slides if pushed), *safer* holds on.
- A handbrake pull at 40 km/h with some lock should swing the rear out by 30–60 degrees.
- No numerical blow-ups: 10 minutes of random inputs at the fixed step keep every value finite.

## The interface

`src/car.h` (written, keep it as is unless something is truly missing):

```c
void  car_setup(Car *c, const V3 wheel_anchor[4]);   /* (re)read the tuning; keeps motion state */
void  car_reset(Car *c, V3 pos, float yaw_rad);       /* parked, engine idling, first gear */
void  car_step(Car *c, const CarInput *in);           /* one FIXED_DT step */
V3    car_local_velocity(const Car *c);               /* x right, y up, z forward, m/s */
float car_yaw(const Car *c);                          /* radians, 0 = facing +z (north), turning towards +x is positive */
```

`wheel_anchor[i]` are the wheel centres in the car's frame (x right, y up, z forward), in the order front
left, front right, rear left, rear right (`FL, FR, RL, RR`).

## Tests the implementation should ship

`car_selftest()` in `src/car.c` (called by `./bush-drift --cartest`): for each preset, print 0–100 time,
top speed, the 30 m skidpad g (a steady circle, speed raised until it can't hold the line), the slide-catch
time, and the 80–0 braking time and distance. And a stability run of random inputs. Print one line per
preset in the format `NAME  0-100 X s, top X km/h, skidpad X g, catch X s, 80-0 X s / X m`.
