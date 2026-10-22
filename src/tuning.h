/* Live-tunable values. The numbers live in tuning-bush.txt (the game) and tuning-car.txt (the ute's
 * handling). The build copies them into the program (build/tuning_defaults.h, not in git), so the game
 * needs no tuning file to run; data/tuning-live.txt, if present, is read on top (F5 re-reads it, F6
 * writes it) and may hold as few lines as wanted. */
#ifndef TUNING_H
#define TUNING_H

#define TUNE_SCALARS \
    /* the ute's box (for trunks and posts, when the model has no outline) and drawn wheels */ \
    X(body_half_w) X(body_front) X(body_rear) X(body_bottom) X(body_top) X(wheel_radius) X(wheel_width) \
    /* tyre marks, sound, input */ \
    X(mark_slip_from) X(mark_life) X(mark_fade) X(skid_sound_from) \
    X(synth_volume) X(synth_engine) X(synth_tyre) X(synth_blip) \
    X(snd_volume) X(snd_engine) X(snd_gravel) X(snd_slide) X(snd_bush) X(snd_kookaburra) X(snd_music) X(snd_idle_rpm) X(snd_load_rpm) X(snd_cross_from_rpm) X(snd_cross_to_rpm) X(use_stick) X(pad_swap_ab) X(manual_gears) \
    /* scoring and run rules */ \
    X(score_rate) X(score_dead_deg) X(score_ref_deg) X(score_curve) X(score_cap_deg) X(score_mult_deg) X(score_mult_from_deg) X(score_mult_hold) X(score_mult_max) \
    X(score_autobank) X(score_gap) X(score_bank_after) X(score_min_deg) X(ditch_rule) X(ditch_wheels) X(ditch_margin) X(ditch_time) \
    X(donut_bonus) X(donut_min_deg) X(donut_max_kmh) X(donut_box) X(donut_carpark) X(donut_turn_deg) X(yodel_time) X(yodel_score) X(yodel_duck) \
    X(wrong_way_time) \
    X(edge_dist) X(edge_w) X(edge_h) X(edge_stop) X(edge_hut_gap) X(edge_trees) X(edge_scrub) X(offroad_from) X(checkpoint_every) X(deer_seed) X(deer_random) X(minimap) X(offroad_reset) X(score_bank_button) X(log_keep) X(wood_full) \
    /* build C: Explorer mode, the fires and the night, the hamper, the creek way */ \
    X(bollard_gap) X(wood_every) X(wood_bridge) X(fire_e) X(fire_n) X(hut_reach) X(night_lights) X(fx_density) X(find_glow) X(fw_whistle) X(fw_finale) \
    /* build D: Low Range, the creek's markers and camp, the dead trees */ \
    X(low_max_kmh) X(creek_bench_h) X(creek_ramp) X(creek_rock_h) X(creek_wall_from) X(creek_scrub) X(rock_drag) X(rock_decel) X(marker_reach) X(camp_reach) X(stag_speed) X(stag_keep) \
    X(body_physics) X(susp_k) X(susp_c) X(body_pitch_i) X(body_roll_i) X(belly_h) X(bumper_h) X(belly_k) X(belly_mu) X(roll_over) X(grip_light) X(knock_max) X(fall_step_h) X(fall_ledge) X(creek_stones) X(moss_scale) X(wet_smash) X(saw_on) X(saw_wear) X(saw_razor_q) \
    X(trials_on) X(trials_knock) X(trials_land) X(trials_slow) X(gates_n) X(dmg_tyre) X(dmg_body) X(flat_grip) X(flat_drag) X(flat_pull) \
    /* the road's cross-section, the bush, camera and look */ \
    X(road_width) X(verge_w) X(ditch_w) X(ditch_depth) X(bank_w) X(tree_density) X(scrub_density) \
    X(ditch_drag) X(ditch_decel) X(rough_drag) X(grade_gravity) X(tree_min_speed) \
    X(chase_dist) X(chase_height) X(chase_look) X(chase_fov) X(chase_follow) X(chase_turn) \
    X(haze_near) X(haze_far) X(view_radius) X(pixel_mode) X(dither) \
    X(lean_roll) X(lean_pitch) X(lean_hz) X(lean_damp) X(dust_rate) X(dust_life) X(dust_size) \
    X(mid_density) X(ground_density) X(near_dist) X(shadows) X(shadow_box) X(shadow_res) X(shadow_bias) \
    X(far_land) X(haze_len) X(haze_max) X(road_detail) X(clouds) \
    /* where the ute is parked at the start, metres E/N of the hut, and the way it faces (compass degrees) */ \
    X(start_e) X(start_n) X(start_brg) \
    /* the ute's handling, src/car.c (tuning-car.txt) */ \
    X(car_mass) X(car_yaw_inertia) X(car_com_height) X(car_front_share) X(car_gravity) \
    X(car_wheelbase) X(car_track_front) X(car_track_rear) X(car_wheel_y) \
    X(car_cda) X(car_air_density) X(car_rolling) \
    X(car_wheel_radius) X(car_wheel_inertia) X(car_mu_front) X(car_mu_rear) X(car_long_grip) \
    X(car_peak_slip_angle_deg) X(car_peak_slip_ratio) X(car_tail) X(car_tail_width) X(car_spin_side_loss) X(car_spin_side_to) X(car_load_sens) \
    X(car_low_speed) X(car_roll_front) X(car_load_lag) \
    X(car_max_steer_deg) X(car_steer_rate) X(car_steer_return_rate) X(car_steer_speed_ref) X(car_steer_min_frac) \
    X(car_steer_assist) X(car_yaw_damp) X(car_slide_from_deg) X(car_slide_full_deg) \
    X(car_pedal_time) X(car_pedal_release_time) \
    X(car_brake_nm) X(car_brake_front) X(car_handbrake_nm) X(car_abs) X(car_abs_slip) X(car_abs_rear_slip) X(car_lock_damping) \
    X(car_idle_rpm) X(car_limit_rpm) X(car_limit_hyst_rpm) X(car_torque_idle) X(car_torque_nm) \
    X(car_torque_from_rpm) X(car_torque_to_rpm) X(car_power_kw) X(car_power_rpm) X(car_torque_at_limit) \
    X(car_driveline_eff) X(car_engine_inertia) X(car_engine_fric) X(car_engine_fric_rpm) X(car_idle_gain) \
    X(car_clutch_nm) X(car_clutch_bite_rpm) X(car_clutch_full_rpm) \
    X(car_gear_r) X(car_gear_1) X(car_gear_2) X(car_gear_3) X(car_gear_4) X(car_gear_5) X(car_final_drive) \
    X(car_shift_up_rpm) X(car_shift_down_rpm) X(car_shift_time) X(car_diff_lock_nm) X(car_substeps) \
    /* build D: Low Range (Explorer only) */ \
    X(car_low_ratio) X(car_low_gears) X(car_low_rear_lock) X(car_low_transfer_nm) X(car_low_pedal_time) X(car_low_throttle_curve)

typedef struct Tuning {
#define X(n) float n;
    TUNE_SCALARS
#undef X
    float gears[7]; /* reverse, neutral (0), then forward gears; filled from car_gear_r, car_gear_1.. by car_setup */
} Tuning;

extern Tuning T;

int tuning_count(void);
const char *tuning_name(int i);
float *tuning_ptr(int i);
/* Puts every value back to the one built into the program. */
void tuning_defaults(void);
/* Built-in values first, then the file's lines on top. Returns the number of keys read from the
 * file, or -1 if it could not be opened (the built-in values are then in force). */
int tuning_load(const char *path);
int tuning_save(const char *path);
/* "key = value" lines applied over the values in force; returns how many keys were known */
int tuning_apply_text(const char *text);
/* names of scalar keys the built-in values do not set, comma separated (empty when complete) */
const char *tuning_missing(void);
#endif
