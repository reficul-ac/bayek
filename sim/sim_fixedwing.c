#include "sim_fixedwing.h"

#include "math_utils.h"

#include <math.h>

static quat_t quat_conjugate(quat_t q) {
  quat_t out = {q.w, -q.x, -q.y, -q.z};
  return out;
}

static vec3_t rotate_ned_to_body(quat_t body_to_ned, vec3_t ned) {
  return quat_rotate_vec3(quat_conjugate(body_to_ned), ned);
}

static int vec3_is_finite_local(vec3_t v) {
  return real_is_finite(v.x) && real_is_finite(v.y) && real_is_finite(v.z);
}

static real_t soft_limited_alpha(real_t alpha_rad, real_t stall_alpha_rad) {
  real_t limit = stall_alpha_rad;
  if (limit <= 0.01f) {
    limit = 0.01f;
  }
  return limit * (real_t)tanhf(alpha_rad / limit);
}

static void fixedwing_forces_moments(const sim_fixedwing_state_t *state,
                                     const sim_fixedwing_params_t *params,
                                     const actuator_cmd_t *actuator,
                                     vec3_t *force_body_n,
                                     vec3_t *moment_body_nm,
                                     real_t *airspeed_mps) {
  vec3_t vel_body = rotate_ned_to_body(state->body.attitude_body_to_ned, state->body.velocity_ned_mps);
  real_t u = vel_body.x;
  real_t v = vel_body.y;
  real_t w = vel_body.z;
  real_t speed = vec3_norm(vel_body);
  real_t qbar;
  real_t alpha;
  real_t alpha_eff;
  real_t cl;
  real_t cd;
  real_t lift_n;
  real_t drag_n;
  real_t side_n;

  if (speed < 0.1f) {
    speed = 0.1f;
  }
  qbar = 0.5f * params->core.air_density_kgpm3 * speed * speed;
  alpha = (real_t)atan2f(w, u);
  alpha_eff = soft_limited_alpha(alpha, params->stall_alpha_rad);
  cl = params->lift_cl0 + params->lift_cl_alpha * alpha_eff + params->lift_cl_elevator * actuator->elevator;
  cd = params->drag_cd0 + params->drag_cd_alpha * alpha_eff * alpha_eff + 0.02f * cl * cl;
  lift_n = qbar * params->wing_area_m2 * cl;
  drag_n = qbar * params->wing_area_m2 * cd;
  side_n = -0.25f * qbar * params->wing_area_m2 * (v / speed) + 0.20f * qbar * params->wing_area_m2 * actuator->rudder;

  force_body_n->x = params->max_thrust_n * clamp_real(actuator->motor, 0.0f, 1.0f) - drag_n;
  force_body_n->y = side_n;
  force_body_n->z = -lift_n;

  moment_body_nm->x = params->roll_aileron_nm * actuator->aileron -
                      params->rate_damping_nms.x * state->body.omega_body_rps.x;
  moment_body_nm->y = params->pitch_elevator_nm * actuator->elevator -
                      0.45f * alpha_eff -
                      params->rate_damping_nms.y * state->body.omega_body_rps.y;
  moment_body_nm->z = params->yaw_rudder_nm * actuator->rudder -
                      params->rate_damping_nms.z * state->body.omega_body_rps.z;
  *airspeed_mps = speed;
}

void sim_fixedwing_default_params(sim_fixedwing_params_t *params) {
  if (!params) {
    return;
  }
  sim6dof_default_params(&params->core);
  params->wing_area_m2 = 0.45f;
  params->wing_span_m = 1.8f;
  params->mean_chord_m = 0.25f;
  params->max_thrust_n = 18.0f;
  params->drag_cd0 = 0.035f;
  params->drag_cd_alpha = 0.80f;
  params->lift_cl0 = 0.18f;
  params->lift_cl_alpha = 4.2f;
  params->lift_cl_elevator = 0.35f;
  params->stall_alpha_rad = 0.28f;
  params->roll_aileron_nm = 1.6f;
  params->pitch_elevator_nm = 1.0f;
  params->yaw_rudder_nm = 0.6f;
  params->rate_damping_nms.x = 0.35f;
  params->rate_damping_nms.y = 0.45f;
  params->rate_damping_nms.z = 0.30f;
}

void sim_fixedwing_init_default(sim_fixedwing_state_t *state) {
  if (!state) {
    return;
  }
  sim6dof_init_level(&state->body, 120.0f, 18.0f);
  state->body.actuator_state.motor = 0.55f;
  state->last_force_body_n.x = 0.0f;
  state->last_force_body_n.y = 0.0f;
  state->last_force_body_n.z = -2.5f * 9.80665f;
  state->last_moment_body_nm.x = 0.0f;
  state->last_moment_body_nm.y = 0.0f;
  state->last_moment_body_nm.z = 0.0f;
  state->last_airspeed_mps = 18.0f;
}

int sim_fixedwing_step(sim_fixedwing_state_t *state,
                       const sim_fixedwing_params_t *params,
                       const actuator_cmd_t *cmd,
                       real_t dt_s) {
  vec3_t force_body_n;
  vec3_t moment_body_nm;
  real_t airspeed_mps;
  if (!state || !params || !cmd) {
    return 0;
  }
  fixedwing_forces_moments(state, params, &state->body.actuator_state, &force_body_n, &moment_body_nm, &airspeed_mps);
  if (!sim6dof_step(&state->body, &params->core, cmd, force_body_n, moment_body_nm, dt_s)) {
    return 0;
  }
  state->last_force_body_n = force_body_n;
  state->last_moment_body_nm = moment_body_nm;
  state->last_airspeed_mps = airspeed_mps;
  return sim_fixedwing_state_is_valid(state);
}

void sim_fixedwing_make_fsw_input(const sim_fixedwing_state_t *state,
                                  const rc_input_t *rc,
                                  real_t dt_s,
                                  uint32_t timestamp_us,
                                  fsw_input_t *in) {
  real_t altitude_m;
  if (!state || !rc || !in) {
    return;
  }
  altitude_m = -state->body.position_ned_m.z;
  in->dt_s = dt_s;
  in->rc = *rc;
  in->imu.gyro_rps = state->body.omega_body_rps;
  in->imu.accel_mps2 = state->body.specific_force_body_mps2;
  in->imu.timestamp_us = timestamp_us;
  in->gps.lat_deg = 0.0f;
  in->gps.lon_deg = 0.0f;
  in->gps.alt_m = altitude_m;
  in->gps.vel_mps = state->body.velocity_ned_mps;
  in->gps.fix_valid = 1U;
  in->gps.timestamp_us = timestamp_us;
  in->baro.pressure_pa = 101325.0f;
  in->baro.altitude_m = altitude_m;
  in->baro.timestamp_us = timestamp_us;
  in->airspeed.true_airspeed_mps = state->last_airspeed_mps;
  in->airspeed.timestamp_us = timestamp_us;
}

int sim_fixedwing_state_is_valid(const sim_fixedwing_state_t *state) {
  if (!state) {
    return 0;
  }
  return sim6dof_state_is_valid(&state->body) &&
         vec3_is_finite_local(state->last_force_body_n) &&
         vec3_is_finite_local(state->last_moment_body_nm) &&
         real_is_finite(state->last_airspeed_mps) &&
         state->last_airspeed_mps >= 0.0f &&
         state->last_airspeed_mps < 120.0f;
}
