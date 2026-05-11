#include "sim6dof.h"

#include "math_utils.h"

#include <math.h>

static int vec3_is_finite(vec3_t v) {
  return real_is_finite(v.x) && real_is_finite(v.y) && real_is_finite(v.z);
}

static quat_t quat_integrate_body_rate(quat_t q, vec3_t omega_body_rps, real_t dt_s) {
  quat_t omega = {0.0f, omega_body_rps.x, omega_body_rps.y, omega_body_rps.z};
  quat_t qdot = quat_multiply(q, omega);
  q.w += 0.5f * qdot.w * dt_s;
  q.x += 0.5f * qdot.x * dt_s;
  q.y += 0.5f * qdot.y * dt_s;
  q.z += 0.5f * qdot.z * dt_s;
  return quat_normalize(q);
}

static actuator_cmd_t actuator_step(actuator_cmd_t state, const actuator_cmd_t *cmd, real_t lag) {
  actuator_cmd_t out;
  out.motor = lerp_real(state.motor, clamp_real(cmd->motor, 0.0f, 1.0f), lag);
  out.aileron = lerp_real(state.aileron, clamp_real(cmd->aileron, -1.0f, 1.0f), lag);
  out.elevator = lerp_real(state.elevator, clamp_real(cmd->elevator, -1.0f, 1.0f), lag);
  out.rudder = lerp_real(state.rudder, clamp_real(cmd->rudder, -1.0f, 1.0f), lag);
  return out;
}

void sim6dof_default_params(sim6dof_params_t *params) {
  if (!params) {
    return;
  }
  params->mass_kg = 2.5f;
  params->inertia_kgm2.x = 0.08f;
  params->inertia_kgm2.y = 0.12f;
  params->inertia_kgm2.z = 0.18f;
  params->gravity_mps2 = 9.80665f;
  params->air_density_kgpm3 = 1.225f;
  params->actuator_lag_hz = 12.0f;
}

void sim6dof_init_level(sim6dof_state_t *state, real_t altitude_m, real_t forward_speed_mps) {
  if (!state) {
    return;
  }
  state->position_ned_m.x = 0.0f;
  state->position_ned_m.y = 0.0f;
  state->position_ned_m.z = -altitude_m;
  state->velocity_ned_mps.x = forward_speed_mps;
  state->velocity_ned_mps.y = 0.0f;
  state->velocity_ned_mps.z = 0.0f;
  state->attitude_body_to_ned = quat_identity();
  state->omega_body_rps.x = 0.0f;
  state->omega_body_rps.y = 0.0f;
  state->omega_body_rps.z = 0.0f;
  state->actuator_state.motor = 0.0f;
  state->actuator_state.aileron = 0.0f;
  state->actuator_state.elevator = 0.0f;
  state->actuator_state.rudder = 0.0f;
  state->specific_force_body_mps2.x = 0.0f;
  state->specific_force_body_mps2.y = 0.0f;
  state->specific_force_body_mps2.z = 0.0f;
  state->time_s = 0.0f;
}

int sim6dof_state_is_valid(const sim6dof_state_t *state) {
  real_t q_norm_sq;
  if (!state) {
    return 0;
  }
  q_norm_sq = state->attitude_body_to_ned.w * state->attitude_body_to_ned.w +
              state->attitude_body_to_ned.x * state->attitude_body_to_ned.x +
              state->attitude_body_to_ned.y * state->attitude_body_to_ned.y +
              state->attitude_body_to_ned.z * state->attitude_body_to_ned.z;
  return vec3_is_finite(state->position_ned_m) &&
         vec3_is_finite(state->velocity_ned_mps) &&
         vec3_is_finite(state->omega_body_rps) &&
         vec3_is_finite(state->specific_force_body_mps2) &&
         real_is_finite(state->attitude_body_to_ned.w) &&
         real_is_finite(state->attitude_body_to_ned.x) &&
         real_is_finite(state->attitude_body_to_ned.y) &&
         real_is_finite(state->attitude_body_to_ned.z) &&
         real_is_finite(state->actuator_state.motor) &&
         real_is_finite(state->actuator_state.aileron) &&
         real_is_finite(state->actuator_state.elevator) &&
         real_is_finite(state->actuator_state.rudder) &&
         real_is_finite(state->time_s) &&
         q_norm_sq > 0.5f && q_norm_sq < 1.5f &&
         vec3_norm(state->velocity_ned_mps) < 300.0f &&
         vec3_norm(state->omega_body_rps) < 50.0f &&
         state->position_ned_m.z > -50000.0f && state->position_ned_m.z < 10000.0f;
}

int sim6dof_step(sim6dof_state_t *state,
                 const sim6dof_params_t *params,
                 const actuator_cmd_t *actuator_cmd,
                 vec3_t force_body_n,
                 vec3_t moment_body_nm,
                 real_t dt_s) {
  vec3_t accel_body_mps2;
  vec3_t accel_ned_mps2;
  vec3_t inertia_omega;
  vec3_t gyro_moment;
  vec3_t omega_dot;
  real_t lag;

  if (!state || !params || !actuator_cmd || dt_s <= 0.0f || dt_s > 0.1f) {
    return 0;
  }
  if (params->mass_kg <= 0.0f ||
      params->inertia_kgm2.x <= 0.0f ||
      params->inertia_kgm2.y <= 0.0f ||
      params->inertia_kgm2.z <= 0.0f ||
      !vec3_is_finite(force_body_n) ||
      !vec3_is_finite(moment_body_nm) ||
      !sim6dof_state_is_valid(state)) {
    return 0;
  }

  lag = clamp_real(dt_s * params->actuator_lag_hz, 0.0f, 1.0f);
  state->actuator_state = actuator_step(state->actuator_state, actuator_cmd, lag);

  accel_body_mps2 = vec3_scale(force_body_n, 1.0f / params->mass_kg);
  accel_ned_mps2 = quat_rotate_vec3(state->attitude_body_to_ned, accel_body_mps2);
  accel_ned_mps2.z += params->gravity_mps2;
  state->specific_force_body_mps2 = accel_body_mps2;

  state->velocity_ned_mps = vec3_add(state->velocity_ned_mps, vec3_scale(accel_ned_mps2, dt_s));
  state->position_ned_m = vec3_add(state->position_ned_m, vec3_scale(state->velocity_ned_mps, dt_s));

  inertia_omega.x = params->inertia_kgm2.x * state->omega_body_rps.x;
  inertia_omega.y = params->inertia_kgm2.y * state->omega_body_rps.y;
  inertia_omega.z = params->inertia_kgm2.z * state->omega_body_rps.z;
  gyro_moment = vec3_cross(state->omega_body_rps, inertia_omega);
  omega_dot.x = (moment_body_nm.x - gyro_moment.x) / params->inertia_kgm2.x;
  omega_dot.y = (moment_body_nm.y - gyro_moment.y) / params->inertia_kgm2.y;
  omega_dot.z = (moment_body_nm.z - gyro_moment.z) / params->inertia_kgm2.z;
  state->omega_body_rps = vec3_add(state->omega_body_rps, vec3_scale(omega_dot, dt_s));
  state->attitude_body_to_ned = quat_integrate_body_rate(state->attitude_body_to_ned, state->omega_body_rps, dt_s);
  state->time_s += dt_s;

  (void)params->air_density_kgpm3;
  return sim6dof_state_is_valid(state);
}
