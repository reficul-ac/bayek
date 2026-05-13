#include "sim6dof.h"

#include "math_utils.h"

#include <math.h>

typedef struct {
  real_t m[3][3];
} mat3_local_t;

static int vec3_is_finite(vec3_t v) {
  return real_is_finite(v.x) && real_is_finite(v.y) && real_is_finite(v.z);
}

static mat3_local_t quat_to_mat3(quat_t q) {
  mat3_local_t m;
  vec3_t x_axis = {1.0f, 0.0f, 0.0f};
  vec3_t y_axis = {0.0f, 1.0f, 0.0f};
  vec3_t z_axis = {0.0f, 0.0f, 1.0f};
  vec3_t x = quat_rotate_vec3(q, x_axis);
  vec3_t y = quat_rotate_vec3(q, y_axis);
  vec3_t z = quat_rotate_vec3(q, z_axis);
  m.m[0][0] = x.x;
  m.m[1][0] = x.y;
  m.m[2][0] = x.z;
  m.m[0][1] = y.x;
  m.m[1][1] = y.y;
  m.m[2][1] = y.z;
  m.m[0][2] = z.x;
  m.m[1][2] = z.y;
  m.m[2][2] = z.z;
  return m;
}

static quat_t mat3_to_quat(mat3_local_t m) {
  real_t trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
  quat_t q;
  if (trace > 0.0f) {
    real_t s = (real_t)sqrtf(trace + 1.0f) * 2.0f;
    q.w = 0.25f * s;
    q.x = (m.m[2][1] - m.m[1][2]) / s;
    q.y = (m.m[0][2] - m.m[2][0]) / s;
    q.z = (m.m[1][0] - m.m[0][1]) / s;
  } else if (m.m[0][0] > m.m[1][1] && m.m[0][0] > m.m[2][2]) {
    real_t s = (real_t)sqrtf(1.0f + m.m[0][0] - m.m[1][1] - m.m[2][2]) * 2.0f;
    q.w = (m.m[2][1] - m.m[1][2]) / s;
    q.x = 0.25f * s;
    q.y = (m.m[0][1] + m.m[1][0]) / s;
    q.z = (m.m[0][2] + m.m[2][0]) / s;
  } else if (m.m[1][1] > m.m[2][2]) {
    real_t s = (real_t)sqrtf(1.0f + m.m[1][1] - m.m[0][0] - m.m[2][2]) * 2.0f;
    q.w = (m.m[0][2] - m.m[2][0]) / s;
    q.x = (m.m[0][1] + m.m[1][0]) / s;
    q.y = 0.25f * s;
    q.z = (m.m[1][2] + m.m[2][1]) / s;
  } else {
    real_t s = (real_t)sqrtf(1.0f + m.m[2][2] - m.m[0][0] - m.m[1][1]) * 2.0f;
    q.w = (m.m[1][0] - m.m[0][1]) / s;
    q.x = (m.m[0][2] + m.m[2][0]) / s;
    q.y = (m.m[1][2] + m.m[2][1]) / s;
    q.z = 0.25f * s;
  }
  return quat_normalize(q);
}

static mat3_local_t mat3_multiply(mat3_local_t a, mat3_local_t b) {
  mat3_local_t out;
  int r;
  int c;
  int k;
  for (r = 0; r < 3; ++r) {
    for (c = 0; c < 3; ++c) {
      out.m[r][c] = 0.0f;
      for (k = 0; k < 3; ++k) {
        out.m[r][c] += a.m[r][k] * b.m[k][c];
      }
    }
  }
  return out;
}

static mat3_local_t mat3_transpose(mat3_local_t a) {
  mat3_local_t out;
  int r;
  int c;
  for (r = 0; r < 3; ++r) {
    for (c = 0; c < 3; ++c) {
      out.m[r][c] = a.m[c][r];
    }
  }
  return out;
}

static mat3_local_t ned_to_ecef_mat(real_t lat_deg, real_t lon_deg) {
  mat3_local_t m;
  vec3_t north;
  vec3_t east;
  vec3_t down;
  sim6dof_ned_basis(lat_deg, lon_deg, &north, &east, &down);
  m.m[0][0] = north.x;
  m.m[1][0] = north.y;
  m.m[2][0] = north.z;
  m.m[0][1] = east.x;
  m.m[1][1] = east.y;
  m.m[2][1] = east.z;
  m.m[0][2] = down.x;
  m.m[1][2] = down.y;
  m.m[2][2] = down.z;
  return m;
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
  params->frame_mode = SIM6DOF_FRAME_ECEF;
  params->earth_model = SIM6DOF_EARTH_SPHERICAL;
  params->earth_radius_m = 6378137.0f;
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
  sim6dof_set_origin(state, 0.0f, 0.0f, 0.0f, 6378137.0f);
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
  sim6dof_sync_ecef_from_ned(state, 6378137.0f);
}

void sim6dof_set_origin(sim6dof_state_t *state,
                        real_t lat_deg,
                        real_t lon_deg,
                        real_t altitude_m,
                        real_t earth_radius_m) {
  if (!state) {
    return;
  }
  state->origin_lat_deg = lat_deg;
  state->origin_lon_deg = lon_deg;
  state->origin_altitude_m = altitude_m;
  sim6dof_sync_ecef_from_ned(state, earth_radius_m);
}

void sim6dof_geodetic_to_ecef(real_t lat_deg,
                              real_t lon_deg,
                              real_t altitude_m,
                              real_t earth_radius_m,
                              vec3_t *position_ecef_m) {
  real_t lat_rad;
  real_t lon_rad;
  real_t radius_m;
  if (!position_ecef_m) {
    return;
  }
  lat_rad = lat_deg * (BAYEK_PI / 180.0f);
  lon_rad = lon_deg * (BAYEK_PI / 180.0f);
  radius_m = earth_radius_m + altitude_m;
  position_ecef_m->x = radius_m * (real_t)cosf(lat_rad) * (real_t)cosf(lon_rad);
  position_ecef_m->y = radius_m * (real_t)cosf(lat_rad) * (real_t)sinf(lon_rad);
  position_ecef_m->z = radius_m * (real_t)sinf(lat_rad);
}

void sim6dof_ecef_to_geodetic(vec3_t position_ecef_m,
                              real_t earth_radius_m,
                              real_t *lat_deg,
                              real_t *lon_deg,
                              real_t *altitude_m) {
  real_t xy_norm = (real_t)sqrtf(position_ecef_m.x * position_ecef_m.x +
                                 position_ecef_m.y * position_ecef_m.y);
  real_t radius_m = vec3_norm(position_ecef_m);
  if (lat_deg) {
    *lat_deg = (real_t)atan2f(position_ecef_m.z, xy_norm) * (180.0f / BAYEK_PI);
  }
  if (lon_deg) {
    *lon_deg = (real_t)atan2f(position_ecef_m.y, position_ecef_m.x) * (180.0f / BAYEK_PI);
  }
  if (altitude_m) {
    *altitude_m = radius_m - earth_radius_m;
  }
}

void sim6dof_ned_basis(real_t lat_deg, real_t lon_deg, vec3_t *north, vec3_t *east, vec3_t *down) {
  real_t lat_rad = lat_deg * (BAYEK_PI / 180.0f);
  real_t lon_rad = lon_deg * (BAYEK_PI / 180.0f);
  real_t sin_lat = (real_t)sinf(lat_rad);
  real_t cos_lat = (real_t)cosf(lat_rad);
  real_t sin_lon = (real_t)sinf(lon_rad);
  real_t cos_lon = (real_t)cosf(lon_rad);
  if (north) {
    north->x = -sin_lat * cos_lon;
    north->y = -sin_lat * sin_lon;
    north->z = cos_lat;
  }
  if (east) {
    east->x = -sin_lon;
    east->y = cos_lon;
    east->z = 0.0f;
  }
  if (down) {
    down->x = -cos_lat * cos_lon;
    down->y = -cos_lat * sin_lon;
    down->z = -sin_lat;
  }
}

vec3_t sim6dof_ned_vector_to_ecef(real_t origin_lat_deg, real_t origin_lon_deg, vec3_t ned) {
  vec3_t north;
  vec3_t east;
  vec3_t down;
  vec3_t ecef;
  sim6dof_ned_basis(origin_lat_deg, origin_lon_deg, &north, &east, &down);
  ecef.x = north.x * ned.x + east.x * ned.y + down.x * ned.z;
  ecef.y = north.y * ned.x + east.y * ned.y + down.y * ned.z;
  ecef.z = north.z * ned.x + east.z * ned.y + down.z * ned.z;
  return ecef;
}

vec3_t sim6dof_ecef_vector_to_ned(real_t origin_lat_deg, real_t origin_lon_deg, vec3_t ecef) {
  vec3_t north;
  vec3_t east;
  vec3_t down;
  vec3_t ned;
  sim6dof_ned_basis(origin_lat_deg, origin_lon_deg, &north, &east, &down);
  ned.x = vec3_dot(ecef, north);
  ned.y = vec3_dot(ecef, east);
  ned.z = vec3_dot(ecef, down);
  return ned;
}

vec3_t sim6dof_ned_position_to_ecef(real_t origin_lat_deg,
                                    real_t origin_lon_deg,
                                    real_t origin_altitude_m,
                                    real_t earth_radius_m,
                                    vec3_t position_ned_m) {
  vec3_t origin_ecef_m;
  vec3_t offset_ecef_m;
  sim6dof_geodetic_to_ecef(origin_lat_deg, origin_lon_deg, origin_altitude_m, earth_radius_m, &origin_ecef_m);
  offset_ecef_m = sim6dof_ned_vector_to_ecef(origin_lat_deg, origin_lon_deg, position_ned_m);
  return vec3_add(origin_ecef_m, offset_ecef_m);
}

vec3_t sim6dof_ecef_position_to_ned(real_t origin_lat_deg,
                                    real_t origin_lon_deg,
                                    real_t origin_altitude_m,
                                    real_t earth_radius_m,
                                    vec3_t position_ecef_m) {
  vec3_t origin_ecef_m;
  sim6dof_geodetic_to_ecef(origin_lat_deg, origin_lon_deg, origin_altitude_m, earth_radius_m, &origin_ecef_m);
  return sim6dof_ecef_vector_to_ned(origin_lat_deg, origin_lon_deg, vec3_sub(position_ecef_m, origin_ecef_m));
}

quat_t sim6dof_body_to_ecef_quat(real_t origin_lat_deg, real_t origin_lon_deg, quat_t attitude_body_to_ned) {
  mat3_local_t r_ecef_ned = ned_to_ecef_mat(origin_lat_deg, origin_lon_deg);
  mat3_local_t r_ned_body = quat_to_mat3(attitude_body_to_ned);
  return mat3_to_quat(mat3_multiply(r_ecef_ned, r_ned_body));
}

quat_t sim6dof_body_to_ned_quat(real_t origin_lat_deg, real_t origin_lon_deg, quat_t attitude_body_to_ecef) {
  mat3_local_t r_ecef_ned = ned_to_ecef_mat(origin_lat_deg, origin_lon_deg);
  mat3_local_t r_ned_ecef = mat3_transpose(r_ecef_ned);
  mat3_local_t r_ecef_body = quat_to_mat3(attitude_body_to_ecef);
  return mat3_to_quat(mat3_multiply(r_ned_ecef, r_ecef_body));
}

void sim6dof_sync_ecef_from_ned(sim6dof_state_t *state, real_t earth_radius_m) {
  if (!state) {
    return;
  }
  state->position_ecef_m = sim6dof_ned_position_to_ecef(state->origin_lat_deg,
                                                        state->origin_lon_deg,
                                                        state->origin_altitude_m,
                                                        earth_radius_m,
                                                        state->position_ned_m);
  state->velocity_ecef_mps = sim6dof_ned_vector_to_ecef(state->origin_lat_deg,
                                                        state->origin_lon_deg,
                                                        state->velocity_ned_mps);
  state->attitude_body_to_ecef = sim6dof_body_to_ecef_quat(state->origin_lat_deg,
                                                          state->origin_lon_deg,
                                                          state->attitude_body_to_ned);
}

void sim6dof_sync_ned_from_ecef(sim6dof_state_t *state, real_t earth_radius_m) {
  if (!state) {
    return;
  }
  state->position_ned_m = sim6dof_ecef_position_to_ned(state->origin_lat_deg,
                                                       state->origin_lon_deg,
                                                       state->origin_altitude_m,
                                                       earth_radius_m,
                                                       state->position_ecef_m);
  state->velocity_ned_mps = sim6dof_ecef_vector_to_ned(state->origin_lat_deg,
                                                       state->origin_lon_deg,
                                                       state->velocity_ecef_mps);
  state->attitude_body_to_ned = sim6dof_body_to_ned_quat(state->origin_lat_deg,
                                                        state->origin_lon_deg,
                                                        state->attitude_body_to_ecef);
}

int sim6dof_state_is_valid(const sim6dof_state_t *state) {
  real_t q_norm_sq;
  real_t q_ecef_norm_sq;
  if (!state) {
    return 0;
  }
  q_norm_sq = state->attitude_body_to_ned.w * state->attitude_body_to_ned.w +
              state->attitude_body_to_ned.x * state->attitude_body_to_ned.x +
              state->attitude_body_to_ned.y * state->attitude_body_to_ned.y +
              state->attitude_body_to_ned.z * state->attitude_body_to_ned.z;
  q_ecef_norm_sq = state->attitude_body_to_ecef.w * state->attitude_body_to_ecef.w +
                   state->attitude_body_to_ecef.x * state->attitude_body_to_ecef.x +
                   state->attitude_body_to_ecef.y * state->attitude_body_to_ecef.y +
                   state->attitude_body_to_ecef.z * state->attitude_body_to_ecef.z;
  return real_is_finite(state->origin_lat_deg) &&
         real_is_finite(state->origin_lon_deg) &&
         real_is_finite(state->origin_altitude_m) &&
         vec3_is_finite(state->position_ned_m) &&
         vec3_is_finite(state->velocity_ned_mps) &&
         vec3_is_finite(state->position_ecef_m) &&
         vec3_is_finite(state->velocity_ecef_mps) &&
         vec3_is_finite(state->omega_body_rps) &&
         vec3_is_finite(state->specific_force_body_mps2) &&
         real_is_finite(state->attitude_body_to_ned.w) &&
         real_is_finite(state->attitude_body_to_ned.x) &&
         real_is_finite(state->attitude_body_to_ned.y) &&
         real_is_finite(state->attitude_body_to_ned.z) &&
         real_is_finite(state->attitude_body_to_ecef.w) &&
         real_is_finite(state->attitude_body_to_ecef.x) &&
         real_is_finite(state->attitude_body_to_ecef.y) &&
         real_is_finite(state->attitude_body_to_ecef.z) &&
         real_is_finite(state->actuator_state.motor) &&
         real_is_finite(state->actuator_state.aileron) &&
         real_is_finite(state->actuator_state.elevator) &&
         real_is_finite(state->actuator_state.rudder) &&
         real_is_finite(state->time_s) &&
         q_norm_sq > 0.5f && q_norm_sq < 1.5f &&
         q_ecef_norm_sq > 0.5f && q_ecef_norm_sq < 1.5f &&
         vec3_norm(state->velocity_ned_mps) < 300.0f &&
         vec3_norm(state->velocity_ecef_mps) < 300.0f &&
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
  vec3_t accel_ecef_mps2;
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
      params->earth_model != SIM6DOF_EARTH_SPHERICAL ||
      params->earth_radius_m <= 1000.0f ||
      (params->frame_mode != SIM6DOF_FRAME_NED && params->frame_mode != SIM6DOF_FRAME_ECEF) ||
      !vec3_is_finite(force_body_n) ||
      !vec3_is_finite(moment_body_nm) ||
      !sim6dof_state_is_valid(state)) {
    return 0;
  }

  lag = clamp_real(dt_s * params->actuator_lag_hz, 0.0f, 1.0f);
  state->actuator_state = actuator_step(state->actuator_state, actuator_cmd, lag);

  accel_body_mps2 = vec3_scale(force_body_n, 1.0f / params->mass_kg);
  state->specific_force_body_mps2 = accel_body_mps2;

  if (params->frame_mode == SIM6DOF_FRAME_ECEF) {
    vec3_t gravity_dir_ecef = vec3_normalize(vec3_scale(state->position_ecef_m, -1.0f));
    accel_ecef_mps2 = quat_rotate_vec3(state->attitude_body_to_ecef, accel_body_mps2);
    accel_ecef_mps2 = vec3_add(accel_ecef_mps2, vec3_scale(gravity_dir_ecef, params->gravity_mps2));
    state->velocity_ecef_mps = vec3_add(state->velocity_ecef_mps, vec3_scale(accel_ecef_mps2, dt_s));
    state->position_ecef_m = vec3_add(state->position_ecef_m, vec3_scale(state->velocity_ecef_mps, dt_s));
  } else {
    accel_ned_mps2 = quat_rotate_vec3(state->attitude_body_to_ned, accel_body_mps2);
    accel_ned_mps2.z += params->gravity_mps2;
    state->velocity_ned_mps = vec3_add(state->velocity_ned_mps, vec3_scale(accel_ned_mps2, dt_s));
    state->position_ned_m = vec3_add(state->position_ned_m, vec3_scale(state->velocity_ned_mps, dt_s));
  }

  inertia_omega.x = params->inertia_kgm2.x * state->omega_body_rps.x;
  inertia_omega.y = params->inertia_kgm2.y * state->omega_body_rps.y;
  inertia_omega.z = params->inertia_kgm2.z * state->omega_body_rps.z;
  gyro_moment = vec3_cross(state->omega_body_rps, inertia_omega);
  omega_dot.x = (moment_body_nm.x - gyro_moment.x) / params->inertia_kgm2.x;
  omega_dot.y = (moment_body_nm.y - gyro_moment.y) / params->inertia_kgm2.y;
  omega_dot.z = (moment_body_nm.z - gyro_moment.z) / params->inertia_kgm2.z;
  state->omega_body_rps = vec3_add(state->omega_body_rps, vec3_scale(omega_dot, dt_s));
  if (params->frame_mode == SIM6DOF_FRAME_ECEF) {
    state->attitude_body_to_ecef = quat_integrate_body_rate(state->attitude_body_to_ecef, state->omega_body_rps, dt_s);
    sim6dof_sync_ned_from_ecef(state, params->earth_radius_m);
  } else {
    state->attitude_body_to_ned = quat_integrate_body_rate(state->attitude_body_to_ned, state->omega_body_rps, dt_s);
    sim6dof_sync_ecef_from_ned(state, params->earth_radius_m);
  }
  state->time_s += dt_s;

  (void)params->air_density_kgpm3;
  return sim6dof_state_is_valid(state);
}
