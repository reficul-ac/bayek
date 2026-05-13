#ifndef BAYEK_SIM6DOF_H
#define BAYEK_SIM6DOF_H

#include "common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  SIM6DOF_FRAME_NED = 0,
  SIM6DOF_FRAME_ECEF = 1
} sim6dof_frame_mode_t;

typedef enum {
  SIM6DOF_EARTH_SPHERICAL = 0
} sim6dof_earth_model_t;

typedef struct {
  real_t mass_kg;
  vec3_t inertia_kgm2;
  real_t gravity_mps2;
  real_t air_density_kgpm3;
  real_t actuator_lag_hz;
  int frame_mode;
  int earth_model;
  real_t earth_radius_m;
} sim6dof_params_t;

typedef struct {
  real_t origin_lat_deg;
  real_t origin_lon_deg;
  real_t origin_altitude_m;
  vec3_t position_ned_m;
  vec3_t velocity_ned_mps;
  quat_t attitude_body_to_ned;
  vec3_t position_ecef_m;
  vec3_t velocity_ecef_mps;
  quat_t attitude_body_to_ecef;
  vec3_t omega_body_rps;
  actuator_cmd_t actuator_state;
  vec3_t specific_force_body_mps2;
  real_t time_s;
} sim6dof_state_t;

void sim6dof_default_params(sim6dof_params_t *params);
void sim6dof_init_level(sim6dof_state_t *state, real_t altitude_m, real_t forward_speed_mps);
void sim6dof_set_origin(sim6dof_state_t *state,
                        real_t lat_deg,
                        real_t lon_deg,
                        real_t altitude_m,
                        real_t earth_radius_m);
void sim6dof_geodetic_to_ecef(real_t lat_deg,
                              real_t lon_deg,
                              real_t altitude_m,
                              real_t earth_radius_m,
                              vec3_t *position_ecef_m);
void sim6dof_ecef_to_geodetic(vec3_t position_ecef_m,
                              real_t earth_radius_m,
                              real_t *lat_deg,
                              real_t *lon_deg,
                              real_t *altitude_m);
void sim6dof_ned_basis(real_t lat_deg, real_t lon_deg, vec3_t *north, vec3_t *east, vec3_t *down);
vec3_t sim6dof_ned_vector_to_ecef(real_t origin_lat_deg, real_t origin_lon_deg, vec3_t ned);
vec3_t sim6dof_ecef_vector_to_ned(real_t origin_lat_deg, real_t origin_lon_deg, vec3_t ecef);
vec3_t sim6dof_ned_position_to_ecef(real_t origin_lat_deg,
                                    real_t origin_lon_deg,
                                    real_t origin_altitude_m,
                                    real_t earth_radius_m,
                                    vec3_t position_ned_m);
vec3_t sim6dof_ecef_position_to_ned(real_t origin_lat_deg,
                                    real_t origin_lon_deg,
                                    real_t origin_altitude_m,
                                    real_t earth_radius_m,
                                    vec3_t position_ecef_m);
quat_t sim6dof_body_to_ecef_quat(real_t origin_lat_deg, real_t origin_lon_deg, quat_t attitude_body_to_ned);
quat_t sim6dof_body_to_ned_quat(real_t origin_lat_deg, real_t origin_lon_deg, quat_t attitude_body_to_ecef);
void sim6dof_sync_ecef_from_ned(sim6dof_state_t *state, real_t earth_radius_m);
void sim6dof_sync_ned_from_ecef(sim6dof_state_t *state, real_t earth_radius_m);
int sim6dof_state_is_valid(const sim6dof_state_t *state);
int sim6dof_step(sim6dof_state_t *state,
                 const sim6dof_params_t *params,
                 const actuator_cmd_t *actuator_cmd,
                 vec3_t force_body_n,
                 vec3_t moment_body_nm,
                 real_t dt_s);

#ifdef __cplusplus
}
#endif

#endif
