#ifndef BAYEK_SIM6DOF_H
#define BAYEK_SIM6DOF_H

#include "common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  real_t mass_kg;
  vec3_t inertia_kgm2;
  real_t gravity_mps2;
  real_t air_density_kgpm3;
  real_t actuator_lag_hz;
} sim6dof_params_t;

typedef struct {
  vec3_t position_ned_m;
  vec3_t velocity_ned_mps;
  quat_t attitude_body_to_ned;
  vec3_t omega_body_rps;
  actuator_cmd_t actuator_state;
  vec3_t specific_force_body_mps2;
  real_t time_s;
} sim6dof_state_t;

void sim6dof_default_params(sim6dof_params_t *params);
void sim6dof_init_level(sim6dof_state_t *state, real_t altitude_m, real_t forward_speed_mps);
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
