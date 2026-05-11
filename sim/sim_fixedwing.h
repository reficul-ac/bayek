#ifndef BAYEK_SIM_FIXEDWING_H
#define BAYEK_SIM_FIXEDWING_H

#include "sim6dof.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  sim6dof_params_t core;
  real_t wing_area_m2;
  real_t wing_span_m;
  real_t mean_chord_m;
  real_t max_thrust_n;
  real_t drag_cd0;
  real_t drag_cd_alpha;
  real_t lift_cl0;
  real_t lift_cl_alpha;
  real_t lift_cl_elevator;
  real_t stall_alpha_rad;
  real_t roll_aileron_nm;
  real_t pitch_elevator_nm;
  real_t yaw_rudder_nm;
  vec3_t rate_damping_nms;
} sim_fixedwing_params_t;

typedef struct {
  sim6dof_state_t body;
  vec3_t last_force_body_n;
  vec3_t last_moment_body_nm;
  real_t last_airspeed_mps;
} sim_fixedwing_state_t;

void sim_fixedwing_default_params(sim_fixedwing_params_t *params);
void sim_fixedwing_init_default(sim_fixedwing_state_t *state);
int sim_fixedwing_step(sim_fixedwing_state_t *state,
                       const sim_fixedwing_params_t *params,
                       const actuator_cmd_t *cmd,
                       real_t dt_s);
void sim_fixedwing_make_fsw_input(const sim_fixedwing_state_t *state,
                                  const rc_input_t *rc,
                                  real_t dt_s,
                                  uint32_t timestamp_us,
                                  fsw_input_t *in);
int sim_fixedwing_state_is_valid(const sim_fixedwing_state_t *state);

#ifdef __cplusplus
}
#endif

#endif
