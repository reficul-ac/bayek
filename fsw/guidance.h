#ifndef BAYEK_FSW_GUIDANCE_H
#define BAYEK_FSW_GUIDANCE_H

#include "common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  real_t throttle;
  real_t roll_rad;
  real_t pitch_rad;
  real_t yaw_rate_rps;
} bayek_guidance_setpoint_t;

bayek_guidance_setpoint_t
bayek_guidance_stabilize_from_rc(const rc_input_t *rc,
                                 const vehicle_params_t *params);
int bayek_guidance_mission_to_waypoint(const fsw_input_t *in,
                                       const state_estimate_t *estimate,
                                       const bayek_mission_waypoint_t *waypoint,
                                       const vehicle_params_t *params,
                                       bayek_guidance_setpoint_t *setpoint,
                                       real_t *horizontal_distance_m);

#ifdef __cplusplus
}
#endif

#endif
