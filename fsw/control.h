#ifndef BAYEK_FSW_CONTROL_H
#define BAYEK_FSW_CONTROL_H

#include "common_types.h"
#include "control_utils.h"
#include "guidance.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  real_t throttle;
  real_t roll;
  real_t pitch;
  real_t yaw;
} bayek_control_request_t;

typedef struct {
  bayek_pid_t roll_pid;
  bayek_pid_t pitch_pid;
  bayek_pid_t yaw_rate_pid;
} bayek_control_state_t;

void bayek_control_init(bayek_control_state_t *control);
void bayek_control_reset(bayek_control_state_t *control);
bayek_control_request_t bayek_control_stabilize_step(bayek_control_state_t *control,
                                                     const bayek_guidance_setpoint_t *setpoint,
                                                     const state_estimate_t *estimate,
                                                     const fsw_input_t *in);

#ifdef __cplusplus
}
#endif

#endif
