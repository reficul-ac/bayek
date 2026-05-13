#include "control.h"

void bayek_control_init(bayek_control_state_t *control) {
  if (!control) {
    return;
  }
  pid_init(&control->roll_pid, 2.0f, 0.1f, 0.02f, -1.0f, 1.0f);
  pid_init(&control->pitch_pid, 2.0f, 0.1f, 0.02f, -1.0f, 1.0f);
  pid_init(&control->yaw_rate_pid, 1.0f, 0.05f, 0.0f, -1.0f, 1.0f);
}

void bayek_control_reset(bayek_control_state_t *control) {
  if (!control) {
    return;
  }
  pid_reset(&control->roll_pid);
  pid_reset(&control->pitch_pid);
  pid_reset(&control->yaw_rate_pid);
}

bayek_control_request_t bayek_control_stabilize_step(bayek_control_state_t *control,
                                                     const bayek_guidance_setpoint_t *setpoint,
                                                     const state_estimate_t *estimate,
                                                     const fsw_input_t *in) {
  bayek_control_request_t request;

  request.throttle = setpoint ? setpoint->throttle : 0.0f;
  request.roll = 0.0f;
  request.pitch = 0.0f;
  request.yaw = 0.0f;

  if (!control || !setpoint || !estimate || !in) {
    return request;
  }

  request.roll = pid_step(&control->roll_pid, setpoint->roll_rad, estimate->euler.roll, in->dt_s);
  request.pitch = pid_step(&control->pitch_pid, setpoint->pitch_rad, estimate->euler.pitch, in->dt_s);
  request.yaw = pid_step(&control->yaw_rate_pid, setpoint->yaw_rate_rps, in->imu.gyro_rps.z, in->dt_s);
  return request;
}
