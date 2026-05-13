#include "guidance.h"

#include "math_utils.h"

bayek_guidance_setpoint_t bayek_guidance_stabilize_from_rc(const rc_input_t *rc,
                                                           const vehicle_params_t *params) {
  bayek_guidance_setpoint_t setpoint;

  setpoint.throttle = rc ? rc->throttle : 0.0f;
  setpoint.roll_rad = 0.0f;
  setpoint.pitch_rad = 0.0f;
  setpoint.yaw_rate_rps = 0.0f;

  if (!rc || !params) {
    return setpoint;
  }

  setpoint.roll_rad = clamp_real(rc->roll, -1.0f, 1.0f) * params->max_roll_rad;
  setpoint.pitch_rad = clamp_real(rc->pitch, -1.0f, 1.0f) * params->max_pitch_rad;
  setpoint.yaw_rate_rps = clamp_real(rc->yaw, -1.0f, 1.0f) * params->max_yaw_rate_rps;
  return setpoint;
}
