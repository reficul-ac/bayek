#include "guidance.h"

#include "math_utils.h"

#include <math.h>

bayek_guidance_setpoint_t
bayek_guidance_stabilize_from_rc(const rc_input_t *rc,
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
  setpoint.pitch_rad =
      clamp_real(rc->pitch, -1.0f, 1.0f) * params->max_pitch_rad;
  setpoint.yaw_rate_rps =
      clamp_real(rc->yaw, -1.0f, 1.0f) * params->max_yaw_rate_rps;
  return setpoint;
}

int bayek_guidance_mission_to_waypoint(const fsw_input_t *in,
                                       const state_estimate_t *estimate,
                                       const bayek_mission_waypoint_t *waypoint,
                                       const vehicle_params_t *params,
                                       bayek_guidance_setpoint_t *setpoint,
                                       real_t *horizontal_distance_m) {
  const real_t earth_radius_m = 6371000.0f;
  const real_t deg_to_rad = BAYEK_PI / 180.0f;
  real_t current_lat_rad;
  real_t target_lat_rad;
  real_t north_m;
  real_t east_m;
  real_t desired_course_rad;
  real_t heading_error_rad;
  real_t alt_error_m;

  if (!in || !estimate || !waypoint || !params || !setpoint ||
      !horizontal_distance_m) {
    return 0;
  }

  current_lat_rad = in->gps.lat_deg * deg_to_rad;
  target_lat_rad = waypoint->lat_deg * deg_to_rad;
  north_m = (target_lat_rad - current_lat_rad) * earth_radius_m;
  east_m = (waypoint->lon_deg - in->gps.lon_deg) * deg_to_rad * earth_radius_m *
           (real_t)cosf(0.5f * (current_lat_rad + target_lat_rad));
  *horizontal_distance_m = (real_t)sqrtf(north_m * north_m + east_m * east_m);

  desired_course_rad = (real_t)atan2f(east_m, north_m);
  heading_error_rad = wrap_pi(desired_course_rad - estimate->euler.yaw);
  alt_error_m = waypoint->alt_m - in->gps.alt_m;

  setpoint->throttle = clamp_real(waypoint->throttle, 0.0f, 1.0f);
  setpoint->roll_rad = clamp_real(heading_error_rad * 1.0f,
                                  -params->max_roll_rad, params->max_roll_rad);
  setpoint->pitch_rad = clamp_real(alt_error_m * 0.01f, -params->max_pitch_rad,
                                   params->max_pitch_rad);
  setpoint->yaw_rate_rps = 0.0f;
  return 1;
}
