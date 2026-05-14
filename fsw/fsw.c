#include "fsw.h"

#include "control.h"
#include "fault.h"
#include "guidance.h"
#include "nav.h"

#include "math_utils.h"

#include <string.h>

typedef struct {
  const bayek_vehicle_interface_t *vehicle;
  const vehicle_params_t *params;
  bayek_control_state_t control;
  state_estimate_t estimate;
  bayek_mission_plan_t mission;
  bayek_mission_status_t mission_status;
} fsw_state_t;

static fsw_state_t g_fsw;

static int mission_waypoint_is_valid(const bayek_mission_waypoint_t *wp) {
  if (!wp) {
    return 0;
  }
  return real_is_finite(wp->lat_deg) && real_is_finite(wp->lon_deg) &&
         real_is_finite(wp->alt_m) && real_is_finite(wp->throttle) &&
         real_is_finite(wp->acceptance_radius_m) && wp->lat_deg >= -90.0f &&
         wp->lat_deg <= 90.0f && wp->lon_deg >= -180.0f &&
         wp->lon_deg <= 180.0f && wp->throttle >= 0.0f &&
         wp->throttle <= 1.0f && wp->acceptance_radius_m > 0.0f;
}

static int mission_plan_is_valid(const bayek_mission_plan_t *mission) {
  uint32_t i;

  if (!mission || mission->waypoint_count == 0U ||
      mission->waypoint_count > BAYEK_MISSION_MAX_WAYPOINTS) {
    return 0;
  }
  for (i = 0U; i < mission->waypoint_count; ++i) {
    if (!mission_waypoint_is_valid(&mission->waypoints[i])) {
      return 0;
    }
  }
  return 1;
}

static void mission_clear(void) {
  memset(&g_fsw.mission, 0, sizeof(g_fsw.mission));
  memset(&g_fsw.mission_status, 0, sizeof(g_fsw.mission_status));
}

static int mission_active_waypoint(bayek_mission_waypoint_t *wp,
                                   real_t *horizontal_distance_m,
                                   const fsw_input_t *in,
                                   const state_estimate_t *estimate,
                                   const vehicle_params_t *params,
                                   bayek_guidance_setpoint_t *setpoint) {
  uint32_t active = g_fsw.mission_status.active_waypoint_index;

  if (!g_fsw.mission_status.loaded || g_fsw.mission.waypoint_count == 0U ||
      active >= g_fsw.mission.waypoint_count) {
    return 0;
  }

  *wp = g_fsw.mission.waypoints[active];
  if (!bayek_guidance_mission_to_waypoint(in, estimate, wp, params, setpoint,
                                          horizontal_distance_m)) {
    return 0;
  }

  while (*horizontal_distance_m <= wp->acceptance_radius_m &&
         active + 1U < g_fsw.mission.waypoint_count) {
    ++active;
    g_fsw.mission_status.active_waypoint_index = active;
    *wp = g_fsw.mission.waypoints[active];
    if (!bayek_guidance_mission_to_waypoint(in, estimate, wp, params, setpoint,
                                            horizontal_distance_m)) {
      return 0;
    }
  }

  g_fsw.mission_status.horizontal_distance_m = *horizontal_distance_m;
  return 1;
}

static int mission_setpoint_from_input(const fsw_input_t *in,
                                       const state_estimate_t *estimate,
                                       const vehicle_params_t *params,
                                       bayek_guidance_setpoint_t *setpoint) {
  bayek_mission_waypoint_t wp;
  real_t horizontal_distance_m;

  if (!in || !estimate || !params || !setpoint ||
      !mission_active_waypoint(&wp, &horizontal_distance_m, in, estimate,
                               params, setpoint)) {
    return 0;
  }
  return 1;
}

static actuator_cmd_t zero_actuators(void) {
  actuator_cmd_t cmd;
  cmd.motor = 0.0f;
  cmd.aileron = 0.0f;
  cmd.elevator = 0.0f;
  cmd.rudder = 0.0f;
  return cmd;
}

void bayek_fsw_init(const bayek_vehicle_interface_t *vehicle) {
  g_fsw.vehicle = vehicle;
  g_fsw.params = vehicle ? vehicle->params : 0;
  bayek_control_init(&g_fsw.control);
  mission_clear();
  bayek_fsw_reset();
}

void bayek_fsw_reset(void) {
  g_fsw.params = g_fsw.vehicle ? g_fsw.vehicle->params : 0;
  bayek_control_reset(&g_fsw.control);
  bayek_nav_reset(&g_fsw.estimate);
}

void bayek_fsw_step(const fsw_input_t *in, fsw_output_t *out) {
  int input_valid;

  if (!out) {
    return;
  }

  if (!g_fsw.vehicle || !g_fsw.params) {
    out->estimate = g_fsw.estimate;
    out->mode = FSW_MODE_FAILSAFE;
    out->actuators = zero_actuators();
    return;
  }

  input_valid = bayek_fault_input_is_valid(in);
  out->mode = bayek_fault_select_mode(in, input_valid);

  if (input_valid) {
    bayek_nav_update(in, &g_fsw.estimate);
  }
  out->estimate = g_fsw.estimate;

  if (out->mode == FSW_MODE_DISARMED || out->mode == FSW_MODE_FAILSAFE) {
    out->actuators = g_fsw.vehicle->safe_actuators(g_fsw.params);
    return;
  }

  if (out->mode == FSW_MODE_MANUAL) {
    out->actuators = g_fsw.vehicle->mix_manual(&in->rc);
    return;
  }

  if (out->mode == FSW_MODE_MISSION) {
    bayek_guidance_setpoint_t setpoint;
    bayek_control_request_t request;

    if (!mission_setpoint_from_input(in, &g_fsw.estimate, g_fsw.params,
                                     &setpoint)) {
      out->mode = FSW_MODE_FAILSAFE;
      out->actuators = g_fsw.vehicle->safe_actuators(g_fsw.params);
      return;
    }

    request = bayek_control_stabilize_step(&g_fsw.control, &setpoint,
                                           &g_fsw.estimate, in);
    out->actuators = g_fsw.vehicle->mix_control(request.throttle, request.roll,
                                                request.pitch, request.yaw);
    return;
  }

  {
    bayek_guidance_setpoint_t setpoint =
        bayek_guidance_stabilize_from_rc(&in->rc, g_fsw.params);
    bayek_control_request_t request = bayek_control_stabilize_step(
        &g_fsw.control, &setpoint, &g_fsw.estimate, in);
    out->actuators = g_fsw.vehicle->mix_control(request.throttle, request.roll,
                                                request.pitch, request.yaw);
  }
}

void bayek_fsw_set_mission(const bayek_mission_plan_t *mission) {
  if (!mission_plan_is_valid(mission)) {
    mission_clear();
    return;
  }
  g_fsw.mission = *mission;
  g_fsw.mission_status.loaded = 1U;
  g_fsw.mission_status.active_waypoint_index = 0U;
  g_fsw.mission_status.waypoint_count = mission->waypoint_count;
  g_fsw.mission_status.horizontal_distance_m = 0.0f;
}

void bayek_fsw_clear_mission(void) { mission_clear(); }

void bayek_fsw_get_mission_status(bayek_mission_status_t *status) {
  if (!status) {
    return;
  }
  *status = g_fsw.mission_status;
}
