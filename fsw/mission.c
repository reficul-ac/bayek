#include "mission.h"

#include "math_utils.h"

#include <string.h>

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

static int mission_plan_is_valid(const bayek_mission_plan_t *plan) {
  uint32_t i;

  if (!plan || plan->waypoint_count == 0U ||
      plan->waypoint_count > BAYEK_MISSION_MAX_WAYPOINTS) {
    return 0;
  }
  for (i = 0U; i < plan->waypoint_count; ++i) {
    if (!mission_waypoint_is_valid(&plan->waypoints[i])) {
      return 0;
    }
  }
  return 1;
}

void bayek_mission_init(bayek_mission_state_t *mission) {
  bayek_mission_clear(mission);
}

void bayek_mission_reset(bayek_mission_state_t *mission) {
  if (!mission || !mission->status.loaded) {
    return;
  }
  mission->status.active_waypoint_index = 0U;
  mission->status.horizontal_distance_m = 0.0f;
}

int bayek_mission_set(bayek_mission_state_t *mission,
                      const bayek_mission_plan_t *plan) {
  if (!mission || !mission_plan_is_valid(plan)) {
    bayek_mission_clear(mission);
    return 0;
  }
  mission->plan = *plan;
  mission->status.loaded = 1U;
  mission->status.active_waypoint_index = 0U;
  mission->status.waypoint_count = plan->waypoint_count;
  mission->status.horizontal_distance_m = 0.0f;
  return 1;
}

void bayek_mission_clear(bayek_mission_state_t *mission) {
  if (!mission) {
    return;
  }
  memset(mission, 0, sizeof(*mission));
}

void bayek_mission_get_status(const bayek_mission_state_t *mission,
                              bayek_mission_status_t *status) {
  if (!status) {
    return;
  }
  if (!mission) {
    memset(status, 0, sizeof(*status));
    return;
  }
  *status = mission->status;
}

int bayek_mission_select_active_waypoint(bayek_mission_state_t *mission,
                                         const fsw_input_t *in,
                                         const state_estimate_t *estimate,
                                         const vehicle_params_t *params,
                                         bayek_guidance_setpoint_t *setpoint) {
  bayek_mission_waypoint_t wp;
  real_t horizontal_distance_m;
  uint32_t active;

  if (!mission || !in || !estimate || !params || !setpoint ||
      !mission->status.loaded || mission->plan.waypoint_count == 0U ||
      mission->status.active_waypoint_index >= mission->plan.waypoint_count) {
    return 0;
  }

  active = mission->status.active_waypoint_index;
  wp = mission->plan.waypoints[active];
  if (!bayek_guidance_mission_to_waypoint(in, estimate, &wp, params, setpoint,
                                          &horizontal_distance_m)) {
    return 0;
  }

  while (horizontal_distance_m <= wp.acceptance_radius_m &&
         active + 1U < mission->plan.waypoint_count) {
    ++active;
    mission->status.active_waypoint_index = active;
    wp = mission->plan.waypoints[active];
    if (!bayek_guidance_mission_to_waypoint(in, estimate, &wp, params, setpoint,
                                            &horizontal_distance_m)) {
      return 0;
    }
  }

  mission->status.horizontal_distance_m = horizontal_distance_m;
  return 1;
}
