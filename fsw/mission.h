#ifndef BAYEK_FSW_MISSION_H
#define BAYEK_FSW_MISSION_H

#include "common_types.h"
#include "guidance.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  bayek_mission_plan_t plan;
  bayek_mission_status_t status;
} bayek_mission_state_t;

void bayek_mission_init(bayek_mission_state_t *mission);
void bayek_mission_reset(bayek_mission_state_t *mission);
int bayek_mission_set(bayek_mission_state_t *mission,
                      const bayek_mission_plan_t *plan);
void bayek_mission_clear(bayek_mission_state_t *mission);
void bayek_mission_get_status(const bayek_mission_state_t *mission,
                              bayek_mission_status_t *status);
int bayek_mission_select_active_waypoint(bayek_mission_state_t *mission,
                                         const fsw_input_t *in,
                                         const state_estimate_t *estimate,
                                         const vehicle_params_t *params,
                                         bayek_guidance_setpoint_t *setpoint);

#ifdef __cplusplus
}
#endif

#endif
