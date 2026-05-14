#ifndef BAYEK_FSW_H
#define BAYEK_FSW_H

#include "common_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  const vehicle_params_t *params;
  actuator_cmd_t (*mix_manual)(const rc_input_t *rc);
  actuator_cmd_t (*mix_control)(real_t throttle, real_t roll_cmd,
                                real_t pitch_cmd, real_t yaw_cmd);
  actuator_cmd_t (*safe_actuators)(const vehicle_params_t *params);
} bayek_vehicle_interface_t;

void bayek_fsw_init(const bayek_vehicle_interface_t *vehicle);
void bayek_fsw_reset(void);
void bayek_fsw_step(const fsw_input_t *in, fsw_output_t *out);
void bayek_fsw_set_mission(const bayek_mission_plan_t *mission);
void bayek_fsw_clear_mission(void);
void bayek_fsw_get_mission_status(bayek_mission_status_t *status);

#ifdef __cplusplus
}
#endif

#endif
