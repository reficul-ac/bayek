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

#ifdef __cplusplus
}
#endif

#endif
