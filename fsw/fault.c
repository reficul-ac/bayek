#include "fault.h"

int bayek_fault_input_is_valid(const fsw_input_t *in) {
  if (!in) {
    return 0;
  }
  if (in->dt_s <= 0.0f || in->dt_s > 0.1f) {
    return 0;
  }
  if (!in->gps.fix_valid) {
    return 0;
  }
  return 1;
}

fsw_mode_t bayek_fault_select_mode(const fsw_input_t *in, int input_valid) {
  if (!in) {
    return FSW_MODE_FAILSAFE;
  }
  if (!in->rc.arm_switch) {
    return FSW_MODE_DISARMED;
  }
  if (!input_valid) {
    return FSW_MODE_FAILSAFE;
  }
  if (in->rc.mode_switch == 0U) {
    return FSW_MODE_MANUAL;
  }
  if (in->rc.mode_switch == 1U) {
    return FSW_MODE_STABILIZE;
  }
  if (in->rc.mode_switch == 2U) {
    return FSW_MODE_MISSION;
  }
  return FSW_MODE_FAILSAFE;
}
