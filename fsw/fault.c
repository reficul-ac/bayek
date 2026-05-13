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
  return in->rc.mode_switch ? FSW_MODE_STABILIZE : FSW_MODE_MANUAL;
}
