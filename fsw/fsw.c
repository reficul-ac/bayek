#include "fsw.h"

#include "control.h"
#include "fault.h"
#include "guidance.h"
#include "nav.h"

typedef struct {
  const bayek_vehicle_interface_t *vehicle;
  const vehicle_params_t *params;
  bayek_control_state_t control;
  state_estimate_t estimate;
} fsw_state_t;

static fsw_state_t g_fsw;

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

  {
    bayek_guidance_setpoint_t setpoint = bayek_guidance_stabilize_from_rc(&in->rc, g_fsw.params);
    bayek_control_request_t request = bayek_control_stabilize_step(&g_fsw.control, &setpoint, &g_fsw.estimate, in);
    out->actuators = g_fsw.vehicle->mix_control(request.throttle, request.roll, request.pitch, request.yaw);
  }
}
