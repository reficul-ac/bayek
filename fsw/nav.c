#include "nav.h"

#include "math_utils.h"

void bayek_nav_reset(state_estimate_t *estimate) {
  if (!estimate) {
    return;
  }
  estimate->attitude = quat_identity();
  estimate->euler.roll = 0.0f;
  estimate->euler.pitch = 0.0f;
  estimate->euler.yaw = 0.0f;
  estimate->angular_rate_rps.x = 0.0f;
  estimate->angular_rate_rps.y = 0.0f;
  estimate->angular_rate_rps.z = 0.0f;
  estimate->position_m.x = 0.0f;
  estimate->position_m.y = 0.0f;
  estimate->position_m.z = 0.0f;
  estimate->velocity_mps.x = 0.0f;
  estimate->velocity_mps.y = 0.0f;
  estimate->velocity_mps.z = 0.0f;
  estimate->airspeed_mps = 0.0f;
}

void bayek_nav_update(const fsw_input_t *in, state_estimate_t *estimate) {
  euler_t delta;
  quat_t dq;

  if (!in || !estimate) {
    return;
  }

  estimate->angular_rate_rps = in->imu.gyro_rps;
  estimate->position_m.x = 0.0f;
  estimate->position_m.y = 0.0f;
  estimate->position_m.z = in->baro.altitude_m;
  estimate->velocity_mps = in->gps.vel_mps;
  estimate->airspeed_mps = in->airspeed.true_airspeed_mps;

  delta.roll = in->imu.gyro_rps.x * in->dt_s;
  delta.pitch = in->imu.gyro_rps.y * in->dt_s;
  delta.yaw = in->imu.gyro_rps.z * in->dt_s;
  dq = quat_from_euler(delta);
  estimate->attitude = quat_normalize(quat_multiply(estimate->attitude, dq));
  estimate->euler = euler_from_quat(estimate->attitude);
}
