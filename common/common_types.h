#ifndef BAYEK_COMMON_TYPES_H
#define BAYEK_COMMON_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef float real_t;

typedef struct {
  real_t x;
  real_t y;
} vec2_t;

typedef struct {
  real_t x;
  real_t y;
  real_t z;
} vec3_t;

typedef struct {
  real_t m[3][3];
} mat3_t;

typedef struct {
  real_t w;
  real_t x;
  real_t y;
  real_t z;
} quat_t;

typedef struct {
  real_t roll;
  real_t pitch;
  real_t yaw;
} euler_t;

typedef struct {
  vec3_t accel_mps2;
  vec3_t gyro_rps;
  uint32_t timestamp_us;
} imu_sample_t;

typedef struct {
  real_t lat_deg;
  real_t lon_deg;
  real_t alt_m;
  vec3_t vel_mps;
  uint8_t fix_valid;
  uint32_t timestamp_us;
} gps_sample_t;

typedef struct {
  real_t pressure_pa;
  real_t altitude_m;
  uint32_t timestamp_us;
} baro_sample_t;

typedef struct {
  real_t true_airspeed_mps;
  uint32_t timestamp_us;
} airspeed_sample_t;

typedef struct {
  real_t throttle;
  real_t roll;
  real_t pitch;
  real_t yaw;
  uint8_t arm_switch;
  uint8_t mode_switch;
} rc_input_t;

typedef struct {
  quat_t attitude;
  euler_t euler;
  vec3_t angular_rate_rps;
  vec3_t position_m;
  vec3_t velocity_mps;
  real_t airspeed_mps;
} state_estimate_t;

typedef struct {
  real_t motor;
  real_t aileron;
  real_t elevator;
  real_t rudder;
} actuator_cmd_t;

typedef struct {
  imu_sample_t imu;
  gps_sample_t gps;
  baro_sample_t baro;
  airspeed_sample_t airspeed;
  rc_input_t rc;
  real_t dt_s;
} fsw_input_t;

typedef enum {
  FSW_MODE_DISARMED = 0,
  FSW_MODE_MANUAL = 1,
  FSW_MODE_STABILIZE = 2,
  FSW_MODE_FAILSAFE = 3,
  FSW_MODE_MISSION = 4
} fsw_mode_t;

typedef struct {
  actuator_cmd_t actuators;
  fsw_mode_t mode;
  state_estimate_t estimate;
} fsw_output_t;

typedef struct {
  real_t max_airspeed_mps;
  real_t min_airspeed_mps;
  real_t max_roll_rad;
  real_t max_pitch_rad;
  real_t max_yaw_rate_rps;
  real_t max_actuator;
  real_t min_actuator;
  real_t safe_motor;
  real_t safe_surface;
} vehicle_params_t;

#ifndef BAYEK_MISSION_MAX_WAYPOINTS
#define BAYEK_MISSION_MAX_WAYPOINTS 16U
#endif

typedef struct {
  real_t lat_deg;
  real_t lon_deg;
  real_t alt_m;
  real_t throttle;
  real_t acceptance_radius_m;
} bayek_mission_waypoint_t;

typedef struct {
  uint32_t waypoint_count;
  bayek_mission_waypoint_t waypoints[BAYEK_MISSION_MAX_WAYPOINTS];
} bayek_mission_plan_t;

typedef struct {
  uint8_t loaded;
  uint32_t active_waypoint_index;
  uint32_t waypoint_count;
  real_t horizontal_distance_m;
} bayek_mission_status_t;

#ifdef __cplusplus
}
#endif

#endif
