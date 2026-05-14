# Flight Domains

Bayek no longer owns a singleton flight loop. Vehicle repositories schedule the loop and call Bayek's reusable domain APIs:

- `fault.c/h`: input validity, disarm handling, mode selection, and failsafe entry.
- `nav.c/h`: state-estimate reset and sensor-driven estimate updates.
- `guidance.c/h`: stabilize and waypoint setpoint generation.
- `control.c/h`: controller state and normalized control requests.
- `mission.c/h`: mission validation, set/clear/status, active-waypoint advancement, and mission setpoint selection.

Altair's concrete flight boundary is `altair_fsw_step()`. That facade owns vehicle-specific scheduling, mission ownership, relative-launch hooks, external-guidance hooks, performance-management hooks, and final actuator mixing.

## Vehicle Interface

Vehicle repositories bind Bayek-compatible mixer behavior through:

```c
typedef struct {
  const vehicle_params_t *params;
  actuator_cmd_t (*mix_manual)(const rc_input_t *rc);
  actuator_cmd_t (*mix_control)(real_t throttle, real_t roll_cmd, real_t pitch_cmd, real_t yaw_cmd);
  actuator_cmd_t (*safe_actuators)(const vehicle_params_t *params);
} bayek_vehicle_interface_t;
```

Bayek owns the interface shape. Vehicle repositories own the concrete implementation and the FSW orchestration that uses it.

## Boundary Rules

Bayek flight domains must not use:

- `malloc`, `free`, or heap-backed containers
- `printf`, `snprintf`, `FILE`, `fopen`, or file I/O
- board or hardware APIs
- simulator APIs
- telemetry transport APIs
- nondeterministic time sources
- vehicle-specific headers

Inputs, outputs, vehicle behavior, and common math utilities are passed in through plain C structs and functions.

## Modes

The shared mode enum remains:

- `FSW_MODE_DISARMED`: command safe actuator values in the vehicle facade.
- `FSW_MODE_MANUAL`: pass normalized RC commands through the vehicle mixer.
- `FSW_MODE_STABILIZE`: map RC sticks to simple attitude/rate setpoints and apply PID utilities.
- `FSW_MODE_FAILSAFE`: command safe actuator values in the vehicle facade.
- `FSW_MODE_MISSION`: guide to the loaded GPS waypoint mission through the same stabilize controller and control mixer.

Mode selection uses the arm switch, GPS validity, and `dt_s` sanity bounds. Vehicle facades may add policy around those reusable primitives.

## Determinism

Bayek domains remain deterministic: they depend only on the input structs, explicit domain state, parameters, and reusable math utilities supplied by the caller.
