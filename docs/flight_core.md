# Flight Core

The flight core lives in `fsw` and exposes a minimal C API:

```c
void bayek_fsw_init(const bayek_vehicle_interface_t *vehicle);
void bayek_fsw_reset(void);
void bayek_fsw_step(const fsw_input_t *in, fsw_output_t *out);
```

## Internal Domains

`fsw.c` is the public facade and step orchestrator. Domain code lives next to it:

- `nav.c/h`: state-estimate reset and sensor-driven estimate updates.
- `fault.c/h`: input validity, disarm handling, mode selection, and failsafe entry.
- `guidance.c/h`: mode-specific setpoints derived from commands and vehicle limits.
- `control.c/h`: controller state and normalized control requests.

These headers are internal implementation boundaries. External callers should continue to use `fsw.h` unless a future replay, test, or integration need justifies a dedicated public domain API.

## Vehicle Interface

Vehicle repositories bind Bayek through:

```c
typedef struct {
  const vehicle_params_t *params;
  actuator_cmd_t (*mix_manual)(const rc_input_t *rc);
  actuator_cmd_t (*mix_control)(real_t throttle, real_t roll_cmd, real_t pitch_cmd, real_t yaw_cmd);
  actuator_cmd_t (*safe_actuators)(const vehicle_params_t *params);
} bayek_vehicle_interface_t;
```

Bayek owns the interface shape. Vehicle repositories own the concrete implementation.

## API Contract

`bayek_fsw_init()` binds the flight core to a vehicle interface.

`bayek_fsw_reset()` resets controller and estimator state while preserving the selected vehicle interface.

`bayek_fsw_step()` consumes one complete input sample and writes one complete output sample. The caller owns both input and output storage.

If `out` is `NULL`, `bayek_fsw_step()` returns immediately because there is no safe place to report mode, estimate, or actuator output. If `in` is `NULL`, the step reports `FSW_MODE_FAILSAFE`, leaves the estimate unchanged, and commands safe actuator values when a vehicle interface is configured.

The core currently keeps its runtime state in a static internal struct. That avoids dynamic allocation while keeping the public API simple. If multiple simultaneous FSW instances are required later, the natural extension is an explicit context API:

```c
void bayek_fsw_context_init(bayek_fsw_context_t *ctx, const bayek_vehicle_interface_t *vehicle);
void bayek_fsw_context_step(bayek_fsw_context_t *ctx, const fsw_input_t *in, fsw_output_t *out);
```

The current API can remain as a default singleton wrapper around that future context API.

## Boundary Rules

The flight core must not use:

- `malloc`, `free`, or heap-backed containers
- `printf`, `snprintf`, `FILE`, `fopen`, or file I/O
- board or hardware APIs
- simulator APIs
- telemetry transport APIs
- nondeterministic time sources
- vehicle-specific headers

Inputs, outputs, vehicle behavior, and common math utilities are passed in through plain C structs and functions.

## Modes

The initial modes are:

- `FSW_MODE_DISARMED`: command safe actuator values.
- `FSW_MODE_MANUAL`: pass normalized RC commands through the configured vehicle mixer.
- `FSW_MODE_STABILIZE`: map RC sticks to simple attitude/rate setpoints and apply PID utilities.
- `FSW_MODE_FAILSAFE`: command safe actuator values.
- `FSW_MODE_MISSION`: guide to the loaded GPS waypoint mission through the same stabilize controller and control mixer.

The mode logic is intentionally simple. It uses the arm switch, GPS validity, and `dt_s` sanity bounds. This is not intended to be a final safety manager; it exists to make mode transitions deterministic and testable.

FSW input validation is explicit and runs before estimator or control updates. An input sample is invalid when:

- `in == NULL`
- `dt_s <= 0.0f`
- `dt_s > 0.1f`
- `gps.fix_valid == 0`

Invalid input samples do not advance the state estimate and do not step PID controllers. Armed invalid input selects `FSW_MODE_FAILSAFE`. Disarmed input remains `FSW_MODE_DISARMED`, with safe actuator values, so the disarm command stays distinct from runtime input failure.

## Estimate Placeholder

The current estimate update integrates gyro rates into an attitude quaternion and copies simple sensor-derived values into a `state_estimate_t`. This is enough for deterministic replay, control loop wiring, and SITL smoke tests.

Future estimator work should be added behind the same input/output boundary. A complementary filter, EKF, or external estimator can replace the placeholder without changing host simulation, telemetry, or board shims.

## Determinism

Determinism is a first-class design constraint. `bayek_fsw_step()` depends only on:

- current input sample
- static FSW state initialized by `bayek_fsw_init()` and `bayek_fsw_reset()`
- selected vehicle interface and parameters

This makes replay tests meaningful and makes host/embedded behavior easier to compare.
