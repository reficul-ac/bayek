# Simulation

Bayek simulation helpers are deliberately outside the flight core.

## Toy Plant

`sim/sim_plant.c` implements a deterministic toy plant. It models:

- actuator lag
- rough roll, pitch, and yaw-rate response
- simple airspeed and altitude evolution
- simulated IMU, GPS, baro, and airspeed samples

This plant is not a flight dynamics model. It exists to validate build boundaries, deterministic closed-loop stepping, finite outputs, bounded actuator commands, and repeatable host execution.

## 6DOF Core

`sim/sim6dof.c` provides a reusable C99 rigid-body integrator. It supports selectable world dynamics frames: local NED (`SIM6DOF_FRAME_NED = 0`) and ECEF (`SIM6DOF_FRAME_ECEF = 1`). ECEF is the default. The implemented Earth model is spherical (`SIM6DOF_EARTH_SPHERICAL`) with a default radius of `6378137.0 m`; WGS84 ellipsoid corrections, Earth rotation, Coriolis, and centrifugal terms are not modeled.

The state keeps synchronized NED and ECEF views: `position_ned_m`, `velocity_ned_mps`, and `attitude_body_to_ned` remain available, while `position_ecef_m`, `velocity_ecef_mps`, and `attitude_body_to_ecef` expose the ECEF truth view. In NED mode the legacy local dynamics are integrated and ECEF fields are derived afterward. In ECEF mode ECEF position/velocity are integrated directly, body forces are rotated through the body-to-ECEF quaternion, gravity points toward the Earth center, and the local NED view is derived relative to the configured origin.

Callers provide body-frame force and moment vectors in FRD axes. The core integrates translation, attitude, diagonal-inertia rotational dynamics, actuator lag, gravity, and simulation time. The quaternion is normalized every step and state validation rejects non-finite or physically unbounded values.

`sim/sim_fixedwing.c` is the first airframe model on top of that core. It is deterministic and parameter-struct based: no config parser, allocation, wind, or sensor noise. The model includes thrust, lift, drag, simple control moments, angular-rate damping, and a soft angle-of-attack lift limit for stall protection. It also converts truth state into `fsw_input_t` samples for SITL use.

`sim/trim_solver.c` owns the generic bounded nonlinear trim solver. `sim/sitl_trim.c` owns the current Bayek fixed-wing SITL trim adapter for level-flight fixed-wing state handoff. Vehicle repositories choose when to enable trim, provide concrete limits and parameters, and decide whether trim failures are fatal for a given run.

## Host SITL Harness

`host/` contains Bayek-owned host-only SITL infrastructure that is reusable across vehicle repositories but intentionally kept outside `fsw`:

- `sitl_initial_conditions.c/h`: dependency-free initial-condition files for geodetic pose, local NED velocity, body rates, airspeed, and RC defaults.
- `sitl_conditions.c/h`: per-step condition files with `when = t_s/step comparator value` rules and assignments to generic targets such as `rc.*`, `input.*`, `vehicle_params.*`, `sim_params.*`, `plant.*`, `trim.*`, and `mission.*`.

The `bayek_host_sitl` target may perform file I/O and use host C library facilities. Embedded code and `bayek/fsw` must not depend on it.

## Vehicle Runners

Bayek does not own concrete SITL or Monte Carlo runner executables. Those belong in vehicle repositories because they bind `bayek_fsw_init()` to a concrete `bayek_vehicle_interface_t` implementation.

Altair's host `sitl_runner` currently exposes two scenarios:

- `smoke`: the original toy-plant plumbing test.
- `cruise6dof`: an airborne fixed-wing 6DOF scenario, for example `sitl_runner --scenario cruise6dof --duration 60 --dt 0.01 --output sitl_cruise6dof.csv`.

## Extension Points

Future simulation work can add:

- richer dynamics
- wind and turbulence models
- sensor noise and bias models
- actuator failure injection
- additional vehicle adapter registries for condition targets
- generic scenario/case runner APIs once more vehicle repositories need them
- binary log output
- HITL transport adapters

Those changes should keep `bayek_fsw_step()` as the shared execution boundary.
