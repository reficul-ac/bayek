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

`sim/sim6dof.c` provides a reusable C99 rigid-body integrator. It uses NED for world position/velocity and FRD for body-frame forces, moments, and angular rates. Callers provide body-frame force and moment vectors, and the core integrates translation, attitude, diagonal-inertia rotational dynamics, actuator lag, gravity, and simulation time. The quaternion is normalized every step and state validation rejects non-finite or physically unbounded values.

`sim/sim_fixedwing.c` is the first airframe model on top of that core. It is deterministic and parameter-struct based: no config parser, allocation, wind, or sensor noise. The model includes thrust, lift, drag, simple control moments, angular-rate damping, and a soft angle-of-attack lift limit for stall protection. It also converts truth state into `fsw_input_t` samples for SITL use.

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
- scenario files
- binary log output
- HITL transport adapters

Those changes should keep `bayek_fsw_step()` as the shared execution boundary.
