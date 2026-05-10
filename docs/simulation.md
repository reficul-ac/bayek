# Simulation

Bayek simulation helpers are deliberately outside the flight core.

## Toy Plant

`sim/sim_plant.c` implements a deterministic toy plant. It models:

- actuator lag
- rough roll, pitch, and yaw-rate response
- simple airspeed and altitude evolution
- simulated IMU, GPS, baro, and airspeed samples

This plant is not a flight dynamics model. It exists to validate build boundaries, deterministic closed-loop stepping, finite outputs, bounded actuator commands, and repeatable host execution.

## Vehicle Runners

Bayek does not own concrete SITL or Monte Carlo runner executables. Those belong in vehicle repositories because they bind `bayek_fsw_init()` to a concrete `bayek_vehicle_interface_t` implementation.

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
