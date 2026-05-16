# Bayek Active Backlog

This file tracks reusable framework work for Bayek. Vehicle-specific follow-up
for Altair remains in the parent repository `TODO.md`.

Roadmap and design context live in:

- [Architecture](docs/architecture.md)
- [Common math and control](docs/common_math_control.md)
- [Flight core](docs/flight_core.md)
- [Simulation](docs/simulation.md)
- [Telemetry](docs/telemetry.md)
- [Testing](docs/testing.md)

## Simulation And Trim

- [ ] Add reusable deterministic wind and turbulence model hooks for host
  simulation.
- [ ] Add reusable sensor noise and bias model hooks for fixed-wing SITL input
  generation.
- [ ] Extend fixed-wing trim helpers beyond the initial level-flight handoff
  case while keeping vehicle policy in callers.
- [ ] Add reusable fixed-wing aero database lookup, interpolation, and
  validation helpers that do not depend on a concrete aircraft.

## Telemetry

- [ ] Define common Bayek telemetry topic contracts for heartbeat, state,
  actuator, and reusable framework diagnostics.
- [ ] Add telemetry packet versioning and compatibility checks for the Bayek
  binary envelope.

## Testing

- [ ] Move generic math, control, simulation, trim, telemetry, and host-SITL
  unit coverage from vehicle repositories into Bayek-owned tests.
- [ ] Add fixture vehicle-interface tests for reusable FSW contracts without
  linking concrete vehicle code.
