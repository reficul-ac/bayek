# Bayek Documentation

Bayek is the reusable C-first framework layer used by Altair and future vehicles. It owns portable common types and utilities, flight software, telemetry packet helpers, and generic simulation helpers.

## Documents

- [Architecture](architecture.md): repository layout, dependency rules, and module responsibilities.
- [Common Math And Control](common_math_control.md): reusable scalar, vector, quaternion, PID, filter, and limiter utilities.
- [Flight Core](flight_core.md): FSW API, state ownership, modes, estimator placeholder, and control flow.
- [Simulation](simulation.md): deterministic toy plant helpers and simulation extension points.
- [Telemetry](telemetry.md): packet format, CRC design, topic ranges, and decode/encode boundaries.
- [Testing Strategy](testing.md): framework-level unit, integration, and performance expectations.
- [Design Rationale](design_rationale.md): reasons for key framework choices and planned extension points.
