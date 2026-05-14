# Testing Strategy

Bayek tests should focus on deterministic framework behavior and boundary correctness.

## Unit Tests

Framework-level unit tests should cover:

- vector math
- quaternion normalization, multiplication through rotation, and Euler conversions
- angle wrapping, clamp, and interpolation
- low-pass filter behavior
- PID output saturation and integrator limits
- explicit integrator anti-windup
- rate and slew limiting
- deadband
- telemetry encode/decode and CRC rejection
- host SITL parsing, condition target validation, and condition evaluation without concrete vehicle symbols

These tests keep low-level numerical and packet behavior visible and separate from vehicle-specific mode wiring.

## Integration Tests

Framework integration tests should use fake or fixture vehicle interfaces rather than linking concrete vehicle code. This keeps Bayek independent from vehicle repositories while still testing the FSW contract.

Vehicle repositories should own tests that verify their parameters, mixers, board code, and concrete `bayek_vehicle_interface_t` implementation.

## Performance Tests

Performance tests should call `bayek_fsw_step()` many times and print timing. Thresholds should be conservative until target hardware, loop rate, and control complexity are better understood.

Early performance numbers are more useful as trend data than hard certification gates.
