# Design Rationale

This document records why Bayek makes certain framework choices. The purpose is to help future changes preserve intent rather than copying the first implementation blindly.

## C First

Bayek is C99 because C has predictable ABI behavior, is easy to integrate into embedded projects, and avoids pulling C++ runtime assumptions into a small control loop.

## `real_t` Is `float`

`real_t` is currently `float` to match common microcontroller hardware and reduce memory bandwidth. The type alias makes a future precision experiment possible without rewriting every interface.

The framework should not add mixed float/double APIs casually. If double precision is needed later, make it a deliberate configuration choice and rerun replay/performance tests.

## No Dynamic Allocation In FSW

The flight core avoids heap allocation to keep timing and failure modes predictable. Caller-owned structs and static internal state are easier to reason about on embedded targets.

If multiple instances are needed later, use caller-owned context structs rather than heap allocation.

## No Formatting Or File I/O In FSW

Formatting and file I/O introduce hidden dependencies, variable timing, and platform assumptions. Logging belongs in host runners, telemetry adapters, or board/HAL layers owned by consumers.

## Static Singleton FSW State

The first API uses a static singleton because it is small and easy for embedded callers. This is not meant to block future multi-instance simulation. The planned extension is an explicit context type while keeping the singleton wrapper for simple targets.

## Vehicle Interface Between Common And FSW

The vehicle interface prevents actuator limits, mixer choices, and safe actuator policies from leaking into generic math or the FSW core. Bayek owns the shape of the interface; vehicle repositories own concrete implementations.

## Toy Plant Instead Of High-Fidelity Dynamics

The current plant validates architecture, determinism, and bounded behavior. A high-fidelity model would take more assumptions than the framework currently has and could obscure whether the software boundaries are correct.

The plant should improve incrementally after the basic interfaces settle.

## Telemetry Outside FSW

Telemetry is independent of the control step because packet formats and transports change more often than control logic. Keeping the boundary separate allows the same FSW outputs to be logged, transmitted, replayed, or ignored by different callers.

## Conservative Tests

The initial tests should focus on simple invariants: math correctness, saturation, determinism, packet CRCs, and smoke execution. They should not overclaim vehicle performance.
