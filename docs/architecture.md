# Architecture

Bayek is organized as a portable C99 framework. Vehicle repositories consume Bayek directly or as a git submodule and provide vehicle-specific parameters, mixers, board code, and host runners.

## Repository Layout

```text
common/      Shared C99 types, math, and control utilities.
fsw/         Portable flight software core and internal FSW domains.
sim/         Generic deterministic toy plant helpers.
hitl/        Future hardware-in-the-loop adapters.
telemetry/  Packet encode/decode helpers.
docs/        Framework documentation.
```

## Dependency Direction

Bayek modules must remain vehicle-agnostic. Code in this repository must not include vehicle-specific headers, link vehicle targets, or assume a particular airframe.

The intended dependency graph is:

```text
common
  ^
  |
fsw, sim, telemetry
  ^
  |
vehicle repositories
```

`fsw` receives vehicle behavior through `bayek_vehicle_interface_t`. Vehicle code owns the concrete parameter set, mixer, and safe actuator policy.

## Module Responsibilities

`common` owns reusable data types and primitive algorithms. These utilities should stay generic enough for any vehicle or board.

`fsw` owns the portable flight software loop. The public API is intentionally small: initialize, reset, and step the core. Internally, the implementation is organized by domain:

- `nav`: state-estimate reset and update.
- `fault`: input validity checks and mode/failsafe selection.
- `guidance`: mode-specific setpoint generation.
- `control`: controller state and normalized control requests.
- `fsw`: the orchestration facade that preserves the public API and calls the configured vehicle interface.

The domain headers are internal implementation boundaries for now. Bayek should expose public `nav`, `guidance`, `control`, or `fault` APIs only when a real caller needs those contracts.

`sim` owns generic plant dynamics, state propagation, and sensor-input helpers. Vehicle-specific scenario runners, Monte Carlo profiles, logs, and CLI workflows belong in the vehicle repository because they bind Bayek to a concrete vehicle interface.

`telemetry` owns binary packet formatting. It is independent of `bayek_fsw_step()` so telemetry can be used by host tools, embedded transports, or HITL without coupling packet handling to control execution.

## Build Model

CMake is the canonical host build path. Bayek exposes named targets:

- `bayek_common`
- `bayek_fsw`
- `bayek_sim`
- `bayek_telemetry`

Vehicle repositories decide which targets to link.
