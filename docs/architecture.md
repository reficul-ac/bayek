# Architecture

Bayek is organized as a portable C99 framework. Vehicle repositories consume Bayek directly or as a git submodule and provide vehicle-specific parameters, mixers, board code, and host runners.

## Repository Layout

```text
common/      Shared C99 types, math, and control utilities.
fsw/         Portable flight software core.
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

`fsw` owns flight-mode selection, caller-visible FSW state, state-estimate updates, and control commands. It consumes generic inputs, calls the configured vehicle interface, and produces generic outputs.

`sim` owns generic toy plant dynamics and sensor generation. Vehicle-specific runners belong in the vehicle repository because they bind Bayek to a concrete vehicle interface.

`telemetry` owns binary packet formatting. It is independent of `bayek_fsw_step()` so telemetry can be used by host tools, embedded transports, or HITL without coupling packet handling to control execution.

## Build Model

CMake is the canonical host build path. Bayek exposes named targets:

- `bayek_common`
- `bayek_fsw`
- `bayek_sim`
- `bayek_telemetry`

Vehicle repositories decide which targets to link.
