# Architecture

Bayek is organized as a portable C99 framework. Vehicle repositories consume Bayek directly or as a git submodule and provide vehicle-specific parameters, mixers, board code, and host runners.

## Repository Layout

```text
common/      Shared C99 types, math, and control utilities.
fsw/         Portable flight software core and internal FSW domains.
sim/         Generic deterministic plant, 6DOF, fixed-wing, and trim helpers.
host/        Vehicle-agnostic host SITL parsers and condition machinery.
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
host
  ^
  |
vehicle repositories
```

`fsw` exposes reusable domain modules plus the vehicle-interface type. Vehicle code owns the concrete parameter set, mixer, safe actuator policy, and main FSW scheduling.

## Module Responsibilities

`common` owns reusable data types and primitive algorithms. These utilities should stay generic enough for any vehicle or board.

`fsw` owns portable, reusable flight domains:

- `nav`: state-estimate reset and update.
- `fault`: input validity checks and mode/failsafe selection.
- `guidance`: mode-specific setpoint generation.
- `control`: controller state and normalized control requests.
- `mission`: waypoint mission validation, active-waypoint advancement, status, and setpoint selection.

Vehicle repositories orchestrate these domains in their own FSW facade. Bayek should keep those domain APIs vehicle-agnostic.

`sim` owns generic plant dynamics, state propagation, sensor-input helpers, fixed-wing dynamics helpers, and generic trim solving support. It may include reusable airframe-class helpers, such as the fixed-wing level-flight trim adapter, as long as they depend only on Bayek types and not on a concrete vehicle repository.

`host` owns vehicle-agnostic SITL support that is not suitable for embedded flight-core code: dependency-free initial-condition parsing, per-step condition parsing/evaluation, common assignment target machinery, and compatibility shims for reusable host simulation workflows. It may use file I/O because it is a host-only target. It must not include vehicle-specific headers or own concrete runner policy.

Vehicle-specific scenario runners, Monte Carlo profiles, logs, CSV schemas, CLI workflows, and default parameter choices belong in the vehicle repository because they bind Bayek to a concrete vehicle interface and workflow policy.

`telemetry` owns binary packet formatting. It is independent of `altair_fsw_step()` so telemetry can be used by host tools, embedded transports, or HITL without coupling packet handling to control execution.

## Build Model

CMake is the canonical host build path. Bayek exposes named targets:

- `bayek_common`
- `bayek_fsw`
- `bayek_sim`
- `bayek_host_sitl`
- `bayek_telemetry`

Vehicle repositories decide which targets to link.
