# Agent Guidance

Bayek is the reusable C99 framework layer used by Altair and future vehicle
repositories. It must remain vehicle-agnostic.

## Workflow

Work directly on Bayek `main` during development. Do not create branches or pull
requests unless the user explicitly asks for them.

Read the nearby code and relevant docs under `docs/` before changing behavior.
Prefer small deterministic APIs, caller-owned state, and existing C99 patterns.

Bayek currently uses `real_t` as `float`. Preserve C99 portability and avoid
compiler, platform, or host assumptions unless the code is explicitly host-only.

## Backlog Hygiene

Treat the repo TODO list as part of the required closeout for every work item,
including Plan mode investigations. Before finalizing, review Bayek's local
`TODO.md` when present; when working from an Altair checkout and Bayek has no
local TODO, use the parent repository `TODO.md` for discovered follow-up work
that affects Bayek or its Altair integration:

- check off any item that the completed work fully resolves
- add a concise unchecked item for any real bug, missing test, or improvement
  discovered during the work but left outside the current scope
- avoid duplicates; update an existing item when that is clearer than adding a
  new one

Keep entries specific enough for a future agent to act on, and place them under
the closest existing heading.

## Boundaries

Bayek must not include Altair headers, link Altair targets, or assume a specific
aircraft, board, actuator layout, runner policy, CSV schema, or UI. Vehicle
repositories own concrete vehicle parameters, mixers, board integration, SITL
scenarios, runner defaults, presentation, and vehicle-facing tests.

Put reusable framework behavior in Bayek:

- common scalar, vector, quaternion, filter, PID, limiter, and geometry helpers
- portable FSW domains and deterministic control contracts
- generic simulation, fixed-wing dynamics, trim, and host-SITL helper code
- vehicle-agnostic telemetry encode/decode and host parsing behavior

Keep flight-software code deterministic and portable. Do not add heap
allocation, file I/O, wall-clock time, networking, Arduino APIs, or simulator
transport logic to portable FSW paths.

Use explicit structs and caller-owned state for APIs that need memory. Avoid
hidden mutable globals unless they are unavoidable constants or lookup tables.

## Verification

Prefer focused unit tests for reusable math, control, simulation, trim,
telemetry, and host parsing behavior. Framework integration tests should use
fake or fixture vehicle interfaces rather than linking concrete vehicle code.

When Bayek is modified from an Altair checkout, also run the Altair checks that
exercise the affected integration surface. Common local checks from the Altair
root are:

```bash
python3 tools/python/format_repo.py --check
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Use Release warnings-as-errors when touching shared C code or build settings:

```bash
cmake -S . -B build-release -DCMAKE_BUILD_TYPE=Release -DALTAIR_WARNINGS_AS_ERRORS=ON
cmake --build build-release --parallel
ctest --test-dir build-release --output-on-failure
```
