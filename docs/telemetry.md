# Telemetry

`telemetry` provides reusable binary packet helpers. Telemetry is separate from `fsw` so packet handling does not affect control-loop execution.

## Packet Format

The current packet layout is little-endian:

```text
byte 0      sync byte: 0xA5
byte 1      packet version: 1
bytes 2-3   topic_id
bytes 4-7   timestamp_us
bytes 8-9   sequence
bytes 10-11 payload_len
bytes 12-13 crc16
bytes 14..  payload
```

The maximum payload length is currently 128 bytes.

This binary packet version is a Bayek telemetry-envelope contract. It does not replace Altair live viewer replay schema versioning; replay JSON remains governed by `ALTAIR_REPLAY_SCHEMA_VERSION`, and CSV/ULog-style imports continue to be translated into live viewer session snapshots.

## CRC

The CRC is CRC-16/CCITT style with polynomial `0x1021` and initial value `0xffff`. During CRC computation the CRC field itself is zeroed.

This is simple, deterministic, and adequate for early packet validation. If the transport or safety requirements grow, the CRC and packet framing can be revised behind the same encode/decode API.

## Topics

Common topics include:

- heartbeat
- state
- actuator

Vehicle-specific topic IDs start at `1000`.

## Boundaries

Telemetry helpers do not call `bayek_fsw_step()` and the flight core does not call telemetry helpers. A board, host logger, or HITL adapter may choose when to encode FSW outputs or decode incoming commands.

This separation keeps telemetry bandwidth, packet format, and transport failures from becoming hidden dependencies inside control logic.
