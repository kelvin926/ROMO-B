# ADR 0002: PCU command byte order is a required hardware setting

## Decision

The serial bridge requires `serial.command_endian` to be explicitly `big` or
`little` before transmit mode can start. Receive-only mode permits
`unverified`. This ROMO-B unit uses `little`.

## Evidence

The verified manual states that HLV-to-PCU 16-bit fields are Big Endian. On
2026-07-17, however, a commanded raw speed of `1` was transmitted as `00 01` and
the physical platform accelerated toward its maximum: feedback first crossed
0.29 m/s and then reached approximately 1.4-1.7 m/s before HLV E-stop. This is
the behavior expected when the PCU interprets `00 01` as Little Endian raw
`0x0100` (256), rather than raw `1`.

The bridge therefore retains both encodings for traceability, selects Little
Endian in this robot's local configuration and onboarding output, and refuses
to transmit when the setting is absent.

During the same recovery, the PCU emitted E-stop byte `0x05`. Feedback parsing
therefore treats E-stop as a bitmask (`0` is off, any nonzero value is active)
instead of rejecting values above `1`.
