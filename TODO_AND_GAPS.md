# TODO / gaps to complete (based on content visible in this zip)

## High-impact build/run blockers
1. `low_level/src/main.cpp` is a stub.
   - Action: choose one of the working mains in `low_level/src/archive/` and make it the canonical `src/main.cpp` (or split into modules and keep `main.cpp` minimal).
2. `low_level/src/telemetry/telemetry.hpp` and `telemetry.cpp` are empty.
   - Action: implement telemetry framing + transport hooks consistent with the MATLAB `Protocol/Network` expectations.
3. `low_level/src/app/app.hpp` and `app.cpp` appear to be placeholders.
   - Action: implement App state machine (BOOT/IDLE/OPERATION), command RX parsing, scheduling, and calls into `actuators`/`sensors`.
4. `low_level/platformio.ini` content in this zip does not show `env` definitions (board/framework/lib_deps/build_flags).
   - Action: restore a complete PlatformIO configuration (esp32dev, arduino, monitor/upload speeds, lib_deps, build flags).

## Integration consistency (protocol/units)
5. Duplicate/overlapping command definitions:
   - `low_level/src/protocol/*` defines `Cmd::...` values.
   - `low_level/lib/command/*` defines `enum Command ...`.
   - Action: consolidate to a single authoritative protocol definition (recommended: `src/protocol`), and delete/adapter-wrap legacy `lib/command`.
6. Endianness and field widths must match MATLAB.
   - Action: document and enforce: frame lengths, little-endian u16/u32, 11-bit PWM scaling, steering limits (55..105), encoder units (x1/x2/x4).
7. Telemetry schema must be versioned.
   - Action: add `proto_ver`, `seq`, `timestamp`, payload_len, and optional CRC.

## Real-time/scheduling robustness
8. Define explicit tick rates and isolate blocking calls:
   - Action: assign periodic tasks (RX, control, sensors, telemetry) and avoid `delay()` in control paths.
9. Add link-loss fail-safe:
   - Action: if no valid command for `T_timeout`, force motor PWM=0 and mode=IDLE.

## Code hygiene / repo distribution
10. Remove `.pio/` from distributable zip (it is generated).
11. Move experimental mains (`src/archive/*`) into tagged branches or document which is current.

## MATLAB-side alignment checks
12. Ensure `+capstone/+io/Protocol.m` fully specifies the frame formats used on ESP32.
13. Ensure MATLAB `Network.m` timeout/retry aligns with ESP32 telemetry rate and RX scheduling.
