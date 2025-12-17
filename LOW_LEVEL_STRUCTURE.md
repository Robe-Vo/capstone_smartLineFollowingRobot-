# low_level (ESP32 PlatformIO)

## Root
- `platformio.ini` (present but appears minimal; no explicit environments are visible in the file content in this zip)
- `.gitignore`

## Folders
- `src/`
  - `main.cpp` (stub/placeholder)
  - `archive/` (multiple full `main.*` variants: bluetooth / wifi / etc.)
  - `app/` (`app.hpp`, `app.cpp` present; content appears stub/placeholder)
  - `control/` (`control.hpp`, `control.cpp`)
  - `protocol/` (`protocol.hpp`, `protocol.cpp`)
  - `telemetry/` (`telemetry.hpp`, `telemetry.cpp` present but empty in this zip)
- `lib/` (project libraries)
  - `actuators/` (motor/servo + encoder measurement/filtering code appears substantial)
  - `sensors/` (line/ultra/mpu placeholders/implementations)
  - `bluetooth/` (Network wrapper for Bluetooth SPP)
  - `tcp/`, `udp/` (network transports)
  - `command/` (legacy command definitions / parsing)
  - `README`
- `include/` (present)
- `test/` (present)
- `.pio/`, `.vscode/` (generated)

## Observed architectural intent
- Transport layer(s): Bluetooth SPP, UDP, TCP.
- Command/protocol layer: `src/protocol/*` and `lib/command/*` (overlap risk).
- Actuation + sensing: `lib/actuators/*`, `lib/sensors/*`.
- Higher-level app loop/state machine: `src/app/*` (currently stub in this zip).
- Telemetry TX: `src/telemetry/*` (currently empty in this zip).
- Multiple historical/experimental mains in `src/archive/*` (likely the real entry points used previously).


## Tree (depth-limited)

```
low_level/
  .gitignore
  platformio.ini
  .pio/
    build/
      project.checksum
      esp32dev/
        idedata.json
    libdeps/
      esp32dev/
        integrity.dat
        ESP32Servo/
        Unity/
  .vscode/
    c_cpp_properties.json
    extensions.json
    launch.json
    settings.json
  include/
    README
    cfg.hpp
    pin.hpp
  lib/
    README
    actuators/
      actuators.cpp
      actuators.hpp
    bluetooth/
      bluetooth.cpp
      bluetooth.hpp
    command/
      command.cpp
      command.hpp
    sensors/
      sensors.cpp
      sensors.hpp
    tcp/
      tcp.cpp
      tcp.hpp
    udp/
      udp.cpp
      udp.hpp
  src/
    main.cpp
    main.doan
    app/
      app.cpp
      app.hpp
    archive/
      main.bluetooth
      main.dd
      main.duyanh
      main.main
      main.wifi
    control/
      control.cpp
      control.hpp
    protocol/
      protocol.cpp
      protocol.hpp
    telemetry/
      telemetry.cpp
      telemetry.hpp
  test/
    README
```
