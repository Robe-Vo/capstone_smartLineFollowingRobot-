# docs/02_control_architecture.md

## Control Layers (as specified)

### 1) Low-Level (ESP32)
**Objective:** regulate DC motor speed.
- Controller: PID
- Period: 10 ms
- Inputs: encoder feedback (2-channel incremental)
- Output: motor command (e.g., PWM duty)  
**Note:** output mapping and saturation limits are not specified.

### 2) High-Level (MATLAB Host)
**Objective:** regulate motor position (or distance) while executing a motion profile and segment-based tuning.
- Controller: PID
- Period: 100 ms
- Motion profile: provided externally (details not included)
- Segment logic: controller type and gains may change by known road segments

### 3) Steering Control (ESP32 or MATLAB-supervised; as specified conceptually)
**Objective:** line tracking via servo steering.
- Controllers in use: PID and ON/OFF
- Adaptation: tuning can change per segment (road state)

## Timing / Dataflow (as specified)
- Host sends commands and receives sensor frames.
- Each iteration processes sensor data, updates road/season state, computes control, transmits commands, logs frames, and plots.
