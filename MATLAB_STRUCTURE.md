# matlab

## Root scripts
- `PID_program_singleState.m`
- `test_speed_steps_idle_operation.m`
- `track_export_and_debug.m`

## Package `+capstone/`
- `+config/`
  - `cfg_frame.m`, `cfg_roads.m`, `cfg_robot.m`, `cfg_season.m`
  - `PID_lineFollower_milestone.m`
- `+io/`
  - `Network.m` (transport + TX/RX handling on PC side)
  - `Protocol.m` (frame definitions; appears partially placeholder in this zip)
- `+logic/`
  - `isRoadEnd.m`
- `+script/`
  - `+jog/jogDrive.m`
  - `+test/test_speed_steps_idle_operation.m`

## Docs
- `docs/` contains multiple markdown documents:
  - `00_project_overview.md`
  - `01_hardware_summary.md`
  - `02_control_architecture.md`
  - `03_data_structures.md`
  - `04_operational_algorithm.md`
  - `05_interfaces_and_comms.md`
  - `06_structure_review.md`

## Observed architectural intent
- MATLAB is the supervisory controller:
  - Generates control frames according to `Protocol`
  - Sends via `Network`
  - Receives sensor/telemetry frames, logs, and plots
  - Contains test scripts for step tests and road-state control logic


## Tree (depth-limited)

```
matlab/
  PID_program_singleState.m
  test_speed_steps_idle_operation.m
  track_export_and_debug.m
  +capstone/
    +config/
      PID_lineFollower_milestone.m
      cfg_frame.m
      cfg_roads.m
      cfg_robot.m
      cfg_season.m
    +io/
      Network.m
      Protocol.m
    +logic/
      isRoadEnd.m
    +script/
      +jog/
        jogDrive.m
      +test/
        test_speed_steps_idle_operation.m
  app/
  docs/
    00_project_overview.md
    01_hardware_summary.md
    02_control_architecture.md
    03_data_structures.md
    04_operational_algorithm.md
    05_interfaces_and_comms.md
    06_structure_review.md
```
