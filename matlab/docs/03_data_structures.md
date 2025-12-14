# docs/03_data_structures.md

## Design Goal
Standardize how road segments, robot parameters, runtime frames, and experiment sessions are represented so that:
- Controllers can be tuned per segment.
- Data can be replayed, plotted, and audited.
- State can persist across frames.

## Structures (as specified)

### 1) `road` struct  
**Location (as specified):** `matlab/+capstone/+config`  
**Purpose:** contain:
- Road segment definition (segment identity, type, boundaries/conditions)
- Full tuning information for that segment (controller selection and gains)

**Notes**
- Exact fields were not provided. Field schema is therefore **not verifiable**.

### 2) `robot` struct  
**Purpose:** contain:
- Hardware constants (motor/servo/encoder parameters)
- Timing parameters (sample times for loops, etc.)

### 3) `frame` struct  
**Location (as specified):** `matlab/+capstone/+config`  
**Purpose:** store all information exchanged/processed in one communication frame:
- Raw sensor readings received
- Derived signals (error, flags, mode, state)
- Control outputs computed
- Transmission payload sent
- Ability to reference the previous frame for processing continuity

### 4) `season` struct  
**Purpose:** contain all required session-level information for one run:
- Array of `frame`
- Fixed setup (hardware config, road plan, initial conditions)
- “Current status” such as stable/unstable/line lost (as specified)

## Persistence / Replay
- `season` is intended to enable saving, post-analysis, and rerun/replay logic using stored frames.
