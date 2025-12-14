# docs/04_operational_algorithm.md

## Modes
- `IDLE`
- `OPERATION`

## Start Procedure (as specified)
1. Check run conditions.
2. Send command to switch robot mode: `IDLE -> OPERATION`.

## Main Loop (OPERATION) — Per Iteration (as specified)
1. Wait for sensor signal / sensor frame.
2. Process sensor signals.
3. Execute logic operations:
   - drive road state machine (`road`)
   - update session state (`season`)
4. Compute control outputs:
   - speed/position control (as per control architecture)
   - steering control (PID or ON/OFF)
5. Transmit control data to the robot.
6. Save frame + plot/log.
7. Repeat.

## Stop Procedure (as specified)
- Send command to switch robot mode: `OPERATION -> IDLE`.

## Stop Conditions
- Stop conditions were referenced but not defined in the prompt.
- **Không đủ dữ liệu để xác minh** the exact “For conditions” for switching `OPERATION -> IDLE` (e.g., distance reached, emergency stop, line lost timeout, user command).
