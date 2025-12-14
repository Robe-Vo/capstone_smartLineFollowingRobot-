# docs/06_structure_review.md

## What is strong about the proposed structure
1. **Clear separation of concerns**
   - `road` for segment tuning and segment metadata
   - `robot` for immutable hardware/timing constants
   - `frame` for per-cycle data and continuity
   - `season` for run-level persistence, replay, and audit

2. **Data lineage is explicit**
   - Keeping `frame` arrays inside `season` supports reproducibility and offline debugging.

3. **Segment-based tuning is first-class**
   - Centralizing segment tuning in `road` aligns with your requirement to switch controller/gains by known distance/segment.

## Risks / gaps (based on provided description)
1. **Schemas are not specified**
   - Without explicit field definitions and units, changes become fragile across MATLAB scripts/apps.
2. **Mode and stop conditions are underspecified**
   - The `OPERATION -> IDLE` trigger conditions are not defined, which is critical for safety and test repeatability.
3. **Timing ownership ambiguity**
   - You specify loops at 10 ms (ESP32) and 100 ms (MATLAB), but the prompt does not define how timing jitter, missed frames, or stale commands are handled.

## Recommended structure refinements (no new assumptions; only design recommendations)
1. Define explicit schemas for each struct:
   - Required fields, units, and default values.
2. Add a version field:
   - `road.schemaVersion`, `frame.schemaVersion`, `season.schemaVersion`.
3. Make units unambiguous everywhere:
   - e.g., `mm`, `m`, `deg`, `rad`, `ticks`, `ticks/s`, `rpm`, `m/s`.
4. Add health/state flags into `frame` (as a structured section):
   - e.g., `frame.status.lineDetected`, `frame.status.commOk`, `frame.status.saturatedActuator`.
5. Store both raw and processed signals:
   - raw sensor bytes and processed error values in the same frame to enable debugging calibration issues.
