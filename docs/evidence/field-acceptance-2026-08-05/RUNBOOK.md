# Native navigation field acceptance

Run ID: `native-nav-20260804T122846Z-3be9496d`

This manifest does not execute motion. Follow each referenced runbook; every live command still requires its explicit confirmation and motion gate.

## 1. live_safety

clear motion area; soft obstacle; physical estop operator; six live scenarios

Gates:

- `go2_read_only_sensor_report`
- `go2_live_motion_suite`
- `go2_live_teleop_estop_arbitration`

Commands / evidence registration:

- `go2_read_only_sensor_report`
  - `python scripts/verify_native_go2_navigation.py --report-path <RUN_DIR>/read-only.json`
  - `python scripts/register_native_navigation_evidence.py go2_read_only_sensor_report <RUN_DIR>/read-only.json --output <RUN_DIR>/go2_read_only_sensor_report-registered.json`
- `go2_live_motion_suite`
  - `run the five --live scenarios in docs/native-navigation-live-acceptance.md into <RUN_DIR>`
  - `python scripts/summarize_native_go2_acceptance.py <SIX_REPORTS> --output <RUN_DIR>/motion-suite.json`
  - `python scripts/register_native_navigation_evidence.py go2_live_motion_suite <RUN_DIR>/motion-suite.json <SIX_REPORTS> --output <RUN_DIR>/go2_live_motion_suite-registered.json`
- `go2_live_teleop_estop_arbitration`
  - `python scripts/verify_native_go2_arbitration.py --live --confirm I_UNDERSTAND_GO2_CONTROL_ARBITRATION --output <RUN_DIR>/arbitration.json`
  - `python scripts/register_native_navigation_evidence.py go2_live_teleop_estop_arbitration <RUN_DIR>/arbitration.json --output <RUN_DIR>/go2_live_teleop_estop_arbitration-registered.json`

References:

- `docs/native-navigation-live-acceptance.md`

## 2. mapping_localization_loop

record mapped route, restart near known initial pose, then record >20 s closed loop

Gates:

- `go2_mapping_replay`
- `go2_relocalization_replay`
- `go2_closed_loop_replay`

Commands / evidence registration:

- `go2_mapping_replay`
  - `python scripts/verify_native_mapping_replay.py mapping <MAPPING_REPLAY> --output <RUN_DIR>/mapping.json`
  - `python scripts/register_native_navigation_evidence.py go2_mapping_replay <RUN_DIR>/mapping.json <MAPPING_REPLAY> --output <RUN_DIR>/go2_mapping_replay-registered.json`
- `go2_relocalization_replay`
  - `python scripts/verify_native_mapping_replay.py relocalization <RELOCALIZATION_REPLAY> --map <MAP> --initial-x <X> --initial-y <Y> --initial-yaw <YAW> --output <RUN_DIR>/relocalization.json`
  - `python scripts/register_native_navigation_evidence.py go2_relocalization_replay <RUN_DIR>/relocalization.json <RELOCALIZATION_REPLAY> <MAP> --output <RUN_DIR>/go2_relocalization_replay-registered.json`
- `go2_closed_loop_replay`
  - `python scripts/verify_native_mapping_replay.py loop_closure <CLOSED_LOOP_REPLAY> --output <RUN_DIR>/loop.json`
  - `python scripts/register_native_navigation_evidence.py go2_closed_loop_replay <RUN_DIR>/loop.json <CLOSED_LOOP_REPLAY> --output <RUN_DIR>/go2_closed_loop_replay-registered.json`

References:

- `docs/native-navigation-mapping-replay-acceptance.md`

## 3. exploration_patrol_visual

partially mapped safe area; four patrol strategies; known-size visual target

Gates:

- `go2_frontier_exploration_trace`
- `go2_four_strategy_patrol_traces`
- `go2_visual_servo_trace`

Commands / evidence registration:

- `go2_frontier_exploration_trace`
  - `python scripts/analyze_native_navigation.py <EXPLORATION_TRACE> --output <RUN_DIR>/exploration.json`
  - `python scripts/register_native_navigation_evidence.py go2_frontier_exploration_trace <RUN_DIR>/exploration.json <EXPLORATION_TRACE> --output <RUN_DIR>/go2_frontier_exploration_trace-registered.json`
- `go2_four_strategy_patrol_traces`
  - `analyze four separate coverage/frontier/random/least_visited traces into four JSON reports`
  - `python scripts/register_native_navigation_evidence.py go2_four_strategy_patrol_traces <FOUR_REPORTS> <FOUR_TRACES> --output <RUN_DIR>/go2_four_strategy_patrol_traces-registered.json`
- `go2_visual_servo_trace`
  - `python scripts/analyze_native_navigation.py <VISUAL_TRACE> --output <RUN_DIR>/visual.json`
  - `python scripts/register_native_navigation_evidence.py go2_visual_servo_trace <RUN_DIR>/visual.json <VISUAL_TRACE> --output <RUN_DIR>/go2_visual_servo_trace-registered.json`

References:

- `docs/native-navigation-exploration-patrol-acceptance.md`
- `docs/native-navigation-visual-servo-acceptance.md`

## 4. terrain_tare

bounded non-flat area with measured step/slope and MCF motion mode

Gates:

- `go2_mid360_terrain_replay`
- `go2_terrain_execution_trace`
- `go2_tare_exploration_trace`

Commands / evidence registration:

- `go2_mid360_terrain_replay`
  - `python scripts/verify_native_terrain3d.py <TERRAIN_REPLAY> --output <RUN_DIR>/terrain.json`
  - `python scripts/register_native_navigation_evidence.py go2_mid360_terrain_replay <RUN_DIR>/terrain.json <TERRAIN_REPLAY> --output <RUN_DIR>/go2_mid360_terrain_replay-registered.json`
- `go2_terrain_execution_trace`
  - `python scripts/analyze_native_navigation.py <TERRAIN_EXECUTION_TRACE> --output <RUN_DIR>/terrain-execution.json`
  - `python scripts/register_native_navigation_evidence.py go2_terrain_execution_trace <RUN_DIR>/terrain-execution.json <TERRAIN_EXECUTION_TRACE> --output <RUN_DIR>/go2_terrain_execution_trace-registered.json`
- `go2_tare_exploration_trace`
  - `python scripts/analyze_native_navigation.py <TARE_TRACE> --output <RUN_DIR>/tare.json`
  - `python scripts/register_native_navigation_evidence.py go2_tare_exploration_trace <RUN_DIR>/tare.json <TARE_TRACE> --output <RUN_DIR>/go2_tare_exploration_trace-registered.json`

References:

- `docs/native-navigation-terrain3d-acceptance.md`

## Final audit

`python scripts/audit_native_navigation_completion.py --run-verifiers --external-evidence-dir <RUN_DIR>`
