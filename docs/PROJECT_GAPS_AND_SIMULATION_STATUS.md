# GRaCEmo ViRa: navigation and simulation status

**Audit date:** 2026-11-21  
**Scope:** the current working tree, ROS 2 Jazzy, Gazebo simulation, and the supplied “Complete Robot Feature Inventory.”  
**Current corridor gate.** The stationary 0.35 m fixture, mirrored placement and larger static box passed in Gazebo. On the exact fixture the padded base maintained 0.131 m of interpolated ground-truth clearance and finished 0.222 m from the requested goal. The study-room mission completed with positive driven clearance. Some proposed study paths and a late-insertion stress test failed the stricter footprint check. The final lifecycle-gated standalone patrol completed 11/11 waypoints in 184.7 s and passed the E-stop and idle-zero checks. General obstacle autonomy and physical release remain open. [Engineering report and current release gates](validation/corridor_routing/REPORT.md)

**Prior campaign baseline.** The final navigation/stop-path tree completed the seeded 50-mission Gate E campaign on the fixed apartment world with 50/50 successes. A one-off static box in an open room was routed around successfully, and the simulated E-stop and continuous idle-zero path passed end to end. A one-command adapter → kernel → ROS → Nav2 → kernel five-room patrol now completes. Before the corridor repair, NavFn produced a path too close to a 0.35 m obstacle for the configured footprint, DWB rejected trajectories, and backup recovery failed. Earlier Smac, inflation and stop-zone trials did not clear that failure. Randomized/moving obstacles, command-source arbitration, terrain, SLAM, sensor-rate, and physical-robot validation remain open.

This report records evidence from the current tree. Older completion reports and benchmark results are historical references; they do not certify this tree. No physical robot movement was performed.

## Adapter / kernel / ROS bridge repair — 2026-11-21

The adapter-to-autonomy boundary had several concrete contract breaks. Coordinator state was posted to a nonexistent `/events` endpoint with a source name the Rust enum did not accept, so mission completion was not returned to the kernel. Rust events also discarded the `trace_id` promised by the shared schema. The Gemini metric-navigation tool could emit a Nav2 action and then independently start the direct NavigationSkill controller, and it reported dispatch as success without waiting for a coordinator result. The previous autonomous mission runner only slept for estimated travel times and inserted fabricated detections.

The bridge path now preserves UUID trace and session IDs across adapters, Rust SSE, the interaction bridge, coordinator mission commands, and coordinator telemetry. Coordinator state uses the kernel's `/emit` endpoint and typed `AutonomyStateChanged` events. The ROS launch starts the kernel bridge even if the kernel is offline at launch, allowing its EventBus client to reconnect to the configured URL. Gemini navigation uses the coordinator route once and waits for a matching `MISSION_COMPLETED` event. `scripts/run_autonomous_mission.py` now sends real configured room waypoints and requires both coordinator mission history and a matching terminal event in kernel history before returning success. Kernel state storage can be redirected with `GRACEMO_STATE_DIR` for isolated runs.

Verification from the earlier bridge repair: Rust workspace tests passed (18 tests), adapter/navigation Python tests passed (47 tests), and the then-current ROS interaction bridge tests passed (9 tests). The final verification for this working tree is recorded below: 13 focused bridge/kernel tests passed, three selected ROS packages built, and the full live Kernel-to-Gazebo patrol completed with a clean process exit.

## What changed in this repair

- Fixed the active Gazebo model. The robot is embedded in `apartment_floor.world`, and its camera and drive settings had diverged from the Xacro model. The active world now uses the Xacro RGB/depth resolutions and rates and the same differential-drive velocity and acceleration limits. The LiDAR minimum range is also consistent.
- Added `scripts/check_simulation_model_consistency.py` to detect future differences in the active world's sensor and drive configuration.
- Reduced the depth point-cloud bridge to the depth camera's configured rate and updated the bridge's camera-info dimensions.
- Made the full simulation launcher own only the process groups it starts, and gave it configurable ROS/Gazebo isolation defaults. The headless ROS launch also supplies a Gazebo configuration-path fallback when the shell has none.
- Added a `--patrol` mode to the standalone launcher. It builds the ROS workspace if missing and the kernel binary if needed, waits for the real autonomy readiness gates, sends the room tour through the kernel, and writes correlated acceptance evidence; ROS logs go under `/tmp` so the launcher also works in a restricted workspace.
- Made the mission runner wait for fresh localization, sensors, Nav2, and an unlatched E-stop before dispatch. It now joins its ROS executor thread before destroying the node/context, so successful missions exit without a process abort.
- Fixed Nav2 startup configuration for explicit goal/progress checker selection and raised the coordinator's AMCL future-TF tolerance to cover the observed simulation clock skew.
- Hardened readiness and stop handling in the coordinator and velocity mux: missions require fresh readiness inputs, and the idle mux publishes and holds a zero command.
- Added `scripts/run_gazebo_navigation_smoke.py`. It checks simulation clock, fresh sensors, Nav2/coordinator readiness, simulation mode, an inactive E-stop, and a connected command subscriber before sending one bounded goal; it reports mission history, displacement, and map-frame goal error.
- Added optional apartment-room doorway staging in the mission coordinator for the bundled simulation. It inserts a route waypoint at a room threshold when a route crosses between the hall and a known room. Nav2's planner, DWB controller, costmaps, and collision monitor stay active for each leg.
- Hardened `scripts/run_gate_e_50_missions.py` with simulator readiness checks, unique mission IDs, Gazebo Harmonic ground truth, per-run timing/path/localization metrics, and safe cancellation if a run aborts. It does not alter obstacle placement.
- Added `scripts/run_gazebo_estop_smoke.py`, a guarded E-stop integration check. It requires motion before asserting STOP, checks the zero command and settled base, verifies a goal is rejected while latched, and requests explicit reset only after those checks pass. Failure paths leave the simulated E-stop asserted.
- Removed unsupported legacy controller-checker selector parameters from the Jazzy config and made the custom behavior tree pass the registered `general_goal_checker` and `progress_checker` IDs directly in each FollowPath action.
- Reworked the navigation watchdog to use measured pose/heading progress rather than commanded velocity, reset its timer on each retry, and expose a 12 s stagnant-motion limit. A controlled blocked route now ends in `NAVIGATION_FAILURE` after three retries instead of hanging or falsely reporting arrival.
- Tightened and parameterized coordinator arrival checks: 0.30 m map error, 0.35 m Gazebo truth error, and 0.15 m map/truth drift. The final 50-mission run passed all goals under these limits alongside Nav2's 0.25 m XY goal checker.
- Changed the command mux to refresh zero at its normal rate while idle. The terrain monitor now refreshes the sole `/cmd_vel` output after 0.5 s without safe upstream commands, so a settled Nav2 smoother cannot leave the base output silent. A 3 s Gazebo probe observed 60 zero commands and 8 mm odometry drift.
- Updated the README with the supported headless launch, model-consistency check, and smoke mission commands. DWB remains the default; RL stays disabled.

## Reproducing the current bounded check

In terminal 1, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch gracemo_gazebo vira_complete.launch.py \
  mode:=localization headless:=true use_rviz:=false \
  launch_interaction:=false use_rl:=false
```

In terminal 2:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 scripts/run_gazebo_navigation_smoke.py --goal-x 0.9 --goal-y 0.0
```

The smoke command sends a real goal to the simulated robot. It does not launch Gazebo by itself. Stop the launch in terminal 1 with Ctrl+C.

To reproduce the seeded 50-mission fixed-world campaign in terminal 2 instead:

```bash
python3 scripts/run_gate_e_50_missions.py \
  --missions 50 --seed 42 \
  --out docs/validation/gate_e_50_missions_2026-11-21_final_arrival_guard.json
```

The same terminal can run the guarded simulated E-stop check:

```bash
python3 scripts/run_gazebo_estop_smoke.py \
  --out docs/validation/gazebo_final_estop_smoke_2026-11-21.json
```

## Current evidence

| Check | Result | Evidence and limits |
|---|---|---|
| Standalone adapter → kernel → ROS → Nav2 → kernel mission | PASS, full apartment patrol | Final one-command headless launch waited for fresh scan/odometry, localization, Nav2 readiness, and an unlatched E-stop before dispatch. All 11/11 generated room and doorway waypoints completed in 159.6 s. Coordinator and kernel both recorded `MISSION_COMPLETED` with the same mission ID and trace ID; the runner exited cleanly. This validates the fixed apartment route, not generalized obstacle or fault recovery. Evidence: [`standalone_patrol_latest.json`](validation/standalone_patrol_latest.json). |
| ROS workspace build | PASS | `colcon build --symlink-install --packages-select gracemo_bridge gracemo_nav2 gracemo_gazebo` completed successfully (3 packages). |
| Focused bridge/kernel tests | PASS | `pytest -q tests/ros2/test_interaction_bridge.py tests/test_kernel_event_contract.py`: 13 passed. Set `ROS_LOG_DIR=/tmp/gracemo_ros_logs` in restricted environments. |
| Autonomous operations tests | PASS with two environment skips | Current offline rerun: 9 passed, 2 skipped. The optional kernel daemon on port 7780 was not running, and the live-ROS test could not initialize `rclpy` in this environment. |
| RL parity/consolidation tests | PASS | `pytest -q --tb=short ros2_ws/src/gracemo_rl/test/test_parity_and_consolidation.py`: 4 passed. This does not qualify the RL policy for use. |
| Model consistency | PASS | `python3 scripts/check_simulation_model_consistency.py` passed after aligning the active world and Xacro. |
| Startup and readiness | PASS, with a Jazzy warning | A fresh headless launch reached `AUTONOMY_READY`. The installed Jazzy controller exposes the checker plugin lists but not legacy `current_*_checker` parameters; the custom BT now supplies the configured checker IDs explicitly. Nav2 may still print its one-time configure warning about those legacy selectors. |
| One-point DWB navigation | PASS, bounded smoke only | After the final watchdog, stop-refresh, and arrival-guard edits, goal `(0.9, 0.0)` completed in 3.6 s with 0.241 m map-frame error and 0.675 m maximum odometry displacement. It is a short regression check, not a reliability result. Evidence: [`gazebo_tolerance_guard_smoke_2026-11-21.json`](validation/gazebo_tolerance_guard_smoke_2026-11-21.json). |
| Prior Gate E 50-mission DWB campaign | PASS, fixed-world scope | Final-tree seed `42`; 50/50 completed, zero false-reached, zero false-blocked, zero manual resets, no failures. Average mission time 34.7 s (range 20.0–54.0 s); mean Gazebo goal error 0.217 m (range 0.120–0.302 m); mean map/Gazebo drift 0.069 m (range 0.006–0.119 m); mean odometry path length 9.245 m. Collision slowdown/stop events were zero. Apartment map and static obstacle placement were fixed; the campaign does not qualify generalized obstacle avoidance. Full records: [`gate_e_50_missions_2026-11-21_final_arrival_guard.json`](validation/gate_e_50_missions_2026-11-21_final_arrival_guard.json). |
| Multi-waypoint and room navigation | PASS, fixed-world scope | All 50 missions completed on the final synchronized model across home, bedroom, study, kitchen, and living room. Automatic doorway staging was exercised in both directions; no resets were performed. Faulted waypoint transitions remain untested. |
| Static obstacle routing before the corridor repair | PARTIAL, four controlled cases plus planner trials | On the final arrival guards, a 0.45 m box in the open study room was routed around; the two-stage mission completed in 22.6 s with 0.228 m goal error and no collision-monitor activation. The 0.35 m corridor box at `(2.0, -0.5)` reproducibly stopped the robot about 0.3 m before its west face: DWB rejected trajectories and three coordinator retries ended in bounded `NAVIGATION_FAILURE`. A measured NavFn path passed within 0.025 m of the box edge. Cost-aware Smac 2D, larger global inflation, and two direction-aware stop-zone configurations did not complete the goal and were reverted. The global costmap marked the box and Smac published a route above it, but DWB and backup recovery still failed. A separate 0.45 m box at `(2.5, 0.0)` was geometrically blocked under a conservative footprint estimate. Evidence: [`gazebo_final_openroom_obstacle_smoke_2026-11-21.json`](validation/gazebo_final_openroom_obstacle_smoke_2026-11-21.json), [`gazebo_final_openroom_obstacle_path_2026-11-21.json`](validation/gazebo_final_openroom_obstacle_path_2026-11-21.json), [`gazebo_corridor_obstacle_reroute_diagnostic_2026-11-21.json`](validation/gazebo_corridor_obstacle_reroute_diagnostic_2026-11-21.json). |
| Goal tolerance consistency | PASS, bounded final-tree campaign | Nav2 uses 0.25 m XY / 0.25 rad yaw. Coordinator rechecks allow 0.30 m map-frame error (5 cm for status/TF sampling), 0.35 m Gazebo truth error, and 0.15 m map/truth drift. The final 50-run campaign stayed within all three guards: maxima 0.255 m TF error, 0.302 m Gazebo error, and 0.119 m drift. Broader pose/fault tests remain. |
| Blocked-route watchdog and collision monitor | PASS, failure handling only | With a stationary 0.35 m box at `(2.0, -0.5)`, Nav2 did not find a route in that injected arrangement. The 12 s measured-progress watchdog retried three times and reported `NAVIGATION_FAILURE` after 59 s, with no false arrival. Collision Monitor logged five slowdowns and five stops; post-failure speed was at most 0.0129 m/s with 0.0025 m drift over the last second. This verifies detection and bounded failure, not successful recovery. Evidence: [`gazebo_blocked_route_watchdog_2026-11-21.json`](validation/gazebo_blocked_route_watchdog_2026-11-21.json). |
| Idle zero and final command output | PASS, Gazebo scope | Following a completed mission, a 3 s probe observed 61 zero twists from `/cmd_vel_mux`, 56 from `/cmd_vel_smoothed`, and 60 from the sole `/cmd_vel` publisher; all sampled velocities were zero. Mux reported `active_source=NONE` and `zero_command_held=true`; odometry drift was 0.0084 m. Collision Monitor emits no idle stream after its source timeout, so Terrain Monitor supplies the persistent final zero. Evidence: [`gazebo_idle_zero_heartbeat_2026-11-21.json`](validation/gazebo_idle_zero_heartbeat_2026-11-21.json). |
| E-stop, latched rejection, and reset | PASS, Gazebo scope | On the exact final coordinator code, the test started a moving goal, latched `EMERGENCY_STOP`, observed zero command after the stop, settled to 0.0192 m/s maximum over the final second with 0.023 m drift, rejected a new goal while latched, and returned to `AUTONOMY_READY` only after explicit reset. Zero remained held after reset. Evidence: [`gazebo_final_estop_smoke_2026-11-21.json`](validation/gazebo_final_estop_smoke_2026-11-21.json). |
| LiDAR | PARTIAL | 360 rays, 274 finite readings, 15.17 Hz, range interval 0.15–12 m, nearest observed return 0.67 m. No known-geometry accuracy or controlled obstacle challenge was run. |
| Wheel odometry | PARTIAL | About 50 Hz; `odom` → `base_footprint` frames valid. Short mission comparison reported zero map/ground-truth drift, but there is no long-run accuracy matrix. |
| IMU | PARTIAL | Valid orientation (unit quaternion) and acceleration observed, about 25 Hz against a configured 50 Hz. The rate shortfall needs investigation. |
| RGB-D camera | PARTIAL | Active world and bridge now agree at RGB 320×240/10 Hz and depth 160×120/5 Hz. Observed ROS rates were about 6.5 Hz RGB and 3.69 Hz depth/point cloud. Data types and camera intrinsics were valid; configured rates were not sustained. |
| TF and maps | PASS for the observed graph | `map → odom → base_footprint → base_link` and LiDAR, IMU, and camera frames resolved. Static map was 368×288 at 0.05 m/cell. A live connected-tree check is not a long-run TF health test. |
| Costmaps | PARTIAL | Global and local costmaps published (368×288 and 100×100); cells were populated. One known box was inserted into the open study room and routed around; exact scan-to-cell alignment and inflation margin across orientations and narrow passages remain unmeasured. |
| Shutdown | PASS, launch-level check | The final Gazebo campaign was stopped with Ctrl+C; `ros2 launch` returned exit code 0 after lifecycle nodes cleaned up. No forced `kill -9` was used. |

Historical reports also mention DWB at 49/50 and RL at 0/50. Those are separate historical runs. The current-tree DWB result is the seeded 50/50 fixed-world campaign above. Run RL only in its disabled/experimental path until it passes the same acceptance criteria as DWB.

## Inventory audit: NAV-01 through NAV-27

`PASS` means the named bounded check was observed; it does not imply broad acceptance. `PARTIAL` means some implementation or direct evidence exists but a required validation remains. `NOT VERIFIED` means there is not current direct evidence for the behavior.

| ID | Status | Current finding and remaining evidence |
|---|---|---|
| NAV-01 Robot URDF | PARTIAL | URDF/Xacro and embedded Gazebo model exist. Sensor and drive parameters were reconciled; full link dimensions, collision geometry, wheel/waist joints, and physical-robot correspondence remain unvalidated. |
| NAV-02 Gazebo spawning | PASS | The robot spawned in the apartment world and published the navigation sensor and odometry topics during current headless runs. |
| NAV-03 Physics | PARTIAL | The robot drove and stopped in simulation. No systematic slip, tip, acceleration, or dynamics sweep was run on the final configuration. |
| NAV-04 Wheel controller | PASS | Bounded Nav2 motion produced odometry displacement and completed a goal. Wheel-level calibration and command-to-wheel dynamics remain unmeasured. |
| NAV-05 LiDAR | PARTIAL | Scan shape, range, and rate were measured. Accuracy against known obstacle geometry remains unverified. |
| NAV-06 Odometry | PARTIAL | Topic rate and frames are correct; short-run pose comparison was favorable. Long-duration drift and varied-motion characterization remain. |
| NAV-07 IMU | PARTIAL | Message contents were valid; observed rate was about half the configured rate. Synchronization and bias/noise characterization remain. |
| NAV-08 TF2 | PASS | Required map, odom, base, LiDAR, IMU, and camera transforms were present in the observed graph. Continuous uptime/frequency has not been measured. |
| NAV-09 Mapping / SLAM | NOT VERIFIED | SLAM Toolbox is present in the architecture; no fresh mapping run was completed. |
| NAV-10 AMCL | PARTIAL | AMCL localized during short missions after future-TF tolerance repair. No kidnapped-robot, recovery, or long-duration localization stress test was run. |
| NAV-11 Costmaps | PARTIAL | The corrected optical depth cloud supplies both obstacle layers. In the 0.35 m corridor case the median count of nearby lethal cells rose from zero to 19 in each map. Forward planar lidar coverage has a self-occluded sector. |
| NAV-12 Global planner | PARTIAL | NavFn paths cleared the full padded rectangle on the pre-positioned corridor box and both placement variants. The delayed-start global margin approached zero. Some proposed study-room paths intersected the padded envelope under offline analysis. |
| NAV-13 Local controller | PASS for the pre-positioned corridor and study fixtures | The final corridor produced 195 selected DWB rollouts, positive clearance and zero all-trajectory rejection cycles. A late insertion still produced a selected rollout with negative padded-footprint clearance. |
| NAV-14 Obstacle avoidance | PARTIAL | The pre-positioned corridor fixtures and the open-room goal passed executed clearance checks. The physically blocked route terminated with no completed waypoint. The late-insertion stress case failed clearance. [Measurements](validation/corridor_routing/REPORT.md) |
| NAV-15 Velocity mux | PARTIAL | Idle `NONE` source and held zero output were observed. Arbitration under competing sources and timeout/stale-command injection remain untested. |
| NAV-16 Collision monitor | PARTIAL | The blocked-route test logged five slowdown and five stop activations. The open-room reroute and 50-mission campaign had zero activations. Stop distance, latency, sensor dropout, and independent stop-path behavior remain unmeasured. |
| NAV-17 Terrain monitor | NOT VERIFIED | IMU telemetry reached the monitor, but ramp/incline thresholds and stop behavior were not exercised. |
| NAV-18 Behaviour tree | PARTIAL | Nominal goals, a staged two-leg room mission, and bounded planner/controller retry exhaustion were exercised. Successful recovery from a blocked route and localization-loss branches remain untested. |
| NAV-19 Mission coordinator | PASS, nominal mission scope | The coordinator completed 50/50 unique mission IDs without manual reset, including staged multi-leg room transitions, with tighter map/Gazebo/drift arrival guards verified in the same campaign. Failure recovery and faulted waypoint transitions remain unverified. |
| NAV-20 Recovery | PARTIAL | The measured-progress watchdog detects a stall after 12 s and exhausts three retries to `NAVIGATION_FAILURE` in about 59 s, preventing false arrival. In the corridor repro, Nav2's backup behavior failed and the robot did not reroute. The mux and collision stop path remained active; successful autonomous recovery remains open. |
| NAV-21 Cancellation | PARTIAL | A timed-out obstacle mission was cancelled back to `AUTONOMY_READY`. The mux zero latch and continuous final `/cmd_vel` output are now verified; cancellation through each command source remains untested. |
| NAV-22 E-stop | PASS, Gazebo autonomous path | Final-tree E-stop smoke passed activation during motion, zero command, bounded settling, rejection of a new goal while latched, explicit reset, and return to `AUTONOMY_READY`. Teleop/mux arbitration and hardware behavior were not tested. |
| NAV-23 RL planner | NOT VERIFIED | Experimental; historical full-mission result was 0/50. RL remains disabled and is not acceptance-certified. |
| NAV-24 Frontier exploration | PARTIAL | Frontier code exists but is not launched in the default navigation path or acceptance-tested. Current implementation needs reachable-frontier clustering and boundary/reachability review before relying on it. |
| NAV-25 Voice navigation | NOT VERIFIED | Voice and place-resolution components exist, but no fresh speech-to-goal-to-completed-navigation path was run. |
| NAV-26 Monitoring | PARTIAL | Coordinator status, sensor freshness, terrain status, and mission history were observed. Fault injection for stale sensors and monitoring continuity remains. |
| NAV-27 Full mission acceptance | PASS, Gate E fixed-world criterion | The final tree completed 50/50 seeded sequential missions with AMCL on, static localization off, no manual resets, zero false-reached/false-blocked outcomes, and zero campaign failure. It also passed the bounded E-stop and one open-room obstacle case. This does not cover randomized/moving obstacles, successful recovery from blocked routes, terrain, SLAM, sensor-rate targets, voice/UI paths, or physical hardware. |

## Remaining completion gates

1. Fix the reproduced corridor execution/recovery defect: the global costmap marked the box and a Smac path sample went around it, but DWB rejected its trajectories and backup failed. Trace the actual path, controller footprint, map/odom pose, and collision-monitor velocities together; then verify swept-footprint clearance and repeated reroutes. After that, expand to varied object sizes/poses, randomized layouts, and moving objects. A separate 0.45 m box arrangement was geometrically blocked under the conservative footprint estimate and should remain a bounded failure case.
2. Add successful blocked-path recovery, Nav2 recovery-timeout, and localization-loss tests. Retry exhaustion has a deterministic 12 s watchdog and bounded `NAVIGATION_FAILURE` outcome, but the Nav2 backup failed in the corridor repro; faulted waypoint transitions remain open.
3. Extend E-stop coverage to competing teleop/RL/navigation sources and stale-input arbitration. The autonomous Nav2 E-stop, latched rejection, explicit reset, and continuous idle-zero path pass in Gazebo.
4. Explain and improve the measured sensor-rate shortfalls (RGB about 6.5/10 Hz, depth about 3.69/5 Hz, IMU about 25/50 Hz) and measure timestamp synchronization under sustained simulation.
5. Run SLAM from a clean world, test ramp thresholds and terrain stop behavior, and evaluate frontier exploration separately from goal-directed Nav2.
6. Validate geometry, wheel calibration, and stopping behavior against the actual robot before any hardware deployment. This audit did not move or command physical hardware.
7. Extend the live adapter → kernel → ROS → Nav2 → kernel acceptance beyond the fixed five-room patrol to randomized routes and sensor/localization faults. The nominal standalone round trip now passes, and the bridge preserves both trace and mission IDs. Speech recognition and UARCC-to-completed-mission paths still need their own live acceptance runs; RL remains disabled until it meets the DWB gate.

## Operational notes

- Use `./scripts/launch_full_simulation.sh --patrol --no-gui` for a one-command, readiness-gated full apartment patrol, or the headless launch and smoke harness above for controlled navigation checks. DWB is the default and RL remains disabled.
- Several older experiment and acceptance shell scripts still contain broad `pkill`, `killall`, or `kill -9` cleanup. They were not run or rewritten as part of this navigation repair; do not run them alongside another simulation until their cleanup is scoped.
- The repository already contained extensive unrelated user changes and generated artifacts. This repair preserved them; inspect `git status` before staging or committing.
- The project instructions require Agent Bridge task discovery, approval checks for consequential actions, and publishing execution evidence. Agent Bridge queue, approval-status, and result-publishing tools were not present in this session's tool inventory. All motion evidence in this report is Gazebo-only; no physical robot was commanded.
