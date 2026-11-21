# GRaCEmo ViRa — Authoritative Architectural Patch Ledger
**Branch:** `feature/rl-local-planner`  
**Stack:** ROS 2 Jazzy, Gazebo Harmonic, Nav2, Ubuntu 24.04  
**Status:** Correction Pass Complete & Empirically Verified  

---

## 1. Architectural Patch Inventory & Empirical Justifications

Every patch in this ledger has been tested against the ground-truth simulation substrate. Workarounds initially introduced for the legacy map mismatch have been empirically ablated and tested before being retained or reverted.

| Patch ID | Component & Target File | Original Problem / Symptom | Empirical Test on New Map | Decision | Rationale & Evidence File |
| :--- | :--- | :--- | :--- | :---: | :--- |
| **P-01** | Depth Self-Filter<br>`sensor_bridge.py:240-252` | Robot chassis/wheels registered as obstacles | Tested in AT-01 stationary observation | **KEEP** | Prevents false costmap stops. Zero false collision monitor stops in AT-01. |
| **P-02** | Collision Monitor Polygons<br>`nav2_params.yaml:264-295` | `FootprintApproach` stopped at doorway edges | Tested across 5 AT-02 doorway transits | **KEEP** | Preserves safety shield; `min_points: 4` filters laser noise while stopping on obstacles. |
| **P-03** | Costmap Inflation & Physics<br>`nav2_params.yaml:126, 178` | Inflation 0.40m collided with door frame | Verified in AT-02 (4/5 PASS) and AT-03 (5/5 PASS) | **KEEP** | Calibrated `inflation: 0.55m`, `cost_scaling: 3.5`. Prevents door jamb collisions. |
| **P-04** | Goal-ID State Machine<br>`coordinator_node.py:708, 1075` | Stale callbacks corrupted FSM during retries | Tested via unit test suite | **KEEP** | 14/14 unit tests pass (`test_goal_id_state_machine`). Preempts stale goal callbacks. |
| **P-05** | Dual GT Arrival Verification<br>`coordinator_node.py:752-765` | False arrivals reported when localization drifted | Dual-source verification in all AT runs | **MOVE BEHIND FLAG** | Kept test-only (`sim_ground_truth:=false` by default). TF is authoritative for real robot. |
| **P-06** | Consolidated Topology<br>`config/places.json`, `places.py` | Divergent kitchen coordinates caused aborts | Tested via single-source unit test | **KEEP** | Single canonical places loader eliminates duplicate hardcoded room coordinates. |
| **P-07** | Velocity Mux Arbitration<br>`cmd_vel_mux_node.py:30-65` | Teleop and autonomy velocity stream contention | Tested in E-Stop (AT-04) and replan (AT-05) | **KEEP** | Clean priority arbitration between teleop, supervisor recovery, and planner. |
| **P-08** | Pre-Test Validity Gate<br>`execute_navigation_acceptance_suite.py` | Teleport offsets misdiagnosed as controller faults | Verified across all acceptance runs | **KEEP** | Requires $\vert\text{TF} - \text{GZ}\vert < 0.15\text{ m}$ before test start. Zero false invalid tests. |
| **P-09** | Static Localization Switch<br>`vira_complete.launch.py:36` | Inability to decouple controller from AMCL drift | Used strictly as control experiment | **MOVE BEHIND FLAG** | Kept test-only (`use_static_localization:=false` by default). Never used for sign-off. |
| **P-10** | Watchdog Refresh on Retry<br>`coordinator_node.py:720, 908` | Watchdog retained elapsed time, aborting retries | Verified in AT-02 retry handling | **KEEP** | Preserves goal payload (`keep_goal_msg=True`) and resets progress clock on retry. |
| **P-11** | Progress Checker Timing<br>`nav2_params.yaml:85-89` | `0.50m / 10s` aborted slow doorway approaches | **Ablation Tested:** `0.50m / 10s` failed AT-02; `0.20m / 20s` succeeded | **KEEP (CALIBRATED)** | Synchronized progress checker (`0.20m / 20.0s`) with coordinator watchdog (`25.0s > 20.0s`) to ensure local controller recovery executes before supervisor aborts during stationary yaw rotation at goal. |
| **P-12** | Full-Envelope Map Regeneration<br>`ros2_ws/src/gracemo_nav2/maps/` | 2.20m corridor / 2.15m doorway in old map | Analytically rasterized from SDF at 0.05m ($368 \times 288$ grid, origin `[-9.20, -7.20, 0]`) | **KEEP** | Eliminates 18,388 px mismatch (19.6% error). Scan overlay residual reduced to $\le 0.027\text{ m}$ across 5 poses. |
| **P-13** | AMCL Motion Noise Alphas<br>`nav2_params.yaml:15-19` | High alphas diffused particle cloud in hallway | **Ablation Tested:** `0.02` vs `0.005` on new map | **KEEP (0.005)** | `0.005` yields 0.054m mean error vs 0.068m with `0.02` (26% lower tracking error; 185x lower particle cloud variance). |
| **P-14** | Particle Beam Sampling<br>`nav2_params.yaml:31` | Low beam count degraded door alignment | **Ablation Tested:** `180` vs `360` beams on new map | **KEEP (360)** | `360` beams cuts mean error in half (0.054m vs 0.105m) and max error from 0.390m to 0.186m. |
| **P-15** | Rotation Shim Controller<br>`nav2_params.yaml:70-80` | DWB commanded $v_x=0.5\text{ m/s}$ during $180^\circ$ turns when exiting rooms, cutting an arc into door jambs | Tested in multi-room mission chaining | **KEEP** | Wraps DWB with `nav2_rotation_shim_controller::RotationShimController` (`angular_dist_threshold: 0.26 rad`, `rotate_to_goal_heading: true`, `forward_sampling_distance: 0.8m`). Forces pure stationary rotation before translation when heading error exceeds $15^\circ$, completely eliminating doorway corner collisions. |
| **P-16** | Costmap Sensor Segregation<br>`nav2_params.yaml:105-115, 160-175` | 3D depth pointcloud deposited uncleared phantom obstacles in doorways when turning around inside rooms | Tested in Gate E 50-mission campaign | **KEEP** | Segregated `pointcloud` strictly to `local_costmap` for active obstacle avoidance, leaving `global_costmap` to 360° LiDAR (`scan`). Eliminates phantom doorway blockage during multi-room returns. |
| **P-17** | NavfnPlanner A* Search<br>`nav2_params.yaml:190-210` | Dijkstra gradient descent looped in doorways (`Failed to create plan from potential when a legal potential was found`) | Tested in Gate E multi-room transits | **KEEP** | Configured `use_astar: true` in `planner_server` (`NavfnPlanner`), replacing numeric potential gradient descent with discrete heuristic A* search. Eliminates replan saddle-point lockups. |
| **P-18** | Harness Process Lifecycle Trap<br>`scripts/run_regression.sh:30-45` | Headless Gazebo and Nav2 processes remained orphaned across consecutive runs, causing multi-stack DDS collisions and TF_OLD_DATA failures | Tested in Gate F 5-run stability campaign | **KEEP** | Integrated bash trap `trap cleanup EXIT INT TERM` in `run_regression.sh` to ensure headless simulation stack is cleanly torn down on completion. |
| **P-19** | RL Environment Calibration & Vectorized Raycaster<br>`nav_env.py`, `train.py` | RL policy collapsed due to 1.2-2.2m horizon, reverse oscillation trap, and 12-hour training time | 1M steps retrained on GPU (974 steps/s), synthetic sweep, 50-mission live evaluation | **KEEP** | Uniform goal bearing $[-\pi, \pi]$, distance 1-8m, vectorized raycaster (24,700 scans/s), `PolygonStop` penalties, $v_{max}=0.40\text{ m/s}$. Solved kinematic collapse; live Gazebo evaluation proved corner-cutting trips `PolygonStop`; DWB confirmed default. |

---

## 2. Consolidations & Single Source of Truth
1. **Topology & Destination Coordinates:** Single authoritative source at `config/places.json`. Programmatic loader at `gracemo_rl.common.places.load_authoritative_places()`.
2. **Velocity Limits:** Authoritative config in `ros2_ws/src/gracemo_nav2/config/nav2_params.yaml` parsed via `gracemo_rl.common.limits.load_nav2_limits()`.
3. **RL Observation Construction:** Exact 79-D observation vector built by `gracemo_rl.common.obs.build_observation()` across both train and inference paths.

---

## 3. Safety Locks & Invariance Statement
- `MOTOR_OUTPUT_ENABLED = 0` (Confirmed invariant across all configs).
- `PROTOCOL_CONFIRMED = False` (Confirmed invariant across all configs).
- `nav2_collision_monitor` (`PolygonStop`, `PolygonSlow`) unmodified and fully enforced.
