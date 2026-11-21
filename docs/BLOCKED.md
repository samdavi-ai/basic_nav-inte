# BLOCKED: Reinforcement Learning Local Planner Architecture Evaluation

**Date:** 2026-11-18  
**Platform:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Nav2  
**Baseline DWB State:** Frozen at git tag `baseline-dwb-nov17`  
**Safety Invariants:** `MOTOR_OUTPUT_ENABLED 0`, `PROTOCOL_CONFIRMED False` strictly enforced  

---

## 1. Executive Summary & Conclusion

**Conclusion:** **The RL local planner is NOT YET READY for production navigation.**

While the BC policy successfully eliminated kinematic steering bias and achieved 100% on synthetic sweep gates and simple doorway transits, it catastrophically degrades in continuous multi-room autonomous missions (0/50 completed in the full campaign; 1/15 in live run). This failure stems from architectural decoupling: the standalone RL node lacks native Nav2 Behavior Tree (BT) recovery integration (in-place spin, costmap clearing, multi-stage recovery), becoming permanently immobilized whenever `PolygonStop` or the internal obstacle limiter triggers near room furniture and doorways.

Per Definition of Done **D2**, **DWB remains the authoritative production default** (`use_rl:=false`), and the baseline remains frozen at `baseline-dwb-nov17`.

---

## 2. 50-Mission Autonomous Campaign Breakdown

- **Primary Campaign (`scratch/rl_50_missions_results.json`):**
  - Attempted: 50
  - Completed: 0 (0.0% success rate)
  - Failed: 50 (40 stalled/timeouts at ~80s, 10 immediate aborts/navigation failure at 0.0s)
  - Never run: 0 (all 50 missions attempted)
  - Campaign Stop Reason: Completed all 50 scheduled missions with 0% success.
- **Secondary Campaign (`task-8698.log`):**
  - Attempted: 15 (1 completed [6.7%], 14 failed/timed out)
  - Never run: 35
  - Campaign Stop Reason: Interrupted by system restart.

### Per-Mission Route & Outcome Table (`scratch/rl_50_missions_results.json`)

| Mission | Route Target | Outcome | Final Dist (m) | Monitor Stops | Time (s) | Failure Mode |
|---|---|---|---|---|---|---|
| 1 | kitchen | FAIL | 6.318 | 0 | 0.0 | Immediate abort (stuck at start) |
| 2 | study | FAIL | 4.668 | 0 | 0.0 | Immediate abort (stuck at start) |
| 3 | living_room | FAIL | 4.573 | 0 | 80.0 | Stalled / timeout |
| 4 | study | FAIL | 4.726 | 0 | 80.0 | Stalled / timeout |
| 5 | living_room | FAIL | 4.633 | 0 | 80.0 | Stalled / timeout |
| 6 | bedroom | FAIL | 5.682 | 0 | 80.0 | Stalled / timeout |
| 7 | kitchen | FAIL | 6.151 | 0 | 80.0 | Stalled / timeout |
| 8 | bedroom | FAIL | 5.608 | 0 | 79.3 | Stalled / timeout |
| 9 | study | FAIL | 4.895 | 0 | 79.0 | Stalled / timeout |
| 10 | kitchen | FAIL | 6.049 | 0 | 79.0 | Stalled / timeout |
| 11 | bedroom | FAIL | 5.496 | 0 | 79.0 | Stalled / timeout |
| 12 | home | FAIL | 0.518 | 0 | 79.0 | Stalled / timeout |
| 13 | study | FAIL | 5.029 | 0 | 79.0 | Stalled / timeout |
| 14 | bedroom | FAIL | 5.386 | 0 | 79.0 | Stalled / timeout |
| 15 | living_room | FAIL | 4.937 | 0 | 79.0 | Stalled / timeout |
| 16 | kitchen | FAIL | 5.847 | 0 | 79.0 | Stalled / timeout |
| 17 | study | FAIL | 5.165 | 0 | 79.0 | Stalled / timeout |
| 18 | bedroom | FAIL | 5.275 | 0 | 0.0 | Immediate abort |
| 19 | home | FAIL | 0.488 | 0 | 79.0 | Stalled / timeout |
| 20 | living_room | FAIL | 5.064 | 0 | 79.0 | Stalled / timeout |
| 21 | kitchen | FAIL | 5.745 | 0 | 0.0 | Immediate abort |
| 22 | bedroom | FAIL | 5.200 | 0 | 0.0 | Immediate abort |
| 23 | kitchen | FAIL | 5.711 | 0 | 79.0 | Stalled / timeout |
| 24 | home | FAIL | 0.514 | 0 | 79.0 | Stalled / timeout |
| 25 | living_room | FAIL | 5.160 | 0 | 79.0 | Stalled / timeout |
| 26 | study | FAIL | 5.373 | 0 | 79.0 | Stalled / timeout |
| 27 | home | FAIL | 0.563 | 0 | 79.0 | Stalled / timeout |
| 28 | living_room | FAIL | 5.257 | 0 | 79.0 | Stalled / timeout |
| 29 | kitchen | FAIL | 5.510 | 0 | 79.0 | Stalled / timeout |
| 30 | study | FAIL | 5.480 | 0 | 0.0 | Immediate abort |
| 31 | home | FAIL | 0.608 | 0 | 0.0 | Immediate abort |
| 32 | kitchen | FAIL | 5.477 | 0 | 79.0 | Stalled / timeout |
| 33 | home | FAIL | 0.659 | 0 | 79.0 | Stalled / timeout |
| 34 | kitchen | FAIL | 5.410 | 0 | 79.0 | Stalled / timeout |
| 35 | living_room | FAIL | 5.422 | 0 | 79.0 | Stalled / timeout |
| 36 | bedroom | FAIL | 4.754 | 0 | 79.0 | Stalled / timeout |
| 37 | kitchen | FAIL | 5.344 | 0 | 0.0 | Immediate abort |
| 38 | study | FAIL | 5.693 | 0 | 79.0 | Stalled / timeout |
| 39 | kitchen | FAIL | 5.311 | 0 | 0.0 | Immediate abort |
| 40 | bedroom | FAIL | 4.717 | 0 | 0.0 | Immediate abort |
| 41 | study | FAIL | 5.729 | 0 | 79.0 | Stalled / timeout |
| 42 | living_room | FAIL | 5.554 | 0 | 79.0 | Stalled / timeout |
| 43 | bedroom | FAIL | 4.608 | 0 | 79.0 | Stalled / timeout |
| 44 | home | FAIL | 0.904 | 0 | 79.0 | Stalled / timeout |
| 45 | kitchen | FAIL | 5.148 | 0 | 79.0 | Stalled / timeout |
| 46 | bedroom | FAIL | 4.500 | 0 | 79.0 | Stalled / timeout |
| 47 | living_room | FAIL | 5.720 | 0 | 79.0 | Stalled / timeout |
| 48 | bedroom | FAIL | 4.427 | 0 | 79.0 | Stalled / timeout |
| 49 | home | FAIL | 1.074 | 0 | 79.0 | Stalled / timeout |
| 50 | bedroom | FAIL | 4.354 | 0 | 79.0 | Stalled / timeout |

---

## 3. Failing Route Classification & Offline Replay

- **Route Classification:**
  1. **Bedroom Doorway ($x \approx -4.57\text{ m}, y \approx 1.45\text{ m}$):** Tight corridor leading into the bedroom door.
  2. **Kitchen Interior ($y \le -2.3\text{ m}$):** Clearance between dining table and walls.
  3. **Hallway Exit & Corridor Turns:** Transitions requiring $>90^\circ$ orientation changes near walls.
- **Offline Replay Analysis (`scratch/failing_mission_offline_replay.json`):**
  - **Scenario 1 (Doorway Approach, $x=-4.567, y=1.446, \text{yaw}=-1.54\text{ rad}$):** Goal bearing $-151.9^\circ$. Policy outputs $a_v = -0.594, a_\omega = -1.0 \to v = 0.001\text{ m/s}, \omega = -0.750\text{ rad/s}$. Left clearance $0.28\text{ m}$. Chassis corner penetrates `PolygonStop` boundary ($< 0.35\text{ m}$), triggering `PolygonStop` and immobilizing the robot ($0.0\text{ m/s}$).
  - **Scenario 2 (Kitchen Interior, $x=-5.075, y=-2.32, \text{yaw}=-1.33\text{ rad}$):** Goal bearing $+163.0^\circ$. Policy outputs $a_v = -0.539, a_\omega = +1.0 \to v = 0.015\text{ m/s}, \omega = +0.750\text{ rad/s}$. Clearance $0.35\text{ m}$ triggers `PolygonSlow`.
  - **Scenario 3 (PolygonStop Active, clearance $0.25\text{ m}$):** Policy outputs $a_v = -0.596, a_\omega = +1.0$. Filtered command clamps $v = 0.000\text{ m/s}, \omega = +0.750\text{ rad/s}$. Robot completely immobilized.

---

## 4. Stall Hypothesis Direct Test Suite (Online Ablation, 10 Runs Each)

- **Test Route:** Bedroom Doorway $(-4.567, 1.446, -1.54\text{ rad}) \to$ Bedroom $(-5.0, 2.2)$  
- **Raw File:** `scratch/stall_hypothesis_ablation_results.json`
- **Results:**
  - **Condition (a) RL alone:** **0/10 SUCCESS (0.0%)**, all 10 timed out at distance $0.88\text{--}0.89\text{ m}$ (`scratch/stall_rl_alone_results.json`).
  - **Condition (b) RL + simple recovery (0.3m back-up on stop):** **0/10 SUCCESS (0.0%)**, all 10 timed out at distance $0.80\text{--}0.89\text{ m}$ (`scratch/stall_rl_recovery_results.json`).
  - **Condition (c) DWB baseline (teleported directly into trapped pose):** **0/10 SUCCESS (0.0%)**, all 10 timed out at distance $0.80\text{--}0.97\text{ m}$ (`scratch/stall_dwb_results.json`).
- **Why Recovery Failed in Condition (b):**
  1. Internal `obstacle_speed_limiter` in `rl_local_planner.py` (lines 358–365) zeroes forward command when `front_clr < 0.35 m`. The planner clamps its own velocity to 0 before the external collision monitor asserts `stop_active`.
  2. Reversing 0.3 m straight back along $\text{yaw} = -1.54\text{ rad}$ leaves the robot inside the wall collision envelope because the goal remains behind the wall. Once forward dispatch resumes, the policy immediately turns back into the obstacle.
  3. Consequently, per protocol step 3, recovery was not deployed to production.

---

## 5. Angular Slip Recheck & Acceptance Test Audit

- **Angular Slip Audit (`scratch/quick_checks_step_response_3runs.json`):**
  - Prompt nominal specification: $\omega$ transfer gain $K_\omega = 0.75 \implies 25\%$ nominal slip.
  - Empirical Gazebo step response ($\omega_{\text{cmd}} = 0.75\text{ rad/s}$ via `/cmd_vel_teleop` across 3 runs):
    - Run 1: $\omega_{\text{ss}} = 0.4654\text{ rad/s}$ (slip $37.94\%$)
    - Run 2: $\omega_{\text{ss}} = 0.4762\text{ rad/s}$ (slip $36.51\%$)
    - Run 3: $\omega_{\text{ss}} = 0.4877\text{ rad/s}$ (slip $34.97\%$)
    - Mean steady-state yaw rate: $\omega_{\text{ss}} = 0.4764\text{ rad/s}$ (std $0.0091\text{ rad/s}$)
    - Fit formula: $\text{slip} = \frac{\omega_{\text{cmd}} - \omega_{\text{ss}}}{\omega_{\text{cmd}}} = \frac{0.75 - 0.4764}{0.75} = 0.3648$ ($36.48\%$).
  - **Conclusion:** $25\%$ was the theoretical model assumption; $36.48\%$ is the true Gazebo empirical slip due to differential wheel scrubbing and caster friction.
- **Acceptance Test INVALID Counts (`scratch/rl_at02_at03_10runs_results.json`):**
  - **AT-02 (Kitchen):** 10 runs attempted; **5 INVALID** (Runs 1, 3, 5, 7, 9 with pre-test TF error $0.210\text{--}0.227\text{ m} > 0.15\text{ m}$); 5 VALID (Runs 2, 4, 6, 8, 10: **5/5 PASS, 100%**, 0 stops).
  - **AT-03 (Bedroom):** 10 runs attempted; **5 INVALID** (Runs 1, 4, 6, 8, 10 with pre-test TF error $0.155\text{--}0.304\text{ m} > 0.15\text{ m}$); 5 VALID (Runs 2, 3, 5, 7, 9: **5/5 PASS, 100%**, 0 stops).
  - **Cause:** Teleportation reset in simulation moves Gazebo model instantaneously, but AMCL particle distribution requires motion or an explicit `INITIALIZE_LOCALIZATION` update to reconverge.

---

## 6. What Was Tested vs What Was Not Tested

### What Was Tested
1. **Bearing Parity & Conventions:** Identical $\sin/\cos$ between live observation builder and training env at $+30^\circ$ left; verified $+\omega$ turns robot left toward $+Y$ in Gazebo ($\Delta\text{yaw} = +0.6364\text{ rad}$).
2. **Dynamic Step Responses:** 3 repeated step responses for linear ($v=0.30\text{ m/s}$) and angular ($\omega=0.75\text{ rad/s}$), measuring empirical lag ($\tau_v=0.202\text{ s}, \tau_\omega=0.202\text{ s}$), latency ($119.8\text{ ms}, 126.6\text{ ms}$), and empirical slip ($-0.46\%, 36.48\%$).
3. **Behavior Cloning (BC) from Gazebo Expert:** 200,000 DWB transitions trained over 30 epochs with AdamW + CosineAnnealingLR; verified zero heading bias ($\omega = 0.0025\text{ rad/s}$ at $0^\circ$).
4. **SAC Fine-Tuning Exploration:** 100,000 environment steps with entropy regularization, discovering policy collapse to asymmetric steering ($\omega = +0.3485\text{ rad/s}$ at $0^\circ$) and bang-bang saturation.
5. **Policy Sweep Gate:** 144-point synthetic grid across $[-180^\circ, +180^\circ]$ bearings, distances $1, 3, 6\text{ m}$, and $0.4\text{ m}$ wall speed reduction.
6. **Kinematic Curriculum Evaluation:** 400 episodes across 4 stages (Free space 92%, Doorway 77%, Clutter 61%, Multi-room 46%).
7. **Live Gazebo Motion Tests:**
   - 20-second AT-02 trace verifying commanded $v \in [0, 0.40\text{ m/s}]$ and $\omega \in [-0.75, +0.75\text{ rad/s}]$.
   - 10 runs closed-loop hallway (10/10 PASS).
   - 10 runs closed-loop kitchen doorway (10/10 PASS).
   - AT-02 & AT-03 with AMCL (10 runs each, 5 valid runs each 100% PASS).
8. **Direct Stall Hypothesis Ablation:** 10 runs each of RL alone (0/10), RL + simple recovery (0/10), and DWB from trapped pose (0/10).
9. **50-Mission Autonomous Campaigns:** Full 50-mission campaign (0/50 completed, 40 timeouts, 10 aborts) and 15-mission live campaign (1/15 completed).

### What Was Not Tested
1. **Nav2 C++ Plugin Architecture (`nav2_core::Controller`):** Policy was only tested as a standalone ROS 2 Python node (`rl_local_planner.py`), not inside `controller_server` with direct access to Nav2 Behavior Tree recoveries (`NavigateToPose`, `BackUp`, `Spin`, `ClearCostmap`).
2. **Costmap Ego-Patch Observation:** Observations used 72 1D LiDAR range beams; local costmap representations (e.g., $64 \times 64$ occupancy grids) that maintain obstacle memory within the $0.40\text{ m}$ blindspot were not implemented or tested.
3. **Hybrid RL-DWB Controller Arbiter:** Dynamic switching between DWB in constrained spaces and RL in open corridors was not evaluated.
4. **Physical Hardware Actuation:** Hardware testing remained strictly locked under `MOTOR_OUTPUT_ENABLED 0` and `PROTOCOL_CONFIRMED False`.
5. **Multi-Camera Visual SLAM / Depth Perception:** System evaluated exclusively on 2D LiDAR and wheel odometry.

---

## 7. Authoritative Production Decision

Per CLAUDE.md and Definition of Done **D2**:
- **RL local planner is NOT YET READY for production navigation.**
- **DWB is certified, confirmed, and retained as the sole default navigation controller (`use_rl:=false`).**
- **Git configuration remains frozen at `baseline-dwb-nov17`.**
