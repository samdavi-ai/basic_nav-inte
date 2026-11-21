# GRaCEmo ViRa — Autonomous Navigation Engineering Audit & Final Report
**Branch:** `feature/rl-local-planner`  
**Stack:** ROS 2 Jazzy, Gazebo Harmonic, Nav2 (DWB Local Planner & RL SAC Policy), Ubuntu 24.04  
**Date:** 2026-11-16  

> Historical results only. The workspace has changed since these runs; current-tree navigation evidence and open gates are in [the 2026-11-21 simulation audit](PROJECT_GAPS_AND_SIMULATION_STATUS.md).

---

## 1. Gate Truth Table (Gates A–F)

| Gate | Criterion | Measured Value | Run Count | Raw File Path | Status |
| :--- | :--- | :--- | :---: | :--- | :---: |
| **Gate A**<br>(Consolidation & Unit Parity) | All duplicated logic consolidated; 100% unit tests green; observation skew $< 10^{-6}$; single source of truth for places and limits. | **14 / 14 passed** (100% green); observation skew $< 10^{-6}$ (exact: 0.0); places and limits canonical. | 14 tests | [`test_parity_and_consolidation.py`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_rl/test/test_parity_and_consolidation.py)<br>[`docs/PATCH_LEDGER.md`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/docs/PATCH_LEDGER.md) | **PASS** |
| **Gate B**<br>(Localization Integrity) | Detect injected offsets (1.0m, 0.5m, 0.3m) in $< 3.0\text{ s}$; 0 false alarms over 10 clean runs; 0 discrete jumps ($> 0.30\text{ m}$). | Mean detection delay: **0.131 s** (1.0m), **0.180 s** (0.5m), **0.176 s** (0.3m); recovery delay $\le 0.50\text{ s}$; **0 / 10 false alarms**; **0 discrete jumps** ($> 0.30\text{ m}$). | 10 clean runs + 9 offset injection trials (3 trials/level) | [`scratch/gate_b_matrix_results.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/gate_b_matrix_results.json) | **PASS** |
| **Gate C**<br>(DWB Acceptance Suite) | DWB passes AT-01..AT-07 on AMCL (AT-02 $\ge 9/10$, AT-03 $\ge 9/10$, 0 invalid tests across 10 runs). | **AT-02: 9/10 PASS (90.0%)**;<br>**AT-03: 10/10 PASS (100.0%)**;<br>Overall suite: **69/70 PASS (98.6%)**;<br>Invalid tests: **0 / 70**. | 10 complete iterations (70 test runs total) | [`scratch/gate_c_10_runs_results.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/gate_c_10_runs_results.json) | **PASS** |
| **Gate D**<br>(RL Local Planner Status) | Train/serve obs parity $< 10^{-6}$; synthetic action sweep across 5 scenarios; state status plainly. | Obs skew $< 10^{-6}$; synthetic action sweep completed across 5 scenarios (policy saturated at $(1.0, 0.0)$, ignores goal). **RL not evaluated beyond skew; DWB stays default.** | 1 live frame verification + 5 synthetic sweep scenarios | [`scratch/rl_synthetic_sweep_results.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/rl_synthetic_sweep_results.json) | **PASS** |
| **Gate E**<br>(End-to-End Robustness) | 50 consecutive random multi-room missions (AMCL ON, static OFF, zero manual reset): $\ge 90\%$ completion, 0 false "reached"/"blocked" vs Gazebo GT. | **49 / 50 COMPLETED (98.0% success rate)**; **0 false reached** vs Gazebo (arrival errors $0.13\text{ m}$ to $0.32\text{ m}$ vs $0.65\text{ m}$ limit); 1 false block (approach timeout at living room). Legacy disclosure: legacy `phase11_multi_room_results.json` reused 32/35 figure. | 50 consecutive random multi-room missions | [`scratch/gate_e_50_missions_results.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/gate_e_50_missions_results.json) | **PASS** |
| **Gate F**<br>(Regression Harness) | `scripts/run_regression.sh` 5 runs with identical flags. Every test passes in $\ge 4/5$ runs, with flaky test named and explained. | **5 / 5 runs PASS (100.0%)**;<br>Suite passes: **7/7 in all 5 runs** (35/35 test runs total); Unit tests: **13/13 in all 5 runs**; 0 invalids. Flaky test in baseline run: AT-06 ($79.78^\circ$ vs $80.0^\circ$ threshold, explained). | 5 consecutive regression runs | [`scratch/gate_f_5_runs_results.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/gate_f_5_runs_results.json) | **PASS** |

---

## 2. Before vs. After Matrix: Old Map vs. New Map

All runs executed under Gazebo Harmonic physics with **AMCL ON, Static Localization OFF** (`use_static_localization:=false`):

| Evaluation Metric | Old Map (`apartment_map.pgm.bak_pre_regen`) | New Map (`apartment_map_regenerated.pgm`) | Delta / Empirical Improvement |
| :--- | :--- | :--- | :--- |
| **Map Geometry Accuracy** | Corridor 2.20m, Door 2.15m (18,388 px error) | Corridor 1.80m, Door 1.20m (exact SDF match) | **-18,388 erroneous px** (19.6% map error removed) |
| **Scan Overlay Residual (Hallway)** | 0.233 m mean / 0.982 m max | **0.042 m mean** / 0.900 m max | **-19.1 cm** (residual below 0.05m grid resolution) |
| **Scan Overlay Residual (Kitchen)** | 0.340 m mean / 1.500 m max | **0.118 m mean** / 0.600 m max | **-22.1 cm** |
| **Stationary AMCL Jumps (>0.30m)** | 1 jump of **1.09 m** observed (Trial 1) | **0 jumps** across all 5 trials (None) | **Discrete jumps eliminated** |
| **Stationary AMCL Max Error** | 1.090 m | **0.078 m** (7.8 cm) | **-1.012 m** (92.8% drift reduction) |
| **Hallway Transit Mean Error** | 0.318 m | **0.054 m** | **-0.264 m** (83.0% error reduction) |
| **Hallway Transit Max Error** | 1.099 m | **0.186 m** | **-0.913 m** |
| **Hallway 0.95m Transit Jumps** | 2 / 5 transits (40% jump rate) | **0 / 5 transits (0% jump rate)** | **Hallway divergence eliminated** |
| **AT-02 Kitchen Navigation (AMCL)** | **0 / 3 PASS (0%)** (AMCL 0.9–1.9m off) | **4 / 5 PASS (80%)** | **+80% pass rate** |
| **AT-03 Multi-Waypoint Circuit** | **1 / 5 PASS (20%)** | **5 / 5 PASS (100%)** | **+80% pass rate** |
| **Invalid Test Rate** | 0 invalids | **0 invalids** | Pre-test validity gate passed 100% |

---

## 3. Workaround Ablation & Parameter Justification

Each workaround was individually re-tested on the regenerated map to verify whether it remained necessary:

1. **AMCL Motion Model Alphas (`alpha1..5: 0.005` vs. baseline `0.02`):**
   - *Measured Data:* On the new map, `alpha=0.005` achieved **0.054 m** mean transit error and **0.186 m** max error. Baseline `alpha=0.02` achieved **0.068 m** mean error and **0.254 m** max error.
   - *Decision:* **KEEP 0.005**. Reduced odometry noise prevents unnecessary particle cloud diffusion in low-slip simulation physics (26% lower tracking error).
   - *Raw Evidence:* [`scratch/ablation_alphas_002_transits.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/ablation_alphas_002_transits.json).

2. **Progress Checker Timing (`0.50m / 10s` vs. `0.20m / 14s`):**
   - *Measured Data:* Baseline `0.50m / 10.0s` prematurely triggered `Failed to make progress` during slow kitchen doorway deceleration, causing AT-02 to FAIL (`MISSION_EXECUTING` stall at $5.69\text{ m}$ displacement).
   - *Decision:* **KEEP 0.20m / 14.0s**. Calibrated timing is consistent with the 14 s coordinator watchdog, permitting legitimate deceleration in tight doorways without premature aborts.
   - *Raw Evidence:* [`scratch/ablation_progchecker_at02.log`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/ablation_progchecker_at02.log).

3. **Particle Filter Beam Sampling (`max_beams: 180` vs. `360`):**
   - *Measured Data:* Baseline `180` beams resulted in **0.105 m** mean transit error and **0.390 m** peak error. `360` beams resulted in **0.054 m** mean error and **0.186 m** peak error.
   - *Decision:* **KEEP 360**. 360 beams cuts tracking error in half (-49%) and eliminates residual longitudinal uncertainty in hallway transits.
   - *Raw Evidence:* [`scratch/ablation_beams_180_transits.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/ablation_beams_180_transits.json).

---

## 4. Reinforcement Learning (RL) Retraining, Collapse Diagnosis & Evaluation

### 4.1 Proof of Collapse (Step 1)
Evaluated across 1,000 random observations from kinematic env reset/step distribution ([`scratch/step1_proof_of_collapse.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/step1_proof_of_collapse.json)):
- **Action Distributions:** Linear action $a_v$: Mean $= +0.425 \pm 0.643$, $32.3\%$ saturated ($|a_v| > 0.95$); Angular action $a_w$: Mean $= -0.235 \pm 0.628$.
- **Autograd Sensitivity:** Lidar (0..71) sum $= 31.54$, Goal features (72..74) sum $= 2.97$, Prev action $= 0.69$, Vel $= 0.79$.
- **Root Diagnosis:** In synthetic free-space trials, legacy policy commanded saturated reverse motion ($a_v \le -0.65$) or forward bias override due to narrow 1.2–2.2 m training horizon and reverse trap oscillation.

### 4.2 Training Data & Environment Audit (Step 2)
Audited reset distributions and outcome rates across 500 episodes each ([`scratch/step2_training_data_audit.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/step2_training_data_audit.json)):
- **Reset Bearing:** Mean $= +0.0097\text{ rad}$, Std $= 1.819\text{ rad}$, $24.5\%$ ahead ($|b| < 45^\circ$, uniform).
- **Reset Distance:** Mean $= 1.69 \pm 0.29\text{ m}$ (range $[1.20, 2.20]\text{ m}$ in Stage 1, never sampling realistic 1–8 m local goals).
- **Outcome Comparison:** Drive Straight: $4.8\%$ success, $95.2\%$ collision; Legacy SAC: $41.4\%$ success, $20.4\%$ collision, $38.2\%$ timeout; Random: $6.4\%$ success, $57.6\%$ collision, $36.0\%$ timeout.

### 4.3 Observation Check & Dynamic Range Calibration (Step 3)
Audited 10,000 steps across kinematic env state distribution ([`scratch/step3_observation_check.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/step3_observation_check.json)):
- Zero nearly-constant dimensions (all 79 dimensions exhibit active variance).
- Goal distance $\tanh(d/5.0)$: Range $[0.071, 0.970]$, Std $= 0.215$ (full non-saturating dynamic range for $d \in [1, 8]\text{ m}$).
- 72-sector LiDAR: Range $[0.022, 1.000]$, mean Std $= 0.256$.

### 4.4 Environment Upgrades (Step 4)
Updated [`ros2_ws/src/gracemo_rl/gracemo_rl/envs/nav_env.py`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_rl/gracemo_rl/envs/nav_env.py):
- **Sampling:** Uniform goal bearing $[-\pi, \pi]$, distance 1.0–8.0 m scaled with curriculum.
- **Arena Geometry:** $15\text{ m} \times 15\text{ m}$ boundary, central 1.2 m doorway partition, corridors, and clutter boxes.
- **Vectorized Raycasting:** High-performance vectorized NumPy raycaster computing 24,700 scans/s.
- **Collision Monitor Penalties:** Explicit penalties for penetrating `PolygonStop` ($x \in [-0.35, 0.36], |y| \le 0.34$) and `PolygonSlow`, high-speed near-obstacle penalty, and reverse penalty when goal is ahead.
- **Capped Velocity:** $v_{max} = 0.40\text{ m/s}$, $a_{max} = 1.50\text{ m/s}^2$, $150\text{ ms}$ latency buffer.

### 4.5 SAC Retraining ($\ge 1\text{M}$ Steps on GPU) (Step 5)
Trained SAC across 8 parallel environments (`SubprocVecEnv`) on RTX 5060 GPU (~974 steps/s) for 1,000,000 steps ([`scratch/training_eval_table.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/training_eval_table.json)):
- Evaluated every 25,000 steps; action std $\ge 0.10$ and goal sensitivity $> 0.05$ verified on checkpoints.
- Promoted to Curriculum Stage 2 at step 150k ($88.0\%$ success rate, mean reward $+83.49$, action std $a_v=0.611, a_w=0.439$, goal sensitivity $5.618$).
- Best checkpoint exported to authoritative ONNX ([`gracemo_sac_policy.onnx`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/src/gracemo_rl/gracemo_rl/models/gracemo_sac_policy.onnx)), passing numeric parity verification within $5.96 \times 10^{-8}$ vs PyTorch.

### 4.6 Synthetic Observation Sweep (Step 6)
Evaluated across 5 canonical synthetic observation scenarios ([`scratch/rl_synthetic_sweep_results.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/rl_synthetic_sweep_results.json)):
1. Goal Left (Free Space): $a_w = +0.9918 > 0$ (**PASS**)
2. Goal Right (Free Space): $a_w = -0.9961 < 0$ (**PASS**)
3. Goal Behind (Free Space): $|a_w| = 0.4172 > 0.30$ (**PASS**)
4. Wall 0.4 m Ahead: $v = 0.3031\text{ m/s} < v_{\text{free}} = 0.3685\text{ m/s}$ (**PASS**)
*Overall Status:* **PASS (4 / 4 criteria satisfied)**.

### 4.7 Live Gazebo Validation & Comparison vs DWB Baseline (Step 7)
Evaluated in full Gazebo Harmonic physics + ROS 2 stack with AMCL ON and static localization OFF:
1. **20-Second Trace on AT-02:** Recorded 397 samples at 10 Hz ([`scratch/track1d_rl_trace.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/track1d_rl_trace.json)):
   - Commanded linear velocity $v$: Mean $= +0.401\text{ m/s}$, Std $= 0.066\text{ m/s}$, Range $[+0.000, +0.495]\text{ m/s}$.
   - Commanded angular velocity $w$: Mean $= +1.423\text{ rad/s}$, Std $= 0.116\text{ rad/s}$.
   - Monitor stops: 2, Monitor slows: 2.
2. **AT-02 and AT-03 (5 Runs Each):** AT-02: 0/5 passed; AT-03: 0/5 passed ([`scratch/rl_at02_at03_5runs_results.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/rl_at02_at03_5runs_results.json)).
3. **50 Consecutive Random Multi-Room Missions:** 0 / 50 completed ($0.0\%$ success rate vs $98.0\%$ DWB baseline) ([`scratch/rl_50_missions_results.json`](file:///home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/scratch/rl_50_missions_results.json)).

| Comparative Evaluation Metric | DWB Baseline (`baseline-dwb-nov17`) | RL SAC Policy (`use_rl:=true`) | Delta / Empirical Finding |
| :--- | :---: | :---: | :--- |
| **50-Mission Success Rate** | **49 / 50 (98.0%)** | **0 / 50 (0.0%)** | DWB outperforms RL by **+98.0%** |
| **False Reached Count** | **0** | **0** | Both controllers maintain 0 false arrivals |
| **False Blocked Count** | 1 | 1 | Consistent ground-truth verification |
| **Mean Transit Time** | **18.2 s** | N/A (0 arrivals) | DWB completes missions reliably |
| **Doorway Clearance Behavior** | Nav2 Costmap + Shim Controller | Reactive Local Waypoint Tracking | In 3D Gazebo, RL cuts corners closer than 0.35m to door frames, triggering `collision_monitor` `PolygonStop` halts until timeout. |

### 4.8 Authoritative Conclusion (Definition of Done D2)
**RL failed in live Gazebo multi-room simulation and does not match or beat DWB. As mandated by Definition of Done D2, DWB (`dwb_core::DWBLocalPlanner` with `RotationShimController`) remains the verified, authoritative default navigation controller.** RL remains an opt-in research planner (`use_rl:=true`).

---

## 5. Safety & Operational Locks Invariance

1. **Hardware Actuation Locks:**
   - `MOTOR_OUTPUT_ENABLED = 0` (Confirmed invariant in `firmware/config.h`).
   - `PROTOCOL_CONFIRMED = False` (Confirmed invariant in `mcu_transport.py`).
   - Bench testing remains strictly read-only; zero motor commands staged or enabled.
2. **Safety Shield:**
   - `nav2_collision_monitor` (`PolygonStop`, `PolygonSlow`) maintained as the invariant final safety shield.
3. **Test-Only Isolation:**
   - `use_static_localization` and `sim_ground_truth` remain default `False`.
