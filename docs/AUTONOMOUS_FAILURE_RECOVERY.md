# GRaCEmo ViRa — Autonomous Failure Recovery & Fault Tolerances

**Document Version:** 1.0.0  
**Status:** VERIFIED  

---

## 1. Fault Detection Matrix

The autonomous system continuously evaluates 7 primary failure categories:

| Fault Category | Detection Mechanism | Trigger Condition | Automated Remediation |
| :--- | :--- | :--- | :--- |
| **Path Obstruction** | Nav2 Costmap Clearing / DWB Controller | Local costmap inflation prevents trajectory generation for $> 3.0\text{ s}$ | Transition to `OBSTACLE_HANDLING`, trigger costmap clearing service, re-plan ($N \le 2$). |
| **Motion Stagnation** | Supervisor Watchdog | Robot displacement $\Delta d < 0.05\text{ m}$ over a rolling $14.0\text{ s}$ window while in `MISSION_EXECUTING` | Cancel active action goal, execute recovery spin/backup, replan. |
| **Localization Loss** | AMCL Covariance Monitor | Covariance trace $\text{tr}(\Sigma) > 0.50\text{ m}^2$ or TF lookup failure | Transition to `LOCALIZATION_LOST`, halt motion (`0.0 m/s`), request operator repositioning. |
| **Sensor Failure** | `/scan` and `/odom` Liveness Monitor | Sensor message frequency drops below $5\text{ Hz}$ for $> 2.0\text{ s}$ | Transition to `SENSOR_FAILURE`, latch zero velocity, alert UARCC and EventBus. |
| **Terrain Instability** | `gracemo_terrain_monitor` | $|\theta_{\text{pitch}}| > 0.25\text{ rad}$ or $|\phi_{\text{roll}}| > 0.25\text{ rad}$ | Transition to `TERRAIN_SAFETY_STOP`, lock `/cmd_vel` output to zero. |
| **Communication Timeout** | Supervisor Command Heartbeat | Missing heartbeat or client disconnect during critical maneuver | Transition to `MISSION_PAUSED`, hold position. |
| **Emergency Stop** | `/terrain/emergency_stop` | Message received with `data: true` | Transition to `EMERGENCY_STOP`, immediate zero-twist lock, disengage Nav2 goals. |

---

## 2. Obstacle Recovery Sequence

When an obstacle blocks the robot's trajectory:
1. Nav2 DWB controller detects lack of forward progress.
2. The supervisor transitions to `OBSTACLE_HANDLING`.
3. If `retry_count < max_retries` (default `2`):
   - Call `/global_costmap/clear_entirely_global_costmap` and `/local_costmap/clear_entirely_local_costmap`.
   - Wait $1.0\text{ s}$ for sensor refresh.
   - Increment `retry_count`.
   - Transition to `MISSION_PLANNING` to compute an alternative path.
4. If `retry_count >= max_retries`:
   - Transition to `PLANNING_FAILURE`.
   - Announce obstruction via GraceEMO speech: *"Obstacle cannot be resolved. Mission halted."*
   - Publish failure notification to EventBus and UARCC event log.
   - Robot remains safely stationary awaiting operator command.

---

## 3. Emergency Stop Recovery Protocol

1. When E-Stop is triggered, the supervisor immediately transitions to `EMERGENCY_STOP`.
2. All pending Nav2 goals are cancelled via action client.
3. `/terrain/emergency_stop` remains latched.
4. To recover:
   - The operator must explicitly toggle the Emergency Stop button in UARCC (or publish `data: false` to `/terrain/emergency_stop`).
   - The supervisor verifies subsystem health and localization.
   - Upon successful verification, the supervisor transitions cleanly to `AUTONOMY_READY`.
