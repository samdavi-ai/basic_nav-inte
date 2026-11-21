# GRaCEmo ViRa — Original Algorithms & Mathematical Formulations

**Project:** GRaCEmo ViRa Autonomous Humanoid Service Robot  
**Auditor:** Principal Robotics Research Scientist & Autonomous Systems Architect  
**Scope:** Algorithm Reverse-Engineering, Mathematical Specifications, Pseudocode, and Novel Formulations  
**Status:** Canonical Implementation Audited | Proposed Algorithmic Enhancements Defined

---

## 1. Introduction & Overview

This document presents the complete mathematical and algorithmic reverse-engineering of the mechanisms currently implemented in GRaCEmo ViRa. It rigorously separates:
1. **Existing Implemented Algorithms:** Exact mathematical logic, pseudocode, parameters, and execution characteristics of the code residing in `vira_interaction/`, `ros2_ws/src/gracemo_nav2/`, and `ros2_ws/src/gracemo_bridge/`.
2. **Standard vs. Novel Classification:** An objective technical evaluation of whether each mechanism represents standard open-source robotics practice or novel intellectual property.
3. **Proposed Original Algorithmic Leap:** The mathematical formulation of the **Multi-Sensor Kinematic Consistency Index ($\Gamma$)** and **Autonomous Mission Integrity Metric ($\mathcal{I}$)** designed to provide genuine patentable novelty.

---

## 2. Existing Implemented Mechanisms

### 2.1 Algorithm 1: Fast-Path Intent Matching and Landmark Spatial Resolution

- **Source Location:** `vira_interaction/core/fast_path.py` and `vira_interaction/core/place_resolver.py`
- **Execution Context:** Python 3.12 synchronous thread inside `InteractionGateway.process_text()`
- **Latency:** $\approx 1.2\text{ ms}$

#### Mathematical & Algorithmic Formulation

Let $u \in \mathcal{U}$ be the raw user utterance string. The system evaluates a set of pre-compiled regular expression patterns $\mathcal{P} = \{P_{stop}, P_{cancel}, P_{pause}, P_{resume}, P_{status}, P_{nav}\}$.

$$\text{Match}(u) = \begin{cases} 
(\text{"EMERGENCY\_STOP"}, \emptyset) & \text{if } u \models P_{stop} \\
(\text{"CANCEL\_MISSION"}, \emptyset) & \text{if } u \models P_{cancel} \\
(\text{"PAUSE\_MISSION"}, \emptyset) & \text{if } u \models P_{pause} \\
(\text{"RESUME\_MISSION"}, \emptyset) & \text{if } u \models P_{resume} \\
(\text{"GET\_STATUS"}, \emptyset) & \text{if } u \models P_{status} \\
(\text{"NAVIGATE"}, \text{Resolve}(\text{ExtractPlace}(u))) & \text{if } u \models P_{nav} \\
(\text{"FALLBACK\_LLM"}, \emptyset) & \text{otherwise}
\end{cases}$$

The landmark resolution function $\text{Resolve}(q): \mathcal{S} \to \mathbb{R}^3 \cup \{\bot\}$ maps query token $q$ against landmark database $\mathcal{D}_{places} = \{(k_i, \mathbf{p}_i, \mathbf{a}_i)\}_{i=1}^N$:

$$\text{Resolve}(q) = \begin{cases} 
\mathbf{p}_i = (x_i, y_i, \theta_i) & \text{if } \exists i \text{ s.t. } q \in \text{aliases}(k_i) \land \mathbf{p}_i \in \Omega_{map} \\
\bot & \text{otherwise}
\end{cases}$$

where map boundary $\Omega_{map} = \{ (x, y) \in \mathbb{R}^2 \mid x_{min} \le x \le x_{max}, y_{min} \le y \le y_{max} \}$.

```python
# Algorithmic Pseudocode: FastPathMatcher & PlaceResolver
def fast_path_pipeline(utterance: str, landmark_db: Dict) -> Tuple[str, Dict]:
    text_clean = utterance.strip().lower()
    
    # 1. Immediate Safety Pre-emption
    if re.search(r"\b(emergency\s+stop|e-?stop|abort|freeze|shutdown)\b", text_clean):
        return ("EMERGENCY_STOP", {"priority": 0})
        
    # 2. Priority Intent Matching
    for intent, pattern in INTENT_PATTERNS.items():
        match = re.search(pattern, text_clean)
        if match:
            if intent == "NAVIGATE":
                raw_target = match.group("destination")
                target_pose = resolve_landmark(raw_target, landmark_db)
                if target_pose and check_bounds(target_pose):
                    return ("NAVIGATE", {"target": target_pose})
                else:
                    return ("NAVIGATE_REJECTED", {"reason": "unknown_or_out_of_bounds"})
            return (intent, {})
            
    return ("FALLBACK_LLM", {"text": utterance})
```

- **Technical Assessment:** **Standard Engineering.** Regex parsing and key-value landmark lookups are textbook natural language understanding (NLU) patterns. Not patentable.

---

### 2.2 Algorithm 2: Multi-Criteria Safety Gate Action Interlock

- **Source Location:** `vira_interaction/core/safety_gate.py`
- **Execution Context:** Synchronous action filter before emitting to `EventBus`

#### Mathematical & Algorithmic Formulation

Let $A = (type, params, t_{req}) \in \mathcal{A}$ be a candidate action. The safety gate evaluates five binary invariants:

$$\text{Gate}(A) = \bigwedge_{j=1}^5 I_j(A)$$

1. **E-Stop Latch Invariant ($I_1$):**
   $$I_1(A) = \neg \text{IsEStopLatched}() \lor (A.type = \text{"RESET\_ESTOP"})$$
2. **Forbidden Phrase / Command Invariant ($I_2$):**
   $$I_2(A) = \forall w \in \mathcal{W}_{forbidden}, w \notin A.params$$
3. **Temporal Rate Limit Invariant ($I_3$):**
   $$I_3(A) = (t_{req} - t_{last\_cmd} \ge \Delta t_{min}) \lor (A.priority < P_{normal})$$
4. **Spatial Boundary Invariant ($I_4$):**
   $$I_4(A) = (A.type \ne \text{"NAVIGATE"}) \lor \mathbf{p}_{target} \in \Omega_{safe}$$
5. **Battery / System Health Invariant ($I_5$):**
   $$I_5(A) = (A.type \ne \text{"NAVIGATE"}) \lor (V_{batt} \ge V_{critical} \land T_{core} \le T_{max})$$

If $\text{Gate}(A) = \text{True}$, $A$ is promoted to an `ActionRequested` event; otherwise, an `ActionRejected` event is emitted.

- **Technical Assessment:** **Standard Engineering.** Rule-based gatekeeper pattern.

---

### 2.3 Algorithm 3: Independent Dual-Threshold Supervisory Trajectory Watchdog

- **Source Location:** `ros2_ws/src/gracemo_nav2/gracemo_nav2/coordinator_node.py` (`_check_navigation_watchdog()`)
- **Execution Context:** 10 Hz periodic ROS 2 timer thread
- **Significance:** Developed during the November 2026 remediation to decouple mission supervision from Nav2 internal feedback and solve the stationary robot / advancing RViz defect.

#### Mathematical & Algorithmic Formulation

Let $\mathbf{p}(t) = (x(t), y(t))^T \in \mathbb{R}^2$ be the estimated robot pose in the fixed `map` coordinate frame obtained via TF lookup (`map` $\to$ `base_footprint`).  
Let $\mathbf{p}_{prog} = (x_{prog}, y_{prog})^T$ be the pose recorded at the last confirmed physical displacement event, and $t_{prog}$ be the timestamp of that event.

At every timer tick $t_k$:

1. **Calculate Euclidean Displacement:**
   $$\Delta d(t_k) = \|\mathbf{p}(t_k) - \mathbf{p}_{prog}\|_2 = \sqrt{(x(t_k) - x_{prog})^2 + (y(t_k) - y_{prog})^2}$$

2. **Evaluate Progress Invariant:**
   $$\text{HasProgressed}(t_k) = \begin{cases}
   \text{True} & \text{if } \Delta d(t_k) \ge \epsilon_{disp} \\
   \text{False} & \text{if } \Delta d(t_k) < \epsilon_{disp}
   \end{cases}$$
   where $\epsilon_{disp} = 0.08\text{ meters}$.

3. **State & Timer Update:**
   $$\text{If } \text{HasProgressed}(t_k): \quad \mathbf{p}_{prog} \leftarrow \mathbf{p}(t_k), \quad t_{prog} \leftarrow t_k$$

4. **Stagnation & Total Mission Timeout Evaluation:**
   $$\Delta t_{stagnant} = t_k - t_{prog}, \quad \Delta t_{total} = t_k - t_{start}$$
   $$\text{TripWatchdog}(t_k) = (\Delta t_{stagnant} > \tau_{stagnant}) \lor (\Delta t_{total} > \tau_{total})$$
   where $\tau_{stagnant} = 14.0\text{ seconds}$ and $\tau_{total} = 180.0\text{ seconds}$.

5. **Recovery Execution on Trip:**
   $$\text{If } \text{TripWatchdog}(t_k) = \text{True}:$$
   - Send `cancel_goal_async()` to Nav2 action server.
   - Increment retry counter: $r \leftarrow r + 1$.
   - If $r \le r_{max}$ ($r_{max} = 3$):
     - Transition state: $S \leftarrow \text{STATE\_OBSTACLE\_HANDLING}$.
     - Asynchronously invoke `/global_costmap/clear_entirely_global_costmap` and `/local_costmap/clear_entirely_local_costmap`.
     - Schedule re-dispatch timer: $t_{retry} = t_k + 1.5\text{ s}$.
   - Else ($r > r_{max}$):
     - Transition state: $S \leftarrow \text{STATE\_IDLE}$.
     - Finalize mission with `success = False`, reason = `"Retries exhausted (watchdog_stagnant)"`.

```python
# Algorithmic Pseudocode: Trajectory Watchdog
def check_navigation_watchdog(self):
    if self.state != STATE_MISSION_EXECUTING or not self.active_goal_handle:
        return
        
    now = self.get_clock().now()
    cur_x, cur_y, _ = self.get_current_map_pose()
    
    # Calculate Euclidean displacement from last verified progress pose
    disp = math.hypot(cur_x - self.last_progress_pose[0], cur_y - self.last_progress_pose[1])
    
    # Dual-threshold progress verification
    if disp >= 0.08:
        self.last_progress_pose = (cur_x, cur_y)
        self.last_progress_time = now
        return
        
    stagnant_sec = (now - self.last_progress_time).nanoseconds / 1e9
    total_sec = (now - self.navigation_start_time).nanoseconds / 1e9
    
    if stagnant_sec > 14.0 or total_sec > 180.0:
        self.abort_active_navigation_goal()
        self.handle_navigation_failure(f"watchdog_stagnant_{stagnant_sec:.1f}s")
```

- **Technical Assessment:** Highly robust engineering remediation that successfully eliminated feedback-loop resets. However, displacement-over-time watchdogging is standard in robotics (Nav2 `SimpleProgressChecker`, iRobot US 2013/0310978 A1). Needs the kinematic consistency formulation in Section 3 to become patentable.

---

### 2.4 Algorithm 4: Serialized 5-Stage Safety Invariant Velocity Pipeline

- **Source Locations:**
  - Stage 1: `ros2_ws/src/gracemo_nav2/gracemo_nav2/cmd_vel_mux_node.py`
  - Stage 2: `nav2_velocity_smoother` (`nav2_params.yaml`)
  - Stage 3: `nav2_collision_monitor` (`nav2_params.yaml`)
  - Stage 4: `ros2_ws/src/gracemo_nav2/gracemo_nav2/terrain_monitor_node.py`
  - Stage 5: `ros2_ws/src/gracemo_hardware/gracemo_hardware/mcu_transport.py`

#### Mathematical Pipeline Flow

$$\mathbf{v}_{nav} \xrightarrow{\text{Stage 1: Priority Mux}} \mathbf{v}_{mux} \xrightarrow{\text{Stage 2: Smoother}} \mathbf{v}_{smooth} \xrightarrow{\text{Stage 3: Collision}} \mathbf{v}_{safe} \xrightarrow{\text{Stage 4: Terrain}} \mathbf{v}_{final} \xrightarrow{\text{Stage 5: MCU Lock}} \text{Motors}$$

#### Detailed Stage Transformations:

1. **Stage 1 (Priority Multiplexing):**
   $$\mathbf{v}_{mux}(t) = \begin{cases} 
   \mathbf{v}_{teleop}(t) & \text{if } (t - t_{teleop} \le 0.5\text{ s}) \\
   \mathbf{v}_{nav}(t) & \text{else if } (t - t_{nav} \le 0.5\text{ s}) \\
   \mathbf{0} & \text{otherwise (timeout)}
   \end{cases}$$

2. **Stage 2 (Dynamic Velocity Smoothing):**
   $$\mathbf{v}_{smooth}(t) = \mathbf{v}_{smooth}(t - \Delta t) + \text{clamp}\left(\mathbf{v}_{mux}(t) - \mathbf{v}_{smooth}(t - \Delta t), -\mathbf{a}_{max}\Delta t, +\mathbf{a}_{max}\Delta t\right)$$
   subject to linear acceleration limit $a_{x,max} = 1.0\text{ m/s}^2$ and angular acceleration limit $\alpha_{z,max} = 1.5\text{ rad/s}^2$.

3. **Stage 3 (Geometric Collision Scaling):**
   Given point cloud / scan points $\mathcal{P}_{scan} \subset \mathbb{R}^2$ and polygonal zones $\Omega_{stop} \subset \Omega_{slowdown}$:
   $$\mathbf{v}_{safe}(t) = \begin{cases}
   \mathbf{0} & \text{if } \exists \mathbf{p} \in \mathcal{P}_{scan} \text{ s.t. } \mathbf{p} \in \Omega_{stop} \\
   \kappa_{scale} \cdot \mathbf{v}_{smooth}(t) & \text{else if } \exists \mathbf{p} \in \mathcal{P}_{scan} \text{ s.t. } \mathbf{p} \in \Omega_{slowdown} \quad (\kappa_{scale} = 0.3) \\
   \mathbf{v}_{smooth}(t) & \text{otherwise}
   \end{cases}$$

4. **Stage 4 (Inertial Terrain Envelope & Waist Counterbalance):**
   Given IMU attitude quaternion $\mathbf{q} = (w, x, y, z)$, compute roll $\phi$ and pitch $\theta$:
   $$\phi = \text{atan2}(2(wx + yz), 1 - 2(x^2 + y^2)), \quad \theta = \arcsin(2(wy - zx))$$
   $$\mathbf{v}_{final}(t) = \begin{cases}
   \mathbf{0} \quad (\text{E-STOP Latch}) & \text{if } |\phi| > 25^\circ \lor |\theta| > 20^\circ \\
   \left(1 - \frac{\max(|\phi| - 15^\circ, |\theta| - 12^\circ)}{10^\circ}\right) \cdot \mathbf{v}_{safe}(t) & \text{else if } |\phi| > 15^\circ \lor |\theta| > 12^\circ \\
   \mathbf{v}_{safe}(t) & \text{otherwise}
   \end{cases}$$
   Simultaneously, active waist pitch servo angle $\psi_{waist}$ is commanded:
   $$\psi_{waist}(t) = -k_{comp} \cdot \theta(t) \quad (k_{comp} = 0.85)$$

5. **Stage 5 (Hardware Safety Gate):**
   $$\mathbf{v}_{actuator}(t) = \begin{cases}
   \mathbf{v}_{final}(t) & \text{if } \text{MOTOR\_OUTPUT\_ENABLED} = 1 \land \text{PROTOCOL\_CONFIRMED} = \text{True} \\
   \mathbf{0} & \text{strictly enforced in simulation}
   \end{cases}$$

- **Technical Assessment:** Highly robust safety cascade conforming to ISO 3691-4 industrial standards. Individual stages use standard components; the system architecture as a whole is clean engineering.

---

### 2.5 Algorithm 5: AMCL Covariance-Bounded Mission Health Guard

- **Source Location:** `ros2_ws/src/gracemo_nav2/gracemo_nav2/coordinator_node.py` (`_on_amcl_pose()`)
- **Execution Context:** Ingests `/amcl_pose` (`geometry_msgs/PoseWithCovarianceStamped`) at $\approx 5\text{ Hz}$

#### Mathematical & Algorithmic Formulation

Given covariance matrix $\mathbf{\Sigma}_{AMCL} \in \mathbb{R}^{6 \times 6}$, extract position variance diagonal elements:
$$\sigma_{xx}^2 = \mathbf{\Sigma}_{AMCL}[0, 0], \quad \sigma_{yy}^2 = \mathbf{\Sigma}_{AMCL}[1, 1]$$

$$\text{LocalizationHealth}(t) = \begin{cases}
\text{HEALTHY} & \text{if } \sigma_{xx}^2 \le \sigma_{thresh}^2 \land \sigma_{yy}^2 \le \sigma_{thresh}^2 \\
\text{DEGRADED} & \text{if } \sigma_{xx}^2 > \sigma_{thresh}^2 \lor \sigma_{yy}^2 > \sigma_{thresh}^2
\end{cases}$$
where $\sigma_{thresh}^2 = 2.5\text{ m}^2$.

When state becomes `DEGRADED`:
1. If state is `STATE_MISSION_EXECUTING`:
   - Transition to `STATE_LOCALIZATION_LOST`.
   - Cancel active Nav2 goal.
   - Asynchronously invoke `/request_nomotion_update` service to force AMCL update cycle while stationary.
   - Enforce map bounding invariant: $\mathbf{p} \in [-10.0, 10.0]^2$.

- **Technical Assessment:** Standard probabilistic robotics practice (Thrun et al., 2005).

---

## 3. The Proposed Original Algorithmic Leap

To elevate GRaCEmo ViRa from standard engineering into a globally patentable invention, we formulate two original algorithms that resolve the fundamental vulnerability of current mobile robots: **undetected wheel slip, ghost odometry, and feedback-loop deception**.

### 3.1 Novel Algorithm A: Real-Time Multi-Sensor Kinematic Consistency Index ($\Gamma$)

#### The Technical Problem
In both Gazebo simulation and real physical robots, drive wheels slip when the robot hits an unmapped obstacle, a curb, or low-friction surfaces. Wheel encoders report rotation $\dot{\theta}_{wheel}$, integrating into advancing odometry. AMCL and local planners assume the robot is moving. Downstream progress checkers fail to trip if they compare odometry against odometry.

#### Mathematical Formulation

Define three independent velocity estimates for the robot base:
1. **Wheel Encoder Kinematic Velocity ($\mathbf{v}_{enc}$):**
   $$\mathbf{v}_{enc}(t) = \begin{pmatrix} \frac{r}{2}(\omega_R + \omega_L) \\ 0 \end{pmatrix}$$
2. **Inertial Measurement Integrated Velocity ($\mathbf{v}_{IMU}$):**
   $$\mathbf{v}_{IMU}(t) = \mathbf{v}_{IMU}(t - \Delta t) + \left(\mathbf{R}_{body}^{world}(t) \mathbf{a}_{IMU}(t) - \mathbf{g}\right) \Delta t$$
   filtered through a high-pass zero-velocity update (ZUPT) filter.
3. **LiDAR Scan-to-Scan Match Velocity ($\mathbf{v}_{scan}$):**
   $$\mathbf{v}_{scan}(t) = \frac{\Delta \mathbf{T}_{ICP}(\mathcal{P}_{k-1}, \mathcal{P}_k)}{\Delta t}$$

Define the **Kinematic Consistency Ratio ($\Gamma_{kinematic}$)**:

$$\Gamma_{kinematic}(t) = \frac{\min\left(\|\mathbf{v}_{scan}(t)\|_2, \|\mathbf{v}_{IMU}(t)\|_2\right) + \delta}{\|\mathbf{v}_{enc}(t)\|_2 + \delta}$$
where $\delta = 10^{-4}$ prevents division by zero during standstill.

**Physical Invariant:**
- For ideal non-slipping rolling contact: $\Gamma_{kinematic} \approx 1.0$.
- For drive wheel slip (wheels spinning, robot stationary): $\|\mathbf{v}_{enc}\| \gg 0$, while $\|\mathbf{v}_{scan}\| \approx 0 \implies \Gamma_{kinematic} \to 0.0$.
- For external towing / pushing (passive motion): $\|\mathbf{v}_{enc}\| \approx 0$, while $\|\mathbf{v}_{scan}\| > 0 \implies \Gamma_{kinematic} \gg 1.0$.

---

### 3.2 Novel Algorithm B: Autonomous Mission Integrity Metric ($\mathcal{I}_{mission}$)

#### Mathematical Formulation

We synthesize kinematic consistency, localization certainty, and terrain stability into a scalar **Mission Integrity Index ($\mathcal{I} \in [0, 1]$)**:

$$\mathcal{I}_{mission}(t) = w_1 \cdot \text{sigmoid}\left(k_\Gamma (\Gamma_{kinematic}(t) - \gamma_{slip})\right) + w_2 \cdot \exp\left(-\frac{\sigma_{xx}^2 + \sigma_{yy}^2}{2 \sigma_0^2}\right) + w_3 \cdot \cos(\phi(t)) \cos(\theta(t))$$

where:
- $w_1 = 0.50$ (Kinematic weight)
- $w_2 = 0.30$ (Localization covariance weight)
- $w_3 = 0.20$ (Terrain attitude weight)
- $w_1 + w_2 + w_3 = 1.0$
- $\gamma_{slip} = 0.40$ (Slip threshold)
- $\sigma_0^2 = 1.0\text{ m}^2$ (Nominal localization variance)

#### Integrity-Triggered Mission Arbitration Protocol

$$\text{SupervisorAction}(t) = \begin{cases}
\text{CONTINUE\_MISSION} & \text{if } \mathcal{I}_{mission}(t) \ge \tau_{nominal} \quad (0.75) \\
\text{THROTTLE\_AND\_LOG} & \text{if } \tau_{warn} \le \mathcal{I}_{mission}(t) < \tau_{nominal} \quad (0.50 \le \mathcal{I} < 0.75) \\
\text{ATOMIC\_MISSION\_HALT} & \text{if } \mathcal{I}_{mission}(t) < \tau_{warn} \quad (< 0.50)
\end{cases}$$

Upon `ATOMIC_MISSION_HALT`:
1. High-level mission state is instantly checkpointed (waypoint index, remaining sub-goal segment, orientation offset).
2. Command Mux is commanded to latch zero velocity ($v_x = 0, \omega_z = 0$).
3. The supervisor classifies the root cause:
   - If $\Gamma_{kinematic} < \gamma_{slip}$: Classify as **WHEEL\_SLIP\_OBSTACLE\_STALL**. Initiate costmap clear, back up 0.15 m, replan.
   - If $\sigma^2 > \sigma_0^2$: Classify as **LOCALIZATION\_DIVERGENCE**. Request AMCL nomotion update, rotate $30^\circ$ in place.
   - If tilt $> \theta_{crit}$: Classify as **TERRAIN\_TIPPING\_HAZARD**. Reverse along entry trajectory.

---

## 4. Algorithmic Comparison Matrix

| Algorithmic Dimension | Standard Nav2 / ROS 2 Practice | GRaCEmo ViRa Current Code | Proposed Original Algorithm ($\Gamma + \mathcal{I}$) |
|---|---|---|---|
| **Stuck Robot Detection** | Nav2 `SimpleProgressChecker` (evaluates controller frame distance) | External supervisor watchdog (`disp >= 0.08m / 14s` in map frame) | Dynamic multi-sensor kinematic consistency ($\Gamma_{kinematic}$) fusing LiDAR, IMU, and encoders |
| **Feedback Loop Coupling** | Controller server and progress checker tightly coupled | Decoupled; watchdog runs on independent timer | Decoupled; mission integrity index governs state transitions |
| **Slip / Ghost Motion** | Cannot detect wheel slip if odometry integrates wheel rotations | Detected empirically via Gazebo vs odom divergence; prevented by parameter tuning | Mathematically detected in real time ($\Gamma < 0.4$) regardless of surface friction |
| **Voice Interruption** | Basic action cancel / goal resend | Checkpointed waypoint index with safe resume conditions | Checkpointed waypoint with integrity-verified safe resume envelope |
| **Safety Pipeline** | Disjoint nodes (`twist_mux`, `collision_monitor`) | Serialized 5-stage cascade with tilt envelope & waist balancing | Multi-invariant arbitration matrix with root-cause diagnostic feedback |
| **Patent Eligibility** | None (Standard OSS) | Low (Engineering assembly of known blocks) | **High (Defensible novel cyber-physical control method)** |

