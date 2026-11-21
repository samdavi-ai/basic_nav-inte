# GRaCEmo ViRa — Deep Prior-Art Patent & Literature Comparison

**Project:** GRaCEmo ViRa Autonomous Humanoid Service Robot  
**Auditor:** Principal Robotics Research Scientist & Patent Prior-Art Research Analyst  
**Databases Searched:** Google Patents, WIPO PATENTSCOPE, Espacenet, Indian Patent Office (InPASS), IEEE Xplore, ACM Digital Library  
**Classification Codes Analyzed:** G05D 1/02 (Autonomous navigation), G05D 1/00 (Control systems for mobile vehicles), B25J 9/16 (Programme-controlled manipulators/robots), B25J 19/06 (Safety devices)

---

## 1. Executive Summary of Prior-Art Landscape

The field of autonomous mobile robot (AMR) mission management, interruption recovery, and stuck-robot detection is mature and heavily encumbered by patent portfolios held by major commercial entities, notably:
- **iRobot Corporation** (robotic floor care, physical stasis detection, obstacle extrication)
- **Brain Corporation / BrainOS** (commercial AMR autonomy, dynamic path re-planning, multi-sensor behavioral intervention)
- **Savioke, Inc. / Relay Robotics** (service robot task interruption, touchscreen/voice pause, intermediate waypoint routing)
- **Amazon Technologies / Kiva Systems** (warehouse AMR fleet coordination, exception handling, path reservation)

Furthermore, the **open-source robotics community (ROS 2, Nav2, MoveIt)** has disclosed and standardized many core patterns (action client goal preemption, waypoint following, costmap recovery, and velocity multiplexing) in the public domain. Under 35 U.S.C. § 102(a)(1) and Article 54(2) EPC, these public disclosures constitute prior art against any broad claims on mission state machines.

---

## 2. Deep Analysis of Primary Patent Prior Art

### 2.1 US Patent Application Publication US 2013/0310978 A1
- **Title:** *Detecting robot stasis*
- **Applicant / Assignee:** iRobot Corporation
- **Inventors:** Mario E. Munich, et al.
- **Publication Date:** November 21, 2013
- **Priority Date:** May 16, 2012
- **Legal Status:** Granted / Expired or In-Force across various family members
- **Core Disclosed Invention:**
  Discloses techniques for detecting when an autonomous mobile robot is experiencing "stasis" (i.e. drive wheels are receiving torque or spinning, but physical translational advancement is impeded). Uses an unpowered stasis indication wheel mounted on a spring-biased suspension with optical/reflective segments to measure true ground displacement, cross-referenced against drive motor current thresholds.
- **Comparison to GRaCEmo ViRa:**
  - *Similarity:* Both systems seek to detect the failure mode where drive commands are issued but the physical chassis remains stationary.
  - *Difference:* iRobot relies on a physical unpowered caster/follower wheel with optical encoder segments and motor current spikes. GRaCEmo ViRa's current code uses a software transform displacement check ($\Delta d \ge 0.08\text{ m}$ in `map` frame via TF).
  - *Patentability Impact on Current GRaCEmo:* High obviousness risk (§ 103). Using software TF transforms instead of an unpowered wheel encoder is a known substitute in modern LiDAR-equipped AMRs.
  - *Proposed Algorithmic Differentiator:* GRaCEmo's proposed Kinematic Consistency Index ($\Gamma_{kinematic}$) evaluates the mathematical ratio of LiDAR scan-matching velocity and IMU acceleration against wheel odometry, detecting wheel slip in contactless environments without dedicated mechanical stasis wheels.

---

### 2.2 US Patent US 10,001,780 B2
- **Title:** *Systems and Methods For Dynamic Route Planning In Autonomous Navigation*
- **Applicant / Assignee:** Brain Corporation
- **Inventors:** Eugene Izhikevich, et al.
- **Grant Date:** June 19, 2018 | **Priority Date:** October 28, 2015
- **Core Disclosed Invention:**
  Discloses systems for mobile robots where an autonomous navigation run along a pre-mapped route is interrupted by unexpected static or dynamic obstacles. The system monitors progress, halts or slows the vehicle, saves route progress coordinates, evaluates local costmaps, and dynamically computes a detour or replanned path before resuming the route.
- **Comparison to GRaCEmo ViRa:**
  - *Similarity:* Both systems interrupt an active navigation trajectory upon obstacle stagnation or user commands, preserve waypoint progress, clear/update local costmaps, and attempt replanning.
  - *Difference:* Brain Corp focuses heavily on learned/demonstrated route following and neural network cost evaluation. GRaCEmo uses a ROS 2 Nav2 DWB/Navfn stack with a serialized 5-stage safety mux and voice-based state preservation.
  - *Patentability Impact on Current GRaCEmo:* Prevents claiming the concept of "interrupting a navigation route upon obstacle detection and resuming from the interrupted location". That concept is fully anticipated.

---

### 2.3 US Patent US 11,268,816 B2
- **Title:** *Intermediate waypoint generator*
- **Applicant / Assignee:** Savioke, Inc. (Relay Robotics)
- **Inventors:** Tessa Lau, Adrian Canoso, et al.
- **Grant Date:** March 8, 2022 | **Priority Date:** September 15, 2017
- **Core Disclosed Invention:**
  Discloses an autonomous service robot navigation system that receives high-level job assignments (e.g. delivery to a hotel room). If the primary path is blocked or the robot is paused via user interface, an intermediate waypoint generator computes intermediate navigation goals to navigate around obstacles or resume the multi-step job without re-homing.
- **Comparison to GRaCEmo ViRa:**
  - *Similarity:* High-level mission dispatch, waypoint queue tracking, user interruption (touchscreen in Savioke, voice in ViRa), and target re-dispatch from current pose.
  - *Difference:* Savioke focuses on topological corridor/room graph routing and elevator integration. GRaCEmo integrates voice NLU (`FastPathMatcher`) and an inertial roll/pitch terrain envelope.
  - *Patentability Impact on Current GRaCEmo:* Completely invalidates any attempt to patent "preserving the active waypoint index during a voice pause and resuming navigation to that waypoint".

---

### 2.4 US Patent US 9,533,413 B2
- **Title:** *Trainable modular robotic apparatus and methods*
- **Applicant / Assignee:** Brain Corporation
- **Inventors:** Oleg Sinyavskiy, et al.
- **Grant Date:** January 3, 2017 | **Priority Date:** June 21, 2013
- **Core Disclosed Invention:**
  Details robotic behavioral intervention based on multi-sensor exceptions, including loss of wheel traction, skidding, motor stall, and user interruption, triggering state transitions in a behavioral hierarchy.
- **Comparison to GRaCEmo ViRa:**
  - *Similarity:* Behavioral state machine with emergency stopping and exception escalation.
  - *Difference:* Brain Corp uses modular spiking neural network controllers; GRaCEmo uses ROS 2 node composition.

---

### 2.5 Open-Source Robotics Public Disclosures (ROS 2 & Nav2)

1. **`nav2_controller::SimpleProgressChecker` (ROS 2 Nav2 Documentation & Source Code):**
   - *Public Disclosure Date:* 2019–2020 (Open Source, Apache 2.0)
   - *Disclosed Mechanism:* Evaluates whether the robot pose has moved by `required_movement_radius` (default 0.5 m) within `movement_time_allowance` (default 10.0 s). If not, raises an error code to trigger BT recovery behaviors.
   - *Significance:* This is direct prior art against GRaCEmo's `_check_navigation_watchdog` ($\Delta d \ge 0.08\text{ m}$ within $14.0\text{ s}$). The concept of measuring Euclidean displacement over a floating time window is identical; GRaCEmo merely changed the numerical thresholds and placed it in an external supervisory node.

2. **`twist_mux` / `nav2_collision_monitor` / `nav2_velocity_smoother`:**
   - *Public Disclosure Date:* 2015–2022
   - *Disclosed Mechanism:* Priority multiplexing of velocity topics with watchdog timeouts; geometric polygon collision detection using 2D laser scans; acceleration-limited velocity profiling.
   - *Significance:* Discloses Stages 1, 2, and 3 of GRaCEmo's velocity safety chain.

---

## 3. Side-by-Side Claim Differentiation Matrix

| Technical Operation | Existing Commercial & OSS Approaches | GRaCEmo ViRa Current Code | Proposed Patentable Differentiation ($\Gamma + \mathcal{I}$) |
|---|---|---|---|
| **Stuck Robot / Stasis Detection** | **iRobot (US20130310978):** Unpowered stasis wheel with optical encoder + motor current.<br>**Nav2:** `SimpleProgressChecker` (controller-frame $\Delta d \ge 0.5\text{ m}$ / $10\text{ s}$). | External supervisor watchdog checking TF `map` frame displacement ($\Delta d \ge 0.08\text{ m}$ / $14\text{ s}$). Decoupled from feedback rate. | **Multi-Sensor Kinematic Consistency Index ($\Gamma_{kinematic}$):** Dynamic ratio of LiDAR scan-match velocity and IMU acceleration against wheel encoders. Detects wheel slip in real time ($\Gamma < 0.4$) without physical contact wheels. |
| **Mission Interruption by Human** | **Savioke (US11268816):** UI touchscreen pause button stops path execution and holds task queue.<br>**Brain Corp (US10001780):** Pause trigger halts vehicle and preserves route pointer. | Natural voice intent parsed via regex fast-path (`"pause navigation"`, `"stop"`). Transitions coordinator FSM to `STATE_MISSION_PAUSED`. | Voice-interrupted mission state preservation integrated with **Mission Integrity Metric ($\mathcal{I}_{mission}$)**: Resume is blocked unless kinematic consistency, AMCL covariance, and terrain attitude are simultaneously validated. |
| **Mission State Resumption** | **Standard Nav2 Waypoint Follower:** Re-sends target pose goal to action server.<br>**Savioke:** Computes intermediate segment to rejoin route. | Reads current TF pose, preserves remaining waypoint queue `self.waypoints[self.current_waypoint_idx:]`, sends new action goal. | Checkpoints multi-modal mission vector (waypoint ID, remaining segment length, localization covariance history) and dynamically adjusts velocity profile based on terrain pitch. |
| **Cross-Layer Safety Arbitration** | Disjoint safety modules: standalone `twist_mux`, standalone collision monitor. | Serialized 5-tier pipeline: Mux $\to$ Smoother $\to$ Collision $\to$ Terrain Monitor $\to$ MCU hardware lock. | Closed-loop cross-layer invariant monitoring: Any layer violation injects feedback into the Mission Integrity Metric ($\mathcal{I}_{mission}$), informing high-level cognitive recovery. |
| **Localization Degradation Handling** | AMCL standard error logging; manual relocalization via RViz `2D Pose Estimate`. | Monitors AMCL covariance diagonal ($\sigma_{xx}^2, \sigma_{yy}^2 > 2.5$). Aborts goal, requests `nomotion_update`, enforces map boundaries. | Dynamic uncertainty covariance weighting in $\mathcal{I}_{mission}$; triggers deterministic sensory sweep maneuvers if covariance fails to settle after $k$ nomotion updates. |

---

## 4. Assessment Under Patent Examination Standards

### 4.1 US Patent Law (35 U.S.C. §§ 101, 102, 103)
- **§ 101 (Subject Matter Eligibility):** Satisfied. The system operates on physical robotic actuators, LiDAR, and IMU data, providing a physical solution to kinematic stall.
- **§ 102 (Anticipation):** Current FSM and waypoint pausing are anticipated by Savioke (US 11,268,816) and Brain Corp (US 10,001,780).
- **§ 103 (Obviousness):** Current implementation would be rejected as an obvious combination of Nav2 `SimpleProgressChecker`, `twist_mux`, and regex voice parsing.
- **The Defensible Path:** Claiming the specific closed-loop coupling of the **Kinematic Consistency Index ($\Gamma$)** with **Autonomous Mission Integrity ($\mathcal{I}$)** for robotic mission arbitration creates clear non-obviousness over the cited art.

### 4.2 Indian Patents Act, 1970 (Section 3(k))
- **Statutory Hurdle:** Section 3(k) explicitly bars patenting of "computer programmes per se or algorithms".
- **Survival Strategy:** Under IPO CRI Guidelines, claims must strictly avoid pure software or method steps like "determining an intent" or "transitioning an FSM".
- **Required Claim Format:** Claims must be directed to:
  > *"A robotic navigation safety control apparatus comprising: a mobile robotic chassis having drive wheels and motor actuators; a LiDAR sensor, an inertial measurement unit (IMU), and wheel encoders; and a supervisory safety controller communicatively coupled to the motor actuators..."*
  The novel technical effect must be specified as:
  > *"preventing mechanical motor wear, false odometry advancement, and chassis tip-over by dynamically inhibiting motor torque upon detecting kinematic divergence between said LiDAR sensor and said wheel encoders."*

