# GRaCEmo ViRa — Autonomous Operation User Guide

**Document Version:** 1.0.0  
**Target Audience:** Operators, Roboticists, Simulation Engineers  

---

## 1. Quick Start Guide

### 1.1 Launching the Simulation Autonomy Stack
Open a terminal and start the integrated stack:

```bash
# 1. Start Rust MNSE Kernel (EventBus server)
cd /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration
./kernel/target/debug/gracemo-kernel

# 2. Launch Simulation, Nav2, Bridges & Pygame GraceEMO (Terminal 2)
source /opt/ros/jazzy/setup.bash
source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/setup.bash
export DISPLAY=:0
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch gracemo_gazebo vira_complete.launch.py mode:=localization

# 3. Launch the Authoritative Autonomous Supervisor (Terminal 3)
source /opt/ros/jazzy/setup.bash
source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 run gracemo_nav2 coordinator_node.py

# 4. Launch the UARCC Control Center GUI (Terminal 4)
source /opt/ros/jazzy/setup.bash
source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/setup.bash
export DISPLAY=:0
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/gracemo_control_center/main.py
```

---

## 2. Operating via UARCC Autonomous Operations Page

1. Open the UARCC GUI window.
2. In the left navigation sidebar under the **WORKSPACE** category, click on **Autonomous Operations** (indicated with a compass icon).
3. The page provides 6 primary sections:
   - **Subsystem Readiness Matrix:** Live green/red status indicators for Localization, TF Tree, Costmaps, Nav2 Lifecycle, and EventBus. All must show green before a mission can be dispatched.
   - **Live Robot Telemetry:** Displays current Pose $(X, Y, \text{Yaw})$, active Waypoint Target, Remaining Distance, Linear Speed, Battery Percentage, and the Authoritative FSM State badge.
   - **Interactive Mission Planner:**
     - Select a predefined destination (e.g., *Hallway*, *Master Bedroom*, *Kitchen*, *Living Room*, *Charging Base*), or input custom coordinates.
     - Set Dwell Time (seconds) to wait at the destination before completing or proceeding to the next waypoint.
     - Click **Start Mission** to dispatch the trajectory.
   - **Multi-Waypoint Patrol:**
     - Click **Patrol Sequence** to run a continuous multi-stop route with dwell delays.
   - **Control Buttons:**
     - **Pause Mission:** Halts motion in place while maintaining current waypoint target.
     - **Resume Mission:** Continues trajectory toward target.
     - **Cancel Mission:** Aborts current route and puts supervisor into Standby.
     - **Return Safe Base:** Automatically returns the robot to home coordinates $(0.0, 0.0)$.
   - **Safety Emergency Stop:** A high-contrast red toggle button. Clicking asserts emergency stop across all subsystems.
   - **Live 2D Map View:** Displays the 2D occupancy grid with costmap obstacles, robot footprint, and global trajectory.

---

## 3. Operating via Voice / Natural Language (GraceEMO)

You can issue voice or text commands directly through the `/interaction/command` topic or by speaking to GraceEMO:

| Voice Command / Utterance | Resulting Behavior |
| :--- | :--- |
| *"GraceEMO, navigate to the kitchen"* | Resolves semantic place `Kitchen` and dispatches Nav2 goal. Avatar displays `FOCUS`. |
| *"GraceEMO, pause mission"* | Pauses active mission trajectory; robot decelerates smoothly to stop. |
| *"GraceEMO, resume mission"* | Resumes navigation along planned path. |
| *"GraceEMO, return to base"* | Cancels current mission and routes robot to `(0.0, 0.0)`. |
| *"Stop!"* or *"Emergency Stop"* | Instantaneous emergency stop assertion; avatar shows `IDLE`/alert and confirms speech. |
| *"GraceEMO, what is your status?"* | Speaks current telemetry status and battery level. |

---

## 4. Operating via CLI

You can dispatch mission JSON payloads directly over the ROS 2 bus:

```bash
# Dispatch a single waypoint mission
ros2 topic pub --once /coordinator/command std_msgs/msg/String "data: '{\"intent\": \"START_MISSION\", \"mission_name\": \"Patrol Mission 1\", \"waypoints\": [{\"name\": \"Bedroom\", \"coords\": {\"x\": -5.0, \"y\": 2.2, \"yaw\": 0.0}, \"dwell_sec\": 3.0}]}'"

# Pause active mission
ros2 topic pub --once /coordinator/command std_msgs/msg/String "data: '{\"intent\": \"PAUSE\"}'"

# Resume active mission
ros2 topic pub --once /coordinator/command std_msgs/msg/String "data: '{\"intent\": \"RESUME\"}'"

# Return to base
ros2 topic pub --once /coordinator/command std_msgs/msg/String "data: '{\"intent\": \"RETURN_HOME\"}'"
```
