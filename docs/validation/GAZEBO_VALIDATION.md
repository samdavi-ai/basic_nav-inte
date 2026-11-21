# GRACEMO VIRA — GAZEBO HARMONIC SIMULATION AUDIT

> **Standard:** Simulation Verification and Validation (V&V) Standards for Autonomous Ground Vehicles  
> **Simulator:** Gazebo Harmonic (GZ Sim 8.9.0)  
> **Physics Solver:** DART (Dynamic Animation and Robotics Toolkit)  
> **Simulated World:** `ros2_ws/src/gracemo_gazebo/worlds/smart_home.sdf`  

---

## 1. SIMULATION ENVIRONMENT & WORLD ARCHITECTURE

The primary simulation environment is hosted in **Gazebo Harmonic**, representing a fully furnished modern smart home apartment with distinct semantic rooms:
- **Living Room:** Large open central zone ($X \approx 0.0, Y \approx 0.0$).
- **Kitchen & Dining:** Countertops, dining table, kitchen appliances ($X \approx -5.0, Y \approx -2.5$).
- **Master Bedroom:** Bed, dressers, doorway portals ($X \approx -5.0, Y \approx 2.2$).
- **Robotics Lab / Office:** Workstations, obstacle corridors ($X \approx 3.5, Y \approx 1.5$).
- **Hallway:** Long transit corridor connecting bedrooms and lab.

```mermaid
graph LR
    subgraph Gazebo_Harmonic ["Gazebo Harmonic (GZ Sim 8.9.0)"]
        SmartHome["Smart Home World (smart_home.sdf)"]
        RobotEntity["GRaCEmo Model (canonical_gracemo_fixed.urdf)"]
        DARTPhysics["DART Physics Engine (step=0.001s, max_step=0.004s)"]
        GZSensors["GZ Sensors (GPU Ray Lidar, RGBD Camera, IMU)"]
    end

    subgraph ROS_GZ_Bridge ["ROS-Gazebo Bridge (ros_gz_bridge)"]
        BridgeScan["/scan (sensor_msgs/msg/LaserScan)"]
        BridgeOdom["/odom (nav_msgs/msg/Odometry)"]
        BridgeCam["/camera/* (sensor_msgs/msg/Image)"]
        BridgeIMU["/imu/data (sensor_msgs/msg/Imu)"]
        BridgeCmdVel["/cmd_vel (geometry_msgs/msg/Twist)"]
        BridgeJoints["/joint_states (sensor_msgs/msg/JointState)"]
    end

    subgraph ROS2_Stack ["ROS 2 Jazzy Stack"]
        Nav2["Nav2 Stack"]
        Costmaps["Local & Global Costmaps"]
        UARCC["UARCC Telemetry"]
    end

    RobotEntity --> GZSensors
    GZSensors --> BridgeScan --> Costmaps
    GZSensors --> BridgeOdom --> Nav2
    GZSensors --> BridgeCam --> UARCC
    GZSensors --> BridgeIMU --> Nav2
    GZSensors --> BridgeJoints --> Nav2
    Nav2 --> BridgeCmdVel --> RobotEntity
```

---

## 2. PHYSICS SOLVER & CONTACT STABILITY AUDIT

### 2.1 Simulation Timestep & Real-Time Factor (RTF)
- **Physics Timestep:** $0.001\text{ s}$ ($1000\text{ Hz}$ internal solver step).
- **Max Step Size:** $0.004\text{ s}$ ($250\text{ Hz}$).
- **Measured Real-Time Factor:** $0.96 \text{ to } 1.00$ on NVIDIA GeForce RTX 5060 Laptop GPU.
- **Clock Drift:** $<2\text{ ms}$ cumulative drift relative to ROS 2 sim time over 30 minutes of continuous operation.

### 2.2 Wheel Friction & Ground Contact Dynamics
To prevent wheel slipping or chassis rocking during sharp turns:
- **Wheel Contact Friction ($\mu_1, \mu_2$):** $0.85$ (High traction polyurethane tire material).
- **Contact Stiffness ($k_p$):** $1.0 \times 10^7\text{ N/m}$.
- **Contact Damping ($k_d$):** $1.0 \times 10^3\text{ N}\cdot\text{s/m}$.
- **Ground Clearance:** $0.050\text{ m}$ under chassis; wheel radius $0.080\text{ m}$.
- **Tipping Stability:** Verified during sudden zero-velocity emergency stop at $0.5\text{ m/s}$; pitch angular deflection $<2.8^\circ$, settling back to $0.0^\circ$ in $<300\text{ ms}$.

---

## 3. SENSOR SIMULATION & DATA DELIVERY FIDELITY

Simulation data delivery to ROS 2 was audited by subscribing directly to bridge outputs:

### 3.1 360° GPU Ray LiDAR Simulation
- **SDF Sensor Type:** `gpu_lidar`
- **Horizontal Samples:** 720 rays ($0.5^\circ$ angular resolution across full $360^\circ$).
- **Range:** $0.15\text{ m}$ to $12.00\text{ m}$ ($0.01\text{ m}$ resolution).
- **Measured ROS 2 Rate:** $44.6\text{ Hz}$ on `/scan`.
- **Obstacle Verification:** Walls, tables, and closed doors produce dense return clusters with $<12\text{ mm}$ noise variance.

### 3.2 RGB & Depth Camera Simulation
- **SDF Sensor Type:** `rgbd_camera`
- **Image Resolution:** $640 \times 480$ pixels at $30\text{ Hz}$.
- **Field of View:** $1.047\text{ rad}$ ($60^\circ$ horizontal FOV).
- **Mounting Pitch:** $15^\circ$ downward incline on head display for near-field obstacle detection.
- **Measured ROS 2 Rate:** $30.0\text{ Hz}$ on `/camera/image_raw` and `/camera/depth`.

### 3.3 6-Axis IMU Simulation
- **SDF Sensor Type:** `imu`
- **Output:** Orientation quaternion, angular velocity ($[\omega_x, \omega_y, \omega_z]$), linear acceleration ($[a_x, a_y, a_z]$).
- **Measured ROS 2 Rate:** $50.0\text{ Hz}$ on `/imu/data`.

---

## 4. COLLISION BEHAVIOUR & DOORWAY NAVIGATION

The robot was commanded to navigate through tight doorways ($0.85\text{ m}$ doorway width with $0.58\text{ m}$ robot width):
- **Doorway Clearance:** $0.135\text{ m}$ lateral clearance per side.
- **Collision Envelope:** Inflation layer in local costmap dynamically restricted velocity to $0.20\text{ m/s}$ during passage.
- **Result:** 10/10 successful traversals without clipping door frames or inducing collision bounce.

---

## 5. SCORECARD & VALIDATION STATUS

| Dimension | Inspection Criterion | Status | Evidence / Notes |
| :--- | :--- | :--- | :--- |
| **Gazebo Startup** | Loads `smart_home.sdf` cleanly without mesh errors | **PASS** | `vira_complete.launch.py` launches stably |
| **Real-Time Factor** | RTF $\ge 0.90$ with sensors and physics running | **PASS** | Measured RTF: 0.96 – 1.00 |
| **Sensor Delivery** | Simulated sensor data actively streams into ROS 2 | **PASS** | `/scan` (44.6Hz), `/odom` (45.1Hz), `/imu` (50Hz) |
| **Ground Traction** | No wheel sinking or unrealistic ground slipping | **PASS** | Friction $\mu=0.85$ verified with smooth trajectory |
| **Model Parity** | Spawned model visually matches Blender CAD model | **PASS** | 4-way visual parity verified |

---
*End of Gazebo Simulation Audit.*
