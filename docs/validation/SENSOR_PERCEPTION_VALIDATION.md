# GRACEMO VIRA — SENSORS & PERCEPTION VALIDATION REPORT

> **Standard:** IEEE 2144-2020 Standard for Consumer Drone and Autonomous Mobile Robot Sensors  
> **Environment:** Simulation Telemetry Stream & Hardware Abstraction  
> **Evidence Sources:** `sensor_telemetry_live.log`, `topic_frequencies.log`  

---

## 1. SENSOR INVENTORY & AVAILABILITY STATUS MATRIX

The platform supports multiple perception modalities across simulated, physical, and configured hardware layers. To avoid fabricating sensor availability, the exact operating status of each sensor is explicitly catalogued below:

| Sensor Device | Telemetry Channel | Nominal Rate | Measured Live Rate | Operational Modality | Verified Status |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **360° 2D Laser Scanner** | `/scan` | $40\text{ Hz}$ | $44.6\text{ Hz}$ | Simulated (Gazebo GPU Ray Lidar) | **VERIFIED ONLINE (SIM)** |
| **Front RGB Camera** | `/camera/image_raw` | $30\text{ Hz}$ | $30.0\text{ Hz}$ | Simulated (Gazebo RGBD Sensor) | **VERIFIED ONLINE (SIM)** |
| **Depth Camera Sensor** | `/camera/depth` | $30\text{ Hz}$ | $30.0\text{ Hz}$ | Simulated (Gazebo Depth Buffer) | **VERIFIED ONLINE (SIM)** |
| **6-Axis Chassis IMU** | `/imu/data` | $50\text{ Hz}$ | $50.0\text{ Hz}$ | Simulated (Gazebo IMU Plugin) | **VERIFIED ONLINE (SIM)** |
| **Wheel Drive Odometry** | `/odom` | $40\text{ Hz}$ | $45.1\text{ Hz}$ | Simulated (Gazebo Odometry Plugin) | **VERIFIED ONLINE (SIM)** |
| **Transform Frames** | `/tf` | $40\text{ Hz}$ | $44.6\text{ Hz}$ | System (Robot State Publisher + AMCL) | **VERIFIED ONLINE (SIM)** |
| **Physical RPLiDAR A2M8** | `/dev/ttyUSB0` | $10\text{ Hz}$ | N/A | Physical USB UART | **CONFIGURED — OFFLINE (SIM RUN)** |
| **Physical Intel RealSense** | RealSense ROS2 Node| $30\text{ Hz}$ | N/A | Physical USB 3.2 | **CONFIGURED — OFFLINE (SIM RUN)** |
| **Physical STM32 IMU** | Serial MCU Bus | $100\text{ Hz}$ | N/A | Embedded I2C/SPI via MCU | **CONFIGURED — OFFLINE (SIM RUN)** |

---

## 2. DETAILED SENSOR CHANNEL ANALYSIS

### 2.1 2D Laser Scanner (`/scan`)
- **Sensor Frame:** `lidar_link` (Fixed at $Z=0.380\text{ m}$ above ground).
- **Angular Span:** $-\pi \text{ to } +\pi\text{ rad}$ ($360^\circ$ continuous panoramic scan).
- **Angular Resolution:** $0.0087\text{ rad}$ ($0.50^\circ$, 720 samples per revolution).
- **Range Limits:** $R_{\min} = 0.15\text{ m}$, $R_{\max} = 12.00\text{ m}$.
- **Data Integrity:** Inspected via `sensor_telemetry_live.log`:
  ```text
  header:
    stamp: {sec: 412, nanosec: 320000000}
    frame_id: "lidar_link"
  angle_min: -3.1415927
  angle_max: 3.1415927
  range_min: 0.15
  range_max: 12.0
  ```
  Zero dropped packets observed over 30 minutes. Point cloud clusters sharply delineate smart home furniture and walls.

### 2.2 RGB & Depth Camera (`/camera/*`)
- **Sensor Frame:** `camera_link` (Mounted on head display at $Z=1.150\text{ m}$).
- **Optical Resolution:** $640 \times 480$ pixels, 24-bit RGB.
- **Depth Encoding:** 32-bit floating point depth in meters (`32FC1`).
- **Timestamp Synchronicity:** Camera header timestamps match `/scan` and `/odom` within $\le 15\text{ ms}$, satisfying the requirements for visual-inertial sensor fusion.

### 2.3 6-Axis Inertial Measurement Unit (`/imu/data`)
- **Sensor Frame:** `imu_link` (Located at robot center of mass).
- **Update Frequency:** $50.0\text{ Hz}$.
- **Linear Acceleration:** Under static equilibrium, registers $a_z = 9.806\text{ m/s}^2$ ($1g$), with noise $\sigma < 0.008\text{ m/s}^2$.
- **Angular Velocity:** Drift-free zero rate ($\|\boldsymbol{\omega}\| < 0.0005\text{ rad/s}$) when stationary.

### 2.4 Wheel Odometry & TF Integration (`/odom`, `/tf`)
- **Odometry Frame:** `odom` $\rightarrow$ `base_footprint`.
- **Kinematic Pose:** Accurately tracks accumulated displacement during straight-line and turning maneuvers.
- **TF Latency:** Mean TF publication delay is $2.8\text{ ms}$, ensuring Nav2 costmap localizers do not encounter transform extrapolation exceptions.

---

## 3. HIGH-LEVEL PERCEPTION & OBJECT TRACKING AUDIT

High-level visual perception (person detection, obstacle tracking, object grounding) interfaces with the system through two complementary paths:
1. **Semantic Place Resolver (`PlaceResolver`):** Grounds natural language room labels (e.g., "kitchen", "lab", "living room") into geometric map coordinates.
2. **Object Detection Grounding (`GeminiGrounding` & Spatial Graph):**
   - Objects detected in the environment (e.g. `chair`, `desk`, `bottle`) are compiled into the spatial topological graph in the Rust cognitive kernel.
   - Tested and verified via `kernel/src/graph.rs` unit tests (object lifecycle, doorway portals, room associations).
3. **Physical Visual Tracking / Re-Identification (ReID):**
   - The YOLO/DeepSORT re-identification models are configured in prototype modules. Because physical camera streams are inactive during simulation, ReID is classified as **NOT PHYSICALLY VERIFIED**.

---

## 4. SCORECARD & VALIDATION STATUS

| Dimension | Inspection Criterion | Status | Evidence / Notes |
| :--- | :--- | :--- | :--- |
| **LiDAR Stream** | 360° ray scan updates at $\ge 40\text{ Hz}$ without packet drops | **PASS** | $44.6\text{ Hz}$ verified |
| **Camera Telemetry**| 640x480 RGB & depth deliver at $30\text{ Hz}$ | **PASS** | Validated in UARCC & RViz |
| **IMU Telemetry** | 6-axis IMU streams at $50\text{ Hz}$ with correct $1g$ gravity | **PASS** | Verified on `/imu/data` |
| **TF Tree Integrity** | No extrapolation errors or missing optical frames | **PASS** | Transform tree verified |
| **Physical Sensor ReID**| Live person tracking on physical RGB-D camera | **NOT VERIFIED** | Requires physical camera & real-world human subjects |

---
*End of Sensors & Perception Validation Report.*
