# GRaCEmo ViRa — Autonomous System Configuration Reference

**Document Version:** 1.0.0  
**Target Release:** GRaCEmo ViRa Unified Autonomous Simulation  

---

## 1. Kinematic & Geometric Parameters

| Parameter | Value | Reference File | Verification Status |
| :--- | :--- | :--- | :--- |
| **Track Width (Wheel Separation)** | `0.570 m` | `apartment_floor.world` & `nav2_params.yaml` | MATCHED |
| **Wheel Radius** | `0.115 m` | `apartment_floor.world` & `diff_drive` plugin | MATCHED |
| **Robot Footprint Radius** | `0.350 m` | `nav2_params.yaml` (circular footprint) | MATCHED |
| **Inflation Radius** | `0.550 m` | `nav2_params.yaml` (global & local costmaps) | MATCHED |
| **Cost Scaling Factor** | `3.0` | `nav2_params.yaml` (inflation layer) | MATCHED |

---

## 2. Velocity & Acceleration Limits

| Limit | Value | Pipeline Stage | Node |
| :--- | :--- | :--- | :--- |
| **Max Linear Velocity ($v_{\text{max}}$)** | `0.40 m/s` | Local Controller / Smoother | `dwb_core` / `velocity_smoother` |
| **Min Linear Velocity ($v_{\text{min}}$)** | `-0.05 m/s` | Local Controller (gentle reverse) | `dwb_core` |
| **Max Angular Velocity ($\omega_{\text{max}}$)** | `0.80 rad/s` | Local Controller / Smoother | `dwb_core` / `velocity_smoother` |
| **Max Linear Acceleration ($a_{\text{max}}$)** | `1.00 m/s²` | Velocity Smoother | `nav2_velocity_smoother` |
| **Max Linear Deceleration ($d_{\text{max}}$)** | `-1.00 m/s²` | Velocity Smoother | `nav2_velocity_smoother` |
| **Max Angular Acceleration ($\alpha_{\text{max}}$)** | `2.00 rad/s²` | Velocity Smoother | `nav2_velocity_smoother` |

---

## 3. Autonomous Supervisor Parameters

| Parameter | Configured Value | Default | Description |
| :--- | :--- | :--- | :--- |
| `arrival_tolerance_m` | `0.65 m` | `0.65 m` | Euclidean distance threshold for physical arrival verification |
| `stagnation_timeout_sec` | `14.0 s` | `14.0 s` | Maximum duration without progress before obstacle recovery triggers |
| `max_recovery_retries` | `2` | `2` | Number of automatic costmap clearing and replanning attempts |
| `telemetry_rate_hz` | `2.0 Hz` | `2.0 Hz` | Periodic publication rate of `/autonomy/state` and `/autonomy/mission_status` |
| `kernel_url` | `http://127.0.0.1:7780` | `http://127.0.0.1:7780` | Rust MNSE EventBus REST endpoint |

---

## 4. Safety Monitor Thresholds

| Sensor / Safety Check | Threshold Limit | Action Upon Violation |
| :--- | :--- | :--- |
| **Pitch Attitude ($\theta_{\text{pitch}}$)** | $|\theta| \le 0.25\text{ rad}\; (14.3^\circ)$ | Zero twist lock on `/cmd_vel` |
| **Roll Attitude ($\phi_{\text{roll}}$)** | $|\phi| \le 0.25\text{ rad}\; (14.3^\circ)$ | Zero twist lock on `/cmd_vel` |
| **Laser Obstacle Proximity (Stop Zone)** | $d \le 0.38\text{ m}$ | Nav2 Collision Monitor forces immediate deceleration to 0 |
| **Laser Obstacle Proximity (Slow Zone)** | $d \le 0.60\text{ m}$ | Nav2 Collision Monitor scales velocity by factor 0.3 |
| **E-Stop Bus (`/terrain/emergency_stop`)** | `True` | Immediate deterministic halt across all controllers |

---

## 5. Hardware Isolation Flags

| Config Variable | Set Value | Location | Required Value | Status |
| :--- | :--- | :--- | :--- | :--- |
| `MOTOR_OUTPUT_ENABLED` | `0` | `firmware/config.h` | `0` | **VERIFIED** |
| `PROTOCOL_CONFIRMED` | `False` | `adapters/robot-bridge/gracemo_bridge/serial_transport.py` | `False` | **VERIFIED** |
