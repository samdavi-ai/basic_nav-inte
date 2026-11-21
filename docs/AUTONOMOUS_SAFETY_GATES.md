# GRaCEmo ViRa — Autonomous Safety Gates & Boundaries

**Document Version:** 1.0.0  
**Status:** STRICT ENFORCEMENT  

---

## 1. Safety Architecture Overview

The GRaCEmo ViRa platform enforces layered safety gates across software, control, and hardware boundaries. No single failure can result in uncontrolled physical or simulated motion.

```mermaid
graph TD
    Nav2[Nav2 Autonomous Velocity /cmd_vel_nav]
    Teleop[Manual Teleop /cmd_vel_teleop]

    subgraph Gate 1: Priority Command Arbitration
        Mux[cmd_vel_mux\nTeleop=100 > Nav2=50\nTimeout=0.5s]
    end

    subgraph Gate 2: Acceleration Boundary
        Smoother[nav2_velocity_smoother\nMax a_lin = 1.0 m/s2\nMax a_ang = 2.0 rad/s2]
    end

    subgraph Gate 3: Geometric Collision Monitor
        CollMon[nav2_collision_monitor\nFootprint Polygon Checks\nSlowdown & Stop Zones]
    end

    subgraph Gate 4: Terrain & Attitude Gate (Sole Publisher)
        Terrain[gracemo_terrain_monitor\nPitch/Roll Limit: <= 0.25 rad\nCliff/Drop Detection]
    end

    subgraph Gate 5: Hardware Lockout Invariants
        Invariants[Firmware: MOTOR_OUTPUT_ENABLED = 0\nSerialTransport: PROTOCOL_CONFIRMED = False]
    end

    Nav2 --> Mux
    Teleop --> Mux
    Mux --> Smoother
    Smoother --> CollMon
    CollMon --> Terrain
    Terrain -->|/cmd_vel| SimulationOnly[Gazebo Harmonic Differential Drive]
    Invariants -.->|Permanently Blocks| PhysicalActuation[Physical Motors Disengaged]
```

---

## 2. Layered Safety Gates

### Gate 1: Priority Command Arbitration (`twist_mux`)
- Teleop manual velocity overrides autonomous velocity deterministically.
- Teleoperation channel (`/cmd_vel_teleop`) has priority `100`.
- Autonomous navigation channel (`/cmd_vel_nav`) has priority `50`.
- Stale messages expire after $500\text{ ms}$, stopping motion if publisher crashes.

### Gate 2: Velocity & Acceleration Limiter (`nav2_velocity_smoother`)
- Enforces dynamic kinematic envelopes:
  - Maximum linear velocity: $v_{\text{max}} = 0.40\text{ m/s}$
  - Maximum angular velocity: $\omega_{\text{max}} = 0.80\text{ rad/s}$
  - Maximum linear acceleration: $a_{\text{max}} = 1.00\text{ m/s}^2$
  - Maximum linear deceleration: $d_{\text{max}} = -1.00\text{ m/s}^2$

### Gate 3: Collision Monitor Zone Checks (`nav2_collision_monitor`)
- Evaluates real-time 2D laser scan points against robot footprint polygons.
- **Slowdown Zone:** Scales velocity down when obstacle enters $0.60\text{ m}$ perimeter.
- **Stop Zone:** Halts velocity to $0.0\text{ m/s}$ when obstacle enters $0.38\text{ m}$ perimeter.

### Gate 4: Terrain Monitor Attitude Safety Gate (`gracemo_terrain_monitor`)
- Acts as the **sole authorized publisher** to `/cmd_vel`.
- Monitors IMU pitch and roll:
  $$|\theta_{\text{pitch}}| \le 0.25\text{ rad}\quad (\approx 14.3^\circ)$$
  $$|\phi_{\text{roll}}| \le 0.25\text{ rad}\quad (\approx 14.3^\circ)$$
- Monitors cliff/drop sensors.
- If threshold is exceeded or emergency stop signal is received on `/terrain/emergency_stop`, zero twist is locked.

### Gate 5: Absolute Hardware Isolation Invariants
- `firmware/config.h`:
  ```c
  #define MOTOR_OUTPUT_ENABLED 0
  ```
  Physical motor PWM outputs are permanently disabled at compile time.
- `adapters/robot-bridge/gracemo_bridge/serial_transport.py`:
  ```python
  PROTOCOL_CONFIRMED: bool = False
  ```
  Serial packets are rejected, preventing physical MCU communication.
- Physical testing is blocked until formal hardware readiness gate review.
