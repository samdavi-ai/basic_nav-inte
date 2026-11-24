# System repair and integration audit — 2026-11-21

## Scope and safety

All work was performed against local ROS 2 Jazzy and Gazebo Harmonic simulation.
No physical robot, motor transport, firmware, or hardware interface was commanded.
The master safety gate reported `MOTOR_OUTPUT_ENABLED = 0` and
`PROTOCOL_CONFIRMED = False` during its smoke test.

## Findings and repair

The supported autonomous baseline is `scripts/launch_full_simulation.sh`: it
uses ROS domain 68, partition `gracemo-68`, isolated process groups, readiness
gating, and a real kernel-mediated mission dispatch.  In contrast,
`start_sim.sh`, `start_mission.sh`, and the former prototype scripts invoked a
separate ROS 2 Humble/distrobox stack and used global `pkill -9` rules.  Those
paths could mix DDS environments or kill an unrelated Gazebo session.

The compatibility entry points now delegate to the supported Jazzy launcher or
mission dispatcher.  `start_kernel.sh` reuses a healthy local kernel instead of
starting a duplicate.  The master launcher now defaults to the baseline domain
and partition, refuses to kill a pre-existing unowned session, and starts its
kernel, ROS launch, and interaction process in separate sessions.  Its cleanup
signals only those recorded process groups, escalating only within those groups
when graceful termination does not complete.

`run_prototype.sh` is now a compatibility alias for the supported launcher.
`stop_prototype.sh` deliberately does not perform a name-based global shutdown;
the owning launcher must be stopped with Ctrl+C.

## Simulation evidence

Command:

```bash
./scripts/launch_full_simulation.sh --patrol --no-gui --timeout 300
```

Result: PASS.  The run dispatched mission
`standalone-6f47c34a-cea9-4500-88e9-da9216f71a10` with trace
`5dcee729-7ed3-4379-94db-b8fd634dcf48`.  Coordinator result was `COMPLETED`,
11/11 waypoints in 179.4 seconds.  Kernel result was `MISSION_COMPLETED`,
reported the same mission and trace IDs, and stated `All 11 destinations
reached`.  The machine-readable record is
`docs/validation/standalone_patrol_latest.json`.

Command:

```bash
./run_gracemo.sh --headless --no-verify --duration 2
```

Result: PASS as a bounded startup/shutdown smoke test.  Kernel, Gazebo/ROS,
Nav2, interaction bridge, Pygame interaction engine, and the offscreen PySide6
control center all started.  The UI's ROS executor then stopped cleanly.  This
test does not establish navigation readiness because verification was explicitly
disabled and the duration is intentionally too short for localization.

Static checks passed:

```bash
bash -n run_gracemo.sh scripts/start_sim.sh scripts/start_mission.sh \
  scripts/start_kernel.sh scripts/run_prototype.sh scripts/stop_prototype.sh \
  scripts/launch_full_simulation.sh
git diff --check
```

## Open release blockers / non-claims

* The control center is a PySide6 desktop application.  No FastAPI, Flask, or
  other HTTP server on port 8000 exists in this repository, so a dashboard at
  `http://localhost:8000` was not claimed or fabricated.
* The interaction engine reports no local Qwen MLX model on Linux and falls
  back to Groq.  The offscreen process starts, but microphone, speaker, camera,
  and cloud-backed conversation need an operator-provisioned device/network
  acceptance test.
* Dynamic obstacle recovery remains a separate, partially accepted validation
  item; see `docs/validation/dynamic_obstacles/REPORT.md`.  It must not be
  promoted as a universal dynamic-obstacle release gate yet.
* This document covers simulation evidence only; it is not physical-robot
  validation.
