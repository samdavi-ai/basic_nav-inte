# Simulation autonomy and voice milestone status — 2026-11-21

| Phase | Status | Evidence / limitation |
|---|---|---|
| A: E-stop and idle safety | PASS for baseline E-stop/idle path | Latest Gazebo E-stop run: all eight checks passed, drift 0.0099 m, final one-second speed 0.0171 m/s.  E-stop during dynamic replan/recovery remains not separately rerun. |
| B: Dynamic detection | PARTIAL | Actual Gazebo LiDAR/depth and both costmaps are captured in the dynamic harness; reports document timing and optical-frame checks.  Not all requested moving/turning placements are accepted. |
| C: Dynamic avoidance | PARTIAL | Private braking and recovery candidates show safe cases, but accepted-stack crossing cases include unsafe/failed outcomes. |
| D: Bounded recovery | PARTIAL | Mission ID/trace preservation, retry bound and safe failure exist; universal safe recovery is not demonstrated. |
| E: Voice hardware | PARTIAL | Device enumeration and camera frame passed; microphone capture, audible output, cloud provider, and spoken end-to-end tests remain open. |
| F: randomized scenarios | NOT TESTED | Existing deterministic 10-case work is recorded; 25/100 scenario campaigns have not been run. |
| G: full regression | PARTIAL | Static corridor suite, blocked-route safe failure, E-stop/idle baseline and five-room 11-waypoint patrol have evidence. |

The current dynamic-recovery release gate remains experimental.  The supported
release baseline is the standalone Gazebo patrol; no physical navigation claim
is made.
