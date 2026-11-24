# E-stop interruption matrix

Times use the observer monotonic clock for latency and the ROS simulation clock for entry. Displacement limit 0.15 m. Final-second component limits 0.05 m/s and 0.05 rad/s.

| State | Probe | Entry (sim s) | Observe zero (ms) | Max displacement (m) | Final-second peak | Post-reset command peak | Terminal |
|---|---|---:|---:|---:|---:|---:|---|
| [COSTMAP_UPDATE](stop_COSTMAP_UPDATE/interrupt.json) | PASS | 6.1000 | 2.5252 | 0.0089 | 0.0172 | 0.0000 | MISSION_CANCELLED |
| [NAVIGATING](stop_NAVIGATING/interrupt.json) | PASS | 3.5500 | 2.6382 | 0.0088 | 0.0191 | 0.0000 | MISSION_CANCELLED |
| [OBSTACLE_DETECTED](stop_OBSTACLE_DETECTED/interrupt.json) | PASS | 5.4500 | 3.2349 | 0.0715 | 0.0170 | 0.0000 | MISSION_CANCELLED |
| [PATH_VALIDATION](stop_PATH_VALIDATION/interrupt.json) | PASS | 8.2000 | 2.4108 | 0.0090 | 0.0164 | 0.0000 | MISSION_CANCELLED |
| [REPLANNING](stop_REPLANNING/interrupt.json) | PASS | 8.5500 | 3.7311 | 0.0095 | 0.0196 | 0.0000 | MISSION_CANCELLED |
| [RESUMING](stop_RESUMING/interrupt.json) | PASS | 8.4500 | 2.2942 | 0.0114 | 0.0186 | 0.0000 | MISSION_CANCELLED |
| [ROUTE_REASSESSMENT](stop_ROUTE_REASSESSMENT/interrupt.json) | PASS | 6.4500 | 2.3661 | 0.0089 | 0.0190 | 0.0000 | MISSION_CANCELLED |
| [SAFETY_STOP](stop_SAFETY_STOP/interrupt.json) | PASS | 4.9000 | 2.9313 | 0.0824 | 0.0188 | 0.0000 | MISSION_CANCELLED |

The stopped internal phases use a Gazebo-only pause at state entry. REPLANNING denotes the recovery FSM phase. This probe does not isolate an instruction inside the planner plugin. NAVIGATING can occur as the first drive command is issued, before measurable wheel speed. The separate moving E-stop smoke test exercises an already moving chassis. Each probe rejected a new request while latched and required explicit reset.
