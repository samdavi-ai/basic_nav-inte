# Deterministic Gazebo results

Clearance is the minimum for the executed interpolated footprint. A positive run with no independent positive intersample bound has partial geometry evidence. Interruption verdicts apply to their probes. Shutdown escalation is recorded independently in results.json.

| Run | Verdict | Kernel | Waypoints | Minimum (m) | Goal error (m) | DWB rejected |
|---|---|---|---:|---:|---:|---:|
| [blocked](blocked/scenario.json) | SAFE TERMINAL FAILURE | NAVIGATION_FAILURE | 0/1 | 0.3711 | 4.6065 | — |
| [budget_patrol](budget_patrol/scenario.json) | UNSAFE FAILURE | NAVIGATION_FAILURE | 0/11 | -0.0691 | — | 122323 |
| [cross](cross/scenario.json) | PARTIAL | MISSION_COMPLETED | 1/1 | 0.2390 | 0.2585 | 37758 |
| [doorway_dense](doorway_dense/scenario.json) | UNSAFE FAILURE | MISSION_COMPLETED | 11/11 | -0.0273 | — | 80530 |
| [doorway_patrol](doorway_patrol/scenario.json) | PARTIAL | MISSION_COMPLETED | 11/11 | 0.0005 | — | 75686 |
| [fenced_4](fenced_4/scenario.json) | UNSAFE FAILURE | NAVIGATION_FAILURE | 0/1 | -0.0298 | 0.6873 | 43495 |
| [fenced_9](fenced_9/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/1 | 0.0002 | 0.7379 | 84003 |
| [front](front/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/1 | 0.3294 | 3.4219 | 26926 |
| [front_v2](front_v2/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/1 | 0.4203 | 3.4779 | 7559 |
| [idle_latch_shutdown](idle_latch_shutdown/scenario.json) | PASS | — | —/— | — | — | — |
| [larger](larger/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/1 | 0.4590 | 4.5437 | 16396 |
| [mirror_cross](mirror_cross/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/1 | 0.4236 | 3.6434 | 31843 |
| [mirrored](mirrored/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/1 | 0.3114 | 3.5100 | 22096 |
| [near](near/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/1 | 0.3682 | 2.8054 | 19863 |
| [original](original/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/1 | 0.4673 | 4.5391 | 13327 |
| [original_v2](original_v2/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/1 | 0.4604 | 4.5425 | 14180 |
| [patrol](patrol/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/11 | 0.3064 | — | — |
| [patrol_v2](patrol_v2/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/11 | 0.3065 | — | — |
| [rect_front](rect_front/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/1 | 0.0172 | 3.0604 | 32360 |
| [rect_larger](rect_larger/scenario.json) | PASS | MISSION_COMPLETED | 1/1 | 0.1445 | 0.2394 | 25503 |
| [rect_mirrored](rect_mirrored/scenario.json) | PASS | MISSION_COMPLETED | 1/1 | 0.1990 | 0.2271 | 25255 |
| [rect_moving_mirror](rect_moving_mirror/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/1 | 0.4201 | 3.6730 | 27506 |
| [rect_original](rect_original/scenario.json) | PASS | MISSION_COMPLETED | 1/1 | 0.0943 | 0.2124 | 24914 |
| [rect_patrol](rect_patrol/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/11 | 0.3069 | — | — |
| [rect_study](rect_study/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/2 | 0.1439 | 2.6978 | 53288 |
| [replanning](replanning/scenario.json) | UNSAFE FAILURE | NAVIGATION_FAILURE | 0/1 | -0.1209 | 0.5895 | 100508 |
| [restart_idle](restart_idle/scenario.json) | FAIL | — | —/— | — | — | — |
| [restart_idle_retry](restart_idle_retry/scenario.json) | PARTIAL | — | —/— | — | — | — |
| [stop_COSTMAP_UPDATE](stop_COSTMAP_UPDATE/scenario.json) | PASS | MISSION_CANCELLED | 0/1 | — | — | — |
| [stop_NAVIGATING](stop_NAVIGATING/scenario.json) | PASS | MISSION_CANCELLED | 0/1 | — | — | — |
| [stop_OBSTACLE_DETECTED](stop_OBSTACLE_DETECTED/scenario.json) | PASS | MISSION_CANCELLED | 0/1 | — | — | — |
| [stop_PATH_VALIDATION](stop_PATH_VALIDATION/scenario.json) | PASS | MISSION_CANCELLED | 0/1 | — | — | — |
| [stop_REPLANNING](stop_REPLANNING/scenario.json) | PASS | MISSION_CANCELLED | 0/1 | — | — | — |
| [stop_RESUMING](stop_RESUMING/scenario.json) | PASS | MISSION_CANCELLED | 0/1 | — | — | — |
| [stop_ROUTE_REASSESSMENT](stop_ROUTE_REASSESSMENT/scenario.json) | PASS | MISSION_CANCELLED | 0/1 | — | — | — |
| [stop_SAFETY_STOP](stop_SAFETY_STOP/scenario.json) | PASS | MISSION_CANCELLED | 0/1 | — | — | — |
| [study](study/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/2 | 0.1299 | 2.5275 | 48101 |
| [study_v2](study_v2/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/2 | 0.1491 | 2.5910 | 31063 |
| [temporary](temporary/scenario.json) | PARTIAL | MISSION_COMPLETED | 1/1 | 0.3948 | 0.2429 | 32987 |
| [turning_patrol](turning_patrol/scenario.json) | FAIL | NAVIGATION_FAILURE | 0/11 | 0.0532 | — | 10544 |
