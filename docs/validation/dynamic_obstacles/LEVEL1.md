# Seed 42 — Level 1 Gazebo evidence

Each index identifies a scenario definition. Repeats replace invalid simulator or actor runs. Durations use the simulation clock. Negative sensor-to-map intervals mean the recorded costmap snapshot preceded the first associated retained return. A dash means the event was not measured.

| Index and evidence | Scenario | Terminal event | Goal error (m) | Minimum scene clearance (m) | Sensor → local mark (s) | Sensor → stop command (s) | Coordinator retries | Interpretation |
|---|---|---|---:|---:|---:|---:|---:|---|
| [0](case_0/metrics.json) | front | NAVIGATION_FAILURE | 3.005 | -0.032 | -0.278 | — | 3 | Traversable. Retries exhausted with footprint overlap. |
| [1](case_1/metrics.json) | beside | MISSION_COMPLETED | 0.212 | 0.243 | 0.708 | — | 0 | Beside route. Goal reached. Surface attribution is ambiguous. |
| [2](crossing_repeat/metrics.json) | crossing | MISSION_COMPLETED | 0.292 | 0.184 | -0.120 | — | 0 | Crossing verified. Goal reached. A proposed rollout intersected the frozen fixture. |
| [3](case_3/metrics.json) | doorway | NAVIGATION_FAILURE | 3.568 | 0.147 | -0.460 | — | 3 | Doorway physically blocked. Bounded failure and final zero. |
| [4](case_4/metrics.json) | replan | Timed out | 0.805 | -0.033 | 0.334 | 2.696 | 2 | Terminal record missing at original deadline. Padded overlap. |
| [5](case_5_retry/metrics.json) | front | MISSION_COMPLETED | 0.213 | 0.032 | 0.384 | — | 0 | Goal reached. No verified stop-and-validate sequence. |
| [6](case_6/metrics.json) | beside | MISSION_COMPLETED | 0.198 | 0.143 | 0.532 | — | 0 | Beside route. Goal reached. Surface attribution is ambiguous. |
| [7](crossing_mirrored/metrics.json) | crossing | MISSION_COMPLETED | 0.190 | -0.001 | 0.272 | — | 0 | Crossing verified. Goal reached with padded overlap. |
| [8](case_8/metrics.json) | doorway | NAVIGATION_FAILURE | 3.439 | 0.207 | 0.478 | — | 3 | Doorway physically blocked. Bounded failure and final zero. |
| [9](case_9/metrics.json) | replan | NAVIGATION_FAILURE | 0.626 | -0.031 | 0.332 | 0.126 | 3 | Near-goal overlap. Retries exhausted. |

Five of ten runs reported correlated mission completion and reached the requested goal tolerance. Two permanently blocked doorway runs had positive executed clearance, three retries, a terminal navigation failure and final zero-command windows of 0.772 and 1.260 s. One run timed out before the coordinator emitted a terminal event. The original timeout cancellation was rejected. No case met every successful-recovery acceptance condition.

The exact UUIDs and measured planning durations are in [the machine-readable summary](level1.json). Each evidence directory contains the mission log, fixture manifest, scenario configuration and gzipped time series. In the first near-goal timeout case the mission log preserves the IDs. The first crossing trials ([case 2](case_2/) and [case 7](case_7/)) lacked the requested verified displacement. The [case 5 startup failure](case_5/) never dispatched a mission.

The generator supports staging poses, goal variations and measured corridor-liner/door-jamb fixtures for higher levels. The 25- and 100-case campaigns have no measured results.
