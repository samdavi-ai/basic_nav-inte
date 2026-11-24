# VIRA master release closure

**DO NOT RELEASE — ACCEPTANCE INCOMPLETE**

Five private Gazebo patrols in this closure sequence ended with the coordinator reporting 0/11 waypoints and the kernel reporting NAVIGATION_FAILURE. The first leg was the kitchen doorway staging pose. The most recent optimized run expired after 45.1 s. The mission and trace identifiers match across each adapter mission envelope and kernel result. The previous standalone control remains the 11/11 functional baseline. The high-rate revised doorway control reached 11/11 while violating clearance.

| Private patrol variation | Executed intersample lower bound (m) | DWB evaluations | Rejected samples | Missed controller deadline warnings |
|---|---:|---:|---:|---:|
| Lattice and swept turn | 0.318360 | 0 | unavailable | 0 |
| Zero-command pause during shim drift | 0.171808 | 165 | 30,328 | 103 |
| Integer nanosecond rollout age | 0.466157 | 0 | unavailable | 0 |
| Planner cell margin | 0.126624 | 194 | 37,922 | 161 |
| Same margin, optimized plugin build | 0.148629 | 220 | 47,965 | 175 |

The bounds apply to the executed prefix only and assume the analyzer's speed bounds of 0.65 m/s and 1.5 rad/s. DWB rejection counts refer to sampled candidate trajectories. Deadline warnings count log entries over different mission durations. The records do not establish which CPU component dominates latency. [Machine-readable run records](results.json)

| Gate | Decision | Evidence |
|---|---|---|
| A. Geometry | Failed | Earlier native poses overlap both the study wall and the projected kitchen chair. No new patrol qualified the full route. [Geometry](GEOMETRY_ACCEPTANCE_REPORT.md) |
| B. Dynamic autonomy | Failed on prior candidate, open on this revision | Front, near-goal and mirrored cases remain unresolved. Synchronized insertion previously produced negative clearance. [Recovery](DYNAMIC_RECOVERY_REPORT.md) |
| C. Navigation | Failed | Planner-grid disagreement was reproduced. The margin trial cleared that specific conflict and still exhausted recovery. [Controller analysis](NAVIGATION_CONTROLLER_REPORT.md) |
| D. E-stop | Incomplete | Eight historical probes passed. The new patrol follow-on smoke requires a dispatch-ready mission. [E-stop](ESTOP_REGRESSION_REPORT.md) |
| E. Idle/restart | Partial | Five post-failure three-second idle windows passed. Restart localization and launcher restart remain open. [Readiness](IDLE_RESTART_READINESS_REPORT.md) |
| F. Normal launcher | Held | Gates A–C are prerequisites. [Integration](NORMAL_LAUNCHER_INTEGRATION_REPORT.md) |
| G. Randomized robustness | Held | Zero randomized trials in this campaign. [Robustness](RANDOMIZED_ROBUSTNESS_REPORT.md) |

## Scope and reproducibility

The initial snapshot records branch main, commit a7945a8d and 21 pre-existing tracked modifications. The accepted navigation YAML and behavior tree match the earlier recorded SHA-256 hashes. All movement used the private Gazebo graph. No changes were integrated into the normal launcher. No physical robot validation, commit or push was performed. [Repository state](repository.txt) · [Initial worktree](initial_status.txt) · [Control fingerprints](baseline_verification.json)

The private runner and approach guard contain the lattice, shim and planning-margin experiments. The mesh exporter, chair replay, grid veto replay and run summarizer support independent inspection. Each run directory retains candidate.yaml, mission.json, native Gazebo poses, compressed ROS telemetry, launch logs and safety observations. The unit tests exercised age and drift boundaries. The recorded mission verdicts came from Gazebo. [Reproduction](REPRODUCTION.md) · [Prior passing and failing scenarios](../master_repair/DETERMINISTIC_RESULTS.md)

## Follow-up with matched braking margins

The private DWB critic now optionally reserves the same additional corner clearance as the approach guard when scoring braking sweeps. The seed-42 Gazebo patrol reported 0/11 after 33.3 s and ended near (−5.13, −1.26) m. The guard recorded an invalid rollout and three braking vetoes. The kernel recorded NAVIGATION_FAILURE under the original mission and trace IDs. DWB published 180 evaluations with 50,022 rejected trajectory samples. Matching this margin alone did not resolve recovery. [Mission](matched_braking/patrol/mission.json) · [Geometry and controller evidence](matched_braking/patrol/geometry.json)

At the nearest southwest hallway wall, the native-pose result was +144.587 mm. The independent configuration-space oracle bounded the interpolated minimum between +144.566 and +144.586 mm. This covers the executed prefix. The robot never reached the previously failing study turn. The full-route geometry and navigation gates remain closed. The post-failure idle window passed. The private profile remains experimental. [Independent oracle](matched_braking/patrol/oracle.json) · [Idle window](matched_braking/patrol/idle.json) · [Reproduction](matched_braking/REPRODUCTION.md)

The follow-on moving E-stop smoke failed its readiness check before triggering a stop. During launcher teardown the ROS kernel bridge exited with code −11. This run supplies neither a moving E-stop pass nor a clean-shutdown pass. [Smoke result](matched_braking/patrol/estop.json) · [Launch log](matched_braking/patrol/launch.log)
