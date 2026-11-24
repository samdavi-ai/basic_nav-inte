# End-to-end scenarios

| Scenario | Result | Evidence or missing stage |
|---|---|---|
| A. Spoken room navigation | PARTIAL | Simulated corridor reached the goal through the standalone adapter. Live microphone input and spoken arrival were unavailable. |
| B. Context-aware question | PARTIAL | Actual adapter context read, no spoken answer. Robot pose alone cannot locate the human asking the question. |
| C. Obstacle awareness | PARTIAL | Static avoidance succeeded. Prior dynamic insertion stopped safely in one trial and exhausted retries. Live object-to-adapter-to-recovery chain remains open. |
| D. Interruption | PARTIAL | Prior eight-state E-stop probes, current ROS bridge and text safety tests. Spoken stop during motion unverified. |
| E. Navigate, look around and report | UNVERIFIED | Tool availability alone gives no ordered skill completions or grounded final report. Library destination requires resolution against actual room inventory. |
| F. Provider failure | PARTIAL | Fault and offline/mock adapter tests. Live provider failover with preserved active mission unverified. |
| G. Restart and memory | PARTIAL | Synthetic observation and graph object survived a real kernel restart. Full voice-originated mission history after a full-system restart remains open. |

[Kernel and adapter evidence](context_adapter_restart.json) · [Gazebo evidence](gazebo_control/mission.json) · [Unit and fixture cases](offline.xml)
