# Dynamic autonomy defect resolution — 2026-11-21

## Accepted configuration preserved

The accepted simulation configuration is ROS 2 Jazzy, Gazebo Harmonic,
0.70 × 0.68 m rectangular footprint, 0.05 m costmap cells, 0.55 m costmap
inflation, NavFn, DWB through RotationShim, velocity smoothing, collision
monitor, command mux, and the terrain E-stop gate.  Global and local obstacle
layers consume both `/scan` and `/camera/depth/points`.

No footprint, inflation, collision-monitor, E-stop, or motor setting was
relaxed.  Robot movement was Gazebo-only.

## Existing unsafe/failed evidence

| Evidence | Defect |
|---|---|
| `case_0` front insertion | Traversable route ended in `NAVIGATION_FAILURE` with −0.032 m padded clearance. |
| `crossing_mirrored` | Mission reported completion despite −0.00058 m padded overlap. |
| `case_4`, `case_9` replan/near-goal | Padded overlap; one timeout lacked its terminal record, the other exhausted retries. |
| `replanning_barrier` | A replacement NavFn plan was computed before obstacle marking and reached −0.0278 m executed clearance. |
| doorway cases | Correctly bounded failures: gaps are narrower than the padded footprint. |

The prior traces establish that centreline/global-path availability alone was
not sufficient.  Costmap observations could be cleared during a recovery and a
blind reverse recovery could be selected before a fresh footprint-valid route
was available.

## Correction

`navigate_to_pose_gracemo.xml` no longer clears local/global costmaps in its
retry tree and no longer invokes `BackUp`.  It waits for fresh observations
instead.  This is the smallest safety correction supported by the failed
experiments: it preserves the detecting sensor evidence and removes an
unvalidated reverse motion.  The existing coordinator remains responsible for
the bounded retry count and terminal `NAVIGATION_FAILURE`.

This correction does **not** by itself promote the private swept-footprint
guard/recovery prototype into the accepted launch.  That candidate must first
pass the complete deterministic recovery and E-stop-interruption matrix.

## Measured safety thresholds

* E-stop acceptance: final one-second odometry speed ≤ 0.05 and post-stop
  displacement ≤ 0.15 m; a zero command must be observed.
* Latest Gazebo E-stop: PASS, 0.0099 m drift, 0.0171 final speed, zero command,
  mission rejection while latched, explicit reset, and zero held after reset.
* Idle-zero acceptance: three seconds of exactly zero `/cmd_vel`,
  `/cmd_vel_mux`, and `/cmd_vel_smoothed`, with mux source `NONE`.
  Existing evidence has 58/58/60 samples at zero and 0.0081 m odometry drift.

## New Gazebo evidence

### Physically blocked hallway

Command:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 scripts/validation/run_routing_case.py OUT --box 2 0 .35 1.65
```

Result: expected-safe-failure PASS.  The runner exits nonzero because its
generic success condition expects arrival, but mission evidence shows three
obstacle-handling/replan attempts, then `NAVIGATION_FAILURE` after 39.1 s,
0/1 completed waypoints, 4.547 m goal error, and no false completion.  The
record is `blocked_after_bt_repair_mission.json`.

### Five-room patrol after correction

`./scripts/launch_full_simulation.sh --patrol --no-gui --timeout 300` passed.
Mission `standalone-df51fc5c-4722-46d8-a87c-511893198bed` and trace
`3f8e1df8-a371-4663-82c5-c13a38a86b50` matched in coordinator and kernel;
11/11 waypoints completed in 190.6 seconds.

## Release status

* Static/corridor patrol safety: PASS.
* Blocked route bounded failure after correction: PASS.
* Dynamic crossing recovery: PARTIAL.  The private guard has one successful
  crossing result, but all failed/mirrored/moving variants and E-stop during
  every recovery substate have not been revalidated on the corrected accepted
  stack.
* 25- and 100-case campaigns: NOT STARTED; their stated prerequisites are not
  yet fully demonstrated.
