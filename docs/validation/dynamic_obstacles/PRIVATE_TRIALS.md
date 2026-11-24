# Private recovery trials

**DO NOT INTEGRATE.** Seed 42 reproduced padded-footprint overlap during a planner invocation. Forward recovery around a traversable front obstacle exhausted its retry budget. Results use Gazebo physics poses and simulation time unless otherwise indicated.

| Private governed case | Kernel terminal | Minimum swept scene clearance | Goal error | Observed DWB rejected samples after insertion request |
|---|---|---:|---:|---:|
| [Front](governed_front_review/metrics.json) | NAVIGATION_FAILURE | +0.390 m | 3.590 m | Unmeasured in this run |
| [Repeated crossing](governed_review_crossing/metrics.json) | MISSION_COMPLETED | +0.216 m | 0.232 m | 29,261 |
| [Mirrored crossing](governed_mirror_review/metrics.json) | MISSION_COMPLETED | +0.091 m | 0.221 m | 38,303 |
| [Near goal](governed_review_replan_trigger_9/metrics.json) | NAVIGATION_FAILURE | +0.125 m | 0.804 m | 6,608 |
| [During planning](governed_review_replan_trigger_4/metrics.json) | NAVIGATION_FAILURE | −0.031 m | 0.644 m | 7,053 |
| [Temporary full blockage](governed_serial_temporary/metrics.json) | MISSION_COMPLETED | +0.456 m | 0.228 m | 18,467 |

The [per-run index](private_trial_index.json) includes mission IDs, trace IDs, timestamps, resumption margins and excluded attempts. Null timing or rejection counts mean unavailable or inapplicable. Complete raw captures are alongside each run’s JSON.

The mirrored fixture reached its verified endpoint. Its actor exited abnormally during teardown, so the motion evidence is usable but the harness run failed clean-shutdown acceptance. The temporary fixture driver confirmed disappearance at 6.802 s. The analyzer retains its dimensions through confirmation, then checks the apartment for the rest of the mission. DWB counts sum negative trajectory scores in received evaluation messages. A missing stream yields an unknown count.

## Geometry and configuration

The published rectangle measures 0.72 × 0.70 m. For a world pose with translation *p* and yaw θ, its four corners are *p* + R(θ)(±0.36, ±0.35) m. The offline analyzer compares the oriented rectangle against modeled scene obstacles, unwraps adjacent yaws and interpolates both translation and rotation. It bounds corner travel per segment by translation distance plus 0.5021 m times the absolute yaw change. It subtracts half the interpolated step bound from the sampled minimum. These figures assume interpolated poses between Gazebo samples. The stationary table also gives the separate bound assuming 0.65 m/s and 1.5 rad/s. The live guard conservatively encloses the rectangle in a 0.72 m square and expands tested cell centers by a cell half-diagonal. Its global test uses map-frame poses and its local test uses odometry. The offline global-route comparison uses the recorded map-to-odom transform. Dynamic route snapshots freeze the obstacle at the plan time. Only the executed comparison follows its recorded trajectory.

The accepted parameter file SHA-256 is `91beac6308cd922881b3a14017a021d968c0478d6d7fd716d4b58a2501666c06`; the accepted behavior tree hash is `573cdd8d2724d3ae0aaf4f10206e2ada58203fae27e0b4301b2e36bc6d5a46e1`. The [front trial configuration](governed_serial_front/candidate.yaml) uses the private controller critics, recovery planner and guard-to-mux remap. Online geometry uses observed grids only. Offline Gazebo truth includes obstacles before the sensors can mark them.

## Stationary fixtures and patrol

| Private case | Outcome | Executed interpolated minimum | Independent speed-based bound | DWB rejected |
|---|---|---:|---:|---:|
| [Original corridor](governed_review_original/geometry.json) | Arrived | +0.112 m | +0.013 m | 18,541 |
| [Larger corridor](governed_serial_larger/geometry.json) | Arrived | +0.082 m | +0.011 m | 27,243 |
| [Mirrored corridor](governed_review_mirrored/geometry.json) | Arrived | +0.211 m | +0.067 m | 22,024 |
| [Open study](governed_review_study/geometry.json) | Failed, 1.844 m short | +0.107 m | −0.004 m | 86,640 |
| [Blocked hallway](governed_review_blocked/mission.json) | Correlated failure after 6.4 s | +0.485 m | +0.452 m | No evaluations received |
| [Five-room patrol](governed_review_patrol/mission.json) | Failed at 0/11 after 9.9 s | +0.312 m | +0.291 m | No evaluations received |

The study watchdog reported stagnation. Its negative speed-based bound leaves intersample clearance unproven. The private patrol produced an initial global path before three recovery planner failures near the origin on the approach to (−5.00, −1.60). The post-patrol [idle test](governed_review_patrol/idle.json) measured zero on all three command topics for three seconds. The subsequent [E-stop smoke test](governed_review_patrol/estop.json) could not begin from the coordinator's navigation-failure state.

## Accepted-stack comparison

With the private guard, critics and recovery coordinator absent, the same Gazebo harness reproduced **11/11** waypoints in **176.5 s** with matching mission and kernel trace identifiers. Gazebo and the map both placed the chassis 0.20 m from the final waypoint. The subsequent [idle-zero](baseline_patrol_review/idle.json) and [moving E-stop, rejection and explicit reset](baseline_patrol_review/estop.json) checks passed. [Mission result](baseline_patrol_review/mission.json)

The [patrol clearance trace](baseline_patrol_review/geometry.json) exposes a separate limitation. Near the northeast hallway wall the minimum at a raw Gazebo pose was +0.00257 m. Between poses, the interpolated padded rectangle reached −0.00060 m and the conservative interpolated lower bound was −0.00110 m. This run validates mission completion, but fails the strictly positive swept-clearance gate. Physical contact cannot be inferred from this small interpolated overlap. This baseline finding remains open.

## Front diagnosis

The previous preflight failure occurred because the launch environment omitted the private critic overlay. The controller explicitly rejected the unknown plugin. The fixture actor then waited for motion until cleanup. The harness now checks plugin registration before launching and reports a missing mission result before waiting on the actor.

With the overlay sourced, depth first observed the front box at 4.600 s, the global costmap marked it at 4.636 s, the guard stopped at 4.650 s, the local costmap marked at 4.818 s, and the chassis reached rest at 5.000 s. The first resumed route had +0.108 m global and +0.177 m local snapshot clearance. A later command triggered the braking veto at 7.172 s while the global plan remained valid. A no-progress watchdog consumed another retry after 12.3 s. A second validated resumption at 23.108 s preceded another braking veto at 23.664 s and terminal failure. This rules out both missing global routes and a missing first-command braking check as sufficient explanations. The guard tests the evolving odometry, map age and requested twist each cycle. The private coordinator treats each fresh obstacle event as a reason to cancel the active goal. More detailed veto telemetry now captures those inputs.

A [repeat with command telemetry](governed_serial_front/metrics.json) recorded three resumptions at 6.972, 9.474 and 11.850 s. The next two vetoes rejected braking envelopes with grid ages of 0.528 and 0.488 s. The fourth event rejected an invalid global path, exhausting the retry budget. There were 26,270 rejected DWB samples in 84 received evaluation messages. The executed minimum remained +0.067 m, with the robot 2.721 m short at failure. The controller selected forward commands even while some simulated trajectories were rejected. Freshness, current twist and the latest selected rollout matter beyond the snapshot clearance checked on resumption. The recorder dropped 22 sensor transformations, so the first matched sensor hit is an upper bound on first observation.

The repeat launch log records four Nav2 cancel acknowledgments. Three replacement goals ran within the same outer mission. The coordinator filters callbacks by its current goal counter. No stale goal completion was observed.

For synchronized insertion, the first retained truth pose after scene confirmation already had −0.0259 m clearance. The worst sampled pose had −0.0266 m at 12.198 s. The safety gate fails on sampled geometry as well as the interpolated bound. The recording cannot isolate the exact physics step of initial overlap.

## Interruption coverage

Each private probe observes the guard transition and active coordinator mission, sends STOP, checks final velocity commands, drift and settled odometry, attempts a second mission while latched, explicitly resets, and observes commands and odometry for one further second. Command latency is measured using the observer's monotonic clock. The numerical limits are 0.05 for each tested odometry component (m/s linear, rad/s angular) and 0.15 m displacement. Join the probe's mission ID to the sibling mission record for the kernel trace.

| Target | Evidence |
|---|---|
| NAVIGATING | [Probe](interrupt_review_NAVIGATING/interrupt.json) |
| OBSTACLE_DETECTED | [Probe](interrupt_review_OBSTACLE_DETECTED/interrupt.json) |
| SAFETY_STOP | [PASS](interrupt_review_SAFETY_STOP/interrupt.json) |
| COSTMAP_UPDATE | NOT TESTED. Map callbacks have no independently interruptible state. |
| ROUTE_REASSESSMENT | NOT TESTED. Guard performs the check inside its callback. |
| REPLANNING | NOT TESTED. The planning barrier currently controls fixture insertion only. |
| PATH_VALIDATION | NOT TESTED. Validation runs synchronously inside a callback. |
| RESUMING | [PASS](interrupt_review_RESUMING/interrupt.json) |

Across the four observed transitions, the first zero arrived within 2.1–23.6 ms of the STOP request (wall clock). Maximum displacement was 0.0191 m. All four rejected the second mission and passed the explicit-reset observation. The standalone mission client subsequently timed out and requested cancellation. Immediate correlated mission termination on E-stop remains an open lifecycle defect.

## Reproduction and packaging

From the repository root, build the private plugin in a temporary prefix, source it, and use a new directory per trial.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
trial=$(mktemp -d)
colcon --log-base "$trial/log" build --base-paths scripts/validation/planner_fixture_barrier --build-base "$trial/build" --install-base "$trial/install"
source "$trial/install/local_setup.bash"
python3 scripts/validation/dynamic_cases.py "$trial/front" --govern-speed --index 0 --seed 42
python3 scripts/validation/analyze_dynamic.py "$trial/front"
python3 scripts/validation/dynamic_cases.py "$trial/barrier" --govern-speed --index 4 --synchronize-replan
python3 scripts/validation/dynamic_cases.py "$trial/temporary" --govern-speed --temporary
python3 scripts/validation/dynamic_cases.py "$trial/stop" --index 0 --interrupt-at SAFETY_STOP
for index in 2 7 9; do
  python3 scripts/validation/dynamic_cases.py "$trial/index-$index" --govern-speed --index "$index" --seed 42
  python3 scripts/validation/analyze_dynamic.py "$trial/index-$index"
done
for fixture in original mirrored larger study blocked; do
  python3 scripts/validation/dynamic_cases.py "$trial/$fixture" --govern-speed --regression "$fixture"
  python3 scripts/validation/analyze_routing.py "$trial/$fixture"
done
for phase in NAVIGATING OBSTACLE_DETECTED RESUMING; do
  python3 scripts/validation/dynamic_cases.py "$trial/stop-$phase" --interrupt-at "$phase"
done
python3 scripts/validation/dynamic_cases.py "$trial/private-patrol" --govern-speed --regression patrol --mission-timeout 300
python3 scripts/validation/dynamic_cases.py "$trial/baseline-patrol" --regression patrol --mission-timeout 300
python3 scripts/validation/summarize_private_trials.py
```

Indices 2, 7 and 9 select crossing, mirror and near-goal. Static fixtures use `--govern-speed --regression original|larger|mirrored|study|blocked`. Patrol uses `--govern-speed --regression patrol --mission-timeout 300`. Competing private trials now serialize before acquiring their DDS domain after two simultaneous sessions recorded backwards clock jumps. Exclude those runs from acceptance.

The current fixture copies the installed package, substitutes a Python coordinator, and installs a recovery tree using forward-only Smac Hybrid. A packaged implementation would require an independently installable DWB plugin with pluginlib export, a ROS executable for the guard with NumPy and SciPy dependencies, an installed tree, and explicit readiness and shutdown ownership. A single mux navigation input must consume the guarded topic. Remapping the guard's input to its output creates a loop. Adding a second unguarded nav publisher defeats the veto. Teleop and RL already have separate arbitration and need independent safety validation. Restarting the guard mid-mission loses its hold and validated-plan history. The downstream collision monitor and final terrain E-stop must retain exclusive ownership of their existing outputs.
