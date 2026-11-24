# Corridor routing in the apartment simulation

**Decision, 21 November 2026.** The stationary 0.35 m corridor fixture passes in Gazebo with the full padded footprint. On the final fresh-start run DWB produced 195 local plans, recorded zero cycles with all trajectories rejected, and finished 0.222 m from the requested goal in independent Gazebo coordinates. The smallest executed clearance was 0.131 m along the interpolated trajectory. A separate speed-based bound gives 0.111 m. The fixture-specific repair is retained. Generalized obstacle routing remains open. [Final trace and geometry](release_corridor/geometry.json) · [Mission](release_corridor/mission.json)

![Original and repaired Gazebo center paths, oriented footprints, fixture and requested goal](corridor-routes.png)

## Simulation acceptance

The two path columns give the worst signed lower bound across **all** global plans. The first compares tangent-oriented paths in the nominal apartment map. The second applies the measured map-to-odom transform at publication. Local rollouts are the paths DWB selected. Executed margins come from the independent Gazebo pose stream. The physically blocked case expects terminal failure. The late insertion is an additional stress case.

| Experiment | Global tangent, map / odom (m) | Chosen rollout min (m) | Executed min (m) | Gazebo goal error (m) | Outcome |
|---|---:|---:|---:|---:|---|
| [Original 0.35 m fixture](release_corridor/geometry.json) | +0.01636 / +0.01510 | +0.1317 | +0.1314 | 0.222 | Pass, 1/1 |
| [Same fixture after 99 s idle](approach_delayed/geometry.json) | +0.01516 / +0.00016 | +0.1078 | +0.1749 | 0.121 | Pass, 1/1 |
| [0.30 × 0.40 m at (2.35, +0.50)](release_mirrored/geometry.json) | +0.07229 / +0.07607 | +0.1576 | +0.2126 | 0.238 | Pass, 1/1 |
| [0.45 × 0.40 m at (2.60, −0.52)](release_larger/geometry.json) | +0.01269 / +0.01193 | +0.1473 | +0.1595 | 0.247 | Pass, 1/1 |
| [Open study, 0.45 × 0.45 m](release_study/geometry.json) | -0.02099 / -0.00027 | +0.0593 | +0.0894 | 0.235 | 2/2, driven route clear. Global path margin failed |
| [0.35 × 1.65 m barrier](release_blocked/geometry.json) | -0.62225 / -0.64172 | +0.0592 | +0.0484 | 5.817 | Expected failure, 0/1 |
| [Box inserted after DWB started](release_late/geometry.json) | -0.13000 / -0.13295 | -0.0240 | -0.0011 | 0.237 | Goal reached. Padded clearance failed |

The barrier left 0.075 m on either side in a 1.80 m corridor. The coordinator exhausted three retries and returned `NAVIGATION_FAILURE` in 80.7 s, before the 115 s deadline. During clearing/recovery one infeasible global plan was published. The robot stayed on the starting side and its padded footprint maintained clearance. [Blocked mission](release_blocked/mission.json) · [Recovery log](release_blocked/launch.log)

On the exact fixture, the baseline stopped at Gazebo (1.43, −0.42), approximately 3.10 m from the goal. The coordinator reported 2.95 m map error. The repaired base reached Gazebo (4.281, 0.037), 0.222 m from (4.5, 0). The original first path had 0.125 m of nominal padded overlap. All three repaired global paths cleared, with a minimum tangent margin of 0.016 m in map and 0.015 m after the measured transform into odom. The local/global maps went from zero/zero median lethal cells near the fixture to 19/19. Candidate scoring still sampled up to 0.65 m/s, and the downstream speed limiter remained active. A 99 s idle start yielded a very narrow 0.00016 m global geometric bound in odom. That figure is below the 0.05 m cell resolution and supplies no practical robustness margin. The actual drive retained 0.175 m. [Baseline trace](baseline/trace.jsonl.gz) · [Final trace](release_corridor/trace.jsonl.gz) · [Delayed run](approach_delayed/geometry.json)

## Reproduction and cause

The fixture is a static 0.35 × 0.35 × 1.00 m Gazebo box centered at (2.0, −0.5, 0.5). The map goal is (4.5, 0.0, 0 rad). The runner verifies the model's position through Gazebo scene information. Both costmaps have 0.05 m cells. The static apartment map contains the walls. The inserted box must enter the costmaps through observations.

The original global obstacle layer used only the planar scan. Beams aimed at the box often returned a point on the robot itself. For example, with the box center 1.96 m from the lidar, three native Gazebo beams toward it returned 0.180, 0.179 and 0.179 m. These are inside the existing body exclusion region. The published scan correctly removed them. In the original delayed run, the first global snapshot had zero lethal cells near the box, the median was zero and the maximum was four. The original local obstacle layer also had a median of zero. The count includes cells within one map cell of the true box bounds to account for rasterization. The recorder retains all cells, including those outside that window. [Original measurements](baseline/geometry.json) · [Native and bridged beam evidence](repaired_delayed/trace.jsonl.gz)

The depth camera could see the object. The bridge published Gazebo forward/left/up coordinates under a ROS right/down/forward optical frame ID and applied the self-filter to the mislabeled coordinates. A central image point was (1.04378, −0.00423, −0.00423) m. Its optical depth should be +1.04378 m. The repair maps each point to (−Y, −Z, X) before applying the existing body filter. The global obstacle layer then consumes the corrected cloud using the same height, marking, clearing and range settings as the local layer. A control run with only the camera conversion still ended in navigation failure. Its local costmap had a median of 29 lethal cells near the fixture, while its scan-only global costmap had a median of two. [Control run](local_depth_only/geometry.json)

The original planner produced a path through the padded footprint's exclusion region. At x ≈ 2.08 m the first path ran near y = −0.100 m. At this heading the robot's lower edge was near −0.450 m and the obstacle's upper edge was −0.325 m. The rectangles overlapped by 0.125 m. The controller reported 27 cycles in which all 499 trajectories failed the BaseObstacle critic, including “Trajectory Hits Obstacle.” Costmap clears and three coordinator retries did not produce a traversable route. The mission failed with zero completed waypoints. The saved diagnostic, this run, and a fresh-start reproduction independently show the failure. [Original launch log](baseline/launch.log) · [Original mission](baseline/mission.json) · [Fresh-start control](original_fresh/mission.json)

## Footprint and feasibility calculation

The configured rectangle is 0.70 × 0.68 m. Published local footprint vertices measured 0.72 × 0.70 m after padding. Its half-length a is 0.36 m, its half-width b is 0.35 m, and its circumscribed radius is √(a²+b²) = 0.5021 m. The world collision boxes put the inside faces of the hallway walls at y = ±0.90 m. At the original box, the upper lane is 1.225 m wide. An axis-aligned center can pass with positive padded clearance for 0.025 < y < 0.55 m. For a heading θ the lateral extent becomes a|sin θ| + b|cos θ|. At any heading a center between y ≈ 0.177 and 0.398 m at the box has room for the entire circumscribed disk. Doorways, box ends and turns require a two-dimensional calculation.

For each pose, the analysis rotates all four rectangle vertices about the base and translates them into the obstacle frame. It reads the wall and furniture collision poses and sizes from the SDF and includes the spawned fixture. The four separating axes of the two rectangles give the signed penetration when they intersect. For disjoint rectangles it computes the smallest Euclidean vertex-to-edge distance in either direction. The study desk's cylindrical legs are enclosed by its projected tabletop. The analysis includes outward-rounded world-aligned bounds for the ten external Fuel models, including the chairs and sofa. The can uses a conservative enclosure for mesh up-axis conversion. The 2-D projection is conservative for furniture at different heights.

Each trace contains global plan coordinates and orientations, chosen DWB local rollouts when subscribed, both raw costmaps, published footprints, scan and depth samples, odometry, velocity commands, map-to-odom estimates, mission state, and independent world poses from Gazebo dynamic pose messages. For NavFn we check both message orientations and the tangent of successive nonzero path segments. Most intermediate NavFn quaternions contain zero yaw. The analysis also expresses the tangent path in odom using the live map-to-odom estimate. Local rollouts already use odom. Gazebo world poses supply executed headings and positions directly.

For interpolated position and unwrapped heading, the movement of any footprint point is at most |Δposition| + 0.5021|Δheading|. Subdivision limits this to 0.01 m per step. Subtracting half the largest step gives the reported lower bound along that interpolation. Nearly touching plans are refined further. An additional bound for the executed path uses the actual interval between adjacent Gazebo samples and the configured 0.65 m/s and 1.5 rad/s ceilings. It subtracts half the interval times (0.65 + 0.5021 × 1.5) from the smaller endpoint clearance. The latter bound assumes those speed limits hold between samples. All margins are outside the padded rectangle. The project specifies no additional stand-off distance for this fixture. [Analysis implementation](../../../scripts/validation/analyze_routing.py)

The baseline uses NavFn A* with 0.75 m planner goal tolerance. DWB runs behind RotationShim, samples 20 forward speeds and 25 angular speeds, and predicts 1.7 s ahead at 0.05 m / 0.025 rad granularity. It allows 0–0.65 m/s forward and up to 1.5 rad/s yaw, with 2.5 m/s² and 3.2 rad/s² acceleration. The downstream smoother caps forward commands at 0.50 m/s. The original BaseObstacle critic scores center positions on the inflated grid. The 0.55 m inflation radius and 3.5 cost scaling factor create a preference around the observed cells. The inflation radius alone does not guarantee 0.55 m of footprint clearance. The missing and mislocated observations explain why increasing critic weights did not resolve this case. [Nav2 DWB reference](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/) · [Nav2 inflation layer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/)

The launcher and mission runner reuse the existing adapter and kernel. A cold-start trace showed that Nav2 exposed the action endpoint before the navigation lifecycle manager activated its nodes. The standalone runner now queries `/lifecycle_manager_navigation/is_active` immediately before sending the mission and clears stale coordinator readiness on a later unready status. [Early rejection log](release_patrol/launch.log) · [Runner](../../../scripts/run_autonomous_mission.py)

## Controlled planner comparisons

All these runs use the corrected depth cloud. The delayed fixture waited 99 seconds before the guarded goal so idle physics drift gave the base a different entry heading. Each configuration used a new Gazebo world. Negative margins denote footprint overlap on at least one proposed global path. DWB's chosen trajectory and the actual drive are assessed separately.

| Delayed-start configuration | Worst global tangent margin, map | Executed interpolated lower bound |
|---|---:|---:|
| NavFn, corrected global and local observations | −0.081 m | +0.142 m |
| NavFn, 0.90 m global inflation reaching the center of the empty corridor | −0.029 m | +0.102 m |
| Same inflation, with a 0.55 m global robot radius enveloping the padded rectangle | −0.073 m | +0.052 m |
| Smac 2D with the corrected observations | −0.108 m | +0.033 m |
| Forward-only Smac Hybrid-A* with the corrected observations | +0.085 m | +0.131 m |

Hybrid-A* produced positive clearance in the delayed corridor. It failed the kitchen patrol waypoint. With the original 0.75 m planner tolerance, the coordinator rejected a nominal Nav2 success 0.36 m from the goal. With a 0.05 m tolerance, the planner exhausted its search near the same waypoint. Differential-drive lattice planning and a validated grid fallback also failed the patrol. These configurations are experimental evidence. [Rejected arrival](final_patrol/launch.log) · [Tight endpoint trial](accepted_patrol/mission.json) · [Lattice trial](lattice_patrol/mission.json) · [Fallback trial](fallback_patrol/mission.json)

With corrected observations, NavFn's *initial* routes cleared the original, delayed, mirrored and larger fixtures. Unconditional 1 Hz replanning later replaced some of them with corner-cutting paths. The candidate behavior tree uses Nav2's goal-change and path-validity conditions to retain a usable route in transit. Within 1.1 m of a waypoint, it replans from the current pose at the original frequency. This is one full DWB rollout at the configured 0.65 m/s × 1.7 s. On the study route, retaining the plan all the way to the waypoint gave a 0.011 m padded-footprint overlap at the doorway and a 0.351 m final Gazebo error. Refreshing the approach improved the executed interpolated minimum to +0.066 m and the independent goal error to 0.224 m. [Validity-only study](validity_study/geometry.json) · [Approach-replanning study](approach_study/geometry.json)

The candidate keeps NavFn, RotationShim, DWB, the original footprint, both inflation layers, controller velocity limits, obstacle weights, collision-monitor polygons, existing recovery behavior, and the existing arrival and E-stop checks. The new runtime changes are in the simulation sensor bridge, the global depth observation entry and the navigation behavior tree. The sensor bridge also handles an invalid executor context during shutdown while propagating errors from an active context. The launcher sends its initial shutdown signal to its owned leaders so ROS launch can forward it to its children once. Its bounded group-level termination fallback is retained. [Sensor bridge](../../../ros2_ws/src/gracemo_bridge/gracemo_bridge/sensor_bridge.py) · [Simulation costmaps](../../../ros2_ws/src/gracemo_nav2/config/nav2_params.yaml) · [Navigation tree](../../../ros2_ws/src/gracemo_nav2/behavior_trees/navigate_to_pose_gracemo.xml) · [Launcher](../../../scripts/launch_full_simulation.sh)

## Reproduction commands

Run from the repository root in Bash with the Jazzy overlay built. Each case requires a new output directory. The runner starts and stops its own headless Gazebo world, creates one box, checks scene coordinates and launches the telemetry recorder and guarded ROS coordinator smoke mission. The corridor fixtures exercise the ROS navigation pipeline. The patrol below exercises the adapter and Rust kernel as well.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
work=$(mktemp -d)
python3 scripts/validation/run_routing_case.py "$work/corridor" --box 2 -.5 .35 .35
python3 scripts/validation/run_routing_case.py "$work/delayed" --box 2 -.5 .35 .35 --settle 99
python3 scripts/validation/run_routing_case.py "$work/mirrored" --box 2.35 .50 .30 .40
python3 scripts/validation/run_routing_case.py "$work/larger" --box 2.6 -.52 .45 .40
python3 scripts/validation/run_routing_case.py "$work/study" --box 4.15 2.25 .45 .45 --goal 4.5 3
python3 scripts/validation/run_routing_case.py "$work/blocked" --box 2 0 .35 1.65
for case in corridor delayed mirrored larger; do
    python3 scripts/validation/analyze_routing.py "$work/$case" --expect arrive --check-global > "$work/$case/geometry.json"
done
python3 scripts/validation/analyze_routing.py "$work/study" --expect arrive --check-global > "$work/study/geometry.json"
python3 scripts/validation/analyze_routing.py "$work/blocked" --expect blocked > "$work/blocked/geometry.json"
python3 -m pytest -q tests/ros2/test_gazebo_depth_frame.py tests/ros2/test_routing_geometry.py tests/ros2/test_standalone_readiness.py
```

The strict open-room global check returns 1 on the recorded final run. Dropping `--check-global` checks final arrival and executed/local geometry alone. The smoke process returns 1 on the expected navigation failure in the blocked case. The separate geometry and terminal-state acceptance command returns 0 when the blockage test passes. Analyzing an original failed case without `--expect` writes diagnostic measurements.

For the fresh-start original reproduction, overlay the original installed sensor bridge, parameters and behavior tree while retaining the current coordinator, launchers and user changes. These commands target new files under the temporary directory and leave the worktree intact.

```bash
base=a7945a8dbeb387a53cca10bca4ad7071784088ab
mkdir -p "$work/original"
cp -rL ros2_ws/install/gracemo_nav2 ros2_ws/install/gracemo_bridge "$work/original/"
git show "$base:ros2_ws/src/gracemo_nav2/config/nav2_params.yaml" \
    > "$work/original/gracemo_nav2/share/gracemo_nav2/config/nav2_params.yaml"
git show "$base:ros2_ws/src/gracemo_nav2/behavior_trees/navigate_to_pose_gracemo.xml" \
    > "$work/original/gracemo_nav2/share/gracemo_nav2/behavior_trees/navigate_to_pose_gracemo.xml"
git show "$base:ros2_ws/src/gracemo_bridge/gracemo_bridge/sensor_bridge.py" \
    > "$work/original/gracemo_bridge/lib/gracemo_bridge/sensor_bridge"
AMENT_PREFIX_PATH="$work/original/gracemo_nav2:$work/original/gracemo_bridge:$AMENT_PREFIX_PATH" \
    python3 scripts/validation/run_routing_case.py "$work/original_fresh" --box 2 -.5 .35 .35
python3 scripts/validation/analyze_routing.py "$work/original_fresh" > "$work/original_fresh/geometry.json"
```

The variant and safety probes were run sequentially for the final evidence. For the standalone regression the launcher and mission ran as separate commands to preserve the prior patrol evidence file. The launcher owned the local kernel on port 7780. Start the launcher in terminal one. Use the same private domain and partition in terminal two. Shut down the owned launcher after the probes.

```bash
# Terminal one
export ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=vira-patrol-study-91 GZ_IP=127.0.0.1
bash scripts/launch_full_simulation.sh --no-gui

# Terminal two, from the repository root
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_PARTITION=vira-patrol-study-91 GZ_IP=127.0.0.1
out=$(mktemp -d)
python3 scripts/run_autonomous_mission.py --timeout 300 --out "$out/mission.json"
sleep 2
python3 scripts/validation/check_idle_zero.py > "$out/idle.json"
python3 scripts/run_gazebo_estop_smoke.py --out "$out/estop.json"
```

## Standalone and safety regressions

The cold-start adapter → Rust kernel → ROS 2 → Nav2 → Gazebo patrol completed **11/11 waypoints across five rooms in 184.7 s**. The navigation manager logged all nodes active before the coordinator received the mission. There were zero startup goal rejections in this run. The coordinator and kernel recorded mission ID `standalone-05c43830-5453-4a53-abb2-1c7445940844`. The runner retrieved the kernel's terminal `MISSION_COMPLETED` event using dispatch trace `eb8970c2-2a89-4c89-b264-07fd0957df9d`. The launcher shut down its owned processes without a recorded traceback or child-process death error. [Patrol evidence](active_patrol/mission.json) · [ROS launch log](active_patrol/launch.log) · [Launcher log](active_patrol/launcher.log)

The moving E-stop test observed a latched stop, rejection of a subsequent mission, explicit reset and zero held after reset. Maximum measured post-stop displacement was **0.0203 m**. In the three-second idle check, all six components were zero on 58 final commands, 61 mux commands and 60 smoothed commands. The mux reported no active source and a held zero. The observer waits for DDS discovery on every monitored stream before starting its common measurement window. Earlier observers that sampled during discovery are retained in the evidence directory. [E-stop checks](active_patrol/estop.json) · [Idle-zero checks](active_patrol/idle.json)

## Remaining defects and release scope

- **Pre-positioned corridor obstacles.** The exact fixture, delayed entry, mirrored box and larger box each passed the goal, oriented global-path, executed footprint and DWB rollout gates in the recorded worlds. These are fixture-specific simulation results. NavFn still has no general swept-polygon guarantee.
- **Open room.** The two-stage mission reached the goal with 0.089 m minimum driven clearance. The worst hypothetical tangent along a proposed path overlapped by 0.021 m in the nominal map; using the measured localization transform reduced the odom-space lower bound to −0.00027 m. The stronger global-plan gate remains open even though all selected DWB rollouts passed.
- **Obstacle appearing during motion.** An extra box was inserted at (2.0, −0.40) after DWB started. The final configuration reached the goal. The measured padded polygon overlapped the box by 0.00055 m at its closest sampled pose, and a proposed DWB rollout had a −0.024 m lower bound. The original plan predated insertion. This run fails clearance independently of the original plan. The padded-envelope overlap alone does not establish physical chassis contact. [Late insertion](release_late/geometry.json)
- **Blocked route.** Clearing obstacle layers can briefly yield a plan through the barrier. The local safety path and actual trajectory retained clearance and the mission terminated within the deadline. The bounded-failure gate passes; clearing and replanning still need coverage for new obstacles near a moving robot.
- **Scope of verification.** Eight focused unit tests passed, including coordinate axes, self-exclusion, signed rectangle distance, rotation sweep, yaw wrap, shutdown and lifecycle readiness. Both changed ROS packages built against the installed workspace dependencies. The earlier seeded 50-mission campaign was not repeated on this revision. The results in this report cover Gazebo. No physical robot, motor parameters or physical navigation profiles were exercised. [Unit log](unit.log) · [Build log](build.log)

The capture, fixture runner, geometry analyzer and idle observer are under [scripts/validation](../../../scripts/validation/). Each completed evidence directory contains the mission and launch records; recorded navigation cases also include compressed time-series data. The raw grids use unsigned cost bytes, zlib compression and base64 encoding. A missing local-plan subscription in the first legacy trace accounts for the zero recorded local plans in that trace. Baseline all-rejection counts come from DWB's own log. Abandoned configurations and interrupted runs are retained as investigation evidence. Only the named runs above support the release assessment.
