# Reproduction from the repository root

Use a fresh directory. Private profiles require the test plugins sourced into the ROS environment.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
out=$(mktemp -d)
colcon --log-base "$out/log" build --base-paths scripts/validation/planner_fixture_barrier --build-base "$out/build" --install-base "$out/install"
source "$out/install/local_setup.bash"
case_runner=scripts/validation/dynamic_cases.py
python3 "$case_runner" "$out/control" --regression patrol --mission-timeout 300
python3 "$case_runner" "$out/doorway" --doorway-trial --dense-truth --regression patrol --mission-timeout 300
python3 "$case_runner" "$out/turning" --turning-corridor --regression patrol --mission-timeout 300
python3 "$case_runner" "$out/budget" --goal-budget --regression patrol --mission-timeout 300
python3 scripts/validation/clearance_oracle.py docs/validation/dynamic_obstacles/baseline_patrol_review/trace.jsonl.gz
python3 scripts/validation/clearance_oracle.py "$out/doorway/truth.jsonl.gz"
for i in 0 2 7; do python3 "$case_runner" "$out/moving-$i" --master-recovery --index "$i" --seed 42; done
for i in 4 9; do python3 "$case_runner" "$out/plan-$i" --master-recovery --index "$i" --synchronize-replan; done
for i in 4 9; do python3 "$case_runner" "$out/fence-$i" --planner-fence --index "$i" --synchronize-replan; done
python3 "$case_runner" "$out/temporary" --master-recovery --temporary
for s in original larger mirrored study blocked patrol; do python3 "$case_runner" "$out/static-$s" --master-recovery --regression "$s" --mission-timeout 300; done
python3 "$case_runner" "$out/clear-front" --clearance-recovery
for s in original study patrol; do python3 "$case_runner" "$out/clear-$s" --clearance-recovery --regression "$s" --mission-timeout 300; done
for s in NAVIGATING OBSTACLE_DETECTED SAFETY_STOP COSTMAP_UPDATE ROUTE_REASSESSMENT REPLANNING PATH_VALIDATION RESUMING; do python3 "$case_runner" "$out/stop-$s" --master-recovery --interrupt-at "$s"; done
python3 "$case_runner" "$out/rect-front" --rectangular-guard
for s in original study larger mirrored; do python3 "$case_runner" "$out/rect-$s" --rectangular-guard --regression "$s"; done
python3 "$case_runner" "$out/rect-patrol" --rectangular-guard --regression patrol --mission-timeout 300
python3 "$case_runner" "$out/rect-cross-mirror" --rectangular-guard --index 7
python3 "$case_runner" "$out/restarts" --restart-trial
python3 "$case_runner" "$out/idle" --regression safety --master-recovery
python3 scripts/validation/analyze_dynamic.py "$out/moving-0"
python3 scripts/validation/analyze_routing.py "$out/doorway"
python3 scripts/validation/master_results.py docs/validation/master_repair
python3 -m pytest -q tests/ros2/test_clearance_oracle.py tests/ros2/test_dynamic_routing_guard.py tests/ros2/test_routing_geometry.py tests/ros2/test_standalone_readiness.py tests/ros2/test_gazebo_depth_frame.py tests/ros2/test_interaction_bridge.py tests/contract tests/integration/test_interaction_pipeline.py
python3 -m pytest -q tests/ros2/test_generation_guard.py
python3 -m pytest -q tests/e2e/test_end_to_end_interaction_loop.py
cargo test --offline --manifest-path kernel/Cargo.toml -p gracemo-kernel --lib
```

The seed determines dimensions and initial conditions. DDS delivery, physics and controller scheduling vary. Use the JSON command arrays where available for each historical run. The final harness additionally rejects expired planning barriers, so repeating an invalid earlier experiment can produce an explicit fixture failure.
