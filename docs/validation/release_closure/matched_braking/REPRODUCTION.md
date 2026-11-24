# Matched braking geometry trial

The private global planner and guard already reserved a margin of (1/√2 + 1/4) × 0.05 m. The DWB critic previously tested its braking sweep without that reserve. This trial adds √2 times the reserve to the critic's braking projections, matching the scalar corner displacement used downstream. The legacy plugin default remains zero. Observation age, braking discretization, rollouts and available world observations can still differ between DWB and the guard. Agreement on one margin does not guarantee agreement on a command.

Run from the repository root with a fresh output path. The runner creates an isolated simulation kernel, DDS domain and Gazebo partition.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
out=$(mktemp -d)
colcon --log-base "$out/log" build --base-paths scripts/validation/planner_fixture_barrier --build-base "$out/build" --install-base "$out/install" --cmake-args -DCMAKE_BUILD_TYPE=Release
source "$out/install/local_setup.bash"
python3 scripts/validation/dynamic_cases.py "$out/patrol" --match-braking --regression patrol --mission-timeout 300
python3 scripts/validation/analyze_routing.py "$out/patrol"
python3 -m pytest -q tests/ros2/test_approach_guard.py tests/ros2/test_clearance_oracle.py tests/ros2/test_generation_guard.py
```
