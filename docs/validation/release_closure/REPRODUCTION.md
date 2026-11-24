# Reproduction

From the repository root in a ROS 2 Jazzy and Gazebo Harmonic development environment, use the following commands. The private runner uses a separate DDS domain, Gazebo partition and isolated simulation kernel. Choose fresh output directories. Each directory contains the exact generated candidate.yaml, scenario parameters and mission IDs.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
out=$(mktemp -d)
colcon --log-base "$out/log" build --base-paths scripts/validation/planner_fixture_barrier --build-base "$out/build" --install-base "$out/install"
source "$out/install/local_setup.bash"
python3 scripts/validation/dynamic_cases.py "$out/patrol" --regression patrol --mission-timeout 300 --approach
python3 scripts/validation/dynamic_cases.py "$out/margin" --regression patrol --mission-timeout 300 --planner-cell-margin
python3 scripts/validation/analyze_routing.py "$out/margin"
python3 scripts/validation/replay_global_veto.py docs/validation/release_closure/patrol_ns/trace.jsonl.gz
c++ -std=c++17 scripts/validation/mesh_triangles.cc -o "$out/mesh_triangles" $(pkg-config --cflags --libs gz-common5-graphics)
python3 scripts/validation/chair_mesh_replay.py docs/validation/master_repair/doorway_dense/truth.jsonl.gz "$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/chair/3/model.sdf" "$out/mesh_triangles"
python3 -m pytest -q tests/ros2/test_approach_guard.py tests/ros2/test_generation_guard.py tests/ros2/test_clearance_oracle.py
```

The first two historical runs used earlier revisions of the guard. The first omitted both the drift-settling pause and the nanosecond freshness fix. The second included the pause only. Replay their saved traces for the reported measurements. The current command exercises the final source. A repeat can encounter a different first veto because localization and controller callbacks vary. The legacy baseline and private regression commands are in the [prior reproduction report](../master_repair/REPRODUCTION.md).

For the optimized-library repeat, use a new overlay in the same shell and keep every YAML setting identical to the margin trial.

```bash
colcon --log-base "$out/opt-log" build --base-paths scripts/validation/planner_fixture_barrier --build-base "$out/opt-build" --install-base "$out/opt-install" --cmake-args -DCMAKE_BUILD_TYPE=Release
source "$out/opt-install/local_setup.bash"
python3 scripts/validation/dynamic_cases.py "$out/optimized" --regression patrol --mission-timeout 300 --planner-cell-margin
python3 scripts/validation/analyze_routing.py "$out/optimized"
python3 scripts/validation/closure_results.py docs/validation/release_closure
```
