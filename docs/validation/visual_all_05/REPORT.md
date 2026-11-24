# VIR-VISUAL-ALL-05 desktop integration

**Release decision — HOLD.** This execution adds the first live run of a visual-odometry-driven RTAB-Map graph in the supported launcher. Mapping failed to cover the route. The prior AMCL navigation path still reached the monitor observation pose. See [goal receipt](live01/visual_goal.json), [backend measurements](backend.json), and [launcher lifecycle](live01/process_lifecycle_evidence.json).

## Architecture and dependency choice

Gazebo RGB/depth and calibration → ROS image bridge → RTAB-Map RGB-D odometry → namespaced odometry topic → RTAB-Map graph database. In parallel, existing perception → Rust kernel → MNSE/K → authorized adapter → coordinator → AMCL/Nav2 → Gazebo. PySide6 observes the established pipeline. The added processes publish no navigation TF and receive no Gazebo truth. The original mapping fixture can still run in its existing mode.

The installed ROS stack included LiDAR slam_toolbox. RTAB-Map's packaged Jazzy odometry and SLAM nodes support RGB-D, keyframes and a persistent SQLite graph. They ran with their native dependencies in a temporary user-space prefix. The existing Python estimator remains a separate advisory estimator. No comparison of localization accuracy between the two is available from this run.

## Results and limits

Six hundred thirty odometry updates produced 89 positive and 541 zero quality values. Eight keyframes span simulation seconds 0.2–13.3. Registration logged too few inliers after bundle adjustment. The SQLite integrity check passed. Both ground-truth frame parameters were empty and all stored ground-truth blobs contained zero bytes throughout. Link types 0 and 1 occurred once each. This count alone cannot establish a revisited location or quantify optimization improvement. No independent truth trajectory was recorded for this particular backend run. No new translation or rotation RMSE is available.

| Gate | ALL-04 → ALL-05 | Evidence or missing measurement |
|---|---|---|
| RGB-D / object localization | PARTIAL → PARTIAL | Live monitor goal passed. Invalid principal points, zero focal lengths and malformed replacement calibration now reject projection. Condition matrix incomplete. |
| VO | IMPLEMENTED BUT NOT ACCEPTED → IMPLEMENTED BUT NOT ACCEPTED | Prior patrol 1.6555 m / 26.68° / 35.68%. New backend lost registration. |
| RGB-D SLAM | MISSING → PARTIAL | Live graph with eight keyframes and clean SQLite integrity check. |
| Loop closure | MISSING → NOT TESTED | No independently verified revisit or before/after error. |
| Persistence / restart | PARTIAL → PARTIAL | New database saved. No RTAB-Map restart trial completed. Earlier advisory backend had one successful route toward ten. |
| Moving person | PARTIAL → PARTIAL | Earlier recall 0.710, IDF1 0.226, seven ID switches. No new actor trial. |
| Face detection / Re-ID | PARTIAL → PARTIAL | No new multi-condition or independent cross-view evaluation. |
| Face recognition | BLOCKED → BLOCKED | No consented held-out evaluation supplied. |
| MNSE / K | PARTIAL → PARTIAL | Live goal used K location. No new correction/deletion lifecycle. Audit still retains original payload. |
| PySide6 | PARTIAL → PARTIAL | Launcher started desktop offscreen. Human walkthrough outstanding. |
| Sustained benchmark | NOT TESTED → NOT TESTED | Full-stack latency, processing rate and memory-growth gates outstanding. |
| Recovery E-stop | NOT TESTED → NOT TESTED | Requires measured stop through loss, relocalization, reset and authorized resumption. |
| Full integration | PARTIAL → PARTIAL | Object route succeeded; simultaneous full-system acceptance outstanding. |

## Reproduce

From the repository root with ROS Jazzy, the workspace overlay and the RTAB-Map odom/slam packages available:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
out=docs/validation/visual_all_05/repeat01
VIRA_RTABMAP_DATABASE="$PWD/$out/map.db" VIRA_TEST_DEPTH_10HZ=1 VIRA_DETECTION_HZ=10 python3 scripts/validation/launcher_recovery.py "$out" --vision --visual-goal
python3 scripts/validation/summarize_rgbd_backend.py "$out"
PYTHONPATH="$PWD:$PWD/adapters/sdk:$PYTHONPATH" python3 -m pytest -q tests/test_perception_pipeline.py tests/test_edge_vision.py tests/ros2/test_gazebo_depth_frame.py
```

Use a new output directory each time. For this machine the unprivileged package prefix was /tmp/vira05-overlay/opt/ros/jazzy, prepended to AMENT_PREFIX_PATH with its lib and lib/x86_64-linux-gnu prepended to LD_LIBRARY_PATH. The mapping log is live01/1_gracemo_rgbd_slam.log. Raw database and launch logs are local. [Component results](perception-tests.txt) cover 31 tests. Files changed are the mapping launch file, run_gracemo.sh, launcher_recovery.py, depth_estimator.py, test_perception_pipeline.py, and summarize_rgbd_backend.py. This is a partial programme record.
