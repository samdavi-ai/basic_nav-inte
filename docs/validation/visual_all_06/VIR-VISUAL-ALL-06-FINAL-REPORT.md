# VIR-VISUAL-ALL-06 | Desktop Gazebo registration and recovery

**Release recommendation — HOLD.** The original five-room AMCL/Nav2 patrol passed 11/11. A clean map from the textured monitor route passed five independent near-origin restart tests. On the moving monitor route, raw visual odometry covered 99.85% of received RGB frames but missed the accuracy targets at 0.567 m / 6.38° RMSE. The original patrol had only 2.89% visual coverage. AMCL remains the navigation localization source.

## Acceptance matrix

The release assessment covers the original desktop apartment. Private-scene results are identified separately so that success on one route is traceable to the world, camera configuration and saved map used.

| Capability | Release status | Measured evidence |
|---|---|---|
| RGB-D input | IMPLEMENTED AND ACCEPTED | Live RGB, floating-point depth and camera matrices were recorded. The matched-resolution profile supplied compatible RGB and depth dimensions. [Original sensor measurements](baseline/evaluation.json) |
| Registration | IMPLEMENTED BUT NOT ACCEPTED | The original apartment lost visual registration for most of both the monitor route and patrol. [Patrol capture](original-patrol01/evaluation.json) |
| Visual odometry | IMPLEMENTED BUT NOT ACCEPTED | The private textured return met all three trajectory limits. The original five-room patrol had 2.89% availability. [Private return](texture-roundtrip02/evaluation.json) |
| Keyframes | PARTIAL | Live color/depth nodes and 3D features persist. Geometric validation covers the optimized subset on the diagnostic route. [Graph coverage](texture-roundtrip02/graph-evaluation.json) |
| RGB-D SLAM | PARTIAL | A connected graph was independently validated in the private scene. Whole-apartment coverage remains open. [Graph](texture-roundtrip02/graph-evaluation.json) |
| Loop closure | IMPLEMENTED AND ACCEPTED | A measured 5.31 m excursion and return resulted in a 61-inlier constraint within 0.024 m / 0.76° of independent relative motion. [Constraint](texture-roundtrip02/graph-evaluation.json) |
| Pose-graph optimization | IMPLEMENTED AND ACCEPTED | On 67 paired nodes, translation RMSE fell from 0.185 to 0.062 m and rotation RMSE from 1.35° to 1.18°. [Paired errors](texture-roundtrip02/graph-evaluation.json) |
| Map persistence | IMPLEMENTED AND ACCEPTED | A complete optimized graph and dictionary were saved on orderly shutdown, checked and loaded into a fresh process. [Map](subtle-roundtrip01/backend.json) · [Reload](subtle-occlusion01/relocalization.json) |
| Visual relocalization | PARTIAL | The clean monitor-route map passed all five independent near-origin tests. The original apartment baseline passed 0/5. Arbitrary starting locations remain unverified. [Five clean-map restarts](#five-clean-map-restarts) |
| AMCL regression | IMPLEMENTED AND ACCEPTED | The original five-room regression passed 11/11. [Mission](original-patrol01/patrol.json) |
| Desktop performance | PARTIAL | The camera-rate route exceeded five graph updates/s in all 13 complete windows. Long-duration memory and queue-occupancy evidence remains open. [Windowed throughput](desktop-roundtrip01/performance.json) |
| Safety | IMPLEMENTED AND ACCEPTED | Gazebo idle-zero and active-navigation stop passed on the monitor route. The primary localization and velocity-control ownership remained with the accepted stack. [Idle](desktop-roundtrip01/idle_zero.json) · [Stop](desktop-roundtrip01/estop.json) |

## Baseline diagnosis

ALL-05 recorded 89 positive out of 630 log updates, eight keyframes, and no independent error measurement. ALL-06 repeated the route with default RTAB-Map settings, 10 Hz RGB and depth, and a passive observer. The new run logged 106 positive and 596 zero-quality updates. The independent observer captured 684 RGB frames over simulation seconds 0.2–68.5. Only 66 had both a non-lost OdomInfo and an odometry pose at the same stamp. Availability was 9.65%. The logs span a slightly longer period than the subscriber capture. A logged positive quality value and a non-lost pose are different measures.

The first observed loss at 0.5 s reported 19 features, zero matches and zero inliers. The corresponding warning reports 19 new features for a required 20. Other frames had 20 matches but only 18 inliers; bundle adjustment also reduced some sets below 20. Over the run there were 1,143 insufficient-inlier warnings, 54 insufficient-feature warnings and 74 post-adjustment warnings. Retries can produce several warnings per frame. These observations establish insufficient geometrically consistent matches. They do not uniquely attribute the losses to scene texture, motion or descriptor choice.

RGB and depth sensors share the same pose on head_link, 1.15 rad horizontal field of view and optical frame. The live bridge emitted 320×240 rgb8 and 160×120 32FC1 with scaled focal lengths (246.896 and 123.448 pixels) and centered principal points. Median valid-depth fraction was 89.78%. There were no duplicate or backwards RGB stamps. Two RGB stamps lacked an exactly coincident depth message. Median and p95 nearest-depth skew were zero and maximum nearest-neighbor separation was 100 ms. This last value is a missing partner, outside the estimator's 25 ms sync window. Calibration is generated by the simulator bridge; empirical pixel reprojection error remains unmeasured. See [full baseline distribution](baseline/evaluation.json).

## Trajectory and graph

Gazebo model pose is recorded on a separate, read-only subscription. The estimator uses only camera data, calibration and robot TF. The evaluator uses one rigid registration from the first valid matched base pose, fixed scale, and nearest truth within 50 ms. Each observed RGB stamp counts in availability. Errors use only valid matched stamps. Baseline RMSE on 66 matches was 0.0391 m and 0.308 degrees. The sparse valid subset cannot support a whole-route accuracy claim. Ten database keyframes ended at 25.3 s. Ground-truth frame parameters were empty and the stored ground-truth transforms were all zero. [Database measurements](baseline/backend.json).

The mapper repeatedly attempts visual_odom-to-base TF lookup before falling back to stamped odometry. This is a separate mapping delay. The failed feature registration originates in the upstream odometry process. Navigation TF publishing remains disabled for both visual nodes.

## Controlled experiments

The optional 2× trial doubled RGB and depth image dimensions in the private world while preserving pose, field of view and inlier thresholds. It logged 276 positive and 235 zero-quality updates and saved 26 keyframes. Readiness failed before motion: map_server's change_state response timed out, AMCL pose was unavailable and only 2/10 Nav2 lifecycle nodes activated. The readiness gate correctly withheld dispatch. Its 33.4% observer availability and lower stationary error cannot establish improved route performance. [Measurements](resolution2x/evaluation.json).

## Map persistence and restart methodology

The reload path checks the specified SHA-256, SQLite integrity and presence of calibrated RGB-D records. It opens the input read-only, backs it up to a fresh trial database, clears the saved localization pose and runs nonincremental localization from an origin prior. The private world starts 0.25 m offset. The origin prior is never counted as a visual match. The observer requires a reported closure/proximity constraint, at least 20 visual inliers, a same-stamp localization pose and independent truth. The trial evaluator uses the mapping session alignment, with no per-trial recentering. The format and intrinsics tests added after these runs are documented below. Other camera models remain unverified.

## Reproduction and evidence

From repository root, with the Jazzy RTAB-Map packages installed or the ALL-05 temporary prefix available:

```bash
export VIRA_RTABMAP_PREFIX=/tmp/vira05-overlay/opt/ros/jazzy
out=docs/validation/visual_all_06/repeat-baseline
bash scripts/validation/run_registration_trial.sh "$out"
python3 scripts/validation/evaluate_registration.py "$out/registration.jsonl"
python3 scripts/validation/summarize_rgbd_backend.py "$out"
VIRA_TEST_RGBD_2X=1 bash scripts/validation/run_registration_trial.sh docs/validation/visual_all_06/repeat-2x
export VIRA_RTABMAP_INPUT="$PWD/$out/map.db"
export VIRA_RTABMAP_SHA256="$(sha256sum "$VIRA_RTABMAP_INPUT" | cut -d' ' -f1)"
for i in 1 2 3 4 5; do
  VIRA_TEST_OFFSET_START=1 bash scripts/validation/run_registration_trial.sh "docs/validation/visual_all_06/repeat-reload$i"
  python3 scripts/validation/summarize_relocalization.py "$out" "docs/validation/visual_all_06/repeat-reload$i"
done
```

Each run requires a new directory. Combined RTAB-Map logs, SQLite databases and full frame-level measurements stay local under those directories. Per-run summaries, lifecycle records and available mission receipts are compact evidence. Component regressions: 32 perception/depth/runner tests passed and 19 Rust workspace tests passed. See [Python output](component.txt) and [Rust output](rust.txt). Changed files include the mapping launch file, run_gracemo.sh, launcher_recovery.py, summarize_rgbd_backend.py and the registration recorder, trial launcher and two independent evaluation scripts under scripts/validation.

## Five independent baseline-map restarts

| Fresh launch | Graph messages | Verified visual matches | AMCL monitor mission |
|---|---:|---:|---|
| [1](reload01/relocalization.json) | 62 | 0 | Reached |
| [2](reload02/relocalization.json) | 78 | 0 | Withheld at readiness (2/10 lifecycle nodes) |
| [3](reload03/relocalization.json) | 57 | 0 | Reached |
| [4](reload04/relocalization.json) | 55 | 0 | Reached |
| [5](reload05/relocalization.json) | 53 | 0 | Reached |

All five loaded the same source database, SHA-256 `ff70f808f132c960f1e0aba6d4d56dcdf7f6937b2880c998f92a1de70ba234b4`. None reported a closure or proximity ID. Visual relocalization was **0/5**. Translation and rotation errors for a successful relocalization are undefined.

## Descriptor and revisit experiment

The SIFT trial changed the odometry feature type and added an authorized return leg after the monitor goal. AMCL reached the monitor and completed both return waypoints, ending at approximately (0.01, 0.17) m according to the coordinator. Across both legs the backend logged 0 positive and 1,459 zero-quality updates. The observer counted one non-lost initial pose among 1,443 RGB frames (0.069%). Its alignment residual is uninformative. The database held one node and zero links. The registration warnings included 1,141 insufficient-feature failures. SIFT was rejected for this scene. [Sensor evaluation](sift-revisit01/evaluation.json) · [Graph](sift-revisit01/graph-evaluation.json) · [Return mission](sift-revisit01/revisit.json)

Over 158.0 wall seconds the observer received 9.08 RGB and 9.05 odometry messages/s, but only 0.0063 matched non-lost poses/s. Median/p95/maximum odometry estimation time was 15.14/20.96/33.74 ms. Median CPU usage was 35.9% of one core for odometry and 10.5% for SLAM. Sampled peak resident memory was 300.3 and 257.6 MiB respectively. These timings do not include end-to-end rendering or message transit. Internal queue occupancy was not measured. [Wall-clock performance](sift-revisit01/performance.json)

The SIFT run passed idle-zero and active-navigation E-stop. Post-stop output was zero, displacement was 0.0219 m and final-second maximum speed was 0.0149 m/s. The observer logged an external ROS shutdown after capture. Its shutdown handler now explicitly handles that exception and closes its subscriptions. [Idle commands](sift-revisit01/idle_zero.json) · [Stop measurements](sift-revisit01/estop.json)

For the original baseline graph, three nodes could be compared before and after optimization. Translation RMSE was 0.02030 m in both comparisons and rotation RMSE was 0.111° in both to three decimals. The three type-1 edges connected poses separated by 0.017–0.027 m. These edges alone do not establish a departure and verified return. [Baseline graph](baseline/graph-evaluation.json)

## Frame-to-frame optical flow in the original apartment

The private candidate used RTAB-Map frame-to-frame odometry and optical-flow correspondences with the same cameras and inlier requirement. Across 1,355 RGB frames, 630 poses could be scored (46.49%). Translation RMSE was **6.38 m** and rotation RMSE **101.40°** on those poses. The first 510 estimator messages included 509 non-lost updates, illustrating why early availability would have given a misleading result. By simulation second 10, Gazebo's base remained within 0.03 m of the start while the aligned visual estimate had moved roughly 2 m and rolled about 31°. All captured RGB stamps had a matching depth stamp. The mapper persisted 62 nodes connected only by 61 neighbor edges. Optimizing those nodes produced no material improvement. The monitor and return missions completed under AMCL. [Trajectory measurements](flow-revisit01/evaluation.json) · [Graph](flow-revisit01/graph-evaluation.json) · [Camera views from the saved map](flow-revisit01/scene-samples.jpg)

The backend logged 631 positive and 738 zero-quality updates. Estimation took 4.78/12.16/28.37 ms (median/p95/max). Over 150 wall seconds, RGB input averaged 9.02 FPS, non-lost matched poses 4.20 FPS and graph updates 0.41/s. Median CPU was 16% of a core for odometry and 11.5% for SLAM. Peak RSS was 224.7 and 331.8 MiB respectively. The recorder exited cleanly. [Resource evidence](flow-revisit01/performance.json) · [Idle-zero](flow-revisit01/idle_zero.json) · [E-stop](flow-revisit01/estop.json)

## Planar motion and scene observability

The private planar candidate added only `Reg/Force3DoF=true` to frame-to-frame flow. Even before dispatch it accumulated 4.22 m / 41.62° RMSE on 42.66% of 783 observed RGB frames. Nav2 failed readiness (2/10 nodes, missing AMCL pose after the map server timed out sending its lifecycle response). The launch withheld the mission. A planar constraint alone did not correct the erroneous motion. [Trajectory](planar-flow01/evaluation.json) · [Readiness](planar-flow01/1_gracemo_readiness.log)

A controlled scene experiment kept the original camera, default feature registration, 20-inlier threshold, obstacle geometry, depth, lighting and navigation stack. It added deterministic, distinct albedo textures to the floor and wall materials in a temporary private world. During staging, the backend logged 559 positive updates, one zero update and 53 keyframes. Matched pose availability was 98.70%, translation RMSE 0.092 m and rotation RMSE 0.965°. Median feature count was 488. Over 60 wall seconds, the observer measured 8.65 matched non-lost poses/s. The same estimator's performance in the original apartment supports insufficient visual surface detail as a cause of registration failure. The existing semantic detector did not supply a unique authorized target at staging; its visual-goal test failed and this first run ended before the return. Comparison of graph keyframes showed slightly *higher* translation and rotation RMSE after optimization. [Visual-goal result](texture-revisit01/visual_goal.json) · [Odometry](texture-revisit01/evaluation.json) · [Graph](texture-revisit01/graph-evaluation.json) · [Performance](texture-revisit01/performance.json)

Saved-map loading now checks the tested database version (0.23.14) and stored camera resolution, intrinsics, distortion, rectification and projection against the private world's camera configuration. Original and doubled-resolution databases were accepted at the matching resolution and rejected at the other resolution. Four separate invalid calibration values were rejected. This compatibility test used local SQLite files. It does not measure empirical reprojection error. [Compatibility results](camera-compatibility.txt)

## Verified return constraint in the textured diagnostic world

On the second textured run, staging and the independent authorized return completed. The monitor request failed again because no unique target was available. The harness preserved the failed visual-goal result after independently testing the return, idle-zero and E-stop. RTAB-Map reported 1,261 positive updates and one zero, stored 119 nodes, and associated 52,225 positive-depth feature observations. Independent coverage was **99.76%**, with 1,238 scored poses, **0.175 m** translation RMSE and **1.30°** rotation RMSE. Translation median/p95 were 0.135/0.294 m and rotation median/p95 were 0.878/2.064°. The observed sequence contained one loss transition and two transitions into tracking including startup. [Odometry](texture-roundtrip02/evaluation.json) · [Saved camera images](texture-roundtrip02/scene-samples.jpg)

Node 101 matched node 12 after 94 simulation seconds. The independently recorded robot left the target area by 5.31 m and returned within 0.285 m of the earlier pose. RTAB-Map reported 61 visual inliers, a stored type-1 constraint, and three optimization iterations. The stored relative constraint differed from independent relative motion by **0.024 m and 0.76°**. Across the 67 comparable optimized keyframes, translation RMSE improved **0.185 → 0.062 m**, and rotation RMSE improved **1.35 → 1.18°**. The database includes additional non-comparable nodes. [Constraint and paired graph errors](texture-roundtrip02/graph-evaluation.json)

During 136 wall seconds, the observer measured 9.00 RGB frames/s, 8.98 matched non-lost poses/s and 0.85 graph updates/s. The mapper's configured detection interval is one second. Odometry estimation latency was 17.89/27.84/40.13 ms (median/p95/max). The configured topic queues held 10 messages, with synchronization queues of 5 for odometry and 10 for SLAM. These are configured limits; direct occupancy and continuous long-duration memory-growth measurements remain open. [Performance](texture-roundtrip02/performance.json) · [E-stop](texture-roundtrip02/estop.json) · [Idle-zero](texture-roundtrip02/idle_zero.json)

Despite SQLite integrity passing, the round-trip database lacked 27,555 distinct dictionary words referenced by stored features. The launch log showed odometry exiting but lacked a clean mapper exit. Fresh-process reload reported rebuilding 21,281 missing words from the loaded nodes. The supported launcher's fixed one-second shutdown interval allowed termination during saving. The launcher now grants its recorded private mapper a bounded additional save interval after the normal safety stop. The five diagnostic reloads use the same pre-fix database for comparability. [Map audit](texture-roundtrip02/backend.json) · [Reload log](texture-reload01/1_gracemo_rgbd_slam.log)

## Additional reproduction commands

Each destination below must be new. Run from the repository root. These commands require the same private desktop Gazebo environment as the baseline.

```bash
export VIRA_RTABMAP_PREFIX=/tmp/vira05-overlay/opt/ros/jazzy
root=docs/validation/visual_all_06
trial=scripts/validation/run_registration_trial.sh
VIRA_RTABMAP_DESCRIPTOR=sift VIRA_TEST_VISUAL_REVISIT=1 bash "$trial" "$root/repeat-sift"
VIRA_RTABMAP_OPTICAL_FLOW=true VIRA_TEST_VISUAL_REVISIT=1 bash "$trial" "$root/repeat-flow"
VIRA_RTABMAP_PLANAR=true VIRA_RTABMAP_OPTICAL_FLOW=true VIRA_TEST_VISUAL_REVISIT=1 bash "$trial" "$root/repeat-planar"
VIRA_TEST_SURFACE_TEXTURE=1 VIRA_TEST_VISUAL_REVISIT=1 bash "$trial" "$root/repeat-texture"
source_run="$root/repeat-texture"
python3 scripts/validation/evaluate_registration.py "$source_run/registration.jsonl"
python3 scripts/validation/evaluate_registration_graph.py "$source_run"
python3 scripts/validation/summarize_registration_performance.py "$source_run/registration.jsonl"
python3 scripts/validation/summarize_rgbd_backend.py "$source_run"
export VIRA_RTABMAP_INPUT="$PWD/$source_run/map.db"
export VIRA_RTABMAP_SHA256="$(sha256sum "$VIRA_RTABMAP_INPUT" | cut -d' ' -f1)"
for i in 01 02 03 04 05; do
    dest="$root/repeat-texture-reload$i"
    VIRA_TEST_SURFACE_TEXTURE=1 VIRA_TEST_STATIONARY_RELOAD=1 VIRA_TEST_OFFSET_START=1 bash "$trial" "$dest"
    python3 scripts/validation/summarize_relocalization.py "$source_run" "$dest"
done
```

A visual-goal refusal produces a nonzero return code even when an independent authorized return mission succeeds. The fixed-window stationary reload records for 20 wall seconds after launcher readiness. Relocalization requires a match to a keyframe ID stored in the source database. The evaluator uses independent truth solely for scoring. Primary navigation uses AMCL in every trial. [Component regressions repeated after harness changes](component-followup.txt)

## Independent textured-map reloads

All five fresh launches passed readiness and observed from a 0.25 m displaced start. All qualified matches referenced source database keyframes. Each trial used an origin prior, ignored the previous localization pose and was scored in the original mapping alignment. These stationary observations establish near-origin relocalization only. All five used the pre-fix map and its runtime dictionary repair.

| Trial | Graph messages | Qualified matches | Maximum position error (m) | Maximum rotation error (°) |
|---|---:|---:|---:|---:|
| [texture-reload01](texture-reload01/relocalization.json) | 28 | 3 | 0.0085 | 0.425 |
| [texture-reload02](texture-reload02/relocalization.json) | 36 | 4 | 0.0056 | 0.350 |
| [texture-reload03](texture-reload03/relocalization.json) | 33 | 3 | 0.0077 | 0.344 |
| [texture-reload04](texture-reload04/relocalization.json) | 46 | 4 | 0.0070 | 0.465 |
| [texture-reload05](texture-reload05/relocalization.json) | 29 | 3 | 0.0061 | 0.408 |

[Ground-truth and visual trajectories](observed-trajectories.png). On the textured return, the backend reported median/p95/maximum image-stamp delays of 50/78/110 ms for odometry and 1.182/1.270/1.354 s for mapping. These simulation-clock delays differ from computation time.


## Map-save repair and camera occlusion

A lower-contrast wall/floor trial produced 98.93% coverage, 0.129 m / 1.70° odometry RMSE, and optimized translation RMSE of 0.073 m across 77 graph nodes. The monitor still had no unique detection at staging. With the shutdown repair, the log recorded completion of the database save followed by a clean mapper exit. All dictionary words referenced by the 77 optimized nodes were present, compared with **21,281 missing** from the earlier textured graph. Features outside the retained optimized graph still referenced 8,479 absent words. The reload validator now rejects missing words on optimized nodes, missing keyframe IDs and inconsistent optimized-pose counts. The old databases can still be inspected offline. New runs of the reproduction commands create fresh databases. [Map audit](subtle-roundtrip01/backend.json) · [Saved-graph compatibility checks](graph-compatibility.txt)

A fresh launch loaded this map without a dictionary regeneration warning. A stationary robot started 0.25 m from the mapping origin. At simulation second 17.3, a solid Gazebo panel was placed across its view. The panel was removed at 22.9 s. The RGB stream remained live. All 49 odometry updates inside the covered interval reported loss. After removal, all 408 updates in the scored interval reported tracking. Time from the recorded scene-change stamp to the first lost update was 0.1 s; time from removal to the first non-lost update was 0.1 s. A geometrically qualified match to a saved node followed 2.6 s after removal. Service times are bracketed by camera timestamps at roughly 0.1 s resolution. All five qualified map matches met the position and orientation limits. This was a stationary visibility interruption. [Events and scored matches](subtle-occlusion01/relocalization.json) · [Mapper log](subtle-occlusion01/1_gracemo_rgbd_slam.log)

With only the floor textured, the existing semantic monitor goal succeeded, but odometry coverage fell to 42.41% with 0.328 m / 3.36° RMSE. The return mission, idle-zero and configured E-stop test passed. The command monitor measured a peak of 0.5 after the stop request, including time before latch confirmation. Drift was 0.0224 m (limit 0.15 m) and final-second speed magnitude was 0.0153 (limit 0.05). That trace does not distinguish a queued pre-latch command from one after latch. Later tests also record request-to-zero time and the post-confirmation peak. The floor-only trial tried disabling the mapper's odometry sensor synchronization option; visual-odom TF lookup warnings persisted and the change was reverted. [Floor-only odometry](floor-roundtrip01/evaluation.json) · [Monitor](floor-roundtrip01/visual_goal.json) · [Stop](floor-roundtrip01/estop.json)

To repeat the opacity test, generate a new map with `VIRA_TEST_SURFACE_TEXTURE=subtle` and `VIRA_TEST_VISUAL_REVISIT=1`. Set the saved map path and SHA-256 as above, then launch a new trial with `VIRA_TEST_SURFACE_TEXTURE=subtle VIRA_TEST_OFFSET_START=1 VIRA_TEST_STATIONARY_RELOAD=1 VIRA_TEST_VISUAL_OCCLUSION=1`. Score the fresh and mapping directories with `summarize_relocalization.py`.

## Effect of the monitor background

Preserving the original east and south walls behind the TV in the otherwise textured scene restored the authorized monitor goal. The return reached its destination, but odometry coverage was only 40.94%. RMSE of 0.040 m / 0.80° describes that sparse subset. E-stop latch confirmation arrived 2.18 ms after request, the first observed zero at 2.53 ms and the peak observed command between latch confirmation and reset was zero. [Odometry](background-roundtrip01/evaluation.json) · [Monitor receipt](background-roundtrip01/visual_goal.json) · [Stop timing](background-roundtrip01/estop.json)

A second variant added upper and lower decorative bands to those two walls and left their centers plain. Odometry coverage reached 99.78%, with RMSE of 0.168 m / 3.37°. A unique target was unavailable at staging and the monitor request failed. Both the independent return and E-stop passed. The visual-goal failure is retained as the overall trial exit status. These two layouts reproduce the tradeoff between feature availability and monitor recognition in the tested detector. [Odometry](bands-roundtrip01/evaluation.json) · [Visual-goal refusal](bands-roundtrip01/visual_goal.json) · [Stop](bands-roundtrip01/estop.json)

## Configuration and regression scope

The navigation-localization owner is AMCL. The private mapping path subscribes to Gazebo RGB, depth and camera calibration over ROS, publishes RGB-D odometry in the visual namespace and builds a database in the visual map frame. Both visual nodes keep TF publication off. The RGB-D backend uses the original RTAB-Map feature and inlier settings unless an explicit diagnostic flag selects SIFT, optical flow or the planar constraint. Source-map localization opens a checked trial copy, selects nonincremental memory with all map nodes initialized, and explicitly starts at the first node. All scored source-map matches must cite an existing saved node.

The 67 optimized nodes from the verified textured return belong to one connected component. Their sampled ground-truth path covers 15.14 m, or 4.42 optimized keyframes per meter. The 52,225 positive-depth feature observations include repeated observations across keyframes; this count is not a count of unique landmarks. The saved bounds and constraint errors are available in the [graph measurements](texture-roundtrip02/graph-evaluation.json).

Changes specific to ALL-06 include the private mapping and reload launch in `ros2_ws/src/gracemo_nav2/launch/gazebo_rgbd_mapping.launch.py`, mapper shutdown in `run_gracemo.sh`, private-world variants and dispatch in `scripts/validation/launcher_recovery.py`, and the registration trial wrapper, ROS/Gazebo observer, odometry/graph/relocalization evaluators, backend/resource summaries and occlusion fixture in `scripts/validation/`. The E-stop smoke test now records latch and first-zero timing alongside the existing thresholds. No candidate publishes navigation TF. The study moved robots only inside Gazebo.

Software checks are separate from Gazebo runs. The Python suite includes RGB-D/depth rejection, object perception, visual odometry map fixtures and validation-launcher path handling. It passed 32 tests in the follow-up. The Rust workspace passed 19 tests. Commands from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH="$PWD:$PWD/adapters/sdk:$PYTHONPATH" python3 -m pytest -q tests/test_perception_pipeline.py tests/test_edge_vision.py tests/ros2/test_gazebo_depth_frame.py tests/test_visual_validation_runner.py
cargo test --manifest-path kernel/Cargo.toml --workspace
```

## Original five-room patrol regression

The supported launcher dispatched the five-room patrol in the original apartment with the RGB-D observer active. The coordinator completed **11/11 waypoints in 195.3 s**, ending at (0.06, 0.18) m. The kernel recorded the same mission and trace IDs. Visual odometry supplied a valid pose on only **2.89%** of observed RGB stamps. Its 57 scored poses had 0.036 m / 0.28° RMSE. AMCL remained the navigation-localization source throughout. [Mission and adapter receipt](original-patrol01/patrol.json) · [Visual odometry](original-patrol01/evaluation.json) · [Launcher lifecycle](original-patrol01/process_lifecycle_evidence.json)

In the original baseline database, a separate comparison of 7,728 saved finite-depth boundaries found a median nearest RGB edge distance of 2 pixels and a p95 of 7 pixels. This nearest-edge check cannot distinguish correspondence errors from weak or absent color contrast and does not substitute for a calibrated target test. [Boundary measurements](baseline/cross-modal-edges.json)

## Close-approach test with a powered display

In the wall-band apartment, a blue powered display inside the original TV bezel restored the semantic goal. The adapter dispatched the monitor observation mission, Nav2 reached it, and the independent return, idle-zero and E-stop checks passed. Through simulation second 55 odometry tracked almost every frame. During approach to within roughly 1.5 m of the TV, median feature count fell to 54 over the next five seconds. Tracking remained lost even after feature counts recovered during the return. Across the complete run availability was 41.40%, translation RMSE 0.073 m and rotation RMSE 0.93° on 549 scored poses. These sparse errors exclude the lost return trajectory. [Visual goal](powered-roundtrip01/visual_goal.json) · [Odometry](powered-roundtrip01/evaluation.json) · [Return](powered-roundtrip01/revisit.json) · [Stop](powered-roundtrip01/estop.json)

To reproduce the powered-display observation use `VIRA_TEST_SURFACE_TEXTURE=wall_bands VIRA_TEST_MONITOR_SCREEN=1 VIRA_TEST_VISUAL_REVISIT=1` with the registration trial wrapper. To reproduce the five-room regression use `VIRA_TEST_REGISTRATION_PATROL=1` with a fresh output directory and the original apartment.

## Camera-rate mapping with a detailed monitor display

The powered-monitor test isolated tracking loss at the close approach. The next private fixture rendered window borders, text and chart traces on the same monitor. The RGB-D mapper was allowed to process every synchronized pair and its transform wait was bounded at 10 ms. The private estimator kept its existing registration threshold and TF ownership. These are separate changes, so the run measures their combined effect. The monitor was observed, an authorized mission reached it, the original return succeeded, and idle-zero and E-stop passed.

Odometry coverage rose to **99.85%**, but over the full route translation RMSE was **0.567 m** and rotation RMSE was **6.38°**. Both error gates failed. For the 283 comparable optimized graph nodes, translation RMSE improved from 0.501 to 0.168 m and rotation RMSE from 4.79° to 3.60°. The graph improvement does not repair the raw odometry errors. The map passed integrity checks and its optimized graph referenced zero missing dictionary words. The mapper exited cleanly. [Visual odometry](desktop-roundtrip01/evaluation.json) · [Graph](desktop-roundtrip01/graph-evaluation.json) · [Map save](desktop-roundtrip01/backend.json) · [Monitor receipt](desktop-roundtrip01/visual_goal.json)

Observed throughput was **8.85 matched visual poses/s and 8.32 graph updates/s**. Each of the 13 complete wall-clock windows (at least ten seconds each) exceeded 5 graph updates/s. The slowest window averaged 5.90. These are measured desktop simulation rates under this load. [Performance](desktop-roundtrip01/performance.json) · [Idle-zero](desktop-roundtrip01/idle_zero.json) · [E-stop](desktop-roundtrip01/estop.json)

Reproduce with `VIRA_TEST_SURFACE_TEXTURE=wall_bands VIRA_TEST_MONITOR_SCREEN=desktop VIRA_TEST_VISUAL_REVISIT=1 VIRA_TEST_MAPPING_CAMERA_RATE=1` and the private registration trial wrapper. `VIRA_TEST_MATCHED_DEPTH_SIZE=1` selects a separate trial in which RGB and depth have equal dimensions.

With only depth resolution changed to 320×240, the monitor and return succeeded. Odometry coverage was 98.62%, translation RMSE 0.493 m and rotation RMSE 5.20°. The mapper terminated during its SQLite statistics write with “database is locked”. A diagnostic camera-compatibility query read the active database during this trial and could have contended with the writer. The map has zero optimized nodes and fails persistence. The case is unsuitable for attributing the mapper failure to the camera change. The odometry process continued independently. Subsequent map reads are performed after shutdown. The trial harness now verifies mapper exit, graph presence, dictionary references and integrity before a successful trial can return. [Trajectory](matched-roundtrip01/evaluation.json) · [Map audit](matched-roundtrip01/backend.json) · [Backend error](matched-roundtrip01/1_gracemo_rgbd_slam.log)

Saved-map loading also compares the stored depth-image header with the configured depth format and dimensions. The original and equal-resolution databases passed their respective camera checks and were rejected when the expected resolution was swapped. This check addresses camera compatibility independently of graph acceptance. [Depth compatibility](depth-map-compatibility.txt)

The equal-resolution 3D-to-3D registration trial completed the authorized monitor visit and the return. Its mapper exited cleanly, and the optimized graph contained all referenced dictionary words. Raw visual odometry still missed both error limits at **0.648 m / 7.23°** despite **98.68%** availability. On the comparable keyframes, optimization reduced error from 0.633 m / 7.72° to 0.199 m / 2.07°. Changing to metric correspondences alone did not remove odometry drift. Reproduce with the equal-resolution desktop command and `VIRA_TEST_METRIC_RGBD=1`. [Trajectory](metric-roundtrip01/evaluation.json) · [Graph](metric-roundtrip01/graph-evaluation.json) · [Saved map](metric-roundtrip01/backend.json) · [Monitor](metric-roundtrip01/visual_goal.json)

A planar visual-motion constraint with the default PnP estimator produced 0.088 m / 0.86° error over the available poses. After the monitor approach it lost tracking, leaving only 46.97% route coverage. The authorized monitor and return succeeded, and the mapper saved a usable optimized graph. The private candidate failed the availability requirement. [Odometry](planar-desktop01/evaluation.json) · [Map](planar-desktop01/backend.json) · [Cross-trial comparison](monitor-route-comparison.json)

At 640×480 RGB and 320×240 depth the authorized monitor and return still completed, but the observer received only 495 RGB payloads for 1,374 distinct RGB or calibration timestamps. The ROS bridge publishes both from one callback. Availability was 20.20% against received RGB images and **7.28%** against the union of camera timestamps. Valid-pose throughput fell below one pose per wall second. The sparse 0.003 m RMSE cannot validate motion over this route. The traces establish payload loss at the observer; they do not isolate rendering, transport or subscriber scheduling as its cause. The evaluator now records both denominators for each newly scored run. Reproduce with the desktop-display camera-rate command plus `VIRA_TEST_RGBD_2X=1`. [Sensor counts and errors](hires-desktop01/evaluation.json) · [Throughput](hires-desktop01/performance.json)

A final private throughput diagnostic changed both camera update rates and bridge publication rates to 20 Hz at the original pixel dimensions. The launcher did not reach mission readiness within its 85 s validation deadline. No mission was dispatched. On this stationary capture, RGB arrived at 19.86 simulation FPS and visual pose coverage across camera timestamps was 75.11%. This excludes the configuration as demonstrated evidence of reliable moving odometry. [Readiness](twentyhz-desktop01/1_gracemo_readiness.log) · [Observer results](twentyhz-desktop01/evaluation.json) · [Wall-clock rates](twentyhz-desktop01/performance.json)

In the 10 Hz desktop-display run, median/p95/maximum image-stamp delay was 52/78/116 ms for odometry and 96/166/316 ms for mapping. Mapper RSS increased from 224.6 to 557.2 MiB as the map grew over 146 s. This run demonstrates throughput but cannot establish a steady memory ceiling. [Timing and resources](desktop-roundtrip01/performance.json)

## Five clean-map restarts

Each run started a fresh kernel and Gazebo partition with the same 0.25 m offset, loaded the same saved map into a private copy, and stayed stationary. All five passed launcher readiness, produced verified visual matches, and exited with a complete 283-node optimized graph. The logs show no dictionary regeneration. The source SHA-256 remained `7f66bed93c9db0a7341dc0124d20fb073c8231df7ab2c27c4b3823898a368b46`.

| Fresh launch | Verified matches | Worst position error (cm) | Worst rotation error (°) |
|---|---:|---:|---:|
| [clean-desktop-reload01](clean-desktop-reload01/relocalization.json) | 4 | 1.13 | 0.454 |
| [clean-desktop-reload02](clean-desktop-reload02/relocalization.json) | 4 | 1.09 | 0.495 |
| [clean-desktop-reload03](clean-desktop-reload03/relocalization.json) | 3 | 1.07 | 0.574 |
| [clean-desktop-reload04](clean-desktop-reload04/relocalization.json) | 4 | 1.09 | 0.362 |
| [clean-desktop-reload05](clean-desktop-reload05/relocalization.json) | 2 | 1.14 | 0.360 |

The following commands reproduce the clean-map diagnostic from the repository root, using fresh destination directories.

~~~bash
root=docs/validation/visual_all_06
source_run="$root/desktop-roundtrip01"
export VIRA_RTABMAP_INPUT="$PWD/$source_run/map.db"
export VIRA_RTABMAP_SHA256="7f66bed93c9db0a7341dc0124d20fb073c8231df7ab2c27c4b3823898a368b46"
export VIRA_RTABMAP_PREFIX=/tmp/vira05-overlay/opt/ros/jazzy
export VIRA_TEST_SURFACE_TEXTURE=wall_bands VIRA_TEST_MONITOR_SCREEN=desktop
export VIRA_TEST_MAPPING_CAMERA_RATE=1 VIRA_TEST_STATIONARY_RELOAD=1 VIRA_TEST_OFFSET_START=1
for i in 01 02 03 04 05; do
    dest="$root/repeat-clean-desktop$i"
    bash scripts/validation/run_registration_trial.sh "$dest" || break
    python3 scripts/validation/summarize_relocalization.py "$source_run" "$dest"
done
~~~

The remaining release blockers are visual tracking coverage in the original apartment and raw odometry drift on the integrated monitor route. The restart evidence covers a single displaced starting area. Whole-apartment visual relocalization and long-duration mapper memory behavior remain unverified.
