# Gazebo autonomy and RGB-D mapping

The five-room patrol remains a release blocker. The private camera-recording patrol completed 0/11 with correlated NAVIGATION_FAILURE. [Mission](patrol/mission.json)

The combined policy and RGB-D corridor run recorded 92 policy proposals. The selected trajectory had a positive policy preference score in all 55 published DWB evaluations. This establishes participation in scoring. It does not establish that the policy changed the winning trajectory or improved routing. The RTAB-Map database contains eight nodes, nine link records and 220 feature records. RTAB-Map used RGB-D images and odometry, while Nav2 used AMCL. Localization from the saved visual map and full-room loop closure remain unverified. [Mapping log](corridor/mapping.log) · [Database](corridor/visual.db) · [DWB trace](corridor/trace.jsonl.gz)

The recovery planner twice rejected its starting pose as occupied. The executed corridor prefix had a +0.056314 m interpolated clearance lower bound. The private braking trial reserved the global planner's extra padding for stopping and turn checks. In the repeat, the robot stopped farther back, with +0.426125 m on the executed prefix. Recovery expired without reaching the goal. The repeat excluded the policy and mapper because the temporary RTAB-Map dependency download failed. It cannot isolate a causal change in clearance. [Original geometry](corridor/geometry.json) · [Trial geometry](corridor_padding/geometry.json) · [Trial mission](corridor_padding/mission.json)

During the front insertion Gazebo confirmed the obstacle at 5.012 s. The analyzer observed rest at 7.420 s. Four post-request plans were published. After an initial rollout veto, three braking vetoes occurred. The robot exhausted its retries 3.373 m from the goal. The interpolated clearance metric was +0.248233 m. Neither case qualified autonomous avoidance. [Dynamic measurements](front_padding/metrics.json) · [Mission](front_padding/mission.json)

The private padding trials used the Release fixture plugin build and the following commands with the ROS, workspace and plugin environments sourced. Use fresh output directories.

```bash
python3 scripts/validation/dynamic_cases.py /tmp/vira-front-repeat --planner-cell-margin
python3 scripts/validation/dynamic_cases.py /tmp/vira-corridor-repeat --planner-cell-margin --regression original
python3 scripts/validation/analyze_dynamic.py /tmp/vira-front-repeat
python3 scripts/validation/analyze_routing.py /tmp/vira-corridor-repeat
```

All 22 guard and geometry unit tests passed. The accepted launcher retains its prior configuration. The remaining simulator gate requires an executable replacement trajectory that preserves braking clearance through the full mission, followed by patrol and safety regressions. [Unit output](padding_tests.log)
