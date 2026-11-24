# ROS 2 and Gazebo integration

A fresh original-corridor run used the simulator fixture, standalone mission adapter, isolated Rust kernel, ROS bridge, coordinator and Nav2. The kernel recorded MISSION_COMPLETED for one of one waypoints. The analyzer measured +0.152060 m at the nearest sampled footprint and a +0.147088 m lower bound for interpolated motion. Its additional speed-based bound was +0.039808 m. These results cover one static route. [Terminal correlation](gazebo_control/mission.json) · [Geometry](gazebo_control/geometry.json)

The historical five-room baseline completed 11/11. Later independent geometry evidence disqualified a doorway trial. The experimental matched-braking profile still exhausted retries. Existing E-stop and idle evidence is scoped to its tested profile and state. [Release gates](../release_closure/MASTER_RELEASE_CLOSURE_REPORT.md) · [Private interruption matrix](../master_repair/ESTOP_INTERRUPTION_MATRIX.md)
