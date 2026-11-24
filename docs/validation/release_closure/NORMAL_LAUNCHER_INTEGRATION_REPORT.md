# Normal launcher integration

**Gate F held.** Gates A, B and C require deterministic acceptance first. The guard, lattice planner and rectangular critic were loaded only through the private runner. The live private graph shows controller and behavior servers on /cmd_vel_nav, guard output on /cmd_vel_guarded_nav, then mux, velocity smoother, collision monitor, terrain E-stop gate and final /cmd_vel consumed by the simulation bridge. [Private graph](patrol/idle.json)

The normal launcher has no acceptance result for this topology. Its release still requires verified exclusive ownership, lifecycle and shutdown behavior, restart readiness and the complete navigation and safety regressions. The private launch log also records Gazebo requiring SIGTERM after SIGINT. [Launch evidence](patrol/launch.log)
