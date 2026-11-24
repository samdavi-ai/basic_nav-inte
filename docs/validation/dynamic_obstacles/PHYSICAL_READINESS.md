# VIRA physical navigation readiness

**Status — preparation only.** The evidence here comes from Gazebo. Physical calibration, stopping distance, speed limits and independent shutdown behavior all require measurements on the assembled robot. The hardware owner should record the date, build, firmware, payload, instrumentation, results and sign-off for each item below.

| Item | Evidence required before supervised mobile trials |
|---|---|
| Sensor calibration | Measure camera intrinsics and depth scale at multiple distances using a surveyed target. Check LiDAR ranges and blind zones against the same target. Record dropouts, latency and time synchronization during load. |
| Camera and LiDAR extrinsics | Survey each sensor relative to the base. Register returns to vertical boards at several bearings and heights. Verify optical axes, timestamps and residual errors through the full supported head travel. Check robot self-returns. |
| Footprint | Measure the outer envelope, wheel protrusions and corners across payload and joint configurations. Account for base-frame offset, overhangs and rotation in doorways. Record the padded footprint and the uncertainty used for each side. |
| Odometry | Against independently surveyed distance and yaw references, repeat straight runs and rotations in both directions. Record scale, wheel separation, heading drift and slip across relevant floor materials and loads. |
| Braking | In a cordoned area with an independent observer and distance reference, measure perception-to-command delay, drive disable delay, speed and stopping distance from progressively higher approved speeds. Cover forward, turning, payload, battery and floor conditions. Include repeated runs and outliers. |
| Maximum operating speed | **Unestablished.** Choose a limit only after the worst measured delay, stopping distance, footprint uncertainty and an engineering margin fit within the shortest reliably observable free space. Verify angular stopping separately. The configured simulation limits are unsuitable as physical evidence. |
| Hardware E-stop | With the chassis secured and an observer at an independent cutoff, verify each stop channel removes drive authority and latches. Record post-stop wheel motion. Reset must require deliberate action and fresh authorization before motion. |
| Motor-driver shutdown | With the chassis secured, measure torque removal or brake engagement under command timeout, process loss, controller reboot and power faults. Record coast distance and the state after restart. |
| Communication loss | Under supervision, interrupt each sensor, control heartbeat and upstream link independently. Measure time to inhibit drive, held zero and restart behavior. Verify stale data and delayed commands cannot restart the robot. |
| Supervised area and intervention | Mark an exclusion area beyond worst-case stopping distance with no public access. Assign a test lead and independent E-stop operator, rehearse stop and isolation, define entry/exit criteria and log near misses. Begin loaded mobile trials only after the secured checks and site risk review are signed. |

No physical test has been performed as part of this investigation.
