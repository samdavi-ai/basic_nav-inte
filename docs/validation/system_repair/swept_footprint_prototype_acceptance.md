# Swept-footprint recovery prototype acceptance — 2026-11-21

## Decision

**Do not integrate the prototype into the accepted stack.**  The candidate has
one successful moving-crossing recovery, but it does not pass its own front
insertion acceptance case and has no completed E-stop interruption matrix.
This decision preserves the accepted Jazzy/Gazebo baseline and does not loosen
the footprint, inflation, collision-monitor, or E-stop configuration.

## Candidate versus accepted stack

| Area | Accepted stack | Private candidate |
|---|---|---|
| Command route | `/cmd_vel_nav` → mux → smoother → collision monitor → terrain E-stop | Guard subscribes `/cmd_vel_nav` and publishes `/cmd_vel_guarded_nav`; private mux override selects it. |
| Coordinator | Installed `AutonomousSupervisorNode` | `recovery_coordinator.py` subclass cancels an active goal after guard risk and swaps the behavior tree. |
| Controller | RotationShim + DWB standard critics | Adds test-built `SweptFootprint` and `ObstacleFootprint` critics. |
| Recovery tree | Accepted tree | Private `dynamic_replan.xml`, with separate assumptions. |
| Execution domain | Normal launcher | Explicitly rejects any partition not beginning `vira-dynamic-`. |

The private code is therefore not a drop-in safety component.  Integrating it
would require packaging its critic, deterministic launch ownership, audited
topic remapping, and an acceptance run in the normal stack.

## Swept-footprint specification

The guard uses the published rectangular padded footprint.  Its measured
half-extent is conservatively circumscribed as `h = max(edge lengths)/2`.
For a pose `(x, y, theta)`, obstacle cells are transformed into the robot frame
with the inverse rotation `R(-theta)`.  A collision is declared if an occupied
cell centre lies within the rectangle expanded by costmap-cell half diagonal:

`max(|R(-theta)(cell - [x,y])|) <= h + margin + resolution / sqrt(2)`.

It interpolates position and unwrapped yaw between poses.  Segment subdivision
uses translational travel plus corner-radius angular travel, at half-cell
spacing.  It validates both published orientations and path-tangent
orientations because NavFn intermediate quaternions can be identity on a curved
path.  The local collision test uses odom/local costmap; the global path test
uses map/global costmap.  The braking test propagates measured/requested twist
with configured 2.5 m/s² and 3.2 rad/s² deceleration and includes command/grid
age lag.  These are conservative simulation assumptions, not physical-robot
certification.

## Existing deterministic evidence

| Case | Result | Key measurements |
|---|---|---|
| `critic_crossing` moving crossing | PASS (prototype only) | Correlated completion, goal error 0.239 m, risk→zero 0.128 s, risk→rest 0.178 s, executed interpolated clearance +0.230 m, replanned global/local clearance +0.172/+0.220 m. |
| `critic_front` front insertion | FAIL integration gate | Correlated `NAVIGATION_FAILURE` after 3 retries.  Stop was safe (+0.363 m interpolated clearance; risk→zero 0.226 s), but no successful route recovery. |
| `recovery_crossing_repeat` | PASS (prototype only) | Correlated completion, +0.217 m executed interpolated clearance, one retry, global/local replan clearance +0.216/+0.210 m. |
| Mirrored crossing | NOT ACCEPTED | Prior accepted-stack evidence includes −0.00058 m padded overlap; the candidate has no complete metrics proving the corresponding corrected case. |
| Replanning barrier | NOT ACCEPTED | Existing trace records a pre-marking replacement plan and negative executed clearance. |
| E-stop interruption states | NOT TESTED | No evidence for the eight required recovery substate injections. |

The candidate reports DWB trajectory/path geometry in trace fields, but the
available scenario summaries do not provide a complete per-case DWB rejection
count.  That missing measurement is another acceptance blocker.

## Regression evidence preserved

* Focused geometry/safety tests: 11 passed.
* Accepted-tree blocked hallway: bounded `NAVIGATION_FAILURE`, 0 completed
  waypoints, no false completion.
* Accepted-tree five-room patrol after the conservative behavior-tree repair:
  11/11 waypoints in 190.6 s with matched coordinator/kernel mission and trace
  IDs.
* Gazebo E-stop baseline: pass; 0.0099 m drift, final one-second velocity
  0.0171, mission rejection while latched, explicit reset, and zero after reset.

## Required work before reconsidering integration

1. Package the guard/critic and run it through the normal launcher without a
   private partition, coordinator, or mux topology change.
2. Pass all listed deterministic dynamic cases, including front, mirrored,
   near-goal, replan-barrier, temporary and permanent blockage.
3. Record global/local marking, stop, clearance, route clearance, DWB rejection
   count, recovery duration, terminal correlation, and mission/trace IDs for
   each case.
4. Pass E-stop injection during every specified recovery state; retain the
   configured ≤0.05 final speed and ≤0.15 m drift thresholds.
5. Only then rerun full patrol/static/E-stop/idle regression and consider the
   25- and 100-case campaigns.
