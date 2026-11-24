# E-stop regression

**Gate D remains open for this candidate.** The automatic moving smoke following each failed patrol attempted startup from NAVIGATION_FAILURE. Consult each run's estop.json for the outcome. A readiness failure before a trigger gives no evidence of latching or braking.

The previous candidate passed all eight interrupted states. Its largest displacement was 0.0824 m against 0.15 m allowed, maximum final-second component 0.0197 against 0.05 allowed, and maximum observed time to a zero command 3.74 ms. Those tests checked mission rejection while latched and explicit reset. The eight states were NAVIGATING, OBSTACLE_DETECTED, SAFETY_STOP, COSTMAP_UPDATE, ROUTE_REASSESSMENT, REPLANNING, PATH_VALIDATION and RESUMING. None was requalified on this approach profile. [Previous per-state measurements](../master_repair/ESTOP_INTERRUPTION_MATRIX.md)
