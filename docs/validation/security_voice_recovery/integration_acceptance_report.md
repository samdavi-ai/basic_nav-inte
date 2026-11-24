# Release decision — HOLD

| Capability | Classification | Evidence |
|---|---|---|
| Kernel-authorized standalone adapter to ROS dispatch | PARTIALLY IMPLEMENTED | Both final corridor trials recorded HTTP 401, zero denied commands and a correlated positive command on the same ROS topic. An authenticated stop also reached Nav2 in a moving patrol. All-transports authorization and the complete audit lifecycle remain open. |
| Real spoken interruption | PARTIALLY IMPLEMENTED | The instrumented microphone client passed. The normal launcher voice application has no passing stop-and-restart sequence. |
| Supported-launcher recovery | PARTIALLY IMPLEMENTED | A shutdown during mission execution, new launcher and kernel processes, preserved event IDs, idle-zero commands, blocked new missions and authorized operator abandonment were demonstrated. In-place safe resume remains missing. |
| Full cross-system workflow | BROKEN | The normal voice app did not deliver a verified stop in the combined trials. Dependent restart acceptance cannot be claimed for those runs. |
| Original static corridor regression | IMPLEMENTED for the tested fixture | Both final missions reached the coordinator goal. Executed interpolated footprint lower bounds were 0.123425 and 0.136731 m. Historical runs reported 0.137120 and 0.133977 m. Speed-based bounds accounting for sample gaps were 0.005475 and 0.023812 m. |
| Evidence completeness | PARTIALLY IMPLEMENTED | Actual kernel, bridge, microphone, controller, UI and launcher measurements are identified separately. Physical robot validation is outside scope. |

The corridor analyzer measures clearance against the recorded geometry. A universally collision-free route, reliable moving-obstacle handling, physical robot safety, and full voice-driven autonomy are outside what these runs demonstrate. See implementation_report.md for reproduction commands, authorization_security_report.md for the trust boundary, and the per-trial files for raw observations.
