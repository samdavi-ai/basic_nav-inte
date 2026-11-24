# Current integration audit

| Capability | Implementation at inspection | Integrated evidence |
|---|---|---|
| Five-room route | BROKEN for clearance | 11/11 destinations, negative northwest wall clearance in Gazebo truth |
| Static obstacle navigation | IMPLEMENTED for original fixture | Two successful trials in unified_simulation |
| Offline emergency voice | PARTIALLY IMPLEMENTED | Nine recognized utterances, missing decoder words and no authenticated receipt in the last normal trial |
| Spoken navigation and TTS | PARTIALLY IMPLEMENTED | Normal face recognizes speech, typed runner verifies arrival. Cumulative audio loop unverified |
| Multi-step planning | PARTIALLY IMPLEMENTED | Deterministic parser handles a subset of the requested utterances. Gemini ranking only applies to standalone scene questions |
| K and MNSE | IMPLEMENTED for mission state lookup; PARTIALLY IMPLEMENTED for planning | Real K memory recall and graph executors; readiness gates use mission state |
| Kernel and SDK | IMPLEMENTED for tested authorization | Capability leases, authenticated receipt and ROS dispatch |
| RGB-D | IMPLEMENTED for synchronized post-arrival observations | Object identification depends on view; an empty frame cannot complete identification |
| Provider failure | PARTIALLY IMPLEMENTED | Unconfigured provider reports failure. Live success and failover unverified |
| Restart | PARTIALLY IMPLEMENTED | Durable history, review and idle verified. Latched E-stop survival still requires verification |
| PySide6 | PARTIALLY IMPLEMENTED | Live-state tabs include context, adapter leases and restart review |
