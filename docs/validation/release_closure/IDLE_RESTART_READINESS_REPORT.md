# Idle and restart readiness

**Gate E remains open.** Each patrol fixture observes three seconds after termination. Per-run idle.json records counts, peak component on each stream, observed mux owner and odometry displacement. The initial failed patrol recorded 60 final, 62 mux and 60 smoothed messages, all exactly zero, with mux owner NONE. [Initial idle window](patrol/idle.json)

The earlier coordinator and Nav2 restarts each passed the post-restart idle window. The moving mission after restart could not dispatch because localization covariance was uninitialized. The attached patrol smoke reports cannot isolate AMCL as their own cause. End-to-end restart readiness, full launcher restart and a correlated missing-localization error remain outstanding. The closure changes do not alter AMCL initialization. [Previous restart evidence](../master_repair/INTEGRATION_SAFETY_AUDIT.md)
