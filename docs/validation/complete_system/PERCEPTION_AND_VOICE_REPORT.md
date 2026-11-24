# Perception and voice

Gazebo supplies RGB, depth, LiDAR and odometry. The corridor trace contains simulator motion and sensor measurements. Prior RGB-D mapping and visual-feature evidence is stored in the [autonomy report](../autonomy_mapping/REPORT.md). The synthetic laptop observation exercises memory and adapter delivery. It supplies no evidence of visual recognition. Person recognition, moving-object identity, disappearance and stale scene reconciliation were not demonstrated end to end in this run.

Voice phrase interpretation, destination rejection and E-stop lockout ran using text fixtures. This environment exposed neither /dev/snd nor /dev/video0. Earlier development-computer evidence recorded a real camera frame, incomplete microphone capture and no operator-confirmed playback. Live Whisper recognition, audible confirmation and spoken interruption remain open. [Peripheral report](../system_repair/voice_hardware_acceptance_latest.md) · [Tests](offline.xml)
