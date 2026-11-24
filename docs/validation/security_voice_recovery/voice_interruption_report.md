# Development microphone and simulated emergency stop

The development computer exposes an ALC257 analog input. A two-second host capture returned 32,000 real signed 16-bit samples at 16 kHz. The restricted environment initially reported zero sound cards. The active desktop shares the microphone through PipeWire. The trials stored timing and recognition metadata rather than retaining raw speech in this evidence directory.

| Run | Observation | Classification |
|---|---|---|
| live_voice_1 | Keyword recognizer fired before the operator cue. ROS latched E-stop. | Invalid evidence for intentional spoken interruption |
| live_voice_2 | Operator confirmed speaking. The recognizer did not return a complete stop. | Failed |
| live_voice_3 | Direct ALSA capture reported device busy. | Failed to acquire audio |
| live_voice_4 | The shared desktop capture received 106,496 samples. Offline Vosk recognized stop, the authenticated kernel bound it to the active patrol, and Nav2 cancelled. | Real microphone and Gazebo integration passed in the instrumented client |
| combined_1 and combined_2 | The supported launcher ran the normal voice application during an active mission. Neither trial produced a confirmed stop receipt in the observation window. | Failed |
| combined_3 | The normal app's continuous emergency listener confirmed incoming audio. No stop receipt appeared during its 58-second window. | Failed |

In live_voice_4, zero command followed recognition by 0.021919 s. Measured rest followed by 0.294039 s. The maximum displacement from the recognition pose was 0.087175 m against the configured 0.15 m limit. Final-second peak absolute linear or angular velocity was 0.020110 against the respective numerical 0.05 thresholds. The coordinator entered EMERGENCY_STOP and Nav2 logged goal cancellation. The figures start at recognition. Acoustic onset, interruption of active speech playback, and microphone-disconnect behavior were not measured.

The first mission timeout revealed that cancelling a mission under E-stop could report AUTONOMY_READY. The coordinator now records a cancelled mission while holding the safety state. A focused test covers a later cancel and an idle repeat. In live_voice_4 the subsequent idle test measured zero commands. Its separate E-stop smoke test expected a ready system, found the previous latch still set and failed its startup condition. That smoke test supplies no second E-stop pass.

The production voice app now runs offline interruption independently of conversational processing on the shared Linux input. It sends the same authenticated kernel action as the measured client. Normal launcher acceptance remains open.
