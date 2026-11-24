# Voice hardware acceptance — 2026-11-21

## Scope

Development computer peripherals only.  No robot motor, firmware, or hardware
transport was enabled or used.

## Implemented providers

* Audio capture: PyAudio / PortAudio.
* STT: Groq Whisper when credentials/network are available; SpeechRecognition
  fallback; stdin fallback in the lightweight engine.
* TTS: Edge TTS with `pw-play`, `aplay`, or pygame playback; pyttsx3 fallback
  in the production adapter; espeak-ng fallback in the legacy GUI path.
* Camera: OpenCV `VideoCapture`.
* Integration: `InteractionGateway` emits typed actions through the EventBus,
  then the Rust kernel and ROS interaction bridge.  Navigation remains routed
  through the coordinator and Nav2 rather than direct velocity commands.

## Results

| Check | Result | Evidence |
|---|---|---|
| Audio device enumeration | PASS | 12 PortAudio devices enumerated; analog input/output and PipeWire/default devices were visible. |
| Microphone capture | PARTIAL | A bounded real capture attempt on the default device blocked without delivering samples; it was terminated and no audio was retained. |
| Camera initialization/frame | PASS | Device 0 opened and produced one real 640 × 480 BGR frame. |
| Speaker/TTS playback | NOT TESTED | No audible playback test was run; it requires an operator-confirmed output path and must not be inferred from enumeration. |
| Cloud STT/TTS | NOT TESTED | Credentials and network availability were not exercised; no API key was inspected or logged. |
| End-to-end spoken navigation | NOT TESTED | Requires a successful microphone capture and provider response.  The downstream coordinator/ROS route is separately proven in Gazebo patrol tests. |

## Duplicate-voice repair

The legacy `model.py` GUI launched one daemon TTS thread per response without a
single playback owner.  Concurrent responses could therefore invoke overlapping
Edge-TTS/player sessions.  `TTS_PLAYBACK_LOCK` now grants one active legacy
playback session; overlapping responses are explicitly suppressed.  The
production `TTSAdapter` already stops its previous playback before starting a
new one.  A process inspection found no duplicate VIRA voice process at the
time of repair.

## Reproduction

```bash
./run_gracemo.sh --headless --no-verify --duration 20
```

For a live peripheral test, select a confirmed microphone/speaker device in
the desktop audio settings first.  Do not treat the above headless launch as
proof of audio capture or audible playback.
