"""Recognize the emergency keyword locally from the microphone's 16 kHz PCM."""
import json
import os
from pathlib import Path
import subprocess
import sys
from functools import lru_cache
from array import array
import time
import uuid


@lru_cache(maxsize=1)
def _model():
    from vosk import Model
    default = Path(sys.prefix) / "share/vosk-model-small-en-us-0.15"
    path = Path(os.environ.get("VIRA_OFFLINE_STT_MODEL", default))
    if not (path / "am/final.mdl").is_file():
        raise FileNotFoundError("Offline speech model unavailable")
    return Model(str(path))


class StopRecognizer:
    def __init__(self):
        from vosk import KaldiRecognizer
        self.decoder = KaldiRecognizer(_model(), 16000)
        self.decoder.SetWords(True)
        self.completed_utterances = 0
        self.samples = 0
        self.diagnostic = os.getenv("VIRA_STOP_DIAGNOSTIC_CUE")
        self.capture_start = None

    def _is_stop(self, result):
        decoded = json.loads(result)
        text = decoded.get("text", "").strip()
        accepted = text in {"stop", "please stop", "emergency stop", "stop now",
                            "stop the robot", "please stop moving", "stop moving", "robot stop"}
        if text and self.diagnostic:
            cue = Path(self.diagnostic)
            words = decoded.get("result", [])
            if words and cue.exists() and self.capture_start is not None:
                onset = self.capture_start + words[0]["start"]
                if onset >= float(cue.read_text() or "inf"):
                    print("Emergency decode " + json.dumps({
                        "text": text, "words": words, "accepted": accepted,
                        "estimated_onset": onset, "decoded_at": time.monotonic(),
                        "capture_seconds": self.samples / 16000,
                    }), flush=True)
        if text:
            self.completed_utterances += 1
        return accepted

    def feed(self, pcm):
        if self.capture_start is None:
            self.capture_start = time.monotonic() - len(pcm) / 32000
        self.samples += len(pcm) // 2
        return bool(self.decoder.AcceptWaveform(pcm)) and self._is_stop(self.decoder.Result())

    def finish(self):
        return self._is_stop(self.decoder.FinalResult())


def dispatch_stop(bus, trace_id=None, mission_id=None):
    from gracemo_sdk.authorization import dispatch_action
    event = bus.create_envelope("ActionRequested", {"action": "Stop"},
                                trace_id=trace_id or str(uuid.uuid4()), session_id=mission_id,
                                source="Voice")
    started = time.monotonic()
    decision = dispatch_action(bus.base_url, event)
    return dict(dispatch_started=started, dispatch_returned=time.monotonic(),
                receipt=decision["receipt"])

def listen_for_stop(enabled, shutdown, on_stop):
    """Keep interruption available while conversation is transcribing or speaking."""
    while not shutdown.is_set():
        if not enabled.wait(.2):
            continue
        try:
            recognizer = StopRecognizer()
            with subprocess.Popen(
                ["arecord", "-q", "-D", "default", "-r", "16000", "-f", "S16_LE",
                 "-c", "1", "-t", "raw"], stdout=subprocess.PIPE
            ) as capture:
                try:
                    active = False
                    samples = 0
                    peak = 0
                    next_report = time.monotonic() + 5
                    while enabled.is_set() and not shutdown.is_set():
                        block = capture.stdout.read(1024)
                        if not block:
                            raise OSError("Microphone stream ended")
                        if not active:
                            print("Offline emergency microphone receiving samples", flush=True)
                            active = True
                        pcm = array("h", block)
                        if sys.byteorder != "little":
                            pcm.byteswap()
                        peak = max(peak, max(abs(sample) for sample in pcm))
                        samples += len(pcm)
                        if time.monotonic() >= next_report:
                            print("Emergency input metrics " + json.dumps({
                                "samples": samples, "peak_pcm": peak,
                                "decoded_utterances": recognizer.completed_utterances,
                            }), flush=True)
                            next_report = time.monotonic() + 5
                            peak = 0
                        if recognizer.feed(block):
                            on_stop()
                finally:
                    capture.terminate()
        except (OSError, RuntimeError, ImportError, ValueError) as exc:
            print(f"Offline interruption unavailable ({type(exc).__name__})", flush=True)
            shutdown.wait(2)
