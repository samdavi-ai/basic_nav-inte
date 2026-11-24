"""Capture a live operator keyword only after an independent observer confirms simulation motion."""
import json
from pathlib import Path
import subprocess
import sys
import time

from vira_interaction.core.event_bus import EventBusClient
from vira_interaction.core.offline_stop import StopRecognizer, dispatch_stop


def main(folder, url):
    recognizer = StopRecognizer()
    deadline = time.monotonic() + 100
    while not (folder / "moving.json").exists():
        if time.monotonic() >= deadline:
            raise RuntimeError("Timed out waiting for simulated motion")
        time.sleep(.02)
    moving = json.loads((folder / "moving.json").read_text())
    mic = subprocess.Popen(["arecord", "-q", "-D", "default",
                            "-r", "16000", "-f", "S16_LE", "-c", "1", "-t", "raw"],
                           stdout=subprocess.PIPE)
    result = {"mission_id": moving["mission_id"], "trace_id": moving["trace_id"]}
    try:
        start = time.monotonic()
        samples = 0
        while time.monotonic() < start + 65:
            data = mic.stdout.read(1024)
            if not data:
                raise RuntimeError("Microphone stream ended")
            if not (folder / "speak.ready").exists():
                (folder / "speak.ready").touch()
            if not (folder / "operator.cue").exists():
                continue
            samples += len(data)//2
            if not (folder / "speak.now").exists():
                (folder / "speak.now").touch()
            if recognizer.feed(data):
                result.update(recognized="stop", detected=time.monotonic(), samples=samples)
                result.update(dispatch_stop(EventBusClient(base_url=url), moving["trace_id"], moving["mission_id"]))
                return
        raise RuntimeError("Microphone keyword was not detected")
    finally:
        result["captured_samples_after_cue"] = samples
        result["completed_utterances"] = recognizer.completed_utterances
        mic.terminate()
        mic.wait(timeout=5)
        (folder / "speech.json").write_text(json.dumps(result, indent=2))


if __name__ == "__main__":
    main(Path(sys.argv[1]), sys.argv[2])
