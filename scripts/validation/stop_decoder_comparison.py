"""Compare offline stop decoders on synthesized PCM without playback or action dispatch."""
import json
from pathlib import Path
import subprocess
import sys

from vosk import KaldiRecognizer
from vira_interaction.core.offline_stop import StopRecognizer, _model


def compare():
    phrases = ("stop", "please stop", "emergency stop", "stop now", "don't stop",
               "do not stop", "where is the stop sign", "go to the kitchen", "the laptop")
    grammar = json.dumps([*phrases, "[unk]"])
    rows = []
    for voice in ("en-us", "en-gb"):
        for phrase in phrases:
            wav = subprocess.check_output(["espeak-ng", "-v", voice, "--stdout", phrase])
            pcm = subprocess.check_output(
                ["ffmpeg", "-loglevel", "error", "-i", "pipe:0", "-ar", "16000",
                 "-ac", "1", "-f", "s16le", "pipe:1"], input=wav
            ) + bytes(40000)
            outputs = {}
            for name in ("normal", "private_grammar"):
                listener = StopRecognizer()
                if name == "private_grammar":
                    listener.decoder = KaldiRecognizer(_model(), 16000, grammar)
                detected = False
                for offset in range(0, len(pcm), 1024):
                    if listener.feed(pcm[offset:offset+1024]):
                        detected = True
                outputs[name] = listener.finish() or detected
            rows.append(dict(voice=voice, phrase=phrase, expected=phrase in phrases[:4], **outputs))
    return rows


if __name__ == "__main__":
    with Path(sys.argv[1]).open("x") as dest:
        json.dump(compare(), dest, indent=2)
