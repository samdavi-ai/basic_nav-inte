"""Verify local OpenCV Zoo models or provision missing files over verified HTTPS."""
import hashlib
import json
import os
from pathlib import Path
import tempfile

import requests


MODELS = (
    ("face_detection_yunet", "face_detection_yunet_2023mar.onnx", 232589,
     "8f2383e4dd3cfbb4553ea8718107fc0423210dc964f9f4280604804ed2552fa4"),
    ("face_recognition_sface", "face_recognition_sface_2021dec.onnx", 38696353,
     "0ba9fbfa01b5270c96627c4ef784da859931e02f04419c829e83484087c34e79"),
)


def verify(path, size, digest):
    with path.open("rb") as stream:
        if path.stat().st_size != size or hashlib.file_digest(stream, "sha256").hexdigest() != digest:
            raise ValueError(f"Integrity check failed for {path.name}")


def main():
    root = Path(__file__).resolve().parents[2] / "models"
    rows = []
    for family, name, size, digest in MODELS:
        target = root / name
        url = f"https://huggingface.co/opencv/{family}/resolve/main/{name}"
        if not target.exists():
            with tempfile.NamedTemporaryFile(dir=root) as out:
                with requests.get(url, stream=True, timeout=60) as response:
                    response.raise_for_status()
                    for chunk in response.iter_content(65536):
                        if out.tell() + len(chunk) > size:
                            raise ValueError(f"Oversize download for {name}")
                        out.write(chunk)
                out.flush()
                verify(Path(out.name), size, digest)
                try:
                    os.link(out.name, target)
                except FileExistsError:
                    pass
        verify(target, size, digest)
        rows.append(dict(name=name, sha256=digest, bytes=size, source=url))
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
