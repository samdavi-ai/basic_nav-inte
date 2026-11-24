"""Local, encrypted, consent-enrolled templates. No embeddings leave this module's caller."""
import fcntl
import json
import os
from pathlib import Path
import tempfile
from contextlib import contextmanager

import numpy as np
from cryptography.fernet import Fernet


class Gallery:
    def __init__(self, directory):
        self.root = Path(directory).expanduser()
        self.root.mkdir(mode=0o700, parents=True, exist_ok=True)
        stat = self.root.lstat()
        if self.root.is_symlink() or stat.st_uid != os.getuid() or stat.st_mode & 0o077:
            raise PermissionError("Gallery must be a private directory owned by this user")

    @contextmanager
    def _store(self, write):
        with (self.root / "lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX if write else fcntl.LOCK_SH)
            external_key = os.getenv("VIRA_FACE_KEY_FILE")
            key = Path(external_key) if external_key else self.root / "key"
            if external_key:
                import stat
                if key.resolve().is_relative_to(self.root.resolve()):
                    raise PermissionError("External gallery key must be outside the gallery")
                key_stat = key.lstat()
                if (not stat.S_ISREG(key_stat.st_mode) or key_stat.st_uid != os.getuid()
                        or key_stat.st_mode & 0o077):
                    raise PermissionError("Expected a private, owner-only gallery key")
            data = self.root / "templates.enc"
            if not key.exists() and not write and not data.exists():
                yield {}
                return
            if not key.exists() and not data.exists():
                with os.fdopen(os.open(key,os.O_CREAT | os.O_EXCL | os.O_WRONLY,0o600),"wb") as f:
                    f.write(Fernet.generate_key())
            cipher = Fernet(key.read_bytes())
            records = json.loads(cipher.decrypt(data.read_bytes())) if data.exists() else {}
            yield records
            if write:
                with tempfile.NamedTemporaryFile(dir=self.root,delete=False) as f:
                    f.write(cipher.encrypt(json.dumps(records).encode()))
                    f.flush()
                    os.fsync(f.fileno())
                os.replace(f.name,data)

    def enroll(self, label, vector, *, consent=False):
        norm = np.linalg.norm(vector)
        if not consent or not label.strip() or len(label)>80 or not np.isfinite(norm) or norm <= 0:
            raise ValueError("Explicit consent, a label and a valid embedding are required")
        with self._store(True) as records:
            if len(records) >= 64 and label not in records:
                raise ValueError("Gallery is full")
            records[label] = (vector / norm).ravel().tolist()

    def delete(self, label):
        with self._store(True) as records:
            return records.pop(label, None) is not None

    def match(self, vector, threshold, margin):
        with self._store(False) as records:
            norm = np.linalg.norm(vector)
            if not np.isfinite(norm) or norm <= 0:
                return "unknown", None
            ranks = sorted(((float(np.dot(v,vector.ravel()/norm)),k) for k,v in records.items()),reverse=True)
        if not ranks or ranks[0][0] < threshold:
            return "unknown", None
        if len(ranks)>1 and ranks[0][0]-ranks[1][0]<margin:
            return "ambiguous", None
        return "matched", ranks[0][1]
