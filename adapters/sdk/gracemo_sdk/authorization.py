"""Submit a canonical action under an operator-provisioned kernel identity."""
import os
import stat
from urllib.parse import urlparse

import requests


def dispatch_action(base_url, envelope):
    token = os.environ.get("VIRA_ADAPTER_TOKEN")
    token_file = os.environ.get("VIRA_ADAPTER_TOKEN_FILE")
    if token_file:
        with os.fdopen(os.open(token_file, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)) as credential:
            meta = os.fstat(credential.fileno())
            if not stat.S_ISREG(meta.st_mode) or meta.st_mode & 0o077 or meta.st_uid != os.getuid():
                raise PermissionError("Adapter credential file must be private and owned by this user")
            token = credential.read(4097).strip()
    if not token:
        raise PermissionError("Adapter credential unavailable")
    url = base_url.rstrip("/")
    target = urlparse(url)
    if target.scheme != "https" and not (
        target.scheme == "http" and target.hostname in ("localhost", "127.0.0.1", "::1")
    ):
        raise ValueError("Credentials require a loopback or HTTPS kernel endpoint")
    headers = {"Authorization": "Bearer " + token}

    def post(path, payload):
        response = requests.post(url + path, headers=headers, json=payload,
                                 timeout=3, allow_redirects=False)
        if response.is_redirect:
            raise ValueError("Unexpected kernel redirect")
        response.raise_for_status()
        return response.json()

    post("/adapters/connect", {})
    receipt = post("/actions/authorize", envelope)
    return post("/actions/dispatch", {"authorization_id": receipt["authorization_id"]})
