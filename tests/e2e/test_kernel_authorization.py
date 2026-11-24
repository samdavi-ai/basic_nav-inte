"""Identity and one-shot dispatch checks using a private kernel with no ROS bridge."""
import hashlib
import json
import os
from pathlib import Path
import secrets
import sys
import time
from uuid import uuid4

import pytest
import requests

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts/validation"))
from kernel_context_demo import kernel


@pytest.fixture
def principal(tmp_path, monkeypatch):
    token = secrets.token_urlsafe(32)
    grant = tmp_path / "grant"
    with open(grant, "x", opener=lambda path, flags: os.open(path, flags, 0o600)) as f:
        json.dump([dict(id="test-reader", token=token, actions=["StatusQuery"])], f)
    monkeypatch.setenv("VIRA_ADAPTER_GRANTS", str(grant))
    with kernel(tmp_path) as url:
        def post(path, payload, authenticated=True):
            headers = {"Authorization": "Bearer " + token} if authenticated else {}
            return requests.post(url + path, json=payload, headers=headers, timeout=3)
        yield post


def event():
    return dict(id=str(uuid4()), trace_id=str(uuid4()), timestamp=int(time.time()),
                source="Brain", observed_by="test-reader", event_type=dict(
                    type="ActionRequested", data=dict(action="StatusQuery", params=dict(query="health"))))


def test_identity_and_capability(principal):
    e = event()
    assert principal("/actions/authorize", e, False).status_code == 401
    assert principal("/actions/authorize", e).status_code == 403
    assert principal("/adapters/connect", {}).ok
    assert principal("/actions/authorize", dict(e, observed_by="different")).status_code == 403
    e["event_type"]["data"] = {"action": "Stop"}
    assert principal("/actions/authorize", e).status_code == 403


def test_dispatch_once(principal):
    e = event()
    assert principal("/adapters/connect", {}).ok
    r = principal("/actions/authorize", e).json()
    assert r["parameter_sha256"] == hashlib.sha256(b'{"query":"health"}').hexdigest()
    handle = dict(authorization_id=r["authorization_id"])
    assert principal("/actions/dispatch", dict(handle, params={})).status_code == 422
    assert principal("/actions/dispatch", handle).json()["receipt"] == r
    assert principal("/actions/dispatch", handle).status_code == 409
    second = principal("/actions/authorize", e)
    if second.ok:
        assert principal("/actions/dispatch", dict(authorization_id=second.json()["authorization_id"])).status_code == 409
    else:
        assert second.status_code == 409


def test_expiry_and_disconnect(principal):
    assert principal("/adapters/connect", {}).ok
    first = principal("/actions/authorize", event()).json()
    assert principal("/adapters/disconnect", {}).ok
    assert principal("/adapters/connect", {}).ok
    assert principal("/actions/dispatch", dict(authorization_id=first["authorization_id"])).status_code == 403
    new = principal("/actions/authorize", event()).json()
    time.sleep(max(0, new["expires_at"] - time.time()) + .05)
    assert principal("/actions/dispatch", dict(authorization_id=new["authorization_id"])).status_code == 410


def test_revocation_invalidates_pending(principal, tmp_path):
    assert principal("/adapters/connect", {}).ok
    receipt = principal("/actions/authorize", event()).json()
    grant_file = tmp_path / "grant"
    grants = json.loads(grant_file.read_text())
    grants[0]["actions"] = []
    grant_file.write_text(json.dumps(grants))
    assert principal("/actions/dispatch", dict(authorization_id=receipt["authorization_id"])).status_code == 409
    assert principal("/adapters/connect", {}).json()["capabilities"] == []
    assert principal("/actions/authorize", event()).status_code == 403
