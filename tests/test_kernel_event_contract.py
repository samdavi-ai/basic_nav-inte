"""Contract tests for adapter-to-Rust-kernel event envelopes."""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "adapters" / "sdk"))
from gracemo_sdk import AdapterClient


@patch("requests.post")
def test_adapter_emits_trace_and_session_ids(mock_post):
    mock_post.return_value = MagicMock(status_code=200)
    trace_id = "c7b5738f-5e4d-4aa3-82bb-14f75c9005a5"
    client = AdapterClient(adapter_name="NavigationSkill", base_url="http://kernel.test")

    assert client.emit(
        "ActionRequested",
        {"action": "NavigateToRoom", "params": {"room": "Kitchen & Dining"}},
        source="Brain",
        trace_id=trace_id,
    )

    envelope = mock_post.call_args.kwargs["json"]
    assert envelope["trace_id"] == trace_id
    assert envelope["session_id"] == "adapter:NavigationSkill"
    assert envelope["source"] == "Brain"
    assert envelope["event_type"]["data"]["action"] == "NavigateToRoom"


@patch("requests.post")
def test_unknown_adapter_source_is_normalized_to_rust_enum(mock_post):
    mock_post.return_value = MagicMock(status_code=200)
    client = AdapterClient(adapter_name="Gazebo-Vision")

    assert client.emit("VisionDetection", {"objects": ["chair"]}, source="SimVision")
    assert mock_post.call_args.kwargs["json"]["source"] == "Vision"
