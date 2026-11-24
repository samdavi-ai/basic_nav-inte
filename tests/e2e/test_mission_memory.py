"""Integration checks against a real, private Rust kernel. No robot commands."""
import sys
import tempfile
from pathlib import Path
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts/validation'))
from kernel_context_demo import get, kernel
from adapters.gemini_robotics.context import MNSEContextProvider
from gracemo_sdk import AdapterClient


def test_ledger_preserves_causal_links_and_unknown_pose():
    with tempfile.TemporaryDirectory() as directory:
        state = Path(directory)
        trace = str(uuid4())
        with kernel(state) as url:
            context = MNSEContextProvider(url)
            assert context.get_current_room() == 'Unknown'
            assert 'Pose[unknown]' in context.get_compiled_context()
            client = AdapterClient('test_inspector', url)
            assert client.emit('AdapterConnected',{'name':'test_inspector'},trace_id=trace)
            parent = client.get_trace_history(trace)[0]['id']
            assert client.emit('AdapterHeartbeat',{'name':'test_inspector'},trace_id=trace,
                               session_id='fixture-session', parent_event_id=parent)
            before = client.get_trace_history(trace)
            assert len(before)==2 and before[0]['parent_event_id']==parent
            assert before[0]['session_id']=='fixture-session'
        with kernel(state) as url:
            client = AdapterClient('new_client',url)
            assert client.get_trace_history(trace)==before
            assert client.get_trace_history(str(uuid4()))==[]
            assert get(url,'/snapshot')['robot_position'] is None


def test_unavailable_provider_cannot_claim_completion_or_dispatch():
    from adapters.gemini_robotics.config import GeminiRoboticsConfig
    from adapters.gemini_robotics.orchestrator import GeminiRoboticsOrchestrator

    with tempfile.TemporaryDirectory() as directory, kernel(Path(directory)) as url:
        cfg = GeminiRoboticsConfig(api_key='', kernel_url=url)
        orchestrator = GeminiRoboticsOrchestrator(config=cfg)
        result = orchestrator.run_task('Go to the library.')
        assert not result.success and not result.verified
        assert not orchestrator.client.mock_mode
        assert result.actions_dispatched == []
        assert not get(url,'/history')
