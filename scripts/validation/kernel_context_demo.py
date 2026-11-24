"""Verify synthetic perception delivery and persistence across two isolated kernel starts."""
import json
import os
from contextlib import contextmanager
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import uuid

import requests

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from adapters.gemini_robotics.context import MNSEContextProvider


@contextmanager
def kernel(state):
    port = state/'listen.port'
    port.unlink(missing_ok=True)
    with (state/'kernel.log').open('a') as log:
        proc = subprocess.Popen(['kernel/target/debug/gracemo-kernel','--isolated-simulation'],
                                env=dict(os.environ,GRACEMO_STATE_DIR=str(state)),stdout=log,stderr=log)
        try:
            until = time.monotonic()+10
            while time.monotonic()<until and proc.poll() is None:
                if port.exists():
                    url = 'http://127.0.0.1:'+port.read_text().strip()
                    try:
                        if requests.get(url+'/health',timeout=.5).ok:
                            break
                    except requests.ConnectionError:
                        pass
                time.sleep(.05)
            else:
                raise RuntimeError('Isolated kernel unavailable')
            yield url
        finally:
            proc.terminate()
            proc.wait(timeout=5)


def get(url,path):
    response = requests.get(url+path,timeout=3)
    response.raise_for_status()
    return response.json()


def main():
    event = dict(id=str(uuid.uuid4()),trace_id=str(uuid.uuid4()),timestamp=int(time.time()),
                 source='Vision',observed_by='synthetic_fixture',parent_event_id=None,
                 event_type=dict(type='ObjectDetected',data=dict(class_name='demo_laptop',confidence=.95,x=4.5,y=3.)))
    with tempfile.TemporaryDirectory() as directory:
        state=Path(directory)
        with kernel(state) as url:
            with requests.get(url+'/events/live',stream=True,timeout=5) as stream:
                stream.raise_for_status()
                reply=requests.post(url+'/emit',json=event,timeout=3)
                reply.raise_for_status()
                delivered=next(json.loads(l[5:]) for l in stream.iter_lines(chunk_size=1) if l.startswith(b'data:'))
                assert delivered['id']==event['id'] and delivered['trace_id']==event['trace_id']
            adapter_context = MNSEContextProvider(url).get_compiled_context()
            assert 'demo_laptop' in adapter_context
            before={p:get(url,p) for p in ('/history','/graph','/context','/snapshot')}
            invalid_action = dict(event,id=str(uuid.uuid4()),event_type=dict(type='ActionRequested',
                                  data=dict(action='NavigateToRoom',params=dict(room=''))))
            rejected = requests.post(url+'/emit',json=invalid_action,timeout=3)
            assert rejected.status_code == 422
            assert not any(r['id']==invalid_action['id'] for r in get(url,'/history'))
            invalid=requests.post(url+'/emit',json={'unexpected':True},timeout=3)
            assert 400<=invalid.status_code<500
        with kernel(state) as url:
            after={p:get(url,p) for p in ('/history','/graph','/context','/snapshot')}
        for result in (before,after):
            assert any(r['id']==event['id'] and r['trace_id']==event['trace_id'] for r in result['/history'])
            assert 'demo_laptop' in json.dumps(result['/graph'])
        print(json.dumps(dict(input=event,sse=delivered,malformed_status=invalid.status_code,
                              invalid_action_status=rejected.status_code,adapter_context=adapter_context,before_restart=before,after_restart=after),indent=2))


if __name__=='__main__':
    main()
