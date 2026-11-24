"""Sample only descendants of a private Gazebo launcher; never retain camera images."""
import json
import threading
import time

import psutil
import requests


class ResourceObserver:
    def __init__(self,pid,url,out):
        self.root = psutil.Process(pid)
        self.url, self.out = url,out
        self.done = threading.Event()
        self.thread = threading.Thread(target=self.sample,daemon=True)

    def start(self):
        self.thread.start()

    def close(self):
        self.done.set()
        self.thread.join()

    def sample(self):
        with (self.out/"visual_resources.jsonl").open("x") as log:
            while not self.done.is_set():
                pss = {}
                skipped = 0
                for proc in self.root.children(recursive=True)+[self.root]:
                    try:
                        name = proc.name()
                        pss[name] = pss.get(name,0)+proc.memory_full_info().pss
                    except (psutil.Error,FileNotFoundError):
                        skipped += 1
                try:
                    snap = requests.get(self.url+"/snapshot",timeout=2).json()
                    data = snap.get("perception",{})
                    query_start = time.perf_counter()
                    reply = requests.post(self.url+"/k/execute",
                        json={"expression":'memory::recall[query:"visible_objects"]'},timeout=2)
                    latency = (time.perf_counter()-query_start)*1000
                    recalled = requests.post(self.url+"/k/execute", json={"expression":
                        'memory::recall[query:"last_observation",entity:"monitor"]'},timeout=2)
                    record = dict(k_last_monitor=recalled.json(),k_visible=reply.json(),k_http_ms=latency,
                                  wall=time.time(),state=snap.get("autonomy_state"),pss_bytes=pss,
                                  skipped=skipped,perception=data)
                except requests.RequestException:
                    record = dict(wall=time.time(),state="snapshot unavailable",pss_bytes=pss,skipped=skipped)
                print(json.dumps(record),file=log,flush=True)
                self.done.wait(1)
