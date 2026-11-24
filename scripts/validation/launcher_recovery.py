"""Interrupt an executing Gazebo mission and inspect a fresh supported launcher instance."""
import fcntl
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import tempfile
import time
import uuid

import requests


def private_domain():
    # Independent launchers may outlive a test client. Keep the lease in the launcher too.
    root = Path(tempfile.gettempdir()) / f"vira-validation-domains-{os.getuid()}"
    root.mkdir(mode=0o700, exist_ok=True)
    if root.is_symlink() or root.stat().st_uid != os.getuid() or root.stat().st_mode & 0o077:
        raise PermissionError("Expected a private domain lease directory")
    for offset in range(80):
        domain = 110 + (os.getpid()+offset) % 80
        lease = (root / str(domain)).open("a")
        try:
            fcntl.flock(lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
            return domain, lease
        except BlockingIOError:
            lease.close()
    raise RuntimeError("No free validation domain")


def validation_pythonpath(repo, inherited=None):
    paths = [repo, repo / "adapters" / "sdk", repo / "vira_interaction",
             repo / "ros2_ws" / "src" / "gracemo_bridge"]
    if inherited:
        paths.extend(Path(item) for item in inherited.split(os.pathsep) if item)
    return os.pathsep.join(str(path) for path in paths)


def apply_surface_textures(tree, state, surface_texture):
    """Decorate only the private world used for RGB-D registration experiments."""
    # Add surface detail only in this private diagnostic world.
    from copy import deepcopy
    import hashlib
    import xml.etree.ElementTree as ET
    import numpy as np
    from PIL import Image
    if surface_texture not in ("1", "subtle", "floor", "monitor_background", "wall_bands"):
        raise ValueError("Unknown private-world texture profile")
    for model in tree.findall(".//world/model"):
        name = model.get("name", "")
        if name != "floor" and not name.startswith(("wall_", "hall_")):
            continue
        if surface_texture == "floor" and name != "floor":
            continue
        if surface_texture == "monitor_background" and name in ("wall_east", "wall_south"):
            continue
        materials = model.findall("./link/visual/material")
        if surface_texture == "wall_bands" and name in ("wall_east", "wall_south"):
            # Leave an untextured central band behind the monitor. Add
            # surface detail above and below, using only visual geometry.
            link = model.find("link")
            original, = link.findall("visual")
            materials = []
            for label, height, z in (("upper", .7, .95), ("lower", .45, -1.075)):
                strip = deepcopy(original)
                strip.set("name", label)
                ET.SubElement(strip, "pose").text = f"0 0 {z} 0 0 0"
                size = strip.find("geometry/box/size")
                x, y, _ = map(float, size.text.split())
                size.text = f"{x + .002} {y + .002} {height}"
                link.append(strip)
                materials.append(strip.find("material"))
        for index, material in enumerate(materials):
            key = f"{model.get('name')}-{index}"
            seed = int.from_bytes(hashlib.sha256(key.encode()).digest()[:8], "big")
            rng = np.random.default_rng(seed)
            diffuse = [float(x) for x in material.findtext("diffuse", "1 1 1 1").split()]
            tone = np.asarray(diffuse[:3]) * 255
            minimum = .38
            if surface_texture == "subtle":
                # The material already carries the base color. Preserve its
                # contrast against the monitor while adding fine detail.
                tone = np.full(3, 255.)
                minimum = .82
            pattern = rng.uniform(minimum, 1.0, (80, 160, 1))
            texture = Image.fromarray((pattern * tone).astype("uint8")).resize((640, 320))
            path = state / f"{key}.png"
            texture.save(path)
            metal = ET.SubElement(ET.SubElement(material, "pbr"), "metal")
            ET.SubElement(metal, "metalness").text = "0"
            ET.SubElement(metal, "roughness").text = "1"
            ET.SubElement(metal, "albedo_map").text = str(path)


def run(out, spoken=False, inspection=False, latched=False, vision=False, visual_goal=False):
    out.mkdir(parents=True, exist_ok=False)
    lifecycle = []
    def save(name, data):
        (out / name).write_text(json.dumps(data, indent=2))
    with tempfile.TemporaryDirectory(prefix="vira-restart-") as directory:
        state = Path(directory)
        domain, lease = private_domain()
        repo = Path(__file__).resolve().parents[2]
        env = dict(os.environ, GRACEMO_STATE_DIR=directory, ROS_DOMAIN_ID=str(domain),
                   ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST",
                   GZ_PARTITION="vira-restart-"+uuid.uuid4().hex, PYTHONUNBUFFERED="1")
        env["PYTHONPATH"] = validation_pythonpath(repo, os.environ.get("PYTHONPATH"))
        if vision and env.get("VIRA_TEST_DEPTH_10HZ") == "1":
            import xml.etree.ElementTree as ET
            from ament_index_python.packages import get_package_share_directory
            original = Path(get_package_share_directory("gracemo_gazebo"))/"worlds"/"apartment_floor.world"
            tree = ET.parse(original)
            rate, = tree.findall(".//sensor[@name='depth_camera']/update_rate")
            if float(rate.text) != 5:
                raise ValueError("Unexpected baseline depth rate")
            rate.text = "10"
            if env.get("VIRA_TEST_RGBD_20HZ") == "1":
                color_rate, = tree.findall(".//sensor[@name='camera']/update_rate")
                color_rate.text = rate.text = "20"
            if env.get("VIRA_TEST_RGBD_2X") == "1":
                for sensor in ("camera","depth_camera"):
                    for axis in ("width","height"):
                        dim, = tree.findall(f".//sensor[@name='{sensor}']/camera/image/{axis}")
                        dim.text = str(2 * int(dim.text))
            if env.get("VIRA_TEST_MATCHED_DEPTH_SIZE") == "1":
                for axis in ("width", "height"):
                    rgb, = tree.findall(f".//sensor[@name='camera']/camera/image/{axis}")
                    depth, = tree.findall(f".//sensor[@name='depth_camera']/camera/image/{axis}")
                    depth.text = rgb.text
            if env.get("VIRA_TEST_OFFSET_START") == "1":
                start, = tree.findall(".//model[@name='gracemo_vira']/pose")
                coords = start.text.split()
                coords[0] = str(float(coords[0]) + 0.25)
                start.text = " ".join(coords)
            surface_texture = env.get("VIRA_TEST_SURFACE_TEXTURE")
            if surface_texture:
                apply_surface_textures(tree, state, surface_texture)
            display = env.get("VIRA_TEST_MONITOR_SCREEN")
            if display in ("1", "desktop"):
                # Render a powered display inside the existing black bezel. Keep the
                # TV model, collision shape and semantic detector unchanged.
                screen, = tree.findall(".//model[@name='tv_unit']/link[@name='screen']")
                screen.append(ET.fromstring("""
                    <visual name="powered_display">
                      <pose>-0.031 0 0 0 0 0</pose>
                      <geometry><box><size>0.001 1.33 0.73</size></box></geometry>
                      <material><ambient>0.12 0.33 0.56 1</ambient>
                        <diffuse>0.16 0.41 0.72 1</diffuse>
                        <emissive>0.07 0.14 0.22 1</emissive></material>
                    </visual>"""))
                if display == "desktop":
                    # Window borders, text and chart traces give the active screen
                    # realistic detail when the camera approaches for inspection.
                    from PIL import Image, ImageDraw
                    image = Image.new("RGB", (640, 360), "#368ac1")
                    canvas = ImageDraw.Draw(image)
                    for x, y, w, h, title in ((20, 18, 380, 240, "System status"),
                                              (280, 150, 330, 180, "Sensor history")):
                        canvas.rectangle((x, y, x+w, y+h), fill="#ecf3f7", outline="#124663", width=3)
                        canvas.rectangle((x, y, x+w, y+26), fill="#244864")
                        canvas.text((x+12, y+6), title, fill="white")
                        for j in range(4):
                            line = [(x+12+i*(w-24)/12, y+45+j*26+((i*i+7*j*i)%19))
                                    for i in range(13)]
                            canvas.line(line, fill=("#107d80", "#cf7119", "#336bb6", "#59516a")[j], width=3)
                    texture = state/"monitor_desktop.png"
                    image.save(texture)
                    material = screen.find("visual[@name='powered_display']/material")
                    material.find("diffuse").text = "1 1 1 1"
                    metal = ET.SubElement(ET.SubElement(material, "pbr"), "metal")
                    ET.SubElement(metal, "metalness").text = "0"
                    ET.SubElement(metal, "roughness").text = "1"
                    ET.SubElement(metal, "albedo_map").text = str(texture)
            tree.write(state/"camera_rate.world", encoding="utf-8", xml_declaration=True)
        if vision and env.get("VIRA_VISUAL_ODOMETRY") == "1":
            env["VIRA_VISUAL_MAP"] = str((out/"visual_map.npz").resolve())
        if spoken:
            env["VIRA_STOP_DIAGNOSTIC_CUE"] = str((out/"operator.cue").resolve())
        token = secrets.token_urlsafe(32)
        reviewer = secrets.token_urlsafe(32)
        voice_token=secrets.token_urlsafe(32)
        voice_file=state/"voice-token"
        with open(voice_file,"x",opener=lambda p,f: os.open(p,f,0o600)) as f:
            f.write(voice_token)
        env["VIRA_VOICE_TOKEN_FILE"]=str(voice_file)
        grants = state / "grants.json"
        with open(grants, "x", opener=lambda p,f: os.open(p,f,0o600)) as file:
            json.dump([dict(id="StandaloneMissionRunner",token=token,actions=["StartMission","Cancel"]),
                       dict(id="recovery-operator",token=reviewer,actions=["ReviewInterruptedMission","ReleaseEmergencyHold"] if latched else ["ReviewInterruptedMission"]),
                       dict(id="graceemo_interaction",token=voice_token,actions=["Stop","StartMission","Cancel"] if inspection else ["Stop"])],file)
        env["VIRA_ADAPTER_GRANTS"] = str(grants)
        if visual_goal:
            with grants.open() as f:
                principals = json.load(f)
            perception_token = secrets.token_urlsafe(32)
            principals.append(dict(id="gemini_robotics_er",token=perception_token,actions=["NavigateTo"]))
            with grants.open("w") as f:
                json.dump(principals,f)
        runner = None
        observer = None
        registration = None
        visual_status = None
        def wait(proc, predicate, timeout):
            deadline = time.monotonic()+timeout
            while proc.poll() is None and time.monotonic()<deadline:
                result = predicate()
                if result:
                    return result
                time.sleep(.15)
            raise RuntimeError("Launcher or readiness deadline reached")
        def history(url, trace):
            r=requests.get(url+"/history",params=dict(trace_id=trace,limit=500),timeout=3)
            r.raise_for_status()
            return r.json()
        for number in ((1,) if inspection or visual_goal else (1,2)):
            with (out/f"launcher_{number}.log").open("x") as log:
                if vision:
                    visual_status = subprocess.Popen([sys.executable,"scripts/validation/record_visual_status.py",
                                                       str(out/"visual_status.jsonl")],env=env,
                                                       stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
                proc=subprocess.Popen(["bash","run_gracemo.sh","--headless","--private-simulation"],
                                      stdout=log,stderr=log,env=env,start_new_session=True,pass_fds=(lease.fileno(),))
                entry=dict(launch=number,launcher_pid=proc.pid,started=time.time(),ros_domain=domain,partition=env["GZ_PARTITION"])
                lifecycle.append(entry)
                save("process_lifecycle_evidence.json",lifecycle)
                try:
                    if vision and env.get("VIRA_RECORD_REGISTRATION") == "1":
                        with (out/"registration_recorder.log").open("x") as diagnostic:
                            registration = subprocess.Popen([sys.executable, "scripts/validation/record_registration.py", str(out)],
                                                            env=env,stdout=diagnostic,stderr=diagnostic)
                    def online():
                        if not (state/"listen.port").exists():
                            return None
                        url="http://127.0.0.1:"+(state/"listen.port").read_text().strip()
                        try:
                            health=requests.get(url+"/health",timeout=.5).json()
                            return (url,health["pid"]) if health.get("pid") else None
                        except requests.RequestException:
                            return None
                    url,entry["kernel_pid"]=wait(proc,online,20)
                    save("process_lifecycle_evidence.json",lifecycle)
                    wait(proc,lambda:"[Stage 5/5]" in (out/f"launcher_{number}.log").read_text(),85)
                    if env.get("VIRA_TEST_STATIONARY_RELOAD") == "1":
                        if not (visual_goal and registration and env.get("VIRA_RTABMAP_INPUT")):
                            raise ValueError("Stationary reload requires a saved map and registration observer")
                        def has_mapping_observation():
                            if registration.poll() is not None:
                                raise RuntimeError("Registration observer exited before map observation")
                            trace = out / "registration.jsonl"
                            if trace.exists():
                                with trace.open() as stream:
                                    return any('"kind": "graph"' in line for line in stream)
                            return False
                        wait(proc, has_mapping_observation, 15)
                        if env.get("VIRA_TEST_VISUAL_OCCLUSION") == "1":
                            subprocess.run([sys.executable,"scripts/validation/visual_occlusion.py", str(out)],
                                           env=env, check=True, timeout=55)
                        # Use a fixed observation period after readiness, independent of match quality.
                        start = time.monotonic()
                        while time.monotonic() - start < 20:
                            if proc.poll() is not None or registration.poll() is not None:
                                raise RuntimeError("Launcher or observer exited during stationary reload")
                            time.sleep(.25)
                    elif visual_goal and env.get("VIRA_TEST_REGISTRATION_PATROL") == "1":
                        start = time.monotonic()
                        duration = int(env.get("VIRA_TEST_PATROL_SECONDS", "0"))
                        index = 0
                        while index == 0 or time.monotonic()-start < duration:
                            index += 1
                            label = "patrol" if index == 1 else f"patrol{index:02d}"
                            with (out/f"{label}.log").open("x") as evidence:
                                result = subprocess.run([sys.executable, "scripts/run_autonomous_mission.py",
                                                "--kernel-url", url, "--timeout", "360", "--out", str(out/f"{label}.json")],
                                               env=dict(env,VIRA_ADAPTER_TOKEN=token), stdout=evidence, timeout=420)
                            if result.returncode:
                                # A failed patrol ends motion. Continue observing the stationary system.
                                while time.monotonic()-start < duration:
                                    if proc.poll() is not None or registration.poll() is not None:
                                        raise RuntimeError("Launcher or observer exited during benchmark")
                                    time.sleep(1)
                                result.check_returncode()
                    elif visual_goal:
                        with (out/"staging.log").open("x") as log_file:
                            subprocess.run([sys.executable,"scripts/run_autonomous_mission.py","--goal","4","-2.5",
                                            "--kernel-url",url,"--timeout","100","--out",str(out/"staging.json")],
                                           env=dict(env,VIRA_ADAPTER_TOKEN=token),stdout=log_file,check=True,timeout=200)
                        visual_result = subprocess.run([sys.executable,"scripts/validation/visual_goal.py",url,str(out)],
                                       env=dict(env,VIRA_ADAPTER_TOKEN=perception_token),timeout=135)
                        if visual_result.returncode:
                            receipt = out / "visual_goal.json"
                            no_target = {"dispatched": False, "reason": "No current unique inanimate target"}
                            if (env.get("VIRA_TEST_VISUAL_REVISIT") != "1" or not receipt.exists()
                                    or json.loads(receipt.read_text()) != no_target):
                                visual_result.check_returncode()
                        if env.get("VIRA_TEST_VISUAL_REVISIT") == "1":
                            # Return to the initial camera viewpoint through the authorized mission runner.
                            with (out/"revisit.log").open("x") as evidence:
                                subprocess.run([sys.executable,"scripts/run_autonomous_mission.py","--goal","0","0",
                                                "--kernel-url",url,"--timeout","120","--out",str(out/"revisit.json")],
                                               env=dict(env,VIRA_ADAPTER_TOKEN=token),stdout=evidence,check=True,timeout=180)
                            with (out/"idle_zero.json").open("x") as evidence:
                                subprocess.run([sys.executable,"scripts/validation/check_idle_zero.py"],
                                               env=env,stdout=evidence,check=True,timeout=24)
                            subprocess.run([sys.executable,"scripts/run_gazebo_estop_smoke.py","--out",str(out/"estop.json")],
                                           env=env,check=True,timeout=100)
                            visual_result.check_returncode()
                    elif inspection:
                        if vision and env.get("VIRA_WALK_MESH"):
                            subprocess.run([sys.executable,"scripts/validation/visual_actors.py",str(out),url],
                                           env=env,timeout=60,check=True)
                        if vision and env.get("VIRA_REFERENCE_IMAGE"):
                            subprocess.run([sys.executable,"scripts/validation/visual_reference.py",str(out),url],
                                           env=env,timeout=50,check=True)
                        # Exercise the same interaction handler the desktop calls after transcription.
                        command = [
                            sys.executable, "-c",
                            "from vira_interaction.core.context_mission import handle; "
                            "from vira_interaction.core.gateway import InteractionGateway; "
                            "print(handle('go to the living room and inspect the environment', InteractionGateway()), flush=True)"
                        ]
                        with (out/"interaction.log").open("x") as interaction_log:
                            if vision:
                                from visual_resources import ResourceObserver
                                resources = ResourceObserver(proc.pid,url,out)
                                resources.start()
                                observer = subprocess.Popen([sys.executable,"scripts/validation/visual_truth.py",
                                                            str(out/"visual_truth.jsonl")],env=env)
                            try:
                                if vision and env.get("VIRA_TEST_PATROL") == "1":
                                    with (out/"idle_zero.json").open("x") as f:
                                        subprocess.run([sys.executable,"scripts/validation/check_idle_zero.py"],
                                                       stdout=f,env=env,check=True,timeout=24)
                                    with (out/"patrol.log").open("x") as f:
                                        subprocess.run([sys.executable,"scripts/run_autonomous_mission.py","--kernel-url",url,
                                            "--timeout","360","--out",str(out/"patrol.json")],
                                            env=dict(env,VIRA_ADAPTER_TOKEN=token),stdout=f,check=True,timeout=420)
                                subprocess.run(command, env=dict(env, VIRA_KERNEL_URL=url, VIRA_GAZEBO_INTERACTION="1",
                                    VIRA_ADAPTER_TOKEN_FILE=str(voice_file)),stdout=interaction_log,
                                    stderr=interaction_log, timeout=470, check=True)
                            finally:
                                if vision:
                                    resources.close()
                        if vision:
                            subprocess.run([sys.executable,"scripts/validation/visual_view.py",url,str(out/"desktop_visual.json")],
                                env=dict(env,QT_QPA_PLATFORM="offscreen"),check=True,timeout=15)
                        events=requests.get(url+"/history",params={"limit":1000},timeout=8).json()
                        inspection_events=[row for row in events if json.loads(row["payload"]).get("data",{}).get("name")=="mission_inspection"]
                        assert inspection_events, "No completed mission_inspection in kernel history"
                        trace=inspection_events[0]["trace_id"]
                        records=history(url,trace)
                        save("inspection_trace.json",records)
                        start=next(row for row in records if json.loads(row["payload"])["type"]=="ActionRequested")
                        request=next(row for row in records if json.loads(row["payload"]).get("data",{}).get("name")=="navigation_inspection_request")
                        assert start["parent_event_id"]==request["id"]
                        assert inspection_events[0]["parent_event_id"]==start["id"]
                    elif number==1:
                        if spoken:
                            with (out/"motion.log").open("x") as motion:
                                observer=subprocess.Popen([sys.executable,"scripts/validation/observe_voice_stop.py",str(out)],
                                    env=env,stdout=motion,stderr=motion,start_new_session=True)
                        with (out/"mission.log").open("x") as mission_log:
                            runner=subprocess.Popen([sys.executable,"scripts/run_autonomous_mission.py","--kernel-url",url,
                                      "--timeout","100"],env=dict(env,VIRA_ADAPTER_TOKEN=token),
                                      stdout=mission_log,stderr=mission_log,start_new_session=True)
                        def executing():
                            snap=requests.get(url+"/snapshot",timeout=2).json()
                            if snap["autonomy_state"]=="MISSION_EXECUTING":
                                return snap["active_mission"]["id"]
                        mid=wait(proc,executing,55)
                        trace=(out/"mission.log").read_text().split("Trace ID: ")[1].splitlines()[0]
                        save("runtime_snapshot.json", requests.get(url+"/snapshot",timeout=3).json())
                        save("adapters.json", requests.get(url+"/adapters",timeout=3).json())
                        query=requests.post(url+"/k/execute",json={"expression":'memory::recall[query:"autonomy_state"]'},timeout=3)
                        query.raise_for_status()
                        save("live_k.json",query.json())
                        shutdown_state="MISSION_EXECUTING"
                        if spoken:
                            voice_log=next(state.glob("launch.*/gracemo_voice.log"))
                            wait(proc,lambda:(out/"moving.json").exists() and "Offline emergency microphone receiving samples" in voice_log.read_text(errors="replace"),20)
                            (out/"speak.ready").touch()
                            def spoken_event():
                                with voice_log.open(errors="replace") as voice:
                                    for line in voice:
                                        if line.startswith("Microphone emergency stop "):
                                            return json.loads(line.split("Microphone emergency stop ",1)[1])
                            event=wait(proc,spoken_event,58)
                            assert (out/"operator.cue").exists() and event["detected"]>=float((out/"operator.cue").read_text())
                            assert event["receipt"]["mission_id"]==mid and event["receipt"]["active_mission_trace_id"]==trace
                            save("speech.json", event)
                            assert observer.wait(timeout=11)==0
                            assert json.loads((out/"interruption.json").read_text())["passed"]
                            shutdown_state=requests.get(url+"/snapshot",timeout=2).json()["autonomy_state"]
                            assert shutdown_state=="EMERGENCY_STOP"
                        if latched:
                            # Independent restart fault injection. No microphone claim.
                            headers = {"Authorization": "Bearer "+voice_token}
                            requests.post(url+"/adapters/connect", json={}, headers=headers, timeout=3).raise_for_status()
                            event = dict(id=str(uuid.uuid4()), trace_id=str(uuid.uuid4()), timestamp=int(time.time()),
                                         source="Voice", observed_by="graceemo_interaction",
                                         event_type={"type":"ActionRequested","data":{"action":"Stop"}})
                            response = requests.post(url+"/actions/authorize", json=event, headers=headers, timeout=3)
                            response.raise_for_status()
                            requests.post(url+"/actions/dispatch", json={"authorization_id":response.json()["authorization_id"]},
                                          headers=headers, timeout=3).raise_for_status()
                            def stopped():
                                snap=requests.get(url+"/snapshot",timeout=2).json()
                                return snap if snap["autonomy_state"]=="EMERGENCY_STOP" and snap["kernel_emergency_hold"]["latched"] else None
                            stopped_state=wait(proc,stopped,10)
                            save("latched_before_restart.json",stopped_state["kernel_emergency_hold"])
                            shutdown_state="EMERGENCY_STOP"
                        before=history(url,trace)
                        save("mission_history_before_restart.json",before)
                        entry.update(mission_id=mid, trace_id=trace,shutdown_state=shutdown_state)
                    else:
                        after=history(url,trace)
                        save("mission_history_after_restart.json",after)
                        view=requests.get(url+"/recovery/missions",timeout=2).json()
                        save("reconciliation.json",view)
                        review=next(x for x in view["at_kernel_start"] if x["mission_id"]==mid and x["trace_id"]==trace)
                        old={x["id"]:x for x in before}
                        new={x["id"]:x for x in after}
                        assert all(new.get(k)==v for k,v in old.items())
                        assert review["requires_operator_review"] and not review["automatic_resume"]
                        assert sum(json.loads(x["payload"])["type"]=="ActionRequested" for x in after)==1
                        assert lifecycle[0]["kernel_pid"]!=entry["kernel_pid"]
                        with (out/"restart_idle.json").open("x") as f:
                            subprocess.run([sys.executable,"scripts/validation/check_idle_zero.py"],stdout=f,
                                           env=env,check=True,timeout=23)
                        subprocess.run([sys.executable,"scripts/validation/recovery_view.py",url,mid,str(out/"desktop_review.json")],
                            env=dict(env,QT_QPA_PLATFORM="offscreen"),check=True,timeout=12)
                        action = next(r for r in before if json.loads(r["payload"])["type"] == "ActionRequested")
                        envelope = dict(id=str(uuid.uuid4()),trace_id=str(uuid.uuid4()),timestamp=int(time.time()),
                                        source="Brain",observed_by="StandaloneMissionRunner",event_type=json.loads(action["payload"]))
                        envelope["event_type"]["data"]["params"]["mission_id"] = str(uuid.uuid4())
                        def post(path, payload, credential):
                            return requests.post(url+path,json=payload,headers={"Authorization":"Bearer "+credential},timeout=3)
                        assert post("/adapters/connect",{},token).ok
                        blocked = post("/actions/authorize",envelope,token)
                        assert blocked.status_code == 409
                        decision=dict(action_id=review["action_id"],trace_id=trace,mission_id=mid)
                        denied = post("/recovery/abandon",decision,token)
                        assert denied.status_code == 403
                        assert post("/adapters/connect",{},reviewer).ok
                        abandoned = post("/recovery/abandon",decision,reviewer)
                        assert abandoned.status_code == 204
                        if latched:
                            restored=requests.get(url+"/snapshot",timeout=3).json()["kernel_emergency_hold"]
                            assert restored["latched"]
                            assert post("/actions/authorize",envelope,token).status_code == 423
                            payload={"event_id":restored["event_id"]}
                            assert post("/recovery/release-emergency-hold",payload,token).status_code==403
                            assert post("/recovery/release-emergency-hold",payload,reviewer).status_code==204
                            assert requests.get(url+"/snapshot",timeout=3).json()["kernel_emergency_hold"]["latched"] is False
                            save("emergency_review.json",dict(restored=restored,after_abandon=423,
                                runner_release=403,explicit_reviewer_release=204))
                        pending = requests.get(url+"/recovery/missions",timeout=3).json()["pending_review"]
                        assert pending == []
                        save("operator_decision.json",dict(mission_id=mid,trace_id=trace,
                             new_mission_before_review=blocked.status_code,runner_review=denied.status_code,
                             authorized_abandon=abandoned.status_code,pending_review=pending))
                finally:
                    if registration and registration.poll() is None:
                        registration.terminate()
                        registration.wait(timeout=5)
                    if visual_status and visual_status.poll() is None:
                        visual_status.terminate()
                        try:
                            visual_status.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            visual_status.kill()
                            visual_status.wait(timeout=5)
                    if observer and observer.poll() is None:
                        observer.kill()
                        observer.wait(timeout=5)
                    if runner and runner.poll() is None:
                        # rclpy installs its own TERM handler. Stop this owned test
                        # client before it can send a timeout cancellation.
                        runner.kill()
                        runner.wait(timeout=5)
                    if proc.poll() is None:
                        os.killpg(proc.pid,signal.SIGTERM)
                    try:
                        proc.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid,signal.SIGTERM)
                        proc.wait(timeout=20)
                    entry.update(ended=time.time(),exit_code=proc.returncode)
                    if "kernel_pid" in entry:
                        entry["kernel_exited"]=not Path(f'/proc/{entry["kernel_pid"]}').exists()
                    save("process_lifecycle_evidence.json",lifecycle)
                    # Only copy simulator/readiness logs. Voice logs may contain personal input.
                    saved=sorted(state.glob("launch.*"),key=lambda p:p.stat().st_mtime)[-1]
                    voice_log=saved/"gracemo_voice.log"
                    if voice_log.exists():
                        markers=("Offline emergency microphone receiving samples","Offline interruption unavailable","Using input device","Start Speaking","Speech End","Pre-STT Filtered",
                                 "Emergency stop delivery unconfirmed","Offline stop recognition unavailable",
                                 "Conversational transcription unavailable","Microphone emergency stop",
                                 "Error opening audio stream","Error reading audio","Traceback (most recent call last)")
                        lines=voice_log.read_text(errors="replace").splitlines()
                        if spoken:
                            save(f"voice_{number}_decodes.json", [
                                json.loads(line.removeprefix("Emergency decode ")) for line in lines
                                if line.startswith("Emergency decode ")
                            ])
                        metrics = [json.loads(line.removeprefix("Emergency input metrics ")) for line in lines
                                   if line.startswith("Emergency input metrics ")]
                        save(f"voice_{number}_input_levels.json", metrics)
                        save(f"voice_{number}_diagnostics.json",{mark:sum(mark in line for line in lines) for mark in markers})
                    for name in ("gracemo_readiness.log","gracemo_simulation.log","gracemo_perception.log","gracemo_rgbd_slam.log"):
                        if (saved/name).exists():
                            (out/f"{number}_{name}").write_bytes((saved/name).read_bytes())
                assert entry.get("kernel_exited")
                if env.get("VIRA_RECORD_REGISTRATION") == "1":
                    from summarize_rgbd_backend import summarize
                    backend = summarize(out)
                    save("backend.json", backend)
                    if (not backend["mapper_finished_cleanly"] or backend["integrity"] != "ok"
                            or not backend["optimized_node_count"] or backend["missing_optimized_dictionary_words"]):
                        raise RuntimeError("RGB-D mapper or saved graph failed validation; see backend.json")


if __name__=="__main__":
    def terminate(signum, frame):
        raise SystemExit(128 + signum)
    signal.signal(signal.SIGTERM, terminate)
    run(Path(sys.argv[1]), "--spoken" in sys.argv[2:], "--inspection" in sys.argv[2:] or "--vision" in sys.argv[2:], "--latched" in sys.argv[2:], "--vision" in sys.argv[2:], "--visual-goal" in sys.argv[2:])
