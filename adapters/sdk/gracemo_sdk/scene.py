"""Read a time-bounded RGB-D observation through the kernel snapshot."""
import time
import requests


def current_scene(kernel_url, *, after=None, after_sim_ns=None, timeout=3.0):
    deadline = time.monotonic()+timeout
    while True:
        response=requests.get(kernel_url.rstrip("/")+"/snapshot",timeout=2)
        response.raise_for_status()
        perception=response.json().get("perception") or {}
        sample=perception.get("sample") or {}
        captured=sample.get("captured_at")
        if (perception.get("current") and isinstance(captured,(int,float))
                and 0 <= time.time()-captured <= 3 and (after is None or captured > after)
                and (after_sim_ns is None or (sample.get("image_stamp_ns") or 0) > after_sim_ns)):
            observations=sample.get("observations")
            if isinstance(observations,list):
                return {"captured_at":captured,"frame":sample["frames_processed"],"image_stamp_ns":sample.get("image_stamp_ns"),"observations":observations}
        if time.monotonic()>=deadline:
            raise RuntimeError("No fresh synchronized RGB-D observation available")
        time.sleep(.15)


def describe(scene):
    observations=scene["observations"]
    if not observations:
        return "I could not localize any recognized objects in the latest camera view."
    return "In the latest camera view I localized " + ", ".join(
        f"{item['class_name']} at {item['depth_m']:.1f} metres ({item['confidence']:.0%} confidence)"
        for item in observations) + "."
