"""Read the live PySide perception tab offscreen."""
import json
import os
import sys
import time
from pathlib import Path
from PySide6.QtWidgets import QApplication
from gracemo_control_center.modules.cognitive.cognitive_view import CognitiveView

os.environ["VIRA_KERNEL_URL"] = sys.argv[1]
app = QApplication([])
view = CognitiveView()
idx = next(i for i in range(view.memory_tabs.count()) if view.memory_tabs.tabText(i)=="Visual perception")
view.memory_tabs.setCurrentIndex(idx)
deadline = time.monotonic()+10
while time.monotonic()<deadline:
    app.processEvents()
    try:
        data = json.loads(view.memory_tabs.widget(idx).toPlainText())
    except ValueError:
        continue
    if data.get("current") and data["sample"].get("frames_processed",0)>0:
        panel = view.memory_tabs.widget(idx)
        visual = data["sample"].get("visual_odometry")
        trajectory_items = len(panel.scene.items()) if hasattr(panel,"scene") else 0
        result = dict(kernel_current=data["current"],frames_processed=data["sample"]["frames_processed"],
                      visual_state=(visual or {}).get("state"),
                      map_keyframes=(visual or {}).get("keyframes"),
                      trajectory_items=trajectory_items,
                      camera_frame=data["sample"].get("camera_frame"),
                      face_status=data["sample"].get("face_status"),
                      dropped_pending=data["sample"].get("dropped_pending"))
        if visual and visual.get("trajectory") and trajectory_items == 0:
            raise RuntimeError("Live visual trajectory was not rendered")
        Path(sys.argv[2]).write_text(json.dumps(result,indent=2))
        break
    time.sleep(.01)
else:
    raise RuntimeError("Current perception unavailable in desktop tab")
