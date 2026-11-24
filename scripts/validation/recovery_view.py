"""Read the recovery tab through the same PySide widget and network manager as the desktop."""
import json
import os
from pathlib import Path
import sys
import time

from PySide6.QtWidgets import QApplication
from gracemo_control_center.modules.cognitive.cognitive_view import CognitiveView


def check(url, mission, path):
    os.environ["VIRA_KERNEL_URL"] = url
    app = QApplication([])
    view = CognitiveView()
    index = next(i for i in range(view.memory_tabs.count()) if view.memory_tabs.tabText(i)=="Restart review")
    view.memory_tabs.setCurrentIndex(index)
    deadline = time.monotonic()+7
    while time.monotonic()<deadline:
        app.processEvents()
        value = view.memory_tabs.widget(index).toPlainText()
        if mission in value:
            data = json.loads(value)
            assert any(r["mission_id"]==mission for r in data["pending_review"])
            path.write_text(json.dumps(data,indent=2))
            return
        time.sleep(.02)
    raise RuntimeError("The desktop review tab did not display the interrupted mission")


if __name__=="__main__":
    check(sys.argv[1],sys.argv[2],Path(sys.argv[3]))
