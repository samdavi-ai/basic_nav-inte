#!/usr/bin/env python3
"""
Captures high-resolution visual regression screenshots of all 12 pages
of the GRaCEmo ViRa UARCC with the new professional industrial design system.
"""

import sys
import os
import time

# Ensure ROS and python paths
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt, QTimer

from gracemo_control_center.config.settings import Settings
from gracemo_control_center.app import MainWindow

def capture():
    # Use existing QApplication or create one
    app = QApplication.instance()
    if not app:
        app = QApplication(sys.argv)

    settings = Settings()
    window = MainWindow(settings)
    window.show()

    out_dir = os.path.abspath("docs/screenshots/after_professional")
    os.makedirs(out_dir, exist_ok=True)

    resolutions = [
        ("1440x900", 1440, 900),
        ("1024x720", 1024, 720)
    ]

    pages = [
        "Overview",
        "Autonomous Operations",
        "Simulation",
        "Sensors",
        "Navigation",
        "Localization",
        "Voice & Interaction",
        "Cognitive System",
        "Safety",
        "Diagnostics",
        "System Logs",
        "Settings"
    ]

    for res_name, w, h in resolutions:
        window.resize(w, h)
        app.processEvents()
        time.sleep(0.15)

        for page in pages:
            window.switch_to_page(page)
            for _ in range(5):
                app.processEvents()
                time.sleep(0.04)

            clean_page_name = page.lower().replace(" & ", "_and_").replace(" ", "_")
            out_path = os.path.join(out_dir, f"{clean_page_name}_{res_name}.png")
            pixmap = window.grab()
            pixmap.save(out_path, "PNG")
            print(f"[CAPTURED] {page} at {res_name} -> {out_path}")

    window.close()
    print("[SUCCESS] All page screenshots captured successfully.")

if __name__ == "__main__":
    capture()
