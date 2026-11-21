#!/usr/bin/env python3
"""
Captures high-resolution visual regression screenshots of the UARCC desktop
workstation with the actual ROS 2 RViz2 interface embedded directly into the central workspace.
Captures at:
- 1500 x 920 (Canonical Workstation Resolution)
- 1366 x 768 (Standard Desktop Resolution)
- 1100 x 750 (Compact Engineering Workstation)
"""

import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QGuiApplication
from PySide6.QtCore import Qt

from gracemo_control_center.config.settings import Settings
from gracemo_control_center.app import MainWindow
from gracemo_control_center.core.ros_manager import RosManager


def capture():
    app = QApplication.instance()
    if not app:
        app = QApplication(sys.argv)

    settings = Settings()
    ros_manager = RosManager()
    ros_manager.start()

    window = MainWindow(settings)
    window.resize(1500, 920)
    window.show()
    window.raise_()
    window.activateWindow()

    # Process events to allow ROS manager and window embedding to settle
    for _ in range(30):
        app.processEvents()
        time.sleep(0.05)

    out_dir = os.path.abspath("docs/screenshots/embedded_rviz")
    os.makedirs(out_dir, exist_ok=True)
    screen = QGuiApplication.primaryScreen()

    # 1. Switch to Navigation (Embedded RViz2 Primary Workspace)
    window.switch_to_page("Navigation")
    for _ in range(40):
        app.processEvents()
        time.sleep(0.05)

    p_1500 = os.path.join(out_dir, "01_uarcc_with_embedded_rviz2_1500x920.png")
    screen.grabWindow(window.winId()).save(p_1500, "PNG")
    print(f"[CAPTURED] UARCC with Embedded RViz2 (1500x920) -> {p_1500}")

    # 2. Capture Canonical Modules at 1500x920
    modules = [
        "Overview",
        "Autonomous Operations",
        "Simulation",
        "Sensors & Perception",
        "Localization & Kinematics",
        "Cognitive System",
        "Safety",
        "Diagnostics",
        "System Logs",
        "Settings",
    ]

    for idx, mod in enumerate(modules, start=2):
        window.switch_to_page(mod)
        for _ in range(10):
            app.processEvents()
            time.sleep(0.04)
        clean = mod.lower().replace(" & ", "_").replace(" ", "_")
        p_mod = os.path.join(out_dir, f"{idx:02d}_{clean}_1500x920.png")
        screen.grabWindow(window.winId()).save(p_mod, "PNG")
        print(f"[CAPTURED] {mod} -> {p_mod}")

    # 3. Switch back to Navigation and test 1366 x 768
    window.switch_to_page("Navigation")
    window.resize(1366, 768)
    for _ in range(25):
        app.processEvents()
        time.sleep(0.04)
    p_1366 = os.path.join(out_dir, "12_uarcc_embedded_rviz2_1366x768.png")
    screen.grabWindow(window.winId()).save(p_1366, "PNG")
    print(f"[CAPTURED] UARCC Embedded RViz2 (1366x768) -> {p_1366}")

    # 4. Test Compact resolution: 1100 x 750
    window.resize(1100, 750)
    for _ in range(25):
        app.processEvents()
        time.sleep(0.04)
    p_1100 = os.path.join(out_dir, "13_uarcc_embedded_rviz2_1100x750.png")
    screen.grabWindow(window.winId()).save(p_1100, "PNG")
    print(f"[CAPTURED] UARCC Embedded RViz2 Compact (1100x750) -> {p_1100}")

    ros_manager.stop()
    window.close()
    print("[SUCCESS] All acceptance criteria screenshots captured successfully!")


if __name__ == "__main__":
    capture()
