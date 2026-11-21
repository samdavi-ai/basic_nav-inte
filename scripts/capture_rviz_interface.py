#!/usr/bin/env python3
"""
Captures high-resolution visual regression screenshots of the RViz-inspired
UARCC desktop engineering interface and all modules.
"""

import sys
import os
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt

from gracemo_control_center.config.settings import Settings
from gracemo_control_center.app import MainWindow


def capture():
    app = QApplication.instance()
    if not app:
        app = QApplication(sys.argv)

    settings = Settings()
    window = MainWindow(settings)
    window.show()

    out_dir = os.path.abspath("docs/screenshots/rviz_interface")
    os.makedirs(out_dir, exist_ok=True)

    # 1. Capture Primary RViz 3D Viewport in different views
    window.resize(1440, 900)
    app.processEvents()
    time.sleep(0.3)

    # Make sure we're on 3D Visualization Workspace
    window.switch_to_page("3D Visualization")
    app.processEvents()
    time.sleep(0.3)

    # A. Default Isometric (3D)
    window.viewport_3d.set_view_preset("iso")
    for _ in range(8):
        app.processEvents()
        time.sleep(0.04)
    pix = window.grab()
    p_iso = os.path.join(out_dir, "01_rviz_workspace_isometric_1440x900.png")
    pix.save(p_iso, "PNG")
    print(f"[CAPTURED] RViz Workspace Isometric -> {p_iso}")

    # B. Top-Down (2D)
    window.viewport_3d.set_view_preset("top")
    for _ in range(8):
        app.processEvents()
        time.sleep(0.04)
    pix = window.grab()
    p_top = os.path.join(out_dir, "02_rviz_workspace_topdown_1440x900.png")
    pix.save(p_top, "PNG")
    print(f"[CAPTURED] RViz Workspace TopDown -> {p_top}")

    # C. Front View
    window.viewport_3d.set_view_preset("front")
    for _ in range(8):
        app.processEvents()
        time.sleep(0.04)
    pix = window.grab()
    p_front = os.path.join(out_dir, "03_rviz_workspace_front_1440x900.png")
    pix.save(p_front, "PNG")
    print(f"[CAPTURED] RViz Workspace Front -> {p_front}")

    # D. Fit Robot View
    window.viewport_3d.fit_robot()
    for _ in range(8):
        app.processEvents()
        time.sleep(0.04)
    pix = window.grab()
    p_fit = os.path.join(out_dir, "04_rviz_workspace_fit_robot_1440x900.png")
    pix.save(p_fit, "PNG")
    print(f"[CAPTURED] RViz Workspace Fit Robot -> {p_fit}")

    # Reset back to isometric
    window.viewport_3d.set_view_preset("iso")
    for _ in range(5):
        app.processEvents()
        time.sleep(0.04)

    # 2. Capture Key Module Pages to verify preservation of engineering functionality
    pages = [
        "Overview",
        "Autonomous Operations",
        "Simulation",
        "Navigation",
        "Sensors",
        "Localization",
        "Safety",
        "Diagnostics",
        "System Logs"
    ]

    for idx, page in enumerate(pages, start=5):
        window.switch_to_page(page)
        for _ in range(6):
            app.processEvents()
            time.sleep(0.04)
        clean = page.lower().replace(" & ", "_").replace(" ", "_")
        p_page = os.path.join(out_dir, f"{idx:02d}_{clean}_1440x900.png")
        window.grab().save(p_page, "PNG")
        print(f"[CAPTURED] {page} -> {p_page}")

    # 3. Compact 1024x720 Resolution Test
    window.resize(1024, 720)
    window.switch_to_page("3D Visualization")
    for _ in range(8):
        app.processEvents()
        time.sleep(0.04)
    p_compact = os.path.join(out_dir, "14_rviz_workspace_compact_1024x720.png")
    window.grab().save(p_compact, "PNG")
    print(f"[CAPTURED] RViz Workspace Compact 1024x720 -> {p_compact}")

    window.close()
    print("[SUCCESS] All RViz interface screenshots captured successfully!")


if __name__ == "__main__":
    capture()
