#!/usr/bin/env python3
"""
GRaCEmo ViRa — UARCC Visual Regression & Audit Capture Script.
Captures all 11 canonical pages and responsive layouts at 1500x920, 1366x768, and 1100x750.
"""

import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from PySide6.QtWidgets import QApplication
from PySide6.QtGui import QGuiApplication
from PySide6.QtCore import Qt

from gracemo_control_center.config.settings import Settings
from gracemo_control_center.app import MainWindow


def capture_all():
    app = QApplication.instance()
    if not app:
        app = QApplication(sys.argv)

    settings = Settings()
    window = MainWindow(settings)
    window.resize(1500, 920)
    window.show()
    window.raise_()
    window.activateWindow()

    # Process events to let everything initialize
    for _ in range(25):
        app.processEvents()
        time.sleep(0.04)

    out_dir = os.path.abspath("docs/screenshots/cleanup_audit")
    os.makedirs(out_dir, exist_ok=True)
    screen = QGuiApplication.primaryScreen()

    # 1. Capture All 11 Canonical Modules at 1500x920
    canonical_modules = [
        "Overview",
        "Autonomous Operations",
        "Simulation",
        "Navigation",
        "Sensors & Perception",
        "Localization & Kinematics",
        "Cognitive System",
        "Safety",
        "Diagnostics",
        "System Logs",
        "Settings",
    ]

    print("[INFO] Capturing 11 Canonical Modules at 1500x920...")
    for idx, mod in enumerate(canonical_modules, start=1):
        window.switch_to_page(mod)
        wait_cycles = 35 if mod == "Navigation" else 15
        for _ in range(wait_cycles):
            app.processEvents()
            time.sleep(0.04)
        clean = mod.lower().replace(" & ", "_").replace(" ", "_")
        p_mod = os.path.join(out_dir, f"{idx:02d}_{clean}_1500x920.png")
        screen.grabWindow(window.winId()).save(p_mod, "PNG")
        print(f"  ✓ [{idx:02d}/11] {mod} -> {p_mod}")

    # 2. Capture Sidebar Collapsed Rail State at 1500x920
    print("[INFO] Capturing Collapsed Sidebar Rail...")
    window.switch_to_page("Navigation")
    window._toggle_sidebar()  # Collapse to 54px rail
    for _ in range(12):
        app.processEvents()
        time.sleep(0.04)
    p_collapsed = os.path.join(out_dir, "12_sidebar_collapsed_rail_1500x920.png")
    screen.grabWindow(window.winId()).save(p_collapsed, "PNG")
    print(f"  ✓ Collapsed Sidebar -> {p_collapsed}")
    window._toggle_sidebar()  # Expand back

    # 3. Capture Responsive Resolutions (1366x768 and 1100x750)
    resolutions = [
        (1366, 768, "13_responsive_1366x768.png"),
        (1100, 750, "14_responsive_1100x750.png"),
    ]

    for width, height, fname in resolutions:
        print(f"[INFO] Resizing to {width}x{height}...")
        window.resize(width, height)
        for _ in range(20):
            app.processEvents()
            time.sleep(0.04)
        p_res = os.path.join(out_dir, fname)
        screen.grabWindow(window.winId()).save(p_res, "PNG")
        print(f"  ✓ Responsive {width}x{height} -> {p_res}")

    window.close()
    print("[SUCCESS] All audit screenshots captured successfully in docs/screenshots/cleanup_audit/!")


if __name__ == "__main__":
    capture_all()
