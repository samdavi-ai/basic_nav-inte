"""
Unit and Integration Test Suite for RViz-Inspired UARCC Engineering Interface.
Validates:
1. RVizViewportWidget 3D rendering and camera orbit geometry
2. DisplaysTreeWidget hierarchical display items, checkboxes, and telemetry binding
3. ViewsInspectorWidget property inspector and camera synchronisation
4. MainWindow desktop engineering layout, menu bar, docks, and toolbar
"""

import os
import sys
import pytest
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt

os.environ["QT_QPA_PLATFORM"] = "offscreen"
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["GZ_PARTITION"] = "gracemo"

from gracemo_control_center.config.settings import Settings
from gracemo_control_center.app import MainWindow
from gracemo_control_center.widgets.rviz_viewport import RVizViewportWidget
from gracemo_control_center.widgets.embedded_rviz_widget import EmbeddedRVizWidget
from gracemo_control_center.widgets.displays_tree import DisplaysTreeWidget
from gracemo_control_center.widgets.views_inspector import ViewsInspectorWidget
from gracemo_control_center.core.event_bus import bus


@pytest.fixture(scope="session")
def qapp():
    app = QApplication.instance()
    if not app:
        app = QApplication([])
    return app


@pytest.fixture
def settings():
    return Settings()


def test_rviz_viewport_initialization(qapp):
    """Validate 3D Viewport default orbit camera parameters and display flags."""
    vp = RVizViewportWidget()
    assert vp.cam_dist == 7.5
    assert vp.cam_pitch == 35.0
    assert vp.cam_yaw == -45.0
    assert vp.show_grid is True
    assert vp.show_robot is True
    assert vp.show_map is True
    assert vp.show_scan is True
    assert vp.show_tf is True
    assert vp.tool_mode == "interact"
    vp.deleteLater()


def test_rviz_viewport_presets_and_camera(qapp):
    """Validate camera perspective presets and programmatic orbit control."""
    vp = RVizViewportWidget()

    # Preset Top-down
    vp.set_view_preset("top")
    assert abs(vp.cam_pitch - 89.9) < 0.1
    assert vp.cam_yaw == 0.0

    # Preset Front
    vp.set_view_preset("front")
    assert vp.cam_pitch == 8.0
    assert vp.cam_yaw == 0.0

    # Preset Side
    vp.set_view_preset("left")
    assert vp.cam_pitch == 8.0
    assert vp.cam_yaw == -90.0

    # Programmatic Orbit Setting
    vp.set_camera_orbit(distance=12.0, pitch=45.0, yaw=30.0, focal_point=(1.0, 2.0, 0.0))
    assert vp.cam_dist == 12.0
    assert vp.cam_pitch == 45.0
    assert vp.cam_yaw == 30.0
    assert (vp.focal_x, vp.focal_y, vp.focal_z) == (1.0, 2.0, 0.0)

    # Reset
    vp.reset_view()
    assert vp.cam_dist == 7.5
    assert vp.cam_pitch == 35.0
    assert vp.cam_yaw == -45.0
    vp.deleteLater()


def test_rviz_viewport_display_toggles(qapp):
    """Validate toggling display layer visibility."""
    vp = RVizViewportWidget()

    vp.set_display_visible("Grid", False)
    assert vp.show_grid is False

    vp.set_display_visible("Robot Model", False)
    assert vp.show_robot is False

    vp.set_display_visible("LaserScan", False)
    assert vp.show_scan is False

    vp.set_display_visible("Map", False)
    assert vp.show_map is False

    vp.set_display_visible("TF", False)
    assert vp.show_tf is False

    vp.set_display_visible("Grid", True)
    assert vp.show_grid is True
    vp.deleteLater()


def test_displays_tree_widget(qapp):
    """Validate DisplaysTreeWidget population, toggling, and property selection."""
    dt = DisplaysTreeWidget()

    # Check that canonical displays are registered
    expected_displays = [
        "Grid", "Robot Model", "TF", "Map", "LaserScan",
        "PointCloud2", "RGB Camera", "Depth Camera", "Odometry",
        "AMCL Particle Cloud", "Global Path", "Costmap", "Waypoints", "Safety Zones"
    ]
    for name in expected_displays:
        assert name in dt._display_items, f"Display {name} missing from tree"

    # Test signal emission on toggle
    toggled_signals = []
    dt.display_toggled.connect(lambda name, st: toggled_signals.append((name, st)))

    item = dt._display_items["LaserScan"]
    item.setCheckState(0, Qt.Unchecked)
    assert len(toggled_signals) > 0
    assert toggled_signals[-1] == ("LaserScan", False)

    item.setCheckState(0, Qt.Checked)
    assert toggled_signals[-1] == ("LaserScan", True)

    # Test property selection
    selected_signals = []
    dt.display_selected.connect(lambda name, props: selected_signals.append((name, props)))
    dt._on_item_clicked(item, 0)
    assert len(selected_signals) > 0
    assert selected_signals[-1][0] == "LaserScan"
    assert "Topic" in selected_signals[-1][1]

    dt.deleteLater()


def test_views_inspector_widget(qapp):
    """Validate ViewsInspectorWidget camera controls and properties inspector."""
    vi = ViewsInspectorWidget()

    # Camera changes signal
    cam_signals = []
    vi.camera_changed.connect(lambda d, p, y: cam_signals.append((d, p, y)))

    vi.spin_dist.setValue(10.0)
    assert len(cam_signals) > 0
    assert cam_signals[-1][0] == 10.0

    # Sync from viewport
    vi.update_camera_values({"distance": 8.5, "pitch": 25.0, "yaw": -30.0, "follow_robot": False})
    assert vi.spin_dist.value() == 8.5
    assert vi.spin_pitch.value() == 25.0
    assert vi.spin_yaw.value() == -30.0

    # Property table loading
    vi.set_item_properties("LaserScan", {"Topic": "/scan", "Size (m)": 0.06})
    assert vi.props_table.rowCount() == 2
    assert vi.props_table.item(0, 0).text() == "Topic"
    assert vi.props_table.item(0, 1).text() == "/scan"

    vi.deleteLater()


def test_mainwindow_rviz_desktop_layout(qapp, settings):
    """Validate MainWindow RViz-inspired layout, menus, docks, and views."""
    win = MainWindow(settings)

    # Check Menu Bar
    menubar = win.menuBar()
    assert menubar is not None
    actions = [a.text() for a in menubar.actions()]
    assert any("File" in a for a in actions)
    assert any("Panels" in a for a in actions)
    assert any("Tools" in a for a in actions)
    assert any("View" in a for a in actions)

    # Check Toolbar
    assert hasattr(win, "toolbar")
    assert win.toolbar is not None
    assert win.btn_tool_interact.isChecked()

    # Check Docks
    assert hasattr(win, "dock_displays")
    assert win.dock_displays.widget() is win.displays_tree
    assert hasattr(win, "dock_views")
    assert win.dock_views.widget() is win.views_inspector

    # Check Central Workspace Stack (11 canonical modules)
    assert win.stack.count() == 11
    assert hasattr(win, "embedded_rviz")
    assert hasattr(win, "sidebar")

    # Check tab switching
    win.switch_to_page("Overview")
    assert win.stack.currentIndex() == 0

    win.switch_to_page("3D Visualization")
    assert win.stack.currentIndex() == 3

    win.switch_to_page("Autonomous Operations")
    assert win.stack.currentIndex() == 1

    win.switch_to_page("Navigation")
    assert win.stack.currentIndex() == 3

    win.close()
