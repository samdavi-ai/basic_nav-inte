"""
Test Suite for Canonical 3D Robot Model Widget in GRaCEmo ViRa UARCC.
Validates model geometry parameters, telemetry binding, projection maths,
view modes, and Qt event handling.
"""

import sys
import unittest
import math
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import Qt, QPoint

# Ensure single QApplication instance
app = QApplication.instance()
if not app:
    app = QApplication(sys.argv)

from gracemo_control_center.widgets.robot_model_widget import (
    CanonicalRobotModelWidget, RobotViewport,
    COLOR_GRACE_ORANGE, COLOR_GRACE_WHITE
)
from gracemo_control_center.core.event_bus import bus


class TestRobotModelWidget(unittest.TestCase):
    """Test suite for CanonicalRobotModelWidget and RobotViewport."""

    def setUp(self):
        self.widget = CanonicalRobotModelWidget()
        self.viewport = self.widget.viewport

    def tearDown(self):
        self.widget.deleteLater()

    def test_initialization(self):
        """Test default states and parameters."""
        self.assertIsNotNone(self.viewport)
        self.assertEqual(self.viewport.cam_pitch, 22.0)
        self.assertEqual(self.viewport.cam_yaw, -45.0)
        self.assertEqual(self.viewport.cam_zoom, 1.0)
        self.assertEqual(self.viewport.nav_state, "IDLE")

    def test_view_modes(self):
        """Test switching camera perspective presets."""
        self.viewport.set_view_mode("top")
        self.assertEqual(self.viewport.cam_pitch, 88.0)
        self.assertEqual(self.viewport.cam_yaw, 0.0)

        self.viewport.set_view_mode("front")
        self.assertEqual(self.viewport.cam_pitch, 0.0)
        self.assertEqual(self.viewport.cam_yaw, 0.0)

        self.viewport.set_view_mode("side")
        self.assertEqual(self.viewport.cam_pitch, 0.0)
        self.assertEqual(self.viewport.cam_yaw, -90.0)

        self.viewport.set_view_mode("iso")
        self.assertEqual(self.viewport.cam_pitch, 22.0)
        self.assertEqual(self.viewport.cam_yaw, -45.0)

    def test_3d_projection(self):
        """Test projection of 3D point into screen space."""
        sx, sy, depth = self.viewport.project_3d(0.0, 0.0, 0.0, 150.0, 150.0, 100.0)
        self.assertIsInstance(sx, float)
        self.assertIsInstance(sy, float)
        self.assertIsInstance(depth, float)

        # Head lidar point should project above ground point in screen Y
        gx, gy, _ = self.viewport.project_3d(0.0, 0.0, 0.0, 150.0, 150.0, 100.0)
        hx, hy, _ = self.viewport.project_3d(0.0, 0.0, 1.175, 150.0, 150.0, 100.0)
        self.assertLess(hy, gy)  # in screen coordinates, higher Z is smaller Y

    def test_telemetry_update(self):
        """Test real-time update of roll, pitch, yaw, and velocities."""
        self.viewport.set_telemetry(0.05, -0.02, 1.57, 0.25, -0.1)
        self.assertAlmostEqual(self.viewport.robot_roll, 0.05)
        self.assertAlmostEqual(self.viewport.robot_pitch, -0.02)
        self.assertAlmostEqual(self.viewport.robot_yaw, 1.57)
        self.assertAlmostEqual(self.viewport.linear_speed, 0.25)
        self.assertAlmostEqual(self.viewport.angular_speed, -0.1)

    def test_event_bus_wiring(self):
        """Test event bus signal dispatch to widget."""
        bus.imu_data_ready.emit({"roll": 0.01, "pitch": -0.01, "yaw": 0.785})
        self.assertAlmostEqual(self.viewport.robot_yaw, 0.785)

        bus.autonomy_state_ready.emit("NAVIGATING")
        self.assertEqual(self.viewport.nav_state, "NAVIGATING")

    def test_render_paint(self):
        """Test that viewport paints cleanly without exceptions."""
        self.viewport.resize(400, 350)
        self.viewport.repaint()


if __name__ == "__main__":
    unittest.main()
