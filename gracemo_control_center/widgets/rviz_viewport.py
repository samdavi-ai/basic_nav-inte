"""
RViz-Inspired 3D Engineering Visualization Viewport for GRaCEmo ViRa UARCC.
Provides a central interactive 3D robotics workspace rendering:
- Ground reference grid (10x10m) with coordinate axes
- Authentic canonical GRaCEmo ViRa 3D robot model (white/orange humanoid with wheels)
- 2D Occupancy Grid map on XY plane
- Live 360° Planar LiDAR LaserScan points
- Nav2 global trajectory path & active destination target
- 6 Authoritative apartment destination pins with labels
- Safety reflex envelopes (0.35m E-Stop, 0.80m Proximity)
- TF coordinate frame hierarchy (map → odom → base_footprint → lidar_link)
- Interactive orbit, pan, zoom, view presets, and follow-robot mode
"""

import math
import time
from typing import List, Tuple, Dict, Any, Optional

from PySide6.QtWidgets import QWidget, QMenu
from PySide6.QtCore import Qt, QPointF, QRectF, Signal, QTimer
from PySide6.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QPolygonF,
    QLinearGradient, QRadialGradient, QPainterPath, QCursor
)

from ..resources.styles.tokens import (
    COLOR_RVIZ_VIEWPORT_BG,
    COLOR_RVIZ_GRID,
    COLOR_RVIZ_GRID_MAJOR,
    COLOR_RVIZ_AXIS_X,
    COLOR_RVIZ_AXIS_Y,
    COLOR_RVIZ_AXIS_Z,
    COLOR_RVIZ_SELECTION,
    COLOR_SUCCESS,
    COLOR_WARNING,
    COLOR_ERROR,
    COLOR_INFO,
    FONT_FAMILY_PRIMARY,
    FONT_FAMILY_MONO,
)
from ..core.event_bus import bus


class RVizViewportWidget(QWidget):
    """Primary 3D Robotics Engineering Viewport inspired by ROS 2 RViz."""

    # Signals
    view_updated = Signal(dict)
    goal_placed = Signal(float, float, float)
    pose_estimated = Signal(float, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 400)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.StrongFocus)

        # Active Tool Mode: "interact", "move_camera", "select", "nav_goal", "pose_estimate"
        self.tool_mode = "interact"

        # Camera Configuration (Orbit around focal point in meters / degrees)
        self.focal_x = 0.0
        self.focal_y = 0.0
        self.focal_z = 0.0
        self.cam_dist = 7.5      # Distance from focal point (meters)
        self.cam_pitch = 35.0    # Elevation angle (degrees, 0 = horizon, 90 = top-down)
        self.cam_yaw = -45.0     # Azimuth angle (degrees)
        self.follow_robot = False

        # Mouse Drag State
        self._last_mouse_pos = None
        self._drag_button = None
        self._nav_goal_start: Optional[QPointF] = None
        self._nav_goal_end: Optional[QPointF] = None

        # Display Visibility Flags (controlled by Displays Tree)
        self.show_grid = True
        self.show_axes = True
        self.show_map = True
        self.show_robot = True
        self.show_scan = True
        self.show_path = True
        self.show_waypoints = True
        self.show_safety_zones = True
        self.show_tf = True

        # Live Robot Telemetry
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_vx = 0.0
        self.robot_wz = 0.0
        self.nav_state = "IDLE"

        # Occupancy Grid Data
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_data: List[int] = []
        self._map_points_cache: List[Tuple[float, float, int]] = []

        # LiDAR Point Cloud Data [(x_world, y_world, intensity), ...]
        self.scan_points: List[Tuple[float, float, float]] = []

        # Global Path [(x, y), ...]
        self.global_path: List[Tuple[float, float]] = []

        # Apartment Destinations
        self.destinations: Dict[str, Dict[str, Any]] = {}
        self.active_goal_key: Optional[str] = None

        # Animation phase for rotating sensors
        self.anim_phase = 0.0
        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._on_anim_tick)
        self._anim_timer.start(50)  # 20 FPS

        # Connect event bus signals
        bus.amcl_pose_ready.connect(self._on_amcl_pose)
        bus.map_ready.connect(self._on_map_ready)
        bus.lidar_scan_ready.connect(self._on_lidar_scan)
        bus.global_path_ready.connect(self._on_global_path)
        bus.coordinator_feedback_ready.connect(self._on_coordinator_feedback)

    def _on_anim_tick(self):
        self.anim_phase = (self.anim_phase + 0.1) % (2.0 * math.pi)
        if self.follow_robot:
            self.focal_x = self.robot_x
            self.focal_y = self.robot_y
            self.focal_z = 0.4
        self.update()

    def set_destinations(self, destinations: Dict[str, Dict[str, Any]]):
        self.destinations = destinations
        self.update()

    def set_tool_mode(self, mode: str):
        self.tool_mode = mode
        if mode in ("nav_goal", "pose_estimate"):
            self.setCursor(Qt.CrossCursor)
        elif mode == "move_camera":
            self.setCursor(Qt.OpenHandCursor)
        else:
            self.setCursor(Qt.ArrowCursor)

    # -------------------------------------------------------------------------
    # Camera Presets
    # -------------------------------------------------------------------------
    def set_view_preset(self, preset: str):
        preset = preset.lower()
        if preset in ("iso", "3d", "isometric"):
            self.cam_pitch = 35.0
            self.cam_yaw = -45.0
        elif preset in ("top", "topdown"):
            self.cam_pitch = 89.9
            self.cam_yaw = 0.0
        elif preset == "front":
            self.cam_pitch = 8.0
            self.cam_yaw = 0.0
        elif preset == "rear":
            self.cam_pitch = 8.0
            self.cam_yaw = 180.0
        elif preset == "left":
            self.cam_pitch = 8.0
            self.cam_yaw = -90.0
        elif preset == "right":
            self.cam_pitch = 8.0
            self.cam_yaw = 90.0
        elif preset == "follow":
            self.follow_robot = not self.follow_robot
        self._emit_view_info()
        self.update()

    def reset_view(self):
        self.focal_x = 0.0
        self.focal_y = 0.0
        self.focal_z = 0.0
        self.cam_dist = 7.5
        self.cam_pitch = 35.0
        self.cam_yaw = -45.0
        self.follow_robot = False
        self._emit_view_info()
        self.update()

    def fit_robot(self):
        self.focal_x = self.robot_x
        self.focal_y = self.robot_y
        self.focal_z = 0.4
        self.cam_dist = 4.5
        self._emit_view_info()
        self.update()

    def set_display_visible(self, name: str, visible: bool):
        """Toggle layer visibility based on display name."""
        key = name.lower().replace(" ", "_")
        if "grid" in key:
            self.show_grid = visible
        elif "robot" in key:
            self.show_robot = visible
        elif "tf" in key:
            self.show_tf = visible
        elif "map" in key:
            self.show_map = visible
        elif "scan" in key or "laser" in key:
            self.show_scan = visible
        elif "path" in key or "plan" in key:
            self.show_path = visible
        elif "waypoint" in key:
            self.show_waypoints = visible
        elif "safety" in key or "zone" in key:
            self.show_safety_zones = visible
        elif "axe" in key:
            self.show_axes = visible
        self.update()

    def set_camera_orbit(self, distance=None, pitch=None, yaw=None, focal_point=None):
        """Configure orbit camera parameters from inspector."""
        if distance is not None:
            self.cam_dist = max(0.5, min(50.0, float(distance)))
        if pitch is not None:
            self.cam_pitch = max(-89.9, min(89.9, float(pitch)))
        if yaw is not None:
            self.cam_yaw = float(yaw)
        if focal_point is not None:
            self.focal_x, self.focal_y, self.focal_z = focal_point
        self._emit_view_info()
        self.update()

    def _emit_view_info(self):
        self.view_updated.emit({
            "type": "Orbit",
            "distance": self.cam_dist,
            "pitch": self.cam_pitch,
            "yaw": self.cam_yaw,
            "focal_x": self.focal_x,
            "focal_y": self.focal_y,
            "focal_z": self.focal_z,
            "follow_robot": self.follow_robot,
        })

    # -------------------------------------------------------------------------
    # Telemetry Ingestion
    # -------------------------------------------------------------------------
    def _on_amcl_pose(self, x, y, yaw, vx, vy, vyaw):
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw
        self.robot_vx = vx
        self.robot_wz = vyaw
        if self.follow_robot:
            self.focal_x = x
            self.focal_y = y
        self.update()

    def _on_map_ready(self, map_dict: dict):
        self.map_width = map_dict.get("width", 0)
        self.map_height = map_dict.get("height", 0)
        self.map_resolution = map_dict.get("resolution", 0.05)
        self.map_origin_x = map_dict.get("origin_x", 0.0)
        self.map_origin_y = map_dict.get("origin_y", 0.0)
        self.map_data = map_dict.get("data", [])

        # Pre-cache occupied and unknown cell positions for fast 3D rendering
        self._map_points_cache.clear()
        step = max(1, int(0.10 / self.map_resolution)) # downsample for high FPS
        w = self.map_width
        h = self.map_height
        res = self.map_resolution
        ox = self.map_origin_x
        oy = self.map_origin_y

        for my in range(0, h, step):
            row_idx = my * w
            for mx in range(0, w, step):
                val = self.map_data[row_idx + mx]
                if val > 50: # Occupied obstacle
                    wx = ox + (mx + 0.5) * res
                    wy = oy + (my + 0.5) * res
                    self._map_points_cache.append((wx, wy, val))
        self.update()

    def _on_lidar_scan(self, scan_data: dict):
        """Transform ranges from robot lidar_link to world XY coordinates."""
        ranges = scan_data.get("ranges", [])
        angle_min = scan_data.get("angle_min", -math.pi)
        angle_inc = scan_data.get("angle_increment", (2.0 * math.pi) / max(1, len(ranges)))

        pts = []
        rx = self.robot_x
        ry = self.robot_y
        ryaw = self.robot_yaw
        lidar_offset_x = 0.0  # base_link to lidar_link

        for i, r in enumerate(ranges):
            if r is not None and not math.isinf(r) and not math.isnan(r) and 0.10 < r < 12.0:
                ang = ryaw + angle_min + i * angle_inc
                wx = rx + lidar_offset_x * math.cos(ryaw) + r * math.cos(ang)
                wy = ry + lidar_offset_x * math.sin(ryaw) + r * math.sin(ang)
                intensity = max(0.0, min(1.0, 1.0 - (r / 8.0)))
                pts.append((wx, wy, intensity))

        self.scan_points = pts
        self.update()

    def _on_global_path(self, path_coords: list):
        self.global_path = path_coords
        self.update()

    def _on_coordinator_feedback(self, state: str, speech: str, details: dict, ts: float):
        self.nav_state = state
        self.active_goal_key = details.get("destination", None)
        self.update()

    # -------------------------------------------------------------------------
    # 3D World to Screen Projection
    # -------------------------------------------------------------------------
    def project_to_screen(self, wx: float, wy: float, wz: float) -> Tuple[float, float, float]:
        """
        Convert ROS world coordinates (wx, wy, wz in meters, Z-up, X-forward, Y-left)
        to viewport screen pixels (sx, sy) and camera depth.
        """
        # Center relative to focal point
        dx = wx - self.focal_x
        dy = wy - self.focal_y
        dz = wz - self.focal_z

        # Rotate around Z by -cam_yaw (azimuth)
        yaw_rad = math.radians(self.cam_yaw)
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        x1 = dx * cos_y - dy * sin_y
        y1 = dx * sin_y + dy * cos_y
        z1 = dz

        # Rotate around X by cam_pitch (elevation)
        pitch_rad = math.radians(self.cam_pitch)
        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)
        x2 = x1
        y2 = y1 * cos_p - z1 * sin_p
        z2 = y1 * sin_p + z1 * cos_p

        # Camera depth (distance along view axis)
        depth = self.cam_dist + y2
        if depth <= 0.2:
            depth = 0.2

        # Perspective projection
        scale = min(self.width(), self.height()) / depth
        cx = self.width() * 0.5
        cy = self.height() * 0.5

        sx = cx + x2 * scale
        sy = cy - z2 * scale # Invert Y for screen coordinates

        return sx, sy, depth

    # -------------------------------------------------------------------------
    # Screen to Ground Plane (Z=0) Raycast
    # -------------------------------------------------------------------------
    def unproject_ground(self, sx: float, sy: float) -> Tuple[float, float]:
        """Convert screen pixel (sx, sy) to ground plane (Z=0) world coordinates (wx, wy)."""
        cx = self.width() * 0.5
        cy = self.height() * 0.5
        scale = min(self.width(), self.height()) / max(0.5, self.cam_dist)

        x2 = (sx - cx) / scale
        z2 = (cy - sy) / scale

        pitch_rad = math.radians(self.cam_pitch)
        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)

        if abs(sin_p) < 0.05:
            sin_p = 0.05

        y1 = -z2 / sin_p
        x1 = x2

        yaw_rad = math.radians(self.cam_yaw)
        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)

        wx = self.focal_x + (x1 * cos_y + y1 * sin_y)
        wy = self.focal_y + (-x1 * sin_y + y1 * cos_y)

        return wx, wy

    # -------------------------------------------------------------------------
    # Paint Event Orchestrator
    # -------------------------------------------------------------------------
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # 1. Background Canvas
        painter.fillRect(self.rect(), QColor(COLOR_RVIZ_VIEWPORT_BG))

        # 2. Reference Ground Grid (10x10m)
        if self.show_grid:
            self._draw_ground_grid(painter)

        # 3. Occupancy Grid Map
        if self.show_map:
            self._draw_map_obstacles(painter)

        # 4. Global Trajectory Path
        if self.show_path and len(self.global_path) > 1:
            self._draw_path(painter)

        # 5. Apartment Waypoint Destination Markers
        if self.show_waypoints:
            self._draw_waypoints(painter)

        # 6. Safety Reflex Envelopes (0.35m E-Stop, 0.80m Proximity)
        if self.show_safety_zones:
            self._draw_safety_zones(painter)

        # 7. Authentic Canonical GRaCEmo ViRa 3D Robot Model
        if self.show_robot:
            self._draw_robot_model(painter)

        # 8. Live LiDAR LaserScan Point Cloud
        if self.show_scan and self.scan_points:
            self._draw_lidar_points(painter)

        # 9. TF Coordinate Frames
        if self.show_tf:
            self._draw_tf_frames(painter)

        # 10. Interactive 2D Nav Goal Drag Arrow
        if self._nav_goal_start and self._nav_goal_end:
            self._draw_goal_arrow(painter)

        # 11. Screen Overlays (Corner Coordinate Axes, Tool Banner, View Stats)
        self._draw_screen_overlays(painter)

    # -------------------------------------------------------------------------
    # Render Sub-Routines
    # -------------------------------------------------------------------------
    def _draw_ground_grid(self, p: QPainter):
        """Draw 10x10m ground grid with 1.0m major and 0.2m minor lines."""
        extent = 5.0 # -5m to +5m
        pen_minor = QPen(QColor(COLOR_RVIZ_GRID), 1.0, Qt.SolidLine)
        pen_major = QPen(QColor(COLOR_RVIZ_GRID_MAJOR), 1.2, Qt.SolidLine)

        # Lines parallel to Y axis (varying X)
        x_steps = 21 # every 0.5m
        for i in range(x_steps):
            gx = -extent + i * 0.5
            is_major = (abs(gx - round(gx)) < 0.05)
            p.setPen(pen_major if is_major else pen_minor)

            s1x, s1y, _ = self.project_to_screen(gx, -extent, 0.0)
            s2x, s2y, _ = self.project_to_screen(gx, extent, 0.0)
            p.drawLine(QPointF(s1x, s1y), QPointF(s2x, s2y))

        # Lines parallel to X axis (varying Y)
        for i in range(x_steps):
            gy = -extent + i * 0.5
            is_major = (abs(gy - round(gy)) < 0.05)
            p.setPen(pen_major if is_major else pen_minor)

            s1x, s1y, _ = self.project_to_screen(-extent, gy, 0.0)
            s2x, s2y, _ = self.project_to_screen(extent, gy, 0.0)
            p.drawLine(QPointF(s1x, s1y), QPointF(s2x, s2y))

    def _draw_map_obstacles(self, p: QPainter):
        """Render 2D occupancy grid obstacles in 3D perspective."""
        if not self._map_points_cache:
            return

        brush = QBrush(QColor(230, 230, 230))
        p.setPen(Qt.NoPen)
        p.setBrush(brush)

        half_res = self.map_resolution * 1.2
        for wx, wy, _ in self._map_points_cache:
            sx, sy, d = self.project_to_screen(wx, wy, 0.01)
            size = max(2.0, min(8.0, 15.0 / d))
            p.drawRect(QRectF(sx - size * 0.5, sy - size * 0.5, size, size))

    def _draw_path(self, p: QPainter):
        """Draw Nav2 planned global trajectory."""
        p.setPen(QPen(QColor(37, 99, 235), 2.5, Qt.SolidLine))
        path_poly = QPolygonF()
        for wx, wy in self.global_path:
            sx, sy, _ = self.project_to_screen(wx, wy, 0.02)
            path_poly.append(QPointF(sx, sy))
        p.drawPolyline(path_poly)

    def _draw_waypoints(self, p: QPainter):
        """Render 6 apartment destination waypoints as 3D ground targets with labels."""
        p.setFont(QFont(FONT_FAMILY_PRIMARY, 8, QFont.Bold))

        for d_key, d_info in self.destinations.items():
            name = d_info.get("name", d_key)
            wx = d_info.get("x", 0.0)
            wy = d_info.get("y", 0.0)
            sx, sy, d = self.project_to_screen(wx, wy, 0.02)

            is_active = (d_key == self.active_goal_key)
            color = QColor(COLOR_RVIZ_SELECTION) if is_active else QColor(100, 116, 139)

            # Target Ring
            r = max(4.0, min(14.0, 28.0 / d))
            p.setPen(QPen(color, 2.0))
            p.setBrush(QBrush(QColor(color.red(), color.green(), color.blue(), 40)))
            p.drawEllipse(QPointF(sx, sy), r, r)

            # Center Dot
            p.setBrush(QBrush(color))
            p.drawEllipse(QPointF(sx, sy), 3.0, 3.0)

            # Name Label Badge
            p.setPen(QColor(240, 240, 240))
            p.drawText(int(sx + r + 4), int(sy + 4), name)

    def _draw_safety_zones(self, p: QPainter):
        """Draw circular safety envelopes around robot (0.35m E-Stop, 0.80m Proximity)."""
        rx = self.robot_x
        ry = self.robot_y

        # 0.35m E-Stop Boundary (Red)
        p.setPen(QPen(QColor(220, 38, 38, 160), 1.5, Qt.DashLine))
        p.setBrush(QBrush(QColor(220, 38, 38, 20)))
        poly_estop = QPolygonF()
        for i in range(24):
            ang = (i / 24.0) * 2.0 * math.pi
            px = rx + 0.35 * math.cos(ang)
            py = ry + 0.35 * math.sin(ang)
            sx, sy, _ = self.project_to_screen(px, py, 0.02)
            poly_estop.append(QPointF(sx, sy))
        p.drawPolygon(poly_estop)

        # 0.80m Proximity Boundary (Amber)
        p.setPen(QPen(QColor(217, 119, 6, 120), 1.2, Qt.DotLine))
        p.setBrush(Qt.NoBrush)
        poly_prox = QPolygonF()
        for i in range(24):
            ang = (i / 24.0) * 2.0 * math.pi
            px = rx + 0.80 * math.cos(ang)
            py = ry + 0.80 * math.sin(ang)
            sx, sy, _ = self.project_to_screen(px, py, 0.02)
            poly_prox.append(QPointF(sx, sy))
        p.drawPolygon(poly_prox)

    def _draw_robot_model(self, p: QPainter):
        """
        Render canonical GRaCEmo ViRa 3D robot model at (robot_x, robot_y, robot_yaw):
        - White & Orange 4-wheel mobile base with treads
        - Humanoid torso, articulated shoulders, neck, head display with glowing cyan visor
        - Planar LiDAR sensor mounted at Z = +1.175m
        - Directional orientation heading arrow
        """
        rx = self.robot_x
        ry = self.robot_y
        ryaw = self.robot_yaw

        cos_r = math.cos(ryaw)
        sin_r = math.sin(ryaw)

        def to_world(lx, ly, lz):
            wx = rx + (lx * cos_r - ly * sin_r)
            wy = ry + (lx * sin_r + ly * cos_r)
            wz = lz
            return self.project_to_screen(wx, wy, wz)

        # 1. Base Heading Arrow on ground
        p.setPen(QPen(QColor(255, 121, 0), 2.5))
        s_base, s_base_y, _ = to_world(0.0, 0.0, 0.03)
        s_tip, s_tip_y, _ = to_world(0.55, 0.0, 0.03)
        p.drawLine(QPointF(s_base, s_base_y), QPointF(s_tip, s_tip_y))

        # Arrow head
        s_a1, s_a1_y, _ = to_world(0.40, 0.10, 0.03)
        s_a2, s_a2_y, _ = to_world(0.40, -0.10, 0.03)
        p.drawLine(QPointF(s_tip, s_tip_y), QPointF(s_a1, s_a1_y))
        p.drawLine(QPointF(s_tip, s_tip_y), QPointF(s_a2, s_a2_y))

        # 2. 4 Rubber Wheels (Black with Orange Rim Accents)
        wheel_positions = [
            (0.18, 0.20), (0.18, -0.20),
            (-0.18, 0.20), (-0.18, -0.20)
        ]
        p.setPen(QPen(QColor(255, 121, 0), 1.2))
        p.setBrush(QBrush(QColor(34, 34, 34)))

        for wx_l, wy_l in wheel_positions:
            poly_w = QPolygonF()
            # Draw extruded wheel cylinder approximation
            for dz in (0.02, 0.14):
                for dx_w, dy_w in ((-0.06, -0.03), (0.06, -0.03), (0.06, 0.03), (-0.06, 0.03)):
                    sx, sy, _ = to_world(wx_l + dx_w, wy_l + dy_w, dz)
                    poly_w.append(QPointF(sx, sy))
            if len(poly_w) >= 3:
                p.drawPolygon(poly_w)

        # 3. Mobile Chassis (White & Dark Grey base plate)
        p.setPen(QPen(QColor(200, 200, 200), 1.0))
        p.setBrush(QBrush(QColor(245, 245, 245)))
        chassis_poly = QPolygonF()
        corners = [(-0.25, -0.18), (0.25, -0.18), (0.25, 0.18), (-0.25, 0.18)]
        for cx_l, cy_l in corners:
            sx, sy, _ = to_world(cx_l, cy_l, 0.22)
            chassis_poly.append(QPointF(sx, sy))
        p.drawPolygon(chassis_poly)

        # 4. Humanoid Torso (White body with Orange side racing stripes)
        # Main Torso Box: Z = 0.22 to 0.75m
        torso_poly = QPolygonF()
        torso_corners = [(-0.14, -0.14), (0.14, -0.14), (0.14, 0.14), (-0.14, 0.14)]
        p.setPen(QPen(QColor(200, 200, 200), 1.0))
        p.setBrush(QBrush(QColor(240, 240, 240)))
        for tx_l, ty_l in torso_corners:
            sx, sy, _ = to_world(tx_l, ty_l, 0.75)
            torso_poly.append(QPointF(sx, sy))
        p.drawPolygon(torso_poly)

        # Torso Orange Accent Badge
        p.setPen(Qt.NoPen)
        p.setBrush(QBrush(QColor(255, 121, 0)))
        badge_poly = QPolygonF()
        for bx_l, by_l in [(0.141, -0.08), (0.141, 0.08), (0.141, 0.08), (0.141, -0.08)]:
            sx, sy, _ = to_world(bx_l, by_l, 0.65)
            badge_poly.append(QPointF(sx, sy))
        if len(badge_poly) >= 3:
            p.drawPolygon(badge_poly)

        # 5. Articulated Shoulder Joints & Arms
        p.setPen(QPen(QColor(40, 45, 52), 1.0))
        p.setBrush(QBrush(QColor(255, 121, 0)))
        # Left shoulder & arm
        sx_ls, sy_ls, _ = to_world(0.0, 0.18, 0.70)
        p.drawEllipse(QPointF(sx_ls, sy_ls), 4.5, 4.5)
        sx_le, sy_le, _ = to_world(0.08, 0.20, 0.45)
        p.drawLine(QPointF(sx_ls, sy_ls), QPointF(sx_le, sy_le))

        # Right shoulder & arm
        sx_rs, sy_rs, _ = to_world(0.0, -0.18, 0.70)
        p.drawEllipse(QPointF(sx_rs, sy_rs), 4.5, 4.5)
        sx_re, sy_re, _ = to_world(0.08, -0.20, 0.45)
        p.drawLine(QPointF(sx_rs, sy_rs), QPointF(sx_re, sy_re))

        # 6. Neck & Head Display with Glowing Cyan Visor
        # Neck: Z = 0.75 to 0.85
        # Head: Z = 0.85 to 1.10
        p.setPen(QPen(QColor(30, 30, 30), 1.0))
        p.setBrush(QBrush(QColor(245, 245, 245)))
        head_poly = QPolygonF()
        head_corners = [(-0.11, -0.12), (0.11, -0.12), (0.11, 0.12), (-0.11, 0.12)]
        for hx_l, hy_l in head_corners:
            sx, sy, _ = to_world(hx_l, hy_l, 1.08)
            head_poly.append(QPointF(sx, sy))
        p.drawPolygon(head_poly)

        # Glowing Cyan Screen Face Display (#00D2FF)
        p.setPen(QPen(QColor(0, 210, 255), 1.5))
        p.setBrush(QBrush(QColor(0, 210, 255, 220)))
        screen_poly = QPolygonF()
        screen_pts = [
            (0.112, -0.09, 0.92), (0.112, 0.09, 0.92),
            (0.112, 0.09, 1.04), (0.112, -0.09, 1.04)
        ]
        for sx_l, sy_l, sz_l in screen_pts:
            sx, sy, _ = to_world(sx_l, sy_l, sz_l)
            screen_poly.append(QPointF(sx, sy))
        p.drawPolygon(screen_poly)

        # 7. Planar LiDAR Sensor Mast mounted at Z = +1.175m
        sx_mast, sy_mast, _ = to_world(0.0, 0.0, 1.175)
        p.setPen(QPen(QColor(30, 30, 30), 1.5))
        p.setBrush(QBrush(QColor(20, 25, 30)))
        p.drawEllipse(QPointF(sx_mast, sy_mast), 5.5, 5.5)

        # Dynamic rotating LiDAR beam indicator
        beam_ang = self.anim_phase
        bx = 0.40 * math.cos(beam_ang)
        by = 0.40 * math.sin(beam_ang)
        sx_b, sy_b, _ = to_world(bx, by, 1.175)
        p.setPen(QPen(QColor(0, 210, 255, 180), 1.2, Qt.DashLine))
        p.drawLine(QPointF(sx_mast, sy_mast), QPointF(sx_b, sy_b))

    def _draw_lidar_points(self, p: QPainter):
        """Render live LaserScan points in 3D world space."""
        p.setPen(Qt.NoPen)
        for wx, wy, intensity in self.scan_points:
            sx, sy, d = self.project_to_screen(wx, wy, 0.05)
            # Rainbow/intensity color: Green to Cyan
            c = QColor(
                int(20 + 200 * (1.0 - intensity)),
                int(220 * intensity + 35),
                int(255 * (1.0 - intensity)),
                210
            )
            p.setBrush(QBrush(c))
            size = max(1.5, min(4.5, 8.0 / d))
            p.drawEllipse(QPointF(sx, sy), size, size)

    def _draw_tf_frames(self, p: QPainter):
        """Draw RGB coordinate axes for main TF frames."""
        frames = [
            ("map", 0.0, 0.0, 0.0, 0.0),
            ("base_link", self.robot_x, self.robot_y, 0.10, self.robot_yaw),
            ("lidar_link", self.robot_x, self.robot_y, 1.175, self.robot_yaw),
        ]

        axis_len = 0.30
        for name, fx, fy, fz, fyaw in frames:
            sx_o, sy_o, _ = self.project_to_screen(fx, fy, fz)

            # X Axis (Red Forward)
            xx = fx + axis_len * math.cos(fyaw)
            xy = fy + axis_len * math.sin(fyaw)
            sx_x, sy_x, _ = self.project_to_screen(xx, xy, fz)
            p.setPen(QPen(QColor(COLOR_RVIZ_AXIS_X), 2.0))
            p.drawLine(QPointF(sx_o, sy_o), QPointF(sx_x, sy_x))

            # Y Axis (Green Left)
            yx = fx - axis_len * math.sin(fyaw)
            yy = fy + axis_len * math.cos(fyaw)
            sx_y, sy_y, _ = self.project_to_screen(yx, yy, fz)
            p.setPen(QPen(QColor(COLOR_RVIZ_AXIS_Y), 2.0))
            p.drawLine(QPointF(sx_o, sy_o), QPointF(sx_y, sy_y))

            # Z Axis (Blue Up)
            sx_z, sy_z, _ = self.project_to_screen(fx, fy, fz + axis_len)
            p.setPen(QPen(QColor(COLOR_RVIZ_AXIS_Z), 2.0))
            p.drawLine(QPointF(sx_o, sy_o), QPointF(sx_z, sy_z))

    def _draw_goal_arrow(self, p: QPainter):
        """Draw interactive arrow while user is dragging a 2D Nav Goal or Pose Estimate."""
        p.setPen(QPen(QColor(COLOR_RVIZ_SELECTION), 3.0))
        p.drawLine(self._nav_goal_start, self._nav_goal_end)

        p.setBrush(QBrush(QColor(COLOR_RVIZ_SELECTION)))
        p.drawEllipse(self._nav_goal_end, 5.0, 5.0)

    def _draw_screen_overlays(self, p: QPainter):
        """Draw viewport corner badges: coordinate frame tri-axis, view info, and mode banner."""
        w = self.width()
        h = self.height()

        # 1. Tri-Axis Widget at Bottom-Left
        ax_cx = 40.0
        ax_cy = h - 40.0
        ax_len = 24.0

        yaw_rad = math.radians(self.cam_yaw)
        pitch_rad = math.radians(self.cam_pitch)

        # X (Red)
        ax_x_x = ax_cx + ax_len * math.cos(yaw_rad)
        ax_x_y = ax_cy - ax_len * math.sin(yaw_rad) * math.sin(pitch_rad)
        p.setPen(QPen(QColor(COLOR_RVIZ_AXIS_X), 2.5))
        p.drawLine(QPointF(ax_cx, ax_cy), QPointF(ax_x_x, ax_x_y))
        p.drawText(int(ax_x_x + 3), int(ax_x_y + 3), "X")

        # Y (Green)
        ax_y_x = ax_cx - ax_len * math.sin(yaw_rad)
        ax_y_y = ax_cy - ax_len * math.cos(yaw_rad) * math.sin(pitch_rad)
        p.setPen(QPen(QColor(COLOR_RVIZ_AXIS_Y), 2.5))
        p.drawLine(QPointF(ax_cx, ax_cy), QPointF(ax_y_x, ax_y_y))
        p.drawText(int(ax_y_x + 3), int(ax_y_y + 3), "Y")

        # Z (Blue)
        ax_z_y = ax_cy - ax_len * math.cos(pitch_rad)
        p.setPen(QPen(QColor(COLOR_RVIZ_AXIS_Z), 2.5))
        p.drawLine(QPointF(ax_cx, ax_cy), QPointF(ax_cx, ax_z_y))
        p.drawText(int(ax_cx + 3), int(ax_z_y - 2), "Z")

        # 2. Viewport Header Info Bar (Top-Left)
        p.setFont(QFont(FONT_FAMILY_MONO, 9))
        p.setPen(QColor(160, 160, 160))
        p.drawText(12, 20, f"Fixed Frame: map  |  Target: <Fixed Frame>  |  Dist: {self.cam_dist:.1f}m  |  Pitch: {self.cam_pitch:.0f}°  |  Yaw: {self.cam_yaw:.0f}°")

        # 3. Active Tool Overlay Banner (Top-Right)
        if self.tool_mode in ("nav_goal", "pose_estimate"):
            tool_name = "2D NAV GOAL (Click & Drag heading)" if self.tool_mode == "nav_goal" else "2D POSE ESTIMATE"
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(QColor(245, 158, 11, 230)))
            p.drawRoundedRect(QRectF(w - 280, 8, 270, 24), 4, 4)
            p.setPen(QColor(0, 0, 0))
            p.setFont(QFont(FONT_FAMILY_PRIMARY, 9, QFont.Bold))
            p.drawText(int(w - 270), 24, tool_name)

    # -------------------------------------------------------------------------
    # Mouse & Keyboard Event Handlers
    # -------------------------------------------------------------------------
    def mousePressEvent(self, event):
        self._last_mouse_pos = event.position()
        self._drag_button = event.button()

        if self.tool_mode in ("nav_goal", "pose_estimate") and event.button() == Qt.LeftButton:
            self._nav_goal_start = event.position()
            self._nav_goal_end = event.position()
            self.update()

    def mouseMoveEvent(self, event):
        if not self._last_mouse_pos:
            return

        delta = event.position() - self._last_mouse_pos
        self._last_mouse_pos = event.position()

        # Handle Nav Goal / Pose Estimate Drag
        if self.tool_mode in ("nav_goal", "pose_estimate") and self._nav_goal_start:
            self._nav_goal_end = event.position()
            self.update()
            return

        # Handle Camera Pan (Middle click OR Shift + Left click)
        if self._drag_button == Qt.MiddleButton or (self._drag_button == Qt.LeftButton and (event.modifiers() & Qt.ShiftModifier)):
            pan_speed = 0.005 * self.cam_dist
            yaw_rad = math.radians(self.cam_yaw)
            dx = delta.x() * pan_speed
            dy = delta.y() * pan_speed

            self.focal_x -= (dx * math.cos(yaw_rad) - dy * math.sin(yaw_rad))
            self.focal_y += (dx * math.sin(yaw_rad) + dy * math.cos(yaw_rad))
            self.follow_robot = False
            self._emit_view_info()
            self.update()

        # Handle Camera Orbit (Left click drag or Move Camera tool)
        elif self._drag_button == Qt.LeftButton:
            self.cam_yaw = (self.cam_yaw - delta.x() * 0.4) % 360.0
            self.cam_pitch = max(2.0, min(89.9, self.cam_pitch - delta.y() * 0.4))
            self._emit_view_info()
            self.update()

        # Handle Camera Zoom (Right click drag)
        elif self._drag_button == Qt.RightButton:
            zoom_factor = 1.0 - (delta.y() * 0.01)
            self.cam_dist = max(1.0, min(30.0, self.cam_dist * zoom_factor))
            self._emit_view_info()
            self.update()

    def mouseReleaseEvent(self, event):
        if self.tool_mode in ("nav_goal", "pose_estimate") and self._nav_goal_start and self._nav_goal_end:
            # Convert screen coordinates to world ground plane
            wx1, wy1 = self.unproject_ground(self._nav_goal_start.x(), self._nav_goal_start.y())
            wx2, wy2 = self.unproject_ground(self._nav_goal_end.x(), self._nav_goal_end.y())

            yaw = math.atan2(wy2 - wy1, wx2 - wx1)

            if self.tool_mode == "nav_goal":
                self.goal_placed.emit(wx1, wy1, yaw)
                bus.navigation_goal_dispatched.emit("Custom 2D Nav Goal", wx1, wy1, yaw)
            elif self.tool_mode == "pose_estimate":
                self.pose_estimated.emit(wx1, wy1, yaw)
                bus.user_input_received.emit(f"set initial pose to {wx1:.2f}, {wy1:.2f}", "RVIZ_TOOL")

            self._nav_goal_start = None
            self._nav_goal_end = None
            self.set_tool_mode("interact")

        self._last_mouse_pos = None
        self._drag_button = None
        self.update()

    def wheelEvent(self, event):
        """Zoom camera via mouse scroll wheel."""
        delta = event.angleDelta().y()
        zoom_factor = 0.90 if delta > 0 else 1.10
        self.cam_dist = max(1.0, min(30.0, self.cam_dist * zoom_factor))
        self._emit_view_info()
        self.update()
