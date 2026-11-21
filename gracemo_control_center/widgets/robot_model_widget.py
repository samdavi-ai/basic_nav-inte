"""
Canonical 3D Robot Model Widget for GRaCEmo ViRa UARCC.
Renders the authoritative white-and-orange 4-wheel mobile humanoid robot
designed in Blender, synchronized with live ROS 2 / Gazebo simulation telemetry.
"""

import math
from typing import List, Tuple, Dict, Any, Optional
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QPushButton, QButtonGroup, QToolButton
)
from PySide6.QtCore import Qt, QPointF, QTimer
from PySide6.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QPolygonF,
    QLinearGradient, QRadialGradient, QPainterPath
)

from ..resources.styles.tokens import (
    COLOR_WORKSPACE,
    COLOR_SUBTLE_SURFACE,
    COLOR_BORDER,
    COLOR_BORDER_LIGHT,
    COLOR_ACCENT_PRIMARY,
    COLOR_ACCENT_SECONDARY,
    COLOR_BRAND_ORANGE,
    COLOR_SUCCESS,
    COLOR_WARNING,
    COLOR_ERROR,
    COLOR_TEXT_PRIMARY,
    COLOR_TEXT_SECONDARY,
    COLOR_TEXT_MUTED,
    FONT_FAMILY_PRIMARY,
    FONT_FAMILY_MONO,
    BORDER_RADIUS_SM,
    BORDER_RADIUS_MD,
)
from ..core.event_bus import bus


# Canonical GRaCEmo ViRa Visual Palette (Matching Blender robot.blend / gracemo_palette.mtl)
COLOR_GRACE_WHITE   = QColor(245, 245, 245)
COLOR_GRACE_SHADOW  = QColor(215, 220, 225)
COLOR_GRACE_ORANGE  = QColor(255, 121, 0)      # #FF7900
COLOR_GRACE_ORANGE_DARK = QColor(204, 96, 0)
COLOR_GRACE_TIRE    = QColor(34, 34, 34)
COLOR_GRACE_CHASSIS = QColor(45, 52, 60)
COLOR_GRACE_SCREEN  = QColor(0, 210, 255)      # #00D2FF Glowing Cyan-Blue Face Display
COLOR_GRACE_LIDAR   = QColor(20, 25, 30)
COLOR_LIDAR_BEAM    = QColor(0, 210, 255, 45)


class RobotViewport(QWidget):
    """Interactive 3D vector canvas rendering canonical GRaCEmo ViRa geometry."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 300)
        self.setMouseTracking(True)

        # Camera view angles (degrees)
        self.cam_pitch = 22.0
        self.cam_yaw = -45.0
        self.cam_zoom = 1.0

        # Mouse dragging state
        self._last_mouse_pos = None
        self._is_dragging = False

        # Live Robot Telemetry State
        self.robot_roll = 0.0
        self.robot_pitch = 0.0
        self.robot_yaw = 0.0      # Radians
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.nav_state = "IDLE"
        self.follow_robot_yaw = False

        # Visual animation timer (simulates lidar sweep and wheel spin)
        self.anim_phase = 0.0
        self._anim_timer = QTimer(self)
        self._anim_timer.timeout.connect(self._on_anim_tick)
        self._anim_timer.start(50)  # 20 FPS

    def _on_anim_tick(self):
        self.anim_phase = (self.anim_phase + 0.08) % (2.0 * math.pi)
        self.update()

    def set_telemetry(self, roll: float, pitch: float, yaw: float, vx: float = 0.0, wz: float = 0.0):
        self.robot_roll = roll
        self.robot_pitch = pitch
        self.robot_yaw = yaw
        self.linear_speed = vx
        self.angular_speed = wz
        self.update()

    def set_nav_state(self, state: str):
        self.nav_state = state
        self.update()

    def set_view_mode(self, mode: str):
        if mode == "iso":
            self.cam_pitch = 22.0
            self.cam_yaw = -45.0
            self.cam_zoom = 1.0
        elif mode == "top":
            self.cam_pitch = 88.0
            self.cam_yaw = 0.0
            self.cam_zoom = 1.05
        elif mode == "front":
            self.cam_pitch = 0.0
            self.cam_yaw = 0.0
            self.cam_zoom = 1.0
        elif mode == "side":
            self.cam_pitch = 0.0
            self.cam_yaw = -90.0
            self.cam_zoom = 1.0
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._is_dragging = True
            self._last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self._is_dragging and self._last_mouse_pos:
            delta = event.pos() - self._last_mouse_pos
            self._last_mouse_pos = event.pos()
            self.cam_yaw += delta.x() * 0.7
            self.cam_pitch = max(-85.0, min(85.0, self.cam_pitch + delta.y() * 0.7))
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._is_dragging = False

    def wheelEvent(self, event):
        zoom_delta = event.angleDelta().y() / 1200.0
        self.cam_zoom = max(0.5, min(2.5, self.cam_zoom + zoom_delta))
        self.update()

    def project_3d(self, x: float, y: float, z: float, origin_x: float, origin_y: float, scale: float) -> Tuple[float, float, float]:
        """Project 3D coordinate (robot frame: X forward, Y left, Z up) to 2D screen coordinate."""
        # Optional orientation rotation by live robot_yaw if following
        eff_yaw = self.robot_yaw if self.follow_robot_yaw else 0.0
        cy_r = math.cos(eff_yaw)
        sy_r = math.sin(eff_yaw)
        rx = x * cy_r - y * sy_r
        ry = x * sy_r + y * cy_r
        rz = z

        # Camera rotation: yaw around Z, pitch around horizontal
        yaw_rad = math.radians(self.cam_yaw)
        pitch_rad = math.radians(self.cam_pitch)

        cos_y = math.cos(yaw_rad)
        sin_y = math.sin(yaw_rad)
        cos_p = math.cos(pitch_rad)
        sin_p = math.sin(pitch_rad)

        # Rotate around Z (camera yaw)
        cx = rx * cos_y - ry * sin_y
        cy = rx * sin_y + ry * cos_y
        cz = rz

        # Rotate around X-axis (camera pitch: looking down onto XY plane)
        # In robotics, Z is up. When camera pitches down, Z tilts toward screen Y
        eye_x = cy
        eye_y = -cz * cos_p + cx * sin_p
        depth = cz * sin_p + cx * cos_p  # for painter depth sorting

        # Isometric/orthographic projection with perspective scaling
        persp = 1.0 / (1.0 + depth * 0.001)
        sx = origin_x + eye_x * scale * self.cam_zoom * persp
        sy = origin_y + eye_y * scale * self.cam_zoom * persp
        return sx, sy, depth

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # Workspace background
        painter.fillRect(self.rect(), QColor(COLOR_WORKSPACE))

        # Subtle Floor Grid (Apartment Simulation Ground Plane)
        cx = w / 2.0
        cy = h / 2.0 + 35.0  # Center base on screen
        scale = min(w, h) * 0.38

        # Draw Ground Shadow & Grid
        self._draw_ground_grid(painter, cx, cy, scale)

        # Draw 3D Canonical GRaCEmo ViRa Robot Model
        self._draw_gracemo_robot(painter, cx, cy, scale)

        # Overlay HUD: Compass, Scale, and Navigation Badge
        self._draw_hud_overlays(painter, w, h)

    def _draw_ground_grid(self, painter: QPainter, cx: float, cy: float, scale: float):
        grid_pen = QPen(QColor(COLOR_BORDER_LIGHT), 1, Qt.SolidLine)
        painter.setPen(grid_pen)

        step = 0.3
        extent = 1.2
        xs = [i * step for i in range(-4, 5)]
        ys = [i * step for i in range(-4, 5)]

        for x in xs:
            p1 = self.project_3d(x, -extent, 0.0, cx, cy, scale)
            p2 = self.project_3d(x, extent, 0.0, cx, cy, scale)
            painter.drawLine(QPointF(p1[0], p1[1]), QPointF(p2[0], p2[1]))

        for y in ys:
            p1 = self.project_3d(-extent, y, 0.0, cx, cy, scale)
            p2 = self.project_3d(extent, y, 0.0, cx, cy, scale)
            painter.drawLine(QPointF(p1[0], p1[1]), QPointF(p2[0], p2[1]))

        # Ground contact shadow ellipse under robot chassis
        shadow_p = self.project_3d(0.0, 0.0, 0.005, cx, cy, scale)
        painter.setBrush(QBrush(QColor(24, 36, 51, 25)))
        painter.setPen(Qt.NoPen)
        rx_pix = 0.40 * scale * self.cam_zoom
        ry_pix = 0.22 * scale * self.cam_zoom * max(0.2, math.sin(max(0.1, math.radians(self.cam_pitch))))
        painter.drawEllipse(QPointF(shadow_p[0], shadow_p[1]), rx_pix, ry_pix)

    def _draw_box(self, painter: QPainter, x_min: float, x_max: float, y_min: float, y_max: float,
                  z_min: float, z_max: float, fill_color: QColor, border_color: QColor,
                  cx: float, cy: float, scale: float):
        """Draw an oriented 3D rectangular box with basic lighting."""
        # 8 corners
        c = [
            self.project_3d(x_min, y_min, z_min, cx, cy, scale),
            self.project_3d(x_max, y_min, z_min, cx, cy, scale),
            self.project_3d(x_max, y_max, z_min, cx, cy, scale),
            self.project_3d(x_min, y_max, z_min, cx, cy, scale),
            self.project_3d(x_min, y_min, z_max, cx, cy, scale),
            self.project_3d(x_max, y_min, z_max, cx, cy, scale),
            self.project_3d(x_max, y_max, z_max, cx, cy, scale),
            self.project_3d(x_min, y_max, z_max, cx, cy, scale),
        ]

        faces = [
            # Top face (Z max)
            ([4, 5, 6, 7], fill_color),
            # Front face (X max)
            ([1, 2, 6, 5], fill_color.darker(110)),
            # Back face (X min)
            ([0, 3, 7, 4], fill_color.darker(125)),
            # Left face (Y max)
            ([2, 3, 7, 6], fill_color.darker(118)),
            # Right face (Y min)
            ([0, 1, 5, 4], fill_color.darker(115)),
        ]

        painter.setPen(QPen(border_color, 1))

        # Sort faces by average depth
        sorted_faces = []
        for indices, col in faces:
            avg_depth = sum(c[i][2] for i in indices) / 4.0
            sorted_faces.append((avg_depth, indices, col))
        sorted_faces.sort(key=lambda item: item[0])

        for _, indices, col in sorted_faces:
            poly = QPolygonF([QPointF(c[i][0], c[i][1]) for i in indices])
            painter.setBrush(QBrush(col))
            painter.drawPolygon(poly)

    def _draw_gracemo_robot(self, painter: QPainter, cx: float, cy: float, scale: float):
        """Render the complete canonical Blender GRaCEmo ViRa hierarchy."""

        # 1. Four Skid-Steer Wheels (Track width 0.570m, Wheelbase 0.440m, Radius 0.115m, Width 0.100m)
        hw_x = 0.220  # half wheelbase
        hw_y = 0.285  # half track
        w_rad = 0.115
        w_w = 0.050   # half wheel width

        wheel_positions = [
            ("FL",  hw_x,  hw_y),
            ("FR",  hw_x, -hw_y),
            ("RL", -hw_x,  hw_y),
            ("RR", -hw_x, -hw_y),
        ]

        for name, wx, wy in wheel_positions:
            y_sign = 1.0 if wy > 0 else -1.0
            # Tire block
            self._draw_box(
                painter,
                wx - w_rad, wx + w_rad,
                wy - w_w, wy + w_w,
                0.0, 2.0 * w_rad,
                COLOR_GRACE_TIRE, QColor(20, 20, 20),
                cx, cy, scale
            )
            # Orange Hubcap Accent Ring (#FF7900)
            hub_center = self.project_3d(wx, wy + y_sign * w_w * 1.05, w_rad, cx, cy, scale)
            painter.setBrush(QBrush(COLOR_GRACE_ORANGE))
            painter.setPen(QPen(COLOR_GRACE_ORANGE_DARK, 1))
            hub_r = 0.06 * scale * self.cam_zoom
            painter.drawEllipse(QPointF(hub_center[0], hub_center[1]), hub_r, hub_r * 0.75)

        # 2. Lower Chassis Base (Deck Z=0.115m to 0.250m)
        self._draw_box(
            painter,
            -0.290, 0.290,
            -0.200, 0.200,
            0.080, 0.240,
            COLOR_GRACE_CHASSIS, QColor(30, 36, 42),
            cx, cy, scale
        )

        # 3. Upper Chassis White Cowling & Orange Accent Belt (#FF7900)
        # Orange Accent Belt
        self._draw_box(
            painter,
            -0.295, 0.295,
            -0.205, 0.205,
            0.240, 0.270,
            COLOR_GRACE_ORANGE, COLOR_GRACE_ORANGE_DARK,
            cx, cy, scale
        )
        # White Deck Cowling (Deck Z=0.270 to 0.320m)
        self._draw_box(
            painter,
            -0.280, 0.280,
            -0.190, 0.190,
            0.270, 0.320,
            COLOR_GRACE_WHITE, COLOR_GRACE_SHADOW,
            cx, cy, scale
        )

        # 4. Humanoid Torso / Chest (Z=0.320m to 0.850m)
        # White sculpted chest armor
        self._draw_box(
            painter,
            -0.130, 0.130,
            -0.170, 0.170,
            0.320, 0.780,
            COLOR_GRACE_WHITE, COLOR_GRACE_SHADOW,
            cx, cy, scale
        )

        # Torso Chest Emblem / Orange Chevron (#FF7900)
        c_top = self.project_3d(0.135, 0.0, 0.680, cx, cy, scale)
        c_left = self.project_3d(0.135, 0.065, 0.610, cx, cy, scale)
        c_right = self.project_3d(0.135, -0.065, 0.610, cx, cy, scale)
        c_bot = self.project_3d(0.135, 0.0, 0.540, cx, cy, scale)
        chevron = QPolygonF([
            QPointF(c_top[0], c_top[1]),
            QPointF(c_left[0], c_left[1]),
            QPointF(c_bot[0], c_bot[1]),
            QPointF(c_right[0], c_right[1]),
        ])
        painter.setBrush(QBrush(COLOR_GRACE_ORANGE))
        painter.setPen(QPen(COLOR_GRACE_ORANGE_DARK, 1))
        painter.drawPolygon(chevron)

        # 5. Dual Humanoid Arms
        # Left Arm (Fixed / Posed resting arm)
        # Left Shoulder
        self._draw_box(
            painter,
            -0.060, 0.060,
            0.170, 0.240,
            0.680, 0.760,
            COLOR_GRACE_WHITE, COLOR_GRACE_SHADOW,
            cx, cy, scale
        )
        # Left Upper Arm & Forearm
        self._draw_box(
            painter,
            -0.050, 0.050,
            0.190, 0.250,
            0.420, 0.680,
            COLOR_GRACE_WHITE, COLOR_GRACE_SHADOW,
            cx, cy, scale
        )
        # Left Hand
        self._draw_box(
            painter,
            -0.040, 0.040,
            0.200, 0.245,
            0.350, 0.420,
            COLOR_GRACE_CHASSIS, QColor(30, 36, 42),
            cx, cy, scale
        )

        # Right Arm (7-DOF Articulated Arm Assembly)
        # Right Shoulder
        self._draw_box(
            painter,
            -0.060, 0.060,
            -0.240, -0.170,
            0.680, 0.760,
            COLOR_GRACE_WHITE, COLOR_GRACE_SHADOW,
            cx, cy, scale
        )
        # Right Upper Arm (Orange Joint Accent)
        self._draw_box(
            painter,
            -0.050, 0.050,
            -0.250, -0.190,
            0.520, 0.680,
            COLOR_GRACE_WHITE, COLOR_GRACE_SHADOW,
            cx, cy, scale
        )
        # Right Elbow & Forearm (Angled forward slightly)
        self._draw_box(
            painter,
            -0.020, 0.120,
            -0.245, -0.195,
            0.420, 0.520,
            COLOR_GRACE_WHITE, COLOR_GRACE_SHADOW,
            cx, cy, scale
        )
        # Right Wrist & Gripper Fingers
        self._draw_box(
            painter,
            0.080, 0.180,
            -0.240, -0.200,
            0.400, 0.450,
            COLOR_GRACE_CHASSIS, COLOR_GRACE_ORANGE,
            cx, cy, scale
        )

        # 6. Neck & Head / Display Assembly (Neck Z=0.780 to 0.860m, Head Z=0.860 to 1.120m)
        # Neck Pillar
        self._draw_box(
            painter,
            -0.050, 0.050,
            -0.050, 0.050,
            0.780, 0.860,
            COLOR_GRACE_CHASSIS, QColor(30, 36, 42),
            cx, cy, scale
        )

        # Head Helmet Shell
        self._draw_box(
            painter,
            -0.120, 0.120,
            -0.140, 0.140,
            0.860, 1.100,
            COLOR_GRACE_WHITE, COLOR_GRACE_SHADOW,
            cx, cy, scale
        )

        # Head Front Display Visor Screen (Glowing Cyan-Blue #00D2FF GraceEMO Face)
        screen_tl = self.project_3d(0.125, 0.110, 1.070, cx, cy, scale)
        screen_tr = self.project_3d(0.125, -0.110, 1.070, cx, cy, scale)
        screen_br = self.project_3d(0.125, -0.110, 0.900, cx, cy, scale)
        screen_bl = self.project_3d(0.125, 0.110, 0.900, cx, cy, scale)
        screen_poly = QPolygonF([
            QPointF(screen_tl[0], screen_tl[1]),
            QPointF(screen_tr[0], screen_tr[1]),
            QPointF(screen_br[0], screen_br[1]),
            QPointF(screen_bl[0], screen_bl[1]),
        ])
        painter.setBrush(QBrush(COLOR_GRACE_SCREEN))
        painter.setPen(QPen(QColor(0, 160, 210), 1.5))
        painter.drawPolygon(screen_poly)

        # Expressive Facial Eye Glow (GraceEMO Visage)
        eye_l = self.project_3d(0.126, 0.045, 1.000, cx, cy, scale)
        eye_r = self.project_3d(0.126, -0.045, 1.000, cx, cy, scale)
        painter.setBrush(QBrush(QColor(255, 255, 255)))
        painter.setPen(Qt.NoPen)
        er = 3.5 * self.cam_zoom
        painter.drawEllipse(QPointF(eye_l[0], eye_l[1]), er, er * 1.4)
        painter.drawEllipse(QPointF(eye_r[0], eye_r[1]), er, er * 1.4)

        # RGB Camera Sensor Bar (Under screen Z=0.880m)
        cam_bar = self.project_3d(0.128, 0.0, 0.880, cx, cy, scale)
        painter.setBrush(QBrush(QColor(15, 20, 25)))
        painter.setPen(QPen(COLOR_GRACE_ORANGE, 1))
        painter.drawRoundedRect(int(cam_bar[0] - 12), int(cam_bar[1] - 3), 24, 6, 2, 2)

        # 7. Crown Planar LiDAR (Mounted on Head Top Z=1.100m to 1.175m)
        self._draw_box(
            painter,
            -0.060, 0.060,
            -0.060, 0.060,
            1.100, 1.160,
            COLOR_GRACE_LIDAR, QColor(10, 10, 15),
            cx, cy, scale
        )
        lidar_top = self.project_3d(0.0, 0.0, 1.175, cx, cy, scale)
        painter.setBrush(QBrush(COLOR_GRACE_ORANGE))
        painter.setPen(Qt.NoPen)
        lr = 0.05 * scale * self.cam_zoom
        painter.drawEllipse(QPointF(lidar_top[0], lidar_top[1]), lr, lr * 0.7)

        # Active 360° LiDAR Scan Pulse Fan
        sweep_angle = self.anim_phase
        for offset in (0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0):
            ang = sweep_angle + offset
            lx = 0.80 * math.cos(ang)
            ly = 0.80 * math.sin(ang)
            beam_end = self.project_3d(lx, ly, 1.160, cx, cy, scale)
            painter.setPen(QPen(COLOR_LIDAR_BEAM, 1.5, Qt.DashLine))
            painter.drawLine(QPointF(lidar_top[0], lidar_top[1]), QPointF(beam_end[0], beam_end[1]))

    def _draw_hud_overlays(self, painter: QPainter, w: int, h: int):
        # 1. Model Badge (Top-Left)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(255, 255, 255, 230)))
        painter.drawRoundedRect(12, 12, 230, 54, 4, 4)
        painter.setPen(QPen(QColor(COLOR_BORDER), 1))
        painter.drawRoundedRect(12, 12, 230, 54, 4, 4)

        # Model Title & Subtitle
        painter.setFont(QFont(FONT_FAMILY_PRIMARY, 9, QFont.Bold))
        painter.setPen(QColor(COLOR_TEXT_PRIMARY))
        painter.drawText(22, 30, "GRaCEmo ViRa (Canonical)")

        painter.setFont(QFont(FONT_FAMILY_PRIMARY, 8, QFont.Normal))
        painter.setPen(QColor(COLOR_TEXT_SECONDARY))
        painter.drawText(22, 46, "Blender Master CAD | 4-Wheel Humanoid")

        painter.setFont(QFont(FONT_FAMILY_MONO, 7, QFont.Normal))
        painter.setPen(QColor(COLOR_BRAND_ORANGE))
        painter.drawText(22, 58, "ORANGE & WHITE SPEC: #FF7900")

        # 2. Navigation State Badge (Top-Right)
        state_col = QColor(COLOR_SUCCESS) if self.nav_state in ("IDLE", "SUCCEEDED") else QColor(COLOR_ACCENT_PRIMARY)
        state_bg = QColor(state_col.red(), state_col.green(), state_col.blue(), 25)

        badge_w, badge_h = 130, 28
        bx = w - badge_w - 12
        by = 12

        painter.setPen(QPen(state_col, 1))
        painter.setBrush(QBrush(state_bg))
        painter.drawRoundedRect(bx, by, badge_w, badge_h, 3, 3)

        painter.setFont(QFont(FONT_FAMILY_PRIMARY, 8, QFont.Bold))
        painter.setPen(state_col)
        painter.drawText(bx + 10, by + 18, f"● {self.nav_state}")

        # 3. View Angle / Orbit Help (Bottom-Left)
        painter.setFont(QFont(FONT_FAMILY_MONO, 7))
        painter.setPen(QColor(COLOR_TEXT_MUTED))
        painter.drawText(12, h - 10, f"Pitch: {self.cam_pitch:.1f}° | Yaw: {self.cam_yaw:.1f}° | Drag to orbit | Scroll to zoom")


class CanonicalRobotModelWidget(QFrame):
    """Complete Canonical Robot Model Visualizer Card with view controls and specs."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_MD}px;
            }}
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(14, 12, 14, 12)
        layout.setSpacing(10)

        # Header Bar with Preset View Controls
        header_row = QHBoxLayout()
        header_row.setSpacing(8)

        title_col = QVBoxLayout()
        title_col.setSpacing(2)
        title_lbl = QLabel("CANONICAL 3D ROBOT MODEL — GRACEMO VIRA", self)
        title_lbl.setStyleSheet(f"color: {COLOR_ACCENT_PRIMARY}; font-weight: 700; font-size: 11px; letter-spacing: 0.5px;")
        sub_lbl = QLabel("Authoritative Blender Reference (robot.blend) | Gazebo & RViz Parity", self)
        sub_lbl.setStyleSheet(f"color: {COLOR_TEXT_SECONDARY}; font-size: 10px;")
        title_col.addWidget(title_lbl)
        title_col.addWidget(sub_lbl)
        header_row.addLayout(title_col)

        header_row.addStretch()

        # View Mode Buttons
        btn_style = f"""
            QPushButton {{
                background-color: {COLOR_SUBTLE_SURFACE};
                color: {COLOR_TEXT_PRIMARY};
                font-family: {FONT_FAMILY_PRIMARY};
                font-size: 10px;
                font-weight: 600;
                padding: 4px 10px;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
            }}
            QPushButton:hover {{
                background-color: #E2E8F0;
                border-color: {COLOR_ACCENT_PRIMARY};
            }}
            QPushButton:checked {{
                background-color: {COLOR_ACCENT_PRIMARY};
                color: #FFFFFF;
                border-color: {COLOR_ACCENT_PRIMARY};
            }}
        """

        self.btn_iso = QPushButton("3D Iso", self)
        self.btn_top = QPushButton("Top", self)
        self.btn_front = QPushButton("Front", self)
        self.btn_side = QPushButton("Side", self)

        for btn in (self.btn_iso, self.btn_top, self.btn_front, self.btn_side):
            btn.setStyleSheet(btn_style)
            btn.setCheckable(True)
            header_row.addWidget(btn)

        self.btn_iso.setChecked(True)

        self.btn_group = QButtonGroup(self)
        self.btn_group.addButton(self.btn_iso)
        self.btn_group.addButton(self.btn_top)
        self.btn_group.addButton(self.btn_front)
        self.btn_group.addButton(self.btn_side)

        self.btn_iso.clicked.connect(lambda: self.viewport.set_view_mode("iso"))
        self.btn_top.clicked.connect(lambda: self.viewport.set_view_mode("top"))
        self.btn_front.clicked.connect(lambda: self.viewport.set_view_mode("front"))
        self.btn_side.clicked.connect(lambda: self.viewport.set_view_mode("side"))

        layout.addLayout(header_row)

        # 3D Viewport
        self.viewport = RobotViewport(self)
        layout.addWidget(self.viewport, stretch=1)

        # Specifications & Physical Parity Footer Card
        spec_frame = QFrame(self)
        spec_frame.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_SUBTLE_SURFACE};
                border: 1px solid {COLOR_BORDER_LIGHT};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 6px;
            }}
        """)
        spec_layout = QHBoxLayout(spec_frame)
        spec_layout.setContentsMargins(8, 4, 8, 4)
        spec_layout.setSpacing(16)

        specs = [
            ("CHASSIS", "4-Wheel Skid-Steer"),
            ("TRACK WIDTH", "0.570 m"),
            ("WHEELBASE", "0.440 m"),
            ("WHEEL RADIUS", "0.115 m"),
            ("HEIGHT", "1.175 m (Crown LiDAR)"),
            ("MASS", "22.2 kg"),
            ("DOFs", "4W + 2 Neck + 7 Arm"),
        ]

        for title, val in specs:
            col = QVBoxLayout()
            col.setSpacing(1)
            t_lbl = QLabel(title, self)
            t_lbl.setStyleSheet(f"font-size: 8px; color: {COLOR_TEXT_MUTED}; font-weight: 700; letter-spacing: 0.5px;")
            v_lbl = QLabel(val, self)
            v_lbl.setStyleSheet(f"font-size: 10px; color: {COLOR_TEXT_PRIMARY}; font-weight: 600; font-family: {FONT_FAMILY_MONO};")
            col.addWidget(t_lbl)
            col.addWidget(v_lbl)
            spec_layout.addLayout(col)

        layout.addWidget(spec_frame)

        # Wire live signals from EventBus
        bus.imu_data_ready.connect(self._on_imu_data)
        bus.odom_data_ready.connect(self._on_odom_data)
        bus.autonomy_state_ready.connect(self.viewport.set_nav_state)

    def _on_imu_data(self, data: dict):
        if isinstance(data, dict):
            r = data.get("roll", 0.0)
            p = data.get("pitch", 0.0)
            y = data.get("yaw", 0.0)
            self.viewport.set_telemetry(r, p, y)

    def _on_odom_data(self, data: dict):
        if isinstance(data, dict):
            vx = data.get("vx", 0.0)
            wz = data.get("wz", 0.0)
            yaw = data.get("yaw", 0.0)
            self.viewport.set_telemetry(self.viewport.robot_roll, self.viewport.robot_pitch, yaw, vx, wz)
