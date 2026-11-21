"""
Interactive 2D Occupancy Grid Navigation Map Widget for GRaCEmo ViRa.
Clean, Minimal Monochrome Engineering Edition:
- Automatic fit-to-map with content bounding box detection
- Automatic centering & dynamic viewport resizing (resizeEvent)
- Aspect-ratio preservation (strictly isotropic scaling)
- Mouse-wheel zoom around cursor & mouse-drag panning
- On-screen controls: Zoom In, Zoom Out, Zoom-to-Fit, Reset View
- Clear visual distinction: Free space (#FFFFFF), Occupied cells (#171717), Unknown cells (#F7F7F7)
- Robot position & orientation (chassis footprint, 4 wheels, heading cone)
- Global path (#2563EB) & Destination markers
"""

from PySide6.QtWidgets import QWidget, QFrame, QPushButton, QHBoxLayout, QLabel
from PySide6.QtCore import Qt, QPointF, QRectF, Signal
from PySide6.QtGui import QPainter, QImage, QColor, QPen, QBrush, QFont, QPolygonF

import math
import numpy as np
from typing import Dict, List, Tuple, Optional

from ..resources.styles.tokens import (
    COLOR_BG_MAIN,
    COLOR_WORKSPACE,
    COLOR_SUBTLE_SURFACE,
    COLOR_BORDER,
    COLOR_BORDER_LIGHT,
    COLOR_ACCENT_PRIMARY,
    COLOR_ACCENT_SECONDARY,
    COLOR_SUCCESS,
    COLOR_WARNING,
    COLOR_ERROR,
    COLOR_TEXT_PRIMARY,
    COLOR_TEXT_SECONDARY,
    COLOR_TEXT_MUTED,
    FONT_FAMILY_PRIMARY,
    FONT_FAMILY_MONO,
    BORDER_RADIUS_SM,
)


class MapViewWidget(QFrame):
    """Interactive 2D Nav2 occupancy grid and multi-room map viewer with auto-fit and pan/zoom."""

    destination_clicked = Signal(str)  # Emits destination key if clicked on map

    def __init__(self, parent=None):
        super().__init__(parent)
        self.map_image: Optional[QImage] = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.05  # meters/pixel
        self.origin_x = 0.0
        self.origin_y = 0.0

        # Explored map content bounding box (pixel coordinates in grid)
        self.content_min_x = 0
        self.content_max_x = 0
        self.content_min_y = 0
        self.content_max_y = 0
        self.has_content_bbox = False

        # Robot Pose
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_robot_pose = False

        # Global Path
        self.global_path: List[Tuple[float, float]] = []

        # Configured Destinations
        self.destinations: Dict[str, Dict] = {}
        self.active_goal_key: Optional[str] = None

        # Viewport Pan & Zoom
        self.zoom_scale = 1.0
        self.pan_offset_x = 0.0
        self.pan_offset_y = 0.0
        self._dragging = False
        self._last_mouse_pos = None
        self._user_panned = False

        self.setFrameShape(QFrame.StyledPanel)
        self.setMinimumSize(400, 350)
        self.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
            }}
        """)

        # Setup Overlay Controls Container (Top-Right of Map)
        self._setup_overlay_controls()

    def _setup_overlay_controls(self):
        """Add minimal monochrome overlay buttons for Zoom In, Zoom Out, Fit, and Reset."""
        self.controls_overlay = QWidget(self)
        self.controls_overlay.setStyleSheet(f"""
            QWidget {{
                background-color: rgba(255, 255, 255, 0.94);
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
            }}
            QPushButton {{
                background-color: #FFFFFF;
                color: {COLOR_TEXT_PRIMARY};
                border: 1px solid {COLOR_BORDER};
                border-radius: 3px;
                font-family: {FONT_FAMILY_PRIMARY};
                font-size: 11px;
                font-weight: 600;
                padding: 4px 8px;
                min-width: 22px;
                min-height: 20px;
            }}
            QPushButton:hover {{
                background-color: #F0F0F0;
                border-color: #BDBDBD;
            }}
            QPushButton:pressed {{
                background-color: #E5E5E5;
            }}
        """)

        overlay_layout = QHBoxLayout(self.controls_overlay)
        overlay_layout.setContentsMargins(6, 4, 6, 4)
        overlay_layout.setSpacing(4)

        self.btn_zoom_in = QPushButton("+", self.controls_overlay)
        self.btn_zoom_in.setToolTip("Zoom In (Mouse Wheel Up)")
        self.btn_zoom_in.clicked.connect(self.zoom_in)
        overlay_layout.addWidget(self.btn_zoom_in)

        self.btn_zoom_out = QPushButton("−", self.controls_overlay)
        self.btn_zoom_out.setToolTip("Zoom Out (Mouse Wheel Down)")
        self.btn_zoom_out.clicked.connect(self.zoom_out)
        overlay_layout.addWidget(self.btn_zoom_out)

        self.btn_fit = QPushButton("⊡ Fit", self.controls_overlay)
        self.btn_fit.setToolTip("Automatic Fit-to-Map (Fill Canvas)")
        self.btn_fit.clicked.connect(self.zoom_to_fit)
        overlay_layout.addWidget(self.btn_fit)

        self.btn_reset = QPushButton("⟲ Center", self.controls_overlay)
        self.btn_reset.setToolTip("Reset View & Center Map")
        self.btn_reset.clicked.connect(self.reset_view)
        overlay_layout.addWidget(self.btn_reset)

        self._position_overlay_controls()

    def _position_overlay_controls(self):
        """Keep overlay controls aligned to top-right of widget."""
        margin = 10
        ow = self.controls_overlay.sizeHint().width()
        oh = self.controls_overlay.sizeHint().height()
        self.controls_overlay.setGeometry(self.width() - ow - margin, margin, ow, oh)

    def resizeEvent(self, event):
        """Handle dynamic viewport resizing and auto-adjust map centering."""
        super().resizeEvent(event)
        self._position_overlay_controls()
        if self.map_image and not self._user_panned:
            self.zoom_to_fit()

    def set_map(self, width: int, height: int, resolution: float, origin_x: float, origin_y: float, data: List[int]):
        """Load and convert ROS 2 OccupancyGrid into clean monochrome QImage with bounding-box detection."""
        self.map_width = width
        self.map_height = height
        self.map_resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

        arr = np.array(data, dtype=np.int8).reshape((height, width))

        # Detect explored content bounds (cells that are free or occupied, not -1)
        known_y, known_x = np.where(arr != -1)
        if len(known_x) > 0:
            pad = 6  # 6 cells padding around discovered boundary
            self.content_min_x = max(0, int(known_x.min()) - pad)
            self.content_max_x = min(width - 1, int(known_x.max()) + pad)
            self.content_min_y = max(0, int(known_y.min()) - pad)
            self.content_max_y = min(height - 1, int(known_y.max()) + pad)
            self.has_content_bbox = True
        else:
            self.content_min_x = 0
            self.content_max_x = width - 1
            self.content_min_y = 0
            self.content_max_y = height - 1
            self.has_content_bbox = False

        # Convert to 4-channel RGBA image adhering to monochrome design
        img_data = np.zeros((height, width, 4), dtype=np.uint8)

        # 1. Unknown space (-1): Light neutral grey secondary background (#F7F7F7)
        unk_mask = (arr == -1)
        img_data[unk_mask] = [247, 247, 247, 255]

        # 2. Free space (0): Crisp pure white workspace (#FFFFFF)
        free_mask = (arr == 0)
        img_data[free_mask] = [255, 255, 255, 255]

        # 3. Occupied space (>50): Dark charcoal solid obstacle (#171717)
        occ_mask = (arr > 50)
        img_data[occ_mask] = [23, 23, 23, 255]

        # Flip vertically for Qt coordinate system (ROS row 0 is bottom)
        img_data = np.ascontiguousarray(np.flipud(img_data))

        qimg = QImage(img_data.data, width, height, width * 4, QImage.Format_RGBA8888).copy()
        self.map_image = qimg

        self.zoom_to_fit()
        self.update()

    def zoom_to_fit(self):
        """Fit explored map area to the available canvas, preserving exact aspect ratio and centering."""
        if not self.map_image or self.map_width == 0 or self.map_height == 0:
            return

        vw = max(50, self.width())
        vh = max(50, self.height())

        if self.has_content_bbox:
            cw = max(10, self.content_max_x - self.content_min_x + 1)
            ch = max(10, self.content_max_y - self.content_min_y + 1)
            scale_x = vw / float(cw)
            scale_y = vh / float(ch)
            self.zoom_scale = min(scale_x, scale_y) * 0.90

            # Content center in flipped image pixel space
            center_img_x = (self.content_min_x + self.content_max_x) / 2.0
            center_img_y = self.map_height - ((self.content_min_y + self.content_max_y) / 2.0)
        else:
            scale_x = vw / float(self.map_width)
            scale_y = vh / float(self.map_height)
            self.zoom_scale = min(scale_x, scale_y) * 0.90
            center_img_x = self.map_width / 2.0
            center_img_y = self.map_height / 2.0

        self.pan_offset_x = (vw / 2.0) - (center_img_x * self.zoom_scale)
        self.pan_offset_y = (vh / 2.0) - (center_img_y * self.zoom_scale)
        self._user_panned = False
        self.update()

    def reset_view(self):
        """Center full map image in view with standard scale."""
        if not self.map_image or self.map_width == 0 or self.map_height == 0:
            return
        vw = max(50, self.width())
        vh = max(50, self.height())
        fit_w = vw / float(self.map_width)
        fit_h = vh / float(self.map_height)
        self.zoom_scale = min(fit_w, fit_h) * 0.92
        self.pan_offset_x = (vw - (self.map_width * self.zoom_scale)) / 2.0
        self.pan_offset_y = (vh - (self.map_height * self.zoom_scale)) / 2.0
        self._user_panned = False
        self.update()

    def zoom_in(self):
        """Zoom in by 25% anchored at canvas center."""
        self._apply_zoom_step(1.25, self.width() / 2.0, self.height() / 2.0)

    def zoom_out(self):
        """Zoom out by 20% anchored at canvas center."""
        self._apply_zoom_step(0.80, self.width() / 2.0, self.height() / 2.0)

    def _apply_zoom_step(self, factor: float, anchor_x: float, anchor_y: float):
        new_scale = max(0.15, min(25.0, self.zoom_scale * factor))
        if abs(new_scale - self.zoom_scale) < 1e-4:
            return
        self.pan_offset_x = anchor_x - (anchor_x - self.pan_offset_x) * (new_scale / self.zoom_scale)
        self.pan_offset_y = anchor_y - (anchor_y - self.pan_offset_y) * (new_scale / self.zoom_scale)
        self.zoom_scale = new_scale
        self._user_panned = True
        self.update()

    def set_robot_pose(self, x: float, y: float, yaw: float):
        """Update localized robot pose and heading."""
        self.robot_x = x
        self.robot_y = y
        self.robot_yaw = yaw
        self.has_robot_pose = True
        self.update()

    def set_global_path(self, waypoints: List[Tuple[float, float]]):
        """Update active Nav2 path trajectory."""
        self.global_path = waypoints
        self.update()

    def set_destinations(self, dests: Dict[str, Dict]):
        """Set known destinations from settings."""
        self.destinations = dests
        self.update()

    def set_active_goal(self, key: Optional[str]):
        """Highlight currently targeted navigation destination."""
        self.active_goal_key = key
        self.update()

    def world_to_screen(self, wx: float, wy: float) -> Tuple[float, float]:
        """Convert ROS world coordinates (meters) to Qt widget screen pixels."""
        if not self.map_image or self.map_resolution == 0:
            return 0.0, 0.0
        mx = (wx - self.origin_x) / self.map_resolution
        my = self.map_height - ((wy - self.origin_y) / self.map_resolution)
        sx = self.pan_offset_x + (mx * self.zoom_scale)
        sy = self.pan_offset_y + (my * self.zoom_scale)
        return sx, sy

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._dragging = True
            self._last_mouse_pos = event.pos()
        elif event.button() == Qt.RightButton:
            self.zoom_to_fit()

    def mouseMoveEvent(self, event):
        if self._dragging and self._last_mouse_pos:
            delta = event.pos() - self._last_mouse_pos
            self.pan_offset_x += delta.x()
            self.pan_offset_y += delta.y()
            self._last_mouse_pos = event.pos()
            self._user_panned = True
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._dragging = False
            self._last_mouse_pos = None

    def wheelEvent(self, event):
        """Smooth mouse-wheel zoom anchored at cursor."""
        zoom_factor = 1.15 if event.angleDelta().y() > 0 else 0.85
        pos = event.position()
        self._apply_zoom_step(zoom_factor, pos.x(), pos.y())

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)

        w = self.width()
        h = self.height()

        if self.map_image:
            # Draw Occupancy Grid Image with pan and zoom
            target_rect = QRectF(
                self.pan_offset_x,
                self.pan_offset_y,
                self.map_width * self.zoom_scale,
                self.map_height * self.zoom_scale
            )
            painter.drawImage(target_rect, self.map_image)

            # Draw Map Bounding Frame Hairline
            painter.setPen(QPen(QColor(COLOR_BORDER), 1))
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(target_rect)

            # Draw Global Planned Path (Clean functional blue #2563EB)
            if len(self.global_path) > 1:
                path_pen = QPen(QColor(COLOR_ACCENT_SECONDARY), 2.5, Qt.SolidLine)
                painter.setPen(path_pen)
                for i in range(len(self.global_path) - 1):
                    x1, y1 = self.world_to_screen(self.global_path[i][0], self.global_path[i][1])
                    x2, y2 = self.world_to_screen(self.global_path[i+1][0], self.global_path[i+1][1])
                    painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

            # Draw Configured Destination Markers
            for dest_key, dest_data in self.destinations.items():
                gx = dest_data.get("x", 0.0)
                gy = dest_data.get("y", 0.0)
                dname = dest_data.get("name", dest_key)
                sx, sy = self.world_to_screen(gx, gy)

                is_active = (dest_key == self.active_goal_key)
                marker_color = QColor(COLOR_ACCENT_SECONDARY if is_active else COLOR_TEXT_PRIMARY)

                # Outer ring
                painter.setPen(QPen(marker_color, 2 if is_active else 1.5))
                painter.setBrush(QBrush(QColor(255, 255, 255, 220)))
                painter.drawEllipse(QPointF(sx, sy), 7, 7)

                # Center dot
                painter.setBrush(QBrush(marker_color))
                painter.setPen(Qt.NoPen)
                painter.drawEllipse(QPointF(sx, sy), 3, 3)

                # Label text
                painter.setPen(QColor(COLOR_TEXT_PRIMARY))
                painter.setFont(QFont(FONT_FAMILY_PRIMARY, 9, QFont.Bold if is_active else QFont.Medium))
                painter.drawText(int(sx + 10), int(sy + 4), dname)

            # Draw Robot Pose Marker & Heading
            if self.has_robot_pose:
                rx, ry = self.world_to_screen(self.robot_x, self.robot_y)
                screen_yaw = -self.robot_yaw
                cos_y = math.cos(screen_yaw)
                sin_y = math.sin(screen_yaw)

                def to_screen_pt(local_x: float, local_y: float) -> QPointF:
                    px = rx + (local_x * cos_y + local_y * sin_y)
                    py = ry + (local_x * sin_y - local_y * cos_y)
                    return QPointF(px, py)

                # Realistic Footprint Scale (0.58m x 0.40m)
                # Compute physical pixel dimension based on current zoom scale
                # 1 meter = (1 / map_resolution) * zoom_scale pixels
                ppm = (1.0 / self.map_resolution) * self.zoom_scale
                scale = max(18.0, min(65.0, ppm))  # Visual boundary bounds

                hw_x = 0.220 * (scale / 26.0)
                hw_y = 0.285 * (scale / 26.0)
                w_len = 0.115 * (scale / 26.0)
                w_th = 0.045 * (scale / 26.0)

                wheel_centers = [
                    (hw_x, hw_y),   # Front Left
                    (hw_x, -hw_y),  # Front Right
                    (-hw_x, hw_y),  # Rear Left
                    (-hw_x, -hw_y), # Rear Right
                ]

                # 1. 4 Skid-Steer Wheels (#171717 solid)
                painter.setBrush(QBrush(QColor("#171717")))
                painter.setPen(QPen(QColor("#404040"), 1))
                for wx, wy in wheel_centers:
                    w_poly = QPolygonF([
                        to_screen_pt(wx - w_len, wy - w_th),
                        to_screen_pt(wx + w_len, wy - w_th),
                        to_screen_pt(wx + w_len, wy + w_th),
                        to_screen_pt(wx - w_len, wy + w_th),
                    ])
                    painter.drawPolygon(w_poly)

                # 2. Main Chassis Body (#FFFFFF with crisp #171717 border)
                ch_x = 0.290 * (scale / 26.0)
                ch_y = 0.200 * (scale / 26.0)
                chassis_poly = QPolygonF([
                    to_screen_pt(-ch_x, -ch_y),
                    to_screen_pt(ch_x, -ch_y),
                    to_screen_pt(ch_x, ch_y),
                    to_screen_pt(-ch_x, ch_y),
                ])
                painter.setBrush(QBrush(QColor("#FFFFFF")))
                painter.setPen(QPen(QColor("#171717"), 1.8))
                painter.drawPolygon(chassis_poly)

                # 3. Upper Torso & Head Display Circle
                painter.setBrush(QBrush(QColor("#2563EB")))
                painter.setPen(QPen(QColor("#171717"), 1))
                head_pt = to_screen_pt(0.04 * (scale / 26.0), 0.0)
                painter.drawEllipse(head_pt, 4.0, 4.0)

                # 4. Crown LiDAR Mount Dot
                painter.setBrush(QBrush(QColor("#171717")))
                painter.setPen(Qt.NoPen)
                lidar_pt = to_screen_pt(0.0, 0.0)
                painter.drawEllipse(lidar_pt, 2.5, 2.5)

                # 5. Forward Heading Arrow (#171717)
                arrow_len = 28.0
                hx = rx + arrow_len * cos_y
                hy = ry + arrow_len * sin_y
                painter.setPen(QPen(QColor("#171717"), 2.0))
                painter.drawLine(QPointF(rx, ry), QPointF(hx, hy))

                # Heading Cone
                cone_len = 36.0
                left_angle = screen_yaw - 0.35
                right_angle = screen_yaw + 0.35
                cone = QPolygonF([
                    QPointF(rx, ry),
                    QPointF(rx + cone_len * math.cos(left_angle), ry + cone_len * math.sin(left_angle)),
                    QPointF(rx + cone_len * math.cos(right_angle), ry + cone_len * math.sin(right_angle))
                ])
                painter.setBrush(QBrush(QColor(37, 99, 235, 30)))  # Subtle blue cone tint
                painter.setPen(Qt.NoPen)
                painter.drawPolygon(cone)
        else:
            # Clean Technical Standby Grid
            painter.fillRect(self.rect(), QColor("#FFFFFF"))
            
            grid_pen = QPen(QColor("#F0F0F0"), 1, Qt.DotLine)
            painter.setPen(grid_pen)
            step = 30
            for gx in range(0, w, step):
                painter.drawLine(gx, 0, gx, h)
            for gy in range(0, h, step):
                painter.drawLine(0, gy, w, gy)

            # Central Status Badge Card
            bw, bh = 300, 70
            bx = int((w - bw) / 2.0)
            by = int((h - bh) / 2.0)
            painter.setPen(QPen(QColor(COLOR_BORDER), 1))
            painter.setBrush(QBrush(QColor(COLOR_WORKSPACE)))
            painter.drawRoundedRect(bx, by, bw, bh, 4, 4)

            # Text
            painter.setPen(QColor(COLOR_TEXT_PRIMARY))
            painter.setFont(QFont(FONT_FAMILY_PRIMARY, 10, QFont.Bold))
            painter.drawText(bx + 18, by + 28, "2D OCCUPANCY GRID MAP")

            painter.setPen(QColor(COLOR_TEXT_MUTED))
            painter.setFont(QFont(FONT_FAMILY_MONO, 9))
            painter.drawText(bx + 18, by + 48, "STANDBY: Waiting for Nav2 /map topic")

        # Top-Left Minimal Legend & Zoom Badge
        zoom_pct = int(round(self.zoom_scale * 100))
        legend_txt = f"2D OCCUPANCY MAP  |  Pan: Drag  •  Zoom: Scroll  •  Scale: {zoom_pct}%"
        legend_w = 340
        painter.fillRect(10, 10, legend_w, 24, QColor(255, 255, 255, 240))
        painter.setPen(QPen(QColor(COLOR_BORDER), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(10, 10, legend_w, 24, 3, 3)
        painter.setPen(QColor(COLOR_TEXT_SECONDARY))
        painter.setFont(QFont(FONT_FAMILY_PRIMARY, 9, QFont.Medium))
        painter.drawText(16, 26, legend_txt)
