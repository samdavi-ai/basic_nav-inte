"""
Module 4: Live Sensor Center for GRaCEmo ViRa UARCC.
Professional multi-modal instrumentation dashboard for RGB Camera, Depth Camera,
360-Degree LiDAR, IMU 6-DOF Attitude, and Differential-Drive Odometry.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QFrame, QTabWidget, QTableWidget, QTableWidgetItem, QHeaderView,
    QScrollArea
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QColor

import math
import time
from typing import Dict, Any

from ...resources.styles.tokens import (
    COLOR_BG_MAIN,
    COLOR_WORKSPACE,
    COLOR_SUBTLE_SURFACE,
    COLOR_BORDER,
    COLOR_ACCENT_PRIMARY,
    COLOR_ACCENT_SECONDARY,
    COLOR_SUCCESS,
    COLOR_WARNING,
    COLOR_ERROR,
    COLOR_OFFLINE,
    COLOR_TEXT_PRIMARY,
    COLOR_TEXT_SECONDARY,
    COLOR_TEXT_MUTED,
    FONT_FAMILY_PRIMARY,
    FONT_FAMILY_MONO,
    BORDER_RADIUS_SM,
    BORDER_RADIUS_MD,
)
from ...widgets.camera_view import CameraViewWidget
from ...widgets.depth_view import DepthViewWidget
from ...widgets.lidar_view import LidarViewWidget
from ...widgets.robot_pose_widget import RobotPoseWidget
from ...widgets.status_indicator import StatusIndicator
from ...widgets.common_components import PageHeader, SectionHeader, MetricDisplay
from ...core.event_bus import bus


class SensorsView(QWidget):
    """Module 4: Comprehensive multi-modal sensor telemetry center."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self.sensor_timestamps = {
            "rgb": 0.0,
            "depth": 0.0,
            "lidar": 0.0,
            "imu": 0.0,
            "odom": 0.0
        }
        self.sensor_counts = {
            "rgb": 0,
            "depth": 0,
            "lidar": 0,
            "imu": 0,
            "odom": 0
        }
        self.sensor_rates = {
            "rgb": 0.0,
            "depth": 0.0,
            "lidar": 0.0,
            "imu": 0.0,
            "odom": 0.0
        }
        self._last_calc_time = time.time()

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(14)

        # 1. Page Header
        self.header = PageHeader(
            title="Sensor Telemetry & Perception",
            subtitle="Multi-modal perception feeds: Forward RGB, Depth map, 360° LiDAR, IMU attitude, and Odometry",
            badge_title="Perception Stream",
            initial_state=StatusIndicator.STATE_READY,
            target_page="Sensors",
            parent=self
        )
        layout.addWidget(self.header)

        # 2. Sensor Health & Subsystems Status Matrix (Table)
        matrix_card = QFrame(self)
        matrix_card.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_MD}px;
            }}
        """)
        matrix_layout = QVBoxLayout(matrix_card)
        matrix_layout.setContentsMargins(12, 10, 12, 10)
        matrix_layout.setSpacing(6)

        m_header = SectionHeader("Perception Subsystem Health Matrix & Data Age", parent=self)
        matrix_layout.addWidget(m_header)

        self.health_table = QTableWidget(5, 6, self)
        self.health_table.setHorizontalHeaderLabels([
            "Subsystem", "Topic / Endpoint", "Frame ID", "Status", "Rate (Hz)", "Data Age"
        ])
        self.health_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.health_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.health_table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.health_table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.health_table.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeToContents)
        self.health_table.horizontalHeader().setSectionResizeMode(5, QHeaderView.ResizeToContents)
        self.health_table.verticalHeader().setVisible(False)
        self.health_table.setSelectionMode(QTableWidget.NoSelection)
        self.health_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.health_table.setFixedHeight(175)
        self.health_table.setStyleSheet(f"""
            QTableWidget {{
                background-color: {COLOR_WORKSPACE};
                border: none;
                gridline-color: {COLOR_BORDER};
            }}
        """)

        # Initial Rows
        sensors_meta = [
            ("Forward RGB Camera", "/camera/image_raw", "camera_optical_frame"),
            ("Depth Sensor", "/camera/depth_image", "camera_depth_optical_frame"),
            ("360° Planar LiDAR", "/scan", "lidar_link"),
            ("IMU 6-DOF", "/imu/data", "imu_link"),
            ("Differential Odometry", "/odom", "odom")
        ]

        for row, (name, topic, frame) in enumerate(sensors_meta):
            item_name = QTableWidgetItem(name)
            item_name.setFont(QFont(FONT_FAMILY_PRIMARY, 9, QFont.Bold))
            self.health_table.setItem(row, 0, item_name)

            item_topic = QTableWidgetItem(topic)
            item_topic.setFont(QFont(FONT_FAMILY_MONO, 9))
            item_topic.setForeground(QColor(COLOR_TEXT_SECONDARY))
            self.health_table.setItem(row, 1, item_topic)

            item_frame = QTableWidgetItem(frame)
            item_frame.setFont(QFont(FONT_FAMILY_MONO, 9))
            item_frame.setForeground(QColor(COLOR_TEXT_MUTED))
            self.health_table.setItem(row, 2, item_frame)

            item_status = QTableWidgetItem("UNAVAILABLE")
            item_status.setFont(QFont(FONT_FAMILY_MONO, 9, QFont.Bold))
            item_status.setForeground(QColor(COLOR_OFFLINE))
            self.health_table.setItem(row, 3, item_status)

            item_rate = QTableWidgetItem("-- Hz")
            item_rate.setFont(QFont(FONT_FAMILY_MONO, 9))
            item_rate.setForeground(QColor(COLOR_TEXT_MUTED))
            self.health_table.setItem(row, 4, item_rate)

            item_age = QTableWidgetItem("No Data")
            item_age.setFont(QFont(FONT_FAMILY_MONO, 9))
            item_age.setForeground(QColor(COLOR_TEXT_MUTED))
            self.health_table.setItem(row, 5, item_age)

        matrix_layout.addWidget(self.health_table)
        layout.addWidget(matrix_card)

        # 3. Tab Widget for Sensor Views
        self.tabs = QTabWidget(self)

        # === TAB 1: Vision & Planar LiDAR Center ===
        tab_vision_lidar = QWidget()
        vl_layout = QHBoxLayout(tab_vision_lidar)
        vl_layout.setContentsMargins(8, 8, 8, 8)
        vl_layout.setSpacing(12)

        # Left Column: Optical Cameras (RGB + Depth)
        cameras_col = QVBoxLayout()
        cameras_col.setSpacing(10)

        # RGB Camera Card
        rgb_card = QFrame(tab_vision_lidar)
        rgb_card.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_MD}px;
            }}
        """)
        rgb_layout = QVBoxLayout(rgb_card)
        rgb_layout.setContentsMargins(8, 8, 8, 8)
        rgb_layout.setSpacing(6)
        rgb_hdr = SectionHeader("Forward RGB Camera Feed (/camera/image_raw)", parent=self)
        rgb_layout.addWidget(rgb_hdr)
        self.rgb_view = CameraViewWidget("RGB FORWARD CAMERA", self)
        rgb_layout.addWidget(self.rgb_view, stretch=1)
        cameras_col.addWidget(rgb_card, stretch=1)

        # Depth Camera Card
        depth_card = QFrame(tab_vision_lidar)
        depth_card.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_MD}px;
            }}
        """)
        depth_layout = QVBoxLayout(depth_card)
        depth_layout.setContentsMargins(8, 8, 8, 8)
        depth_layout.setSpacing(6)
        depth_hdr = SectionHeader("Depth Sensor Preview (/camera/depth_image)", parent=self)
        depth_layout.addWidget(depth_hdr)
        self.depth_view = DepthViewWidget("DEPTH SENSOR", self)
        depth_layout.addWidget(self.depth_view, stretch=1)
        cameras_col.addWidget(depth_card, stretch=1)

        vl_layout.addLayout(cameras_col, stretch=1)

        # Right Column: Planar LiDAR Radar Sweep
        lidar_card = QFrame(tab_vision_lidar)
        lidar_card.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_MD}px;
            }}
        """)
        lc_layout = QVBoxLayout(lidar_card)
        lc_layout.setContentsMargins(8, 8, 8, 8)
        lc_layout.setSpacing(6)
        lidar_hdr = SectionHeader("360° Planar LiDAR Polar Sweep (/scan on lidar_link)", parent=self)
        lc_layout.addWidget(lidar_hdr)
        self.lidar_view = LidarViewWidget("360° PLANAR LIDAR", self)
        lc_layout.addWidget(self.lidar_view, stretch=1)
        vl_layout.addWidget(lidar_card, stretch=1)

        self.tabs.addTab(tab_vision_lidar, "Vision && LiDAR Streams")

        # === TAB 2: Kinematics & IMU Attitude Center ===
        tab_kinematics = QWidget()
        k_layout = QHBoxLayout(tab_kinematics)
        k_layout.setContentsMargins(12, 12, 12, 12)
        k_layout.setSpacing(14)

        # Left: Odometry Pose Card
        self.pose_card = RobotPoseWidget(self)
        k_layout.addWidget(self.pose_card, stretch=1)

        # Right: IMU Attitude Card
        self.imu_card = QFrame(tab_kinematics)
        self.imu_card.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_MD}px;
            }}
        """)
        imu_layout = QVBoxLayout(self.imu_card)
        imu_layout.setContentsMargins(14, 14, 14, 14)
        imu_layout.setSpacing(12)

        imu_hdr = SectionHeader("IMU 6-DOF Attitude & Accelerometer (/imu/data)", parent=self)
        imu_layout.addWidget(imu_hdr)

        self.lbl_rpy = QLabel("Attitude (RPY): Roll=+0.0°, Pitch=+0.0°, Yaw=+0.0°", self)
        self.lbl_accel = QLabel("Linear Accel (m/s²): ax=+0.00, ay=+0.00, az=+9.81", self)
        self.lbl_gyro = QLabel("Angular Rate (rad/s): gx=+0.000, gy=+0.000, gz=+0.000", self)
        self.lbl_imu_health = QLabel("Frame: imu_link  |  Expected: ~100 Hz  |  Status: WAITING", self)

        for lbl in (self.lbl_rpy, self.lbl_accel, self.lbl_gyro):
            lbl.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 13px; font-weight: 600; color: {COLOR_TEXT_PRIMARY};")
            imu_layout.addWidget(lbl)

        self.lbl_imu_health.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_MUTED};")
        imu_layout.addWidget(self.lbl_imu_health)
        imu_layout.addStretch()

        k_layout.addWidget(self.imu_card, stretch=1)
        self.tabs.addTab(tab_kinematics, "Kinematics && IMU Telemetry")

        # === TAB 3: Planar LiDAR Radar Profiler ===
        tab_lidar_profile = QWidget()
        lp_layout = QHBoxLayout(tab_lidar_profile)
        lp_layout.setContentsMargins(12, 12, 12, 12)
        lp_layout.setSpacing(14)

        # LiDAR Info Sidebar
        lidar_info_card = QFrame(tab_lidar_profile)
        lidar_info_card.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_MD}px;
            }}
        """)
        li_layout = QVBoxLayout(lidar_info_card)
        li_layout.setContentsMargins(14, 12, 14, 12)
        li_layout.setSpacing(10)

        li_hdr = SectionHeader("Planar LiDAR Sensor Engineering Envelopes", parent=self)
        li_layout.addWidget(li_hdr)

        self.lbl_lidar_specs = QLabel(
            "Coordinate Frame: lidar_link\n"
            "Sensor Height: Z = +1.175 m\n"
            "Field of View: 360° (2π rad)\n"
            "Angular Samples: 720 rays (~0.5° res)\n"
            "Scan Rate: 10.0 Hz (100 ms)\n"
            "Safety Envelope (E-Stop): 0.35 m\n"
            "Proximity Warning Zone: 0.80 m\n"
            "Nav2 Obstacle Inflation: 0.55 m\n"
            "Reliability QoS: Best Effort\n"
            "Maximum Range: 12.0 m"
        )
        self.lbl_lidar_specs.setStyleSheet(f"""
            font-family: {FONT_FAMILY_MONO};
            font-size: 11px;
            color: {COLOR_TEXT_PRIMARY};
            line-height: 1.6;
        """)
        li_layout.addWidget(self.lbl_lidar_specs)
        li_layout.addStretch()
        lp_layout.addWidget(lidar_info_card, stretch=1)

        self.tabs.addTab(tab_lidar_profile, "LiDAR Specifications && Envelopes")

        layout.addWidget(self.tabs, stretch=1)

        # Watchdog & Rate Calculation Timer (1 Hz)
        self.calc_timer = QTimer(self)
        self.calc_timer.timeout.connect(self._update_telemetry_matrix)
        self.calc_timer.start(1000)

        # Connect signals
        bus.rgb_frame_ready.connect(self._on_rgb_frame)
        bus.depth_frame_ready.connect(self._on_depth_frame)
        bus.lidar_scan_ready.connect(self._on_lidar_scan)
        bus.imu_data_ready.connect(self._on_imu_data)
        bus.odom_data_ready.connect(self._on_odom_data)
        bus.amcl_pose_ready.connect(self.pose_card.update_amcl_pose)

    def _on_rgb_frame(self, qimage, fps, resolution, timestamp):
        self.sensor_timestamps["rgb"] = time.time()
        self.sensor_counts["rgb"] += 1
        self.rgb_view.update_frame(qimage, fps, resolution, timestamp)

    def _on_depth_frame(self, qimage, min_d, max_d, timestamp):
        self.sensor_timestamps["depth"] = time.time()
        self.sensor_counts["depth"] += 1
        self.depth_view.update_depth(qimage, min_d, max_d, timestamp)

    def _on_lidar_scan(self, scan_data: dict):
        self.sensor_timestamps["lidar"] = time.time()
        self.sensor_counts["lidar"] += 1
        points = scan_data.get("points", [])
        min_r = scan_data.get("min_range", 0.0)
        max_r = scan_data.get("max_range", 0.0)
        freq = scan_data.get("freq", 10.0)
        ts = scan_data.get("timestamp", 0.0)
        self.lidar_view.update_scan(points, min_r, max_r, freq, ts)

    def _on_imu_data(self, imu: dict):
        self.sensor_timestamps["imu"] = time.time()
        self.sensor_counts["imu"] += 1

        r_deg = math.degrees(imu.get("roll", 0.0))
        p_deg = math.degrees(imu.get("pitch", 0.0))
        y_deg = math.degrees(imu.get("yaw", 0.0))
        self.lbl_rpy.setText(f"Attitude (RPY): Roll={r_deg:+.1f}°, Pitch={p_deg:+.1f}°, Yaw={y_deg:+.1f}°")

        ax = imu.get("ax", 0.0)
        ay = imu.get("ay", 0.0)
        az = imu.get("az", 9.81)
        self.lbl_accel.setText(f"Linear Accel (m/s²): ax={ax:+.2f}, ay={ay:+.2f}, az={az:+.2f}")

        gx = imu.get("gx", 0.0)
        gy = imu.get("gy", 0.0)
        gz = imu.get("gz", 0.0)
        self.lbl_gyro.setText(f"Angular Rate (rad/s): gx={gx:+.3f}, gy={gy:+.3f}, gz={gz:+.3f}")
        self.lbl_imu_health.setText("Frame: imu_link  |  Rate: ~100 Hz  |  Status: ACTIVE")

    def _on_odom_data(self, odom: dict):
        self.sensor_timestamps["odom"] = time.time()
        self.sensor_counts["odom"] += 1
        self.pose_card.update_odometry(
            odom.get("vx", 0.0), odom.get("vy", 0.0), odom.get("wz", 0.0)
        )

    def _update_telemetry_matrix(self):
        """Update sensor rate and data freshness matrix every second."""
        now = time.time()
        dt = now - self._last_calc_time
        if dt <= 0:
            return

        sensor_keys = ["rgb", "depth", "lidar", "imu", "odom"]
        for row, key in enumerate(sensor_keys):
            cnt = self.sensor_counts[key]
            rate = cnt / dt
            self.sensor_rates[key] = rate
            self.sensor_counts[key] = 0

            ts = self.sensor_timestamps[key]
            if ts == 0.0:
                status_text = "UNAVAILABLE"
                status_color = COLOR_OFFLINE
                age_text = "No Data"
                rate_text = "-- Hz"
            else:
                age = now - ts
                if age < 2.0:
                    status_text = "ACTIVE"
                    status_color = COLOR_SUCCESS
                    age_text = f"{age:.1f}s ago"
                    rate_text = f"{rate:.1f} Hz"
                elif age < 5.0:
                    status_text = "STALE"
                    status_color = COLOR_WARNING
                    age_text = f"{age:.1f}s ago"
                    rate_text = f"{rate:.1f} Hz"
                else:
                    status_text = "DISCONNECTED"
                    status_color = COLOR_ERROR
                    age_text = f"{age:.0f}s ago"
                    rate_text = "0.0 Hz"

            item_status = self.health_table.item(row, 3)
            if item_status:
                item_status.setText(status_text)
                item_status.setForeground(QColor(status_color))

            item_rate = self.health_table.item(row, 4)
            if item_rate:
                item_rate.setText(rate_text)

            item_age = self.health_table.item(row, 5)
            if item_age:
                item_age.setText(age_text)

        self._last_calc_time = now
