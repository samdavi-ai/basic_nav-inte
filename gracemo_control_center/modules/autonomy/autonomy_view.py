"""
GRaCEmo ViRa — Autonomous Operations Command & Mission Management Center
Phase 6: Dedicated Autonomous Operations Page in UARCC.
Coordinates real-time subsystem readiness, 11-state mission FSM, sequential
multi-destination waypoint dispatch, live 2D map visualization, safety arbitration,
and telemetry event logs.
"""

from __future__ import annotations
import json
import time
from typing import Dict, Any, Optional, List

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QFrame, QPushButton, QComboBox, QTextEdit, QScrollArea,
    QProgressBar, QGroupBox, QSplitter
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont

from ...resources.styles.tokens import (
    COLOR_BG_MAIN,
    COLOR_SURFACE,
    COLOR_PANEL_MAIN,
    COLOR_PANEL_SECONDARY,
    COLOR_SUBTLE_SURFACE,
    COLOR_BORDER,
    COLOR_BORDER_LIGHT,
    COLOR_BRAND_ORANGE,
    COLOR_ACCENT_PRIMARY,
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
from ...widgets.map_view import MapViewWidget
from ...widgets.status_indicator import StatusIndicator
from ...widgets.common_components import PageHeader, SectionHeader, MetricDisplay
from ...config.settings import Settings
from ...core.event_bus import bus


class AutonomyView(QWidget):
    """
    Dedicated Autonomous Operations Control Center for GRaCEmo ViRa.
    Provides complete supervisory control over the autonomous mobile robot.
    """

    def __init__(self, settings: Settings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.destinations = settings.destinations

        self.current_fsm_state = "SYSTEM_INITIALIZING"
        self.active_mission_name = "None"
        self.current_destination_name = "None"
        self.remaining_distance = 0.0
        self.dwell_remaining = 0.0
        self.elapsed_time = 0.0
        self.is_estop_active = False

        # Main Layout
        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(18, 16, 18, 16)
        root_layout.setSpacing(12)

        # 1. Header
        self.header = PageHeader(
            title="Autonomous Operations",
            subtitle="Central supervisory FSM, multi-waypoint patrol manager, and safety arbitration gate",
            badge_title="Autonomy FSM",
            initial_state=StatusIndicator.STATE_INITIALIZING,
            target_page="Autonomous Operations",
            parent=self
        )
        self.badge_fsm = self.header.badge
        root_layout.addWidget(self.header)

        # 2. Main Content Splitter (Left: Controls & Telemetry, Right: Live Map)
        main_splitter = QSplitter(Qt.Horizontal, self)
        main_splitter.setHandleWidth(8)

        # === LEFT COLUMN: System Status, Robot Telemetry, Mission Control, Safety ===
        left_scroll = QScrollArea(self)
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 8, 0)
        left_layout.setSpacing(12)

        # A. SYSTEM STATUS MATRIX
        sys_status_panel = self._create_system_status_panel()
        left_layout.addWidget(sys_status_panel)

        # B. ROBOT STATUS & TELEMETRY
        robot_status_panel = self._create_robot_status_panel()
        left_layout.addWidget(robot_status_panel)

        # C. MISSION CONTROL
        mission_control_panel = self._create_mission_control_panel()
        left_layout.addWidget(mission_control_panel)

        # E. SAFETY PANEL
        safety_panel = self._create_safety_panel()
        left_layout.addWidget(safety_panel)

        left_scroll.setWidget(left_widget)
        main_splitter.addWidget(left_scroll)

        # === RIGHT COLUMN: Live 2D Map & Event Log ===
        right_container = QWidget(self)
        right_layout = QVBoxLayout(right_container)
        right_layout.setContentsMargins(8, 0, 0, 0)
        right_layout.setSpacing(12)

        # D. LIVE MAP
        map_container = QFrame(self)
        map_container.setProperty("class", "cardPanel")
        mc_layout = QVBoxLayout(map_container)
        mc_layout.setContentsMargins(10, 10, 10, 10)
        mc_layout.setSpacing(8)

        map_hdr = SectionHeader("Live Occupancy Grid & Autonomous Route Tracking", parent=self)
        mc_layout.addWidget(map_hdr)

        self.map_view = MapViewWidget(self)
        self.map_view.set_destinations(self.destinations)
        mc_layout.addWidget(self.map_view, stretch=1)
        right_layout.addWidget(map_container, stretch=3)

        # F. REAL-TIME EVENT LOG
        event_log_panel = self._create_event_log_panel()
        right_layout.addWidget(event_log_panel, stretch=1)

        main_splitter.addWidget(right_container)
        main_splitter.setStretchFactor(0, 1)
        main_splitter.setStretchFactor(1, 1)
        main_splitter.setSizes([620, 680])

        root_layout.addWidget(main_splitter)

        # Wire EventBus signals
        bus.autonomy_state_ready.connect(self._on_autonomy_state_ready)
        bus.autonomy_mission_status_ready.connect(self._on_autonomy_mission_status)
        bus.coordinator_feedback_ready.connect(self._on_coordinator_feedback)
        bus.map_ready.connect(self._on_map_ready)
        bus.amcl_pose_ready.connect(self._on_amcl_pose)
        bus.global_path_ready.connect(self.map_view.set_global_path)
        bus.lidar_scan_ready.connect(self._on_lidar_scan)
        bus.emergency_stop_state.connect(self._on_estop_state)

        # Local refresh timer for clock/elapsed calculation
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._update_ui_tick)
        self.refresh_timer.start(250)

    # -------------------------------------------------------------------------
    # Panel Builders
    # -------------------------------------------------------------------------
    def _create_system_status_panel(self) -> QFrame:
        panel = QFrame(self)
        panel.setProperty("class", "cardPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)

        layout.addWidget(SectionHeader("A. Subsystem Readiness Matrix", parent=self))

        grid = QGridLayout()
        grid.setSpacing(6)

        self.ind_kernel = StatusIndicator("Kernel (Port 7780)", StatusIndicator.STATE_READY, target_page="Cognitive System")
        self.ind_ros2 = StatusIndicator("ROS 2 Jazzy", StatusIndicator.STATE_READY, target_page="Diagnostics")
        self.ind_gazebo = StatusIndicator("Gazebo Harmonic", StatusIndicator.STATE_READY, target_page="Simulation")
        self.ind_nav2 = StatusIndicator("Nav2 Lifecycle", StatusIndicator.STATE_READY, target_page="Navigation")
        self.ind_amcl = StatusIndicator("Localization (AMCL)", StatusIndicator.STATE_NOT_CONNECTED, target_page="Localization")
        self.ind_map = StatusIndicator("SLAM / Map Server", StatusIndicator.STATE_READY, target_page="Navigation")
        self.ind_sensors = StatusIndicator("Sensor Freshness", StatusIndicator.STATE_READY, target_page="Sensors")
        self.ind_terrain = StatusIndicator("Terrain Gate", StatusIndicator.STATE_READY, target_page="Safety")
        self.ind_bridge = StatusIndicator("Interaction Bridge", StatusIndicator.STATE_READY, target_page="Voice & Interaction")

        items_2col = [
            (self.ind_kernel, self.ind_ros2),
            (self.ind_gazebo, self.ind_nav2),
            (self.ind_amcl, self.ind_map),
            (self.ind_sensors, self.ind_terrain),
            (self.ind_bridge, None)
        ]
        for r, (item_a, item_b) in enumerate(items_2col):
            grid.addWidget(item_a, r, 0)
            if item_b:
                grid.addWidget(item_b, r, 1)

        layout.addLayout(grid)
        return panel

    def _create_robot_status_panel(self) -> QFrame:
        panel = QFrame(self)
        panel.setProperty("class", "cardPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)

        layout.addWidget(SectionHeader("B. Robot Telemetry & Pose", parent=self))

        # Metrics grid
        grid = QGridLayout()
        grid.setSpacing(6)

        self.met_pose_x = MetricDisplay("Map X", "0.00 m", parent=self)
        self.met_pose_y = MetricDisplay("Map Y", "0.00 m", parent=self)
        self.met_pose_yaw = MetricDisplay("Orientation", "0.00 rad", parent=self)
        self.met_lin_vel = MetricDisplay("Linear Vel", "0.00 m/s", parent=self)
        self.met_ang_vel = MetricDisplay("Angular Vel", "0.00 rad/s", parent=self)
        self.met_battery = MetricDisplay("Battery (Sim)", "98.5 %", parent=self)

        grid.addWidget(self.met_pose_x, 0, 0)
        grid.addWidget(self.met_pose_y, 0, 1)
        grid.addWidget(self.met_pose_yaw, 0, 2)
        grid.addWidget(self.met_lin_vel, 1, 0)
        grid.addWidget(self.met_ang_vel, 1, 1)
        grid.addWidget(self.met_battery, 1, 2)

        layout.addLayout(grid)

        # Mission Progress Bar
        self.lbl_progress_info = QLabel("Mission Progress: IDLE (No active mission)")
        self.lbl_progress_info.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_SECONDARY};")
        layout.addWidget(self.lbl_progress_info)

        self.progress_bar = QProgressBar(self)
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setFixedHeight(8)
        self.progress_bar.setTextVisible(False)
        self.progress_bar.setStyleSheet(f"""
            QProgressBar {{
                background-color: {COLOR_SUBTLE_SURFACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: 4px;
            }}
            QProgressBar::chunk {{
                background-color: {COLOR_TEXT_PRIMARY};
                border-radius: 3px;
            }}
        """)
        layout.addWidget(self.progress_bar)

        return panel

    def _create_mission_control_panel(self) -> QFrame:
        panel = QFrame(self)
        panel.setProperty("class", "cardPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)

        layout.addWidget(SectionHeader("C. Autonomous Mission Management", parent=self))

        # Single Destination Selector
        dest_row = QHBoxLayout()
        lbl_dest = QLabel("Destination:")
        lbl_dest.setStyleSheet(f"font-size: 11px; font-weight: 600; color: {COLOR_TEXT_SECONDARY};")
        dest_row.addWidget(lbl_dest)

        self.dest_combo = QComboBox(self)
        for key, info in self.destinations.items():
            name = info.get("name", key)
            coords = f"({info.get('x', 0.0):.1f}, {info.get('y', 0.0):.1f})"
            self.dest_combo.addItem(f"{name} {coords}", userData=key)
        dest_row.addWidget(self.dest_combo, stretch=1)
        layout.addLayout(dest_row)

        # Primary Mission Action Buttons Grid
        btn_grid = QGridLayout()
        btn_grid.setSpacing(6)

        self.btn_start = QPushButton("▶ Start Single Mission", self)
        self.btn_start.setProperty("class", "primaryButton")
        self.btn_start.clicked.connect(self._start_single_mission)
        btn_grid.addWidget(self.btn_start, 0, 0)

        self.btn_patrol = QPushButton("🔄 Patrol Waypoint Sequence", self)
        self.btn_patrol.setProperty("class", "secondaryButton")
        self.btn_patrol.clicked.connect(self._start_patrol_mission)
        btn_grid.addWidget(self.btn_patrol, 0, 1)

        self.btn_pause = QPushButton("⏸ Pause", self)
        self.btn_pause.setProperty("class", "secondaryButton")
        self.btn_pause.clicked.connect(self._pause_mission)
        btn_grid.addWidget(self.btn_pause, 1, 0)

        self.btn_resume = QPushButton("▶ Continue", self)
        self.btn_resume.setProperty("class", "secondaryButton")
        self.btn_resume.clicked.connect(self._resume_mission)
        btn_grid.addWidget(self.btn_resume, 1, 1)

        self.btn_cancel = QPushButton("⏹ Cancel Mission", self)
        self.btn_cancel.setProperty("class", "secondaryButton")
        self.btn_cancel.clicked.connect(self._cancel_mission)
        btn_grid.addWidget(self.btn_cancel, 2, 0)

        self.btn_home = QPushButton("🏠 Return to Safe Base", self)
        self.btn_home.setProperty("class", "secondaryButton")
        self.btn_home.clicked.connect(self._return_home)
        btn_grid.addWidget(self.btn_home, 2, 1)

        layout.addLayout(btn_grid)
        return panel

    def _create_safety_panel(self) -> QFrame:
        panel = QFrame(self)
        panel.setProperty("class", "cardPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(8)

        layout.addWidget(SectionHeader("E. Safety Envelope & Simulation Boundary", parent=self))

        # Status Indicators
        st_row = QHBoxLayout()
        self.ind_collision = StatusIndicator("Collision Monitor", StatusIndicator.STATE_READY, target_page="Safety")
        self.ind_invariants = StatusIndicator("Safety Invariants Locked", StatusIndicator.STATE_READY, target_page="Safety")
        st_row.addWidget(self.ind_collision)
        st_row.addWidget(self.ind_invariants)
        st_row.addStretch()
        layout.addLayout(st_row)

        lbl_sim_lock = QLabel("🔒 Hardware Invariants strictly enforced: MOTOR_OUTPUT_ENABLED=0 | MCU_CONFIRMED=False")
        lbl_sim_lock.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 10px; color: {COLOR_TEXT_MUTED};")
        layout.addWidget(lbl_sim_lock)

        # Big E-Stop Button
        self.btn_estop = QPushButton("🚨 EMERGENCY STOP", self)
        self.btn_estop.setProperty("class", "dangerButton")
        self.btn_estop.setFixedHeight(36)
        self.btn_estop.clicked.connect(self._toggle_emergency_stop)
        layout.addWidget(self.btn_estop)

        return panel

    def _create_event_log_panel(self) -> QFrame:
        panel = QFrame(self)
        panel.setProperty("class", "cardPanel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        layout.addWidget(SectionHeader("F. Mission State & Transition Event Log", parent=self))

        self.txt_event_log = QTextEdit(self)
        self.txt_event_log.setReadOnly(True)
        self.txt_event_log.setStyleSheet(f"""
            QTextEdit {{
                background-color: {COLOR_PANEL_SECONDARY};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                color: {COLOR_TEXT_PRIMARY};
                font-family: {FONT_FAMILY_MONO};
                font-size: 11px;
                padding: 6px;
            }}
        """)
        layout.addWidget(self.txt_event_log)
        return panel

    # -------------------------------------------------------------------------
    # UI Action Handlers
    # -------------------------------------------------------------------------
    def _start_single_mission(self):
        d_key = self.dest_combo.currentData()
        if d_key in self.destinations:
            dest = self.destinations[d_key]
            payload = {
                "intent": "START_MISSION",
                "mission_name": f"Transit to {dest.get('name', d_key)}",
                "waypoints": [
                    {
                        "name": dest.get("name", d_key),
                        "coords": {
                            "x": dest.get("x", 0.0),
                            "y": dest.get("y", 0.0),
                            "yaw": dest.get("yaw", 0.0),
                            "frame_id": dest.get("frame_id", "map")
                        },
                        "dwell_sec": 0.0
                    }
                ]
            }
            bus.mission_command_dispatched.emit(payload)
            self._log_event(f"User dispatched single mission to {dest.get('name', d_key)}")

    def _start_patrol_mission(self):
        """Campus Delivery / Apartment Patrol Sequence."""
        payload = {
            "intent": "START_MISSION",
            "mission_name": "Campus Delivery Patrol",
            "waypoints": [
                {
                    "name": "Central Hallway",
                    "coords": {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame_id": "map"},
                    "dwell_sec": 3.0
                },
                {
                    "name": "Master Bedroom",
                    "coords": {"x": -5.0, "y": 2.2, "yaw": 1.57, "frame_id": "map"},
                    "dwell_sec": 5.0
                },
                {
                    "name": "Home Study",
                    "coords": {"x": 4.5, "y": 3.0, "yaw": 1.57, "frame_id": "map"},
                    "dwell_sec": 5.0
                },
                {
                    "name": "Central Hallway",
                    "coords": {"x": 0.0, "y": 0.0, "yaw": 0.0, "frame_id": "map"},
                    "dwell_sec": 0.0
                }
            ]
        }
        bus.mission_command_dispatched.emit(payload)
        self._log_event("User dispatched multi-waypoint patrol: Hallway -> Bedroom (5s) -> Study (5s) -> Hallway")

    def _pause_mission(self):
        payload = {"intent": "PAUSE_MISSION"}
        bus.mission_command_dispatched.emit(payload)
        self._log_event("User requested mission pause")

    def _resume_mission(self):
        payload = {"intent": "RESUME_MISSION"}
        bus.mission_command_dispatched.emit(payload)
        self._log_event("User requested mission resume")

    def _cancel_mission(self):
        payload = {"intent": "CANCEL_MISSION"}
        bus.mission_command_dispatched.emit(payload)
        self._log_event("User requested mission cancellation")

    def _return_home(self):
        payload = {"intent": "RETURN_HOME"}
        bus.mission_command_dispatched.emit(payload)
        self._log_event("User commanded Return to Safe Base")

    def _toggle_emergency_stop(self):
        self.is_estop_active = not self.is_estop_active
        bus.emergency_stop_state.emit(self.is_estop_active, "UARCC Autonomy Page")
        if self.is_estop_active:
            self.btn_estop.setText("RESET EMERGENCY STOP")
            self.btn_estop.setStyleSheet(f"background-color: {COLOR_WARNING}; color: #000; font-weight: bold;")
            self._log_event("🚨 OPERATOR ENGAGED EMERGENCY STOP")
        else:
            self.btn_estop.setText("🚨 EMERGENCY STOP")
            self.btn_estop.setStyleSheet("")
            self.btn_estop.setProperty("class", "dangerButton")
            self._log_event("Emergency Stop released. Returning to Autonomy Ready.")

    # -------------------------------------------------------------------------
    # Telemetry and EventBus Listeners
    # -------------------------------------------------------------------------
    def _on_autonomy_state_ready(self, state_str: str):
        self.current_fsm_state = state_str
        self._update_fsm_badge(state_str)
        self._log_event(f"FSM State ➔ {state_str}")

    def _on_autonomy_mission_status(self, data: dict):
        state = data.get("state", self.current_fsm_state)
        self._update_fsm_badge(state)

        # Robot Telemetry
        robot = data.get("robot", {})
        pose = robot.get("pose", {})
        twist = robot.get("twist", {})
        self.met_pose_x.set_value(f"{pose.get('x', 0.0):+.2f} m")
        self.met_pose_y.set_value(f"{pose.get('y', 0.0):+.2f} m")
        self.met_pose_yaw.set_value(f"{pose.get('yaw', 0.0):+.2f} rad")
        self.met_lin_vel.set_value(f"{twist.get('linear', 0.0):.2f} m/s")
        self.met_ang_vel.set_value(f"{twist.get('angular', 0.0):.2f} rad/s")
        self.met_battery.set_value(f"{robot.get('battery_pct', 98.5):.1f} %")

        # Update Map View Robot Pose
        self.map_view.set_robot_pose(pose.get("x", 0.0), pose.get("y", 0.0), pose.get("yaw", 0.0))

        # Subsystems
        sub = data.get("subsystems", {})
        self.ind_sensors.set_state(StatusIndicator.STATE_READY if sub.get("scan_fresh") and sub.get("odom_fresh") else StatusIndicator.STATE_DEGRADED)
        self.ind_nav2.set_state(StatusIndicator.STATE_READY if sub.get("nav2_ready") else StatusIndicator.STATE_NOT_CONNECTED)

        if robot.get("has_localization"):
            self.ind_amcl.set_state(StatusIndicator.STATE_READY)
        else:
            self.ind_amcl.set_state(StatusIndicator.STATE_INITIALIZING)

        # Mission Progress
        msn = data.get("mission", {})
        name = msn.get("name") or "None"
        wp_idx = msn.get("waypoint_index", 0)
        tot_wp = msn.get("total_waypoints", 0)
        active_wp = msn.get("active_waypoint") or "None"
        dist = msn.get("remaining_distance", 0.0)
        dwell = msn.get("dwell_remaining_sec", 0.0)

        if tot_wp > 0 and state in ("MISSION_EXECUTING", "MISSION_PLANNING", "DESTINATION_REACHED", "OBSTACLE_HANDLING"):
            pct = int(((wp_idx + 1) / max(1, tot_wp)) * 100)
            self.progress_bar.setValue(pct)
            if dwell > 0:
                self.lbl_progress_info.setText(f"Mission: {name} | Waypoint {wp_idx + 1}/{tot_wp} ({active_wp}) | Dwelling: {dwell:.1f}s remaining")
            else:
                self.lbl_progress_info.setText(f"Mission: {name} | Waypoint {wp_idx + 1}/{tot_wp} ({active_wp}) | Dist: {dist:.1f}m")
        else:
            self.progress_bar.setValue(0)
            self.lbl_progress_info.setText(f"Mission: {name} | Status: {state}")

    def _on_coordinator_feedback(self, state: str, speech: str, details: dict, ts: float):
        self._update_fsm_badge(state)
        if speech:
            self._log_event(f"🗣️ [{state}] {speech}")

    def _on_map_ready(self, map_payload: dict):
        self.map_view.set_map(
            map_payload["width"], map_payload["height"],
            map_payload["resolution"], map_payload["origin_x"], map_payload["origin_y"],
            map_payload["data"]
        )
        self.ind_map.set_state(StatusIndicator.STATE_READY)

    def _on_amcl_pose(self, x: float, y: float, yaw: float, vx: float, vy: float, vyaw: float):
        self.map_view.set_robot_pose(x, y, yaw)
        self.ind_amcl.set_state(StatusIndicator.STATE_READY)

    def _on_lidar_scan(self, scan_data: dict):
        if hasattr(self.map_view, "set_lidar_scan"):
            self.map_view.set_lidar_scan(scan_data)
        self.ind_sensors.set_state(StatusIndicator.STATE_READY)

    def _on_estop_state(self, active: bool, source: str):
        self.is_estop_active = active
        if active:
            self.ind_terrain.set_state(StatusIndicator.STATE_ERROR)
            self.btn_estop.setText("RESET EMERGENCY STOP")
            self.btn_estop.setStyleSheet(f"background-color: {COLOR_WARNING}; color: #000; font-weight: bold;")
        else:
            self.ind_terrain.set_state(StatusIndicator.STATE_READY)
            self.btn_estop.setText("🚨 EMERGENCY STOP")
            self.btn_estop.setStyleSheet("")
            self.btn_estop.setProperty("class", "dangerButton")

    def _update_fsm_badge(self, state_str: str):
        self.header.set_title("Autonomous Operations")
        if self.badge_fsm:
            if state_str in ("AUTONOMY_READY", "DESTINATION_REACHED", "MISSION_COMPLETED"):
                self.badge_fsm.set_state(StatusIndicator.STATE_READY, details=f"FSM State: {state_str}")
            elif state_str in ("MISSION_ACCEPTED", "MISSION_PLANNING", "MISSION_EXECUTING", "OBSTACLE_HANDLING"):
                self.badge_fsm.set_state(StatusIndicator.STATE_INITIALIZING, details=f"FSM State: {state_str}")
            elif state_str in ("SYSTEM_INITIALIZING", "SYSTEM_HEALTH_CHECK", "WAITING_FOR_LOCALIZATION", "MISSION_PAUSED"):
                self.badge_fsm.set_state(StatusIndicator.STATE_INITIALIZING, details=f"FSM State: {state_str}")
            elif state_str in ("EMERGENCY_STOP", "TERRAIN_SAFETY_STOP", "SENSOR_FAILURE", "NAVIGATION_FAILURE", "PLANNING_FAILURE"):
                self.badge_fsm.set_state(StatusIndicator.STATE_ERROR, details=f"FSM State: {state_str}")
            else:
                self.badge_fsm.set_state(StatusIndicator.STATE_NOT_CONNECTED, details=f"FSM State: {state_str}")
            self.badge_fsm.state_label.setText(state_str[:14])

    def _update_ui_tick(self):
        # Keeps timers or dwell countdowns refreshed if needed
        pass

    def _log_event(self, text: str):
        t_str = time.strftime("%H:%M:%S")
        self.txt_event_log.append(f"[{t_str}] {text}")
        sb = self.txt_event_log.verticalScrollBar()
        sb.setValue(sb.maximum())
