"""
Module 3: Autonomous Navigation Center for GRaCEmo ViRa UARCC.
Interactive 2D Occupancy Grid map, authoritative 6-destination selector,
coordinator-mediated action dispatch, cancellation, arrival verification,
and live navigation prerequisites checklist.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QFrame, QPushButton, QComboBox, QTextEdit, QMessageBox,
    QStackedWidget, QButtonGroup
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont

import json
import time
from datetime import datetime
from typing import Dict, Any, Optional

from ...resources.styles.tokens import (
    COLOR_BG_MAIN,
    COLOR_WORKSPACE,
    COLOR_SURFACE,
    COLOR_SURFACE_ALT,
    COLOR_PANEL_MAIN,
    COLOR_PANEL_SECONDARY,
    COLOR_SUBTLE_SURFACE,
    COLOR_BORDER,
    COLOR_BORDER_LIGHT,
    COLOR_SUCCESS,
    COLOR_WARNING,
    COLOR_ERROR,
    COLOR_INFO,
    COLOR_OFFLINE,
    COLOR_TEXT_PRIMARY,
    COLOR_TEXT_SECONDARY,
    COLOR_TEXT_MUTED,
    FONT_FAMILY_PRIMARY,
    FONT_FAMILY_MONO,
    BORDER_RADIUS_SM,
    BORDER_RADIUS_MD,
)
from ...widgets.map_view import MapViewWidget
from ...widgets.embedded_rviz_widget import EmbeddedRVizWidget
from ...widgets.status_indicator import StatusIndicator
from ...widgets.common_components import (
    PageHeader, SectionHeader, MetricDisplay, AlertBanner, WarningBanner
)
from ...config.settings import Settings
from ...core.event_bus import bus


class NavigationView(QWidget):
    """Module 3: Autonomous Navigation Command & Tracking Center."""

    def __init__(self, settings: Settings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.destinations = settings.destinations

        self.active_destination_key: Optional[str] = None
        self.navigation_start_time: Optional[float] = None
        self.coordinator_state = "IDLE"
        self.has_map = False
        self.has_localization = False
        self.coordinator_ready = False
        self.is_estopped = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(16)

        # 1. Page Header
        self.header = PageHeader(
            title="Autonomous Navigation",
            subtitle="Embedded ROS 2 RViz2 3D workspace, waypoint routing, Nav2 tracking, and safe-stop action",
            badge_title="Nav2 State",
            initial_state=StatusIndicator.STATE_NOT_CONNECTED,
            target_page="Navigation",
            parent=self
        )
        self.ind_nav_state = self.header.badge
        layout.addWidget(self.header)

        # 2. Main Body Splitter/Layout (Workspace on Left, Controls on Right)
        body_layout = QHBoxLayout()
        body_layout.setSpacing(16)

        # === LEFT: Primary Visualization Workspace (65% width) ===
        map_container = QFrame(self)
        map_container.setProperty("class", "cardPanel")
        mc_layout = QVBoxLayout(map_container)
        mc_layout.setContentsMargins(12, 12, 12, 12)
        mc_layout.setSpacing(8)

        viz_hdr_layout = QHBoxLayout()
        map_hdr = SectionHeader("Primary Robotics Visualization Workspace", parent=self)
        viz_hdr_layout.addWidget(map_hdr)
        viz_hdr_layout.addStretch()

        self.btn_view_group = QButtonGroup(self)
        self.btn_view_group.setExclusive(True)

        self.btn_mode_rviz = QPushButton("3D RViz2 Workspace", self)
        self.btn_mode_rviz.setCheckable(True)
        self.btn_mode_rviz.setChecked(True)
        self.btn_mode_rviz.setStyleSheet(f"""
            QPushButton {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 4px 10px;
                font-size: 11px;
                font-weight: 600;
            }}
            QPushButton:checked {{
                background-color: #1677C8;
                color: #FFFFFF;
                border-color: #1677C8;
            }}
        """)
        self.btn_mode_rviz.clicked.connect(lambda: self.viz_stack.setCurrentIndex(0))
        self.btn_view_group.addButton(self.btn_mode_rviz)
        viz_hdr_layout.addWidget(self.btn_mode_rviz)

        self.btn_mode_map = QPushButton("2D Occupancy Grid", self)
        self.btn_mode_map.setCheckable(True)
        self.btn_mode_map.setStyleSheet(f"""
            QPushButton {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 4px 10px;
                font-size: 11px;
                font-weight: 600;
            }}
            QPushButton:checked {{
                background-color: #1677C8;
                color: #FFFFFF;
                border-color: #1677C8;
            }}
        """)
        self.btn_mode_map.clicked.connect(lambda: self.viz_stack.setCurrentIndex(1))
        self.btn_view_group.addButton(self.btn_mode_map)
        viz_hdr_layout.addWidget(self.btn_mode_map)

        mc_layout.addLayout(viz_hdr_layout)

        # Central Visualization Stack
        self.viz_stack = QStackedWidget(map_container)
        self.embedded_rviz = EmbeddedRVizWidget(self)
        self.viz_stack.addWidget(self.embedded_rviz)

        self.map_view = MapViewWidget(self)
        self.map_view.set_destinations(self.destinations)
        self.viz_stack.addWidget(self.map_view)

        mc_layout.addWidget(self.viz_stack, stretch=1)

        body_layout.addWidget(map_container, stretch=3)

        # === RIGHT: Command & Telemetry Column (35% width) ===
        control_panel = QFrame(self)
        control_panel.setProperty("class", "cardPanel")
        cp_layout = QVBoxLayout(control_panel)
        cp_layout.setContentsMargins(14, 14, 14, 14)
        cp_layout.setSpacing(12)

        # Sub-header
        cp_hdr = SectionHeader("Waypoint Dispatch & Control", parent=self)
        cp_layout.addWidget(cp_hdr)

        # 1. Live Navigation Prerequisites Checklist Card
        prereq_box = QFrame(self)
        prereq_box.setObjectName("prereqBox")
        prereq_box.setStyleSheet(f"""
            QFrame#prereqBox {{
                background-color: {COLOR_SUBTLE_SURFACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 10px 12px;
            }}
        """)
        prereq_layout = QVBoxLayout(prereq_box)
        prereq_layout.setContentsMargins(0, 0, 0, 0)
        prereq_layout.setSpacing(6)

        prereq_title = QLabel("NAVIGATION PREREQUISITES", prereq_box)
        prereq_title.setStyleSheet(f"font-size: 10px; font-weight: 700; color: {COLOR_TEXT_SECONDARY}; letter-spacing: 0.5px;")
        prereq_layout.addWidget(prereq_title)

        checklist_grid = QGridLayout()
        checklist_grid.setHorizontalSpacing(12)
        checklist_grid.setVerticalSpacing(4)

        self.chk_map = QLabel("○ Map: Awaiting /map", self)
        self.chk_loc = QLabel("○ Localization: Awaiting /amcl_pose", self)
        self.chk_coord = QLabel("○ Coordinator: Standby", self)
        self.chk_estop = QLabel("✓ E-Stop: Disengaged (Normal)", self)

        for lbl in (self.chk_map, self.chk_loc, self.chk_coord, self.chk_estop):
            lbl.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_MUTED}; background: transparent; border: none;")

        self.chk_estop.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_SUCCESS}; background: transparent; border: none;")

        checklist_grid.addWidget(self.chk_map, 0, 0)
        checklist_grid.addWidget(self.chk_loc, 0, 1)
        checklist_grid.addWidget(self.chk_coord, 1, 0)
        checklist_grid.addWidget(self.chk_estop, 1, 1)

        prereq_layout.addLayout(checklist_grid)
        cp_layout.addWidget(prereq_box)

        # Destination Selection Card
        dest_box = QFrame(self)
        dest_box.setObjectName("destBox")
        dest_box.setStyleSheet(f"""
            QFrame#destBox {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 10px 12px;
            }}
        """)
        db_layout = QVBoxLayout(dest_box)
        db_layout.setContentsMargins(0, 0, 0, 0)
        db_layout.setSpacing(8)

        lbl_select = QLabel("Target Apartment Destination:", self)
        lbl_select.setStyleSheet(f"font-size: 11px; font-weight: 600; color: {COLOR_TEXT_PRIMARY}; text-transform: uppercase; letter-spacing: 0.4px; background: transparent; border: none;")
        db_layout.addWidget(lbl_select)

        self.dest_combo = QComboBox(self)
        self.dest_combo.setFixedHeight(34)
        for d_key, d_info in self.destinations.items():
            name = d_info.get("name", d_key)
            coords = f"({d_info.get('x', 0.0):.1f}, {d_info.get('y', 0.0):.1f})"
            self.dest_combo.addItem(f"{name} {coords}", userData=d_key)
        self.dest_combo.currentIndexChanged.connect(self._on_dest_selected)
        db_layout.addWidget(self.dest_combo)

        # Display Selected Coordinates
        self.lbl_selected_coords = QLabel("Target: X=0.00 m, Y=0.00 m, Yaw=0.00 rad", self)
        self.lbl_selected_coords.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_SECONDARY}; background: transparent; border: none;")
        db_layout.addWidget(self.lbl_selected_coords)

        cp_layout.addWidget(dest_box)

        # Dispatch Status & Prerequisites Explanation Banner
        self.lbl_dispatch_banner = QLabel("Awaiting AMCL map localization on /amcl_pose before dispatching.", self)
        self.lbl_dispatch_banner.setObjectName("dispatchBanner")
        self.lbl_dispatch_banner.setWordWrap(True)
        self.lbl_dispatch_banner.setStyleSheet(f"""
            QLabel#dispatchBanner {{
                background-color: #FEF3C7;
                border: 1px solid #F59E0B;
                border-radius: {BORDER_RADIUS_SM}px;
                color: #92400E;
                font-size: 11px;
                font-weight: 600;
                padding: 6px 10px;
            }}
        """)
        cp_layout.addWidget(self.lbl_dispatch_banner)

        # Command Action Buttons
        btn_grid = QGridLayout()
        btn_grid.setSpacing(8)

        self.btn_dispatch = QPushButton("▶ Dispatch Goal", self)
        self.btn_dispatch.setProperty("class", "primaryButton")
        self.btn_dispatch.setFixedHeight(36)
        self.btn_dispatch.clicked.connect(self._dispatch_navigation_goal)
        btn_grid.addWidget(self.btn_dispatch, 0, 0)

        self.btn_cancel = QPushButton("⏹ Cancel Goal", self)
        self.btn_cancel.setProperty("class", "secondaryButton")
        self.btn_cancel.setFixedHeight(36)
        self.btn_cancel.clicked.connect(self._cancel_navigation_goal)
        btn_grid.addWidget(self.btn_cancel, 0, 1)

        self.btn_estop = QPushButton("🚨 EMERGENCY STOP", self)
        self.btn_estop.setProperty("class", "dangerButton")
        self.btn_estop.setFixedHeight(38)
        self.btn_estop.clicked.connect(self._trigger_emergency_stop)
        btn_grid.addWidget(self.btn_estop, 1, 0, 1, 2)

        cp_layout.addLayout(btn_grid)

        # Active Mission Telemetry
        telem_box = QFrame(self)
        telem_box.setObjectName("telemBox")
        telem_box.setStyleSheet(f"""
            QFrame#telemBox {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 10px 12px;
            }}
        """)
        tb_layout = QVBoxLayout(telem_box)
        tb_layout.setContentsMargins(0, 0, 0, 0)
        tb_layout.setSpacing(6)

        tb_hdr = SectionHeader("Active Mission Progress", parent=self)
        tb_layout.addWidget(tb_hdr)

        self.lbl_active_dest = QLabel("Destination: None", self)
        self.lbl_dist_remaining = QLabel("Distance Remaining: 0.00 m", self)
        self.lbl_elapsed_time = QLabel("Elapsed Time: 0.0 s", self)
        self.lbl_arrival_status = QLabel("Status: IDLE (STANDBY)", self)

        for lbl in (self.lbl_active_dest, self.lbl_dist_remaining, self.lbl_elapsed_time):
            lbl.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_SECONDARY}; background: transparent; border: none;")
            tb_layout.addWidget(lbl)

        self.lbl_arrival_status.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; font-weight: 700; color: {COLOR_OFFLINE}; background: transparent; border: none;")
        tb_layout.addWidget(self.lbl_arrival_status)

        cp_layout.addWidget(telem_box)

        # Coordinator Verification & Feedback Log
        fb_hdr = SectionHeader("Coordinator Feedback Stream", parent=self)
        cp_layout.addWidget(fb_hdr)

        self.txt_feedback = QTextEdit(self)
        self.txt_feedback.setReadOnly(True)
        self.txt_feedback.setMaximumHeight(140)
        self.txt_feedback.setStyleSheet(f"""
            QTextEdit {{
                background-color: {COLOR_SUBTLE_SURFACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                color: {COLOR_TEXT_PRIMARY};
                font-family: {FONT_FAMILY_MONO};
                font-size: 11px;
                padding: 8px;
            }}
        """)
        cp_layout.addWidget(self.txt_feedback)

        body_layout.addWidget(control_panel, stretch=2)
        layout.addLayout(body_layout)

        # Elapsed time timer
        self.elapsed_timer = QTimer(self)
        self.elapsed_timer.timeout.connect(self._update_elapsed_time)

        # Connect bus signals
        bus.map_ready.connect(self._on_map_ready)
        bus.amcl_pose_ready.connect(self._on_amcl_pose)
        bus.global_path_ready.connect(self.map_view.set_global_path)
        bus.coordinator_feedback_ready.connect(self._on_coordinator_feedback)
        bus.emergency_stop_state.connect(self._on_emergency_stop_state)

        self._on_dest_selected(0)
        self._update_prerequisites_ui()

    def _update_prerequisites_ui(self):
        """Update live checklist, dispatch banner explanation, and button state."""
        # Map item
        if self.has_map:
            self.chk_map.setText("✓ Map: Loaded (2D Occupancy)")
            self.chk_map.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_SUCCESS};")
        else:
            self.chk_map.setText("○ Map: Awaiting /map")
            self.chk_map.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_MUTED};")

        # Localization item
        if self.has_localization:
            self.chk_loc.setText("✓ AMCL: Localized (/amcl_pose)")
            self.chk_loc.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_SUCCESS};")
        else:
            self.chk_loc.setText("○ AMCL: Awaiting /amcl_pose")
            self.chk_loc.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_MUTED};")

        # Coordinator item
        if self.coordinator_ready:
            self.chk_coord.setText(f"✓ Coordinator: Ready ({self.coordinator_state})")
            self.chk_coord.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_SUCCESS};")
        else:
            self.chk_coord.setText("○ Coordinator: Standby")
            self.chk_coord.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_MUTED};")

        # E-Stop item
        if self.is_estopped:
            self.chk_estop.setText("🚨 E-Stop: ASSERTED (Blocked)")
            self.chk_estop.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; font-weight: 700; color: {COLOR_ERROR};")
        else:
            self.chk_estop.setText("✓ E-Stop: Disengaged (Normal)")
            self.chk_estop.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_SUCCESS};")

        # Banner & Dispatch readiness determination
        reasons = []
        if self.is_estopped:
            reasons.append("Emergency Stop is currently active. Clear E-Stop before dispatching.")
        if not self.has_map:
            reasons.append("Occupancy grid map has not been received on /map.")
        if not self.has_localization:
            reasons.append("Awaiting AMCL map localization on /amcl_pose.")

        can_dispatch = (len(reasons) == 0)

        if can_dispatch:
            self.lbl_dispatch_banner.setText("✓ All prerequisites met. System ready for waypoint dispatch.")
            self.lbl_dispatch_banner.setStyleSheet(f"""
                QLabel {{
                    background-color: #DCFCE7;
                    border: 1px solid #16A34A;
                    border-radius: {BORDER_RADIUS_SM}px;
                    color: #166534;
                    font-size: 11px;
                    font-weight: 600;
                    padding: 6px 10px;
                }}
            """)
            self.btn_dispatch.setEnabled(True)
        else:
            self.lbl_dispatch_banner.setText(f"⚠ {reasons[0]}")
            self.lbl_dispatch_banner.setStyleSheet(f"""
                QLabel {{
                    background-color: #FEF3C7;
                    border: 1px solid #F59E0B;
                    border-radius: {BORDER_RADIUS_SM}px;
                    color: #92400E;
                    font-size: 11px;
                    font-weight: 600;
                    padding: 6px 10px;
                }}
            """)

    def _on_dest_selected(self, index: int):
        d_key = self.dest_combo.currentData()
        if d_key in self.destinations:
            d = self.destinations[d_key]
            self.lbl_selected_coords.setText(
                f"Target: X={d.get('x', 0.0):+.2f} m, Y={d.get('y', 0.0):+.2f} m, Yaw={d.get('yaw', 0.0):+.2f} rad"
            )

    def _on_map_ready(self, map_dict: dict):
        self.has_map = True
        self.map_view.set_map(
            map_dict["width"], map_dict["height"],
            map_dict["resolution"], map_dict["origin_x"], map_dict["origin_y"],
            map_dict["data"]
        )
        self._update_prerequisites_ui()

    def _on_amcl_pose(self, x, y, yaw, vx, vy, vyaw):
        self.has_localization = True
        self.map_view.set_robot_pose(x, y, yaw)
        self._update_prerequisites_ui()

    def _on_emergency_stop_state(self, active: bool, source: str):
        self.is_estopped = active
        self._update_prerequisites_ui()

    def _log_feedback(self, tag: str, msg: str, tag_color: str = COLOR_INFO):
        """Format and append clean timestamped monospace entry to the feedback stream."""
        now_str = datetime.now().strftime("%H:%M:%S")
        html = (
            f'<div style="margin-bottom: 2px;">'
            f'<span style="color: {COLOR_TEXT_MUTED}; font-family: monospace;">[{now_str}]</span> '
            f'<span style="color: {tag_color}; font-weight: 600; font-family: monospace;">[{tag}]</span> '
            f'<span style="color: {COLOR_TEXT_PRIMARY}; font-family: monospace;">{msg}</span>'
            f'</div>'
        )
        self.txt_feedback.append(html)

    def _on_coordinator_feedback(self, state: str, speech: str, details: dict, ts: float):
        self.coordinator_ready = True
        self.coordinator_state = state
        self.ind_nav_state.set_state(StatusIndicator.STATE_READY)

        # Determine severity / color for state tag in restrained palette
        tag_colors = {
            "NAVIGATING": COLOR_INFO,
            "PLANNING": COLOR_INFO,
            "SUCCEEDED": COLOR_SUCCESS,
            "ARRIVED": COLOR_SUCCESS,
            "PAUSED": COLOR_WARNING,
            "FAILED": COLOR_ERROR,
            "CANCELED": COLOR_TEXT_MUTED,
            "CANCELLED": COLOR_TEXT_MUTED,
            "IDLE": COLOR_TEXT_MUTED,
        }
        color = tag_colors.get(state, COLOR_INFO)
        self._log_feedback(state, speech, color)

        dist = details.get("distance", None)
        if dist is not None:
            self.lbl_dist_remaining.setText(f"Distance Remaining: {dist:.2f} m")

        dest = details.get("destination", None)
        if dest:
            self.lbl_active_dest.setText(f"Destination: {dest}")

        # Update status banner
        if state == "NAVIGATING":
            if not self.elapsed_timer.isActive():
                self.navigation_start_time = time.time()
                self.elapsed_timer.start(500)
            self.lbl_arrival_status.setText("Status: NAVIGATING (IN TRANSIT)")
            self.lbl_arrival_status.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; font-weight: 700; color: {COLOR_INFO};")
        elif state == "PLANNING":
            self.lbl_arrival_status.setText("Status: PLANNING GLOBAL TRAJECTORY")
            self.lbl_arrival_status.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; font-weight: 700; color: {COLOR_INFO};")
        elif state == "PAUSED":
            self.lbl_arrival_status.setText("Status: MISSION PAUSED")
            self.lbl_arrival_status.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; font-weight: 700; color: {COLOR_WARNING};")
        elif state in ("SUCCEEDED", "ARRIVED"):
            self.elapsed_timer.stop()
            self.lbl_arrival_status.setText("Status: ARRIVED & VERIFIED ✅")
            self.lbl_arrival_status.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; font-weight: 700; color: {COLOR_SUCCESS};")
            self.active_destination_key = None
            self.map_view.set_active_goal(None)
        elif state == "FAILED":
            self.elapsed_timer.stop()
            self.lbl_arrival_status.setText("Status: NAVIGATION FAILED ❌")
            self.lbl_arrival_status.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; font-weight: 700; color: {COLOR_ERROR};")
            self.active_destination_key = None
            self.map_view.set_active_goal(None)
        elif state in ("CANCELED", "CANCELLED"):
            self.elapsed_timer.stop()
            self.lbl_arrival_status.setText("Status: MISSION CANCELLED ⏹")
            self.lbl_arrival_status.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; font-weight: 700; color: {COLOR_TEXT_MUTED};")
            self.active_destination_key = None
            self.map_view.set_active_goal(None)
        elif state == "IDLE":
            self.lbl_arrival_status.setText("Status: IDLE (STANDBY)")
            self.lbl_arrival_status.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; font-weight: 700; color: {COLOR_OFFLINE};")

        self._update_prerequisites_ui()

    def _update_elapsed_time(self):
        if self.navigation_start_time:
            dt = time.time() - self.navigation_start_time
            self.lbl_elapsed_time.setText(f"Elapsed Time: {dt:.1f} s")

    def _dispatch_navigation_goal(self):
        """Dispatch navigation goal strictly through authoritative coordinator."""
        if not self.has_localization:
            self._log_feedback("DISPATCH BLOCKED", "AMCL localization required before dispatching goal.", COLOR_WARNING)
            QMessageBox.warning(self, "Navigation Safety Gate", "Cannot dispatch goal: AMCL localization has not established a valid map pose (/amcl_pose)!")
            return

        if self.is_estopped:
            self._log_feedback("DISPATCH BLOCKED", "Emergency Stop is active. Cannot dispatch goal.", COLOR_ERROR)
            QMessageBox.critical(self, "Safety Gate Block", "Cannot dispatch goal: Emergency Stop is active!")
            return

        d_key = self.dest_combo.currentData()
        if not d_key or d_key not in self.destinations:
            return

        dest = self.destinations[d_key]
        self.active_destination_key = d_key
        self.map_view.set_active_goal(d_key)

        bus.navigation_goal_dispatched.emit(
            dest.get("name", d_key), dest.get("x", 0.0), dest.get("y", 0.0), dest.get("yaw", 0.0)
        )
        self._log_feedback("DISPATCH", f"Sent goal for '{dest.get('name', d_key)}' to coordinator.", COLOR_INFO)

    def _cancel_navigation_goal(self):
        """Cancel navigation safely through coordinator."""
        self._log_feedback("CANCEL", "Sending cancellation request to coordinator...", COLOR_WARNING)
        bus.navigation_cancel_requested.emit()
        bus.user_input_received.emit("cancel navigation", "TEXT")

    def _trigger_emergency_stop(self):
        """Trigger immediate emergency stop via authoritative terrain gate."""
        self._log_feedback("EMERGENCY STOP", "Triggering emergency stop across all drive nodes!", COLOR_ERROR)
        bus.emergency_stop_state.emit(True, "GUI_ESTOP_BUTTON")

