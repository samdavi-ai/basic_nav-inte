"""
RViz-Inspired Views & Properties Inspector Panel for GRaCEmo ViRa UARCC.
Provides dynamic camera orbit configuration, selected visualization property inspector,
and quick robotics navigation controls.
"""

from typing import Dict, Any, Optional
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QDoubleSpinBox, QComboBox, QPushButton, QTableWidget,
    QTableWidgetItem, QHeaderView, QTabWidget, QScrollArea
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont, QColor

from ..resources.styles.tokens import (
    COLOR_BG_MAIN,
    COLOR_SURFACE,
    COLOR_SURFACE_ALT,
    COLOR_BORDER,
    COLOR_BORDER_LIGHT,
    COLOR_TEXT_PRIMARY,
    COLOR_TEXT_SECONDARY,
    COLOR_TEXT_MUTED,
    COLOR_BRAND_ORANGE,
    COLOR_SUCCESS,
    COLOR_WARNING,
    COLOR_ERROR,
    COLOR_RVIZ_DOCK_BG,
    FONT_FAMILY_PRIMARY,
    FONT_FAMILY_MONO,
    BORDER_RADIUS_SM,
)
from ..core.event_bus import bus


class ViewsInspectorWidget(QWidget):
    """
    RViz-inspired Right Dock Inspector Panel.
    Contains Camera Orbit Views, Dynamic Item Properties, and Quick Nav Goal Controls.
    """

    camera_changed = Signal(float, float, float)  # distance, pitch, yaw
    preset_requested = Signal(str)               # preset name
    destination_dispatched = Signal(str)          # destination name
    cancel_requested = Signal()
    estop_requested = Signal(bool)

    DESTINATIONS = [
        "Kitchen",
        "Living Room",
        "Bedroom",
        "Bathroom",
        "Entrance",
        "Office",
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("rvizViewsInspector")
        self._init_ui()
        self._connect_bus()

    def _init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(6)

        # Tab Widget separating Views, Properties, and Quick Operations
        self.tabs = QTabWidget(self)
        self.tabs.setStyleSheet(f"""
            QTabWidget::pane {{
                border: 1px solid {COLOR_BORDER};
                background: #FFFFFF;
                border-radius: {BORDER_RADIUS_SM}px;
            }}
            QTabBar::tab {{
                background: {COLOR_SURFACE_ALT};
                color: {COLOR_TEXT_SECONDARY};
                padding: 5px 10px;
                font-size: 10px;
                font-weight: 600;
                border: 1px solid {COLOR_BORDER};
                border-bottom: none;
                margin-right: 2px;
                border-top-left-radius: 3px;
                border-top-right-radius: 3px;
            }}
            QTabBar::tab:selected {{
                background: #FFFFFF;
                color: {COLOR_TEXT_PRIMARY};
                border-bottom: 1px solid #FFFFFF;
            }}
        """)

        # Tab 1: Views
        self.tab_views = self._create_views_tab()
        self.tabs.addTab(self.tab_views, "Views")

        # Tab 2: Properties
        self.tab_properties = self._create_properties_tab()
        self.tabs.addTab(self.tab_properties, "Properties")

        # Tab 3: Quick Nav
        self.tab_quicknav = self._create_quicknav_tab()
        self.tabs.addTab(self.tab_quicknav, "Navigation")

        main_layout.addWidget(self.tabs, stretch=1)

    # -------------------------------------------------------------------------
    # Tab 1: Views (Camera Configuration)
    # -------------------------------------------------------------------------
    def _create_views_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        # View Type
        h_type = QHBoxLayout()
        lbl_type = QLabel("Type:", widget)
        lbl_type.setStyleSheet(f"font-size: 11px; color: {COLOR_TEXT_SECONDARY};")
        self.combo_cam_type = QComboBox(widget)
        self.combo_cam_type.addItems(["Orbit (RViz)", "TopDownOrtho", "FPS (First Person)"])
        self.combo_cam_type.setStyleSheet(f"font-size: 11px; padding: 2px 6px; border: 1px solid {COLOR_BORDER};")
        h_type.addWidget(lbl_type)
        h_type.addWidget(self.combo_cam_type, stretch=1)
        layout.addLayout(h_type)

        # Target Frame
        h_frame = QHBoxLayout()
        lbl_frame = QLabel("Target Frame:", widget)
        lbl_frame.setStyleSheet(f"font-size: 11px; color: {COLOR_TEXT_SECONDARY};")
        self.combo_target_frame = QComboBox(widget)
        self.combo_target_frame.addItems(["<Fixed Frame>", "map", "odom", "base_link"])
        self.combo_target_frame.setStyleSheet(f"font-size: 11px; padding: 2px 6px; border: 1px solid {COLOR_BORDER};")
        h_frame.addWidget(lbl_frame)
        h_frame.addWidget(self.combo_target_frame, stretch=1)
        layout.addLayout(h_frame)

        # Spinboxes for Distance, Pitch, Yaw
        self.spin_dist = self._add_num_control(layout, "Distance (m)", 0.5, 50.0, 7.5, 0.5)
        self.spin_pitch = self._add_num_control(layout, "Pitch (deg)", -89.0, 89.0, 35.0, 5.0)
        self.spin_yaw = self._add_num_control(layout, "Yaw (deg)", -180.0, 180.0, -45.0, 15.0)

        # Connect value change signals
        self.spin_dist.valueChanged.connect(self._on_cam_control_changed)
        self.spin_pitch.valueChanged.connect(self._on_cam_control_changed)
        self.spin_yaw.valueChanged.connect(self._on_cam_control_changed)

        layout.addSpacing(6)

        # Preset View Buttons
        lbl_presets = QLabel("Camera Presets", widget)
        lbl_presets.setStyleSheet(f"font-weight: 700; font-size: 11px; color: {COLOR_TEXT_PRIMARY};")
        layout.addWidget(lbl_presets)

        preset_grid = QVBoxLayout()
        preset_grid.setSpacing(4)

        row1 = QHBoxLayout()
        btn_iso = self._make_btn("Isometric (3D)", lambda: self.preset_requested.emit("iso"))
        btn_top = self._make_btn("Top-Down (2D)", lambda: self.preset_requested.emit("top"))
        row1.addWidget(btn_iso)
        row1.addWidget(btn_top)
        preset_grid.addLayout(row1)

        row2 = QHBoxLayout()
        btn_front = self._make_btn("Front View", lambda: self.preset_requested.emit("front"))
        btn_side = self._make_btn("Side View", lambda: self.preset_requested.emit("left"))
        row2.addWidget(btn_front)
        row2.addWidget(btn_side)
        preset_grid.addLayout(row2)

        row3 = QHBoxLayout()
        self.btn_follow = self._make_btn("Follow Robot: OFF", self._toggle_follow)
        btn_reset = self._make_btn("Reset View", lambda: self.preset_requested.emit("reset"))
        row3.addWidget(self.btn_follow)
        row3.addWidget(btn_reset)
        preset_grid.addLayout(row3)

        layout.addLayout(preset_grid)
        layout.addStretch()

        return widget

    def _add_num_control(self, parent_layout: QVBoxLayout, label: str, v_min: float, v_max: float, v_init: float, step: float) -> QDoubleSpinBox:
        h = QHBoxLayout()
        lbl = QLabel(label)
        lbl.setStyleSheet(f"font-size: 11px; color: {COLOR_TEXT_SECONDARY};")
        spin = QDoubleSpinBox()
        spin.setRange(v_min, v_max)
        spin.setValue(v_init)
        spin.setSingleStep(step)
        spin.setDecimals(1)
        spin.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; padding: 2px 4px; border: 1px solid {COLOR_BORDER};")
        h.addWidget(lbl)
        h.addWidget(spin, stretch=1)
        parent_layout.addLayout(h)
        return spin

    def _on_cam_control_changed(self):
        dist = self.spin_dist.value()
        pitch = self.spin_pitch.value()
        yaw = self.spin_yaw.value()
        self.camera_changed.emit(dist, pitch, yaw)

    def _toggle_follow(self):
        if "OFF" in self.btn_follow.text():
            self.btn_follow.setText("Follow Robot: ON")
            self.btn_follow.setStyleSheet(f"background-color: {COLOR_BRAND_ORANGE}; color: #FFFFFF; font-weight: 600; font-size: 10px; padding: 4px; border-radius: {BORDER_RADIUS_SM}px;")
        else:
            self.btn_follow.setText("Follow Robot: OFF")
            self.btn_follow.setStyleSheet(f"background-color: #FFFFFF; border: 1px solid {COLOR_BORDER}; font-size: 10px; padding: 4px; border-radius: {BORDER_RADIUS_SM}px;")
        self.preset_requested.emit("follow")

    def update_camera_values(self, view_dict: dict):
        """Called when viewport emits view_updated to synchronize spinboxes."""
        self.spin_dist.blockSignals(True)
        self.spin_pitch.blockSignals(True)
        self.spin_yaw.blockSignals(True)

        if "distance" in view_dict:
            self.spin_dist.setValue(view_dict["distance"])
        if "pitch" in view_dict:
            self.spin_pitch.setValue(view_dict["pitch"])
        if "yaw" in view_dict:
            self.spin_yaw.setValue(view_dict["yaw"])
        if "follow_robot" in view_dict:
            if view_dict["follow_robot"]:
                self.btn_follow.setText("Follow Robot: ON")
                self.btn_follow.setStyleSheet(f"background-color: {COLOR_BRAND_ORANGE}; color: #FFFFFF; font-weight: 600; font-size: 10px; padding: 4px; border-radius: {BORDER_RADIUS_SM}px;")
            else:
                self.btn_follow.setText("Follow Robot: OFF")
                self.btn_follow.setStyleSheet(f"background-color: #FFFFFF; border: 1px solid {COLOR_BORDER}; font-size: 10px; padding: 4px; border-radius: {BORDER_RADIUS_SM}px;")

        self.spin_dist.blockSignals(False)
        self.spin_pitch.blockSignals(False)
        self.spin_yaw.blockSignals(False)

    # -------------------------------------------------------------------------
    # Tab 2: Properties Inspector
    # -------------------------------------------------------------------------
    def _create_properties_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        self.lbl_selected_item = QLabel("Selected: Robot Model", widget)
        self.lbl_selected_item.setStyleSheet(f"font-weight: 700; font-size: 11px; color: {COLOR_TEXT_PRIMARY};")
        layout.addWidget(self.lbl_selected_item)

        self.props_table = QTableWidget(widget)
        self.props_table.setColumnCount(2)
        self.props_table.setHorizontalHeaderLabels(["Property", "Value"])
        self.props_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Interactive)
        self.props_table.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.props_table.horizontalHeader().resizeSection(0, 110)
        self.props_table.verticalHeader().setVisible(False)
        self.props_table.setStyleSheet(f"""
            QTableWidget {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                font-family: {FONT_FAMILY_PRIMARY};
                font-size: 11px;
                color: {COLOR_TEXT_PRIMARY};
            }}
            QHeaderView::section {{
                background-color: {COLOR_SURFACE_ALT};
                color: {COLOR_TEXT_SECONDARY};
                font-weight: 600;
                font-size: 10px;
                padding: 4px;
                border: none;
                border-bottom: 1px solid {COLOR_BORDER};
            }}
        """)
        layout.addWidget(self.props_table, stretch=1)

        # Set default properties for Robot Model
        self.set_item_properties("Robot Model", {
            "Description Topic": "/robot_description",
            "Canonical Model": "GRaCEmo ViRa (Humanoid + Wheels)",
            "Visual Enabled": "True",
            "Collision Enabled": "False",
            "Alpha": "1.0",
            "Fixed Frame": "map"
        })

        return widget

    def set_item_properties(self, name: str, props: Dict[str, Any]):
        self.lbl_selected_item.setText(f"Selected: {name}")
        self.props_table.setRowCount(len(props))
        for row, (k, v) in enumerate(props.items()):
            item_k = QTableWidgetItem(str(k))
            item_k.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable)
            item_k.setForeground(QColor(COLOR_TEXT_SECONDARY))

            item_v = QTableWidgetItem(str(v))
            item_v.setFlags(Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsEditable)
            item_v.setFont(QFont(FONT_FAMILY_MONO, 9))
            item_v.setForeground(QColor(COLOR_TEXT_PRIMARY))

            self.props_table.setItem(row, 0, item_k)
            self.props_table.setItem(row, 1, item_v)

    # -------------------------------------------------------------------------
    # Tab 3: Quick Navigation
    # -------------------------------------------------------------------------
    def _create_quicknav_tab(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)

        lbl_hdr = QLabel("Waypoint Dispatcher", widget)
        lbl_hdr.setStyleSheet(f"font-weight: 700; font-size: 11px; color: {COLOR_TEXT_PRIMARY};")
        layout.addWidget(lbl_hdr)

        self.combo_dest = QComboBox(widget)
        self.combo_dest.addItems(self.DESTINATIONS)
        self.combo_dest.setStyleSheet(f"font-size: 11px; padding: 4px 8px; border: 1px solid {COLOR_BORDER}; border-radius: {BORDER_RADIUS_SM}px;")
        layout.addWidget(self.combo_dest)

        btn_dispatch = QPushButton("Dispatch Nav Goal", widget)
        btn_dispatch.setStyleSheet(f"""
            QPushButton {{
                background-color: {COLOR_BRAND_ORANGE};
                color: #FFFFFF;
                font-weight: 600;
                font-size: 11px;
                padding: 6px;
                border-radius: {BORDER_RADIUS_SM}px;
                border: none;
            }}
            QPushButton:hover {{
                background-color: #E05A00;
            }}
        """)
        btn_dispatch.clicked.connect(self._on_dispatch)
        layout.addWidget(btn_dispatch)

        btn_cancel = QPushButton("Cancel Active Goal", widget)
        btn_cancel.setStyleSheet(f"""
            QPushButton {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                font-size: 11px;
                padding: 5px;
                border-radius: {BORDER_RADIUS_SM}px;
                color: {COLOR_TEXT_PRIMARY};
            }}
            QPushButton:hover {{
                background-color: {COLOR_SURFACE_ALT};
            }}
        """)
        btn_cancel.clicked.connect(lambda: self.cancel_requested.emit())
        layout.addWidget(btn_cancel)

        layout.addSpacing(10)

        # Prominent Emergency Stop
        lbl_safety = QLabel("Hardware Safety Gate", widget)
        lbl_safety.setStyleSheet(f"font-weight: 700; font-size: 11px; color: {COLOR_ERROR};")
        layout.addWidget(lbl_safety)

        self.btn_estop = QPushButton("🚨 EMERGENCY STOP", widget)
        self.btn_estop.setStyleSheet(f"""
            QPushButton {{
                background-color: {COLOR_ERROR};
                color: #FFFFFF;
                font-weight: 800;
                font-size: 12px;
                padding: 10px;
                border-radius: {BORDER_RADIUS_SM}px;
                border: 1px solid #991B1B;
            }}
            QPushButton:hover {{
                background-color: #DC2626;
            }}
        """)
        self.btn_estop.clicked.connect(self._toggle_estop)
        layout.addWidget(self.btn_estop)

        self.lbl_nav_status = QLabel("State: IDLE", widget)
        self.lbl_nav_status.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 10px; color: {COLOR_TEXT_SECONDARY}; padding: 4px;")
        layout.addWidget(self.lbl_nav_status)

        layout.addStretch()
        return widget

    def _on_dispatch(self):
        target = self.combo_dest.currentText()
        self.destination_dispatched.emit(target)
        bus.request_destination.emit(target)

    def _toggle_estop(self):
        bus.emergency_stop_state.emit(True, "UARCC Views Inspector")

    def _connect_bus(self):
        bus.coordinator_feedback_ready.connect(self._on_feedback)

    def _on_feedback(self, status: str, speech: str, details: dict, timestamp: float):
        self.lbl_nav_status.setText(f"State: {status}")

    def _make_btn(self, text: str, callback) -> QPushButton:
        btn = QPushButton(text, self)
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 4px;
                font-size: 10px;
                color: {COLOR_TEXT_PRIMARY};
            }}
            QPushButton:hover {{
                background-color: {COLOR_SURFACE_ALT};
                border-color: {COLOR_BORDER_LIGHT};
            }}
        """)
        btn.clicked.connect(callback)
        return btn
