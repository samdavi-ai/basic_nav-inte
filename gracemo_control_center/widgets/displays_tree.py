"""
RViz-Inspired Displays Hierarchical Tree Widget for GRaCEmo ViRa UARCC.
Manages 3D visualization layers, property hierarchy, ROS topic bindings,
and live data stream health status.
"""

from typing import Dict, Any, Optional
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTreeWidget, QTreeWidgetItem,
    QLabel, QPushButton, QHeaderView, QFrame
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor, QBrush, QFont

from ..resources.styles.tokens import (
    COLOR_BG_MAIN,
    COLOR_SURFACE,
    COLOR_SURFACE_ALT,
    COLOR_BORDER,
    COLOR_BORDER_LIGHT,
    COLOR_TEXT_PRIMARY,
    COLOR_TEXT_SECONDARY,
    COLOR_TEXT_MUTED,
    COLOR_SUCCESS,
    COLOR_WARNING,
    COLOR_ERROR,
    COLOR_INFO,
    COLOR_RVIZ_DOCK_BG,
    COLOR_RVIZ_SELECTION,
    FONT_FAMILY_PRIMARY,
    FONT_FAMILY_MONO,
    BORDER_RADIUS_SM,
)
from ..core.event_bus import bus


class DisplaysTreeWidget(QWidget):
    """
    Hierarchical Displays Tree mirroring ROS 2 RViz Displays panel.
    Controls visibility, properties, and health telemetry of visualization layers.
    """

    display_toggled = Signal(str, bool)     # (display_name, is_visible)
    display_selected = Signal(str, dict)    # (display_name, properties_dict)
    fixed_frame_changed = Signal(str)       # new_frame

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("rvizDisplaysPanel")

        self._display_items: Dict[str, QTreeWidgetItem] = {}
        self._display_props: Dict[str, Dict[str, Any]] = {}
        self._status_labels: Dict[str, QTreeWidgetItem] = {}

        self._init_ui()
        self._populate_displays()
        self._connect_bus_signals()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        # Header bar
        header_frame = QFrame(self)
        header_frame.setStyleSheet(f"background-color: {COLOR_SURFACE_ALT}; border: 1px solid {COLOR_BORDER}; border-radius: {BORDER_RADIUS_SM}px; padding: 4px 8px;")
        h_layout = QHBoxLayout(header_frame)
        h_layout.setContentsMargins(0, 0, 0, 0)
        h_layout.setSpacing(6)

        lbl_title = QLabel("Displays", header_frame)
        lbl_title.setStyleSheet(f"font-weight: 700; font-size: 11px; color: {COLOR_TEXT_PRIMARY};")
        h_layout.addWidget(lbl_title)
        h_layout.addStretch()

        self.lbl_stream_count = QLabel("Active: 8 / 11", header_frame)
        self.lbl_stream_count.setStyleSheet(f"font-size: 10px; color: {COLOR_SUCCESS}; font-family: {FONT_FAMILY_MONO};")
        h_layout.addWidget(self.lbl_stream_count)

        layout.addWidget(header_frame)

        # Tree Widget
        self.tree = QTreeWidget(self)
        self.tree.setColumnCount(2)
        self.tree.setHeaderLabels(["Display / Property", "Value / Status"])
        self.tree.header().setSectionResizeMode(0, QHeaderView.Interactive)
        self.tree.header().setSectionResizeMode(1, QHeaderView.Stretch)
        self.tree.header().resizeSection(0, 160)
        self.tree.setIndentation(14)
        self.tree.setAnimated(True)
        self.tree.setStyleSheet(f"""
            QTreeWidget {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                font-family: {FONT_FAMILY_PRIMARY};
                font-size: 11px;
                color: {COLOR_TEXT_PRIMARY};
            }}
            QTreeWidget::item {{
                height: 22px;
                padding-right: 4px;
            }}
            QTreeWidget::item:hover {{
                background-color: {COLOR_SURFACE_ALT};
            }}
            QTreeWidget::item:selected {{
                background-color: #E2E8F0;
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

        self.tree.itemChanged.connect(self._on_item_changed)
        self.tree.itemClicked.connect(self._on_item_clicked)
        layout.addWidget(self.tree, stretch=1)

        # Bottom Action Bar (Add, Remove, Reset)
        btn_bar = QHBoxLayout()
        btn_bar.setContentsMargins(0, 0, 0, 0)
        btn_bar.setSpacing(4)

        btn_style = f"""
            QPushButton {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 3px 8px;
                font-size: 10px;
                font-weight: 500;
                color: {COLOR_TEXT_PRIMARY};
            }}
            QPushButton:hover {{
                background-color: {COLOR_SURFACE_ALT};
                border-color: {COLOR_BORDER_LIGHT};
            }}
        """

        self.btn_expand_all = QPushButton("Expand All", self)
        self.btn_expand_all.setStyleSheet(btn_style)
        self.btn_expand_all.clicked.connect(self.tree.expandAll)
        btn_bar.addWidget(self.btn_expand_all)

        self.btn_collapse_all = QPushButton("Collapse All", self)
        self.btn_collapse_all.setStyleSheet(btn_style)
        self.btn_collapse_all.clicked.connect(self.tree.collapseAll)
        btn_bar.addWidget(self.btn_collapse_all)

        self.btn_reset = QPushButton("Reset Defaults", self)
        self.btn_reset.setStyleSheet(btn_style)
        self.btn_reset.clicked.connect(self._reset_defaults)
        btn_bar.addWidget(self.btn_reset)

        layout.addLayout(btn_bar)

    def _populate_displays(self):
        """Build the canonical RViz display tree entries."""
        self.tree.clear()
        self._display_items.clear()
        self._display_props.clear()

        # 1. Global Options
        global_item = QTreeWidgetItem(self.tree, ["Global Options", ""])
        global_item.setFont(0, QFont(FONT_FAMILY_PRIMARY, 10, QFont.Bold))
        self._add_child(global_item, "Fixed Frame", "map", editable=True)
        self._add_child(global_item, "Background Color", "#1E1E1E")
        self._add_child(global_item, "Frame Rate", "30 FPS")
        global_item.setExpanded(True)

        # 2. Grid
        self._create_display_entry(
            name="Grid",
            default_checked=True,
            status_text="OK",
            status_color=COLOR_SUCCESS,
            props={
                "Reference Frame": "map",
                "Plane": "XY",
                "Cell Size": "1.0 m",
                "Grid Cells": "10",
                "Color": "64, 64, 64",
                "Alpha": "0.5"
            }
        )

        # 3. Robot Model
        self._create_display_entry(
            name="Robot Model",
            default_checked=True,
            status_text="OK",
            status_color=COLOR_SUCCESS,
            props={
                "Description Topic": "/robot_description",
                "Canonical Model": "GRaCEmo ViRa (Humanoid + Wheels)",
                "Visual Enabled": "True",
                "Collision Enabled": "False",
                "Alpha": "1.0",
                "TF Prefix": ""
            }
        )

        # 4. TF (Transform Tree)
        self._create_display_entry(
            name="TF",
            default_checked=True,
            status_text="OK",
            status_color=COLOR_SUCCESS,
            props={
                "Frames": "map → odom → base_link → lidar_link",
                "Show Names": "True",
                "Show Axes": "True",
                "Marker Scale": "0.4 m",
                "Update Rate": "50 Hz"
            }
        )

        # 5. Map
        self._create_display_entry(
            name="Map",
            default_checked=True,
            status_text="OK (384x384)",
            status_color=COLOR_SUCCESS,
            props={
                "Topic": "/map",
                "Resolution": "0.05 m/cell",
                "Update Topic": "/map_updates",
                "Color Scheme": "map (Occupancy)",
                "Alpha": "0.7"
            }
        )

        # 6. LaserScan
        self._create_display_entry(
            name="LaserScan",
            default_checked=True,
            status_text="OK (10 Hz)",
            status_color=COLOR_SUCCESS,
            props={
                "Topic": "/scan",
                "Frame": "lidar_link",
                "Size (m)": "0.06",
                "Decay Time": "0.0 s",
                "Color Transformer": "Intensity / Orange"
            }
        )

        # 7. PointCloud2 (Disabled if idle)
        self._create_display_entry(
            name="PointCloud2",
            default_checked=False,
            status_text="No Data (Idle)",
            status_color=COLOR_TEXT_MUTED,
            props={
                "Topic": "/camera/points",
                "Frame": "camera_depth_optical_frame",
                "Size (m)": "0.02",
                "Color Transformer": "RGB8"
            }
        )

        # 8. RGB Camera
        self._create_display_entry(
            name="RGB Camera",
            default_checked=True,
            status_text="OK (15 Hz)",
            status_color=COLOR_SUCCESS,
            props={
                "Topic": "/camera/image_raw",
                "Transport": "raw",
                "Resolution": "640 x 480",
                "Encoding": "rgb8"
            }
        )

        # 9. Depth Camera
        self._create_display_entry(
            name="Depth Camera",
            default_checked=True,
            status_text="OK (15 Hz)",
            status_color=COLOR_SUCCESS,
            props={
                "Topic": "/camera/depth/image_raw",
                "Transport": "raw",
                "Resolution": "640 x 480",
                "Encoding": "16UC1"
            }
        )

        # 10. Odometry
        self._create_display_entry(
            name="Odometry",
            default_checked=True,
            status_text="OK (50 Hz)",
            status_color=COLOR_SUCCESS,
            props={
                "Topic": "/odom",
                "Keep": "100",
                "Position Tolerance": "0.1 m",
                "Angle Tolerance": "0.1 rad"
            }
        )

        # 11. AMCL Particle Cloud
        self._create_display_entry(
            name="AMCL Particle Cloud",
            default_checked=True,
            status_text="OK (500 pts)",
            status_color=COLOR_SUCCESS,
            props={
                "Topic": "/particle_cloud",
                "Shape": "Arrows",
                "Color": "255, 140, 0",
                "Alpha": "0.8"
            }
        )

        # 12. Global Path
        self._create_display_entry(
            name="Global Path",
            default_checked=True,
            status_text="OK",
            status_color=COLOR_SUCCESS,
            props={
                "Topic": "/plan",
                "Line Width": "0.08 m",
                "Color": "0, 210, 255 (Cyan)",
                "Alpha": "1.0"
            }
        )

        # 13. Costmap
        self._create_display_entry(
            name="Costmap",
            default_checked=True,
            status_text="OK",
            status_color=COLOR_SUCCESS,
            props={
                "Topic": "/global_costmap/costmap",
                "Color Scheme": "costmap",
                "Alpha": "0.5"
            }
        )

        # 14. Waypoints
        self._create_display_entry(
            name="Waypoints",
            default_checked=True,
            status_text="6 Registered",
            status_color=COLOR_SUCCESS,
            props={
                "Locations": "Kitchen, Living Room, Bedroom, Bathroom, Entrance, Office",
                "Marker Type": "Pin & Label",
                "Active Target": "None"
            }
        )

        # 15. Safety Zones
        self._create_display_entry(
            name="Safety Zones",
            default_checked=True,
            status_text="Monitoring",
            status_color=COLOR_SUCCESS,
            props={
                "Emergency Stop Envelope": "0.35 m (Red)",
                "Proximity Envelope": "0.80 m (Amber)",
                "Collision Monitor": "Active"
            }
        )

    def _create_display_entry(self, name: str, default_checked: bool, status_text: str, status_color: str, props: Dict[str, str]):
        item = QTreeWidgetItem(self.tree)
        item.setText(0, name)
        item.setText(1, status_text)
        item.setForeground(1, QBrush(QColor(status_color)))
        item.setCheckState(0, Qt.Checked if default_checked else Qt.Unchecked)
        item.setFont(0, QFont(FONT_FAMILY_PRIMARY, 10, QFont.DemiBold))

        self._display_items[name] = item
        self._display_props[name] = props

        # Status row child
        st_child = self._add_child(item, "Status", status_text)
        st_child.setForeground(1, QBrush(QColor(status_color)))
        self._status_labels[name] = st_child

        # Property rows
        for k, v in props.items():
            self._add_child(item, k, str(v))

    def _add_child(self, parent_item: QTreeWidgetItem, label: str, value: str, editable: bool = False) -> QTreeWidgetItem:
        child = QTreeWidgetItem(parent_item, [f"  {label}", value])
        child.setFont(0, QFont(FONT_FAMILY_PRIMARY, 9))
        child.setFont(1, QFont(FONT_FAMILY_MONO, 9))
        child.setForeground(0, QBrush(QColor(COLOR_TEXT_SECONDARY)))
        child.setForeground(1, QBrush(QColor(COLOR_TEXT_PRIMARY)))
        return child

    def _on_item_changed(self, item: QTreeWidgetItem, column: int):
        if column == 0:
            name = item.text(0).strip()
            if name in self._display_items:
                is_checked = (item.checkState(0) == Qt.Checked)
                self.display_toggled.emit(name, is_checked)

    def _on_item_clicked(self, item: QTreeWidgetItem, column: int):
        name = item.text(0).strip()
        # If clicked child, find parent
        parent = item.parent()
        if parent:
            parent_name = parent.text(0).strip()
            if parent_name in self._display_props:
                self.display_selected.emit(parent_name, self._display_props[parent_name])
                return

        if name in self._display_props:
            self.display_selected.emit(name, self._display_props[name])

    def _reset_defaults(self):
        for name, item in self._display_items.items():
            if name == "PointCloud2":
                item.setCheckState(0, Qt.Unchecked)
            else:
                item.setCheckState(0, Qt.Checked)
        self.tree.expandAll()

    # -------------------------------------------------------------------------
    # Telemetry Ingestion to keep status authentic
    # -------------------------------------------------------------------------
    def _connect_bus_signals(self):
        bus.ros_connected.connect(self._on_ros_status)
        bus.gazebo_connected.connect(self._on_gazebo_status)
        bus.map_ready.connect(self._on_map_ready)
        bus.lidar_scan_ready.connect(self._on_scan_ready)
        bus.amcl_pose_ready.connect(self._on_amcl_ready)

    def _on_ros_status(self, connected: bool):
        st = "OK" if connected else "Disconnected"
        color = COLOR_SUCCESS if connected else COLOR_ERROR
        if "TF" in self._status_labels:
            self._status_labels["TF"].setText(1, st)
            self._status_labels["TF"].setForeground(1, QBrush(QColor(color)))

    def _on_gazebo_status(self, connected: bool):
        st = "OK (Harmonic)" if connected else "GZ Offline"
        color = COLOR_SUCCESS if connected else COLOR_WARNING
        if "Robot Model" in self._status_labels:
            self._status_labels["Robot Model"].setText(1, st)
            self._status_labels["Robot Model"].setForeground(1, QBrush(QColor(color)))

    def _on_map_ready(self, map_dict: dict):
        w = map_dict.get("width", 0)
        h = map_dict.get("height", 0)
        st = f"OK ({w}x{h})"
        if "Map" in self._status_labels:
            self._status_labels["Map"].setText(1, st)
            self._status_labels["Map"].setForeground(1, QBrush(QColor(COLOR_SUCCESS)))

    def _on_scan_ready(self, scan_dict: dict):
        cnt = len(scan_dict.get("ranges", []))
        st = f"OK ({cnt} pts @ 10 Hz)"
        if "LaserScan" in self._status_labels:
            self._status_labels["LaserScan"].setText(1, st)
            self._status_labels["LaserScan"].setForeground(1, QBrush(QColor(COLOR_SUCCESS)))

    def _on_amcl_ready(self, *args):
        if "AMCL Particle Cloud" in self._status_labels:
            self._status_labels["AMCL Particle Cloud"].setText(1, "OK (Localized)")
            self._status_labels["AMCL Particle Cloud"].setForeground(1, QBrush(QColor(COLOR_SUCCESS)))
