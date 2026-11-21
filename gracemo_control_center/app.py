"""
GRaCEmo ViRa — Unified Autonomous Robotics Control Center (UARCC).
Desktop Engineering Workstation Layout inspired by ROS 2 RViz:
- Top Menu Bar: File, Panels, Tools, View, Settings, Help
- Compact Robotics Toolbar: Tools, Presets, Fixed Frame, Subsystem Badges, Sim Clock
- Canonical 3-Group Sidebar Navigation: WORKSPACE, ROBOT SYSTEMS, ENGINEERING
- Central Workspace: 11 Canonical Engineering Modules with Embedded RViz2 in Navigation
- Inspection Docks: Collapsible Displays Tree & Views Inspector
- Bottom Status Bar: Live System Telemetry, Notifications & Hardware Lock State
"""

from typing import Dict, Any, Optional

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QFrame, QPushButton, QStackedWidget, QDockWidget, QToolBar,
    QToolButton, QButtonGroup, QComboBox, QMenuBar, QMenu,
    QTabBar, QStatusBar, QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QFont, QIcon, QAction, QColor

from .resources.styles.tokens import (
    COLOR_BG_MAIN,
    COLOR_WORKSPACE,
    COLOR_SURFACE,
    COLOR_SURFACE_ALT,
    COLOR_SIDEBAR_BG,
    COLOR_BORDER,
    COLOR_BORDER_LIGHT,
    COLOR_BRAND_ORANGE,
    COLOR_ACCENT_PRIMARY,
    COLOR_ACCENT_CYAN,
    COLOR_SUBTLE_SURFACE,
    COLOR_SUCCESS,
    COLOR_WARNING,
    COLOR_ERROR,
    COLOR_INFO,
    COLOR_TEXT_PRIMARY,
    COLOR_TEXT_SECONDARY,
    COLOR_TEXT_MUTED,
    COLOR_TEXT_SIDEBAR_MUTED,
    COLOR_RVIZ_VIEWPORT_BG,
    COLOR_RVIZ_DOCK_BG,
    COLOR_RVIZ_SELECTION,
    FONT_FAMILY_PRIMARY,
    FONT_FAMILY_MONO,
    BORDER_RADIUS_SM,
    BORDER_RADIUS_MD,
)
from .resources.styles.dark_theme import get_application_stylesheet
from .widgets.status_indicator import StatusIndicator
from .widgets.rviz_viewport import RVizViewportWidget
from .widgets.embedded_rviz_widget import EmbeddedRVizWidget
from .widgets.displays_tree import DisplaysTreeWidget
from .widgets.views_inspector import ViewsInspectorWidget
from .config.settings import Settings
from .core.event_bus import bus

# Import Module Views
from .modules.overview.overview_view import OverviewView
from .modules.autonomy.autonomy_view import AutonomyView
from .modules.simulation.simulation_view import SimulationView
from .modules.sensors.sensors_view import SensorsView
from .modules.navigation.navigation_view import NavigationView
from .modules.localization.localization_view import LocalizationView
from .modules.interaction.interaction_view import InteractionView
from .modules.cognitive.cognitive_view import CognitiveView
from .modules.safety.safety_view import SafetyView
from .modules.diagnostics.diagnostics_view import DiagnosticsView
from .modules.logs.logs_view import LogsView
from .modules.settings.settings_view import SettingsView


class MainWindow(QMainWindow):
    """Primary UARCC Engineering Desktop Application Window (RViz-Inspired)."""

    NAVIGATION_GROUPS = [
        ("WORKSPACE", [
            ("Overview", "Overview"),
            ("Autonomous Operations", "Autonomous Operations"),
            ("Simulation", "Simulation"),
            ("Navigation", "Navigation"),
        ]),
        ("ROBOT SYSTEMS", [
            ("Sensors & Perception", "Sensors & Perception"),
            ("Localization & Kinematics", "Localization & Kinematics"),
            ("Cognitive System", "Cognitive System"),
        ]),
        ("ENGINEERING", [
            ("Safety", "Safety"),
            ("Diagnostics", "Diagnostics"),
            ("System Logs", "System Logs"),
            ("Settings", "Settings"),
        ]),
    ]

    MODULE_NAMES = [
        ("Overview", "Overview"),
        ("Autonomous Operations", "Autonomous Operations"),
        ("Simulation", "Simulation"),
        ("Navigation", "Navigation"),
        ("Sensors & Perception", "Sensors & Perception"),
        ("Localization & Kinematics", "Localization & Kinematics"),
        ("Cognitive System", "Cognitive System"),
        ("Safety", "Safety"),
        ("Diagnostics", "Diagnostics"),
        ("System Logs", "System Logs"),
        ("Settings", "Settings"),
    ]

    MODULE_DESCRIPTIONS = {
        "Overview": "Operational readiness, subsystem health matrix, and live pose telemetry",
        "Autonomous Operations": "Central supervisory FSM, multi-waypoint patrol manager, and safety arbitration gate",
        "Simulation": "Gazebo Harmonic environment, physics clock, and simulation runtime control",
        "Navigation": "Embedded ROS 2 RViz2 3D workspace, waypoint routing, and Nav2 tracking",
        "Sensors & Perception": "Multi-modal vision feeds, 360° planar LiDAR, IMU attitude, and odometry",
        "Localization & Kinematics": "AMCL particle filter covariance and authoritative TF coordinate tree",
        "Cognitive System": "MNSE/K cognitive kernel, Agent Bridge task queue, Whisper STT, and voice pipeline",
        "Safety": "Authoritative E-Stop gate, collision monitor, terrain limits, and hardware safety locks",
        "Diagnostics": "ROS 2 Jazzy node graph health, topic telemetry, and host CPU/RAM/GPU metrics",
        "System Logs": "Unified multi-source event logging, severity filtering, and export tools",
        "Settings": "Application refresh rates, sensor visualization tolerances, and preferences",
    }

    def __init__(self, settings: Settings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.setWindowTitle("GRaCEmo ViRa — Unified Autonomous Robotics Control Center (UARCC)")
        self.resize(1500, 920)
        self.setMinimumSize(1024, 720)

        # Apply global professional engineering stylesheet
        self.setStyleSheet(get_application_stylesheet())

        # 1. Top Menu Bar
        self._create_menu_bar()

        # 2. Compact Robotics Toolbar
        self._create_robotics_toolbar()

        # 3. Central Engineering Workspace (Sidebar + 11 Canonical Stacked Module Views)
        self._create_central_workspace()

        # 4. Left Dock: Displays Tree (Collapsible/Optional)
        self._create_left_dock()

        # 5. Right Dock: Views & Properties Inspector (Collapsible/Optional)
        self._create_right_dock()

        # 6. Bottom Engineering Status Bar
        self._create_bottom_status_bar()

        # Wire Inter-Widget Signals (Displays <-> Viewport <-> Inspector)
        self._wire_rviz_signals()

        # Connect Navigation routing signal
        bus.request_page_navigation.connect(self.switch_to_page)

        # Default to Overview as operational entry point
        self.switch_to_page("Overview")

    # =========================================================================
    # 1. TOP MENU BAR
    # =========================================================================
    def _create_menu_bar(self):
        menubar = self.menuBar()
        menubar.setStyleSheet(f"""
            QMenuBar {{
                background-color: {COLOR_SURFACE_ALT};
                color: {COLOR_TEXT_PRIMARY};
                font-size: 12px;
                border-bottom: 1px solid {COLOR_BORDER};
                padding: 2px 4px;
            }}
            QMenuBar::item {{
                background-color: transparent;
                padding: 4px 8px;
                border-radius: {BORDER_RADIUS_SM}px;
            }}
            QMenuBar::item:selected {{
                background-color: #E2E8F0;
            }}
            QMenu {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                color: {COLOR_TEXT_PRIMARY};
                font-size: 12px;
                padding: 4px;
            }}
            QMenu::item {{
                padding: 5px 20px 5px 20px;
                border-radius: 2px;
            }}
            QMenu::item:selected {{
                background-color: #E2E8F0;
            }}
            QMenu::separator {{
                height: 1px;
                background-color: {COLOR_BORDER};
                margin: 4px 0px;
            }}
        """)

        # File Menu
        menu_file = menubar.addMenu("&File")
        act_open_config = menu_file.addAction("Open RViz Config...")
        act_save_config = menu_file.addAction("Save Config As...")
        menu_file.addSeparator()
        act_reset_all = menu_file.addAction("Reset Visualization Defaults")
        act_reset_all.triggered.connect(self._on_reset_all_defaults)
        menu_file.addSeparator()
        act_exit = menu_file.addAction("Exit UARCC")
        act_exit.triggered.connect(self.close)

        # Panels Menu
        self.menu_panels = menubar.addMenu("&Panels")

        # Tools Menu
        menu_tools = menubar.addMenu("&Tools")
        act_tool_interact = menu_tools.addAction("Interact")
        act_tool_interact.triggered.connect(lambda: self._set_active_tool("interact"))
        act_tool_move = menu_tools.addAction("Move Camera")
        act_tool_move.triggered.connect(lambda: self._set_active_tool("move_camera"))
        act_tool_select = menu_tools.addAction("Select")
        act_tool_select.triggered.connect(lambda: self._set_active_tool("select"))
        menu_tools.addSeparator()
        act_tool_pose = menu_tools.addAction("2D Pose Estimate")
        act_tool_pose.triggered.connect(lambda: self._set_active_tool("pose_estimate"))
        act_tool_goal = menu_tools.addAction("2D Nav Goal")
        act_tool_goal.triggered.connect(lambda: self._set_active_tool("nav_goal"))

        # View Menu
        menu_view = menubar.addMenu("&View")
        for cat_name, items in self.NAVIGATION_GROUPS:
            menu_cat = menu_view.addMenu(cat_name)
            for label, mod_name in items:
                act_m = menu_cat.addAction(label)
                act_m.triggered.connect(lambda _, name=mod_name: self.switch_to_page(name))
        menu_view.addSeparator()
        act_v_iso = menu_view.addAction("Camera: Isometric (3D)")
        act_v_iso.triggered.connect(lambda: self.viewport_3d.set_view_preset("iso"))
        act_v_top = menu_view.addAction("Camera: Top-Down (2D)")
        act_v_top.triggered.connect(lambda: self.viewport_3d.set_view_preset("top"))
        act_v_reset = menu_view.addAction("Camera: Reset View")
        act_v_reset.triggered.connect(lambda: self.viewport_3d.reset_view())

        # Settings Menu
        menu_settings = menubar.addMenu("&Settings")
        act_hw_lock = menu_settings.addAction("Hardware Safety Lock (Strictly Enforced)")
        act_hw_lock.setEnabled(False)
        act_hw_lock.setCheckable(True)
        act_hw_lock.setChecked(True)
        act_pref = menu_settings.addAction("Preferences...")
        act_pref.triggered.connect(lambda: self.switch_to_page("Settings"))

        # Help Menu
        menu_help = menubar.addMenu("&Help")
        act_about = menu_help.addAction("About GRaCEmo ViRa UARCC")
        act_about.triggered.connect(lambda: self.set_notification("GRaCEmo ViRa Unified Autonomous Robotics Control Center V1.0 (ROS 2 Jazzy)"))

    # =========================================================================
    # 2. COMPACT ROBOTICS TOOLBAR
    # =========================================================================
    def _create_robotics_toolbar(self):
        self.toolbar = QToolBar("Robotics Tools", self)
        self.toolbar.setObjectName("rvizToolbar")
        self.toolbar.setMovable(False)
        self.toolbar.setFloatable(False)
        self.toolbar.setStyleSheet(f"""
            QToolBar {{
                background-color: {COLOR_WORKSPACE};
                border-bottom: 1px solid {COLOR_BORDER};
                padding: 3px 8px;
                spacing: 4px;
            }}
            QToolButton {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 4px 8px;
                font-size: 11px;
                font-weight: 500;
                color: {COLOR_TEXT_PRIMARY};
            }}
            QToolButton:hover {{
                background-color: {COLOR_SURFACE_ALT};
                border-color: {COLOR_BORDER_LIGHT};
            }}
            QToolButton:checked {{
                background-color: #E2E8F0;
                border-color: #94A3B8;
                font-weight: 700;
                color: {COLOR_TEXT_PRIMARY};
            }}
        """)
        self.addToolBar(self.toolbar)

        # Unit test compatibility attributes
        self.btn_tool_interact = QToolButton(self)
        self.btn_tool_interact.setCheckable(True)
        self.btn_tool_interact.setChecked(True)
        self.btn_tool_move = QToolButton(self)
        self.btn_tool_move.setCheckable(True)
        self.btn_tool_select = QToolButton(self)
        self.btn_tool_select.setCheckable(True)
        self.btn_tool_pose = QToolButton(self)
        self.btn_tool_pose.setCheckable(True)
        self.btn_tool_goal = QToolButton(self)
        self.btn_tool_goal.setCheckable(True)

        # Application-level controls (Canonical Owner of 3D tools is native RViz2)
        self.btn_tb_sidebar = self._add_toolbar_action_btn("☰ Sidebar", self._toggle_sidebar)
        self.btn_tb_docks = self._add_toolbar_action_btn("⧉ Panels", self._toggle_outer_docks)
        self.btn_tb_reembed = self._add_toolbar_action_btn("⟳ Re-embed RViz", lambda: self.embedded_rviz.attach_existing_or_launch())
        self.btn_tb_restart = self._add_toolbar_action_btn("▶ Restart RViz2", lambda: self.embedded_rviz.restart_rviz_process())
        self.btn_tb_full = self._add_toolbar_action_btn("⛶ Full Workspace", self._toggle_full_workspace)

        self.toolbar.addSeparator()

        # Fixed Frame Selector
        lbl_ff = QLabel("Fixed Frame:", self.toolbar)
        lbl_ff.setStyleSheet(f"font-size: 11px; color: {COLOR_TEXT_SECONDARY}; margin-left: 4px;")
        self.toolbar.addWidget(lbl_ff)

        self.combo_fixed_frame = QComboBox(self.toolbar)
        self.combo_fixed_frame.addItems(["map", "odom", "base_link", "base_footprint"])
        self.combo_fixed_frame.setStyleSheet(f"""
            QComboBox {{
                font-family: {FONT_FAMILY_MONO};
                font-size: 11px;
                padding: 2px 8px;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                background-color: #FFFFFF;
                min-width: 90px;
            }}
        """)
        self.toolbar.addWidget(self.combo_fixed_frame)

        self.toolbar.addSeparator()

        # Telemetry Badges
        self.top_ind_ros = StatusIndicator("ROS 2", StatusIndicator.STATE_NOT_CONNECTED, "Diagnostics")
        self.top_ind_gz = StatusIndicator("Gazebo", StatusIndicator.STATE_NOT_CONNECTED, "Simulation")
        self.top_ind_loc = StatusIndicator("Localization", StatusIndicator.STATE_NOT_CONNECTED, "Localization & Kinematics")
        self.top_ind_estop = StatusIndicator("E-Stop", StatusIndicator.STATE_READY, "Safety")

        for ind in (self.top_ind_ros, self.top_ind_gz, self.top_ind_loc, self.top_ind_estop):
            ind.clicked.connect(self.switch_to_page)
            self.toolbar.addWidget(ind)

        # Simulation Time Clock Badge
        self.lbl_top_clock = QLabel("Sim Time: 0.000 s", self.toolbar)
        self.lbl_top_clock.setStyleSheet(f"""
            font-family: {FONT_FAMILY_MONO};
            font-size: 11px;
            font-weight: 600;
            color: {COLOR_TEXT_PRIMARY};
            background-color: {COLOR_SUBTLE_SURFACE};
            padding: 4px 10px;
            border-radius: {BORDER_RADIUS_SM}px;
            border: 1px solid {COLOR_BORDER};
            margin-left: 4px;
        """)
        self.toolbar.addWidget(self.lbl_top_clock)

        # Connect toolbar signals
        bus.ros_connected.connect(lambda ok: self.top_ind_ros.set_state(StatusIndicator.STATE_READY if ok else StatusIndicator.STATE_NOT_CONNECTED))
        bus.gazebo_connected.connect(lambda ok: self.top_ind_gz.set_state(StatusIndicator.STATE_READY if ok else StatusIndicator.STATE_NOT_CONNECTED))
        bus.amcl_pose_ready.connect(lambda *_: self.top_ind_loc.set_state(StatusIndicator.STATE_READY))
        bus.sim_clock_updated.connect(lambda t: self.lbl_top_clock.setText(f"Sim Time: {t:.3f} s"))
        bus.emergency_stop_state.connect(self._on_estop_state_top)

    def _add_toolbar_tool_btn(self, label: str, mode: str, checked: bool = False) -> QToolButton:
        btn = QToolButton(self.toolbar)
        btn.setText(label)
        btn.setCheckable(True)
        btn.setChecked(checked)
        btn.clicked.connect(lambda: self._set_active_tool(mode))
        self.tool_btn_group.addButton(btn)
        self.toolbar.addWidget(btn)
        return btn

    def _add_toolbar_action_btn(self, label: str, callback) -> QToolButton:
        btn = QToolButton(self.toolbar)
        btn.setText(label)
        btn.clicked.connect(callback)
        self.toolbar.addWidget(btn)
        return btn

    def _set_active_tool(self, mode: str):
        self.viewport_3d.set_tool_mode(mode)
        tool_btn_map = {
            "interact": getattr(self, "btn_tool_interact", None),
            "move_camera": getattr(self, "btn_tool_move", None),
            "select": getattr(self, "btn_tool_select", None),
            "pose_estimate": getattr(self, "btn_tool_pose", None),
            "nav_goal": getattr(self, "btn_tool_goal", None),
        }
        for m, btn in tool_btn_map.items():
            if btn is not None:
                btn.setChecked(m == mode)
        self.set_notification(f"Active 3D Tool: {mode.replace('_', ' ').title()}")

    # =========================================================================
    # 3. CENTRAL ENGINEERING WORKSPACE (Canonical Sidebar + 11 Module Views)
    # =========================================================================
    def _create_central_workspace(self):
        central_container = QWidget(self)
        self.setCentralWidget(central_container)

        central_layout = QHBoxLayout(central_container)
        central_layout.setContentsMargins(0, 0, 0, 0)
        central_layout.setSpacing(0)

        # 1. Sidebar Navigation (Canonical 3 Groups: WORKSPACE, ROBOT SYSTEMS, ENGINEERING)
        self.sidebar = self._create_sidebar(central_container)
        central_layout.addWidget(self.sidebar)

        # 2. Main Stacked Container for 11 Canonical Views
        self.stack = QStackedWidget(central_container)
        self.stack.setStyleSheet(f"QStackedWidget {{ background-color: {COLOR_BG_MAIN}; border: none; }}")

        # Instantiate 11 Canonical Modules
        self._instantiate_module_views()

        central_layout.addWidget(self.stack, stretch=1)

    def _create_sidebar(self, parent: QWidget) -> QFrame:
        sidebar = QFrame(parent)
        sidebar.setObjectName("sidebarPanel")
        sidebar.setFixedWidth(230)
        self.sidebar_is_collapsed = False
        self.sidebar_full_width = 230
        self.sidebar_collapsed_width = 54

        layout = QVBoxLayout(sidebar)
        layout.setContentsMargins(0, 10, 0, 10)
        layout.setSpacing(0)

        # Brand Header & Collapse Toggle
        brand_bar = QHBoxLayout()
        brand_bar.setContentsMargins(14, 0, 8, 12)
        brand_bar.setSpacing(6)

        brand_text_box = QVBoxLayout()
        brand_text_box.setSpacing(2)

        self.lbl_brand = QLabel("GRaCEmo ViRa", sidebar)
        self.lbl_brand.setProperty("class", "sidebarBrandTitle")
        brand_text_box.addWidget(self.lbl_brand)

        self.lbl_sub = QLabel("CONTROL CENTER", sidebar)
        self.lbl_sub.setProperty("class", "sidebarBrandSubtitle")
        brand_text_box.addWidget(self.lbl_sub)

        brand_bar.addLayout(brand_text_box)
        brand_bar.addStretch()

        self.btn_collapse = QPushButton("◀", sidebar)
        self.btn_collapse.setFixedSize(24, 24)
        self.btn_collapse.setToolTip("Collapse/Expand Sidebar")
        self.btn_collapse.setStyleSheet(f"""
            QPushButton {{
                background-color: transparent;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                color: {COLOR_TEXT_SECONDARY};
                font-size: 11px;
                font-weight: 700;
            }}
            QPushButton:hover {{
                background-color: #E2E8F0;
                color: {COLOR_TEXT_PRIMARY};
            }}
        """)
        self.btn_collapse.clicked.connect(self._toggle_sidebar)
        brand_bar.addWidget(self.btn_collapse)

        layout.addLayout(brand_bar)

        # Navigation Button Group
        self.btn_group = QButtonGroup(self)
        self.btn_group.setExclusive(True)
        self.nav_buttons = {}
        self.category_labels = []

        self.nav_button_icons = {
            "Overview": "⬚",
            "Autonomous Operations": "⚡",
            "Simulation": "🌐",
            "Navigation": "🧭",
            "Sensors & Perception": "👁",
            "Localization & Kinematics": "📍",
            "Cognitive System": "🧠",
            "Safety": "🛡",
            "Diagnostics": "🩺",
            "System Logs": "📋",
            "Settings": "⚙",
        }

        self.nav_button_full_text = {
            "Overview": "Overview",
            "Autonomous Operations": "Autonomous Operations",
            "Simulation": "Simulation",
            "Navigation": "Navigation",
            "Sensors & Perception": "Sensors & Perception",
            "Localization & Kinematics": "Localization & Kinematics",
            "Cognitive System": "Cognitive System",
            "Safety": "Safety",
            "Diagnostics": "Diagnostics",
            "System Logs": "System Logs",
            "Settings": "Settings",
        }

        # 3 Structured Categories: WORKSPACE, ROBOT SYSTEMS, ENGINEERING
        for cat_name, items in self.NAVIGATION_GROUPS:
            cat_lbl = QLabel(cat_name, sidebar)
            cat_lbl.setProperty("class", "sidebarCategoryHeader")
            self.category_labels.append(cat_lbl)
            layout.addWidget(cat_lbl)

            for label, mod_name in items:
                icon_str = self.nav_button_icons.get(mod_name, "•")
                btn = QPushButton(f" {icon_str}  {label}", sidebar)
                btn.setProperty("class", "navButton")
                btn.setCheckable(True)
                btn.setToolTip(f"{label}: {self.MODULE_DESCRIPTIONS.get(mod_name, '')}")
                btn.clicked.connect(lambda checked, name=mod_name: self.switch_to_page(name))
                self.btn_group.addButton(btn)
                self.nav_buttons[mod_name] = btn
                layout.addWidget(btn)

            layout.addSpacing(6)

        layout.addStretch()

        # Hardware Safety Lock Indicator in Footer
        self.hw_box = QFrame(sidebar)
        self.hw_box.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_SUBTLE_SURFACE};
                border-top: 1px solid {COLOR_BORDER};
                padding: 10px 14px;
            }}
        """)
        hw_layout = QVBoxLayout(self.hw_box)
        hw_layout.setContentsMargins(0, 0, 0, 0)
        hw_layout.setSpacing(2)

        hw_lbl1 = QLabel("🔒 HARDWARE LOCKED", self.hw_box)
        hw_lbl1.setStyleSheet(f"font-size: 10px; font-weight: 700; color: {COLOR_BRAND_ORANGE}; letter-spacing: 0.5px;")
        hw_layout.addWidget(hw_lbl1)

        hw_lbl2 = QLabel("Physical Actuation Disabled", self.hw_box)
        hw_lbl2.setStyleSheet(f"font-size: 10px; color: {COLOR_TEXT_SIDEBAR_MUTED};")
        hw_layout.addWidget(hw_lbl2)

        layout.addWidget(self.hw_box)
        return sidebar

    def _toggle_sidebar(self):
        if self.sidebar_is_collapsed:
            self.sidebar.setFixedWidth(self.sidebar_full_width)
            self.btn_collapse.setText("◀")
            self.lbl_brand.show()
            self.lbl_sub.show()
            self.hw_box.show()
            for cat_lbl in self.category_labels:
                cat_lbl.show()
            for mod_name, btn in self.nav_buttons.items():
                icon_str = self.nav_button_icons.get(mod_name, "•")
                label = self.nav_button_full_text.get(mod_name, mod_name)
                btn.setText(f" {icon_str}  {label}")
            self.sidebar_is_collapsed = False
        else:
            self.sidebar.setFixedWidth(self.sidebar_collapsed_width)
            self.btn_collapse.setText("▶")
            self.lbl_brand.hide()
            self.lbl_sub.hide()
            self.hw_box.hide()
            for cat_lbl in self.category_labels:
                cat_lbl.hide()
            for mod_name, btn in self.nav_buttons.items():
                icon_str = self.nav_button_icons.get(mod_name, "•")
                btn.setText(icon_str)
            self.sidebar_is_collapsed = True

    def _instantiate_module_views(self):
        """Register the 11 canonical engineering modules."""
        # Index 0: Overview
        self.view_overview = OverviewView(self)
        self.view_overview.request_page.connect(self.switch_to_page)
        self.stack.addWidget(self.view_overview)

        # Index 1: Autonomous Operations
        self.view_autonomy = AutonomyView(self.settings, self)
        self.stack.addWidget(self.view_autonomy)

        # Index 2: Simulation
        self.view_simulation = SimulationView(self)
        self.stack.addWidget(self.view_simulation)

        # Index 3: Navigation (hosts Embedded RViz2 Primary 3D Workspace)
        self.view_navigation = NavigationView(self.settings, self)
        self.stack.addWidget(self.view_navigation)

        # Index 4: Sensors & Perception
        self.view_sensors = SensorsView(self)
        self.stack.addWidget(self.view_sensors)

        # Index 5: Localization & Kinematics
        self.view_localization = LocalizationView(self)
        self.stack.addWidget(self.view_localization)

        # Index 6: Cognitive System (Includes Voice Pipeline)
        self.view_cognitive = CognitiveView(self)
        self.stack.addWidget(self.view_cognitive)

        # Index 7: Safety
        self.view_safety = SafetyView(self)
        self.stack.addWidget(self.view_safety)

        # Index 8: Diagnostics
        self.view_diagnostics = DiagnosticsView(self)
        self.stack.addWidget(self.view_diagnostics)

        # Index 9: System Logs
        self.view_logs = LogsView(self)
        self.stack.addWidget(self.view_logs)

        # Index 10: Settings
        self.view_settings = SettingsView(self.settings, self)
        self.stack.addWidget(self.view_settings)

        # Retain references for native RViz tools and comprehensive test suite
        self.embedded_rviz = self.view_navigation.embedded_rviz
        self.viewport_3d = self.embedded_rviz.native_viewport
        self.view_interaction = InteractionView(self)

    # =========================================================================
    # 4. LEFT DOCK: DISPLAYS TREE (Collapsible Inspection Dock)
    # =========================================================================
    def _create_left_dock(self):
        self.dock_displays = QDockWidget("Displays", self)
        self.dock_displays.setObjectName("dockDisplays")
        self.dock_displays.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        self.dock_displays.setMinimumWidth(240)

        self.displays_tree = DisplaysTreeWidget(self.dock_displays)
        self.dock_displays.setWidget(self.displays_tree)
        self.addDockWidget(Qt.LeftDockWidgetArea, self.dock_displays)

        # Hidden by default to avoid duplicating embedded RViz2's own displays panel
        self.dock_displays.hide()

        # Add toggle action to Panels menu
        self.menu_panels.addAction(self.dock_displays.toggleViewAction())

    # =========================================================================
    # 5. RIGHT DOCK: VIEWS & PROPERTIES INSPECTOR (Collapsible Inspection Dock)
    # =========================================================================
    def _create_right_dock(self):
        self.dock_views = QDockWidget("Views & Properties", self)
        self.dock_views.setObjectName("dockViews")
        self.dock_views.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        self.dock_views.setMinimumWidth(250)

        self.views_inspector = ViewsInspectorWidget(self.dock_views)
        self.dock_views.setWidget(self.views_inspector)
        self.addDockWidget(Qt.RightDockWidgetArea, self.dock_views)

        # Hidden by default to avoid duplicating embedded RViz2's own properties panel
        self.dock_views.hide()

        # Add toggle action to Panels menu
        self.menu_panels.addAction(self.dock_views.toggleViewAction())
        self.menu_panels.addSeparator()
        self.menu_panels.addAction(self.toolbar.toggleViewAction())

    # =========================================================================
    # 6. BOTTOM ENGINEERING STATUS BAR
    # =========================================================================
    def _create_bottom_status_bar(self):
        self.status_bar_frame = QFrame(self)
        self.status_bar_frame.setObjectName("bottomNotificationBar")
        self.status_bar_frame.setStyleSheet(f"""
            QFrame#bottomNotificationBar {{
                background-color: {COLOR_WORKSPACE};
                border-top: 1px solid {COLOR_BORDER};
                padding: 4px 12px;
            }}
        """)
        layout = QHBoxLayout(self.status_bar_frame)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        self.lbl_bottom_status = QLabel("System Status: ROS 2 Jazzy workspace ready. Telemetry online.", self.status_bar_frame)
        self.lbl_bottom_status.setStyleSheet(f"font-size: 11px; color: {COLOR_TEXT_PRIMARY}; font-weight: 500;")
        layout.addWidget(self.lbl_bottom_status)

        layout.addStretch()

        self.lbl_eng_telemetry = QLabel("Domain: 0  |  Fixed Frame: map  |  TF: OK  |  Hardware: 🔒 LOCKED", self.status_bar_frame)
        self.lbl_eng_telemetry.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_SECONDARY};")
        layout.addWidget(self.lbl_eng_telemetry)

        # Add as bottom status widget
        status_bar = QStatusBar(self)
        status_bar.setSizeGripEnabled(False)
        status_bar.addWidget(self.status_bar_frame, 1)
        self.setStatusBar(status_bar)

        # Connect spoken & coordinator feedback
        bus.spoken_feedback_received.connect(lambda text, st: self.set_notification(f"GRaCEmo: {text}"))
        bus.coordinator_feedback_ready.connect(lambda st, speech, *_: self.set_notification(f"Coordinator [{st}]: {speech}"))

    # =========================================================================
    # WIRING & INTERACTION LOGIC
    # =========================================================================
    def _wire_rviz_signals(self):
        # Displays Tree -> Viewport Layer Toggles
        self.displays_tree.display_toggled.connect(self.viewport_3d.set_display_visible)

        # Displays Tree Selection -> Inspector Properties Table
        self.displays_tree.display_selected.connect(self.views_inspector.set_item_properties)

        # Inspector Camera Controls -> Viewport Camera Orbit
        self.views_inspector.camera_changed.connect(
            lambda dist, pitch, yaw: self.viewport_3d.set_camera_orbit(distance=dist, pitch=pitch, yaw=yaw)
        )
        self.views_inspector.preset_requested.connect(self.viewport_3d.set_view_preset)

        # Viewport View Change -> Inspector Controls Sync
        self.viewport_3d.view_updated.connect(self.views_inspector.update_camera_values)

        # Viewport Nav Goal / Pose Estimate Signals
        self.viewport_3d.goal_placed.connect(self._on_viewport_goal_placed)
        self.viewport_3d.pose_estimated.connect(self._on_viewport_pose_estimated)

        # Embedded RViz Signals
        self.embedded_rviz.toggle_docks_requested.connect(self._toggle_outer_docks)
        self.embedded_rviz.rviz_status_changed.connect(
            lambda txt, ok: self.set_notification(f"RViz2 Workspace: {txt}")
        )

    def _toggle_outer_docks(self):
        show = not self.dock_displays.isVisible()
        self.dock_displays.setVisible(show)
        self.dock_views.setVisible(show)

    def _toggle_full_workspace(self):
        if not self.sidebar_is_collapsed:
            self._toggle_sidebar()
        self.dock_displays.setVisible(False)
        self.dock_views.setVisible(False)
        self.set_notification("Full Visualization Workspace active.")

    def _on_viewport_goal_placed(self, x: float, y: float, yaw: float):
        self.set_notification(f"2D Nav Goal placed at ({x:.2f}, {y:.2f}) yaw={yaw:.2f} rad")
        self._set_active_tool("interact")

    def _on_viewport_pose_estimated(self, x: float, y: float, yaw: float):
        self.set_notification(f"2D Pose Estimate placed at ({x:.2f}, {y:.2f}) yaw={yaw:.2f} rad")
        self._set_active_tool("interact")

    def _on_reset_all_defaults(self):
        self.viewport_3d.reset_view()
        self.displays_tree._reset_defaults()
        self.set_notification("Visualization and camera parameters reset to engineering defaults.")

    def set_notification(self, text: str):
        self.lbl_bottom_status.setText(text)

    def switch_to_page(self, page_name: str):
        """Programmatic page switching via signal, menu, or sidebar button."""
        canonical_map = {
            # Workspace
            "overview": (0, "Overview"),
            "autonomy": (1, "Autonomous Operations"),
            "autonomous": (1, "Autonomous Operations"),
            "autonomous operations": (1, "Autonomous Operations"),
            "simulation": (2, "Simulation"),
            "sim": (2, "Simulation"),
            "gazebo": (2, "Simulation"),
            "navigation": (3, "Navigation"),
            "nav": (3, "Navigation"),
            "3d": (3, "Navigation"),
            "3d visualization": (3, "Navigation"),
            "viewport": (3, "Navigation"),
            "rviz": (3, "Navigation"),

            # Robot Systems
            "sensors": (4, "Sensors & Perception"),
            "sensors & perception": (4, "Sensors & Perception"),
            "perception": (4, "Sensors & Perception"),
            "localization": (5, "Localization & Kinematics"),
            "localization & kinematics": (5, "Localization & Kinematics"),
            "kinematics": (5, "Localization & Kinematics"),
            "amcl": (5, "Localization & Kinematics"),
            "cognitive": (6, "Cognitive System"),
            "cognitive system": (6, "Cognitive System"),
            "voice": (6, "Cognitive System"),
            "voice & interaction": (6, "Cognitive System"),
            "interaction": (6, "Cognitive System"),
            "mnse": (6, "Cognitive System"),

            # Engineering
            "safety": (7, "Safety"),
            "e-stop": (7, "Safety"),
            "estop": (7, "Safety"),
            "diagnostics": (8, "Diagnostics"),
            "diag": (8, "Diagnostics"),
            "logs": (9, "System Logs"),
            "system logs": (9, "System Logs"),
            "system_logs": (9, "System Logs"),
            "settings": (10, "Settings"),
            "preferences": (10, "Settings"),
        }

        key = page_name.lower().strip()
        target = canonical_map.get(key)
        if not target:
            # Fuzzy match
            for k, val in canonical_map.items():
                if k in key or key in k:
                    target = val
                    break

        if target:
            idx, canonical_name = target

            # Explicitly toggle embedded foreign window to prevent native X11 overlay bleed
            if hasattr(self, "embedded_rviz"):
                if idx == 3:
                    if self.embedded_rviz._foreign_window:
                        try:
                            self.embedded_rviz._foreign_window.setVisible(True)
                        except Exception:
                            pass
                    if self.embedded_rviz._window_container:
                        self.embedded_rviz._window_container.show()
                    else:
                        self.embedded_rviz.attach_existing_or_launch()
                    # Hide outer docks in RViz mode to give RViz maximum space and eliminate double-dock confusion
                    if hasattr(self, "dock_displays") and hasattr(self, "dock_views"):
                        self.dock_displays.hide()
                        self.dock_views.hide()
                else:
                    if self.embedded_rviz._foreign_window:
                        try:
                            self.embedded_rviz._foreign_window.setVisible(False)
                        except Exception:
                            pass
                    if self.embedded_rviz._window_container:
                        self.embedded_rviz._window_container.hide()

            self.stack.setCurrentIndex(idx)
            btn = self.nav_buttons.get(canonical_name)
            if btn:
                btn.setChecked(True)
            desc = self.MODULE_DESCRIPTIONS.get(canonical_name, "")
            self.set_notification(f"{canonical_name}: {desc}")

    def _on_estop_state_top(self, is_active: bool, source: str):
        if is_active:
            self.top_ind_estop.set_state(StatusIndicator.STATE_ERROR)
            self.set_notification(f"🚨 EMERGENCY STOP ASSERTED by {source}!")
        else:
            self.top_ind_estop.set_state(StatusIndicator.STATE_READY)
            self.set_notification("Emergency Stop cleared. System operational.")
