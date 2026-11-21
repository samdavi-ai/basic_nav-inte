"""
Integrated ROS 2 Jazzy RViz2 Embedded Visualization Workspace for UARCC.
Seamlessly reparents and embeds the authentic RViz2 Ogre 3D visualization window
into the PySide6 desktop engineering layout using QWindow.fromWinId() and createWindowContainer().
"""

import os
import sys
import re
import time
import subprocess
from typing import Optional, Tuple

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QFrame, QStackedWidget, QSizePolicy, QComboBox
)
from PySide6.QtCore import Qt, QTimer, Signal
from PySide6.QtGui import QWindow, QColor, QFont

from ..resources.styles.tokens import (
    COLOR_WORKSPACE,
    COLOR_BG_MAIN,
    COLOR_SURFACE,
    COLOR_SURFACE_ALT,
    COLOR_BORDER,
    COLOR_BORDER_LIGHT,
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
)
from .rviz_viewport import RVizViewportWidget


class EmbeddedRVizWidget(QWidget):
    """
    Central Workspace hosting the real, authentic ROS 2 RViz2 window embedded directly
    within UARCC, with seamless fallback to the native 3D viewport.
    """

    rviz_status_changed = Signal(str, bool)  # status_text, is_connected
    toggle_docks_requested = Signal()

    CONFIG_NORMAL = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/gracemo_nav2/share/gracemo_nav2/rviz/gracemo_normal.rviz"
    CONFIG_DEBUG = "/home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/gracemo_nav2/share/gracemo_nav2/rviz/gracemo_debug.rviz"
    CANONICAL_RVIZ_CONFIG = CONFIG_NORMAL

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("embeddedRVizWorkspace")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.current_rviz_config = self.CONFIG_NORMAL
        self._rviz_process: Optional[subprocess.Popen] = None
        self._rviz_win_id: Optional[int] = None
        self._foreign_window: Optional[QWindow] = None
        self._window_container: Optional[QWidget] = None
        self._is_headless = (os.environ.get("QT_QPA_PLATFORM") == "offscreen" or not os.environ.get("DISPLAY"))

        self._init_ui()

        # Connect delayed embed timer (allows parent layout to settle)
        if not self._is_headless:
            self._embed_timer = QTimer(self)
            self._embed_timer.setSingleShot(True)
            self._embed_timer.timeout.connect(self.attach_existing_or_launch)
            self._embed_timer.start(250)

    def showEvent(self, event):
        super().showEvent(event)
        if self._foreign_window:
            try:
                self._foreign_window.setVisible(True)
            except Exception:
                pass
        if self._window_container:
            self._window_container.show()
        elif not self._is_headless:
            QTimer.singleShot(100, self.attach_existing_or_launch)

    def hideEvent(self, event):
        super().hideEvent(event)
        if self._foreign_window:
            try:
                self._foreign_window.setVisible(False)
            except Exception:
                pass
        if self._window_container:
            self._window_container.hide()

    def _init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # 1. Compact Engineering Workspace Banner
        self.banner = self._create_banner()
        main_layout.addWidget(self.banner)

        # 2. Central Stack: Index 0 = RViz Container / Placeholder, Index 1 = Native Viewport
        self.view_stack = QStackedWidget(self)
        self.view_stack.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Page 0: RViz Host Container
        self.rviz_host_widget = QWidget(self.view_stack)
        self.rviz_host_widget.setAttribute(Qt.WA_NativeWindow, True)
        self.rviz_host_widget.winId()
        self.rviz_host_layout = QVBoxLayout(self.rviz_host_widget)
        self.rviz_host_layout.setContentsMargins(0, 0, 0, 0)
        self.rviz_host_layout.setSpacing(0)

        # Placeholder card shown when RViz is connecting or offline
        self.placeholder = self._create_placeholder()
        self.rviz_host_layout.addWidget(self.placeholder)
        self.view_stack.addWidget(self.rviz_host_widget)

        # Page 1: Native 3D Viewport (Available as alternate / companion view)
        self.native_viewport = RVizViewportWidget(self.view_stack)
        self.view_stack.addWidget(self.native_viewport)

        main_layout.addWidget(self.view_stack, stretch=1)

    def _create_banner(self) -> QFrame:
        banner = QFrame(self)
        banner.setObjectName("rvizWorkspaceBanner")
        banner.setStyleSheet(f"""
            QFrame#rvizWorkspaceBanner {{
                background-color: {COLOR_WORKSPACE};
                border-bottom: 1px solid {COLOR_BORDER};
                padding: 4px 10px;
            }}
        """)
        layout = QHBoxLayout(banner)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        # Status badge
        self.lbl_rviz_badge = QLabel("ROS 2 Jazzy RViz2: Initializing...", banner)
        self.lbl_rviz_badge.setStyleSheet(f"""
            font-size: 11px;
            font-weight: 700;
            color: {COLOR_TEXT_PRIMARY};
        """)
        layout.addWidget(self.lbl_rviz_badge)

        self.lbl_rviz_meta = QLabel("Config: gracemo_nav2.rviz  |  Node: rviz2_nav2", banner)
        self.lbl_rviz_meta.setStyleSheet(f"""
            font-family: {FONT_FAMILY_MONO};
            font-size: 10px;
            color: {COLOR_TEXT_SECONDARY};
        """)
        layout.addWidget(self.lbl_rviz_meta)

        layout.addStretch()

        # Profile Selector
        self.combo_profile = QComboBox(banner)
        self.combo_profile.setObjectName("comboRVizProfile")
        self.combo_profile.addItem("🎯 Normal Operation (Clean)", self.CONFIG_NORMAL)
        self.combo_profile.addItem("🛠️ Engineering Debug (TF Axes)", self.CONFIG_DEBUG)
        self.combo_profile.setStyleSheet(f"""
            QComboBox {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 2px 8px;
                font-size: 10px;
                font-weight: 600;
                color: {COLOR_TEXT_PRIMARY};
            }}
            QComboBox::drop-down {{
                border: none;
                width: 14px;
            }}
        """)
        self.combo_profile.currentIndexChanged.connect(self._on_profile_combo_changed)
        layout.addWidget(self.combo_profile)

        # Action Buttons
        btn_style = f"""
            QPushButton {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 3px 8px;
                font-size: 10px;
                font-weight: 600;
                color: {COLOR_TEXT_PRIMARY};
            }}
            QPushButton:hover {{
                background-color: {COLOR_SURFACE_ALT};
                border-color: {COLOR_BORDER_LIGHT};
            }}
        """

        self.btn_refresh = QPushButton("⟳ Re-embed RViz", banner)
        self.btn_refresh.setStyleSheet(btn_style)
        self.btn_refresh.clicked.connect(self.attach_existing_or_launch)
        layout.addWidget(self.btn_refresh)

        self.btn_restart = QPushButton("▶ Restart RViz2", banner)
        self.btn_restart.setStyleSheet(btn_style)
        self.btn_restart.clicked.connect(lambda: self.restart_rviz_process())
        layout.addWidget(self.btn_restart)

        self.btn_toggle_docks = QPushButton("Toggle Side Docks", banner)
        self.btn_toggle_docks.setStyleSheet(btn_style)
        self.btn_toggle_docks.clicked.connect(lambda: self.toggle_docks_requested.emit())
        layout.addWidget(self.btn_toggle_docks)

        self.btn_toggle_view = QPushButton("Toggle Native 3D Viewport", banner)
        self.btn_toggle_view.setStyleSheet(btn_style)
        self.btn_toggle_view.clicked.connect(self._toggle_view_mode)
        layout.addWidget(self.btn_toggle_view)

        return banner

    def _create_placeholder(self) -> QWidget:
        widget = QWidget(self)
        layout = QVBoxLayout(widget)
        layout.setAlignment(Qt.AlignCenter)
        layout.setSpacing(12)

        lbl_icon = QLabel("📡", widget)
        lbl_icon.setStyleSheet("font-size: 32px;")
        lbl_icon.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_icon)

        lbl_title = QLabel("ROS 2 RViz2 Visualization Workspace", widget)
        lbl_title.setStyleSheet(f"font-size: 14px; font-weight: 700; color: {COLOR_TEXT_PRIMARY};")
        lbl_title.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_title)

        lbl_sub = QLabel("Connecting to authentic RViz2 rendering context via X11 container...", widget)
        lbl_sub.setStyleSheet(f"font-size: 11px; color: {COLOR_TEXT_SECONDARY};")
        lbl_sub.setAlignment(Qt.AlignCenter)
        layout.addWidget(lbl_sub)

        btn_launch = QPushButton("Launch / Connect RViz2", widget)
        btn_launch.setStyleSheet(f"""
            QPushButton {{
                background-color: {COLOR_BRAND_ORANGE};
                color: #FFFFFF;
                font-weight: 600;
                font-size: 11px;
                padding: 6px 16px;
                border-radius: {BORDER_RADIUS_SM}px;
                border: none;
            }}
            QPushButton:hover {{
                background-color: #E05A00;
            }}
        """)
        btn_launch.clicked.connect(self.attach_existing_or_launch)
        layout.addWidget(btn_launch, alignment=Qt.AlignCenter)

        return widget

    # -------------------------------------------------------------------------
    # Core X11 Reparenting & Window Embedding Logic
    # -------------------------------------------------------------------------
    def attach_existing_or_launch(self):
        """Find existing RViz2 window or spawn a new instance and embed it."""
        if self._is_headless:
            self._set_status("Headless / Offscreen Mode (Simulated)", False)
            return

        wid = self._find_rviz_window_id()
        if wid:
            self._embed_window_id(wid)
        else:
            self._spawn_rviz_process()

    def _find_rviz_window_id(self) -> Optional[int]:
        """Query X11 window tree to find the running RViz2 client window."""
        try:
            out = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True, stderr=subprocess.DEVNULL)
            candidates = []
            for line in out.splitlines():
                # Extract Window ID at start of line
                m_id = re.search(r"^\s*(0x[0-9a-fA-F]+)", line)
                if not m_id:
                    continue
                wid = int(m_id.group(1), 16)

                line_lower = line.lower()
                # Exclude mutter frames or non-rviz
                if "mutter-x11-frames" in line_lower:
                    continue
                if not ("rviz2" in line_lower or "rviz" in line_lower):
                    continue

                # Parse exact geometry: e.g. 1000x700+0+0
                m_geo = re.search(r"\s+(\d+)x(\d+)\+[-0-9]+\+[-0-9]+", line)
                if not m_geo:
                    continue
                w, h = int(m_geo.group(1)), int(m_geo.group(2))

                # Must be a primary client window (not 1x1, 1x304 dock, or 3x3 helper)
                if w >= 200 and h >= 200:
                    candidates.append((wid, w * h, line))

            if candidates:
                # Pick the largest window matching rviz (the main workspace window)
                candidates.sort(key=lambda x: x[1], reverse=True)
                return candidates[0][0]
        except Exception:
            pass
        return None

    def _embed_window_id(self, win_id: int):
        """Strip decorations and reparent the foreign X11 window into our Qt container."""
        if self._window_container and self._rviz_win_id == win_id:
            # Already embedded
            self._set_status(f"● RViz2 Embedded (WinID: {hex(win_id)})", True)
            return

        try:
            # Clean up existing container if any
            if self._window_container:
                self.rviz_host_layout.removeWidget(self._window_container)
                self._window_container.deleteLater()
                self._window_container = None
                self._foreign_window = None

            # Explicit X11 decoration removal via _MOTIF_WM_HINTS
            try:
                import ctypes
                x11 = ctypes.cdll.LoadLibrary('libX11.so.6')
                disp = x11.XOpenDisplay(None)
                if disp:
                    atom_motif = x11.XInternAtom(disp, b"_MOTIF_WM_HINTS", False)
                    # struct MotifWmHints { flags, functions, decorations, input_mode, status }
                    # flags = 2 (MWM_HINTS_DECORATIONS), decorations = 0 (none)
                    hints = (ctypes.c_ulong * 5)(2, 0, 0, 0, 0)
                    x11.XChangeProperty(disp, win_id, atom_motif, atom_motif, 32, 0, ctypes.cast(hints, ctypes.c_char_p), 5)
                    x11.XFlush(disp)
                    x11.XCloseDisplay(disp)
            except Exception:
                pass

            # Create foreign QWindow and native Qt container
            self._foreign_window = QWindow.fromWinId(win_id)
            self._window_container = QWidget.createWindowContainer(self._foreign_window, self.rviz_host_widget)
            self._window_container.setAttribute(Qt.WA_NativeWindow, True)
            self._window_container.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

            # Hide placeholder and show container
            self.placeholder.hide()
            self.rviz_host_layout.addWidget(self._window_container, stretch=1)
            self.view_stack.setCurrentIndex(0)

            self._rviz_win_id = win_id
            self._set_status(f"● RViz2 Embedded (WinID: {hex(win_id)})", True)
        except Exception as e:
            self._set_status(f"Embed Error: {str(e)}", False)

    def _on_profile_combo_changed(self, index: int):
        target_config = self.combo_profile.itemData(index) or (self.CONFIG_NORMAL if index == 0 else self.CONFIG_DEBUG)
        self.switch_profile(target_config)

    def switch_profile(self, config_path: str):
        """Switch between RViz2 visualization profiles (Normal vs Debug)."""
        self.current_rviz_config = config_path
        is_debug = "debug" in config_path.lower()
        profile_name = "Engineering Debug (TF Axes)" if is_debug else "Normal Operation (Clean)"
        self.lbl_rviz_meta.setText(f"Profile: {profile_name}  |  Node: rviz2_nav2")
        
        # Sync combo index without re-triggering signal
        target_idx = 1 if is_debug else 0
        if self.combo_profile.currentIndex() != target_idx:
            self.combo_profile.blockSignals(True)
            self.combo_profile.setCurrentIndex(target_idx)
            self.combo_profile.blockSignals(False)

        self.restart_rviz_process(config_path=config_path)

    def _spawn_rviz_process(self, config_path: Optional[str] = None):
        """Spawn the canonical RViz2 process and wait for its window to appear."""
        config_to_use = config_path or getattr(self, "current_rviz_config", self.CANONICAL_RVIZ_CONFIG)
        self._set_status(f"Starting RViz2 ({os.path.basename(config_to_use)})...", False)

        env = os.environ.copy()
        env["DISPLAY"] = os.environ.get("DISPLAY", ":0")
        env["ROS_DOMAIN_ID"] = "0"
        env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
        env["GZ_PARTITION"] = "gracemo"

        cmd = [
            "bash", "-c",
            f"source /opt/ros/jazzy/setup.bash && "
            f"source /home/sabarishchandrasekaran/projects/VIRA-agent-bridge-migration/ros2_ws/install/setup.bash && "
            f"export DISPLAY={env['DISPLAY']} && export ROS_DOMAIN_ID=0 && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST && "
            f"rviz2 -d {config_to_use} --ros-args -r __node:=rviz2_nav2"
        ]

        try:
            self._rviz_process = subprocess.Popen(
                cmd, env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            # Poll for window appearance
            self._poll_attempts = 0
            self._poll_timer = QTimer(self)
            self._poll_timer.timeout.connect(self._poll_for_rviz_window)
            self._poll_timer.start(500)
        except Exception as e:
            self._set_status(f"Failed to launch RViz2: {e}", False)

    def _poll_for_rviz_window(self):
        self._poll_attempts += 1
        wid = self._find_rviz_window_id()
        if wid:
            self._poll_timer.stop()
            self._embed_window_id(wid)
        elif self._poll_attempts > 20:
            self._poll_timer.stop()
            self._set_status("RViz2 window detection timed out", False)

    def restart_rviz_process(self, config_path: Optional[str] = None):
        """Kill existing RViz2 instance and restart cleanly with specified or current profile."""
        if config_path:
            self.current_rviz_config = config_path
        try:
            subprocess.run(["pkill", "-f", "rviz2"], check=False)
        except Exception:
            pass
        self._rviz_win_id = None
        if self._window_container:
            self.rviz_host_layout.removeWidget(self._window_container)
            self._window_container.deleteLater()
            self._window_container = None
            self.placeholder.show()
        target = self.current_rviz_config
        QTimer.singleShot(1000, lambda: self._spawn_rviz_process(target))

    def _toggle_view_mode(self):
        cur = self.view_stack.currentIndex()
        new_idx = 1 if cur == 0 else 0
        self.view_stack.setCurrentIndex(new_idx)
        if new_idx == 1:
            self.btn_toggle_view.setText("Switch to Real RViz2")
            self._set_status("Native 3D Viewport Active", True)
        else:
            self.btn_toggle_view.setText("Switch to Native Viewport")
            if self._rviz_win_id:
                self._set_status(f"● RViz2 Embedded (WinID: {hex(self._rviz_win_id)})", True)
            else:
                self._set_status("RViz2 Host Active", False)

    def _set_status(self, text: str, ok: bool):
        color = COLOR_SUCCESS if ok else (COLOR_ERROR if "Error" in text or "timed out" in text else COLOR_TEXT_PRIMARY)
        self.lbl_rviz_badge.setText(text)
        self.lbl_rviz_badge.setStyleSheet(f"font-size: 11px; font-weight: 700; color: {color};")
        self.rviz_status_changed.emit(text, ok)

    # -------------------------------------------------------------------------
    # Delegated Methods for Compatibility
    # -------------------------------------------------------------------------
    def set_display_visible(self, name: str, visible: bool):
        self.native_viewport.set_display_visible(name, visible)

    def set_camera_orbit(self, distance=None, pitch=None, yaw=None, focal_point=None):
        self.native_viewport.set_camera_orbit(distance, pitch, yaw, focal_point)

    def set_view_preset(self, preset: str):
        self.native_viewport.set_view_preset(preset)

    def reset_view(self):
        self.native_viewport.reset_view()

    def fit_robot(self):
        self.native_viewport.fit_robot()

    def set_tool_mode(self, mode: str):
        self.native_viewport.set_tool_mode(mode)
