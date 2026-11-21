"""
Module 2: Gazebo Simulation View for GRaCEmo ViRa UARCC.
Professional simulation monitoring interface separating Overview, Physics Health,
Process Controls, and a dedicated Diagnostic Terminal Console.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QFrame, QPushButton, QTextEdit, QScrollArea, QApplication
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QColor

import subprocess
import os
import time
from datetime import datetime

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
    COLOR_TEXT_PRIMARY,
    COLOR_TEXT_SECONDARY,
    COLOR_TEXT_MUTED,
    FONT_FAMILY_PRIMARY,
    FONT_FAMILY_MONO,
    BORDER_RADIUS_SM,
    BORDER_RADIUS_MD,
)
from ...widgets.status_indicator import StatusIndicator
from ...widgets.common_components import PageHeader, SectionHeader, MetricDisplay, AlertBanner
from ...core.event_bus import bus


class SimulationView(QWidget):
    """Module 2: Gazebo Harmonic simulation monitoring & GUI controls."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self.sim_time = 0.0
        self.last_clock_receipt = 0.0
        self.gz_running = False

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("background: transparent;")

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(14)

        # 1. Page Header
        self.header = PageHeader(
            title="Simulation Management",
            subtitle="Gazebo Harmonic physics clock synchronization, bridge health, and external viewer controls",
            badge_title="Simulation Status",
            initial_state=StatusIndicator.STATE_NOT_CONNECTED,
            target_page="Simulation",
            parent=self
        )
        self.ind_gz_status = self.header.badge
        layout.addWidget(self.header)

        # 2. Section 1: Simulation Overview Key Telemetry Metrics
        metric_row = QHBoxLayout()
        metric_row.setSpacing(12)

        self.metric_clock = MetricDisplay("Sim Clock", "0.000 s", status_color=COLOR_ACCENT_PRIMARY, parent=self)
        self.metric_rtf = MetricDisplay("Real-Time Factor", "1.00x", status_color=COLOR_SUCCESS, parent=self)
        self.metric_bridge = MetricDisplay("Bridge Telemetry", "DISCONNECTED", status_color=COLOR_WARNING, parent=self)
        self.metric_world = MetricDisplay("Active World", "Apartment", status_color=COLOR_TEXT_PRIMARY, parent=self)

        metric_row.addWidget(self.metric_clock)
        metric_row.addWidget(self.metric_rtf)
        metric_row.addWidget(self.metric_bridge)
        metric_row.addWidget(self.metric_world)
        layout.addLayout(metric_row)

        # 3. Section 2: Environment & Physics Details Card
        clock_card = QFrame(self)
        clock_card.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_MD}px;
            }}
        """)
        c_layout = QVBoxLayout(clock_card)
        c_layout.setContentsMargins(14, 12, 14, 12)
        c_layout.setSpacing(10)

        c_hdr = SectionHeader("Physics Engine & Bridge Telemetry", parent=self)
        c_layout.addWidget(c_hdr)

        grid = QGridLayout()
        grid.setSpacing(8)

        self.lbl_clock = QLabel("Sim Time: 0.000 s", self)
        self.lbl_rtf = QLabel("Real-Time Factor: 1.00x (Physics target 1000 Hz)", self)
        self.lbl_bridge = QLabel("Bridge: NOT CONNECTED", self)
        self.lbl_model = QLabel("Model Entity: gracemo_vira", self)
        self.lbl_world = QLabel("World: apartment_floor.world", self)
        self.lbl_partition = QLabel("GZ_PARTITION: gracemo  |  GZ_IP: 127.0.0.1", self)

        for lbl in (self.lbl_clock, self.lbl_rtf, self.lbl_bridge, self.lbl_model, self.lbl_world, self.lbl_partition):
            lbl.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_PRIMARY};")

        grid.addWidget(self.lbl_clock, 0, 0)
        grid.addWidget(self.lbl_rtf, 0, 1)
        grid.addWidget(self.lbl_bridge, 1, 0)
        grid.addWidget(self.lbl_model, 1, 1)
        grid.addWidget(self.lbl_world, 2, 0)
        grid.addWidget(self.lbl_partition, 2, 1)
        c_layout.addLayout(grid)
        layout.addWidget(clock_card)

        # 4. Section 3: Gazebo Controls & Process Health
        ctrl_card = QFrame(self)
        ctrl_card.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_MD}px;
            }}
        """)
        ctrl_layout = QVBoxLayout(ctrl_card)
        ctrl_layout.setContentsMargins(14, 12, 14, 12)
        ctrl_layout.setSpacing(10)

        ctrl_hdr = SectionHeader("Gazebo 3D Visualization & Process Controls", parent=self)
        ctrl_layout.addWidget(ctrl_hdr)

        note_lbl = QLabel(
            "UARCC connects cleanly to the existing headless Gazebo server partition without duplicate process spawn.\n"
            "Use the controls below to attach the 3D visualization client or poll the OS process table.",
            self
        )
        note_lbl.setStyleSheet(f"color: {COLOR_TEXT_SECONDARY}; font-size: 11px; line-height: 1.4;")
        ctrl_layout.addWidget(note_lbl)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)

        self.btn_launch_gz_gui = QPushButton("Attach Gazebo 3D Viewer (gz sim -g)", self)
        self.btn_launch_gz_gui.setProperty("class", "primaryButton")
        self.btn_launch_gz_gui.setFixedHeight(34)
        self.btn_launch_gz_gui.clicked.connect(self._launch_gz_gui)
        btn_row.addWidget(self.btn_launch_gz_gui)

        self.btn_check_processes = QPushButton("Poll Simulation Processes", self)
        self.btn_check_processes.setProperty("class", "secondaryButton")
        self.btn_check_processes.setFixedHeight(34)
        self.btn_check_processes.clicked.connect(self._inspect_processes)
        btn_row.addWidget(self.btn_check_processes)

        btn_row.addStretch()
        ctrl_layout.addLayout(btn_row)
        layout.addWidget(ctrl_card)

        # 5. Section 4: Dedicated Terminal Console Card
        term_card = QFrame(self)
        term_card.setStyleSheet(f"""
            QFrame {{
                background-color: {COLOR_WORKSPACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_MD}px;
            }}
        """)
        term_layout = QVBoxLayout(term_card)
        term_layout.setContentsMargins(14, 12, 14, 12)
        term_layout.setSpacing(8)

        # Terminal Header with Actions
        th_row = QHBoxLayout()
        term_hdr = SectionHeader("Simulation Diagnostic Terminal Console", parent=self)
        th_row.addWidget(term_hdr)
        th_row.addStretch()

        self.btn_copy_log = QPushButton("Copy Console", self)
        self.btn_copy_log.setProperty("class", "secondaryButton")
        self.btn_copy_log.clicked.connect(self._copy_console)
        th_row.addWidget(self.btn_copy_log)

        self.btn_clear_log = QPushButton("Clear", self)
        self.btn_clear_log.setProperty("class", "secondaryButton")
        self.btn_clear_log.clicked.connect(lambda: self.txt_diag.clear())
        th_row.addWidget(self.btn_clear_log)

        term_layout.addLayout(th_row)

        self.txt_diag = QTextEdit(self)
        self.txt_diag.setReadOnly(True)
        self.txt_diag.setMinimumHeight(180)
        self.txt_diag.setStyleSheet(f"""
            QTextEdit {{
                background-color: {COLOR_SUBTLE_SURFACE};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                color: {COLOR_TEXT_PRIMARY};
                font-family: {FONT_FAMILY_MONO};
                font-size: 11px;
                padding: 10px;
                line-height: 1.4;
            }}
        """)
        self.txt_diag.setPlaceholderText("Simulation runtime diagnostics and process status events will stream here...")
        term_layout.addWidget(self.txt_diag)
        layout.addWidget(term_card)

        layout.addStretch()
        scroll.setWidget(container)

        page_layout = QVBoxLayout(self)
        page_layout.setContentsMargins(0, 0, 0, 0)
        page_layout.addWidget(scroll)

        # Connect bus signals
        bus.sim_clock_updated.connect(self._on_sim_clock)
        bus.gazebo_connected.connect(self._on_gazebo_status)

        # Periodic check for Gazebo process state
        self.proc_timer = QTimer(self)
        self.proc_timer.timeout.connect(self._periodic_check)
        self.proc_timer.start(2000)

        # Initial log entry
        self._append_log("INFO", "Simulation view initialized. Monitoring /clock and ros_gz_bridge telemetry.")

    def _append_log(self, severity: str, message: str):
        ts = datetime.now().strftime("%H:%M:%S")
        if severity == "ERROR":
            color = "#EF4444"
        elif severity == "WARN":
            color = "#F59E0B"
        elif severity == "SUCCESS":
            color = "#10B981"
        else:
            color = "#60A5FA"

        html_line = f"<span style='color: #64748B;'>[{ts}]</span> <strong style='color: {color};'>[{severity}]</strong> <span style='color: #E2E8F0;'>{message}</span><br>"
        self.txt_diag.insertHtml(html_line)
        self.txt_diag.ensureCursorVisible()

    def _copy_console(self):
        clipboard = QApplication.clipboard()
        clipboard.setText(self.txt_diag.toPlainText())
        self._append_log("INFO", "Console text copied to clipboard.")

    def _on_sim_clock(self, clock_sec: float):
        self.sim_time = clock_sec
        self.last_clock_receipt = time.time()
        self.lbl_clock.setText(f"Sim Time: {clock_sec:.3f} s")
        self.metric_clock.set_value(f"{clock_sec:.2f} s")
        self.lbl_bridge.setText("Bridge: CONNECTED & STREAMING")
        self.lbl_bridge.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_SUCCESS};")
        self.metric_bridge.set_value("CONNECTED", status_color=COLOR_SUCCESS)
        self.ind_gz_status.set_state(StatusIndicator.STATE_READY)

    def _on_gazebo_status(self, connected: bool):
        prev = self.gz_running
        self.gz_running = connected
        st = StatusIndicator.STATE_READY if connected else StatusIndicator.STATE_NOT_CONNECTED
        self.ind_gz_status.set_state(st)
        if prev != connected:
            if connected:
                self.metric_bridge.set_value("CONNECTED", status_color=COLOR_SUCCESS)
                self._append_log("SUCCESS", "Gazebo Harmonic physics clock active and synchronized.")
            else:
                self.metric_bridge.set_value("OFFLINE", status_color=COLOR_WARNING)
                self._append_log("WARN", "Gazebo connection dropped or server is paused.")

    def _launch_gz_gui(self):
        """Safely attach Gazebo GUI to existing server partition without creating duplicate instances."""
        try:
            chk = subprocess.run(["pgrep", "-f", "gz sim gui|ruby.*gz.*gui"], capture_output=True, text=True)
            if chk.returncode == 0:
                self._append_log("WARN", f"Gazebo GUI client is already running (PID: {chk.stdout.strip()})")
                return

            env = os.environ.copy()
            env["GZ_PARTITION"] = "gracemo"
            env["GZ_IP"] = "127.0.0.1"

            # Pass NVIDIA vendor library drivers if available
            if os.path.exists("/usr/share/glvnd/egl_vendor.d/10_nvidia.json"):
                env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
                env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
                env["__EGL_VENDOR_LIBRARY_FILENAMES"] = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"

            # Ensure complete Gazebo resource paths
            from pathlib import Path
            repo_root = Path(__file__).resolve().parent.parent.parent.parent
            ws_install = repo_root / "ros2_ws" / "install"
            fuel_cache = os.path.expanduser("~/.gz/fuel/fuel.gazebosim.org/openrobotics/models")
            fuel_ign = os.path.expanduser("~/.ignition/fuel/fuel.gazebosim.org/openrobotics/models")
            ros_share = "/opt/ros/jazzy/share"
            res_paths = [
                str(ws_install / "gracemo_gazebo" / "share" / "gracemo_gazebo" / "models"),
                str(ws_install / "gracemo_description" / "share"),
                fuel_cache,
                fuel_ign,
                ros_share,
            ]
            existing_res = env.get("GZ_SIM_RESOURCE_PATH", "")
            if existing_res:
                res_paths.append(existing_res)
            env["GZ_SIM_RESOURCE_PATH"] = ":".join(res_paths)

            subprocess.Popen(["gz", "sim", "-g"], env=env)
            self._append_log("SUCCESS", "Attached external Gazebo Harmonic GUI client to partition 'gracemo'.")
        except Exception as e:
            self._append_log("ERROR", f"Failed to launch Gazebo GUI: {e}")

    def _inspect_processes(self):
        """Query active Gazebo and ROS simulation processes."""
        try:
            res = subprocess.run(
                "ps aux | grep -E 'gz sim|ros_gz_bridge|prototype.launch.py|coordinator_node' | grep -v grep",
                shell=True, capture_output=True, text=True
            )
            output = res.stdout.strip()
            if output:
                self._append_log("INFO", f"Active Simulation Processes:\n{output}")
            else:
                self._append_log("WARN", "Zero simulation processes detected in OS process table.")
        except Exception as e:
            self._append_log("ERROR", f"Error querying processes: {e}")

    def _periodic_check(self):
        # Watchdog for stale clock (>2s without /clock tick)
        if self.last_clock_receipt > 0 and (time.time() - self.last_clock_receipt > 2.0):
            self.lbl_bridge.setText("Bridge: STALE CLOCK")
            self.lbl_bridge.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_WARNING};")
            self.metric_bridge.set_value("STALE", status_color=COLOR_WARNING)
            self.ind_gz_status.set_state(StatusIndicator.STATE_STALE)
