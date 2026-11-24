"""
Module 7: Cognitive Architecture & Voice Interaction Center for GRaCEmo ViRa UARCC.
Monitors the GRaCEmo Kernel daemon (MNSE), K Language engine,
Agent Bridge MCP task queue, approval gates, Whisper STT, and dialogue grounding.
Consolidates voice interaction into the Cognitive System without redundant sidebar pages.
"""

import os
import urllib.request
import json
import time
from datetime import datetime

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QFrame, QPushButton, QLineEdit, QTextEdit, QScrollArea, QTabWidget
)
from PySide6.QtCore import Qt, QTimer, QUrl
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkRequest, QNetworkReply
from PySide6.QtGui import QFont

from .visual_panel import VisualPanel
from ...resources.styles.tokens import (
    COLOR_BG_MAIN,
    COLOR_SURFACE,
    COLOR_PANEL_MAIN,
    COLOR_PANEL_SECONDARY,
    COLOR_BORDER,
    COLOR_BORDER_LIGHT,
    COLOR_BRAND_ORANGE,
    COLOR_ACCENT_PRIMARY,
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
from ...widgets.status_indicator import StatusIndicator
from ...widgets.common_components import PageHeader, SectionHeader, MetricDisplay
from ...core.event_bus import bus


class CognitiveView(QWidget):
    """Module 7: Unified Cognitive Architecture, Task Queue & Voice Dialogue Center."""

    def __init__(self, parent=None):
        super().__init__(parent)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setStyleSheet("background: transparent;")

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(16)

        # 1. Page Header
        self.header = PageHeader(
            title="Cognitive System & Voice Interaction",
            subtitle="MNSE neural-symbolic kernel, FastMCP task queue, Whisper STT pipeline, and dialogue grounding",
            badge_title="Cognitive Engine",
            initial_state=StatusIndicator.STATE_NOT_CONNECTED,
            target_page="Cognitive System",
            parent=self
        )
        self.ind_kernel = self.header.badge
        layout.addWidget(self.header)

        # 2. Key Metrics Row
        metric_row = QHBoxLayout()
        metric_row.setSpacing(12)

        self.metric_kernel = MetricDisplay("MNSE Daemon", "PROBING", status_color=COLOR_WARNING, parent=self)
        self.metric_bridge = MetricDisplay("Agent Bridge", "UNVERIFIED", status_color=COLOR_WARNING, parent=self)
        self.metric_voice = MetricDisplay("Voice Pipeline", "UNVERIFIED", status_color=COLOR_WARNING, parent=self)
        self.metric_gates = MetricDisplay("Approval Gates", "UNVERIFIED", status_color=COLOR_WARNING, parent=self)

        metric_row.addWidget(self.metric_kernel)
        metric_row.addWidget(self.metric_bridge)
        metric_row.addWidget(self.metric_voice)
        metric_row.addWidget(self.metric_gates)
        layout.addLayout(metric_row)

        # 3. Kernel Channels & Security Authorization Gates Card
        info_card = QFrame(self)
        info_card.setProperty("class", "cardPanel")
        ic_layout = QVBoxLayout(info_card)
        ic_layout.setSpacing(12)

        ic_hdr = SectionHeader("Communication Channels & Consequential Action Gates", parent=self)
        ic_layout.addWidget(ic_hdr)

        grid = QGridLayout()
        grid.setSpacing(10)

        self.lbl_rest = QLabel("REST Endpoint: http://127.0.0.1:7780 (PROBING...)")
        self.lbl_sock = QLabel("Domain Socket: /tmp/gracemo.sock (CHECKING...)")
        self.lbl_bridge = QLabel("Agent Bridge: queue status unavailable")
        self.lbl_approval = QLabel("Approval Gates: runtime status unavailable")

        for lbl in (self.lbl_rest, self.lbl_sock, self.lbl_bridge, self.lbl_approval):
            lbl.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_PRIMARY};")

        grid.addWidget(self.lbl_rest, 0, 0)
        grid.addWidget(self.lbl_sock, 0, 1)
        grid.addWidget(self.lbl_bridge, 1, 0)
        grid.addWidget(self.lbl_approval, 1, 1)
        ic_layout.addLayout(grid)

        # Probe Button Row
        btn_row = QHBoxLayout()
        btn_probe = QPushButton("Probe Kernel Health Now", self)
        btn_probe.setStyleSheet(f"""
            QPushButton {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 4px 12px;
                font-size: 11px;
                color: {COLOR_TEXT_PRIMARY};
            }}
            QPushButton:hover {{
                background-color: {COLOR_PANEL_SECONDARY};
            }}
        """)
        btn_probe.clicked.connect(self._probe_kernel_health)
        btn_row.addWidget(btn_probe)
        btn_row.addStretch()
        ic_layout.addLayout(btn_row)

        layout.addWidget(info_card)

        # 4. Integrated Voice Pipeline & Natural Language Grounding Card
        voice_card = QFrame(self)
        voice_card.setProperty("class", "cardPanel")
        vc_layout = QVBoxLayout(voice_card)
        vc_layout.setSpacing(10)

        vc_hdr = SectionHeader("Voice & Interaction Pipeline Stages", parent=self)
        vc_layout.addWidget(vc_hdr)

        pipeline_row = QHBoxLayout()
        pipeline_row.setSpacing(10)

        self.ind_stt = StatusIndicator("Whisper STT", StatusIndicator.STATE_NOT_CONNECTED)
        self.ind_intent = StatusIndicator("Intent Parser", StatusIndicator.STATE_NOT_CONNECTED)
        self.ind_grounding = StatusIndicator("Place Grounding", StatusIndicator.STATE_NOT_CONNECTED)
        self.ind_tts = StatusIndicator("Spoken TTS", StatusIndicator.STATE_NOT_CONNECTED)

        for ind in (self.ind_stt, self.ind_intent, self.ind_grounding, self.ind_tts):
            pipeline_row.addWidget(ind)
        pipeline_row.addStretch()
        vc_layout.addLayout(pipeline_row)

        layout.addWidget(voice_card)

        # 5. Dialogue History & Cognitive Task Stream Card
        dialogue_card = QFrame(self)
        dialogue_card.setProperty("class", "cardPanel")
        dc_layout = QVBoxLayout(dialogue_card)
        dc_layout.setSpacing(10)

        dc_hdr = SectionHeader("Dialogue History & Reasoning Event Stream", parent=self)
        dc_layout.addWidget(dc_hdr)

        self.txt_conversation = QTextEdit(self)
        self.txt_conversation.setReadOnly(True)
        self.txt_conversation.setMinimumHeight(150)
        self.txt_conversation.setMaximumHeight(220)
        self.txt_conversation.setStyleSheet(f"""
            QTextEdit {{
                background-color: {COLOR_PANEL_SECONDARY};
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                color: {COLOR_TEXT_PRIMARY};
                font-family: {FONT_FAMILY_MONO};
                font-size: 11px;
                padding: 8px;
            }}
        """)
        dc_layout.addWidget(self.txt_conversation)

        # Text Command Input Row
        input_row = QHBoxLayout()
        input_row.setSpacing(8)

        self.txt_input = QLineEdit(self)
        self.txt_input.setPlaceholderText("Enter natural language command (e.g. 'navigate to kitchen' or 'report status')...")
        self.txt_input.setStyleSheet(f"""
            QLineEdit {{
                background-color: #FFFFFF;
                border: 1px solid {COLOR_BORDER};
                border-radius: {BORDER_RADIUS_SM}px;
                padding: 6px 10px;
                font-size: 11px;
                color: {COLOR_TEXT_PRIMARY};
            }}
        """)
        self.txt_input.returnPressed.connect(self._send_text_command)
        input_row.addWidget(self.txt_input, stretch=1)

        self.btn_send = QPushButton("Dispatch Command", self)
        self.btn_send.setStyleSheet(f"""
            QPushButton {{
                background-color: {COLOR_BRAND_ORANGE};
                color: #FFFFFF;
                font-weight: 600;
                font-size: 11px;
                padding: 6px 14px;
                border-radius: {BORDER_RADIUS_SM}px;
                border: none;
            }}
            QPushButton:hover {{
                background-color: #E05A00;
            }}
        """)
        self.btn_send.clicked.connect(self._send_text_command)
        input_row.addWidget(self.btn_send)

        dc_layout.addLayout(input_row)
        layout.addWidget(dialogue_card)

        self.memory_tabs = QTabWidget(self)
        self.memory_network = QNetworkAccessManager(self)
        self.memory_paths = ('/snapshot','/context','/graph','/history?limit=20','/recovery/missions','/adapters','/perception')
        self.memory_pending = set()
        for title in ('Live state','Compiled context','Knowledge graph','Recorded events','Restart review','Adapter connections','Visual perception'):
            if title == "Visual perception":
                panel = VisualPanel(self.memory_tabs)
            else:
                panel = QTextEdit(self.memory_tabs)
                panel.setReadOnly(True)
            self.memory_tabs.addTab(panel,title)
        self.memory_tabs.currentChanged.connect(self._refresh_memory)
        layout.addWidget(self.memory_tabs)
        layout.addStretch()
        scroll.setWidget(container)

        page_layout = QVBoxLayout(self)
        page_layout.setContentsMargins(0, 0, 0, 0)
        page_layout.addWidget(scroll)

        # Periodic timer for kernel health probe
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._probe_kernel_health)
        self.timer.start(3000)

        # Connect event bus signals
        bus.user_input_received.connect(self._on_user_input)
        bus.spoken_feedback_received.connect(self._on_spoken_feedback)
        bus.coordinator_feedback_ready.connect(self._on_coordinator_feedback)

        # Initial probe
        self._probe_kernel_health()

    def _refresh_memory(self, index):
        if index < 0 or index in self.memory_pending:
            return
        self.memory_pending.add(index)
        request = QNetworkRequest(QUrl(os.environ.get("VIRA_KERNEL_URL","http://127.0.0.1:7780")+self.memory_paths[index]))
        request.setTransferTimeout(2000)
        reply = self.memory_network.get(request)

        def received():
            try:
                if reply.error() != QNetworkReply.NetworkError.NoError:
                    text = "Service unavailable"
                else:
                    text = json.dumps(json.loads(bytes(reply.readAll())),indent=2,ensure_ascii=False)
                self.memory_tabs.widget(index).setPlainText(text)
            except (ValueError, UnicodeError):
                self.memory_tabs.widget(index).setPlainText("Invalid kernel response")
            finally:
                self.memory_pending.discard(index)
                reply.deleteLater()
        reply.finished.connect(received)

    def _probe_kernel_health(self):
        """Perform real non-blocking check on MNSE kernel socket and REST endpoint."""
        self._refresh_memory(self.memory_tabs.currentIndex())
        sock_exists = os.path.exists("/tmp/gracemo.sock")

        # Socket status
        if sock_exists:
            self.lbl_sock.setText("Domain Socket: /tmp/gracemo.sock (file present)")
            self.lbl_sock.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_SUCCESS};")
        else:
            self.lbl_sock.setText("Domain Socket: /tmp/gracemo.sock (OFFLINE ❌)")
            self.lbl_sock.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_MUTED};")

        # REST Endpoint
        rest_ok = False
        try:
            req = urllib.request.Request(os.environ.get("VIRA_KERNEL_URL","http://127.0.0.1:7780")+"/health", method="GET")
            with urllib.request.urlopen(req, timeout=0.5) as resp:
                if resp.status == 200:
                    rest_ok = True
        except Exception:
            rest_ok = False

        if rest_ok:
            self.lbl_rest.setText("REST Endpoint: http://127.0.0.1:7780 (ONLINE ✅)")
            self.lbl_rest.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_SUCCESS};")
            self.metric_kernel.set_value("ONLINE", status_color=COLOR_SUCCESS)
        else:
            self.lbl_rest.setText("REST Endpoint: http://127.0.0.1:7780 (OFFLINE / UNREACHABLE)")
            self.lbl_rest.setStyleSheet(f"font-family: {FONT_FAMILY_MONO}; font-size: 11px; color: {COLOR_TEXT_MUTED};")
            self.metric_kernel.set_value("OFFLINE", status_color=COLOR_OFFLINE)

        if rest_ok:
            self.ind_kernel.set_state(StatusIndicator.STATE_READY)
        else:
            self.ind_kernel.set_state(StatusIndicator.STATE_NOT_CONNECTED)

    def _send_text_command(self):
        cmd = self.txt_input.text().strip()
        if not cmd:
            return
        self.txt_input.clear()
        ts = datetime.now().strftime("%H:%M:%S")
        self.txt_conversation.append(f"[{ts}] USER: {cmd}")
        bus.user_input_received.emit(cmd, "TEXT")

    def _on_user_input(self, text: str, source: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.txt_conversation.append(f"[{ts}] [{source}] USER: {text}")

    def _on_spoken_feedback(self, text: str, status: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.txt_conversation.append(f"[{ts}] GRACEMO [{status}]: {text}")

    def _on_coordinator_feedback(self, status: str, speech: str, details: dict, timestamp: float):
        if speech:
            ts = datetime.now().strftime("%H:%M:%S")
            self.txt_conversation.append(f"[{ts}] COORDINATOR [{status}]: {speech}")
