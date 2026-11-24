#!/usr/bin/env python3
"""
GRaCEmo ViRa — Production Interaction Gateway
Coordinates the end-to-end interactive nervous system pipeline:
  Audio/Text Input
        ↓
  STT Transcription (Groq Whisper / Fallback)
        ↓
  Intent Routing (Fast-Path vs Brain)
        ↓
  Semantic Command Extraction
        ↓
  Safety & Validation Gate
        ↓
  VIRA EventBus Publication (with trace_id)
        ↓
  TTS & Avatar Synchronization
"""
from __future__ import annotations
import os
import logging
import time
import uuid
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Union
import numpy as np
import requests

from .state_machine import CognitiveState, Expression, InteractionFSM
from .safety_gate import CommandValidator, SafetyGate, ValidationResult
from .event_bus import EventBusClient
from .fast_path import FastPathMatcher, FastPathResult
from .brain_adapter import BrainAdapter, BrainResult
from .stt_adapter import STTAdapter, STTResult
from .tts_adapter import TTSAdapter
from .memory_adapter import MemoryAdapter

log = logging.getLogger("gracemo.gateway")


@dataclass
class GatewayResult:
    response: str
    intent: str
    confidence: float
    expression: Expression
    actions: List[Dict[str, Any]] = field(default_factory=list)
    semantic_commands: List[Dict[str, Any]] = field(default_factory=list)
    trace_id: str = ""
    session_id: str = ""
    state: str = "IDLE"
    latency_ms: float = 0.0
    error: Optional[str] = None


class InteractionGateway:
    """
    Central Coordinator for GRaCEmo ViRa Interaction.
    Ensures safe, observable, and contract-compliant interaction.
    """

    def __init__(
        self,
        fsm: Optional[InteractionFSM] = None,
        event_bus: Optional[EventBusClient] = None,
        safety_gate: Optional[SafetyGate] = None,
        fast_path: Optional[FastPathMatcher] = None,
        brain: Optional[BrainAdapter] = None,
        stt: Optional[STTAdapter] = None,
        tts: Optional[TTSAdapter] = None,
        memory: Optional[MemoryAdapter] = None,
    ):
        self.fsm = fsm or InteractionFSM()
        self.event_bus = event_bus or EventBusClient(base_url=os.environ.get("VIRA_KERNEL_URL", "http://127.0.0.1:7780"))
        self.safety_gate = safety_gate or SafetyGate()
        self.fast_path = fast_path or FastPathMatcher()
        self.brain = brain or BrainAdapter()
        self.stt = stt or STTAdapter()
        self.tts = tts or TTSAdapter()
        self.memory = memory or MemoryAdapter()
        self.current_location = "Starting Point"

        # Connect FSM state changes to EventBus
        self.fsm.add_listener(self._on_fsm_transition)
        self.fsm.transition_to(CognitiveState.IDLE, reason="Gateway initialized")

    def _on_fsm_transition(self, prev: CognitiveState, curr: CognitiveState, exp: Expression, reason: str):
        self.event_bus.emit(
            "InteractionStateChanged",
            {
                "previous_state": prev.value,
                "current_state": curr.value,
                "active_expression": exp.value,
                "reason": reason,
            },
            source="Voice",
        )

    def process_text(
        self,
        text: str,
        session_id: Optional[str] = None,
        trace_id: Optional[str] = None,
    ) -> GatewayResult:
        """Process a text command or query through the pipeline."""
        start_t = time.perf_counter()
        tid = trace_id or str(uuid.uuid4())
        sid = session_id or "session_default"

        try:
            return self._route_text(text, tid, sid, start_t)
        except (PermissionError, requests.RequestException) as exc:
            log.warning("Kernel dispatch unconfirmed (%s)", type(exc).__name__)
            return GatewayResult(response="Command delivery could not be confirmed.",
                                 intent="DISPATCH_UNCONFIRMED", confidence=0.0,
                                 expression=self.fsm.expression, trace_id=tid, session_id=sid,
                                 state=self.fsm.state.value, error="kernel_dispatch_unconfirmed")

    def _route_text(self, text, tid, sid, start_t):
        clean_text = (text or "").strip()
        if not clean_text:
            return GatewayResult(
                response="",
                intent="NONE",
                confidence=0.0,
                expression=self.fsm.expression,
                trace_id=tid,
                session_id=sid,
                state=self.fsm.state.value,
            )

        log.info(f"📩 Gateway Ingestion [trace={tid[:8]}]: \"{clean_text}\"")

        # 0. Emergency Stop Guard Protocol
        RESET_ESTOP_PHRASES = (
            "reset emergency stop", "clear emergency stop", "reset estop",
            "clear estop", "reset safety", "clear safety"
        )
        is_estop_active = (
            self.fsm.state == CognitiveState.EMERGENCY_STOP
            or self.safety_gate.is_emergency_stop_latched()
        )

        if is_estop_active:
            clean_lower = clean_text.lower()
            # Explicit operator reset authorizes recovery
            if any(p in clean_lower for p in RESET_ESTOP_PHRASES):
                log.info("🔓 Operator authorized Emergency Stop reset")
                if hasattr(self.safety_gate, "reset_emergency_stop"):
                    self.safety_gate.reset_emergency_stop()
                elif hasattr(self.safety_gate, "clear_emergency_stop"):
                    self.safety_gate.clear_emergency_stop()
                self.fsm.transition_to(CognitiveState.IDLE, reason="Emergency stop reset authorized by operator")
                return GatewayResult(
                    response="Emergency stop cleared. System is standing by.",
                    intent="CLEAR_ESTOP",
                    confidence=1.0,
                    expression=Expression.IDLE,
                    actions=[],
                    semantic_commands=[],
                    trace_id=tid,
                    session_id=sid,
                    state=self.fsm.state.value,
                    latency_ms=(time.perf_counter() - start_t) * 1000.0,
                )

        # 1. State -> THINKING (only if not locked in emergency stop)
        if not is_estop_active:
            self.fsm.transition_to(CognitiveState.THINKING, reason="Processing text input")

        # 2. Check Fast-Path (STOP, GREET, IDENTITY, NAVIGATE)
        fp_result: FastPathResult = self.fast_path.match(clean_text)

        if fp_result.matched:
            # Handle STOP immediately
            if fp_result.is_emergency or fp_result.intent == "STOP":
                return self._handle_emergency_stop(fp_result, tid, sid, start_t)

            return self._handle_fast_path(fp_result, clean_text, tid, sid, start_t)

        # 3. Cognitive Brain Reasoning
        return self._handle_brain_reasoning(clean_text, tid, sid, start_t)

    def process_audio(
        self,
        audio_data: Union[np.ndarray, bytes, str],
        session_id: Optional[str] = None,
        trace_id: Optional[str] = None,
        sample_rate: int = 16000,
    ) -> GatewayResult:
        """Process an audio utterance buffer through STT then text pipeline."""
        start_t = time.perf_counter()
        tid = trace_id or str(uuid.uuid4())
        sid = session_id or "session_default"

        # 1. State -> LISTENING -> THINKING
        self.fsm.transition_to(CognitiveState.LISTENING, reason="Audio captured")
        stt_res: STTResult = self.stt.transcribe(audio_data, sample_rate=sample_rate)

        if not stt_res.is_valid or not stt_res.text:
            log.info(f"⤍ [STT Filtered] Ambient noise or silence (trace={tid[:8]})")
            self.fsm.transition_to(CognitiveState.IDLE, reason="Speech was silent or filtered")
            return GatewayResult(
                response="",
                intent="FILTERED",
                confidence=0.0,
                expression=self.fsm.expression,
                trace_id=tid,
                session_id=sid,
                state=self.fsm.state.value,
                latency_ms=(time.perf_counter() - start_t) * 1000.0,
            )

        # Emit VoiceDetected
        self.event_bus.emit(
            "VoiceDetected",
            {
                "transcription": stt_res.text,
                "confidence": stt_res.confidence,
                "language": "en",
            },
            trace_id=tid,
            session_id=sid,
            source="Voice",
        )

        return self.process_text(stt_res.text, session_id=sid, trace_id=tid)

    def _handle_emergency_stop(
        self, fp: FastPathResult, trace_id: str, session_id: str, start_t: float
    ) -> GatewayResult:
        """Independent sub-20ms emergency stop dispatch."""
        self.tts.stop()  # Interrupt active audio
        self.fsm.emergency_stop(reason="Voice STOP matched")

        validated_actions = []
        for act in fp.semantic_actions:
            v_res: ValidationResult = self.safety_gate.evaluate_action(act)
            if v_res.valid:
                validated_actions.append(v_res.sanitized_action)
                self.event_bus.emit(
                    "ActionRequested",
                    v_res.sanitized_action,
                    trace_id=trace_id,
                    session_id=session_id,
                    source="Voice",
                )

        latency = (time.perf_counter() - start_t) * 1000.0
        log.info(f"🛑 E-STOP dispatched successfully in {latency:.2f}ms (trace={trace_id[:8]})")

        return GatewayResult(
            response=fp.response_text,
            intent="STOP",
            confidence=fp.confidence,
            expression=Expression.IDLE,
            actions=validated_actions,
            semantic_commands=validated_actions,
            trace_id=trace_id,
            session_id=session_id,
            state=self.fsm.state.value,
            latency_ms=latency,
        )

    def _handle_fast_path(
        self, fp: FastPathResult, raw_text: str, trace_id: str, session_id: str, start_t: float
    ) -> GatewayResult:
        # 1. Determine target state with safety and intent awareness
        is_estop_active = self.safety_gate.is_emergency_stop_latched()
        response_text = fp.response_text

        # Mark en-route status in brain if navigating (do NOT overwrite current_location prematurely)
        if fp.intent == "NAVIGATE" and fp.semantic_actions:
            target = fp.semantic_actions[0].get("params", {}).get("target_name")
            if target and self.brain and hasattr(self.brain, "set_robot_location"):
                self.brain.set_robot_location(target, is_en_route=True)

        # Contextualize location query if known
        if fp.intent == "STATUS":
            for act in fp.semantic_actions:
                if act.get("params", {}).get("query") == "location" and hasattr(self, "current_location"):
                    response_text = f"I am currently positioned inside the {self.current_location}."
        expression = fp.expression

        if fp.intent == "STOP":
            target_state = CognitiveState.EMERGENCY_STOP
        elif is_estop_active and fp.intent in ("VOICE_RESUME_GOODBYE", "RESUME_MISSION", "RESUME"):
            target_state = CognitiveState.EMERGENCY_STOP
            response_text = "Safety fault active. Cannot resume navigation."
            expression = Expression.IDLE
            return GatewayResult(
                response=response_text,
                intent="RESUME_REJECTED_ESTOP",
                confidence=1.0,
                expression=Expression.IDLE,
                actions=[],
                semantic_commands=[],
                trace_id=trace_id,
                session_id=session_id,
                state=self.fsm.state.value,
                latency_ms=(time.perf_counter() - start_t) * 1000.0,
            )
        elif fp.intent == "VOICE_INTERRUPT_WAKE":
            target_state = CognitiveState.LISTENING
        elif is_estop_active:
            target_state = CognitiveState.EMERGENCY_STOP
        elif fp.semantic_actions:
            target_state = CognitiveState.EXECUTING
        else:
            target_state = CognitiveState.SPEAKING

        if self.fsm.state != target_state:
            self.fsm.transition_to(target_state, reason=f"Fast-path {fp.intent}", expression=expression)

        # 2. Validate and dispatch semantic actions
        validated_actions = []
        for act in fp.semantic_actions:
            v_res = self.safety_gate.evaluate_action(act)
            if v_res.valid:
                validated_actions.append(v_res.sanitized_action)
                self.event_bus.emit(
                    "ActionRequested",
                    v_res.sanitized_action,
                    trace_id=trace_id,
                    session_id=session_id,
                    source="Voice",
                )
            else:
                log.warning(f"⚠️ Action rejected by SafetyGate: {v_res.error_message}")
                if fp.intent in ("VOICE_RESUME_GOODBYE", "RESUME_MISSION"):
                    response_text = "Safety fault active. Cannot resume navigation."
                    expression = Expression.IDLE

        # 3. Emit VoiceIntent
        self.event_bus.emit(
            "VoiceIntent",
            {
                "intent": fp.intent,
                "confidence": fp.confidence,
                "raw_transcript": raw_text,
            },
            trace_id=trace_id,
            session_id=session_id,
            source="Voice",
        )

        latency = (time.perf_counter() - start_t) * 1000.0
        return GatewayResult(
            response=response_text,
            intent=fp.intent,
            confidence=fp.confidence,
            expression=expression,
            actions=validated_actions,
            semantic_commands=validated_actions,
            trace_id=trace_id,
            session_id=session_id,
            state=self.fsm.state.value,
            latency_ms=latency,
        )

    def _handle_brain_reasoning(
        self, text: str, trace_id: str, session_id: str, start_t: float
    ) -> GatewayResult:
        if self.brain is None:
            log.warning("☁️ Cloud Brain is offline/unconfigured. Falling back to local offline response.")
            latency = (time.perf_counter() - start_t) * 1000.0
            self.fsm.transition_to(CognitiveState.SPEAKING, reason="Brain offline fallback", expression=Expression.SAD)
            return GatewayResult(
                response="I am currently offline from the cloud brain, but local navigation and safety remain active.",
                intent="OFFLINE_FALLBACK",
                confidence=0.0,
                expression=Expression.SAD,
                actions=[],
                semantic_commands=[],
                trace_id=trace_id,
                session_id=session_id,
                state=self.fsm.state.value,
                latency_ms=latency,
            )

        brain_res: BrainResult = self.brain.reason(text)

        validated_actions = []
        for act in brain_res.semantic_actions:
            # Policy Gate: Arbitrary low-level motor writes or raw open-loop velocities remain prohibited
            act_name = act.get("action", "")
            if act_name in ("SetTwist", "MotorCommand", "DirectMotorControl"):
                log.warning(f"🛡️ Safety Gate: Unsolicited raw motor write '{act_name}' from conversational LLM blocked.")
                continue

            # PHASE 5 MANDATORY INVARIANT: Conversational LLM must NEVER directly dispatch navigation actions.
            # All physical navigation actions must pass deterministic IntentClassifier validation.
            if act_name in ("NavigateTo", "NavigateToRoom", "StartMission", "PatrolRooms", "NavigateWaypoints"):
                val_res = self.fast_path.intent_classifier.classify(text)
                if not (val_res.is_navigation and (val_res.place_info or val_res.places_info)):
                    log.warning(f"🛡️ Safety Gate: Unsolicited navigation action '{act_name}' from conversational LLM blocked.")
                    continue
                log.info(f"🛡️ Safety Gate: Navigation action '{act_name}' from Brain verified by IntentClassifier.")

            v_res = self.safety_gate.evaluate_action(act)
            if v_res.valid:
                validated_actions.append(v_res.sanitized_action)
                self.event_bus.emit(
                    "ActionRequested",
                    v_res.sanitized_action,
                    trace_id=trace_id,
                    session_id=session_id,
                    source="Brain",
                )
            else:
                log.warning(f"⚠️ Action from Brain rejected by SafetyGate: {v_res.error_message}")

        # Emit VoiceIntent
        self.event_bus.emit(
            "VoiceIntent",
            {
                "intent": brain_res.intent,
                "confidence": brain_res.confidence,
                "raw_transcript": text,
            },
            trace_id=trace_id,
            session_id=session_id,
            source="Brain",
        )

        target_state = CognitiveState.EXECUTING if validated_actions else CognitiveState.SPEAKING
        self.fsm.transition_to(target_state, reason=f"Brain {brain_res.intent}", expression=brain_res.expression)

        latency = (time.perf_counter() - start_t) * 1000.0
        return GatewayResult(
            response=brain_res.response_text,
            intent=brain_res.intent,
            confidence=brain_res.confidence,
            expression=brain_res.expression,
            actions=validated_actions,
            semantic_commands=validated_actions,
            trace_id=trace_id,
            session_id=session_id,
            state=self.fsm.state.value,
            latency_ms=latency,
        )
