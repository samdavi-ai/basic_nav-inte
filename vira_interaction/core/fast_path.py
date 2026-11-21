#!/usr/bin/env python3
"""
GRaCEmo ViRa — Fast-Path Intent & Place Dispatcher
Implements deterministic, zero-LLM command routing for critical actions:
STOP, GREET, IDENTITY, STATUS, and known campus/apartment places.
Guarantees sub-15ms execution for emergency stop and high-frequency intents.
"""
from __future__ import annotations
import json
import logging
import os
import re
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
from .state_machine import Expression

log = logging.getLogger("gracemo.fast_path")

DEFAULT_CAMPUS_PLACES: Dict[str, Tuple[float, float, str]] = {
    "library": (4.5, 3.0, "Library"),
    "central library": (4.5, 3.0, "Library"),
    "classroom": (4.5, 3.0, "Classroom"),
    "study": (4.5, 3.0, "Study Room"),
    "study room": (4.5, 3.0, "Study Room"),
    "home study": (4.5, 3.0, "Study Room"),
    "lab": (2.5, 2.0, "Robotics Research Lab"),
    "laboratory": (2.5, 2.0, "Robotics Research Lab"),
    "robotics": (2.5, 2.0, "Robotics Research Lab"),
    "robotics lab": (2.5, 2.0, "Robotics Research Lab"),
    "robotics research lab": (2.5, 2.0, "Robotics Research Lab"),
    "reception": (2.5, -2.5, "Welcome Reception Desk"),
    "reception desk": (2.5, -2.5, "Welcome Reception Desk"),
    "kitchen": (-5.0, -2.5, "Kitchen & Dining"),
    "dining": (-4.0, -2.5, "Kitchen & Dining"),
    "dining room": (-4.0, -2.5, "Kitchen & Dining"),
    "bedroom": (-4.0, 2.2, "Master Bedroom"),
    "master bedroom": (-4.0, 2.2, "Master Bedroom"),
    "living room": (4.0, -2.5, "Living Room"),
    "living": (4.0, -2.5, "Living Room"),
    "lounge": (4.0, -2.5, "Living Room"),
    "hallway": (0.0, 0.0, "Central Hallway"),
    "central hallway": (0.0, 0.0, "Central Hallway"),
    "starting point": (0.0, 0.0, "Starting Point"),
    "start": (0.0, 0.0, "Starting Point"),
    "home": (0.0, 0.0, "Starting Point"),
    "mall": (30.0, -40.0, "Uni-Mall Shopping Center"),
    "gate": (0.0, -95.0, "Main Campus Gate 1"),
    "auditorium": (60.0, 15.0, "Shanti Devi Mittal Auditorium"),
}


from .place_resolver import PlaceResolver, PlaceInfo
from .intent_classifier import IntentClassifier, IntentType, IntentClassificationResult

@dataclass
class FastPathResult:
    matched: bool
    intent: str
    response_text: str
    expression: Expression
    semantic_actions: List[Dict[str, Any]] = field(default_factory=list)
    is_emergency: bool = False
    confidence: float = 1.0


class FastPathMatcher:
    """
    Evaluates raw speech transcripts against deterministic rules.
    If matched, returns ready-to-execute actions and spoken responses.
    Uses authoritative PlaceResolver for spatial waypoint grounding.
    """

    STOP_WORDS = ("stop the robot", "stop robot", "emergency stop", "halt", "freeze", "stop")
    CANCEL_PHRASES = (
        "cancel mission", "cancel navigation", "abort mission", "abort navigation",
        "cancel the mission", "cancel the navigation", "cancel"
    )
    PAUSE_PHRASES = (
        "pause navigation", "pause the navigation", "wait here", "pause the mission",
        "pause mission", "hold position", "pause", "wait"
    )
    RESUME_PHRASES = (
        "resume navigation", "continue to the destination", "continue to destination",
        "continue the mission", "continue mission", "resume the mission",
        "resume mission", "resume", "continue"
    )
    RETURN_HOME_PHRASES = (
        "return to home", "go back to the starting point", "go back to starting point",
        "back to starting point", "return to reception", "return to base", "return to start",
        "return home", "go to reception", "go home"
    )
    DESTINATION_QUERY_PHRASES = (
        "where are you going", "where are we going", "what is your destination",
        "what is our destination", "target destination"
    )
    DISTANCE_QUERY_PHRASES = (
        "how far is it", "how far", "distance remaining", "how much further",
        "how long will it take", "estimated time", "eta"
    )
    LOCATION_QUERY_PHRASES = (
        "what's your current location", "what is your current location",
        "where are you right now", "where are you", "current location", "what is your location",
        "where the state you are now", "what state you are now", "which room are you in",
        "what room are you in", "tell me where you are", "are you in the bedroom",
        "are you in the kitchen", "are you in the study", "are you in the living room",
        "are you in the room"
    )
    NAV_STATUS_PHRASES = (
        "what's the navigation status", "what is the navigation status", "navigation status",
        "what is your current status", "what's your current status", "what is your status",
        "what is current status", "current status", "status report", "system status", "robot status",
        "status", "diagnostics", "battery"
    )
    MOVING_QUERY_PHRASES = (
        "are you moving", "is the robot moving", "are we moving", "is robot moving",
        "are you in motion", "are you driving", "is gracemo moving"
    )
    STUCK_QUERY_PHRASES = (
        "are you stuck", "is the robot stuck", "why did you stop", "why are you stopped",
        "why did the robot stop", "why have you stopped", "are we stuck", "why did we stop",
        "why did you halt", "why are we stopped"
    )
    WAKE_PHRASES = (
        "hi graceemo", "hi gracemo", "hi gresimo", "hi greyzimo", "hi vira",
        "wake up graceemo", "wake up gracemo", "wake up gresimo", "wake up vira",
    )
    GOODBYE_PHRASES = (
        "bye bye graceemo", "bye graceemo", "goodbye graceemo",
        "bye bye gracemo", "bye gracemo", "goodbye gracemo",
        "bye bye", "goodbye",
    )
    GREET_WORDS = ("hello", "hi", "hey", "wave", "hand hi", "good morning", "good afternoon")
    NAVIGATE_PHRASES = (
        "navigate to", "navigating to", "navigated to", "navigate",
        "take me to", "take us to",
        "lead me to", "lead us to",
        "guide me to", "guide us to",
        "bring me to", "bring us to",
        "move to", "proceed to", "proceeds to", "proceeding to",
        "drive to", "drives to", "driving to",
        "walk to", "walks to", "walking to",
        "travel to", "travels to", "traveling to",
        "go to", "head to", "head over to", "head towards",
        "come to", "visit"
    )
    WAKE_PREFIX_RE = re.compile(
        r"^(?:(?:hi|hey|hello|ok|okay|wake up)?\s*(?:graceemo|gracemo|gresimo|greyzimo|vira|robot)\s*[,:\s]*|please\s+|could you\s*(?:please\s*)?|can you\s*(?:please\s*)?)+",
        re.IGNORECASE,
    )
    IDENTITY_PHRASES = ("who are you", "what is your name", "what's your name", "your identity", "what is the name", "tell me your name", "who are u")
    VC_PHRASES = (
        "vice chancellor", "real vice chancellor", "who is the vice chancellor",
        "who is the vc", "name of the vice chancellor", "actual vice chancellor",
        "who is vc", "current vc", "lpu vc", "vc name", "name of vc", "vice-chancellor",
        "vicechancellor", "the vc", "about vc", "about the vc", "who's the vc", "who's the vice chancellor"
    )
    CHANCELLOR_PHRASES = ("chancellor", "who is the chancellor", "founder chancellor", "ashok mittal")
    LOCATION_PHRASES = (
        "where is lpu", "where is lpu located", "location of lpu", "lpu location",
        "where the lpu is located", "where lpu is located", "lpu address", "address of lpu",
        "where is the university located", "where is campus located", "where is the campus",
        "where is lovely professional university", "where is university located", "lpu is located"
    )
    CREATOR_PHRASES = (
        "who created you", "who built you", "who made you", "your creators",
        "who developed you", "who designed you", "who is your creator"
    )
    GRATITUDE_PHRASES = ("thank you", "thanks", "thank u", "many thanks", "appreciate it")
    LOOK_AT_OBJECTS: Dict[str, Tuple[float, float, float, str]] = {
        "bed": (-6.0, 5.0, 0.5, "Master Bed"),
        "master bed": (-6.0, 5.0, 0.5, "Master Bed"),
        "wardrobe": (-8.6, 4.0, 1.0, "Wardrobe"),
        "nightstand": (-4.2, 3.4, 0.6, "Nightstand"),
        "table": (-4.5, -4.5, 0.75, "Dining Table"),
        "dining table": (-4.5, -4.5, 0.75, "Dining Table"),
        "fridge": (-7.5, -5.5, 1.0, "Refrigerator"),
        "refrigerator": (-7.5, -5.5, 1.0, "Refrigerator"),
        "sofa": (5.0, -4.5, 0.6, "Sofa"),
        "couch": (5.0, -4.5, 0.6, "Sofa"),
        "tv": (5.0, -1.5, 1.0, "Television"),
        "television": (5.0, -1.5, 1.0, "Television"),
        "desk": (4.5, 3.0, 0.75, "Study Desk"),
        "study desk": (4.5, 3.0, 0.75, "Study Desk"),
        "bookshelf": (7.5, 5.5, 1.2, "Bookshelf"),
    }

    def __init__(self, places_config_path: Optional[str] = None):
        cfg_path = Path(places_config_path) if places_config_path else None
        self.place_resolver = PlaceResolver(cfg_path)
        self.intent_classifier = IntentClassifier(self.place_resolver)
        self.places: Dict[str, Tuple[float, float, str]] = {
            k: (p.x, p.y, p.name) for k, p in self.place_resolver.places.items()
        }

    def _contains_pattern(self, text: str, patterns: Tuple[str, ...]) -> bool:
        """Matches whole words or phrases using regex word boundaries."""
        for p in patterns:
            pattern = r"\b" + re.escape(p) + r"\b"
            if re.search(pattern, text, re.IGNORECASE):
                return True
        return False

    def match(self, text: str) -> FastPathResult:
        if not text:
            return FastPathResult(matched=False, intent="NONE", response_text="", expression=Expression.IDLE)

        clean = text.lower().strip()
        command_text = self.WAKE_PREFIX_RE.sub("", clean).strip()
        command_text = command_text.lstrip(",:;- ").strip()

        # 1. EMERGENCY STOP FAST PATH (Highest Priority)
        if self._contains_pattern(clean, self.STOP_WORDS):
            log.info("🚨 Fast-Path: Emergency STOP matched!")
            return FastPathResult(
                matched=True,
                intent="STOP",
                response_text="Stopping immediately. Standing by for instructions.",
                expression=Expression.IDLE,
                semantic_actions=[{"action": "Stop", "params": {"reason": "Voice STOP matched"}}],
                is_emergency=True,
                confidence=1.0,
            )

        # 1A. CANCEL MISSION
        if self._contains_pattern(clean, self.CANCEL_PHRASES):
            log.info("⏹️ Fast-Path: CANCEL matched!")
            return FastPathResult(
                matched=True,
                intent="CANCEL",
                response_text="Navigation cancelled. Standing by.",
                expression=Expression.IDLE,
                semantic_actions=[{"action": "Cancel", "params": {}}],
                confidence=1.0,
            )

        # 1B. VOICE INTERRUPT WAKE PHRASE ("Hi GraceEMO")
        # Triggers only when utterance is a pure wake/interrupt greeting without a trailing command
        if self._contains_pattern(clean, self.WAKE_PHRASES) and (not command_text or command_text in self.GREET_WORDS):
            log.info("🎙️ Fast-Path: Wake phrase 'Hi GraceEMO' matched!")
            return FastPathResult(
                matched=True,
                intent="VOICE_INTERRUPT_WAKE",
                response_text="Hello! I have safely paused navigation and am listening. How can I help you?",
                expression=Expression.INTENT,
                semantic_actions=[{"action": "VoiceInterrupt", "params": {"phrase": clean}}],
                confidence=1.0,
            )

        # 1C. VOICE RESUME GOODBYE PHRASE ("Bye bye GraceEMO")
        if self._contains_pattern(clean, self.GOODBYE_PHRASES):
            log.info("👋 Fast-Path: Goodbye phrase 'Bye bye GraceEMO' matched!")
            return FastPathResult(
                matched=True,
                intent="VOICE_RESUME_GOODBYE",
                response_text="Goodbye! Resuming my mission now.",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "VoiceResume", "params": {"phrase": clean}}],
                confidence=1.0,
            )

        # 1D. PAUSE NAVIGATION
        if self._contains_pattern(clean, self.PAUSE_PHRASES):
            log.info("⏸️ Fast-Path: PAUSE matched!")
            intent_name = "PAUSE_MISSION" if "pause the mission" in clean else "PAUSE"
            return FastPathResult(
                matched=True,
                intent=intent_name,
                response_text="Navigation paused. Holding position.",
                expression=Expression.IDLE,
                semantic_actions=[{"action": "Pause", "params": {}}],
                confidence=1.0,
            )

        # 1E. RESUME / CONTINUE NAVIGATION
        if self._contains_pattern(clean, self.RESUME_PHRASES):
            log.info("▶️ Fast-Path: RESUME matched!")
            intent_name = "RESUME_MISSION" if "continue the mission" in clean else "RESUME"
            return FastPathResult(
                matched=True,
                intent=intent_name,
                response_text="Continuing to your destination.",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "Resume", "params": {}}],
                confidence=1.0,
            )

        # 1F. RETURN TO RECEPTION / HOME / STARTING POINT
        if self._contains_pattern(clean, self.RETURN_HOME_PHRASES):
            log.info("🏠 Fast-Path: RETURN HOME matched!")
            return FastPathResult(
                matched=True,
                intent="RETURN_HOME",
                response_text="Returning to the starting point.",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "ReturnHome", "params": {}}],
                confidence=1.0,
            )

        # 1G. IN-FLIGHT DESTINATION QUERIES
        if "where are we going" in clean or "what is our destination" in clean:
            return FastPathResult(
                matched=True,
                intent="NAVIGATION_QUERY_DESTINATION",
                response_text="We are currently en route to our assigned destination.",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "StatusQuery", "params": {"query": "destination"}}],
                confidence=0.99,
            )

        if self._contains_pattern(clean, self.DESTINATION_QUERY_PHRASES):
            return FastPathResult(
                matched=True,
                intent="STATUS",
                response_text="We are currently en route to our assigned destination.",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "StatusQuery", "params": {"query": "destination"}}],
                confidence=0.99,
            )

        # 1G2. IN-FLIGHT DISTANCE QUERIES
        if self._contains_pattern(clean, self.DISTANCE_QUERY_PHRASES):
            return FastPathResult(
                matched=True,
                intent="NAVIGATION_QUERY_DISTANCE",
                response_text="We are proceeding smoothly toward the destination. Distance remaining is being updated in real time.",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "StatusQuery", "params": {"query": "distance"}}],
                confidence=0.99,
            )

        # 1H. CURRENT LOCATION QUERIES
        if self._contains_pattern(clean, self.LOCATION_QUERY_PHRASES):
            return FastPathResult(
                matched=True,
                intent="STATUS",
                response_text="I am currently monitoring our active coordinates within the facility.",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "StatusQuery", "params": {"query": "location"}}],
                confidence=0.99,
            )

        # 1I. NAVIGATION & SYSTEM STATUS QUERIES
        if self._contains_pattern(clean, self.MOVING_QUERY_PHRASES):
            return FastPathResult(
                matched=True,
                intent="STATUS",
                response_text="I am checking my current motion and velocity state.",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "StatusQuery", "params": {"query": "moving"}}],
                confidence=0.99,
            )

        if self._contains_pattern(clean, self.STUCK_QUERY_PHRASES):
            return FastPathResult(
                matched=True,
                intent="STATUS",
                response_text="I am evaluating my current trajectory and obstacles.",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "StatusQuery", "params": {"query": "stuck"}}],
                confidence=0.99,
            )

        if self._contains_pattern(clean, self.NAV_STATUS_PHRASES):
            return FastPathResult(
                matched=True,
                intent="STATUS",
                response_text="Autonomous navigation and safety subsystems are fully operational.",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "StatusQuery", "params": {"query": "system"}}],
                confidence=0.95,
            )

        # 1J. CAMERA & LOOK AT TARGET QUERIES
        if any(p in clean for p in ("show me the", "show the", "look at the", "point at the", "point camera at the", "look at", "show me")):
            for obj_key, (ox, oy, oz, oname) in self.LOOK_AT_OBJECTS.items():
                if re.search(r"\b" + re.escape(obj_key) + r"\b", clean, re.IGNORECASE):
                    return FastPathResult(
                        matched=True,
                        intent="LOOK_AT",
                        response_text=f"Directing my camera toward the {oname}.",
                        expression=Expression.FOCUS,
                        semantic_actions=[{"action": "LookAt", "params": {"x": ox, "y": oy, "z": oz, "target_name": oname}}],
                        confidence=0.98,
                    )

        # 2. GREETINGS
        if self._contains_pattern(clean, self.GREET_WORDS):
            words = clean.split()
            if len(words) <= 4:
                if "evening" in clean:
                    resp = "Good evening. It is a pleasure to be with you. I hope your evening is going well."
                elif "morning" in clean:
                    resp = "Good morning. It is a pleasure to be with you. I hope your day is going well."
                elif "afternoon" in clean:
                    resp = "Good afternoon. It is a pleasure to be with you. How may I assist you today?"
                else:
                    resp = "Hello. It is a pleasure to be with you. How may I be of assistance today?"
                return FastPathResult(
                    matched=True,
                    intent="GREET",
                    response_text=resp,
                    expression=Expression.HAPPY,
                    semantic_actions=[{"action": "Express", "params": {"emotion": "happy"}}],
                    confidence=0.99,
                )

        # 3. LEADERSHIP & VC QUERY
        if self._contains_pattern(clean, self.VC_PHRASES):
            return FastPathResult(
                matched=True,
                intent="QUERY",
                response_text="The Vice Chancellor of Lovely Professional University is Professor Doctor Jaspal Singh Sandhu. I am GRaCEmo ViRa, also known as GraceEMO, an autonomous robotic assistant developed at LPU. <exp-focus>",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "Express", "params": {"emotion": "focus"}}],
                confidence=0.99,
            )

        if self._contains_pattern(clean, self.CHANCELLOR_PHRASES):
            return FastPathResult(
                matched=True,
                intent="QUERY",
                response_text="The Founder Chancellor of Lovely Professional University is Doctor Ashok Kumar Mittal, Honorable Member of Parliament in the Rajya Sabha. <exp-focus>",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "Express", "params": {"emotion": "focus"}}],
                confidence=0.99,
            )

        # 4. IDENTITY & CREATORS
        if self._contains_pattern(clean, self.IDENTITY_PHRASES):
            return FastPathResult(
                matched=True,
                intent="IDENTITY",
                response_text="I am GRaCEmo ViRa, also known as GraceEMO, an autonomous robotic assistant at Lovely Professional University embodying the communication standards and intellectual depth of an academic leader.",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "Express", "params": {"emotion": "focus"}}],
                confidence=0.99,
            )

        if self._contains_pattern(clean, self.CREATOR_PHRASES):
            return FastPathResult(
                matched=True,
                intent="QUERY",
                response_text="I was developed by a team of student researchers at LPU, including Sam Davi, Sameer, and Muzammil, under the faculty mentorship of Doctor Mohit Arora. <exp-focus>",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "Express", "params": {"emotion": "focus"}}],
                confidence=0.99,
            )

        # 5. LOCATION QUERY
        if self._contains_pattern(clean, self.LOCATION_PHRASES):
            return FastPathResult(
                matched=True,
                intent="QUERY",
                response_text="Lovely Professional University is situated on the Jalandhar-Delhi G.T. Road in Phagwara, Punjab. <exp-focus>",
                expression=Expression.FOCUS,
                semantic_actions=[{"action": "Express", "params": {"emotion": "focus"}}],
                confidence=0.99,
            )

        # 4. GRATITUDE
        if self._contains_pattern(clean, self.GRATITUDE_PHRASES):
            return FastPathResult(
                matched=True,
                intent="GRATITUDE",
                response_text="You are very welcome. I am glad to be of assistance.",
                expression=Expression.HAPPY,
                semantic_actions=[{"action": "Express", "params": {"emotion": "happy"}}],
                confidence=0.98,
            )

        # 6. GESTURES: HANDS
        if self._contains_pattern(clean, ("hand up", "hands up")):
            return FastPathResult(
                matched=True,
                intent="HAND_UP",
                response_text="Raising hands in greeting.",
                expression=Expression.HAPPY,
                semantic_actions=[{"action": "Express", "params": {"emotion": "happy"}}],
                confidence=0.95,
            )
        if self._contains_pattern(clean, ("hand down", "hands down")):
            return FastPathResult(
                matched=True,
                intent="HAND_DOWN",
                response_text="Lowering hands to resting position.",
                expression=Expression.IDLE,
                semantic_actions=[{"action": "Express", "params": {"emotion": "idle"}}],
                confidence=0.95,
            )

        # 6B. MULTI-ROOM PATROL / ALL ROOMS TOUR DISPATCH ("visit each and every room", "visit all rooms", "patrol")
        patrol_triggers = (
            "visit each and every room", "visit every room", "visit all rooms",
            "go to each and every room", "go to every room", "go to all rooms",
            "check each and every room", "check every room", "check all rooms",
            "inspect each and every room", "inspect every room", "inspect all rooms",
            "tour each and every room", "tour every room", "tour all rooms",
            "tour the apartment", "tour the house", "tour the rooms", "tour all",
            "patrol the apartment", "patrol the house", "patrol all rooms", "patrol each and every room",
            "patrol every room", "start patrol", "patrol",
            "explore the apartment", "explore all rooms", "explore each and every room",
            "explore every room", "explore the house",
            "each and every room", "every room", "all rooms"
        )
        if any(pt in clean for pt in patrol_triggers) or (command_text and any(pt in command_text for pt in patrol_triggers)):
            log.info("🚀 Fast-Path: Multi-room patrol tour triggered!")
            return FastPathResult(
                matched=True,
                intent="PATROL_ROOMS",
                response_text="Starting a complete tour of all rooms: Study Room, Master Bedroom, Kitchen, and Living Room. Please follow me!",
                expression=Expression.FOCUS,
                semantic_actions=[
                    {
                        "action": "StartMission",
                        "params": {
                            "mission_name": "Tour of All Rooms",
                            "waypoints": [
                                {"name": "Study Room", "coords": {"x": 4.5, "y": 3.0, "yaw": 1.57}},
                                {"name": "Master Bedroom", "coords": {"x": -4.0, "y": 2.2, "yaw": 1.57}},
                                {"name": "Kitchen & Dining", "coords": {"x": -4.0, "y": -2.5, "yaw": -1.57}},
                                {"name": "Living Room", "coords": {"x": 4.0, "y": -2.5, "yaw": -1.57}},
                                {"name": "Starting Point", "coords": {"x": 0.0, "y": 0.0, "yaw": 0.0}}
                            ]
                        }
                    }
                ],
                confidence=0.99,
            )

        # 7. DETERMINISTIC INTENT CLASSIFICATION & DESTINATION EXTRACTION
        # Strictly separates conversational speech from physical navigation actions.
        # MANDATORY INVARIANT: Conversational input must NEVER produce a navigation goal.
        intent_res = self.intent_classifier.classify(text)

        # 7A. Conversational Speech -> Route to conversational engine with ZERO robot actions
        if intent_res.intent == IntentType.CONVERSATIONAL:
            return FastPathResult(
                matched=False,
                intent="CONVERSATIONAL",
                response_text="",
                expression=Expression.IDLE,
                semantic_actions=[],
                confidence=intent_res.confidence,
            )

        # 7B. Informational Questions -> Route to conversational engine with ZERO robot actions
        if intent_res.intent == IntentType.QUESTION:
            return FastPathResult(
                matched=False,
                intent="QUESTION",
                response_text="",
                expression=Expression.IDLE,
                semantic_actions=[],
                confidence=intent_res.confidence,
            )

        # 7C. Compound Multi-Waypoint Navigation
        if intent_res.intent == IntentType.COMPOUND_NAVIGATION and intent_res.is_navigation:
            wps = []
            for pl in intent_res.places_info:
                wps.append({
                    "name": pl.name,
                    "coords": {
                        "x": pl.x,
                        "y": pl.y,
                        "yaw": pl.yaw if pl.yaw is not None else 0.0,
                    }
                })
            dest_names = ", then ".join(intent_res.canonical_destinations)
            return FastPathResult(
                matched=True,
                intent="COMPOUND_NAVIGATION",
                response_text=f"Navigating to {dest_names}. Please follow me!",
                expression=Expression.FOCUS,
                semantic_actions=[
                    {
                        "action": "StartMission",
                        "params": {
                            "mission_name": f"Compound Mission to {dest_names}",
                            "waypoints": wps,
                        }
                    }
                ],
                confidence=intent_res.confidence,
            )

        # 7D. Single-Leg NAVIGATE_TO
        if intent_res.intent == IntentType.NAVIGATE_TO:
            if intent_res.clarification_prompt:
                log.info(f"❓ Fast-Path: Ambiguous destination detected ({intent_res.raw_destination_phrase})")
                return FastPathResult(
                    matched=True,
                    intent="AMBIGUOUS_DESTINATION",
                    response_text=intent_res.clarification_prompt,
                    expression=Expression.INTENT,
                    semantic_actions=[],
                    confidence=intent_res.confidence,
                )

            if intent_res.is_navigation and intent_res.place_info:
                pl = intent_res.place_info
                # Bounds check: Apartment simulation bounds [-8.8, 8.8] x [-6.3, 6.3]
                if not self.place_resolver.is_within_bounds(pl):
                    log.warning(f"⚠️ Fast-Path: Destination '{pl.name}' ({pl.x}, {pl.y}) out of active bounds")
                    return FastPathResult(
                        matched=True,
                        intent="UNREACHABLE_DESTINATION",
                        response_text=f"Destination '{pl.name}' is outside the active map boundaries and cannot be reached.",
                        expression=Expression.SAD,
                        semantic_actions=[],
                        confidence=1.0,
                    )

                return FastPathResult(
                    matched=True,
                    intent="NAVIGATE",
                    response_text=f"Navigating to {pl.name}. Please follow me!",
                    expression=Expression.FOCUS,
                    semantic_actions=[
                        {
                            "action": "NavigateTo",
                            "params": {
                                "x": pl.x,
                                "y": pl.y,
                                "yaw": pl.yaw,
                                "target_name": pl.name,
                                "tolerance_m": pl.tolerance_m,
                            }
                        }
                    ],
                    confidence=intent_res.confidence,
                )

            if intent_res.raw_destination_phrase:
                return FastPathResult(
                    matched=True,
                    intent="UNKNOWN_DESTINATION",
                    response_text=f"I cannot find '{intent_res.raw_destination_phrase}' and do not have it in my map. Known destinations include library, classroom, study room, laboratory, reception, kitchen, bedroom, and living room.",
                    expression=Expression.SAD,
                    semantic_actions=[],
                    confidence=intent_res.confidence,
                )

        return FastPathResult(matched=False, intent="UNKNOWN", response_text="", expression=Expression.IDLE)

    def extract_place(self, text: str) -> Optional[Tuple[float, float, str]]:
        """Matches place name from text against known places dictionary."""
        p = self.place_resolver.resolve_place(text)
        if p:
            return (p.x, p.y, p.name)
        return None

    def _detect_compound_navigation(self, clean: str, command_text: Optional[str]) -> Optional[FastPathResult]:
        eval_text = command_text or clean
        tl = eval_text.lower()
        if self._contains_pattern(clean, ("where is", "who is", "what is", "how is", "tell me")):
            return None

        # Sequence indicator required to prevent false-positives on single compound names
        sequence_hints = (" then", " and then", " followed by", " after that", ", then", " and to the", " then to", " and the ", " and each", " and every")
        has_seq = any(sh in tl for sh in sequence_hints) or ("," in tl and any(c in tl for c in ("and", "then", "to")))
        if not has_seq:
            return None

        # Sort place keys longest first to prevent sub-phrases like "home" matching inside "home study"
        sorted_keys = sorted(self.place_resolver.places.keys(), key=len, reverse=True)
        occupied_spans = []
        matches = []

        for key in sorted_keys:
            pattern = r'\b' + re.escape(key) + r'\b'
            for m in re.finditer(pattern, tl):
                start, end = m.span()
                # Check overlap with already matched longer keys
                if any(start < o_end and end > o_start for o_start, o_end in occupied_spans):
                    continue
                occupied_spans.append((start, end))
                matches.append((start, self.place_resolver.places[key]))

        if len(matches) < 2:
            return None

        matches.sort(key=lambda x: x[0])
        unique_places = []
        seen_names = set()
        for pos, p in matches:
            if p.name not in seen_names:
                unique_places.append(p)
                seen_names.add(p.name)

        if len(unique_places) >= 2:
            names_str = ", then ".join([p.name for p in unique_places])
            wps = [
                {
                    "name": p.name,
                    "coords": {"x": p.x, "y": p.y, "yaw": p.yaw if p.yaw is not None else 0.0}
                }
                for p in unique_places
            ]
            return FastPathResult(
                matched=True,
                intent="COMPOUND_NAVIGATION",
                response_text=f"Navigating to {names_str}. Please follow me!",
                expression=Expression.FOCUS,
                semantic_actions=[
                    {
                        "action": "StartMission",
                        "params": {
                            "mission_name": f"Sequence: {' -> '.join([p.name for p in unique_places])}",
                            "waypoints": wps
                        }
                    }
                ],
                confidence=0.98,
            )
        return None
