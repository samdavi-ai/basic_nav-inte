#!/usr/bin/env python3
"""
GRaCEmo ViRa — Strict English & Multilingual Intent Classifier
============================================================
Enforces deterministic intent classification and boundary-aware destination extraction.

Key Invariant:
  CONVERSATIONAL INPUT MUST NEVER PRODUCE A NAVIGATION GOAL.
  A location name alone must NEVER trigger navigation.

Supported Intent Types:
  1. NAVIGATE_TO
  2. COMPOUND_NAVIGATION
  3. CANCEL_NAVIGATION
  4. EMERGENCY_STOP
  5. FOLLOW_PERSON
  6. CONVERSATIONAL
  7. QUESTION
  8. GREETING
  9. THANK_YOU
  10. UNKNOWN
"""
from __future__ import annotations

import enum
import logging
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

from .place_resolver import PlaceInfo, PlaceResolver

log = logging.getLogger("gracemo.intent_classifier")


class IntentType(str, enum.Enum):
    NAVIGATE_TO = "NAVIGATE_TO"
    COMPOUND_NAVIGATION = "COMPOUND_NAVIGATION"
    CANCEL_NAVIGATION = "CANCEL_NAVIGATION"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    FOLLOW_PERSON = "FOLLOW_PERSON"
    CONVERSATIONAL = "CONVERSATIONAL"
    QUESTION = "QUESTION"
    GREETING = "GREETING"
    THANK_YOU = "THANK_YOU"
    UNKNOWN = "UNKNOWN"


@dataclass
class IntentClassificationResult:
    intent: IntentType
    confidence: float
    canonical_destination: Optional[str] = None
    canonical_destinations: List[str] = field(default_factory=list)
    raw_destination_phrase: Optional[str] = None
    place_info: Optional[PlaceInfo] = None
    places_info: List[PlaceInfo] = field(default_factory=list)
    clarification_prompt: Optional[str] = None
    reason: str = ""
    is_navigation: bool = False


class IntentClassifier:
    """
    Production-grade Intent Classifier and Destination Extractor for GRaCEmo ViRa.
    Separates conversational dialogue from physical locomotion imperatives.
    """

    # 1. Wake prefixes to strip safely
    WAKE_PREFIX_RE = re.compile(
        r"^(?:(?:hi|hey|hello|ok|okay|wake up)?\s*(?:graceemo|gracemo|gresimo|greyzimo|vira|robot)\s*[,:\s]*|please\s+|could you\s*(?:please\s*)?|can you\s*(?:please\s*)?)+",
        re.IGNORECASE,
    )

    # 2. EMERGENCY STOP keywords (< 20ms priority)
    EMERGENCY_STOP_PATTERNS = (
        r"\b(?:emergency\s+stop|estop|freeze|halt\s+immediately|halt|stop\s+immediately|stop\s+right\s+now|stop\s+moving)\b",
        r"^stop[.!?,]?$",
    )

    # 3. CANCEL NAVIGATION
    CANCEL_PATTERNS = (
        r"\b(?:cancel\s+navigation|cancel\s+mission|cancel\s+the\s+mission|cancel\s+this\s+mission)\b",
        r"\b(?:abort\s+navigation|abort\s+mission|abort\s+the\s+mission|abort)\b",
        r"\b(?:stop\s+navigation|stop\s+the\s+mission|stop\s+the\s+tour|stop\s+patrol)\b",
        r"\b(?:never\s+mind|nevermind)\b",
        r"^cancel[.!?,]?$",
    )

    # 4. FOLLOW PERSON
    FOLLOW_PATTERNS = (
        r"\b(?:follow\s+me|follow\s+behind\s+me|come\s+with\s+me|shadow\s+me)\b",
        r"^follow[.!?,]?$",
    )

    # 5. GREETINGS
    GREET_PATTERNS = (
        r"^(?:hello|hi|hey|good\s+morning|good\s+afternoon|good\s+evening|howdy)[.!?,]?$",
        r"^(?:hello|hi|hey)\s+(?:there|vira|robot|graceemo)[.!?,]?$",
    )

    # 6. THANK YOU / GRATITUDE
    THANK_PATTERNS = (
        r"\b(?:thank\s+you\s+very\s+much|thank\s+you\s+so\s+much|thank\s+you|thanks\s+a\s+lot|thanks|much\s+appreciated|appreciate\s+it)\b",
    )

    # 7. QUESTION / INFORMATIONAL QUERY INDICATORS
    QUESTION_STARTERS = (
        r"^(?:what|where|who|how|why|when|which)\b",
        r"^(?:tell\s+me\s+about|can\s+you\s+tell\s+me\s+about|could\s+you\s+tell\s+me\s+about)\b",
        r"^(?:explain|describe|what\s+is\s+inside|what\s+is\s+in|who\s+is|where\s+is)\b",
        r"^(?:is\s+there|are\s+there|do\s+you\s+have|does\s+the)\b",
    )

    # 8. CONVERSATIONAL FALSE-POSITIVE REJECTION FILTERS
    # Words/phrases that signal evaluation, description, praise, past events, or personal activities
    CONVERSATIONAL_MARKERS = (
        # Praises and emotions
        r"\b(?:proud\s+of\s+you|good\s+job|great\s+job|well\s+done|nice\s+work|you\s+are\s+amazing)\b",
        # Adjectives describing rooms / places
        r"\b(?:beautiful\s+place|looks\s+beautiful|looks\s+nice|is\s+very\s+beautiful|is\s+beautiful)\b",
        r"\b(?:very\s+beautiful|so\s+beautiful|looks\s+clean|looks\s+great|looks\s+good|looks\s+bad)\b",
        r"\b(?:is\s+quiet|is\s+noisy|is\s+clean|is\s+dirty|is\s+large|is\s+small|is\s+cozy)\b",
        # Preference statements
        r"\b(?:i\s+like\s+the|i\s+love\s+the|i\s+prefer\s+the|my\s+favorite)\b",
        # Welcome statements
        r"\b(?:welcome\s+to\s+the|welcome\s+to\s+our)\b",
        # Past-tense recollections
        r"\b(?:was\s+in\s+the|were\s+in\s+the|visited\s+the|went\s+to\s+the|stayed\s+in\s+the)\b",
        r"\b(?:yesterday|last\s+night|last\s+week|earlier|ago)\b",
        # Action inside a room (cooking, sleeping, studying, working)
        r"\b(?:going\s+to\s+cook\s+in|cook\s+in\s+the|cooking\s+in\s+the|sleep\s+in\s+the|sleeping\s+in\s+the)\b",
        r"\b(?:study\s+in\s+the|studying\s+in\s+the|work\s+in\s+the|working\s+in\s+the|eat\s+in\s+the)\b",
        # Existential statements
        r"\b(?:there\s+is\s+a|there\s+are|near\s+the\s+reception|next\s+to\s+the)\b",
        # Conversational uncertainty or fragments
        r"\b(?:not\s+sure\s+how\s+to\s+go|don't\s+know\s+how\s+to\s+go|second\s+part\s+is|one\s+of\s+them)\b",
    )

    # 9. COMPOUND NAVIGATION SEPARATORS
    COMPOUND_SPLIT_RE = re.compile(
        r"\s*(?:,\s*then\s+|,\s*and\s+then\s+|\s+then\s+|\s+and\s+then\s+|\s+after\s+that\s+|\s+followed\s+by\s+|\s+next\s+)\s*",
        re.IGNORECASE,
    )

    # 10. NAVIGATION VERBS & INTENT PATTERNS (English and Tamil)
    # Strict regex with non-greedy destination extraction bounded by punctuation or conversational text
    NAVIGATE_PATTERNS = [
        # Directed Imperatives (English):
        # "we're going to go to kitchen", "we are going to the kitchen", "i'm going to master bedroom",
        # "let's go to kitchen", "take me to bedroom", "can you take me to the kitchen", etc.
        re.compile(
            r"^(?:"
            r"(?:we're|we\s+are|i'm|i\s+am)\s+(?:going\s+to\s+go\s+to|going\s+to|heading\s+to|heading\s+over\s+to)|"
            r"(?:we|i)\s+(?:want\s+to\s+go\s+to|would\s+like\s+to\s+go\s+to|want\s+to\s+visit|would\s+like\s+to\s+visit|will\s+go\s+to|'ll\s+go\s+to)|"
            r"(?:i\s+want\s+you\s+to\s+go\s+to|i\s+would\s+like\s+you\s+to\s+go\s+to|we\s+want\s+you\s+to\s+go\s+to)|"
            r"(?:can\s+we|could\s+we|shall\s+we|can\s+you|could\s+you|please)\s+(?:go\s+to|take\s+us\s+to|take\s+me\s+to|guide\s+us\s+to|guide\s+me\s+to|lead\s+us\s+to|lead\s+me\s+to|visit|(?:navigate|nav\s*gate|navi\s*gate|nail\s*gate|now\s*gate)\s+to)|"
            r"(?:let's|let\s+us)\s+(?:go\s+to|visit|head\s+to|head\s+over\s+to|proceed\s+to|travel\s+to)|"
            r"(?:take\s+me\s+to|take\s+us\s+to|guide\s+me\s+to|guide\s+us\s+to|lead\s+me\s+to|lead\s+us\s+to|bring\s+me\s+to|bring\s+us\s+to)|"
            r"(?:(?:navigate|nav\s*gate|navi\s*gate|nail\s*gate|now\s*gate)\s+to|go\s+to|move\s+to|head\s+over\s+to|head\s+towards|head\s+to|walk\s+to|proceed\s+to|travel\s+to|drive\s+to|visit)"
            r")\s+(?:the\s+)?([^.!?,;]+)",
            re.IGNORECASE,
        ),
        # Tamil Navigation: "படுக்கையறைக்குப் போ", "சமையலறைக்குச் செல்"
        re.compile(r"^(.+?)(?:க்குப்\s*போ|க்குச்\s*செல்|க்கு\s*போ|க்கு\s*செல்)$", re.IGNORECASE),
    ]

    # Non-navigation action verbs that immediately disqualify candidate destination phrases
    DISQUALIFYING_ACTION_VERBS = {
        "cook", "sleep", "eat", "study", "work", "read", "write", "change", "do",
        "be", "have", "see", "make", "know", "ask", "tell", "use", "get", "try",
        "help", "find", "call", "look", "feel", "leave", "put", "keep", "let",
        "begin", "start", "show", "hear", "play", "run", "say", "solve", "speak",
        "learn", "check", "update", "test", "edit", "insert", "delete", "create",
        "fix", "switch", "turn", "open", "close", "explain", "clarify", "buy"
    }

    # Navigation verbs that must NEVER be extracted as destination names
    NAVIGATION_VERBS = {
        "go", "goes", "going", "gone", "navigate", "navigating", "navigated",
        "move", "moves", "moving", "moved", "visit", "visiting", "visited",
        "take", "takes", "taking", "took", "travel", "travels", "traveling",
        "proceed", "proceeds", "proceeding", "head", "heads", "heading",
        "walk", "walks", "walking", "lead", "leads", "guide", "guides",
        "bring", "brings", "drive", "drives", "reach", "reach to", "reach the"
    }

    # Approved Tamil to English Canonical Place Key Mappings
    TAMIL_PLACE_MAP: Dict[str, str] = {
        "படுக்கையறை": "bedroom",
        "சமையலறை": "kitchen",
        "படிக்கும் அறை": "study room",
        "வரவேற்பறை": "living room",
        "வரவேற்பு": "reception",
        "நூலகம்": "library",
        "ஆய்வகம்": "laboratory",
    }

    def __init__(self, place_resolver: Optional[PlaceResolver] = None):
        self.place_resolver = place_resolver or PlaceResolver()

    def classify(self, text: str) -> IntentClassificationResult:
        """
        Classifies incoming speech transcript into a mutually exclusive intent category.
        Ensures strict boundary between conversation and physical motion.
        """
        if not text or not text.strip():
            return IntentClassificationResult(
                intent=IntentType.UNKNOWN,
                confidence=0.0,
                reason="Empty transcript",
            )

        clean = text.strip()
        lower = clean.lower()

        # 1. Strip wake words for command analysis
        cmd_text = self.WAKE_PREFIX_RE.sub("", clean).strip()
        cmd_text_lower = cmd_text.lower() if cmd_text else lower

        # 2. EMERGENCY STOP (Priority 1)
        for pat in self.EMERGENCY_STOP_PATTERNS:
            if re.search(pat, lower) or re.search(pat, cmd_text_lower):
                return IntentClassificationResult(
                    intent=IntentType.EMERGENCY_STOP,
                    confidence=1.0,
                    reason="Matched Emergency Stop pattern",
                )

        # 3. CANCEL NAVIGATION (Priority 2)
        for pat in self.CANCEL_PATTERNS:
            if re.search(pat, lower) or re.search(pat, cmd_text_lower):
                return IntentClassificationResult(
                    intent=IntentType.CANCEL_NAVIGATION,
                    confidence=1.0,
                    reason="Matched Cancel Navigation pattern",
                )

        # 4. FOLLOW PERSON (Priority 3)
        for pat in self.FOLLOW_PATTERNS:
            if re.search(pat, lower) or re.search(pat, cmd_text_lower):
                return IntentClassificationResult(
                    intent=IntentType.FOLLOW_PERSON,
                    confidence=0.95,
                    reason="Matched Follow Person pattern",
                )

        # 5. GREETING (Priority 4)
        for pat in self.GREET_PATTERNS:
            if re.search(pat, lower) or re.search(pat, cmd_text_lower):
                return IntentClassificationResult(
                    intent=IntentType.GREETING,
                    confidence=0.99,
                    reason="Matched Greeting pattern",
                )

        # 6. THANK YOU / GRATITUDE (Priority 5)
        for pat in self.THANK_PATTERNS:
            if re.search(pat, lower) or re.search(pat, cmd_text_lower):
                # Ensure it's not a compound command like "Thank you, go to the bedroom"
                if not any(re.search(p, cmd_text_lower) for p in (r"\bgo\s+to\b", r"\bnavigate\b", r"\btake\s+me\b")):
                    return IntentClassificationResult(
                        intent=IntentType.THANK_YOU,
                        confidence=0.98,
                        reason="Matched Gratitude pattern",
                    )

        # 6B. CHECK FOR INITIAL EXPLICIT NAVIGATION IMPERATIVE
        # (e.g. "Go to the bedroom. I am very proud of you." or "Navigate to the kitchen. Navigate to the kitchen.")
        # If the leading sentence is an unambiguous navigation command, process it before conversational markers.
        first_sentence = re.split(r"[.!?]\s+", clean)[0].strip()
        first_cmd_text = self.WAKE_PREFIX_RE.sub("", first_sentence).strip()
        compound_check = self.COMPOUND_SPLIT_RE.split(cmd_text if cmd_text else clean)
        if len(compound_check) < 2:
            single_first_res = self._extract_single_destination(first_cmd_text if first_cmd_text else first_sentence)
            if single_first_res and single_first_res.is_navigation:
                return single_first_res

        # 7. CONVERSATIONAL FALSE-POSITIVE FILTER (Crucial Safety Barrier)
        # Check if the text matches conversational markers before testing for navigation
        for marker in self.CONVERSATIONAL_MARKERS:
            if re.search(marker, lower):
                return IntentClassificationResult(
                    intent=IntentType.CONVERSATIONAL,
                    confidence=0.95,
                    reason=f"Matched conversational marker: {marker}",
                    is_navigation=False,
                )

        # 8. QUESTION / INFORMATIONAL QUERY
        # Questions containing room names ("Tell me about the bedroom", "Where is the kitchen?")
        is_question = lower.endswith("?") or any(re.search(p, lower) for p in self.QUESTION_STARTERS)
        if is_question:
            # Verify it's not a polite navigation command ("Could you take me to the kitchen?")
            is_polite_nav = any(
                re.search(p, lower)
                for p in (
                    r"\b(?:take\s+me\s+to|take\s+us\s+to|lead\s+me\s+to|guide\s+me\s+to)\b",
                    r"\b(?:can\s+you\s+go\s+to|could\s+you\s+go\s+to)\b",
                    r"\b(?:can\s+you\s+navigate|could\s+you\s+navigate)\b"
                )
            )
            if not is_polite_nav:
                return IntentClassificationResult(
                    intent=IntentType.QUESTION,
                    confidence=0.95,
                    reason="Matched Question pattern",
                    is_navigation=False,
                )

        # 9. COMPOUND NAVIGATION
        compound_legs = self.COMPOUND_SPLIT_RE.split(cmd_text if cmd_text else clean)
        if len(compound_legs) >= 2:
            resolved_places = []
            canonical_names = []
            all_valid = True

            # First leg must have navigation imperative
            first_leg = compound_legs[0].strip()
            first_res = self._extract_single_destination(first_leg)
            if first_res and first_res.place_info:
                resolved_places.append(first_res.place_info)
                canonical_names.append(first_res.place_info.name)

                # Subsequent legs can either have navigation verbs or be elliptical place names
                for leg in compound_legs[1:]:
                    leg_clean = leg.strip()
                    leg_res = self._extract_single_destination(leg_clean)
                    if leg_res and leg_res.place_info:
                        resolved_places.append(leg_res.place_info)
                        canonical_names.append(leg_res.place_info.name)
                    else:
                        # Check elliptical place name without repeating navigation verb
                        leg_cand = re.sub(r"^(?:the|to|a|an)\s+", "", leg_clean, flags=re.IGNORECASE).rstrip(".,;:!?-").strip()
                        place = self.place_resolver.resolve_place(leg_cand)
                        if place:
                            resolved_places.append(place)
                            canonical_names.append(place.name)
                        else:
                            all_valid = False
                            break

                if all_valid and len(resolved_places) >= 2:
                    return IntentClassificationResult(
                        intent=IntentType.COMPOUND_NAVIGATION,
                        confidence=0.98,
                        canonical_destinations=canonical_names,
                        places_info=resolved_places,
                        is_navigation=True,
                        reason="Resolved valid sequential compound navigation legs",
                    )

        # 10. SINGLE-LEG NAVIGATE_TO
        single_res = self._extract_single_destination(cmd_text if cmd_text else clean)
        if single_res:
            return single_res

        # 11. GENERAL CONVERSATION (Default safe fallback)
        return IntentClassificationResult(
            intent=IntentType.CONVERSATIONAL,
            confidence=0.85,
            reason="Unmatched conversational dialogue",
            is_navigation=False,
        )

    def _extract_single_destination(self, text: str) -> Optional[IntentClassificationResult]:
        """
        Attempts to extract a strictly validated navigation destination from an utterance leg.
        Guarantees that a place name mentioned in passing is NEVER extracted without a navigation imperative.
        """
        clean = text.strip()
        lower = clean.lower()

        # Check for Tamil navigation pattern
        for tamil_place, eng_key in self.TAMIL_PLACE_MAP.items():
            if tamil_place in clean and any(v in clean for v in ("போ", "செல்")):
                place = self.place_resolver.resolve_place(eng_key)
                if place:
                    return IntentClassificationResult(
                        intent=IntentType.NAVIGATE_TO,
                        confidence=0.99,
                        canonical_destination=place.name,
                        place_info=place,
                        raw_destination_phrase=tamil_place,
                        is_navigation=True,
                        reason="Matched Tamil navigation imperative",
                    )

        # Check English Navigation Patterns
        matched_candidate: Optional[str] = None
        for pat in self.NAVIGATE_PATTERNS:
            m = pat.search(clean)
            if m:
                raw_cand = m.group(1).strip()
                # Stop candidate at sentence boundaries or conversational clauses
                raw_cand = re.split(r"[.!?]|(?:\b(?:because|since|while|so|i\s+am|we\s+are)\b)", raw_cand, flags=re.IGNORECASE)[0].strip()
                matched_candidate = raw_cand
                break

        if not matched_candidate:
            return None

        clean_cand = matched_candidate.rstrip(".,;:!?-").strip()
        words = clean_cand.lower().split()
        if not words:
            return None

        # Disqualify if candidate is merely a navigation verb itself
        if clean_cand.lower() in self.NAVIGATION_VERBS:
            return None

        # Check Ambiguity first (e.g. "room", "desk")
        ambig_prompt = self.place_resolver.check_ambiguity(clean_cand)
        if ambig_prompt:
            return IntentClassificationResult(
                intent=IntentType.NAVIGATE_TO,
                confidence=0.70,
                raw_destination_phrase=clean_cand,
                clarification_prompt=ambig_prompt,
                is_navigation=False,
                reason="Ambiguous destination requiring user clarification",
            )

        # Match against canonical destinations in places.json
        place = self.place_resolver.resolve_place(clean_cand)
        if place:
            return IntentClassificationResult(
                intent=IntentType.NAVIGATE_TO,
                confidence=0.98,
                canonical_destination=place.name,
                place_info=place,
                raw_destination_phrase=clean_cand,
                is_navigation=True,
                reason=f"Matched canonical destination '{place.name}'",
            )

        # Disqualify if candidate begins with an action verb (e.g., "be in the kitchen", "change the video")
        if words[0] in self.DISQUALIFYING_ACTION_VERBS:
            return None

        # Navigation imperative present, but destination unknown in map
        return IntentClassificationResult(
            intent=IntentType.NAVIGATE_TO,
            confidence=0.90,
            raw_destination_phrase=clean_cand,
            is_navigation=False,
            reason=f"Destination '{clean_cand}' is not present in active map",
        )
