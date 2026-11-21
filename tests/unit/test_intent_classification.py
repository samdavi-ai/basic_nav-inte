#!/usr/bin/env python3
"""
Regression Tests: Strict Intent Classification & Destination Extraction
========================================================================
Validates that conversational speech is NEVER misinterpreted as a navigation command,
while genuine navigation commands (including natural-language phrasing) continue working.

Covers Test Categories A-J from Phases 6 & 7:
  A. Conversational False Positives (including the exact observed bug)
  B. Short English Greetings and Acknowledgements
  C. Genuine English Navigation Commands
  D. English Destination Extraction
  E. Unknown Destinations
  F. Compound English Navigation
  G. Speech during an Active Navigation Mission
  H. Explicit Cancellation
  I. Ambiguous English Speech
  J. Repeated or Noisy Transcripts
"""

import pytest
from vira_interaction.core.gateway import InteractionGateway, GatewayResult
from vira_interaction.core.intent_classifier import IntentClassifier, IntentType
from vira_interaction.core.place_resolver import PlaceResolver


@pytest.fixture
def gateway():
    gw = InteractionGateway()
    gw.safety_gate.reset_emergency_stop()
    return gw


@pytest.fixture
def classifier():
    return IntentClassifier()


# =============================================================================
# Category A: Conversational False Positives
# =============================================================================

@pytest.mark.parametrize(
    "text",
    [
        # The primary observed bug:
        "This is a beautiful place to be in the kitchen. I am very proud of you. So, I am very proud of you.",
        # Location mentioned in social praise or welcome:
        "Gretchenberg, welcome to the master bedroom.",
        "I am proud of you.",
        "I'm not sure how to go.",
        "Second part is the food gate.",
        "One of them.",
        "This kitchen looks beautiful.",
        "Tell me about the master bedroom.",
        "What is inside the laboratory?",
        "I was in the library yesterday.",
        "My bedroom is very beautiful.",
        "I like the bedroom.",
        "I am going to cook in the kitchen.",
        "There is a classroom near the reception.",
        "We have a spacious kitchen.",
        "The study room is very quiet today.",
    ],
)
def test_category_a_conversational_false_positives(gateway: InteractionGateway, text: str):
    """Assert that conversational utterances NEVER dispatch any navigation goals."""
    res: GatewayResult = gateway.process_text(text)
    
    # 1. Assert no navigation actions
    nav_actions = [
        a for a in res.actions
        if a.get("action") in ("NavigateTo", "NavigateToRoom", "StartMission", "PatrolRooms", "NavigateWaypoints")
    ]
    assert len(nav_actions) == 0, f"Conversational utterance emitted navigation action: {nav_actions}"

    # 2. Assert no unknown destination map error
    assert "cannot find" not in res.response.lower(), f"Unexpected 'cannot find destination' error in: {res.response}"
    assert "not in my map" not in res.response.lower()

    # 3. Assert conversational response or informational query
    assert res.response, "Gateway must return a spoken response to conversational speech"


# =============================================================================
# Category B: Short English Greetings and Acknowledgements
# =============================================================================

@pytest.mark.parametrize(
    "text,expected_intent",
    [
        ("Hello.", "GREET"),
        ("Hi.", "GREET"),
        ("Hey.", "GREET"),
        ("Good morning.", "GREET"),
        ("Good afternoon.", "GREET"),
        ("Thank you.", "GRATITUDE"),
        ("Thanks.", "GRATITUDE"),
        ("Thank you very much.", "GRATITUDE"),
        ("Thanks a lot.", "GRATITUDE"),
    ],
)
def test_category_b_short_greetings_and_acknowledgements(gateway: InteractionGateway, text: str, expected_intent: str):
    """Assert short polite greetings and acknowledgements produce zero navigation commands."""
    res = gateway.process_text(text)
    assert res.intent == expected_intent
    nav_actions = [
        a for a in res.actions
        if a.get("action") in ("NavigateTo", "NavigateToRoom", "StartMission", "PatrolRooms", "NavigateWaypoints")
    ]
    assert len(nav_actions) == 0


# =============================================================================
# Category C & D: Genuine English Navigation Commands & Destination Extraction
# =============================================================================

@pytest.mark.parametrize(
    "text,expected_destination,expected_x,expected_y",
    [
        # The secondary observed input:
        ("I'm going to go to the master bedroom.", "Master Bedroom", -4.0, 2.2),
        ("Go to the kitchen.", "Kitchen & Dining", -4.0, -2.5),
        ("Please take me to the bedroom.", "Master Bedroom", -4.0, 2.2),
        ("Navigate to the master bedroom.", "Master Bedroom", -4.0, 2.2),
        ("I want you to go to the library.", "Library", 4.5, 3.0),
        ("ViRa, let's visit the kitchen.", "Kitchen & Dining", -4.0, -2.5),
        ("Take me to the reception.", "Welcome Reception Desk", 2.5, -2.5),
        ("Move to the classroom.", "Classroom", 4.5, 3.0),
        ("Please visit the laboratory.", "Robotics Research Lab", 2.5, 2.0),
        ("I want you to go to the reception.", "Welcome Reception Desk", 2.5, -2.5),
        ("ViRa, let's go to the kitchen.", "Kitchen & Dining", -4.0, -2.5),
        # Tamil Navigation Commands
        ("படுக்கையறைக்குப் போ.", "Master Bedroom", -4.0, 2.2),
        ("சமையலறைக்குச் செல்.", "Kitchen & Dining", -4.0, -2.5),
    ],
)
def test_category_c_and_d_genuine_navigation_commands(
    gateway: InteractionGateway, text: str, expected_destination: str, expected_x: float, expected_y: float
):
    """Assert genuine navigation imperatives extract and dispatch exact canonical destination coordinates."""
    res = gateway.process_text(text)
    assert res.intent == "NAVIGATE"
    assert len(res.actions) == 1
    action = res.actions[0]
    assert action["action"] == "NavigateTo"
    params = action["params"]
    assert params["target_name"] == expected_destination
    assert params["x"] == expected_x
    assert params["y"] == expected_y
    assert expected_destination in res.response


# =============================================================================
# Category E: Unknown Destinations
# =============================================================================

@pytest.mark.parametrize(
    "text,unknown_target",
    [
        ("Navigate to the moon.", "moon"),
        ("Go to Paris.", "Paris"),
        ("Please take me to the swimming pool.", "swimming pool"),
    ],
)
def test_category_e_unknown_destinations(gateway: InteractionGateway, text: str, unknown_target: str):
    """Assert commands targeting places not in places.json are safely rejected without movement."""
    res = gateway.process_text(text)
    assert res.intent == "UNKNOWN_DESTINATION"
    assert len(res.actions) == 0, "No action should be dispatched for unknown map destinations"
    assert f"cannot find '{unknown_target.lower()}'" in res.response.lower()


# =============================================================================
# Category F: Compound English Navigation
# =============================================================================

@pytest.mark.parametrize(
    "text,expected_destinations",
    [
        ("Visit the laboratory, then go to the classroom.", ["Robotics Research Lab", "Classroom"]),
        ("Go to the kitchen, then visit the bedroom.", ["Kitchen & Dining", "Master Bedroom"]),
        ("Go to study room then living room.", ["Study Room", "Living Room"]),
    ],
)
def test_category_f_compound_navigation(gateway: InteractionGateway, text: str, expected_destinations: list):
    """Assert multi-waypoint commands correctly preserve ordering and generate StartMission actions."""
    res = gateway.process_text(text)
    assert res.intent == "COMPOUND_NAVIGATION"
    assert len(res.actions) == 1
    action = res.actions[0]
    assert action["action"] == "StartMission"
    waypoints = action["params"]["waypoints"]
    assert len(waypoints) == len(expected_destinations)
    for idx, expected_name in enumerate(expected_destinations):
        assert waypoints[idx]["name"] == expected_name


# =============================================================================
# Category G: Speech During an Active Navigation Mission
# =============================================================================

def test_category_g_speech_during_active_mission(gateway: InteractionGateway):
    """Assert conversational speech while navigating does not alter or abort the active destination."""
    # 1. Start navigation
    res_nav = gateway.process_text("Go to the kitchen.")
    assert res_nav.intent == "NAVIGATE"
    assert len(res_nav.actions) == 1

    # 2. Conversational comment while robot is in transit
    res_conv = gateway.process_text("This is a beautiful place to be in the kitchen. I am very proud of you.")
    # Must NOT emit any new navigation actions or route change
    conv_nav_actions = [
        a for a in res_conv.actions
        if a.get("action") in ("NavigateTo", "NavigateToRoom", "StartMission", "PatrolRooms")
    ]
    assert len(conv_nav_actions) == 0


# =============================================================================
# Category H: Explicit Cancellation & Emergency Stop
# =============================================================================

@pytest.mark.parametrize(
    "text,expected_action",
    [
        ("Cancel navigation.", "Cancel"),
        ("Cancel the mission.", "Cancel"),
        ("Abort mission.", "Cancel"),
        ("Stop immediately.", "Stop"),
        ("Emergency stop.", "Stop"),
    ],
)
def test_category_h_explicit_cancellation_and_estop(gateway: InteractionGateway, text: str, expected_action: str):
    """Assert explicit cancellation and emergency stop always succeed."""
    res = gateway.process_text(text)
    assert len(res.actions) == 1
    assert res.actions[0]["action"] == expected_action


# =============================================================================
# Category I: Ambiguous English Speech
# =============================================================================

@pytest.mark.parametrize(
    "text,expected_clarification_sub",
    [
        ("Go to the room.", "Which room would you like to visit"),
        ("Take me to the desk.", "Which desk would you like to visit"),
    ],
)
def test_category_i_ambiguous_speech(gateway: InteractionGateway, text: str, expected_clarification_sub: str):
    """Assert ambiguous destination commands prompt for clarification and do not move the robot."""
    res = gateway.process_text(text)
    assert res.intent == "AMBIGUOUS_DESTINATION"
    assert len(res.actions) == 0
    assert expected_clarification_sub in res.response


# =============================================================================
# Category J: Repeated or Noisy English Transcripts
# =============================================================================

@pytest.mark.parametrize(
    "text",
    [
        "Navigate to the kitchen. Navigate to the kitchen.",
        "Go to the bedroom. I am very proud of you.",
        "Hey robot, could you please go to the library?",
    ],
)
def test_category_j_repeated_and_prefixed_transcripts(gateway: InteractionGateway, text: str):
    """Assert deduplication and prefix stripping cleanly resolve the underlying command."""
    res = gateway.process_text(text)
    nav_actions = [a for a in res.actions if a.get("action") == "NavigateTo"]
    assert len(nav_actions) == 1


# =============================================================================
# Category K: Natural Plural and Modal Navigation Phrases
# =============================================================================

@pytest.mark.parametrize(
    "text,expected_place",
    [
        ("We're going to go to kitchen.", "Kitchen & Dining"),
        ("We are going to the kitchen.", "Kitchen & Dining"),
        ("We're going to the master bedroom.", "Master Bedroom"),
        ("Can we go to the study room?", "Study Room"),
        ("We want to go to living room.", "Living Room"),
        ("Let us go to the kitchen.", "Kitchen & Dining"),
    ],
)
def test_category_k_plural_and_modal_navigation(gateway: InteractionGateway, text: str, expected_place: str):
    """Assert plural human navigation intents ('we are going to...', 'can we go...') resolve to NavigateTo."""
    res = gateway.process_text(text)
    assert res.intent == "NAVIGATE"
    nav_actions = [a for a in res.actions if a.get("action") == "NavigateTo"]
    assert len(nav_actions) == 1
    assert nav_actions[0]["params"]["target_name"] == expected_place
