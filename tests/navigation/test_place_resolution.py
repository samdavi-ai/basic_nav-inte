#!/usr/bin/env python3
"""
Unit and Contract tests for Authoritative Place Resolution & Voice Navigation Intents.
Verifies:
  - Canonical landmark and room resolution (Library, Classroom, Laboratory, Study Room, Kitchen, Bedroom, Living Room)
  - Disambiguation of ambiguous destinations ('room', 'desk') with zero commands issued.
  - Rejection of out-of-bounds locations (Gate, Mall, Auditorium) with zero commands issued.
  - Rejection of non-existent destinations (e.g. Moon Lab) with zero commands issued.
  - Complete 17-phrase deterministic voice intent routing (NAVIGATE, STOP, PAUSE, RESUME, CANCEL, STATUS, RETURN).
"""
import unittest
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(ROOT / "graceemo_interaction"))

from core.place_resolver import PlaceResolver, PlaceInfo
from core.fast_path import FastPathMatcher
from core.state_machine import Expression


class TestPlaceResolutionAndVoiceIntents(unittest.TestCase):
    def setUp(self):
        self.resolver = PlaceResolver()
        self.fast_path = FastPathMatcher()

    def test_canonical_destinations_resolution(self):
        # 1. Library (Apartment Study/Library Desk)
        p = self.resolver.resolve_place("Take me to the library")
        self.assertIsNotNone(p)
        self.assertEqual(p.name, "Library")
        self.assertEqual(p.x, 4.5)
        self.assertEqual(p.y, 3.0)

        # 2. Classroom
        p = self.resolver.resolve_place("navigate to the classroom")
        self.assertIsNotNone(p)
        self.assertEqual(p.name, "Classroom")
        self.assertEqual(p.x, 4.5)
        self.assertEqual(p.y, 3.0)

        # 3. Laboratory / Lab
        p = self.resolver.resolve_place("Take me to the laboratory")
        self.assertIsNotNone(p)
        self.assertEqual(p.name, "Robotics Research Lab")
        self.assertEqual(p.x, 2.5)
        self.assertEqual(p.y, 2.0)

        # 4. Study Room
        p = self.resolver.resolve_place("go to the study room")
        self.assertIsNotNone(p)
        self.assertEqual(p.name, "Study Room")
        self.assertEqual(p.x, 4.5)
        self.assertEqual(p.y, 3.0)

        # 5. Kitchen & Dining
        p = self.resolver.resolve_place("head to the kitchen")
        self.assertIsNotNone(p)
        self.assertEqual(p.name, "Kitchen & Dining")
        self.assertEqual(p.x, -4.0)
        self.assertEqual(p.y, -2.5)

        # 6. Master Bedroom
        p = self.resolver.resolve_place("walk to the master bedroom")
        self.assertIsNotNone(p)
        self.assertEqual(p.name, "Master Bedroom")
        self.assertEqual(p.x, -4.0)
        self.assertEqual(p.y, 2.2)

        # 7. Living Room
        p = self.resolver.resolve_place("drive to the living room")
        self.assertIsNotNone(p)
        self.assertEqual(p.name, "Living Room")
        self.assertEqual(p.x, 4.0)
        self.assertEqual(p.y, -2.5)

        # 8. Starting Point / Home
        p = self.resolver.resolve_place("starting point")
        self.assertIsNotNone(p)
        self.assertEqual(p.name, "Starting Point")
        self.assertEqual(p.x, 0.0)
        self.assertEqual(p.y, 0.0)

    def test_ambiguous_destination_clarification(self):
        """Ambiguous queries ('room', 'desk') must prompt for clarification with ZERO robot commands."""
        # Test 'the room'
        res = self.fast_path.match("Take me to the room")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "AMBIGUOUS_DESTINATION")
        self.assertEqual(res.expression, Expression.INTENT)
        self.assertEqual(len(res.semantic_actions), 0, "Ambiguous destination MUST emit ZERO semantic actions")
        self.assertIn("Which room would you like to visit", res.response_text)

        # Test 'the desk'
        res = self.fast_path.match("Go to the desk")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "AMBIGUOUS_DESTINATION")
        self.assertEqual(len(res.semantic_actions), 0)
        self.assertIn("Which desk would you like to visit", res.response_text)

    def test_out_of_bounds_destination_rejection(self):
        """Campus locations outside apartment boundaries (e.g. Gate, Mall) must be rejected with ZERO robot commands."""
        res = self.fast_path.match("Navigate to the main campus gate")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "UNREACHABLE_DESTINATION")
        self.assertEqual(res.expression, Expression.SAD)
        self.assertEqual(len(res.semantic_actions), 0, "Out-of-bounds destination MUST emit ZERO semantic actions")
        self.assertIn("outside the active map boundaries", res.response_text)

    def test_unknown_location_rejection(self):
        """Unknown locations (e.g. Moon Lab) must be rejected with ZERO robot commands."""
        p, unknown_phrase = self.resolver.resolve_navigation_request("Take me to the Moon Lab.")
        self.assertIsNone(p)
        self.assertIsNotNone(unknown_phrase)
        self.assertIn("moon lab", unknown_phrase.lower())

        res = self.fast_path.match("Take me to the Moon Lab.")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "UNKNOWN_DESTINATION")
        self.assertEqual(res.expression, Expression.SAD)
        self.assertEqual(len(res.semantic_actions), 0, "Unknown location MUST emit ZERO semantic actions")
        self.assertIn("cannot find", res.response_text)
        self.assertIn("moon lab", res.response_text.lower())

    def test_voice_navigate_intents(self):
        # 1. "Go to the library"
        res = self.fast_path.match("Go to the library")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "NAVIGATE")
        self.assertEqual(len(res.semantic_actions), 1)
        self.assertEqual(res.semantic_actions[0]["action"], "NavigateTo")
        self.assertEqual(res.semantic_actions[0]["params"]["target_name"], "Library")
        self.assertEqual(res.semantic_actions[0]["params"]["x"], 4.5)
        self.assertEqual(res.semantic_actions[0]["params"]["y"], 3.0)

        # 2. "Navigate to the classroom"
        res = self.fast_path.match("Navigate to the classroom")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "NAVIGATE")
        self.assertEqual(res.semantic_actions[0]["params"]["target_name"], "Classroom")
        self.assertEqual(res.semantic_actions[0]["params"]["x"], 4.5)

        # 3. "Take me to the laboratory"
        res = self.fast_path.match("Take me to the laboratory")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "NAVIGATE")
        self.assertEqual(res.semantic_actions[0]["params"]["target_name"], "Robotics Research Lab")
        self.assertEqual(res.semantic_actions[0]["params"]["x"], 2.5)
        self.assertEqual(res.semantic_actions[0]["params"]["y"], 2.0)

        # 4. "Go to the study room"
        res = self.fast_path.match("Go to the study room")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "NAVIGATE")
        self.assertEqual(res.semantic_actions[0]["params"]["target_name"], "Study Room")
        self.assertEqual(res.semantic_actions[0]["params"]["x"], 4.5)
        self.assertEqual(res.semantic_actions[0]["params"]["y"], 3.0)

        # 5. "Hi Gresimo, move to the bedroom." (Conversational wake prefix + 'move to')
        res = self.fast_path.match("Hi Gresimo, move to the bedroom.")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "NAVIGATE")
        self.assertEqual(res.semantic_actions[0]["params"]["target_name"], "Master Bedroom")
        self.assertEqual(res.semantic_actions[0]["params"]["x"], -4.0)
        self.assertEqual(res.semantic_actions[0]["params"]["y"], 2.2)

        # 6. "move to the bedroom"
        res = self.fast_path.match("move to the bedroom")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "NAVIGATE")
        self.assertEqual(res.semantic_actions[0]["params"]["target_name"], "Master Bedroom")
        self.assertEqual(res.semantic_actions[0]["params"]["x"], -4.0)
        self.assertEqual(res.semantic_actions[0]["params"]["y"], 2.2)

        # 7. "Hi GraceEMO, go to the bedroom" (Wake phrase prefix followed by destination command)
        res = self.fast_path.match("Hi GraceEMO, go to the bedroom")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "NAVIGATE")
        self.assertEqual(res.semantic_actions[0]["params"]["target_name"], "Master Bedroom")
        self.assertEqual(res.semantic_actions[0]["params"]["x"], -4.0)
        self.assertEqual(res.semantic_actions[0]["params"]["y"], 2.2)

    def test_voice_stop_intents(self):
        # 1. "Stop"
        res = self.fast_path.match("Stop")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "STOP")
        self.assertTrue(res.is_emergency)
        self.assertEqual(res.semantic_actions[0]["action"], "Stop")

        # 2. "Stop the robot"
        res = self.fast_path.match("Stop the robot")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "STOP")
        self.assertTrue(res.is_emergency)
        self.assertEqual(res.semantic_actions[0]["action"], "Stop")

    def test_voice_pause_intents(self):
        # 1. "Pause navigation"
        res = self.fast_path.match("Pause navigation")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "PAUSE")
        self.assertFalse(res.is_emergency)
        self.assertEqual(res.semantic_actions[0]["action"], "Pause")

        # 2. "Wait here"
        res = self.fast_path.match("Wait here")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "PAUSE")
        self.assertEqual(res.semantic_actions[0]["action"], "Pause")

    def test_voice_resume_intents(self):
        # 1. "Resume navigation"
        res = self.fast_path.match("Resume navigation")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "RESUME")
        self.assertEqual(res.semantic_actions[0]["action"], "Resume")

        # 2. "Continue to the destination"
        res = self.fast_path.match("Continue to the destination")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "RESUME")
        self.assertEqual(res.semantic_actions[0]["action"], "Resume")

    def test_voice_cancel_intents(self):
        # 1. "Cancel mission"
        res = self.fast_path.match("Cancel mission")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "CANCEL")
        self.assertEqual(res.semantic_actions[0]["action"], "Cancel")

        # 2. "Cancel navigation"
        res = self.fast_path.match("Cancel navigation")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "CANCEL")
        self.assertEqual(res.semantic_actions[0]["action"], "Cancel")

    def test_voice_status_intents(self):
        # 1. "Where are you going?"
        res = self.fast_path.match("Where are you going?")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "STATUS")
        self.assertEqual(res.semantic_actions[0]["action"], "StatusQuery")

        # 2. "What's your current location?"
        res = self.fast_path.match("What's your current location?")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "STATUS")
        self.assertEqual(res.semantic_actions[0]["action"], "StatusQuery")

        # 3. "What's the navigation status?"
        res = self.fast_path.match("What's the navigation status?")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "STATUS")
        self.assertEqual(res.semantic_actions[0]["action"], "StatusQuery")

    def test_voice_return_intents(self):
        # 1. "Return to home"
        res = self.fast_path.match("Return to home")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "RETURN_HOME")
        self.assertEqual(res.semantic_actions[0]["action"], "ReturnHome")

        # 2. "Go back to the starting point"
        res = self.fast_path.match("Go back to the starting point")
        self.assertTrue(res.matched)
        self.assertEqual(res.intent, "RETURN_HOME")
        self.assertEqual(res.semantic_actions[0]["action"], "ReturnHome")


if __name__ == "__main__":
    unittest.main()
