#!/usr/bin/env python3
"""
Unit tests for Fast-Path Intent & Place Dispatcher (FastPathMatcher).
Validates sub-millisecond deterministic matching for STOP, CANCEL, PAUSE, RESUME,
RETURN, STATUS, GREET, IDENTITY, and campus/apartment place extraction.
"""
import unittest
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
interaction_dir = ROOT / "vira_interaction" if (ROOT / "vira_interaction").exists() else ROOT / "graceemo_interaction"
sys.path.insert(0, str(interaction_dir))

from core.fast_path import FastPathMatcher
from core.state_machine import Expression


class TestFastPathMatcher(unittest.TestCase):
    def setUp(self):
        self.matcher = FastPathMatcher()

    def test_emergency_stop_intents(self):
        stop_phrases = ["Stop", "stop now", "please halt", "emergency freeze", "Stop the robot"]
        for phrase in stop_phrases:
            res = self.matcher.match(phrase)
            self.assertTrue(res.matched, f"Failed for phrase: '{phrase}'")
            self.assertEqual(res.intent, "STOP")
            self.assertTrue(res.is_emergency)
            self.assertEqual(len(res.semantic_actions), 1)
            self.assertEqual(res.semantic_actions[0]["action"], "Stop")

    def test_cancel_intents(self):
        cancel_phrases = ["Cancel mission", "cancel navigation", "abort mission", "Cancel"]
        for phrase in cancel_phrases:
            res = self.matcher.match(phrase)
            self.assertTrue(res.matched, f"Failed for phrase: '{phrase}'")
            self.assertEqual(res.intent, "CANCEL")
            self.assertEqual(res.semantic_actions[0]["action"], "Cancel")

    def test_pause_intents(self):
        pause_phrases = ["Pause navigation", "Wait here", "pause mission", "hold position", "WAIT"]
        for phrase in pause_phrases:
            res = self.matcher.match(phrase)
            self.assertTrue(res.matched, f"Failed for phrase: '{phrase}'")
            self.assertEqual(res.intent, "PAUSE")
            self.assertEqual(res.semantic_actions[0]["action"], "Pause")

    def test_resume_intents(self):
        resume_phrases = ["Resume navigation", "continue to the destination", "continue mission"]
        for phrase in resume_phrases:
            res = self.matcher.match(phrase)
            self.assertTrue(res.matched, f"Failed for phrase: '{phrase}'")
            self.assertEqual(res.intent, "RESUME")
            self.assertEqual(res.semantic_actions[0]["action"], "Resume")

    def test_return_home_intents(self):
        return_phrases = ["Return to home", "go back to the starting point", "return to reception"]
        for phrase in return_phrases:
            res = self.matcher.match(phrase)
            self.assertTrue(res.matched, f"Failed for phrase: '{phrase}'")
            self.assertEqual(res.intent, "RETURN_HOME")
            self.assertEqual(res.semantic_actions[0]["action"], "ReturnHome")

    def test_status_intents(self):
        status_phrases = ["Where are you going?", "what's your current location?", "what's the navigation status?"]
        for phrase in status_phrases:
            res = self.matcher.match(phrase)
            self.assertTrue(res.matched, f"Failed for phrase: '{phrase}'")
            self.assertEqual(res.intent, "STATUS")
            self.assertEqual(res.semantic_actions[0]["action"], "StatusQuery")

    def test_greeting_intents(self):
        greetings = ["Hello GraceEMO", "Hi", "Hey there", "good morning", "Wave"]
        for phrase in greetings:
            res = self.matcher.match(phrase)
            self.assertTrue(res.matched, f"Failed for greeting: '{phrase}'")
            self.assertEqual(res.intent, "GREET")
            self.assertEqual(res.expression, Expression.HAPPY)

    def test_identity_intents(self):
        queries = ["who are you", "what is your name", "what's your name"]
        for phrase in queries:
            res = self.matcher.match(phrase)
            self.assertTrue(res.matched, f"Failed for identity: '{phrase}'")
            self.assertEqual(res.intent, "IDENTITY")
            self.assertIn("GraceEMO", res.response_text)
            self.assertTrue(
                "Vice Chancellor" in res.response_text
                or "academic leader" in res.response_text
            )

    def test_campus_navigation_places(self):
        test_cases = [
            ("Take me to the Robotics Lab", 2.5, 2.0, "Robotics Research Lab"),
            ("Go to the library", 4.5, 3.0, "Library"),
            ("Navigate to classroom", 4.5, 3.0, "Classroom"),
            ("Head to the kitchen", -4.0, -2.5, "Kitchen & Dining"),
            ("Navigate to master bedroom", -4.0, 2.2, "Master Bedroom"),
            ("Walk to home study", 4.5, 3.0, "Study Room"),
        ]

        for utterance, exp_x, exp_y, exp_name in test_cases:
            res = self.matcher.match(utterance)
            self.assertTrue(res.matched, f"Failed to match navigation for '{utterance}'")
            self.assertEqual(res.intent, "NAVIGATE")
            self.assertEqual(res.expression, Expression.FOCUS)

            nav_action = [a for a in res.semantic_actions if a["action"] == "NavigateTo"]
            self.assertEqual(len(nav_action), 1)
            params = nav_action[0]["params"]
            self.assertEqual(params["x"], exp_x)
            self.assertEqual(params["y"], exp_y)
            self.assertEqual(params["target_name"], exp_name)

    def test_patrol_all_rooms(self):
        patrol_phrases = [
            "visit each and every room",
            "visit all rooms",
            "tour the apartment",
            "patrol each and every room",
            "Hi GraceEMO, move to the bedroom, kitchen and every room, like show me the every room by navigating each one of them",
        ]
        for phrase in patrol_phrases:
            res = self.matcher.match(phrase)
            self.assertTrue(res.matched, f"Failed to match patrol for '{phrase}'")
            self.assertIn(res.intent, ("PATROL_ROOMS", "COMPOUND_NAVIGATION"))
            self.assertTrue(len(res.semantic_actions) > 0)
            self.assertEqual(res.semantic_actions[0]["action"], "StartMission")
            wps = res.semantic_actions[0]["params"]["waypoints"]
            self.assertGreaterEqual(len(wps), 2)

    def test_compound_navigation(self):
        compound_phrases = [
            ("Navigate to bedroom then kitchen", ["Master Bedroom", "Kitchen & Dining"]),
            ("Go to study room and then living room", ["Study Room", "Living Room"]),
        ]
        for phrase, expected_names in compound_phrases:
            res = self.matcher.match(phrase)
            self.assertTrue(res.matched, f"Failed to match compound navigation for '{phrase}'")
            self.assertEqual(res.intent, "COMPOUND_NAVIGATION")
            self.assertEqual(res.semantic_actions[0]["action"], "StartMission")
            wps = res.semantic_actions[0]["params"]["waypoints"]
            self.assertEqual(len(wps), len(expected_names))
            for i, name in enumerate(expected_names):
                self.assertEqual(wps[i]["name"], name)

    def test_unmatched_falls_through(self):
        unmatched = [
            "What is quantum computing?",
            "Can you explain the admissions criteria for B.Tech in CSE?",
            "Tell me about Dr. Mohit Arora's latest research publication.",
        ]
        for phrase in unmatched:
            res = self.matcher.match(phrase)
            self.assertFalse(res.matched, f"Phrase '{phrase}' should not have matched fast-path")


if __name__ == "__main__":
    unittest.main()
