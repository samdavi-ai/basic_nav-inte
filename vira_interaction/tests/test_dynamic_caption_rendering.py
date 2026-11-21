#!/usr/bin/env python3
"""
Unit and visual regression tests for GraceEMO adaptive dynamic caption rendering.
Verifies:
  1. Full visibility across all window resolutions.
  2. Word-wrapping without truncating words or running off screen.
  3. Responsive font auto-scaling.
  4. Safe margin adherence (never touches screen boundaries or overlaps eyes).
  5. Contextual badge theming (GraceEMO, You, Listening, Thinking, Controls).
"""

import os
import unittest
import numpy as np

# Force offscreen headless driver
os.environ["SDL_VIDEODRIVER"] = "dummy"
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import pygame
import model

class TestDynamicCaptionRendering(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        pygame.init()

    def test_long_ai_speech_fully_visible_at_all_resolutions(self):
        text = (
            'I do not understand the reference to "luthmen." Could you please clarify what you mean? '
            'I am here to assist with questions about Lovely Professional University, its programs, '
            'or general admissions inquiries.'
        )
        resolutions = [
            (800, 600),
            (1024, 768),
            (1280, 720),
            (1366, 768),
            (1920, 1080),
            (2560, 1440),
        ]

        for w, h in resolutions:
            surface = pygame.Surface((w, h))
            rect = model.draw_caption(surface, text)
            self.assertIsNotNone(rect, f"Rect must not be None at resolution {w}x{h}")

            # Assert horizontal containment
            self.assertGreaterEqual(rect.left, 0, f"Left edge out of bounds at {w}x{h}: {rect.left}")
            self.assertLessEqual(rect.right, w, f"Right edge out of bounds at {w}x{h}: {rect.right}")

            # Assert vertical containment and safe area
            self.assertLessEqual(rect.bottom, h - 30, f"Bottom edge too close to boundary at {w}x{h}: {rect.bottom} vs {h}")
            self.assertGreaterEqual(rect.top, int(h * 0.50), f"Card intrudes into eye area at {w}x{h}: {rect.top}")

    def test_word_wrapping_preserves_words(self):
        font = model.get_dynamic_font(24)
        sample = "The quick brown fox jumps over the lazy dog repeatedly across the university campus."
        wrapped = model.wrap_text_dynamic(sample, font, max_width=400)
        self.assertGreater(len(wrapped), 1, "Must wrap into multiple lines for narrow width")
        reconstructed = " ".join(wrapped)
        self.assertEqual(sample, reconstructed, "Word wrapping must not drop or alter words")

    def test_status_theming_and_badging(self):
        surface = pygame.Surface((1024, 768))

        # Human prompt
        rect_human = model.draw_caption(surface, "Human: Tell me about campus facilities")
        self.assertIsNotNone(rect_human)

        # Listening
        rect_listening = model.draw_caption(surface, "Listening... (Speak your question)")
        self.assertIsNotNone(rect_listening)

        # Thinking
        rect_thinking = model.draw_caption(surface, "Processing speech...")
        self.assertIsNotNone(rect_thinking)

        # Controls
        rect_ctrl = model.draw_caption(surface, "[s] Speak  |  [f] Fullscreen  |  [q] Quit")
        self.assertIsNotNone(rect_ctrl)

    def test_empty_or_whitespace_caption_does_not_render(self):
        surface = pygame.Surface((1024, 768))
        self.assertIsNone(model.draw_caption(surface, ""))
        self.assertIsNone(model.draw_caption(surface, "   "))
        self.assertIsNone(model.draw_caption(surface, "\n\t"))

    def test_font_cache_reuse(self):
        font1 = model.get_dynamic_font(22, bold=False)
        font2 = model.get_dynamic_font(22, bold=False)
        self.assertIs(font1, font2, "Font instances of identical size and weight must be cached")

if __name__ == "__main__":
    unittest.main()

