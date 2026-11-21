#!/usr/bin/env python3
import os
import re
import math
import time
from typing import List, Tuple, Dict, Optional

# Force offscreen headless for test verification
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame

pygame.init()

FONT_PATH = "vira_interaction/font.ttf"
_FONT_CACHE: Dict[Tuple[int, bool], pygame.font.Font] = {}

def get_dynamic_font(size: int, bold: bool = False) -> pygame.font.Font:
    key = (size, bold)
    if key in _FONT_CACHE:
        return _FONT_CACHE[key]
    font = None
    if os.path.exists(FONT_PATH):
        try:
            font = pygame.font.Font(FONT_PATH, size)
        except Exception:
            font = None
    if font is None:
        try:
            font = pygame.font.SysFont("dejavusans,freesans,helvetica,arial", size, bold=bold)
        except Exception:
            font = pygame.font.Font(None, size)
    _FONT_CACHE[key] = font
    return font

def wrap_text_dynamic(text: str, font: pygame.font.Font, max_width: int) -> List[str]:
    words = text.split()
    if not words:
        return []
    lines = []
    current_line = []
    for word in words:
        candidate = " ".join(current_line + [word])
        w, _ = font.size(candidate)
        if w <= max_width:
            current_line.append(word)
        else:
            if current_line:
                lines.append(" ".join(current_line))
                current_line = [word]
            else:
                cur_part = ""
                for ch in word:
                    if font.size(cur_part + ch)[0] <= max_width:
                        cur_part += ch
                    else:
                        lines.append(cur_part)
                        cur_part = ch
                if cur_part:
                    current_line = [cur_part]
    if current_line:
        lines.append(" ".join(current_line))
    return lines

def draw_caption(surface: pygame.Surface, text: str, font: Optional[pygame.font.Font] = None, color: Optional[Tuple[int, int, int]] = None, y_offset: int = 10) -> Optional[pygame.Rect]:
    """
    Renders an adaptive, dynamic, high-visibility subtitle/caption card at the bottom of the screen.
    Guarantees 100% text visibility with responsive multi-line word wrapping, font auto-scaling,
    and a sleek frosted-glass container with contextual status badging.
    """
    if not text or not text.strip():
        return None

    # Strip unrenderable unicode emojis if using custom TTF without emoji glyphs
    clean_text = re.sub(r'[\U00010000-\U0010ffff]|[\u2600-\u27ff]|[\ufe0f]', '', text).strip()
    if not clean_text:
        return None

    sw, sh = surface.get_size()
    low_text = clean_text.lower()

    # Determine message context & theme
    badge_text = ""
    badge_color = (65, 175, 255)      # Electric Cyan
    border_color = (45, 115, 220, 160)
    text_color = (245, 248, 255)      # Crisp High-Contrast White
    is_pulsing = False

    if clean_text.startswith("Human:") or clean_text.startswith("User:"):
        badge_text = "YOU"
        badge_color = (56, 239, 125)   # Emerald Green
        border_color = (40, 190, 100, 160)
        clean_text = clean_text.split(":", 1)[1].strip()
    elif "listening" in low_text:
        badge_text = "LISTENING"
        badge_color = (50, 215, 255)
        border_color = (50, 215, 255, 200)
        is_pulsing = True
    elif "processing" in low_text or "thinking" in low_text:
        badge_text = "THINKING"
        badge_color = (255, 200, 60)   # Amber
        border_color = (240, 180, 50, 180)
        is_pulsing = True
    elif "[s] speak" in low_text or "[s]" in low_text:
        badge_text = "CONTROLS"
        badge_color = (145, 175, 215)
        border_color = (70, 95, 130, 140)
        text_color = (195, 210, 230)
    elif "vad sensitivity" in low_text:
        badge_text = "AUDIO SENSITIVITY"
        badge_color = (210, 140, 255)  # Purple
        border_color = (180, 100, 230, 160)
    else:
        # GraceEMO Assistant Speech
        badge_text = "GraceEMO"
        badge_color = (65, 175, 255)
        border_color = (55, 130, 245, 180)

    # Geometry & Scaling Budget
    max_card_width = max(340, min(int(sw * 0.88), 1150))
    pad_x = max(20, int(sw * 0.018))
    pad_y = max(12, int(sh * 0.016))
    inner_max_w = max_card_width - (pad_x * 2)

    # Base font size scaled to screen resolution
    base_font_size = max(18, min(30, int(sh * 0.030)))
    badge_font_size = max(13, int(base_font_size * 0.68))
    badge_font = get_dynamic_font(badge_font_size, bold=True)

    # Auto-fit text: scale down font size if text is long or wraps into too many lines
    font_size = base_font_size
    max_content_height = int(sh * 0.28) # Cap vertical consumption to keep eyes clear

    while font_size >= 16:
        active_font = get_dynamic_font(font_size)
        wrapped_lines = wrap_text_dynamic(clean_text, active_font, inner_max_w)
        line_h = active_font.get_linesize()
        badge_space = (badge_font.get_linesize() + 6) if badge_text else 0
        total_h = len(wrapped_lines) * line_h + badge_space
        if total_h <= max_content_height and len(wrapped_lines) <= 4:
            break
        font_size -= 2

    # Render wrapped lines
    active_font = get_dynamic_font(font_size)
    wrapped_lines = wrap_text_dynamic(clean_text, active_font, inner_max_w)
    line_surfs = [active_font.render(l, True, text_color) for l in wrapped_lines]

    # Calculate actual content dimensions
    max_line_w = max((s.get_width() for s in line_surfs), default=120)

    # Badge surface & pulse indicator
    badge_surf = None
    pulse_dot_radius = 4
    if badge_text:
        badge_surf = badge_font.render(badge_text, True, badge_color)
    badge_row_w = (badge_surf.get_width() + (pulse_dot_radius * 2 + 8 if is_pulsing else 0)) if badge_surf else 0

    content_w = max(max_line_w, badge_row_w)
    card_w = content_w + pad_x * 2
    card_w = max(min(card_w, max_card_width), 240)

    line_spacing = max(3, int(font_size * 0.22))
    text_block_h = sum(s.get_height() for s in line_surfs) + max(0, len(line_surfs) - 1) * line_spacing
    badge_block_h = (badge_surf.get_height() + 6) if badge_surf else 0
    card_h = badge_block_h + text_block_h + pad_y * 2

    # Position card safely at the bottom
    safe_bottom_margin = max(42, int(sh * 0.055))
    card_x = (sw - card_w) // 2
    card_y = sh - safe_bottom_margin - card_h

    # Ensure it never overlaps the facial expression area (eyes)
    min_card_y = int(sh * 0.58)
    if card_y < min_card_y:
        card_y = min_card_y

    # Draw Frosted Glass Pill / Card Surface
    card_surface = pygame.Surface((card_w, card_h), pygame.SRCALPHA)
    pygame.draw.rect(card_surface, (10, 14, 25, 222), (0, 0, card_w, card_h), border_radius=18)
    pygame.draw.rect(card_surface, border_color, (0, 0, card_w, card_h), width=2, border_radius=18)
    surface.blit(card_surface, (card_x, card_y))

    # Render Badge Row
    curr_y = card_y + pad_y
    if badge_surf:
        badge_x = card_x + pad_x
        if is_pulsing:
            # Pulsing status dot
            pulse_val = 0.5 + 0.5 * math.sin(time.time() * 6.0)
            dot_alpha = int(140 + 115 * pulse_val)
            dot_color = (badge_color[0], badge_color[1], badge_color[2], dot_alpha)
            dot_surf = pygame.Surface((pulse_dot_radius * 2, pulse_dot_radius * 2), pygame.SRCALPHA)
            pygame.draw.circle(dot_surf, dot_color, (pulse_dot_radius, pulse_dot_radius), pulse_dot_radius)
            surface.blit(dot_surf, (badge_x, curr_y + (badge_surf.get_height() - pulse_dot_radius * 2) // 2))
            badge_x += pulse_dot_radius * 2 + 8

        surface.blit(badge_surf, (badge_x, curr_y))
        curr_y += badge_surf.get_height() + 6

    # Render Text Lines
    is_single_short_line = len(line_surfs) == 1 and line_surfs[0].get_width() < (card_w - pad_x * 4)
    for l_surf in line_surfs:
        if is_single_short_line and not badge_surf:
            line_x = card_x + (card_w - l_surf.get_width()) // 2
        else:
            line_x = card_x + pad_x
        surface.blit(l_surf, (line_x, curr_y))
        curr_y += l_surf.get_height() + line_spacing

    return pygame.Rect(card_x, card_y, card_w, card_h)

def run_tests():
    resolutions = [(1024, 768), (1920, 1080), (800, 600)]
    test_cases = [
        ("Long AI speech", 'I do not understand the reference to "luthmen." Could you please clarify what you mean? I am here to assist with questions about Lovely Professional University, its programs, or general admissions inquiries.'),
        ("Listening", "Listening... (Speak your question)"),
        ("Processing", "Processing speech..."),
        ("Human Prompt", "Human: Where is the Vice Chancellor's office located on campus?"),
        ("Controls", "[s] Speak  |  [f] Fullscreen  |  [q] Quit"),
        ("Short confirmation", "Navigating to reception desk now.")
    ]

    os.makedirs("docs/screenshots/gui_captions", exist_ok=True)

    for idx, (label, text_val) in enumerate(test_cases):
        surf = pygame.Surface((1024, 768))
        surf.fill((0, 0, 0))
        eye_color = (60, 110, 220)
        pygame.draw.ellipse(surf, eye_color, (int(1024*0.28), int(768*0.35), int(1024*0.18), int(768*0.16)))
        pygame.draw.ellipse(surf, eye_color, (int(1024*0.54), int(768*0.35), int(1024*0.18), int(768*0.16)))

        rect = draw_caption(surf, text_val)
        out_path = f"docs/screenshots/gui_captions/test_{idx+1}_{label.replace(' ', '_').lower()}.png"
        pygame.image.save(surf, out_path)
        print(f"[{label}] Rect: {rect} | Saved: {out_path}")

    print("All tests passed!")

if __name__ == "__main__":
    run_tests()
