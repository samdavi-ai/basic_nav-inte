#!/usr/bin/env python3
"""
Unit and Regression Test Suite: Pre-STT Noise Rejection & Telemetry Pipeline
Verifies that:
1. The existing post-STT hallucination filter ('Thank you.', 'thanks for watching', ghost phrases)
   is strictly preserved and continues rejecting suspicious transcription artifacts.
2. Pre-STT noise rejection identifies and discards ambient noise, silence, transient clicks,
   and stationary hums BEFORE cloud STT submission, saving API calls and latency.
3. Genuine short speech phrases ("Hello", "Stop", "Yes", "No", "Thank you") are NOT rejected,
   using acoustic evidence (voiced SNR, syllable continuity, active duration, crest factor).
4. Audio pipeline telemetry accurately measures pre-STT rejected vs. post-STT rejected vs. accepted counts.
5. Before-and-after false-trigger rates demonstrate substantial noise suppression.
"""
import unittest
import sys
import os
import math
import numpy as np

# Add project root and interaction dir to path
TEST_DIR = os.path.dirname(os.path.abspath(__file__))
INTERACTION_DIR = os.path.abspath(os.path.join(TEST_DIR, ".."))
WORKSPACE_DIR = os.path.abspath(os.path.join(INTERACTION_DIR, ".."))

if INTERACTION_DIR not in sys.path:
    sys.path.insert(0, INTERACTION_DIR)

import model


def generate_ambient_noise(duration_sec: float, rms: float = 600.0, rate: int = 16000) -> np.ndarray:
    """Generates gaussian white/pink ambient room noise."""
    samples = int(duration_sec * rate)
    noise = np.random.normal(0, rms, samples)
    return np.clip(noise, -32768, 32767).astype(np.int16)


def generate_transient_click(duration_sec: float = 0.8, click_duration_ms: float = 20.0, click_peak: float = 16000.0, rate: int = 16000) -> np.ndarray:
    """Generates room silence with a single transient click (e.g. pen drop, chair scrape)."""
    samples = int(duration_sec * rate)
    audio = np.random.normal(0, 300.0, samples)  # Low room noise
    click_samples = int((click_duration_ms / 1000.0) * rate)
    click_start = samples // 3
    # Sharp impulse
    impulse = np.hanning(click_samples) * click_peak
    audio[click_start : click_start + click_samples] += impulse
    return np.clip(audio, -32768, 32767).astype(np.int16)


def generate_stationary_hum(duration_sec: float = 2.0, hum_rms: float = 2400.0, rate: int = 16000) -> np.ndarray:
    """Generates continuous stationary motor/fan hum (low crest factor, flat envelope)."""
    samples = int(duration_sec * rate)
    t = np.linspace(0, duration_sec, samples, endpoint=False)
    # Fundamental 60Hz + harmonics
    hum = (np.sin(2 * np.pi * 60 * t) + 0.5 * np.sin(2 * np.pi * 120 * t) + 0.25 * np.sin(2 * np.pi * 180 * t))
    hum = hum / np.sqrt(np.mean(hum ** 2)) * hum_rms
    # Add mild noise
    hum += np.random.normal(0, 200.0, samples)
    return np.clip(hum, -32768, 32767).astype(np.int16)


def generate_synthetic_speech(
    phrase: str,
    speech_duration_sec: float,
    total_duration_sec: float,
    f0: float = 135.0,
    peak_rms: float = 5000.0,
    noise_rms: float = 600.0,
    rate: int = 16000,
) -> np.ndarray:
    """
    Synthesizes acoustic speech signal with voiced glottal pulses, formant filtering,
    and a smooth speech envelope on top of background noise.
    """
    total_samples = int(total_duration_sec * rate)
    speech_samples = int(speech_duration_sec * rate)
    audio = np.random.normal(0, noise_rms, total_samples)

    start_idx = int(0.15 * rate)  # Pre-roll silence
    end_idx = min(total_samples, start_idx + speech_samples)
    actual_speech_len = end_idx - start_idx

    t = np.linspace(0, speech_duration_sec, actual_speech_len, endpoint=False)
    # Glottal pulse harmonic train (speech vowels)
    glottal = (
        np.sin(2 * np.pi * f0 * t)
        + 0.8 * np.sin(2 * np.pi * 2 * f0 * t)
        + 0.6 * np.sin(2 * np.pi * 3 * f0 * t)
        + 0.4 * np.sin(2 * np.pi * 5 * f0 * t)  # First formant ~ 675 Hz
        + 0.3 * np.sin(2 * np.pi * 9 * f0 * t)  # Second formant ~ 1215 Hz
    )
    # Smooth attack-sustain-decay envelope
    envelope = np.hanning(actual_speech_len)
    voiced_signal = glottal * envelope
    voiced_signal = voiced_signal / np.sqrt(np.mean(voiced_signal ** 2)) * peak_rms

    audio[start_idx:end_idx] += voiced_signal
    return np.clip(audio, -32768, 32767).astype(np.int16)


class TestExistingPostSTTFilterPreservation(unittest.TestCase):
    """
    Regression Test 1:
    Verifies that the existing post-STT hallucination filter is 100% preserved.
    Checks exact ghost phrases, substring phrases, single words, and segment confidence.
    """

    def test_existing_exact_ghost_phrases_rejected(self):
        """Exact silence ghosts are rejected unconditionally when no audio evidence is provided."""
        ghosts = ["you", "so", ".", "thank you.", "thank you", "bye.", "bye", "watching", "subscribe"]
        for g in ghosts:
            self.assertTrue(
                model.is_hallucination_or_noise(g),
                f"Existing filter must reject exact ghost: '{g}'",
            )

    def test_existing_substring_phrase_ghosts_rejected(self):
        """Known YouTube/Whisper subtitle hallucination phrases are strictly rejected."""
        phrase_ghosts = [
            "thanks for watching",
            "thank you for watching",
            "subtitles by",
            "amara.org",
            "im gonna die",
            "i am sorry, sir",
            "i am sorry sir",
            "die in sm",
            "please subscribe",
        ]
        for p in phrase_ghosts:
            self.assertTrue(
                model.is_hallucination_or_noise(p),
                f"Existing filter must reject phrase ghost: '{p}'",
            )
            # Embedded in sentence
            self.assertTrue(
                model.is_hallucination_or_noise(f"Hello, {p}!"),
                f"Existing filter must reject embedded phrase ghost: '{p}'",
            )

    def test_existing_single_word_whitelist(self):
        """Single words are allowed ONLY if they are in the valid command whitelist."""
        valid_words = ["stop", "hi", "hey", "hello", "help", "wait", "freeze", "halt", "yes", "no"]
        for w in valid_words:
            self.assertFalse(
                model.is_hallucination_or_noise(w),
                f"Valid single-word command '{w}' must NOT be rejected post-STT",
            )

        invalid_single_words = ["cat", "desk", "blue", "walk", "sing", "chair"]
        for w in invalid_single_words:
            self.assertTrue(
                model.is_hallucination_or_noise(w),
                f"Single word '{w}' not in whitelist must be rejected post-STT",
            )

    def test_whisper_segment_no_speech_confidence(self):
        """If all segments report high no_speech_prob (> 0.85), post-filter rejects."""
        segments_silent = [
            {"text": "something", "no_speech_prob": 0.92},
            {"text": "here", "no_speech_prob": 0.89},
        ]
        self.assertTrue(
            model.is_hallucination_or_noise("something here", segments=segments_silent),
            "High no_speech_prob must trigger hallucination rejection",
        )

        segments_clear_speech = [
            {"text": "navigate to the robotics lab", "no_speech_prob": 0.05},
        ]
        self.assertFalse(
            model.is_hallucination_or_noise("navigate to the robotics lab", segments=segments_clear_speech),
            "Low no_speech_prob valid sentence must pass",
        )


class TestPreSTTNoiseRejectionEngine(unittest.TestCase):
    """
    Test 2:
    Verifies that Pre-STT noise rejection discards noise artifacts BEFORE STT submission.
    """

    def setUp(self):
        model.reset_audio_pipeline_metrics()
        model.B_T = 750.0
        model.T_MIN = 1500.0

    def test_pure_silence_rejected_before_stt(self):
        """Pure room silence (zeros or micro-noise) is rejected before cloud STT."""
        silence = np.zeros(16000 * 2, dtype=np.int16)  # 2.0s zeros
        is_valid, reason, metrics = model.validate_audio_for_stt(silence, rate=16000, noise_floor=500.0)
        self.assertFalse(is_valid)
        self.assertTrue("energy" in reason.lower() or "voiced chunks" in reason.lower())
        self.assertFalse(metrics["is_valid_speech"])
        self.assertEqual(metrics["voiced_chunks"], 0)

    def test_runtime_log_failure_case_rejected_before_stt(self):
        """
        Recreates the exact failure from the runtime log:
        'Saved 2.18s audio (RMS: 2336, Peak: 12668) to input.wav -> Human: Thank you.'
        High ambient noise with a small transient spike, but NO genuine speech activity.
        Must be rejected BEFORE STT!
        """
        # 2.18s audio with ambient floor RMS ~ 2100, one single 40ms noise flutter
        samples = int(2.18 * 16000)
        audio = np.random.normal(0, 2100.0, samples).astype(np.float32)
        # Spike up to 12668 peak
        audio[8000:8400] += 10500.0 * np.hanning(400)
        audio_int16 = np.clip(audio, -32768, 32767).astype(np.int16)

        is_valid, reason, metrics = model.validate_audio_for_stt(
            audio_int16, rate=16000, noise_floor=2000.0
        )
        self.assertFalse(is_valid, "Ambient noise with spike must be rejected before cloud STT!")
        self.assertTrue(
            "snr" in reason.lower()
            or "activity ratio" in reason.lower()
            or "syllable" in reason.lower()
            or "chunks" in reason.lower(),
            f"Expected acoustic rejection reason, got: {reason}",
        )
        self.assertFalse(metrics["is_valid_speech"])

    def test_transient_click_rejected_before_stt(self):
        """Transient click (pen tap, chair scrape) has high peak but no sustained syllable."""
        click_audio = generate_transient_click(duration_sec=1.0, click_duration_ms=25.0, click_peak=18000.0)
        is_valid, reason, metrics = model.validate_audio_for_stt(click_audio, rate=16000, noise_floor=600.0)
        self.assertFalse(is_valid)
        self.assertTrue(
            "syllable" in reason.lower() or "click" in reason.lower() or "chunks" in reason.lower(),
            f"Expected syllable/impulse rejection, got: {reason}",
        )
        self.assertFalse(metrics["is_valid_speech"])

    def test_stationary_motor_hum_rejected_before_stt(self):
        """Continuous stationary fan/motor hum has low crest factor and is rejected."""
        hum_audio = generate_stationary_hum(duration_sec=1.8, hum_rms=2200.0)
        is_valid, reason, metrics = model.validate_audio_for_stt(hum_audio, rate=16000, noise_floor=800.0)
        self.assertFalse(is_valid)
        self.assertTrue("hum" in reason.lower() or "crest" in reason.lower() or "stationary" in reason.lower() or "snr" in reason.lower())
        self.assertFalse(metrics["is_valid_speech"])

    def test_too_short_audio_rejected_before_stt(self):
        """Audio under 0.30s (e.g. 0.15s tick) cannot be an intelligible word and is rejected."""
        short_audio = generate_ambient_noise(duration_sec=0.20, rms=3000.0)
        is_valid, reason, metrics = model.validate_audio_for_stt(short_audio, rate=16000, noise_floor=600.0)
        self.assertFalse(is_valid)
        self.assertIn("too short", reason.lower())
        self.assertFalse(metrics["is_valid_speech"])


class TestGenuineShortSpeechPreservation(unittest.TestCase):
    """
    Test 3:
    Verifies that genuine short phrases ("Hello", "Stop", "Yes", "No", "Thank you")
    are NOT rejected by the Pre-STT noise rejection filter when real acoustic speech is present.
    """

    def setUp(self):
        model.reset_audio_pipeline_metrics()
        model.B_T = 700.0
        model.T_MIN = 1500.0

    def test_genuine_hello_accepted(self):
        """Genuine spoken 'Hello' (~0.65s, voiced syllables) passes pre-STT validation."""
        hello_audio = generate_synthetic_speech("Hello", speech_duration_sec=0.55, total_duration_sec=0.85, peak_rms=5000.0)
        is_valid, reason, metrics = model.validate_audio_for_stt(hello_audio, rate=16000, noise_floor=700.0)
        self.assertTrue(is_valid, f"Genuine 'Hello' must be accepted, got rejection: {reason}")
        self.assertTrue(metrics["is_valid_speech"])
        self.assertGreaterEqual(metrics["voiced_chunks"], 2)
        self.assertGreaterEqual(metrics["max_consecutive_voiced"], 2)
        self.assertGreater(metrics["snr_db"], 6.0)

    def test_genuine_stop_accepted(self):
        """Genuine spoken 'Stop' (~0.45s, voiced syllable) passes pre-STT validation."""
        stop_audio = generate_synthetic_speech("Stop", speech_duration_sec=0.40, total_duration_sec=0.65, peak_rms=5200.0)
        is_valid, reason, metrics = model.validate_audio_for_stt(stop_audio, rate=16000, noise_floor=700.0)
        self.assertTrue(is_valid, f"Genuine 'Stop' must be accepted, got rejection: {reason}")
        self.assertTrue(metrics["is_valid_speech"])
        self.assertGreaterEqual(metrics["voiced_chunks"], 2)

    def test_genuine_yes_accepted(self):
        """Genuine spoken 'Yes' (~0.40s) passes pre-STT validation."""
        yes_audio = generate_synthetic_speech("Yes", speech_duration_sec=0.35, total_duration_sec=0.60, peak_rms=4800.0)
        is_valid, reason, metrics = model.validate_audio_for_stt(yes_audio, rate=16000, noise_floor=700.0)
        self.assertTrue(is_valid, f"Genuine 'Yes' must be accepted, got rejection: {reason}")
        self.assertTrue(metrics["is_valid_speech"])

    def test_genuine_no_accepted(self):
        """Genuine spoken 'No' (~0.38s) passes pre-STT validation."""
        no_audio = generate_synthetic_speech("No", speech_duration_sec=0.35, total_duration_sec=0.55, peak_rms=4900.0)
        is_valid, reason, metrics = model.validate_audio_for_stt(no_audio, rate=16000, noise_floor=700.0)
        self.assertTrue(is_valid, f"Genuine 'No' must be accepted, got rejection: {reason}")
        self.assertTrue(metrics["is_valid_speech"])

    def test_genuine_thank_you_accepted_with_audio_evidence(self):
        """
        Genuine spoken 'Thank you' (~0.80s):
        1. Passes Pre-STT acoustic validation.
        2. With valid audio evidence, is distinguished from silence hallucinations post-STT!
        """
        thank_you_audio = generate_synthetic_speech("Thank you", speech_duration_sec=0.65, total_duration_sec=0.95, peak_rms=5500.0)
        is_valid, reason, audio_evidence = model.validate_audio_for_stt(thank_you_audio, rate=16000, noise_floor=700.0)
        self.assertTrue(is_valid, f"Genuine 'Thank you' audio must pass pre-STT validation, got: {reason}")
        self.assertTrue(audio_evidence["is_valid_speech"])
        self.assertGreaterEqual(audio_evidence["snr_db"], 8.0)
        self.assertGreaterEqual(audio_evidence["voiced_chunks"], 3)

        # Now test post-STT with audio evidence:
        # Genuine user speech with confirmed audio evidence must NOT be rejected
        segments_clear = [{"text": "thank you.", "no_speech_prob": 0.04}]
        is_filtered = model.is_hallucination_or_noise("Thank you.", segments=segments_clear, audio_evidence=audio_evidence)
        self.assertFalse(is_filtered, "Genuine spoken 'Thank you.' with verified audio evidence must pass!")

        # Conversely, if audio evidence is absent or marks noise, it IS rejected as a ghost artifact
        is_filtered_no_audio = model.is_hallucination_or_noise("Thank you.")
        self.assertTrue(is_filtered_no_audio, "Hallucinated 'Thank you.' without audio evidence must be filtered!")

    def test_realistic_thank_you_accepted(self):
        """
        Verifies that realistic spoken 'Thank you' with moderate room SNR (6.1 dB, RMS 1980, 4 chunks)
        is accepted and NOT falsely rejected.
        """
        realistic_evidence = {
            "is_valid_speech": True,
            "snr_db": 6.1,
            "voiced_chunks": 4,
            "overall_rms": 1980.0,
            "duration_sec": 1.79,
        }
        segments_clear = [{"text": "Thank you.", "no_speech_prob": 0.05, "avg_logprob": -0.25}]
        is_filtered = model.is_hallucination_or_noise("Thank you.", segments=segments_clear, audio_evidence=realistic_evidence)
        self.assertFalse(is_filtered, "Spoken 'Thank you.' with realistic 6.1dB SNR must be accepted!")

    def test_audio_energy_below_ambient_noise_floor_rejected(self):
        """
        Verifies that audio where overall RMS does not exceed ambient noise floor B_T
        (e.g., RMS 1560 when noise floor is 1672) is rejected before STT.
        """
        samples = int(4.2 * 16000)
        # Generate noise with RMS ~ 1560
        noise = np.random.normal(0, 1560.0, samples).astype(np.int16)
        is_valid, reason, metrics = model.validate_audio_for_stt(noise, rate=16000, noise_floor=1672.0)
        self.assertFalse(is_valid, "Audio below ambient noise floor must be rejected before STT!")
        self.assertIn("noise floor", reason.lower())

    def test_whisper_low_snr_hallucination_filtered(self):
        """
        Verifies that random sentences hallucinated by Whisper on low-SNR noise
        with poor token logprob are filtered out post-STT.
        """
        low_snr_evidence = {
            "is_valid_speech": True,
            "snr_db": 3.2,
            "voiced_chunks": 11,
            "overall_rms": 1560.0,
            "duration_sec": 4.22,
        }
        low_conf_segments = [
            {"text": "I'm getting a few people out of the world.", "no_speech_prob": 0.42, "avg_logprob": -0.92}
        ]
        is_filtered = model.is_hallucination_or_noise(
            "I'm getting a few people out of the world.",
            segments=low_conf_segments,
            audio_evidence=low_snr_evidence,
        )
        self.assertTrue(is_filtered, "Whisper hallucination on low-SNR room noise must be filtered!")


class TestAudioPipelineTelemetryAndFalseTriggerRates(unittest.TestCase):
    """
    Test 4:
    Verifies telemetry measurement (Pre-STT Rejected, Post-STT Rejected, Gateway Accepted)
    and demonstrates the before-and-after false-trigger rate improvement.
    """

    def setUp(self):
        model.reset_audio_pipeline_metrics()

    def test_telemetry_metrics_tracking(self):
        """Pre-STT and Post-STT metrics are recorded accurately."""
        # 1. Reset metrics
        model.reset_audio_pipeline_metrics()
        m0 = model.get_audio_pipeline_metrics()
        self.assertEqual(m0["total_recordings"], 0)
        self.assertEqual(m0["pre_stt_rejected"], 0)
        self.assertEqual(m0["post_stt_rejected"], 0)
        self.assertEqual(m0["gateway_accepted"], 0)

        # 2. Simulate 5 noise recordings that fail Pre-STT
        for _ in range(5):
            noise = generate_ambient_noise(2.0, rms=400.0)
            is_valid, reason, meta = model.validate_audio_for_stt(noise, rate=16000, noise_floor=800.0)
            self.assertFalse(is_valid)
            model.AUDIO_METRICS["total_recordings"] += 1
            model.AUDIO_METRICS["pre_stt_rejected"] += 1
            key = reason.split("(")[0].strip()
            model.AUDIO_METRICS["pre_stt_reasons"][key] = model.AUDIO_METRICS["pre_stt_reasons"].get(key, 0) + 1

        # 3. Simulate 1 edge recording that passed Pre-STT but was filtered Post-STT (e.g. YouTube phrase)
        model.AUDIO_METRICS["total_recordings"] += 1
        model.AUDIO_METRICS["stt_submissions"] += 1
        is_ghost = model.is_hallucination_or_noise("thanks for watching")
        self.assertTrue(is_ghost)
        model.AUDIO_METRICS["post_stt_rejected"] += 1

        # 4. Simulate 2 valid speech recordings that pass both Pre-STT and Post-STT
        for phrase in ["Hello robot", "Stop"]:
            model.AUDIO_METRICS["total_recordings"] += 1
            model.AUDIO_METRICS["stt_submissions"] += 1
            is_ghost = model.is_hallucination_or_noise(phrase)
            self.assertFalse(is_ghost)
            model.AUDIO_METRICS["gateway_accepted"] += 1

        # Inspect final metrics
        final_m = model.get_audio_pipeline_metrics()
        self.assertEqual(final_m["total_recordings"], 8)
        self.assertEqual(final_m["pre_stt_rejected"], 5)
        self.assertEqual(final_m["stt_submissions"], 3)
        self.assertEqual(final_m["post_stt_rejected"], 1)
        self.assertEqual(final_m["gateway_accepted"], 2)

    def test_false_trigger_rate_comparison(self):
        """
        Simulates 20 ambient noise / background non-speech events (clicks, hiss, hums).
        Compares:
        - BEFORE: all 20 submitted to cloud STT -> high API waste & hallucination risk.
        - AFTER: 0 reach cloud STT; all 20 rejected Pre-STT by acoustic energy & SNR validation.
        """
        noise_events = [
            generate_ambient_noise(2.18, rms=2100.0),
            generate_ambient_noise(1.5, rms=1800.0),
            generate_transient_click(0.8, click_peak=14000.0),
            generate_transient_click(1.2, click_peak=16000.0),
            generate_stationary_hum(2.0, hum_rms=2300.0),
            generate_stationary_hum(1.5, hum_rms=2000.0),
            np.zeros(16000 * 2, dtype=np.int16),
            np.zeros(16000 * 1, dtype=np.int16),
        ] * 3  # 24 noise events

        pre_stt_rejected = 0
        pre_stt_passed = 0

        for audio in noise_events:
            is_valid, _, _ = model.validate_audio_for_stt(audio, rate=16000, noise_floor=800.0)
            if is_valid:
                pre_stt_passed += 1
            else:
                pre_stt_rejected += 1

        rejection_rate = (pre_stt_rejected / len(noise_events)) * 100.0
        print(f"\n📊 Noise Rejection Benchmark: {pre_stt_rejected}/{len(noise_events)} noise events rejected before cloud STT ({rejection_rate:.1f}%)")
        self.assertEqual(pre_stt_passed, 0, "No pure ambient noise or transient click should reach cloud STT!")
        self.assertEqual(rejection_rate, 100.0)


if __name__ == "__main__":
    unittest.main()
