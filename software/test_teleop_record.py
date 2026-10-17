"""Recorder checks with simulated hardware; never imports the vendor SDK."""
import json
from pathlib import Path
import tempfile
import unittest

from teleop_record import apply_key, check_tracking, Episode


class RecorderTests(unittest.TestCase):
    def test_each_key_changes_only_selected_joint_by_one_degree(self):
        start = [90, 135, 0, 45, 89, 52]
        for joint in range(1, 7):
            result = apply_key(start, joint, 1)
            self.assertEqual(sum(abs(a - b) for a, b in zip(start, result)), 1)
            self.assertEqual(result[joint - 1], start[joint - 1] + 1)
        self.assertEqual(start, [90, 135, 0, 45, 89, 52])

    def test_invalid_or_out_of_range_move_rejected(self):
        for joint, direction in [(3, -1), (0, 1), (7, 1), (1, 10)]:
            with self.subTest(joint=joint), self.assertRaises(ValueError):
                apply_key([90, 135, 0, 45, 89, 52], joint, direction)
        with self.assertRaises(ValueError):
            apply_key([90, 135, 0, 45, 270, 52], 5, 1)

    def test_drift_or_missing_feedback_stops_controller(self):
        with self.assertRaises(RuntimeError):
            check_tracking([90, 135, 0, 45, 89, 52], [96, 135, 0, 45, 89, 52])
        for bad in ([90] * 5, [float("nan")] * 6, [None] * 6):
            with self.subTest(bad=bad), self.assertRaises(RuntimeError):
                check_tracking(bad, [90] * 6)

    def test_episode_keeps_measured_state_separate_from_sent_action(self):
        class Frame:
            shape = (480, 640, 3)

        class ImageWriter:
            IMWRITE_JPEG_QUALITY = 1

            @staticmethod
            def imwrite(path, frame, options):
                Path(path).write_bytes(b"simulated image")
                return True

        with tempfile.TemporaryDirectory() as folder:
            episode = Episode(Path(folder), "pick and place", "/dev/video0", ImageWriter())
            episode.add(Frame(), 7, 100, [90] * 6, 110, 120, [91] * 6, 130, 140, 1.0)
            episode.add(Frame(), 10, 200, [91] * 6, 210, 220, [92] * 6, 230, 240, 1.1)
            episode.finish("success")
            rows = [json.loads(row) for row in (episode.path / "samples.jsonl").read_text().splitlines()]
            meta = json.loads((episode.path / "metadata.json").read_text())
            self.assertEqual(rows[0]["timestamp"], 0)
            self.assertAlmostEqual(rows[1]["timestamp"], 0.1)
            self.assertEqual(rows[0]["observation.state"], [90] * 6)
            self.assertEqual(rows[0]["action"], [91] * 6)
            self.assertLess(rows[0]["state_read_end_monotonic_ns"], rows[0]["action_write_start_monotonic_ns"])
            self.assertEqual(meta["frames"], 2)
            self.assertTrue(meta["operator_success"])
            self.assertFalse(meta["train_ready"])


if __name__ == "__main__":
    unittest.main()
