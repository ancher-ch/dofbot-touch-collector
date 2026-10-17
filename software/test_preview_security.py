"""Publication security checks; no physical robot, camera or I2C is opened."""
import unittest
from unittest.mock import MagicMock, patch
from pathlib import Path
import teleop_record
import trajectory_teach

class PreviewSecurityTests(unittest.TestCase):
    def test_preview_binds_only_loopback(self):
        with patch.object(teleop_record, 'ThreadingHTTPServer') as factory, patch.object(teleop_record.threading, 'Thread') as thread:
            server = teleop_record.start_preview(MagicMock(), MagicMock(), 8765)
            self.assertEqual(factory.call_args.args[0], ('127.0.0.1', 8765))
            self.assertIs(server, factory.return_value)
            thread.return_value.start.assert_called_once()

    def test_legacy_session_uses_the_same_source_root(self):
        self.assertEqual(trajectory_teach.SESSION, Path(trajectory_teach.__file__).with_name('.learning_session.json'))

if __name__ == '__main__':
    unittest.main()
