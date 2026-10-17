"""Test the publication guard using synthesized examples, not real secrets."""
import unittest
from check_publication import inspect


class PublicationTests(unittest.TestCase):
    def test_credential_patterns_and_personal_paths(self):
        token = ''.join(['gh', 'p', '_']) + ('X' * 36)
        self.assertIn('GitHub credential', inspect('example.py', token.encode()))
        header = '-----' + 'BEGIN ' + 'PRIVATE KEY' + '-----'
        self.assertIn('private key header', inspect('example.py', header.encode()))
        url = 'https://' + 'user:password' + '@example.invalid'
        self.assertIn('credential in URL', inspect('example.py', url.encode()))
        win = 'C:' + chr(92) + 'Users' + chr(92) + 'example'
        self.assertIn('personal Windows path', inspect('example.py', win.encode()))

    def test_real_network_ranges_and_localhost(self):
        private = '.'.join(map(str, (10, 23, 42, 9)))
        self.assertIn('embedded private network address', inspect('README.md', private.encode()))
        self.assertEqual(inspect('README.md', b'http://127.0.0.1:8765/'), [])

    def test_runtime_files_and_binary_are_rejected(self):
        self.assertTrue(inspect('software/home.json', b'{}'))
        self.assertTrue(inspect('episodes/entry/metadata.json', b'{}'))
        self.assertTrue(inspect('.env.local', b'EXAMPLE=1'))
        self.assertTrue(inspect('capture.jpg', bytes((255, 0, 254))))
        self.assertEqual(inspect('test_example.py', b'assert True\n'), [])


if __name__ == '__main__':
    unittest.main()
