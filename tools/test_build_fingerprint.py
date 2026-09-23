#!/usr/bin/env python3
"""What the build fingerprint ignores, and what it must not."""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(__file__))
from build_fingerprint import normalize  # noqa: E402


class CTests(unittest.TestCase):
    def same(self, a, b):
        self.assertEqual(normalize('x.c', a.encode()), normalize('x.c', b.encode()))

    def differ(self, a, b):
        self.assertNotEqual(normalize('x.c', a.encode()), normalize('x.c', b.encode()))

    def test_comments_and_whitespace_do_not_count(self):
        self.same('int a = 1; // one\n/* block\n comment */ int b;',
                  'int  a = 1;\n\n\nint b;  // renamed comment')

    def test_code_counts(self):
        self.differ('int a = 1;', 'int a = 2;')
        self.differ('typedef enum { A = 1 } e_t;', 'typedef enum { A = 1, B = 2 } e_t;')

    def test_comment_markers_inside_strings_are_code(self):
        self.differ('const char *s = "http://a";', 'const char *s = "http://b";')


class JsonAndPythonTests(unittest.TestCase):
    def test_prose_fields_do_not_count(self):
        a = b'{"pads": [1, 2], "description": "old", "updated_at": "x"}'
        b = b'{"description": "new", "pads": [1, 2]}'
        self.assertEqual(normalize('d.json', a), normalize('d.json', b))
        self.assertNotEqual(normalize('d.json', a), normalize('d.json', b'{"pads": [1, 3]}'))

    def test_docstrings_and_comments_do_not_count(self):
        a = b'def f():\n    """Doc."""\n    return 1  # one\n'
        b = b'def f():\n    return 1\n'
        self.assertEqual(normalize('m.py', a), normalize('m.py', b))
        self.assertNotEqual(normalize('m.py', b), normalize('m.py', b'def f():\n    return 2\n'))

    def test_hash_inside_a_string_is_code(self):
        a = b'x = "a#b"  # note\n'
        self.assertEqual(normalize('m.py', a), b'x = "a#b"')
        self.assertNotEqual(normalize('m.py', a), normalize('m.py', b'x = "a#c"\n'))


if __name__ == '__main__':
    unittest.main()
