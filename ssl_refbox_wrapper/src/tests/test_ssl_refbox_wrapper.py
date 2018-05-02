#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

import ssl_refbox_wrapper_node


class TestRefboxWrpper(unittest.TestCase):
    def setup(self):
        pass

    def test_init(self):
        self.assertEqual(True, True)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('ssl_refbox_wrapper', 'test_ssl_refbox_wrapper', TestRefboxWrpper)
