#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

class TestCoordinate(unittest.TestCase):

    def setUp(self):
        pass 

    def test_init(self):
        self.assertEqual(True,True)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_coordinate', TestCoordinate)
