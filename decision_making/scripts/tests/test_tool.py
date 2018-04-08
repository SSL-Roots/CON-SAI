#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest
import math

from geometry_msgs.msg import Quaternion, Point

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

import tool

class TestTool(unittest.TestCase):

    def setUp(self):
        pass

    def test_yaw_from_quaternion(self):
        test_quaternion = Quaternion()
        test_quaternion.x = 0.000
        test_quaternion.y = 0.000
        test_quaternion.z = 0.707
        test_quaternion.w = 0.707

        expected = math.radians(90)
        actual = tool.yawFromQuaternion(test_quaternion)
        self.assertAlmostEqual(expected, actual)

        test_quaternion.x = 0.000
        test_quaternion.y = 0.000
        test_quaternion.z = 0.000
        test_quaternion.w = 1.000

        expected = math.radians(0)
        actual = tool.yawFromQuaternion(test_quaternion)
        self.assertAlmostEqual(expected, actual)

    def test_yaw_from_tf_quaternion(self):
        test_quaternion = [0.000, 0.000, 1.000, 0.000]
        expected = math.radians(180)
        actual = tool.yawFromTfQuaternion(test_quaternion)
        self.assertAlmostEqual(expected, actual)
        
        test_quaternion = [0.000, 0.000, 0.707, -0.707]
        expected = math.radians(-90)
        actual = tool.yawFromTfQuaternion(test_quaternion)
        self.assertAlmostEqual(expected, actual)

    def test_point_from_translation(self):
        test_translation = [1, 2, 0]

        expected = Point()
        expected.x = 1
        expected.y = 2

        actual = tool.pointFromTranslation(test_translation)
        self.assertAlmostEqual(expected, actual)

    def test_normalize(self):
        self._test_normalize(0, 0)
        self._test_normalize(math.pi * 0.5, math.pi * 0.5)
        self._test_normalize(math.pi, math.pi)
        self._test_normalize(-math.pi * 0.5, -math.pi * 0.5)
        self._test_normalize(math.pi * 0.5 + 2 * math.pi, math.pi * 0.5)
        self._test_normalize(math.pi * 0.5 + 3 * math.pi, -math.pi * 0.5)

        
    def _test_normalize(self, test_angle, expected):
        actual = tool.normalize(test_angle)
        self.assertAlmostEqual(expected, actual)



if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_tool', TestTool)
