#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

from world_model import State

from consai_msgs.msg import Pose
from consai_msgs.msg import Pose as Velocity

class TestState(unittest.TestCase):
    def setUp(self):
        pass

    def _assertAlmostEqualPose(self, pose1, pose2):
        self.assertAlmostEqual(pose1.x, pose2.x)
        self.assertAlmostEqual(pose1.y, pose2.y)
        self.assertAlmostEqual(pose1.theta, pose2.theta)

    def test_set_get_pose(self):
        test_pose = Pose(1, 2, 3)

        state = State()
        state.set_pose(test_pose)

        actual = state.get_pose()
        self._assertAlmostEqualPose(test_pose, actual)

    def test_set_get_velocity(self):
        test_velocity = Velocity(1, 2, 3)

        state = State()
        state.set_velocity(test_velocity)

        actual = state.get_velocity()
        self._assertAlmostEqualPose(test_velocity, actual)

    def test_is_enabled(self):
        state = State()

        expected = False
        actual = state.is_enabled()
        self.assertEqual(expected, actual)

        state.enable()
        expected = True
        actual = state.is_enabled()
        self.assertEqual(expected, actual)

        state.desable()
        expected = False
        actual = state.is_enabled()
        self.assertEqual(expected, actual)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_state', TestState)
