#!/usr/bin/env python
#encoding: utf8

import sys
import unittest

from scripts.observer import Observer
from consai_msgs.msg import Pose

class TestObserver(unittest.TestCase):

    def setUp(self):
        self.observer = Observer()

    def test_ball_is_in_field(self):
        test_pose = Pose()

        expected = True
        actual = self.observer.ball_is_in_field(test_pose)
        self.assertEqual(expected, actual)

        test_pose.x, test_pose.y = 100, 100
        expected = False
        actual = self.observer.ball_is_in_field(test_pose)
        self.assertEqual(expected, actual)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_observer', TestObserver)
