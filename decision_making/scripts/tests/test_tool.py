#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest
import math

from geometry_msgs.msg import Quaternion, Point
from consai_msgs.msg import Pose

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

import tool

class TestTool(unittest.TestCase):

    def setUp(self):
        pass
    
    def _assertAlmostEqualPose(self, pose1, pose2):
        self.assertAlmostEqual(pose1.x, pose2.x)
        self.assertAlmostEqual(pose1.y, pose2.y)
        self.assertAlmostEqual(pose1.theta, pose2.theta)

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
        self.assertAlmostEqual(expected.x, actual.x)
        self.assertAlmostEqual(expected.y, actual.y)

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

    def test_invert_angle(self):
        test_angle = math.radians(90)
        expected = math.radians(-90)
        actual = tool.invertAngle(test_angle)
        self.assertAlmostEqual(expected, actual)

    def test_get_angle(self):
        from_pose = Pose()

        to_pose = Pose()
        to_pose.x = 1.0
        self._test_get_angle(from_pose, to_pose, math.radians(0))

        to_pose.x = 0
        to_pose.y = 1.0
        self._test_get_angle(from_pose, to_pose, math.radians(90))

        to_pose.x = -1.0
        to_pose.y = 0
        self._test_get_angle(from_pose, to_pose, math.radians(180))
        
        to_pose.x = 0
        to_pose.y = -1.0
        self._test_get_angle(from_pose, to_pose, math.radians(-90))

    def _test_get_angle(self, from_pose, to_pose, expected):
        actual = tool.getAngle(from_pose, to_pose)
        self.assertAlmostEqual(expected, actual)

    def test_get_size(self):
        from_pose = Pose()

        to_pose = Pose()
        to_pose.x = 1.0
        self._test_get_size(from_pose, to_pose, 1.0)

        to_pose.x = 1.0
        to_pose.y = 1.0
        self._test_get_size(from_pose, to_pose, math.sqrt(2))

        to_pose.x = 0
        to_pose.y = -3.0
        self._test_get_size(from_pose, to_pose, 3.0)

    def _test_get_size(self, from_pose, to_pose, expected):
        actual = tool.getSize(from_pose, to_pose)
        self.assertAlmostEqual(expected, actual)

    def test_get_size_from_center(self):
        test_pose = Pose()
        test_pose.x = 1.0
        self._test_get_size_from_center(test_pose, 1.0)

        test_pose.x = 1.0
        test_pose.y = 1.0
        self._test_get_size_from_center(test_pose, math.sqrt(2))

    def _test_get_size_from_center(self, test_pose, expected):
        actual = tool.getSizeFromCenter(test_pose)
        self.assertAlmostEqual(expected, actual)

    def test_get_angle_from_center(self):
        test_pose = Pose()
        test_pose.x = 1.0
        self._test_get_angle_from_center(test_pose, math.radians(0))

        test_pose.x = 0
        test_pose.y = 1.0
        self._test_get_angle_from_center(test_pose, math.radians(90))

    def _test_get_angle_from_center(self, test_pose, expected):
        actual = tool.getAngleFromCenter(test_pose)
        self.assertAlmostEqual(expected, actual)

    def test_get_conjugate(self):
        test_pose = Pose()
        test_pose.x = 1
        test_pose.y = 1

        expected = Pose()
        expected.x = 1
        expected.y = -1

        actual = tool.getConjugate(test_pose)
        self._assertAlmostEqualPose(expected, actual)

    def test_get_intersection(self):
        pose1 = Pose(4, 3, 0)
        pose2 = Pose(-2, -3, 0)
        pose3 = Pose(4, -3, 0)
        pose4 = Pose(-2, 3, 0)
        expected = Pose(1, 0, 0)

        actual = tool.get_intersection(pose1, pose2, pose3, pose4)
        self._assertAlmostEqualPose(expected, actual)

        # not across lines
        pose1 = Pose(2, 3, 0)
        pose2 = Pose(1, 2, 0)
        expected = Pose(0, 1, 0)
        actual = tool.get_intersection(pose1, pose2, pose3, pose4)
        self._assertAlmostEqualPose(expected, actual)

        # parallel lines
        pose1 = Pose(1, 2, 0)
        pose2 = Pose(2, 1, 0)
        expected = None
        actual = tool.get_intersection(pose1, pose2, pose3, pose4)
        self.assertEqual(expected, actual)

    def test_limit(self):
        test_value = 1
        test_high = 2
        test_low = -3

        expected = 1
        actual = tool.limit(test_value, test_high, test_low)
        self.assertEqual(expected, actual)

        test_value = 3
        expected = test_high
        actual = tool.limit(test_value, test_high, test_low)
        self.assertEqual(expected, actual)

        test_value = -10
        expected = test_low
        actual = tool.limit(test_value, test_high, test_low)
        self.assertEqual(expected, actual)

    def test_trans(self):
        from_pose = Pose(1, 1, 0)
        to_pose = Pose(1, 2, 0)
        angle = tool.getAngle(from_pose, to_pose)

        trans = tool.Trans(from_pose, angle)

        self._test_transform(trans, from_pose, Pose(0, 0, 0))
        self._test_transform(trans, to_pose, Pose(1, 0, 0))
        self._test_transform(trans, Pose(1, -2, 0), Pose(-3, 0, 0))

        self._test_transform_angle(trans, math.radians(0), math.radians(-90))
        self._test_transform_angle(trans, math.radians(90), math.radians(0))
        self._test_transform_angle(trans, math.radians(180), math.radians(90))
        self._test_transform_angle(trans, math.radians(-90), math.radians(-180))

    def _test_transform(self, trans, pose, expected):
        trPose = trans.transform(pose)
        self._assertAlmostEqualPose(expected, trPose)

        invPose = trans.invertedTransform(trPose)
        self._assertAlmostEqualPose(pose, invPose)

    def _test_transform_angle(self, trans, angle, expected):
        trAngle = trans.transformAngle(angle)
        self.assertAlmostEqual(expected, trAngle)

        invAngle = trans.invertedTransformAngle(trAngle)
        self.assertAlmostEqual(angle, invAngle)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_tool', TestTool)
