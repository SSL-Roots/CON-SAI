#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest
import math

from std_msgs.msg import Time
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

from lib.pose_maker import PoseMaker
from proto import messages_robocup_ssl_detection_pb2 as detection


class TestPoseMaker(unittest.TestCase):

    def setUp(self):
        time = Time()
        frame_id = 0
        self._poses = PoseMaker(time, frame_id)


    def test_add_and_get(self):
        # Add detected ball pose
        ball = detection.SSL_DetectionBall()
        ball.x = 1000
        ball.y = 2000

        result = self._poses.add(ball)
        expected = True
        self.assertEqual(result, expected)

        # Add detected robot pose
        robot = detection.SSL_DetectionRobot()
        robot.x = 3000
        robot.y = 4000
        robot.orientation = math.pi

        result = self._poses.add(robot)
        expected = True
        self.assertEqual(result, expected)

    
        # Get pose array
        pose_array = self._poses.get()

        pose_ball = pose_array.poses[0]
        pose_robot = pose_array.poses[1]

        self._assert_equal_pose(ball, pose_ball)
        self._assert_equal_pose(robot, pose_robot)

        
        # get length
        result = self._poses.len()
        expected = 2
        self.assertEqual(result, expected)


    def test_clear_poses(self):
        self._poses.clear_poses()

        result = self._poses.len()
        expected = 0

        self.assertEqual(result, expected)


    def _assert_equal_pose(self, pose_detected, pose):
        expected = pose_detected.x / 1000
        result = pose.position.x
        self.assertAlmostEqual(result, expected, msg="Wrong value x")

        expected = pose_detected.y / 1000
        result = pose.position.y
        self.assertAlmostEqual(result, expected, msg="Wrong value y")

        if hasattr(pose_detected, 'orientation'):
            quat_tuple = quaternion_from_euler(0.0, 0.0, pose_detected.orientation)
            detected_orientation = Quaternion(*quat_tuple)

            self.assertAlmostEqual(detected_orientation.x, pose.orientation.x)
            self.assertAlmostEqual(detected_orientation.y, pose.orientation.y)
            self.assertAlmostEqual(detected_orientation.z, pose.orientation.z)
            self.assertAlmostEqual(detected_orientation.w, pose.orientation.w)

            


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('grsim_wrapper', 'test_pose_maker', TestPoseMaker)
