#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest
import math

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

from coordinate import Coordinate
from consai_msgs.msg import Pose
from world_model import WorldModel


class TestCoordinate(unittest.TestCase):

    def setUp(self):
        self.coord = Coordinate()

    def _assertAlmostEqualPose(self, pose1, pose2):
        self.assertAlmostEqual(pose1.x, pose2.x)
        self.assertAlmostEqual(pose1.y, pose2.y)
        self.assertAlmostEqual(pose1.theta, pose2.theta)

    def test_generate_approach_pose(self):
        self.coord.set_approach_to_shoot('Role_0')

        self._test_generate_approach_pose(
                Pose(0,0,0), Pose(3,0,0), Pose(1, 0, 0), 1)
        self._test_generate_approach_pose(
                Pose(0,0,0), Pose(3,0,0), Pose(-0.5, -3, 0), 2)
        self._test_generate_approach_pose(
                Pose(0,0,0), Pose(3,0,0), Pose(-0.5, 3, 0), 2)

    def _test_generate_approach_pose(self, ball, target, role, expected_stage):
        pose, stage = self.coord._generate_approach_pose(ball, target, role)
        self.assertEqual(expected_stage, stage)


    def test_is_lower_side(self):
        self._test_is_lower_side(0.0, True, True)
        self._test_is_lower_side(0.0, False, False)
        self._test_is_lower_side(1.0, True, False)
        self._test_is_lower_side(1.0, False, False)
        self._test_is_lower_side(-1.0, True, True)
        self._test_is_lower_side(-1.0, False, True)

    def _test_is_lower_side(self, tr_role_pose_y, prev_is_lower_side, expected):
        actual = self.coord._is_lower_side(tr_role_pose_y, prev_is_lower_side)
        self.assertEqual(expected, actual)

    def test_get_angle_pivot_to_role(self):
        self.coord.set_approach_to_shoot('Role_0')

        tr_role_pose = Pose(0, 1, 0)
        is_lower_side = False
        expected = math.radians(90)
        actual = self.coord._get_angle_pivot_to_role(tr_role_pose, is_lower_side)
        self.assertAlmostEqual(expected, actual)

        tr_role_pose = Pose(0, -1, 0)
        is_lower_side = True
        expected = math.radians(-90)
        actual = self.coord._get_angle_pivot_to_role(tr_role_pose, is_lower_side)
        self.assertAlmostEqual(expected, actual)

    def test_generate_stage1_pose(self):
        self.coord.set_approach_to_shoot('Role_0')
        pose_max = self.coord._pose_max

        expected = Pose(pose_max.x, pose_max.y, 0)
        actual = self.coord._generate_stage1_pose(False)
        self._assertAlmostEqualPose(expected, actual)

        expected = Pose(pose_max.x, -pose_max.y, 0)
        actual = self.coord._generate_stage1_pose(True)
        self._assertAlmostEqualPose(expected, actual)

    @unittest.skipIf(True, "Value Check")
    def test_generate_stage2_pose(self):
        self.coord.set_approach_to_shoot('Role_0')
        pose_max = self.coord._pose_max
        limit_angle = self.coord._tuning_limit_angle

        expected = Pose(pose_max.x, 0, 0)
        actual = self.coord._generate_stage2_pose(math.radians(-70), True)
        self._assertAlmostEqualPose(expected, actual)

    @unittest.skipIf(True, "Value Check")
    def test_generate_stage3_pose(self):
        self.coord.set_approach_to_shoot('Role_0')
        pose_max = self.coord._pose_max

        expected = Pose(pose_max.x, 0, 0)
        actual = self.coord._generate_stage3_pose(math.radians(30), True)
        self._assertAlmostEqualPose(expected, actual)

    def test_reflect(self):
        self.coord.set_reflect('Role_0')

        id_list = [0]
        WorldModel.set_existing_friends_id(id_list)
        WorldModel.update_assignments()

        expected = True
        actual = self.coord.update()
        self.assertEqual(expected, actual)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_coordinate', TestCoordinate)
