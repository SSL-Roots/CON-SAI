#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest
import math

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

import constants
from formation import Formation
from observer import Observer
from world_model import State
from consai_msgs.msg import Pose
from consai_msgs.msg import Pose as Velocity

class TestFormation(unittest.TestCase):

    def setUp(self):
        self.formation = Formation()
        self.observer = Observer()

        self.formation.intialize_poses()

    def test_initialize_poses(self):
        self.formation.intialize_poses()

        # target_poseがdefence areaに入ってないことをチェック
        for pose in self.formation._target_poses:
            self._check_in_defence_area(pose)

    def _check_in_defence_area(self, pose):
        expected = False

        actual = self.observer._is_in_our_defence(pose, False)
        self.assertEqual(expected, actual)
        actual = self.observer._is_in_their_defence(pose, False)
        self.assertEqual(expected, actual)

    def test_get_pose(self):
        expected = Pose(0,0,0)
        actual = self.formation.get_pose('Role_0')
        self.assertEqual(expected, actual)

    def test_update(self):
        object_states = dict()
        object_states['Ball'] = State()
        object_states['Role_0'] = State()
        object_states['Role_1'] = State()
        object_states['Role_2'] = State()
        object_states['Role_3'] = State()
        object_states['Enemy_0'] = State()

        object_states['Ball'].set_all(Pose(0,0,0), Velocity(0,0,0))
        object_states['Role_0'].set_all(Pose(0,0,0), Velocity(0,0,0))
        object_states['Role_1'].set_all(Pose(0,0,0), Velocity(0,0,0))
        object_states['Role_2'].set_all(Pose(0,0,0), Velocity(0,0,0))
        object_states['Role_3'].set_all(Pose(0,0,0), Velocity(0,0,0))
        object_states['Enemy_0'].set_all(Pose(-1,0,0), Velocity(0,0,0))

        self.formation.update(object_states)

        pose = self.formation.get_pose('Role_0')
        self.assertNotEqual(pose, None)
        pose = self.formation.get_pose('Role_1')
        self.assertNotEqual(pose, None)
        pose = self.formation.get_pose('Role_2')
        self.assertNotEqual(pose, None)
        pose = self.formation.get_pose('Role_3')
        self.assertNotEqual(pose, None)

        pose = self.formation.get_pose('Role_4')
        self.assertEqual(pose, Pose(0,0,0))

        # Value Check
        # self.assertEqual(self.formation._target_number, False)

    def test_select_target_number(self):
        ball_pose = Pose(0,0,0)
        role_pose = Pose(1, 0, 0)
        prev_number = 5

        expected = None
        actual = self.formation._select_target_number(
                ball_pose, role_pose, prev_number)
        self.assertNotEqual(expected, actual)




if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_formation', TestFormation)
