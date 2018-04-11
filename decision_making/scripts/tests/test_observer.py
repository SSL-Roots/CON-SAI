#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

import constants
from observer import Observer
from world_model import State
from consai_msgs.msg import Pose
from consai_msgs.msg import Pose as Velocity

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

    def test_ball_is_moved(self):
        init_pose = Pose(0, 0, 0)
        self.observer.set_ball_initial_pose(init_pose)

        actual = self.observer.ball_is_moved(init_pose)
        expected = False
        self.assertEqual(expected, actual)

        test_pose = Pose(1, 0, 0)
        actual = self.observer.ball_is_moved(test_pose)
        expected = True
        self.assertEqual(expected, actual)

        self.observer.set_ball_initial_pose(test_pose)
        actual = self.observer.ball_is_moved(test_pose)
        expected = False
        self.assertEqual(expected, actual)

    def test_ball_is_in_defence_area(self):
        our_side = True
        self._test_in_defence(Pose(0, 0, 0), our_side, False)
        self._test_in_defence(Pose(-constants.FieldHalfX, 0, 0), our_side, True)
        self._test_in_defence(Pose(constants.FieldHalfX, 0, 0), our_side, False)

        our_side = False
        self._test_in_defence(Pose(0, 0, 0), our_side, False)
        self._test_in_defence(Pose(-constants.FieldHalfX, 0, 0), our_side, False)
        self._test_in_defence(Pose(constants.FieldHalfX, 0, 0), our_side, True)

    def _test_in_defence(self, pose, our_side, expected):
        actual = self.observer.ball_is_in_defence_area(pose, our_side)
        self.assertEqual(expected, actual)

    def test_ball_is_moving(self):
        expected = False
        actual = self.observer.ball_is_moving(Velocity(0, 0, 0))
        self.assertEqual(expected, actual)

        expected = True
        actual = self.observer.ball_is_moving(Velocity(2, 0, 0))
        self.assertEqual(expected, actual)

        expected = False
        actual = self.observer.ball_is_moving(Velocity(0, 0, 0))
        self.assertEqual(expected, actual)

    def test_is_on_trajectory(self):
        target_pose = Pose(0, 0, 0)
        target_velocity = Pose(2, 0, 0)

        my_pose = Pose(3, 1, 0)

        result, dist = self.observer.is_on_trajectory(my_pose, target_pose, target_velocity)
        self.assertEqual(result, True)
        self.assertAlmostEqual(dist , 1)
        
        my_pose = Pose(2, -3, 0)

        result, dist = self.observer.is_on_trajectory(my_pose, target_pose, target_velocity)
        self.assertEqual(result, False)
        self.assertAlmostEqual(dist , 3)

    def test_are_no_obstacles(self):
        object_states = dict()
        object_states['Role_0'] = State()
        object_states['Role_0'].set_all(Pose(1, 0, 0), Velocity(0, 0, 0))

        start_pose = Pose(0, 0, 0)
        target_pose = Pose(3, 0, 0)
        expected = False
        actual = self.observer.are_no_obstacles(start_pose, target_pose, object_states)

        object_states['Enemy_0'] = State()
        object_states['Enemy_0'].set_all(Pose(0, 1, 0), Velocity(0, 0, 0))
        target_pose = Pose(0, 2, 0)
        expected = False
        actual = self.observer.are_no_obstacles(start_pose, target_pose, object_states)

        object_states['Ball_0'] = State()
        object_states['Ball_0'].set_all(Pose(-3, 0, 0), Velocity(0, 0, 0))
        target_pose = Pose(-2, 0, 0)
        expected = True
        actual = self.observer.are_no_obstacles(start_pose, target_pose, object_states)

    def test_can_receive(self):
        object_states = dict()
        object_states['Ball'] = State()
        object_states['Role_0'] = State()

        test_pose = Pose(0, 0, 0)
        test_vel = Velocity(2, 0, 0)
        object_states['Ball'].set_all(test_pose, test_vel)

        expected = False
        actual = self.observer.can_receive('Role_0', object_states)
        self.assertEqual(expected, actual, "Why does Role_0 exist?")

        test_pose = Pose(-1, 0, 0)
        test_vel = Velocity(0, 0, 0)
        object_states['Role_0'].set_all(test_pose, test_vel)

        expected = False
        actual = self.observer.can_receive('Role_0', object_states)
        self.assertEqual(expected, actual, "Why can Role_0 receive ball?")
        
        test_pose = Pose(0, 3, 0)
        test_vel = Velocity(0, 0, 0)
        object_states['Role_0'].set_all(test_pose, test_vel)

        expected = False
        actual = self.observer.can_receive('Role_0', object_states)
        self.assertEqual(expected, actual, "Why can Role_0 receive ball?")
        
        test_pose = Pose(0, 0, 0)
        test_vel = Velocity(0, 0, 0)
        object_states['Role_0'].set_all(test_pose, test_vel)

        expected = True
        actual = self.observer.can_receive('Role_0', object_states)
        self.assertEqual(expected, actual, "Why can not Role_0 receive ball?")

        test_pose = Pose(2, 0, 0)
        test_vel = Velocity(0, 0, 0)
        object_states['Role_0'].set_all(test_pose, test_vel)

        expected = True
        actual = self.observer.can_receive('Role_0', object_states)
        self.assertEqual(expected, actual, "Why can not Role_0 receive ball?")

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_observer', TestObserver)
