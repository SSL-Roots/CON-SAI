#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest
import math

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
        self.assertEqual(expected, actual)

        object_states['Enemy_0'] = State()
        object_states['Enemy_0'].set_all(Pose(0, 1, 0), Velocity(0, 0, 0))
        target_pose = Pose(0, 2, 0)
        expected = False
        actual = self.observer.are_no_obstacles(start_pose, target_pose, object_states)
        self.assertEqual(expected, actual)

        object_states['Ball'] = State()
        object_states['Ball'].set_all(Pose(-3, 0, 0), Velocity(0, 0, 0))
        target_pose = Pose(-2, 0, 0)
        expected = True
        actual = self.observer.are_no_obstacles(start_pose, target_pose, object_states)
        self.assertEqual(expected, actual)

        start_pose = Pose(0, 0, 0)
        target_pose = Pose(3, 0, 0)
        expected = True
        actual = self.observer.are_no_obstacles(start_pose, target_pose, object_states, exclude_key='Role_0')
        self.assertEqual(expected, actual)

    def test_can_receive(self):
        object_states = dict()
        object_states['Ball'] = State()
        object_states['Role_0'] = State()

        test_pose = Pose(0, 0, 0)
        test_vel = Velocity(2, 0, 0)
        object_states['Ball'].set_all(test_pose, test_vel)
        self._test_can_receive('Role_0', object_states, False)

        test_pose = Pose(-1, 0, 0)
        test_vel = Velocity(0, 0, 0)
        object_states['Role_0'].set_all(test_pose, test_vel)
        self._test_can_receive('Role_0', object_states, False)
        
        test_pose = Pose(0, 3, 0)
        test_vel = Velocity(0, 0, 0)
        object_states['Role_0'].set_all(test_pose, test_vel)
        self._test_can_receive('Role_0', object_states, False)

        test_pose = Pose(0, 0, 0)
        test_vel = Velocity(0, 0, 0)
        object_states['Role_0'].set_all(test_pose, test_vel)
        self._test_can_receive('Role_0', object_states, True)

        test_pose = Pose(2, 0, 0)
        test_vel = Velocity(0, 0, 0)
        object_states['Role_0'].set_all(test_pose, test_vel)
        self._test_can_receive('Role_0', object_states, True)

    def _test_can_receive(self, role, object_states, expected):
        actual = self.observer.can_receive(role, object_states)
        self.assertEqual(expected, actual)

    def test_can_shoot(self):
        object_states = dict()
        object_states['Ball'] = State()
        object_states['Role_0'] = State()
        object_states['Enemy_0'] = State()

        object_states['Ball'].set_all(Pose(0,0,0), Velocity(0,0,0))

        target = Pose(4.5, 0, 0)
        expected = True
        actual = self.observer.can_shoot('Role_0', target, object_states)
        self.assertEqual(expected, actual)
        
        object_states['Role_0'].set_all(Pose(0,0,0), Velocity(0,0,0))
        object_states['Enemy_0'].set_all(Pose(0,0,0), Velocity(0,0,0))
        expected = True
        actual = self.observer.can_shoot('Role_0', target, object_states)
        self.assertEqual(expected, actual)

        object_states['Role_0'].set_pose(Pose(0,1,0))
        object_states['Enemy_0'].set_pose(Pose(0,-1,0))
        expected = True
        actual = self.observer.can_shoot('Role_0', target, object_states)
        self.assertEqual(expected, actual)

        object_states['Enemy_0'].set_pose(Pose(1,0.05,0))
        expected = False
        actual = self.observer.can_shoot('Role_0', target, object_states)
        self.assertEqual(expected, actual)

    def test_can_pose_shoot(self):
        object_states = dict()
        object_states['Ball'] = State()
        object_states['Role_0'] = State()
        object_states['Enemy_0'] = State()

        object_states['Ball'].set_all(Pose(0,0,0), Velocity(0,0,0))
        object_states['Role_0'].set_all(Pose(0,1,0), Velocity(0,0,0))
        object_states['Enemy_0'].set_all(Pose(2,1,0), Velocity(0,0,0))

        exclude_role = 'Role_0'
        from_pose = object_states[exclude_role].get_pose()
        expected = True
        actual = self.observer.can_pose_shoot(exclude_role, from_pose, object_states)
        self.assertEqual(expected, actual)


    def test_can_pass(self):
        object_states = dict()
        object_states['Ball'] = State()
        object_states['Role_0'] = State()
        object_states['Role_1'] = State()
        object_states['Role_2'] = State()
        object_states['Enemy_0'] = State()

        object_states['Ball'].set_all(Pose(0,0,0), Velocity(0,0,0))
        object_states['Role_0'].set_all(Pose(1,0,0), Velocity(0,0,0))
        object_states['Role_1'].set_all(Pose(-2,0,0), Velocity(0,0,0))
        object_states['Role_2'].set_all(Pose(-3,0,0), Velocity(0,0,0))
        object_states['Enemy_0'].set_all(Pose(-1,0,0), Velocity(0,0,0))

        expected_result = False
        expected_role  = None
        result, target_role = self.observer.can_pass('Role_0', object_states)
        self.assertEqual(expected_result, result)
        self.assertEqual(expected_role, target_role)

        object_states['Role_1'].set_all(Pose(-2,2,0), Velocity(0,0,0))
        expected_result = True
        expected_role  = 'Role_1'
        result, target_role = self.observer.can_pass('Role_0', object_states)
        self.assertEqual(expected_result, result)
        self.assertEqual(expected_role, target_role)

        object_states['Role_2'].set_all(Pose(2,2,0), Velocity(0,0,0))
        expected_result = True
        expected_role  = 'Role_2'
        result, target_role = self.observer.can_pass('Role_0', object_states)
        self.assertEqual(expected_result, result)
        self.assertEqual(expected_role, target_role)

    def test_can_reflect(self):
        object_states = dict()
        object_states['Ball'] = State()
        object_states['Role_0'] = State()

        object_states['Ball'].set_all(Pose(0,0,0), Velocity(0,0,0))
        object_states['Role_0'].set_all(Pose(0,0,0), Velocity(0,0,0))

        target_pose = constants.poses['CONST_THEIR_GOAL']

        expected = False
        actual = self.observer.can_reflect('Role_0', target_pose, object_states)
        self.assertEqual(expected, actual)

        object_states['Ball'].set_all(Pose(1,0,0), Velocity(2,0,0))
        actual = self.observer.can_reflect('Role_0', target_pose, object_states)
        self.assertEqual(expected, actual)

        expected = True
        object_states['Ball'].set_all(Pose(1,0,0), Velocity(-2,0,0))
        actual = self.observer.can_reflect('Role_0', target_pose, object_states)
        self.assertEqual(expected, actual)

        expected = True
        object_states['Ball'].set_all(Pose(2,0.5,0), Velocity(-2,-0.5,0))
        actual = self.observer.can_reflect('Role_0', target_pose, object_states)
        self.assertEqual(expected, actual)

        expected = True
        object_states['Ball'].set_all(Pose(2,-0.5,0), Velocity(-2,0.5,0))
        actual = self.observer.can_reflect('Role_0', target_pose, object_states)
        self.assertEqual(expected, actual)

    def test_closest_role(self):
        object_states = dict()
        object_states['Ball'] = State()
        object_states['Role_0'] = State()
        object_states['Role_1'] = State()
        object_states['Role_2'] = State()
        object_states['Role_3'] = State()
        object_states['Enemy_0'] = State()

        object_states['Ball'].set_all(Pose(0,0,0), Velocity(0,-2,0))
        object_states['Role_0'].set_all(Pose(1,0,0), Velocity(0,0,0))
        object_states['Role_1'].set_all(Pose(0,1.2,0), Velocity(0,0,0))
        object_states['Role_2'].set_all(Pose(-2,0,0), Velocity(0,0,0))
        object_states['Role_3'].set_all(Pose(0,-4,0), Velocity(0,0,0))
        object_states['Enemy_0'].set_all(Pose(-1,0,0), Velocity(0,0,0))

        expected = 'Role_0'
        actual = self.observer.closest_role(Pose(0,0,0), object_states, True)
        self.assertEqual(expected, actual)

        expected = 'Role_1'
        actual = self.observer.closest_role(Pose(0,0,0), object_states, True, 'Role_1')
        self.assertEqual(expected, actual)

        expected = 'Role_3'
        actual = self.observer.closest_role(Pose(0,0,0), object_states, True, 
                'Role_1', True)
        self.assertEqual(expected, actual)

        expected = 'Enemy_0'
        actual = self.observer.closest_role(Pose(0,0,0), object_states, False)
        self.assertEqual(expected, actual)

        object_states['Enemy_0'].desable()
        expected = None
        actual = self.observer.closest_role(Pose(0,0,0), object_states, False)
        self.assertEqual(expected, actual)
        
    def test_is_looking(self):
        my_pose = Pose(0, 0, math.radians(45))
        target_pose = Pose(1, 1, 0)

        expected = True
        actual = self.observer.is_looking(my_pose, target_pose)
        self.assertEqual(expected, actual)

        my_pose = Pose(0, 0, math.radians(90))
        expected = False
        actual = self.observer.is_looking(my_pose, target_pose)
        self.assertEqual(expected, actual)

    def test_has_ball(self):
        object_states = dict()
        object_states['Ball'] = State()
        object_states['Role_0'] = State()

        object_states['Ball'].set_all(Pose(0,0,0), Velocity(0,0,0))
        object_states['Role_0'].set_all(Pose(-0.01,0,0), Velocity(0,0,0))

        expected = True
        actual = self.observer.has_ball('Role_0', object_states)
        self.assertEqual(expected, actual)

        object_states['Role_0'].set_all(Pose(0.01,0,math.radians(180)), Velocity(0,0,0))
        expected = True
        actual = self.observer.has_ball('Role_0', object_states)
        self.assertEqual(expected, actual)

        object_states['Role_0'].set_all(Pose(-0.01,1,0), Velocity(0,0,0))
        expected = False
        actual = self.observer.has_ball('Role_0', object_states)
        self.assertEqual(expected, actual)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_observer', TestObserver)
