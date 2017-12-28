#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

from consai_msgs.msg import Pose
from consai_msgs.msg import Pose as Velocity
import tool
import constants

import rospy

class Observer(object):

    def __init__(self):
        self._hysteresis = 0.05 # unit:meter
        self._moved_threshold = 0.03 # unit:meter
        self._ball_initial_pose = Pose()
        self._moving_speed_threshold = 1.0
        self._moving_speed_hysteresis = 0.3

        self._ball_is_in_field = False
        self._ball_is_moved = False
        self._ball_is_in_our_defence = False
        self._ball_is_in_their_defence = False
        self._ball_is_moving = False


    def ball_is_in_field(self, pose):
        fabs_x = math.fabs(pose.x)
        fabs_y = math.fabs(pose.y)

        if self._ball_is_in_field == True:
            if fabs_x > constants.FieldHalfX + self._hysteresis or \
                    fabs_y > constants.FieldHalfY + self._hysteresis:
                self._ball_is_in_field = False

        else:
            if fabs_x < constants.FieldHalfX - self._hysteresis and \
                    fabs_y < constants.FieldHalfY - self._hysteresis:
                self._ball_is_in_field = True

        return self._ball_is_in_field


    def set_ball_initial_pose(self, pose):
        self._ball_initial_pose = pose
        self._ball_is_moved = False


    def ball_is_moved(self, pose):
        if tool.getSize(self._ball_initial_pose, pose) > self._moved_threshold:
            self._ball_is_moved = True

        return self._ball_is_moved
        

    def ball_is_in_defence_area(self, pose, our_side=False):
        target_upper = Pose()
        target_lower = Pose()
        is_in_defence = False

        if our_side:
            target_upper = constants.poses['CONST_OUR_GOAL_UPPER']
            target_lower = constants.poses['CONST_OUR_GOAL_LOWER']
            is_in_defence = self._ball_is_in_our_defence
        else:
            target_upper = constants.poses['CONST_THEIR_GOAL_UPPER']
            target_lower = constants.poses['CONST_THEIR_GOAL_LOWER']
            is_in_defence = self._ball_is_in_their_defence


        if is_in_defence:
            threshold = constants.DefenceLength + self._hysteresis

            if tool.getSize(pose, target_upper) > threshold and \
                    tool.getSize(pose, target_lower) > threshold:
                is_in_defence = False

        else:
            threshold = constants.DefenceLength - self._hysteresis

            if tool.getSize(pose, target_upper) < threshold or \
                    tool.getSize(pose, target_lower) < threshold:
                is_in_defence = True


        if our_side:
            self._ball_is_in_our_defence = is_in_defence
        else:
            self._ball_is_in_their_defence = is_in_defence


        return is_in_defence


    def ball_is_moving(self, velocity):
        ball_speed = tool.getSizeFromCenter(velocity)

        if self._ball_is_moving == False and \
                ball_speed > self._moving_speed_threshold + self._moving_speed_hysteresis:
            self._ball_is_moving = True

        elif self._ball_is_moving == True and \
                ball_speed < self._moving_speed_threshold - self._moving_speed_hysteresis:
            self._ball_is_moving = False

        return self._ball_is_moving


