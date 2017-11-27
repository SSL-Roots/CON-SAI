#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

from consai_msgs.msg import Pose
import tool
import constants

import rospy

class Observer(object):

    def __init__(self):
        self._hysteresis = 0.05 # unit:meter
        self._moved_threshold = 0.3 # unit:meter
        self._ball_initial_pose = Pose()

        self._ball_is_in_field = False
        self._ball_is_moved = False


    def ball_is_in_field(self, pose):
        fabs_x = math.fabs(pose.x)
        fabs_y = math.fabs(pose.y)

        if self._ball_is_in_field == True:
            if fabs_x > 4.5 + self._hysteresis or fabs_y > 3.0 + self._hysteresis:
                self._ball_is_in_field = False

        else:
            if fabs_x < 4.5 - self._hysteresis and fabs_y < 3.0 - self._hysteresis:
                self._ball_is_in_field = True

        return self._ball_is_in_field


    def set_ball_initial_pose(self, pose):
        self._ball_initial_pose = pose
        self._ball_is_moved = False


    def ball_is_moved(self, pose):
        if tool.getSize(self._ball_initial_pose, pose) > self._moved_threshold:
            self._ball_is_moved = True

        return self._ball_is_moved
        
