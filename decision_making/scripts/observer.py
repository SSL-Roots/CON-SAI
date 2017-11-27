#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import tool
import constants

import rospy

class Observer(object):

    def __init__(self):
        self._hysteresis = 0.05 # unit:meter
        self._ball_is_in_field = False

    
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
