#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

from world_model import WorldModel
import tool
import constants


class Observer(object):

    def __init__(self):
        self._hysteresis = 0.05 # unit:meter
        self._ball_is_in_field = False


    def update(self):
        self._update_ball_position()

    
    def ball_is_in_field(self):
        return self._ball_is_in_field


    def _update_ball_position(self):
        ball_pose = WorldModel.get_pose('Ball')
        
        self._ball_is_in_field = self._is_in_field(ball_pose, self._ball_is_in_field)


    def _is_in_field(self, pose, is_in_field):
        fabs_x = math.fabs(pose.x)
        fabs_y = math.fabs(pose.y)

        if is_in_field == True:
            if fabs_x > 4.5 + self._hysteresis or fabs_y > 3.0 + self._hysteresis:
                is_in_field = False

        else:
            if fabs_x < 4.5 - self._hysteresis and fabs_y < 3.0 - self._hysteresis:
                is_in_field = True

        return is_in_field


