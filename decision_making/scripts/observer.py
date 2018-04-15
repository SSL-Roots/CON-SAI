#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import re

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
        self._is_on_threshold_x = 0.5
        self._is_on_threshold_y = 2.0

        self._ball_is_in_field = False
        self._ball_is_moved = False
        self._ball_is_in_our_defence = False
        self._ball_is_in_their_defence = False
        self._ball_is_moving = False

        # receive_ball
        self._can_receive_dist = 1.0 # unit:meter
        self._can_receive_hysteresis = 0.3
        self._receiving = dict()
        for i in range(6):
            self._receiving['Role_' + str(i)] = False

        # can_shoot
        self._can_shoot_width = 0.1 # unit:meter
        self._can_shoot_hysteresis = 0.03
        self._shooting = False

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
        is_in_defence = False

        if our_side:
            is_in_defence = self._is_in_our_defence(pose, self._ball_is_in_our_defence)
            self._ball_is_in_our_defence = is_in_defence
        else:
            is_in_defence = self._is_in_their_defence(pose, self._ball_is_in_their_defence)
            self._ball_is_in_their_defence = is_in_defence

        return is_in_defence

    
    def _is_in_our_defence(self, pose, is_in_defence):
        target_x = pose.x
        target_y = math.fabs(pose.y)
        if is_in_defence:
            target_x -= self._hysteresis
            target_y -= self._hysteresis
        
        if target_y < constants.PenaltyY and \
            target_x < -constants.PenaltyX:
        
            return True

        return False


    def _is_in_their_defence(self, pose, is_in_defence):
        target_x = pose.x
        target_y = math.fabs(pose.y)
        if is_in_defence:
            target_x += self._hysteresis
            target_y -= self._hysteresis
        
        if target_y < constants.PenaltyY and \
            target_x > constants.PenaltyX:
        
            return True
        
        return False


    def ball_is_moving(self, velocity):
        ball_speed = tool.getSizeFromCenter(velocity)

        if self._ball_is_moving == False and \
                ball_speed > self._moving_speed_threshold + self._moving_speed_hysteresis:
            self._ball_is_moving = True

        elif self._ball_is_moving == True and \
                ball_speed < self._moving_speed_threshold - self._moving_speed_hysteresis:
            self._ball_is_moving = False

        return self._ball_is_moving


    def is_on_trajectory(self, my_pose, target_pose, target_velocity):
        angle = tool.getAngleFromCenter(target_velocity)

        trans = tool.Trans(target_pose, angle)

        my_trans_pose = trans.transform(my_pose)

        dist_to_trajectory = math.fabs(my_trans_pose.y)

        if my_trans_pose.x > self._is_on_threshold_x and \
            dist_to_trajectory < self._is_on_threshold_y:
                
            return True, dist_to_trajectory
        else:
            return False, dist_to_trajectory


    def are_no_obstacles(self, start_pose, target_pose, object_states, 
            start_dist = 0.01, check_width = 0.1):
        no_obstacles = True

        angle_to_target = tool.getAngle(start_pose, target_pose)
        trans = tool.Trans(start_pose, angle_to_target)
        trTarget = trans.transform(target_pose)
        for key, state in object_states.items():
            if state.is_enabled is False:
                continue

            pose = state.get_pose()
            tr_pose = trans.transform(pose)

            if math.fabs(tr_pose.y) < check_width and \
                    tr_pose.x > start_dist and tr_pose.x < trTarget.x:

                no_obstacles = False
                break

        return no_obstacles

    def can_receive(self, role, object_states):
        result = False

        ball_pose = object_states['Ball'].get_pose()
        ball_vel = object_states['Ball'].get_velocity()

        if object_states[role].is_enabled() is False:
            self._receiving[role] = False
            return False

        if self.ball_is_moving(ball_vel):
            angle_velocity = tool.getAngleFromCenter(ball_vel)
            trans = tool.Trans(ball_pose, angle_velocity)

            role_pose = object_states[role].get_pose()

            tr_pose = trans.transform(role_pose)

            if tr_pose.x < 0:
                # role is on backside of ball
                self._receiving[role] = False
                return False

            fabs_y = math.fabs(tr_pose.y)

            if self._receiving[role] == False and \
                    fabs_y < self._can_receive_dist - self._can_receive_hysteresis:
                self._receiving[role] = True

            elif self._receiving[role] == True and \
                    fabs_y > self._can_receive_dist + self._can_receive_hysteresis:
                self._receiving[role] = False

            result = self._receiving[role]

        return result

    def can_shoot(self, target_pose, object_states):
        result = True

        ball_pose = object_states['Ball'].get_pose()
        angle_to_target = tool.getAngle(ball_pose, target_pose)
        trans = tool.Trans(ball_pose, angle_to_target)

        can_shoot_width = self._can_shoot_width
        if self._shooting:
            can_shoot_width -= self._can_shoot_hysteresis

        for key in object_states.keys():
            state = object_states[key]

            if state.is_enabled() is False:
                continue

            pose = state.get_pose()
            trPose = trans.transform(pose)

            if trPose.x > 0 and math.fabs(trPose.y) < can_shoot_width:
                result = False
                break

        self._shooting = result
        return result

    def can_pass(self, role, object_states):
        result = False
        target_role = None

        ball_pose = object_states['Ball'].get_pose()

        for key in object_states.keys():
            if not re.match('Role', key) or key == role:
                continue

            state = object_states[key]
            if state.is_enabled() is False:
                continue

            pose = state.get_pose()

            if self.are_no_obstacles(ball_pose, pose, object_states):
                result = True
                target_role = key
                break

        return result, target_role
        

