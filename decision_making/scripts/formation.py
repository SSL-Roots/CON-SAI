#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import re

from consai_msgs.msg import Pose
import tool
import constants

import rospy

class Formation(object):

    def __init__(self):
        self._default_poses = [
                Pose(0.5, 0.7, 0),
                Pose(0.5, 0, 0),
                Pose(0.5, -0.7, 0),
                Pose(0.3, 0.5, 0),
                Pose(0.3, -0.5, 0),
                Pose(0, 0.7, 0),
                Pose(0, 0, 0),
                Pose(0, -0.7, 0),
                Pose(-0.5, 0.7, 0),
                Pose(-0.5, 0, 0),
                Pose(-0.5, -0.7, 0)
                ]
        self._target_poses = [Pose()] * len(self._default_poses)

        self._target_number = dict()

        for i in range(constants.ROBOT_NUM):
            role_name = 'Role_' + str(i)
            self._target_number[role_name] = None

        self._DEFAULT_THRESH = 1.0


    def intialize_poses(self):
        for i, pose in enumerate(self._default_poses):
            self._target_poses[i] = Pose(
                    pose.x * constants.FieldHalfX,
                    pose.y * constants.FieldHalfY,
                    0)


    def get_pose(self, role_name):
        number = self._target_number[role_name]
        if number is None:
            return Pose(0,0,0)

        return self._target_poses[number]


    def update(self, object_states):
        ball_pose = object_states['Ball'].get_pose()

        for role in object_states.keys():
            if not re.match('Role', role):
                continue

            # Roleが存在しなければ、target_numberをリセットする
            state = object_states[role]
            if state.is_enabled() is False:
                self._target_number[role] = None
                continue

            prev_number = self._target_number[role]
            role_pose = state.get_pose()
            self._target_number[role] = self._select_target_number(
                    ball_pose, role_pose, prev_number)


    def _select_target_number(self, ball_pose, role_pose, prev_number):
        threshold = self._DEFAULT_THRESH
        result_number = None

        for i, pose in enumerate(self._target_poses):
            value = 0

            if i == prev_number:
                # 前回と同じnumberであれば加算
                value += 1.0
            elif i in self._target_number.values():
                # 他のメンバのnumberであればスキップ
                continue

            # ボールに近すぎれば減算
            # ボールから離れているほど加算
            dist_to_ball = tool.getSize(pose, ball_pose)
            if dist_to_ball < 0.5:
                value -= 1.0
            else:
                value += dist_to_ball / constants.FieldX

            # role_poseに近いほど加算
            dist_to_role = tool.getSize(pose, role_pose)
            value += 1.0 - dist_to_role / constants.FieldX

            if value > threshold:
                result_number = i
                threshold = value

        return result_number

