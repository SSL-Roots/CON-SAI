#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import tool
import constants
from world_model import WorldModel

from consai_msgs.msg import Pose


class Coordinate(object):

    def __init__(self):
        self.pose = Pose() # pos_x, pos_y, thta

        self._base = None
        self._target = None
        self._update_func = None

        # arrival parameters
        self._arrived_position_tolerance = 0.05 # unit:meter
        self._arrived_angle_tolerance = 15.0 * math.pi / 180.0

        # interpose
        self._to_dist = None
        self._from_dist = None

        # approach to shoot
        self._pose_max = Pose()
        self._my_role = None
        self._role_is_lower_side = False
        self._role_pose_hystersis = 0.1
        self._tuning_param_x = 0.5
        self._tuning_param_y = 0.3
        self._tuning_param_pivot_y = 0.3
        self._tuning_angle = 30.0 * math.pi / 180.0  # 0 ~ 90 degree, do not edit 'math.pi / 180.0'


    def update(self):
        result = False
        if self._update_func:
            result = self._update_func()

        return result


    def set_interpose(self, base="CONST_OUR_GOAL", target="Ball", to_dist=None, from_dist=None):

        self._base = base
        self._target = target
        self._to_dist = to_dist
        self._from_dist = from_dist

        self._update_func = self._update_interpose

    
    def set_approach_to_shoot(self, my_role=None, target="CONST_THEIR_GOAL"):

        self._my_role = my_role
        self._target = target
        self._pose_max.x = constants.BallRadius + self._tuning_param_x
        self._pose_max.y = constants.BallRadius + constants.RobotRadius + self._tuning_param_y

        self._update_func = self._update_approach_to_shoot


    def is_arrived(self, role):
        # robotが目標位置に到着したかを判断する
        # 厳し目につける

        role_pose = WorldModel.get_pose(role)

        if role_pose is None:
            return False

        arrived = False

        distance = tool.getSize(self.pose, role_pose)

        # 目標位置との距離、目標角度との差がtolerance以下であれば到着判定
        if distance < self._arrived_position_tolerance:
            diff_angle = self.pose.theta - role_pose.theta
            
            if tool.normalize(diff_angle) < self._arrived_angle_tolerance:
                arrived = True

        return arrived


    def _update_interpose(self):
        base_pose = WorldModel.get_pose(self._base)
        target_pose = WorldModel.get_pose(self._target)

        if base_pose is None or target_pose is None:
            return False

        angle_to_target = tool.getAngle(base_pose, target_pose)
        
        interposed_pose = Pose(0, 0, 0)
        if not self._to_dist is None:
            trans = tool.Trans(base_pose, angle_to_target)
            tr_interposed_pose = Pose(self._to_dist, 0.0, 0)
            interposed_pose = trans.invertedTransform(tr_interposed_pose)
        elif not self._from_dist is None:
            angle_to_base = tool.getAngle(target_pose, base_pose)
            trans = tool.Trans(target_pose, angle_to_base)
            tr_interposed_pose = Pose(self._from_dist, 0.0, 0)
            interposed_pose = trans.invertedTransform(tr_interposed_pose)

        interposed_pose.theta = angle_to_target

        self.pose = interposed_pose
        
        return True


    def _update_approach_to_shoot(self):
        # Reference to this idea
        # http://wiki.robocup.org/images/f/f9/Small_Size_League_-_RoboCup_2014_-_ETDP_RoboDragons.pdf

        ball_pose = WorldModel.get_pose('Ball')
        target_pose = WorldModel.get_pose(self._target)
        role_pose = WorldModel.get_pose(self._my_role)

        if target_pose is None or role_pose is None:
            return False

        # ボールからターゲットを見た座標系で計算する
        angle_ball_to_target = tool.getAngle(ball_pose, target_pose)
        trans = tool.Trans(ball_pose, angle_ball_to_target)
        tr_role_pose = trans.transform(role_pose)

        # tr_role_poseのloser_side判定にヒステリシスをもたせる
        if self._role_is_lower_side == True and \
                tr_role_pose.y > self._role_pose_hystersis:
            self._role_is_lower_side = False

        elif self._role_is_lower_side == False and \
                tr_role_pose.y < - self._role_pose_hystersis:
            self._role_is_lower_side = True

        if self._role_is_lower_side:
            tr_role_pose.y *= -1.0


        tr_approach_pose = Pose(0, 0, 0)
        if tr_role_pose.x > 0:
            # 1.ボールの斜め後ろへ近づく

            # copysign(x,y)でyの符号に合わせたxを取得できる
            tr_approach_pose = Pose(
                    -self._pose_max.x,
                    math.copysign(self._pose_max.y, tr_role_pose.y), 
                    0)

        else:
            # ボール裏へ回るためのピボットを生成
            pivot_pose = Pose(0, self._tuning_param_pivot_y, 0)
            angle_pivot_to_role = tool.getAngle(pivot_pose,tr_role_pose)

            limit_angle = self._tuning_angle + math.pi * 0.5

            if tr_role_pose.y > self._tuning_param_pivot_y and \
                    angle_pivot_to_role < limit_angle:
                # 2.ボール後ろへ回りこむ
            
                diff_angle = limit_angle - angle_pivot_to_role
                decrease_coef = diff_angle / self._tuning_angle
            
                tr_approach_pose = Pose(
                        -self._pose_max.x,
                        self._pose_max.y * decrease_coef, 
                        0)
            
            else:
                # 3.ボールに向かう
                diff_angle = tool.normalize(angle_pivot_to_role - limit_angle)
                approach_coef = diff_angle / (math.pi * 0.5 - self._tuning_angle)
            
                if approach_coef > 1.0:
                    approach_coef = 1.0

                pos_x = approach_coef * (2.0 * constants.BallRadius 
                        - self._tuning_param_x) + self._tuning_param_x
            
                tr_approach_pose = Pose(-pos_x, 0, 0)

        # 上下反転していたapproach_poseを元に戻す
        if self._role_is_lower_side:
            tr_approach_pose.y *= -1.0

        self.pose = trans.invertedTransform(tr_approach_pose)
        self.pose.theta = angle_ball_to_target
        
        return True

