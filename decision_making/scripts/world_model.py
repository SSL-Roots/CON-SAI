#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import re
from collections import OrderedDict, defaultdict

from nav_msgs.msg import Odometry
from consai_msgs.msg import Pose
from consai_msgs.msg import Pose as Velocity
from consai_msgs.msg import RefereeTeamInfo
from consai_msgs.msg import TestAICommand

import tool
import constants
import assignor
from proto.referee_pb2 import SSL_Referee
from command import Command
from observer import Observer


class State(object):
    def __init__(self):
        self._pose = Pose()
        self._velocity = Velocity()
        self._is_enabled = False

    def set_pose(self, pose):
        self._pose = pose

    def set_velocity(self, velocity):
        self._velocity = velocity

    def set_all(self, pose, velocity):
        self._pose = pose
        self._velocity = velocity
        self._is_enabled = True

    def enable(self):
        self._is_enabled = True

    def desable(self):
        self._is_enabled = False

    def get_pose(self):
        return self._pose

    def get_velocity(self):
        return self._velocity

    def is_enabled(self):
        return self._is_enabled


class WorldModel(object):
    _s = ['HALT', 'STOP', 'FORCE_START',
            'OUR_PRE_KICKOFF', 'OUR_KICKOFF_START',
            'OUR_PRE_PENALTY', 'OUR_PENALTY_START',
            'OUR_DIRECT', 'OUR_INDIRECT', 'OUR_TIMEOUT',
            'THEIR_PRE_KICKOFF', 'THEIR_KICKOFF_START',
            'THEIR_PRE_PENALTY', 'THEIR_PENALTY_START',
            'THEIR_DIRECT', 'THEIR_INDIRECT', 'THEIR_TIMEOUT',
            'BALL_IN_OUTSIDE', 'IN_PLAY',
            'BALL_IN_OUR_DEFENCE', 'BALL_IN_THEIR_DEFENCE']

    # テスト用のsituations
    _test = []
    for i in range(100):
        key = 'TEST' + str(i)
        _test.append(key)

    # KeyError を防ぐためdefaultdictを使う
    situations = defaultdict(lambda : False)
    for key in _s:
        situations[key] = False

    for key in _test:
        situations[key] = False

    _current_situation = 'HALT'
    _current_test = 'DUMMY_TEST'

    assignments = OrderedDict()
    enemy_assignments = OrderedDict()
    _threat_assignments = OrderedDict()

    _object_states = dict()
    _object_states['Ball'] = State()
    for i in range(6):
        key = 'Role_' + str(i)
        assignments[key] = None
        _object_states[key] = State()

        key = 'Enemy_' + str(i)
        enemy_assignments[key] = None
        _object_states[key] = State()

        key = 'Threat_' + str(i)
        _threat_assignments[key] = None

    commands = {'Role_0' : Command(), 'Role_1' : Command(),
                'Role_2' : Command(), 'Role_3' : Command(),
                'Role_4' : Command(), 'Role_5' : Command()}

    _ball_odom = Odometry()

    _friend_goalie_id = 0
    _friend_color = 'blue'
    _friend_odoms = [Odometry()] * 12
    _existing_friends_id = [None] * 6
    _friend_team_info = RefereeTeamInfo()

    _enemy_goalie_id = 0
    _enemy_odoms = [Odometry()] * 12
    _existing_enemies_id = [None] * 6
    _enemy_team_info = RefereeTeamInfo()

    tf_listener = tf.TransformListener()

    _raw_refbox_command = None
    _refbox_command_changed = False
    _current_refbox_command = ''

    _refbox_dict_blue = {SSL_Referee.HALT : 'HALT', SSL_Referee.STOP : 'STOP',
            SSL_Referee.NORMAL_START : 'NORMAL_START', 
            SSL_Referee.FORCE_START : 'FORCE_START',
            SSL_Referee.PREPARE_KICKOFF_BLUE : 'OUR_PRE_KICKOFF',
            SSL_Referee.PREPARE_PENALTY_BLUE : 'OUR_PRE_PENALTY',
            SSL_Referee.DIRECT_FREE_BLUE : 'OUR_DIRECT',
            SSL_Referee.INDIRECT_FREE_BLUE : 'OUR_INDIRECT',
            SSL_Referee.TIMEOUT_BLUE : 'OUR_TIMEOUT',
            SSL_Referee.PREPARE_KICKOFF_YELLOW : 'THEIR_PRE_KICKOFF',
            SSL_Referee.PREPARE_PENALTY_YELLOW : 'THEIR_PRE_PENALTY',
            SSL_Referee.DIRECT_FREE_YELLOW : 'THEIR_DIRECT',
            SSL_Referee.INDIRECT_FREE_YELLOW : 'THEIR_INDIRECT',
            SSL_Referee.TIMEOUT_YELLOW : 'THEIR_TIMEOUT'}

    _refbox_dict_yellow = {SSL_Referee.HALT : 'HALT', SSL_Referee.STOP : 'STOP',
            SSL_Referee.NORMAL_START : 'NORMAL_START', 
            SSL_Referee.FORCE_START : 'FORCE_START',
            SSL_Referee.PREPARE_KICKOFF_BLUE : 'THEIR_PRE_KICKOFF',
            SSL_Referee.PREPARE_PENALTY_BLUE : 'THEIR_PRE_PENALTY',
            SSL_Referee.DIRECT_FREE_BLUE : 'THEIR_DIRECT',
            SSL_Referee.INDIRECT_FREE_BLUE : 'THEIR_INDIRECT',
            SSL_Referee.TIMEOUT_BLUE : 'THEIR_TIMEOUT',
            SSL_Referee.PREPARE_KICKOFF_YELLOW : 'OUR_PRE_KICKOFF',
            SSL_Referee.PREPARE_PENALTY_YELLOW : 'OUR_PRE_PENALTY',
            SSL_Referee.DIRECT_FREE_YELLOW : 'OUR_DIRECT',
            SSL_Referee.INDIRECT_FREE_YELLOW : 'OUR_INDIRECT',
            SSL_Referee.TIMEOUT_YELLOW : 'OUR_TIMEOUT'}
    _refbox_dict = _refbox_dict_blue

    _observer = Observer()
    _ball_kicked_speed = 1.0

    _ball_closest_frined_role = None
    _ball_closest_enemy_role = None

    _test_ai_command = TestAICommand()

    _current_shoot_target = constants.poses['CONST_THEIR_GOAL']
    _current_pass_role = None

    @classmethod
    def update_world(cls):
        WorldModel._update_situation()
        rospy.logdebug('Referee: ' + WorldModel._current_refbox_command)
        rospy.logdebug('Situation: ' + WorldModel._current_situation)

        WorldModel._update_enemy_assignments()
        WorldModel._update_threat_assignments()
        WorldModel._update_object_states()

    
    @classmethod
    def update_assignments(cls, assignment_type=None):
        ball_pose = WorldModel.get_pose('Ball')

        WorldModel._ball_closest_frined_role = WorldModel._observer.closest_role(
                ball_pose, WorldModel._object_states, 
                True, WorldModel._ball_closest_frined_role)

        # Exclude goalie role
        closest_role = WorldModel._ball_closest_frined_role
        if closest_role == 'Role_0':
            closest_role = None

        WorldModel.assignments = assignor.update_assignments(
                WorldModel.assignments,
                WorldModel._existing_friends_id,
                'Role_',
                WorldModel._friend_goalie_id,
                assignment_type,
                closest_role)


    @classmethod
    def _update_enemy_assignments(cls):
        closest_role = None
        WorldModel.enemy_assignments = assignor.update_assignments(
                WorldModel.enemy_assignments,
                WorldModel._existing_enemies_id,
                'Enemy_',
                WorldModel._enemy_goalie_id)

    
    @classmethod
    def set_friend_color(cls, data):
        WorldModel._friend_color = data
        if data == 'blue':
            WorldModel._refbox_dict = WorldModel._refbox_dict_blue
        elif data == 'yellow':
            WorldModel._refbox_dict = WorldModel._refbox_dict_yellow


    @classmethod
    def set_friend_goalie_id(cls, robot_id):
        WorldModel._friend_goalie_id = robot_id


    @classmethod
    def set_blue_info(cls, team_info):
        if WorldModel._friend_color == 'blue':
            WorldModel._friend_team_info = team_info
        else:
            WorldModel._enemy_team_info = team_info
            WorldModel._enemy_goalie_id = team_info.goalie


    @classmethod
    def set_yellow_info(cls, team_info):
        if WorldModel._friend_color == 'yellow':
            WorldModel._friend_team_info = team_info
        else:
            WorldModel._enemy_team_info = team_info
            WorldModel._enemy_goalie_id = team_info.goalie


    def set_enemy_goalie_id(cls, robot_id):
        WorldModel._enemy_goalie_id = robot_id


    @classmethod
    def set_ball_odom(cls, msg):
        WorldModel._ball_odom = msg
        

    @classmethod
    def set_existing_friends_id(cls, data):
        WorldModel._existing_friends_id  = list(data)
        
    
    @classmethod
    def set_existing_enemies_id(cls, data):
        WorldModel._existing_enemies_id = list(data)
        

    @classmethod
    def set_friend_odom(cls, msg, robot_id):
        WorldModel._friend_odoms[robot_id] = msg


    @classmethod
    def set_enemy_odom(cls, msg, robot_id):
        WorldModel._enemy_odoms[robot_id] = msg


    @classmethod
    def set_refbox_command(cls, data):
        if WorldModel._raw_refbox_command != data and \
                data in WorldModel._refbox_dict:
            WorldModel._refbox_command_changed = True
            WorldModel._raw_refbox_command = data

    
    @classmethod
    def set_test_name(cls, data):
        WorldModel._current_test = data

    @classmethod
    def set_test_ai_command(cls, msg):
        WorldModel._test_ai_command = msg

    @classmethod
    def get_pose(cls, name):
        pose = None

        if re.match('Ball', name):
            ball_pose = WorldModel._ball_odom.pose.pose.position
            pose = Pose(ball_pose.x, ball_pose.y, 0)

        elif re.match('Role', name):
            robot_id = WorldModel.assignments[name]
            pose = WorldModel.get_friend_pose(robot_id)

        elif re.match('Enemy', name):
            robot_id = WorldModel.enemy_assignments[name]
            pose = WorldModel.get_enemy_pose(robot_id)

        elif re.match('Threat', name):
            robot_id = WorldModel._threat_assignments[name]
            pose = WorldModel.get_enemy_pose(robot_id)

        elif re.match('CONST', name):
            pose = constants.poses[name]
        
        elif re.match('Shoot', name):
            pose = WorldModel._current_shoot_target

        elif re.match('Pass', name):
            robot_id = WorldModel.assignments[WorldModel._current_pass_role]
            pose = WorldModel.get_friend_pose(robot_id)

        return pose


    @classmethod
    def get_velocity(cls, name):
        velocity = None

        if re.match('Ball', name):
            linear = WorldModel._ball_odom.twist.twist.linear
            velocity = Velocity(linear.x, linear.y, 0)

        elif re.match('Role', name):
            robot_id = WorldModel.assignments[name]
            velocity = WorldModel.get_friend_velocity(robot_id)

        elif re.match('Enemy', name):
            robot_id = WorldModel.enemy_assignments[name]
            velocity = WorldModel.get_enemy_velocity(robot_id)

        elif re.match('Threat', name):
            robot_id = WorldModel._threat_assignments[name]
            velocity = WorldModel.get_enemy_velocity(robot_id)

        return velocity

    
    @classmethod
    def ball_kicked(cls):
        kicked = False

        velocity = WorldModel.get_velocity('Ball')

        if tool.getSizeFromCenter(velocity) > WorldModel._ball_kicked_speed:
            kicked = True

        return kicked


    @classmethod
    def ball_is_moving(cls):
        velocity = WorldModel.get_velocity('Ball')

        return WorldModel._observer.ball_is_moving(velocity)


    @classmethod
    def get_friend_pose(cls, robot_id):

        if robot_id is None:
            return None

        position = WorldModel._friend_odoms[robot_id].pose.pose.position
        orientation = WorldModel._friend_odoms[robot_id].pose.pose.orientation
        yaw = tool.yawFromQuaternion(orientation)

        return Pose(position.x, position.y, yaw)
        

    @classmethod
    def get_enemy_pose(cls, robot_id):

        if robot_id is None:
            return None

        position = WorldModel._enemy_odoms[robot_id].pose.pose.position
        orientation = WorldModel._enemy_odoms[robot_id].pose.pose.orientation
        yaw = tool.yawFromQuaternion(orientation)

        return Pose(position.x, position.y, yaw)


    @classmethod
    def get_friend_velocity(cls, robot_id):

        if robot_id is None:
            return None

        linear = WorldModel._friend_odoms[robot_id].twist.twist.linear
        angular = WorldModel._friend_odoms[robot_id].twist.twist.angular

        return Velocity(linear.x, linear.y, angular.z)
        

    @classmethod
    def get_enemy_velocity(cls, robot_id):

        if robot_id is None:
            return None

        linear = WorldModel._enemy_odoms[robot_id].twist.twist.linear
        angular = WorldModel._enemy_odoms[robot_id].twist.twist.angular

        return Velocity(linear.x, linear.y, angular.z)

    @classmethod
    def get_test_ai_command(cls):
        return WorldModel._test_ai_command


    @classmethod
    def get_object_states(cls):
        return WorldModel._object_states


    @classmethod
    def _update_situation(cls):
        if WorldModel._refbox_command_changed:
            WorldModel._refbox_command_changed = False

            # raw_refbox_commandをチームカラーによって見方/敵commandへ加工する
            refbox_command = WorldModel._refbox_dict[WorldModel._raw_refbox_command]

            # NORMAL_STARTはKICKOFFとPENALTYのトリガーになるため、その切り分けを行う
            if refbox_command == 'NORMAL_START':
                if WorldModel._current_situation == 'OUR_PRE_KICKOFF':
                    refbox_command = 'OUR_KICKOFF_START'

                elif WorldModel._current_situation == 'OUR_PRE_PENALTY':
                    refbox_command = 'OUR_PENALTY_START'

                elif WorldModel._current_situation == 'THEIR_PRE_KICKOFF':
                    refbox_command = 'THEIR_KICKOFF_START'

                elif WorldModel._current_situation == 'THEIR_PRE_PENALTY':
                    refbox_command = 'THEIR_PENALTY_START'

                else:
                    refbox_command = 'FORCE_START'

            WorldModel._set_current_situation(refbox_command)
            WorldModel._current_refbox_command = refbox_command


        ball_pose = WorldModel.get_pose('Ball')

        # ボールが動いたらインプレイ判定、refbox_commandを上書きする
        if WorldModel._current_refbox_command[-5:] == 'START' or \
                WorldModel._current_refbox_command[-6:] == 'DIRECT':
            if WorldModel._observer.ball_is_moved(ball_pose):
                WorldModel._current_refbox_command = 'IN_PLAY'
        else:
            WorldModel._observer.set_ball_initial_pose(ball_pose)

        # current_refbox_commandがIN_PLAYのとき、ボール位置で戦況を判定する
        if WorldModel._current_refbox_command == 'IN_PLAY':
            WorldModel._set_current_situation('IN_PLAY')

            if WorldModel._observer.ball_is_in_defence_area(ball_pose, True):
                # 自分のディフェンスエリアに入ったか判定
                WorldModel._set_current_situation('BALL_IN_OUR_DEFENCE')

            elif WorldModel._observer.ball_is_in_defence_area(ball_pose, False):
                # 相手のディフェンスエリアに入ったか判定
                WorldModel._set_current_situation('BALL_IN_THEIR_DEFENCE')

        # ボールがフィールド外に出ることを判定
        # update_situationの最後に実行すること
        if WorldModel._observer.ball_is_in_field(ball_pose):
            if WorldModel._current_refbox_command != 'IN_PLAY':
                WorldModel._set_current_situation(WorldModel._current_refbox_command)
        else:
            WorldModel._set_current_situation('BALL_IN_OUTSIDE')

        # Test実行の判定
        if WorldModel._current_refbox_command != 'HALT' and \
                WorldModel._current_test in WorldModel.situations:
            WorldModel._set_current_situation(WorldModel._current_test)


    @classmethod
    def _set_current_situation(cls, situation):
        WorldModel.situations[WorldModel._current_situation] = False
        WorldModel._current_situation = situation
        WorldModel.situations[WorldModel._current_situation] = True


    @classmethod
    def _update_closest_role(cls, is_friend_role=True):
        # ボールに一番いroleをclosest_roleにセットする
        # ボールが動いている場合は、その軌道上にいるロボットがclosest_roleにする

        thresh_dist = 1000
        hysteresis = 0.2

        ball_pose = WorldModel.get_pose('Ball')
        ball_vel = WorldModel.get_velocity('Ball')
        ball_is_moving = WorldModel._observer.ball_is_moving(ball_vel)
        closest_role = None

        prev_closest_role = None
        role_text = ''
        
        if is_friend_role:
            role_text = 'Role_'
            prev_closest_role = WorldModel._ball_closest_frined_role
        else:
            role_text = 'Enemy_'
            prev_closest_role = WorldModel._ball_closest_enemy_role

        for i in range(6):
            role = role_text + str(i)
            pose = WorldModel.get_pose(role)
            
            if pose is None:
                continue

            if ball_is_moving:
                # ボールが動いてたら、ボール軌道に一番近いroleを抽出する

                # ロボットがボール軌道上にいるかチェック
                is_on_trajectory, dist = WorldModel._observer.is_on_trajectory(
                        pose, ball_pose, ball_vel)

                if is_on_trajectory:
                    # ヒステリシスをもたせる
                    if role == prev_closest_role:
                        dist -= hysteresis

                    if dist < thresh_dist:
                        thresh_dist = dist
                        closest_role = role

            else:
                # ボールが止まっていたら、ボールに一番近いroleを抽出する
                dist_to_ball = tool.getSize(pose, ball_pose)

                # ヒステリシスをもたせる
                if role == prev_closest_role:
                    dist_to_ball -= hysteresis

                if dist_to_ball < thresh_dist:
                    thresh_dist = dist_to_ball
                    closest_role = role


        if is_friend_role:
            WorldModel._ball_closest_frined_role = closest_role
        else:
            WorldModel._ball_closest_enemy_role = closest_role
            
        return closest_role


    @classmethod
    def _update_threat_assignments(cls):
        # Ballに一番近いEnemyをThreat_0にする
        closest_role = WorldModel._update_closest_role(False)
        if closest_role:
            closest_id = WorldModel.enemy_assignments[closest_role]
            WorldModel._threat_assignments['Threat_0'] = closest_id


    @classmethod
    def _update_object_states(cls):
        key = 'Ball'
        pose = WorldModel.get_pose(key)
        velocity = WorldModel.get_velocity(key)
        WorldModel._object_states[key].set_all(pose, velocity)

        WorldModel._set_states(WorldModel.assignments)
        WorldModel._set_states(WorldModel.enemy_assignments)

    @classmethod
    def _set_states(cls, assignments):
        for role, robot_id in assignments.items():
            if robot_id is None:
                WorldModel._object_states[role].desable()
                continue
                
            pose = WorldModel.get_pose(role)
            velocity = WorldModel.get_velocity(role)
            WorldModel._object_states[role].set_all(pose, velocity)

    @classmethod
    def can_receive(cls, role):
        return WorldModel._observer.can_receive(role, WorldModel._object_states)

    @classmethod
    def can_shoot(cls):
        target = WorldModel.get_pose('CONST_THEIR_GOAL')

        WorldModel._current_shoot_target = target

        return WorldModel._observer.can_shoot(target, WorldModel._object_states)

    @classmethod
    def can_pass(cls, role):
        result, pass_role = WorldModel._observer.can_pass(role, WorldModel._object_states)

        if result:
            WorldModel._current_pass_role = pass_role
        else:
            WorldModel._current_pass_role = None

        return result

