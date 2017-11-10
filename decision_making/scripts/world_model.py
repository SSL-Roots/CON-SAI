#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from collections import OrderedDict

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from consai_msgs.msg import Pose
from consai_msgs.msg import Pose as Velocity

from referee_pb2 import SSL_Referee
import tool
import constants

class Command(object):
    def __init__(self):
        self.target_pose = PoseStamped()
        self.target_velocity = Twist()
        self.aim = Pose()
        self.kick_power = 0.0
        self.dribble_power = 0.0
        self.velocity_control_enable = False
        self.chip_enable = False
        self.navigation_enable = False

    
    def set_target_pose(self, x, y, yaw, frame_id):
        self.target_pose.header.stamp = rospy.Time()
        self.target_pose.header.frame_id = frame_id

        self.target_pose.pose.position.x = x;
        self.target_pose.pose.position.y = y;

        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        self.target_pose.pose.orientation.x = quaternion[0]
        self.target_pose.pose.orientation.y = quaternion[1]
        self.target_pose.pose.orientation.z = quaternion[2]
        self.target_pose.pose.orientation.w = quaternion[3]

        self.velocity_control_enable = False


    def set_target_velocity(self, vx, vy, vyaw):
        self.target_velocity.linear.x = vx;
        self.target_velocity.linear.y = vy;
        self.target_velocity.angular.z = vyaw;
        self.velocity_control_enable = True

    
    def set_aim(self, x, y):
        self.aim.x = x
        self.aim.y = y


class WorldModel(object):
    situations = {'HALT' : False, 'STOP' : False, 'FORCE_START' : False,
            'OUR_KICKOFF_PRE' : False, 'OUR_KICKOFF_START' : False,
            'OUR_PENALTY_PRE' : False, 'OUR_PENALTY_START' : False,
            'OUR_DIRECT' : False, 'OUR_INDIRECT' : False,
            'OUR_TIMEOUT' : False,
            'THEIR_KICKOFF' : False, 'THEIR_KICKOFF_START' : False,
            'THEIR_PENALTY' : False, 'THEIR_PENALTY_START' : False,
            'THEIR_DIRECT' : False, 'THEIR_INDIRECT' : False,
            'THEIR_TIMEOUT' : False}

    recent_situations = {'BALL_MOVED' : False, 'OUR_ROBOT_CHANGED' : False}
    current_situation = 'HALT'

    assignments = OrderedDict()
    assignments['Role_0'] = None
    assignments['Role_1'] = None
    assignments['Role_2'] = None
    assignments['Role_3'] = None
    assignments['Role_4'] = None
    assignments['Role_5'] = None

    commands = {'Role_0' : Command(), 'Role_1' : Command(),
                'Role_2' : Command(), 'Role_3' : Command(),
                'Role_4' : Command(), 'Role_5' : Command()}

    ball_odom = Odometry()

    friend_goalie_id = 0
    friend_color = 'blue'
    friend_odoms = [Odometry()] * 12
    existing_friends_id = [None] * 6

    enemy_goalie_id = 0
    enemy_odoms = [Odometry()] * 12
    existing_enemies_id = [None] * 6

    enemy_assignments = OrderedDict()
    enemy_assignments['Enemy_Goalie'] = None
    enemy_assignments['Enemy_1'] = None
    enemy_assignments['Enemy_2'] = None
    enemy_assignments['Enemy_3'] = None
    enemy_assignments['Enemy_4'] = None
    enemy_assignments['Enemy_5'] = None

    tf_listener = tf.TransformListener()

    _refbox_command = None
    _refbox_command_changed = False

    _refbox_dict_blue = {SSL_Referee.HALT : 'HALT', SSL_Referee.STOP : 'STOP',
            SSL_Referee.NORMAL_START : 'NORMAL_START', 
            SSL_Referee.FORCE_START : 'FORCE_START',
            SSL_Referee.PREPARE_KICKOFF_BLUE : 'OUR_KICKOFF_PRE',
            SSL_Referee.PREPARE_PENALTY_BLUE : 'OUR_PENALTY_PRE',
            SSL_Referee.DIRECT_FREE_BLUE : 'OUR_DIRECT',
            SSL_Referee.INDIRECT_FREE_BLUE : 'OUR_INDIRECT',
            SSL_Referee.TIMEOUT_BLUE : 'OUR_TIMEOUT',
            SSL_Referee.PREPARE_KICKOFF_YELLOW : 'THEIR_KICKOFF_PRE',
            SSL_Referee.PREPARE_PENALTY_YELLOW : 'THEIR_PENALTY_PRE',
            SSL_Referee.DIRECT_FREE_YELLOW : 'THEIR_DIRECT',
            SSL_Referee.INDIRECT_FREE_YELLOW : 'THEIR_INDIRECT',
            SSL_Referee.TIMEOUT_YELLOW : 'THEIR_TIMEOUT'}

    _refbox_dict_yellow = {SSL_Referee.HALT : 'HALT', SSL_Referee.STOP : 'STOP',
            SSL_Referee.NORMAL_START : 'NORMAL_START', 
            SSL_Referee.FORCE_START : 'FORCE_START',
            SSL_Referee.PREPARE_KICKOFF_BLUE : 'THEIR_KICKOFF_PRE',
            SSL_Referee.PREPARE_PENALTY_BLUE : 'THEIR_PENALTY_PRE',
            SSL_Referee.DIRECT_FREE_BLUE : 'THEIR_DIRECT',
            SSL_Referee.INDIRECT_FREE_BLUE : 'THEIR_INDIRECT',
            SSL_Referee.TIMEOUT_BLUE : 'THEIR_TIMEOUT',
            SSL_Referee.PREPARE_KICKOFF_YELLOW : 'OUR_KICKOFF_PRE',
            SSL_Referee.PREPARE_PENALTY_YELLOW : 'OUR_PENALTY_PRE',
            SSL_Referee.DIRECT_FREE_YELLOW : 'OUR_DIRECT',
            SSL_Referee.INDIRECT_FREE_YELLOW : 'OUR_INDIRECT',
            SSL_Referee.TIMEOUT_YELLOW : 'OUR_TIMEOUT'}
    _refbox_dict = _refbox_dict_blue


    @classmethod
    def update_world(cls):
        WorldModel._update_situation()

        WorldModel._update_enemy_assignments()


    @classmethod
    def reset_recent_situations(cls):
        pass

    
    @classmethod
    def update_assignments(cls):
        unassigned_roles, unassigned_IDs = WorldModel._check_unassignment()

        # Goalie_IDはRole_0に固定する
        if WorldModel.friend_goalie_id in unassigned_IDs:
            WorldModel.assignments['Role_0'] = WorldModel.friend_goalie_id
            unassigned_IDs.remove(WorldModel.friend_goalie_id)
            unassigned_roles.remove('Role_0')

        for role in unassigned_roles:
            if unassigned_IDs and role != 'Role_0':
                WorldModel.assignments[role] = unassigned_IDs.pop(0)
            else:
                WorldModel.assignments[role] = None

    
    @classmethod
    def set_friend_color(cls, data):
        WorldModel.friend_color = data
        if data == 'blue':
            WorldModel._refbox_dict = WorldModel._refbox_dict_blue
        elif data == 'yellow':
            WorldModel._refbox_dict = WorldModel._refbox_dict_yellow
        

    @classmethod
    def set_existing_friends_id(cls, data):
        WorldModel.existing_friends_id  = list(data)
        
    
    @classmethod
    def set_existing_enemies_id(cls, data):
        WorldModel.existing_enemies_id = list(data)
        

    @classmethod
    def set_refbox_command(cls, data):
        if WorldModel._refbox_command != data:
            WorldModel._refbox_command_changed = True
            WorldModel._refbox_command = data


    @classmethod
    def get_pose(cls, name):
        pose = None

        if name == 'Ball':
            ball_pose = WorldModel.ball_odom.pose.pose.position
            pose = Pose(ball_pose.x, ball_pose.y, 0)

        elif name[:4] == 'Role':
            pose = WorldModel.get_friend_pose(name)

        elif name[:5] == 'Enemy':
            pose = WorldModel.get_enemy_pose(name)

        elif name[:5] == 'CONST':
            pose = constants.poses[name]

        return pose


    @classmethod
    def get_velocity(cls, name):
        velocity = None

        if name == 'Ball':
            linear = WorldModel.ball_odom.twist.twist.linear
            velocity = Velocity(linear.x, linear.y, 0)

        elif name[:4] == 'Role':
            velocity = WorldModel.get_friend_velocity(name)

        elif name[:5] == 'Enemy':
            velocity = WorldModel.get_enemy_velocity(name)

        return velocity


    @classmethod
    def get_friend_pose(cls, role):
        robot_id = WorldModel.assignments[role]

        if robot_id is None:
            return None

        position = WorldModel.friend_odoms[robot_id].pose.pose.position
        orientation = WorldModel.friend_odoms[robot_id].pose.pose.orientation
        yaw = tool.yawFromQuaternion(orientation)

        return Pose(position.x, position.y, yaw)
        

    @classmethod
    def get_enemy_pose(cls, role):
        robot_id = WorldModel.enemy_assignments[role]

        if robot_id is None:
            return None

        position = WorldModel.enemy_odoms[robot_id].pose.pose.position
        orientation = WorldModel.enemy_odoms[robot_id].pose.pose.orientation
        yaw = tool.yawFromQuaternion(orientation)

        return Pose(position.x, position.y, yaw)


    @classmethod
    def get_friend_velocity(cls, role):
        robot_id = WorldModel.assignments[role]

        if robot_id is None:
            return None

        linear = WorldModel.friend_odoms[robot_id].twist.twist.linear
        angular = WorldModel.friend_odoms[robot_id].twist.twist.angular

        return Velocity(linear.x, linear.y, angular.z)
        

    @classmethod
    def get_enemy_velocity(cls, role):
        robot_id = WorldModel.enemy_assignments[role]

        if robot_id is None:
            return None

        linear = WorldModel.enemy_odoms[robot_id].twist.twist.linear
        angular = WorldModel.enemy_odoms[robot_id].twist.twist.angular

        return Velocity(linear.x, linear.y, angular.z)


    @classmethod
    def _check_unassignment(cls):
        unassigned_roles = []
        unassigned_IDs = WorldModel.existing_friends_id

        for role, robot_id in WorldModel.assignments.items():
            # IDがアサインされていないRoleを取り出す
            if robot_id is None:
                unassigned_roles.append(role)

            # アサインされてるIDが存在しないRoleを取り出す
            elif not robot_id in unassigned_IDs:
                unassigned_roles.append(role)

            # アサインされてるIDが存在したら、unassigned_IDsから取り出す
            elif robot_id in unassigned_IDs:
                unassigned_IDs.remove(robot_id)

        return unassigned_roles, unassigned_IDs
        

    @classmethod
    def _update_enemy_assignments(cls):
        raw_id_list = list(WorldModel.existing_enemies_id)
        
        # enemy_assignmnetsを初期化
        for key in WorldModel.enemy_assignments.keys():
            WorldModel.enemy_assignments[key] = None
        
        # raw listからgoalieのIDを取り除く
        if WorldModel.enemy_goalie_id in raw_id_list:
            raw_id_list.remove(WorldModel.enemy_goalie_id)
            WorldModel.enemy_assignments['Enemy_Goalie'] = WorldModel.enemy_goalie_id

        key_i = 1
        for enemy_id in raw_id_list:
            key = 'Enemy_' + str(key_i)
            WorldModel.enemy_assignments[key] = enemy_id
            key_i += 1


    @classmethod
    def _update_situation(cls):
        if WorldModel._refbox_command_changed:
            WorldModel._refbox_command_changed = False
            WorldModel.situations[WorldModel.current_situation] = False

            refbox_command = WorldModel._refbox_dict[WorldModel._refbox_command]

            # NORMAL_STARTはKICKOFFとPENALTYのトリガーになるため、その切り分けを行う
            if refbox_command == 'NORMAL_START':
                if WorldModel.current_situation == 'OUR_KICKOFF_PRE':
                    WorldModel.current_situation = 'OUR_KICKOFF_START'

                elif WorldModel.current_situation == 'OUR_PENALTY_PRE':
                    WorldModel.current_situation = 'OUR_PENALTY_START'

                elif WorldModel.current_situation == 'THEIR_KICKOFF_PRE':
                    WorldModel.current_situation = 'THEIR_KICKOFF_START'

                elif WorldModel.current_situation == 'THEIR_PENALTY_PRE':
                    WorldModel.current_situation = 'THEIR_PENALTY_START'

                else:
                    WorldModel.current_situation = 'FORCE_START'
            else:
                WorldModel.current_situation = refbox_command

            WorldModel.situations[refbox_command] = True



