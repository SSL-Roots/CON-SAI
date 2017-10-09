#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from collections import OrderedDict

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

from referee_pb2 import SSL_Referee
import tool

class Command(object):
    def __init__(self):
        self.target_pose = PoseStamped()
        self.target_velocity = Twist()
        self.aim = Point()
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

    goalie_id = 0
    friend_color = 'blue'
    ball_odom = Odometry()
    existing_friends_id = [None] * 6
    existing_enemies_id = [None] * 6

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
        # situation の更新
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



    @classmethod
    def reset_recent_situations(cls):
        pass

    
    @classmethod
    def update_assignments(cls):
        unassigned_roles, unassigned_IDs = WorldModel._check_unassignment()

        # Goalie_IDはRole_0に固定する
        if WorldModel.goalie_id in unassigned_IDs:
            WorldModel.assignments['Role_0'] = WorldModel.goalie_id
            unassigned_IDs.remove(WorldModel.goalie_id)
            unassigned_roles.remove('Role_0')

        for role in unassigned_roles:
            if unassigned_IDs and role != 'Role_0':
                WorldModel.assignments[role] = unassigned_IDs.pop()
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
    def get_robot_pose(cls, role):
        robot_id = WorldModel.assignments[role]

        if robot_id is None:
            return None

        target = 'map'
        base = tool.getFriendBase(robot_id)
        WorldModel.tf_listener.waitForTransform(target, base, rospy.Time(0), rospy.Duration(3.0))
        (point, orientation) = WorldModel.tf_listener.lookupTransform(target, base, rospy.Time(0))
        yaw = tool.yawFromTfQuaternion(orientation)

        return point[0], point[1], yaw
        

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
        

