#!/usr/bin/env python

import rospy
import tf
import math
import constants

rospy.init_node('decision_maker')

from world_model import WorldModel
from play_executer import PlayExecuter

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point

from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Header
from std_msgs.msg import UInt16MultiArray as UIntArray
from std_msgs.msg import String

from nav_msgs.msg import Odometry
from consai_msgs.msg import AIStatus
from consai_msgs.msg import RefereeTeamInfo
from consai_msgs.msg import GeometryFieldSize, FieldLineSegment, FieldCircularArc


def publish():
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'

    for role, robot_id in WorldModel.assignments.items():
        if robot_id is not None:
            command = WorldModel.commands[role]

            if command.velocity_control_enable:
                msg = TwistStamped()
                msg.header = header
                msg.twist = command.target_velocity
                pubs_velocity[robot_id].publish(msg)
            else:
                send_pose = WorldModel.tf_listener.transformPose("map",command.target_pose)
                send_pose.header.stamp = rospy.Time.now()
                pubs_position[robot_id].publish(send_pose)

            pubs_kick_velocity[robot_id].publish(Float32(command.kick_power))

            status = AIStatus()
            status.avoidBall = command.avoid_ball
            status.avoidDefenceArea = command.avoid_defence_area
            status.do_chip = command.chip_enable
            status.dribble_power = command.dribble_power
            pubs_ai_status[robot_id].publish(status)

            command.reset_adjustments()


def friendIDCallback(msg):
    WorldModel.set_existing_friends_id(msg.data)


def enemyIDCallback(msg):
    WorldModel.set_existing_enemies_id(msg.data)


def refboxCallback(msg):
    WorldModel.set_refbox_command(msg.data)


def ballCallback(msg):
    WorldModel.set_ball_odom(msg)


def callback_friend_odom(msg, robot_id):
    WorldModel.set_friend_odom(msg, robot_id)


def callback_enemy_odom(msg, robot_id):
    WorldModel.set_enemy_odom(msg, robot_id)


def callback_blue_info(msg):
    WorldModel.set_blue_info(msg)


def callback_yellow_info(msg):
    WorldModel.set_yellow_info(msg)


def callback_test_name(msg):
    WorldModel.set_test_name(msg.data)


def callback_geometry(msg):
    constants.set_field(msg.field_length, msg.field_width)

    for line in msg.field_lines:
        if line.name == 'RightPenaltyStretch':
            p_x = math.fabs(line.p1_x)
            p_y = math.fabs(line.p1_y)
            constants.set_penalty(p_x, p_y)


def main():
    r   = rospy.Rate(30)

    while not rospy.is_shutdown():
        play_executer.update()

        r.sleep()

        publish()


if __name__ == '__main__':

    ROBOTS_NUM = rospy.get_param('robots_num', 6)
    ID_MAX = rospy.get_param('id_max', 12)
    WorldModel.initialize_world(ID_MAX, ROBOTS_NUM)

    play_executer = PlayExecuter()

    # Publishers for the robots controll
    pubs_position = []
    pubs_velocity = []
    pubs_kick_velocity = []
    pubs_ai_status = []

    # Subscribers
    sub_refbox_command  = rospy.Subscriber("refbox/command", Int8, refboxCallback)
    sub_refbox_blue_info = rospy.Subscriber("refbox/blue_info", RefereeTeamInfo, callback_blue_info)
    sub_refbox_yellow_info = rospy.Subscriber("refbox/yellow_info", RefereeTeamInfo, callback_yellow_info)
    sub_ball            = rospy.Subscriber("ball_observer/estimation", Odometry, ballCallback)
    sub_friend_id              = rospy.Subscriber("existing_friends_id",UIntArray,friendIDCallback)
    sub_enemy_id        = rospy.Subscriber("existing_enemies_id",UIntArray,enemyIDCallback)
    subs_friend_odom = []
    subs_enemy_odom = []
    sub_test_name = rospy.Subscriber('test_name', String, callback_test_name)
    sub_geometry = rospy.Subscriber('geometry_field_size', GeometryFieldSize, callback_geometry)

    for robot_id in xrange(ID_MAX):
        id_str = str(robot_id)
        topic_friend_odom = "robot_" + id_str + "/odom"
        topic_enemy_odom = "enemy_" + id_str + "/odom"
        topic_position = "robot_"+id_str+"/move_base_simple/goal"
        topic_velocity = "robot_"+id_str+"/move_base_simple/target_velocity"
        topic_kick_velocity = "robot_"+id_str+"/kick_velocity"
        topic_ai_status = "robot_"+id_str+"/ai_status"

        pubs_position.append(
                rospy.Publisher(
                    topic_position,
                    PoseStamped,
                    queue_size=10))
        pubs_velocity.append(
                rospy.Publisher(
                    topic_velocity,
                    TwistStamped,
                    queue_size=10))
        pubs_kick_velocity.append(
                rospy.Publisher(
                    topic_kick_velocity,
                    Float32,
                    queue_size=10))
        pubs_ai_status.append(
                rospy.Publisher(
                    topic_ai_status,
                    AIStatus,
                    queue_size=10))

        subs_friend_odom.append(
                rospy.Subscriber(topic_friend_odom, Odometry,
                    callback_friend_odom, callback_args=robot_id))

        subs_enemy_odom.append(
                rospy.Subscriber(topic_enemy_odom, Odometry,
                    callback_enemy_odom, callback_args=robot_id))

    WorldModel.set_friend_color(rospy.get_param('friend_color'))
    WorldModel.set_friend_goalie_id(rospy.get_param('goalie_id'))


    main()
