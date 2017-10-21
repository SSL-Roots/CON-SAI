#!/usr/bin/env python

import rospy
import  tf

rospy.init_node('decision_maker')

from world_model import WorldModel
from play_manager import PlayManager

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point

from std_msgs.msg import Int8
from std_msgs.msg import Float32
from std_msgs.msg import Header
from std_msgs.msg import UInt16MultiArray as UIntArray

from nav_msgs.msg import Odometry
from consai_msgs.msg import AIStatus


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
            # TODO(Asit) use navigation_enable instead avoidBall.
            status.avoidBall = True
            status.do_chip = command.chip_enable
            status.dribble_power = command.dribble_power
            pubs_ai_status[robot_id].publish(status)


def friendIDCallback(msg):
    WorldModel.set_existing_friends_id(msg.data)


def enemyIDCallback(msg):
    WorldModel.set_existing_enemies_id(msg.data)


def refboxCallback(msg):
    WorldModel.set_refbox_command(msg.data)


def ballCallback(msg):
    WorldModel.ball_odom = msg


def callback_friend_odom(msg, robot_id):
    WorldModel.friend_odoms[robot_id] = msg


def callback_enemy_odom(msg, robot_id):
    WorldModel.enemy_odoms[robot_id] = msg


def main():
    r   = rospy.Rate(10)

    while not rospy.is_shutdown():
        play_manager.update()

        r.sleep()

        publish()


if __name__ == '__main__':

    play_manager = PlayManager()

    # Publishers for the robots controll
    pubs_position = []
    pubs_velocity = []
    pubs_kick_velocity = []
    pubs_ai_status = []

    # Subscribers
    sub_refbox_command  = rospy.Subscriber("/refbox/command", Int8, refboxCallback)
    sub_ball            = rospy.Subscriber("/ball_observer/estimation", Odometry, ballCallback)
    sub_friend_id              = rospy.Subscriber("/existing_friends_id",UIntArray,friendIDCallback)
    sub_enemy_id        = rospy.Subscriber("/existing_enemies_id",UIntArray,enemyIDCallback)
    subs_friend_odom = []
    subs_enemy_odom = []

    for robot_id in xrange(12):
        id_str = str(robot_id)
        topic_friend_odom = "/robot_" + id_str + "/odom"
        topic_enemy_odom = "/enemy_" + id_str + "/odom"
        topic_position = "/robot_"+id_str+"/move_base_simple/goal"
        topic_velocity = "/robot_"+id_str+"/move_base_simple/target_velocity"
        topic_kick_velocity = "/robot_"+id_str+"/kick_velocity"
        topic_ai_status = "/robot_"+id_str+"/ai_status"

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

    WorldModel.set_friend_color(rospy.get_param('/friend_color'))


    main()
