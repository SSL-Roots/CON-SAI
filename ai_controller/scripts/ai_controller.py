#!/usr/bin/env  python
import rospy
import socket
import math

import  std_msgs.msg
import  geometry_msgs.msg
from consai_msgs.msg import AIStatus, robot_commands


G_cmd_vel   = geometry_msgs.msg.Twist()
G_kick_vel  = std_msgs.msg.Float32()
G_AIStatus = AIStatus()
G_is_cmdvel_received   = False
G_is_kickvel_received   = False
G_is_AIStatus_received = False

def recvCmdvel(twist):
    global  G_cmd_vel
    global  G_is_cmdvel_received

    G_cmd_vel   = twist
    G_is_cmdvel_received    = True

def recvKickVel(msg):
    global  G_kick_vel
    global  G_is_kickvel_received

    G_kick_vel  = msg
    G_is_kickvel_received   = True

def recvAIStatus(msg):
    global G_AIStatus
    global G_is_AIStatus_received

    G_AIStatus = msg
    G_is_AIStatus_received = True



if __name__ == '__main__':
    rospy.init_node('ai_controller')

    # set publisher
    pub = rospy.Publisher(
        'robot_commands', robot_commands, queue_size=10)

    # Define Subscriber
    rospy.Subscriber("cmd_vel", geometry_msgs.msg.Twist, recvCmdvel)
    rospy.Subscriber("kick_velocity", std_msgs.msg.Float32, recvKickVel)
    rospy.Subscriber("ai_status", AIStatus, recvAIStatus)

    r   = rospy.Rate(60)

    while not   rospy.is_shutdown():

        if G_is_cmdvel_received \
                and G_is_kickvel_received \
                and G_is_AIStatus_received:
            commands = robot_commands()

            commands.vel_surge  = G_cmd_vel.linear.x
            commands.vel_sway   = G_cmd_vel.linear.y
            commands.omega      = G_cmd_vel.angular.z

            commands.kick_speed_x = G_kick_vel.data

            if G_AIStatus.do_chip:
                commands.kick_speed_z = G_kick_vel.data

            commands.dribble_power = G_AIStatus.dribble_power

            pub.publish(commands)

            G_is_cmdvel_received = False
            G_is_kickvel_received = False
            G_is_AIStatus_received = False

        r.sleep()
