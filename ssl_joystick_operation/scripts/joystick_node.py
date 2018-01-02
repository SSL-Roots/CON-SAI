#!/usr/bin/env  python
import rospy
import socket
import math

from std_msgs.msg import String
from std_msgs.msg import Time
from std_msgs.msg import UInt8
from std_msgs.msg import UInt32
from sensor_msgs.msg import Joy
from consai_msgs.msg import robot_commands
import topic_tools.srv
# import ssl_refbox.msg


class Buttons():

    """docstring for JoyInput"""

    def __init__(self, ):
        self.old_input = int()
        self.now_input = int()

    def refresh(self, buttons):
        self.old_input = self.now_input
        self.now_input = buttons

    def isPushed(self, num):
        if self.now_input[num] == 1:
            return True
        else:
            return False

    def isEdgeOn(self, num):
        if self.now_input[num] == 1 and self.old_input[num] == 0:
            return True
        else:
            return False

    def isEdgeOff(self, num):
        if self.now_input[num] == 0 and self.old_input[num] == 1:
            return True
        else:
            return False


# class Referee():
#
#     """docstring for Referee"""
#
#     def __init__(self):
#         self.refbox_msg = ssl_refbox.msg.SSLReferee()
#         self.pub = rospy.Publisher(
#             '~refbox', ssl_refbox.msg.SSLReferee, queue_size=10)
#
#         self.halt_button = rospy.get_param('~halt_button')
#         self.start_button = rospy.get_param('~start_button')
#         self.stopgame_button = rospy.get_param('~stopgame_button')
#         self.force_start_button = rospy.get_param('~force_start_button')
#
#     def publish(self, command):
#         msg = ssl_refbox.msg.SSLReferee()
#         msg.stage = 'NORMAL_FIRST_HALF_PRE'
#         msg.command = command
#
#         self.pub.publish(msg)


class RobotCommand():

    """docstring for RobotCommand"""

    def __init__(self):
        self.pub = rospy.Publisher(
            '~robot_commands', robot_commands, queue_size=10)

        self.surge_axis = rospy.get_param('~surge_axis')
        self.sway_axis = rospy.get_param('~sway_axis')
        self.turn_l_axis    = rospy.get_param('~turn_l_axis')
        self.turn_r_axis    = rospy.get_param('~turn_r_axis')
        self.kick_x_button = rospy.get_param('~kick_x_button')
        self.kick_z_button = rospy.get_param('~kick_z_button')
        self.dribble_button = rospy.get_param('~dribble_button')
        self.turbo_button = rospy.get_param('~turbo_button')
        self.select_button = rospy.get_param('~select_button')

        self.holonomic = True

    def publish(self, joy):
        commands = robot_commands()

        if buttons.isEdgeOn(self.select_button) == True:
            self.holonomic  = not self.holonomic
            if self.holonomic == True:
                rospy.loginfo('Change controller mode : holonomic')
            else:
                rospy.loginfo('Change controller mode : NON-holonomic')

        if joy.buttons[self.turbo_button] == 0:
            vel_scale = 1
        else:
            vel_scale = 3

        if self.holonomic == True:
            commands.vel_surge = joy.axes[self.surge_axis] * vel_scale
            commands.vel_sway = 0.0
            commands.omega = joy.axes[self.sway_axis] * 1.5 * vel_scale

        else:
            commands.vel_surge = joy.axes[self.surge_axis] * vel_scale
            commands.vel_sway =  joy.axes[self.sway_axis] * vel_scale
            omega   = -joy.axes[self.turn_l_axis] + joy.axes[self.turn_r_axis]
            commands.omega   = omega * 1.5 * vel_scale
        if joy.buttons[self.kick_x_button] == 1:
            commands.kick_speed_x = 3
            commands.kick_speed_z = 0
        elif joy.buttons[self.kick_z_button] == 1:
            commands.kick_speed_x = 3
            commands.kick_speed_z = 3
        else:
            commands.kick_speed_x = 0
            commands.kick_speed_z = 0

        if joy.buttons[self.dribble_button] == 1:
            commands.dribble_power = 1
        else:
            commands.dribble_power = 0

        self.pub.publish(commands)




def receiveJoy(joy):
    buttons.refresh(joy.buttons)
    # referee.publish(joy)
    commands.publish(joy)


if __name__ == '__main__':
    rospy.init_node('joystick_node')

    # class inistantinate
    buttons = Buttons()
    # referee = Referee()
    commands = RobotCommand()

    # get parameters
    friend_color = rospy.get_param('friend_color')

    # Define Subscriber
    rospy.Subscriber("joy", Joy, receiveJoy)
    rospy.spin()
