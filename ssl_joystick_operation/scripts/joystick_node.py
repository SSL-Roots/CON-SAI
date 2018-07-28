#!/usr/bin/env  python
#encoding: utf8

import rospy
from sensor_msgs.msg import Joy
from consai_msgs.msg import TestAICommand
from std_msgs.msg import String


class Core(object):
    def __init__(self):
        self._sub_joy = rospy.Subscriber('joy', Joy, self._callback_joy)


        self._pub_command = rospy.Publisher('test_ai_command', TestAICommand, queue_size=1)
        self._pub_name = rospy.Publisher('test_name', String, queue_size=1)

        self._A = rospy.get_param('~button_A')
        self._B = rospy.get_param('~button_B')
        self._X = rospy.get_param('~button_X')
        self._Y = rospy.get_param('~button_Y')
        self._L = rospy.get_param('~button_L')
        self._R = rospy.get_param('~button_R')
        self._SEL = rospy.get_param('~button_SEL')
        self._START = rospy.get_param('~button_START')
        self._AXES_X = rospy.get_param('~axes_X')
        self._AXES_Y = rospy.get_param('~axes_Y')

        self._prev_test_name = "TEST_NONE"
        self._prev_test_command = TestAICommand()
        self._buttons = []

        self._MAX_VELOCITY = 0.5
        self._DRIBBLE_POWER = 6
        self._KICK_POWER = 6

    
    def _callback_joy(self, msg):
        test_name = self._prev_test_name
        test_command = self._prev_test_command

        if msg.buttons[self._SEL]:
            test_name = "TEST_NONE"

        if msg.buttons[self._START]:
            test_name = "TEST2"

        if msg.buttons[self._A]:
            rospy.logdebug("button A")
        if msg.buttons[self._X]:
            rospy.logdebug("button X")
        if msg.buttons[self._Y]:
            rospy.logdebug("button Y")

        if test_name == "TEST2":
            # 全手動操作モード
            test_command.vel_x = self._MAX_VELOCITY * msg.axes[self._AXES_Y]
            test_command.vel_y = self._MAX_VELOCITY * msg.axes[self._AXES_X]

            if msg.buttons[self._L]:
                test_command.vel_yaw = self._MAX_VELOCITY
            elif msg.buttons[self._R]:
                test_command.vel_yaw = -1.0 * self._MAX_VELOCITY
            else:
                test_command.vel_yaw = 0

            if msg.buttons[self._B]:
                test_command.dribble_power = self._DRIBBLE_POWER
            else:
                test_command.dribble_power = 0

            if msg.buttons[self._Y]:
                test_command.kick_power = self._KICK_POWER
            else:
                test_command.kick_power = 0


        self._prev_test_name = test_name
        self._prev_test_command = test_command

        self._pub_name.publish(self._prev_test_name)
        self._pub_command.publish(self._prev_test_command)


def main():
    rospy.init_node('joystick_node')

    core = Core()

    rospy.spin()


if __name__ == '__main__':
    main()

