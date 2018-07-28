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

    
    def _callback_joy(self, msg):
        if msg.buttons[self._A]:
            rospy.logerr("button A")
        if msg.buttons[self._B]:
            rospy.logerr("button B")
        if msg.buttons[self._X]:
            rospy.logerr("button X")
        if msg.buttons[self._Y]:
            rospy.logerr("button Y")
        if msg.buttons[self._L]:
            rospy.logerr("button L")
        if msg.buttons[self._R]:
            rospy.logerr("button R")
        if msg.buttons[self._SEL]:
            rospy.logerr("button SEL")
        if msg.buttons[self._START]:
            rospy.logerr("button START")
        if msg.axes[self._AXES_X]:
            rospy.logerr("axes x :" + str(msg.axes[self._AXES_X]))
        if msg.axes[self._AXES_Y]:
            rospy.logerr("axes y :" + str(msg.axes[self._AXES_Y]))



def receiveJoy(joy):
    buttons.refresh(joy.buttons)
    # referee.publish(joy)
    commands.publish(joy)

def main():
    rospy.init_node('joystick_node')

    core = Core()

    rospy.spin()


if __name__ == '__main__':
    main()

