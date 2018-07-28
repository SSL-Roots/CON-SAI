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


    
    def _callback_joy(self, msg):
        if msg.buttons[self._A]:
            rospy.logerr("button A")
        else:
            rospy.logerr("other")



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

