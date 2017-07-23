#!/usr/bin/env  python

import  rospy
import  std_msgs.msg
from consai_msgs.msg import robot_commands


def recvAI(commands):
    if mode == 'ai':
        pub.publish(commands)

def recvJoy(commands):
    if mode == 'joy':
        pub.publish(commands)


if __name__ == '__main__':
    rospy.init_node('controller_switcher')

    mode = rospy.get_param('~mode', 'ai')

    # set publisher
    pub = rospy.Publisher('robot_commands', robot_commands, queue_size=10)
    mode_pub = rospy.Publisher('~mode', std_msgs.msg.String, queue_size=10)

    # set subscriber
    ai_sub  = rospy.Subscriber('ai_controller/robot_commands', robot_commands, recvAI)
    joy_sub = rospy.Subscriber('ssl_joy/ssl_joystick/robot_commands', robot_commands, recvJoy)

    pub_str = std_msgs.msg.String();
    pub_str.data = mode
    mode_pub.publish(pub_str)

    rospy.spin()
