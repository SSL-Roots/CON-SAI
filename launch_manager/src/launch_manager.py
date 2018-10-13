#!/usr/bin/env python

import rospy
import shlex
import subprocess
import  std_msgs.msg
from consai_msgs.msg import VisionIDList

# Global Constant
EXPIRATION_TIME_SEC = 1.0
MAIN_CYCLE = 10

# Global varianble
last_id_list = VisionIDList()
friend_processes = {}
enemy_processes = {}

ai_name = ""


def callback_id_list(msg):
    global last_id_list
    last_id_list = msg

def manageRobots():
    # TODO : refactor copy and paste

    for robot_id in last_id_list.friend_id_list:
        if (robot_id in friend_processes) == False:
            cmd = "roslaunch ai_core robot.launch number:=" + str(robot_id) \
                    + " ai_name:=" + str(ai_name)
            p = subprocess.Popen(shlex.split(cmd))
            friend_processes[robot_id] = [p, EXPIRATION_TIME_SEC]
            rospy.loginfo("Spawn friend : ID " + str(robot_id))

        else:
            friend_processes[robot_id][1] = EXPIRATION_TIME_SEC

    for k, v in friend_processes.items():
        friend_processes[k][1] -= 1.0 / MAIN_CYCLE
        if friend_processes[k][1] < 0:
            # life expired
            friend_processes[k][0].terminate()
            friend_processes.pop(k)
            rospy.loginfo("Terminate friend : ID " + str(k))


    for robot_id in last_id_list.enemy_id_list:
        if (robot_id in enemy_processes) == False:
            cmd = "roslaunch ai_core enemy.launch number:=" + str(robot_id) \
                    + " ai_name:=" + str(ai_name)
            p = subprocess.Popen(shlex.split(cmd))
            enemy_processes[robot_id] = [p, EXPIRATION_TIME_SEC]
            rospy.loginfo("Spawn enemy : ID " + str(robot_id))

        else:
            enemy_processes[robot_id][1] = EXPIRATION_TIME_SEC

    for k, v in enemy_processes.items():
        enemy_processes[k][1] -= 1.0 / MAIN_CYCLE
        if enemy_processes[k][1] < 0:
            # life expired
            enemy_processes[k][0].terminate()
            enemy_processes.pop(k)
            rospy.loginfo("Terminate enemy : ID " + str(k))


def publishExistingIds():
    msg = std_msgs.msg.UInt16MultiArray()

    msg.data    = friend_processes.keys()
    pub_existing_friends_id.publish(msg)

    msg.data    = enemy_processes.keys()
    pub_existing_enemies_id.publish(msg)


if __name__ == '__main__':
    rospy.init_node("launch_manager")

    sub_id_list = rospy.Subscriber('vision_id_list', VisionIDList, callback_id_list)
    pub_existing_friends_id  = rospy.Publisher('existing_friends_id', std_msgs.msg.UInt16MultiArray, queue_size=10)
    pub_existing_enemies_id  = rospy.Publisher('existing_enemies_id', std_msgs.msg.UInt16MultiArray, queue_size=10)

    EXPIRATION_TIME_SEC    = rospy.get_param('~robot_expiration_time', 1.0) # robots expired time

    ai_name = rospy.get_param('ai_name', '/')

    r = rospy.Rate(MAIN_CYCLE)

    while not rospy.is_shutdown():
        manageRobots()
        publishExistingIds()

        r.sleep()
