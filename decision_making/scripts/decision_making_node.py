#!/usr/bin/env python

import rospy
import  tf
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *
import  referee_pb2

rospy.init_node('decision_maker')   # write this line before import GlobalData

from GlobalData import GlobalInfo

from TestMove.TestParent import TestParent
from AIParent import GameMain

import  geometry_msgs.msg
import  std_msgs.msg
from    tf.transformations import quaternion_from_euler
from    std_msgs.msg import Int8
from    std_msgs.msg import Float32
from    std_msgs.msg import String
from    std_msgs.msg import UInt16MultiArray as UIntArray
from    nav_msgs.msg import Odometry
from consai_msgs.msg import nodeData, nodeDataArray, AIStatus
from geometry_msgs.msg import Point

import IDLinker


def publish():
    header  = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'

    for control in GlobalInfo.controls:
        if control.has_id() == True:
            robot_id = control.get_id()

            if control.is_velocity_control:
                msg = geometry_msgs.msg.TwistStamped()
                msg.header = header
                msg.twist = control.target_velocity
                pubs_velocity[robot_id].publish(msg)
            else:
                send_pose = GlobalInfo.tf_listener.transformPose("map",control.target_pose)
                send_pose.header.stamp = rospy.Time.now()
                pubs_position[robot_id].publish(send_pose)

            pubs_kick_velocity[robot_id].publish(std_msgs.msg.Float32(control.kick_velocity))

            status = AIStatus()
            status.avoidBall = control.avoid_ball()
            status.do_chip = control.do_chip()
            status.dribble_power = control.get_dribble_power()
            pubs_ai_status[robot_id].publish(status)
    
    pub_nodes.publish(G_nodeArray)


def friendIDCallback(msg):
    IDLinker.updateGlobalFriendIDs(msg.data)

def enemyIDCallback(msg):
    IDLinker.updateGlobalEnemyIDs(msg.data)

def refboxCallback(msg):
    GlobalInfo.setRefCommand(msg.data)


def ballCallback(msg):
    GlobalInfo.setBallInfo(msg)


def logChildren(parent):
    for child in parent.children:
        node = nodeData()
        node.parentName = parent.name
        node.myName = child.name
        node.myType = child.node_type
        if child.status is None:
            node.myStatus = 3
        else:
            node.myStatus = child.status

        global G_nodeArray
        G_nodeArray.nodes.append(node)

        logChildren(child)


def main():
    behavior_tree = Selector("Behavior")

    # test_parent = TestParent("Test_Parent")
    # behavior_tree.add_child(test_parent)
    #
    GAME_MAIN = GameMain("GameMain")
    behavior_tree.add_child(GAME_MAIN)

    r   = rospy.Rate(10)

    print_tree(behavior_tree, 0, True)

    while not rospy.is_shutdown():
        behavior_tree.run()
        r.sleep()

        logChildren(behavior_tree)

        publish()
        global G_nodeArray
        G_nodeArray.nodes = []


if __name__ == '__main__':

    pubs_position = []
    pubs_velocity = []
    pubs_kick_velocity = []
    pubs_ai_status = []

    # Publishers for the robots controll
    for robot_id in xrange(12):
        id_str = str(robot_id)
        topic_position = "/robot_"+id_str+"/move_base_simple/goal"
        topic_velocity = "/robot_"+id_str+"/move_base_simple/target_velocity"
        topic_kick_velocity = "/robot_"+id_str+"/kick_velocity"
        topic_ai_status = "/robot_"+id_str+"/ai_status"

        pubs_position.append(
                rospy.Publisher(
                    topic_position,
                    geometry_msgs.msg.PoseStamped,
                    queue_size=10))
        pubs_velocity.append(
                rospy.Publisher(
                    topic_velocity,
                    geometry_msgs.msg.TwistStamped,
                    queue_size=10))
        pubs_kick_velocity.append(
                rospy.Publisher(
                    topic_kick_velocity,
                    std_msgs.msg.Float32,
                    queue_size=10))
        pubs_ai_status.append(
                rospy.Publisher(
                    topic_ai_status,
                    AIStatus,
                    queue_size=10))

    pub_nodes = rospy.Publisher('node_data_array',nodeDataArray,queue_size=10)
    # Subscriber
    sub_refbox_command  = rospy.Subscriber("/refbox/command", Int8, refboxCallback)
    sub_ball            = rospy.Subscriber("/ball_observer/estimation", Odometry, ballCallback)
    sub_id              = rospy.Subscriber("/existing_friends_id",UIntArray,friendIDCallback)
    sub_enemy_id        = rospy.Subscriber("/existing_enemies_id",UIntArray,enemyIDCallback)

    friend_color = rospy.get_param("/friend_color")

    GlobalInfo.setFriendColor(friend_color)

    G_nodeArray = nodeDataArray()

    main()
