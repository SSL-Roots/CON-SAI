#!/usr/bin/env python
import rospy
import tf
import math
import nav_msgs.msg
import geometry_msgs.msg

robot_pose = geometry_msgs.msg.Pose()
last_vel_world = geometry_msgs.msg.Twist()
last_time = rospy.Time()


def odomCallBack(msg):
    global robot_pose

    robot_pose = msg.pose.pose


def twistCallBack(vel_robot):
    global last_vel_world, last_time

    current_time = rospy.Time.now()

    quaternion = (
        robot_pose.orientation.x,
        robot_pose.orientation.y,
        robot_pose.orientation.z,
        robot_pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]

    vel_world = geometry_msgs.msg.Twist()
    vel_world.linear.x = vel_robot.linear.x * math.cos(yaw) - vel_robot.linear.y * math.sin(yaw)
    vel_world.linear.y = vel_robot.linear.x * math.sin(yaw) + vel_robot.linear.y * math.cos(yaw)
    vel_world.angular.z = vel_robot.angular.z

    acc_world = geometry_msgs.msg.Accel()
    dt = (current_time - last_time).to_sec()
    acc_world.linear.x = (vel_world.linear.x - last_vel_world.linear.x) / dt
    acc_world.linear.y = (vel_world.linear.y - last_vel_world.linear.y) / dt
    acc_world.angular.z = (vel_world.angular.z - last_vel_world.angular.z) / dt

    last_vel_world = vel_world
    last_time = current_time

    pub_vel_world.publish(vel_world)
    pub_acc_world.publish(acc_world)




if __name__ == '__main__':
    rospy.init_node('velocity_transformer')

    pub_vel_world = rospy.Publisher("cmd_vel_world", geometry_msgs.msg.Twist, queue_size=10)
    pub_acc_world = rospy.Publisher("accel_world", geometry_msgs.msg.Accel, queue_size=10)

    sub_odom = rospy.Subscriber("odom", nav_msgs.msg.Odometry, odomCallBack)
    sub_cmdvel = rospy.Subscriber("cmd_vel", geometry_msgs.msg.Twist, twistCallBack)


    rospy.spin()
