#!/usr/bin/env  python
import rospy
import tf
import math
import multicast

from proto import messages_robocup_ssl_wrapper_pb2

import geometry_msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from consai_msgs.msg import RobotPoses, VisionObservations, VisionRobotPackets, VisionPacket
from consai_msgs.msg import GeometryFieldSize, FieldLineSegment, FieldCircularArc


def reversePoint(self):
    self.x = -self.x
    self.y = -self.y
setattr(geometry_msgs.msg.Point, "reverse", reversePoint)


def reverseOrientation(self):
    euler = euler_from_quaternion((self.x, self.y, self.z, self.w))
    euler = list(euler)
    euler[2] = euler[2] + math.pi
    quat_tuple = quaternion_from_euler(*euler)
    self.x = quat_tuple[0]
    self.y = quat_tuple[1]
    self.z = quat_tuple[2]
    self.w = quat_tuple[3]
setattr(geometry_msgs.msg.Quaternion, "reverse", reverseOrientation)


def reversePose(self):
    self.position.reverse()
    self.orientation.reverse()
setattr(geometry_msgs.msg.Pose, "reverse", reversePose)


def convertOrientation(self, yaw):
    quat = quaternion_from_euler(0.0, 0.0, yaw)
    self.x = quat_tuple[0]
    self.y = quat_tuple[1]
    self.z = quat_tuple[2]
    self.w = quat_tuple[3]
setattr(geometry_msgs.msg.Quaternion, "convert", convertOrientation)


def convertPose(self, x, y, yaw):
    self.position.x = x
    self.position.y = y
    self.orientation.convertOrientation(yaw)
setattr(geometry_msgs.msg.Pose, "convert", convertPose)


class AbstractGeometry(object):
    """ Abstract geometry publisher """

    def __init__(self, team_side, topic_name):
        if team_side == 'LEFT':
            self.side_coeff = 1
        else:
            self.side_coeff = -1

        self.pub = rospy.Publisher(topic_name, PoseArray, queue_size=10)

    def publish(self, pose_array):
        if len(pose_array.poses) == 0:
            return

        i = 0
        for pose in pose_array.poses:
            pose_array.poses[i] = self.setSide(pose)
            i = i + 1

        self.pub.publish(pose_array)

    def setSide(self, pose):
        if self.side_coeff == -1:
            pose.reverse()

        return pose


class PoseMaker(object):
    """docstring for PoseMaker"""

    def __init__(self, time, frame_id):
        super(PoseMaker, self).__init__()
        self.pose_array = PoseArray()
        self.pose_array.header.stamp = time
        self.pose_array.header.frame_id = frame_id

    def get(self):
        return self.pose_array

    def add(self, pose_detected):
        self.pose_array.poses.append(self.msgToPose(pose_detected))

    def len(self):
        return len(self.pose_array.poses)

    def msgToPose(self, detection_pose):
        pose = Pose()

        pose.position.x = detection_pose.x / 1000
        pose.position.y = detection_pose.y / 1000

        if hasattr(detection_pose, 'orientation'):
            quat_tuple = quaternion_from_euler(0.0, 0.0, detection_pose.orientation)
            pose.orientation = Quaternion(*quat_tuple)

        return pose


class FormatConverter:

    def __init__(self, friend_color, do_side_invert):
        self.table = []
        self.friend_color = friend_color
        self.do_side_invert = do_side_invert

    def protobufToTable(self, protobuf_binary):
        ssl_wrapper = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
        ssl_wrapper.ParseFromString(protobuf_binary)

        header = {
            'frame_number': ssl_wrapper.detection.frame_number, 't_capture': ssl_wrapper.detection.t_capture, 't_sent': ssl_wrapper.detection.t_sent, 'camera_id': ssl_wrapper.detection.camera_id
        }

        for ball in ssl_wrapper.detection.balls:
            observation = {
                'color': 'ball', 'confidence': ball.confidence, 'robot_id': None, 'x': ball.x, 'y': ball.y, 'z': ball.z, 'orientation': None, 'area': ball.area
            }
            self.table.append(dict(header, **observation))

        for robot in ssl_wrapper.detection.robots_yellow:
            observation = {
                'color': 'yellow', 'confidence': robot.confidence, 'robot_id': robot.robot_id, 'x': robot.x, 'y': robot.y, 'z': robot.height, 'orientation': robot.orientation, 'area': None
            }
            self.table.append(dict(header, **observation))

        for robot in ssl_wrapper.detection.robots_blue:
            observation = {
                'color': 'blue', 'confidence': robot.confidence, 'robot_id': robot.robot_id, 'x': robot.x, 'y': robot.y, 'z': robot.height, 'orientation': robot.orientation, 'area': None
            }
            self.table.append(dict(header, **observation))


    def getTable(self):
        return self.table

    def refreshTable(self):
        self.table = []

    def tableToRosmsg(self):
        rosmsg = VisionObservations()

        rosmsg.header.stamp = rospy.Time.now()

        for obs in self.table:
            if obs['color'] == 'ball':
                vision_packet   = self.observationToVisionPacket(obs)
                rosmsg.ball.append(vision_packet)

            elif    obs['color'] == self.friend_color:
                id_index  = None
                for i, friend in enumerate(rosmsg.friends):
                    # search detected robot id
                    if friend.robot_id == obs['robot_id']:
                        id_index   = i
                        break

                if id_index == None:
                    # this robot has not detected yet (in this observation)
                    robot_packets    = VisionRobotPackets()
                    robot_packets.robot_id   = obs['robot_id']
                    rosmsg.friends.append(robot_packets)
                    id_index   = len(rosmsg.friends) - 1

                vision_packet   = self.observationToVisionPacket(obs)
                rosmsg.friends[id_index].packets.append(vision_packet)

            else:
                id_index  = None
                for i, enemy in enumerate(rosmsg.enemies):
                    # search detected robot id
                    if enemy.robot_id == obs['robot_id']:
                        id_index   = i
                        break

                if id_index == None:
                    # this robot has not detected yet (in this observation)
                    robot_packets    = VisionRobotPackets()
                    robot_packets.robot_id   = obs['robot_id']
                    rosmsg.enemies.append(robot_packets)
                    id_index   = len(rosmsg.enemies) - 1

                vision_packet   = self.observationToVisionPacket(obs)
                rosmsg.enemies[id_index].packets.append(vision_packet)

        return rosmsg


    def observationToVisionPacket(self, observation):
        vision_packet   = VisionPacket()

        vision_packet.t_capture = observation['t_capture']
        vision_packet.t_sent    = observation['t_sent']
        vision_packet.camera_id = observation['camera_id']
        vision_packet.pose  = self.observationToPose(observation)

        return  vision_packet


    def observationToPose(self, observation):
        pose    = geometry_msgs.msg.Pose()

        if observation['orientation'] == None:
            # if observation is ball
            observation['orientation']  = 0.0

        if self.do_side_invert == True:
            observation['x']    = -observation['x']
            observation['y']    = -observation['y']
            observation['orientation']    = observation['orientation'] + math.pi

        pose.position.x = observation['x'] / 1000
        pose.position.y = observation['y'] / 1000
        pose.position.z = observation['z'] / 1000

        quat = quaternion_from_euler(0.0, 0.0, observation['orientation'])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return  pose


class VisionReceiver:
    def __init__(self):

        # This class receives vision data and publish it
        self._robot_list = rospy.get_param('~robots_id_list', [0, 1, 2, 3, 4, 5])
        self._enemy_list = rospy.get_param('~enemies_id_list', [0, 1, 2, 3, 4, 5])

        self._our_color = rospy.get_param('friend_color', 'yellow').upper()
        self._our_side = rospy.get_param('team_side', 'right').upper()
        self._host = rospy.get_param('~multicast_addr', '224.5.23.2')
        self._port = rospy.get_param('~multicast_port', 10006)

        self._sock = multicast.Multicast(self._host, self._port)


        # make publishers
        self._ball_publisher = AbstractGeometry(self._our_side, 'pose_ball')
        self._robot_publisher = {}
        for n in self._robot_list:
            self._robot_publisher[n] = AbstractGeometry(self._our_side, 'pose_friend_' + str(n))

        self._enemy_publisher = {}
        for n in self._enemy_list:
            self._enemy_publisher[n] = AbstractGeometry(self._our_side, 'pose_enemy_' + str(n))

        self._friend_list_publisher = rospy.Publisher('friend_poses', RobotPoses, queue_size=10)
        self._enemy_list_publisher = rospy.Publisher('enemy_poses', RobotPoses, queue_size=10)

        self._vision_observations_publisher = rospy.Publisher(
            'vision_observations', VisionObservations, queue_size=10)

        self._geometry_field_size_publisher = rospy.Publisher(
            'geometry_field_size', GeometryFieldSize, queue_size=10)

        self._do_side_invert = True
        if self._our_side == 'LEFT':
            self._do_side_invert = False

        self._format_converter = FormatConverter(self._our_color.lower(), self._do_side_invert)


    def receive(self):

        buf = self._sock.recv(2*1024)
        if buf == None:
            return  False

        while not buf == None:
            self._format_converter.protobufToTable(buf)
            self._receive_legacy(buf)
            self._receive_geometry(buf)
            buf = self._sock.recv(2*1024)

        ros_msg = self._format_converter.tableToRosmsg()
        self._format_converter.refreshTable()
        self._vision_observations_publisher.publish(ros_msg)


    def _receive_legacy(self, buf):
        current_time = rospy.Time.now()
        ssl_wrapper = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
        ssl_wrapper.ParseFromString(buf)  # Parse sent data

        # Publish the ball information
        ball_poses = PoseMaker(current_time, 'map')
        for pose in ssl_wrapper.detection.balls:
            ball_poses.add(pose)
        self._ball_publisher.publish(ball_poses.get())

        # Publish robot information
        if self._our_color == 'BLUE':
            detection_friend = ssl_wrapper.detection.robots_blue
            detection_enemy = ssl_wrapper.detection.robots_yellow
        else:
            detection_friend = ssl_wrapper.detection.robots_yellow
            detection_enemy = ssl_wrapper.detection.robots_blue

        friend_pose_arrays = self._convertMsgToPoseArray(current_time, self._robot_list, detection_friend)
        enemy_pose_arrays = self._convertMsgToPoseArray(current_time, self._enemy_list, detection_enemy)

        for i in self._robot_list:
            self._robot_publisher[i].publish(friend_pose_arrays[i])

        for i in self._enemy_list:
            self._enemy_publisher[i].publish(enemy_pose_arrays[i])

        friend_poses_msg = self._getRobotPosesFromProtobufMsg(detection_friend)
        enemy_poses_msg = self._getRobotPosesFromProtobufMsg(detection_enemy)
        self._friend_list_publisher.publish(friend_poses_msg)
        self._enemy_list_publisher.publish(enemy_poses_msg)


    def _receive_geometry(self, buf):
        ssl_wrapper = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
        ssl_wrapper.ParseFromString(buf)


        if ssl_wrapper.HasField('geometry'):
            field_size = GeometryFieldSize()

            field = ssl_wrapper.geometry.field

            # SSL_geometry uses meter unit
            # CON-SAI uses milli meters

            field_size.field_length = field.field_length / 1000.0
            field_size.field_width = field.field_width / 1000.0
            field_size.goal_width = field.goal_width / 1000.0
            field_size.goal_depth = field.goal_depth / 1000.0
            field_size.boundary_width = field.boundary_width / 1000.0

        
            for line in field.field_lines:
                line_segment = FieldLineSegment()

                line_segment.name = line.name
                line_segment.p1_x = line.p1.x / 1000.0
                line_segment.p1_y = line.p1.y / 1000.0
                line_segment.p2_x = line.p2.x / 1000.0
                line_segment.p2_y = line.p2.y / 1000.0
                line_segment.thickness = line.thickness / 1000.0
                field_size.field_lines.append(line_segment)
            
            for arc in field.field_arcs:
                circular_arc = FieldCircularArc()

                circular_arc.name = arc.name
                circular_arc.center_x = arc.center.x / 1000.0
                circular_arc.center_y = arc.center.y / 1000.0
                circular_arc.radius = arc.radius / 1000.0
                circular_arc.a1 = arc.a1
                circular_arc.a2 = arc.a2
                circular_arc.thickness = arc.thickness / 1000.0
                field_size.field_arcs.append(circular_arc)

            self._geometry_field_size_publisher.publish(field_size)


    def _convertMsgToPoseArray(self, time, id_list, detection_msg):
        # return each robot's PoseArray

        # instatinate PoseMaker
        robot_poses = {}
        for i in id_list:
            robot_poses[i] = PoseMaker(time, 'map')

        # change detection_msg to PoseArray
        for robot in detection_msg:
            if robot.robot_id in id_list:
                robot_poses[robot.robot_id].add(robot)

        # align PoseArrays
        pose_arrays = {}
        for i in id_list:
            pose_arrays[i] = robot_poses[i].get()

        return pose_arrays


    def _getRobotPosesFromProtobufMsg(self, detection_msg):
        robot_poses_msg = RobotPoses()
        current_time = rospy.Time.now()

        poses = []
        id_list = []

        for robot in detection_msg:
            if robot.robot_id not in id_list:
                id_list.append(robot.robot_id)
                poses.append(PoseMaker(current_time, 'map'))

            index = id_list.index(robot.robot_id)
            poses[index].add(robot)

        robot_poses_msg.header.stamp = current_time
        robot_poses_msg.header.frame_id = 'map'

        robot_poses_msg.robot_id = id_list
        for p in poses:
            robot_poses_msg.poses.append(p.get())

        return robot_poses_msg


if __name__ == '__main__':
    rospy.init_node('grsim_geometry_loader')

    receiver = VisionReceiver()

    r   = rospy.Rate(60)
    while not rospy.is_shutdown():
        # receive()
        receiver.receive()

        r.sleep()
