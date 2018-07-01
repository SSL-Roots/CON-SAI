#!/usr/bin/env  python
import rospy
import tf
# import math
import multicast

from proto import messages_robocup_ssl_wrapper_pb2
from lib.pose_maker import PoseMaker
from lib.format_converter import FormatConverter

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from consai_msgs.msg import RobotPoses, VisionObservations, VisionRobotPackets, VisionPacket
from consai_msgs.msg import GeometryFieldSize, FieldLineSegment, FieldCircularArc


class VisionReceiver:
    def __init__(self):

        self._our_color = rospy.get_param('friend_color', 'yellow').upper()
        self._our_side = rospy.get_param('team_side', 'right').upper()
        self._host = rospy.get_param('~multicast_addr', '224.5.23.2')
        self._port = rospy.get_param('~multicast_port', 10006)

        self._sock = multicast.Multicast(self._host, self._port)


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

        if self._our_color == 'BLUE':
            detection_friend = ssl_wrapper.detection.robots_blue
            detection_enemy = ssl_wrapper.detection.robots_yellow
        else:
            detection_friend = ssl_wrapper.detection.robots_yellow
            detection_enemy = ssl_wrapper.detection.robots_blue


    def _receive_geometry(self, buf):
        ssl_wrapper = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
        ssl_wrapper.ParseFromString(buf)


        if ssl_wrapper.HasField('geometry'):
            field_size = GeometryFieldSize()

            field = ssl_wrapper.geometry.field

            # SSL_geometry uses milli meters unit
            # CON-SAI uses meter

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
