#!/usr/bin/env  python
import rospy
import tf
import math
import multicast

from proto import messages_robocup_ssl_wrapper_pb2

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
from consai_msgs.msg import VisionObservations, VisionRobotPackets, VisionPacket
from consai_msgs.msg import GeometryFieldSize, FieldLineSegment, FieldCircularArc


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
        pose = Pose()

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
            self._receive_geometry(buf)
            buf = self._sock.recv(2*1024)

        ros_msg = self._format_converter.tableToRosmsg()
        self._format_converter.refreshTable()
        self._vision_observations_publisher.publish(ros_msg)


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


if __name__ == '__main__':
    rospy.init_node('grsim_geometry_loader')

    receiver = VisionReceiver()

    r   = rospy.Rate(60)
    while not rospy.is_shutdown():
        # receive()
        receiver.receive()

        r.sleep()
