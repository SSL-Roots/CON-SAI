#!/usr/bin/env  python

import rospy
import multicast

from proto import messages_robocup_ssl_wrapper_pb2
from lib.format_converter import FormatConverter

from consai_msgs.msg import VisionObservations, FixedVisionPackets
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

        self._pubs_friend_vision = []
        self._pubs_enemy_vision = []
        self._pub_ball_vision = rospy.Publisher(
                'ball_vision_packets', FixedVisionPackets, queue_size=10)

        for robot_id in range(12):
            id_str = str(robot_id)
            topic_friend_vision = "robot_" + id_str + "/vision_packets"
            topic_enemy_vision = "enemy_" + id_str + "/vision_packets"

            self._pubs_friend_vision.append(
                    rospy.Publisher(
                        topic_friend_vision,
                        FixedVisionPackets,
                        queue_size=10))
            self._pubs_enemy_vision.append(
                    rospy.Publisher(
                        topic_enemy_vision,
                        FixedVisionPackets,
                        queue_size=10))

        self._do_side_invert = True
        if self._our_side == 'LEFT':
            self._do_side_invert = False

        self._format_converter = FormatConverter(self._our_color.lower(), self._do_side_invert)

        self._TO_METER = 0.001


    def receive(self):
        buf = self._sock.recv(2*1024)
        if buf == None:
            return  False

        while not buf == None:
            self._format_converter.protobufToTable(buf)
            self._receive_geometry(buf)
            buf = self._sock.recv(2*1024)

        ros_msg = self._format_converter.tableToRosmsg(rospy.Time.now())
        self._format_converter.refreshTable()
        self._vision_observations_publisher.publish(ros_msg)
        self._publish_fixed_packets()


    def _publish_fixed_packets(self):
        friend_packets = self._format_converter.get_friend_packets()
        enemy_packets = self._format_converter.get_enemy_packets()

        for packets in friend_packets:
            robot_id = packets.robot_id
            self._pubs_friend_vision[robot_id].publish(packets)

        for packets in enemy_packets:
            robot_id = packets.robot_id
            self._pubs_enemy_vision[robot_id].publish(packets)

        ball_packets = self._format_converter.get_ball_packets()
        self._pub_ball_vision.publish(ball_packets)


    def _receive_geometry(self, buf):
        ssl_wrapper = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
        ssl_wrapper.ParseFromString(buf)


        if ssl_wrapper.HasField('geometry'):
            field_size = GeometryFieldSize()

            field = ssl_wrapper.geometry.field

            # SSL_geometry uses milli meters unit
            # CON-SAI uses meter
            field_size.field_length = field.field_length * self._TO_METER
            field_size.field_width = field.field_width * self._TO_METER
            field_size.goal_width = field.goal_width * self._TO_METER
            field_size.goal_depth = field.goal_depth * self._TO_METER
            field_size.boundary_width = field.boundary_width * self._TO_METER

        
            for line in field.field_lines:
                line_segment = FieldLineSegment()

                line_segment.name = line.name
                line_segment.p1_x = line.p1.x * self._TO_METER
                line_segment.p1_y = line.p1.y * self._TO_METER
                line_segment.p2_x = line.p2.x * self._TO_METER
                line_segment.p2_y = line.p2.y * self._TO_METER
                line_segment.thickness = line.thickness * self._TO_METER
                field_size.field_lines.append(line_segment)
            
            for arc in field.field_arcs:
                circular_arc = FieldCircularArc()

                circular_arc.name = arc.name
                circular_arc.center_x = arc.center.x * self._TO_METER
                circular_arc.center_y = arc.center.y * self._TO_METER
                circular_arc.radius = arc.radius * self._TO_METER
                circular_arc.a1 = arc.a1
                circular_arc.a2 = arc.a2
                circular_arc.thickness = arc.thickness * self._TO_METER
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
