#!/usr/bin/env  python
import rospy
import multicast

from consai_msgs.msg import VisionObservations
from consai_msgs.msg import GeometryFieldSize, FieldLineSegment, FieldCircularArc

from proto import messages_robocup_ssl_wrapper_pb2
from format_converter import FormatConverter


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


    def _receive_geometry(self, buf):
        ssl_wrapper = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
        ssl_wrapper.ParseFromString(buf)


        if ssl_wrapper.HasField('geometry'):
            field_size = GeometryFieldSize()

            field = ssl_wrapper.geometry.field

            # SSL_geometry uses meter unit
            # CON-SAI uses milli meters

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
