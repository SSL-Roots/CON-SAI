#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest
import math

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

from proto import messages_robocup_ssl_wrapper_pb2 as wrapper
from format_converter import FormatConverter
from consai_msgs.msg import VisionObservations


class TestFormatConverter(unittest.TestCase):

    def setUp(self):
        pass

    def test_protobuf_to_rosmsg(self):

        # Test Packet data
        packet = wrapper.SSL_WrapperPacket()
        detection = packet.detection
        detection.frame_number = 0
        detection.t_capture = t_capture = 1.1
        detection.t_sent = t_sent = 1.2
        detection.camera_id = camera_id = 2

        # Ball data
        ball = detection.balls.add()
        ball.confidence = 0.0
        ball.x = ball_x = 1.0
        ball.y = ball_y = 2.0
        ball.pixel_x = 0
        ball.pixel_y = 0

        # Robot data
        robot = detection.robots_blue.add()
        robot.confidence = 0.0
        robot.robot_id = robot_id = 3
        robot.x = robot_x = 3.0
        robot.y = robot_y = 4.0
        robot.orientation = orientation = math.pi
        robot.pixel_x = 0
        robot.pixel_y = 0

        message = packet.SerializeToString()

        converter = FormatConverter(friend_color='blue', do_side_invert=False)
        converter.protobufToTable(message)

        ros_msg = VisionObservations()
        ros_msg = converter.tableToRosmsg(0.0)

        # convert unit
        TO_METER = 0.001
        ball_x *= TO_METER
        ball_y *= TO_METER
        robot_x *= TO_METER
        robot_y *= TO_METER

        self.assertAlmostEqual(ros_msg.ball[0].t_capture, t_capture)
        self.assertAlmostEqual(ros_msg.ball[0].t_sent, t_sent)
        self.assertAlmostEqual(ros_msg.ball[0].camera_id, camera_id)
        self.assertAlmostEqual(ros_msg.ball[0].pose.position.x, ball_x)
        self.assertAlmostEqual(ros_msg.ball[0].pose.position.y, ball_y)

        self.assertAlmostEqual(ros_msg.friends[0].robot_id, robot_id)
        self.assertAlmostEqual(ros_msg.friends[0].packets[0].t_capture, t_capture)
        self.assertAlmostEqual(ros_msg.friends[0].packets[0].t_sent, t_sent)
        self.assertAlmostEqual(ros_msg.friends[0].packets[0].camera_id, camera_id)
        self.assertAlmostEqual(ros_msg.friends[0].packets[0].pose.position.x, robot_x)
        self.assertAlmostEqual(ros_msg.friends[0].packets[0].pose.position.y, robot_y)


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun('grsim_wrapper', 'test_format_converter', TestFormatConverter)

