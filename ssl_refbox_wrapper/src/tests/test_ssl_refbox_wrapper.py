#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

import ssl_refbox_wrapper_node as wrapper
import referee_pb2
from consai_msgs.msg import Pose


class TestRefboxWrpper(unittest.TestCase):
    def setup(self):
        pass

    def test_extract_designated_position(self):
        protobuf = referee_pb2.SSL_Referee()
        our_side = "LEFT"

        has_pose, pose = wrapper.extract_designated_position(
                protobuf, our_side)

        expected = False
        self.assertEqual(expected, has_pose)
        expected_pose = Pose(0,0,0)
        self.assertAlmostEqual(expected_pose, pose)

        protobuf.designated_position.x = 1000
        protobuf.designated_position.y = 2000
        has_pose, pose = wrapper.extract_designated_position(
                protobuf, our_side)

        expected = True
        self.assertEqual(expected, has_pose)
        expected_pose = Pose(1.0,2.0,0)
        self.assertAlmostEqual(expected_pose, pose)

        our_side = "RIGHT"
        has_pose, pose = wrapper.extract_designated_position(
                protobuf, our_side)

        expected = True
        self.assertEqual(expected, has_pose)
        expected_pose = Pose(-1.0,-2.0,0)
        self.assertAlmostEqual(expected_pose, pose)



if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('ssl_refbox_wrapper', 'test_ssl_refbox_wrapper', TestRefboxWrpper)
