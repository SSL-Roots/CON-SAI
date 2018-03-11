#!/usr/bin/env python
#encoding: utf8

import rospy, unittest, rostest
import rosnode
import time


class VelocityTransformerTest(unittest.TestCase):
    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/robot_0/velocity_transformer', nodes, 'node does not exist')

if __name__ == '__main__':
    time.sleep(3) # テスト対象のノードが立ち上がるのを待つ
    rospy.init_node('test_velocity_transformer')
    rostest.rosrun('velocity_transformer', 'test_velocity_transformer', VelocityTransformerTest)
