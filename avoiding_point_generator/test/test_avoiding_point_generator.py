#!/usr/bin/env python
#encoding: utf8

import rospy, unittest, rostest
import rosnode
import time


class AvoidingPointGeneratorTest(unittest.TestCase):
    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/avoiding_point_generator', nodes, 'node does not exist')

if __name__ == '__main__':
    time.sleep(3) # テスト対象のノードが立ち上がるのを待つ
    rospy.init_node('test_avoiding_point_generator')
    rostest.rosrun('avoiding_point_generator', 'test_avoiding_point_generator', AvoidingPointGeneratorTest)
