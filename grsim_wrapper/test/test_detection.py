#!/usr/bin/env python
#encoding: utf8

import rospy, unittest, rostest
import rosnode
import time


class DetectionTest(unittest.TestCase):
    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/receiver', nodes, 'node does not exist')

if __name__ == '__main__':
    time.sleep(3) # テスト対象のノードが立ち上がるのを待つ
    rospy.init_node('test_detection')
    rostest.rosrun('grsim_wrapper', 'test_detection', DetectionTest)
