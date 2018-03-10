#!/usr/bin/env python
#encoding: utf8

import rospy, unittest, rostest
import rosnode
import time


class TrapezoidalControlTest(unittest.TestCase):
    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/robot_0/trapezoidal_control', nodes, 'node does not exist')

if __name__ == '__main__':
    time.sleep(3) # テスト対象のノードが立ち上がるのを待つ
    rospy.init_node('test_trapezoidal_control')
    rostest.rosrun('trapezoidal_control', 'test_trapezoidal_control', TrapezoidalControlTest)
