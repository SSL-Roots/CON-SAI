#!/usr/bin/env python
#encoding: utf8

import rospy, unittest, rostest
import rosnode
import time


class SSLRefboxWrapperTest(unittest.TestCase):
    def test_node_exist(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/refbox', nodes, 'node does not exist')

if __name__ == '__main__':
    time.sleep(3) # テスト対象のノードが立ち上がるのを待つ
    rospy.init_node('test_ssl_refbox_wrapper')
    rostest.rosrun('ssl_refbox_wrapper', 'test_ssl_refbox_wrapper', SSLRefboxWrapperTest)
