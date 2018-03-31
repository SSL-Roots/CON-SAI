#!/usr/bin/env python
#encoding: utf8

import sys

from scripts.world_model import WorldModel
import unittest

class TestWorldModel(unittest.TestCase):
    
    def setUp(self):
        self.world_model = WorldModel()

    def test_set_friend_color(self):
        blue_dict = self.world_model._refbox_dict_blue
        yellow_dict = self.world_model._refbox_dict_yellow

        color = 'blue'

        self.world_model.set_friend_color(color)
        result = self.world_model._friend_color
        self.assertEqual(color, result)

        result = self.world_model._refbox_dict
        self.assertEqual(blue_dict, result)
        
        color = 'yellow'

        self.world_model.set_friend_color(color)
        result = self.world_model._friend_color
        self.assertEqual(color, result)

        result = self.world_model._refbox_dict
        self.assertEqual(yellow_dict, result)


    def test_set_friend_goalie_id(self):
        robot_id = 2

        self.world_model.set_friend_goalie_id(robot_id)
        result = self.world_model._friend_goalie_id

        self.assertEqual(robot_id, result)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_world_model', TestWorldModel)
    # unittest.main()

