#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest
from collections import OrderedDict

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

from world_model import WorldModel
from world_model import SSL_Referee
import constants

from consai_msgs.msg import RefereeTeamInfo
from nav_msgs.msg import Odometry


class TestWorldModel(unittest.TestCase):

    def setUp(self):
        self.init_odom = Odometry() # All parameters are zero

        self.test_odom = Odometry() 
        self.test_odom.pose.pose.position.x = 1.0 # Inside of field
        self.test_odom.pose.pose.position.y = 2.0
        self.test_odom.twist.twist.linear.x = 3.0 # Moving
        self.test_odom.twist.twist.linear.y = 4.0

        self.outside_odom = Odometry()
        self.outside_odom.pose.pose.position.x = 100 # Outside of field
        self.outside_odom.pose.pose.position.y = 100

        # Our defence area
        self.ourside_odom = Odometry()
        self.ourside_odom.pose.pose.position.x = -(constants.FieldHalfX - 0.3)

        # Their defence area
        self.theirside_odom = Odometry()
        self.theirside_odom.pose.pose.position.x = (constants.FieldHalfX - 0.3)


    def test_set_friend_color(self):
        blue_dict = WorldModel._refbox_dict_blue
        yellow_dict = WorldModel._refbox_dict_yellow

        color = 'blue'

        WorldModel.set_friend_color(color)
        actual = WorldModel._friend_color
        self.assertEqual(color, actual)

        actual = WorldModel._refbox_dict
        self.assertEqual(blue_dict, actual)
        
        color = 'yellow'

        WorldModel.set_friend_color(color)
        actual = WorldModel._friend_color
        self.assertEqual(color, actual)

        actual = WorldModel._refbox_dict
        self.assertEqual(yellow_dict, actual)


    def test_set_friend_goalie_id(self):
        robot_id = 0

        WorldModel.set_friend_goalie_id(robot_id)
        actual = WorldModel._friend_goalie_id

        self.assertEqual(robot_id, actual)


    def test_set_team_info(self):
        test_info = RefereeTeamInfo()
        test_info.goalie = 0

        WorldModel.set_friend_color('blue')
        WorldModel.set_yellow_info(test_info)
        actual = WorldModel._enemy_goalie_id
        self.assertEqual(test_info.goalie, actual)
        
        test_info.goalie = 0
        WorldModel.set_friend_color('yellow')
        WorldModel.set_blue_info(test_info)
        actual = WorldModel._enemy_goalie_id
        self.assertEqual(test_info.goalie, actual)


    def test_set_refbox_command(self):
        WorldModel.set_refbox_command(SSL_Referee.NORMAL_START)
        actual = WorldModel._raw_refbox_command
        self.assertEqual(SSL_Referee.NORMAL_START, actual)
        
        WorldModel._refbox_command_changed = False
        WorldModel.set_refbox_command(SSL_Referee.FORCE_START)
        actual = WorldModel._raw_refbox_command
        self.assertEqual(SSL_Referee.FORCE_START, actual)

        actual = WorldModel._refbox_command_changed
        self.assertEqual(True, actual)

        WorldModel._refbox_command_changed = False
        WorldModel.set_refbox_command(SSL_Referee.FORCE_START)
        actual = WorldModel._refbox_command_changed
        self.assertEqual(False, actual)


    def test_set_get_ball_odom(self):
        WorldModel.set_ball_odom(self.test_odom)

        actual = WorldModel.get_pose('Ball')
        self.assertAlmostEqual(self.test_odom.pose.pose.position.x, actual.x)
        self.assertAlmostEqual(self.test_odom.pose.pose.position.y, actual.y)

        actual = WorldModel.get_velocity('Ball')
        self.assertAlmostEqual(self.test_odom.twist.twist.linear.x, actual.x)
        self.assertAlmostEqual(self.test_odom.twist.twist.linear.y, actual.y)

    
    def test_set_existing_id(self):
        id_list = [0,1,2,3,4,5]

        WorldModel.set_existing_friends_id(id_list)
        actual = WorldModel._existing_friends_id
        self.assertEqual(id_list, actual)

        WorldModel.set_existing_enemies_id(id_list)
        actual = WorldModel._existing_enemies_id
        self.assertEqual(id_list, actual)


    def test_update_assignments(self):
        test_assignments = OrderedDict()

        # Test blank id_list
        id_list = []
        WorldModel.set_existing_friends_id(id_list)
        for i in range(6):
            key = 'Role_' + str(i)
            test_assignments[key] = None
        WorldModel.update_assignments()
        self.assertEqual(test_assignments, WorldModel.assignments, "Blank id_list")

        # Test ordered id_list
        id_list = [0, 1, 2, 3, 4, 5]
        WorldModel.set_existing_friends_id(id_list)
        for i in range(6):
            key = 'Role_' + str(i)
            test_assignments[key] = i
        WorldModel.update_assignments()
        self.assertEqual(test_assignments, WorldModel.assignments, "Ordered id_list")

        # Test other ID
        id_list = [0, 6, 7, 8, 9, 10]
        WorldModel.set_existing_friends_id(id_list)
        for i in range(6):
            key = 'Role_' + str(i)
            test_assignments[key] = id_list[i]
        WorldModel.update_assignments()
        self.assertEqual(test_assignments, WorldModel.assignments, "Other ID")

        # Test blank goalie ID
        id_list = [11, 6, 7, 8, 9, 10]
        WorldModel.set_existing_friends_id(id_list)
        test_assignments['Role_0'] = None
        for i in range(1,6):
            key = 'Role_' + str(i)
            test_assignments[key] = id_list[i]
        WorldModel.update_assignments()
        self.assertEqual(test_assignments, WorldModel.assignments, "Blank goalie ID")

        # Test exitsts goalie ID only
        id_list = [0]
        WorldModel.set_existing_friends_id(id_list)
        test_assignments['Role_0'] = 0
        for i in range(1,6):
            key = 'Role_' + str(i)
            test_assignments[key] = None
        WorldModel.update_assignments()
        self.assertEqual(test_assignments, WorldModel.assignments, "Goalie ID Only")

        # Test exchanges Role
        id_list = [0, 1, 2, 3]
        WorldModel.set_existing_friends_id(id_list)
        test_assignments['Role_0'] = 0
        test_assignments['Role_1'] = 1
        test_assignments['Role_2'] = 2
        test_assignments['Role_3'] = 3
        WorldModel.update_assignments()
        self.assertEqual(test_assignments, WorldModel.assignments, "1:Exchanges Role")

        id_list = [0, 2, 3]
        WorldModel.set_existing_friends_id(id_list)
        test_assignments['Role_0'] = 0
        test_assignments['Role_1'] = 3
        test_assignments['Role_2'] = 2
        test_assignments['Role_3'] = None
        WorldModel.update_assignments()
        self.assertEqual(test_assignments, WorldModel.assignments, "2:Exchanges Role")

        id_list = [0, 1, 2, 3]
        WorldModel.set_existing_friends_id(id_list)
        test_assignments['Role_0'] = 0
        test_assignments['Role_1'] = 3
        test_assignments['Role_2'] = 2
        test_assignments['Role_3'] = 1
        WorldModel.update_assignments()
        self.assertEqual(test_assignments, WorldModel.assignments, "3:Exchanges Role")

        id_list = [0, 1, 2, 3, 10, 11]
        WorldModel.set_existing_friends_id(id_list)
        test_assignments['Role_0'] = 0
        test_assignments['Role_1'] = 3
        test_assignments['Role_2'] = 2
        test_assignments['Role_3'] = 1
        test_assignments['Role_4'] = 10
        test_assignments['Role_5'] = 11
        WorldModel.update_assignments()
        self.assertEqual(test_assignments, WorldModel.assignments, "4:Exchanges Role")

        id_list = [0, 1, 3, 11]
        WorldModel.set_existing_friends_id(id_list)
        test_assignments['Role_0'] = 0
        test_assignments['Role_1'] = 3
        test_assignments['Role_2'] = 11
        test_assignments['Role_3'] = 1
        test_assignments['Role_4'] = None
        test_assignments['Role_5'] = None
        WorldModel.update_assignments()
        self.assertEqual(test_assignments, WorldModel.assignments, "5:Exchanges Role")


    def test_set_get_role_odom(self):
        # Initialize id_list
        id_list = [0, 1, 2]
        WorldModel.set_existing_friends_id(id_list)
        WorldModel.update_assignments()

        WorldModel.set_friend_odom(self.test_odom, 0)

        actual = WorldModel.get_pose('Role_0')
        self.assertAlmostEqual(self.test_odom.pose.pose.position.x, actual.x)
        self.assertAlmostEqual(self.test_odom.pose.pose.position.y, actual.y)

        actual = WorldModel.get_velocity('Role_0')
        self.assertAlmostEqual(self.test_odom.twist.twist.linear.x, actual.x)
        self.assertAlmostEqual(self.test_odom.twist.twist.linear.y, actual.y)


    def test_enemy_assignments(self):
        test_assignments = OrderedDict()

        # Test blank id_list
        id_list = []
        WorldModel.set_existing_enemies_id(id_list)
        for i in range(6):
            key = 'Enemy_' + str(i)
            test_assignments[key] = None
        WorldModel._update_enemy_assignments()
        self.assertEqual(test_assignments, WorldModel.enemy_assignments, "Blank id_list")

        # Test ordered id_list
        id_list = [0, 1, 2, 3, 4, 5]
        WorldModel.set_existing_enemies_id(id_list)
        for i in range(6):
            key = 'Enemy_' + str(i)
            test_assignments[key] = i
        WorldModel._update_enemy_assignments()
        self.assertEqual(test_assignments, WorldModel.enemy_assignments, "Ordered id_list")

        # Test other ID
        id_list = [0, 6, 7, 8, 9, 10]
        WorldModel.set_existing_enemies_id(id_list)
        for i in range(6):
            key = 'Enemy_' + str(i)
            test_assignments[key] = id_list[i]
        WorldModel._update_enemy_assignments()
        self.assertEqual(test_assignments, WorldModel.enemy_assignments, "Other ID")

        # Test blank goalie ID
        id_list = [11, 6, 7, 8, 9, 10]
        WorldModel.set_existing_enemies_id(id_list)
        test_assignments['Enemy_0'] = None
        for i in range(1,6):
            key = 'Enemy_' + str(i)
            test_assignments[key] = id_list[i]
        WorldModel._update_enemy_assignments()
        self.assertEqual(test_assignments, WorldModel.enemy_assignments, "Blank goalie ID")

        # Test exitsts goalie ID only
        id_list = [0]
        WorldModel.set_existing_enemies_id(id_list)
        test_assignments['Enemy_0'] = 0
        for i in range(1,6):
            key = 'Enemy_' + str(i)
            test_assignments[key] = None
        WorldModel._update_enemy_assignments()
        self.assertEqual(test_assignments, WorldModel.enemy_assignments, "Goalie ID Only")

        # Test exchanges Role
        id_list = [0, 1, 2, 3]
        WorldModel.set_existing_enemies_id(id_list)
        test_assignments['Enemy_0'] = 0
        test_assignments['Enemy_1'] = 1
        test_assignments['Enemy_2'] = 2
        test_assignments['Enemy_3'] = 3
        WorldModel._update_enemy_assignments()
        self.assertEqual(test_assignments, WorldModel.enemy_assignments, "1:Exchanges Role")

        id_list = [0, 2, 3]
        WorldModel.set_existing_enemies_id(id_list)
        test_assignments['Enemy_0'] = 0
        test_assignments['Enemy_1'] = 3
        test_assignments['Enemy_2'] = 2
        test_assignments['Enemy_3'] = None
        WorldModel._update_enemy_assignments()
        self.assertEqual(test_assignments, WorldModel.enemy_assignments, "2:Exchanges Role")

        id_list = [0, 1, 2, 3]
        WorldModel.set_existing_enemies_id(id_list)
        test_assignments['Enemy_0'] = 0
        test_assignments['Enemy_1'] = 3
        test_assignments['Enemy_2'] = 2
        test_assignments['Enemy_3'] = 1
        WorldModel._update_enemy_assignments()
        self.assertEqual(test_assignments, WorldModel.enemy_assignments, "3:Exchanges Role")

        id_list = [0, 1, 2, 3, 10, 11]
        WorldModel.set_existing_enemies_id(id_list)
        test_assignments['Enemy_0'] = 0
        test_assignments['Enemy_1'] = 3
        test_assignments['Enemy_2'] = 2
        test_assignments['Enemy_3'] = 1
        test_assignments['Enemy_4'] = 10
        test_assignments['Enemy_5'] = 11
        WorldModel._update_enemy_assignments()
        self.assertEqual(test_assignments, WorldModel.enemy_assignments, "4:Exchanges Role")

        id_list = [0, 1, 3, 11]
        WorldModel.set_existing_enemies_id(id_list)
        test_assignments['Enemy_0'] = 0
        test_assignments['Enemy_1'] = 3
        test_assignments['Enemy_2'] = 11
        test_assignments['Enemy_3'] = 1
        test_assignments['Enemy_4'] = None
        test_assignments['Enemy_5'] = None
        WorldModel._update_enemy_assignments()
        self.assertEqual(test_assignments, WorldModel.enemy_assignments, "5:Exchanges Role")


    def test_set_get_enemy_odom(self):
        # Initialize id_list
        id_list = [0, 1, 2]
        WorldModel.set_existing_enemies_id(id_list)
        WorldModel._update_enemy_assignments()

        WorldModel.set_enemy_odom(self.test_odom, 0)

        actual = WorldModel.get_pose('Enemy_0')
        self.assertAlmostEqual(self.test_odom.pose.pose.position.x, actual.x)
        self.assertAlmostEqual(self.test_odom.pose.pose.position.y, actual.y)

        actual = WorldModel.get_velocity('Enemy_0')
        self.assertAlmostEqual(self.test_odom.twist.twist.linear.x, actual.x)
        self.assertAlmostEqual(self.test_odom.twist.twist.linear.y, actual.y)


    def test_ball_kicked(self):
        ball_odom = self.test_odom
        WorldModel.set_ball_odom(ball_odom)

        expected = True
        actual = WorldModel.ball_kicked()
        self.assertEqual(expected, actual)

        ball_odom.twist.twist.linear.x = 0
        ball_odom.twist.twist.linear.y = 0
        WorldModel.set_ball_odom(ball_odom)

        expected = False
        actual = WorldModel.ball_kicked()
        self.assertEqual(expected, actual)


    def test_ball_is_moving(self):
        ball_odom = self.test_odom
        WorldModel.set_ball_odom(ball_odom)

        expected = True
        actual = WorldModel.ball_is_moving()
        self.assertEqual(expected, actual)

        ball_odom.twist.twist.linear.x = 0
        ball_odom.twist.twist.linear.y = 0
        WorldModel.set_ball_odom(ball_odom)

        expected = False
        actual = WorldModel.ball_is_moving()
        self.assertEqual(expected, actual)


    def test_update_situation(self):
        self._test_update_situation_color('blue')
        self._test_update_situation_color('yellow')

    def _test_update_situation_color(self, color):
        PRE_KICKOFF_BLUE = 'OUR_PRE_KICKOFF'
        KICKOFF_START_BLUE = 'OUR_KICKOFF_START'
        PRE_PENALTY_BLUE = 'OUR_PRE_PENALTY'
        PENALTY_START_BLUE = 'OUR_PENALTY_START'
        DIRECT_BLUE = 'OUR_DIRECT'
        INDIRECT_BLUE = 'OUR_INDIRECT'
        TIMEOUT_BLUE = 'OUR_TIMEOUT'
        PRE_KICKOFF_YELLOW = 'THEIR_PRE_KICKOFF'
        KICKOFF_START_YELLOW = 'THEIR_KICKOFF_START'
        PRE_PENALTY_YELLOW = 'THEIR_PRE_PENALTY'
        PENALTY_START_YELLOW = 'THEIR_PENALTY_START'
        DIRECT_YELLOW = 'THEIR_DIRECT'
        INDIRECT_YELLOW = 'THEIR_INDIRECT'
        TIMEOUT_YELLOW = 'THEIR_TIMEOUT'

        if color == 'yellow':
            PRE_KICKOFF_BLUE = 'THEIR_PRE_KICKOFF'
            KICKOFF_START_BLUE = 'THEIR_KICKOFF_START'
            PRE_PENALTY_BLUE = 'THEIR_PRE_PENALTY'
            PENALTY_START_BLUE = 'THEIR_PENALTY_START'
            DIRECT_BLUE = 'THEIR_DIRECT'
            INDIRECT_BLUE = 'THEIR_INDIRECT'
            TIMEOUT_BLUE = 'THEIR_TIMEOUT'
            PRE_KICKOFF_YELLOW = 'OUR_PRE_KICKOFF'
            KICKOFF_START_YELLOW = 'OUR_KICKOFF_START'
            PRE_PENALTY_YELLOW = 'OUR_PRE_PENALTY'
            PENALTY_START_YELLOW = 'OUR_PENALTY_START'
            DIRECT_YELLOW = 'OUR_DIRECT'
            INDIRECT_YELLOW = 'OUR_INDIRECT'
            TIMEOUT_YELLOW = 'OUR_TIMEOUT'


        # Initialize information
        WorldModel.set_friend_color(color)
        WorldModel.set_ball_odom(self.init_odom)
        
        self._test_situation(SSL_Referee.HALT, 'HALT')

        self._test_situation(SSL_Referee.STOP, 'STOP')
        self._test_situation(SSL_Referee.FORCE_START, 'FORCE_START')
        WorldModel.set_ball_odom(self.test_odom) # Moving Ball
        self._test_situation(SSL_Referee.NORMAL_START, 'IN_PLAY')
        WorldModel.set_ball_odom(self.init_odom) # Stop Ball

        self._test_situation(SSL_Referee.STOP, 'STOP')
        self._test_situation(SSL_Referee.PREPARE_KICKOFF_BLUE, PRE_KICKOFF_BLUE) 
        self._test_situation(SSL_Referee.NORMAL_START, KICKOFF_START_BLUE)
        WorldModel.set_ball_odom(self.test_odom) # Moving Ball
        self._test_situation(SSL_Referee.NORMAL_START, 'IN_PLAY')
        WorldModel.set_ball_odom(self.init_odom) # Stop Ball

        self._test_situation(SSL_Referee.STOP, 'STOP')
        self._test_situation(SSL_Referee.PREPARE_PENALTY_BLUE, PRE_PENALTY_BLUE)
        self._test_situation(SSL_Referee.NORMAL_START, PENALTY_START_BLUE)
        WorldModel.set_ball_odom(self.test_odom) # Moving Ball
        self._test_situation(SSL_Referee.NORMAL_START, 'IN_PLAY')
        WorldModel.set_ball_odom(self.init_odom) # Stop Ball

        self._test_situation(SSL_Referee.STOP, 'STOP')
        self._test_situation(SSL_Referee.DIRECT_FREE_BLUE, DIRECT_BLUE)
        WorldModel.set_ball_odom(self.test_odom) # Moving Ball
        self._test_situation(SSL_Referee.DIRECT_FREE_BLUE, 'IN_PLAY')
        WorldModel.set_ball_odom(self.init_odom) # Stop Ball

        self._test_situation(SSL_Referee.STOP, 'STOP')
        self._test_situation(SSL_Referee.INDIRECT_FREE_BLUE, INDIRECT_BLUE)
        WorldModel.set_ball_odom(self.test_odom) # Moving Ball
        self._test_situation(SSL_Referee.INDIRECT_FREE_BLUE, 'IN_PLAY')
        WorldModel.set_ball_odom(self.init_odom) # Stop Ball

        self._test_situation(SSL_Referee.STOP, 'STOP')
        self._test_situation(SSL_Referee.TIMEOUT_BLUE, TIMEOUT_BLUE)

        self._test_situation(SSL_Referee.STOP, 'STOP')
        self._test_situation(SSL_Referee.PREPARE_KICKOFF_YELLOW, PRE_KICKOFF_YELLOW)
        self._test_situation(SSL_Referee.NORMAL_START, KICKOFF_START_YELLOW)
        WorldModel.set_ball_odom(self.test_odom) # Moving Ball
        self._test_situation(SSL_Referee.NORMAL_START, 'IN_PLAY')
        WorldModel.set_ball_odom(self.init_odom) # Stop Ball

        self._test_situation(SSL_Referee.STOP, 'STOP')
        self._test_situation(SSL_Referee.PREPARE_PENALTY_YELLOW, PRE_PENALTY_YELLOW)
        self._test_situation(SSL_Referee.NORMAL_START, PENALTY_START_YELLOW)
        WorldModel.set_ball_odom(self.test_odom) # Moving Ball
        self._test_situation(SSL_Referee.NORMAL_START, 'IN_PLAY')
        WorldModel.set_ball_odom(self.init_odom) # Stop Ball

        self._test_situation(SSL_Referee.STOP, 'STOP')
        self._test_situation(SSL_Referee.DIRECT_FREE_YELLOW, DIRECT_YELLOW)
        WorldModel.set_ball_odom(self.test_odom) # Moving Ball
        self._test_situation(SSL_Referee.DIRECT_FREE_YELLOW, 'IN_PLAY')
        WorldModel.set_ball_odom(self.init_odom) # Stop Ball

        self._test_situation(SSL_Referee.STOP, 'STOP')
        self._test_situation(SSL_Referee.INDIRECT_FREE_YELLOW, INDIRECT_YELLOW)
        WorldModel.set_ball_odom(self.test_odom) # Moving Ball
        self._test_situation(SSL_Referee.INDIRECT_FREE_YELLOW, 'IN_PLAY')
        WorldModel.set_ball_odom(self.init_odom) # Stop Ball

        self._test_situation(SSL_Referee.STOP, 'STOP')
        self._test_situation(SSL_Referee.TIMEOUT_YELLOW, TIMEOUT_YELLOW)

        self._test_situation(SSL_Referee.STOP, 'STOP')
        self._test_situation(SSL_Referee.FORCE_START, 'FORCE_START')
        WorldModel.set_ball_odom(self.test_odom) # Moving Ball
        self._test_situation(SSL_Referee.NORMAL_START, 'IN_PLAY')
        WorldModel.set_ball_odom(self.outside_odom) # Ball out of field
        self._test_situation(SSL_Referee.NORMAL_START, 'BALL_IN_OUTSIDE')
        WorldModel.set_ball_odom(self.ourside_odom) # Ball in our defence_area
        self._test_situation(SSL_Referee.NORMAL_START, 'BALL_IN_OUR_DEFENCE')
        WorldModel.set_ball_odom(self.theirside_odom) # Ball in their defence_area
        self._test_situation(SSL_Referee.NORMAL_START, 'BALL_IN_THEIR_DEFENCE')
        WorldModel.set_ball_odom(self.init_odom) # Initialize Ball odom

        
        self._test_situation(SSL_Referee.STOP, 'STOP')
        WorldModel.set_test_name("TEST0")
        self._test_situation(SSL_Referee.STOP, 'TEST0')
        self._test_situation(SSL_Referee.HALT, 'HALT')
        self._test_situation(SSL_Referee.STOP, 'TEST0')
        WorldModel.set_test_name("") # Initialize TEST situation
        self._test_situation(SSL_Referee.STOP, 'STOP')


    def _test_situation(self, refbox_command, expected):
        WorldModel.set_refbox_command(refbox_command)
        WorldModel._update_situation()
        actual = WorldModel._current_situation
        self.assertEqual(actual, expected, "Wrong situation :" + expected)


    def test_update_closest_role(self):

        # Initialize
        id_list = []
        WorldModel.set_existing_friends_id(id_list)
        WorldModel.update_assignments()

        id_list = [0, 1, 2, 3, 4, 5]
        WorldModel.set_existing_friends_id(id_list)
        for i in range(6):
            WorldModel.set_friend_odom(self.outside_odom, i)
        WorldModel.update_assignments()

        # Stop Ball
        WorldModel.set_ball_odom(self.init_odom) 
        WorldModel.set_friend_odom(self.init_odom, 3)
        expected = "Role_3"
        actual = WorldModel._update_closest_role(True)
        self.assertEqual(expected, actual)
    
        # Moving Ball
        moving_odom = Odometry()
        moving_odom.pose.pose.position.x = 1.0
        moving_odom.twist.twist.linear.x = 2.0
        WorldModel.set_ball_odom(moving_odom)

        target_odom = Odometry()
        target_odom.pose.pose.position.x = 3.0
        WorldModel.set_friend_odom(target_odom, 2)
        expected = "Role_2"
        actual = WorldModel._update_closest_role(True)
        self.assertEqual(expected, actual)


        # Enemy
        id_list = []
        WorldModel.set_existing_enemies_id(id_list)
        WorldModel._update_enemy_assignments()

        id_list = [0, 1, 2, 3, 4, 5]
        WorldModel.set_existing_enemies_id(id_list)
        for i in range(6):
            WorldModel.set_enemy_odom(self.outside_odom, i)
        WorldModel._update_enemy_assignments()

        # Stop Ball
        WorldModel.set_ball_odom(self.init_odom) 
        WorldModel.set_enemy_odom(self.init_odom, 3)
        expected = "Enemy_3"
        actual = WorldModel._update_closest_role(False)
        self.assertEqual(expected, actual)

        # Moving Ball
        WorldModel.set_ball_odom(moving_odom)
        WorldModel.set_enemy_odom(target_odom, 2)
        expected = "Enemy_2"
        actual = WorldModel._update_closest_role(False)
        self.assertEqual(expected, actual)


    def test_update_object_states(self):
        # Initialize
        WorldModel.set_ball_odom(self.test_odom)

        id_list = [0, 1, 2, 3, 4, 5]
        WorldModel.set_existing_friends_id(id_list)
        WorldModel.set_existing_enemies_id(id_list)
        for i in range(6):
            WorldModel.set_friend_odom(self.test_odom, i)
            WorldModel.set_enemy_odom(self.test_odom, i)
        WorldModel.update_assignments()
        WorldModel._update_enemy_assignments()


        WorldModel._update_object_states()

        states = WorldModel.get_object_states()

        expected = self.test_odom.pose.pose.position.x
        actual = states['Ball'].get_pose().x
        self.assertAlmostEqual(expected, actual)

        actual = states['Role_2'].get_pose().x
        self.assertAlmostEqual(expected, actual)

    def test_can_receive(self):
        # Initialize
        WorldModel.set_ball_odom(self.test_odom)

        id_list = [0, 1, 2, 3, 4, 5]
        WorldModel.set_existing_friends_id(id_list)
        WorldModel.set_existing_enemies_id(id_list)
        for i in range(6):
            WorldModel.set_friend_odom(self.test_odom, i)
            WorldModel.set_enemy_odom(self.test_odom, i)
        WorldModel.update_assignments()
        WorldModel._update_enemy_assignments()

        expected = False
        actual = WorldModel.can_receive('Role_0')
        self.assertEqual(expected, actual)


if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_world_model', TestWorldModel)
