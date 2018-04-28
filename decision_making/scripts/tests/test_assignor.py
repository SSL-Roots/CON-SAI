#!/usr/bin/env python
#encoding: utf8

import sys, os
import unittest
from collections import OrderedDict

pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

import assignor
import constants

class TestAssignor(unittest.TestCase):

    def setUp(self):
        self._assignments = OrderedDict()

        for i in range(constants.ROBOT_NUM):
            key = 'Role_' + str(i)
            self._assignments[key] = None

    def test_update_assignments(self):
        id_list = [0, 1, 2]
        goalie_id = 0

        expected = self._assignments
        expected['Role_0'] = 0
        expected['Role_1'] = 1
        expected['Role_2'] = 2
        actual = assignor.update_assignments(
                self._assignments, id_list, 'Role_', goalie_id)
        self.assertEqual(expected, actual)

        expected = self._assignments
        expected['Role_0'] = 0
        expected['Role_1'] = 2
        expected['Role_2'] = 1
        actual = assignor.update_assignments(
                self._assignments, id_list, 'Role_', goalie_id,
                'CLOSEST_BALL', 'Role_2')
        self.assertEqual(expected, actual)

        actual = assignor.update_assignments(
                self._assignments, id_list, 'Role_', goalie_id,
                'CLOSEST_BALL', None)
        self.assertEqual(expected, actual)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_assignor', TestAssignor)
