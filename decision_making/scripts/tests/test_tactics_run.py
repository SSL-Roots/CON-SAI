#!/usr/bin/env python
#encoding: utf8

import sys,os
import unittest
pardir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pardir)

from pi_trees_lib.pi_trees_lib import TaskStatus

from world_model import WorldModel
from plays.tactics.tactic_halt import TacticHalt
from plays.tactics.tactic_attacker import TacticAttacker

class TestTacticsRun(unittest.TestCase):

    def setUp(self):
        pass

    def test_tactic_halt(self):
        tactic = TacticHalt("TacticHalt", "Role_0")
        expected = TaskStatus.RUNNING
        actual = tactic.run()

        self.assertEqual(expected, actual)

    def test_tactic_attacker(self):
        tactic = TacticAttacker("TactickAttacker", "Role_0")
        expected = TaskStatus.RUNNING
        actual = tactic.run()

        self.assertEqual(expected, actual)



if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_tactics_run', TestTacticsRun)
