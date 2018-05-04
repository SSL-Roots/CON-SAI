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
from plays.tactics.tactic_setplay_shoot import TacticSetplayShoot
from plays.tactics.tactic_shoot import TacticShoot
from plays.tactics.tactic_pass import TacticPass
from plays.tactics.tactic_receive import TacticReceive
from plays.tactics.tactic_formation import TacticFormation
from plays.tactics.tactic_placement import TacticPlacement
from plays.tactics.tactic_reflect_shoot import TacticReflectShoot

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
        expected = TaskStatus.FAILURE
        actual = tactic.run()

        self.assertEqual(expected, actual)

    def test_tactic_setplay_shoot(self):
        tactic = TacticSetplayShoot("TacticSetplayShoot", "Role_0")
        expected = TaskStatus.RUNNING
        actual = tactic.run()

        self.assertEqual(expected, actual)

    def test_tactic_shoot(self):
        tactic = TacticShoot("TacticShoot", "Role_0")
        expected = TaskStatus.FAILURE
        actual = tactic.run()

        self.assertEqual(expected, actual)

    def test_tactic_pass(self):
        tactic = TacticPass("TacticPass", "Role_0")
        expected = TaskStatus.FAILURE
        actual = tactic.run()

        self.assertEqual(expected, actual)

    def test_tactic_receive(self):
        tactic = TacticReceive("TacticReceive", "Role_0")
        expected = TaskStatus.FAILURE
        actual = tactic.run()

        self.assertEqual(expected, actual)

    def test_tactic_formation(self):
        tactic = TacticFormation("TacticFormation", "Role_0")
        expected = TaskStatus.RUNNING
        actual = tactic.run()

        self.assertEqual(expected, actual)

    def test_tactic_placement(self):
        tactic = TacticPlacement("TacticPlacement", "Role_0")
        expected = TaskStatus.FAILURE
        actual = tactic.run()

        self.assertEqual(expected, actual)

    def test_tactic_reflect_shoot(self):
        tactic = TacticReflectShoot("TacticReflectShoot", "Role_0")
        expected = TaskStatus.FAILURE
        actual = tactic.run()

        self.assertEqual(expected, actual)

if __name__ == "__main__":
    import rosunit
    rosunit.unitrun('decision_making', 'test_tactics_run', TestTacticsRun)
