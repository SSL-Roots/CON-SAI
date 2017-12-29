#!/usr/bin/env python
# -*- coding: utf-8 -*-

from play_base import Play

from tactics.tactic_halt import TacticHalt
from tactics.tactic_inplay_shoot import TacticInplayShoot
from tactics.tactic_position import TacticPosition

import math


# 壁打ちテスト
class Test0(Play):
    def __init__(self):
        super(Test0, self).__init__('Test0')

        self.applicable = "TEST0"
        self.done_aborted = "TEST0"
        self.assignment_type = "CLOSEST_BALL"

        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticHalt('TacticHalt', self.roles[0].my_role))

        self.roles[1].loop_enable = True
        self.roles[1].behavior.add_child(
                TacticInplayShoot('TacticInplayShoot', self.roles[1].my_role))

        for i in range(2,6):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticHalt("TacticHalt", self.roles[i].my_role))


# 雑巾がけテスト
class Test1(Play):
    def __init__(self):
        super(Test1, self).__init__('Test1')

        self.applicable = "TEST1"
        self.done_aborted = "TEST1"
        self.assignment_type = "CLOSEST_BALL"

        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticHalt('TacticHalt', self.roles[0].my_role))

        self.roles[1].loop_enable = True
        self.roles[1].behavior.add_child(
                TacticPosition('TacticPosition1', self.roles[1].my_role,
                    0.0, 2.0, math.pi * 0.5))
        self.roles[1].behavior.add_child(
                TacticPosition('TacticPosition2', self.roles[1].my_role, 
                    0.0, -2.0, math.pi * 0.5))

        for i in range(2,6):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticHalt("TacticHalt", self.roles[i].my_role))
