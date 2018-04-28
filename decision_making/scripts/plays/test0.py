#!/usr/bin/env python
# -*- coding: utf-8 -*-

from play_base import Play

from tactics.tactic_halt import TacticHalt
from tactics.tactic_inplay_shoot import TacticInplayShoot
from tactics.tactic_position import TacticPosition
from tactics.tactic_velocity import TacticVelocity
from tactics.tactic_command import TacticCommand

import math

import sys,os
sys.path.append(os.pardir)
import constants

# 壁打ちテスト
class Test0(Play):
    def __init__(self):
        super(Test0, self).__init__('Test0')

        self.applicable = "TEST0"
        self.done_aborted = "TEST0"
        self.assignment_type = "CLOSEST_BALL"

        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticInplayShoot('TacticInplayShoot', self.roles[0].my_role))

        for i in range(1,constants.ROBOT_NUM):
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
                TacticPosition('TacticPosition1', self.roles[0].my_role,
                    -1.5, 2.0, math.pi * 0.5))
        self.roles[0].behavior.add_child(
                TacticPosition('TacticPosition2', self.roles[0].my_role, 
                    -1.5, -2.0, math.pi * 0.5))

        for i in range(1,constants.ROBOT_NUM):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticHalt("TacticHalt", self.roles[i].my_role))

# Command 送信テスト
class Test2(Play):
    def __init__(self):
        super(Test2, self).__init__('Test2')

        self.applicable = "TEST2"
        self.done_aborted = "TEST2"

        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticCommand('TacticCommand', self.roles[0].my_role))

        for i in range(1,constants.ROBOT_NUM):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticHalt("TacticHalt", self.roles[i].my_role))

# Command 一斉送信テスト
class Test3(Play):
    def __init__(self):
        super(Test3, self).__init__('Test3')

        self.applicable = "TEST3"
        self.done_aborted = "TEST3"

        for i in range(constants.ROBOT_NUM):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticCommand('TacticCommand', self.roles[i].my_role))
