#!/usr/bin/env python
# -*- coding: utf-8 -*-

from play_base import Play

from tactics.tactic_halt import TacticHalt
from tactics.tactic_inplay_shoot import TacticInplayShoot
from tactics.tactic_position import TacticPosition
from tactics.tactic_velocity import TacticVelocity
from tactics.tactic_command import TacticCommand, TacticTrapezoidalCommand
from tactics.tactic_formation import TacticFormation

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

        self.roles[0].behavior.add_child(
                TacticHalt("TacticHalt", self.roles[0].my_role))

        self.roles[1].loop_enable = True
        self.roles[1].behavior.add_child(
                TacticPosition('TacticPosition1', self.roles[1].my_role,
                    -1.5, 2.0, math.pi * 0.5))
        self.roles[1].behavior.add_child(
                TacticPosition('TacticPosition2', self.roles[1].my_role, 
                    -1.5, -2.0, math.pi * 0.5))

        for i in range(2,constants.ROBOT_NUM):
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

# Formationのテスト
class Test4(Play):
    def __init__(self):
        super(Test4, self).__init__('Test4')

        self.applicable = "TEST4"
        self.done_aborted = "TEST4"

        for i in range(constants.ROBOT_NUM):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticFormation('TacticFormation', self.roles[i].my_role))

# x方向の台形指令テスト
class Test5(Play):
    def __init__(self):
        super(Test5, self).__init__('Test5')

        self.applicable = "TEST5"
        self.done_aborted = "TEST5"

        self.roles[0].behavior.add_child(
                TacticHalt("TacticHalt", self.roles[0].my_role))

        self.roles[1].loop_enable = True
        self.roles[1].behavior.add_child(
                TacticTrapezoidalCommand('TacticTrapezoidalCommand', 
                    self.roles[1].my_role, True, False, False))

        for i in range(2,constants.ROBOT_NUM):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticHalt("TacticHalt", self.roles[i].my_role))

# y方向の台形指令テスト
class Test6(Play):
    def __init__(self):
        super(Test6, self).__init__('Test6')

        self.applicable = "TEST6"
        self.done_aborted = "TEST6"

        self.roles[0].behavior.add_child(
                TacticHalt("TacticHalt", self.roles[0].my_role))

        self.roles[1].loop_enable = True
        self.roles[1].behavior.add_child(
                TacticTrapezoidalCommand('TacticTrapezoidalCommand', 
                    self.roles[1].my_role, False, True, False))

        for i in range(2,constants.ROBOT_NUM):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticHalt("TacticHalt", self.roles[i].my_role))

# yaw方向の台形指令テスト
class Test7(Play):
    def __init__(self):
        super(Test7, self).__init__('Test7')

        self.applicable = "TEST7"
        self.done_aborted = "TEST7"

        self.roles[0].behavior.add_child(
                TacticHalt("TacticHalt", self.roles[0].my_role))

        self.roles[1].loop_enable = True
        self.roles[1].behavior.add_child(
                TacticTrapezoidalCommand('TacticTrapezoidalCommand', 
                    self.roles[1].my_role, False, False, True))

        for i in range(2,constants.ROBOT_NUM):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticHalt("TacticHalt", self.roles[i].my_role))
