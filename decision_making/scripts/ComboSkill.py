# -*- coding: utf-8 -*-
import rospy
import tf
import math

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from GlobalData import GlobalInfo
from geometry_msgs.msg import Point

import BaseSkill
import Kick
import Analyst


class PassShoot(Sequence):
    def __init__(self, name, passerNo, shooterNo):
        super(PassShoot, self).__init__(name)

        # 1 passerがボールに近づく
        FIRST = ParallelOne("FIRST")
        FIRST.add_child(Kick.SetPassTarget("Set pass target", passerNo, shooterNo))
        FIRST.add_child(Kick.Approach("Passer approach", passerNo))
        FIRST.add_child(BaseSkill.SuccessTimer("SuccessTimer1.0", 1.0))
        self.add_child(FIRST)

        # 2 shooterがshoot位置に移動する
        SECOND = ParallelOne("SECOND")
        SECOND.add_child(BaseSkill.WaitToReceive("Shooter wait", shooterNo))
        SECOND.add_child(Analyst.IsOnTarget("Shooter is on target", shooterNo, 0.5))
        SECOND.add_child(BaseSkill.SuccessTimer("SuccessTimer2.0", 3.0))
        self.add_child(SECOND)
        #
        # 3 passerがボールを蹴る
        THIRD = ParallelAll("THIRD")
        THIRD.add_child(Kick.SetPassTarget("Set pass target", passerNo, shooterNo))
        THIRD.add_child(Kick.Pass("PASS", passerNo))

        # 3 shooterが受け取りshootする
        THIRD.add_child(Kick.CalcuKickTarget("CalcuKickTarget", shooterNo))
        THIRD.add_child(Kick.ReceiveShoot("ReceiveShoot", shooterNo))
        THIRD.add_child(BaseSkill.FailureTimer("FailureTimer7.0", 10.0))
        INVERT = Invert("Invert")
        INVERT.add_child(Analyst.IsBaller("IsBaller", shooterNo, 1.0))
        THIRD.add_child(INVERT)
        self.add_child(THIRD)
