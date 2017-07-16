# -*- coding: utf-8 -*-

import rospy
import tf
import math

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from GlobalData import GlobalInfo
from geometry_msgs.msg import Point

import Tool
import Constants
import AnalystTool


class AllRobots_Halt(Task):
    def __init__(self, name):
        super(AllRobots_Halt, self).__init__(name)

    def run(self):
        self.announce()

        for control in GlobalInfo.controls:
            control.setTargetVelocity(0.0, 0.0, 0.0)
            control.setKickVelocity(0.0)

        return  TaskStatus.RUNNING

class Stay(Task):
    def __init__(self, name, number, x, y, yaw):
        super(Stay, self).__init__(name)

        self._number = number
        self._poseX = x
        self._poseY = y
        self._poseYaw = yaw

    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.SUCCESS

        GlobalInfo.controls[self._number] \
                .setTargetPose(self._poseX, self._poseY, self._poseYaw, "map", True)
        GlobalInfo.controls[self._number].setKickVelocity(0.0)

        return TaskStatus.RUNNING


class WaitToReceive(Task):
    def __init__(self, name, number):
        super(WaitToReceive, self).__init__(name)

        self._number = number
        self._waitPosXList = [1.0, 2.0, 3.0]
        self._targetList = [
                Point(Constants.GoalEnemy.x, Constants.GoalHalfSize, 0),
                Point(Constants.GoalEnemy.x, 0, 0),
                Point(Constants.GoalEnemy.x, -Constants.GoalHalfSize, 0)]

        self._waitPosYLimit = 2.5

    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.RUNNING


        ballPos = GlobalInfo.ball.pose.pose.position
        
        waitPos = Point()
        waitPos.y = -ballPos.y

        if waitPos.y > self._waitPosYLimit:
            waitPos.y = self._waitPosYLimit
        elif waitPos.y < -self._waitPosYLimit:
            waitPos.y = -self._waitPosYLimit

        # ボールとwaitPos間に敵ロボットがいないかチェック
        for waitX in self._waitPosXList:
            waitPos.x = waitX
            if AnalystTool.isNoObstacle(ballPos, waitPos):
                break
        
        x = waitPos.x
        y = waitPos.y
        yaw = Tool.getAngle(waitPos, ballPos)

        GlobalInfo.controls[self._number] \
                .setTargetPose(x, y, yaw, "map", True)
        GlobalInfo.controls[self._number].setKickVelocity(0.0)

        return TaskStatus.RUNNING

class WaitPenaltyShoot(Task):
    def __init__(self, name, number):
        super(WaitPenaltyShoot, self).__init__(name)

        self._number = number
        self._waitPos = Point()

        self._waitPos.x = -2.0 + 0.4*self._number
        self._waitPos.y = 2.0

        self._waitYaw = - 0.5*math.pi


    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.RUNNING

        GlobalInfo.controls[self._number] \
                .setTargetPose(self._waitPos.x, self._waitPos.y, self._waitYaw, "map", True)
        GlobalInfo.controls[self._number].setKickVelocity(0.0)

        return TaskStatus.RUNNING

class SuccessTimer(Task):
    def __init__(self, name, timeout):
        super(SuccessTimer, self).__init__(name)

        self._timeout = timeout
        self.start_time = rospy.get_time()
        self.count_start = False

    def run(self):
        if self.count_start == False:
            self.start_time = rospy.get_time()
            self.count_start = True

        if rospy.get_time() - self.start_time > self._timeout:
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.RUNNING

    def reset(self):
        super(SuccessTimer, self).reset()
        self.count_start = False


class FailureTimer(Task):
    def __init__(self, name, timeout):
        super(FailureTimer, self).__init__(name)

        self._timeout = timeout
        self.start_time = rospy.get_time()
        self.count_start = False

    def run(self):
        if self.count_start == False:
            self.start_time = rospy.get_time()
            self.count_start = True

        if rospy.get_time() - self.start_time > self._timeout:
            return TaskStatus.FAILURE
        else:
            return TaskStatus.RUNNING

    def reset(self):
        super(FailureTimer, self).reset()
        self.count_start = False

