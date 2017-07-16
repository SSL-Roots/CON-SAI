# -*- coding: utf-8 -*-
import math

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from GlobalData import GlobalInfo
import Constants
import Tool
import Kick
import AnalystTool


class StopGame(Task):
    def __init__(self, name, number):
        super(StopGame, self).__init__(name)

        self._number = number
        self._dist = 0.5

    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.RUNNING

        ballPos = GlobalInfo.ball.pose.pose.position
        goalPos = Constants.GoalFriend

        angleGoalToBall = Tool.getAngle(goalPos, ballPos)

        x = -self._dist * math.cos(angleGoalToBall)
        y = -self._dist * math.sin(angleGoalToBall)
        yaw = angleGoalToBall

        GlobalInfo.controls[self._number].setTargetPose(x,y,yaw,"ball",True)
        GlobalInfo.controls[self._number].setKickVelocity(0.0)

        return TaskStatus.RUNNING


class ChipKickGuard(Task):
    def __init__(self, name, number):
        super(ChipKickGuard, self).__init__(name)

        self._number = number

    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.RUNNING

        ballPos = GlobalInfo.ball.pose.pose.position
        goalPos = Constants.GoalFriend

        angleGoalToBall = Tool.getAngle(goalPos, ballPos)

        x = -3.0
        y = 0.1
        yaw = angleGoalToBall

        GlobalInfo.controls[self._number].setTargetPose(x,y,yaw,"map")
        GlobalInfo.controls[self._number].setKickVelocity(0.0)

        return TaskStatus.RUNNING


class Offense(LoopInf):
    def __init__(self, name, number):
        super(Offense, self).__init__(name)

        SHOOT_PARA = ParallelOne("SHOOT_PARA")

        SHOOT_PARA.add_child(Kick.CalcuKickTarget("calcu", number))
        # SHOOT_PARA.add_child(Kick.CalcuOwnGoalTarget("calcu", number))

        SHOOT_PARA.add_child(Kick.Shoot("shoot", number))
        # SHOOT_PARA.add_child(Kick.SetplayShoot("SetplayShoot", number))

        self.add_child(SHOOT_PARA)


class UltimateOffense(Task):
    def __init__(self, name, number):
        super(UltimateOffense, self).__init__(name)
        
        self._number = number
        self.ballIsUpperside = True
        self.MARGIN = 0.25

    def run(self):

        ballPos = GlobalInfo.ball.pose.pose.position

        if ballPos.y > self.MARGIN:
            self.ballIsUpperside = True
        if ballPos.y < -self.MARGIN:
            self.ballIsUpperside = False


        x = -3.0

        y = 1.0
        if self.ballIsUpperside == False:
            y = -1.0

        yaw = 0

        GlobalInfo.controls[self._number] \
            .setTargetPose(x,y,yaw,"map",True)
        GlobalInfo.controls[self._number].setKickVelocity(0.0)

        return TaskStatus.RUNNING

class Defense(Task):
    def __init__(self, name, number):
        super(Defense, self).__init__(name)

        self._number = number
        self._distToBall = 0.5
        self._longDistToBall = 1.4

    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.RUNNING

        ballPos = GlobalInfo.ball.pose.pose.position
        goalPos = Constants.GoalFriend

        shootVel = 0.0
        avoidBall = True

        angleBallToGoal = Tool.getAngle(ballPos, goalPos)
        
        # ボールが敵ディフェンスエリアにあれば離れる
        dist = self._distToBall
        defensePos = ballPos
        if AnalystTool.isInDefenceArea(False, ballPos, 1.25):
            dist = self._longDistToBall

            trans = Tool.Trans(ballPos, angleBallToGoal)
            trDefensePos = Point(dist, 0, 0)
            defensePos = trans.invertedTransform(trDefensePos)
        else:
            target = "map"
            GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
            (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))
            robotPos = Point(point[0], point[1],0)

            trans = Tool.Trans(ballPos, angleBallToGoal)
            trRobotPos = trans.transform(robotPos)

            # 味方ゴールとボールの直線上にいれば、ボールに向かう
            if abs(trRobotPos.y) < 0.3:
                dist = 0
                shootVel = 8.0
                avoidBall = False
            trDefensePos = Point(dist, 0, 0)
            defensePos = trans.invertedTransform(trDefensePos)


        defensePos, canReceive = Tool.convertToReceivePos(defensePos)

        x = defensePos.x
        y = defensePos.y
        yaw = Tool.ivertAngle(angleBallToGoal)

        GlobalInfo.controls[self._number] \
            .setTargetPose(x,y,yaw,"map",avoidBall)

        if canReceive:
            shootVel = 8.0
        GlobalInfo.controls[self._number].setKickVelocity(shootVel)

        return TaskStatus.RUNNING

