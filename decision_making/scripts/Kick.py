# -*- coding: utf-8 -*-

import rospy
import tf
import math
import random

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from geometry_msgs.msg import Point

from GlobalData import GlobalInfo
import Tool
import Analyst
import BaseSkill
import Constants
import AnalystTool


class CalcuKickTarget(Task):
    def __init__(self, name, number):
        super(CalcuKickTarget, self).__init__(name)
        self._number = number
        self._THRESH = 2.5

        self._kickTarget = Constants.PenaltyEnemy
        self.judgeTarget()


    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        self.judgeTarget()
        rospy.loginfo("Constx:%f, y:%f",Constants.GoalEnemy.x, Constants.GoalEnemy.y)

        if base is None:
            return TaskStatus.FAILURE

        GlobalInfo.controls[self._number].setKickTarget(self._kickTarget.x, self._kickTarget.y)

        return TaskStatus.RUNNING

    # def reset(self):
    #     super(CalcuKickTarget, self).reset()
    #     self.judgeTarget()

    def judgeTarget(self):
        ballPos = GlobalInfo.ball.pose.pose.position
        if AnalystTool.isInDefenceArea(False, ballPos, self._THRESH):
            self._kickTarget = Constants.GoalEnemy
        else:
            # self._kickTarget = Constants.PenaltyEnemy
            self._kickTarget = Constants.GoalEnemy





class CalcuOwnGoalTarget(CalcuKickTarget):
    def __init__(self, name, number):
        super(CalcuOwnGoalTarget, self).__init__(name, number)

        self._kickTarget = Constants.GoalFriend
        


class SetPassTarget(Task):
    def __init__(self, name, number, targetNumber):
        super(SetPassTarget, self).__init__(name)

        self._number = number
        self._targetNum = targetNumber

    def run(self):
        base = GlobalInfo.controls[self._targetNum].get_base()

        if base is None:
            return TaskStatus.FAILURE

        target = "map"
        GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
        (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))

        GlobalInfo.controls[self._number].setKickTarget(point[0], point[1])

        return TaskStatus.RUNNING


class Shoot(Sequence):
    def __init__(self, name, number):
        super(Shoot, self).__init__(name)

        self.APPROACH = OngameApproach("Approach", number)

        self.KICKING = ParallelOne("Kicking")
        self.KICKING.add_child(Analyst.BallMoved("BallMoved"))
        self.KICKING.add_child(GoKicking("GoKicking", number, 8.0))

        self.add_child(self.APPROACH)
        self.add_child(self.KICKING)


class SetplayShoot(Sequence):
    def __init__(self, name, number):
        super(SetplayShoot, self).__init__(name)

        self.APPROACH = Approach("Approach", number, 0.1)

        self.KICKING = ParallelOne("Kicking")
        self.KICKING.add_child(Analyst.BallMoved("BallMoved"))
        self.KICKING.add_child(GoKicking("GoKicking", number, 8.0))
        # self.KICKING.add_child(SetplayGoKicking("SetplayGoKicking", number, 8.0))

        self.add_child(self.APPROACH)
        self.add_child(self.KICKING)

class OngameShoot(Sequence):
    def __init__(self, name, number):
        super(OngameShoot, self).__init__(name)

        GO_TO_BALL = ParallelOne("GO_TO_BALL")
        GO_TO_BALL.add_child(OngameApproach("OngameApproach",number))
        GO_TO_BALL.add_child(Analyst.HasBall("HasBall",0))

        KICKING = ParallelOne("OngameKicking")
        KICKING.add_child(Analyst.BallMoved("BallMoved",True))
        KICKING.add_child(OngameGoKicking("OngameGoKicking", number, 8.0))

        self.add_child(GO_TO_BALL)
        self.add_child(KICKING)

class ReceiveShoot(Task):
    def __init__(self, name, number):
        super(ReceiveShoot, self).__init__(name)

        self._number = number

    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.FAILURE

        # target = "map"
        # GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
        # (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))
        #
        # robotPos = Point(point[0],point[1],0)
        target = "map"
        GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
        (myPoint,myOrientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))
        myYaw = Tool.yawFromTfQuaternion(myOrientation)
        
        poseStamped = GlobalInfo.tf_listener \
            .transformPose("map",GlobalInfo.controls[self._number].target_pose)
        targetPoint = poseStamped.pose.position

        shootTarget = GlobalInfo.controls[self._number].getKickTarget()

        targetPos, yaw, canShoot = Tool.convertToReceiveShootPose(targetPoint, shootTarget)
        # targetPos, yaw, canShoot = Tool.convertToReceiveShootPose(robotPos, shootTarget)


        x = targetPos.x
        y = targetPos.y
        GlobalInfo.controls[self._number].setTargetPose(x,y,yaw,"map")

        shootVel = 0.0
        if canShoot:
            shootVel = 8.0
        GlobalInfo.controls[self._number].setKickVelocity(shootVel)

        return TaskStatus.RUNNING


class Pass(Sequence):
    def __init__(self, name, number):
        super(Pass, self).__init__(name)

        self.APPROACH = Approach("Approach", number, 0.5)

        self.KICKING = ParallelOne("Kicking")
        self.KICKING.add_child(GoKicking("GoKicking", number, 6.0))
        # self.KICKING.add_child(SetplayGoKicking("GoKicking", number, 6.0))
        self.KICKING.add_child(Analyst.BallMoved("BallMoved"))

        self.add_child(self.APPROACH)
        self.add_child(self.KICKING)


class Approach(ParallelOne):
    def __init__(self, name, number, timeout=0.5):
        super(Approach, self).__init__(name)

        self.CALC = CalcuApproachPosition("CalcApproachPos", number)
        self.IS_ON_TARGET = Analyst.IsOnTarget("Is On Target",number, timeout)

        self.add_child(self.CALC)
        self.add_child(self.IS_ON_TARGET)

class OngameApproach(ParallelOne):
    def __init__(self, name, number):
        super(OngameApproach, self).__init__(name)


        CALC = CalcuApproachPosition("CalcApproachPos", number)
        IS_BACKSIDE = Analyst.IsBackSide("Is BackSide", number)

        self.add_child(CALC)
        self.add_child(IS_BACKSIDE)
        

class CalcuApproachPosition(Task):
    def __init__(self, name, number):
        super(CalcuApproachPosition, self).__init__(name)

        self._number = number
        self._approachDist = 0.3
        self._threshAngle = 80.0 * math.pi / 180.0

    def run(self):
        
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.FAILURE

        target = "map"
        GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
        (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))

        robotPos = Point(point[0],point[1],0)
        ballPos = GlobalInfo.ball.pose.pose.position
        kickTarget = GlobalInfo.controls[self._number].getKickTarget()
        angleToTarget = Tool.getAngle(ballPos, kickTarget)

        trans = Tool.Trans(ballPos,angleToTarget)
        trRobotPos = trans.transform(robotPos)
        trRobotPosAngle = Tool.getAngleFromCenter(trRobotPos)
        distToRobot = Tool.getSizeFromCenter(trRobotPos)

        # Targetに対して、Ballの裏側に回ったら、Approach位置を近づける
        approachDist = self._approachDist
        if abs(trRobotPosAngle) > self._threshAngle \
                and distToRobot < self._approachDist:
            approachDist = distToRobot

        trApproachPos = Point(-approachDist,0.0,0)
        approachPos = trans.invertedTransform(trApproachPos)
        
        x,y = approachPos.x, approachPos.y

        GlobalInfo.controls[self._number].setTargetPose(x,y,angleToTarget,"map",True)
        GlobalInfo.controls[self._number].setKickVelocity(0.0)

        return TaskStatus.RUNNING

class GoKicking(Task):
    def __init__(self, name, number, velocity):
        super(GoKicking, self).__init__(name)

        self._number = number
        self._velocity = velocity

    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.FAILURE

        ballPos = GlobalInfo.ball.pose.pose.position
        kickTarget = GlobalInfo.controls[self._number].getKickTarget()
        angleToTarget = Tool.getAngle(ballPos, kickTarget)

        x = ballPos.x
        y = ballPos.y

        GlobalInfo.controls[self._number].setTargetPose(x,y,angleToTarget,"map")
        GlobalInfo.controls[self._number].setKickVelocity(self._velocity)

        return TaskStatus.RUNNING


class SetplayGoKicking(Task):
    def __init__(self, name, number, kickVelocity):
        super(SetplayGoKicking, self).__init__(name)

        self._number = number
        self._kickVelocity = kickVelocity

    def run(self):

        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.FAILURE

        GlobalInfo.controls[self._number].setTargetVelocity(0.5, 0, 0)
        GlobalInfo.controls[self._number].setKickVelocity(self._kickVelocity)

        return TaskStatus.RUNNING

class OngameGoKicking(Task):
    def __init__(self, name, number, velocity):
        super(OngameGoKicking, self).__init__(name)

        self._number = number
        self._velocity = velocity
        self._threshKickAngle = 5.0 * math.pi / 180.0

    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.FAILURE

        target = "map"
        GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
        (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))

        robotYaw = Tool.yawFromTfQuaternion(orientation)
        ballPos = GlobalInfo.ball.pose.pose.position
        kickTarget = GlobalInfo.controls[self._number].getKickTarget()

        angleToTarget = Tool.getAngle(ballPos, kickTarget)
        x,y = ballPos.x, ballPos.y

        diffAngle = Tool.normalize(angleToTarget - robotYaw)
        kickVelocity = 0.0
        if abs(diffAngle) < self._threshKickAngle:
            kickVelocity = self._velocity

        GlobalInfo.controls[self._number].setTargetPose(x,y,angleToTarget,"map")
        GlobalInfo.controls[self._number].setKickVelocity(kickVelocity)

        return TaskStatus.RUNNING

