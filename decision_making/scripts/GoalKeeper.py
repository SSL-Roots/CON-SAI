import math

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from GlobalData import GlobalInfo
import Constants
import Tool
import Kick


class StopGame(Task):
    def __init__(self, name, number):
        super(StopGame, self).__init__(name)

        self._number = number
        self._calcu = Calculator()

    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.FAILURE

        ballPos = GlobalInfo.ball.pose.pose.position
        goalPos = Constants.GoalFriend

        angleGoalToBall = Tool.getAngle(goalPos, ballPos)
        defensePos = self._calcu.calcuDefensePos(goalPos, angleGoalToBall)

        x = defensePos.x
        y = defensePos.y
        yaw = angleGoalToBall

        GlobalInfo.controls[self._number].setTargetPose(x,y,yaw,"map",True)
        GlobalInfo.controls[self._number].setKickVelocity(0.0)

        return TaskStatus.RUNNING

class UltimateOffense(LoopInf):
    def __init__(self, name, number):
        super(UltimateOffense, self).__init__(name)

        SHOOT_PARA = ParallelOne("SHOOT_PARA")
        SHOOT_PARA.add_child(Kick.CalcuKickTarget("calcu", number))
        SHOOT_PARA.add_child(Kick.Shoot("shoot", number))

        self.add_child(SHOOT_PARA)

class Defense(Task):
    def __init__(self, name, number):
        super(Defense, self).__init__(name)

        self._number = number
        self._calcu = Calculator()


    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.FAILURE

        ballPos = GlobalInfo.ball.pose.pose.position
        goalPos = Constants.GoalFriend

        angleGoalToBall = Tool.getAngle(goalPos, ballPos)
        defensePos = self._calcu.calcuDefensePos(goalPos,angleGoalToBall)
        
        defensePos, canReceive = Tool.convertToReceivePos(defensePos)

        x = defensePos.x
        y = defensePos.y
        yaw = Tool.getAngle(defensePos, ballPos)

        GlobalInfo.controls[self._number].setTargetPose(x,y,yaw,"map",True)

        shootVel = 0.0
        dribblePower = 0.0
        if canReceive:
            shootVel = 8.0
            dribblePower = 8.0
        GlobalInfo.controls[self._number].setKickVelocity(shootVel, dribblePower)

        return TaskStatus.RUNNING


class SetplayDefense(Task):
    def __init__(self, name, number):
        super(SetplayDefense, self).__init__(name)

        self._number = number
        self._calcu = Calculator()


    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.FAILURE

        ballPos = GlobalInfo.ball.pose.pose.position
        goalPos = Constants.GoalFriend

        angleGoalToBall = Tool.getAngle(goalPos, ballPos)
        angleGoalToBall *= -1.0
        defensePos = self._calcu.calcuDefensePos(goalPos,angleGoalToBall)
        
        defensePos, canReceive = Tool.convertToReceivePos(defensePos)

        x = defensePos.x
        y = defensePos.y
        yaw = -Tool.getAngle(defensePos, ballPos)

        GlobalInfo.controls[self._number].setTargetPose(x,y,yaw,"map",True)

        shootVel = 0.0
        dribblePower = 0.0
        if canReceive:
            shootVel = 8.0
            dribblePower = 8.0
        GlobalInfo.controls[self._number].setKickVelocity(shootVel, dribblePower)

        return TaskStatus.RUNNING

class PenaltyDefense(Task):
    def __init__(self, name, number):
        super(PenaltyDefense, self).__init__(name)

        self._number = number

    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.FAILURE
        
        nearestID = GlobalInfo.nearestEnemyID
        if nearestID is None:
            return TaskStatus.FAILURE

        enemyBase = Tool.getEnemyBase(nearestID)
        target = "map"
        GlobalInfo.tf_listener.waitForTransform(target,enemyBase,rospy.Time(0),rospy.Duration(3.0))
        (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,enemyBase,rospy.Time(0))

        yaw = Tool.yawFromTfQuaternion(orientation)

        x = Constants.GoalFriend.x + 0.05
        y = math.sin(yaw)
        yLimit = Constants.GoalHalfSize - (Constants.RobotRadius + 0.05)
        if y > yLimit:
            y = yLimit 
        elif y < -yLimit:
            y = -yLimit

        targetYaw = 0

        GlobalInfo.controls[self._number].setTargetPose(x,y,targetYaw,"map")
        GlobalInfo.controls[self._number].setKickVelocity(0.0)

        return TaskStatus.RUNNING


class Calculator():
    def __init__(self):
        self._dist = 0.7
        self._angleLimit = 80.0 * math.pi / 180.0

    def calcuDefensePos(self, goalPos, angleGoalToBall):
        
        if angleGoalToBall > self._angleLimit:
            angleGoalToBall = self._angleLimit

        if angleGoalToBall < -self._angleLimit:
            angleGoalToBall = -self._angleLimit

        trans = Tool.Trans(goalPos, angleGoalToBall)
        trDefensePos = Point(self._dist,0.0,0)
        defensePos = trans.invertedTransform(trDefensePos)

        return defensePos


