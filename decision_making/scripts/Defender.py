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
        self._calcu = Calculator()
        self._THRESH_OFFENSE = 0.5
        self._THRESH_OFFENSE_DEFAULT = 0.4
        self._THRESH_OFFENSE_MARGIN = 0.3
        self._MARGIN = 0.5
        self.ballIsUpperSide = True
        
    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.RUNNING

        ballPos = GlobalInfo.ball.pose.pose.position

        defensePos = Point()
        if ballPos.x > self._THRESH_OFFENSE:
            if ballPos.y > self._MARGIN:
                self.ballIsUpperSide = True
            elif ballPos.y < -self._MARGIN:
                self.ballIsUpperSide = False

            posY = 2.0
            if self.ballIsUpperSide == False:
                posY *= -1.0


            defensePos = Point(0.0, posY, 0.0)
            self._THRESH_OFFENSE = self._THRESH_OFFENSE_MARGIN
        else:
            defensePos = self._calcu.calcuDefensePos()
            self._THRESH_OFFENSE = self._THRESH_OFFENSE_DEFAULT

        angleToBall = Tool.getAngle(defensePos, ballPos)

        x = defensePos.x
        y = defensePos.y
        yaw = angleToBall

        GlobalInfo.controls[self._number].setTargetPose(x,y,yaw,"map",True)
        GlobalInfo.controls[self._number].setKickVelocity(0.0)

        return TaskStatus.RUNNING

class Defense(Task):
    def __init__(self, name, number):
        super(Defense, self).__init__(name)

        self._number = number
        self._calcu = Calculator()
        
    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.RUNNING

        ballPos = GlobalInfo.ball.pose.pose.position
        defensePos = self._calcu.calcuDefensePos()

        defensePos, canReceive = Tool.convertToReceivePos(defensePos)

        angleToBall = Tool.getAngle(defensePos, ballPos)

        x = defensePos.x
        y = defensePos.y
        yaw = angleToBall

        GlobalInfo.controls[self._number].setTargetPose(x,y,yaw,"map",True)

        shootVel = 0.0
        if canReceive:
            shootVel = 8.0
        GlobalInfo.controls[self._number].setKickVelocity(shootVel)

        return TaskStatus.RUNNING


class SetplayDefense(Task):
    def __init__(self, name, number):
        super(SetplayDefense, self).__init__(name)

        self._number = number
        self._calcu = Calculator()
        
    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.RUNNING

        ballPos = GlobalInfo.ball.pose.pose.position
        defensePos = self._calcu.calcuSetplayDefensePos()

        defensePos, canReceive = Tool.convertToReceivePos(defensePos)

        angleToBall = Tool.getAngle(defensePos, ballPos)

        x = defensePos.x
        y = defensePos.y
        yaw = -angleToBall

        GlobalInfo.controls[self._number].setTargetPose(x,y,yaw,"map",True)

        shootVel = 0.0
        if canReceive:
            shootVel = 8.0
        GlobalInfo.controls[self._number].setKickVelocity(shootVel)

        return TaskStatus.RUNNING



class UltimateOffense(Task):
    def __init__(self, name, number):
        super(UltimateOffense, self).__init__(name)

        self._number = number
        self._calcu = Calculator()
        
    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.RUNNING

        ballPos = GlobalInfo.ball.pose.pose.position
        defensePos = self._calcu.calcuDefensePos()
        defensePos = self._calcu.calcuUltimateOffensePos()

        defensePos, canReceive = Tool.convertToReceivePos(defensePos)

        x = defensePos.x
        y = defensePos.y
        yaw = 0.0

        GlobalInfo.controls[self._number].setTargetPose(x,y,yaw,"map",True)

        shootVel = 0.0
        if canReceive:
            shootVel  = 8.0
        GlobalInfo.controls[self._number].setKickVelocity(shootVel)

        return TaskStatus.RUNNING


class DefenderOffense(LoopInf):
    def __init__(self, name, number):
        super(DefenderOffense, self).__init__(name)

        SHOOT_PARA = ParallelOne("SHOOT_PARA")
        SHOOT_PARA.add_child(Kick.CalcuKickTarget("calcu", number))
        SHOOT_PARA.add_child(Kick.Shoot("shoot", number))

        self.add_child(SHOOT_PARA)


class Calculator():
    def __init__(self):

        self.ballIsUpperSide = True
        self._ANGLE_MARGIN = 15.0 * math.pi / 180.0
        self._ANGLE_TO_DEFEND = 25.0 * math.pi / 180.0
        self._DIST_TO_DEFEND = 1.4
        self._ANGLE_TO_ULT_OFFENSE = 30.0 * math.pi / 180.0
        self._ANGLE_LIMIT_TO_SETPLY = 70.0 * math.pi / 180.0

    def calcuDefensePos(self):
        ballPos = GlobalInfo.ball.pose.pose.position
        goalPos = Constants.GoalFriend
        
        angleGoalToBall = Tool.getAngle(goalPos, ballPos)
        self.judgeBallSide(angleGoalToBall)

        defenseAngle = angleGoalToBall
        if self.ballIsUpperSide == True:
            defenseAngle -= self._ANGLE_TO_DEFEND
        else:
            defenseAngle += self._ANGLE_TO_DEFEND

        defensePos = Point()
        defensePos.x = self._DIST_TO_DEFEND * math.cos(defenseAngle) + goalPos.x
        defensePos.y = self._DIST_TO_DEFEND * math.sin(defenseAngle) + goalPos.y

        return defensePos

    def calcuSetplayDefensePos(self):
        ballPos = GlobalInfo.ball.pose.pose.position
        goalPos = Constants.GoalFriend
        
        angleGoalToBall = Tool.getAngle(goalPos, ballPos)

        if angleGoalToBall > self._ANGLE_LIMIT_TO_SETPLY:
            angleGoalToBall = self._ANGLE_LIMIT_TO_SETPLY
        elif angleGoalToBall < -self._ANGLE_LIMIT_TO_SETPLY:
            angleGoalToBall = -self._ANGLE_LIMIT_TO_SETPLY

        defensePos = Point()
        defensePos.x = self._DIST_TO_DEFEND * math.cos(-angleGoalToBall) + goalPos.x
        defensePos.y = self._DIST_TO_DEFEND * math.sin(-angleGoalToBall) + goalPos.y
        
        return defensePos


    def calcuUltimateOffensePos(self):
        ballPos = GlobalInfo.ball.pose.pose.position
        goalPos = Constants.GoalFriend
        
        angleGoalToBall = Tool.getAngle(goalPos, ballPos)
        self.judgeBallSide(angleGoalToBall)

        defenseAngle = angleGoalToBall
        if abs(angleGoalToBall) > self._ANGLE_TO_ULT_OFFENSE:
            defenseAngle = 0.0
        else:
            if self.ballIsUpperSide == True:
                defenseAngle -= self._ANGLE_TO_DEFEND
            else:
                defenseAngle += self._ANGLE_TO_DEFEND

        defensePos = Point()
        defensePos.x = self._DIST_TO_DEFEND * math.cos(defenseAngle) + goalPos.x
        defensePos.y = self._DIST_TO_DEFEND * math.sin(defenseAngle) + goalPos.y

        return defensePos


    def judgeBallSide(self, angleGoalToBall):
        if self.ballIsUpperSide == True:
            if angleGoalToBall < -self._ANGLE_MARGIN:
                self.ballIsUpperSide = False
        else:
            if angleGoalToBall > self._ANGLE_MARGIN:
                self.ballIsUpperSide = True

