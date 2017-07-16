import rospy
import tf
import math
import random

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import sys,os
sys.path.append(os.pardir)
from GlobalData import GlobalInfo
import Tool
import BaseSkill
import Analyst
import Kick

class HaltPattern(Task):
    def __init__(self, name):
        super(HaltPattern, self).__init__(name)

    def run(self):
        for control in GlobalInfo.controls:
            control.setTargetVelocity(0.0, 0.0, 0.0)

        return TaskStatus.RUNNING

class VelocityTest(Task):
    def __init__(self, name):
        super(VelocityTest, self).__init__(name)

        self.starttime = rospy.get_time()

        self.speed = 0

    def run(self):

        diffTime = rospy.get_time() - self.starttime

        if diffTime < 3.0:
            self.speed = 1.0
        elif diffTime < 6.0:
            self.speed = 0.0
        else:
            self.starttime = rospy.get_time()


        x = self.speed
        y = 0
        yaw = 0
        GlobalInfo.controls[0].setTargetVelocity(x, y, yaw)

        return TaskStatus.RUNNING


            
class BackandForth(Sequence):
    def __init__(self, name):
        super(BackandForth, self).__init__(name)

        FORTH = ParallelAll("FORTH")
        BACK = ParallelAll("BACK")

        for number in xrange(6):
            x = -3 + 0.5*number
            y = 2.5
            angle = 90 * math.pi / 180.0

            name = "forth_" + str(number)
            forth = GoToPosition(name,number,x,y,angle)
            FORTH.add_child(forth)

            y *= -1
            name = "back_" + str(number)
            back = GoToPosition(name,number,x,y,angle)
            BACK.add_child(back)

        self.add_child(FORTH)
        self.add_child(BACK)

class GoToRandom(Sequence):
    def __init__(self, name):
        super(GoToRandom, self).__init__(name)

        for repeat in xrange(20):
            nodeName = "RANDOM" + str(repeat)
            RANDOM = ParallelAll(nodeName)

            for number in xrange(6):
                x = random.uniform(0,4.3)
                y = random.uniform(-3.0, 3.0)
                angle = random.uniform(-math.pi, math.pi)

                name = "random1_" + str(number)
                r = GoToPosition(name,number,x,y,angle)
                RANDOM.add_child(r)

            self.add_child(RANDOM)

class GoToPosition(ParallelOne):
    def __init__(self, name, number, x, y, yaw):
        super(GoToPosition, self).__init__(name)

        self.STAY = BaseSkill.Stay("Stay", number, x, y, yaw)
        self.IS_ON_TARGET = Analyst.IsOnTarget("IsOnTarget", number, 0.5)

        self.add_child(self.STAY)
        self.add_child(self.IS_ON_TARGET)

class PassAndPass(ParallelAll):
    def __init__(self, name, num1, num2):
        super(PassAndPass, self).__init__(name)

        wait1X, wait1Y, wait1Yaw = (-2.0, 2.0, -math.pi*0.5)
        wait2X, wait2Y, wait2Yaw = (-2.0, -2.0, math.pi*0.5)

        PLAYER1 = Sequence("player1")
        wait_ = ParallelOne("wait1")
        wait_.add_child(Analyst.BallEnteringQuadrant("ballEntering",2))
        wait_.add_child(BaseSkill.Stay("stay1", num1, wait1X, wait1Y, wait1Yaw))

        pass_ = ParallelOne("pass1")
        pass_.add_child(Kick.SetPassTarget("setTarget1",num1, num2))
        pass_.add_child(Kick.Pass("GoPassing1", num1))
        pass_.add_child(Analyst.BallEnteringQuadrant("ballEntering",3))

        PLAYER1.add_child(wait_)
        PLAYER1.add_child(pass_)
        PLAYER1.add_child(GoToPosition("goto1",num1,wait1X,wait1Y,wait1Yaw))
        
        self.add_child(PLAYER1)

        PLAYER2 = Sequence("player2")
        wait_ = ParallelOne("wait2")
        wait_.add_child(Analyst.BallEnteringQuadrant("ballEntering",3))
        wait_.add_child(BaseSkill.Stay("stay2", num2, wait2X, wait2Y, wait2Yaw))

        pass_ = ParallelOne("pass2")
        pass_.add_child(Kick.SetPassTarget("setTarget2",num2, num1))
        pass_.add_child(Kick.Pass("GoPassing2", num2))
        pass_.add_child(Analyst.BallEnteringQuadrant("ballEntering",2))

        PLAYER2.add_child(wait_)
        PLAYER2.add_child(pass_)
        PLAYER2.add_child(GoToPosition("goto2",num2,wait2X,wait2Y,wait2Yaw))
        
        self.add_child(PLAYER2)







