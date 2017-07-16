from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsDIRECT_FREE_FRIEND
import BaseSkill
import Kick
import InPlay
import GoalKeeper
import Defender
import Analyst


class DirectFriend(ParallelAll):
    def __init__(self, name):
        super(DirectFriend , self).__init__(name)

        IS_DIRECT_FRIEND = IsDIRECT_FREE_FRIEND("Is direct friend")
        
        EXE_SELECTOR = Selector("EXE_SELECTOR")

        ONE_ROBOT = ParallelAll("ONE_ROBOT")
        ONE_ROBOT.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        ONE_ROBOT.add_child(OneRobotExecute("OneRobotExecute"))
        EXE_SELECTOR.add_child(ONE_ROBOT)

        SOME_ROBOTS = ParallelAll("SOME_ROBOTS")
        SOME_ROBOTS.add_child(Analyst.ThereAreRobots("ThereAreRobots"))
        SOME_ROBOTS.add_child(Execute("NormalExecute"))
        EXE_SELECTOR.add_child(SOME_ROBOTS)

        self.add_child(IS_DIRECT_FRIEND)
        self.add_child(EXE_SELECTOR)


class OneRobotExecute(Sequence):
    def __init__(self, name):
        super(OneRobotExecute, self).__init__(name)

        self.add_child(OneRobotDirectAction("OneRobotAction"))
        self.add_child(InPlay.Execute("Execute inplay"))


class OneRobotDirectAction(ParallelOne):
    def __init__(self, name):
        super(OneRobotDirectAction, self).__init__(name)

        limitTime = 7.0
        # limitTime = 12.0
        timerName = "SuccessTimer" + str(limitTime)
        TIMER = BaseSkill.SuccessTimer(timerName, limitTime)

        self.add_child(TIMER)

        self.add_child(Kick.CalcuKickTarget("CalcuKickTarget", 0))
        self.add_child(Kick.SetplayShoot("Shoot", 0))


class Execute(Sequence):
    def __init__(self, name):
        super(Execute, self).__init__(name)

        ACTION = DirectAction("DirectAction")
        INPLAY = InPlay.Execute("Execute inplay")

        self.add_child(ACTION)
        self.add_child(INPLAY)

class DirectAction(ParallelOne):
    def __init__(self, name):
        super(DirectAction, self).__init__(name)

        limitTime = 7.0
        # limitTime = 12.0
        timerName = "SuccessTimer" + str(limitTime)
        TIMER = BaseSkill.SuccessTimer(timerName, limitTime)

        CALCU = Kick.CalcuKickTarget("CalcuKickTarget", 1)
        SHOOT = Kick.SetplayShoot("Shoot", 1)

        self.add_child(TIMER)
        self.add_child(CALCU)
        self.add_child(SHOOT)
        
        self.add_child(GoalKeeper.Defense("GoalKeeperDefense", 0))
        self.add_child(Defender.Defense("DefenderDefense", 2))

