from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsINDIRECT_FREE_FRIEND
import BaseSkill
import Kick
import InPlay
import GoalKeeper
import Defender
import ComboSkill
import Analyst


class IndirectFriend(ParallelAll):
    def __init__(self, name):
        super(IndirectFriend , self).__init__(name)

        IS_INDIRECT_FRIEND = IsINDIRECT_FREE_FRIEND("Is indirect friend")

        EXE_SELECTOR = Selector("EXE_SELECTOR")

        ONE_ROBOT = ParallelAll("ONE_ROBOT")
        ONE_ROBOT.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        ONE_ROBOT.add_child(OneRobotExecute("OneRobotExecute"))
        EXE_SELECTOR.add_child(ONE_ROBOT)


        SOME_ROBOTS = ParallelAll("SOME_ROBOTS")
        SOME_ROBOTS.add_child(Analyst.ThereAreRobots("ThereAreRobots"))
        SOME_ROBOTS.add_child(Execute("NormalExecute"))
        EXE_SELECTOR.add_child(SOME_ROBOTS)

        self.add_child(IS_INDIRECT_FRIEND)
        self.add_child(EXE_SELECTOR)

class OneRobotExecute(Sequence):
    def __init__(self, name):
        super(OneRobotExecute, self).__init__(name)

        self.add_child(OneRobotIndirectAction("OneRobotAction"))
        self.add_child(InPlay.Execute("Execute inplay"))


class OneRobotIndirectAction(ParallelOne):
    def __init__(self, name):
        super(OneRobotIndirectAction, self).__init__(name)

        # limitTime = 12.0
        limitTime = 7.0
        timerName = "SuccessTimer" + str(limitTime)
        TIMER = BaseSkill.SuccessTimer(timerName, limitTime)

        DO_SHOOT = ParallelOne("DO_SHOOT")
        DO_SHOOT.add_child(Kick.CalcuKickTarget("CalcuKickTarget", 0))
        DO_SHOOT.add_child(Kick.SetplayShoot("Shoot", 0))

        self.add_child(TIMER)
        self.add_child(DO_SHOOT)

class Execute(Sequence):
    def __init__(self, name):
        super(Execute, self).__init__(name)

        ACTION = IndirectAction("IndirectAction")
        INPLAY = InPlay.Execute("Execute inplay")

        self.add_child(ACTION)
        self.add_child(INPLAY)


class IndirectAction(ParallelOne):
    def __init__(self, name):
        super(IndirectAction, self).__init__(name)

        limitTime = 30.0
        # limitTime = 7.0
        timerName = "SuccessTimer" + str(limitTime)
        TIMER = BaseSkill.SuccessTimer(timerName, limitTime)

        ACTION_SELECTOR = Selector("ACTION_SELECTOR")
        DO_PASS_SHOOT = Sequence("DO_PASS_SHOOT")
        DO_PASS_SHOOT.add_child(Analyst.BallIsEnemySide("BallIsEnemySide"))
        DO_PASS_SHOOT.add_child(Analyst.ThereAreThreeRobots("ThreeRobots"))
        DO_PASS_SHOOT.add_child(ComboSkill.PassShoot("PassShoot", 2, 1))

        ACTION_SELECTOR.add_child(DO_PASS_SHOOT)

        DO_SHOOT = ParallelOne("DO_SHOOT")
        DO_SHOOT.add_child(Kick.CalcuKickTarget("CalcuKickTarget", 1))
        DO_SHOOT.add_child(Kick.SetplayShoot("Shoot", 1))
        DO_SHOOT.add_child(Defender.Defense("DefenderDefense", 2))

        ACTION_SELECTOR.add_child(DO_SHOOT)


        self.add_child(TIMER)
        self.add_child(ACTION_SELECTOR)
        self.add_child(GoalKeeper.Defense("GoalKeeperDefense", 0))

