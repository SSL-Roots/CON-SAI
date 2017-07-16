from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsPENALTY_FRIEND, IsNORMAL_START
import Kick
import GoalKeeper
import Defender
import Analyst


class PenaltyFriend(Selector):
    def __init__(self, name):
        super(PenaltyFriend , self).__init__(name)

        self.prepare    = Prepare('Prepare friend penalty kick')
        self.execute    = Execute('Execute friend penalty kick')

        # add parent tasks
        self.add_child(self.prepare)
        self.add_child(self.execute)

class Prepare(ParallelAll):
    def __init__(self, name):
        super(Prepare, self).__init__(name)

        EXE_SELECTOR = Selector("EXE_SELECTOR")

        ONE_ROBOT = ParallelAll("ONE_ROBOT")
        ONE_ROBOT.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        ONE_ROBOT.add_child(Kick.CalcuKickTarget("CalcuKickTarget", 0))
        ONE_ROBOT.add_child(Kick.Approach("Approach", 0))
        EXE_SELECTOR.add_child(ONE_ROBOT)

        SOME_ROBOTS = ParallelAll("SOME_ROBOTS")
        SOME_ROBOTS.add_child(Analyst.ThereAreRobots("ThereAreRobots"))
        SOME_ROBOTS.add_child(GoalKeeper.StopGame("GoalKeeperStop", 0))
        SOME_ROBOTS.add_child(Kick.CalcuKickTarget("CalcuKickTarget", 1))
        SOME_ROBOTS.add_child(Kick.Approach("Approach", 1))
        SOME_ROBOTS.add_child(Defender.StopGame("DefenderStop", 2))
        EXE_SELECTOR.add_child(SOME_ROBOTS)

        self.add_child(IsPENALTY_FRIEND("IsPENALTY_FRIEND"))
        self.add_child(EXE_SELECTOR)


class Execute(ParallelAll):
    def __init__(self, name):
        super(Execute, self).__init__(name)

        EXE_SELECTOR = Selector("EXE_SELECTOR")

        ONE_ROBOT = ParallelAll("ONE_ROBOT")
        ONE_ROBOT.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        ONE_ROBOT.add_child(Kick.CalcuKickTarget("CalcuKickTarget", 0))
        ONE_ROBOT.add_child(Kick.SetplayShoot("Shoot", 0))
        EXE_SELECTOR.add_child(ONE_ROBOT)

        SOME_ROBOTS = ParallelAll("SOME_ROBOTS")
        SOME_ROBOTS.add_child(Analyst.ThereAreRobots("ThereAreRobots"))
        SOME_ROBOTS.add_child(GoalKeeper.StopGame("GoalKeeperStop", 0))
        SOME_ROBOTS.add_child(Kick.CalcuKickTarget("CalcuKickTarget", 1))
        SOME_ROBOTS.add_child(Kick.SetplayShoot("Shoot", 1))
        SOME_ROBOTS.add_child(Defender.StopGame("DefenderStop", 2))
        EXE_SELECTOR.add_child(SOME_ROBOTS)


        self.add_child(IsNORMAL_START("IsNORMAL_START"))
        self.add_child(EXE_SELECTOR)

