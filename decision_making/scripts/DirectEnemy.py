from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsDIRECT_FREE_ENEMY
import GoalKeeper
import Attacker
import Defender
import Analyst
import InPlay

class DirectEnemy(ParallelAll):
    def __init__(self, name):
        super(DirectEnemy , self).__init__(name)

        IS_DIRECT_ENEMY = IsDIRECT_FREE_ENEMY('Is Direct Enemy')
        INPLAY_SWITCH = InplaySwitch("InplaySwitch")

        self.add_child(IS_DIRECT_ENEMY)
        self.add_child(INPLAY_SWITCH)


class InplaySwitch(Sequence):
    def __init__(self, name):
        super(InplaySwitch, self).__init__(name)

        INPLAY_DEFEND = ParallelOne("PARALLEL_DEFEND")
        INPLAY_DEFEND.add_child(Analyst.BallMoved("Ball moved"))

        EXE_SELECTOR = Selector("EXE_SELECTOR")

        ONE_ROBOT = ParallelAll("ONE_ROBOT")
        ONE_ROBOT.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        ONE_ROBOT.add_child(Attacker.StopGame("AttackerStop", 0))
        EXE_SELECTOR.add_child(ONE_ROBOT)

        SOME_ROBOTS = ParallelAll("SOME_ROBOTS")
        SOME_ROBOTS.add_child(Analyst.ThereAreRobots("ThereAreRobots"))
        SOME_ROBOTS.add_child(GoalKeeper.SetplayDefense("GoalKeeperDefense", 0))
        SOME_ROBOTS.add_child(Attacker.StopGame("AttackerStop", 1))
        SOME_ROBOTS.add_child(Defender.SetplayDefense("DefenderDefense", 2))
        EXE_SELECTOR.add_child(SOME_ROBOTS)

        INPLAY_DEFEND.add_child(EXE_SELECTOR)
        INPLAY = InPlay.Execute("Execute inplay")

        self.add_child(INPLAY_DEFEND)
        self.add_child(INPLAY)

