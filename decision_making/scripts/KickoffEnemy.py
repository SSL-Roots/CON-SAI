from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsKICKOFF_ENEMY, IsNORMAL_START
import GoalKeeper
import Analyst
import InPlay
import Attacker
import Defender


class KickoffEnemy(Selector):
    def __init__(self, name):
        super(KickoffEnemy , self).__init__(name)

        self.prepare    = Prepare('Prepare enemy kickoff')
        self.execute    = Execute('Execute enemy kickoff')

        self.add_child(self.prepare)
        self.add_child(self.execute)

class Prepare(ParallelAll):
    def __init__(self, name):
        super(Prepare, self).__init__(name)


        EXE_SELECTOR = Selector("EXE_SELECTOR")

        ONE_ROBOT = ParallelAll("ONE_ROBOT")
        ONE_ROBOT.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        ONE_ROBOT.add_child(Attacker.StopGame("AttackerStop", 0))
        EXE_SELECTOR.add_child(ONE_ROBOT)

        SOME_ROBOTS = ParallelAll("SOME_ROBOTS")
        SOME_ROBOTS.add_child(Analyst.ThereAreRobots("ThereAreRobots"))
        SOME_ROBOTS.add_child(GoalKeeper.Defense("GoalKeeperDefense", 0))
        # SOME_ROBOTS.add_child(Attacker.StopGame("AttackerStop", 1))
        SOME_ROBOTS.add_child(Attacker.ChipKickGuard("ChipKickGuard", 1))
        SOME_ROBOTS.add_child(Defender.SetplayDefense("DefenderDefense", 2))
        EXE_SELECTOR.add_child(SOME_ROBOTS)

        self.add_child(IsKICKOFF_ENEMY("IsKICKOFF_ENEMY"))
        self.add_child(EXE_SELECTOR)

class Execute(ParallelAll):
    def __init__(self, name):
        super(Execute, self).__init__(name)

        IS_NORMAL_START = IsNORMAL_START('is_normal_start')
        INPLAY_SWITCH = InplaySwitch("InplaySwitch")

        self.add_child(IS_NORMAL_START)
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
        SOME_ROBOTS.add_child(GoalKeeper.Defense("GoalKeeperDefense",0))
        # SOME_ROBOTS.add_child(Attacker.StopGame("AttackerStop", 1))
        SOME_ROBOTS.add_child(Attacker.ChipKickGuard("ChipKickGuard", 1))
        SOME_ROBOTS.add_child(Defender.Defense("DefenderDefense", 2))
        EXE_SELECTOR.add_child(SOME_ROBOTS)

        INPLAY_DEFEND.add_child(EXE_SELECTOR)

        INPLAY = InPlay.Execute("Execute inplay")

        self.add_child(INPLAY_DEFEND)
        self.add_child(INPLAY)

