from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import Kick
import Analyst
import GoalKeeper
import Attacker
import Defender

class Execute(LoopInf):
    def __init__(self, name):
        super(Execute, self).__init__(name)

        PLAY = Selector("PLAY")
        PLAY.add_child(Offense("Offense"))
        PLAY.add_child(UltimateOffense("UltimateOffense"))
        PLAY.add_child(DefenderOffense("DefenderOffense"))
        PLAY.add_child(Defense("Defense"))

        self.add_child(PLAY)

class Offense(ParallelAll):
    def __init__(self, name):
        super(Offense, self).__init__(name)

        IS_OFFENSIVE = Analyst.IsOffensive("IS_OFFENSIVE")

        EXE_SELECTOR = Selector("EXE_SELECTOR")

        ONE_ROBOT = ParallelAll("ONE_ROBOT")
        ONE_ROBOT.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        ONE_ROBOT.add_child(Attacker.Offense("AttackerOffense", 0))
        EXE_SELECTOR.add_child(ONE_ROBOT)


        SOME_ROBOTS = ParallelAll("SOME_ROBOTS")
        SOME_ROBOTS.add_child(Analyst.ThereAreRobots("ThereAreRobots"))
        SOME_ROBOTS.add_child(GoalKeeper.Defense("GoalKeeperDefense", 0))
        SOME_ROBOTS.add_child(Attacker.Offense("AttackerOffense", 1))
        SOME_ROBOTS.add_child(Defender.Defense("DefenderDefense", 2))
        EXE_SELECTOR.add_child(SOME_ROBOTS)

        self.add_child(IS_OFFENSIVE)
        self.add_child(EXE_SELECTOR)

class UltimateOffense(ParallelAll):
    def __init__(self, name):
        super(UltimateOffense, self).__init__(name)

        IS_ULTIMATE_OFFENSIVE = Analyst.IsUltimateOffensive("IS_ULTIMATE_OFFENSIVE")

        EXE_SELECTOR = Selector("EXE_SELECTOR")

        ONE_ROBOT = ParallelAll("ONE_ROBOT")
        ONE_ROBOT.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        ONE_ROBOT.add_child(GoalKeeper.UltimateOffense("GoalKeeperUltOffense", 0))
        EXE_SELECTOR.add_child(ONE_ROBOT)

        SOME_ROBOTS = ParallelAll("SOME_ROBOTS")
        SOME_ROBOTS.add_child(Analyst.ThereAreRobots("ThereAreRobots"))

        SOME_ROBOTS.add_child(GoalKeeper.UltimateOffense("GoalKeeperUltOffense", 0))
        SOME_ROBOTS.add_child(Attacker.UltimateOffense("AttackerUltOffense", 1))
        SOME_ROBOTS.add_child(Defender.UltimateOffense("DefenderUltOffense", 2))
        EXE_SELECTOR.add_child(SOME_ROBOTS)

        self.add_child(IS_ULTIMATE_OFFENSIVE)
        self.add_child(EXE_SELECTOR)


class DefenderOffense(ParallelAll):
    def __init__(self, name):
        super(DefenderOffense, self).__init__(name)

        IS_DEFENDER_OFFENSIVE = Analyst.IsDefenderOffensive("IS_DEFENDER_OFFENSIVE")

        EXE_SELECTOR = Selector("EXE_SELECTOR")

        ONE_ROBOT = ParallelAll("ONE_ROBOT")
        ONE_ROBOT.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        ONE_ROBOT.add_child(Defender.DefenderOffense("DefenderOffense", 0))
        EXE_SELECTOR.add_child(ONE_ROBOT)

        SOME_ROBOTS = ParallelAll("SOME_ROBOTS")
        SOME_ROBOTS.add_child(Analyst.ThereAreRobots("ThereAreRobots"))

        SOME_ROBOTS.add_child(GoalKeeper.Defense("GoalKeeperDefense", 0))
        SOME_ROBOTS.add_child(Attacker.Offense("AttackerOffense", 1))
        SOME_ROBOTS.add_child(Defender.DefenderOffense("DefenderOffense", 2))
        EXE_SELECTOR.add_child(SOME_ROBOTS)

        self.add_child(IS_DEFENDER_OFFENSIVE)
        self.add_child(EXE_SELECTOR)


class Defense(ParallelAll):
    def __init__(self, name):
        super(Defense, self).__init__(name)

        IS_DEFENSIVE = Analyst.IsDefensive("IS_DEFENSIVE")

        EXE_SELECTOR = Selector("EXE_SELECTOR")

        ONE_ROBOT = ParallelAll("ONE_ROBOT")
        ONE_ROBOT.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        ONE_ROBOT.add_child(Attacker.Defense("AttackerDefense", 0))
        EXE_SELECTOR.add_child(ONE_ROBOT)

        SOME_ROBOTS = ParallelAll("SOME_ROBOTS")
        SOME_ROBOTS.add_child(Analyst.ThereAreRobots("ThereAreRobots"))
        SOME_ROBOTS.add_child(GoalKeeper.Defense("GoalKeeperDefense",0))
        SOME_ROBOTS.add_child(Attacker.Defense("AttackerDefense",1))
        SOME_ROBOTS.add_child(Defender.Defense("DefenderDefense", 2))
        EXE_SELECTOR.add_child(SOME_ROBOTS)

        self.add_child(IS_DEFENSIVE)
        self.add_child(EXE_SELECTOR)
