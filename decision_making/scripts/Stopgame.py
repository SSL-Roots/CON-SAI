from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsSTOP, IsGOAL_FRIEND, IsGOAL_ENEMY
import GoalKeeper
import Attacker
import Defender
import Analyst


class Stopgame(ParallelAll):
    def __init__(self, name):
        super(Stopgame , self).__init__(name)

        REFEREE = ParallelOne("RefCommandAsStop")
        REFEREE.add_child(IsSTOP("Is stop"))
        REFEREE.add_child(IsGOAL_FRIEND("Is goal friend"))
        REFEREE.add_child(IsGOAL_ENEMY("Is goal enemy"))

        self.add_child(REFEREE)

        EXE_SELECTOR = Selector("EXE_SELECTOR")
        EXE_SELECTOR.add_child(ExeOneRobot("ExeOneRobot"))
        EXE_SELECTOR.add_child(ExeNormal("ExeNormal"))

        self.add_child(EXE_SELECTOR)


class ExeOneRobot(ParallelAll):
    def __init__(self, name):
        super(ExeOneRobot, self).__init__(name)

        self.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        self.add_child(Attacker.StopGame("Attacker", 0))

class ExeNormal(ParallelAll):
    def __init__(self, name):
        super(ExeNormal, self).__init__(name)

        self.add_child(Analyst.ThereAreRobots("ThereAreRobots"))
        self.add_child(GoalKeeper.StopGame("GoalKeeper", 0))
        self.add_child(Attacker.StopGame("Attacker", 1))
        self.add_child(Defender.StopGame("Defender", 2))
