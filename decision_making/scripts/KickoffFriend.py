from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsKICKOFF_FRIEND, IsNORMAL_START
import GoalKeeper
import Kick
import InPlay
import Defender
import Analyst

class KickoffFriend(Selector):
    def __init__(self, name):
        super(KickoffFriend, self).__init__(name)

        KICKOFF_PRE = ParallelAll("KICKOFF_PRE")
        KICKOFF_PRE.add_child(IsKICKOFF_FRIEND("IsKICKOFF_FRIEND"))

        EXE_SELECTOR_PRE = Selector("EXE_SELECTOR_PRE")
        EXE_SELECTOR_PRE.add_child(ExeOneRobot_Pre("ExeOneRobot_Pre"))
        EXE_SELECTOR_PRE.add_child(ExeNormal_Pre("ExeNormal_Pre"))
        KICKOFF_PRE.add_child(EXE_SELECTOR_PRE)


        NORMAL_START = ParallelAll("NORMAL_START")
        NORMAL_START.add_child(IsNORMAL_START("IsNORMAL_START"))

        EXE_SELECTOR_SHOOT = Selector("EXE_SELECTOR_SHOOT")
        EXE_SELECTOR_SHOOT.add_child(ExeOneRobot_Shoot("ExeOneRobot_Shoot"))
        EXE_SELECTOR_SHOOT.add_child(ExeNormal_Shoot("ExeNormal_Shoot"))
        NORMAL_START.add_child(EXE_SELECTOR_SHOOT)

        self.add_child(KICKOFF_PRE)
        self.add_child(NORMAL_START)


class ExeOneRobot_Pre(ParallelAll):
    def __init__(self, name):
        super(ExeOneRobot_Pre, self).__init__(name)

        self.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        self.add_child(Kick.CalcuKickTarget("CalcuKickTarget", 0))
        self.add_child(Kick.Approach("Approach", 0))


class ExeOneRobot_Shoot(ParallelAll):
    def __init__(self, name):
        super(ExeOneRobot_Shoot, self).__init__(name)

        self.add_child(Analyst.ThereIsOnlyOneRobot("ThereIsOneRobot"))
        
        SHOOT_ITERATOR = Iterator("SHOOT_ITERATOR")
        SHOOT_ITERATOR.add_child(Kick.SetplayShoot("Shoot", 0))
        SHOOT_ITERATOR.add_child(InPlay.Execute("Execute inplay"))
        self.add_child(SHOOT_ITERATOR)


class ExeNormal_Pre(ParallelAll):
    def __init__(self, name):
        super(ExeNormal_Pre, self).__init__(name)

        self.add_child(Analyst.ThereAreRobots("ThereAreRobots"))
        self.add_child(GoalKeeper.StopGame("GoalKeeperStop", 0))
        self.add_child(Kick.CalcuKickTarget("CalcuKickTarget", 1))
        self.add_child(Kick.Approach("Approach", 1))
        self.add_child(Defender.StopGame("DefenderStop", 2))


class ExeNormal_Shoot(ParallelAll):
    def __init__(self, name):
        super(ExeNormal_Shoot, self).__init__(name)

        self.add_child(Analyst.ThereAreRobots("ThereAreRobots"))

        SHOOT_ITERATOR = Iterator("SHOOT_ITERATOR")
        SHOOT_ITERATOR.add_child(Kick.SetplayShoot("Shoot", 1))
        SHOOT_ITERATOR.add_child(InPlay.Execute("Execute inplay"))
        self.add_child(SHOOT_ITERATOR)

class KickoffExecute(Iterator):
    def __init__(self, name):
        super(KickoffExecute, self).__init__(name)

        SHOOT = Kick.SetplayShoot("Shoot",1)
        INPLAY = InPlay.Execute("Execute inplay")

        self.add_child(SHOOT)
        self.add_child(INPLAY)
