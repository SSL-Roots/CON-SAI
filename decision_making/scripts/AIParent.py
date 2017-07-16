
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from Halt import Halt
from Stopgame import Stopgame
from KickoffFriend import KickoffFriend
from Forcestart import Forcestart
from IndirectFriend import IndirectFriend
from DirectFriend import DirectFriend
from PenaltyFriend import PenaltyFriend
from KickoffEnemy import KickoffEnemy
from IndirectEnemy import IndirectEnemy
from DirectEnemy import DirectEnemy
from PenaltyEnemy import PenaltyEnemy
from TimeoutFriend import TimeoutFriend
from TimeoutEnemy import TimeoutEnemy

import Analyst
import BaseSkill

class GameMain(Selector):
    def __init__(self, name):
        super(GameMain, self).__init__(name)

        BALL_INFIELD = ParallelAll("BALL_INFIELD")
        BALL_INFIELD.add_child(Analyst.BallIsInField("BallIsInField"))
        BALL_INFIELD.add_child(AI_Parent("AI_PARENT"))
        BALL_INFIELD.add_child(Analyst.InfoUpdate("INFO_UPDATE"))

        BALL_OUTFIELD = ParallelAll("BALL_OUTFIELD")
        BALL_OUTFIELD.add_child(Analyst.BallIsOutField("BallIsOutField"))
        BALL_OUTFIELD.add_child(BaseSkill.AllRobots_Halt("AllRobots_Halt"))
        self.add_child(BALL_INFIELD)
        self.add_child(BALL_OUTFIELD)


class AI_Parent(Selector):
    def __init__(self, name):
        super(AI_Parent, self).__init__(name)

        halt = Halt("Halt")
        stop_game = Stopgame("Stop_game")
        kickoff_friend = KickoffFriend("KickoffFriend")
        force_start = Forcestart("Force_start")
        indirect_friend = IndirectFriend("IndirectFriend")
        direct_friend = DirectFriend("DirectFriend")
        penalty_friend = PenaltyFriend("PenaltyFriend")
        kickoff_enemy = KickoffEnemy("KickoffEnemy")
        indirect_enemy = IndirectEnemy("IndirectEnemy")
        direct_enemy = DirectEnemy("DirectEnemy")
        penalty_enemy = PenaltyEnemy("PenaltyEnemy")
        timeout_friend = TimeoutFriend("TimeoutFriend")
        timeout_enemy = TimeoutEnemy("TimeoutEnemy")

        self.add_child(halt)
        self.add_child(stop_game)
        self.add_child(kickoff_friend)
        self.add_child(force_start)
        self.add_child(indirect_friend)
        self.add_child(direct_friend)
        self.add_child(penalty_friend)
        self.add_child(kickoff_enemy)
        self.add_child(indirect_enemy)
        self.add_child(direct_enemy)
        self.add_child(penalty_enemy)
        self.add_child(timeout_friend)
        self.add_child(timeout_enemy)
