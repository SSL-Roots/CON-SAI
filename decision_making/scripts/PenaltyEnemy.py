from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsPENALTY_ENEMY, IsNORMAL_START
import GoalKeeper
import Analyst
import Kick
import BaseSkill


class PenaltyEnemy(Selector):
    def __init__(self, name):
        super(PenaltyEnemy , self).__init__(name)

        self.prepare    = Prepare('Prepare Enemy penalty kick')
        self.execute    = Execute('Execute Enemy penalty kick')

        # add parent tasks
        self.add_child(self.prepare)
        self.add_child(self.execute)

class Prepare(ParallelAll):
    def __init__(self, name):
        super(Prepare, self).__init__(name)

        IS_PENALTY_ENEMY = IsPENALTY_ENEMY("Is Penalty enemy")

        self.add_child(IS_PENALTY_ENEMY)
        self.add_child(GoalKeeper.PenaltyDefense("PenaltyDefense", 0))
        self.add_child(BaseSkill.WaitPenaltyShoot("Wait1", 1))
        self.add_child(BaseSkill.WaitPenaltyShoot("Wait2", 2))


class Execute(ParallelAll):
    def __init__(self, name):
        super(Execute, self).__init__(name)

        IS_NORMAL_START = IsNORMAL_START('is_normal_start')

        INPLAY_SWITCH = Sequence("INPLAY_SWITCH")

        EXE_DEFEND = ParallelOne("EXE_DEFEND")
        EXE_DEFEND.add_child(Analyst.BallMoved("BallMoved"))
        EXE_DEFEND.add_child(GoalKeeper.PenaltyDefense("PenaltyDefense", 0))
        EXE_DEFEND.add_child(BaseSkill.WaitPenaltyShoot("Wait1", 1))
        EXE_DEFEND.add_child(BaseSkill.WaitPenaltyShoot("Wait2", 2))
        INPLAY_SWITCH.add_child(EXE_DEFEND)

        EXE_SHOOT = ParallelAll("EXE_SHOOT")
        EXE_SHOOT.add_child(Kick.CalcuKickTarget("CalcuKickTarget",0))
        EXE_SHOOT.add_child(Kick.Shoot("Shoot",0))
        EXE_SHOOT.add_child(BaseSkill.WaitPenaltyShoot("Wait1", 1))
        EXE_SHOOT.add_child(BaseSkill.WaitPenaltyShoot("Wait2", 2))
        INPLAY_SWITCH.add_child(EXE_SHOOT)


        self.add_child(IS_NORMAL_START)
        self.add_child(INPLAY_SWITCH)
