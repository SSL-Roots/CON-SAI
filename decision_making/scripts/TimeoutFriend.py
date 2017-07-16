from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsTIMEOUT_FRIEND
import BaseSkill

class TimeoutFriend(ParallelAll):
    def __init__(self, name):
        super(TimeoutFriend , self).__init__(name)

        IS_TIMEOUT_FRIEND = IsTIMEOUT_FRIEND('is timeout friend')
        HALT = BaseSkill.AllRobots_Halt("AllRobots_Halt")

        self.add_child(IS_TIMEOUT_FRIEND)
        self.add_child(HALT)
