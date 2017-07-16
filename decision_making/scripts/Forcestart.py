from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from CheckRefboxCommand import IsFORCE_START
import InPlay

class Forcestart(ParallelAll):
    def __init__(self, name):
        super(Forcestart, self).__init__(name)

        IS_FORCE_START = IsFORCE_START("Is force start")
        EXECUTE = InPlay.Execute("Execute inplay")
        

        self.add_child(IS_FORCE_START)
        self.add_child(EXECUTE)
