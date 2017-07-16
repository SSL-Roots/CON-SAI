from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import TestPattern

import sys,os
sys.path.append(os.pardir)
import  GlobalData
import  CheckRefboxCommand

class TestParent(Selector):
    def __init__(self, name):
        super(TestParent, self).__init__(name)

        self.stop = StopChild("stop_child")
        self.halt = HaltChild("halt_child")
        
        self.add_child(self.stop)
        self.add_child(self.halt)


class StopChild(ParallelAll):
    def __init__(self, name):
        super(StopChild, self).__init__(name)

        self.is_stop = CheckRefboxCommand.IsSTOP('is_stop')
        self.pattern = TestPattern.BackandForth("back_and_forth")
        # self.pattern = TestPattern.GoToRandom("go_to_random")
        # self.pattern = TestPattern.PassAndPass("passAndPass", 0, 1)
        # self.pattern = TestPattern.VelocityTest("VelocityTest")

        invert = Invert("invert")
        invert.add_child(self.pattern)

        self.add_child(self.is_stop)
        self.add_child(invert)

class HaltChild(ParallelAll):
    def __init__(self, name):
        super(HaltChild, self).__init__(name)

        self.is_halt = CheckRefboxCommand.IsHALT('is_halt')
        self.pattern = TestPattern.HaltPattern("halt_pattern")

        self.add_child(self.is_halt)
        self.add_child(self.pattern)
