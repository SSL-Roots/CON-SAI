
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import sys,os
sys.path.append(os.pardir)
from world_model import WorldModel
import tool


class WithKick(Task):
    def __init__(self, name, my_role, kick_power=6.0, target_name=None,is_pass=False):
        super(WithKick, self).__init__(name)

        self._my_role = my_role
        self._kick_power = kick_power
        self._target_name = target_name
        self._is_pass = is_pass

        self._PASS_GAIN = 1.0 # 6 m/s * 9 m

    def run(self):
        kick_power = self._kick_power

        if self._is_pass and self._target_name:
            role_pose = WorldModel.get_pose(self._my_role)
            target_pose = WorldModel.get_pose(self._target_name)

            dist_to_target = tool.getSize(role_pose, target_pose)
            kick_power = dist_to_target * self._PASS_GAIN

        WorldModel.commands[self._my_role].set_kick(kick_power)

        return TaskStatus.SUCCESS


class NoNavigation(Task):
    def __init__(self, name, my_role):
        super(NoNavigation, self).__init__(name)

        self._my_role = my_role

    def run(self):
        WorldModel.commands[self._my_role].navigation_enable = False

        return TaskStatus.RUNNING


class NoBallAvoidance(Task):
    def __init__(self, name, my_role):
        super(NoBallAvoidance, self).__init__(name)

        self._my_role = my_role

    def run(self):
        WorldModel.commands[self._my_role].avoid_ball = False

        return TaskStatus.SUCCESS


class NoDefenceAreaAvoidance(Task):
    def __init__(self, name, my_role):
        super(NoDefenceAreaAvoidance, self).__init__(name)

        self._my_role = my_role

    def run(self):
        WorldModel.commands[self._my_role].avoid_defence_area = False

        return TaskStatus.RUNNING


class WithChip(Task):
    def __init__(self, name, my_role, kick_power=6.0):
        super(WithChip, self).__init__(name)

        self._my_role = my_role
        self._kick_power = kick_power


    def run(self):
        WorldModel.commands[self._my_role].set_kick(self._kick_power,True)

        return TaskStatus.RUNNING


class WithDribble(Task):
    def __init__(self, name, my_role, dribble_power=6.0):
        super(WithDribble, self).__init__(name)

        self._my_role = my_role
        self._dribble_power = dribble_power


    def run(self):
        WorldModel.commands[self._my_role].set_dribble(self._dribble_power)

        return TaskStatus.SUCCESS

