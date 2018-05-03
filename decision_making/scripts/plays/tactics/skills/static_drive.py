

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from consai_msgs.msg import TestAICommand
import sys,os
import math
sys.path.append(os.pardir)
from world_model import WorldModel

class SendVelocity(Task):
    def __init__(self, name, my_role, vel_x, vel_y, vel_yaw):
        super(SendVelocity, self).__init__(name)
        self.my_role = my_role

        self._vel_x = vel_x
        self._vel_y = vel_y
        self._vel_yaw = vel_yaw

    def run(self):
        WorldModel.commands[self.my_role].set_target_velocity(
                self._vel_x, self._vel_y, self._vel_yaw)

        return TaskStatus.RUNNING


class SendCommand(Task):
    def __init__(self, name, my_role):
        super(SendCommand, self).__init__(name)

        self.my_role = my_role

    def run(self):

        command = WorldModel.get_test_ai_command()

        WorldModel.commands[self.my_role].set_target_velocity(
                command.vel_x, command.vel_y, command.vel_yaw)
        WorldModel.commands[self.my_role].set_kick(
                command.kick_power, command.do_chip)
        WorldModel.commands[self.my_role].set_dribble(
                command.dribble_power)
        return TaskStatus.RUNNING


class SendTrapezoidalCommand(Task):
    def __init__(self, name, my_role, is_vel_x=False, is_vel_y=False, is_vel_yaw=False):
        super(SendTrapezoidalCommand, self).__init__(name)

        self.my_role = my_role

        self._current_vel = 0
        self._current_count = 0
        self._is_vel_x = is_vel_x
        self._is_vel_y = is_vel_y
        self._is_vel_yaw = is_vel_yaw

        self._state = "ACC"

    def run(self):
        command = WorldModel.get_test_ai_command()
        # use test_command for parameters
        acc = command.vel_x
        dec = command.vel_y
        keep_count = command.vel_yaw
        limit = command.kick_power

        if self._state == "ACC":
            self._current_vel += acc
            if self._current_vel >= limit:
                self._current_vel = limit
                self._state = "KEEP"

        if self._state == "KEEP":
            self._current_count += 1
            if self._current_count >= keep_count:
                self._state = "DEC"

        if self._state == "DEC":
            self._current_vel -= dec
            if self._current_vel <= 0:
                self._current_vel = 0
                self._state = "FINISH"

        vel_x = 0
        vel_y = 0
        vel_yaw = 0

        if self._is_vel_x:
            vel_x = self._current_vel
        elif self._is_vel_y:
            vel_y = self._current_vel
        elif self._is_vel_yaw:
            vel_yaw = self._current_vel

        WorldModel.commands[self.my_role].set_target_velocity(
                vel_x, vel_y, vel_yaw)

        return TaskStatus.RUNNING

