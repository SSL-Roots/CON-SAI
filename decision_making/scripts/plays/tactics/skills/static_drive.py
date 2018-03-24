

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from consai_msgs.msg import TestAICommand
import sys,os
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
