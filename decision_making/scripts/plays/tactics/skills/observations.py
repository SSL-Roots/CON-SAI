
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import rospy
import sys,os
sys.path.append(os.pardir)
from world_model import WorldModel
import tool


class BallKicked(Task):
    def __init__(self, name):
        super(BallKicked, self).__init__(name)

    
    def run(self):
        if WorldModel.ball_kicked():
            return TaskStatus.SUCCESS

        return TaskStatus.RUNNING

class CanReceive(Task):
    def __init__(self, name, my_role):
        super(CanReceive, self).__init__(name)

        self._my_role = my_role

    
    def run(self):
        if WorldModel.can_receive(self._my_role):
            return TaskStatus.SUCCESS
        
        return TaskStatus.FAILURE

class CanShoot(Task):
    def __init__(self, name, my_role):
        super(CanShoot, self).__init__(name)

        self._my_role = my_role

    def run(self):
        if WorldModel.can_shoot(self._my_role):
            return TaskStatus.SUCCESS

        return TaskStatus.FAILURE

class CanPass(Task):
    def __init__(self, name, my_role):
        super(CanPass, self).__init__(name)

        self._my_role = my_role

    def run(self):
        if WorldModel.can_pass(self._my_role):
            return TaskStatus.SUCCESS

        return TaskStatus.FAILURE

class CanReflectShoot(Task):
    def __init__(self, name, my_role, target_name):
        super(CanReflectShoot, self).__init__(name)

        self._my_role = my_role
        self._target = target_name

    def run(self):
        if WorldModel.can_reflect_shoot(self._my_role, self._target):
            return TaskStatus.SUCCESS

        return TaskStatus.FAILURE

class IsLooking(Task):
    def __init__(self, name, my_role, target):
        super(IsLooking, self).__init__(name)

        self._my_role = my_role
        self._target = target

    def run(self):
        if WorldModel.is_looking(self._my_role, self._target):
            return TaskStatus.SUCCESS

        return TaskStatus.RUNNING

class IsClose(Task):
    def __init__(self, name, target1, target2, thresh_dist):
        super(IsClose, self).__init__(name)

        self._target1 = target1
        self._target2 = target2
        self._thresh_dist = thresh_dist

    def run(self):
        pose1 = WorldModel.get_pose(self._target1)
        pose2 = WorldModel.get_pose(self._target2)
        dist = tool.getSize(pose1, pose2)

        if dist < self._thresh_dist:
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.RUNNING

class BallIsInField(Task):
    def __init__(self, name):
        super(BallIsInField, self).__init__(name)

        pass

    def run(self):
        if WorldModel.ball_is_in_field():
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.RUNNING

class HasBall(Task):
    def __init__(self, name, my_role):
        super(HasBall, self).__init__(name)

        self._my_role = my_role

    def run(self):
        if WorldModel.has_ball(self._my_role):
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE
