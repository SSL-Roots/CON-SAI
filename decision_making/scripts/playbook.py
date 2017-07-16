from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *


class   StayAroundOrigin(Task):
    def __init__(self, name):
        super(StayAroundOrigin, self).__init__(name)

        self.name = name
        self.count = 0

    def run(self):
        rospy.loginfo("running!")

        if self.count > 10:
            return  TaskStatus.SUCCESS

        self.count = self.count + 1

        return TaskStatus.RUNNING

    def reset(self):
        self.count = 0


class StayAroundBall(Task):
    def __init__(self, name):
        super(StayAroundBall, self).__init__(name)

        self.name   = name

    def run(self):
        rospy.loginfo("2tume")
        return  TaskStatus.RUNNING

