import  rospy
import  tf
import  math
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import  GlobalData
from GlobalData import GlobalInfo


def checkHaveBall():
#     GlobalData.GlobalData.tf_listener.waitForTransform('friend_0/base_link', 'ball', rospy.Time(0), rospy.Duration(3.0))
#     (trans, rot)    = GlobalData.GlobalData.tf_listener.lookupTransform('friend_0/base_link', 'ball', rospy.Time(0))
    
    if GlobalInfo.controls[0].has_id() == False:
        return False

    id = GlobalInfo.controls[0].get_id()
    base = "friend_"+str(id)+"/base_link"
    GlobalInfo.tf_listener.waitForTransform(base,"ball",rospy.Time(0),rospy.Duration(3.0))
    (trans, rot) = GlobalInfo.tf_listener.lookupTransform(base,"ball",rospy.Time(0))

    angle   = math.atan2(trans[1], trans[0])
    dist    = math.hypot(trans[0], trans[1])

    if abs(angle) < 0.4 and dist < 0.15:
        # have ball
        return  True
    else:
        return  False

def checkMoveBall():
#     spd_x   = GlobalData.GlobalData.ball.twist.twist.linear.x
#     spd_y   = GlobalData.GlobalData.ball.twist.twist.linear.y
    spd_x = GlobalInfo.ball.twist.twist.linear.x
    spd_y = GlobalInfo.ball.twist.twist.linear.y

    if math.hypot(spd_x, spd_y) > 0.7:
        return  True
    else:
        return  False


class Halt(Task):
    def __init__(self, name):
        super(Halt, self).__init__(name)

    def run(self):
        self.announce()

#         GlobalData.GlobalData.setTargetVelocity(0.0, 0.0, 0.0)
        for control in GlobalInfo.controls:
            control.setTargetVelocity(0.0, 0.0, 0.0)
            control.setKickVelocity(0.0)

        return  TaskStatus.RUNNING


class StayAroundBall(Task):
    def __init__(self, name):
        super(StayAroundBall, self).__init__(name)

    def run(self):
        self.announce()

        # GlobalData.GlobalData.tf_listener.waitForTransform('ball', 'base_link', rospy.Time(0), rospy.Duration(3.0))
        # (trans_robot, rot_robot)    = GlobalData.GlobalData.tf_listener.lookupTransform('ball', 'base_link', rospy.Time())
        # now_angle      = math.atan2(trans_robot[1], trans_robot[0])

#         target_x    = GlobalData.GlobalData.ball.pose.pose.position.x - 0.5
#         target_y    = GlobalData.GlobalData.ball.pose.pose.position.y

        target_x = GlobalInfo.ball.pose.pose.position.x - 0.5
        target_y = GlobalInfo.ball.pose.pose.position.y
        
        if target_x < -4.5 :
#             target_x    = GlobalData.GlobalData.ball.pose.pose.position.x
            target_x = GlobalInfo.ball.pose.pose.position.x

            if target_y < 0.0 :
#                 target_y    = GlobalData.GlobalData.ball.pose.pose.position.y + 0.5
                target_y = GlobalInfo.ball.pose.pose.position.y + 0.5
            else:
                target_x = GlobalInfo.ball.pose.pose.position.x
                target_y = GlobalInfo.ball.pose.pose.position.y -0.5
#                 target_x    = GlobalData.GlobalData.ball.pose.pose.position.x
#                 target_y    = GlobalData.GlobalData.ball.pose.pose.position.y - 0.5

#         GlobalData.GlobalData.setTargetPose(target_x, target_y, 0.0, 'map')
        GlobalInfo.controls[0].setTargetPose(target_x,target_y,0.0,"map")

        return  TaskStatus.RUNNING


class CornerStandby(Task):
    """"""
    def __init__(self, name):
        super(CornerStandby, self).__init__(name)

    def run(self):
        self.announce()

        if GlobalInfo.controls[0].has_id() == False:
            return TaskStatus.FAILURE

#         GlobalData.GlobalData.tf_listener.waitForTransform('enemy_goal_center', 'ball', rospy.Time(0), rospy.Duration(3.0))
#         (trans_ball, rot_ball)    = GlobalData.GlobalData.tf_listener.lookupTransform('enemy_goal_center', 'ball', rospy.Time(0))
#         GlobalData.GlobalData.tf_listener.waitForTransform('ball', 'friend_0/base_link', rospy.Time(0), rospy.Duration(3.0))
#         (trans_robot, rot_robot)    = GlobalData.GlobalData.tf_listener.lookupTransform('ball', 'friend_0/base_link', rospy.Time())

        GlobalInfo.tf_listener.waitForTransform("enemy_goal_center","ball",rospy.Time(0),rospy.Duration(3.0))
        (trans_ball, rot_ball) = GlobalInfo.tf_listener.lookupTransform("enemy_goal_center","ball",rospy.Time(0))

        id = GlobalInfo.controls[0].get_id()
        base = "friend_"+str(id)+"/base_link"
        GlobalInfo.tf_listener.waitForTransform("ball",base,rospy.Time(0),rospy.Duration(3.0))
        (trans_robot, rot_robot) = GlobalInfo.tf_listener.lookupTransform("ball",base,rospy.Time(0))

        target_angle   = math.atan2(trans_ball[1], trans_ball[0])
        now_angle      = math.atan2(trans_robot[1], trans_robot[0])

        R = 0.5
        target_x    = R * math.cos(target_angle)
        target_y    = R * math.sin(target_angle)
        target_ang  = target_angle - math.pi

#         GlobalData.GlobalData.setTargetPose(target_x, target_y, target_ang, 'ball')
#         GlobalData.GlobalData.setKickVelocity(0.0)
        GlobalInfo.controls[0].setTargetPose(target_x, target_y, target_ang, "ball")
        GlobalInfo.setKickVelocity(0.0)

        if abs(target_angle - now_angle) < 0.4:
            print   'aligh sitayo!'
            return  TaskStatus.SUCCESS
        return  TaskStatus.RUNNING


class WrapAroundBall(Task):
    """"""
    def __init__(self, name):
        super(WrapAroundBall, self).__init__(name)

    def run(self):
        self.announce()

        spd_x = GlobalInfo.ball.twist.twist.linear.x
        spd_y = GlobalInfo.ball.twist.twist.linear.y

        if math.hypot(spd_x, spd_y) > 0.7:
            GlobalInfo.controls[0].setTargetVelocity(0.0,0.0,0.0)
            return TaskStatus.RUNNING

        if GlobalInfo.controls[0].has_id() == False:
            return TaskStatus.FAILURE

#         GlobalData.GlobalData.tf_listener.waitForTransform('enemy_goal_center', 'ball', rospy.Time(0), rospy.Duration(3.0))
#         (trans_ball, rot_ball)    = GlobalData.GlobalData.tf_listener.lookupTransform('enemy_goal_center', 'ball', rospy.Time(0))
#         GlobalData.GlobalData.tf_listener.waitForTransform('ball', 'friend_0/base_link', rospy.Time(0), rospy.Duration(3.0))
#         (trans_robot, rot_robot)    = GlobalData.GlobalData.tf_listener.lookupTransform('ball', 'friend_0/base_link', rospy.Time())

        GlobalInfo.tf_listener.waitForTransform("enemy_goal_center","ball",rospy.Time(0),rospy.Duration(3.0))
        (trans_ball,rot_ball) = GlobalInfo.tf_listener.lookupTransform("enemy_goal_center","ball",rospy.Time(0))

        id = GlobalInfo.controls[0].get_id()
        base = "friend_"+str(id)+"/base_link"
        GlobalInfo.tf_listener.waitForTransform("ball",base,rospy.Time(0),rospy.Duration(3.0))
        (trans_robot,rot_robot) = GlobalInfo.tf_listener.lookupTransform("ball",base,rospy.Time(0))

        target_angle   = math.atan2(trans_ball[1], trans_ball[0])
        now_angle      = math.atan2(trans_robot[1], trans_robot[0])

        R = 0.5
        target_x    = R * math.cos(target_angle)
        target_y    = R * math.sin(target_angle)
        target_ang  = target_angle - math.pi

#         GlobalData.GlobalData.setTargetPose(target_x, target_y, target_ang, 'ball')
#         GlobalData.GlobalData.setKickVelocity(0.0)
        GlobalInfo.controls[0].setTargetPose(target_x,target_y,target_ang,"ball")
        GlobalInfo.controls[0].setKickVelocity(0.0)

        if abs(target_angle - now_angle) < 1.0:
            print   'aligh sitayo!'
            return  TaskStatus.SUCCESS
        return  TaskStatus.RUNNING




        # GlobalData.GlobalData.tf_listener.waitForTransform('base_link', 'enemy_goal_center', rospy.Time(0), rospy.Duration(3.0))
        # (trans_center, rot_center)    = GlobalData.GlobalData.tf_listener.lookupTransform('base_link', 'enemy_goal_center', rospy.Time(0))
        # GlobalData.GlobalData.tf_listener.waitForTransform('base_link', 'enemy_goal_left', rospy.Time(0), rospy.Duration(3.0))
        # (trans_left, rot_left)    = GlobalData.GlobalData.tf_listener.lookupTransform('base_link', 'enemy_goal_left', rospy.Time(0))
        # GlobalData.GlobalData.tf_listener.waitForTransform('base_link', 'enemy_goal_right', rospy.Time(0), rospy.Duration(3.0))
        # (trans_right, rot_right)    = GlobalData.GlobalData.tf_listener.lookupTransform('base_link', 'enemy_goal_right', rospy.Time(0))
        #
        # angle_center   = math.atan2(trans_center[1], trans_center[0])
        # angle_left = math.atan2(trans_left[1], trans_left[0])
        # angle_right = math.atan2(trans_right[1], trans_right[0])
        #
        # R = 0.5
        # target_x    = R * math.cos(angle_center)
        # target_y    = R * math.sin(angle_center)
        # target_ang  = target_angle - math.pi
        #
        # GlobalData.GlobalData.setTargetPose(target_x, target_y, target_ang, 'ball')
        # GlobalData.GlobalData.setKickVelocity(0.0)
        #
        # GlobalData.GlobalData.setTargetVelocity(1.0, 0.0, angle_center*1.5)
        # GlobalData.GlobalData.setKickVelocity(0.0)





class AimToGoal(Task):
    def __init__(self, name):
        super(AimToGoal, self).__init__(name)

    def run(self):
        self.announce()
        
        if GlobalInfo.controls[0].has_id() == False:
            return TaskStatus.FAILURE

#         GlobalData.GlobalData.tf_listener.waitForTransform('friend_0/base_link', 'enemy_goal_center', rospy.Time(0), rospy.Duration(3.0))
#         (trans_center, rot_center)    = GlobalData.GlobalData.tf_listener.lookupTransform('friend_0/base_link', 'enemy_goal_center', rospy.Time(0))
#         GlobalData.GlobalData.tf_listener.waitForTransform('friend_0/base_link', 'enemy_goal_left', rospy.Time(0), rospy.Duration(3.0))
#         (trans_left, rot_left)    = GlobalData.GlobalData.tf_listener.lookupTransform('friend_0/base_link', 'enemy_goal_left', rospy.Time(0))
#         GlobalData.GlobalData.tf_listener.waitForTransform('friend_0/base_link', 'enemy_goal_right', rospy.Time(0), rospy.Duration(3.0))
#         (trans_right, rot_right)    = GlobalData.GlobalData.tf_listener.lookupTransform('friend_0/base_link', 'enemy_goal_right', rospy.Time(0))
        
        id = GlobalInfo.controls[0].get_id()
        base = "friend_"+str(id)+"/base_link"

        GlobalInfo.tf_listener.waitForTransform(base,"enemy_goal_center",rospy.Time(0),rospy.Duration(3.0))
        (trans_center, rot_center) = GlobalInfo.tf_listener.lookupTransform(base,"enemy_goal_center",rospy.Time(0))
        GlobalInfo.tf_listener.waitForTransform(base,"enemy_goal_left",rospy.Time(0),rospy.Duration(3.0))
        (trans_left,rot_left) = GlobalInfo.tf_listener.lookupTransform(base,"enemy_goal_left",rospy.Time(0))
        GlobalInfo.tf_listener.waitForTransform(base,"enemy_goal_right",rospy.Time(0),rospy.Duration(3.0))
        (trans_right,rot_right) = GlobalInfo.tf_listener.lookupTransform(base,"enemy_goal_right",rospy.Time(0))


        angle_center   = math.atan2(trans_center[1], trans_center[0])
        angle_left = math.atan2(trans_left[1], trans_left[0])
        angle_right = math.atan2(trans_right[1], trans_right[0])

#         GlobalData.GlobalData.setTargetVelocity(1.0, 0.0, angle_center*1.5)
#         GlobalData.GlobalData.setKickVelocity(0.0)

        GlobalInfo.controls[0].setTargetVelocity(0.3, 0.0, angle_center*1.0)
        GlobalInfo.controls[0].setKickVelocity(0.0)

        if checkHaveBall() == False:
            return  TaskStatus.FAILURE
        elif angle_left > 0.0 and angle_right < 0.0:
            return  TaskStatus.SUCCESS
        else:
            return  TaskStatus.RUNNING

class Shoot(Task):
    def __init__(self, name):
        super(Shoot, self).__init__(name)
        self.start_time = rospy.Time.now()

    def run(self):
        self.announce()

#         GlobalData.GlobalData.setKickVelocity(8.0)
        GlobalInfo.controls[0].setKickVelocity(8.0)

        duration = (rospy.Time.now() - self.start_time).to_sec()
        if  duration < 0.5:
            return  TaskStatus.RUNNING

        return  TaskStatus.SUCCESS

    def reset(self):
        super(Shoot, self).reset()
        self.start_time = rospy.Time.now()


class MoveTowardBall(Task):
    def __init__(self, name):
        super(MoveTowardBall, self).__init__(name)

    def run(self):
        self.announce()

        if GlobalInfo.controls[0].has_id() == False:
            return TaskStatus.FAILURE

        id = GlobalInfo.controls[0].get_id()
        base = "friend_"+str(id)+"/base_link"

        GlobalInfo.tf_listener.waitForTransform(base,"ball",rospy.Time(0),rospy.Duration(3.0))
        (trans, rot) = GlobalInfo.tf_listener.lookupTransform(base,"ball",rospy.Time(0))

#         GlobalData.GlobalData.tf_listener.waitForTransform('friend_0/base_link', 'ball', rospy.Time(0), rospy.Duration(3.0))
#         (trans, rot)    = GlobalData.GlobalData.tf_listener.lookupTransform('friend_0/base_link', 'ball', rospy.Time(0))

        angle   = math.atan2(trans[1], trans[0])
        dist    = math.hypot(trans[0], trans[1])

        L = 0.3 # 0.5
        offset_x    = L * math.cos(angle)
        offset_y    = L * math.sin(angle)

#         GlobalData.GlobalData.setTargetPose(trans[0] + offset_x, trans[1] + offset_y, angle, 'friend_0/base_link')
#         GlobalData.GlobalData.setKickVelocity(0.0)
        GlobalInfo.controls[0].setTargetPose(trans[0] + offset_x, trans[1] + offset_y, angle, base)
        GlobalInfo.controls[0].setKickVelocity(0.0)

        if checkHaveBall():
            print   'success getball'
            return  TaskStatus.SUCCESS
        else:
            print   'running getball'
            return  TaskStatus.RUNNING


class Defence(Task):
    def __init__(self, name):
        super(Defence, self).__init__(name)

    def run(self):
        self.announce()

        GlobalData.GlobalData.tf_listener.waitForTransform('friend_goal_center', 'ball', rospy.Time(0), rospy.Duration(3.0))
        (trans, rot)    = GlobalData.GlobalData.tf_listener.lookupTransform('friend_goal_center', 'ball', rospy.Time(0))

        GlobalInfo.tf_listener.waitForTransform('friend_goal_center', 'ball', rospy.Time(0), rospy.Duration(3.0))
        (trans, rot)    = GlobalInfo.tf_listener.lookupTransform('friend_goal_center', 'ball', rospy.Time(0))

        angle   = math.atan2(trans[1], trans[0])

        R   = 0.5;
        pose_x  = R * math.cos(angle)
        pose_y  = R * math.sin(angle)
        pose_yaw    = angle

#         GlobalData.GlobalData.setTargetPose(pose_x, pose_y, pose_yaw, 'friend_goal_center')
        GlobalInfo.controls[0].setTargetPose(pose_x,pose_y,pose_yaw,"friend_goal_center")

        return  TaskStatus.RUNNING


class   StayAroundOrigin(Task):
    def __init__(self, name):
        super(StayAroundOrigin, self).__init__(name)

    def run(self):
        self.announce()

        target_x    = -0.5
        target_y    = 0.0

#         GlobalData.GlobalData.setTargetPose(target_x, target_y, 0.0, 'map')
        GlobalInfo.controls[0].setTargetPose(target_x,target_y,0.0,"map")

        return  TaskStatus.RUNNING


class BackHome(Task):
    def __init__(self, name):
        super(BackHome, self).__init__(name)

    def run(self):
        self.announce()
        # move to home position
#         GlobalData.GlobalData.setTargetPose(-4.0, 0.0, 0.0, 'map')
        GlobalInfo.controls[0].setTargetPose(-4.0, 0.0, 0.0, "map")
        return  TaskStatus.RUNNING


class WaitInplay(Task):
    def __init__(self, name):
        super(WaitInplay, self).__init__(name)

    def run(self):
        self.announce()

        if checkMoveBall():
            return  TaskStatus.SUCCESS
        else:
            return  TaskStatus.RUNNING
