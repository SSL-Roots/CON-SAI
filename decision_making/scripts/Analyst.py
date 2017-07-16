
import tf
import math

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from GlobalData import GlobalInfo
import Tool
import AnalystTool


class IsOnTarget(Task):
    def __init__(self, name, number, keepingTime):
        super(IsOnTarget, self).__init__(name)

        self._number = number
        self._reachingPosThreshold = 0.05 # 0.05
        self._reachingAngleThreshold = 3.0 * math.pi / 180.0 # 5.0

        self._keeping_time = keepingTime
        self.approach_time = 0.0


    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.FAILURE

        target = "map"
        GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
        (myPoint,myOrientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))
        myYaw = Tool.yawFromTfQuaternion(myOrientation)
        
        poseStamped = GlobalInfo.tf_listener \
            .transformPose("map",GlobalInfo.controls[self._number].target_pose)
        targetPoint = poseStamped.pose.position
        targetYaw = Tool.yawFromQuaternion(poseStamped.pose.orientation)

        diff_x = abs(targetPoint.x - myPoint[0])
        diff_y = abs(targetPoint.y - myPoint[1])
        diff_yaw = abs(Tool.normalize(targetYaw - myYaw))


        if diff_x < self._reachingPosThreshold \
                and diff_y < self._reachingPosThreshold \
                and diff_yaw < self._reachingAngleThreshold:
            
            if rospy.get_time() - self.approach_time > self._keeping_time:
                return TaskStatus.SUCCESS
            else:
                return TaskStatus.RUNNING
        else:
            self.approach_time = rospy.get_time()
            
            return TaskStatus.RUNNING

class HasBall(Task):
    def __init__(self, name, number):
        super(HasBall, self).__init__(name)

        self._number = number
        self._threshDist = 0.1
        self._threshAngle = 15.0 * math.pi / 180.0

    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.FAILURE

        target = "map"
        GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
        (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))

        robotPos = Point(point[0],point[1],0)
        robotYaw = Tool.yawFromTfQuaternion(orientation)

        ballPos = GlobalInfo.ball.pose.pose.position
        angleToBall = Tool.getAngle(robotPos,ballPos)

        trans = Tool.Trans(robotPos, angleToBall)
        trBallPos = trans.transform(ballPos)
        distToBall = Tool.getSizeFromCenter(trBallPos)
        trRobotYaw = trans.transformAngle(robotYaw)

        if distToBall < self._threshDist \
                and abs(trRobotYaw) < self._threshAngle:
            return TaskStatus.SUCCESS

        return TaskStatus.RUNNING



class BallMoved(Task):
    def __init__(self,name,onlySpeed=False):
        super(BallMoved, self).__init__(name)

        self._movingSpeedThreshold = 0.8
        self._movingPosThreshold = 0.3
        self._onlySpeed = onlySpeed

        self.ballInitialPos = Point()
        self.ballPosInitilaized = False

    def run(self):
        ballPos = GlobalInfo.ball.pose.pose.position
        ballVel = GlobalInfo.ball.twist.twist.linear

        if self.ballPosInitilaized == False:
            self.ballInitialPos = ballPos
            self.ballPosInitilaized = True
        ballSpeed = math.hypot(ballVel.x, ballVel.y)


        if ballSpeed > self._movingSpeedThreshold:
            return TaskStatus.SUCCESS

        if Tool.getSize(self.ballInitialPos, ballPos) > self._movingPosThreshold \
                and self._onlySpeed == False:
            return TaskStatus.SUCCESS

        return TaskStatus.RUNNING


    def reset(self):
        super(BallMoved, self).reset()
        self.ballPosInitilaized = False

class BallIsMoving(Task):
    def __init__(self, name):
        super(BallIsMoving, self).__init__(name)

        self._movingSpeedThreshold = 0.5

    def run(self):
        ballVel = GlobalInfo.ball.twist.twist.linear
        ballSpeed = math.hypot(ballVel.x, ballVel.y)

        if ballSpeed > self._movingSpeedThreshold:
            return TaskStatus.RUNNING
        else:
            return TaskStatus.FAILURE

class BallIsInField(Task):
    def __init__(self, name):
        super(BallIsInField, self).__init__(name)

    def run(self):
        ballPos = GlobalInfo.ball.pose.pose.position

        if abs(ballPos.x) > 4.5 or abs(ballPos.y) > 3.0:
            return TaskStatus.FAILURE
        return TaskStatus.RUNNING

class BallIsOutField(Task):
    def __init__(self, name):
        super(BallIsOutField, self).__init__(name)

    def run(self):
        ballPos = GlobalInfo.ball.pose.pose.position

        if abs(ballPos.x) > 4.5 or abs(ballPos.y) > 3.0:
            return TaskStatus.RUNNING
        return TaskStatus.FAILURE


class BallEnteringQuadrant(Task):
    def __init__(self, name, quadrant):
        super(BallEnteringQuadrant, self).__init__(name)
        # 2  |  1
        # ___|___
        #    |
        # 3  |  4

        self._quadrant = quadrant

    def run(self):
        ballPos = GlobalInfo.ball.pose.pose.position

        enterFlag = False

        if self._quadrant == 1:
            if ballPos.x > 0 and ballPos.y > 0:
                enterFlag = True

        elif self._quadrant == 2:
            if ballPos.x < 0 and ballPos.y > 0:
                enterFlag = True

        elif self._quadrant == 3:
            if ballPos.x < 0 and ballPos.y < 0:
                enterFlag = True

        else:
            if ballPos.x > 0 and ballPos.y < 0:
                enterFlag = True

        if enterFlag == True:
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.RUNNING
        

class InfoUpdate(Task):
    def __init__(self, name):
        super(InfoUpdate, self).__init__(name)

    def run(self):
        AnalystTool.calcuNearestID()
        AnalystTool.calcuGameSituation()

        return TaskStatus.RUNNING

class IsOffensive(Task):
    def __init__(self, name):
        super(IsOffensive, self).__init__(name)

    def run(self):
        if GlobalInfo.isOffensive == True \
                and GlobalInfo.isInOurGoal == False \
                and GlobalInfo.isInDefenderArea == False:

            return TaskStatus.RUNNING
        else:
            return TaskStatus.FAILURE


class IsUltimateOffensive(Task):
    def __init__(self, name):
        super(IsUltimateOffensive, self).__init__(name)

    def run(self):
        if GlobalInfo.isOffensive == True \
                and GlobalInfo.isInOurGoal == True:
            return TaskStatus.RUNNING
        else:
            return TaskStatus.FAILURE


class IsDefensive(Task):
    def __init__(self, name):
        super(IsDefensive, self).__init__(name)

    def run(self):
        if GlobalInfo.isOffensive == False:
            return TaskStatus.RUNNING
        else:
            return TaskStatus.FAILURE


class IsDefenderOffensive(Task):
    def __init__(self, name):
        super(IsDefenderOffensive, self).__init__(name)

    def run(self):
        if GlobalInfo.isOffensive == True \
                and GlobalInfo.isInDefenderArea == True:
            return TaskStatus.RUNNING
        else:
            return TaskStatus.FAILURE


class BallIsEnemySide(Task):
    def __init__(self, name):
        super(BallIsEnemySide, self).__init__(name)

    def run(self):
        ballPos = GlobalInfo.ball.pose.pose.position

        if ballPos.x > 0.5:
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE


class ThereAreThreeRobots(Task):
    def __init__(self, name):
        super(ThereAreThreeRobots, self).__init__(name)

    def run(self):
        if len(GlobalInfo.friendIDs) >= 3:
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.FAILURE

class ThereIsOnlyOneRobot(Task):
    def __init__(self, name):
        super(ThereIsOnlyOneRobot, self).__init__(name)

    def run(self):
        if len(GlobalInfo.friendIDs) == 1:
            return TaskStatus.RUNNING
        else:
            return TaskStatus.FAILURE

class ThereAreRobots(Task):
    def __init__(self, name):
        super(ThereAreRobots, self).__init__(name)

    def run(self):
        if len(GlobalInfo.friendIDs) > 1:
            return TaskStatus.RUNNING
        else:
            return TaskStatus.FAILURE

class IsBackSide(Task):
    def __init__(self, name, number):
        super(IsBackSide, self).__init__(name)


        self._number = number
        self._THRESH_IN_BACKSIDE = 150.0 * math.pi / 180.0

    def run(self):
        base = GlobalInfo.controls[self._number].get_base()

        if base is None:
            return TaskStatus.FAILURE

        target = "map"
        GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
        (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))

        robotPos = Point(point[0],point[1],0)
        ballPos = GlobalInfo.ball.pose.pose.position
        kickTarget= GlobalInfo.controls[self._number].getKickTarget()

        angleBallToTarget = Tool.getAngle(ballPos, kickTarget)
        trans = Tool.Trans(ballPos, angleBallToTarget)

        trRobotPos = trans.transform(robotPos)
        posAngle = Tool.getAngleFromCenter(trRobotPos)

        if abs(posAngle) > self._THRESH_IN_BACKSIDE:
            return TaskStatus.SUCCESS

        return TaskStatus.RUNNING

class IsBaller(Task):
    def __init__(self, name, number, keepingTime):
        super(IsBaller, self).__init__(name)

        self._number = number
        self._keeping_time = keepingTime

    def run(self):
        myID = GlobalInfo.controls[self._number].get_id()
        
        if myID is None:
            return TaskStatus.FAILURE

        nearetID = GlobalInfo.nearestFriendID

        if myID == nearetID:
            return TaskStatus.SUCCESS

        return TaskStatus.RUNNING
