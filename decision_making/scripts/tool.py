# -*- coding: utf-8 -*-
import math
import cmath
import tf
import numpy

from geometry_msgs.msg import Point
from consai_msgs.msg import Pose
from GlobalData import GlobalInfo

def yawFromQuaternion(quaternion):
    euler = tf.transformations.euler_from_quaternion(
            (quaternion.x,quaternion.y,quaternion.z,quaternion.w))
    return euler[2]

def yawFromTfQuaternion(quaternion):
    euler = tf.transformations.euler_from_quaternion(
            (quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
    return euler[2]

def pointFromTranslation(translation):
    output = Point()
    output.x = translation[0]
    output.y = translation[1]

    return output

def normalize(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle

def invertAngle(angle):
    return normalize(angle + math.pi)

def getAngle(fromPose, toPose):
    diffPose = Pose()

    diffPose.x = toPose.x - fromPose.x
    diffPose.y = toPose.y - fromPose.y

    return math.atan2(diffPose.y, diffPose.x)

def getSize(fromPose, toPose):
    diffPose = Pose()

    diffPose.x = toPose.x - fromPose.x
    diffPose.y = toPose.y - fromPose.y

    return math.hypot(diffPose.x, diffPose.y)

def getSizeFromCenter(pose):
    return math.hypot(pose.x, pose.y)

def getAngleFromCenter(pose):
    return math.atan2(pose.y, pose.x)


def getConjugate(pose):
    output = Pose()
    output.x = pose.x
    output.y = -pose.y

    return output

class Trans():
    def __init__(self, center , theta):

        normalizedTheta = normalize(theta)
        self._c_center = center.x + center.y * 1.0j
        self._c_rotate = cmath.rect(1.0,normalizedTheta) 
        self._c_angle = normalizedTheta

    def transform(self, pose):
        c_point = pose.x + pose.y * 1.0j
        c_output = (c_point - self._c_center) * numpy.conj(self._c_rotate)

        output = Pose()
        output.x = c_output.real
        output.y = c_output.imag

        return output

    def invertedTransform(self, pose):
        c_point = pose.x + pose.y * 1.0j
        c_output = c_point * self._c_rotate + self._c_center

        output = Pose()
        output.x = c_output.real
        output.y = c_output.imag

        return output

    def transformAngle(self, angle):
        return normalize(angle - self._c_angle)

    def invertedTransformAngle(self, angle):
        return normalize(angle + self._c_angle)


def getEnemyBase(ID):
    base = "enemy_"+str(ID)+"/base_link"
    return base

def getFriendBase(ID):
    base = "friend_"+str(ID)+"/base_link"
    return base


def convertToReceivePos(pose):
    THRESH_BALL_MOVING = 1.0
    THRESH_RECEIVE_DIST = 1.0


    ballPos = GlobalInfo.ball.pose.pose.position
    ballVel = GlobalInfo.ball.twist.twist.linear

    ballSpeed = getSizeFromCenter(ballVel)

    canReceive = False
    if ballSpeed > THRESH_BALL_MOVING:
        angleOfSpeed = getAngleFromCenter(ballVel)
        trans = Trans(ballPos, angleOfSpeed)
        trPose = trans.transform(pose)

        if abs(trPose.y) < THRESH_RECEIVE_DIST \
                and trPose.x > 0.0 :
            trPose.y = 0.0
            pose = trans.invertedTransform(trPose)
            canReceive = True

    return pose, canReceive


def convertToReceiveShootPose(pose, targetPos):
    THRESH_BALL_MOVING = 1.0
    THRESH_RECEIVE_DIST = 1.5
    DRIBLLER_DIST = 0.080

    angleRobotToTarget = getAngle(pose, targetPos)

    # convert robotPos to dribblerPos
    dribblerPos = Pose()
    dribblerPos.x = pose.x + DRIBLLER_DIST * math.cos(angleRobotToTarget)
    dribblerPos.y = pose.y + DRIBLLER_DIST * math.sin(angleRobotToTarget)

    ballPos = GlobalInfo.ball.pose.pose.position
    ballVel = GlobalInfo.ball.twist.twist.linear

    ballSpeed = getSizeFromCenter(ballVel)

    canShoot = False
    if ballSpeed > THRESH_BALL_MOVING:
        angleOfSpeed = getAngleFromCenter(ballVel)
        trans = Trans(ballPos, angleOfSpeed)
        trPose = trans.transform(dribblerPos)


        if abs(trPose.y) < THRESH_RECEIVE_DIST \
                and trPose.x > 0.0 :
            trPose.y = 0.0
            pose = trans.invertedTransform(trPose)
            canShoot = True

            # invert dribblerPos to robotPos
            pose.x = pose.x - DRIBLLER_DIST * math.cos(angleRobotToTarget)
            pose.y = pose.y - DRIBLLER_DIST * math.sin(angleRobotToTarget)

    return pose, angleRobotToTarget, canShoot
