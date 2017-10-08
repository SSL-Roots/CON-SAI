# -*- coding: utf-8 -*-
import math
import cmath
import tf
import numpy

from geometry_msgs.msg import Point
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

def ivertAngle(angle):
    return normalize(angle + math.pi)

def getAngle(fromPoint, toPoint):
    diffPoint = Point()

    diffPoint.x = toPoint.x - fromPoint.x
    diffPoint.y = toPoint.y - fromPoint.y

    return math.atan2(diffPoint.y, diffPoint.x)

def getSize(fromPoint, toPoint):
    diffPoint = Point()

    diffPoint.x = toPoint.x - fromPoint.x
    diffPoint.y = toPoint.y - fromPoint.y

    return math.hypot(diffPoint.x, diffPoint.y)

def getSizeFromCenter(point):
    return math.hypot(point.x, point.y)

def getAngleFromCenter(point):
    return math.atan2(point.y, point.x)


def getConjugate(point):
    output = Point()
    output.x = point.x
    output.y = -point.y

    return output

class Trans():
    def __init__(self, center , theta):

        normalizedTheta = normalize(theta)
        self._c_center = center.x + center.y * 1.0j
        self._c_rotate = cmath.rect(1.0,normalizedTheta) 
        self._c_angle = normalizedTheta

    def transform(self, point):
        c_point = point.x + point.y * 1.0j
        c_output = (c_point - self._c_center) * numpy.conj(self._c_rotate)

        output = Point()
        output.x = c_output.real
        output.y = c_output.imag

        return output

    def invertedTransform(self, point):
        c_point = point.x + point.y * 1.0j
        c_output = c_point * self._c_rotate + self._c_center

        output = Point()
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


def convertToReceivePos(point):
    THRESH_BALL_MOVING = 1.0
    THRESH_RECEIVE_DIST = 1.0


    ballPos = GlobalInfo.ball.pose.pose.position
    ballVel = GlobalInfo.ball.twist.twist.linear

    ballSpeed = getSizeFromCenter(ballVel)

    canReceive = False
    if ballSpeed > THRESH_BALL_MOVING:
        angleOfSpeed = getAngleFromCenter(ballVel)
        trans = Trans(ballPos, angleOfSpeed)
        trPoint = trans.transform(point)

        if abs(trPoint.y) < THRESH_RECEIVE_DIST \
                and trPoint.x > 0.0 :
            trPoint.y = 0.0
            point = trans.invertedTransform(trPoint)
            canReceive = True

    return point, canReceive


def convertToReceiveShootPose(point, targetPos):
    THRESH_BALL_MOVING = 1.0
    THRESH_RECEIVE_DIST = 1.5
    DRIBLLER_DIST = 0.080

    angleRobotToTarget = getAngle(point, targetPos)

    # convert robotPos to dribblerPos
    dribblerPos = Point()
    dribblerPos.x = point.x + DRIBLLER_DIST * math.cos(angleRobotToTarget)
    dribblerPos.y = point.y + DRIBLLER_DIST * math.sin(angleRobotToTarget)

    ballPos = GlobalInfo.ball.pose.pose.position
    ballVel = GlobalInfo.ball.twist.twist.linear

    ballSpeed = getSizeFromCenter(ballVel)

    canShoot = False
    if ballSpeed > THRESH_BALL_MOVING:
        angleOfSpeed = getAngleFromCenter(ballVel)
        trans = Trans(ballPos, angleOfSpeed)
        trPoint = trans.transform(dribblerPos)


        if abs(trPoint.y) < THRESH_RECEIVE_DIST \
                and trPoint.x > 0.0 :
            trPoint.y = 0.0
            point = trans.invertedTransform(trPoint)
            canShoot = True

            # invert dribblerPos to robotPos
            point.x = point.x - DRIBLLER_DIST * math.cos(angleRobotToTarget)
            point.y = point.y - DRIBLLER_DIST * math.sin(angleRobotToTarget)

    return point, angleRobotToTarget, canShoot
