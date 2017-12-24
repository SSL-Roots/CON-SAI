# -*- coding: utf-8 -*-
import math
import cmath
import tf
import numpy

from geometry_msgs.msg import Point
from consai_msgs.msg import Pose


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


def get_intersection(pose1, pose2, pose3, pose4):
    # get intersection of line1(pose1, pose2) and line2(pose3, pose4)
    # reference:http://imagingsolution.blog107.fc2.com/blog-entry-137.html
    s1 = ((pose4.x - pose3.x) * (pose1.y - pose3.y) \
            - (pose4.y - pose3.y) * (pose1.x - pose3.x)) / 2.0

    s2 = ((pose4.x - pose3.x) * (pose3.y - pose2.y) \
            - (pose4.y - pose3.y) * (pose3.x - pose2.x)) / 2.0

    coefficient = s1 / (s1 + s2)

    output = Pose(0,0,0)
    output.x = pose1.x + (pose2.x - pose1.x) * coefficient
    output.y = pose1.y + (pose2.y - pose1.y) * coefficient

    return output


def limit(value, limit_high, limit_low):

    output = value

    if value > limit_high:
        output = limit_high
    elif value < limit_low:
        output = limit_low

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

