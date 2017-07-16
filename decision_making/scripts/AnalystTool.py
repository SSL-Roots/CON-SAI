# -*- coding: utf-8 -*-

import rospy
import tf
import math

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from GlobalData import GlobalInfo
import Tool
import Constants


def calcuNearestID():
    target = "ball"

    dist = 1000
    nearestID = None

    # nearest Friend ID
    for friendID in GlobalInfo.friendIDs:
        base = Tool.getFriendBase(friendID)

        GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
        (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))

        distToBall = math.hypot(point[0],point[1])
        if distToBall < dist:
            dist = distToBall
            nearestID = friendID

    GlobalInfo.nearestFriendID = nearestID

    # nearest Enemy ID
    dist = 1000
    nearestID = None

    # nearest Friend ID
    for enemyID in GlobalInfo.enemyIDs:
        base = Tool.getEnemyBase(enemyID)

        GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
        (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))

        distToBall = math.hypot(point[0],point[1])
        if distToBall < dist:
            dist = distToBall
            nearestID = enemyID

    GlobalInfo.nearestEnemyID = nearestID


def calcuGameSituation():
    THRESH_DIST = 0.5

    ballPos = GlobalInfo.ball.pose.pose.position

    # ボールが敵ディフェンスエリアに入ったらディフェンスモード
    # ヒステリシス性をもたせる
    IS_IN_THRESH = 1.2
    if GlobalInfo.isOffensive == False:
        IS_IN_THRESH = 1.3

    if isInDefenceArea(False, ballPos, IS_IN_THRESH) == True:
        GlobalInfo.isOffensive = False
        return

    # ボールが見方ディフェンスエリアに入ったらスーパーオフェンスモード
    # ヒステリシス性をもたせる
    IS_IN_THRESH = 1.2
    if GlobalInfo.isInOurGoal == True:
        IS_IN_THRESH = 1.3

    if isInDefenceArea(True, ballPos, IS_IN_THRESH) == True:
        GlobalInfo.isOffensive = True
        GlobalInfo.isInOurGoal = True
        return
    GlobalInfo.isInOurGoal = False

    # ボールが味方ディフェンスエリアに近ければ、DefenderOffenceモード
    # ヒステリシス性をもたせる
    # IS_IN_THRESH = 1.7
    # if GlobalInfo.isInDefenderArea == True:
    #     IS_IN_THRESH = 2.0
    #
    # if isInDefenceArea(True, ballPos, IS_IN_THRESH) == True:
    #     GlobalInfo.isOffensive = True
    #     GlobalInfo.isInDefenderArea = True
    #     return
    GlobalInfo.isInDefenderArea = False



    # nearestIDが存在しなければオフェンスモード
    nearestFriendID = GlobalInfo.nearestFriendID
    nearestEnemyID = GlobalInfo.nearestEnemyID

    if nearestEnemyID is None or nearestFriendID is None:
        GlobalInfo.isOffensive = True
        return

    # ボールとロボットの距離でモード判定

    # ボールが敵側にあればオフェンスモード優先
    if ballPos.x > -1.5:
        # 見方ロボットがボールに近ければオフェンスモード
        if thereIsBallerRobot(True):
            GlobalInfo.isOffensive = True
            return

        # 敵ロボットがボールに近ければディフェンスモード
        if thereIsBallerRobot(False):
            GlobalInfo.isOffensive = False
            return
    else:
        # 敵ロボットがボールに近ければディフェンスモード
        if thereIsBallerRobot(False):
            GlobalInfo.isOffensive = False
            return

        # 見方ロボットがボールに近ければオフェンスモード
        if thereIsBallerRobot(True):
            GlobalInfo.isOffensive = True
            return

    target = "ball"
    # 敵ロボットがボールに近ければディフェンスモード
    # base = Tool.getEnemyBase(nearestEnemyID)
    # GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
    # (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))
    #
    # distToBall = math.hypot(point[0],point[1])
    # if distToBall < THRESH_DIST:
    #     GlobalInfo.isOffensive = False
    #     return


    # 見方ロボットがボールに近ければオフェンスモード
    # base = Tool.getFriendBase(nearestFriendID)
    # GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
    # (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))
    #
    # distToBall = math.hypot(point[0],point[1])
    # if distToBall < THRESH_DIST:
    #     GlobalInfo.isOffensive = True
    #     return
    #

    # 何もなければオフェンスモード
    GlobalInfo.isOffensive = True


def thereIsBallerRobot(isFriend, THRESH_DIST = 0.5):
    target = "ball"

    base = ""
    if isFriend == True:
        base = Tool.getFriendBase(GlobalInfo.nearestFriendID)
    else:
        base = Tool.getEnemyBase(GlobalInfo.nearestEnemyID)

    GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
    (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))

    distToBall = math.hypot(point[0],point[1])
    if distToBall < THRESH_DIST:
        return True

    return False


def isInDefenceArea(isFriend, point, thresh= 1.0):
    target = Point()
    if isFriend == True:
        target = Constants.GoalFriend
    else:
        target = Constants.GoalEnemy

    dist = Tool.getSize(point,target)

    if dist < thresh:
        return True
    else:
        return False


def isNoObstacle(fromPoint, toPoint):
    OBSTACLE_DIST = 0.1

    angle = Tool.getAngle(fromPoint, toPoint)
    trans = Tool.Trans(fromPoint, angle)

    target = "map"
    for ID in GlobalInfo.enemyIDs:
        base = Tool.getEnemyBase(ID)
        GlobalInfo.tf_listener.waitForTransform(target,base,rospy.Time(0),rospy.Duration(3.0))
        (point,orientation) = GlobalInfo.tf_listener.lookupTransform(target,base,rospy.Time(0))

        obstPos = Point(point[0],point[1],0)
        trObstPos = trans.transform(obstPos)

        if trObstPos.x > 0 and abs(trObstPos.y) < OBSTACLE_DIST:
            return False

    return True



