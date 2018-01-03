#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import rospkg
import tf
import math

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt,QPointF, QRectF
from python_qt_binding.QtGui import QWidget,QPainter, QPen, QColor

from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16MultiArray as UIntArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from geometry_msgs.msg import Point

class ConstWorld():
    def __init__(self):
        self.FIELD_HEIGHT = 7.4
        self.FIELD_WIDTH = 10.4
        self.FIELD_H_PER_W = self.FIELD_HEIGHT/self.FIELD_WIDTH
        self.FIELD_W_PER_H = self.FIELD_WIDTH/self.FIELD_HEIGHT
        self.FIELD_CENTER_RADIUS = 0.5
        self.FIELD_DEFENCE_RADIUS = 1.0
        self.FIELD_STREACH = 0.25
        self.WALL_HEIGHT = self.FIELD_HEIGHT - 0.4 * 2.0
        self.WALL_WIDTH = self.FIELD_WIDTH - 0.4 * 2.0
        self.PLAY_FIELD_HEIGHT = 6.0
        self.PLAY_FIELD_WIDTH = 9.0
        self.GOAL_HEIGHT = 1.0
        self.GOAL_WIDTH = 0.18
        self.BALL_RADIUS = 0.043
        self.ROBOT_RADIUS = 0.18 * 0.5

class PaintWidget(QWidget):
    def __init__(self,parent=None):
        super(PaintWidget,self).__init__(parent)

        self.CW = ConstWorld()
        self.scaleOnField = 1.0
        self.fieldHeight = 0.0
        self.fieldWidth = 0.0
        self.rotatingWorld = False
        self.trans = QPointF(0.0,0.0) # 慢性的なトランス
        self.mouseTrans = QPointF(0.0, 0.0) # マウス操作で発生する一時的なトランス
        self.scale = QPointF(1.0,1.0)
        self.clickPoint = QPointF(0.0,0.0)

        self.friendDrawColor = Qt.cyan
        self.enemyDrawColor = Qt.yellow
        self.friend_color = rospy.get_param("friend_color", "blue")
        if self.friend_color != "blue":
            self.friendDrawColor = Qt.yellow
            self.enemyDrawColor = Qt.cyan

        self.targetPosDrawColor = QColor(102, 0, 255, 100)


        self.ballOdom = Odometry()
        self.sub_ballPosition = rospy.Subscriber("ball_observer/estimation", 
                Odometry,self.callbackBallOdom)

        self.friendsIDArray = UIntArray()
        self.sub_friendsID = rospy.Subscriber("existing_friends_id", 
                UIntArray, self.callbackFriendsID)

        self.friendOdoms = [Odometry()] * 12
        self.sub_friendOdoms = []

        self.enemyIDArray = UIntArray()
        self.sub_enemiesID = rospy.Subscriber("existing_enemies_id",
                UIntArray, self.callbackEnemiesID)

        self.enemyOdoms = [Odometry()] * 12
        self.sub_enemyOdoms = []

        self.targetPositions = [PoseStamped()] * 12
        self.sub_targetPositions =[]
        self.targetVelocities = [TwistStamped()] * 12
        self.sub_targetVelocities = []
        self.targetIsPosition = [False] * 12

        self.avoidPoints = [Point()] * 12
        self.sub_avoidPoints = []

        for i in xrange(12):
            strID = str(i)
            topicFriend = "robot_" + strID + "/odom"
            topicEnemy = "enemy_" + strID + "/odom"
            topicPosition = "robot_" + strID + "/move_base_simple/goal"
            topicVelocity = "robot_" + strID + "/move_base_simple/target_velocity"
            topicAvoidPoint = "robot_" + strID + "/avoid_point"

            self.sub_friendOdoms.append(
                    rospy.Subscriber(topicFriend, Odometry, 
                        self.callbackFriendOdom, callback_args=i))

            self.sub_enemyOdoms.append(
                    rospy.Subscriber(topicEnemy, Odometry,
                        self.callbackEnemiesOdom, callback_args=i))

            self.sub_targetPositions.append(
                    rospy.Subscriber(topicPosition, PoseStamped,
                        self.callbackTargetPosition, callback_args=i))

            self.sub_targetVelocities.append(
                    rospy.Subscriber(topicVelocity, TwistStamped,
                        self.callbackTargetVelocity, callback_args=i))

            self.sub_avoidPoints.append(
                    rospy.Subscriber(topicAvoidPoint, Point,
                        self.callbackAvoidPoint, callback_args=i))

    def callbackBallOdom(self, msg):
        self.ballOdom = msg
        # self.update()

    def callbackFriendsID(self, msg):
        self.friendsIDArray = msg
        # self.update()

    def callbackFriendOdom(self, msg, robot_id):
        self.friendOdoms[robot_id] = msg

    def callbackEnemiesID(self, msg):
        self.enemyIDArray = msg

    def callbackEnemiesOdom(self, msg, robot_id):
        self.enemyOdoms[robot_id] = msg

    def callbackTargetPosition(self, msg, robot_id):
        self.targetPositions[robot_id] = msg
        self.targetIsPosition[robot_id] = True

    def callbackTargetVelocity(self, msg, robot_id):
        self.targetVelocities[robot_id] = msg
        self.targetIsPosition[robot_id] = False

    def callbackAvoidPoint(self, msg, robot_id):
        self.avoidPoints[robot_id] = msg

    def mousePressEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            self.clickPoint = event.posF()
        elif event.buttons() == Qt.RightButton:
            self.resetPainterState()

        self.update()

    def mouseMoveEvent(self, event):
        if event.buttons() == Qt.LeftButton:
            pos = event.posF()
            self.mouseTrans = (pos - self.clickPoint) / self.scale.x()

        self.update()

    def mouseReleaseEvent(self, event):
        self.trans += self.mouseTrans
        self.mouseTrans = QPointF(0.0, 0.0)

        self.update()


    def wheelEvent(self, event):
        # マウスのホイール操作でスケールを変える
        if event.delta() > 0:
            s = self.scale.x()
            self.scale.setX(s + 0.1)
            self.scale.setY(s + 0.1)
        else:
            s = self.scale.x()
            if s > 0.2 :
                self.scale.setX(s - 0.1)
                self.scale.setY(s - 0.1)

        self.update()


    def resizeEvent(self, event):
        # widgetのサイズ変更によるイベント
        self.updateDrawState()

    def paintEvent(self, event):
        painter = QPainter(self)

        # 描画の中心をWidgetの中心に持ってくる
        cx = float(self.width()) * 0.5
        cy = float(self.height()) * 0.5
        painter.translate(cx,cy)

        # これ以降にトランスとスケール操作を持ってくる
        painter.scale(self.scale.x(), self.scale.y())
        painter.translate(self.trans + self.mouseTrans)

        if self.rotatingWorld == True:
            painter.rotate(-90)


        # これ以降に描きたいものを重ねていく
        self.drawField(painter)
        
        self.drawTargets(painter)

        # self.drawAvoidPoints(painter)

        self.drawFriends(painter)
        self.drawEnemis(painter)
        self.drawBallVelocity(painter)
        self.drawBall(painter)

    def resetPainterState(self):
        self.trans = QPointF(1.0,1.0)
        self.mouseTrans = QPointF(0.0, 0.0)
        self.scale = QPointF(1.0, 1.0)

    def updateDrawState(self):
        # Widgetのサイズに合わせて、描くフィールドのサイズを変える
        # 描画の回転判断もしてくれるすぐれもの

        widgetHeight = float(self.height())
        widgetWidth = float(self.width())
        w_per_h = widgetWidth/widgetHeight

        if w_per_h >= self.CW.FIELD_W_PER_H:
            # Widgetが横長のとき
            self.fieldHeight = widgetHeight
            self.fieldWidth = widgetHeight * self.CW.FIELD_W_PER_H
            self.rotatingWorld = False
        elif w_per_h <= self.CW.FIELD_H_PER_W:
            # Widgetが縦長のとき
            self.fieldHeight = widgetWidth
            self.fieldWidth = widgetWidth * self.CW.FIELD_W_PER_H
            self.rotatingWorld = True
        else:
            # 描画回転にヒステリシス性をもたせる
            if self.rotatingWorld == True:
                self.fieldHeight = widgetHeight * self.CW.FIELD_H_PER_W
                self.fieldWidth = widgetHeight
            else:
                self.fieldHeight = widgetWidth * self.CW.FIELD_H_PER_W
                self.fieldWidth = widgetWidth

        self.scaleOnField = self.fieldWidth / self.CW.FIELD_WIDTH

    def convertToDrawWorld(self, x, y):
        drawX = x * self.scaleOnField
        drawY = -y * self.scaleOnField
        point = QPointF(drawX, drawY)

        return point

    def drawField(self, painter):
        # draw green surface rectangle
        painter.setPen(Qt.black)
        painter.setBrush(Qt.green)

        rx = -self.fieldWidth * 0.5
        ry = -self.fieldHeight * 0.5

        rect = QRectF(rx, ry, self.fieldWidth, self.fieldHeight)
        painter.drawRect(rect)

        # draw wall rectangle
        painter.setPen(QPen(Qt.black,3))
        painter.setBrush(Qt.NoBrush)
        
        sizeX = self.CW.WALL_WIDTH * self.scaleOnField
        sizeY = self.CW.WALL_HEIGHT * self.scaleOnField

        rx = -sizeX * 0.5
        ry = -sizeY * 0.5

        rect = QRectF(rx, ry, sizeX, sizeY)
        painter.drawRect(rect)

        # draw center circle
        painter.setPen(QPen(Qt.white,2))
        point = self.convertToDrawWorld(0.0, 0.0)
        size = self.CW.FIELD_CENTER_RADIUS * self.scaleOnField
        painter.drawEllipse(point, size, size)

        # draw play field rectangle
        sizeX = self.CW.PLAY_FIELD_WIDTH * self.scaleOnField
        sizeY = self.CW.PLAY_FIELD_HEIGHT * self.scaleOnField

        rx = -sizeX * 0.5
        ry = -sizeY * 0.5

        rect = QRectF(rx, ry, sizeX, sizeY)
        painter.drawRect(rect)

        # draw mid-line
        point1 = self.convertToDrawWorld(-self.CW.PLAY_FIELD_WIDTH * 0.5, 0)
        point2 = self.convertToDrawWorld(self.CW.PLAY_FIELD_WIDTH * 0.5, 0)

        painter.drawLine(point1, point2)

        # draw center line
        point1 = self.convertToDrawWorld(0, self.CW.PLAY_FIELD_HEIGHT * 0.5)
        point2 = self.convertToDrawWorld(0, -self.CW.PLAY_FIELD_HEIGHT * 0.5)

        painter.drawLine(point1, point2)

        # draw streach line
        x = self.CW.PLAY_FIELD_WIDTH * 0.5 - self.CW.FIELD_DEFENCE_RADIUS
        point1 = self.convertToDrawWorld(x, self.CW.FIELD_STREACH * 0.5)
        point2 = self.convertToDrawWorld(x, -self.CW.FIELD_STREACH * 0.5)
        painter.drawLine(point1, point2)

        x *= -1.0
        point1 = self.convertToDrawWorld(x, self.CW.FIELD_STREACH * 0.5)
        point2 = self.convertToDrawWorld(x, -self.CW.FIELD_STREACH * 0.5)
        painter.drawLine(point1, point2)

        # draw defence arc
        sizeX = self.CW.FIELD_DEFENCE_RADIUS * 2.0 * self.scaleOnField
        sizeY = self.CW.FIELD_DEFENCE_RADIUS * 2.0 * self.scaleOnField

        rx = self.CW.PLAY_FIELD_WIDTH * 0.5 - self.CW.FIELD_DEFENCE_RADIUS
        ry = self.CW.FIELD_STREACH * 0.5 + self.CW.FIELD_DEFENCE_RADIUS
        ry *= -1.0
        rx *= self.scaleOnField
        ry *= self.scaleOnField

        rect = QRectF(rx, ry, sizeX, sizeY)
        startAngle = 90 * 16
        spanAngle = 90 * 16
        painter.drawArc(rect, startAngle, spanAngle) # top right
        
        ry = self.CW.FIELD_STREACH * 0.5 - self.CW.FIELD_DEFENCE_RADIUS
        ry *= self.scaleOnField
        rect = QRectF(rx, ry, sizeX, sizeY)
        startAngle = 180 * 16
        spanAngle = 90 * 16
        painter.drawArc(rect, startAngle, spanAngle) # bottom right


        rx = -self.CW.PLAY_FIELD_WIDTH * 0.5 - self.CW.FIELD_DEFENCE_RADIUS
        ry = self.CW.FIELD_STREACH * 0.5 + self.CW.FIELD_DEFENCE_RADIUS
        ry *= -1.0
        rx *= self.scaleOnField
        ry *= self.scaleOnField

        rect = QRectF(rx, ry, sizeX, sizeY)
        startAngle = 0 * 16
        spanAngle = 90 * 16
        painter.drawArc(rect, startAngle, spanAngle) # top left

        ry = self.CW.FIELD_STREACH * 0.5 - self.CW.FIELD_DEFENCE_RADIUS
        ry *= self.scaleOnField

        rect = QRectF(rx, ry, sizeX, sizeY)
        startAngle = 270 * 16
        spanAngle = 90 * 16
        painter.drawArc(rect, startAngle, spanAngle) # bottom left

        
        # draw goal rectangle
        sizeX = self.CW.GOAL_WIDTH * self.scaleOnField
        sizeY = self.CW.GOAL_HEIGHT * self.scaleOnField

        rx = self.CW.PLAY_FIELD_WIDTH * 0.5
        ry = -self.CW.GOAL_HEIGHT * 0.5
        rx *= self.scaleOnField
        ry *= self.scaleOnField

        rect = QRectF(rx, ry, sizeX, sizeY)
        painter.drawRect(rect)

        rx = -self.CW.PLAY_FIELD_WIDTH * 0.5 - self.CW.GOAL_WIDTH
        rx *= self.scaleOnField
        rect = QRectF(rx, ry, sizeX, sizeY)
        painter.drawRect(rect)


    def drawBall(self, painter):
        posX = self.ballOdom.pose.pose.position.x
        posY = self.ballOdom.pose.pose.position.y

        point = self.convertToDrawWorld(posX,posY)
        size = self.CW.BALL_RADIUS * self.scaleOnField

        painter.setPen(Qt.black)
        painter.setBrush(Qt.red)
        painter.drawEllipse(point, size, size)

    def drawBallVelocity(self, painter):
        ballPos = self.ballOdom.pose.pose.position
        ballVel = self.ballOdom.twist.twist.linear

        if math.hypot(ballVel.x, ballVel.y) < 1.0:
            return 

        angleOfSpeed = math.atan2(ballVel.y, ballVel.x)

        paintDist = 10.0

        velPosX = paintDist * math.cos(angleOfSpeed) + ballPos.x
        velPosY = paintDist * math.sin(angleOfSpeed) + ballPos.y

        ballPosPoint = self.convertToDrawWorld(ballPos.x, ballPos.y)
        velPosPoint = self.convertToDrawWorld(velPosX, velPosY)

        painter.setPen(QPen(QColor(102,0,255),2))
        painter.drawLine(ballPosPoint, velPosPoint)


    def drawFriends(self, painter):
        for robot_id in self.friendsIDArray.data:
            self.drawRobot(painter, robot_id, 
                    self.friendOdoms[robot_id], self.friendDrawColor)

    def drawEnemis(self, painter):
        for robot_id in self.enemyIDArray.data:
            self.drawRobot(painter, robot_id,
                    self.enemyOdoms[robot_id], self.enemyDrawColor)

    def drawRobot(self, painter,robot_id, odom, color):
        # draw robot body on its position
        posX = odom.pose.pose.position.x
        posY = odom.pose.pose.position.y

        point = self.convertToDrawWorld(posX, posY)
        size = self.CW.ROBOT_RADIUS * self.scaleOnField

        painter.setPen(Qt.black)
        painter.setBrush(color)
        painter.drawEllipse(point, size, size)

        # draw robot angle on its body
        orientation = odom.pose.pose.orientation
        # euler_from_quaternion does not support geometry_msgs:Quaternion
        euler = tf.transformations.euler_from_quaternion(
                (orientation.x, orientation.y, orientation.z, orientation.w))
        # euler <- (roll, pitch, yaw)
        linePosX = self.CW.ROBOT_RADIUS * math.cos(euler[2])
        linePosY = self.CW.ROBOT_RADIUS * math.sin(euler[2])
        linePoint = point + self.convertToDrawWorld(linePosX, linePosY)
        painter.drawLine(point, linePoint)

        # draw robot_id on its head
        textPosX = 0.15
        textPosY = 0.15
        textPoint = point + self.convertToDrawWorld(textPosY, textPosY)
        painter.drawText(textPoint, str(robot_id))

    def drawTargets(self, painter):
        for robot_id in self.friendsIDArray.data:
            if self.targetIsPosition[robot_id] == True:
                self.drawTargetPosition(painter, 
                        robot_id, self.targetPositions[robot_id])
            else:
                self.drawTargetVelocity(painter,
                        robot_id, self.targetVelocities[robot_id])

    def drawTargetPosition(self, painter, robot_id, positionStamped):
        posX = positionStamped.pose.position.x
        posY = positionStamped.pose.position.y

        point = self.convertToDrawWorld(posX, posY)
        size = self.CW.ROBOT_RADIUS * self.scaleOnField

        painter.setPen(Qt.black)
        painter.setBrush(self.targetPosDrawColor)
        painter.drawEllipse(point, size, size)

        orientation = positionStamped.pose.orientation
        # euler_from_quaternion does not support geometry_msgs:Quaternion
        euler = tf.transformations.euler_from_quaternion(
                (orientation.x, orientation.y, orientation.z, orientation.w))
        # euler <- (roll, pitch, yaw)
        linePosX = self.CW.ROBOT_RADIUS * math.cos(euler[2])
        linePosY = self.CW.ROBOT_RADIUS * math.sin(euler[2])
        linePoint = point + self.convertToDrawWorld(linePosX, linePosY)
        painter.drawLine(point, linePoint)

        # draw robot_id on its head
        textPosX = 0.15
        textPosY = 0.15
        textPoint = point + self.convertToDrawWorld(textPosY, textPosY)
        painter.drawText(textPoint, str(robot_id))

    def drawTargetVelocity(self, painter, robot_id, twistStamped):
        odom = self.friendOdoms[robot_id]
        posX = odom.pose.pose.position.x
        posY = odom.pose.pose.position.y

        point = self.convertToDrawWorld(posX, posY)

        # draw robot_id on its head
        textPosX = 0.15
        textPosY = -0.15
        textPoint = point + self.convertToDrawWorld(textPosY, textPosY)

        velX = twistStamped.twist.linear.x
        velY = twistStamped.twist.linear.y
        velZ = twistStamped.twist.angular.z

        text = "(" + str(velX) + ", " + str(velY) + ", " + str(velZ) + ")"
        painter.setPen(Qt.red)
        painter.drawText(textPoint, text)

    def drawAvoidPoints(self, painter):
        for robot_id in self.friendsIDArray.data:
            self.drawAvoidPoint(painter, robot_id, self.avoidPoints[robot_id])

    def drawAvoidPoint(self, painter, robot_id, point):
        drawPoint = self.convertToDrawWorld(point.x, point.y)
        size = self.CW.ROBOT_RADIUS * self.scaleOnField

        painter.setPen(Qt.black)
        painter.setBrush(Qt.red)
        painter.drawEllipse(drawPoint, size, size)

        # draw robot_id on its head
        textPosX = 0.15
        textPosY = 0.15
        textPoint = drawPoint + self.convertToDrawWorld(textPosY, textPosY)
        painter.drawText(textPoint, str(robot_id))
