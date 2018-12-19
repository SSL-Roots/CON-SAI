#!/usr/bin/env  python

import rospy
import tf
import socket
from datetime import *
import time
import math

from proto import grSim_Packet_pb2
from geometry_msgs.msg import Twist
from consai_msgs.msg import robot_commands
from consai_msgs.msg import ReplaceBall
from consai_msgs.msg import ReplaceRobot


class Sender:
    def __init__(self):
        # initialize a socket
        self.host = rospy.get_param('~server_address', '127.0.0.1')
        self.port = rospy.get_param('~server_port', 20011)
        self.friend_color = rospy.get_param('friend_color', 'yellow')
        self._ID_MAX = rospy.get_param('id_max', 12)

        rospy.loginfo('server address is set to [' + self.host         + ']')
        rospy.loginfo('server port is set to ['    + str(self.port)    + ']')
        rospy.loginfo('team color is set to ['     + self.friend_color + ']')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Replacement
        self.team_side = rospy.get_param('team_side', 'left')
        self.REVERSE = 1.0
        self.REVERSE_ANGLE = 0.0
        if self.team_side == 'right':
            self.REVERSE = -1.0
            self.REVERSE_ANGLE = -180

        # make subscribers
        self.subscribers = []
        for i in xrange(self._ID_MAX):
            topic = "robot_" + str(i) + "/robot_commands"
            self.subscribers.append(
                    rospy.Subscriber(
                        topic,
                        robot_commands,
                        self.sendCommands,
                        callback_args=i))

        self.subscribers.append(
                rospy.Subscriber(
                    "replacement_ball",
                    ReplaceBall,
                    self.send_ball_replacement))
        self.subscribers.append(
                rospy.Subscriber(
                    "replacement_robot",
                    ReplaceRobot,
                    self.send_robot_replacement))


    def sendCommands(self,data,id):
        packet = grSim_Packet_pb2.grSim_Packet()
        now_time = (time.mktime(datetime.now().timetuple()))
        packet.commands.timestamp = now_time

        if self.friend_color == "yellow":
            packet.commands.isteamyellow = True
        else:
            packet.commands.isteamyellow = False

        commands = packet.commands.robot_commands.add()
        commands.id = id
        commands.kickspeedx = data.kick_speed_x
        commands.kickspeedz = data.kick_speed_z
        if math.isnan(data.vel_surge)  \
            or math.isnan(data.vel_sway)  \
            or math.isnan(data.omega):
            commands.veltangent = 0
            commands.velnormal = 0
            commands.velangular = 0
        else:
            commands.veltangent = data.vel_surge
            commands.velnormal = data.vel_sway
            commands.velangular = data.omega

        if data.dribble_power > 0:
            commands.spinner = True
        else:
            commands.spinner = False
            
        commands.wheelsspeed = False

        message = packet.SerializeToString()
        self.sock.sendto(message,(self.host,self.port))


    def send_ball_replacement(self, data):
        packet = grSim_Packet_pb2.grSim_Packet()

        replace_ball = packet.replacement.ball
        replace_ball.x = self.REVERSE * data.pos_x
        replace_ball.y = self.REVERSE * data.pos_y
        replace_ball.vx = self.REVERSE * data.vel_x
        replace_ball.vy = self.REVERSE * data.vel_y

        message = packet.SerializeToString()
        self.sock.sendto(message,(self.host, self.port))


    def send_robot_replacement(self, data):
        packet = grSim_Packet_pb2.grSim_Packet()

        replace_robot = packet.replacement.robots.add()
        replace_robot.id = data.robot_id
        replace_robot.yellowteam = data.is_yellow
        replace_robot.x = self.REVERSE * data.pos_x
        replace_robot.y = self.REVERSE * data.pos_y
        replace_robot.dir = self.REVERSE_ANGLE + data.dir
        replace_robot.turnon = data.turn_on

        message = packet.SerializeToString()
        self.sock.sendto(message,(self.host, self.port))


if  __name__ == '__main__':
  rospy.init_node('grsim_commnad_sender')

  sender = Sender()

  rospy.spin()
