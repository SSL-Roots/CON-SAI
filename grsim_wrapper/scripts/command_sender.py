#!/usr/bin/env  python
import  rospy
import  tf
import  socket
import  grSim_Packet_pb2
from    datetime import *
import  time
import math

from    geometry_msgs.msg import Twist
from    consai_msgs.msg import robot_commands


class Sender:
    def __init__(self):
        # initialize a socket
        self.host = rospy.get_param('~server_address', '127.0.0.1')
        self.port = rospy.get_param('~server_port', 20011)
        self.friend_color = rospy.get_param('/friend_color', 'yellow')

        rospy.loginfo('server address is set to [' + self.host         + ']')
        rospy.loginfo('server port is set to ['    + str(self.port)    + ']')
        rospy.loginfo('team color is set to ['     + self.friend_color + ']')

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # make subscribers
        self.subscribers = []
        for i in xrange(12):
            topic = "/robot_" + str(i) + "/robot_commands"
            self.subscribers.append(
                    rospy.Subscriber(
                        topic,
                        robot_commands,
                        self.sendCommands,
                        callback_args=i))

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


if  __name__ == '__main__':
  rospy.init_node('grsim_commnad_sender')

  sender = Sender()

  rospy.spin()
