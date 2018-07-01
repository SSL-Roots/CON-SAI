

import rospy
import math

import geometry_msgs
from proto import messages_robocup_ssl_wrapper_pb2
from tf.transformations import quaternion_from_euler

from consai_msgs.msg import VisionObservations, VisionPacket, VisionRobotPackets


class FormatConverter:

    def __init__(self, friend_color, do_side_invert):
        self.table = []
        self.friend_color = friend_color
        self.do_side_invert = do_side_invert

    def protobufToTable(self, protobuf_binary):
        ssl_wrapper = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
        ssl_wrapper.ParseFromString(protobuf_binary)

        header = {
            'frame_number': ssl_wrapper.detection.frame_number, 't_capture': ssl_wrapper.detection.t_capture, 't_sent': ssl_wrapper.detection.t_sent, 'camera_id': ssl_wrapper.detection.camera_id
        }

        for ball in ssl_wrapper.detection.balls:
            observation = {
                'color': 'ball', 'confidence': ball.confidence, 'robot_id': None, 'x': ball.x, 'y': ball.y, 'z': ball.z, 'orientation': None, 'area': ball.area
            }
            self.table.append(dict(header, **observation))

        for robot in ssl_wrapper.detection.robots_yellow:
            observation = {
                'color': 'yellow', 'confidence': robot.confidence, 'robot_id': robot.robot_id, 'x': robot.x, 'y': robot.y, 'z': robot.height, 'orientation': robot.orientation, 'area': None
            }
            self.table.append(dict(header, **observation))

        for robot in ssl_wrapper.detection.robots_blue:
            observation = {
                'color': 'blue', 'confidence': robot.confidence, 'robot_id': robot.robot_id, 'x': robot.x, 'y': robot.y, 'z': robot.height, 'orientation': robot.orientation, 'area': None
            }
            self.table.append(dict(header, **observation))


    def getTable(self):
        return self.table

    def refreshTable(self):
        self.table = []

    def tableToRosmsg(self):
        rosmsg = VisionObservations()

        rosmsg.header.stamp = rospy.Time.now()

        for obs in self.table:
            if obs['color'] == 'ball':
                vision_packet   = self.observationToVisionPacket(obs)
                rosmsg.ball.append(vision_packet)

            elif    obs['color'] == self.friend_color:
                id_index  = None
                for i, friend in enumerate(rosmsg.friends):
                    # search detected robot id
                    if friend.robot_id == obs['robot_id']:
                        id_index   = i
                        break

                if id_index == None:
                    # this robot has not detected yet (in this observation)
                    robot_packets    = VisionRobotPackets()
                    robot_packets.robot_id   = obs['robot_id']
                    rosmsg.friends.append(robot_packets)
                    id_index   = len(rosmsg.friends) - 1

                vision_packet   = self.observationToVisionPacket(obs)
                rosmsg.friends[id_index].packets.append(vision_packet)

            else:
                id_index  = None
                for i, enemy in enumerate(rosmsg.enemies):
                    # search detected robot id
                    if enemy.robot_id == obs['robot_id']:
                        id_index   = i
                        break

                if id_index == None:
                    # this robot has not detected yet (in this observation)
                    robot_packets    = VisionRobotPackets()
                    robot_packets.robot_id   = obs['robot_id']
                    rosmsg.enemies.append(robot_packets)
                    id_index   = len(rosmsg.enemies) - 1

                vision_packet   = self.observationToVisionPacket(obs)
                rosmsg.enemies[id_index].packets.append(vision_packet)

        return rosmsg


    def observationToVisionPacket(self, observation):
        vision_packet   = VisionPacket()

        vision_packet.t_capture = observation['t_capture']
        vision_packet.t_sent    = observation['t_sent']
        vision_packet.camera_id = observation['camera_id']
        vision_packet.pose  = self.observationToPose(observation)

        return  vision_packet


    def observationToPose(self, observation):
        pose    = geometry_msgs.msg.Pose()

        if observation['orientation'] == None:
            # if observation is ball
            observation['orientation']  = 0.0

        if self.do_side_invert == True:
            observation['x']    = -observation['x']
            observation['y']    = -observation['y']
            observation['orientation']    = observation['orientation'] + math.pi

        pose.position.x = observation['x'] / 1000
        pose.position.y = observation['y'] / 1000
        pose.position.z = observation['z'] / 1000

        quat = quaternion_from_euler(0.0, 0.0, observation['orientation'])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return  pose

