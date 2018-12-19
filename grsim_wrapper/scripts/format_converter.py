
import math

from proto import messages_robocup_ssl_wrapper_pb2

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

from consai_msgs.msg import VisionPacket, VisionData


class FormatConverter:

    def __init__(self, friend_color, do_side_invert, id_max):
        self.table = []
        self.friend_color = friend_color
        self.do_side_invert = do_side_invert

        self._ball_packet = VisionPacket()
        self._friend_packets = []
        self._enemy_packets = []

        self._ID_MAX = id_max

        for robot_id in range(self._ID_MAX):
            self._friend_packets.append(VisionPacket())
            self._friend_packets[robot_id].robot_id = robot_id
            self._enemy_packets.append(VisionPacket())
            self._enemy_packets[robot_id].robot_id = robot_id

        self._TO_METER = 0.001


    def convert_protobuf(self, protobuf_binary):
        ssl_wrapper = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
        ssl_wrapper.ParseFromString(protobuf_binary)

        t_capture = ssl_wrapper.detection.t_capture
        t_sent = ssl_wrapper.detection.t_sent
        camera_id = ssl_wrapper.detection.camera_id

        for ball in ssl_wrapper.detection.balls:
            data = VisionData()
            data.t_capture = t_capture
            data.t_sent = t_sent
            data.camera_id = camera_id
            data.pose = self._vision_ball_to_pose(
                    ball.x, ball.y, ball.z)
            self._ball_packet.data.append(data)


        friend_robots = []
        enemy_robots = []
        if self.friend_color == 'blue':
            friend_robots = ssl_wrapper.detection.robots_blue
            enemy_robots = ssl_wrapper.detection.robots_yellow
        else:
            friend_robots = ssl_wrapper.detection.robots_yellow
            enemy_robots = ssl_wrapper.detection.robots_blue

        for robot in friend_robots:
            data = VisionData()
            data.t_capture = t_capture
            data.t_sent = t_sent
            data.camera_id = camera_id
            data.pose = self._vision_robot_to_pose(
                    robot.x, robot.y, robot.height, robot.orientation)
            robot_id = robot.robot_id
            self._friend_packets[robot_id].data.append(data)

        for robot in enemy_robots:
            data = VisionData()
            data.t_capture = t_capture
            data.t_sent = t_sent
            data.camera_id = camera_id
            data.pose = self._vision_robot_to_pose(
                    robot.x, robot.y, robot.height, robot.orientation)
            robot_id = robot.robot_id
            self._enemy_packets[robot_id].data.append(data)


    def buffer_clear(self):
        self._ball_packet.data = []

        for robot_id in range(len(self._friend_packets)):
            self._friend_packets[robot_id].data = []

        for robot_id in range(len(self._enemy_packets)):
            self._enemy_packets[robot_id].data = []
    

    def get_ball_packet(self):
        return self._ball_packet

    def get_friend_packets(self):
        return self._friend_packets

    def get_enemy_packets(self):
        return self._enemy_packets


    def _vision_ball_to_pose(self, pos_x, pos_y, pos_z):
        pose = Pose()

        if self.do_side_invert:
            pos_x = -pos_x
            pos_y = -pos_y

        pose.position.x = pos_x * self._TO_METER
        pose.position.y = pos_y * self._TO_METER
        pose.position.z = pos_z * self._TO_METER

        return pose


    def _vision_robot_to_pose(self, pos_x, pos_y, height, orientation):
        pose = Pose()

        if self.do_side_invert:
            pos_x = -pos_x
            pos_y = -pos_y
            orientation += math.pi

        pose.position.x = pos_x * self._TO_METER
        pose.position.y = pos_y * self._TO_METER
        pose.position.z = height * self._TO_METER

        quat = quaternion_from_euler(0.0, 0.0, orientation)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose

