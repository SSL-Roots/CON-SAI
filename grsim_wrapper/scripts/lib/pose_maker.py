
# PoseMaker converts vision packets to PoseArray
# Inputs are vision packets SSL_DetectionBall, SSL_DetectionRobot
# Output is PoseArray

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion

class PoseMaker(object):

    def __init__(self, time, frame_id):
        super(PoseMaker, self).__init__()
        self.pose_array = PoseArray()
        self.pose_array.header.stamp = time
        self.pose_array.header.frame_id = frame_id


    def get(self):
        return self.pose_array


    def add(self, pose_detected):
        self.pose_array.poses.append(self._msg_to_pose(pose_detected))
        
        return True


    def clear_poses(self):
        self.pose_array.poses = []


    def len(self):
        return len(self.pose_array.poses)


    def _msg_to_pose(self, detection_pose):
        pose = Pose()

        pose.position.x = detection_pose.x / 1000
        pose.position.y = detection_pose.y / 1000

        if hasattr(detection_pose, 'orientation'):
            quat_tuple = quaternion_from_euler(0.0, 0.0, detection_pose.orientation)
            pose.orientation = Quaternion(*quat_tuple)

        return pose

