
import rospy
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from consai_msgs.msg import Pose


class Command(object):
    def __init__(self):
        self.target_pose = PoseStamped()
        self.target_velocity = Twist()
        self.aim = Pose()
        self.kick_power = 0.0
        self.dribble_power = 0.0
        self.velocity_control_enable = False
        self.chip_enable = False
        self.navigation_enable = True

    
    def set_target_pose(self, x, y, yaw, frame_id):
        self.target_pose.header.stamp = rospy.Time()
        self.target_pose.header.frame_id = frame_id

        self.target_pose.pose.position.x = x;
        self.target_pose.pose.position.y = y;

        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        self.target_pose.pose.orientation.x = quaternion[0]
        self.target_pose.pose.orientation.y = quaternion[1]
        self.target_pose.pose.orientation.z = quaternion[2]
        self.target_pose.pose.orientation.w = quaternion[3]

        self.velocity_control_enable = False


    def set_target_velocity(self, vx, vy, vyaw):
        self.target_velocity.linear.x = vx;
        self.target_velocity.linear.y = vy;
        self.target_velocity.angular.z = vyaw;
        self.velocity_control_enable = True

    
    def set_aim(self, x, y):
        self.aim.x = x
        self.aim.y = y


    def reset_adjustments(self):
        self.kick_power = 0
        self.chip_enable = False
        self.dribble_power = 0
        self.navigation_enable = True

