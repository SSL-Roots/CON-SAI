import rospy
import tf

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point

class ControlData():
    def __init__(self):
        self._id = None
        self._has_id = False
        self.is_velocity_control = False
        self.target_pose = PoseStamped()
        self.target_velocity = Twist()
        self.kick_velocity = 0.0
        self._base = None
        self.setTargetVelocity(0.0, 0.0, 0.0)
        self._avoid_ball = False
        self._kick_target = Point()
        self._do_chip = False
        self._dribble_power = 0.0
            
    def set_id(self,id):
        self._id = id
        self._has_id = True
        self._base = "friend_"+str(id)+"/base_link"

    def delete_id(self):
        self._id = None
        self._has_id = False
        self._base = None
        self.setTargetVelocity(0.0, 0.0, 0.0)
        self.setKickVelocity(0.0)
        self._do_chip = False
        self._dribble_power = 0.0

    def has_id(self):
        return self._has_id

    def get_id(self):
        return self._id

    def get_base(self):
        return self._base

    def avoid_ball(self):
        return self._avoid_ball

    def do_chip(self):
        return self._do_chip

    def get_dribble_power(self):
        return self._dribble_power

    def setControlMode(self, mode):
        if mode == 'position':
            self.is_velocity_control  = False
        else:
            self.is_velocity_control  = True

    def setTargetVelocity(self, x, y, yaw):
        self.target_velocity.linear.x = x;
        self.target_velocity.linear.y = y;
        self.target_velocity.angular.z = yaw;
        self.is_velocity_control  = True

    def setTargetPose(self, x, y, yaw, frame_id, avoid_ball = False):
        self.target_pose.header.stamp = rospy.Time()
        self.target_pose.header.frame_id  = frame_id

        self.target_pose.pose.position.x   = x;
        self.target_pose.pose.position.y   = y;

        quaternion  = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.target_pose.pose.orientation.x    = quaternion[0]
        self.target_pose.pose.orientation.y    = quaternion[1]
        self.target_pose.pose.orientation.z    = quaternion[2]
        self.target_pose.pose.orientation.w    = quaternion[3]

        self._avoid_ball = avoid_ball
        self.is_velocity_control  = False

    def setKickVelocity(self, vel, dribble_power=0.0, do_chip=False):
        self.kick_velocity = vel
        self._dribble_power = dribble_power
        self._do_chip = do_chip

    def setKickTarget(self, x, y):
        self._kick_target.x = x
        self._kick_target.y = y

    def getKickTarget(self):
        return self._kick_target

class GlobalInfo():
    ref_command = 0
    team_color= "yellow"
    
    ball = Odometry()

    controls = []

    friendIDs = []
    enemyIDs = []

    controls.append(ControlData())
    controls.append(ControlData())
    controls.append(ControlData())
    controls.append(ControlData())
    controls.append(ControlData())
    controls.append(ControlData())

    tf_listener = tf.TransformListener()

    nearestFriendID = None
    nearestEnemyID = None
    isOffensive = True
    isInOurGoal = False
    isInDefenderArea = False

    @classmethod
    def setFriendColor(cls,color):
        GlobalInfo.team_color = color

    @classmethod
    def setRefCommand(cls,command):
        GlobalInfo.ref_command = command

    @classmethod
    def setBallInfo(cls,odom):
        GlobalInfo.ball = odom


class GlobalData():
    """Store data as global variable"""
    ref_command = 0
    team_color = 'yellow'

    ball    = Odometry()

    is_velocity_control = False
    target_pose = PoseStamped()
    target_velocity = Twist()
    kick_velocity = 0.0;

#     tf_listener = tf.TransformListener()

    @classmethod
    def setFriendColor(cls, color):
        GlobalData.team_color   = color

    @classmethod
    def setRefCommand(cls, command):
        GlobalData.ref_command  = command

    @classmethod
    def setBallInfo(cls, odom):
        GlobalData.ball = odom

    @classmethod
    def setControlMode(cls, mode):
        if mode == 'position':
            GlobalData.is_velocity_control  = False
        else:
            GlobalData.is_velocity_control  = True

    @classmethod
    def setTargetVelocity(cls, x, y, yaw):
        GlobalData.target_velocity.linear.x = x;
        GlobalData.target_velocity.linear.y = y;
        GlobalData.target_velocity.angular.z = yaw;
        GlobalData.is_velocity_control  = True

    @classmethod
    def setTargetPose(cls, x, y, yaw, frame_id):
        GlobalData.target_pose.header.stamp = rospy.Time()
        GlobalData.target_pose.header.frame_id  = frame_id

        GlobalData.target_pose.pose.position.x   = x;
        GlobalData.target_pose.pose.position.y   = y;

        quaternion  = tf.transformations.quaternion_from_euler(0, 0, yaw)
        GlobalData.target_pose.pose.orientation.x    = quaternion[0]
        GlobalData.target_pose.pose.orientation.y    = quaternion[1]
        GlobalData.target_pose.pose.orientation.z    = quaternion[2]
        GlobalData.target_pose.pose.orientation.w    = quaternion[3]

        GlobalData.is_velocity_control  = False

    @classmethod
    def setKickVelocity(cls, vel):
        GlobalData.kick_velocity = vel
