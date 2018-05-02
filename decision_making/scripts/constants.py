from geometry_msgs.msg import Point
import rospy

ROBOT_NUM = 8
ID_MAX = 12

FieldX = 9.0
FieldY = 6.0

PenaltyX = 3.3
PenaltyY = 1.2

FieldHalfX = FieldX * 0.5
FieldHalfY = FieldY * 0.5

DefenceHalfStreach = 0.25
DefenceLength = 1.0

GoalSize = 1.0
GoalHalfSize = GoalSize * 0.5

RobotRadius = 0.09
BallRadius = 0.0215

poses = {
        'CONST_OUR_GOAL' : Point(-FieldHalfX, 0.0, 0),
        'CONST_OUR_GOAL_UPPER' : Point(-FieldHalfX, DefenceHalfStreach, 0),
        'CONST_OUR_GOAL_LOWER' : Point(-FieldHalfX, -DefenceHalfStreach, 0),
        'CONST_THEIR_GOAL' : Point(FieldHalfX, 0.0, 0),
        'CONST_THEIR_GOAL_UPPER' : Point(FieldHalfX, DefenceHalfStreach, 0),
        'CONST_THEIR_GOAL_LOWER' : Point(FieldHalfX, -DefenceHalfStreach, 0),
        'CONST_THEIR_PENALTY_MARK' : Point(FieldHalfX - DefenceLength, 0.0, 0)
        }

shoot_targets = ['CONST_THEIR_GOAL', 'CONST_THEIR_GOAL_UPPER', 'CONST_THEIR_GOAL_LOWER']


def set_field(length, width):
    global FieldX, FieldY, FieldHalfX, FieldHalfY

    FieldX = length
    FieldY = width

    FieldHalfX = FieldX * 0.5
    FieldHalfY = FieldY * 0.5

    _set_poses()

def set_penalty(x, y):
    global PenaltyX, PenaltyY

    PenaltyX = x
    PenaltyY = y

def _set_poses():
    global poses

    poses = {
        'CONST_OUR_GOAL' : Point(-FieldHalfX, 0.0, 0),
        'CONST_OUR_GOAL_UPPER' : Point(-FieldHalfX, DefenceHalfStreach, 0),
        'CONST_OUR_GOAL_LOWER' : Point(-FieldHalfX, -DefenceHalfStreach, 0),
        'CONST_THEIR_GOAL' : Point(FieldHalfX, 0.0, 0),
        'CONST_THEIR_GOAL_UPPER' : Point(FieldHalfX, DefenceHalfStreach, 0),
        'CONST_THEIR_GOAL_LOWER' : Point(FieldHalfX, -DefenceHalfStreach, 0),
        'CONST_THEIR_PENALTY_MARK' : Point(FieldHalfX - DefenceLength, 0.0, 0)
        }

