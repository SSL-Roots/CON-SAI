from geometry_msgs.msg import Point

FieldHalfX = 4.5
FieldHalfY = 3.0

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
