from geometry_msgs.msg import Point

poses = {
        'CONST_OUR_GOAL' : Point(-4.5, 0.0, 0),
        'CONST_THEIR_GOAL' : Point(4.5, 0.0, 0),
        'CONST_PENALTY_ENEMY' : Point(3.5, 0.0, 0)}

GoalSize = 1.0
GoalHalfSize = GoalSize * 0.5

RobotRadius = 0.09
BallRadius = 0.0215
