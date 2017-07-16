from geometry_msgs.msg import Point

GoalFriend = Point()
GoalFriend.x, GoalFriend.y = -4.5, 0.0
GoalEnemy = Point()
GoalEnemy.x, GoalEnemy.y = 4.5, 0.0
PenaltyEnemy = Point(3.5, 0.0, 0)

GoalSize = 1.0
GoalHalfSize = GoalSize * 0.5

RobotRadius = 0.09
