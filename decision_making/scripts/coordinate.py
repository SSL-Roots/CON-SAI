
import tool
import constants
from world_model import WorldModel

from geometry_msgs.msg import Point

class Coordinate(object):

    def __init__(self):
        self._base = None
        self._target = None
        self.pose = (0, 0, 0) # pos_x, pos_y, thta

        self._update_func = None

        # interpose
        self._to_dist = None
        self._from_dist = None

    def update(self):
        if self._update_func:
            self._update_func()


    def set_interpose(self, base="OUR_GOAL", target="BALL", to_dist=None, from_dist=None):
        self._base = base
        self._target = target
        self._to_dist = to_dist
        self._from_dist = from_dist

        self._update_func = self._update_interpose
    

    def _update_interpose(self):
        goal_pos = constants.GoalFriend
        ball_pos = WorldModel.ball_odom.pose.pose.position

        angle_to_ball = tool.getAngle(goal_pos, ball_pos)
        trans = tool.Trans(goal_pos, angle_to_ball)
        tr_interposed_pos = Point(self._to_dist, 0.0, 0)
        interposed_pos = trans.invertedTransform(tr_interposed_pos)

        self.pose = interposed_pos.x, interposed_pos.y, angle_to_ball

        
        
