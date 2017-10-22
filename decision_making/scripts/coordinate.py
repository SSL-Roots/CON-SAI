
import tool
import constants
from world_model import WorldModel

from geometry_msgs.msg import Point

class Coordinate(object):

    def __init__(self):
        self.pose = (0, 0, 0) # pos_x, pos_y, thta

        self._base = None
        self._target = None
        self._update_func = None

        # interpose
        self._to_dist = None
        self._from_dist = None


    def update(self):
        if self._update_func:
            self._update_func()


    def set_interpose(self, base="CONST_OUR_GOAL", target="Ball", to_dist=None, from_dist=None):

        self._base = base
        self._target = target
        self._to_dist = to_dist
        self._from_dist = from_dist

        self._update_func = self._update_interpose
    

    def _update_interpose(self):
        base_pos = WorldModel.get_pose(self._base)
        target_pos = WorldModel.get_pose(self._target)

        angle_to_target = tool.getAngle(base_pos, target_pos)
        
        interposed_pos = Point(0, 0, 0)
        if self._to_dist:
            trans = tool.Trans(base_pos, angle_to_target)
            tr_interposed_pos = Point(self._to_dist, 0.0, 0)
            interposed_pos = trans.invertedTransform(tr_interposed_pos)
        elif self._from_dist:
            angle_to_base = tool.getAngle(target_pos, base_pos)
            trans = tool.Trans(target_pos, angle_to_base)
            tr_interposed_pos = Point(self._from_dist, 0.0, 0)
            interposed_pos = trans.invertedTransform(tr_interposed_pos)

        self.pose = interposed_pos.x, interposed_pos.y, angle_to_target

