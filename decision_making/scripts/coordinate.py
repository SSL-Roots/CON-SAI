
import tool
import constants
from world_model import WorldModel

from consai_msgs.msg import Pose

class Coordinate(object):

    def __init__(self):
        self.pose = Pose() # pos_x, pos_y, thta

        self._base = None
        self._target = None
        self._update_func = None

        # interpose
        self._to_dist = None
        self._from_dist = None


    def update(self):
        result = False
        if self._update_func:
            result = self._update_func()

        return result

    def set_interpose(self, base="CONST_OUR_GOAL", target="Ball", to_dist=None, from_dist=None):

        self._base = base
        self._target = target
        self._to_dist = to_dist
        self._from_dist = from_dist

        self._update_func = self._update_interpose
    

    def _update_interpose(self):
        base_pose = WorldModel.get_pose(self._base)
        target_pose = WorldModel.get_pose(self._target)

        if base_pose is None or target_pose is None:
            return False

        angle_to_target = tool.getAngle(base_pose, target_pose)
        
        interposed_pose = Pose(0, 0, 0)
        if self._to_dist:
            trans = tool.Trans(base_pose, angle_to_target)
            tr_interposed_pose = Pose(self._to_dist, 0.0, 0)
            interposed_pose = trans.invertedTransform(tr_interposed_pose)
        elif self._from_dist:
            angle_to_base = tool.getAngle(target_pose, base_pose)
            trans = tool.Trans(target_pose, angle_to_base)
            tr_interposed_pose = Pose(self._from_dist, 0.0, 0)
            interposed_pose = trans.invertedTransform(tr_interposed_pose)

        interposed_pose.theta = angle_to_target

        self.pose = interposed_pose
        
        return True

