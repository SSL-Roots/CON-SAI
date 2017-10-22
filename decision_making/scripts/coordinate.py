
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

        self._target_id = None
        self._base_id = None
        self._target_pose_dict = {
                'OUR_GOAL' : constants.GoalFriend,
                'BALL' : Point(),
                'ENEMY' : Point(),
                'FRIEND' : Point()}

        self._base_pose_dict = {
                'OUR_GOAL' : constants.GoalFriend,
                'BALL' : Point(),
                'ENEMY' : Point(),
                'FRIEND' : Point()}

        # interpose
        self._to_dist = None
        self._from_dist = None

    def update(self):
        self._update_poses()

        if self._update_func:
            self._update_func()


    def set_interpose(self, base="OUR_GOAL", target="BALL", 
            base_id=None, target_id=None, to_dist=None, from_dist=None):

        self._base = base
        self._target = target
        self._base_id = base_id
        self._target_id = target_id
        self._to_dist = to_dist
        self._from_dist = from_dist

        self._update_func = self._update_interpose
    

    def _update_poses(self):
        ball_pose = WorldModel.ball_odom.pose.pose.position
        self._target_pose_dict['BALL'] = Point(ball_pose.x, ball_pose.y, 0)
        self._base_pose_dict['BALL'] = Point(ball_pose.x, ball_pose.y, 0)

        # set Role_* to target/base _id to get friend robot pose
        if self._target == 'FRIEND':
            self._target_pose_dict['FRIEND'] = WorldModel.get_role_pose(self._target_id)
        elif self._target == 'ENEMY':
            self._target_pose_dict['ENEMY'] = WorldModel.get_enemy_pose(self._target_id)

        if self._base == 'FRIEND':
            self._base_pose_dict['FRIEND'] = WorldModel.get_role_pose(self._base_id)
        elif self._base == 'ENEMY':
            self._base_pose_dict['ENEMY'] = WorldModel.get_enemy_pose(self._base_id)


    def _update_interpose(self):
        base_pos = self._base_pose_dict[self._base]
        target_pos = self._target_pose_dict[self._target]

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

        
        
