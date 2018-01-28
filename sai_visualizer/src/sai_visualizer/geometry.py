
class Geometry(object):
    def __init__(self):
        # Qt objects uses width & height
        # SSL-Vision uses length & width :(

        self.FIELD_WIDTH        = 9.0
        self.FIELD_HEIGHT       = 6.0

        # We hope the vision will send these parameters
        self.DIST_TO_WALL       = 0.3
        self.DIST_TO_OUTSIDE    = 0.7

        self.BALL_RADIUS        = 0.043
        self.ROBOT_RADIUS       = 0.18 * 0.5

        self.WORLD_WIDTH    = self.FIELD_WIDTH + self.DIST_TO_OUTSIDE * 2.0
        self.WORLD_HEIGHT   = self.FIELD_HEIGHT + self.DIST_TO_OUTSIDE * 2.0

        self.WALL_WIDTH     = self.FIELD_WIDTH + self.DIST_TO_WALL * 2.0
        self.WALL_HEIGHT    = self.FIELD_HEIGHT + self.DIST_TO_WALL * 2.0

        self.WORLD_H_PER_W = self.WORLD_HEIGHT / self.WORLD_WIDTH
        self.WORLD_W_PER_H = self.WORLD_WIDTH / self.WORLD_HEIGHT


    def set_field(self, width, height):
        self.FIELD_WIDTH = width
        self.FIELD_HEIGHT = height
        self._reflesh()

    def set_dist_to_wall(self, dist_to_wall):
        self.DIST_TO_WALL = dist_to_wall
        self._reflesh


    def set_dist_to_outside(self, dist_to_outside):
        self.DIST_TO_OUTSIDE = dist_to_outside
        self._reflesh()


    def _reflesh(self):
        self.WORLD_WIDTH    = self.FIELD_WIDTH + self.DIST_TO_OUTSIDE * 2.0
        self.WORLD_HEIGHT   = self.FIELD_HEIGHT + self.DIST_TO_OUTSIDE * 2.0

        self.WALL_WIDTH     = self.FIELD_WIDTH + self.DIST_TO_WALL * 2.0
        self.WALL_HEIGHT    = self.FIELD_HEIGHT + self.DIST_TO_WALL * 2.0

        self.WORLD_H_PER_W = self.WORLD_HEIGHT / self.WORLD_WIDTH
        self.WORLD_W_PER_H = self.WORLD_WIDTH / self.WORLD_HEIGHT
