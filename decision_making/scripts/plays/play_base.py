
from role_base import Role

import sys,os
sys.path.append(os.pardir)
import constants

class Play(object):
    def __init__(self, name):
        self.name = name
        self.applicable = None
        self.done = None
        self.done_aborted = None
        self.recent_done = None
        self.recent_done_aborted = None
        self.timeout = None
        self.assignment_type = None
        self.formation_type = None
        self.roles = []
        for i in range(constants.ROBOT_NUM):
            role_name = "Role_" + str(i)
            self.roles.append(Role(role_name))
