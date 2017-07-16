from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *
import  referee_pb2

# from GlobalData import *
from GlobalData import GlobalInfo


class RefCommandSuper(Task):
    """docstring for RefCommandSuper"""
    def __init__(self, name, command):
        super(RefCommandSuper, self).__init__(name)
        self.command = command

    def run(self):
        self.announce()
#         if GlobalData.ref_command == self.command:
        if GlobalInfo.ref_command == self.command:
            return  TaskStatus.RUNNING
        else:
            return  TaskStatus.FAILURE

class IsHALT(RefCommandSuper):
    def __init__(self, name):
        super(IsHALT, self).__init__(name, referee_pb2.SSL_Referee.HALT)

class IsSTOP(RefCommandSuper):
    def __init__(self, name):
        super(IsSTOP, self).__init__(name, referee_pb2.SSL_Referee.STOP)

class IsNORMAL_START(RefCommandSuper):
    def __init__(self, name):
        super(IsNORMAL_START, self).__init__(name, referee_pb2.SSL_Referee.NORMAL_START)

class IsFORCE_START(RefCommandSuper):
    def __init__(self, name):
        super(IsFORCE_START, self).__init__(name, referee_pb2.SSL_Referee.FORCE_START)

class IsKICKOFF_FRIEND(RefCommandSuper):
    def __init__(self, name):
#         if GlobalData.team_color == 'yellow':
        if GlobalInfo.team_color == "yellow":
            command = referee_pb2.SSL_Referee.PREPARE_KICKOFF_YELLOW
        else:
            command = referee_pb2.SSL_Referee.PREPARE_KICKOFF_BLUE
        super(IsKICKOFF_FRIEND, self).__init__(name, command)

class IsKICKOFF_ENEMY(RefCommandSuper):
    def __init__(self, name):
#         if GlobalData.team_color == 'yellow':
        if GlobalInfo.team_color == "yellow":
            command = referee_pb2.SSL_Referee.PREPARE_KICKOFF_BLUE
        else:
            command = referee_pb2.SSL_Referee.PREPARE_KICKOFF_YELLOW
        super(IsKICKOFF_ENEMY, self).__init__(name, command)

class IsPENALTY_FRIEND(RefCommandSuper):
    def __init__(self, name):
#         if GlobalData.team_color == 'yellow':
        if GlobalInfo.team_color == "yellow":
            command = referee_pb2.SSL_Referee.PREPARE_PENALTY_YELLOW
        else:
            command = referee_pb2.SSL_Referee.PREPARE_PENALTY_BLUE
        super(IsPENALTY_FRIEND, self).__init__(name, command)

class IsPENALTY_ENEMY(RefCommandSuper):
    def __init__(self, name):
#         if GlobalData.team_color == 'yellow':
        if GlobalInfo.team_color == "yellow":
            command = referee_pb2.SSL_Referee.PREPARE_PENALTY_BLUE
        else:
            command = referee_pb2.SSL_Referee.PREPARE_PENALTY_YELLOW
        super(IsPENALTY_ENEMY, self).__init__(name, command)

class IsDIRECT_FREE_FRIEND(RefCommandSuper):
    def __init__(self, name):
#         if GlobalData.team_color == 'yellow':
        if GlobalInfo.team_color == "yellow":
            command = referee_pb2.SSL_Referee.DIRECT_FREE_YELLOW
        else:
            command = referee_pb2.SSL_Referee.DIRECT_FREE_BLUE
        super(IsDIRECT_FREE_FRIEND, self).__init__(name, command)

class IsDIRECT_FREE_ENEMY(RefCommandSuper):
    def __init__(self, name):
#         if GlobalData.team_color == 'yellow':
        if GlobalInfo.team_color == "yellow":
            command = referee_pb2.SSL_Referee.DIRECT_FREE_BLUE
        else:
            command = referee_pb2.SSL_Referee.DIRECT_FREE_YELLOW
        super(IsDIRECT_FREE_ENEMY, self).__init__(name, command)

class IsINDIRECT_FREE_FRIEND(RefCommandSuper):
    def __init__(self, name):
#         if GlobalData.team_color == 'yellow':
        if GlobalInfo.team_color == "yellow":
            command = referee_pb2.SSL_Referee.INDIRECT_FREE_YELLOW
        else:
            command = referee_pb2.SSL_Referee.INDIRECT_FREE_BLUE
        super(IsINDIRECT_FREE_FRIEND, self).__init__(name, command)

class IsINDIRECT_FREE_ENEMY(RefCommandSuper):
    def __init__(self, name):
#         if GlobalData.team_color == 'yellow':
        if GlobalInfo.team_color == "yellow":
            command = referee_pb2.SSL_Referee.INDIRECT_FREE_BLUE
        else:
            command = referee_pb2.SSL_Referee.INDIRECT_FREE_YELLOW
        super(IsINDIRECT_FREE_ENEMY, self).__init__(name, command)

class IsTIMEOUT_FRIEND(RefCommandSuper):
    def __init__(self, name):
#         if GlobalData.team_color == 'yellow':
        if GlobalInfo.team_color == "yellow":
            command = referee_pb2.SSL_Referee.TIMEOUT_YELLOW
        else:
            command = referee_pb2.SSL_Referee.TIMEOUT_BLUE
        super(IsTIMEOUT_FRIEND, self).__init__(name, command)

class IsTIMEOUT_ENEMY(RefCommandSuper):
    def __init__(self, name):
#         if GlobalData.team_color == 'yellow':
        if GlobalInfo.team_color == "yellow":
            command = referee_pb2.SSL_Referee.TIMEOUT_BLUE
        else:
            command = referee_pb2.SSL_Referee.TIMEOUT_YELLOW
        super(IsTIMEOUT_ENEMY, self).__init__(name, command)

class IsGOAL_FRIEND(RefCommandSuper):
    def __init__(self, name):
#         if GlobalData.team_color == 'yellow':
        if GlobalInfo.team_color == "yellow":
            command = referee_pb2.SSL_Referee.GOAL_YELLOW
        else:
            command = referee_pb2.SSL_Referee.GOAL_BLUE
        super(IsGOAL_FRIEND, self).__init__(name, command)

class IsGOAL_ENEMY(RefCommandSuper):
    def __init__(self, name):
#         if GlobalData.team_color == 'yellow':
        if GlobalInfo.team_color == "yellow":
            command = referee_pb2.SSL_Referee.GOAL_BLUE
        else:
            command = referee_pb2.SSL_Referee.GOAL_YELLOW
        super(IsGOAL_ENEMY, self).__init__(name, command)
