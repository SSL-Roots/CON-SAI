
from play_halt import PlayHalt
from play_stop import PlayStop
from play_pre_kickoff import PlayPreKickoff
from play_kickoff import PlayKickoff
from play_force_start import PlayForceStart
from play_inplay import PlayInPlay

class PlayBook(object):

    book = [] 
    book.append(PlayHalt())
    book.append(PlayStop())
    book.append(PlayPreKickoff())
    book.append(PlayKickoff())
    book.append(PlayForceStart())
    book.append(PlayInPlay())
