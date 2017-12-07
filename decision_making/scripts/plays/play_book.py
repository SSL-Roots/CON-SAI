
from play_halt import PlayHalt
from play_stop import PlayStop
from play_pre_kickoff import PlayPreKickoff

class PlayBook(object):

    book = [] 
    book.append(PlayHalt())
    book.append(PlayStop())
    book.append(PlayPreKickoff())
