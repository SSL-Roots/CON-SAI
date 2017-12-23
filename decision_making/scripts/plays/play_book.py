
from play_halt import PlayHalt
from play_outside import PlayOutside
from play_stop import PlayStop
from play_pre_kickoff import PlayPreKickoff
from play_kickoff_start import PlayKickoffStart
from play_force_start import PlayForceStart
from play_inplay import PlayInPlay
from play_indirect import PlayIndirect
from play_direct import PlayDirect
from play_their_pre_kickoff import PlayTheirPreKickoff
from play_their_kickoff_start import PlayTheirKickoffStart
from play_their_indirect import PlayTheirIndirect
from play_their_direct import PlayTheirDirect
from play_their_pre_penalty import PlayTheirPrePenalty
from play_their_penalty_start import PlayTheirPenaltyStart

class PlayBook(object):

    book = [] 
    book.append(PlayHalt())
    book.append(PlayOutside())
    book.append(PlayStop())
    book.append(PlayPreKickoff())
    book.append(PlayKickoffStart())
    book.append(PlayForceStart())
    book.append(PlayInPlay())
    book.append(PlayIndirect())
    book.append(PlayDirect())
    book.append(PlayTheirPreKickoff())
    book.append(PlayTheirKickoffStart())
    book.append(PlayTheirIndirect())
    book.append(PlayTheirDirect())
    book.append(PlayTheirPrePenalty())
    book.append(PlayTheirPenaltyStart())
