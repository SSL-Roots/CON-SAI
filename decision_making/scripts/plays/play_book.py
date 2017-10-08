
from play_halt import PlayHalt
from play_stop import PlayStop

class PlayBook(object):

    book = [] 
    book.append(PlayHalt())
    book.append(PlayStop())
