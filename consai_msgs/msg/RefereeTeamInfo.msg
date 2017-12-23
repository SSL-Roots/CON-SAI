# This msg file is copied from 'referee.proto'

# The team's name (empty string if operator has not typed anything).
string name
# The number of goals scored by the team during normal play and overtime.
uint32 score
# The number of red cards issued to the team since the beginning of the game.
uint32 red_cards
# The amount of time (in microseconds) left on each yellow card issued to the team.
# If no yellow cards are issued, this array has no elements.
# Otherwise, times are ordered from smallest to largest.
uint32[] yellow_card_times
# The total number of yellow cards ever issued to the team.
uint32 yellow_cards
# The number of timeouts this team can still call.
# If in a timeout right now, that timeout is excluded.
uint32 timeouts
# The number of microseconds of timeout this team can use.
uint32 timeout_time
# The pattern number of this team's goalie.
uint32 goalie
