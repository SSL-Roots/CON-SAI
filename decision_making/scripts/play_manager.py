
import rospy

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from world_model import WorldModel
from plays.play_book import PlayBook
from plays.play_dummy import PlayDummy


class PlayManager(object):

    def __init__(self):
        self._play_termination = True
        self._play = PlayDummy()
        self._play_past_time = 0.0


    def update(self):
        WorldModel.update_world()

        self._select_play()

        WorldModel.update_assignments()
        rospy.loginfo(WorldModel.assignments)

        self._execute_play()

        self._evaluate_play()

    
    def _select_play(self):
        if self._play_termination:

            # Extract possible plays from playbook
            possible_plays = []
            for play in PlayBook.book:
                if WorldModel.situations[play.applicalbe]:
                    possible_plays.append(play)

            # TODO(Asit) select a play randomly
            if possible_plays:
                self._play = possible_plays[0]
            else:
                self._play = PlayDummy()

            self._play_past_time = rospy.get_time()
            self._play_termination = False


    def _execute_play(self):
        for role in self._play.roles:
            role.behavior.run()

        text = "execute : " + self._play.name
        rospy.loginfo(text)


    def _evaluate_play(self):
        for role in self._play.roles:
            status = role.behavior.get_status()

            if role.loop_enable:
                if status != TaskStatus.RUNNING:
                    role.behavior.reset()
            else:
                if status != TaskStatus.RUNNING:
                    self._play_termination = True
                    
        if self._play.timeout:
            if rospy.get_time - self._play_past_time > self._play.timeout:
                self._play_termination = True

        # TODO(Asit) write recent_done termination 
        if (self._play.done and WorldModel.situations[self._play.done]) or \
                (self._play.done_aborted and 
                        not WorldModel.situations[self._play.done_aborted]):

            self._play_termination = True



