#!/usr/bin/env python

import random
from abc import ABCMeta, abstractmethod, abstractproperty

import rospy


class BlackBoard(object):
    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

class State(object):
    __metaclass__ = ABCMeta

    #_shared = {}
    board = BlackBoard()

    def __init__(self):
        self.transitions = {}

    @abstractproperty
    def name(self):
        pass

    @abstractproperty
    def outcomes(self):
        pass
    
    @abstractmethod
    def setup(self):
        pass
    
    @abstractmethod
    def execute(self):
        pass

    # def __getitem__(self, key):
    #     return self._shared[key]

    # def __setitem__(self, key, value):
    #     self._shared[key] = value

    def __repr__(self):
        return self.name

    def __eq__(self, other):
        return self.name == other.name

    def __ne__(self, other):
        return not self == other



class StateMachine(State):

    name = 'DUMMY_NAME'

    def __init__(self, name=None):
        super(StateMachine, self).__init__()

        if name:
            self.name = name
        else:
            self.name = 'STATE_MACHINE_{}'.format(random.randint(10000, 99999))

        self.states = {}

    @property
    def outcomes(self):
        return {state.outcomes for state in self.states.values()}

    def setup(self):
        pass

    def execute(self):
        while not rospy.is_shutdown():
            outcome = self.state.execute()
            if outcome in self.state.transitions:
                next_state_name = self.state.transitions[outcome]
                next_state = self.states[next_state_name]
                rospy.loginfo('{}: Changing state {} -> {}'.format(rospy.get_name(), self.state, next_state))
                self.state = next_state
                self.state.setup()
            elif outcome == 'running':
                rospy.loginfo_throttle(1, '{}: Currently running state {}'.format(rospy.get_name(), self.state))
            else:
                return outcome
            
            self.board.rate.sleep()

    def add_state(self, state):
        if len(self.states) == 0:
            self.state = state
        self.states[state.name] = state


