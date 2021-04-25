#!/usr/bin/env python

from transitions import Machine 	# installed by sudo pip install transitions, allows transititons functions

class BaxterKeurigState(object):

	# define states
	states = ['init_lid_check', 'power_off', 'lift_to_begin', 'check_if_kuerig_cup_exists', 'no_leftover_kuerig_cup', 'ready']
	
	# define transitions
	transitions = [
		{ 'trigger': 'lid_open',         	 'source': 'init_lid_check', 			 'dest': 'init_lid_check' },
		{ 'trigger': 'lid_closed',       	 'source': 'init_lid_check', 			 'dest': 'power_off' },
		{ 'trigger': 'rec_begin',        	 'source': 'power_off',      			 'dest': 'lift_to_begin' },
		{ 'trigger': 'last_state_not_READY', 'source': 'lift_to_begin',              'dest': 'check_if_kuerig_cup_exists' },
		{ 'trigger': 'last_state_READY', 	 'source': 'lift_to_begin', 			 'dest': 'lift_to_begin' },
		{ 'trigger': 'rec_kcup',         	 'source': 'check_if_kuerig_cup_exists', 'dest': 'lift_to_begin' },
		{ 'trigger': 'unrec_kcup', 		 	 'source': 'check_if_kuerig_cup_exists', 'dest': 'no_leftover_kuerig_cup' },
		{ 'trigger': 'rec_ready', 	 		 'source': 'no_leftover_kuerig_cup', 	 'dest': 'ready' },
		{ 'trigger': 'rec_begin', 	 		 'source': 'ready', 	 				 'dest': 'lift_to_begin' }
	]
	
	def __init__(self, name):
		
		# set name
		self.name = name
		
		# Set lastState
		self.lastState = ''
		
		# initialize state machine
		self.machine = Machine(model=self, states=BaxterKeurigState.states, transitions=BaxterKeurigState.transitions, initial='lift_to_begin')
		
