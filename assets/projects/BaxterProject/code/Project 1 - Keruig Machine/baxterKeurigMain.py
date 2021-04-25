#!/usr/bin/env python

from stateFunctions import InitCheck_State, PowerOff_State, Begin_State, CheckKCup_State, No_LeftOver_KCup_State, Ready_State		# import functions for moving and featture detection for each state
from stateBaxterKeurig import BaxterKeurigState			# Import class that contains the state info for Baxter

def main():
	baxter = BaxterKeurigState("Baxter")
	
	while True:
		if baxter.state == 'init_lid_check':
			InitCheck_State(baxter)
		elif baxter.state == 'power_off':
			PowerOff_State(baxter)
		elif baxter.state == 'lift_to_begin':
			Begin_State(baxter)
		elif baxter.state == 'check_if_kuerig_cup_exists':
			CheckKCup_State(baxter)
		elif baxter.state == 'no_leftover_kuerig_cup':
			No_LeftOver_KCup_State(baxter)
		elif baxter.state == 'ready':
			Ready_State(baxter)
	
if __name__ == "__main__":
	main()
