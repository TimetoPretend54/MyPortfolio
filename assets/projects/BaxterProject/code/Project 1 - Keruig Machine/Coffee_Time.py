"""
BAXTER MAKES COFFEE

Programmed by Adrian Beehner, Samantha Freitas, Amanda Ward
Class: CS404

	This program takes an assortment of modules designed
	to communicate different states to Baxter. Based
	on these states, Baxter knows what to do next.
	
	Baxter recieves voice input to make coffee, thus
	opening this program. From there, he inquires and
	recieves input on the type of coffee. Baxter is
	capable of image matching the coffee type and locating
	the matching Keurig cup and it's location. He then
	picks it up and proceeds to work the Keurig machine
	to make a cup of coffee.
	
	Baxter regularly images the state of the Keurig machine
	so that he can keep track of when events take place that
	need to be responded to. Once he realizes an event has occured
	he calls the appropriate module to respond.
	
Program Modules:

	Samantha Freitas - Program Lead
					   Keurig Cup Vision
					   Keurig Cup Pickup
					   
	Adrian Beehner   - Image Head
					   State Vision
					   State Determination
					   Camera Images
	
	Amanda Ward		 - Movement Head
					   Movement Events
					   Coordinate Control
"""

import rospy
import time
import numpy as np
import os
import sys

#import Find_Cups
#import pickup

import baxter_interface
from FeatureSIFTRecImg import recognizeImage

#import close_lid
#import place_kcup
#import opening_lid
#import serve_coffee
#import remove_kcup

#from Make_Coffee import pickupA     #Import function for finding and picking up kcup

from stateFunctions import InitCheck_State, PowerOff_State, Begin_State, CheckKCup_State, No_LeftOver_KCup_State, Ready_State		# import functions for moving and featture detection for each state
from stateBaxterKeurig import BaxterKeurigState			# Import class that contains the state info for Baxter

rospy.init_node('Coffee_Time')

def main():
	
	#import baxter state info
	baxter = BaxterKeurigState("Baxter")
	
	#Step 1: Recieve input 
	#Planned Path to do this: Move robot_left arm to kcups area
	os.system('python to_kcups.py')
	
	#Now that it's in the area, call the image matching for kcups.
	#This will also get the input for the type of coffee
	os.system('python Find_Cups.py')
	
	#Switch to checking for an existing kcup in the Keurig
	os.system('python opening_lid.py')
	#Call the state function to determine if a kcup is present.
	#If a cup is present, it should remove it inside the function.
	#CheckKCup_State(baxter)                                            """ For initial demo, this function is removed """
	
	#Next call the script to pickup the kcup
	#os.system('python pickupA.py')
	#Plan B kcup pickup (randomized)
	os.system('python pickupB.py')
	
	#Place the kcup inside the keurig
	os.system('python place_kcup.py')
	os.system('python close_lid.py')
	
	##Place coffee cup in brewing area
	os.system('python get_cup.py')
	
	#Place arm where picture of state can be taken
	os.system('python home_arms.py')
	
	# set the image to check for begin screen, so we can take coffee out
	imgs = ['Assets/beginclose1.png']
	
	# Stay in loop till we find the begin state
	while True:
		if(recognizeImage(imgs, 15)):
			break
		else:
			#wait 10 seconds to check for picture
			time.sleep(10)
			#Place arm where picture of state can be taken
			os.system('python home_arms.py')
		#keep checking
		
	#Loop into checking if coffee is ready and serve
	#while True:
		#Ready_State(baxter)
	#Remove coffee and serve it to designated area
	os.system('python serve_coffee.py')
	
	#Remove kcup at the end
	#os.system('python opening_lid.py')
	#os.system('python remove_kcup.py')
	#os.system('python close_lid.py')
	

if __name__ == '__main__':
	main()

