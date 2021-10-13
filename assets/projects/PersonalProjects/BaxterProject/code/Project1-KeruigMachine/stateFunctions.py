#!/usr/bin/env python

from FeatureSIFT import recognizeImage			# Import function to use SIFT opencv to check for feature detection

#
# These three imports below: saveImageBaxter, rospy, and baxter_interface required ROS & Baxter to be running,
# even if they are not being used, otherwise, there will be import error stating "no module named..."
#

import os
import sys
from saveImageBaxter import saveBaxterImage		# from saveImageBaxter.py, import the saveBaxterImage function
import rospy										# rospy - ROS Python Python API
import baxter_interface							# baxter_interface - Baxter Python API


# Images to check for when looking at cups (For testing, will be the same format), we dont know/care what cup we get,
# so we need to check them all and find the first match we get (IF we get a match at all)
imgCup = ['Assets/DonutShopClose1.png', 'Assets/NewmanOwnClose1.png', 'Assets/GreenMountainClose1.png', 'Assets/SwissMissClose1.png']


#													NOTE:	
#
# Don't need to set big list of screen images to check for, as whenever we search for a screen "icon", we KNOW what state
# we are in, and from that state info Baxter knows to only want to recongize that ONE object for that state that is important
# to determining what to do next, so only one 'picture' or object (the current screen we want for that state) is needed in 
# the current state for the Keurig Screen Checking.
#imgScreen = ['Assets/beginclose1.png', 'Assets/readyclose1.png', 'Assets/poweroffclose2.png']


#########################################################################################
# 						  		 InitCheck_State										#
# Function to perform Baxter Movements for "inital check" state, where Baxter will		#
# perform movement actions for this state and then check if the lid is open or closed 	#
# by determining if he can see the power off icon on the screen to determine what state #
# we are in next																		#
#########################################################################################
def InitCheck_State(self):
	
	print('%s:' % self.state)
	
	#
	# Code for setting up Baxter for checking if lid is open or closed
	#
	
	# Image to search for objects in (will eventually be 'BaxterCamPicture/imageCapture.png')
	img2 = 'Assets/poweroff1.png'
	
	# Number of good matches needed to find 'poweroff' icon in scene/screen (we get around 8, so lower # by 2 or something just in case)
	goodMatch = 6
	
	print('\tChecking for "Power" screen...')
	
	# instead of 1st parameter being a list of a bunch of pictures,
	# just send in the picture we KNOW we want to check if it exists in scene (like here we only really want to find
	# the power off screen, so instead of sending a list, why not just send the object picture of the power off screen?, 
	# make sure to send the variable as a list with only one image in it though, as our image recognizer function takes list for first parameter)
	# NOTE: Probably wont work for the kcups though, as we dont know what EXACTLY type of cup we are looking for
	imgScreen = ['Assets/poweroffclose2.png']
	
	if recognizeImage(imgScreen, img2, goodMatch):
		print('\tFound "Power" screen, performing movement (if needed) and \n\t\t then changing state...')
		#
		# Code/Function for Movement after finding "power" screen...
		#
		
		os.system('python close_lid.py')
		
		self.lastState = self.state
		self.lid_closed()
		print('\tState is now: %s\n' % self.state)
		
	else:
		print('\tDid not find "Power" screen, performing movement (if needed) and \n\t\t then changing state...')
		#
		# Code/Function for Movement after not finding "power" screen...
		#
		self.lastState = self.state
		self.lid_open()
		print('\tState is now: %s\n' % self.state)
		
	
#########################################################################################
# 								  PowerOff_State										#
# Function to perform Baxter movements for "power off" state, where Baxter will perform	#
# movement actions for this state and then check check if if he recongizes the "begin" 	#
# screen on the Keurig to determine what state we are in next		 					#
#########################################################################################
def PowerOff_State(self):
	
	print('%s:' % self.state)
	
	# Only need to check if desired object is in scene image that Baxter took a picture of, since
	# we KNOW exactly what object we want to check for (begin screen)
	imgScreen = ['Assets/beginclose1.png']
	
	# Image to search for objects in (will eventually be 'BaxterCamPicture/imageCapture.png')
	img2 = 'Assets/begin1.png'
	
	# Number of good matches needed to find 'begin' icon in scene/screen (we get around 25, so lower # by 2 or something just in case)
	goodMatch = 20
	
	print('\tChecking for "Begin" screen...')
	
	if recognizeImage(imgScreen, img2, goodMatch):
		print('\tFound "Begin" screen, performing movement (if needed) and \n\t\t then changing state...')
		#
		# Code/Function for Movement after finding "begin" screen...
		#
		self.lastState = self.state
		self.rec_begin()
		print('\tState is now: %s\n' % self.state)
	
	# Else, we do nothing, and just call this function again to check, don't need else statement
	# to do that


#########################################################################################
# 							 		Begin_State											#
#  Function to perform Baxter movements for "begin" state, where Baxter will perform	#
#  movement actions for this state and then check if the previous state was the ready	#
#  state, to determine what state we are in next		 								#
#########################################################################################
def Begin_State(self):
	
	print('%s:' % self.state)
	
	print('\tChecking if "ready" was previous state...')
	
	if self.lastState == 'ready':
		print('\tready was previous state, performing movement (if needed) and \n\t\t then changing state...')
		#
		# Code/Function for Movement after finding "ready" state as the pevious state...
		#
		self.last_state_READY()
		self.lastState = self.state			
		print('\tState is now: %s\n' % self.state)

	else:
		print('\tready was not previous state, performing movement (if needed) and \n\t\t then changing state...')
		#
		# Code/Function for Movement after not finding "ready" state as the previous state...
		#
		self.lastState = self.state
		self.last_state_not_READY()
		print('\tState is now: %s\n' % self.state)


#########################################################################################
# 								 CheckKCup_State										#
#  Function to perform Baxter movements for "checkKCup" state, where Baxter will 		#
#  perform movement actions for this state and then check if if he recongizes a K-cup 	#
#  in the machine or not to determine what state we are in next	 						#
#########################################################################################
def CheckKCup_State(self):
	
	print('%s:' % self.state)
	
	# Image to search for objects in (will eventually be 'BaxterCamPicture/imageCapture.png')
	# Here we test the flow of program by checking for item we know that wont be recognized by
	# the list of images, because the goal in this function is to NOT recognize a kcup, otherwise
	# we have to throw it away and go back to the lift begin state
	img2 = 'Assets/NoKCup.png'
	
	# Number of good matches needed to find 'kcup' in scene
	goodMatch = 45
	
	print('\tChecking for kcup...')
	
	if recognizeImage(imgCup, img2, goodMatch):
		print('\tFound kCup, performing movement (if needed) and \n\t\t then changing state...')
		#
		# Code/Function for Movement after finding kCup...
		#
		self.lastState = self.state
		self.rec_kcup()
		print('\tState is now: %s\n' % self.state)
	else:
		print('\tDid not find kCup, performing movement (if needed) and \n\t\t then changing state...')
		#
		# Code/Function for Movement after not finding kCup...
		#
		os.system('python remove_kcup.py')
		
		self.lastState = self.state
		self.unrec_kcup()
		print('\tState is now: %s\n' % self.state)


#########################################################################################
# 								No_LeftOver_KCup_State									#
#  Function to perform Baxter movements for "no_leftover_kCup" state, where Baxter 		#
#  will perform movement actions for this state and then check if if he recongizes 		#
#  the "ready" screen on the Keurig to determine what state we are in next 				#
#########################################################################################
def No_LeftOver_KCup_State(self):
	
	print('%s:' % self.state)

	# Only need to check if desired object is in scene image that Baxter took a picture of, since
	# we KNOW exactly what object we want to check for (ready screen)
	imgScreen = ['Assets/readyclose1.png']
	
	# Image to search for objects in (will eventually be 'BaxterCamPicture/imageCapture.png')
	img2 = 'Assets/Ready.png'
	
	# Number of good matches needed to find 'ready' icon in scene/screen
	goodMatch = 200
	
	print('\tChecking for ready screen...')
	
	if recognizeImage(imgScreen, img2, goodMatch):
		print('\tFound "ready" screen, performing movement (if needed) and \n\t\t then changing state...')
		#
		# Code/Function for Movement after finding "ready" screen...
		#
		self.lastState = self.state
		self.rec_ready()
		print('\tState is now: %s\n' % self.state)
	
	# Else, we do nothing, and just call this function again to check, don't need else statement
	# to do 
	
		
#########################################################################################
# 							 		Ready_State											#
#  Function to perform Baxter movements for "ready" state, where Baxter will 			#
#  perform movement actions for this state and then check if if he recongizes the 		#
#  "begin" state on the Keurig to determine what state we are in next 					#
#########################################################################################
def Ready_State(self):
	
	print('%s:' % self.state)

	# Only need to check if desired object is in scene image that Baxter took a picture of, since
	# we KNOW exactly what object we want to check for (begin screen)
	imgScreen = ['Assets/beginclose1.png']
	
	# Image to search for objects in (will eventually be 'BaxterCamPicture/imageCapture.png')
	img2 = 'Assets/begin1.png'
	
	# Number of good matches needed to find 'begin' icon in scene/screen
	goodMatch = 90
	
	print('\tChecking for "Begin" screen...')
	
	if recognizeImage(imgScreen, img2, goodMatch):
		print('\tFound "Begin" screen, performing movement (if needed) and \n\t\t then changing state...')
		#
		# Code/Function for Movement after finding "begin" screen...
		#
		os.system('python serve_coffee.py')
		self.lastState = self.state
		self.rec_begin()
		print('\tState is now: %s\n' % self.state)
		return False
	else:
		Ready_State(self)
	
	# Else, we do nothing, and just call this function again to check, don't need else statement
	# to do that
