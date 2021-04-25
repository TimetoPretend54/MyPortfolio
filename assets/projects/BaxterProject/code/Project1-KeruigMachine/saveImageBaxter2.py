#!/usr/bin/env python

import rospy
import os,sys
import time
import cv2
from pathlib import Path

# from baxterCamer.py, import the image_save class
from baxterCamera import image_save


def saveBaxterImage():
	# Lopp this method two times, for some reson, camera publisher doesn't update image
	# Until 2nd time around
	for i in range (0, 2):
		
		# Delete the old saved Baxter image, so we can save the new one
		try:
			os.remove('BaxterCamPicture/imageCapture.png')
			# Deleting file may take at least 1-2 seconds
			time.sleep(2)
		except:	
			pass
			
		#initialize node named 'save_picture_node (DONT NEED THIS IF RUNNING FROM Some other Python program, as
		#		it will have an init_node itself!
		rospy.init_node('save_picture_node', anonymous = True) 
		
		# intialize a image_save class object
		ic = image_save('BaxterCamPicture/imageCapture.png')
		
		# Give enough time to write file (Sometimes otherwise the file doesn't write could also do rospy.spin)
		check_file = Path('BaxterCamPicture/imageCapture.png')
		while not (check_file.is_file()):
			pass
			
		# Resize the file (only care about screen or kCups, not the background)
		image = cv2.imread('BaxterCamPicture/imageCapture.png');
		
		#try:
			#rospy.spin()					# Prevent Python main thread from exiting
		#except KeyboardInterrupt:
			#print ("Shutting down")
		#cv2.destroyAllWindows()			# Shuts down any opened windows

#saveBaxterImage()							# If running program alone, call function (if not using it for within another program)
