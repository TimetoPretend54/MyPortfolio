#!/usr/bin/env python
from __future__ import print_function		# Must be first line in file

import roslib
import baxter_interface
roslib.load_manifest('baxter_tools')		# For accessing camera package in Baxter
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_save:
	
	def __init__(self, path_save):
		# "cam=" and "cam.resolution" CANNOT be used on simulator!
		#cam = baxter_interface.camera.CameraController("right_hand_camera")
		#cam.resolution = (960, 600)	# doesn't work for simulator
		#cam.exposure            = -1             # range, 0-100 auto = -1
        #cam.gain                = -1             # range, 0-79 auto = -1
        #cam.white_balance_blue  = -1             # range 0-4095, auto = -1
        #cam.white_balance_green = -1             # range 0-4095, auto = -1
        #cam.white_balance_red   = -1             # range 0-4095, auto = -1
		self.path_save = path_save
		self.image_pub = rospy.Publisher("cameras/right_hand_camera/image", Image, queue_size = 1)    
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("cameras/right_hand_camera/image",Image, self.capture_picture)
	def capture_picture(self,data):
		try:
			 cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
			print (e)                                               
		cv2.imwrite(self.path_save, cv_image)
		
		# Once written to unregister the subcriber (so we dont constantly overwrite picture)
		self.image_sub.unregister()
		self.image_pub.unregister()
		

#def saveBaxterImage():
	##initialize node named 'save_picture_node
	#rospy.init_node('save_picture_node', anonymous = True) 
	#ic = image_save('/home/rosbox/Pictures/Baxter/KeurigPicture/imageCapture.png')
	## Give enough time to write file (Sometimes otherwise the file doesn't write)
	#check_file = Path('/home/rosbox/Pictures/Baxter/KeurigPicture/imageCapture.png')
	#while not (check_file.is_file()):
		#pass
	#try:
		#rospy.spin()				# Prevent Python main thread from exiting
	#except KeyboardInterrupt:
		#print ("Shutting down")
	#cv2.destroyAllWindows()			# Shuts down any opened windows
	
#saveBaxterImage()
