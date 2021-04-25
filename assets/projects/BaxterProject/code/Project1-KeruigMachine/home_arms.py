#!/usr/bin/env python

"""
Script to return Baxter's arms to a "home" position
"""

#
# These three imports below: saveImageBaxter, rospy, and baxter_interface required ROS & Baxter to be running,
# even if they are not being used, otherwise, there will be import error stating "no module named..."
#

from saveImageBaxter import saveBaxterImage		# from saveImageBaxter.py, import the saveBaxterImage function
import rospy									# rospy - ROS Python Python API
import baxter_interface							# baxter_interface - Baxter Python API

print("Running home_arms.py")

#initialize our ROS node, registering it with the Master
rospy.init_node('Home_Arms')

#create instances of baxter_interface's Limb class
limb_right = baxter_interface.Limb('right')
limb_left = baxter_interface.Limb('left')

#saveBaxterImage()

#store the position of the arms for the camera

position_1_right = {'right_s0': 1.2471263805508415, 'right_s1': -0.2876213977285151, 'right_w0': 3.0591411862404865, 'right_w1': -0.8049564184428709, 'right_w2': -1.5351312734763278, 'right_e0': -0.9433981845495295, 'right_e1': 1.2712865779600366}
psoition_1_left = {'left_w0': 0.6715000898968398, 'left_w1': 1.0289176134741413, 'left_w2': -0.49777676566881673, 'left_e0': -1.188068120217253, 'left_e1': 1.941252687068991, 'left_s0': -0.07861651537912745, 'left_s1': -1.0016894544891752}

#Slow arm down
limb_right.set_joint_position_speed(2)

#move both arms to home position
limb_right.move_to_joint_positions(position_1_right)
limb_left.move_to_joint_positions(psoition_1_left)

#saveBaxterImage()

## store the home position of the arms
#home_zero_right = {	'right_s0': 0.00,
					#'right_s1': 0.00, 
					#'right_w0': 0.00,
					#'right_w1': 0.00, 
					#'right_w2': 0.00, 
					#'right_e0': 0.00,
					#'right_e1': 0.00}

#home_zero_left = {	'left_s0': 0.00,
					#'left_s1': 0.00,
					#'left_w0': 0.00,
					#'left_w1': 0.00,
					#'left_w2': 0.00,
					#'left_e0': 0.00,
					#'left_e1': 0.00}
					
#limb_right.move_to_joint_positions(home_zero_right)
#limb_left.move_to_joint_positions(home_zero_left)

saveBaxterImage()

quit()
