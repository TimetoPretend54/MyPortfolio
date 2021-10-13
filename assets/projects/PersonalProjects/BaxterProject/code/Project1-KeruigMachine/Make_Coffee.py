#<directory path when decided>

#####################################################################################
#  BAXTER MAKES COFFEE                                                              #
#                                                                                   #
#  Coded by: Amanda, Adrian, and Samantha                                           #
#                                                                                   #
#  Insert Large Project Description Here                                            #
#####################################################################################

import rospy
import roslib
import baxter_interface

import cv
import cv2
import cv_bridge

import numpy
import math
import os
import sys
import string
import time
import random
import tf

import std_srvs.srv

from matplotlib import pyplot as plt

from moveit_commander import conversions
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion         #Helps with printing arm coordinates
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest   #Looks like it might help with trajectories?

#Initialize the node
rospy.init_node('Coffee_Baxter', anonymous = True) 

# locate class
class locate():
    def __init__(self, arm, distance):
        global image_directory
        # arm ("left" or "right")
        self.limb           = arm
        self.limb_interface = baxter_interface.Limb(self.limb)

        if arm == "left":
            self.other_limb = "right"
        else:
            self.other_limb = "left"

        self.other_limb_interface = baxter_interface.Limb(self.other_limb)

        # gripper ("left" or "right")
        self.gripper = baxter_interface.Gripper(arm)
        
    # move a limb
    def baxter_ik_move(self, limb, rpy_pose):
        quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")

        node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        ik_service = rospy.ServiceProxy(node, SolvePositionIK)
        ik_request = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id="base")

        ik_request.pose_stamp.append(quaternion_pose)
        try:
            rospy.wait_for_service(node, 5.0)
            ik_response = ik_service(ik_request)
        except (rospy.ServiceException, rospy.ROSException), error_message:
            rospy.logerr("Service request failed: %r" % (error_message,))
            sys.exit("ERROR - baxter_ik_move - Failed to append pose")

        if ik_response.isValid[0]:
            print("PASS: Valid joint configuration found")
            # convert response to joint position control dictionary
            limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
            # move limb
            if self.limb == limb:
                self.limb_interface.move_to_joint_positions(limb_joints)
            else:
                self.other_limb_interface.move_to_joint_positions(limb_joints)
        else:
            # display invalid move message on head display
            self.splash_screen("Invalid", "move")
            # little point in continuing so exit with error message
            print "requested move =", rpy_pose
            sys.exit("ERROR - baxter_ik_move - No valid joint configuration found")

        if self.limb == limb:               # if working arm
            quaternion_pose = self.limb_interface.endpoint_pose()
            position        = quaternion_pose['position']

            # if working arm remember actual (x,y) position achieved
            self.pose = [position[0], position[1],                                \
                         self.pose[2], self.pose[3], self.pose[4], self.pose[5]]
    # find distance of limb from nearest line of sight object
    def get_distance(self, limb):
        if limb == "left":
            dist = baxter_interface.analog_io.AnalogIO('left_hand_range').state()
        elif limb == "right":
            dist = baxter_interface.analog_io.AnalogIO('right_hand_range').state()
        else:
            sys.exit("ERROR - get_distance - Invalid limb")

        # convert mm to m and return distance
        return float(dist / 1000.0)

    # update pose in x and y direction
    def update_pose(self, dx, dy):
        x = self.pose[0] + dx
        y = self.pose[1] + dy
        pose = [x, y, self.pose[2], self.roll, self.pitch, self.yaw]
        self.baxter_ik_move(self.limb, pose)
        
		
    def pick_a_cup(self):
		# slow down to reduce scattering of neighbouring cups
            self.limb_interface.set_joint_position_speed(0.1)

            # move down to pick up cup
            pose = (self.pose[0] + self.cam_x_offset,
                    self.pose[1] + self.cam_y_offset,
                    self.pose[2] + (0.112 - self.distance),
                    self.pose[3],
                    self.pose[4],
                    angle)
            self.baxter_ik_move(self.limb, pose)
            self.print_arm_pose()

            # close the gripper
            self.gripper.close()

# print all 6 arm coordinates (only required for program development)
def print_arm_pose(self):
	return
	pi = math.pi

	quaternion_pose = self.limb_interface.endpoint_pose()
	position        = quaternion_pose['position']
	quaternion      = quaternion_pose['orientation']
	euler           = tf.transformations.euler_from_quaternion(quaternion)

	print
	print "             %s" % self.limb
	print 'front back = %5.4f ' % position[0]
	print 'left right = %5.4f ' % position[1]
	print 'up down    = %5.4f ' % position[2]
	print 'roll       = %5.4f radians %5.4f degrees' %euler[0], 180.0 * euler[0] / pi
	print 'pitch      = %5.4f radians %5.4f degrees' %euler[1], 180.0 * euler[1] / pi
	print 'yaw        = %5.4f radians %5.4f degrees' %euler[2], 180.0 * euler[2] / pi
		
# read the setup parameters from setup.dat
def get_setup():
    global image_directory
    file_name = image_directory + "setup.dat"

    try:
        f = open(file_name, "r")
    except ValueError:
        sys.exit("ERROR: coffee_setup must be run before coffee")

    # find limb
    s = string.split(f.readline())
    if len(s) >= 3:
        if s[2] == "left" or s[2] == "right":
            limb = s[2]
        else:
            sys.exit("ERROR: invalid limb in %s" % file_name)
    else:
        sys.exit("ERROR: missing limb in %s" % file_name)

    # find distance to table
    s = string.split(f.readline())
    if len(s) >= 3:
        try:
            distance = float(s[2])
        except ValueError:
            sys.exit("ERROR: invalid distance in %s" % file_name)
    else:
        sys.exit("ERROR: missing distance in %s" % file_name)

    return limb, distance
        
def pickupA():
	
    # get setup parameters
    limb, distance = get_setup()
    print "limb     = ", limb
    print "distance = ", distance
    
    coffee = locate(limb, distance)
    
    coffee.print_arm_pose()
    
    return
    


