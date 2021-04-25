# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface
 
#I needed to import the gripper
from baxter_interface import Gripper

# initialize our ROS node, registering it with the Master
rospy.init_node('Hello_Baxter')

# create an instance of baxter_interface's Limb class
limb = baxter_interface.Limb('left')

# get the right limb's current joint angles
angles = limb.joint_angles()

# print the current joint angles
print angles

left_gripper=baxter_interface.Gripper('left')

# This is a transition point for the arm

position_1 = {'left_w0': 0.6469563972906732, 'left_w1': 0.7781117546548761, 'left_w2': -0.6983447536848346, 'left_e0': -0.49969424165367354, 'left_e1': 1.3460681413694506, 'left_s0': -0.47553404424447826, 'left_s1': -0.4812864721990486}



# movement sequence
for _move in range(1):
	limb.move_to_joint_positions(position_1)

	#right_gripper.close()

