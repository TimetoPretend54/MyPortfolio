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

right_gripper=baxter_interface.Gripper('left')


# This places baxter back into the hover arm over circle

position_1 = {'left_w0': 0.5767767762449155, 'left_w1': 1.1478011245352608, 'left_w2': 0.661912709972556, 'left_e0': -0.7025632008515195, 'left_e1': 1.2547962844902685, 'left_s0': -0.3474466484560462, 'left_s1': -0.661912709972556}
# This brings the arm closer to the coffee maker

position_2 = {'left_w0': 0.4973932704718454, 'left_w1': 1.6509468229616766, 'left_w2': 0.7869321441852173, 'left_e0': -0.5303738574113818, 'left_e1': -0.0502378708032473, 'left_s0': -1.3199904679753984, 'left_s1': -0.11581554948534874}

# This aims the arm at the coffee maker
position_3 = {'left_w0': -0.8245146734884099, 'left_w1': 1.109068109641154, 'left_w2': -0.028762139772851508, 'left_e0': -0.4751505490475069, 'left_e1': 0.37045636027432743, 'left_s0': -1.0273836326862558, 'left_s1': -0.05062136600021865}

# This places the kcup in the coffee maker

position_4 = {'left_w0': -0.5292233718204677, 'left_w1': 0.6680486331240977, 'left_w2': -0.27611654181937445, 'left_e0': -0.44447093328979864, 'left_e1': 0.25579129637989273, 'left_s0': -1.3621749396422473, 'left_s1': -0.013805827090968724}
# This moves the arm slighly away from the coffee maker

position_5 = {'left_w0': -0.5215534678810406, 'left_w1': 1.266301140399409, 'left_w2': -0.24351945007680942, 'left_e0': -0.48358744338087667, 'left_e1': -0.049854375606275945, 'left_s0': -1.189218605808167, 'left_s1': 0.0222427214243385}

# This moves the arm above the coffee maker so I have better access to lid location
position_6 = {'left_w0': -2.5920440363293777, 'left_w1': 1.9086555953264261, 'left_w2': 0.03604854851530722, 'left_e0': -0.44715539966859813, 'left_e1': 1.8607186957050068, 'left_s0': 1.031985575049912, 'left_s1': -0.47246608266870743}





# movement sequence
for _move in range(1):
	limb.move_to_joint_positions(position_1)
        limb.move_to_joint_positions(position_2)
	limb.move_to_joint_positions(position_3)
	
	limb.move_to_joint_positions(position_4)
	right_gripper.open()
	limb.move_to_joint_positions(position_5)
	limb.move_to_joint_positions(position_6)
	
# get the right limb's current joint angles
angles = limb.joint_angles()

print 'now I am getting the new location'

# print the current joint angles
print angles




# quit
quit()


