# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface
 
#I needed to import the gripper
from baxter_interface import Gripper

# initialize our ROS node, registering it with the Master
rospy.init_node('Hello_Baxter')

# create an instance of baxter_interface's Limb class
limb = baxter_interface.Limb('right')

# get the right limb's current joint angles
angles = limb.joint_angles()

# print the current joint angles
print angles

right_gripper=baxter_interface.Gripper('right')


# This places baxter back into the hover arm over circle

position_1 = {'right_s0': 1.103315681686584, 'right_s1': 0.845223414124863, 'right_e0': -3.0334470080434057, 'right_e1': 1.140514715792805, 'right_w0':  3.023859628119122, 'right_w1': 1.825053642386671, 'right_w2': 1.85880121972015}

# This aims the arm at the coffee maker

position_2 = {'right_s0': 1.2221991927477034, 'right_s1': 0.9108010928069644, 'right_w0': 1.62640313035551, 'right_w1': 1.963878903690301, 'right_w2': 1.6237186639767105, 'right_e0': -3.029612056073692, 'right_e1': 0.8801214770492561}


# This takes the kcup into the coffee maker
position_3 = {'right_s0': 0.979830228261808, 'right_s1': 1.0473253829287663, 'right_w0': 1.8031944161593039, 'right_w1': 1.7441361558257154, 'right_w2': 1.5347477782793564, 'right_e0': -3.0334470080434057, 'right_e1': 1.2298690966871304}
# This repeats the aim position so we can back out of he coffee maker without hitting the lid

position_4 = {'right_s0': 1.2221991927477034, 'right_s1': 0.9108010928069644, 'right_w0': 1.62640313035551, 'right_w1': 1.963878903690301, 'right_w2': 1.6237186639767105, 'right_e0': -3.029612056073692, 'right_e1': 0.8801214770492561}

# This moves the arm slighly away from the coffee maker
position_5 = {'right_s0': 0.7999709808822433, 'right_s1': 1.0477088781257375, 'right_w0': 3.0595246814374577, 'right_w1': 2.0413449334785145, 'right_w2': 1.469170099597255, 'right_e0': -3.032680017649463, 'right_e1': 1.8170002432502725}

# This moves the arm above the coffee maker so I have better access to lid location
position_6 = {'right_s0': 0.8923933233523395, 'right_s1': 0.9683253723526675, 'right_w0': 2.8735295109063514, 'right_w1': 1.988806091493439, 'right_w2': 1.6183497312191115, 'right_e0': -3.0319130272555204, 'right_e1': 2.357728470979881}

# This moves the arm above directly above the coffee lid but not touching
position_7 = {'right_s0': 0.847524385306691, 'right_s1': 0.9529855644738133, 'right_w0': 2.4190877024952977, 'right_w1': 1.8948497682354573, 'right_w2': 1.5803837067189475, 'right_e0': -3.032680017649463, 'right_e1': 2.164446891706319}

# This moves the arm above directly above the coffee lid but not touching
position_8 = {'right_s0': 1.096029272944128, 'right_s1': 1.0235486807165424, 'right_w0': 1.491029325824622, 'right_w1': 1.653631289340476, 'right_w2': 0.8145437983671547, 'right_e0': -3.0330635128464345, 'right_e1': 1.3575729972785913}

# This starts to close the lid and stops half way
position_9 = {'right_s0': 1.0561457724591075, 'right_s1': 1.049242858913623, 'right_w0': 2.0708740636453085, 'right_w1': 1.726495376765033, 'right_w2': 1.0335195558377974, 'right_e0': -3.032680017649463, 'right_e1': 1.5577574900976376}

# This will pull arm out for midpoint change of position
position_10 = {'right_s0': 1.1044661672774978, 'right_s1': 1.0239321759135136, 'right_w0': 1.6616846884768743, 'right_w1': 1.6820099339163561, 'right_w2': 1.5378157398551273, 'right_e0': -3.0280780752858067, 'right_e1': 1.0676506283682479}

# This will aim the arm in a position to close lid
position_11 = {'right_s0': 0.8877913809886832, 'right_s1': 1.0477088781257375, 'right_w0': 2.5540780118292137, 'right_w1': 1.8971507394172855, 'right_w2': 1.4196992191879505, 'right_e0': -3.032680017649463, 'right_e1': 1.8254371375836425}

# This will close lid
position_12 = {'right_s0': 0.8904758473674826, 'right_s1': 1.049242858913623, 'right_w0': 2.483514895586485, 'right_w1': 1.8611021909019783, 'right_w2': 1.4338885414758904, 'right_e0': -3.032680017649463, 'right_e1': 1.673189544386015}




# movement sequence
for _move in range(1):
	limb.move_to_joint_positions(position_1)
        limb.move_to_joint_positions(position_2)
	limb.move_to_joint_positions(position_3)
	right_gripper.open()
	limb.move_to_joint_positions(position_4)
	limb.move_to_joint_positions(position_5)
	limb.move_to_joint_positions(position_6)
	limb.move_to_joint_positions(position_7)
	limb.move_to_joint_positions(position_8)
	limb.move_to_joint_positions(position_9)
	limb.move_to_joint_positions(position_10)
	limb.move_to_joint_positions(position_11)
	limb.move_to_joint_positions(position_12)

# get the right limb's current joint angles
angles = limb.joint_angles()

print 'now I am getting the new location'

# print the current joint angles
print angles




# quit
quit()


