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

limb.set_joint_position_speed(.8)


# This is a transition point for the arm

position_1 = {'right_s0': 0.38157772098649667, 'right_s1': -1.0200972239438002, 'right_w0': -1.0097428536255737, 'right_w1': -1.5500875861582106, 'right_w2': 0.7040971816394049, 'right_e0': -0.7873156393821886, 'right_e1': 1.9124905472961395}

position_2 = {'right_s0': 1.37137882436956, 'right_s1': -0.6247136758663347, 'right_w0': -0.26422819071326253, 'right_w1': -1.2620826932327243, 'right_w2': 0.5878981369570848, 'right_e0': -0.7409127205486549, 'right_e1': 2.1671313580851184}
# This aims at the cup
position_3 = {'right_s0': 1.3625584348392188, 'right_s1': 0.5602864827751474, 'right_w0': 1.5838351634916896, 'right_w1': -0.5786942522297723, 'right_w2': 3.0476363303313456, 'right_e0': -0.9318933286403889, 'right_e1': 0.826815644670238}

# This places the hand arround the cup
position_4 = {'right_s0': 1.2559467700811826, 'right_s1': 0.5763932810479442, 'right_w0': 1.5443351582036402, 'right_w1': -0.3992185000471789, 'right_w2': 3.046869339937403, 'right_e0': -1.0918108257774433, 'right_e1': 0.7255729126698007}

#close the gripper hand here

# This lifts the cup
position_5 = {'right_s0': 1.4289031039152629, 'right_s1': 0.5683398819115458, 'right_w0': 1.7437526606287441, 'right_w1': -0.24045148850103862, 'right_w2': 3.046869339937403, 'right_e0': -1.2670681307933518, 'right_e1': 0.9453156605343862}

# This puts the coffee cup in the coffee maker
position_6 = {'right_s0': 1.445393397385031, 'right_s1': 0.5330583237901813, 'right_w0': 1.800126454583533, 'right_w1': -0.1606844875309971, 'right_w2': 3.0472528351343744, 'right_e0': -1.0707185899440188, 'right_e1': 1.3284273623087683}

# This backs the arm away from the coffee maker
position_7 = {'right_s0': 1.4829759266882236, 'right_s1': 0.43258258218368667, 'right_w0': 1.9274468599780223, 'right_w1': -0.6270146470481629, 'right_w2': 2.9920295267704997, 'right_e0': -0.7861651537912745, 'right_e1': 1.2755050251267215}

# This transitions away 
position_8 = {'right_s0': 1.5405002062339268, 'right_s1': 0.22281070944035636, 'right_w0': 1.453830291718401, 'right_w1': -0.39500005288049406, 'right_w2': 3.046869339937403, 'right_e0': -0.786548648988246, 'right_e1': 1.2593982268539248}

# get to location to push the Brew Button
position_9  = {'right_s0': 1.570029336400721, 'right_s1': 0.23930100291012454, 'right_w0': 3.055306234270773, 'right_w1': -0.9146360447766779, 'right_w2': 2.047864351827027, 'right_e0': -1.0768545130955605, 'right_e1': 1.2697525971721513}



# Push the button
position_10 = {'right_s0': 1.5274613695369008, 'right_s1': 0.2972087776527989, 'right_w0': 3.057607205452601, 'right_w1': -0.75088359566991, 'right_w2': 1.9082721001294547, 'right_e0': -1.1339952974442922, 'right_e1': 1.3127040592329429}


# back away
position_11 = {'right_s0': 1.607995360900885, 'right_s1': -0.2876213977285151, 'right_w0': 3.0583741958465436, 'right_w1': -1.0994807297168703, 'right_w2': -1.5217089415823304, 'right_e0': -1.1485681149292035, 'right_e1': 1.2302525918841019}

# This is a resting spot until needed again
position_12 = {'right_s0': -0.6438884357149024, 'right_s1': -1.004757416064946, 'right_w0': -0.7570195188214517, 'right_w1': -1.5715633171886063, 'right_w2': 3.0464858447404315, 'right_e0': -0.349364124440903, 'right_e1': 1.9025196721748845}











# movement sequence
for _move in range(1):
	limb.move_to_joint_positions(position_1)
        limb.move_to_joint_positions(position_2)
	limb.move_to_joint_positions(position_3)
	
	limb.move_to_joint_positions(position_4)
	right_gripper.close()
	limb.move_to_joint_positions(position_5)
	limb.move_to_joint_positions(position_6)
	right_gripper.open()
	limb.move_to_joint_positions(position_7)
	limb.move_to_joint_positions(position_8)
	limb.move_to_joint_positions(position_9)
	limb.move_to_joint_positions(position_10)
	limb.move_to_joint_positions(position_11)
	limb.move_to_joint_positions(position_12)
	limb.move_to_joint_positions(position_13)
	limb.move_to_joint_positions(position_14)
	limb.move_to_joint_positions(position_15)
	

	
# get the right limb's current joint angles
angles = limb.joint_angles()

print 'now I am getting the new location'

# print the current joint angles
print angles




# quit
quit()


