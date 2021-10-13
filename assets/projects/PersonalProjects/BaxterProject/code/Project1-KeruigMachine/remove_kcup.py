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


# This aims at the coffee maker

position_1 = {'left_w0': -0.8992962368978238, 'left_w1': 1.131694326262464, 'left_w2': 0.9326603190343316, 'left_e0': -0.4157087935169471, 'left_e1': 0.349364124440903, 'left_s0': -1.0484758685196802, 'left_s1': -0.06557767868210143}
# This places suction cup over kcup

position_2 = {'left_w0': -0.6918253353363216, 'left_w1': 0.5986360024722828, 'left_w2': -0.2811019793800021, 'left_e0': -0.13460681413694506, 'left_e1': -0.03911651009107805, 'left_s0': -1.4933302970064504, 'left_s1': 0.15109710760671324}
#gripper close

# This backs the kcup away from the machine
position_3 = {'left_w0': -0.9050486648523941, 'left_w1': 1.092577816171386, 'left_w2': -0.23163109897069747, 'left_e0': -0.03681553890924993, 'left_e1': 0.19980099762207515, 'left_s0': -1.275888520323693, 'left_s1': -0.10699515995500761}

# This is a transition position

position_4 = {'left_w0': -0.07669903939427068, 'left_w1': 1.2824079386722058, 'left_w2': -0.2339320701525256, 'left_e0': 0.1062281695610649, 'left_e1': 0.3240534414407937, 'left_s0': -1.5911215722341454, 'left_s1': -0.2768835322133172}

# This moves the arm over to the trash
position_5 = {'left_w0': -0.4302816110018586, 'left_w1': 0.7021797056545481, 'left_w2': 0.18599517053110642, 'left_e0': 0.11926700625809092, 'left_e1': 1.660150707688989, 'left_s0': -1.4384904838395467, 'left_s1': -0.5453301700932646}
# This lowers the hand closer to the trash
position_6 = {'left_w0': -0.6622962051695274, 'left_w1': 0.056373793954788955, 'left_w2': 0.18637866572807776, 'left_e0': 0.10776215034895031, 'left_e1': 1.647111870991963, 'left_s0': -1.5761652595522626, 'left_s1': -0.11619904468232009}

#release the kcup into the trash

# transition spot
position_7 = {'left_w0': -0.43948549572917106, 'left_w1': 1.4595827196729712, 'left_w2': 0.604388430426853, 'left_e0': 0.2841699409557729, 'left_e1': 1.3341797902633385, 'left_s0': -1.290077842611633, 'left_s1': -0.6910583449423789}

# this is the resting spot
position_8 = {'left_w0': -2.1935925266761416, 'left_w1': 2.0862138715241625, 'left_w2': 0.9518350788828992, 'left_e0': -0.023009711818281205, 'left_e1': 2.1866896131306572, 'left_s0': 0.9253739102918759, 'left_s1': -0.8229806927005244}




# movement sequence
for _move in range(1):
	limb.move_to_joint_positions(position_1)
        limb.move_to_joint_positions(position_2)
	rospy.sleep(3.0)
	right_gripper.close()
	rospy.sleep(3.0)
	limb.move_to_joint_positions(position_3)
	
	limb.move_to_joint_positions(position_4)
	limb.move_to_joint_positions(position_5)
	limb.move_to_joint_positions(position_6)
	right_gripper.open()
	limb.move_to_joint_positions(position_7)
	
	limb.move_to_joint_positions(position_8)
	
# get the right limb's current joint angles
angles = limb.joint_angles()

print 'now I am getting the new location'

# print the current joint angles
print angles




# quit
quit()


