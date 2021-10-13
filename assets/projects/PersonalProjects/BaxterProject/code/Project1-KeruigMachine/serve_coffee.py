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


# This is a transition point for the arm

position_1 = {'right_s0': 1.0553787820651646, 'right_s1': -0.9104175976099931, 'right_w0': -0.19059711289476267, 'right_w1': -1.5642769084461507, 'right_w2': 3.0472528351343744, 'right_e0': -0.2718980946526896, 'right_e1': 1.8787429699626605}
# This aims at the cup

position_2 = {'right_s0': 1.3794322235059584, 'right_s1': 0.6645971763513555, 'right_w0': -2.3811216779951336, 'right_w1': -1.571179821991635, 'right_w2': 2.0716410540392514, 'right_e0': -1.8338740319170121, 'right_e1': 0.21015536794030168}
# This places the hand arround the cup
position_3 = {'right_s0': 1.0783884938834458, 'right_s1': 1.0477088781257375, 'right_w0': -1.9159420040688817, 'right_w1': -1.076471017898589, 'right_w2': 1.9144080232809964, 'right_e0': -2.4213886736771255, 'right_e1': 0.6239466854723921}
#close the gripper hand here

# This lifts the cup
position_4 = {'right_s0': 1.0837574266410448, 'right_s1': 1.0465583925348236, 'right_w0': -1.9144080232809964, 'right_w1': -1.077238008292532, 'right_w2': 1.912874042493111, 'right_e0': -2.4321265391923235, 'right_e1': 0.6596117387907279}

#This takes the cup to the cup square
position_5 = {'right_s0': 1.096029272944128, 'right_s1': 1.021247709534714, 'right_w0': -2.0566847413573686, 'right_w1': -0.5660389107297177, 'right_w2': 1.9109565665082542, 'right_e0': -2.605082873026404, 'right_e1': 0.6515583396543295}
# This puts the coffee cup on the table

position_6 = {'right_s0': 1.183849673050568, 'right_s1': 0.9437816797465008, 'right_w0': -2.341621672707084, 'right_w1': -0.48473792897179074, 'right_w2': 1.8396264598715824, 'right_e0': -2.213150781721681, 'right_e1': 0.5487816268660067}
#release cup

# This backs the arm away from the coffee 

position_7 = {'right_s0': 1.279339977096435, 'right_s1': 0.8218302071096104, 'right_w0': -2.032141048751202, 'right_w1': -1.015495281580144, 'right_w2': 1.8400099550685538, 'right_e0': -2.3374032255403994, 'right_e1': 0.3712233506682701}
# This transitions away 

position_8 = {'right_s0': 1.5132720472489607, 'right_s1': 0.336325287743877, 'right_w0': -2.557529468601956, 'right_w1': -1.2808739578843205, 'right_w2': 1.6352235198858511, 'right_e0': -1.791306065053192, 'right_e1': 0.6427379501239884}

# This transitions away 

position_9 = {'right_s0': 1.5815341923098616, 'right_s1': -0.10162622719740866, 'right_w0': -1.584218658688661, 'right_w1': -1.5715633171886063, 'right_w2': 0.9276748814737039, 'right_e0': -1.9397187062811057, 'right_e1': 1.5466361293854685}
# This transitions away 

position_10 = {'right_s0': -0.5219369630780121, 'right_s1': -0.3056456719861687, 'right_w0': -1.0208642143377429, 'right_w1': -1.571179821991635, 'right_w2': 0.8874078857917118, 'right_e0': -2.0436459046603423, 'right_e1': 0.11083011192472114}

# This is a resting spot until needed again

position_11 = {'right_s0': -0.6438884357149024, 'right_s1': -1.004757416064946, 'right_w0': -0.7570195188214517, 'right_w1': -1.5715633171886063, 'right_w2': 3.0464858447404315, 'right_e0': -0.349364124440903, 'right_e1': 1.9025196721748845}














# movement sequence
for _move in range(1):
	limb.move_to_joint_positions(position_1)
        limb.move_to_joint_positions(position_2)
	limb.move_to_joint_positions(position_3)
	right_gripper.close()
	rospy.sleep(2)
	limb.move_to_joint_positions(position_4)
	
	limb.move_to_joint_positions(position_5)
	rospy.sleep(2)
	limb.move_to_joint_positions(position_6)
	right_gripper.open()
	limb.move_to_joint_positions(position_7)
	limb.move_to_joint_positions(position_8)
	limb.move_to_joint_positions(position_9)
	limb.move_to_joint_positions(position_10)
	limb.move_to_joint_positions(position_11)
	

	
# get the right limb's current joint angles
angles = limb.joint_angles()

print 'now I am getting the new location'

# print the current joint angles
print angles




# quit
quit()


