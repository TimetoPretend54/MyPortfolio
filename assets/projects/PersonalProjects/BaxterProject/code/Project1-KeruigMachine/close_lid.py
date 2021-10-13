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


# This is a transition point so I do not smack the machine

position_1 = {'right_s0': 0.6343010557906186, 'right_s1': 1.0469418877317949, 'right_w0': 2.8562722270426404, 'right_w1': 1.8070293681290175, 'right_w2': 2.9694033101491897, 'right_e0': -3.034213998437348, 'right_e1': 2.616971224132516}

# This puts the hand on the lid

position_2 = {'right_s0': 0.9974710073224903, 'right_s1': 1.0473253829287663, 'right_w0': 1.5915050674311169, 'right_w1': 1.5458691389915258, 'right_w2': 2.497704217874425, 'right_e0': -3.0334470080434057, 'right_e1': 1.5892040962492886}

# This completes the close
position_3 = {'right_s0': 0.9936360553527768, 'right_s1': 1.0484758685196802, 'right_w0': 1.5251603983550726, 'right_w1': 1.535514768673299, 'right_w2': 3.046869339937403, 'right_e0': -3.0322965224524916, 'right_e1': 0.941864203761644}
# This aims at on button
position_4 = {'right_s0': 0.948383622110157, 'right_s1': 1.0473253829287663, 'right_w0': 2.0946507658575326, 'right_w1': 1.7337817855074888, 'right_w2': 2.813320764981849, 'right_e0': -3.032680017649463, 'right_e1': 1.363708920430133}

# This pushes the button
position_5 = {'right_s0': 1.096029272944128, 'right_s1': 1.04272344056511, 'right_w0': 1.674340029976929, 'right_w1': 1.7418351846438873, 'right_w2': 1.0860583978228728, 'right_e0': -3.029228560876721, 'right_e1': 1.363708920430133}

# This aims at screen

position_6 = {'right_s0': 1.050393344504537, 'right_s1': 1.049242858913623, 'right_w0': 1.7176749872346921, 'right_w1': 1.7383837278711451, 'right_w2': 1.226034144717417, 'right_e0': -3.0330635128464345, 'right_e1': 1.4465438829759452}



# This goes to the spot adrian needs to read

position_7 = {'right_s0': 1.2801069674903778, 'right_s1': -0.32060198466805145, 'right_w0': 3.0526217678919734, 'right_w1': -0.9338108046252456, 'right_w2': -1.479907965112453, 'right_e0': -0.9756117810951231, 'right_e1': 1.294296289778318}

# this is a resting spot until the arm is needed again


position_7 = {'right_s0': 0.5767767762449155, 'right_s1': -0.9713933339284383, 'right_w0': -0.48780589054756157, 'right_w1': -1.4634176716426848, 'right_w2': 3.0472528351343744, 'right_e0': -1.1884516154142244, 'right_e1': 2.156009997372949}











# movement sequence
for _move in range(1):
	limb.move_to_joint_positions(position_1)
        limb.move_to_joint_positions(position_2)
	limb.move_to_joint_positions(position_3)
	
	limb.move_to_joint_positions(position_4)
	
	limb.move_to_joint_positions(position_5)
	limb.move_to_joint_positions(position_6)
	limb.move_to_joint_positions(position_7)
	
# get the right limb's current joint angles
angles = limb.joint_angles()

print 'now I am getting the new location'

# print the current joint angles
print angles




# quit
quit()


