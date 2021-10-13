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



# This place where baxter views the screen

position_1 = {'right_s0': 0.9767622666860372, 'right_s1': 1.0477088781257375, 'right_w0': 1.753340040553028, 'right_w1': 1.781718685128908, 'right_w2': 1.2950632801722606, 'right_e0': -3.0334470080434057, 'right_e1': 1.3763642619301875}


# I have baxter move back from the machine to give his arm space to make its text move

position_2 = {'right_s0': 1.2221991927477034, 'right_s1': 0.9108010928069644, 'right_w0': 1.62640313035551, 'right_w1': 1.963878903690301, 'right_w2': 1.6237186639767105, 'right_e0': -3.029612056073692, 'right_e1': 0.8801214770492561}

# I then aim baxter's arm

position_3 = {'right_s0': 1.0331360606408262, 'right_s1': 1.0465583925348236, 'right_w0': 1.3663933868089322, 'right_w1': 1.5987914761735724, 'right_w2': 3.0472528351343744, 'right_e0': -3.032680017649463, 'right_e1': 0.6024709544419963}
# This places the gripper hand in position to lift

position_4 = {'right_s0': 0.9871166370042638, 'right_s1': 1.0496263541105944, 'right_w0': 1.309636097657172, 'right_w1': 1.5454856437945543, 'right_w2': 3.0472528351343744, 'right_e0': -3.033830503240377, 'right_e1': 0.6389029981542749}


# This changes the angle for a better position
position_5 = {'right_s0': 0.9748447907011805, 'right_s1': 1.049242858913623, 'right_w0': 1.8921653018566578, 'right_w1': 1.58958759144626, 'right_w2': 2.6925197779358725, 'right_e0': -3.0330635128464345, 'right_e1': 1.5247769031581013}
# This completes the lift
position_6 = {'right_s0': 0.5698738626994312, 'right_s1': 0.698728248881806, 'right_w0': 2.7929955195423672, 'right_w1': 2.058985712539197, 'right_w2': 2.7423741535421486, 'right_e0': -3.032680017649463, 'right_e1': 1.6478788613859057}
# This backs the arm away from the lid

position_7 = {'right_s0': 1.189218605808167, 'right_s1': 0.9081166264281649, 'right_w0': 1.8139322816745018, 'right_w1': 1.920543946432538, 'right_w2': 2.6254081184658857, 'right_e0': -3.0334470080434057, 'right_e1': 1.814315776871473}

# This is a transition position to move arm better
position_8 = {'right_s0': 1.0971797585350422, 'right_s1': 1.0235486807165424, 'right_w0': 1.307335126475344, 'right_w1': 1.3449176557785365, 'right_w2': 1.8028109209623324, 'right_e0': -3.0334470080434057, 'right_e1': 2.598563454677891}


# This will move the arm out of the way until it is needed again
position_10 = {'right_s0': -0.7382282541698554, 'right_s1': 1.0488593637166517, 'right_w0': 2.975922728497703, 'right_w1': 1.976917740387327, 'right_w2': 2.8846508716185206, 'right_e0': -3.0334470080434057, 'right_e1': 2.616971224132516}


position_11 = {'right_s0': 0.6293156182299909, 'right_s1': -1.1842331682475393, 'right_w0': 0.5284563814265251, 'right_w1': 1.196888509747594, 'right_w2': 2.199344954630712, 'right_e0': -0.9836651802315216, 'right_e1': 1.6206507024009396}

# movement sequence
for _move in range(1):
	limb.move_to_joint_positions(position_1)
	limb.move_to_joint_positions(position_2)
	limb.move_to_joint_positions(position_3)
	limb.move_to_joint_positions(position_4)
	limb.move_to_joint_positions(position_5)
	limb.move_to_joint_positions(position_6)
	limb.move_to_joint_positions(position_7)
	limb.move_to_joint_positions(position_8)
	#limb.move_to_joint_positions(position_9)
	limb.move_to_joint_positions(position_10)
	limb.move_to_joint_positions(position_11)
	
	
	

# get the right limb's current joint angles
angles = limb.joint_angles()

print 'now I am getting the new location'

# print the current joint angles
print angles




# quit
quit()


