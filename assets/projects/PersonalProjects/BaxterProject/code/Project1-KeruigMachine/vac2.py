#!/usr/bin/env python

import rospy
import baxter_interface

rospy.init_node('vac')

rs = baxter_interface.RobotEnable()
rs.enable()

left_gripper = baxter_interface.Gripper('left')

rospy.sleep(1)


left_gripper.close()
rospy.sleep(1)
left_gripper.open()
rospy.sleep(1)
