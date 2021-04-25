
import rospy, baxter_interface

import time

rospy.init_node("vacuum_test")
l_vacuum_sensor = baxter_interface.AnalogIO('left_vacuum_sensor_analog')

print "l_vac_senso", l_vacuum_sensor.state()

from baxter_interface import Gripper
left_gripper = Gripper('left')

left_gripper.command_suction(block=False, timeout=5.0)

time.sleep(5)
print "l_vac_senso", l_vacuum_sensor.state()
