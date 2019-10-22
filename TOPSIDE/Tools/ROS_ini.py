#!/usr/bin/env python

import os
import time

def rosserial_rov():
	os.system("sudo sshpass -p 'pi' ssh pi@192.168.1.200 'rosrun rosserial_python serial_node.py /dev/ttyS4'")

def joy():
		os.system('/opt/ros/kinetic/bin/rosparam set joy_node/dev "/dev/input/js0"')
		os.system('/opt/ros/kinetic/bin/rosparam set joy_node/coalesce_interval "0.1"')
		os.system('/opt/ros/kinetic/bin/rosparam set joy_node/deadzone "0.1"')
		os.system('/opt/ros/kinetic/bin/rosrun joy joy_node')

#os.system("xterm -e 'bash -c \"roscore ; bash\" '")
#time.sleep(5)
#rosserial_rov()
#time.sleep(5)
joy()
