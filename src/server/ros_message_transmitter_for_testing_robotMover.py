#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__== '__main__':
	rospy.init_node('text_command_transmitter')

	pub = rospy.Publisher("/text_commands", String, queue_size=10) # queue_size gives time for subscriber to process data it gets
	
	print("Give capitalized commands:")
	print("Go to home position: ")
	print("HOME")
	print("")
	print("Stepping mode: ")
	print("MOVE UP, DOWN, LEFT, RIGHT, FORWARD, BACKWARD")
	print("")
	print("Direction and distance mode:")
	print("MOVE UP 10, MOVE DOWN 20...")
	print("")
	print("Change step size:")
	print("STEP SIZE LOW/MEDIUM/HIGH")
	print("")
	print("Change mode:")
	print("MODE STEP/DISTANCE")
	print("")
	print("Control gripper:")
	print("GRIPPER OPEN/CLOSE/[0 - 80] or only OPEN/CLOSE")
	print("")
	

	while not rospy.is_shutdown():
		inputCommand = input()
		msg = String()
		msg.data = inputCommand
		pub.publish(msg)

	rospy.loginfo("Node was stopped")
