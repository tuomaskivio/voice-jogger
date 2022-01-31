#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__== '__main__':
	rospy.init_node('text_command_transmitter2')

	pub = rospy.Publisher("/text_commands", String, queue_size=10) # queue_size gives time for subscriber to process data it gets
	
	print(92*"_")
	print("")
	print("HOME                                  -Go to home position")
	print("")

	print("UP/DOWN/LEFT/RIGHT/FRONT/BACK         -Move amount of step size to the given direction")
	print("UP/DOWN [distance]                    -Move given distance (mm) to the given direction")
	print("")

	print("POSITION [position name]              -Move to saved position")
	print("")

	print("TASK/DO/PLAY [task name]              -Execute saved task")
	print("")

	print("OPEN/CLOSE                            -Open or close gripper")
	print("OPEN/CLOSE [0 - 80]                   -Open or close gripper given distance (mm)")
	print("ROTATE                                -Rotate gripper clockwise amount of step size")
	print("ROTATE OPPOSITE                       -Rotate gripper counter-clockwise amount of step size")
	print("")

	print("LIST TASKS/POSITIONS                  -List saved tasks or positions")
	print("SAVE POSITION [position name]         -Save robot's current position with name")
	print("REMOVE POSITION [position name]       -Remove saved position with name")
	print("")

	print("RECORD [task name]                    -Start recording task with name")
	print("REMOVE [task name]                    -Remove saved task with name")
	print("")

	print("STEP SIZE LOW/MEDIUM/HIGH             -Change step size: low(10mm), medium(50mm), high(100mm)")
	print("MODE STEP/DISTANCE                    -Change mode")
	print(92*"_")
	

	while not rospy.is_shutdown():
		inputCommand = input()
		msg = String()
		msg.data = inputCommand
		pub.publish(msg)

	rospy.loginfo("Node was stopped")
