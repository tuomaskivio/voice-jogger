#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
from rospy_message_converter import message_converter

def write_task(data):
	for taskname in data:
		for dataName in data[taskname]:
			# Originally type of start_pose is ros message: geometry_msgs/Pose
			# Convert it to python dict.
			if dataName == "start_pose":
				startPoseDict = message_converter.convert_ros_message_to_dictionary(data[taskname]["start_pose"])
				data[taskname]["start_pose"] = startPoseDict

	# Convert data-dict to JSON
	dataJSON = json.dumps(data, indent=4)

	# Open file. "w" - overwrite any existing content
	textfile = open("tasks.txt", "w")
	# Write to the file
	textfile.write(dataJSON)


def load_task():
	# Open file
	textfile = open("tasks.txt", "r")
	dataJSON = textfile.read()
	# Convert data to python format
	data = json.loads(dataJSON)

	for taskname in data:
		for dataName in data[taskname]:
			# Convert Python dict back to ros message
			if dataName == "start_pose":
				startPoseRosMessage = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose', data[taskname]["start_pose"])
				data[taskname]["start_pose"] = startPoseRosMessage
	return data


def write_position(data):
	for posename in data:
		pose = message_converter.convert_ros_message_to_dictionary(data[posename])
		data[posename] = pose
		
	dataJSON = json.dumps(data, indent=4)
	textfile = open("positions.txt", "w")
	textfile.write(dataJSON)
	
	
def load_position():
	textfile = open("positions.txt", "r")
	dataJSON = textfile.read()
	data = json.loads(dataJSON)
	
	for posename in data:
		pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose', data[posename])
		data[posename] = pose
	return data
	

def deleteItem(filename, name):
	data = load(filename)
	data.pop(name)
	write(filename, data)



