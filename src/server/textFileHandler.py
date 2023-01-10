#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
from rospy_message_converter import message_converter

def write_task(data):
	for taskname in data:
		for dataName in data[taskname]:
			if dataName == 'moves':
				for move in data[taskname][dataName]:
					# Originally type of pose is ros message: geometry_msgs/Pose
					# Convert it to python dict.
					if move[0] == "pose":
						poseDict = message_converter.convert_ros_message_to_dictionary(move[1])
						move[1] = poseDict

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
			if dataName == 'moves':
				for move in data[taskname][dataName]:
					# Convert Python dict back to ros message
					if move[0] == "pose":
						poseRosMessage = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose', move[1])
						move[1] = poseRosMessage
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
	if filename == "positions.txt":
		data = load_position()
		data.pop(name)
		write_position(data)
		
	else:
		data = load_task()
		data.pop(name)
		write_task(data)



