#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import pickle

from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import textFileHandler as tfh

class RobotMover(object):
	def __init__(self):
		super(RobotMover, self).__init__()

		# Initialize moveit_commander and a rospy node
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node("robot_mover", anonymous=True)

		# Instantiate a RobotCommander object. Provides information such as the robot's
		# kinematic model and the robot's current joint states
		robot = moveit_commander.RobotCommander()

		# Instantiate a PlanningSceneInterface object.  This provides a remote interface
		# for getting, setting, and updating the robot's internal understanding of the
		# surrounding world:
		scene = moveit_commander.PlanningSceneInterface()
        
		# Instantiate a  MoveGroupCommander object.  This object is an interface
		# to a planning group (group of joints).  In this tutorial the group is the primary
		# arm joints in the Panda robot, so we set the group's name to "panda_arm".
		# If you are using a different robot, change this value to the name of your robot
		# arm planning group.
		# This interface can be used to plan and execute motions:
		group_name = "panda_arm"
		move_group = moveit_commander.MoveGroupCommander(group_name)
		
		group_name = "panda_arm_hand"
		move_group_gripper = moveit_commander.MoveGroupCommander(group_name)
        
		# Create a `DisplayTrajectory`_ ROS publisher which is used to display
		# trajectories in Rviz:
		display_trajectory_publisher = rospy.Publisher(
				"/move_group/display_planned_path",
			moveit_msgs.msg.DisplayTrajectory,
			queue_size=20,
		)
        
		# Subscriber for text commands
		text_command_subscriber = rospy.Subscriber("/text_commands", String, self.handle_received_command)
        
		# class variables
		self.robot = robot
		self.scene = scene
		self.move_group = move_group
		self.move_group_gripper = move_group_gripper
		self.step_size = 0.05
		self.mode = 'STEP' # step, distance
		self.position1 = None
		self.position2 = None
		self.recording_task_name = None
		self.saved_positions = tfh.load_position()
#		self.saved_positions = {}
		self.saved_tasks = tfh.load_task()
		
		
	def move_robot_home(self):
		pose = geometry_msgs.msg.Pose()
		pose.position.x = 0.3
		pose.position.y = 0
		pose.position.z = 0.6
		pose.orientation.x = 1
		pose.orientation.y = -0.43
		pose.orientation.z = 0
		pose.orientation.w = -0.01
		
		(plan, fraction) = self.move_group.compute_cartesian_path([pose], 0.01, 0.0)  # jump_threshold
		self.move_group.execute(plan, wait=True)
	
		
	def move_robot_to_position(self, position):
		target = self.saved_positions[position]
        
		# If there isn't saved position, dont't do nothing but inform user
		if target != None:
			rospy.loginfo("Robot moved to position " + position)
			waypoints = []
			waypoints.append(copy.deepcopy(target))
			(plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  # jump_threshold
			self.move_group.execute(plan, wait=True)
		else:
			rospy.loginfo("Position " + position + " not saved.")
            
			
	def move_robot_cartesian(self, direction, stepSize):
		rospy.loginfo("Mode: " + self.mode + " " + direction + " " + str(stepSize) + " m")
        
		waypoints = []
		robot_pose = self.move_group.get_current_pose().pose
		
		if direction == "up":
			robot_pose.position.z += stepSize
		if direction == "down":
			robot_pose.position.z -= stepSize
		if direction == "left":
			robot_pose.position.y -= stepSize
		if direction == "right":
			robot_pose.position.y += stepSize
		if direction == "forward":
			robot_pose.position.x += stepSize
		if direction == "backward":
			robot_pose.position.x -= stepSize
			
		waypoints.append(copy.deepcopy(robot_pose))
		(plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  # jump_threshold
		self.move_group.execute(plan, wait=True)
        
        
	def set_gripper(self, distance):
		rospy.loginfo("Set gripper to %s mm", distance*1000)
		joint_goal = self.move_group_gripper.get_current_joint_values()
		
		joint_goal[7] = distance/2
		joint_goal[8] = distance/2
            
		self.move_group_gripper.go(joint_goal, wait=True)
		self.move_group_gripper.stop()
        
        
	def handle_received_command(self, command):
		if type(command) == String:
			cmd = command.data.split(' ')
		elif type(command) == list:
			cmd = command
			
		if self.recording_task_name is not None:
			if cmd[0] == "FINISH":
				rospy.loginfo("Finished recording task with name %s", self.recording_task_name)

				# Write tasks to text file
				tfh.write_task(self.saved_tasks)
				# Load tasks from text file. 
				self.saved_tasks = tfh.load_task()

				self.recording_task_name = None
			elif cmd[0] == "RECORD":
				pass
			else:
				self.saved_tasks[self.recording_task_name]["moves"].append(cmd)
        
        #________________MOVE COMMANDS___________________________
		if cmd[0] == "HOME":
			self.move_robot_home()
		
		elif cmd[0] == "MOVE":

			# step size depend on mode
			if self.mode == 'STEP':
				stepSize = self.step_size
			if self.mode == 'DISTANCE':
				stepSize = float(cmd[2]) / 100

			if cmd[1] == "UP":
				self.move_robot_cartesian("up", stepSize)
			elif cmd[1] == "DOWN":
				self.move_robot_cartesian("down", stepSize)
			elif cmd[1] == "LEFT":
				self.move_robot_cartesian("left", stepSize)
			elif cmd[1] == "RIGHT":
				self.move_robot_cartesian("right", stepSize)
			elif cmd[1] == "FORWARD":
				self.move_robot_cartesian("forward", stepSize)
			elif cmd[1] == "BACKWARD":
				self.move_robot_cartesian("backward", stepSize)

			elif cmd[1] == 'POSITION': # move robot to saved position
				self.move_robot_to_position(cmd[2])


			else:
				rospy.loginfo("Command not found.")
		
		elif cmd[0] == "UP":
			self.move_robot_cartesian("up", self.step_size)
		elif cmd[0] == "DOWN":
			self.move_robot_cartesian("down", self.step_size)
		elif cmd[0] == "LEFT":
			self.move_robot_cartesian("left", self.step_size)
		elif cmd[0] == "RIGHT":
			self.move_robot_cartesian("right", self.step_size)
		elif cmd[0] == "FORWARD":
			self.move_robot_cartesian("forward", self.step_size)
		elif cmd[0] == "BACKWARD":
			self.move_robot_cartesian("backward", self.step_size)
			
		elif cmd[0] == 'POSITION': # move robot to saved position
			self.move_robot_to_position(cmd[1])
        
        #________________GRIPPER COMMANDS_________________________
		elif cmd[0] == "GRIPPER" or cmd[0] == "TOOL":
			if len(cmd) == 2:
				if cmd[1] == "OPEN":
					self.set_gripper(0.08)
				elif cmd[1] == "CLOSE":
					self.set_gripper(0)
				else:
					try:
						distance = float(cmd[1]) / 1000
						self.set_gripper(distance)
					except ValueError:
						rospy.loginfo('Invalid gripper command "%s" received, available commands are:', cmd[1])
						rospy.loginfo('OPEN, CLOSE, or distance between fingers in units mm between 0-80')
						
		elif cmd[0] == "OPEN":
			self.set_gripper(0.08)
		elif cmd[0] == ("CLOSE" or "GRASP"):
			print(cmd)
			self.set_gripper(0)
        
		#________________CHANGE MODE_____________________________
		elif cmd[0] == "MODE":
			if cmd[1] == "STEP":
				self.mode = 'STEP'
				rospy.loginfo("Mode: STEP")
			elif cmd[1] == "DISTANCE":
				self.mode = 'DISTANCE'
				rospy.loginfo("Mode: DIRECTION AND DISTANCE.")
			else:
				rospy.loginfo("Command not found.")


		#________________CHANGE STEP SIZE________________________
		elif cmd[0] == 'STEP' and cmd[1] == 'SIZE':
			if cmd[2] == 'LOW':
				self.step_size = 0.01
				rospy.loginfo("Step size LOW (1 cm)")
			elif cmd[2] == 'MEDIUM':
				self.step_size = 0.05
				rospy.loginfo("Step size MEDIUM (5 cm)")
			elif cmd[2] == 'HIGH':
				self.step_size = 0.1
				rospy.loginfo("Step size HIGH (10 cm)")
			else:
				rospy.loginfo("Command not found.")


		#________________SAVE ROBOT POSITION_____________________
		elif cmd[0] == 'SAVE' and cmd[1] == 'POSITION':
			if cmd[2] in self.saved_positions.keys():
				rospy.loginfo("There was already a stored position with the name %s so it was overwritten", cmd[2])
			self.saved_positions[cmd[2]] = self.move_group.get_current_pose().pose
			tfh.write_position(self.saved_positions)
			self.saved_positions = tfh.load_position()
			
			
		#___________________TASK RECORDINGS______________________
		elif cmd[0] == 'RECORD':
			rospy.loginfo("Begin recording task with name %s", cmd[1])
			self.recording_task_name = cmd[1]
			if self.recording_task_name not in self.saved_tasks.keys():
				self.saved_tasks[self.recording_task_name] = {}
				self.saved_tasks[self.recording_task_name]["start_step_size"] = self.step_size
				start_pose = copy.deepcopy(self.move_group.get_current_pose().pose)
				self.saved_tasks[self.recording_task_name]["start_pose"] = start_pose
				self.saved_tasks[self.recording_task_name]["moves"] = []
		
		elif cmd[0] in self.saved_tasks.keys():
			step_size_before_task = self.step_size
			self.step_size = self.saved_tasks[cmd[0]]["start_step_size"]
			start_pose = self.saved_tasks[cmd[0]]["start_pose"]
			(plan, fraction) = self.move_group.compute_cartesian_path([start_pose], 0.01, 0.0)
			self.move_group.execute(plan, wait=True)
			for step in self.saved_tasks[cmd[0]]["moves"]:
				print("calling handle received command with params", step)
				self.handle_received_command(step)
			self.step_size = step_size_before_task


		#___________________TEXT FILE HANDLING______________________
		elif cmd[0] == 'LIST' and cmd[1] == 'TASKS':
			print("")
			print("Saved tasks:")
			print("")
			for taskname in self.saved_tasks:
				print(taskname)	
			print("")

		elif cmd[0] == 'REMOVE':
			if len(cmd) < 2:
				print("REMOVE error: give task name.")
			elif len(cmd) > 2:
				print("REMOVE error: Too many arguments.")
			else:
				if cmd[1] in self.saved_tasks.keys():
					tfh.deleteItem("tasks.txt", cmd[1])
					self.saved_tasks = tfh.load_task()
					print("Task " + cmd[1] + " removed.")
				else:
					print("REMOVE error: No task named " + cmd[1] + " found. Use LIST TASK command too see tasks.")
				
		
		else:
			rospy.loginfo("Command not found.")



def main():
	robotMover = RobotMover()
	rospy.loginfo("RobotMover node started.")
	rospy.spin()

if __name__ == "__main__":
    main()