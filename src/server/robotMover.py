#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

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
		self.step_size = 0.05
		self.mode = 'STEP' # step, distance


	def step_mode(self, direction, stepSize):
		rospy.loginfo("Mode: " + self.mode + " " + direction + " " + str(stepSize) + " m")

		waypoints = []

		robot_pose = self.move_group.get_current_pose().pose

		if direction == "up":
			robot_pose.position.z += stepSize
			waypoints.append(copy.deepcopy(robot_pose))
		if direction == "down":
			robot_pose.position.z -= stepSize
			waypoints.append(copy.deepcopy(robot_pose))
		if direction == "left":
			robot_pose.position.y -= stepSize
			waypoints.append(copy.deepcopy(robot_pose))
		if direction == "right":
			robot_pose.position.y += stepSize
			waypoints.append(copy.deepcopy(robot_pose))
		if direction == "forward":
			robot_pose.position.x += stepSize
			waypoints.append(copy.deepcopy(robot_pose))
		if direction == "backward":
			robot_pose.position.x -= stepSize
			waypoints.append(copy.deepcopy(robot_pose))

		(plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  # jump_threshold
		self.move_group.execute(plan, wait=True)


	def handle_received_command(self, command):
		cmd = command.data.split(' ')

		#________________MOVE COMMANDS___________________________
		if cmd[0] == "MOVE":

			# step size depend on mode
			if self.mode == 'STEP':
				stepSize = self.step_size
			if self.mode == 'DISTANCE':
				stepSize = int(cmd[2]) / 100

			if cmd[1] == "UP":
				self.step_mode("up", stepSize)
			elif cmd[1] == "DOWN":
				self.step_mode("down", stepSize)
			elif cmd[1] == "LEFT":
				self.step_mode("left", stepSize)
			elif cmd[1] == "RIGHT":
				self.step_mode("right", stepSize)
			elif cmd[1] == "FORWARD":
				self.step_mode("forward", stepSize)
			elif cmd[1] == "BACKWARD":
				self.step_mode("backward", stepSize)
			else:
				rospy.loginfo("Command not found.")


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

		else:
			rospy.loginfo("Command not found.")


def main():
    robotMover = RobotMover()
    rospy.loginfo("RobotMover node started.")
    rospy.spin()

if __name__ == "__main__":
    main()