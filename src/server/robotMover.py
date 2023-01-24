#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import copy
import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal
from franka_msgs.msg import ErrorRecoveryActionGoal
from word2number import w2n
from franka_gripper.msg import ( GraspAction, GraspGoal,
                                 HomingAction, HomingGoal,
                                 MoveAction, MoveGoal,
                                 StopAction, StopGoal,
                                 GraspEpsilon )

import textFileHandler as tfh

from enum import Enum
class Velocity(Enum):
    LOW = 1
    MEDIUM = 2
    HIGH = 3
velocities = {
    Velocity.LOW: 0.05,
    Velocity.MEDIUM: 0.2,
    Velocity.HIGH: 1.0
}

SIMULATION = False

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
        text_command_subscriber = rospy.Subscriber("/text_commands_priority", String, self.handle_received_priority_command)

        # Clients to send commands to the gripper
        self.grasp_action_client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        self.move_action_client = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        self.stop_action_client = actionlib.SimpleActionClient('/franka_gripper/stop', StopAction)

        # Publish to joint trajectory controller
        if SIMULATION:
            joint_trajectory_topic = '/effort_joint_trajectory_controller/follow_joint_trajectory/goal'
        else:
            joint_trajectory_topic = '/position_joint_trajectory_controller/follow_joint_trajectory/goal'
        self.joint_trajectory_goal_pub = rospy.Publisher(
                                      joint_trajectory_topic,
                                      FollowJointTrajectoryActionGoal, 
                                      queue_size = 10)

        # Publish to error recovery topic
        self.error_recovery_pub = rospy.Publisher(
                                        '/franka_control/error_recovery/goal',
                                        ErrorRecoveryActionGoal,
                                        queue_size = 10)

        # class constants
        self.pickup_area_offset = 0.05
        self.pickup_approach_height = 0.15
        self.default_pickup_height = 0.01
        self.object_size = 0.1
        self.home = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]

        # class variables
        self.stopped = True
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.step_size = 0.05
        self.mode = 'STEP'  # step, distance
        self.position1 = None
        self.position2 = None
        self.recording_task_name = None
        self.saved_positions = tfh.load_position()
        self.saved_tasks = tfh.load_task()
        # Updating waypoint is used to calculate final goal pose of AND command chain.
        self.updating_waypoint = []
        self.saved_objects = tfh.load_object()
        self.velocity = Velocity.MEDIUM
        self.waiting_for_tool_name = False
        self.current_tool = None
        
        
    def move_gripper_home(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.30701957
        pose.position.y = 0
        pose.position.z = 0.59026955
        pose.orientation.x = 0.92395569
        pose.orientation.y = -0.38249949
        pose.orientation.z = 0
        pose.orientation.w = 0

        self.move_robot([pose])


    def move_robot_home(self):
        self.move_group.set_max_velocity_scaling_factor(velocities[self.velocity])
        self.move_group.go(self.home, wait=True)
        self.move_group.stop()

    def move_robot(self, waypoints):
        if self.stopped:
            return
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)  # jump_threshold
        plan = self.move_group.retime_trajectory(self.move_group.get_current_state(), plan, velocity_scaling_factor = velocities[self.velocity])
        self.move_group.execute(plan, wait=True)
        
    def move_robot_to_position(self, position):
        # If there isn't saved position, do nothing but inform user
        if position in self.saved_positions.keys():
            target = self.saved_positions[position]
            rospy.loginfo("Robot moved to position " + position)
            waypoints = []
            waypoints.append(copy.deepcopy(target))
            self.move_robot(waypoints)
        else:
            rospy.loginfo("Position " + position + " not saved.")
            self.shake_gripper()
            
            
    def move_robot_cartesian(self, direction, stepSize, is_and=False, is_end_of_and=False):
        rospy.loginfo("Mode: " + self.mode + " " + direction + " " + str(stepSize) + " m")
        
        waypoints = []

        robot_pose = self.move_group.get_current_pose().pose

        # Calculate new position for updating waypoint.
        if is_and and len(self.updating_waypoint) != 0:
            if direction == "up":
                self.updating_waypoint[0].position.z += stepSize
            if direction == "down":
                self.updating_waypoint[0].position.z -= stepSize
            if direction == "left":
                self.updating_waypoint[0].position.y -= stepSize
            if direction == "right":
                self.updating_waypoint[0].position.y += stepSize
            if direction == "forward":
                self.updating_waypoint[0].position.x += stepSize
            if direction == "backward":
                self.updating_waypoint[0].position.x -= stepSize

            if not is_end_of_and:
                return

        else:
            # Calculate a new goal pose

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

            waypoint = copy.deepcopy(robot_pose)
            waypoints.append(waypoint)


            if is_and:
                self.updating_waypoint.append(waypoint)
                if not is_end_of_and:
                    return

        if is_and and is_end_of_and:
            waypoints.append(self.updating_waypoint[0])

        self.move_robot(waypoints)

        self.updating_waypoint = []
        
    def open_gripper(self, wait=True):
        if self.stopped:
            return
        if self.recording_task_name is not None:
            self.saved_tasks[self.recording_task_name]["moves"].append(['gripper', 'open'])
        movegoal = MoveGoal()
        movegoal.width = 0.08
        movegoal.speed = 0.05
        self.move_action_client.send_goal(movegoal)
        if wait:
            self.move_action_client.wait_for_result()

    def set_gripper_distance(self, distance):
        if self.stopped:
            return
        if self.recording_task_name is not None:
            self.saved_tasks[self.recording_task_name]["moves"].append(['gripper', 'distance', distance])
        movegoal = MoveGoal()
        movegoal.width = distance
        movegoal.speed = 0.05
        self.move_action_client.send_goal(movegoal)
        self.move_action_client.wait_for_result()

    def close_gripper(self):
        if self.stopped:
            return
        if self.recording_task_name is not None:
            self.saved_tasks[self.recording_task_name]["moves"].append(['gripper', 'close'])
        graspgoal = GraspGoal()
        graspgoal.width = 0.00
        graspgoal.speed = 0.05
        graspgoal.force = 2  # limits 0.01 - 50 N
        graspgoal.epsilon = GraspEpsilon(inner=0.08, outer=0.08)
        self.grasp_action_client.send_goal(graspgoal)
        self.grasp_action_client.wait_for_result()

    def rotate_gripper(self, stepSize, clockwise = True):
        if self.stopped:
            return
        # step size are: 0.01, 0.05, 0.1
        # Increase rotating step size
        stepSize = stepSize * 10
        if not clockwise:
            stepSize = stepSize * (-1)
        joint_goal = self.move_group.get_current_joint_values()

        # Joit 7 limits. max: 2.8973, min: -2.8973
        if joint_goal[6] + stepSize >= 2.8973:
            print("Joint 7 upper limit reached. Rotate counter-clockwise. Command: ROTATE BACK")
            self.shake_gripper()
        elif joint_goal[6] + stepSize <= -2.8973:
            print("Joint 7 lower limit reached. Rotate clockwise. Command: ROTATE")
            self.shake_gripper()
        else:
            joint_goal[6] = joint_goal[6] + stepSize
            value2decimals = "{:.2f}".format(joint_goal[6])
            rospy.loginfo("Gripper rotated. Joint 7 value: " + value2decimals + ". Max: 2.90, Min: -2.90.")
            print(joint_goal[6])
            self.move_group.set_max_velocity_scaling_factor(velocities[self.velocity])
            self.move_group.go(joint_goal, wait=True)

    def give_tool(self, name):
        if "CORNER1" not in self.saved_positions.keys() or "CORNER2" not in self.saved_positions.keys():
            rospy.loginfo("Corners not saved")
            self.shake_gripper()
            return

        # If there isn't saved position, dont't do nothing but inform user
        if name not in self.saved_objects.keys():
            rospy.loginfo("Object " + name + " not saved.")
            self.shake_gripper()
            return

        if not self.saved_objects[name][1]:
            rospy.loginfo("Object " + name + " not in place.")
            self.shake_gripper()
            return

        target = copy.deepcopy(self.saved_objects[name][0])
        approach_target = copy.deepcopy(target)
        approach_target.position.z += self.pickup_approach_height
        # Pick up tool
        self.open_gripper(wait=False) # Start opening gripper while moving
        self.move_robot([approach_target])
        self.open_gripper(wait=True) # Verify gripper is open before moving to target
        self.move_robot([target])
        self.close_gripper()
        # Drop tool
        target2 = copy.deepcopy(self.saved_positions["CORNER2"])
        target2.position.x -= self.pickup_area_offset
        target2.position.y -= self.pickup_area_offset
        approach_target2 = copy.deepcopy(target2)
        approach_target2.position.z += self.pickup_approach_height
        target2.position.z = self.saved_objects[name][0].position.z
        waypoints = []
        waypoints.append(approach_target)
        waypoints.append(approach_target2)
        waypoints.append(target2)
        self.move_robot(waypoints)
        self.open_gripper()
        # Move up
        self.move_robot([approach_target2])

        self.change_object_status(name, 0)

    def drop_tool(self, name):
        if "DROP" not in self.saved_positions.keys():
            rospy.loginfo("Drop position not saved")
            self.shake_gripper()
            return

        # If there isn't saved position, dont't do nothing but inform user
        if name not in self.saved_objects.keys():
            rospy.loginfo("Object " + name + " not saved.")
            self.shake_gripper()
            return

        if not self.saved_objects[name][1]:
            rospy.loginfo("Object " + name + " not in place.")
            self.shake_gripper()
            return

        target = copy.deepcopy(self.saved_objects[name][0])
        approach_target = copy.deepcopy(target)
        approach_target.position.z += self.pickup_approach_height
        # Pick up tool
        self.open_gripper(wait=False) # Start opening gripper while moving
        self.move_robot([approach_target])
        self.open_gripper(wait=True) # Verify gripper is open before moving to target
        self.move_robot([target])
        self.close_gripper()
        # Drop tool
        drop_target = copy.deepcopy(self.saved_positions["DROP"])
        waypoints = []
        waypoints.append(approach_target)
        waypoints.append(drop_target)
        self.move_robot(waypoints)
        self.open_gripper()

        self.change_object_status(name, 0)

    def drop_all(self):
        for tool in self.saved_objects.keys():
            if (self.saved_objects[tool][1]):
                self.drop_tool(tool)

    def pickup_tool(self, name=""):
        if "CORNER1" not in self.saved_positions.keys() or "CORNER2" not in self.saved_positions.keys():
            rospy.loginfo("Corners not saved")
            self.shake_gripper()
            return

        if name != "" and self.saved_objects[name][1]:
            rospy.loginfo("Object " + name + " is already in place.")
            self.shake_gripper()
            return

        target = copy.deepcopy(self.saved_positions["CORNER2"])
        target.position.x -= self.pickup_area_offset
        target.position.y -= self.pickup_area_offset
        approach_target = copy.deepcopy(target)
        approach_target.position.z += self.pickup_approach_height

        if name == "":
            target.position.z += self.default_pickup_height
        else:
            target.position.z = self.saved_objects[name][0].position.z

        # Go to pickup point
        self.open_gripper(wait=False) # Start opening gripper while moving
        self.move_robot([approach_target])
        self.open_gripper(wait=True) # Verify gripper is open before moving to target
        self.move_robot([target])
        # Pick tool
        self.close_gripper()
        # Move up
        waypoints = []
        waypoints.append(approach_target)
        if name == "":
            self.move_robot(waypoints)
            self.waiting_for_tool_name = True
        else:
            target2 = copy.deepcopy(self.saved_objects[name][0])
            approach_target2 = copy.deepcopy(target2)
            approach_target2.position.z = approach_target.position.z
            waypoints.append(approach_target2)
            waypoints.append(target2)
            self.move_robot(waypoints)
            self.open_gripper()
            self.move_robot([approach_target2])

            self.change_object_status(name, 1)

        self.current_tool = None

        
    def save_tool(self, name):
        if "CORNER1" not in self.saved_positions.keys() or "CORNER2" not in self.saved_positions.keys():
            rospy.loginfo("Corners not saved")
            self.shake_gripper()
            return

        orig_target = copy.deepcopy(self.saved_positions["CORNER1"])
        orig_target.position.x += 0.01
        orig_target.position.y += 0.01
        target = copy.deepcopy(orig_target)
        while not self.is_free(target.position):
            target.position.x += 0.01
            if target.position.x + self.object_size > self.saved_positions["CORNER2"].position.x:
                target.position.x = orig_target.position.x
                target.position.y += 0.01
                if target.position.y + self.object_size > self.saved_positions["CORNER2"].position.y:
                    rospy.loginfo("Table is full")
                    self.shake_gripper()
                    self.open_gripper()
                    self.waiting_for_tool_name = False
                    return
        target.position.z = self.saved_positions["CORNER1"].position.z + self.default_pickup_height
        # Save tool position
        if name in self.saved_objects.keys():
            rospy.loginfo("There was already a stored object with the name %s so it was overwritten", name)
        self.saved_objects[name] = [copy.deepcopy(target), 1]
        tfh.write_object(self.saved_objects)
        self.saved_objects = tfh.load_object()
        print("Object " + name + " saved.")
        self.waiting_for_tool_name = False

        waypoints = []
        approach_target = copy.deepcopy(target)
        approach_target.position.z = self.saved_positions["CORNER1"].position.z + self.pickup_approach_height
        waypoints.append(approach_target)
        waypoints.append(target)
        self.move_robot(waypoints)
        self.open_gripper()
        self.move_robot([approach_target])


    def is_free(self, target):
        for position in self.saved_objects.keys():
            position = self.saved_objects[position][0].position
            
            if position.x > target.x - self.object_size and \
               position.x < target.x + self.object_size and \
               position.y > target.y - self.object_size and \
               position.y < target.y + self.object_size:
                return False
        
        # Check pickup area
        position = self.saved_positions["CORNER2"].position
        if position.x - self.pickup_area_offset > target.x - self.object_size and \
           position.x - self.pickup_area_offset < target.x + self.object_size and \
           position.y - self.pickup_area_offset > target.y - self.object_size and \
           position.y - self.pickup_area_offset < target.y + self.object_size:
            return False
        return True

    def change_object_status(self, name, status):
        self.saved_objects[name][1] = status
        tfh.write_object(self.saved_objects)
        self.saved_objects = tfh.load_object()

    def robot_stop(self):
        # Replace current trajectory with stopping trajectory
        joint_values = self.move_group.get_current_joint_values()
        print(joint_values)
        stop_trajectory = JointTrajectory()
        stop_trajectory.joint_names = ['panda_joint1',
                                       'panda_joint2',
                                       'panda_joint3',
                                       'panda_joint4',
                                       'panda_joint5',
                                       'panda_joint6',
                                       'panda_joint7']
        stop_trajectory.points.append(JointTrajectoryPoint())
        stop_trajectory.points[0].positions = joint_values
        stop_trajectory.points[0].velocities = [0.0 for i in joint_values]
        stop_trajectory.points[0].time_from_start = rospy.Duration(1) # Stopping time
        stop_trajectory.header.frame_id = 'world'
        stop_goal = FollowJointTrajectoryActionGoal()
        stop_goal.goal.trajectory = stop_trajectory
        self.joint_trajectory_goal_pub.publish(stop_goal)

        # Stop gripper
        goal = StopGoal()
        self.stop_action_client.send_goal(goal)
        self.stopped = True
        rospy.loginfo("Stopped")

    def error_recovery(self):
        error_recovery_goal = ErrorRecoveryActionGoal()
        self.error_recovery_pub.publish(error_recovery_goal)
        rospy.loginfo("Recovered from errors")

    def robot_start(self):
        self.stopped = False
        self.error_recovery()
        rospy.loginfo("Started")

    def shake_gripper(self):
        if self.stopped:
            return            
        joint_goal = self.move_group.get_current_joint_values()
        step_size = 0.1
        # Joit 7 limits. max: 2.8973, min: -2.8973
        if joint_goal[6] + step_size >= 2.8973:
            step_size = -1 * step_size
            
        joint_goal[6] = joint_goal[6] + step_size
        self.move_group.set_max_velocity_scaling_factor(velocities[Velocity.HIGH])
        self.move_group.go(joint_goal, wait=True)
        joint_goal[6] = joint_goal[6] - step_size
        self.move_group.go(joint_goal, wait=True)
        self.move_group.set_max_velocity_scaling_factor(velocities[self.velocity])

    def handle_received_priority_command(self, command):
        if type(command) == String:
            cmd = command.data.split(' ')
        elif type(command) == list:
            cmd = command

        #________________SAFETY COMMANDS___________________________
        if cmd[0] == "STOP":
            self.robot_stop()
        
        
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
            #elif cmd[0] == "RECORD":
            #    pass
            #else:
                #self.saved_tasks[self.recording_task_name]["moves"].append(cmd)


        # Bool parameters for command chaining
        and_bool_parameter = False
        is_end_bool_parameter = False
        if cmd[0] == "UP" or cmd[0] == "DOWN" or cmd[0] == "LEFT" or cmd[0] == "RIGHT" \
                or cmd[0] == "FORWARD" or cmd[0] == "BACKWARD":
            if self.mode == 'STEP':
                if len(cmd) > 1:
                    if cmd[1] == "False":
                        and_bool_parameter = False
                    else:
                        and_bool_parameter = True
                if len(cmd) > 2:
                    if cmd[2] == "False":
                        is_end_bool_parameter = False
                    else:
                        is_end_bool_parameter = True
            elif self.mode == 'DISTANCE':
                if len(cmd) > 2:
                    if cmd[2] == "False":
                        and_bool_parameter = False
                    else:
                        and_bool_parameter = True
                if len(cmd) > 3:
                    if cmd[3] == "False":
                        is_end_bool_parameter = False
                    else:
                        is_end_bool_parameter = True


        #________________STATUS COMMANDS_________________________
        if cmd[0] == 'START':
            self.robot_start()
        elif cmd[0] == 'RECOVER':
            self.error_recovery()
            
        #________________MOVE COMMANDS___________________________
        elif cmd[0] == "HOME":
            self.move_robot_home()

        elif cmd[0] == "MOVE":

            if cmd[1] != 'POSITION':
                # Bool parameters for command chaining
                if self.mode == 'STEP':
                    if len(cmd) > 2:
                        if cmd[2] == "False":
                            and_bool_parameter = False
                        else:
                            and_bool_parameter = True
                    if len(cmd) > 3:
                        if cmd[3] == "False":
                            is_end_bool_parameter = False
                        else:
                            is_end_bool_parameter = True
                elif self.mode == 'DISTANCE':
                    if len(cmd) > 3:
                        if cmd[3] == "False":
                            and_bool_parameter = False
                        else:
                            and_bool_parameter = True
                    if len(cmd) > 4:
                        if cmd[4] == "False":
                            is_end_bool_parameter = False
                        else:
                            is_end_bool_parameter = True

            # step size depend on mode
            stepSize = self.step_size
            if self.mode == 'STEP':
                stepSize = self.step_size
            if self.mode == 'DISTANCE':
                stepSize = float(cmd[2]) / 1000

            if cmd[1] == "UP":
                self.move_robot_cartesian("up", stepSize, and_bool_parameter, is_end_bool_parameter)
            elif cmd[1] == "DOWN":
                self.move_robot_cartesian("down", stepSize, and_bool_parameter, is_end_bool_parameter)
            elif cmd[1] == "LEFT":
                self.move_robot_cartesian("left", stepSize, and_bool_parameter, is_end_bool_parameter)
            elif cmd[1] == "RIGHT":
                self.move_robot_cartesian("right", stepSize, and_bool_parameter, is_end_bool_parameter)
            elif cmd[1] == "FORWARD":
                self.move_robot_cartesian("forward", stepSize, and_bool_parameter, is_end_bool_parameter)
            elif cmd[1] == "BACKWARD":
                self.move_robot_cartesian("backward", stepSize, and_bool_parameter, is_end_bool_parameter)

            elif cmd[1] == 'POSITION': # move robot to saved position
                self.move_robot_to_position(cmd[2])


            else:
                rospy.loginfo("Command not found.")
                self.shake_gripper()

        # Without word MOVE
        elif cmd[0] == "UP":
            if self.mode == 'STEP':
                stepsize = self.step_size
            else:
                if len(cmd) > 1:
                    stepsize = get_number(cmd[1:]) / 1000
                else:
                    stepsize = self.step_size
            self.move_robot_cartesian("up", stepsize, and_bool_parameter, is_end_bool_parameter)
        elif cmd[0] == "DOWN":
            if self.mode == 'STEP':
                stepsize = self.step_size
            else:
                if len(cmd) > 1:
                    stepsize = float(get_number(cmd[1:])) / 1000
                else:
                    stepsize = self.step_size
            self.move_robot_cartesian("down", stepsize, and_bool_parameter, is_end_bool_parameter)
        elif cmd[0] == "LEFT":
            if self.mode == 'STEP':
                stepsize = self.step_size
            else:
                if len(cmd) > 1:
                    stepsize = float(get_number(cmd[1:])) / 1000
                else:
                    stepsize = self.step_size
            self.move_robot_cartesian("left", stepsize, and_bool_parameter, is_end_bool_parameter)
        elif cmd[0] == "RIGHT":
            if self.mode == 'STEP':
                stepsize = self.step_size
            else:
                if len(cmd) > 1:
                    stepsize = float(get_number(cmd[1:])) / 1000
                else:
                    stepsize = self.step_size
            self.move_robot_cartesian("right", stepsize, and_bool_parameter, is_end_bool_parameter)
        elif cmd[0] == "FORWARD" or cmd[0] == "FRONT":
            if self.mode == 'STEP':
                stepsize = self.step_size
            else:
                if len(cmd) > 1:
                    stepsize = float(get_number(cmd[1:])) / 1000
                else:
                    stepsize = self.step_size
            self.move_robot_cartesian("forward", stepsize, and_bool_parameter, is_end_bool_parameter)
        elif cmd[0] == "BACKWARD" or cmd[0] == "BACK":
            if self.mode == 'STEP':
                stepsize = self.step_size
            else:
                if len(cmd) > 1:
                    stepsize = float(get_number(cmd[1:])) / 1000
                else:
                    stepsize = self.step_size
            self.move_robot_cartesian("backward", stepsize, and_bool_parameter, is_end_bool_parameter)
            
        elif cmd[0] == 'POSITION': # move robot to saved position
            self.move_robot_to_position(cmd[1])
        
        #________________GRIPPER COMMANDS_________________________
        elif cmd[0] == "GRIPPER" or cmd[0] == "TOOL":
            if len(cmd) == 2:
                if cmd[1] == "OPEN":
                    self.open_gripper()
                elif cmd[1] == "CLOSE":
                    self.close_gripper()
                elif cmd[1] == "ROTATE" or cmd[1] == "TURN" or cmd[1] == "SPIN":
                    self.rotate_gripper(self.step_size)
                elif cmd[1] == "HOME":
                    self.move_gripper_home()
                else:
                    try:
                        distance = float(cmd[1]) / 1000
                        self.set_gripper_distance(distance)
                    except ValueError:
                        rospy.loginfo('Invalid gripper command "%s" received, available commands are:', cmd[1])
                        rospy.loginfo('OPEN, CLOSE, ROTATE or distance between fingers in units mm between 0-80')
                        self.shake_gripper()
            if len(cmd) == 3:
                if cmd[1] == "ROTATE" or cmd[1] == "TURN" or cmd[1] == "SPIN":
                    if cmd[2] == 'BACK':
                        self.rotate_gripper(self.step_size, False)
                        
        elif cmd[0] == "OPEN":
            self.open_gripper()
        elif cmd[0] == "CLOSE" or cmd[0] == "GRASP":
            print(cmd)
            self.close_gripper()
        elif cmd[0] == "ROTATE" or cmd[0] == "TURN" or cmd[0] == "SPIN":
            if len(cmd) < 2:
                self.rotate_gripper(self.step_size)
            else:
                if cmd[1] == 'BACK':
                    self.rotate_gripper(self.step_size, False)


        
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
                self.shake_gripper()


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
                self.shake_gripper()


        #________________CHANGE VELOCITY________________________
        elif cmd[0] == 'VELOCITY':
            if cmd[1] == 'LOW':
                self.velocity = Velocity.LOW
                rospy.loginfo("Velocity LOW")
            elif cmd[1] == 'MEDIUM':
                self.velocity = Velocity.MEDIUM
                rospy.loginfo("Velocity MEDIUM")
            elif cmd[1] == 'HIGH':
                self.velocity = Velocity.HIGH
                rospy.loginfo("Velocity HIGH")
            else:
                rospy.loginfo("Command not found.")
                self.shake_gripper()


        #________________SAVE ROBOT POSITION_____________________
        elif cmd[0] == 'SAVE' and cmd[1] == 'POSITION':
            if cmd[2] in self.saved_positions.keys():
                rospy.loginfo("There was already a stored position with the name %s so it was overwritten", cmd[2])
            self.saved_positions[cmd[2]] = self.move_group.get_current_pose().pose
            tfh.write_position(self.saved_positions)
            self.saved_positions = tfh.load_position()
            print("Position " + cmd[2] + " saved.")
            
            
        #_______________REMOVE ROBOT POSITION____________________
        elif cmd[0] == 'REMOVE' and cmd[1] == 'POSITION':
            if cmd[2] in self.saved_positions.keys():
                tfh.deleteItem('positions.txt', cmd[2])
                self.saved_positions = tfh.load_position()
                print("Position " + cmd[2] + " removed.")
            else:
                rospy.loginfo("Not enough arguments, expected REMOVE POSITION [position name]")
                self.shake_gripper()

        #________________SAVE TOOL POSITION_____________________
        elif cmd[0] == 'SAVE' and cmd[1] == 'TOOL':
            if cmd[2] in self.saved_objects.keys():
                rospy.loginfo("There was already a stored tool with the name %s so it was overwritten", cmd[2])
            self.saved_objects[cmd[2]] = [self.move_group.get_current_pose().pose, 1]
            tfh.write_object(self.saved_objects)
            self.saved_objects = tfh.load_object()
            print("Tool " + cmd[2] + " saved.")
            
            
        #_______________REMOVE TOOL POSITION____________________
        elif cmd[0] == 'REMOVE' and cmd[1] == 'TOOL':
            if cmd[2] in self.saved_objects.keys():
                tfh.deleteItem('objects.txt', cmd[2])
                self.saved_objects = tfh.load_object()
                print("Tool " + cmd[2] + " removed.")
            else:
                rospy.loginfo("Not enough arguments, expected REMOVE TOOL [tool name]")
                self.shake_gripper()

        #_______________ TAKE TOOL__________________________
        elif cmd[0] == 'TAKE':
            if cmd[1] == 'NEW':
                if len(cmd) == 2:
                    # Pickup new tool
                    self.pickup_tool()
                elif self.waiting_for_tool_name:
                    # Save tool with name and take it to empty position
                    self.save_tool(cmd[2])
                else:
                    print('No tool to name')
                    self.shake_gripper()
            elif len(cmd) > 1:
                self.pickup_tool(cmd[1])
            else:
                rospy.loginfo("Not enough arguments, expected TAKE [tool name] or TAKE NEW TOOL")
                self.shake_gripper()

        elif cmd[0] == 'RETURN':
            if not self.current_tool:
                rospy.loginfo("No current tool to return")
                self.shake_gripper()
            else:
                self.pickup_tool(self.current_tool)


        #___________________GIVE TOOL____________________________
        elif cmd[0] == 'GIVE':
            if len(cmd) > 1:
                self.give_tool(cmd[1])
                self.current_tool = cmd[1]
            else:
                rospy.loginfo("Not enough arguments, expected GIVE [tool name]")
                self.shake_gripper()

        #___________________DROP TOOL____________________________
        elif cmd[0] == 'DROP':
            if len(cmd) > 1:
                if cmd[1] == 'ALL':
                    self.drop_all()
                else:
                    self.drop_tool(cmd[1])
            else:
                rospy.loginfo("Not enough arguments, expected GIVE [tool name]")
                self.shake_gripper()

            
        #___________________TASK RECORDINGS______________________
        elif cmd[0] == 'RECORD':
            if len(cmd) > 1:
                rospy.loginfo("Begin recording task with name %s", cmd[1])
                self.recording_task_name = cmd[1]
                if self.recording_task_name not in self.saved_tasks.keys():
                    self.saved_tasks[self.recording_task_name] = {}
                    self.saved_tasks[self.recording_task_name]["moves"] = []
            else:
                print("RECORD error: give task name.")
                self.shake_gripper()
        
        elif cmd[0] == 'TASK' or cmd[0] == 'DO' or cmd[0] == 'PLAY':
            if len(cmd) > 1:
                if cmd[1] in self.saved_tasks.keys():

                    waypoints = []
                    for step in self.saved_tasks[cmd[1]]["moves"]:
                        if self.stopped:
                            break
                        if step[0] == 'pose':
                            waypoints.append(copy.deepcopy(step[1]))
                        elif len(waypoints) > 0:
                            self.move_robot(waypoints)
                            waypoints = []
                        if step[0] == 'gripper':
                            if step[1] == 'open':
                                self.open_gripper()
                            elif step[1] == 'distance':
                                self.set_gripper_distance(step[2])
                            elif step[1] == 'close':
                                self.close_gripper()
                    if len(waypoints) > 0:
                        self.move_robot(waypoints)
                else:
                    print("Executing task failed: Task name " + cmd[1] + " not in recorded tasks.")
                    self.shake_gripper()
            else:
                print("Executing task failed: Correct command: TASK/DO/PLAY [task name]")
                self.shake_gripper()


        #___________________TEXT FILE HANDLING______________________
        #___________________LIST TASKS/POSITIONS______________________
        elif cmd[0] == 'LIST':
            if len(cmd) != 1:
                if cmd[1] == 'TASKS':
                    print("")
                    print("Saved tasks:")
                    print("")
                    self.saved_tasks = tfh.load_task()
                    for taskname in self.saved_tasks:
                        print(taskname)    
                    print("")
                elif cmd[1] == 'POSITIONS':
                    print("")
                    print("Saved positions:")
                    print("")
                    self.saved_positions = tfh.load_position()
                    for position in self.saved_positions:
                        print(position)
                    print("")
                else:
                    print("Listing failed. List tasks: LIST TASKS. List positions: LIST POSITIONS.")
                    self.shake_gripper()
            else:
                print("Listing failed. List tasks: LIST TASKS. List positions: LIST POSITIONS.")
                self.shake_gripper()


        elif cmd[0] == 'REMOVE':
            if len(cmd) < 2:
                print("REMOVE error: give task name.")
                self.shake_gripper()
            elif len(cmd) > 2:
                print("REMOVE error: Too many arguments.")
                self.shake_gripper()
            else:
                if cmd[1] in self.saved_tasks.keys():
                    tfh.deleteItem("tasks.txt", cmd[1])
                    self.saved_tasks = tfh.load_task()
                    print("Task " + cmd[1] + " removed.")
                else:
                    print("REMOVE error: No task named " + cmd[1] + " found. Use LIST TASK command too see tasks.")
                    self.shake_gripper()
                
        else:
            print("Command not found.")
            self.shake_gripper()
          
        #___________________RECORD WAYPOINT______________________
        if self.recording_task_name is not None:
            pose = copy.deepcopy(self.move_group.get_current_pose().pose)
            self.saved_tasks[self.recording_task_name]["moves"].append(['pose', pose])  

def get_number(words):
    number_words = copy.copy(words)
    print("print numbers: ", number_words)
    # Replace words that sound like number with numbers
    try:
        value = float(w2n.word_to_num(' '.join(number_words)))
        return value
    except Exception as a:
        print("Invalid number.")
        return 0


def main():
    robotMover = RobotMover()
    rospy.loginfo("RobotMover node started.")
    rospy.spin()

if __name__ == "__main__":
    main()
