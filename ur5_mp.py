#!/usr/bin/env python3

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import Twist
import moveit_msgs.msg
from sensor_msgs.msg import Image
from ur5_notebook.msg import Tracker
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from time import sleep
tracker = Tracker()

# Create a class called ur5_mp
class ur5_mp:
    # Initialize the ur5_mp object
    def __init__(self):
        rospy.init_node("ur5_mp", anonymous=False)
        # subscribe to cxy (coming from ur5_vision node)
        self.cxy_sub = rospy.Subscriber('cxy', Tracker, self.tracking_callback, queue_size=1)
        # publish to cxy1 (send to vacuum grippers)
        self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)
        self.phase = 1
        self.object_cnt = 0
        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0 # center of image (800x800)
        self.cy = 400.0 # center of image (800x800)
        self.points=[]
        self.state_change_time = rospy.Time.now()

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.1)
        self.arm.set_max_acceleration_scaling_factor(.5)
        self.arm.set_max_velocity_scaling_factor(.5)

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # Initialize the waypoints list
        self.waypoints= []
        self.pointx = []
        self.pointy = []
        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)

        # Set the next waypoint to the right 0.5 meters

        wpose.position.x = -0.2
        wpose.position.y = -0.2
        wpose.position.z = 0.3
        self.waypoints.append(deepcopy(wpose))

        # wpose.position.x = 0.1052
        # wpose.position.y = -0.4271
        # wpose.position.z = 0.4005
        #
        # wpose.orientation.x = 0.4811
        # wpose.orientation.y = 0.5070
        # wpose.orientation.z = -0.5047
        # wpose.orientation.w = 0.5000

        # self.waypoints.append(deepcopy(wpose))

        # if the distance between the position you want to go to and the initial position is small, warn the user
        if np.sqrt((wpose.position.x-start_pose.position.x)**2+(wpose.position.x-start_pose.position.x)**2 \
            +(wpose.position.x-start_pose.position.x)**2)<0.1:
            rospy.loginfo("Warnig: target position overlaps with the initial position!")

        # self.arm.set_pose_target(wpose)

        # Specify default (idle) joint states
        self.default_joint_states = self.arm.get_current_joint_values()
        self.default_joint_states[0] = -1.57691
        self.default_joint_states[1] = -1.71667
        self.default_joint_states[2] = 1.79266
        self.default_joint_states[3] = -1.67721
        self.default_joint_states[4] = -1.5705
        self.default_joint_states[5] = 0.0

        self.arm.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()
        plan = self.arm.plan()

        self.arm.execute(plan[1])

        # Specify end states (drop object)
        self.end_joint_states = deepcopy(self.default_joint_states) # hovering above the box
        self.end_joint_states[0] = -3.65 # drop into the box
        # self.end_joint_states[1] = -1.3705

        self.transition_pose = deepcopy(self.default_joint_states)
        self.transition_pose[0] = -3.65 # Start from the dropped state
        self.transition_pose[4] = -1.95 # lift up 

    # Function that stops the robot and shuts down moveit cleanly
    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    # This gets the info from the robot and saves data to publish to cxy1 for the vacuum grippers
    # This is the main method that is called that describes the workflow
    def tracking_callback(self, msg):

        # Define parameters
        self.track_flag = msg.flag1
        self.cx = msg.x # Communicates the x location
        self.cy = msg.y # Communicates the y location
        self.error_x = msg.error_x
        self.error_y = msg.error_y
        # track when you have a certain number of waypoints
        if len(self.pointx)>9:
            self.track_flag = True
        if self.phase == 2:
            self.track_flag = False
            self.phase = 1
        # When the robot is in the right position and we want to follow a block, execute tracking the block
        if (self.track_flag and -0.6 < self.waypoints[0].position.x and self.waypoints[0].position.x < 0.6):
            self.execute()
            self.default_pose_flag = False
        # If you don't want to track the block or if you aren't in the correct range, don't track the block but try to put it in a box
        else: 
            if not self.default_pose_flag: # If you aren't at the center position, stop tracking the conveyer belt and execute the position
                self.track_flag = False
                self.execute()
                self.default_pose_flag = True

    # Called by tracking_callback to execute different functions based on flags
    def execute(self):

        # If you are tracking the block....
        if self.track_flag:
            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

            # Initialize the waypoints list
            self.waypoints= []

            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            # wpose.position.x = -0.5215
            # wpose.position.y = 0.2014
            # wpose.position.z = 0.4102

            # If the number of points to execute is greater than 8, we have enough waypoints to do something!
            if len(self.pointx)>8:
                # If the number of waypoints is equal to 9, alter the speed
                if len(self.pointx)==9:
                    x_speed = np.mean(np.asarray(self.pointx[4:8]) - np.asarray(self.pointx[3:7]))
                    wpose.position.x += 2 * x_speed # scale the speed
                    wpose.position.z = 0.05 # scale the position
                else:
                    # If the number of points if 11, publish the position
                    if len(self.pointx)==11:
                        tracker.flag2 = 1
                        tracker.blockColor = 0
                        self.cxy_pub.publish(tracker)
                    # If less than 12 points, move a little faster
                    if len(self.pointx)<12:
                        x_speed = np.mean(np.asarray(self.pointx[4:8])-np.asarray(self.pointx[3:7]))
                        wpose.position.x += (x_speed-self.error_x*0.015/105)
                    else: # If we are greater than or equal to 12, you have a lot of waypoints you probably have a block
                        if tracker.flag2: # If you are holding a block
                            self.track_flag=False # Stop tracking
                        transition_pose = deepcopy(start_pose) # and move to the starting position
                        transition_pose.position.z = 0.4000 # Move the arm up

                        # Move to the transition pose (dropping the block in the bin)
                        self.waypoints.append(deepcopy(transition_pose))
                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.arm.execute(plan) # execute (moveit_commander, not this class's execute)

                        self.arm.set_max_acceleration_scaling_factor(.15)
                        self.arm.set_max_velocity_scaling_factor(.25)

                        # set the joint to go to the transition_pose (hovering above the box)
                        self.arm.set_joint_value_target(self.transition_pose)
                        self.arm.set_start_state_to_current_state()
                        plan = self.arm.plan()
                        self.arm.execute(plan[1])

                        # set the joint to go to the end_joint_states (dropping in the box)
                        self.arm.set_joint_value_target(self.end_joint_states)
                        self.arm.set_start_state_to_current_state()
                        plan = self.arm.plan()
                        self.arm.execute(plan[1])

                        # Tracks how many objects are in the box
                        # If the number of objects exceed 15, this will affect how low in the box the object is dropped
                        if -0.1+0.02*self.object_cnt<0.2:
                            self.object_cnt += 1

                        # Adjust the transition pose based on the number of objects in the box
                        self.waypoints = []
                        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                        transition_pose = deepcopy(start_pose) # get the position of the end effector
                        transition_pose.position.x -= 0.1 # go left a bit
                        transition_pose.position.z = -0.1 + self.object_cnt*0.025 # go up an amount dependent on the number of objects in the box
                        self.waypoints.append(deepcopy(transition_pose)) # drop the object in the box

                        # execute the motion plan and move up to the transition pose
                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.arm.execute(plan)

                        # After you are in the transition pose (the block has been dropped into the box)
                        self.phase = 2
                        tracker.flag2 = 0 #Set flag 2 to 0
                        tracker.blockColor = 0 # Publish a color of red... this is arbitrary but the blockColor should be initialized before publishing
                        self.cxy_pub.publish(tracker) # Let the vacuum grippers know we are in position to let go of the box

            # Set the next waypoint to the right 0.5 meters
            else:
                wpose.position.x -= self.error_x*0.05/105
                wpose.position.y += self.error_y*0.04/105
                wpose.position.z = 0.15
                #wpose.position.z = 0.4005

            if self.phase == 1:
                self.waypoints.append(deepcopy(wpose))


                self.pointx.append(wpose.position.x)
                self.pointy.append(wpose.position.y)

                # Set the internal state to the current state
                # self.arm.set_pose_target(wpose)

                self.arm.set_start_state_to_current_state()

                # Plan the Cartesian path connecting the waypoints

                """moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(
                        self, waypoints, eef_step, jump_threshold, avoid_collisios= True)

                   Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the
                   poses specified as waypoints. Configurations are computed for every eef_step meters;
                   The jump_threshold specifies the maximum distance in configuration space between consecutive points
                   in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed,
                   the actual RobotTrajectory.

                """
                plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)


                # plan = self.arm.plan()

                # If we have a complete plan, execute the trajectory
                if 1-fraction < 0.2:
                    rospy.loginfo("Path computed successfully. Moving the arm.")
                    num_pts = len(plan.joint_trajectory.points)
                    rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                    self.arm.execute(plan)
                    rospy.loginfo("Path execution complete.")
                else:
                    rospy.loginfo("Path planning failed")

        else:
            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

            # Initialize the waypoints list
            self.waypoints= []
            self.pointx = []
            self.pointy = []
            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            # Set the next waypoint to the right 0.5 meters

            wpose.position.x = 0.1052
            wpose.position.y = -0.4271
            wpose.position.z = 0.4005

            wpose.orientation.x = 0.4811
            wpose.orientation.y = 0.4994
            wpose.orientation.z = -0.5121
            wpose.orientation.w = 0.5069

            self.pointx.append(wpose.position.x)
            self.pointy.append(wpose.position.y)
            self.waypoints.append(deepcopy(wpose))
            # Set the internal state to the current state
            # self.arm.set_pose_target(wpose)

            self.arm.set_start_state_to_current_state()

            # Plan the Cartesian path connecting the waypoints

            """moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(
                    self, waypoints, eef_step, jump_threshold, avoid_collisios= True)

               Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the
               poses specified as waypoints. Configurations are computed for every eef_step meters;
               The jump_threshold specifies the maximum distance in configuration space between consecutive points
               in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed,
               the actual RobotTrajectory.

            """
            plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)


            # plan = self.arm.plan()

            # If we have a complete plan, execute the trajectory
            if 1-fraction < 0.2:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                num_pts = len(plan.joint_trajectory.points)
                rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                self.arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed")
        # print self.points




mp=ur5_mp()

rospy.spin()
