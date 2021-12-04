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

from os import posix_fallocate
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
        self.phase = 1 # Tracking the block --> being inside the box, becomes phase = 2 dropped the block --> to default position
        self.object_cnt = 0 # How many objects are in the bin
        self.track_flag = False # Tracking the block versus picked up the block
        self.default_pose_flag = True # Are we in the default pose (looking at the conveyer belt)
        self.cx = 400.0 # center of image (800x800)
        self.cy = 400.0 # center of image (800x800)
        self.points=[]
        self.state_change_time = rospy.Time.now()
        self.blockColor = 0 # Default block color is red

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

        # Specify poses for red
        # End Joint States specify the pose where we drop the object
        self.end_joint_states_red = deepcopy(self.default_joint_states) # inside the box
        self.end_joint_states_red[0] = 0.21
        # Transition poses specify the pose where we are hovering above the box but haven't dropped the object
        self.transition_pose_red = deepcopy(self.default_joint_states) # hovering above the box
        self.transition_pose_red[0] = 0.21
        self.transition_pose_red[4] = -1.95

        # Blue
        self.end_joint_states_blue = deepcopy(self.default_joint_states) # inside the box
        self.end_joint_states_blue[0] = -3.65

        self.transition_pose_blue = deepcopy(self.default_joint_states) # hovering above the box
        self.transition_pose_blue[0] = -3.65
        self.transition_pose_blue[4] = -1.95

        # Yellow
        self.end_joint_states_yellow = deepcopy(self.default_joint_states) # inside the box
        self.end_joint_states_yellow[0] = 1.5

        self.transition_pose_yellow = deepcopy(self.default_joint_states) # hovering above the box
        self.transition_pose_yellow[0] = 1.5
        self.transition_pose_yellow[4] = -1.95

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

        # When you have enough waypoints, 
        # stop looking for new blocks and stop trying to grab the block you are already holding
        if len(self.pointx)<9:
            self.track_flag = msg.flag1
            if len(self.pointx) == 7: # With 7 way points, you have the block in sight but haven't picked it up yet
                self.blockColor = msg.blockColor # Only look at the block color when you are ready to grab the block
        
        # If we are tracking, update the location and error
        if(self.track_flag):
            self.cx = msg.x # Communicates the x location
            self.cy = msg.y # Communicates the y location
            self.error_x = msg.error_x
            self.error_y = msg.error_y
        #print(msg.blockColor)
        #self.blockColor = msg.blockColor # Get the current block color
        # track when you have a 9 waypoints, you are definitely tracking. Make sure it knows you are tracking
        if len(self.pointx)>9:
            self.track_flag = True
        # Phase 2 means you have dropped the block in a box and want ot come back to starting position
        if self.phase == 2:
            self.track_flag = False
            self.phase = 1

        # We have detected a block (track_flag=1), the first waypoint is centered appropriately and ready to go!
        if (self.track_flag and -0.6 < self.waypoints[0].position.x and self.waypoints[0].position.x < 0.6):
            self.execute()
            self.default_pose_flag = False # We are no longer in the default pose
        # We are not tracking or we are not in range so return to the default position
        else: 
            if not self.default_pose_flag: # If you have seen something but are not centered (either holding the block on on the way back)
                self.track_flag = False # Disable tracking
                self.execute() # Continue executing
                self.default_pose_flag = True

    # Called by tracking_callback to execute different functions based on flags
    def execute(self):

        # If you are tracking the block....
        if self.track_flag:
            # Get the current pose
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

            # Initialize the waypoints list
            self.waypoints= []

            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            # wpose.position.x = -0.5215
            # wpose.position.y = 0.2014
            # wpose.position.z = 0.4102
            print(self.pointx)
            # We have tracked the block for more than 8 iterations
            if len(self.pointx)>8:
                # If the number of waypoints is equal to 9, alter the speed to hover above the block
                if len(self.pointx)==9:
                    x_speed = np.mean(np.asarray(self.pointx[4:8]) - np.asarray(self.pointx[3:7]))
                    wpose.position.x += 2 * x_speed # scale the speed
                    wpose.position.z = 0.05 # Move upwards
                else:
                    # We are ready to pick the block up! Send a message to the vaccuum grippers
                    if len(self.pointx)==11:
                        tracker.flag2 = 1 # flag 2 = in position to pick up
                        tracker.blockColor = self.blockColor
                        self.cxy_pub.publish(tracker)
                    # If less than 12 points, move a little faster
                    if len(self.pointx)<12: # 10 and 11 iterations
                        x_speed = np.mean(np.asarray(self.pointx[4:8])-np.asarray(self.pointx[3:7]))
                        wpose.position.x += (x_speed-self.error_x*0.015/105)
                    else: # You are holding a block (at least you should be holding a block)
                        if tracker.flag2: # 1 when vacuum gripper should have picked up a block
                            self.track_flag=False # Stop tracking since you are holding the block
                        transition_pose = deepcopy(start_pose) # and move to the starting position
                        transition_pose.position.z = 0.4000 # Move the arm up

                        # Lift up from the conveyer belt
                        self.waypoints.append(deepcopy(transition_pose))
                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        self.arm.execute(plan) # execute (moveit_commander, not this class's execute)

                        # Adjust the max_acceleration and velocity to be slower
                        self.arm.set_max_acceleration_scaling_factor(.15)
                        self.arm.set_max_velocity_scaling_factor(.25)

                        # Based on the color of block present
                        # Hover above the box (ee position will be off though)
                        yellowAlert = 0 # alerts if yellow needs to be dealt with
                        if(self.blockColor == 0):# 0 means red
                            myTransitionPose = self.transition_pose_red
                            myEndJointStates = self.end_joint_states_red
                        elif(self.blockColor == 1):# 1 means yellow
                            myTransitionPose = self.transition_pose_yellow
                            myEndJointStates = self.end_joint_states_yellow
                            yellowAlert = 1 # Alert the program that the block is yellow and additional steps need to be taken
                        else:
                            myTransitionPose = self.transition_pose_blue
                            myEndJointStates =self.end_joint_states_blue
                        self.arm.set_joint_value_target(myTransitionPose)
                        self.arm.set_start_state_to_current_state()
                        plan = self.arm.plan() # returns a motiion plan based on the joints arguement
                        self.arm.execute(plan[1])

                        # Move joint states to their final position (ee facing down)
                        self.arm.set_joint_value_target(myEndJointStates)
                        self.arm.set_start_state_to_current_state()
                        plan = self.arm.plan()
                        self.arm.execute(plan[1])

                        # Tracks how many objects are in the box
                        # If the number of objects exceed 15, this will affect how low in the box the object is dropped
                        if -0.1+0.02*self.object_cnt<0.2:
                            self.object_cnt += 1

                        # Add waypoint to drop into the box depending on how many things are in there
                        self.waypoints = []
                        start_pose = self.arm.get_current_pose(self.end_effector_link).pose
                        transition_pose = deepcopy(start_pose) # get the position of the end effector
                        rospy.loginfo("The block is " + str(self.blockColor))
                        if(self.blockColor == 1): # If yellow
                            transition_pose.position.y += 0.05 
                        if(self.blockColor == 0 or self.blockColor == 1): # If red or yellow
                            transition_pose.position.x += 0.1
                        if(self.blockColor == 2 ): # If blue
                            transition_pose.position.x -= 0.1
                        transition_pose.position.z = -0.1 + self.object_cnt*0.025 # go up an amount dependent on the number of objects in the box
                        self.waypoints.append(deepcopy(transition_pose))

                        self.arm.set_start_state_to_current_state()
                        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.02, 0.0, True)
                        # Execute the movement if the path was planned correctly
                        if 1-fraction < 0.2: 
                            rospy.loginfo("Path computed successfully. Moving the arm.")
                            num_pts = len(plan.joint_trajectory.points)
                            rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                            self.arm.execute(plan)
                            rospy.loginfo("Path execution complete.")
                        else:
                            rospy.loginfo("Path planning failed")                            
                            
                        # Indicate to the vaccuum grippers to release. We are now in phase 2 (returning to default pose)
                        self.phase = 2
                        tracker.flag2 = 0 # tell vaccuum grippers to release
                        tracker.blockColor = 0 # Publish a color of red... this is arbitrary but the blockColor should be initialized before publishing
                        self.cxy_pub.publish(tracker) # Let the vacuum grippers know we are in position to let go of the box
                        
                        # If yellow, take necessary actions to prevent a singularity
                        if(yellowAlert): 
                            # Let's hand hold the robot so we don't hit singularities yay!
                            # Move to the red position if yellow
                            self.arm.set_joint_value_target(self.end_joint_states_red)
                            self.arm.set_start_state_to_current_state()
                            plan = self.arm.plan()
                            self.arm.execute(plan[1])

            # Center the end effector over the center of the block
            # Robot has seen the block and is trying to center itself over the block
            # Keep following the block to the right by 0.5 meters so it says on screen
            else:
                wpose.position.x -= self.error_x*0.05/105
                wpose.position.y += self.error_y*0.04/105
                wpose.position.z = 0.15
                #wpose.position.z = 0.4005
            # In the first phase (we are following the block and need to pick it up), follow the block and keep executing
            if self.phase == 1:
                self.waypoints.append(deepcopy(wpose)) # Append the waypoint to the list

                # Set pointx and pointy to hold the position of the last appended pose
                self.pointx.append(wpose.position.x)
                self.pointy.append(wpose.position.y)

                # Set the internal state to the current state
                self.arm.set_start_state_to_current_state()

                # Plan the Cartesian path connecting the waypoints
                plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)

                # With 80% accuracy in trajectory, execute the path
                if 1-fraction < 0.2:
                    rospy.loginfo("Path computed successfully. Moving the arm.")
                    num_pts = len(plan.joint_trajectory.points)
                    rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                    self.arm.execute(plan)
                    rospy.loginfo("Path execution complete.")
                else:
                    rospy.loginfo("Path planning failed")

        else: # We are not tracking yet (phase 2)
            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

            # Initialize the waypoints list
            self.waypoints= []
            self.pointx = []
            self.pointy = []
            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            # Set the next waypoint to the default state so that we can look for something to track
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
            plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)

            # If the path can be followed with 80% accuracy, this was successful and execute the trajectory
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
