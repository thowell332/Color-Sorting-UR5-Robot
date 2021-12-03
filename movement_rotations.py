# Duke University
# ECE 495, Instructor Oca
# BINS' LOCATIONS: red_bin -y 0.8 -x -0.5 -z 0.05" / blue_bin -y 1.2 -x 0.0 -z 0.05" / yellow_bin -y 0.8 -x 0.5 -z 0.05 - RETRIEVED FROM URDF

# import everything that I might need
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
#from ur5_notebook.msg import Tracker # for vision
#tracker = Tracker()
#tracker.blockColor = 0 # initialize - how to retrieve it??

#import what is needed to calculate distance, angles and other mathematical parts
from math import pi, tau, dist, fabs, cos, sqrt
# tau is being used to make things simpler, it represents a 360 degree angle
tau = 2.0 * pi
# this is how I would calculate distance
def dist(p, q):
    return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

# define a function to test if values in two different lists are within a tolerance of each other
def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        # check each
        for index in range(len(goal)):
            #consider a threshold
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class MoveItContext(object):

    def __init__(self):
        super(MoveItContext, self).__init__()

        # Initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
        # Instantiating the "RobotCommander" object, which provides information on the bot's
        # kinematic model and current joint states
        robot = moveit_commander.RobotCommander()
        # Instantiating the "PlanningSceneInterface" object, which provides a remote interface
        # so we can get, set and update the bot's understanding of the surroundings
        scene = moveit_commander.PlanningSceneInterface()

        # This interface can be used to plan and execute motions
        # "manipulator" is based on the robot I'm using, ur5e
        group_name = "manipulator" #that's the arm planning group
        move_group = moveit_commander.MoveGroupCommander(group_name)

        #can get name of reference frame for the robot
        planning_frame = move_group.get_planning_frame()
        end_effector_link = move_group.get_end_effector_link()
        #can also get a list of all the groups in the robot
        group_names = robot.get_group_names()

        # Allow replanning to increase the odds of a solution
        move_group.allow_replanning(True)

        # Allow some tolerance in position (meters) and orientation (radians)
        # this makes the robot work better
        move_group.set_goal_position_tolerance(0.01)
        move_group.set_goal_orientation_tolerance(0.1)

        # Create a `DisplayTrajectory`_ ROS publisher which is used to display
        # trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
     "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.end_effector_link = end_effector_link
        self.group_names = group_names

    def go_to_joint_state(self): # go back to standard pose after grabbing block
        # Specify default (idle) joint states
        group_name = "manipulator" #that's the arm planning group
        move_group = moveit_commander.MoveGroupCommander(group_name)
        self.default_joint_states = move_group.get_current_joint_values()
        self.default_joint_states[0] = -1.57691
        self.default_joint_states[1] = -1.71667
        self.default_joint_states[2] = 1.79266
        self.default_joint_states[3] = -1.67721
        self.default_joint_states[4] = -1.5705
        self.default_joint_states[5] = 0.0

        move_group.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        move_group.set_start_state_to_current_state()
        plan = move_group.plan()

        move_group.execute(plan[1])
    
    def go_to_joint_state1(self):
        group_name = "manipulator" #that's the arm planning group
        move_group = moveit_commander.MoveGroupCommander(group_name)

        self.default_joint_states = move_group.get_current_joint_values()
        self.default_joint_states[0] = 0
        #self.default_joint_states[1] = tau/5
        
        move_group.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        move_group.set_start_state_to_current_state()
        plan = move_group.plan()

        move_group.execute(plan[1])

    def go_to_joint_state2(self):
        group_name = "manipulator" #that's the arm planning group
        move_group = moveit_commander.MoveGroupCommander(group_name)

        self.default_joint_states = move_group.get_current_joint_values()
        self.default_joint_states[0] = -tau/8
        #self.default_joint_states[1] = tau/5

        move_group.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        move_group.set_start_state_to_current_state()
        plan = move_group.plan()

        move_group.execute(plan[1])

    def go_to_joint_state3(self):
        group_name = "manipulator" #that's the arm planning group
        move_group = moveit_commander.MoveGroupCommander(group_name)

        self.default_joint_states = move_group.get_current_joint_values()
        self.default_joint_states[0] = tau/2
        self.default_joint_states[1] = -1.8 # go down?
        
        move_group.set_joint_value_target(self.default_joint_states)

        # Set the internal state to the current state
        move_group.set_start_state_to_current_state()
        plan = move_group.plan()

        move_group.execute(plan[1])
    
def main():
    # Now, play everything needed!
    try:
        movecontext = MoveItContext()

        # If the box is detected to be red
        #if tracker.blockColor == 0:
            
        # If the box is detected to be blue
        #elif tracker.blockColor == 2:

        # If the box is detected to be yellow
        #elif tracker.blockColor == 1:
        movecontext.go_to_joint_state()
        movecontext.go_to_joint_state3()

        print("============ Complete!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

# call main to start everything
if __name__ == "__main__":
    main()
