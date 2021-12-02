# Duke University
# ECE 495, Instructor Oca
# BINS' LOCATIONS: red_bin -y 0.8 -x -0.5 -z 0.05" / blue_bin -y 1.2 -x 0.0 -z 0.05" / yellow_bin -y 0.8 -x 0.5 -z 0.05

# import everything that I might need
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from ur5_notebook.msg import Tracker # for vision
tracker.blockColor = 0
tracker = Tracker()
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

        #can get name of reference frame for the robot, can also comment it out as
        # I'm not using it
        planning_frame = move_group.get_planning_frame()
        end_effector_link = move_group.get_end_effector_link()
        #can also get a list of all the groups in the robot
        group_names = robot.get_group_names()

        # Allow replanning to increase the odds of a solution
        move_group.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
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

    def go_to_joint_state(self):
        move_group = self.move_group

        ## tau = 2*pi
        # We get the joint values from the group
        # This would be useful if I wanted to move the robot to a slighly better configuration
        joint_goal = move_group.get_current_joint_values()

        # Specify default (idle) joint states
        self.default_joint_states = self.move_group.get_current_joint_values()
        self.default_joint_states[0] = -1.57691
        self.default_joint_states[1] = -1.71667
        self.default_joint_states[2] = 1.79266
        self.default_joint_states[3] = -1.67721
        self.default_joint_states[4] = -1.5705
        self.default_joint_states[5] = 0.0

        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self,xs,ys,zs):
        # Here I'm planning a motion for this group
        # The idea is to define a desired pose for the end effector

        move_group = self.move_group

        # The values are being specified as parameters since I changed them a lot while
        # testing the robot
        # I kept orientation the same, so to make things easier it is defined as 1.0 here
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = xs
        pose_goal.position.y = ys
        pose_goal.position.z = zs

        move_group.set_pose_target(pose_goal)

        ## Call the planner to compute the plan and execute it
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # Clear pose targets after planning poses
        move_group.clear_pose_targets()
        # Update
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path1(self, scale=1):
        # red bin
        # BINS' LOCATIONS: red_bin -y 0.8 -x -0.5 -z 0.05" / blue_bin -y 1.2 -x 0.0 -z 0.05" / yellow_bin -y 0.8 -x 0.5 -z 0.05
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        wpose = move_group.get_current_pose().pose
        wpose.position.x -= scale * 0.5  # Move sideways (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -= scale * 0.1  # Then move down to first box
        waypoints.append(copy.deepcopy(wpose))

        # Cartesian path is interpolated at a resolution of 1 cm and the jump threshold is disabled
        # So it is set to 0.0
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )  # jump_threshold

        # For now, I'm just planning
        return plan, fraction

    def plan_cartesian_path2(self, scale=1):
        # blue bin
        # BINS' LOCATIONS: red_bin -y 0.8 -x -0.5 -z 0.05" / blue_bin -y 1.2 -x 0.0 -z 0.05" / yellow_bin -y 0.8 -x 0.5 -z 0.05
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        # rotate
        joint_goal[0] = tau / 2 #base joint
        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # Then move down to second box
        waypoints.append(copy.deepcopy(wpose))

        # Cartesian path is interpolated at a resolution of 1 cm and the jump threshold is disabled
        # So it is set to 0.0
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )  # jump_threshold

        # For now, I'm just planning
        return plan, fraction

    def plan_cartesian_path3(self, scale=1):

        # yellow bin
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        # BINS' LOCATIONS: yellow_bin -y 0.8 -x 0.5 -z 0.05
        wpose = move_group.get_current_pose().pose
        wpose.position.x += scale * 0.5  # Move sideways (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z -= scale * 0.1  # Then move down to first box
        waypoints.append(copy.deepcopy(wpose))

        # Cartesian path is interpolated at a resolution of 1 cm and the jump threshold is disabled
        # So it is set to 0.0
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0
        )  # jump_threshold

        # For now, I'm just planning
        return plan, fraction

    def display_trajectory(self, plan):
        # Now, display the trajectory

        # Copying class variables to local variables
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        # Populate trajectory_start with the current robot state
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        # And now execute it!!
        move_group = self.move_group
        # To execute a plan that has been previously computed
        move_group.execute(plan, wait=True)

def main():
    # Now, play everything needed!
    try:
        movecontext = MoveItContext()

        if tracker.blockColor == 0:
            # Execute movement using joint state goal
            movecontext.go_to_joint_state()
            # Go to pose goal
            movecontext.go_to_pose_goal(0.2, 0.4, 0.2)
            # Plan and display a Cartesian path
            cartesian_plan, fraction = movecontext.plan_cartesian_path1()
            # Display a saved trajectory, replaying the Cartesian path
            movecontext.display_trajectory(cartesian_plan)
            # Execute saved path
            movecontext.execute_plan(cartesian_plan)

        elif tracker.blockColor == 2:
            movecontext.go_to_joint_state()
            # Go to pose goal
            movecontext.go_to_pose_goal(0.2, 0.4, 0.2)
            # Plan and display a Cartesian path
            cartesian_plan, fraction = movecontext.plan_cartesian_path2()
            # Display a saved trajectory, replaying the Cartesian path
            movecontext.display_trajectory(cartesian_plan)
            # Execute saved path
            movecontext.execute_plan(cartesian_plan)

        elif tracker.blockColor == 1:
            movecontext.go_to_joint_state()
            # Go to pose goal
            movecontext.go_to_pose_goal(0.2, 0.4, 0.2)
            # Plan and display a Cartesian path
            cartesian_plan, fraction = movecontext.plan_cartesian_path3()
            # Display a saved trajectory, replaying the Cartesian path
            movecontext.display_trajectory(cartesian_plan)
            # Execute saved path
            movecontext.execute_plan(cartesian_plan)

        print("============ Complete!")

    # If I were to get inputs from user, I'm keeping it as I might modify the code later
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

# call main to start everything
if __name__ == "__main__":
    main()
