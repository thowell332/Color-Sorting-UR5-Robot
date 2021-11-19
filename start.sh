#!/usr/bin/bash

xfce4-terminal -H -x bash -c 'source ~/catkin_ws/devel/setup.bash ; roslaunch ur_gazebo ur5.launch limited:=true'
xfce4-terminal -H -x bash -c 'source ~/catkin_ws/devel/setup.bash ; roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true'
xfce4-terminal -H -x bash -c 'source ~/catkin_ws/devel/setup.bash ; roslaunch ur5_moveit_config moveit_rviz.launch config:=true'