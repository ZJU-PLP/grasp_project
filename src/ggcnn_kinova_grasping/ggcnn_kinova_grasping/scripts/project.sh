#!/bin/bash

echo "###############   LOADING ROBOT DESCRIPTION   ###########"
gnome-terminal -e 'roslaunch ur_gazebo ur5.launch'

sleep 3
echo "###############   STARTING GAZEBO PAUSED   ###############"
gnome-terminal -e 'roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:="true"'
