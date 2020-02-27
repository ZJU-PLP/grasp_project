#!/bin/bash
####################################
#
# Install the necessary packages 
#
####################################

echo '###### Cloning the universal_robot package ######'
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot ../universal_robot

echo '###### Cloning the ur_modern_driver package ######'
git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver ../ur_modern_driver

echo '###### Cloning the moveit_python package ######'
git clone https://github.com/mikeferguson/moveit_python ../moveit_python

echo '###### Cloning the robotiq package ######'
git clone https://github.com/crigroup/robotiq ../robotiq
cp ../grasp_project/files_to_substitute/CMakeLists.txt ../robotiq/robotiq_description

echo '###### Installing ros-kinetic-moveit ######'
sudo apt-get install ros-kinetic-moveit

echo '###### Installing controllers ######'
sudo apt-get install ros-kinetic-gripper*controller

echo '###### Cloning the realsense-ros package ######'
git clone https://github.com/IntelRealSense/realsense-ros ../realsense-ros

echo '###### Cloning the realsense_gazebo_plugin package ######'
git clone https://github.com/pal-robotics/realsense_gazebo_plugin ../realsense_gazebo_plugin
cp ../grasp_project/files_to_substitute/gazebo_ros_realsense.cpp ../realsense_gazebo_plugin/src
cp ../grasp_project/files_to_substitute/RealSensePlugin.cpp ../realsense_gazebo_plugin/src

# Move into the catkin_ws folder
cd ../..
rosdep install --from-paths src --ignore-src -r -y --rosdistro kinetic
catkin build

cd src/realsense_gazebo_plugin
if [ -d ./build ] 
then
    echo "Build folder already exists" 
    cd build
    cmake ../
	make
else
    mkdir -p build
    cd build
    cmake ../
	make
fi

echo $PWD
echo '###### Cloning the cob_gazebo_plugins package ######'
git clone https://github.com/ipa320/cob_gazebo_plugins ../../cob_gazebo_plugins
cd ../../cob_gazebo_plugins/cob_gazebo_ros_control
if [ -d ./build ] 
then
    echo "Build folder already exists" 
    cd build
    cmake ../
	make
else
    mkdir -p build
    cd build
    cmake ../
    make
fi