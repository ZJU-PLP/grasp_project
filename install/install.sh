#!/bin/bash
####################################
#
# Install the necessary packages 
#
####################################

echo '###### Installing some controllers ######'
# Install catkin tools
sudo apt-get install ros-kinetic-catkin python-catkin-tools

# Install some controllers
sudo apt-get install ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-effort-controllers
sudo apt-get install ros-kinetic-position-controllers

echo '###### Cloning the universal_robot package ######'
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot ../../universal_robot

echo '###### Cloning the ur_modern_driver package ######'
git clone -b kinetic-devel https://github.com/ros-industrial/ur_modern_driver ../../ur_modern_driver

echo '###### Cloning the moveit_python package ######'
git clone https://github.com/mikeferguson/moveit_python ../../moveit_python

echo '###### Cloning the robotiq Gazebo package ######'
git clone https://github.com/crigroup/robotiq ../../robotiq
cp ../files_to_substitute/CMakeLists.txt ../../robotiq/robotiq_description
cp ../files_to_substitute/robotiq_85_gripper.transmission.xacro ../../robotiq/robotiq_description/urdf
mv ../../robotiq ../../robotiq_gaz

echo '###### Cloning the robotiq package ######'
git clone https://github.com/ros-industrial/robotiq ../../robotiq

echo '###### Cloning the plotJuggler package ######'
git clone https://github.com/facontidavide/PlotJuggler ../../PlotJuggler
sudo apt-get install ros-kinetic-plotjuggler 

echo '###### Installing ros-kinetic-moveit ######'
sudo apt-get install ros-kinetic-moveit

echo '###### Installing controllers ######'
sudo apt-get install ros-kinetic-gripper*controller

echo '###### Cloning the realsense-ros package ######'
sudo apt-get install ros-kinetic-realsense2-camera
git clone https://github.com/IntelRealSense/realsense-ros --branch 2.2.11 ../../realsense-ros

echo '###### Cloning the realsense_gazebo_plugin package ######'
git clone https://github.com/pal-robotics/realsense_gazebo_plugin ../../realsense_gazebo_plugin
cp ../files_to_substitute/gazebo_ros_realsense.cpp ../../realsense_gazebo_plugin/src
cp ../files_to_substitute/RealSensePlugin.cpp ../../realsense_gazebo_plugin/src

# Move into the workspace folder
cd ../../..
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

