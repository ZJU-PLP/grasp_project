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
cp ../grasp_project/files_to_substitute/robotiq_85_gripper.transmission.xacro ../robotiq/robotiq_description/urdf

echo '###### Cloning the plotJuggler package ######'
git clone https://github.com/facontidavide/PlotJuggler ../PlotJuggler
sudo apt-get install ros-kinetic-plotjuggler 

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

# Required to URSIM
# JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
# export JAVA_HOME
# PATH=$PATH:$JAVA_HOME/bin
# export PATH

# Required for CUDA
# export PATH=/usr/local/cuda-10.0/bin${PATH:+:${PATH}}

# Required for ROS
# source /opt/ros/kinetic/setup.bash
# source ~/2_ROS/devel/setup.bash

echo "alias gazebo1='roslaunch grasp_project gazebo_ur5.launch'" >> ~/.bashrc
echo "alias gazebo2='rosrun grasp_project spawn_model.py'" >> ~/.bashrc
echo "alias gazebo3='rosrun grasp_project change_gazebo_properties.py'" >> ~/.bashrc
echo "alias gazebo4='roslaunch grasp_project tf_transforms.launch gazebo:=true'" >> ~/.bashrc
echo "alias gazebo5='rosrun grasp_project command_vel_gazebo.py --armarker --OriON --gazebo'" >> ~/.bashrc
echo "alias rviz1='roslaunch grasp_project rviz_ur5.launch'" >> ~/.bashrc
echo "alias rviz2='cd ~/Downloads/ursim-3.9.1.64192; sudo ./start-ursim.sh '" >> ~/.bashrc
echo "alias rviz3='roslaunch grasp_project ur5_ros_control.launch robot_ip:=127.0.1.1'" >> ~/.bashrc
echo "alias rviz4='rosrun grasp_project tf_node.py'" >> ~/.bashrc
echo "alias rviz5='rosrun grasp_project command_vel_rviz.py --armarker --OriON'" >> ~/.bashrc
echo "alias ufba_real_ros='roslaunch ur_modern_driver ur5_ros_control.launch robot_ip:=192.168.131.12'" >> ~/.bashrc
echo "alias track1='roslaunch kinect2_bridge kinect2_bridge.launch depth_method:=opengl reg_method:=cpu'" >> ~/.bashrc
echo "alias track2='roslaunch ar_track_alvar pr2_indiv_no_kinect_caio.launch'" >> ~/.bashrc
echo "alias track3='roslaunch grasp_project tf_transforms.launch'" >> ~/.bashrc
echo "alias ggcnn1='rosrun grasp_project command_GGCNN_ur5.py --OriON --gazebo'" >> ~/.bashrc
echo "alias ggcnn2='rosrun grasp_project run_ggcnn_ur5.py'" >> ~/.bashrc
echo "alias fix_performance='sudo cpupower frequency-set --governor performance'" >> ~/.bashrc
echo "alias lag_mouse='sudo service lightdm restart'" >> ~/.bashrc
echo "alias check_performance='cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor'" >> ~/.bashrc
echo "alias set_performance='echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor'" >> ~/.bashrc
echo "alias fix_usb1='sudo apt remove fwupd'" >> ~/.bashrc
echo "alias fix_usb2='sudo killall fwupd'" >> ~/.bashrc
echo "alias clean1='sudo du -sh /var/cache/apt'" >> ~/.bashrc
echo "alias clean2='sudo apt-get clean'" >> ~/.bashrc
echo "alias real1='roslaunch grasp_project ur5_ros_control.launch robot_ip:=192.168.131.13'"
echo "alias real2='roslaunch grasp_project rs_d435_camera.launch'"
echo "alias real3='rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0'"
echo "alias real4='rosrun grasp_project run_ggcnn_ur5.py --real' "
echo "alias real5='rosrun grasp_project command_GGCNN_ur5.py'"
echo "alias real6='roslaunch grasp_project rviz_ur5.launch'"
echo "# Gripper listener"
echo "alias real5='rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py'"