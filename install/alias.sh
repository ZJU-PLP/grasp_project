# It was created for personal use

echo "# Commands to control the simulated UR5 in Gazebo" >> ~/.bashrc
echo "alias gazebo1='roslaunch grasp_project gazebo_ur5.launch'" >> ~/.bashrc
echo "alias gazebo2='rosrun grasp_project command_GGCNN_ur5.py --gazebo'" >> ~/.bashrc
echo "alias gazebo3='rosrun grasp_project run_ggcnn_ur5.py'" >> ~/.bashrc

echo "# Commands to open RVIZ" >> ~/.bashrc
echo "alias rviz1='roslaunch grasp_project rviz_ur5.launch'" >> ~/.bashrc

echo "# Commands to control the real UR5" >> ~/.bashrc
echo "alias real1='roslaunch grasp_project ur5_ros_control.launch robot_ip:=192.168.131.13'" >> ~/.bashrc
echo "alias real2='roslaunch grasp_project rs_d435_camera.launch'" >> ~/.bashrc
echo "alias real3='rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0'" >> ~/.bashrc
echo "alias real4='rosrun grasp_project run_ggcnn_ur5.py --real' " >> ~/.bashrc
echo "alias real5='rosrun grasp_project command_GGCNN_ur5.py'" >> ~/.bashrc
echo "alias real6='roslaunch grasp_project rviz_ur5.launch'" >> ~/.bashrc

echo "# Gripper listener" >> ~/.bashrc
echo "alias real7='rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py'" >> ~/.bashrc
echo "alias real8='rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py'" >> ~/.bashrc 