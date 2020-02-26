------------

<a id="top"></a>
### Contents
1. [Description](#1.0)
2. [Required packages - Kinetic Version](#2.0)
3. [Run GGCNN in Gazebo and RVIZ](#3.0)
4. [Sending commands through the action server](#4.0)
5. [Connecting with real UR5](#5.0)
6. [Meetings minutes](#6.0)

------------

<a name="1.0"></a>
### 1.0 Description

Repository created to store my research about robot grasping. This repository is kept for backup only. It is not intended to be fully organized (in future versions, for sure)

The method used here is based on [GGCNN](https://github.com/dougsm/ggcnn_kinova_grasping) created by Doug Morrison.

> **_NOTE:_**  This package should be placed into your src folder

<a name="2.0"></a>
### 2.0 Required packages - Kinetic Version

- [Realsense Gazebo Plugin](https://github.com/pal-robotics/realsense_gazebo_plugin)
- [Realsense-ros](https://github.com/IntelRealSense/realsense-ros)
- [Moveit Kinetic](https://moveit.ros.org/install/)
- [Moveit Python](https://github.com/mikeferguson/moveit_python)
- [Robotiq Gripper](https://github.com/crigroup/robotiq)
- [Universal Robot](https://github.com/ros-industrial/universal_robot)
- [ur_modern_driver](https://github.com/ros-industrial/ur_modern_driver)


In order to use the Realsense Gazebo Plugin, create a build folder inside the plugin package and run the following codes:
`cmake ../` and then `make`

Install ros-control dependencies
```bash
sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-msgs ros-kinetic-gazebo-plugins ros-kinetic-gazebo-ros-control
```

Install any dependencies you might have missed by using this command in catkin_ws folder
```bash
rosdep install --from-paths src --ignore-src -r -y
```

#### System and further required packages:
Please check the correct version of the cuda based on the Nvidia driver version (https://docs.nvidia.com/deploy/cuda-compatibility/index.html)
Please check the correct version of Tensorflow based on the cuda and CuDNN version (https://www.tensorflow.org/install/source#tested_build_configurations)

Packages used:
- Ubuntu - 16.04
- Nvidia - 410.78
- Cuda - 10.0.130
- tensorflow-estimator - 1.14
- tensorflow-gpu - 1.14.0 
- tensorflow-tensorboard - 0.4.0
- Keras - 2.1.5 
- Keras-Applications - 1.0.8 
- Keras-Preprocessing - 1.1.0 
- CuDNN - 7.4.2 

<a name="3.0"></a>
### 3.0 Run GGCNN in Gazebo and RVIZ

Launch Gazebo first:
```bash
roslaunch grasp_project gazebo_ur5.launch
```

Spawn a box into the Gazebo:
```bash
rosrun grasp_project spawn_model.py
```

Launch RVIZ if you want to see the frame (object_detected) corresponding to the object detected by GGCNN and the point cloud.
In order to see the point cloud, please add pointcloud2 into the RVIZ and select the correct topic:
```bash
roslaunch grasp_project rviz_ur5.launch
```

Run the GGCNN. This node will publish a frame corresponding to the object detected by the GGCNN.
```bash
rosrun grasp_project run_ggcnn_ur5.py
```

Running this node will move the robot to the position published by the run_ggcnn_ur5.py node.
```bash
rosrun grasp_project command_GGCNN_ur5.py --OriON --gazebo
```

You might want to see the grasp or any other image. In order to do that, you can use the rqt_image_view.
```bash
rosrun rqt_image_view
```

<a name="4.0"></a>
### 4.0 Sending commands through the action server

If you want to test the position controller sending commands directly to the /'controller_command'/command topic use
the following:

```bash
rostopic pub -1 /pos_based_pos_traj_controller/command trajectory_msgs/JointTrajectory "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
joint_names: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
points:
  - positions: [1.57, 0, 0, 0, 0, 0]
    time_from_start: {secs: 1, nsecs: 0}"
```

<a name="5.0"></a>
### 5.0 Connecting with real UR5

Firstly check the machine IP. The IP configured on the robot must have the last digit different.

```bash
ifconfig
```

Disable firewall

```bash
sudo ufw disable
```

Set up a static IP on UR5 according to the following figure

![config](https://user-images.githubusercontent.com/28100951/71323978-2ca7d380-24b8-11ea-954c-940b009cfd93.jpg)

Set up a connection on Ubuntu according to the following figure

![config_ethernet2](https://user-images.githubusercontent.com/28100951/71323962-fe29f880-24b7-11ea-86dc-756729932de4.jpg)

Use the following command in order to connect with real UR5.
If you are using velocity control, do not use bring_up. Use ur5_ros_control instead.

```
roslaunch ur_modern_driver ur5_ros_control.launch robot_ip:=192.168.131.12
```

<a name="6.0"></a>
### 6.0 Meetings minutes
#### Meeting - 25/11/2019
Topics covered:
1. Preferably use devices already in the lab, such as UR5, Intel Realsense and Gripper 2-Fingers Robotiq
2. Check how to use neural networks to predict the position of objects. Thus, the proposed method would be robust against camera limitations regarding the proximity of the object, that is, even if there is no depth information, the neural network would use past data to predict where the object is at the given moment.
3. Search for grasping applications.
4. Translate the thesis into English
