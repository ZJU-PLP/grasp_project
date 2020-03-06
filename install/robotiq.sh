# Install Modbus TCP:
rosdep install robotiq_modbus_tcp

# Install ros-kinetic-soeml
sudo apt-get install ros-kinetic-soem

sudo usermod -a -G dialout $USER

sudo apt-get install ros-kinetic-ros-canopen	

# Run the following commands in separated terminals
#sudo chmod 777 /dev/ttyUSB0
# rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
# rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
# rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py