# Ubuntu 16.04 | Cuda 10.0 | Cudnn 7.4.2 | ROS kinetic
sudo apt install python-pip
pip install --upgrade pip

pip install keras==2.1.5
pip install Keras-Applications==1.0.8 
pip install Keras-Preprocessing==1.1.0 
pip install matplotlib==2.2.4 
pip install scikit-image==0.14.5 
pip install tensorboard==1.14.0 
pip install tensorflow-estimator==1.14.0 
pip install tensorflow-gpu==1.14.0 
pip install tensorflow-tensorboard==0.4.0 
pip install pathlib

# Install catkin tools
sudo apt-get install ros-kinetic-catkin python-catkin-tools

# Install some controllers
sudo apt-get install ros-kinetic-joint-state-controller
sudo apt-get install ros-kinetic-effort-controllers
sudo apt-get install ros-kinetic-position-controllers

# Install Terminator
sudo apt-get install terminator

# Install Sublime
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
sudo apt-get install apt-transport-https
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list