# %%%%%%%%%%%%%%%%%%%%%%%
# Install Nvidia Driver 
# %%%%%%%%%%%%%%%%%%%%%%%

#Go to additional drivers and select 410 

sudo add-apt-repository ppa:graphics-drivers/ppa 
sudo apt-get update 

# %%%%%%%%%%%%%%%%%%%%%%%
# Download CUDA Toolkit 
# %%%%%%%%%%%%%%%%%%%%%%%

# You have to make some choices in the terminal: 
# 	Install NVIDIA Accelerated Graphics Driver for Linux-x86_64 384.81? n 
# 	Install the CUDA 10.0 Toolkit? y 
# 	Do you want to install a symbolic link at /usr/local/cuda? y 
# 	Install the CUDA 10.0 Samples? y 
# Leave the rest as default. ENTER 

wget https://developer.nvidia.com/compute/cuda/10.0/Prod/local_installers/cuda_10.0.130_410.48_linux

sudo sh cuda_10.0.130_410.48_linux.run 

# add the following lines to bashrc
export PATH=/usr/local/cuda-10.0/bin${PATH:+:${PATH}}$
export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# Test it 
# Find ./deviceQuery in /usr/local/cuda/extra/demo_suit folder and run it 

# %%%%%%%%%%%%%%%%%%%%%%%
# Installing CuDNN 
# %%%%%%%%%%%%%%%%%%%%%%%

# Download the following packages: 
# cuDNN Runtime Library for Ubuntu16.04 (Deb) 
https://developer.nvidia.com/compute/machine-learning/cudnn/secure/v7.4.2/prod/10.0_20181213/Ubuntu16_04-x64/libcudnn7_7.4.2.24-1%2Bcuda10.0_amd64.deb 

# cuDNN Developer Library for Ubuntu16.04 (Deb) 	
https://developer.nvidia.com/compute/machine-learning/cudnn/secure/v7.4.2/prod/10.0_20181213/Ubuntu16_04-x64/libcudnn7-dev_7.4.2.24-1%2Bcuda10.0_amd64.deb 

# cuDNN Code Samples and User Guide for Ubuntu16.04 (Deb) 
https://developer.nvidia.com/compute/machine-learning/cudnn/secure/v7.4.2/prod/10.0_20181213/Ubuntu16_04-x64/libcudnn7-doc_7.4.2.24-1%2Bcuda10.0_amd64.deb 

sudo dpkg -i libcudnn7_7.4.2.24-1+cuda10.0_amd64.deb 
sudo dpkg -i libcudnn7-dev_7.4.2.24-1+cuda10.0_amd64.deb  
sudo dpkg -i libcudnn7-doc_7.4.2.24-1+cuda10.0_amd64.deb 