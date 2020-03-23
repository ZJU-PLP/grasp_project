# This file was created because the realsense-ros release version 2.2.13 was not working with the neal librealsense
# ros version v2.33.1. So I had to use the realsense-ros release version 2.2.11 and the librealsense version v2.31.0

# These commands will install the librealsense version v.2.31.0

sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade 
sudo apt-get install --install-recommends linux-generic-lts-xenial xserver-xorg-core-lts-xenial xserver-xorg-lts-xenial xserver-xorg-video-all-lts-xenial xserver-xorg-input-all-lts-xenial libwayland-egl1-mesa-lts-xenial 
# Go into the home folder and clone
git clone https://github.com/IntelRealSense/librealsense --branch v2.31.0
sudo apt-get install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

# Go into the librealsense root folder and run
./scripts/setup_udev_rules.sh

# if your kernel is 4.15 or below, run:
./scripts/patch-realsense-ubuntu-lts.sh

# if your kernel is 4.16 or above, run:
./scripts/patch-ubuntu-kernel-4.16.sh

echo 'hid_sensor_custom' | sudo tee -a /etc/modules

# move into the librealsense root folder and run:
mkdir build && cd build
cmake ../ -DBUILD_EXAMPLES=true -DBUILD_GRAPHICAL_EXAMPLES=false
sudo make uninstall && make clean && make && sudo make install