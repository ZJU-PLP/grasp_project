sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u

sudo apt-get install librealsense2-dkms

sudo apt-get install librealsense2-utils

# Verify that the kernel is updated :
# should include realsense string
modinfo uvcvideo | grep "version:" 