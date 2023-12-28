#!/bin/bash

sudo apt-get update

sudo apt-get install -y build-essential cmake python3 python3-dev python3-pip python3-empy g++


# Install packages from requirements.txt
echo "Installing Python packages from requirements.txt..."
# sudo pip3 install -r requirements.txt

# sudo apt-get update && apt-get install -y --allow-unauthenticated \
#     python3-opencv==4.6.0
pip install opencv-python scikit-image imutils matplotlib

# ROS
pip install empy catkin_pkg

# Install ROS Base and rosserial packages
sudo apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python

# Install ssh
sudo apt-get update && apt-get install -y \
    openssh-server \
    openssh-client 

sudo service ssh start
systemctl enable ssh.socket
sudo dpkg-reconfigure openssh-server
sudo service ssh restart

## Install Arduino
# sudo apt-get install -y arduino
# ls -l /dev/ttyACM*
# sudo usermod -a -G dialout team2

# Install other useful tools and dependencies
sudo apt-get install -y git wget nano

echo "Installation complete."