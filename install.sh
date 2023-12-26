#!/bin/bash

sudo apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3 \
    python3-dev \
    python3-pip \
    g++

# Install packages from requirements.txt
echo "Installing Python packages from requirements.txt..."
sudo pip3 install -r requirements.txt

# Install ROS Base and rosserial packages
sudo apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python

# Install other useful tools and dependencies
sudo apt-get install -y git wget nano

echo "Installation complete."