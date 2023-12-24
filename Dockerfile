# Start with an ARM-compatible base image
FROM arm32v7/ubuntu:focal

# Set environment variables to non-interactive
# This avoids user interaction hanging the build
ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO noetic

# Setup the ROS repository
RUN apt-get update && apt-get install -y gnupg2 lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get install -y \
    g++

# Install ROS Base and rosserial packages
RUN apt-get update && apt-get install -y \
    ros-noetic-ros-base \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python

# Install other useful tools and dependencies
RUN apt-get install -y git wget nano

# # Install Python packages for OpenCV, NumPy, Matplotlib, scikit-image, and imutils
# RUN apt-get install -y python3-pip
# RUN pip3 install numpy matplotlib scikit-image imutils
# RUN apt-get install -y python3-opencv

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-dev

# Install Python packages for OpenCV, NumPy, Matplotlib, scikit-image, and imutils
RUN apt-get install -y python3-pip
RUN pip3 install numpy
RUN pip3 install matplotlib
RUN pip3 install scikit-image
RUN pip3 install imutils
RUN apt-get install -y python3-opencv



# Set up the ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc

# Create a new user
RUN useradd -m rosuser && \
    echo "rosuser:rosuser" | chpasswd && \
    adduser rosuser sudo && \
    echo "rosuser ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER rosuser
WORKDIR /home/rosuser

# Source the ROS setup for every bash session
RUN echo "source /opt/ros/noetic/setup.bash" >> .bashrc

CMD ["bash"]
