FROM arm32v7/ubuntu:focal

# Environment variables
ENV DEBIAN_FRONTEND noninteractive
ENV ROS_DISTRO noetic

# Setup the ROS repository
RUN apt-get update && apt-get install -y gnupg2 lsb-release
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Base
RUN apt-get update && apt-get install -y ros-noetic-ros-base


RUN adduser --disabled-password \
    --gecos "Default user" \
    --uid ${UID} \
    ${USER}

ENV HOME=/home/${USER}

WORKDIR /home/${USER}

# Copy the .bashrc file from the root user to the new user
RUN cp /root/.bashrc ${HOME}/.bashrc

# Set passwords for the root and new user (consider more secure practices in production)
RUN echo "root:root" | chpasswd
RUN echo "${USER}:111111" | chpasswd

# Source ROS setup on each bash login
RUN echo 'source /opt/ros/noetic/setup.bash' >> ${HOME}/.bashrc

# Install rosserial
RUN apt-get update && apt-get install -y \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-rosserial-python && \
    rm -rf /var/lib/apt/lists/*

# Change ownership of the home directory
RUN chown -R $USER:$USER ${HOME}/

# Setup sudoers
RUN echo "root ALL=(ALL)  ALL" > /etc/sudoers
RUN echo "${USER} ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

USER ${USER}

WORKDIR ${HOME}
CMD ["bash"]
