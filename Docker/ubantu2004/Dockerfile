FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO noetic

ENV SHELL=/bin/bash \
    USER=team2 \
    UID=1000

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
