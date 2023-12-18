# #!/usr/bin/env bash

ARGS=("$@")

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<<"$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Get the current version of docker-ce
# Strip leading stuff before the version number so it can be compared
DOCKER_VER=$(dpkg-query -f='${Version}' --show docker-ce | sed 's/[0-9]://')
DOCKER_OPTS=""

# X11 authentication file
XAUTH=/tmp/.docker.xauth

# Prevent executing "docker run" when xauth failed.
if [ ! -f $XAUTH ]; then
    echo "[$XAUTH] was not properly created. Exiting..."
    exit 1
fi

# Run the Docker container
# Replace '/path/to/your/directory' with the path of the directory you want to mount
docker run $DOCKER_OPTS -it \
    -v /home/$USER/Traffic-Sign-Detection:/home/team2/Traffic-Sign-Detection \
    -w "/home/team2/Traffic-Sign-Detection" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -e DISPLAY \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev:/dev" \
    -v "/var/run/docker.sock:/var/run/docker.sock" \
    --name traffic-sign-detection \
    --user "root:root" \
    --network host \
    --rm \
    --privileged \
    --security-opt seccomp=unconfined \
    zhuchi76/traffic-sign-detection:latest \
    bash

