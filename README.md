# Traffic Sign Detection
DEMO VIDEO LINK: https://youtu.be/kVttsuUXtOw

## Front installation
1. PC OS: Ubantu 20.04
https://releases.ubuntu.com/focal/

2. Raspberry Pi OS: Ubantu 20.04
https://www.raspberrypi.com/software/

Please refer to setup file

```bash
sudo apt-get update
```
```bash
sudo apt-get upgrade
```
```bash
sudo apt-get install git
```

## Swap RAM
https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-20-04

## ROS & some installation and setup
https://wiki.ros.org/Installation/Ubuntu


## Setting github ssh key
```bash
ssh-keygen -t rsa -b 4096 -C <your_email>
```
```bash
ssh-keygen -t rsa -b 4096 -C tzuchichen.sc08@nycu.edu.tw
```

```bash
cat ~/.ssh/id_rsa.pub
```

Set the key on github webpage


## Clone the repo
```bash
git clone git@github.com:zhuchi76/Traffic-Sign-Detection.git
```
```bash
git submodule update --init
```

## On rpi
```bash
source install_rpi.sh
```

## On PC
```bash
source install_pc.sh
```

## Arduino
https://blog.csdn.net/weixin_51331359/article/details/122289768


## Compile the ROS system
```bash
cd ~/Traffic-Sign-Detection/catkin
```

```bash
catkin_make
```

## Download Rpi camera repo
https://index.ros.org/r/raspicam_node/github-UbiquityRobotics-raspicam_node/#noetic

```bash
sudo cp ~/Traffic-Sign-Detection/30-ubiquity.list /etc/ros/rosdep/sources.list.d/30-ubiquity.list
```
## Run (on rpi)
Modify environment_laptop.sh and environment_rpi.sh.
Add these files content to ~/.bashrc in PC and rpi.

### Terminal 1
```bash
source ~/Traffic-Sign-Detection/catkin_ws/devel/setup.bash
```

```bash
roslaunch mobile_robot arduino_comm.launch
```

### Terminal 2
```bash
source ~/Traffic-Sign-Detection/catkin_ws/devel/setup.bash
```

```bash
roslaunch raspicam_node camerav2_1280x960.launch
```

## Run (on pc)
```bash
source ~/Traffic-Sign-Detection/catkin_ws/devel/setup.bash
```

```bash
roslaunch mobile_robot traffic_sign_detector.launch
```


