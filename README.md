# Traffic Sign Detection

## Front installation
1. PC OS: Ubantu 20.04
https://releases.ubuntu.com/focal/

2. Raspberry Pi OS: Ubantu 20.04
https://www.raspberrypi.com/software/

```bash
sudo apt-get update
```
```bash
sudo apt-get upgrade
```

## Swap RAM
https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-20-04

## ROS & some installation and setup
https://wiki.ros.org/Installation/Ubuntu

```bash
source install.sh
```

## Setting github ssh key
```bash
ssh-keygen -t rsa -b 4096 -C <your_email>
```

```bash
cat ~/.ssh/id_rsa.pub
```

Set the key on github webpage


## Clone the repo
```bash
git clone git@github.com:zhuchi76/Traffic-Sign-Detection.git
```

## Compile the ROS system
```bash
cd Traffic-Sign-Detection/catkin
```

```bash
catkin_make
```

## Download Rpi camera repo
```bash
git@github.com:UbiquityRobotics/raspicam_node.git
```