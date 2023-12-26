# Traffic Sign Detection

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

## ssh
```bash
sudo apt-get update
```
```bash
sudo apt-get install openssh-server openssh-client
```
```bash
sudo service ssh start
```
```bash
systemctl enable ssh.socket
```
```bash
sudo dpkg-reconfigure openssh-server
```
```bash
sudo service ssh restart
```
## Arduino
```bash
sudo apt-get install arduino
```
```bash
ls -l /dev/ttyACM*
```
```bash
sudo usermod -a -G dialout team2
```

## Setting github ssh key
```bash
ssh-keygen -t rsa -b 4096 -C tzuchichen.sc08@nycu.edu.tw
```

```bash
cat ~/.ssh/id_rsa.pub
```


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
