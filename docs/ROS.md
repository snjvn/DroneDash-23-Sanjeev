# ROS Noetic Installation Guide
Installation Instructions for ROS Noetic derived from the [Official Installation Guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

## Prerequisites
You be running Ubuntu 20.04.

## Setup your sources.list
Setup your computer to accept software from packages.ros.org.
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

## Set up your keys
```bash
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

## Installation
Update the package index and then install the `ros-noetic-dektop-full` package, which installs all ROS packages plus Gazebo.
```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full
```

## Environment Setup
You must source this script in every terminal you use ROS in.
```bash
source /opt/ros/noetic/setup.bash
```

It can be convenient to automatically source this script every time a new shell is launched. The following command will do that for you.
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Setup Workspace
Install build tools.
```bash
sudo apt update
sudo apt install -y python3-catkin-tools
```

Create a catkin workspace and build it.
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
```

## Sourcing the Workspace
Source the workspace.
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```