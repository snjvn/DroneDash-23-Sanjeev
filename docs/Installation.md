# Installation Guide
This guide will walk you through setting up the development environment for Drone Dash.

## Ubuntu 20.04
The development environment is based on Ubuntu 20.04. Ubuntu 20.04 was installed as part of the Linux Installation Drive recently conducted by the Techical Council. If you could not attend the drive, you can follow the provided instructions [here](https://docs.google.com/presentation/d/1V-vmox13ZVTD80zHWISHAr5jIA98rXx4nFpERsKTPJI/edit?usp=sharing).

It is recommended that you have atleast 25 GB of free space in your Ubuntu partition. If you have separate `/` and `/home` partitions, make sure that `/` has atleast 17 GB of free space and `/home` has atleast 8 GB of free space.

Before continuing, make sure that Ubuntu is up to date.
```bash
sudo apt update
sudo apt upgrade -y
python3 -m pip install --upgrade pip
```

Make sure that you have the following packages installed.
```bash
sudo apt update
sudo apt install -y git wget curl python-is-python3
```

## ROS Noetic & Gazebo
Install ROS Noetic using the instructions provided [here](./ROS.md).

## MAVROS
Install MAVROS using the instructions provided [here](./MAVROS.md).

## PX4
Install PX4 using the instructions provided [here](./PX4.md).

## Drone Dash
Clone the Drone Dash repository.
```bash
cd ~/catkin_ws/src
git clone https://github.com/IITDh-Robotics/DroneDash-23.git
```

Add the gazebo models to the path. You can do this by running the following command,
```bash
echo "GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/DroneDash-23/models" >> ~/.bashrc
```

Build the workspace.
```bash
cd ~/catkin_ws
catkin build
```