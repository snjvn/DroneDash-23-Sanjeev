# Installation Guide
This guide will walk you through setting up the development environment for Drone Dash.

## Ubuntu 20.04
The development environment is based on Ubuntu 20.04. Ubuntu 20.04 was installed as part of the Linux Installation Drive recently conducted by the Techical Council. If you could not attend the drive, you can follow the provided instructions [here](https://docs.google.com/presentation/d/1V-vmox13ZVTD80zHWISHAr5jIA98rXx4nFpERsKTPJI/edit?usp=sharing).

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