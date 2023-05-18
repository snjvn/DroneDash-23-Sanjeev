# MAVROS Installation Guide
Installation Instructions for MAVROS derived from the [Official Installation Guide](https://github.com/mavlink/mavros/tree/master/mavros#installation).

## Prerequisites
You must have ROS Noetic installed. See [ROS Noetic Installation Guide](ROS.md).

## Installation
Install MAVROS from the ROS binary packages.
```bash
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
```

Then install GeographicLib datasets by running the `install_geographiclib_datasets.sh` script.
```bash
wget -O /tmp/install_geographiclib_datasets.sh https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash /tmp/install_geographiclib_datasets.sh
```
