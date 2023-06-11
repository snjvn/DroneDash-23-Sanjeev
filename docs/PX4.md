# PX4 Installation Guide
Installation Instructions for PX4 derived from the [Official Installation Guide](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#ros-gazebo-classic).

## Prerequisites
You be running Ubuntu 20.04.

## Installation

### Download PX4 Source Code
```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout tags/v1.13.3
git submodule update --init --recursive
```

### Run the install script
```bash
bash ./Tools/setup/ubuntu.sh  --no-sim-tools --no-nuttx
```

### Install PX4 Dependencies
```bash
sudo apt update
sudo apt install -y protobuf-compiler libeigen3-dev libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
```

### Build PX4
```bash
DONT_RUN=1 make px4_sitl gazebo
```

### Add PX4 to your path
Update your `~/.bashrc` to add PX4 to your path. You can do this by running the following commands.
```bash
echo "source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo" >> ~/.bashrc
echo "export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins" >> ~/.bashrc
source ~/.bashrc
```