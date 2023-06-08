# Drone Dash 2023
This is the official boilerplate repository for Drone Dash @ Summer of Innovation 2023, IIT Dharwad.

## Objective
The objective of Drone Dash is to design a obstacle avoidance algorithm for a drone. There is an onboard color and depth camera which can be used for this task. You may use any algorithm you like. The goal is to reach the end of the track in the shortest time possible.

## Development Environment
You will be developing the algorithm for use with the [PX4 Autopilot Stack](https://px4.io/) in [ROS](https://www.ros.org/). The PX4 Autopilot is a flight control software stack for drones. ROS is a robotics middleware that provides a set of tools and libraries for developing robot applications.

You will be using the [Gazebo](https://classic.gazebosim.org/) simulator to test your algorithm. Gazebo is a 3D robotics simulator that provides a physics engine, high-quality graphics, and convenient programmatic and graphical interfaces.

Instructions for setting up the development environment are provided in the [Installation Guide](./docs/Installation.md).

## Demo Script
We have provided a [demo script](./scripts/demo_node.py) to get you started. The script gets the drone to takeoff and then move forward and backward in a loop. You can launch a simulation with the demo script using the following command.
```bash
roslaunch dronedash demo.launch
```

To see the RGB and depth images from the camera in RVIZ, set the `rviz` parameter to `true`.
```bash
roslaunch dronedash demo.launch rviz:=true
```

Note: If the demo does not work, and you encounter the message `Failsafe mode activated` in the terminal see the [Troubleshooting](#failsafe-mode-activated) section.

## Resources
We have provided a list of recommended resources that you can use to learn more about the development environment and the PX4 Autopilot Stack.

### ROS
* [ROS Wiki](https://wiki.ros.org/)
* [ROS Tutorials](https://wiki.ros.org/ROS/Tutorials)
* [Getting Started With ROS - Articulated Robotics on Youtube](https://youtube.com/playlist?list=PLunhqkrRNRhYYCaSTVP-qJnyUPkTxJnBt)

### PX4 Autopilot
* [Flight Modes](https://docs.px4.io/v1.13/en/flight_modes/#multicopter)
* [User Guide](https://docs.px4.io/v1.13/en/ros/ros1.html)

## Troubleshooting

### Failsafe mode activated
It may happen that after running the demo for the first time that the drone does not takeoff and you encounter the message `Failsafe mode activated` in the terminal. To solve this paste the following command in the same terminal without stopping the running command.
```bash
param set COM_RCL_EXCEPT 4
```

If done correctly you should see the message `COM_RCL_EXCEPT: curr: 0 -> new: 4` in the terminal and the drone should takeoff.

[Source](https://discuss.px4.io/t/failsafe-mode-activating-constantly-after-the-vehicle-enters-the-offboard-mode/24460/3)