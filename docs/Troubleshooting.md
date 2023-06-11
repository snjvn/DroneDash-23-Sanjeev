# Troubleshooting
Steps to troubleshoot some frequently encountered errors.

- [RLException: [demo.launch] is neither a launch file in package [dronedash] nor is [dronedash] a launch file name](#rlexception-demo-launch-is-neither-a-launch-file-in-package-dronedash-nor-is-dronedash-a-launch-file-name)
- [Resource not found: px4](#resource-not-found-px4)
- [Failsafe mode activated](#failsafe-mode-activated)

## RLException: [demo.launch] is neither a launch file in package [dronedash] nor is [dronedash] a launch file name
To resolve this error, make sure that you have [sourced the workspace](./ROS.md#sourcing-the-workspace) before running the launching the demo.

## Resource not found: px4
To resolve this error, make sure that you have [added PX4 to your path](./PX4.md#add-px4-to-your-path) before running the demo.

## Failsafe mode activated
It may happen that after running the demo for the first time that the drone does not takeoff and you encounter the message `Failsafe mode activated` in the terminal. To solve this paste the following command in the same terminal without stopping the running command.
```bash
param set COM_RCL_EXCEPT 4
```

If done correctly you should see the message `COM_RCL_EXCEPT: curr: 0 -> new: 4` in the terminal and the drone should takeoff.

[Source](https://discuss.px4.io/t/failsafe-mode-activating-constantly-after-the-vehicle-enters-the-offboard-mode/24460/3)