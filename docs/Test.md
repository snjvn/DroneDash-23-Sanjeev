```
roslaunch px4 px4.launch
```
```
roslaunch gazebo_ros empty_world.launch world_name:=/home/om/PX4-Autopilot/Tools/sitl_gazebo/worlds/empty.world
```
```
rosrun gazebo_ros spawn_model -sdf -file /home/om/PX4-Autopilot/Tools/sitl_gazebo/models/iris/iris.sdf -model iris -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0
```