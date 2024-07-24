# ogmen_coding_round

# Manan's Robot Project

This repository contains three ROS packages: `bot_description`, `bot_world`, and `bot_control`. These packages provide the following functionalities:

1. `bot_description`: Contains the URDF model of a robot, RViz launch files, Gazebo launch files, and teleoperation control.
2. `bot_world`: Defines a custom Gazebo world and launches it with the robot.
3. `bot_control`: Provides a script to read, filter, and publish laser scan data.



## bot_description Package

### URDF Description

The `bot_description` package contains the URDF model of the robot. The URDF file is located at `bot_description/urdf/bot.urdf`.

### RViz Visualization

To visualize the robot in RViz, use the `rviz.launch` file:

```sh
ros2 launch bot_description rviz.launch
```
### Gazebo Simulation without world

To spawn the robot in an empty Gazebo world, use the spawn.launch file:

```sh
ros2 launch bot_description spawn.launch
```

### Gazebo Simulation with world

To spawn the robot in Gazebo world, use the my_robot_gazebo.launch.xml file:

```sh
ros2 launch bot_world my_robot_gazebo.launch.xml
```

###Laser Scan Reading and Filtering

The bot_control package contains a script, reading_laser.py, which reads laser scan data, filters it to a 0 to 120 degrees field of view, and publishes the filtered data.

```sh
ros2 run bot_control reading_laser
```
