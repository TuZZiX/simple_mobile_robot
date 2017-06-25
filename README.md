# Simple Mobile Robot

A ROS package for a simple two wheel drive robot.

![Build Badge](https://img.shields.io/badge/build-passing-brightgreen.svg)

Tested on ROS Indigo and ROS Kinetic.

Built as a task for an interview. But I personally want to some more cool stuffs and see how far this small robot can go.

## Dependencies

Packages that are not shipped with ros-desktop-full installation are listed here:

**[catkin_simple](https://github.com/catkin/catkin_simple)**: only available as source code, need to clone into your workspace

**[amcl](http://wiki.ros.org/amcl)**: binary installation package available on both Indigo and Kinetic

**[map_server](http://wiki.ros.org/map_server)**: binary installation package available on both Indigo and Kinetic

**[gazebo_ros_control](http://gazebosim.org/tutorials?tut=ros_installing)**: required to simulate lidar sensor and diff wheel drive controller

**[twist_mux](http://wiki.ros.org/twist_mux)**: binary installation package available on both Indigo and Kinetic

**[interactive_marker_twist_server](http://wiki.ros.org/interactive_marker_twist_server)**: binary installation package available on both Indigo and Kinetic

**[robot_state_publisher](http://wiki.ros.org/robot_state_publisher)**: binary installation package available on both Indigo and Kinetic

## Usage

### Empty world launch file

Videos: https://youtu.be/vKvUAmQZhHI

### Robot in pen launch file

Videos: https://youtu.be/B8IkBJNDmC8

### Keyboard teleop

### Using rqt or other teleop source

### Square move

Video: https://youtu.be/boI3VW1YflU

### Circle move

Video: https://youtu.be/zx1Ngz1CBWU

### LIDAR safety stop

Video: https://youtu.be/-PnMeZPLats

### Delay launcher

## Experimental

### Trajectory Builder

### Path server + Absolute path move

### Steering Correction

## About the model

This robot model have a passive wheel to support the robot. However, it can not be simulated perfectly in Gazebo. Unlike the real world, when there is an over 90 deg rotation on that wheel, it will fight against the other wheels and cause drifting on robot's orientation.

Modified from a robot build by our lab, the original model can be found [here](https://github.com/wsnewman/learning_ros/tree/master/Part_2/mobot_urdf)

### Some hacking

#### Back to 4 wheels

#### Alternative actuator and controller

transmission interface and diff_drive_controller

## TODO List

- Fix and improve path server and use path server to complete square and circle move

- Add option of using AMCL odom instead of perfect odom from Gazebo

- Add full navigation stack: costmap, planner, move_base, etc.
