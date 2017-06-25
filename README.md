# Simple Mobile Robot

A ROS package for a simple two wheel drive robot.

![Build Badge](https://img.shields.io/badge/build-passing-brightgreen.svg)

Tested on ROS Indigo and ROS Kinetic.

Built as a task for an interview. But I personally want to some more cool stuffs and see how far this small robot can go.

## Dependencies

Packages that are not shipped with ros-desktop-full installation are listed here (based on Ubuntu 14.04 or 16.04):

**[catkin_simple](https://github.com/catkin/catkin_simple)**: only available as source code, need to clone into your workspace

**[amcl](http://wiki.ros.org/amcl)**: binary installation package available on both Indigo and Kinetic

**[map_server](http://wiki.ros.org/map_server)**: binary installation package available on both Indigo and Kinetic

**[gazebo_ros_control](http://gazebosim.org/tutorials?tut=ros_installing)**: required to simulate lidar sensor and diff wheel drive controller

**[twist_mux](http://wiki.ros.org/twist_mux)**: binary installation package available on both Indigo and Kinetic

**[interactive_marker_twist_server](http://wiki.ros.org/interactive_marker_twist_server)**: binary installation package available on both Indigo and Kinetic

**[robot_state_publisher](http://wiki.ros.org/robot_state_publisher)**: binary installation package available on both Indigo and Kinetic

## Usage

Make sure your have ROS Indigo or ROS Kinetic and all the required dependency installed.

Clone this repository into your workspace and do catkin_make:
```
roscd && cd src
git clone https://github.com/TuZZiX/simple_mobile_robot.git
roscd && catkin_make
```

### Launch files

#### Robot in starting pen

To spawn the robot in a starting pen:

`roslaunch simple_mobile_robot robot_in_pen.launch`

Following arguments are supported:

- `move`: default is `none`
```
    Select way to control the mobile robot, options:
		[none] - do nothing
		[keyboard] - control the robot by WASD key
		[square] - run a square path, specify the side length by extra argument [length], e.g. [length:=3.0]
		[circle] - run a circular path, specify the diameter by extra argument [length], e.g. [length:=3.0]
```

- `length`: default is `3.0`
```
    Length of the path when moving the robot in square or circle mode, see argument move definition for detail.
```

- `rviz`: default is `true` <-- turn this off if you don't want rviz
```
    Start rviz and interactive marker to control the robot.
```

- `amcl`: default is `true`
```
    Load map and start AMCL for localization, for now, this is only for make the view in rviz looks better.

    Also, if you found your computer is slow, try to turn this off.
```

- `lidar`: default is `true`
```
    Start lidar estop node to prevent robot from hitting anything.
```

After running this launch file, you can run other node to command the robot.

The launch file also support specify command by launcher arguments, e.g. if you want to run keyboard teleop to command the robot, do:

`roslaunch simple_mobile_robot robot_in_pen.launch move:=keyboard`

It will wait a short period for Gazebo to finish loading and then start the move.

Here is a demo about using this launch file and ways to interact with the robot: https://youtu.be/B8IkBJNDmC8

#### Empty world

To start the robot in an empty world:

`roslaunch simple_mobile_robot simple_mobile_robot.launch`

Following arguments are supported:

- `move`: default is `none`
```
    Select way to control the mobile robot, options:
		[none] - do nothing
		[keyboard] - control the robot by WASD key
		[square] - run a square path, specify the side length by extra argument [length], e.g. [length:=3.0]
		[circle] - run a circular path, specify the diameter by extra argument [length], e.g. [length:=3.0]
```

- `length`: default is `3.0`
```
    Length of the path when moving the robot in square or circle mode, see argument move definition for detail
```

- `rviz`: default is `true` <-- turn this off if you don't want rviz
```
    Start rviz and interactive marker to control the robot.
```

After running this launch file, you can run other node to command the robot.

The launch file also support specify command by launcher arguments, e.g. if you want to do a circle move with diameter = 3.0 in clockwise, do:

`roslaunch simple_mobile_robot simple_mobile_robot.launch move:=circle length:=3`

It will wait a short period for Gazebo to finish loading and then start the move.

Here is a demo about using this launch file and ways to interact with the robot: https://youtu.be/vKvUAmQZhHI

---

### Nodes

After the launch file finish loading, you can start the following nodes to command the robot.

#### Keyboard teleop

As this robot needs a smooth speed profile, so I implement my own cubic speed controller. With an intuitive way to control just like playing racing games

Run:

`rosrun simple_mobile_robot keyboard_controller`

Parameters can be change in [parameters.yaml](/config/parameters.yaml)

```yaml
# keyboard teleop
KEY_TIMEOUT: 0.4    # continue moving for a short period after key released, also to prevent key jitter, in second
STOP_DEACC: 1.5     # when key released, instead of using max acceleration, use a slower deceleration to simulate inertia
MAX_ACC: 3.0        # max acceleration on speed in m/s^2 to start and stop
MAX_SPIN_ACC: 2.4
ACC_JERK: 10.0      # max acceleration jerk for both MAX_ACC and MAX_SPIN_ACC in m/s^3 or rad/s^3
MAX_SPEED: 1.0      # max speed in m/s
MAX_SPIN_RATE: 0.8  # max spinning rate in rad/s
UPDATE_RATE: 100    # control loop running frequency, in hz
```

**Notice: ** If you want to use other teleop package, you need to remap /cmd_vel to /extra_vel to avoid conflict with other command velocity source.

For demo video, see demo for launch files.

#### Square move

Keep looping a square path with open loop control.

Run:

`rosrun simple_mobile_robot square_move [length]`

[length] is the side length of the square path in meter, if you want the robot to move counter clockwise, specify a negative number

Example: `rosrun simple_mobile_robot square_move 3`, run a clockwise square path of length = 3

Video: https://youtu.be/boI3VW1YflU

This video also shows how to passing different arguments into launch file to select functionality

#### Circle move

Keep looping a circle path with open loop control.

Run:

`rosrun simple_mobile_robot circle_move [diameter]`

[diameter] is the diameter of the circular path in meter, if you want the robot to move counter clockwise, specify a negative number

Example: `rosrun simple_mobile_robot circle_move 5`, this run a clockwise circular path of diameter = 5

Speed can be change in [parameters.yaml](/config/parameters.yaml), by changing `MAX_SPEED`.

Video: https://youtu.be/zx1Ngz1CBWU

This video also shows cmd_vel overriding.

#### LIDAR safety stop

A feature to prevent the robot from hitting the wall, it will force the robot to stop when too close to obstacles.

It publishes `lidar_alarm_l`, `lidar_alarm_l` to indicates if there is any obstacles on the left or on the right. `/lidar_alarm` is a combination of both. It will try to move the robot back a little bit to release the alarm. When alarm present, no other velocity command can be execute.

`rosrun simple_mobile_robot lidar_estop`

Parameters can be change in [parameters.yaml](/config/parameters.yaml)

```yaml
# lidar brake
MIN_SAFE_DISTANCE: 0.65 # Safety distance counting from the middle of the laser scan, in m
ROBOT_WIDTH: 0.5        # Width of the robot, in m
```

Demo Video: https://youtu.be/-PnMeZPLats

### Delay launcher

Wait for certain time and run commands. Usually used for waiting for gazebo or rviz to finish loading.

`rosrun simple_mobile_robot delay_launcher.py [sleep time] ...[commands]`

[sleep time] is the amount of time to wait before run the commands, in second.

...[commands] a list of commands to run, just type as you normally do in terminal.

Example: `rosrun simple_mobile_robot delay_launcher.py 2 ls -l`, this will run `ls -l` after 2 seconds.

## Experimental

Do not use them as they are still under construction.

### Trajectory Builder

A path builder that can build a sequence of via points and states between specify start and end pose.

### Absolute path move

Command the robot by specify absolute points (with respect with odom_frame). Response to lidar_alarm to gracefully come to halt. Also listening to `/append_path_queue` and response to `/flush_path_queue_service` for adding goal points or remove all goal points.

Use Trajectory Builder to build the path.

Run:

`rosrun simple_mobile_robot path_move`

or `rosrun simple_mobile_robot path_move test`

or `rosrun simple_mobile_robot path_move square 1.5`

### Steering Correction

A steering correction algorithm to correct the drifting cause by the passive wheel or any source. It will try to align the robot to `desired state published by path_move.

 Run:

`rosrun simple_mobile_robot steering_controller`

If you don't want any steering correction, simply forward velocity command from desired state published by path_move.

`rosrun simple_mobile_robot des_state_forwarder`

## About the model

This robot model have a passive wheel to support the robot. However, it can not be simulated perfectly in Gazebo. Unlike the real world, when there is an over 90 deg rotation on that wheel, it will fight against the other wheels and cause drifting on robot's orientation.

Modified from a robot build by our lab, the original model can be found [here](https://github.com/wsnewman/learning_ros/tree/master/Part_2/mobot_urdf)

## TODO List

- Fix and improve path server and use path server to complete square and circle move

- Add option of using AMCL odom instead of perfect odom from Gazebo

- Add full navigation stack: costmap, planner, move_base, etc.
