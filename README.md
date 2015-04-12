# navigation_assistant_3dpc
ROS package containing a navigation assistant based on 3D Point Clouds.

## Overview
The aim of this ROS Package is to provide a navigation assistant based on a shared control approach. The user will 
teleoperate a mobile robot using a joystick and the commands will be adapted in order to avoid collisions. 

The information from the 3D Point Cloud (3DPC) is used to create a d_coll function that estimates the distance from 
the robot to an obstacle. This way, a maximum velocity free of collisions can be computed and applied to the robot.
The navigation assistant will correct the robot's velocity at real time in order to avoid collisions. 

Moreover, in order to deal with delays, in order to correct the navigation at a period T_k some predictions are done using information from the previous T_(k-1)

## Quick Start
To begin with, you need to have a 3D Point Cloud topic and a joystick command publisher.
```
[/NAMESPACE]/points2
[/NAMESPACE]/cmd_vel_pre
```
To start the navigation assistant issue the following command;
```
$ rosrun navigation_assistant_3dpc nav_assistant
```
To check that the system is working, you can check if there is activity in the `/cmd_vel` topic when the joystick is moved. In this topic the corrected commands will be produced:

`$ rostopic hz /cmd_vel`

## Nodes

### Navigation Assistant (`nav_assistant`)
This node, given a joystick command, will produced a collision-free velocity to the robot. In order to do so, it will use the information from the 3D Point Clouds. 

#### Subscribed topics

* `/points2` (`sensor_msgs/PointCloud2`)

Contains the obstacle information in the form of point cloud. This will be used to produce the d_coll function

* `/cmd_vel_pre` (`geometry_msgs/TwistStamped`)

Joystick command from the teleoperator, the one that will be corrected for a safer navigation

#### Published topics

* `/cmd_vel` (`geometry_msgs/Twist`)

Corrected velocity command that is sent back to the robot.

### Parameters

#### Kinematics

* `~a_max (double, default=0.4)`

Maximum acceleration that the robot can handle

* `~max_v_max (double, default=0.4)`

Maximum linear velocity that the robot can handle

* `~reverse_driven (bool, default=false)`

Indicates whether the robot is being driven forward or backwards

* `~dist_axis_border (double, default=0)`

Distance of the robot's center of coordinates to its border.

* `~dist_axis_camera (double, default=0)`

Distance of the robot's center of coordinates to its camera.

* `~freq (double, default=10)`
 
Frequency in which the navigation assistant applies braking commands.

* `~t_update_vj (double, default=0.8)`
 
Time window used for predicitions

#### d_coll calculation

* `interp_factor (double, default=0.7)`

Constant used for interpolation. The bigger its value, the less likely a space between two points will be considered an obstacle.

* `far (double, default=50)`

Interpolation constant. The bigger its value, the less likely a space between two points will be considered an obstacle.

#### Profiling

* `~measure_time (bool, default=false)`

If enabled, time information (communication and processing times) will be saved in text files.

* `~output_filename_proc (string, default=time_buffer_proc.csv)`

Path to the text file where Navigation Assistant's computation times will be written

* `~output_filename_comm (string, default=time_buffer_comm.csv)`

Path to the text where Navigation Assistant's communication times will be written

### Verbosity

* `~verbose (bool, default=false)`

Print verbose messages using `ROS_INFO`

## Docker images

A docker image repository is available with the cloud_3dpc_extractor pre-installed and ready for deployment in any cloud or local system. Please check https://registry.hub.docker.com/u/javsalgar/cloud_3dpc_extractor/
