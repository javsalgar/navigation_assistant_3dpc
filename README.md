# navigation_assistant_3dpc
ROS Package containing a cloud-based dynamically scalable stereo vision platform based on stereo_image_proc.

## Overview
The aim of this ROS Package is to provide a 3D Point Cloud (3DPC) extraction platform using Cloud Computing's main 
features, especially that of dynamic scalability. This implies that, should the user require faster stereo image 
processing times, then it is possible to launch more 3D Point Cloud Extractors at runtime to satisfy the demand. On the other
side, if the user required less computing power, then it is possible to shut down the extractors at runtime without
having to restart the platform. 

This platform uses the stereo_image_proc library (http://wiki.ros.org/stereo_image_proc) for the 3D Point Cloud Extraction,
and consists of two nodes: the front-end buffer (`stereo_cam_buffer`) and the 3DPC Extractors (extractor_node). In order
to exploit the parallelism, a pipeline is created. The front-end node receives the stereo stream and scatters it to the 3DPC Extractors in a pipeline fashion. 

## Quick Start
To begin with, you need to have a calibrated stereo camera running with (at least) the following topics:
```
[/NAMESPACE]/left/image_raw
[/NAMESPACE]/right/image_raw
[/NAMESPACE]/left/camera_info
[/NAMESPACE]/right/camera_info
```
A minimal architecture would consist of one `stereo_cam_buffer` instance and one `extractor_node` instance. To launch them (in the same machine) use the following commands:
```
$ rosrun stereo_cam_buffer buffer
$ rosrun extractor_node extractor_node.py
```
To check that the system is working, you can check if there is activity in the `/points2` topic, where the generated 3DPCs are available:

`$ rostopic hz /points2`

You can also use visualization tools like `rviz`. Should you require more computing power, you can launch more `extractor_node` instances at runtime:

`$ rosrun extractor_node extractor_node.py`

## Cloud execution
If you have a cloud (or a cluster) available, you can execute the architecture using multiple nodes. In order to do so, one of the nodes must be the master. The following example will suppose four machines: `camera.example.com` ,`frontend.example.com`, `extractor1.example.com` and `extractor2.example.com`. camera.example.com will have the stereo camera running, and will be the ROS Master. `frontend.example.com` will have a `stereo_cam_buffer` instance, extractor1.example.com will have one `extractor_node` instance and `extractor2.example.com` will have two `extractor_node` instances. In order to do so, the following commands are necessary:

```
user@camera:~$ roscore &
user@camera:~$ <command to launch the camera node>

user@frontend:~$ ROS_MASTER_URI=http://camera.example.com:11311 rosrun stereo_cam_buffer buffer

user@extractor1:~$ ROS_MASTER_URI=http://camera.example.com:11311 rosrun extractor_node extractor_node.py

user@extractor2:~$ ROS_MASTER_URI=http://camera.example.com:11311 rosrun extractor_node extractor_node.py
```

This example assumes that you have a DNS server that can resolve all the domain names. Otherwise, you will need to
edit the /etc/hosts file in all the computers. 

## Nodes

### Stereo Cam Buffer (`stereo_cam_buffer`)
This node receives the stereo stream from the camera and scatters it in a round-robin fashion. In other words, it buffers and forwards the stereo frame pairs to have them processed by the 3DPC Extractors. It allows the use of either tcp or udp, together with different compression techniques that the image_transport package offers (raw, theora, compressed). 

#### Subscribed topics
##### Left Camera
* `left/image_raw` (`sensor_msgs/Image`)

 Image stream from the left camera
* `left/camera_info` (`sensor_msgs/CameraInfo`)

 Metadata from the left camera

##### Right Camera
* `right/image_raw` (`sensor_msgs/Image`)

 Image stream from the right camera
* `right/camera_info` (`sensor_msgs/CameraInfo`)

 Metadata from the right camera

##### Overall 3DPC Extractor management
* new_3dpc_extractor (`cloud_3dpc_extractor_msgs/New3DPCExtractor`)
 
When a new 3DPC Extractor is available, the Stereo Cam Buffer will be notified using this topic.

##### Management of each 3DPC Extractor (there is one of these topics per 3DPC Extractor running) 
* <3DPC id>/bond (`bond/Status`)

Checks whether the 3DPC Extractor is alive 

#### Published topics

##### Management of each 3DPC Extractor (there is one of these topics per 3DPC Extractor running) 
* <3DPC Extr. id>/bond (`bond/Status`)

Checks whether the 3DPC Extractor is alive 

##### Stereo frame forwarding  (there is one of these topics per 3DPC Extractor running) 

* `<3DPC Extr. id>/left/image_raw` (`sensor_msgs/Image`)

 Forwarded stream to the left camera
* `<3DPC Extr. id>left/camera_info` (`sensor_msgs/CameraInfo`)

 Metadata from the left camera

* `<3DPC Extr. id>right/image_raw` (`sensor_msgs/Image`)

Forwarded stream to the right camera

* `<3DPC Extr. id>right/camera_info` (`sensor_msgs/CameraInfo`)

### Parameters

#### Camera stream reception

* `~use_udp (bool, default=false)`

If enabled, the camera stream is received using the UDP transport protocol.

* `~transport (string, default=raw)`

Compression mechanism of the camera stream. The options are those allowed by `image_transport` (raw, theora, compressed).

* `~queue_size (int, default=1)`

Size of the stereo frame queue (for systems with real time constraints, a value higher than 1 is not recommended) 

* `~approximate_sync (bool, default=false)`
 
If enabled, the frame synchronization (from left and right cameras) will not be based on exact timestamp but an approximation.

#### 3DPC Extractor's bond management

* `~check_if_broken (bool, default=true)`

If enabled, the Stereo Cam Buffer will check if the 3DPC Extractors are still alive. If a 3DPC Extractor dies, then no more frames will be sent to that node. 

* `~heartbeat_period (double, default=5.0)`
 
Bond's heartbeat period (for more information, see bond library's documentation page)

* `~heartbeat_timeout (double, default=10.0)`
 
Bond's heartbeat timeout (for more information, see bond library's documentation page)

#### Profiling

* `~measure_time (bool, default=false)`

If enabled, time information (communication and processing times) will be saved in text files.

* `~output_filename_proc (string, default=time_buffer_proc.csv)`

Path to the text file where Stereo Cam Buffer's computation times will be written

* `~output_filename_comm (string, default=time_buffer_comm.csv)`

Path to the text where Stereo Cam Buffer's communication times will be written

### 3DPC Extractor (`extractor_node`)
This node is a wrapper of the `stereo_image_proc` library conceived to be replicated as much times as necessary to 
obtain better 3DPC Extraction processing times. It is also able to communicate with the Stereo Cam Buffer and notify
whether it is alive or not (so the platform can scale out and back depending on the needs). All 3DPC Extractors will
share the same /points2 and /disparity topic, so for the subscribed process it will appear as one single stream 
of 3DPCs. For more information on how the 3DPC extraction is done, please refer to the `stereo_image_proc` documentation. 

#### Published topics

##### Contact with Stereo Cam Buffer

* `/bond` (`bond/Status)`

Used to communicate status with the Stereo Cam Buffer.

* `/new_3dpc_extractor` (`cloud_3dpc_extractor_msgs/New3DPCExtractor`)

When this node is awake, it will notify the Stereo Cam Buffer.

##### stereo_image_proc topics (please refer to stereo_image_proc documentation for more information)

* `left/image_mono` (`sensor_msgs/Image`)
* `left/image_rect` (`sensor_msgs/Image`)
* `left/image_color` (`sensor_msgs/Image`)
* `left/image_rect_color` (`sensor_msgs/Image`)
* `right/image_mono` (`sensor_msgs/Image`)
* `right/image_rect` (`sensor_msgs/Image`)
* `right/image_color` (`sensor_msgs/Image`)
* `right/image_rect_color` (`sensor_msgs/Image`)
* `points` (`sensor_msgs/PointCloud`)
* `disparity` (`stereo_msgs/DisparityImage`)
* `points2` (`sensor_msgs/PointCloud2`)

#### Subscribed topics

##### Contact with Stereo Cam Buffer

* `/bond` (`bond/Status)`

Used to communicate status with the Stereo Cam Buffer.

##### stereo_image_proc topics (please refer to `stereo_image_proc` documentation for more information)

* `left/image_raw` (`sensor_msgs/Image`)
* `left/camera_info` (`sensor_msgs/CameraInfo`)
* `right/image_raw` (`sensor_msgs/Image`)
* `right/camera_info` (`sensor_msgs/CameraInfo`)

### Parameters

#### Wrapper

* `~delay (int, default=5)`
 
Time to wait before starting the 3DPC Extractor

* `~restore_time (int, default=5)`
 
Period T where the bond is restored in case of fail.

* `~heartbeat_period (double, default=5.0)`
 
Bond's heartbeat period (for more information, see bond library's documentation page)

* `~heartbeat_timeout (double, default=10.0)`
 
Bond's heartbeat timeout (for more information, see bond library's documentation page)

#### `stereo_image_proc` parameters (please refer to `stereo_image_proc` documentation for more information)

* `~prefilter_size` (`int, default: 23`)
* `~prefilter_cap` (`int, default: 33`)
* `~correlation_window_size` (`int, default: 41`)
* `~min_disparity` (`int, default: 44`)
* `~disparity_range` (`int, default: 64`)
* `~uniqueness_ratio` (`double, default: 15.0`)
* `~texture_threshold` (`int, default: 10`)
* `~speckle_size` (`int, default: 356`)
* `~speckle_range` (`int, default: 7`)

## Docker images

A docker image repository is available with the cloud_3dpc_extractor pre-installed and ready for deployment in any cloud or local system. Please check https://registry.hub.docker.com/u/javsalgar/cloud_3dpc_extractor/
