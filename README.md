# rgbd_ros_to_lcm
ROS node to republish RGBD sensor data in LCM

This ROS package contains a ROS node that subscribes to RGB and depth images from an RGBD sensor (Kinect, RealSense, etc.) and republishes the data to an LCM channel. The output LCM message format matches the format of the [`openni2-camera-lcm`](https://github.com/openhumanoids/openni2-camera-lcm) driver and is compatible with the [LabelFusion](http://labelfusion.csail.mit.edu/) pipeline for generating ground truth labels of RGBD data.

For converting LCM messages to ROS, please refer to the [`lcm_to_ros`](https://github.com/nrjl/lcm_to_ros) package.

# Installation

## Dependencies

* LCM: [Build Instructions](https://github.com/lcm-proj/lcm/blob/master/docs/content/build-instructions.md)
* [PCL](https://github.com/PointCloudLibrary/pcl)

Clone this repository into your catkin workspace: 
```bash
cd ~/catkin_ws/src
git clone https://github.com/MobileManipulation/rgbd_ros_to_lcm.git
cd rgbd_ros_to_lcm
cd ~/catkin_ws
catkin_make
```

To generate C++ LCM message definitions for the image LCM types:

```bash
cd include
lcm-gen -x ../lcmtypes/*.lcm
```

# Usage

The `lcm_republisher` ROS node subscribes to the output of an RGBD sensor driver ROS node, like the [RealSense driver](https://github.com/intel-ros/realsense). First, launch the sensor driver. Then, to run the `lcm_republisher` node, use the example launch file provided:

```bash
roslaunch rgbd_ros_to_lcm lcm_republisher.launch
```

## Parameters

### Input Parameters

* `subscribe_point_cloud`: subscribe to an organized RGB point cloud topic instead of individually subscribing to RGB and depth topics
* `cloud_topic`: organized RGB point cloud topic 
* `rgb_topic`: RGB image topic
* `depth_topic`: registered depth image topic

*Note:* If subscribing to RGB and depth image topics individually for use with LabelFusion, make sure the depth topic is registered into the RGB image frame to prevent alignment problems. 

### Output Parameters

* `compress_rgb`: use JPEG compression for RGB images
* `compress_depth`: use ZLIB compression for depth images
* `output_lcm_channel`: name of LCM channel for published output
* `lcm_url`: LCM URL

The default values of these parameters match the output of `openni2-camera-lcm` with both JPEG and ZLIB compression enabled. The data is published on the `OPENNI_FRAME` LCM channel.

# Who do I talk to?

To report bugs or problems with the package, please open an issue on our issue tracker.




