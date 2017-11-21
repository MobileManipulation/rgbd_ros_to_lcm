# rgbd_ros_to_lcm
ROS node to republish RGBD sensor data in LCM

This ROS package contains a ROS node that subscribes to RGB and depth images from an RGBD sensor (Kinect, RealSense, etc.) and republishes the data to an LCM channel. The output LCM message format matches the format of the [`openni2-camera-lcm`](https://github.com/openhumanoids/openni2-camera-lcm) driver and is compatible with the [LabelFusion](http://labelfusion.csail.mit.edu/) pipeline for generating ground truth labels of RGBD data.

# Installation

## Dependencies

* LCM: [Build Instructions](https://github.com/lcm-proj/lcm/blob/master/docs/content/build-instructions.md)
* [PCL](https://github.com/PointCloudLibrary/pcl)

Clone this repository into your catkin workspace: 
```bash
cd ~/catkin_ws/src
git clone https://github.com/MobileManipulation/rgbd_ros_to_lcm.git
cd rgbd_ros_to_lcm
```

Before invoking `catkin_make`, the C++ LCM message definitions need to be generated for the image LCM types:

```bash
cd include
lcm-gen -x ../lcmtypes/*.lcm
cd ~/catkin_ws
catkin_make
```

# Usage

To run the `lcm_republisher` node, use the example launch file provided:

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

* `output_lcm_channel`: name of LCM channel for published output
* `compress_rgb`: use JPEG compression for RGB images
* `compress_depth`: use ZLIB compression for depth images

The default values of these parameters match the output of `openni2-camera-lcm` with both JPEG and ZLIB compression enabled. The data is published on the `OPENNI_FRAME` LCM channel.


