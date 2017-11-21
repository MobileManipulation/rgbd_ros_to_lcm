 /********************************************************************
  The Charles Stark Draper Laboratory, Inc.
  555 Technology Square
  Cambridge, MA 02139-3563

  File: lcm_republisher.cpp
  Author: Syler Wagner
  Date: 2017-06-30

  Software License Agreement (BSD License)

  Copyright (c) The Charles Stark Draper Laboratory, Inc. (Draper) 2017.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

  1. Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
  2. Redistributions in binary form must reproduce the above
     copyright notice, this list of conditions and the following
     disclaimer in the documentation and/or other materials provided
     with the distribution.
  3. Neither the name of the copyright holder nor the names of its 
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************/

#include <ros/ros.h>

#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>

// #include <rgbd/conversions.h>
#include <rgbd_ros_to_lcm/jpeg_utils.h>
#include <zlib.h>

#include <lcm/lcm-cpp.hpp>
#include <bot_core/image_t.hpp>
#include <bot_core/images_t.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ImageAndDepthSyncPolicy; 
typedef message_filters::Subscriber<sensor_msgs::Image> ImageSubscriber; 

int i = 0;

class LCMRepublisher
{
  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  // Subscriber for the Kinect RGB stream
  ImageSubscriber *rgb_sub_;
  // Subscriber for the Kinect Depth Map stream
  ImageSubscriber *depth_map_sub_;
  // Kinect RGB-D Signal Synchronizer for Cropping Cloud
  message_filters::Synchronizer<ImageAndDepthSyncPolicy>* rgb_depth_sync_policy;

  ros::Publisher rgb_pub_, depth_pub_;
  sensor_msgs::CameraInfo::ConstPtr camera_info;
  image_geometry::PinholeCameraModel cam_model_;


  std::string sensor_name;
  std::string sensor_frame_id;

  std::string cam_info_topic, cloud_topic, lcm_url, rgb_topic, depth_topic, lcm_channel;
  bool subscribe_point_cloud;


  bool debug_output, debug_print_statements;
  bool compress_rgb;
  bool compress_depth;
  std::string output_topic;

  lcm::LCM lcm;

  // Compression Buffers:
  int jpeg_quality_ = 94;
  uint8_t* image_buf_;
  int image_buf_size_;

  uint16_t* depth_unpack_buf_;
  int depth_unpack_buf_size_;

  uint8_t* depth_compress_buf_;
  int depth_compress_buf_size_;

  bot_core::image_t * rgb_lcm;
  bot_core::image_t * depth_lcm;

public:
  LCMRepublisher()
  {
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("lcm_url", lcm_url, "");
    private_nh.param<std::string>("output_lcm_channel", lcm_channel, "");

    
    if(!lcm.good())
        ros::shutdown();

    private_nh.getParam("compress_rgb", compress_rgb);
    private_nh.getParam("subscribe_point_cloud", subscribe_point_cloud);
    private_nh.getParam("compress_depth", compress_depth);
    if (private_nh.getParam("debug_output", debug_output))
    {
      depth_pub_ = private_nh.advertise<sensor_msgs::Image>("depth", 1, true);
      rgb_pub_ = private_nh.advertise<sensor_msgs::Image>("image", 1, true);
    }


    private_nh.param<std::string>("sensor_name", sensor_name, "camera");
    
    private_nh.param("debug_print_statements", debug_print_statements, false);

    private_nh.param<std::string>("camera_info_topic", cam_info_topic, "camera_info");

    if (subscribe_point_cloud)
    {
      private_nh.param<std::string>("cloud_topic", cloud_topic, "point_cloud_topic");
      sub_ = nh_.subscribe<PointCloud>(cloud_topic, 1, &LCMRepublisher::cloudCallbackLCM, this);
      ROS_WARN("Subscribed to cloud topic: %s", cloud_topic.c_str());
    }
    else
    {
      private_nh.param<std::string>("rgb_topic", rgb_topic, "rgb_topic");
      private_nh.param<std::string>("depth_topic", depth_topic, "depth_topic");

      rgb_sub_ = new ImageSubscriber(nh_, rgb_topic, 1);
      depth_map_sub_ = new ImageSubscriber(nh_, depth_topic, 1);

      rgb_depth_sync_policy = new message_filters::Synchronizer<ImageAndDepthSyncPolicy>(ImageAndDepthSyncPolicy(10), *rgb_sub_, *depth_map_sub_);
      rgb_depth_sync_policy->registerCallback(boost::bind(&LCMRepublisher::syncCallbackLCM, this, _1, _2));
      ROS_WARN("Subscribed to synchronized topics: %s, %s", rgb_topic.c_str(), depth_topic.c_str());
    }

    //$ initialize pinhole camera model from camera info 
    ROS_WARN("Waiting for camera parameters on topic: %s", cam_info_topic.c_str());

    camera_info = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(cam_info_topic, nh_);
    cam_model_.fromCameraInfo(camera_info);
    ROS_WARN("Got camera parameters on %s", cam_info_topic.c_str());
    sensor_frame_id = camera_info->header.frame_id;

    private_nh.param<std::string>("output_topic", output_topic, "/"+sensor_name+"/image_and_depth");

    //$ JPEG compression parameters
    image_buf_size_ = 640 * 480 * 10;
    if (0 != posix_memalign((void**) &image_buf_, 16, image_buf_size_)) {
      fprintf(stderr, "Error allocating image buffer\n");
    }

    //$ allocate space for ZLIB compression depth data
    depth_compress_buf_size_ = 640 * 480 * sizeof(int16_t) * 4;
    depth_compress_buf_ = (uint8_t*) malloc(depth_compress_buf_size_);

    rgb_lcm = new bot_core::image_t();
    depth_lcm = new bot_core::image_t();
  }

  ~LCMRepublisher()
  {
  }


  /**
   * Convert an organized RGB point cloud to an RGB image and depth image.
   * @param cloud       input organized color point cloud
   * @param image     output RGB image cv::Mat
   * @param depth     output depth image cv::Mat
   * @return        true if conversion successful
   */
bool cloudToImageAndDepthMat(const PointCloud::ConstPtr& cloud, cv::Mat& image, cv::Mat& depth)
{
  if ((cloud->width * cloud->height) == 0)
  {
    return false; //$ return if the cloud is not dense!
  }

  try
  {
    sensor_msgs::Image image_msg;
    //$ cloud to RGB image
    pcl::toROSMsg (*cloud, image_msg); 
    cv_bridge::CvImagePtr input_bridge;

    try 
    {
      input_bridge = cv_bridge::toCvCopy(image_msg, image_msg.encoding);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex)
    {
      ROS_ERROR("Error getting RGB cv::Mat from point cloud derived image message.");
      return false;
    }

  }
  catch (std::runtime_error e)
  {
    ROS_ERROR_STREAM("Error in converting cloud to RGB image message: "
      << e.what());
    return false;
  }

  depth = cv::Mat(cloud->height, 
    cloud->width, CV_32FC1); 

  int i = 0;
  int j = 0;

  //$ extract RGB depth image from point cloud
  BOOST_FOREACH (const pcl::PointXYZRGB& pt, cloud->points) 
  {
    depth.at<float>(j, i) = pt.z;

    i++;
    if (i == depth.cols)
    {
      i = 0;
      j++;
    }
  }

  //$ convert meters to Kinect-type depth images with integer content in mm 
  depth.convertTo(depth, CV_16UC1, 1000.0); 

  return true;
}

  void publishLCM(long unsigned int timestamp, cv::Mat rgb, cv::Mat depth)
  {
    // create LCM message

    rgb_lcm->utime = timestamp;

    rgb_lcm->width = rgb.cols;
    rgb_lcm->height = rgb.rows;
    rgb_lcm->row_stride = rgb.cols;

    rgb_lcm->nmetadata = 0;

    rgb_lcm->row_stride = rgb.step;

    rgb_lcm->size = rgb.cols * rgb.rows * 3;
    rgb_lcm->data = std::vector<unsigned char>(rgb.ptr(), rgb.ptr() + rgb_lcm->size);
    if (!compress_rgb) {
      rgb_lcm->pixelformat = 861030210; //$ PIXEL_FORMAT_BGR
    }
    else
    {
      int compressed_size =  rgb_lcm->height*rgb_lcm->row_stride;
      int compression_status = jpegijg_compress_8u_rgb(rgb_lcm->data.data(), rgb_lcm->width, rgb_lcm->height, rgb_lcm->row_stride,
       image_buf_, &compressed_size, jpeg_quality_);

      if (0 != compression_status) {
        fprintf(stderr, "JPEG compression failed...\n");
      }
      memcpy(&rgb_lcm->data[0], image_buf_, compressed_size);
      rgb_lcm->size = compressed_size;
      rgb_lcm->pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;      
    }

    depth_lcm->utime = rgb_lcm->utime; 

    depth_lcm->width = depth.cols;
    depth_lcm->height = depth.rows;
    depth_lcm->row_stride = depth.cols;

    depth_lcm->nmetadata = 0;
    depth_lcm->row_stride = depth.step;

    if (!compress_depth) {
      depth_lcm->size = depth.cols * depth.rows * 2;
      depth_lcm->data = std::vector<unsigned char>(depth.ptr(), depth.ptr() + depth_lcm->size);
      depth_lcm->pixelformat = 357; //$ PIXEL_FORMAT_BE_GRAY16
    }
    else
    {
      int uncompressed_size = depth_lcm->height * depth_lcm->width * sizeof(short);
      if (debug_print_statements)
        ROS_INFO("depth uncompressed_size: %d", uncompressed_size);
      unsigned long compressed_size = depth_compress_buf_size_;
      if (debug_print_statements)
        ROS_INFO("depth compressed_size: %lu", compressed_size);
      depth_lcm->data = std::vector<unsigned char>(depth.ptr(), depth.ptr() + depth.cols * depth.rows * 2);

      compress2(depth_compress_buf_, &compressed_size, (const Bytef*) depth_lcm->data.data(), uncompressed_size,
        Z_BEST_SPEED);  
      if (debug_print_statements)
        ROS_INFO("depth compressed successfully with zlib!");

      depth_lcm->size =(int)compressed_size;
      memcpy(&depth_lcm->data[0], depth_compress_buf_, compressed_size);
      depth_lcm->pixelformat = -2; 
      if (debug_print_statements)
        ROS_INFO("memcpy to depth_lcm message complete!");

    }

    bot_core::images_t images;
    images.utime = rgb_lcm->utime;
    images.n_images = 2;
    images.images.push_back(*rgb_lcm);
    images.images.push_back(*depth_lcm);
    images.image_types.push_back( 0 ) ; //$ LEFT = 0

    if (!compress_depth) {
      images.image_types.push_back( 4 ) ; //$ DEPTH_MM = 4
    }
    else
    {
      images.image_types.push_back( 6 ) ; //$ DEPTH_MM_ZIPPED = 6
      // z depth, values similar to the OpenNI format, zipped with zlib
    }
    lcm.publish(lcm_channel, &images);

    i++;
    ROS_WARN("Frame %d", i);

  }
  /**
   * Cloud callback method.
   * @param image_msg   message with time-synchronized RGB image and depth
   */
   void
   cloudCallbackLCM(const PointCloud::ConstPtr& cloud)
   {

    std_msgs::Header cloud_header;
    pcl_conversions::fromPCL(cloud->header, cloud_header);
    ros::Time acquisition_time = cloud_header.stamp;

    if (debug_print_statements)
      ROS_INFO("Point cloud acquired at %lu",  (long unsigned int) cloud->header.stamp);

    // printf ("Cloud: width = %d, height = %d\n", cloud->width, cloud->height);

    cv::Mat rgb, depth;

    if (!cloudToImageAndDepthMat(cloud, rgb, depth))
    {
      //$ return from callback if conversion to images fails
      return;
    }

    if (debug_print_statements)
      ROS_INFO("conversion to rgb and depth cv::Mat done");

    publishLCM((long unsigned int) cloud->header.stamp, rgb, depth);

  }


  /**
   * Synchronized RGB and depth image callback method.
   * @param image_msg   message with time-synchronized RGB image and depth
   */
   void
   syncCallbackLCM(const sensor_msgs::ImageConstPtr rgb_msg, const sensor_msgs::ImageConstPtr depth_msg)
   {


    // Converting to OpenCV Mat
    cv::Mat rgb = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding)->image;
    cv::Mat depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;

    if (debug_print_statements)
      ROS_INFO("Stamp %llu",  rgb_msg->header.stamp.toNSec());

    long unsigned int timestamp = (long unsigned int) (rgb_msg->header.stamp.toNSec() / 1e3);

    if (debug_print_statements)
      ROS_INFO("Images acquired at %lu",  timestamp);

    publishLCM(timestamp, rgb, depth);
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lcm_republisher");
  LCMRepublisher republisher;
  ros::spin();
}
