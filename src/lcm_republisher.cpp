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

class LCMRepublisher
{
  int i_ = 0;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;

  ImageSubscriber* rgb_sub_;
  ImageSubscriber* depth_map_sub_;

  //$ message synchronizer for RGB and depth image
  message_filters::Synchronizer<ImageAndDepthSyncPolicy>* rgb_depth_sync_policy_;

  ros::Publisher rgb_pub_, depth_pub_;

  bool debug_output_, debug_print_statements_;
  bool compress_rgb_;
  bool compress_depth_;

  lcm::LCM lcm_;
  std::string lcm_channel_;

  //$ compression buffers:
  int jpeg_quality_ = 94;
  uint8_t* image_buf_;
  int image_buf_size_;

  uint16_t* depth_unpack_buf_;
  int depth_unpack_buf_size_;

  uint8_t* depth_compress_buf_;
  int depth_compress_buf_size_;

  bot_core::image_t* rgb_lcm_msg_;
  bot_core::image_t* depth_lcm_msg_;

  bool enforce_resize_ {};
  int resize_width_ {};
  int resize_height_ {};
  bool assert_ratio_maintained_ {};

public:
  LCMRepublisher()
  {
    ros::NodeHandle private_nh("~");

    std::string cloud_topic, lcm_url, rgb_topic, depth_topic;


    private_nh.param<std::string>("lcm_url", lcm_url, "");
    private_nh.param<std::string>("output_lcm_channel", lcm_channel_, "OPENNI_FRAME");

    if(!lcm_.good())
        ros::shutdown();

    private_nh.getParam("compress_rgb", compress_rgb_);
    private_nh.getParam("compress_depth", compress_depth_);
    if (private_nh.getParam("debug_output", debug_output_))
    {
      depth_pub_ = private_nh.advertise<sensor_msgs::Image>("depth", 1, true);
      rgb_pub_ = private_nh.advertise<sensor_msgs::Image>("image", 1, true);
    }
    
    private_nh.param("debug_print_statements", debug_print_statements_, false);

    bool subscribe_point_cloud;
    private_nh.getParam("subscribe_point_cloud", subscribe_point_cloud);

    private_nh.param<std::string>("rgb_topic", rgb_topic, "rgb_topic");
    private_nh.param<std::string>("depth_topic", depth_topic, "depth_topic");

    int rgb_width, rgb_height, depth_width, depth_height;

    if (subscribe_point_cloud)
    {
      private_nh.param<std::string>("cloud_topic", cloud_topic, "point_cloud_topic");
      sub_ = nh_.subscribe<PointCloud>(cloud_topic, 1, &LCMRepublisher::cloudCallbackLCM, this);
      ROS_INFO("Subscribed to cloud topic: %s", cloud_topic.c_str());

      sensor_msgs::PointCloud2::ConstPtr cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(cloud_topic, nh_);
      rgb_width = depth_width = cloud_msg->width;
      rgb_height = depth_height = cloud_msg->height;
    }
    else
    {
      private_nh.param<std::string>("rgb_topic", rgb_topic, "rgb_topic");
      private_nh.param<std::string>("depth_topic", depth_topic, "depth_topic");

      rgb_sub_ = new ImageSubscriber(nh_, rgb_topic, 1);
      depth_map_sub_ = new ImageSubscriber(nh_, depth_topic, 1);

      rgb_depth_sync_policy_ = new message_filters::Synchronizer<ImageAndDepthSyncPolicy>(ImageAndDepthSyncPolicy(10), *rgb_sub_, *depth_map_sub_);
      rgb_depth_sync_policy_->registerCallback(boost::bind(&LCMRepublisher::syncCallbackLCM, this, _1, _2));
      ROS_INFO("Subscribed to synchronized topics: %s, %s", rgb_topic.c_str(), depth_topic.c_str());

      //$ get one message from each topic to get image dimensions
      sensor_msgs::Image::ConstPtr rgb_msg = ros::topic::waitForMessage<sensor_msgs::Image>(rgb_topic, nh_);
      sensor_msgs::Image::ConstPtr depth_msg = ros::topic::waitForMessage<sensor_msgs::Image>(depth_topic, nh_);
      rgb_width = rgb_msg->width;
      rgb_height = rgb_msg->height;
      depth_width = depth_msg->width;
      depth_height = depth_msg->height;

      if (rgb_width != depth_width || rgb_height != depth_height) {
        ROS_WARN("Expected rgb dimensions (%d,%d) to be equal to depth dimensions (%d,%d)",
        rgb_width, rgb_height, depth_width, depth_height);
      }

      // user-specified outgoing image dimensions for both depth and rgb
      private_nh.param<bool>("enforce_resize", enforce_resize_, false);
      private_nh.param<int>("resize_width", resize_width_, rgb_width);
      private_nh.param<int>("resize_height", resize_height_, rgb_height);
      private_nh.param<bool>("assert_ratio_maintained", assert_ratio_maintained_, false);

      if (resize_width_ == rgb_width && resize_height_ == rgb_height) {
        ROS_WARN("Resize desired, but real image dimensions already equal desired");
        enforce_resize_ = false;
      }

      auto aspect_ratio_orig {rgb_width / rgb_height};
      auto aspect_ratio_resized {resize_width_ / resize_height_};
      if (assert_ratio_maintained_ && aspect_ratio_orig != aspect_ratio_resized) {
        ROS_ERROR("Aspect ratio not maintained in resized dimension!");
        ros::shutdown();
      }
    }

    //$ JPEG compression parameters
    image_buf_size_ = rgb_width * rgb_height * 10;
    if (0 != posix_memalign((void**) &image_buf_, 16, image_buf_size_)) 
    {
      ROS_ERROR("Error allocating image buffer");
      ros::shutdown();
    }
    ROS_INFO("Allocated jpeg compression buffer for dimensions %d x %d", rgb_width, rgb_height);

    //$ allocate space for ZLIB compression depth data
    depth_compress_buf_size_ = depth_width * depth_height * sizeof(int16_t) * 4;
    depth_compress_buf_ = (uint8_t*) malloc(depth_compress_buf_size_);

    ROS_INFO("Allocated zlib compression buffer for dimensions %d x %d", depth_width, depth_height);

    rgb_lcm_msg_ = new bot_core::image_t();
    depth_lcm_msg_ = new bot_core::image_t();
  }

  ~LCMRepublisher()
  {
  }


  /**
   * Convert an organized RGB point cloud to an RGB image and depth image.
   * @param cloud     input organized color point cloud
   * @param image     output RGB image cv::Mat
   * @param depth     output depth image cv::Mat
   * @return          true if conversion successful
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
    catch (std::runtime_error& e)
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

 /**
  * Publish LCM message with combined RGB and depth image.
  * @param timestamp     time images were acquired
  * @param rgb           RGB image cv::Mat
  * @param depth         depth image cv::Mat
  */
  void publishLCM(long unsigned int timestamp, cv::Mat rgb, cv::Mat depth)
  {
    //$ create LCM message

    rgb_lcm_msg_->utime = timestamp;

    rgb_lcm_msg_->width = rgb.cols;
    rgb_lcm_msg_->height = rgb.rows;
    rgb_lcm_msg_->row_stride = rgb.cols;

    rgb_lcm_msg_->nmetadata = 0;

    rgb_lcm_msg_->row_stride = rgb.step;

    rgb_lcm_msg_->size = rgb.cols * rgb.rows * 3;
    rgb_lcm_msg_->data = std::vector<unsigned char>(rgb.ptr(), rgb.ptr() + rgb_lcm_msg_->size);
    if (!compress_rgb_)
    {
      rgb_lcm_msg_->pixelformat = 861030210; //$ PIXEL_FORMAT_BGR
    }
    else
    {
      int compressed_size =  rgb_lcm_msg_->height*rgb_lcm_msg_->row_stride;
      int compression_status = jpegijg_compress_8u_rgb(rgb_lcm_msg_->data.data(), rgb_lcm_msg_->width, rgb_lcm_msg_->height, rgb_lcm_msg_->row_stride,
       image_buf_, &compressed_size, jpeg_quality_);

      if (0 != compression_status) 
      {
        fprintf(stderr, "JPEG compression failed...\n");
      }
      memcpy(&rgb_lcm_msg_->data[0], image_buf_, compressed_size);
      rgb_lcm_msg_->size = compressed_size;
      rgb_lcm_msg_->pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
    }

    depth_lcm_msg_->utime = rgb_lcm_msg_->utime;

    depth_lcm_msg_->width = depth.cols;
    depth_lcm_msg_->height = depth.rows;
    depth_lcm_msg_->row_stride = depth.cols;

    depth_lcm_msg_->nmetadata = 0;
    depth_lcm_msg_->row_stride = depth.step;

    if (!compress_depth_)
    {
      depth_lcm_msg_->size = depth.cols * depth.rows * 2;
      depth_lcm_msg_->data = std::vector<unsigned char>(depth.ptr(), depth.ptr() + depth_lcm_msg_->size);
      depth_lcm_msg_->pixelformat = 357; //$ PIXEL_FORMAT_BE_GRAY16
    }
    else
    {
      int uncompressed_size = depth_lcm_msg_->height * depth_lcm_msg_->width * sizeof(short);
      if (debug_print_statements_)
        ROS_INFO("depth uncompressed_size: %d", uncompressed_size);
      unsigned long compressed_size = depth_compress_buf_size_;
      if (debug_print_statements_)
        ROS_INFO("depth compressed_size: %lu", compressed_size);
      depth_lcm_msg_->data = std::vector<unsigned char>(depth.ptr(), depth.ptr() + depth.cols * depth.rows * 2);

      compress2(depth_compress_buf_, &compressed_size, (const Bytef*) depth_lcm_msg_->data.data(), uncompressed_size,
        Z_BEST_SPEED);  
      if (debug_print_statements_)
        ROS_INFO("depth compressed successfully with zlib!");

      depth_lcm_msg_->size = (int)compressed_size;
      memcpy(&depth_lcm_msg_->data[0], depth_compress_buf_, compressed_size);
      depth_lcm_msg_->pixelformat = -2;
      if (debug_print_statements_)
        ROS_INFO("memcpy to depth_lcm message complete!");

    }

    bot_core::images_t images;
    images.utime = rgb_lcm_msg_->utime;
    images.n_images = 2;
    images.images.push_back(*rgb_lcm_msg_);
    images.images.push_back(*depth_lcm_msg_);
    images.image_types.push_back(0); //$ LEFT = 0

    if (!compress_depth_)
    {
      images.image_types.push_back(4); //$ DEPTH_MM = 4
    }
    else
    {
      images.image_types.push_back(6); //$ DEPTH_MM_ZIPPED = 6
      // z depth, values similar to the OpenNI format, zipped with zlib
    }
    lcm_.publish(lcm_channel_, &images);

    i_++;
    ROS_INFO("Frame %d", i_);

  }
  /**
   * Cloud callback method.
   * @param cloud     input organized color point cloud
   */
   void cloudCallbackLCM(const PointCloud::ConstPtr& cloud)
   {

    std_msgs::Header cloud_header;
    pcl_conversions::fromPCL(cloud->header, cloud_header);

    if (debug_print_statements_)
      ROS_INFO("Point cloud acquired at %lu", (long unsigned int) cloud->header.stamp);

    // printf ("Cloud: width = %d, height = %d\n", cloud->width, cloud->height);

    cv::Mat rgb, depth;

    if (!cloudToImageAndDepthMat(cloud, rgb, depth))
    {
      //$ return from callback if conversion to images fails
      return;
    }
      
    //$ convert channel order
    cv::cvtColor(rgb, rgb, CV_RGB2BGR);

    if (debug_print_statements_)
      ROS_INFO("conversion to rgb and depth cv::Mat done");

    if (enforce_resize_) {
      cv::Mat resized_rgb;
      cv::Mat resized_depth;

      cv::resize(rgb, resized_rgb, cv::Size(resize_width_, resize_height_));
      cv::resize(depth, resized_depth, cv::Size(resize_width_, resize_height_));

      publishLCM((long unsigned int) cloud->header.stamp, resized_rgb, resized_depth);
    } else {
      publishLCM((long unsigned int) cloud->header.stamp, rgb, depth);
    }

  }


  /**
   * Synchronized RGB and depth image callback method.
   * @param rgb_msg     message with RGB image
   * @param depth_msg   message depth image
   */
   void syncCallbackLCM(const sensor_msgs::ImageConstPtr rgb_msg, const sensor_msgs::ImageConstPtr depth_msg)
   {
    // Converting to OpenCV Mat
    cv::Mat rgb = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding)->image;

    ROS_INFO("RGB encoding: %s", rgb_msg->encoding.c_str());


    ROS_INFO("Depth encoding: %s", depth_msg->encoding.c_str());

    cv::Mat depth = cv_bridge::toCvShare(depth_msg, depth_msg->encoding)->image;

    if (debug_print_statements_)
      ROS_INFO("Stamp %lu", rgb_msg->header.stamp.toNSec());

    long unsigned int timestamp = (long unsigned int) (rgb_msg->header.stamp.toNSec() / 1e3);

    if (debug_print_statements_)
      ROS_INFO("Images acquired at %lu", timestamp);

    //$ check RGB encoding
    if (rgb_msg->encoding == sensor_msgs::image_encodings::RGB8)
    {
      if (debug_print_statements_)
        ROS_INFO("Encoding is rgb8");
    }
    else if (rgb_msg->encoding == sensor_msgs::image_encodings::BGR8)
    {
      if (debug_print_statements_)
        ROS_WARN("Encoding is bgr8, converting channel oder");
      
      //$ convert channel order
      cv::cvtColor(rgb, rgb, CV_RGB2BGR);
      }
    else
    {
      ROS_ERROR("Unexpected image encoding %s in input RGB image, only bgr8 and rgb8 encodings are supported.", rgb_msg->encoding.c_str());
      ros::shutdown();
    }

    //$ check depth encoding
    if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
    {
      if (debug_print_statements_)
        ROS_INFO("Depth image format is 16UC1");
    }
    else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
    {
      if (debug_print_statements_)
        ROS_INFO("Converting depth from 32FC1 to 16UC1");

      //$ convert to 16UC1
      depth.convertTo(depth, CV_16UC1, 1000.0); 
    }
    else
    {
      ROS_ERROR("Unexpected image encoding %s in input depth image, only 16UC1 and 32FC1 encodings are supported.", depth_msg->encoding.c_str());
      ros::shutdown();
    }

    if (enforce_resize_) {
      cv::Mat resized_rgb;
      cv::Mat resized_depth;

      cv::resize(rgb, resized_rgb, cv::Size(resize_width_, resize_height_));
      cv::resize(depth, resized_depth, cv::Size(resize_width_, resize_height_));

      publishLCM(timestamp, resized_rgb, resized_depth);
    } else {
      publishLCM(timestamp, rgb, depth);
    }
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lcm_republisher");
  LCMRepublisher republisher;
  ros::spin();
}
