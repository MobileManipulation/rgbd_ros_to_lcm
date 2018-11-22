#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>

#include <rgbd_ros_to_lcm/jpeg_utils.h>

#include <zlib.h>

#include <lcm/lcm-cpp.hpp>
#include <bot_core/image_t.hpp>
#include <bot_core/images_t.hpp>


class ImageExtractor
{
  std::string lcm_logfile_path_;
  std::string lcm_channel_;

public:
  ImageExtractor()
  {
    ros::NodeHandle private_nh("~");

    private_nh.param<std::string>("lcm_logfile", lcm_logfile_path_, "");
    private_nh.param<std::string>("lcm_channel", lcm_channel_, "OPENNI_FRAME");

    lcm::LogFile* logfile = new lcm::LogFile(lcm_logfile_path_, "r");
    ROS_INFO("Reading logfile from %s", lcm_logfile_path_.c_str());

    const lcm::LogEvent* event = nullptr;
    int i = 0;
    while (true)
    {
        event = logfile->readNextEvent();
        if (event == nullptr) {
            //$ reached the end of the log
            break;
        }
        if (event->channel == lcm_channel_) {
            //$ only consider events that are on the desired channel
            i++;
        }
    }

    ROS_INFO("Found %d events on %s channel", i, lcm_channel_.c_str()); 

  }

  ~ImageExtractor()
  {
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_extractor");
  ImageExtractor extractor;
}
