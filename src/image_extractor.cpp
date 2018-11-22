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

    unsigned short * depth;
    unsigned char * rgb;

    int width;
    int height;
    int num_pixels_;

    unsigned char * depth_decompress_buf_;
    unsigned char * image_decompress_buf_;
    int32_t depth_size_;
    int32_t image_size_;

public:
  ImageExtractor() : depth(0),
                     rgb(0),
                     depth_decompress_buf_(0),
                     image_decompress_buf_(0)
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
            bot_core::images_t message;
            message.decode(event->data, 0, event->datalen);
            if (i == 0) {
                initializeDimensions(message);
            }
            decompressAndSaveImages(message);
            i++;
        }
    }

    ROS_INFO("Found %d events on %s channel", i, lcm_channel_.c_str()); 

  }

  ~ImageExtractor()
  {
  }

  void initializeDimensions(bot_core::images_t message)
  {
    bot_core::image_t color_image = message.images[0];
    num_pixels_ = color_image.width * color_image.height;
    depth_decompress_buf_ = new unsigned char[num_pixels_ * 2];
    image_decompress_buf_ = new unsigned char[num_pixels_ * 3];
  }

  void decompressAndSaveImages(bot_core::images_t message)
  {
    int64_t timestamp = message.utime;
    std::cout << "timestamp: " << timestamp << std::endl;

    bot_core::image_t color_image = message.images[0];
    bot_core::image_t depth_image = message.images[1];


    bool zlib_compressed = false;

    if (depth_image.pixelformat == bot_core::image_t::PIXEL_FORMAT_INVALID)
    {
      zlib_compressed = true;
    }

    depth_size_ = depth_image.size;
    image_size_ = color_image.size;

    if(depth_size_ == num_pixels_ * 2)
    {
       // printf("copying depth image\n");
        memcpy(&depth_decompress_buf_[0], depth_image.data.data(), num_pixels_ * 2);
    }
    else
    {
      // printf("uncompress depth image\n");
        unsigned long decomp_length = num_pixels_ * 2;
        uncompress(&depth_decompress_buf_[0], (unsigned long *)&decomp_length, (const Bytef *)depth_image.data.data(), depth_size_);
    }

    if(image_size_ == num_pixels_ * 3)
    {
      //  printf("copy color image\n");
        memcpy(&image_decompress_buf_[0], color_image.data.data(), num_pixels_ * 3);
    }
    else if(image_size_ > 0)
    {
      //  printf("jpeg read color image\n");
        std::cout << "before jpeg read data" << std::endl;
        std::cout << "image_size_: " << image_size_ << std::endl;
        size_t w = color_image.width;
        size_t h = color_image.height;
        jpegijg_decompress_8u_rgb(color_image.data.data(), color_image.size, (unsigned char *)&image_decompress_buf_[0], w, h, w*3);

        std::cout << "after jpeg read data" << std::endl;
    }
    else
    {
        memset(&image_decompress_buf_[0], 0, num_pixels_ * 3);
    }

    depth = (unsigned short *)depth_decompress_buf_;
    rgb = (unsigned char *)&image_decompress_buf_[0];


  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_extractor");
  ImageExtractor extractor;
}
