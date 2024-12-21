#include <algorithm>
#include <librealsense2/rs.hpp>
#include <map>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stdexcept>
#include <vector>

#define STREAM RS2_STREAM_COLOR
#define FORMAT RS2_FORMAT_RGB8
#define WIDTH 424
#define HEIGHT 240
#define FPS 90
#define STREAM_INDEX 0

class RealsensePublisher {
public:
  RealsensePublisher(
      ros::NodeHandle &nh,
      std::vector<std::string> camera_serials = std::vector<std::string>())
      : nh_(nh) {
    ctx_ = std::make_shared<rs2::context>();

    std::vector<rs2::device> devices = ctx_->query_devices();
    std::vector<std::pair<rs2::device, std::string>> sorted_devices;

    for (const auto &device : devices) {
      std::string serial = device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      sorted_devices.push_back({device, serial});
    }

    std::sort(sorted_devices.begin(), sorted_devices.end(),
              [](const auto &a, const auto &b) { return a.second < b.second; });

    std::vector<std::string> found_serials;
    for (const auto &device_serial : sorted_devices) {
      found_serials.push_back(device_serial.second);
    }

    std::vector<std::string> selected_serials;
    if (camera_serials.empty()) {
      selected_serials = found_serials;
    } else {
      for (const auto &serial : camera_serials) {
        if (std::find(found_serials.begin(), found_serials.end(), serial) ==
            found_serials.end()) {
          throw std::runtime_error("Camera with serial number " + serial +
                                   " not found");
        }
        selected_serials.push_back(serial);
      }
    }

    for (size_t i = 0; i < sorted_devices.size(); ++i) {
      const auto &device_serial = sorted_devices[i];
      const auto &serial = device_serial.second;
      if (std::find(selected_serials.begin(), selected_serials.end(), serial) ==
          selected_serials.end()) {
        continue;
      }

      ROS_INFO("Using device with serial number: %s", serial.c_str());

      publishers_[serial] = nh_.advertise<sensor_msgs::Image>(
          "color_image_" + std::to_string(publishers_.size()), 10);

      rs2::pipeline pipe(*ctx_);
      rs2::config cfg;
      cfg.enable_device(serial);
      cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_RGB8, FPS);

      pipe.start(cfg);
      pipelines_.push_back(pipe);
    }

    if (pipelines_.empty()) {
      throw std::runtime_error("No cameras were initialized");
    }

    timer_ = nh_.createTimer(ros::Duration(0.033),
                             &RealsensePublisher::timerCallback, this);
  }

private:
  void timerCallback(const ros::TimerEvent &) {
    for (size_t camera_index = 0; camera_index < pipelines_.size();
         ++camera_index) {
      rs2::frameset frames;
      if (pipelines_[camera_index].poll_for_frames(&frames)) {
        auto color_frame = frames.get_color_frame();
        if (color_frame) {
          sensor_msgs::Image msg;
          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = "camera_frame_" + std::to_string(camera_index);
          msg.height = color_frame.get_height();
          msg.width = color_frame.get_width();
          msg.encoding = "rgb8";
          msg.is_bigendian = false;
          msg.step = color_frame.get_width() * 3;
          msg.data.assign(
              (uint8_t *)color_frame.get_data(),
              (uint8_t *)color_frame.get_data() +
                  (color_frame.get_height() * color_frame.get_width() * 3));

          std::string serial = pipelines_[camera_index]
                                   .get_active_profile()
                                   .get_device()
                                   .get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
          publishers_[serial].publish(msg);
        }
      }
    }
  }

  ros::NodeHandle &nh_;
  std::shared_ptr<rs2::context> ctx_;
  std::vector<rs2::pipeline> pipelines_;
  std::map<std::string, ros::Publisher> publishers_;
  ros::Timer timer_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "realsense_publisher");
  ros::NodeHandle nh;

  std::vector<std::string> camera_serials;
  for (int i = 1; i < argc; ++i) {
    camera_serials.push_back(argv[i]);
  }

  try {
    RealsensePublisher publisher(nh, camera_serials);
    ros::spin();
  } catch (const std::exception &e) {
    ROS_ERROR("Error: %s", e.what());
    return 1;
  }

  return 0;
}
