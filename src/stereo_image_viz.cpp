#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "galaxy_camera_u3v/visibility_control.h"

// Include opencv2
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <iomanip>

using namespace std::chrono_literals;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

#define FRAME_RATE 30.0

namespace camera {
class StereoImageViz: public rclcpp::Node
{
public:
  CAMERA_PUBLIC
  explicit StereoImageViz(const rclcpp::NodeOptions &options)
  : Node("stereo_image_viz", rclcpp::NodeOptions(options).use_intra_process_comms(true))
  {
    auto qos = rclcpp::SensorDataQoS();
    qos.reliable();
    // auto qos = rclcpp::ServicesQoS();

    callback_group_0_subscribers_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_1_subscribers_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto img_viz = [this](sensor_msgs::msg::Image::ConstSharedPtr img_msg, std::string window_name){
      const string& raw_encoding = img_msg->encoding;
      if (raw_encoding != enc::BAYER_RGGB8
        && raw_encoding != enc::BAYER_RGGB16
        && raw_encoding != enc::RGB8
        ) {
        RCLCPP_ERROR(this->get_logger(),"need BAYER_RGGB8|16 or RGB8 encoding .. exiting");
        exit(-1);
      }
      int raw_type;
      int code;
      uint8_t *img_buf_u8_p = const_cast<uint8_t*>(&img_msg->data[0]);
      auto bayer_u8_data = std::vector<uint8_t>();
      cv::Mat raw;
      if (raw_encoding == enc::RGB8) {
        raw_type = CV_8UC3;
        code = cv::COLOR_RGB2BGR;
      } else if (raw_encoding == enc::BAYER_RGGB8) {
        raw_type = CV_8UC1;
        code = cv::COLOR_BayerBG2BGR;
      } else if (raw_encoding == enc::BAYER_RGGB16) {
        raw_type = CV_16UC1;
        code = cv::COLOR_BayerBG2BGR;
      }
      // RCLCPP_INFO_ONCE(get_logger(),"img_msg - height: %d width: %d step: %d", img_msg->height, img_msg->width, img_msg->step);

      if (raw_type == CV_16UC1) {

        // bayer_u8_data.resize(img_msg->height * img_msg->width);
        // auto bayer_16uc1 = cv::Mat(img_msg->height, img_msg->width, raw_type, img_buf_u8_p, img_msg->step);
        // bayer_16uc1.forEach<uint16_t>(
        //   [&bayer_u8_data,img_msg](uint16_t &v, const int *position){

        //     auto bit_extract = [] (uint16_t value, int begin, int end) {
        //         uint16_t mask = (1 << (end - begin)) - 1;
        //         return (value >> begin) & mask;
        //     };

        //     bayer_u8_data[(position[0]*img_msg->width) +position[1]]=bit_extract(v, 1, 9) ;
        //   }
        // );

        // // RCLCPP_INFO(get_logger(), "%s bayer_u8_data.size(): %ld", window_name.c_str(), bayer_u8_data.size());
        // raw = cv::Mat(img_msg->height, img_msg->width, CV_8UC1, &bayer_u8_data.data()[0], img_msg->step/2);
        raw = cv::Mat(img_msg->height, img_msg->width, raw_type, img_buf_u8_p, img_msg->step);
        // raw = cv::Mat(img_msg->height, img_msg->width, CV_8UC1);
        // bayer_16uc1.convertTo(raw, CV_8UC1, 0.0625);

      } else {
        raw = cv::Mat(img_msg->height, img_msg->width, raw_type,
                      img_buf_u8_p, img_msg->step);
      }

      // RCLCPP_INFO(get_logger(),"%s cv::Mat raw - type(): %d total(): %ld", window_name.c_str(), raw.type(), raw.total());

      std::ostringstream stampTimeStream;
      // stampTimeStream << img_msg->header.stamp.sec << "." << std::showpoint << std::fixed << std::setprecision(9) << img_msg->header.stamp.nanosec/1e-9;
      stampTimeStream << img_msg->header.stamp.sec<<"."<< std::setfill('0') <<std::setw(9) << img_msg->header.stamp.nanosec;
      cv::Mat img(img_msg->height, img_msg->width, CV_8UC3);
      // RCLCPP_INFO(get_logger(),"%s cv::Mat img - type(): %d total(): %ld", window_name.c_str(), img.type(), img.total());
      cv::cvtColor(raw, img, code);
      if (img.type() == CV_16UC3) {
        // img.convertTo(img, CV_8UC3, 0.0625);
        cv::Mat img_scaled;
        cv::convertScaleAbs(img, img_scaled);
        img_scaled.copyTo(img);
      }
      // RCLCPP_INFO(get_logger(),"%s cv::Mat img - type(): %d total(): %ld", window_name.c_str(), img.type(), img.total());

      cv::putText(img, img_msg->encoding, cv::Point(10,30), cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255.0,0.0,0.0));
      cv::putText(img, stampTimeStream.str(), cv::Point(10,60), cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255.0,0.0,0.0));
      cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
      cv::imshow(window_name, img);
      cv::waitKey(1000/FRAME_RATE);
    };

    auto subscriptions_0_opt = rclcpp::SubscriptionOptions();
    subscriptions_0_opt.callback_group = callback_group_0_subscribers_;
    auto subscriptions_1_opt = rclcpp::SubscriptionOptions();
    subscriptions_0_opt.callback_group = callback_group_1_subscribers_;

    string topic_0_param_name = "left_camera_topic";
    string topic_0_value = "/stereo/left/image_raw";
    topic_0_value = this->declare_parameter<string>(topic_0_param_name, topic_0_value);
    RCLCPP_INFO(this->get_logger(), "subscribing to topic: %s", topic_0_value.c_str());
    image_0_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_0_value, qos,
      [img_viz](sensor_msgs::msg::Image::ConstSharedPtr img_msg){
        img_viz(img_msg, "left camera");
      },
      subscriptions_0_opt
    );

    string topic_1_param_name = "right_camera_topic";
    string topic_1_value = "/stereo/right/image_raw";
    topic_1_value = this->declare_parameter<string>(topic_1_param_name, topic_1_value);
    RCLCPP_INFO(this->get_logger(), "subscribing to topic: %s", topic_1_value.c_str());
    image_1_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_1_value, qos,
      [img_viz](sensor_msgs::msg::Image::ConstSharedPtr img_msg){
        img_viz(img_msg, "right camera");
      },
      subscriptions_1_opt
    );

  }

  CAMERA_LOCAL
  ~StereoImageViz() {
    RCLCPP_INFO(this->get_logger(),"finished");
  }
private:
  rclcpp::CallbackGroup::SharedPtr callback_group_0_subscribers_;
  rclcpp::CallbackGroup::SharedPtr callback_group_1_subscribers_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr image_0_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr image_1_sub_;
};
} // end namespace car_captu

RCLCPP_COMPONENTS_REGISTER_NODE(camera::StereoImageViz)
