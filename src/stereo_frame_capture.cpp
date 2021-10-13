#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
#include <deque>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "galaxy_camera_u3v/visibility_control.h"

// Include opencv2
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco/charuco.hpp>


using namespace std::chrono_literals;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

#define FRAME_RATE 30.0

namespace stereo_capture {

enum CameraFrame {left_frame, right_frame};

struct FrameCaptureData {
  CameraFrame camera_frame;
  cv::Mat img;
  sensor_msgs::msg::Image::ConstSharedPtr img_msg;
};

class StereoFrameCap: public rclcpp::Node
{
public:
  CAMERA_PUBLIC
  explicit StereoFrameCap(const rclcpp::NodeOptions &options)
  : Node("stereo_frame_cap", rclcpp::NodeOptions(options).use_intra_process_comms(true))
  {
    auto qos = rclcpp::SensorDataQoS();
    qos.reliable();

    image_path_ = declare_parameter<std::string>("image_path","/tmp");


    RCLCPP_INFO(get_logger(),"starting - image_path: %s", image_path_.c_str());

    match_img_n_ = 0;

    // webp parameters for imwrite
    webp_write_params_.push_back(cv::IMWRITE_WEBP_QUALITY);
    webp_write_params_.push_back(100);

    // setup run timestamp string
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    std::time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", timeinfo);
    run_ts_ = std::string(buffer);

    capture_stereo_timer_ = create_wall_timer(33ms, std::bind(&StereoFrameCap::capture_stereo_callback, this));

    auto img_viz_cap = [this](sensor_msgs::msg::Image::ConstSharedPtr img_msg, std::string window_name, CameraFrame camera_frame){
      const string& raw_encoding = img_msg->encoding;
      if (raw_encoding != enc::BAYER_RGGB8
        && raw_encoding != enc::BAYER_RGGB16
        && raw_encoding != enc::RGB8
        ) {
        RCLCPP_ERROR(this->get_logger(),"need BAYER_RGGB8|16 or RGB8 encoding .. exiting");
        exit(-1);
      }
      int raw_type;
      int img_code;
      int gray_code;
      uint8_t *img_buf_u8_p = const_cast<uint8_t*>(&img_msg->data[0]);
      auto bayer_u8_data = std::vector<uint8_t>();
      cv::Mat raw;
      if (raw_encoding == enc::RGB8) {
        raw_type = CV_8UC3;
        img_code = cv::COLOR_RGB2BGR;
        gray_code = cv::COLOR_RGB2GRAY;
      } else if (raw_encoding == enc::BAYER_RGGB8) {
        raw_type = CV_8UC1;
        img_code = cv::COLOR_BayerBG2BGR;
        gray_code = cv::COLOR_BayerBG2GRAY;
      } else if (raw_encoding == enc::BAYER_RGGB16) {
        raw_type = CV_16UC1;
        img_code = cv::COLOR_BayerBG2BGR;
        gray_code = cv::COLOR_BayerBG2GRAY;
      }
      RCLCPP_INFO_ONCE(get_logger(),"img_msg - height: %d width: %d step: %d", img_msg->height, img_msg->width, img_msg->step);

      raw = cv::Mat(img_msg->height, img_msg->width, raw_type, img_buf_u8_p, img_msg->step);


      std::ostringstream stampTimeStream;
      // stampTimeStream << img_msg->header.stamp.sec << "." << std::showpoint << std::fixed << std::setprecision(9) << img_msg->header.stamp.nanosec/1e-9;
      stampTimeStream << img_msg->header.stamp.sec<<"."<< std::setfill('0') <<std::setw(9) << img_msg->header.stamp.nanosec;
      cv::Mat img(img_msg->height, img_msg->width, CV_8UC3);
      // RCLCPP_INFO(get_logger(),"%s cv::Mat img - type(): %d total(): %ld", window_name.c_str(), img.type(), img.total());
      cv::cvtColor(raw, img, img_code);
      if (img.type() == CV_16UC3) {
        cv::Mat img_scaled;
        cv::convertScaleAbs(img, img_scaled);
        img_scaled.copyTo(img);
      }

      FrameCaptureData frame_data = {camera_frame, img.clone(), img_msg};
      auto frame_data_ptr = std::make_shared<FrameCaptureData>(frame_data);
      frame_queue_.push_back(frame_data_ptr);
      RCLCPP_INFO_ONCE(get_logger(), "queuing first %s frame ...", window_name.c_str());

      cv::Mat img_d;

      cv::resize(img, img_d, cv::Size(img_msg->width/2, img_msg->height/2), cv::INTER_LINEAR);

      cv::putText(img_d, img_msg->encoding, cv::Point(10,30), cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255.0,0.0,0.0));
      cv::putText(img_d, stampTimeStream.str(), cv::Point(10,60), cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255.0,0.0,0.0));
      cv::namedWindow(window_name, cv::WINDOW_GUI_EXPANDED);
      cv::imshow(window_name, img_d);

      cv::waitKey(1000/FRAME_RATE);
    };


    string topic_0_param_name = "left_camera_topic";
    string topic_0_value = "/stereo/left/image_raw";
    topic_0_value = this->declare_parameter<string>(topic_0_param_name, topic_0_value);
    RCLCPP_INFO(this->get_logger(), "subscribing to topic: %s", topic_0_value.c_str());
    image_0_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_0_value, qos,
      [img_viz_cap](sensor_msgs::msg::Image::ConstSharedPtr img_msg){
        img_viz_cap(img_msg, "left camera", left_frame);
      }
    );

    string topic_1_param_name = "right_camera_topic";
    string topic_1_value = "/stereo/right/image_raw";
    topic_1_value = this->declare_parameter<string>(topic_1_param_name, topic_1_value);
    RCLCPP_INFO(this->get_logger(), "subscribing to topic: %s", topic_1_value.c_str());
    image_1_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_1_value, qos,
      [img_viz_cap](sensor_msgs::msg::Image::ConstSharedPtr img_msg){
        img_viz_cap(img_msg, "right camera", right_frame);
      }
    );

  }

  CAMERA_LOCAL
  ~StereoFrameCap() {
    RCLCPP_INFO(this->get_logger(),"finished");
  }
private:

  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr image_0_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr image_1_sub_;

  rclcpp::TimerBase::SharedPtr capture_stereo_timer_;

  std::string image_path_;
  std::string run_ts_;
  std::string size_;

  std::deque<std::shared_ptr<stereo_capture::FrameCaptureData>> frame_queue_;

  std::shared_ptr<stereo_capture::FrameCaptureData> left_frame_, left_frame_prev_;
  std::shared_ptr<stereo_capture::FrameCaptureData> right_frame_, right_frame_prev_;

  u_int32_t match_img_n_;

  std::vector<int> webp_write_params_;

  CAMERA_LOCAL
  void capture_stereo_callback () {

    // just return if nothing to do
    if (frame_queue_.size() == 0) {
      return;
    }

    while (frame_queue_.size() > 0) {
      auto frame = frame_queue_.front();
      switch (frame->camera_frame) {
        case left_frame:
          left_frame_ = frame;
          break;
        case right_frame:
          right_frame_ = frame;
      }
      frame_queue_.pop_front();

      if (left_frame_.use_count() ==0 || right_frame_.use_count() == 0) {
        RCLCPP_INFO(get_logger(), "waiting for stereo pair ...");
        continue; // we need a left and right frame
      }
      RCLCPP_INFO_ONCE(get_logger(), "have first left frame and right frame ..");

      auto left_time = rclcpp::Time(left_frame_->img_msg->header.stamp);
      auto right_time = rclcpp::Time(right_frame_->img_msg->header.stamp);

      auto stamp_diff = rclcpp::Time(left_time)-rclcpp::Time(right_time);
      // if (abs(stamp_diff.nanoseconds()) > 5000000) {
      if (abs(stamp_diff.nanoseconds()) > 60000) {
        // if time stamps arent close - lets wait till they are
        continue;
      }

      // initialise the prev frames if the
      if (left_frame_prev_.use_count() == 0)
        left_frame_prev_ = left_frame_;
      if (right_frame_prev_.use_count() == 0)
        right_frame_prev_ = right_frame_;

      RCLCPP_INFO(get_logger(),"stamp_diff(ns): %ld left_time: %f right_time: %f ", stamp_diff.nanoseconds(), left_time.seconds(), right_time.seconds());

      cv::Size size (left_frame_->img_msg->width, left_frame_->img_msg->height); // right is the same

      cv::Mat img_lc(left_frame_->img.clone());
      cv::Mat img_rc(right_frame_->img.clone());

      cv::Mat img_l, img_r, img_d;

      cv::resize(img_lc, img_l, cv::Size(size.width/2, size.height/2), cv::INTER_LINEAR);
      cv::putText(img_l, to_string(match_img_n_), cv::Point(10,30), cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255.0,0.0,0.0));
      cv::resize(img_rc, img_r, cv::Size(size.width/2, size.height/2), cv::INTER_LINEAR);
      cv::putText(img_r, to_string(match_img_n_), cv::Point(10,30), cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255.0,0.0,0.0));
      cv::hconcat(img_l, img_r, img_d);
      auto window_name="stereo images";
      cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
      cv::imshow(window_name, img_d);
      cv::waitKey(1000/FRAME_RATE);

      std::stringstream ss;
      ss << std::setw(4) << std::setfill('0') << match_img_n_;
      std::string left_file = image_path_+"/"+run_ts_+"-"+ss.str()+"-left.webp";
      cv::imwrite(left_file, left_frame_->img, webp_write_params_);
      std::string right_file = image_path_+"/"+run_ts_+"-"+ss.str()+"-right.webp";
      cv::imwrite(right_file, right_frame_->img, webp_write_params_);

      RCLCPP_INFO(get_logger(), "%s %s %s", ss.str().c_str(), left_file.c_str(), right_file.c_str());

      // save this frame as the previous
      switch (frame->camera_frame) {
        case left_frame:
          left_frame_prev_ = left_frame_;
          break;
        case right_frame:
          right_frame_prev_ = right_frame_;
      }
      match_img_n_++;
    }
  }
};
} // end namespace stereo_capture

RCLCPP_COMPONENTS_REGISTER_NODE(stereo_capture::StereoFrameCap)