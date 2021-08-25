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



using namespace std::chrono_literals;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

#define FRAME_RATE 30.0

namespace camera {

enum CameraFrame {left_frame, right_frame};

struct FrameData {
  CameraFrame camera_frame;
  cv::Size pattersize;
  sensor_msgs::msg::Image::ConstSharedPtr img_msg;
  cv::Mat img;
  bool pattern_found;
  vector<cv::Point2f> corners;
};

class CalibStereoImageCap: public rclcpp::Node
{
public:
  CAMERA_PUBLIC
  explicit CalibStereoImageCap(const rclcpp::NodeOptions &options)
  : Node("calib_stereo_image_cap", rclcpp::NodeOptions(options).use_intra_process_comms(true))
  {
    auto qos = rclcpp::SensorDataQoS();
    qos.reliable();

    image_path_ = declare_parameter<std::string>("image_path","/tmp");
    size_ = declare_parameter<std::string>("size","10x7");

    string size_str(size_);
    char *token1 = strtok(const_cast<char *>(size_str.c_str()),"x");
    columns_ = atoi(token1);
    rows_ = atoi(strtok(NULL,"x"));

    RCLCPP_INFO(get_logger(),"starting - image_path: %s size: %s (columns = %d, rows = %d)",
      image_path_.c_str(), size_.c_str(), columns_, rows_);

    match_img_n_ = 0;

    capture_stereo_timer_ = create_wall_timer(100ms, std::bind(&CalibStereoImageCap::capture_stereo_callback, this));

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
      // RCLCPP_INFO_ONCE(get_logger(),"img_msg - height: %d width: %d step: %d", img_msg->height, img_msg->width, img_msg->step);

      if (raw_type == CV_16UC1) {

        bayer_u8_data.resize(img_msg->height * img_msg->width);
        auto bayer_16uc1 = cv::Mat(img_msg->height, img_msg->width, raw_type, img_buf_u8_p, img_msg->step);
        bayer_16uc1.forEach<uint16_t>(
          [&bayer_u8_data,img_msg](uint16_t &v, const int *position){

            auto bit_extract = [] (uint16_t value, int begin, int end) {
                uint16_t mask = (1 << (end - begin)) - 1;
                return (value >> begin) & mask;
            };

            bayer_u8_data[(position[0]*img_msg->width) +position[1]]=bit_extract(v, 2, 10) ;
          }
        );

        raw = cv::Mat(img_msg->height, img_msg->width, CV_8UC1, &bayer_u8_data.data()[0], img_msg->step/2);

      } else {
        raw = cv::Mat(img_msg->height, img_msg->width, raw_type,
                      img_buf_u8_p, img_msg->step);
      }

      // RCLCPP_INFO(get_logger(),"%s cv::Mat raw - type(): %d total(): %ld", window_name.c_str(), raw.type(), raw.total());

      std::ostringstream stampTimeStream;
      // stampTimeStream << img_msg->header.stamp.sec << "." << std::showpoint << std::fixed << std::setprecision(9) << img_msg->header.stamp.nanosec/1e-9;
      stampTimeStream << img_msg->header.stamp.sec<<"."<< std::setfill('0') <<std::setw(9) << img_msg->header.stamp.nanosec;
      cv::Mat img(img_msg->height, img_msg->width, CV_8UC3);
      cv::Mat gray(img_msg->height, img_msg->width, CV_8UC1);
      // RCLCPP_INFO(get_logger(),"%s cv::Mat img - type(): %d total(): %ld", window_name.c_str(), img.type(), img.total());
      cv::cvtColor(raw, img, img_code);
      cv::cvtColor(raw, gray, gray_code);
      if (img.type() == CV_16UC3) {
        img.convertTo(img, CV_8UC3, 0.0625);
      }

      // RCLCPP_INFO(get_logger(),"%s cv::Mat img - type(): %d total(): %ld", window_name.c_str(), img.type(), img.total());

      // detect chessboard
      cv::Size patternsize(columns_, rows_);
      vector<cv::Point2f> corners;
      int flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK;
      bool pattern_found = cv::findChessboardCorners(gray, patternsize, corners, flags);

      if (pattern_found) {
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

        // add to frame_queue to process later
        FrameData frame_data = {camera_frame, patternsize, img_msg, img.clone(), pattern_found, corners};
        frame_queue_.push_back(std::make_shared<FrameData>(frame_data));
      }

      cv::drawChessboardCorners(img, patternsize, cv::Mat(corners), pattern_found);

      // display results

      cv::Mat img_d;
      cv::Mat gray_d;

      cv::resize(img, img_d, cv::Size(img_msg->width/2, img_msg->height/2), cv::INTER_LINEAR);
      cv::resize(gray, gray_d, cv::Size(img_msg->width/2, img_msg->height/2), cv::INTER_LINEAR);

      cv::putText(img_d, img_msg->encoding, cv::Point(10,30), cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255.0,0.0,0.0));
      cv::putText(img_d, stampTimeStream.str(), cv::Point(10,60), cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255.0,0.0,0.0));
      cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
      cv::Mat disp_img;
      cv::Mat gray_c3;
      cv::cvtColor(gray_d, gray_c3, cv::COLOR_GRAY2BGR, 3);
      // RCLCPP_INFO(get_logger(), " img_d.type(): %d img_d.dims: %d img_d.rows: %d gray_c3.type(): %d  gray_c3.dims: %d gray_c3.rows: %d",
      //         img_d.type(), img_d.dims, img_d.rows,
      //         gray_c3.type(), gray_c3.dims, gray_c3.rows);

      cv::vconcat(img_d, gray_c3, disp_img);
      cv::imshow(window_name, disp_img);
      // auto window_name_gray = window_name+" gray";
      // cv::namedWindow(window_name_gray, cv::WINDOW_AUTOSIZE);
      // cv::imshow(window_name_gray, gray_d);

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
  ~CalibStereoImageCap() {
    RCLCPP_INFO(this->get_logger(),"finished");
  }
private:

  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr image_0_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr image_1_sub_;

  rclcpp::TimerBase::SharedPtr capture_stereo_timer_;

  std::string image_path_;
  std::string size_;
  uint8_t rows_;
  uint8_t columns_;

  std::deque<std::shared_ptr<camera::FrameData>> frame_queue_;

  std::shared_ptr<camera::FrameData> left_frame_, left_frame_prev_;
  std::shared_ptr<camera::FrameData> right_frame_, right_frame_prev_;

  u_int32_t match_img_n_;

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
        continue; // we need a left and right frame
      }
      RCLCPP_INFO_ONCE(get_logger(), "have first left frame and right frame ..");

      auto left_stamp = left_frame_->img_msg->header.stamp;
      auto right_stamp = right_frame_->img_msg->header.stamp;

      auto stamp_diff = rclcpp::Time(left_stamp)-rclcpp::Time(right_stamp);
      if (stamp_diff > 10ms) {
        // if time stamps arent close - lets wait till they are
        continue;
      }

      // initialise the prev frames if the
      if (left_frame_prev_.use_count() == 0)
        left_frame_prev_ = left_frame_;
      if (right_frame_prev_.use_count() == 0)
        right_frame_prev_ = right_frame_;

      // if we've found a match previously and these chessboards are the same just continue
      if ((corners_equal(left_frame_prev_->corners, left_frame_->corners)
       || corners_equal(right_frame_prev_->corners, right_frame_->corners))
       && match_img_n_ > 0) {
         continue;
      }

      cv::Size size (left_frame_->img_msg->width, left_frame_->img_msg->height); // right is the same

      cv::Mat img_lc(left_frame_->img.clone());
      cv::drawChessboardCorners(img_lc, left_frame_->pattersize, cv::Mat(left_frame_->corners), left_frame_->pattern_found);
      cv::Mat img_rc(right_frame_->img.clone());
      cv::drawChessboardCorners(img_rc, right_frame_->pattersize, cv::Mat(right_frame_->corners), right_frame_->pattern_found);

      cv::Mat img_l, img_r, img_d;

      cv::resize(img_lc, img_l, cv::Size(size.width/2, size.height/2), cv::INTER_LINEAR);
      cv::putText(img_l, to_string(match_img_n_), cv::Point(10,30), cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255.0,0.0,0.0));
      cv::resize(img_rc, img_r, cv::Size(size.width/2, size.height/2), cv::INTER_LINEAR);
      cv::putText(img_r, to_string(match_img_n_), cv::Point(10,30), cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255.0,0.0,0.0));
      cv::hconcat(img_l, img_r, img_d);
      auto window_name="stereo images that have chessboards";
      cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
      cv::imshow(window_name, img_d);
      cv::waitKey(1000/FRAME_RATE);

      std::vector<int> write_params;
      write_params.push_back(cv::IMWRITE_WEBP_QUALITY);
      write_params.push_back(100);

      std::stringstream ss;
      ss << std::setw(4) << std::setfill('0') << match_img_n_;
      std::string left_file = image_path_+"/left-"+ss.str()+".webp";
      cv::imwrite(left_file, left_frame_->img, write_params);
      std::string right_file = image_path_+"/right-"+ss.str()+".webp";
      cv::imwrite(right_file, right_frame_->img, write_params);

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

  CAMERA_LOCAL
  bool corners_equal (std::vector<cv::Point2f> a, std::vector<cv::Point2f> b) {
    if (a.size() != b.size()){
      return false;
    }

    for (int i = 0; i < (int)a.size(); i++){
      double res = cv::norm(a[i]-b[i]);
      // RCLCPP_INFO(get_logger(),"i: %d a: %f,%f b: %f,%f res: %f",i, a[i].x, a[i].y, b[i].x, b[i].y, res);
      // if a pixel is within 5 units distance from previous then same they are the same
      if (res > 5.00) {
        return false;
      }
    }
    return true;
  }
};

} // end namespace camera

RCLCPP_COMPONENTS_REGISTER_NODE(camera::CalibStereoImageCap)
