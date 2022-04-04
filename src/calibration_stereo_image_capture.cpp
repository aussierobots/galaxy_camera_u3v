#include <chrono>
#include <iostream>
#include <iomanip>
#include <string>
// #include <deque>
#include <map>
#include <fstream>
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

namespace camera {

std::string string_thread_id()
{
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

enum CameraFrame {left_frame, right_frame};
enum Calibration {chessboard, ChArUco};

struct FrameData {
  Calibration calibration;
  CameraFrame camera_frame;
  cv::Size pattersize;
  sensor_msgs::msg::Image::ConstSharedPtr img_msg;
  cv::Mat img;
  bool pattern_found;
  vector<cv::Point2f> corners;
  vector<vector<cv::Point2f>> marker_corners;
  vector<int> marker_ids;
  vector<int> charuco_ids;
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

    logfile_.open("/tmp/calibration_stereo_frame_capture.log" , std::ofstream::out | std::ios_base::trunc);

    size_ = declare_parameter<std::string>("size","11x7");
    string size_str(size_);
    char *token1 = strtok(const_cast<char *>(size_str.c_str()),"x");
    columns_ = atoi(token1);
    rows_ = atoi(strtok(NULL,"x"));

    square_length_ = declare_parameter<float>("square_length", .65f);
    marker_length_ = declare_parameter<float>("marker_length", .51f);

    auto calibration_p = declare_parameter<std::string>("calibration", "chessboard");
    if (calibration_p.compare("chessboard") == 0) {
      calibration_ = Calibration::chessboard;
      RCLCPP_INFO(get_logger(), "chessboard calibration detection");
    } else if (calibration_p.compare("ChArUco") == 0) {
      calibration_ = Calibration::ChArUco;
      RCLCPP_INFO(get_logger(), "ChArUco calibration detection");
       cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_100);
       board_ = cv::aruco::CharucoBoard::create(rows_, columns_,square_length_, marker_length_, dictionary);
       aruco_params_ = cv::aruco::DetectorParameters::create();
       aruco_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    } else {
      RCLCPP_ERROR(get_logger(), "unknown calibration parameter: %s exiting...", calibration_p.c_str());
      exit(-1);
    }

    RCLCPP_INFO(get_logger(),"starting - image_path: %s size: %s (columns = %d, rows = %d)",
      image_path_.c_str(), size_.c_str(), columns_, rows_);

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


    callback_group_0_subscribers_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_1_subscribers_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_group_writers_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    capture_stereo_timer_ = create_wall_timer(100ms, std::bind(&CalibStereoImageCap::capture_stereo_callback, this), callback_group_writers_);

    auto img_viz_cap = [this](sensor_msgs::msg::Image::ConstSharedPtr img_msg, std::string window_name, CameraFrame camera_frame){
      RCLCPP_INFO_ONCE(get_logger(), "starting %s calibration image capture - thread %s", window_name.c_str(), string_thread_id().c_str());
      const string& raw_encoding = img_msg->encoding;
      if (raw_encoding != enc::BAYER_RGGB8
        && raw_encoding != enc::BAYER_RGGB16
        && raw_encoding != enc::RGB8
        ) {
        RCLCPP_ERROR(get_logger(),"need BAYER_RGGB8|16 or RGB8 encoding .. exiting");
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

        // bayer_u8_data.resize(img_msg->height * img_msg->width);
        // auto bayer_16uc1 = cv::Mat(img_msg->height, img_msg->width, raw_type, img_buf_u8_p, img_msg->step);
        // bayer_16uc1.forEach<uint16_t>(
        //   [&bayer_u8_data,img_msg](uint16_t &v, const int *position){

        //     auto bit_extract = [] (uint16_t value, int begin, int end) {
        //         uint16_t mask = (1 << (end - begin)) - 1;
        //         return (value >> begin) & mask;
        //     };

        //     bayer_u8_data[(position[0]*img_msg->width) +position[1]]=bit_extract(v, 2, 10) ;
        //   }
        // );

        // raw = cv::Mat(img_msg->height, img_msg->width, CV_8UC1, &bayer_u8_data.data()[0], img_msg->step/2);
        raw = cv::Mat(img_msg->height, img_msg->width, raw_type, img_buf_u8_p, img_msg->step);
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
      if (img.type() == CV_16UC3) {
        cv::Mat img_scaled;
        cv::convertScaleAbs(img, img_scaled);
        img_scaled.copyTo(img);
      }
      cv::cvtColor(raw, gray, gray_code);
      if (gray.type() == CV_16UC1) {
        cv::Mat gray_scaled;
        cv::convertScaleAbs(gray, gray_scaled);
        gray_scaled.copyTo(gray);
      }


      // RCLCPP_INFO(get_logger(),"%s cv::Mat img - type(): %d total(): %ld", window_name.c_str(), img.type(), img.total());
      vector<cv::Point2f> corners;
      vector<vector<cv::Point2f>> marker_corners;
      vector<int> marker_ids;
      vector<int> charuco_ids;
      cv::Size patternsize(columns_, rows_);

      if (calibration_ == Calibration::chessboard) {
        // detect chessboard
        int flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK;
        bool pattern_found = cv::findChessboardCorners(gray, patternsize, corners, flags);

        if (pattern_found) {
          cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
              cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

          // add to frame_queue to process later
          FrameData frame_data = {calibration_, camera_frame, patternsize, img_msg, img.clone(), pattern_found, corners, marker_corners, marker_ids, charuco_ids};
          auto frame_data_ptr = std::make_shared<FrameData>(frame_data);
          long long frame_key = (long long)((long long)img_msg->header.stamp.sec*(long long)1000000000) + img_msg->header.stamp.nanosec;
          RCLCPP_DEBUG(get_logger(),"stamp.sec: %d stamp.nanosec: %09d frame_key: %lld camera: %d", img_msg->header.stamp.sec, img_msg->header.stamp.nanosec, frame_key, camera_frame);
          frame_queue_.insert(pair <long long, std::shared_ptr<FrameData>> (frame_key, frame_data_ptr));
          // frame_queue_.push_back(std::make_shared<FrameData>(frame_data));
        }

        cv::drawChessboardCorners(gray, patternsize, cv::Mat(corners), pattern_found);

      } else if (calibration_ == Calibration::ChArUco) {
        cv::aruco::detectMarkers(img, board_->dictionary, marker_corners, marker_ids, aruco_params_);

        if (marker_ids.size() > 0 ) {
          bool pattern_found = true;
          cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, gray, board_, corners, charuco_ids);
          // add to frame_queue to process later
          FrameData frame_data = {calibration_, camera_frame, patternsize, img_msg, img.clone(), pattern_found, corners, marker_corners, marker_ids, charuco_ids};
          auto frame_data_ptr = std::make_shared<FrameData>(frame_data);
          long long frame_key = (long long)((long long)img_msg->header.stamp.sec*(long long)1000000000) + img_msg->header.stamp.nanosec;
          RCLCPP_DEBUG(get_logger(),"stamp.sec: %d stamp.nanosec: %09d frame_key: %lld camera: %d", img_msg->header.stamp.sec, img_msg->header.stamp.nanosec, frame_key, camera_frame);
          frame_queue_.insert(pair <long long, std::shared_ptr<FrameData>> (frame_key, frame_data_ptr));
          // frame_queue_.push_back(std::make_shared<FrameData>(frame_data));

          cv::aruco::drawDetectedMarkers(img, marker_corners, marker_ids);
          // if at least one charuco corner detected
          if (charuco_ids.size() > 0)
              cv::aruco::drawDetectedCornersCharuco(img, corners, charuco_ids, cv::Scalar(255, 0, 0));
        }
      }

      // display results

      cv::Mat img_d;
      // cv::Mat gray_d;

      cv::resize(img, img_d, cv::Size(img_msg->width/2, img_msg->height/2), cv::INTER_LINEAR);
      // cv::resize(gray, gray_d, cv::Size(img_msg->width/2, img_msg->height/2), cv::INTER_LINEAR);

      cv::putText(img_d, img_msg->encoding, cv::Point(10,30), cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255.0,0.0,0.0));
      cv::putText(img_d, stampTimeStream.str(), cv::Point(10,60), cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(255.0,0.0,0.0));
      cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
      cv::Mat disp_img;
      // cv::Mat gray_c3;
      // cv::cvtColor(gray_d, gray_c3, cv::COLOR_GRAY2BGR, 3);
      // RCLCPP_INFO(get_logger(), " img_d.type(): %d img_d.dims: %d img_d.rows: %d gray_c3.type(): %d  gray_c3.dims: %d gray_c3.rows: %d",
      //         img_d.type(), img_d.dims, img_d.rows,
      //         gray_c3.type(), gray_c3.dims, gray_c3.rows);

      // cv::vconcat(img_d, gray_c3, disp_img);
      // cv::imshow(window_name, disp_img);
      cv::imshow(window_name, img_d);
      // auto window_name_gray = window_name+" gray";
      // cv::namedWindow(window_name_gray, cv::WINDOW_AUTOSIZE);
      // cv::imshow(window_name_gray, gray_d);

      cv::waitKey(1000/FRAME_RATE);
    };

    auto subscriptions_0_opt = rclcpp::SubscriptionOptions();
    subscriptions_0_opt.callback_group = callback_group_0_subscribers_;
    auto subscriptions_1_opt = rclcpp::SubscriptionOptions();
    subscriptions_1_opt.callback_group = callback_group_1_subscribers_;

    string topic_0_param_name = "left_camera_topic";
    string topic_0_value = "/stereo/left/image_raw";
    topic_0_value = this->declare_parameter<string>(topic_0_param_name, topic_0_value);
    RCLCPP_INFO(this->get_logger(), "subscribing to topic: %s", topic_0_value.c_str());
    image_0_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_0_value, qos,
      [img_viz_cap](sensor_msgs::msg::Image::ConstSharedPtr img_msg){
        img_viz_cap(img_msg, "left camera", left_frame);
      },
      subscriptions_0_opt
    );

    string topic_1_param_name = "right_camera_topic";
    string topic_1_value = "/stereo/right/image_raw";
    topic_1_value = this->declare_parameter<string>(topic_1_param_name, topic_1_value);
    RCLCPP_INFO(this->get_logger(), "subscribing to topic: %s", topic_1_value.c_str());
    image_1_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_1_value, qos,
      [img_viz_cap](sensor_msgs::msg::Image::ConstSharedPtr img_msg){
        img_viz_cap(img_msg, "right camera", right_frame);
      },
      subscriptions_1_opt
    );

  }

  CAMERA_LOCAL
  ~CalibStereoImageCap() {
    RCLCPP_INFO(this->get_logger(),"finished");
  }
private:


  rclcpp::CallbackGroup::SharedPtr callback_group_0_subscribers_;
  rclcpp::CallbackGroup::SharedPtr callback_group_1_subscribers_;
  rclcpp::CallbackGroup::SharedPtr callback_group_writers_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr image_0_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr image_1_sub_;

  rclcpp::TimerBase::SharedPtr capture_stereo_timer_;

  std::string image_path_;
  std::string run_ts_;
  std::string size_;
  uint8_t rows_;
  uint8_t columns_;
  float square_length_; // in meters
  float marker_length_; // in meters

  // std::deque<std::shared_ptr<camera::FrameData>> frame_queue_;
  std::multimap<long long, std::shared_ptr<camera::FrameData>> frame_queue_;

  std::shared_ptr<camera::FrameData> left_frame_, left_frame_prev_;
  std::shared_ptr<camera::FrameData> right_frame_, right_frame_prev_;

  u_int32_t match_img_n_;

  std::vector<int> webp_write_params_;
  std::ofstream logfile_;

  Calibration calibration_;
  cv::Ptr<cv::aruco::CharucoBoard> board_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

  CAMERA_LOCAL
  void capture_stereo_callback () {

    RCLCPP_INFO_ONCE(get_logger(), "starting capture_stereo_callback - thread %s", string_thread_id().c_str());
    // just return if nothing to do
    if (frame_queue_.size() == 0) {
      return;
    }

    // want at least a second of images queued from both cameras
    // they can come in out of order from left and right - hence the need for the multi-map
    if (frame_queue_.size() < 30) {
      return;
    }
    auto time_now = rclcpp::Clock().now();
    auto time_wait = rclcpp::Time((time_now.seconds()-2)*1000000000); // want them to be at least 1 full second old
    long long wait_key = (long long)time_wait.seconds()*(long long)1000000000;
    RCLCPP_DEBUG(get_logger(),"wait_key: %lld", wait_key);

    auto itlow = frame_queue_.lower_bound(wait_key);
    auto low_key = itlow->first;

    RCLCPP_DEBUG(get_logger(),"itlow frame_key: %lld, begin() frame_key: %lld ", low_key, frame_queue_.begin()->first);

    while (frame_queue_.size() > 0 && frame_queue_.begin()->first <= low_key) {

      auto iter = frame_queue_.begin();

      // auto frame = frame_queue_.front();
      // auto frame_key = iter->first;
      auto frame = iter->second;

      logfile_ << frame->img_msg->header.stamp.sec << "." << std::setfill('0') << std::setw(9) << frame->img_msg->header.stamp.nanosec;
      logfile_ << "," << frame->camera_frame << std::endl;

      switch (frame->camera_frame) {
        case left_frame:
          left_frame_ = frame;
          break;
        case right_frame:
          right_frame_ = frame;
          break;
      }
      // frame_queue_.pop_front();
      frame_queue_.erase(iter->first);

      if (left_frame_.use_count() ==0 || right_frame_.use_count() == 0) {
        continue; // we need a left and right frame
      }
      RCLCPP_INFO_ONCE(get_logger(), "have first left frame and right frame ..");

      auto left_time = rclcpp::Time(left_frame_->img_msg->header.stamp);
      auto right_time = rclcpp::Time(right_frame_->img_msg->header.stamp);

      auto stamp_diff = rclcpp::Time(left_time)-rclcpp::Time(right_time);

      auto asd = abs(stamp_diff.seconds());
      if (asd > 0.07) {
        // if time stamps arent close - lets wait till they are
        RCLCPP_DEBUG(get_logger(), "absolute stamp diff: %f too high ", asd);
        continue;
      }

      // only interested in perfectly detected boards
      auto board_size = (size_t)((rows_-1) * (columns_-1));
      if (left_frame_->corners.size() != board_size
      || right_frame_->corners.size() != board_size) {
        RCLCPP_WARN(get_logger(), "board size: %ld not perfect left: %ld right: %ld", board_size, left_frame_->corners.size(), right_frame_->corners.size());
        return;
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
         RCLCPP_WARN(get_logger(),"Chessboards same. move!");
         continue;
      }

      RCLCPP_INFO(get_logger(),"stamp_diff(ns): %ld left_time: %f right_time: %f ", stamp_diff.nanoseconds(), left_time.seconds(), right_time.seconds());

      cv::Size size (left_frame_->img_msg->width, left_frame_->img_msg->height); // right is the same

      cv::Mat img_lc(left_frame_->img.clone());
      if (left_frame_->calibration == Calibration::chessboard) {
        cv::drawChessboardCorners(img_lc, left_frame_->pattersize, cv::Mat(left_frame_->corners), left_frame_->pattern_found);
      } else if (left_frame_->calibration == Calibration::ChArUco) {
          cv::aruco::drawDetectedMarkers(img_lc, left_frame_->marker_corners, left_frame_->marker_ids);
          // if at least one charuco corner detected
          if (left_frame_->charuco_ids.size() > 0)
              cv::aruco::drawDetectedCornersCharuco(img_lc, left_frame_->corners, left_frame_->charuco_ids, cv::Scalar(255, 0, 0));
      }
      cv::Mat img_rc(right_frame_->img.clone());
      if (right_frame_->calibration == Calibration::chessboard) {
        cv::drawChessboardCorners(img_rc, right_frame_->pattersize, cv::Mat(right_frame_->corners), right_frame_->pattern_found);
      } else if (right_frame_->calibration == Calibration::ChArUco) {
          cv::aruco::drawDetectedMarkers(img_rc, right_frame_->marker_corners, right_frame_->marker_ids);
          // if at least one charuco corner detected
          if (right_frame_->charuco_ids.size() > 0)
              cv::aruco::drawDetectedCornersCharuco(img_rc, right_frame_->corners, right_frame_->charuco_ids, cv::Scalar(255, 0, 0));
      }

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
