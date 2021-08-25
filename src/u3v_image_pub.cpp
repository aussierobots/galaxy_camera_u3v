#include <chrono>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <string>
#include "galaxy_camera_u3v/GxIAPI.h"
#include "galaxy_camera_u3v/DxImageProc.h"
#include "galaxy_camera_u3v/gx_utils.h"
#include <unistd.h>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <opencv2/core.hpp>
#include <image_transport/image_transport.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include "galaxy_camera_u3v/visibility_control.h"

using namespace std::chrono_literals;

#define FRAME_RATE 30.0
#define BAYER_ENCODING "BAYER"

namespace camera {
//
// this is a read only camera publisher - the stereo_capture_ctl inits and updates
// the camera settings
//
class U3vImagePub: public rclcpp::Node
{
public:
  CAMERA_PUBLIC
  explicit U3vImagePub(const rclcpp::NodeOptions &options)
  : Node("u3v_image_pub", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  // : Node("u3v_image_pub", options),

  camera_info_url_("package://galaxy_camera_u3v/camera_info/${NAME}.yaml")

  {
    // this flag is used control if certain parameters can be updated
    is_initialising_ = true;

    auto qos = rclcpp::SensorDataQoS();
    qos.reliable();
    // auto qos = rclcpp::ServicesQoS();

    // ros2 parameter call backs
    parameters_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&U3vImagePub::on_set_parameters_callback, this, std::placeholders::_1));

    // initial camera info
    topic_ = this->declare_parameter<std::string>("topic","stereo/right");
    RCLCPP_INFO(this->get_logger(),"parameter topic_: %s", topic_.c_str());

    RCLCPP_INFO(this->get_logger(),"Using camera_info_url_: %s", camera_info_url_.c_str());
    camera_info_manager::CameraInfoManager cim(this, topic_, camera_info_url_);
    camera_info_ = cim.getCameraInfo();

    // variables for GX Library
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t num_devices;

    // initialise library
    status = GXInitLib();
    if (status != GX_STATUS_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "GXInitLib failed ... ");
      exit (-1);
    }

    // Get device enumerated number - should be at least 1
    status = GXUpdateDeviceList(&num_devices, 1000);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "%s", error_msg);
      exit (-2);
    }
    if(num_devices <= 0){
      RCLCPP_ERROR(this->get_logger(),"no camera devices found ... plug in and try again");
      exit (-3);
    }
    RCLCPP_INFO(this->get_logger(), "Found %d cameras ....", num_devices);

    device_sn_ = this->declare_parameter<std::string>("device_sn","FDS20110008");
    RCLCPP_INFO(this->get_logger(),"parameter device_sn_: %s", device_sn_.c_str());

    GX_OPEN_PARAM gx_open_param;
    gx_open_param.accessMode = GX_ACCESS_EXCLUSIVE;
    gx_open_param.openMode = GX_OPEN_SN;
    std::vector<char> device_sn_cstr (device_sn_.c_str(), device_sn_.c_str() + device_sn_.size()+1);
    gx_open_param.pszContent = device_sn_cstr.data();
    status = GXOpenDevice(&gx_open_param, &this->gx_dev_handle_);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "error opening sn: %s camera: %s", gx_open_param.pszContent, error_msg);
      exit (-5);
    }
    RCLCPP_DEBUG(this->get_logger(), "gx_dev_handle_0: %p", gx_dev_handle_);


    // status = GXGetInt(this->gx_dev_handle_, GX_INT_PAYLOAD_SIZE, &this->payload_size_);
    // if (status != GX_STATUS_SUCCESS) {
    //   auto error_msg = GetErrorString(status);
    //   RCLCPP_ERROR(this->get_logger(), "error getting payload_size_: %s", error_msg);
    //   exit (-7);
    // }
    this->declare_parameter<int64_t>("pixel_format", GX_PIXEL_FORMAT_BAYER_RG10);
    // this->declare_parameter<int64_t>("pixel_format", GX_PIXEL_FORMAT_BAYER_RG8);

    CameraDeviceInfo camera_info;
    status = GetCameraInfo(gx_dev_handle_, &camera_info);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "error getting device camera info: %s", error_msg);
      exit (-6);
    }
    RCLCPP_INFO(this->get_logger(),"camera - vendor_name: %s model_name: %s serial_number: %s device_version: %s firmware_version: %s color_filter: %d",
      camera_info.vendor_name.c_str(),
      camera_info.model_name.c_str(),
      camera_info.serial_number.c_str(),
      camera_info.device_version.c_str(),
      camera_info.device_firmware_version.c_str(),
      camera_info.color_filter
    );

    if (!camera_info.color_filter) {
      RCLCPP_ERROR(this->get_logger(), "camera not bayer color .. exiting");
      exit(-6);
    }

    status = GXGetEnum(this->gx_dev_handle_, GX_ENUM_PIXEL_SIZE, &this->pixel_size_);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "error getting pixel_size: %s", error_msg);
      exit (-6);
    }

    status = GXGetEnum(this->gx_dev_handle_, GX_ENUM_PIXEL_COLOR_FILTER, &this->color_filter_);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "error getting color_filter: %s", error_msg);
      exit (-6);
    }

    ImageFormat image_format;
    status = GetImageFormat(gx_dev_handle_, &image_format);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "error getting image format: %s", error_msg);
      exit (-6);
    }
    RCLCPP_INFO(get_logger(),"image format - sensor width: %ld sensor height: %ld width max: %ld height max: %ld width: %ld height: %ld offset x: %ld offset y:%ld",
      image_format.sensor_width, image_format.sensor_height,
      image_format.width_max, image_format.height_max,
      image_format.width, image_format.height,
      image_format.offset_x, image_format.offset_y
    );


    // not implemented on test device
    // status = GXSetEnum(this->gx_dev_handle_, GX_ENUM_DEAD_PIXEL_CORRECT, GX_DEAD_PIXEL_CORRECT_ON);
    // if (status != GX_STATUS_SUCCESS) {
    //   auto error_msg = GetErrorString(status);
    //   RCLCPP_ERROR(this->get_logger(), "error setting GX_DEAD_PIXEL_CORRECT_ON: %s", error_msg);
    //   exit (-6);
    // }

    // iamge encoding to publish
    this->declare_parameter<std::string>("image_encoding", "RGB8");

    // camera parameters - these are the default values. Values will be set on the camera if appropriate
    this->declare_parameter<int64_t>("acquisition_mode", GX_ACQ_MODE_CONTINUOUS);
    this->declare_parameter<int64_t>("trigger_mode",GX_TRIGGER_MODE_OFF);
    this->declare_parameter<int64_t>("trigger_source",GX_TRIGGER_SOURCE_SOFTWARE);
    this->declare_parameter<int64_t>("exposure_mode", GX_EXPOSURE_MODE_TIMED);
    this->declare_parameter<double_t>("exposure_time", 100.0);
    this->declare_parameter<int64_t>("exposure_auto", GX_EXPOSURE_AUTO_CONTINUOUS);
    this->declare_parameter<double_t>("auto_exposure_time_min", 20.0); // microseconds
    this->declare_parameter<double_t>("auto_exposure_time_max", 1000000.0); //microseconds
    this->declare_parameter<int64_t>("expected_gray_value", 120);
    this->declare_parameter<int64_t>("acquisition_frame_rate_mode",GX_ACQUISITION_FRAME_RATE_MODE_ON);
    this->declare_parameter<double_t>("acquisition_frame_rate", 56.0);
    // this->declare_parameter<double_t>("acquisition_frame_rate", 30.0);
    this->declare_parameter<double_t>("current_acquisition_frame_rate",0.0);
    this->declare_parameter<double_t>("gain",0.0); // read only - gets updated periodically
    this->declare_parameter<int64_t>("gain_auto", GX_GAIN_AUTO_CONTINUOUS);
    // this->declare_parameter<int64_t>("gain_auto", GX_GAIN_AUTO_OFF);
    this->declare_parameter<double_t>("auto_gain_min", 0.0); // dB
    this->declare_parameter<double_t>("auto_gain_max", 24.0); // dB
    this->declare_parameter<int64_t>("balance_ratio_selector", GX_BALANCE_RATIO_SELECTOR_RED);
    this->declare_parameter<double_t>("balance_ratio",1.0); // read only when continuous - gets updated periodically
    this->declare_parameter<int64_t>("balance_white_auto", GX_BALANCE_WHITE_AUTO_CONTINUOUS);
    int roi_width=1024;
    int roi_height=768;
    this->declare_parameter<int64_t>("awb_roi_width", roi_width);
    this->declare_parameter<int64_t>("awb_roi_height", roi_height);
    this->declare_parameter<int64_t>("awb_roi_offset_x", int16_t(2048/2 - (roi_width/2)));
    this->declare_parameter<int64_t>("awb_roi_offset_y", int16_t(1536/2 - (roi_height/2)));
    this->declare_parameter<int64_t>("awb_lamp_house", GX_AWB_LAMP_HOUSE_ADAPTIVE);


    status = GXStreamOn(gx_dev_handle_);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "error stream on (%s): %s", device_sn_.c_str(), error_msg);
      exit (-8);
    }
    // setup c style buffers for the camera
    this->RGB_image_buf_ = new u_char[image_format.sensor_height * image_format.sensor_width * 3];
    this->image_buf_ = new u_char[this->payload_size_];

    // publishers
    pub_ = image_transport::create_camera_publisher(this, topic_+"/image_raw", qos.get_rmw_qos_profile());

    // initialise are start the timer to work out the frames per second)
    auto start_time = std::chrono::steady_clock::now();
    frame_time_ = start_time;
    frame_count_ = 0;
    frame_timer_ = this->create_wall_timer(1s, std::bind(&U3vImagePub::frame_timer_callback, this));

    param_timer_ = this->create_wall_timer(1s, std::bind(&U3vImagePub::param_timer_callback, this));

    capture_timer_ = this->create_wall_timer(10ns, std::bind(&U3vImagePub::capture_timer_callback, this));

    // just use for testing - trigger should come from the controller
    // auto frame_duration = 1000000us/FRAME_RATE;
    // trigger_timer_ = this->create_wall_timer(frame_duration, std::bind(&U3vImagePub::trigger_timer_callback, this));


    capture_trigger_sub_ = this->create_subscription<sensor_msgs::msg::TimeReference>("/capture_trigger", qos, std::bind(&U3vImagePub::capture_trigger_callback, this, std::placeholders::_1));
    is_initialising_ = false;
  }

  CAMERA_LOCAL
  ~U3vImagePub(){

    if (this->gx_dev_handle_ != NULL) {
      RCLCPP_INFO(this->get_logger(), "closing gx_dev_handle_");
      GXStreamOff(this->gx_dev_handle_);
      GXCloseDevice(this->gx_dev_handle_);
    }
    GXCloseLib();

    if (RGB_image_buf_ != NULL)
      delete[] RGB_image_buf_;

    RCLCPP_INFO(this->get_logger(),"finished");
  }

private:
  bool is_initialising_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
  rclcpp::TimerBase::SharedPtr frame_timer_;
  rclcpp::TimerBase::SharedPtr param_timer_;
  // rclcpp::TimerBase::SharedPtr trigger_timer_;
  rclcpp::TimerBase::SharedPtr info_timer_;
  rclcpp::TimerBase::SharedPtr capture_timer_;

  rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr capture_trigger_sub_;

  // ros2 camera
  image_transport::CameraPublisher pub_;

  std::string topic_;

  std::string camera_info_url_;
  camera_info_manager::CameraInfo camera_info_;

  std::string image_encoding_;

  // camera related
  std::string device_sn_;
  rclcpp::Time trigger_timestamp_;
  std::string trigger_source_;
  GX_DEV_HANDLE gx_dev_handle_;

  int64_t color_filter_;
  int64_t pixel_size_;
  int64_t payload_size_;

  u_char *RGB_image_buf_;
  u_char *image_buf_;

  std::chrono::time_point<std::chrono::steady_clock> frame_time_;

  uint16_t frame_count_;

  CAMERA_LOCAL
  void update_payload_size() {
    auto status = GXGetInt(gx_dev_handle_, GX_INT_PAYLOAD_SIZE, &payload_size_);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(get_logger(), "error getting payload_size_: %s", error_msg);
      exit (-9);
    }

    RCLCPP_INFO(get_logger(),"payload_size_: %ld", payload_size_);
  }

  CAMERA_LOCAL
  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(const std::vector<rclcpp::Parameter> &parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const rclcpp::Parameter &parameter: parameters){

      if (parameter.get_name() == "pixel_format" &&
          parameter.get_type() ==  rclcpp::ParameterType::PARAMETER_INTEGER){
          result = param_gx_set_enum(GX_ENUM_PIXEL_FORMAT, parameter.as_int());
          update_payload_size();
      }
      if (parameter.get_name() == "image_encoding" &&
          parameter.get_type() ==  rclcpp::ParameterType::PARAMETER_STRING){
          image_encoding_ = parameter.as_string();
      }
      if (parameter.get_name() == "topic" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
        if (!topic_.empty()) {
          result.successful = false;
          result.reason = "topic can't be changed, once set!";
        }
      }
      if (parameter.get_name() == "device_sn" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING){
        if (!device_sn_.empty()) {
          result.successful = false;
          result.reason = "device_sn can't be changed, once set!";
        }
      }
      if (parameter.get_name() == "acquisition_mode" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_ACQUISITION_MODE, parameter.as_int());
      }
      if (parameter.get_name() == "trigger_mode" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_TRIGGER_MODE, parameter.as_int());
      }
      if (parameter.get_name() == "trigger_source" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_TRIGGER_SOURCE, parameter.as_int());
      }
      if (parameter.get_name() == "exposure_mode" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_EXPOSURE_MODE, parameter.as_int());
      }
      if (parameter.get_name() == "exposure_time" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_EXPOSURE_TIME, parameter.as_double());
      }
      if (parameter.get_name() == "exposure_auto" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_EXPOSURE_AUTO, parameter.as_int());
      }
      if (parameter.get_name() == "auto_exposure_time_min" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, parameter.as_double());
      }
      if (parameter.get_name() == "auto_exposure_time_max" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, parameter.as_double());
      }
      if (parameter.get_name() == "expected_gray_value" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_int(GX_INT_GRAY_VALUE, parameter.as_int());
      }
      if (parameter.get_name() == "acquisition_frame_rate_mode" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_ACQUISITION_FRAME_RATE_MODE, parameter.as_int());
      }
      if (parameter.get_name() == "acquisition_frame_rate" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_ACQUISITION_FRAME_RATE, parameter.as_double());
      }
      if (parameter.get_name() == "gain" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_GAIN, parameter.as_double());
      }
      if (parameter.get_name() == "gain_auto" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_GAIN_AUTO, parameter.as_int());
      }
      if (parameter.get_name() == "auto_gain_min" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_AUTO_GAIN_MIN, parameter.as_double());
      }
      if (parameter.get_name() == "auto_gain_max" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_AUTO_GAIN_MAX, parameter.as_double());
      }
      if (parameter.get_name() == "balance_ratio_selector" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_BALANCE_RATIO_SELECTOR, parameter.as_int());
      }
      if (parameter.get_name() == "balance_ratio" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          result = param_gx_set_float(GX_FLOAT_BALANCE_RATIO, parameter.as_double());
      }
      if (parameter.get_name() == "balance_white_auto" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_BALANCE_WHITE_AUTO, parameter.as_int());
      }
      if (parameter.get_name() == "awb_lamp_house" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_enum(GX_ENUM_AWB_LAMP_HOUSE, parameter.as_int());
      }
      if (parameter.get_name() == "awb_roi_width" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_int(GX_INT_AWBROI_WIDTH, parameter.as_int());
      }
      if (parameter.get_name() == "awb_roi_height" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_int(GX_INT_AWBROI_HEIGHT, parameter.as_int());
      }
      if (parameter.get_name() == "awb_roi_offset_x" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_int(GX_INT_AWBROI_OFFSETX, parameter.as_int());
      }
      if (parameter.get_name() == "awb_roi_offset_y" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          result = param_gx_set_int(GX_INT_AWBROI_OFFSETY, parameter.as_int());
      }

      if (!result.successful) {
        RCLCPP_WARN(this->get_logger(), "parameter %s not set - %s",parameter.get_name().c_str(), result.reason.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(), "parameter set %s: %s", parameter.get_name().c_str(), parameter.value_to_string().c_str());
      }
    }
    return result;
  }

  CAMERA_LOCAL
  rcl_interfaces::msg::SetParametersResult param_gx_set_enum(GX_FEATURE_ID_CMD feature_id, int64_t n_value) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    auto status = GXSetEnum(gx_dev_handle_, feature_id, n_value);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      result.successful = false;
      result.reason = error_msg;
    }
    return result;
  }

  CAMERA_LOCAL
  rcl_interfaces::msg::SetParametersResult param_gx_set_int(GX_FEATURE_ID_CMD feature_id, int64_t n_value) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_writeable = false;
    status = GXIsWritable(gx_dev_handle_, feature_id, &is_writeable);
    if (status == GX_STATUS_SUCCESS) {
      if(is_writeable){
        status = GXSetInt(gx_dev_handle_, feature_id, n_value);
      }
    }
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      result.successful = false;
      result.reason = error_msg;
    }
    return result;
  }

  CAMERA_LOCAL
  rcl_interfaces::msg::SetParametersResult param_gx_set_float(GX_FEATURE_ID_CMD feature_id, double_t n_value) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_writeable = false;
    // silently fail if not writeable - timer callback updates parameter with actual values
    status = GXIsWritable(gx_dev_handle_, feature_id, &is_writeable);
    if (status == GX_STATUS_SUCCESS) {
      if(is_writeable){
        status = GXSetFloat(gx_dev_handle_, feature_id, n_value);
      }
    }
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      result.successful = false;
      result.reason = error_msg;
    }
    return result;
  }

  CAMERA_LOCAL
  void param_timer_callback(){
    update_changed_enum_param("acquisition_mode", GX_ENUM_ACQUISITION_MODE);
    update_changed_enum_param("trigger_mode", GX_ENUM_TRIGGER_MODE);
    update_changed_enum_param("exposure_mode", GX_ENUM_EXPOSURE_MODE);
    update_changed_float_param("exposure_time", GX_FLOAT_EXPOSURE_TIME);
    update_changed_enum_param("exposure_auto", GX_ENUM_EXPOSURE_MODE);
    update_changed_float_param("auto_exposure_time_min", GX_FLOAT_AUTO_EXPOSURE_TIME_MIN);
    update_changed_float_param("auto_exposure_time_max", GX_FLOAT_AUTO_EXPOSURE_TIME_MAX);
    update_changed_int_param("expected_gray_value", GX_INT_GRAY_VALUE);
    update_changed_enum_param("acquisition_frame_rate_mode", GX_ENUM_ACQUISITION_FRAME_RATE_MODE);
    update_changed_float_param("acquisition_frame_rate", GX_FLOAT_ACQUISITION_FRAME_RATE);
    update_changed_float_param("current_acquisition_frame_rate", GX_FLOAT_CURRENT_ACQUISITION_FRAME_RATE);
    update_changed_float_param("gain", GX_FLOAT_GAIN);
    update_changed_enum_param("gain_auto", GX_ENUM_GAIN_AUTO);
    update_changed_float_param("auto_gain_min", GX_FLOAT_AUTO_GAIN_MIN);
    update_changed_float_param("auto_gain_max", GX_FLOAT_AUTO_GAIN_MAX);
    update_changed_enum_param("balance_ratio_selector", GX_ENUM_BALANCE_RATIO_SELECTOR);
    update_changed_float_param("balance_ratio", GX_FLOAT_BALANCE_RATIO);
    update_changed_enum_param("balance_white_auto", GX_ENUM_BALANCE_WHITE_AUTO);
    update_changed_enum_param("awb_lamp_house", GX_ENUM_AWB_LAMP_HOUSE);
  }

  CAMERA_LOCAL
  void update_changed_enum_param(std::string param_name, GX_FEATURE_ID_CMD feature_id) {
    auto param = this->get_parameter(param_name);
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_readable = false;
    status = GXIsReadable(gx_dev_handle_, feature_id, &is_readable);
    if (status == GX_STATUS_SUCCESS && is_readable){
      int64_t p_value = param.as_int();
      int64_t gx_value = 0;
      status = GXGetEnum(gx_dev_handle_, feature_id, &gx_value);
      if (status == GX_STATUS_SUCCESS && p_value != gx_value) {
        auto updated_param = rclcpp::Parameter(param_name, gx_value);
        this->set_parameter(updated_param);
      }
    }
  }

  CAMERA_LOCAL
  void update_changed_int_param(std::string param_name, GX_FEATURE_ID_CMD feature_id) {
    auto param = this->get_parameter(param_name);
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_readable = false;
    status = GXIsReadable(gx_dev_handle_, feature_id, &is_readable);
    if (status == GX_STATUS_SUCCESS && is_readable){
      int64_t p_value = param.as_int();
      int64_t gx_value = 0;
      status = GXGetInt(gx_dev_handle_, feature_id, &gx_value);
      if (status == GX_STATUS_SUCCESS && p_value != gx_value) {
        auto updated_param = rclcpp::Parameter(param_name, gx_value);
        this->set_parameter(updated_param);
      }
    }
  }

  CAMERA_LOCAL
  void update_changed_float_param(std::string param_name, GX_FEATURE_ID_CMD feature_id) {
    auto param = this->get_parameter(param_name);
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_readable = false;
    status = GXIsReadable(gx_dev_handle_, feature_id, &is_readable);
    if (status == GX_STATUS_SUCCESS && is_readable){
      double p_value = param.as_double();
      double gx_value = 0.0;
      status = GXGetFloat(gx_dev_handle_, feature_id, &gx_value);
      if (status == GX_STATUS_SUCCESS && p_value != gx_value) {
        auto updated_param = rclcpp::Parameter(param_name, gx_value);
        this->set_parameter(updated_param);
      }
    }
  }

  CAMERA_LOCAL
  void frame_timer_callback() {
    auto end_time = std::chrono::steady_clock::now();
    auto frame_count = frame_count_;
    frame_count_ = 0;
    auto frame_time = frame_time_;
    frame_time_=end_time;

    auto fps = frame_count/(std::chrono::duration_cast<std::chrono::milliseconds>(end_time-frame_time).count()/1000.0);

    RCLCPP_INFO(this->get_logger(), "fps: %f", fps);
  }

  // CAMERA_LOCAL
  // void trigger_timer_callback() {
  //   this->trigger_timestamp_ = rclcpp::Clock().now();

  //   if(triggered_) {
  //     RCLCPP_DEBUG(this->get_logger(),"last trigger not finished triggered_: %d", triggered_);
  //     return;
  //   }

  //   // GX_STATUS status = GXSendCommand(this->gx_dev_handle_, GX_COMMAND_TRIGGER_SOFTWARE);
  //   triggered_ = true;

  //   // if (status != GX_STATUS_SUCCESS) {
  //   //   auto error_msg = GetErrorString(status);
  //   //   RCLCPP_WARN(this->get_logger(), "unable to trigger gx_dev_handle_(%d): %s", this->gx_dev_handle_, error_msg);
  //   // }
  // }

  CAMERA_LOCAL
  bool trigger_mode() {
    auto param = get_parameter("trigger_mode");
    if (param.as_int() == GX_TRIGGER_MODE_OFF){
      return false;
    } else {
      return true;
    }
  }

  CAMERA_LOCAL
  void capture_trigger_callback(const sensor_msgs::msg::TimeReference::SharedPtr msg) {
    // if it was started in continuous and we receive capture trigger message enable it
    if (!trigger_mode()) {
      set_parameter(rclcpp::Parameter("trigger_mode", GX_TRIGGER_MODE_ON));
      set_parameter(rclcpp::Parameter("trigger_source", GX_TRIGGER_SOURCE_SOFTWARE));
      // started in timer
      capture_timer_->cancel();
    }

    this->trigger_timestamp_ = msg->time_ref;
    this->trigger_source_ = msg->source;

    GX_STATUS status = GXSendCommand(this->gx_dev_handle_, GX_COMMAND_TRIGGER_SOFTWARE);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_WARN(this->get_logger(), "unable to trigger gx_dev_handle_(%p): %s", this->gx_dev_handle_, error_msg);
    }

    this->capture_device(topic_, gx_dev_handle_, RGB_image_buf_, image_buf_, color_filter_, payload_size_);
  }

  CAMERA_LOCAL
  void capture_timer_callback() {
    trigger_timestamp_ = rclcpp::Clock().now();
    this->capture_device(topic_, gx_dev_handle_, RGB_image_buf_, image_buf_, color_filter_, payload_size_);
  }

  CAMERA_LOCAL
  void capture_device(std::string topic, GX_DEV_HANDLE gx_dev_handle, u_char * RGB_image_buf, u_char * image_buf, int64_t color_filter, int64_t payload_size) {

    auto stamp = this->trigger_timestamp_;
    GX_STATUS status = GX_STATUS_SUCCESS;
    PGX_FRAME_BUFFER frame_buffer = NULL;

    status = GXDQBuf(gx_dev_handle, &frame_buffer, 25);
    if (status == GX_STATUS_TIMEOUT) {
      RCLCPP_DEBUG(get_logger(), "%s timeout handle %p capture", topic.c_str(), gx_dev_handle);
      return;
    } else  if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "%s error GXDQBuf: %s", topic.c_str(), error_msg);
      return;
    }

    if (frame_buffer->nStatus != GX_FRAME_STATUS_SUCCESS) {
      RCLCPP_WARN(get_logger(),"%s abnormal camera acquisition - code: %d", topic.c_str(), frame_buffer->nStatus);
    } else if (frame_buffer->nPixelFormat != GX_PIXEL_FORMAT_BAYER_RG8 && frame_buffer->nPixelFormat != GX_PIXEL_FORMAT_BAYER_RG10) {
      RCLCPP_ERROR(get_logger(),"%s unknown pixel format %d", topic.c_str(), frame_buffer->nPixelFormat);
    } else if (frame_buffer->nPixelFormat == GX_PIXEL_FORMAT_BAYER_RG8 && image_encoding_=="BAYER_RGGB16") {
      RCLCPP_ERROR(get_logger(),"image_encoding %s with pixel_format BAYER_RG8 capture not supported", image_encoding_.c_str());
    } else {

      RCLCPP_DEBUG(get_logger(), "%s handle %p capture frame_id: %ld timestamp: %lu.%.10lu", topic.c_str(), gx_dev_handle, frame_buffer->nFrameID, uint64_t(stamp.seconds()),stamp.nanoseconds());

      // Initialize a shared pointer to an Image message.
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      msg->header.stamp = stamp + rclcpp::Duration(0,get_parameter("exposure_time").as_double()*1000);
      if (!trigger_source_.empty()) {
        msg->header.frame_id = trigger_source_;
      } else {
        msg->header.frame_id = device_sn_;
      }
      msg->is_bigendian = false;
      msg->height = frame_buffer->nHeight;
      msg->width = frame_buffer->nWidth;

      if (image_encoding_ == "RGB8") {
        // convert image to rgb8
        PixelFormatConvert(frame_buffer, color_filter, RGB_image_buf);

        size_t msg_size = frame_buffer->nHeight * frame_buffer->nWidth * 3;
        msg->encoding = sensor_msgs::image_encodings::RGB8;
        msg->step = frame_buffer->nWidth*3;
        msg->data.resize(msg_size);
        memcpy(&msg->data[0],RGB_image_buf,msg_size);
      } else if (image_encoding_ == "BAYER_RGGB8") {
        msg->encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
        msg->step = frame_buffer->nWidth;
        size_t msg_size = frame_buffer->nHeight * frame_buffer->nWidth;
        if (frame_buffer->nPixelFormat == GX_PIXEL_FORMAT_BAYER_RG8) {
          msg->data.resize(msg_size);
          memcpy(&msg->data[0],frame_buffer->pImgBuf,msg_size);
        } else {
          Raw16toRaw8(frame_buffer, color_filter, image_buf);
          msg->data.resize(msg_size);
          memcpy(&msg->data[0], image_buf, msg_size);
        }
      } else if (image_encoding_ == "BAYER_RGGB16"
              && frame_buffer->nPixelFormat == GX_PIXEL_FORMAT_BAYER_RG10) {
        // Raw10PackedtoRaw16(frame_buffer, image_buf);
        size_t msg_size = frame_buffer->nHeight * frame_buffer->nWidth * 2;
        msg->encoding = sensor_msgs::image_encodings::BAYER_RGGB16;
        msg->step = frame_buffer->nWidth*2;
        msg->data.resize(msg_size);
        // memcpy(&msg->data[0],image_buf,msg_size);
        memcpy(&msg->data[0],frame_buffer->pImgBuf,msg_size);
      } else {
        RCLCPP_ERROR(this->get_logger(), "%s invalid encoding. Not publishing image!", image_encoding_.c_str());
        return;
      }
      frame_count_++;
      auto msg_stamp = msg->header.stamp;
      // RCLCPP_INFO(get_logger(), "trigger_timestamp: %ld msg->header.stamp: %d.%d",
      //     trigger_timestamp_.nanoseconds(),
      //     msg_stamp.sec, msg_stamp.nanosec);
      pub_.publish(*std::move(msg),camera_info_);
    }

    status = GXQBuf(gx_dev_handle, frame_buffer);
    if (status != GX_STATUS_SUCCESS) {
      auto error_msg = GetErrorString(status);
      RCLCPP_ERROR(this->get_logger(), "%s error GXQBuf: %s", topic.c_str(), error_msg);
    }
  }
};
} // end namespace car_capture

RCLCPP_COMPONENTS_REGISTER_NODE(camera::U3vImagePub)