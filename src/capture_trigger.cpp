#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include "galaxy_camera_u3v/visibility_control.h"

using namespace std::chrono_literals;

namespace camera {
class CaptureTrigger: public rclcpp::Node
{
public:
  CAMERA_PUBLIC
  explicit CaptureTrigger(const rclcpp::NodeOptions &options)
  : Node("capture_trigger", rclcpp::NodeOptions(options).use_intra_process_comms(true))
  {
    auto qos = rclcpp::SensorDataQoS();
    qos.reliable();

    trigger_pub_ = this->create_publisher<sensor_msgs::msg::TimeReference>("capture_trigger", qos);
    auto trigger_hz = declare_parameter<int8_t>("trigger_hz", 30);

    auto trigger_period = 1000000us/trigger_hz;
    RCLCPP_INFO(get_logger(), "starting with trigger_hz: %ld, trigger_period: %ld", trigger_hz, std::chrono::microseconds(trigger_period).count());

    trigger_timer_ = create_wall_timer(trigger_period, std::bind(&CaptureTrigger::trigger_timer_callback, this));
  }


  CAMERA_LOCAL
  ~CaptureTrigger() {
    RCLCPP_INFO(get_logger(), "finished");
  }

private:
  rclcpp::TimerBase::SharedPtr trigger_timer_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr trigger_pub_;
  uint64_t timer_count_ = 0;

  CAMERA_LOCAL
  void trigger_timer_callback() {
    auto trigger_time = rclcpp::Clock().now();
    auto msg = std::make_unique<sensor_msgs::msg::TimeReference>();
    msg->header.frame_id = "trigger_timer";
    msg->header.stamp = trigger_time;
    msg->source = std::to_string(timer_count_++);
    msg->time_ref=trigger_time;
    trigger_pub_->publish(*msg);
  }
};

}  // end namespace camera

RCLCPP_COMPONENTS_REGISTER_NODE(camera::CaptureTrigger)