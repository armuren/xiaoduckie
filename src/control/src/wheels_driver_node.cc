#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "wheels_driver_node.h"

namespace xiaoduckie {
  namespace control {

using std::placeholders::_1;

WheelsDriverNode::WheelsDriverNode()
: Node("wheels_driver_node")
{
  subscription_ = this->create_subscription<xiaocar_msgs::msg::WheelsCmdStamped>(
  "/xiaoduckie/wheel_velocities", 10, std::bind(&WheelsDriverNode::Callback, this, _1));

  driver_ = std::make_shared<Driver>();
  driver_.get()->setLeftWheelVelocity(0.0);
  driver_.get()->setRightWheelVelocity(0.0);
  driver_.get()->PWMUpdate();
}

void WheelsDriverNode::Callback(const xiaocar_msgs::msg::WheelsCmdStamped & msg) const {
  RCLCPP_INFO(this->get_logger(), "Received Velocities %f - %f", msg.vel_left, msg.vel_right);
  driver_.get()->setLeftWheelVelocity(msg.vel_left);
  driver_.get()->setRightWheelVelocity(msg.vel_right);
  driver_.get()->PWMUpdate();
}

  } // namespace control
} // namespace xiaoduckie

// #include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
// RCLCPP_COMPONENTS_REGISTER_NODE(xiaoduckie::control::WheelsDriverNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<xiaoduckie::control::WheelsDriverNode>());
  rclcpp::shutdown();
  return 0;
}
