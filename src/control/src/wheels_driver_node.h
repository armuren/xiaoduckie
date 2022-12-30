#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <control/motor/driver.h>
#include <xiaocar_msgs/msg/wheels_cmd_stamped.hpp>

namespace xiaoduckie {
  namespace control {

class WheelsDriverNode : public rclcpp::Node
{
  public:
    WheelsDriverNode();
  private:
    void Callback(const xiaocar_msgs::msg::WheelsCmdStamped & msg) const;
    rclcpp::Subscription<xiaocar_msgs::msg::WheelsCmdStamped>::SharedPtr subscription_;
    std::shared_ptr<Driver> driver_;
};

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<MinimalPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }
  } // namespace control
} // namespace xiaoduckie
