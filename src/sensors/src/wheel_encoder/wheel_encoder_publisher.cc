#include <iostream>

#include "wheel_encoder_publisher.h"

namespace xiaoduckie {
namespace sensors {


} // namespace sensors
} // namespace xiaoduckie

int main(int argc, char * argv[])
{
 
  std::cout << "starting ... " << std::endl;
  rclcpp::init(argc, argv);

  xiaoduckie::sensors::WheelEncodersNode::WheelEncoderConfig config = {
    18,
    135,
    30,
    10,
    "left",
    "Left_Wheel_Encoder"
  };
  std::vector<xiaoduckie::sensors::WheelEncodersNode::WheelEncoderConfig> configs = {config};
  rclcpp::spin(std::make_shared<xiaoduckie::sensors::WheelEncodersNode>(configs));
  rclcpp::shutdown();
  return 0;
}