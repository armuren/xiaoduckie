#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include "wheel_encoder/wheel_encoder_sensor.h"

namespace xiaoduckie {
namespace sensors {

/*!
 * @brief This is the ROS2 Node that publishes the encoder messages.
 * It is constructed with an std::vector of WheelEncodersNode::WheelEncoderConfig
 * objects and creates an instance of a WheelEncoderSensor for each, then assigns
 * to each its own callback using a lambda function.  The lambda function publishes
 * the encoder value.
 * @ingroup Sensors
 */
class WheelEncodersNode : public rclcpp::Node {

  typedef rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr PublisherPtr;
  
  public:
    /*!
      * @brief This Struct holds the config for each WheelEncoderSensor
      * @ingroup Sensors
      */
    struct WheelEncoderConfig {
        int gpio_pin_;
        int resolution_;
        int publish_frequency_;
        int queue_limit;
        std::string orientation_;
        std::string topic_name_;
    };

    /*!
      * @brief Default constructor.
      * @param encoderConfigs const Reference to the std::vector of WheelEncoderConfig.
      */
    WheelEncodersNode(const std::vector<WheelEncoderConfig>& encoderConfigs)
    : Node("WheelEncodersNode"), encoderConfigs_(encoderConfigs)
    {
      int index = 0;
      for (auto const &config : encoderConfigs_) {
        encoderTotalTick_.push_back(0);

        PublisherPtr p = this->create_publisher<std_msgs::msg::Int32>(
          config.topic_name_,
          config.publish_frequency_);

        std::shared_ptr<WheelEncoderSensor> encoder = std::make_shared<WheelEncoderSensor>();
        
        encoder.get()->Init(
          config.gpio_pin_,
          [p, index, this] (int encoder_ticks) {

            auto message = std_msgs::msg::Int32();
            message.data = encoder_ticks;

            this->encoderTotalTick_[index] = encoder_ticks;
            p->publish(message);
          }
        );

        encoders_.push_back(encoder);
        ++index;
      }
    };

  private:
    std::vector<int> encoderTotalTick_;
    std::vector<WheelEncoderConfig> encoderConfigs_;
    std::vector<std::shared_ptr<WheelEncoderSensor>> encoders_;
    std::vector<PublisherPtr> publishers_;
};

} // namespace sensors
} // namespace xiaoduckie