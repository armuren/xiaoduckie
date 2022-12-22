#pragma once

#include <iostream>
#include <memory>
#include <functional>
#include <JetsonGPIO.h>

namespace xiaoduckie {
namespace sensors {

// class WheelEncoderPublisher;
/*!
 * @brief This class manages the WheelEncoder sensor connected to the GPIO
 * @ingroup Sensors
 */
// template <typename F>
class WheelEncoderSensor {

  enum DIRECTION {
    FORWARD = 1,
    BACKWARD = -1
  };

  class EncoderCallback
  {
  public:
    EncoderCallback(
      const std::string& name,
      std::shared_ptr<int> ticks,
      std::function<void(int)> callback,
      std::shared_ptr<WheelEncoderSensor::DIRECTION> direction)
      : name(name),
      ticks_(ticks),
      callback_(std::move(callback)),
      direction_(direction)
    {};

    EncoderCallback(const EncoderCallback&) = default; // Copy-constructible

    void operator()(const std::string& channel) // Callable with one string type argument
    {
      std::cout << "A callback named " << name;
      std::cout << " called from channel " << channel << std::endl;
      *ticks_ += *direction_;
      callback_(*ticks_);
    }

    bool operator==(const EncoderCallback& other) const // Equality-comparable
    {
        return name == other.name;
    }

    bool operator!=(const EncoderCallback& other) const 
    {
        return !(*this == other);
    }
      
  private:
    std::string name;
    std::function<void(int)> callback_;
    std::shared_ptr<int> ticks_ = std::make_shared<int>(0);
    std::shared_ptr<WheelEncoderSensor::DIRECTION> direction_ = std::make_shared<WheelEncoderSensor::DIRECTION>();
  };

  public:
    WheelEncoderSensor() = default;

    bool Init(int gpio, std::function<void(int)> callback) {
      gpio_pin_ = gpio;
      *direction_ = DIRECTION::FORWARD;

      EncoderCallback callback_(
        "EncoderCallback",
        ticks_,
        std::move(callback),
        direction_
      );

      GPIO::setmode(GPIO::BCM);
      GPIO::setup(gpio_pin_, GPIO::IN);
      GPIO::add_event_detect(gpio_pin_, GPIO::RISING, callback_);

      initialized_ = true;
      return true;
    };
  private:
    int gpio_pin_;
    bool initialized_ = false;
    std::shared_ptr<int> ticks_ = std::make_shared<int>(0);
    std::shared_ptr<WheelEncoderSensor::DIRECTION> direction_ = std::make_shared<WheelEncoderSensor::DIRECTION>();
};


} // namespace sensors
} // namespace xiaoduckie
