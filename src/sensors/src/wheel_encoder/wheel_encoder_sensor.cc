#include <iostream>
#include <memory>
#include <functional>
#include <JetsonGPIO.h>

#include <sensors/wheel_encoder/wheel_encoder_sensor.h>

namespace xiaoduckie {
  namespace sensors {

bool WheelEncoderSensor::Init(int gpio, std::function<void(int)> callback) {
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
}

  } // namespace sensors
} // namespace xiaoduckie
