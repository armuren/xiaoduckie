#pragma once

#include <iostream>
#include <memory>
#include <functional>
#include <JetsonGPIO.h>

namespace xiaoduckie {
namespace sensors {

/*!
 * @brief This class manages the WheelEncoder sensor connected to the GPIO
 * pins in the NVIDIA Jetson Nano Board
 * @ingroup Sensors
*/
class WheelEncoderSensor {

  /*!
    * @brief ENUM that represents the direction of motion for the robot.
    * This value is updated when the controller makes a decision about
    * direction.
    * The value is used to update the global tick count on the encoder.
    * @ingroup Sensors
    */
  enum DIRECTION {
    FORWARD = 1,
    BACKWARD = -1
  };

  /*!
    * @brief This class is used by the Jetson GPIO c++ library as a callback
    * for the event detect on a GPIO pin.
    * We cannot use a regular lambda bcause this Jetson GPIO library requires
    * the callbacks must be equaility-comparable.
    * More details: https://github.com/pjueon/JetsonGPIO/issues/55#issuecomment-1059888835
    * @ingroup Sensors
    */
  class EncoderCallback
  {
  public:
    /*!
      * @brief Default constructor.
      * @param name Reference to the name assigned to this callback.
      * @param ticks Pointer referring to total ticks in the WheelEncoderSensor.
      * @param callback The external callback passed to WheelEncoderSensor.
      * @param direction Pointer to the DIRECTION enum in WheelEncoderSensor.
      */
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

    /*!
      * @brief Default copy constructor
      */
    EncoderCallback(const EncoderCallback&) = default; // Copy-constructible

    /*!
      * @brief Callable executed by the GPIO library upon event detection.
      * Used to update the global tick count and execute the callback provided
      * to the Encoder through its constructor.
      * @param channel Reference to the GPIO pin passed in by the library.
      */
    void operator()(const std::string& channel) // Callable with one string type argument
    {
      std::cout << "A callback named " << name;
      std::cout << " called from channel " << channel << std::endl;
      *ticks_ += *direction_;
      callback_(*ticks_);
    }

    /*!
      * @brief equality operator used by GPIO library to remove callbacks.
      * @param other Reference to the object being compared.
      */
    bool operator==(const EncoderCallback& other) const // Equality-comparable
    {
        return name == other.name;
    }

    /*!
      * @brief in-equality operator used by GPIO library to remove callbacks.
      * @param other Reference to the object being compared.
      */
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
    /*!
      * @brief Default constructor.
      */
    WheelEncoderSensor() = default;

    /*!
      * @brief The Init function sets the GPIO used by this encoder,
      * accepts an additional callback to use when the encoder is
      * updated.
      * It initializes the internal callback mechanism and passes it
      * the additional/external one.
      * @param gpio The GPIO pin to which the encoder is connected.
      * @param callback The external callback.
      */
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
