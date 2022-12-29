#include <sensors/hat/hat.h>
#include <sensors/hat/pwm.h>
#include <sensors/motor_controller/motor.h>

namespace xiaoduckie {
  namespace sensors {

HAT::HAT(int i2c_address, double frequency)
: i2c_address_(i2c_address),
frequency_(frequency)
{
  pwm_ = std::make_shared<PWM>(i2c_address_);
  std::cout << "HAT:  address of pwm_ " << pwm_ << std::endl;
  pwm_.get()->setPWMFreq(frequency_);
}

std::shared_ptr<Motor> HAT::getLeftMotor() {
  left_motor_ = std::make_shared<Motor>(
    "left",
    pwm_,
    10,
    9,
    8,
    MotorDirectionControl::PWM
  );
  return std::move(left_motor_);
}

std::shared_ptr<Motor> HAT::getRightMotor() {
  right_motor_ = std::make_shared<Motor>(
    "right",
    pwm_,
    33,
    31,
    13,
    MotorDirectionControl::GPIO
  );
  return std::move(right_motor_);
}

  } // namespace sensors
} // namespace xiaoduckie
