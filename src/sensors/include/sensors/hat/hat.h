#pragma once

#include <sensors/hat/pwm.h>
#include <sensors/motor_controller/motor.h>

namespace xiaoduckie {
  namespace sensors {

class HAT {
public:
  HAT(int i2c_address=0x60, double frequency=1600.0);

  std::shared_ptr<Motor> getLeftMotor();

  std::shared_ptr<Motor> getRightMotor();

private:
  int i2c_address_;
  double frequency_;

  int right_motor_in1_pin = 10;
  int right_motor_in2_pin = 9;
  int right_motor_pwm_pin = 8;
  
  std::shared_ptr<PWM> pwm_ = nullptr;
  std::shared_ptr<Motor> left_motor_ = nullptr;
  std::shared_ptr<Motor> right_motor_ = nullptr;
};

  } // namespace sensors
} // namespace xiaoduckie