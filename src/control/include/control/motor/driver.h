#include <memory>
#include <math.h>
#include <algorithm>

#include <sensors/hat/hat.h>

namespace xiaoduckie {
  namespace control {

class Driver
{
private:
  int LEFT_MOTOR_MIN_PWM = 60;  // Minimum speed for left motor
  int LEFT_MOTOR_MAX_PWM = 255;  // Maximum speed for left motor
  int RIGHT_MOTOR_MIN_PWM = 60;  // Minimum speed for right motor
  int RIGHT_MOTOR_MAX_PWM = 255;  // Maximum speed for right motor
  double SPEED_TOLERANCE = 1.0e-2;  // Speed tolerance level
  
  
  std::shared_ptr<sensors::HAT> hat_ = nullptr;
  std::shared_ptr<sensors::Motor> left_motor_ = nullptr;
  std::shared_ptr<sensors::Motor> right_motor_ = nullptr;

public:
  double velocity_left_;
  double velocity_right_;

  Driver() {
    hat_ = std::make_shared<sensors::HAT>();
    left_motor_ = hat_.get()->getLeftMotor();
    right_motor_ = hat_.get()->getRightMotor();

    velocity_left_ = 0;
    velocity_left_ = 0;

  }

  int getPWMValue(double velocity, double min_pwm, int max_pwm) {
    int pwm = 0;
    if (fabs(velocity) > SPEED_TOLERANCE) {
      pwm = static_cast<int>(floor(fabs(velocity)) *  (max_pwm - min_pwm) + min_pwm);
    }
    else if (fabs(velocity) < SPEED_TOLERANCE) {
      return pwm;
    }
    return std::min(pwm, max_pwm);
  }

  void PWMUpdate() {
    int pwm_left = getPWMValue(velocity_left_, LEFT_MOTOR_MIN_PWM, LEFT_MOTOR_MAX_PWM);
    int pwm_right = getPWMValue(velocity_right_, RIGHT_MOTOR_MIN_PWM, RIGHT_MOTOR_MAX_PWM);

    sensors::MotorDirection left_motor_mode = sensors::MotorDirection::RELEASE;
    sensors::MotorDirection right_motor_mode = sensors::MotorDirection::RELEASE;

    if (velocity_left_ > 0 && pwm_left > 0) {
      left_motor_mode = sensors::MotorDirection::FORWARD;
    }
    else if (velocity_left_ < 0) {
      left_motor_mode = sensors::MotorDirection::BACKWARD;
    }

    if (velocity_right_ > 0 && pwm_right > 0) {
      right_motor_mode = sensors::MotorDirection::FORWARD;
    }
    else if (velocity_right_ < 0) {
      right_motor_mode = sensors::MotorDirection::BACKWARD;
    }

    left_motor_.get()->set(left_motor_mode, pwm_left);
    right_motor_.get()->set(right_motor_mode, pwm_right);

  }
};


  }
}