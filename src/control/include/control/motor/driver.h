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
  
  double velocity_left_;
  double velocity_right_;
  std::shared_ptr<sensors::HAT> hat_ = nullptr;
  std::shared_ptr<sensors::Motor> left_motor_ = nullptr;
  std::shared_ptr<sensors::Motor> right_motor_ = nullptr;

public:

  Driver();

  void setLeftWheelVelocity(double velocity_left);
  void setRightWheelVelocity(double velocity_right);

  int getPWMValue(double velocity, double min_pwm, int max_pwm);

  void PWMUpdate();
};


  }
}