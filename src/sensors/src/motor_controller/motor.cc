#include <map>
#include <utility>
#include <memory>
#include <JetsonGPIO.h>

#include <sensors/hat/pwm.h>
#include <sensors/motor_controller/motor.h>

namespace xiaoduckie {
  namespace sensors {

AbstractMotorDirectionController::AbstractMotorDirectionController(int in1_pin, int in2_pin)
  : in1_pin_(in1_pin), in2_pin_(in2_pin) {};

PWMMotorDirectionController::PWMMotorDirectionController(int in1_pin, int in2_pin, std::shared_ptr<PWM> pwm)
  : AbstractMotorDirectionController(in1_pin, in2_pin) {
  pwm_ = pwm;
};

void PWMMotorDirectionController::set(MotorDirection direction) {
  std::pair<HIGH_LOW, HIGH_LOW> signals = _DIRECTION_TO_SIGNALS[direction];
  auto in1_signal = signals.first;
  auto in2_signal = signals.second;
  auto in1_value = _PWM_VALUES[in1_signal];
  auto in2_value = _PWM_VALUES[in2_signal];
  pwm_.get()->setPWM(in1_pin_, in1_value.first, in1_value.second);
  pwm_.get()->setPWM(in2_pin_, in2_value.first, in2_value.second);
}

void GPIOMotorDirectionController::setup() {
  GPIO::setmode(GPIO::BOARD);
  GPIO::setup(in1_pin_, GPIO::OUT);
  GPIO::setup(in2_pin_, GPIO::OUT);
}

void GPIOMotorDirectionController::set(MotorDirection direction) {
  std::pair<HIGH_LOW, HIGH_LOW> signals = _DIRECTION_TO_SIGNALS[direction];
  auto in1_signal = signals.first;
  auto in2_signal = signals.second;
  GPIO::output(in1_pin_, static_cast<int>(in1_signal));
  GPIO::output(in2_pin_, static_cast<int>(in2_signal));

}

Motor::Motor(std::string name, std::shared_ptr<PWM> pwm, int in1_pin, int in2_pin, int pwm_pin, MotorDirectionControl control)
 : name_(name),
  pwm_(pwm),
  in1_pin_(in1_pin),
  in2_pin_(in2_pin),
  pwm_pin_(pwm_pin),
  direction_(control)
{
  switch (control) {
    case MotorDirectionControl::PWM:
      controller_ = std::make_shared<PWMMotorDirectionController>(in1_pin_, in2_pin_, pwm_);
      break;
    case MotorDirectionControl::GPIO:
      controller_ = std::make_shared<GPIOMotorDirectionController>(in1_pin, in2_pin);
      break;
  }
}

void Motor::set(MotorDirection direction, int speed) {
  speed = std::max(0, std::min(speed, 255));
  controller_.get()->set(direction);
  pwm_.get()->setPWM(pwm_pin_, 0, speed * _K);
}

  } // namespace sensors
} // namespace xiaoduckie
