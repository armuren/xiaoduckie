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
  std::cout << "<<<<<<<<<<<<<<<<<<<<<<<<<<< IN PWMMotorDirectionController constructor " << std::endl;
  pwm_ = pwm;
  std::cout << "PWMMotorDirectionController: address of pwm_ " << pwm_ << std::endl;
};

void PWMMotorDirectionController::set(MotorDirection direction) {
  std::cout << "PWM Motor Direction Controller: setting direction --";
  std::pair<HIGH_LOW, HIGH_LOW> signals = _DIRECTION_TO_SIGNALS[direction];
  auto in1_signal = signals.first;
  auto in2_signal = signals.second;
  std::cout << " -- in1_signal " << (int) in1_signal << " and in2_signal " << (int) in2_signal << std::endl;
  auto in1_value = _PWM_VALUES[in1_signal];
  auto in2_value = _PWM_VALUES[in2_signal];
  std::cout << " -- in1_value " << in1_value.first << " and in2_value " << in2_value.second << std::endl;
  pwm_.get()->setPWM(in1_pin_, in1_value.first, in1_value.second);
  std::cout << "----------------------------------------------------------------" << std::endl;
  pwm_.get()->setPWM(in2_pin_, in2_value.first, in2_value.second);
  std::cout << "_PWM_VALUES _PWM_VALUES _PWM_VALUES _PWM_VALUES " << std::endl;
}

void GPIOMotorDirectionController::setup() {
  GPIO::setmode(GPIO::BOARD);
  GPIO::setup(in1_pin_, GPIO::OUT);
  GPIO::setup(in2_pin_, GPIO::OUT);
}

void GPIOMotorDirectionController::set(MotorDirection direction) {
  std::cout << "GPIO Motor Direction Controller: setting direction --";
  std::pair<HIGH_LOW, HIGH_LOW> signals = _DIRECTION_TO_SIGNALS[direction];
  auto in1_signal = signals.first;
  auto in2_signal = signals.second;
  std::cout << " -- in1_signal " << (int) in1_signal << " and in2_signal " << (int) in2_signal << std::endl;
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
  std::cout << "Motor::Motor: address of pwm_ " << pwm_ << std::endl;
  switch (control) {
    case MotorDirectionControl::PWM:
      std::cout << " >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << std::endl;
      controller_ = std::make_shared<PWMMotorDirectionController>(in1_pin_, in2_pin_, pwm_);
      break;
    case MotorDirectionControl::GPIO:
      controller_ = std::make_shared<GPIOMotorDirectionController>(in1_pin, in2_pin);
      break;
  }
  std::cout << "WHAT WHAT WHAT WHAT" << controller_ << std::endl;
}

void Motor::set(MotorDirection direction, int speed) {
  speed = std::max(0, std::min(speed, 255));
  std::cout << "???????????????? Motor Controller : setting pwm speed to " << speed << std::endl;
  std::cout << (controller_) << std::endl;
  controller_.get()->set(direction);
  std::cout << "DIRECTION HAS BEEN SET" << std::endl;
  std::cout << " address of pwm_ " << pwm_ << std::endl;
  pwm_.get()->setPWM(pwm_pin_, 0, speed * _K);
}

  } // namespace sensors
} // namespace xiaoduckie
