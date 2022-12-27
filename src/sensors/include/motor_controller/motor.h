#include <map>
#include <utility>
#include <memory>
#include <motor_controller/pwm.h>

namespace xiaoduckie {
  namespace sensors {

enum class HIGH_LOW {
  LOW = 0,
  HIGH = 1
};

enum class MotorDirection {
  RELEASE = 0,
  FORWARD = 1,
  BACKWARD = -1
};

enum class MotorDirectionControl {
  PWM = 0,
  GPIO = 1
};

struct MotorPis
{
  /* data */
  int in1;
  int in2;
  int pwm;
  MotorDirectionControl control;
};


class AbstractMotorDirectionController {
public:
  AbstractMotorDirectionController(int in1_pin, int int2_pin)
  : int1_pin_(in1_pin), int2_pin_(int2_pin) {};
  virtual void setup() = 0;
  virtual void set(MotorDirection direction) = 0;
protected:
int int1_pin_;
int int2_pin_;
};

class PWMMotorDirectionController: public AbstractMotorDirectionController {
public:
  std::map<MotorDirection, std::pair<HIGH_LOW, HIGH_LOW>> _DIRECTION_TO_SIGNALS = {
    {MotorDirection::RELEASE, std::make_pair(HIGH_LOW::LOW, HIGH_LOW::HIGH)},
    {MotorDirection::FORWARD, std::make_pair(HIGH_LOW::HIGH, HIGH_LOW::LOW)},
    {MotorDirection::BACKWARD, std::make_pair(HIGH_LOW::LOW, HIGH_LOW::HIGH)}
  };
  std::map<HIGH_LOW, std::pair<PWM_HIGH_LOW, PWM_HIGH_LOW>> _PWM_VALUES = {
    {HIGH_LOW::LOW, std::make_pair(PWM_HIGH_LOW::LOW, PWM_HIGH_LOW::HIGH)},
    {HIGH_LOW::HIGH, std::make_pair(PWM_HIGH_LOW::HIGH, PWM_HIGH_LOW::LOW)},
  };

  std::shared_ptr<PWM> pwm_;
  
  PWMMotorDirectionController(int in1_pin, int int2_pin, std::shared_ptr<PWM> pwm)
  : AbstractMotorDirectionController(in1_pin, int2_pin) {
    pwm_ = std::move(pwm);
  };

  void set(MotorDirection direction) {
    std::pair<HIGH_LOW, HIGH_LOW> signals = _DIRECTION_TO_SIGNALS[direction];
    auto in1_signal = signals.first;
    auto in2_signal = signals.second;

    auto in1_value = _PWM_VALUES[in1_signal];
    auto in2_value = _PWM_VALUES[in2_signal];
  }

};





class Motor {
public:
  int _K = 16;

private:
};
  }
}