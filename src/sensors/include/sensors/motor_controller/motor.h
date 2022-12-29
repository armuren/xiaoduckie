#pragma once

#include <map>
#include <utility>
#include <memory>
#include <sensors/hat/pwm.h>
#include <JetsonGPIO.h>

namespace xiaoduckie {
  namespace sensors {

const static int PWM_LOW = 0;
const static int PWM_HIGH = 4096;

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

struct MotorPins
{
  /* data */
  int in1;
  int in2;
  int pwm;
  MotorDirectionControl control;
};


class AbstractMotorDirectionController {
public:
  AbstractMotorDirectionController(int in1_pin, int in2_pin);
  
  virtual void set(MotorDirection direction) = 0;
protected:
int in1_pin_;
int in2_pin_;
};

class PWMMotorDirectionController: public AbstractMotorDirectionController {
public:
  std::string test = "PWMMotorDirectionController";
  std::map<MotorDirection, std::pair<HIGH_LOW, HIGH_LOW>> _DIRECTION_TO_SIGNALS = {
    {MotorDirection::RELEASE, std::make_pair(HIGH_LOW::LOW, HIGH_LOW::HIGH)},
    {MotorDirection::FORWARD, std::make_pair(HIGH_LOW::HIGH, HIGH_LOW::LOW)},
    {MotorDirection::BACKWARD, std::make_pair(HIGH_LOW::LOW, HIGH_LOW::HIGH)}
  };
  std::map<HIGH_LOW, std::pair<int, int>> _PWM_VALUES = {
    {HIGH_LOW::LOW, std::make_pair(PWM_LOW, PWM_HIGH)},
    {HIGH_LOW::HIGH, std::make_pair(PWM_HIGH, PWM_LOW)},
  };

  std::shared_ptr<PWM> pwm_;
  
  PWMMotorDirectionController(int in1_pin, int in2_pin, std::shared_ptr<PWM> pwm);

  void set(MotorDirection direction);

};

class GPIOMotorDirectionController: public AbstractMotorDirectionController {
public:
  std::map<MotorDirection, std::pair<HIGH_LOW, HIGH_LOW>> _DIRECTION_TO_SIGNALS = {
    {MotorDirection::RELEASE, std::make_pair(HIGH_LOW::HIGH, HIGH_LOW::HIGH)},
    {MotorDirection::FORWARD, std::make_pair(HIGH_LOW::HIGH, HIGH_LOW::LOW)},
    {MotorDirection::BACKWARD, std::make_pair(HIGH_LOW::LOW, HIGH_LOW::HIGH)}
  };

  GPIOMotorDirectionController(int in1_pin, int in2_pin) 
  : AbstractMotorDirectionController(in1_pin, in2_pin) {
    setup();
  };

  void setup();

  void set(MotorDirection direction);
  
};


class Motor {
public:
  int _K = 16;
  std::string name_;
  std::shared_ptr<PWM> pwm_;
  int in1_pin_;
  int in2_pin_;
  int pwm_pin_;
  MotorDirectionControl direction_;

  std::shared_ptr<AbstractMotorDirectionController> controller_ = nullptr;

  Motor(std::string name, std::shared_ptr<PWM> pwm, int in1_pin, int in2_pin, int pwm_pin, MotorDirectionControl control);

  void set(MotorDirection direction, int speed = 0);

private:
};

  }
}