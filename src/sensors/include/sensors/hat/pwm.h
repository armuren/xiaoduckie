#pragma once

#include <memory>
#include <thread>
#include <chrono>
#include <sensors/hat/i2c.h>
#include <math.h>

namespace xiaoduckie {
  namespace sensors {

class PWM {
private :
  // Registers/etc.
  int __MODE1 = 0x00;
  int __MODE2 = 0x01;
  int __SUBADR1 = 0x02;
  int __SUBADR2 = 0x03;
  int __SUBADR3 = 0x04;
  int __PRESCALE = 0xFE;
  int __LED0_ON_L = 0x06;
  int __LED0_ON_H = 0x07;
  int __LED0_OFF_L = 0x08;
  int __LED0_OFF_H = 0x09;
  int __ALL_LED_ON_L = 0xFA;
  int __ALL_LED_ON_H = 0xFB;
  int __ALL_LED_OFF_L = 0xFC;
  int __ALL_LED_OFF_H = 0xFD;

  // Bits
  int __RESTART = 0x80;
  int __SLEEP = 0x10;
  int __ALLCALL = 0x01;
  int __INVRT = 0x10;
  int __OUTDRV = 0x04;

  bool debug_ = true;
  std::shared_ptr<I2C> i2c_ = nullptr;

public:
  PWM(int i2c_address);

  void setPWM(int channel, int on, int off);

  void setPWMFreq(double freq);

  void setAllPWM(int on, int off);
};


  }
}