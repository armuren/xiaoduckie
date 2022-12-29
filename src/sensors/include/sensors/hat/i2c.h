#pragma once

#include <string>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>

namespace xiaoduckie {
  namespace sensors {

class I2C {
public:

  I2C(int bus_number, int device_address);

  void handle_error(std::string& err_msg);

  int put_address_on_bus();

  void write8(unsigned char reg, unsigned char value);

  int32_t readU8(unsigned char reg);

private:
  int bus_number_;
  int bus_fd_;
  int device_address_;
  std::string device_name_;
  bool debug = false;
};

  }
}