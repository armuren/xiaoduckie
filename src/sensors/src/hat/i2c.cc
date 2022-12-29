extern "C" {
  #include <linux/i2c-dev.h>
  #include <i2c/smbus.h>
  #include <sys/ioctl.h>
}
#include<stdio.h>

#include <sensors/hat/i2c.h>

#include <string>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sstream>

namespace xiaoduckie {
  namespace sensors {

I2C::I2C(int bus_number, int device_address) {
  device_address_ = device_address;
  device_name_ = std::string("/dev/i2c-") + std::to_string(bus_number);
  bus_fd_ = open(device_name_.c_str(), O_RDWR);
  if (bus_fd_ < 0 ) {
    std::string err_msg = std::string("Error: Cannot open i2c bus device ") + device_name_;
    std::cout << err_msg << std::endl;
    throw std::runtime_error(err_msg);
  }
}

void I2C::handle_error(std::string& err_msg) {
  throw std::runtime_error(err_msg);
}

int I2C::put_address_on_bus() {
  int res = ioctl(bus_fd_, I2C_SLAVE, device_address_);
  if (res == -1) {
    std::string error_msg = std::string("I2C: ioctl: Error setting device address " + device_address_);
    handle_error(error_msg);
  }
  return res;
}

void I2C::write8(unsigned char reg, unsigned char value) {
  put_address_on_bus();
  int32_t res = i2c_smbus_write_byte_data(bus_fd_, reg, value);
  if (res == -1) {
    std::string error_msg = std::string("Error  writing to register ");
    error_msg.push_back(reg);
    handle_error(error_msg);
  }
  if (debug) {
    std::cout << "I2C: Device " 
      << device_name_ << ": successfully wrote " 
      << printf("0x%02X", value) << " to register " << printf("0x%02X", reg)
      << " results in >> " << res << std::endl;
  }
};

int32_t I2C::readU8(unsigned char reg) {
  put_address_on_bus();
  int32_t res = i2c_smbus_read_byte_data(bus_fd_, reg);
  if (debug) {
    std::cout << "I2C: Device " 
      << device_name_ 
      << " returned " << printf("0x%02X", res) << " from register " 
      << printf("0x%02X", reg) << std::endl;
  }
  if (res == -1) {
    std::string error_msg = std::string("Error reading from register ");
    error_msg.push_back(reg);
    handle_error(error_msg);
  }
  return res;
};

  } // namespace sensors
} // namespace xiaoduckie
