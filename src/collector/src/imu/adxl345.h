#ifndef FLSLAM_SRC_COLLECTOR_SRC_IMU_ADXL345_H
#define FLSLAM_SRC_COLLECTOR_SRC_IMU_ADXL345_H

#include <string>
#include <termios.h>

namespace collector
{

class Imu {
public:

  int fd_;

  struct termios defaults_;

  double accl[3];

  Imu();
  char read_address(char address);
  int16_t read_short(char address);
  char write_address(char address, char data);
  int open_device(const std::string device);
  void close_device();
  int get_product_id(unsigned char& data);
  int get_seq_count(int16_t& data);
  int update(void);
};

}

#endif // FLSLAM_SRC_COLLECTOR_SRC_IMU_ADXL345_H