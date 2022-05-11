#ifndef FLSLAM_SRC_COLLECTOR_SRC_IMU_IMU_NODE_H
#define FLSLAM_SRC_COLLECTOR_SRC_IMU_IMU_NODE_H

#include "boost/thread.hpp"
#include "sensor_msgs/Imu.h"

#include "adxl345.h"
#include "collector_dds_client.h"

namespace collector
{

class ImuNode {
public:
  Imu m_imu;
  std::string m_device;
  std::string m_frame_id;
  int m_delay;
  boost::thread* m_read_thread;
  std::shared_ptr<CollectorDDSClient> m_ddsClient;

  explicit ImuNode(std::shared_ptr<CollectorDDSClient> dds_client, int delay /*ms*/);

  ~ImuNode();

  bool is_opened(void);

  bool open(void);

  int publish_imu_data();

  void readThread();

};


}

#endif // FLSLAM_SRC_COLLECTOR_SRC_IMU_IMU_NODE_H