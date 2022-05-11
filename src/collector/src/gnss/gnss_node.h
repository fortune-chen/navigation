#ifndef FLSLAM_SRC_COLLECTOR_SRC_IMU_GNSS_NODE_H
#define FLSLAM_SRC_COLLECTOR_SRC_IMU_GNSS_NODE_H

#include "boost/thread.hpp"
#include "sensor_msgs/NavSatFix.h"

#include "collector_dds_client.h"

namespace collector
{

typedef struct {
    int32_t                    id;
} GPSDriverNMEA;

class GnssNode {
public:
  GPSDriverNMEA m_gnss;

  int m_delay;
  boost::thread* m_read_thread;
  std::shared_ptr<CollectorDDSClient> m_ddsClient;

  explicit GnssNode(std::shared_ptr<CollectorDDSClient> dds_client, int delay /*ms*/);

  ~GnssNode();

  bool is_opened(void);

  bool open(void);

  int publish_gnss_data();

  void readThread();

};


}

#endif // FLSLAM_SRC_COLLECTOR_SRC_IMU_GNSS_NODE_H