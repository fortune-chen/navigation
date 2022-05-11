#ifndef FLSLAM_SRC_COLLECTOR_SRC_COLLECTOR_DDS_CLIENT_H
#define FLSLAM_SRC_COLLECTOR_SRC_COLLECTOR_DDS_CLIENT_H

#include <vector>
#include <memory>
#include <functional>

#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include "dds_publisher.h"
#include "dds_subscriber.h"

namespace collector {

using TwistCallbackType = std::function<void(const geometry_msgs::Twist&)>;

class CollectorDDSClient {
 public:
  CollectorDDSClient();

  ~CollectorDDSClient();

  CollectorDDSClient(const CollectorDDSClient&) = delete;

  CollectorDDSClient& operator=(const CollectorDDSClient&) = delete;

  // start() with sensor flag, 0x01(laser), 0x02(imu), 0x04(odom),0x08(gnss)
  void startSensor(unsigned char sensor);

  unsigned char sensorStarted();

  void postImu(const sensor_msgs::Imu&);

  void postLaser(const sensor_msgs::LaserScan&);

  void postOdom(const nav_msgs::Odometry&);

  void postGnss(const sensor_msgs::NavSatFix&);

  void registerTwistCallback(const TwistCallbackType& cb);

  TwistCallbackType m_TwistCallback;

 private:
  std::shared_ptr<DDSSubscriber> m_twistSubscribers;

  std::unique_ptr<DDSPublisher> m_imuPublisher;

  std::unique_ptr<DDSPublisher> m_laserPublisher;

  std::unique_ptr<DDSPublisher> m_odomPublisher;

  std::unique_ptr<DDSPublisher> m_gnssPublisher;

  unsigned char m_started;
};

}  // namespace collector

#endif  // FLSLAM_SRC_COLLECTOR_SRC_COLLECTOR_DDS_CLIENT_H
