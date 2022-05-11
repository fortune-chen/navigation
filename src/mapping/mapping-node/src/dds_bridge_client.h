#ifndef FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_DDS_BRIDGE_CLIENT_H
#define FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_DDS_BRIDGE_CLIENT_H

#include <vector>
#include <memory>
#include <functional>

#include "udpsocket.hpp"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/NavSatFix.h>
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"

#include "dds_publisher.h"
#include "dds_subscriber.h"

namespace mapping_node {

using OdomCallbackType = std::function<void(const nav_msgs::OdometryConstPtr&)>;
#if USED_MULTI_ECHO_SCAN
using LaserScanCallbackType = std::function<void(const sensor_msgs::MultiEchoLaserScanConstPtr&)>;
#else
using LaserScanCallbackType = std::function<void(const sensor_msgs::LaserScanConstPtr&)>;
#endif
using ImuCallbackType = std::function<void(const sensor_msgs::ImuConstPtr&)>;
using GnssCallbackType = std::function<void(const sensor_msgs::NavSatFixConstPtr&)>;
using ControlCommandCallbackType = std::function<void(int, std::string&)>;

class DDSBridgeClient {
 public:
  DDSBridgeClient(const std::string& localip, const std::string& bridge_ip);

  ~DDSBridgeClient();

  DDSBridgeClient(const DDSBridgeClient&) = delete;

  DDSBridgeClient& operator=(const DDSBridgeClient&) = delete;

  // start() with sensor flag, 0x01(laser), 0x02(imu), 0x04(odom),0x08(gnss)
  void startSensor(unsigned char sensor);

  unsigned char sensorStarted();

  void postGridMap(const nav_msgs::OccupancyGrid&);

  void postPose(const geometry_msgs::PoseStamped&);

  void registerOdomCallback(const OdomCallbackType& cb);

  void registerLaserScanCallback(const LaserScanCallbackType& cb);

  void registerImuCallback(const ImuCallbackType& cb);

  void registerGnssCallback(const GnssCallbackType& cb);

  void registerControlCommandCallback(const ControlCommandCallbackType& cb);

  OdomCallbackType m_odomCallback;

  LaserScanCallbackType m_laserScanCallback;

  ImuCallbackType m_imuCallback;

  GnssCallbackType m_GnssCallback;

  ControlCommandCallbackType m_controlCommandCallback;

 private:
  std::vector<std::shared_ptr<DDSSubscriber>> m_subscribers;

  std::unique_ptr<DDSPublisher> m_mapPublisher;

  std::unique_ptr<DDSPublisher> m_posePublisher;

  std::unique_ptr<UDPSocket> m_udpSocket;

  unsigned char m_started;
};

}  // namespace mapping_node

#endif  // FLSLAM_SRC_MAPPING_MAPPING_NODE_SRC_DDS_BRIDGE_CLIENT_H
