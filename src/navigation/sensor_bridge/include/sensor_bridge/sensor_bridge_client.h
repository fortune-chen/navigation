#ifndef _SENSOR_BRIDGE_CLIENT_H
#define _SENSOR_BRIDGE_CLIENT_H

#include <vector>
#include <memory>
#include <functional>

// #include "tcpsocket.hpp"
#include "udpsocket.hpp"
#include "sensor_bridge.h"

#include "dds_publisher.h"
#include "dds_subscriber.h"

class SensorBridgeClient {
 public:
  SensorBridgeClient(const std::string& localip, const std::string& bridge_ip);
  ~SensorBridgeClient();
  SensorBridgeClient(const SensorBridgeClient&) = delete;
  SensorBridgeClient& operator=(const SensorBridgeClient&) = delete;

  void registerGridMapCallback(const GridMapCallbackType&);
  void registerGoalCallback(const GoalCallbackType&);
  void registerOdomCallback(const OdomCallbackType& cb);
  void registerLaserScanCallback(const LaserScanCallbackType& cb);
  void registerFullCoverageCallback(const FullCoverageCallbackType&);
  void registerGlobalPathCallback(const GlobalPathCallbackType&);

  void sendGlobalPath(const nav_msgs::Path&);
  void sendDWAGlobalPath(const nav_msgs::Path&);
  void sendDWALocalPath(const nav_msgs::Path&);
  void sendVelocity(const geometry_msgs::Twist& vel_cmd);
  void sendFootprint(const geometry_msgs::PolygonStamped& footprint);
  void sendLocalCostmap(const nav_msgs::OccupancyGrid& map);

  OdomCallbackType odomCallback_;
  LaserScanCallbackType laserScanCallback_;
  ImuCallbackType imuCallback_;
  GridMapCallbackType gridMapCallback_;
  GoalCallbackType goalCallback_;
  FullCoverageCallbackType fullCoverageCallback_;
  GlobalPathCallbackType globalPathCallback_;

 private:
  //std::unique_ptr<TCPSocket> tcpSocket_;

  std::vector<std::shared_ptr<DDSSubscriber>> subscribers_;

  std::unique_ptr<DDSPublisher> global_plan_publisher_;

  std::unique_ptr<DDSPublisher> dwa_global_plan_publisher_;

  std::unique_ptr<DDSPublisher> dwa_local_plan_publisher_;

  std::unique_ptr<DDSPublisher> local_costmap_publisher_;

  std::unique_ptr<DDSPublisher> velocity_publisher_;

  std::unique_ptr<DDSPublisher> footprint_publisher_;

  std::unique_ptr<UDPSocket> udpSocket_;
};

#endif  // _SENSOR_BRIDGE_CLIENT_H
