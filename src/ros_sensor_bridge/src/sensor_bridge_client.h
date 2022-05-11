#ifndef _SENSOR_BRIDGE_CLIENT_H
#define _SENSOR_BRIDGE_CLIENT_H

#include <memory>
#include <functional>

// #define USE_TCP
#ifdef USE_TCP
#include "tcpsocket.hpp"
#else
#include "udpsocket.hpp"
#endif

#include "sensor_bridge.h"

class SensorBridgeClient: public SensorBridge {
 public:
  SensorBridgeClient(const std::string& ip, const uint16_t port);
  ~SensorBridgeClient();

  bool sendGridMap(const nav_msgs::OccupancyGrid&) override;

  bool registerOdomCallback(const OdomCallbackType& cb) override;
  bool registerLaserScanCallback(const LaserScanCallbackType& cb) override;
  bool registerImuCallback(const ImuCallbackType& cb) override;
  bool registerGnssCallback(const GnssCallbackType& cb) override;

  bool registerGridMapCallback(const GridMapCallbackType&) override;
  bool registerGoalCallback(const GoalCallbackType&) override;
  bool sendGlobalPath(const nav_msgs::Path&) override;
  void sendDWAGlobalPath(const nav_msgs::Path&) override;
  void sendDWALocalPath(const nav_msgs::Path&) override;
  bool sendPose(const geometry_msgs::PoseStamped&) override;
  void sendLocalCostmap(const nav_msgs::OccupancyGrid&) override;
  void sendFootPrint(const geometry_msgs::PolygonStamped&) override;
  void sendVelocity(const geometry_msgs::Twist&) override;

 private:
  int sensorDataHandler(const std::string& message);
  void sendData(std::string data);
  // void sendData(const char* data, int len);

#ifdef USE_TCP
  TCPSocket socket_;
#else
  UDPSocket socket_;
#endif

  OdomCallbackType odomCallback_;
  LaserScanCallbackType laserScanCallback_;
  ImuCallbackType imuCallback_;
  GridMapCallbackType gridMapCallback_;
  GoalCallbackType goalCallback_;
  GnssCallbackType gnssCallback_;
};

#endif  // _SENSOR_BRIDGE_CLIENT_H
