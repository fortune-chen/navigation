#ifndef __SENSOR_RECEIVER__
#define __SENSOR_RECEIVER__

#define USE_BOOST_FUNCTION

#ifdef USE_BOOST_FUNCTION
#include <boost/function.hpp>
#define MyFunction boost::function
#else
#include <functional>
#define MyFunction std::function
#endif

#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PolygonStamped.h>

template<typename M>
using MsgConstPtr = boost::shared_ptr<const M>;

template<typename M>
using MessageCallback = MyFunction<void(const MsgConstPtr<M>&)>;

using OdomCallbackType = MessageCallback<nav_msgs::Odometry>;
#if USED_MULTI_ECHO_SCAN
using LaserScanCallbackType = MessageCallback<sensor_msgs::MultiEchoLaserScan>;
#else
using LaserScanCallbackType = MessageCallback<sensor_msgs::LaserScan>;
#endif
using ImuCallbackType = MessageCallback<sensor_msgs::Imu>;
using GnssCallbackType = MessageCallback<sensor_msgs::NavSatFix>;
using GridMapCallbackType = MessageCallback<nav_msgs::OccupancyGrid>;
using GoalCallbackType = MessageCallback<geometry_msgs::PoseStamped>;

#ifdef USED_ROS_TRANSFORM
#include <tf2_msgs/TFMessage.h>
using TransformCallbackType = MessageCallback<tf2_msgs::TFMessage>;
#endif

template<typename T>
class DisableCopy {
public:
    DisableCopy() = default;
    DisableCopy(const DisableCopy&) = delete;
    DisableCopy& operator=(const DisableCopy&) = delete;
};

// spin with background thread
class SensorBridge: public DisableCopy<SensorBridge> {
public:
    static SensorBridge& getInstance(const std::string& ip = "", const uint16_t port = 0);

    virtual ~SensorBridge() = default;

    virtual bool registerOdomCallback(const OdomCallbackType& cb) = 0;
    virtual bool registerLaserScanCallback(const LaserScanCallbackType& cb) = 0;
    virtual bool registerImuCallback(const ImuCallbackType& cb) = 0;
    virtual bool registerGnssCallback(const GnssCallbackType& cb) = 0;

    virtual bool sendGridMap(const nav_msgs::OccupancyGrid& map) = 0;
    
    virtual bool registerGridMapCallback(const GridMapCallbackType&) = 0;
    virtual bool registerGoalCallback(const GoalCallbackType&) = 0;

#ifdef USED_ROS_TRANSFORM
    virtual bool registerTransformCallback(const TransformCallbackType&) {};
#endif

    virtual bool sendGlobalPath(const nav_msgs::Path&) = 0;
    virtual void sendDWAGlobalPath(const nav_msgs::Path&) = 0;
    virtual void sendDWALocalPath(const nav_msgs::Path&) = 0;
    virtual bool sendPose(const geometry_msgs::PoseStamped&) = 0;
    virtual void sendFootPrint(const geometry_msgs::PolygonStamped&) = 0;
    virtual void sendLocalCostmap(const nav_msgs::OccupancyGrid&) = 0;
    virtual void sendVelocity(const geometry_msgs::Twist&) = 0;
};

#endif // __SENSOR_RECEIVER__
