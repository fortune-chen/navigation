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
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PolygonStamped.h"

template<typename M>
using MsgConstPtr = boost::shared_ptr<const M>;

template<typename M>
using MessageCallback = MyFunction<void(const MsgConstPtr<M>&)>;

using OdomCallbackType = MessageCallback<nav_msgs::Odometry>;
using LaserScanCallbackType = MessageCallback<sensor_msgs::LaserScan>;
using ImuCallbackType = MessageCallback<sensor_msgs::Imu>;
using GridMapCallbackType = MessageCallback<nav_msgs::OccupancyGrid>;
using GoalCallbackType = MessageCallback<geometry_msgs::PoseStamped>;
using FullCoverageCallbackType = MessageCallback<geometry_msgs::PoseStamped>;
using GlobalPathCallbackType = MessageCallback<nav_msgs::Path>;

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

    virtual bool sendGridMap(const nav_msgs::OccupancyGrid& map) = 0;
    
    virtual bool registerGridMapCallback(const GridMapCallbackType&) = 0;
    virtual bool registerGoalCallback(const GoalCallbackType&) = 0;
    virtual bool sendGlobalPath(const nav_msgs::Path&) = 0;

    virtual bool sendVelocity(const geometry_msgs::Twist& vel_cmd) = 0;

    virtual bool sendFootprint(const geometry_msgs::PolygonStamped& footprint) = 0;
};

#endif // __SENSOR_RECEIVER__
