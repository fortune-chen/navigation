#ifndef __ROS_DATA__
#define __ROS_DATA__

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/MultiEchoLaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Image.h"

enum ROSDataType {
    ODOM = 0,
    SCAN,
    IMU,
    MAP,
    IMAGE,
    PATH,
    POSE,
    VELOCITY,
    POLYGON,
    GNSS,
};

enum NodeControlCommandType {
    Mapping = 1,
    Navigation,
};

const std::string kLaserScanTopicName = "fl_scan";
const std::string kImuTopicName = "fl_imu";
const std::string kOdometryTopicName = "fl_odom";
const std::string kGnssTopicName = "fl_gnss";

const std::string kGlobalMapTopicName = "fl_global_map";
const std::string kLocalMapTopicName = "fl_local_map";
const std::string kNavMapTopicName = "fl_nav_map";

const std::string kGlobalPlanTopicName = "fl_global_plan";
const std::string kDWAGlobalPlanTopicName = "fl_dwa_global_plan";
const std::string kDWALocalPlanTopicName = "fl_dwa_local_plan";

const std::string kGoalPoseStampedTopicName = "fl_nav_goal";
const std::string kPoseStampedTopicName = "fl_posestamped";
const std::string kPolygonStampedTopicName = "fl_polygonstampedd";
const std::string kCoveragePoseStampedTopicName = "fl_cov_init";

const std::string kVelocityTopicName = "fl_velocity";
const std::string kTrajectoryTopicName = "fl_trajectory";
const std::string kTeleopKeyTopicName = "fl_teleop_key";

const std::string kNodeControlCommandTopicName = "fl_node_control";

const unsigned short kRosBridgeTopicPort      = 51880;
const unsigned short kMappingTopicPort        = 51881;
const unsigned short kNavigationTopicPort     = 51882;
const unsigned short kTrimmerTopicPort        = 51883;

template<typename M>
struct MessageTraits {
    // default, no type member
};

template<>
struct MessageTraits<nav_msgs::Odometry> {
    const static ROSDataType type = ROSDataType::ODOM;
};

#if USED_MULTI_ECHO_SCAN
template<>
struct MessageTraits<sensor_msgs::MultiEchoLaserScan> {
    const static ROSDataType type = ROSDataType::SCAN;
};
#else
template<>
struct MessageTraits<sensor_msgs::LaserScan> {
    const static ROSDataType type = ROSDataType::SCAN;
};
#endif

template<>
struct MessageTraits<sensor_msgs::Imu> {
    const static ROSDataType type = ROSDataType::IMU;
};

template<>
struct MessageTraits<nav_msgs::OccupancyGrid> {
    const static ROSDataType type = ROSDataType::MAP;
};

template<>
struct MessageTraits<sensor_msgs::Image> {
    const static ROSDataType type = ROSDataType::IMAGE;
};

template<>
struct MessageTraits<nav_msgs::Path> {
    const static ROSDataType type = ROSDataType::PATH;
};

template<>
struct MessageTraits<geometry_msgs::PoseStamped> {
    const static ROSDataType type = ROSDataType::POSE;
};

template<>
struct MessageTraits<geometry_msgs::Twist> {
    const static ROSDataType type = ROSDataType::VELOCITY;
};

template<>
struct MessageTraits<geometry_msgs::PolygonStamped> {
    const static ROSDataType type = ROSDataType::POLYGON;
};

template<>
struct MessageTraits<sensor_msgs::NavSatFix> {
    const static ROSDataType type = ROSDataType::GNSS;
};

#endif // __ROS_DATA__