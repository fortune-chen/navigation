#include <thread>
#include "ros_sensor_bridge.h"

#define ODOM_TOPIC_NAME         "odom"
#if USED_MULTI_ECHO_SCAN
#define LASER_SCAN_TOPIC_NAME   "horizontal_laser_2d"
#else
#define LASER_SCAN_TOPIC_NAME   "scan"
#endif
#define IMU_TOPIC_NAME          "imu"
#define MAP_TOPIC_NAME          "map"
#define GNSS_TOPIC_NAME          "gps"

#define GOAL_TOPIC_NAME             "move_base_simple/goal"
#define GLOBAL_PATH_TOPIC_NAME      "move_base/GlobalPlanner/plan"
#define DWA_GLOBAL_PATH_TOPIC_NAME  "move_base/DWAPlannerROS/global_plan"
#define DWA_LOCAL_PATH_TOPIC_NAME   "move_base/DWAPlannerROS/local_plan"
#define LOCAL_COSTMAP_TOPIC_NAME    "move_base/local_costmap/costmap"
#define FOOT_PRINT_TOPIC_NAME       "move_base/local_costmap/footprint"
#define VELOCITY_TOPIC_NAME         "cmd_vel"
#define POSE_TOPIC_NAME          "tracked_pose"

using namespace std;

SensorBridge& SensorBridge::getInstance(const std::string& ip, const uint16_t port) {
    static RosSensorBridge sInstance;
    return sInstance;
}

RosSensorBridge::RosSensorBridge() {
    int i = 0;
    ros::init(i, nullptr, "sensor_bridge");
    m_pNodeHandle.reset(new ros::NodeHandle);

    m_mapPub = m_pNodeHandle->advertise<nav_msgs::OccupancyGrid>(MAP_TOPIC_NAME, 5);
    assert(m_mapPub);
    m_GlobalPlanPub = m_pNodeHandle->advertise<nav_msgs::Path>(GLOBAL_PATH_TOPIC_NAME, 5);
    assert(m_GlobalPlanPub);
    m_DWAGlobalPlanPub = m_pNodeHandle->advertise<nav_msgs::Path>(DWA_GLOBAL_PATH_TOPIC_NAME, 5);
    assert(m_DWAGlobalPlanPub);
    m_DWALocalPlanPub = m_pNodeHandle->advertise<nav_msgs::Path>(DWA_LOCAL_PATH_TOPIC_NAME, 5);
    assert(m_DWALocalPlanPub);
    m_posePub = m_pNodeHandle->advertise<geometry_msgs::PoseStamped>(POSE_TOPIC_NAME, 5);
    assert(m_posePub);
    m_LocalCostmapPub = m_pNodeHandle->advertise<nav_msgs::OccupancyGrid>(LOCAL_COSTMAP_TOPIC_NAME, 5);
    assert(m_LocalCostmapPub);
    m_FootPrintPub = m_pNodeHandle->advertise<geometry_msgs::PolygonStamped>(FOOT_PRINT_TOPIC_NAME, 5);
    assert(m_FootPrintPub);
    m_VelocityPub = m_pNodeHandle->advertise<geometry_msgs::Twist>(VELOCITY_TOPIC_NAME, 5);
    assert(m_VelocityPub);

    std::thread([] {
        ros::spin();
    }).detach();
}

RosSensorBridge::~RosSensorBridge() {
    // ros::shutdown();
    // m_threadSpin.join();
}

bool RosSensorBridge::registerOdomCallback(const OdomCallbackType& cb) {
    try {
        m_odomSub = m_pNodeHandle->subscribe(ODOM_TOPIC_NAME, 100, cb);
        return m_odomSub;
    } catch (...) {
        cerr << "subscribe failed, topic: " << ODOM_TOPIC_NAME << endl;
        return false;
    }
}

bool RosSensorBridge::registerLaserScanCallback(const LaserScanCallbackType& cb) {
    try {
        m_scanSub = m_pNodeHandle->subscribe(LASER_SCAN_TOPIC_NAME, 100, cb);
        return m_scanSub;
    } catch (...) {
        cerr << "subscribe failed, topic: " << LASER_SCAN_TOPIC_NAME << endl;
        return false;
    }
}

bool RosSensorBridge::registerImuCallback(const ImuCallbackType& cb) {
    try {
        m_imuSub = m_pNodeHandle->subscribe(IMU_TOPIC_NAME, 100, cb);
        return m_imuSub;
    } catch (...) {
        cerr << "subscribe failed, topic: " << IMU_TOPIC_NAME << endl;
        return false;
    }
}

bool RosSensorBridge::registerGnssCallback(const GnssCallbackType& cb) {
    try {
        m_gnssSub = m_pNodeHandle->subscribe(GNSS_TOPIC_NAME, 100, cb);
        return m_gnssSub;
    } catch (...) {
        cerr << "subscribe failed, topic: " << GNSS_TOPIC_NAME << endl;
        return false;
    }
}

bool RosSensorBridge::sendGridMap(const nav_msgs::OccupancyGrid& map) {
    m_mapPub.publish(map);
    return true;
}

bool RosSensorBridge::registerGridMapCallback(const GridMapCallbackType& cb) {
    try {
        m_mapSub = m_pNodeHandle->subscribe(MAP_TOPIC_NAME, 100, cb);
        return m_mapSub;
    } catch (...) {
        cerr << "subscribe failed, topic: " << MAP_TOPIC_NAME << endl;
        return false;
    }
}

bool RosSensorBridge::registerGoalCallback(const GoalCallbackType& cb) {
    try {
        m_goalSub = m_pNodeHandle->subscribe(GOAL_TOPIC_NAME, 100, cb);
        return m_goalSub;
    } catch (...) {
        cerr << "subscribe failed, topic: " << GOAL_TOPIC_NAME << endl;
        return false;
    }
}

#ifdef USED_ROS_TRANSFORM
#define TRANSFORM_TOPIC_NAME     "/tf"
bool RosSensorBridge::registerTransformCallback(const TransformCallbackType& cb) {
    try {
        m_tfSub = m_pNodeHandle->subscribe(TRANSFORM_TOPIC_NAME, 100, cb);
        return m_tfSub;
    } catch (...) {
        cerr << "subscribe failed, topic: " << TRANSFORM_TOPIC_NAME << endl;
        return false;
    }
}
#endif

bool RosSensorBridge::sendGlobalPath(const nav_msgs::Path& path) {
    m_GlobalPlanPub.publish(path);
    return true;
}

void RosSensorBridge::sendDWAGlobalPath(const nav_msgs::Path& path) {
    m_DWAGlobalPlanPub.publish(path);
}

void RosSensorBridge::sendDWALocalPath(const nav_msgs::Path& path) {
    m_DWALocalPlanPub.publish(path);
}

bool RosSensorBridge::sendPose(const geometry_msgs::PoseStamped& pose) {
    m_posePub.publish(pose);
    return true;
}

void RosSensorBridge::sendLocalCostmap(const nav_msgs::OccupancyGrid& map) {
    m_LocalCostmapPub.publish(map);
}

void RosSensorBridge::sendFootPrint(const geometry_msgs::PolygonStamped& footprint) {
    m_FootPrintPub.publish(footprint);
}

void RosSensorBridge::sendVelocity(const geometry_msgs::Twist& velocity) {\
    printf("[velocity] (%lf, %lf)\n", velocity.linear.x, velocity.linear.y);
    m_VelocityPub.publish(velocity);
}