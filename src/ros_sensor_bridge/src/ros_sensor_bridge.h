#ifndef __ROS_SENSOR_RECEIVER__
#define __ROS_SENSOR_RECEIVER__

#include <memory>
#include "ros/ros.h"
#include "sensor_bridge.h"

// spin with background thread
class RosSensorBridge: public SensorBridge {
public:
    RosSensorBridge();
    ~RosSensorBridge();

    bool registerOdomCallback(const OdomCallbackType& cb) override;
    bool registerLaserScanCallback(const LaserScanCallbackType& cb) override;
    bool registerImuCallback(const ImuCallbackType& cb) override;
    bool registerGnssCallback(const GnssCallbackType& cb) override;

    bool sendGridMap(const nav_msgs::OccupancyGrid& map) override;

    bool registerGridMapCallback(const GridMapCallbackType&) override;
    bool registerGoalCallback(const GoalCallbackType&) override;

#ifdef USED_ROS_TRANSFORM
    bool registerTransformCallback(const TransformCallbackType&) override;
#endif

    bool sendGlobalPath(const nav_msgs::Path&) override;
    void sendDWAGlobalPath(const nav_msgs::Path&) override;
    void sendDWALocalPath(const nav_msgs::Path&) override;
    bool sendPose(const geometry_msgs::PoseStamped&) override;
    void sendLocalCostmap(const nav_msgs::OccupancyGrid&) override;
    void sendFootPrint(const geometry_msgs::PolygonStamped&) override;
    void sendVelocity(const geometry_msgs::Twist&) override;

private:
    std::unique_ptr<ros::NodeHandle> m_pNodeHandle;
    ros::Subscriber m_odomSub;
    ros::Subscriber m_scanSub;
    ros::Subscriber m_imuSub;
    ros::Subscriber m_gnssSub;
    ros::Publisher m_mapPub;

    ros::Subscriber m_mapSub;
    ros::Subscriber m_goalSub;
    ros::Publisher m_GlobalPlanPub;
    ros::Publisher m_DWAGlobalPlanPub;
    ros::Publisher m_DWALocalPlanPub;
    ros::Publisher m_posePub;
    ros::Publisher m_LocalCostmapPub;
    ros::Publisher m_FootPrintPub;
    ros::Publisher m_VelocityPub;

#ifdef USED_ROS_TRANSFORM
    ros::Subscriber m_tfSub;
#endif
};


#endif // __ROS_SENSOR_RECEIVER__
