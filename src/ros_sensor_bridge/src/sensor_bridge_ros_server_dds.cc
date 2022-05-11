#include <iostream>
#include <chrono>
#include "sensor_bridge.h"

#include "dds_publisher.h"
#include "dds_subscriber.h"
#include "RosAdapterContentPubSubTypes.h"

#ifdef USED_ROS_TRANSFORM
#include "ros/ros_type_print.h"
#include "TF2Message.h"
#include "TF2MessagePubSubTypes.h"
#endif

using namespace ros::serialization;

static std::shared_ptr<DDSPublisher> kDDSLaserScanPublisher;
static std::shared_ptr<DDSPublisher> kDDSImuPublisher;
static std::shared_ptr<DDSPublisher> kDDSOdometryPublisher;
static std::shared_ptr<DDSPublisher> kDDSGnssPublisher;
static std::shared_ptr<DDSPublisher> kDDSNavMapPublisher;
static std::shared_ptr<DDSPublisher> kDDSGoalPoseStampedPublisher;

static std::shared_ptr<DDSSubscriber> kGlobalMapSubscriber;
static std::shared_ptr<DDSSubscriber> kLocalCostmapSubscriber;
static std::shared_ptr<DDSSubscriber> kGlobalPlanSubscriber;
static std::shared_ptr<DDSSubscriber> kDWAGlobalPlanSubscriber;
static std::shared_ptr<DDSSubscriber> kDWALocalPlanSubscriber;
static std::shared_ptr<DDSSubscriber> kPoseSubscriber;
static std::shared_ptr<DDSSubscriber> kFootPrintSubscriber;
static std::shared_ptr<DDSSubscriber> kVelocitySubscriber;

int64_t getLocalTimeSeconds() {
  return std::chrono::time_point_cast<std::chrono::seconds>(std::chrono::system_clock::now()).time_since_epoch().count();
}

int64_t getLocalTimeMillis() {
  return std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
}

int64_t getLocalTimeNanos() {
  return std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
}

template<typename M>
void Ros2DDSCallback(const boost::shared_ptr<const M>& msg) {
}
//#if USED_MULTI_ECHO_SCAN
static int scan_count = 0;
static int imu_count = 0;
#if USED_MULTI_ECHO_SCAN
template<>
void Ros2DDSCallback<sensor_msgs::MultiEchoLaserScan>(const boost::shared_ptr<const sensor_msgs::MultiEchoLaserScan>& msg) {
    //if (scan_count++ % 3 == 0) {
        scan_count++;
        kDDSLaserScanPublisher->publishRos<sensor_msgs::MultiEchoLaserScan>(*msg);
    //    return;
    //}
}
#else
template<>
void Ros2DDSCallback<sensor_msgs::LaserScan>(const boost::shared_ptr<const sensor_msgs::LaserScan>& msg) {
    scan_count++;
    kDDSLaserScanPublisher->publishRos<sensor_msgs::LaserScan>(*msg);
}
#endif
template<>
void Ros2DDSCallback<sensor_msgs::Imu>(const boost::shared_ptr<const sensor_msgs::Imu>& msg) {
    kDDSImuPublisher->publishRos<sensor_msgs::Imu>(*msg);
//#if USED_MULTI_ECHO_SCAN
    if (imu_count == 0) {
    std::cout << "ptr->header.stamp: " << msg->header.stamp << std::endl;
    }
    imu_count++;
//#endif
}
template<>
void Ros2DDSCallback<nav_msgs::Odometry>(const boost::shared_ptr<const nav_msgs::Odometry>& msg) {
    kDDSOdometryPublisher->publishRos<nav_msgs::Odometry>(*msg);
}
//#if USED_MULTI_ECHO_SCAN
static int map_count = 0;
//#endif
template<>
void Ros2DDSCallback<nav_msgs::OccupancyGrid>(const boost::shared_ptr<const nav_msgs::OccupancyGrid>& msg) {
    
//#if USED_MULTI_ECHO_SCAN
    std::cout << "map_count: " << map_count++ << " ---> scan_count:" << scan_count << " imu_count:" << imu_count << std::endl;
//#endif
#ifdef USED_NAVIGATION_ONLY
    std::thread([msg]() {
        while (true) {
            std::cout << "[ros_node] send nav map by dds\n";
            kDDSNavMapPublisher->publishMap<nav_msgs::OccupancyGrid>(*msg);
            sleep(5);
        }
    }).detach();
#endif
}
template<>
void Ros2DDSCallback<geometry_msgs::PoseStamped>(const boost::shared_ptr<const geometry_msgs::PoseStamped>& msg) {
    std::cout << "[ros_node] send goal by dds\n";
    kDDSGoalPoseStampedPublisher->publishRos<geometry_msgs::PoseStamped>(*msg);
}

template<>
void Ros2DDSCallback<sensor_msgs::NavSatFix>(const boost::shared_ptr<const sensor_msgs::NavSatFix>& msg) {
    kDDSGnssPublisher->publishRos<sensor_msgs::NavSatFix>(*msg);
}

#ifdef USED_ROS_TRANSFORM
static std::shared_ptr<DDSPublisher> kDDSTransformsPublisher;

template<typename T>
static void serializeToString(const T& in, std::string& out) {
    auto m = ros::serialization::serializeMessage(in);
    if (m.num_bytes > 1024) {
        std::cout << "\n";
        printf("[error] TF2Message->sequence<char, 1024> not big enough, need %zu bytes\n", m.num_bytes);
        assert(false);
    }
    out.assign(reinterpret_cast<char*>(m.message_start), m.num_bytes);
}

static void cast(const tf2_msgs::TFMessage& in, bool isStatic, TF2Message& out) {
    std::string str;
    serializeToString(in, str);
    out.serialized().resize(str.size());
    out.serialized().assign(str.begin(), str.end());
    out.isStatic(isStatic);
}

static void publishTransforms(const tf2_msgs::TFMessage& message, bool isStatic) {
    TF2Message t;
#if 1
    static uint32_t index = 0;
    index++;
    // std::cout << "------------------------ sending TF2Message-index: " << index << std::endl;
    if (index % 100000 == 0) {
        std::cout << "------------------------ sending TF2Message-index: " << index << std::endl;
    }
    t.index(index);
#endif
    cast(message, isStatic, t);
    kDDSTransformsPublisher->publish<TF2Message>(t);
}

template<>
void Ros2DDSCallback<tf2_msgs::TFMessage>(const boost::shared_ptr<const tf2_msgs::TFMessage>& msg) {
    tf2_msgs::TFMessage tf_msg = *msg;
    for (auto& t : tf_msg.transforms) {
        uint32_t s = getLocalTimeSeconds();
        auto ns = getLocalTimeNanos();
        ns = ns % s;
        ros::Time stamp(s, (uint32_t)ns);
        t.header.stamp = stamp;
    }
    publishTransforms(tf_msg, false);

    // just print
    static uint32_t count = 0;
    count++;
    if (count % 50 != 0) {
        return;
    }
    for (const auto& t : tf_msg.transforms) {
        if (t.header.frame_id == "odom" || t.header.frame_id == "map") {
            PRINT_TRANSFORM_STAMPED("transform", t);
        }
    }
}

#endif

template <typename T>
class DDS2RosListener : public TopicListener {
public:
    DDS2RosListener(SensorBridge* bridge, const std::string& topic) : TopicListener((void*)&m_data) {
      m_bridge = bridge;
      m_topic = topic;
    }
    void onTopicReceived(void* msg) override {
        T* map = (T*)msg;
        std::cout << "received data" << std::endl;
    };
    T m_data;
    SensorBridge* m_bridge;
    std::string m_topic;
};
template<>
void DDS2RosListener<nav_msgs::OccupancyGrid>::onTopicReceived(void* msg) {
    nav_msgs::OccupancyGrid map = *(nav_msgs::OccupancyGrid*)msg;
    if (m_topic == kLocalMapTopicName) {
        m_bridge->sendLocalCostmap(map);
    } else {
        m_bridge->sendGridMap(map);
    }
}
template<>
void DDS2RosListener<nav_msgs::Path>::onTopicReceived(void* msg) {
    nav_msgs::Path path = *(nav_msgs::Path*)msg;
    if (m_topic == kGlobalPlanTopicName) {
        std::cout << "[ros_node] received global path by dds\n";
        m_bridge->sendGlobalPath(path);
    } else if (m_topic == kDWAGlobalPlanTopicName) {
        std::cout << "[ros_node] received dwa global path by dds\n";
        m_bridge->sendDWAGlobalPath(path);
    } else if (m_topic == kDWALocalPlanTopicName) {
        std::cout << "[ros_node] received dwa local path by dds\n";
        m_bridge->sendDWALocalPath(path);
    } else {
        std::cout << "[error] unknown\n";
        assert(false);
    }
}
template<>
void DDS2RosListener<geometry_msgs::PoseStamped>::onTopicReceived(void* msg) {
    geometry_msgs::PoseStamped pose = *(geometry_msgs::PoseStamped*)msg;
    //std::cout << "[ros_node] received pose by dds\n";
    m_bridge->sendPose(pose);
}
template<>
void DDS2RosListener<geometry_msgs::PolygonStamped>::onTopicReceived(void* msg) {
    geometry_msgs::PolygonStamped footprint = *(geometry_msgs::PolygonStamped*)msg;
    //std::cout << "[ros_node] received footprint by dds\n";
    m_bridge->sendFootPrint(footprint);
}
template<>
void DDS2RosListener<geometry_msgs::Twist>::onTopicReceived(void* msg) {
    geometry_msgs::Twist velocity = *(geometry_msgs::Twist*)msg;
    m_bridge->sendVelocity(velocity);
}

int main(int argc, char **argv)
{
    std::string localip;
    std::string remoteip;
    auto& bridge = SensorBridge::getInstance();

    if (argc > 2) {
        localip  = argv[1];
        remoteip = argv[2];
    }

    std::shared_ptr<RosAdapterContentPubSubType> ros_type(new RosAdapterContentPubSubType);
    std::shared_ptr<NavMapContentPubSubType> map_type(new NavMapContentPubSubType);
    if (remoteip.empty()) {
        kDDSLaserScanPublisher = std::make_shared<DDSPublisher>(kLaserScanTopicName, ros_type);
        kDDSImuPublisher = std::make_shared<DDSPublisher>(kImuTopicName, ros_type);
        kDDSOdometryPublisher = std::make_shared<DDSPublisher>(kOdometryTopicName, ros_type);
        kDDSGnssPublisher = std::make_shared<DDSPublisher>(kGnssTopicName, ros_type);
        kDDSNavMapPublisher = std::make_shared<DDSPublisher>(kNavMapTopicName, map_type);
        kDDSGoalPoseStampedPublisher = std::make_shared<DDSPublisher>(kGoalPoseStampedTopicName, ros_type);

        kGlobalMapSubscriber = std::make_shared<DDSSubscriber>(kGlobalMapTopicName, map_type,
                                std::make_shared<DDS2RosListener<nav_msgs::OccupancyGrid>>(&bridge, kGlobalMapTopicName));

        // Receive from navigation
        kLocalCostmapSubscriber = std::make_shared<DDSSubscriber>(kLocalMapTopicName, map_type,
                                  std::make_shared<DDS2RosListener<nav_msgs::OccupancyGrid>>(&bridge, kLocalMapTopicName));
        kGlobalPlanSubscriber = std::make_shared<DDSSubscriber>(kGlobalPlanTopicName, map_type,
                                std::make_shared<DDS2RosListener<nav_msgs::Path>>(&bridge, kGlobalPlanTopicName));        
        kDWAGlobalPlanSubscriber = std::make_shared<DDSSubscriber>(kDWAGlobalPlanTopicName, map_type,
                                   std::make_shared<DDS2RosListener<nav_msgs::Path>>(&bridge, kDWAGlobalPlanTopicName));
        kDWALocalPlanSubscriber = std::make_shared<DDSSubscriber>(kDWALocalPlanTopicName, map_type,
                                  std::make_shared<DDS2RosListener<nav_msgs::Path>>(&bridge, kDWALocalPlanTopicName));
        kFootPrintSubscriber = std::make_shared<DDSSubscriber>(kPolygonStampedTopicName, ros_type,
                                std::make_shared<DDS2RosListener<geometry_msgs::PolygonStamped>>(&bridge, kPolygonStampedTopicName));
        kVelocitySubscriber = std::make_shared<DDSSubscriber>(kVelocityTopicName, ros_type,
                                std::make_shared<DDS2RosListener<geometry_msgs::Twist>>(&bridge, kVelocityTopicName));

        kPoseSubscriber = std::make_shared<DDSSubscriber>(kPoseStampedTopicName, ros_type,
                                std::make_shared<DDS2RosListener<geometry_msgs::PoseStamped>>(&bridge, kPoseStampedTopicName));
    #ifdef USED_ROS_TRANSFORM
        std::shared_ptr<TF2MessagePubSubType> tf2Msg(new TF2MessagePubSubType);
        kDDSTransformsPublisher = std::make_shared<DDSPublisher>("TF2-SET-TRANSFORM", tf2Msg);
    #endif
    } else {
        kDDSLaserScanPublisher = std::make_shared<DDSPublisher>(localip, kRosBridgeTopicPort, kLaserScanTopicName, ros_type);
        kDDSImuPublisher = std::make_shared<DDSPublisher>(localip, kRosBridgeTopicPort, kImuTopicName, ros_type);
        kDDSOdometryPublisher = std::make_shared<DDSPublisher>(localip, kRosBridgeTopicPort, kOdometryTopicName, ros_type);
        kDDSNavMapPublisher = std::make_shared<DDSPublisher>(localip, kRosBridgeTopicPort, kNavMapTopicName, map_type);
        kDDSGoalPoseStampedPublisher = std::make_shared<DDSPublisher>(localip, kRosBridgeTopicPort, kGoalPoseStampedTopicName, ros_type);

        kGlobalMapSubscriber = std::make_shared<DDSSubscriber>(remoteip, kMappingTopicPort, kGlobalMapTopicName, map_type,
                                std::make_shared<DDS2RosListener<nav_msgs::OccupancyGrid>>(&bridge, kGlobalMapTopicName));

        // Receive from navigation
        kLocalCostmapSubscriber = std::make_shared<DDSSubscriber>(remoteip, kMappingTopicPort, kLocalMapTopicName, map_type,
                                  std::make_shared<DDS2RosListener<nav_msgs::OccupancyGrid>>(&bridge, kLocalMapTopicName));
        kGlobalPlanSubscriber = std::make_shared<DDSSubscriber>(remoteip, kNavigationTopicPort, kGlobalPlanTopicName, map_type,
                                std::make_shared<DDS2RosListener<nav_msgs::Path>>(&bridge, kGlobalPlanTopicName));
        kDWAGlobalPlanSubscriber = std::make_shared<DDSSubscriber>(remoteip, kNavigationTopicPort, kDWAGlobalPlanTopicName, map_type,
                                   std::make_shared<DDS2RosListener<nav_msgs::Path>>(&bridge, kDWAGlobalPlanTopicName));
        kDWALocalPlanSubscriber = std::make_shared<DDSSubscriber>(remoteip, kNavigationTopicPort, kDWALocalPlanTopicName, map_type,
                                  std::make_shared<DDS2RosListener<nav_msgs::Path>>(&bridge, kDWALocalPlanTopicName));
        kFootPrintSubscriber = std::make_shared<DDSSubscriber>(remoteip, kMappingTopicPort, kPolygonStampedTopicName, ros_type,
                               std::make_shared<DDS2RosListener<geometry_msgs::PolygonStamped>>(&bridge, kPolygonStampedTopicName));
        kVelocitySubscriber = std::make_shared<DDSSubscriber>(remoteip, kMappingTopicPort, kVelocityTopicName, ros_type,
                               std::make_shared<DDS2RosListener<geometry_msgs::Twist>>(&bridge, kVelocityTopicName));

        kPoseSubscriber = std::make_shared<DDSSubscriber>(remoteip, kMappingTopicPort, kPoseStampedTopicName, ros_type,
                                std::make_shared<DDS2RosListener<geometry_msgs::PoseStamped>>(&bridge, kPoseStampedTopicName));
    #ifdef USED_ROS_TRANSFORM
        std::shared_ptr<TF2MessagePubSubType> tf2Msg(new TF2MessagePubSubType);
        kDDSTransformsPublisher = std::make_shared<DDSPublisher>(localip, kRosBridgeTopicPort, "TF2-SET-TRANSFORM", tf2Msg);
    #endif
    }

    bool r = false;
    r = bridge.registerOdomCallback(Ros2DDSCallback<nav_msgs::Odometry>);
    assert(r);
#if USED_MULTI_ECHO_SCAN
    r = bridge.registerLaserScanCallback(Ros2DDSCallback<sensor_msgs::MultiEchoLaserScan>);
#else
    r = bridge.registerLaserScanCallback(Ros2DDSCallback<sensor_msgs::LaserScan>);
#endif
    assert(r);
    r = bridge.registerImuCallback(Ros2DDSCallback<sensor_msgs::Imu>);
    assert(r);
    r = bridge.registerGridMapCallback(Ros2DDSCallback<nav_msgs::OccupancyGrid>);
    assert(r);
    r = bridge.registerGoalCallback(Ros2DDSCallback<geometry_msgs::PoseStamped>);
    assert(r);
    r = bridge.registerGnssCallback(Ros2DDSCallback<sensor_msgs::NavSatFix>);
    assert(r);

#ifdef USED_ROS_TRANSFORM
    r = bridge.registerTransformCallback(Ros2DDSCallback<tf2_msgs::TFMessage>);
    assert(r);
#endif

    pause();
    return 0;
}