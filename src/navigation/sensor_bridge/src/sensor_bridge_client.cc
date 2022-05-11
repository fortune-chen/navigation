
#include "sensor_bridge/sensor_bridge_client.h"

#include "sensor_bridge/sensor_data.h"
#include "RosAdapterContentPubSubTypes.h"
#include "ros/log_transfer.h"

#define FRAGMENT_SIZE 0xf000
using namespace sensor_data;

struct Host {
    std::string ip;
    int port;
};
bool operator<(const Host& l, const Host& r) {
    return l.ip < r.ip || l.port < r.port;
}
template<typename Os>
Os& operator<<(Os& os, const Host& host) {
    return os << "ip: " << host.ip
            << ", port: " << host.port;
}

using namespace std;

template <typename T>
class SensorTopicListener : public TopicListener {
public:
    SensorTopicListener(SensorBridgeClient* bridge, const std::string& topic) : TopicListener((void*)&m_data) {
      m_bridge = bridge;
      m_topic = topic;
    }
    void onTopicReceived(void* msg) override {
        T* map = (T*)msg;
        std::cout << "received data" << std::endl;
    };
    T m_data;
    SensorBridgeClient* m_bridge;
    std::string m_topic;
};

template<>
void SensorTopicListener<nav_msgs::OccupancyGrid>::onTopicReceived(void* msg) {
  ROS_INFO("[MAP] cb start in %lf\n", ros::Time::now().toSec());
  nav_msgs::OccupancyGrid *map = new nav_msgs::OccupancyGrid;
  *map = *(nav_msgs::OccupancyGrid*)msg;
  nav_msgs::OccupancyGridConstPtr ptr(map);
  if (m_topic == kNavMapTopicName || m_topic == kGlobalMapTopicName) {
    std::cout << "[nav_node] received nav map by dds" << std::endl;
    if (m_bridge->gridMapCallback_) {
      m_bridge->gridMapCallback_(ptr);
    }
  } else {
    ROS_WARN("unsubscribe topic : %s", m_topic.c_str());
  }
  ROS_INFO("[MAP] cb end in %lf\n", ros::Time::now().toSec());
}

template<>
void SensorTopicListener<geometry_msgs::PoseStamped>::onTopicReceived(void* msg) {
  ROS_INFO("[GOAL] cb start in %lf\n", ros::Time::now().toSec());
  geometry_msgs::PoseStamped p = *(geometry_msgs::PoseStamped*)msg;
  std::thread([this, p]() {
    geometry_msgs::PoseStamped *pose = new geometry_msgs::PoseStamped;
    *pose = p;
    geometry_msgs::PoseStampedConstPtr ptr(pose);
    if (m_topic == kGoalPoseStampedTopicName) {
      ROS_INFO("received goal by dds, (%lf, %lf)", ptr->pose.position.x, ptr->pose.position.y);
      if (m_bridge->goalCallback_) {
        m_bridge->goalCallback_(ptr);
      }
    } else if (m_topic == kCoveragePoseStampedTopicName) {
      if (m_bridge->fullCoverageCallback_) {
        m_bridge->fullCoverageCallback_(ptr);
      }
    } else {
      ROS_WARN("unsubscribe topic : %s", m_topic.c_str());
    }
  }).detach();
  ROS_INFO("[GOAL] cb end in %lf\n", ros::Time::now().toSec());
}

template<>
void SensorTopicListener<sensor_msgs::LaserScan>::onTopicReceived(void* msg) {
  // std::cout << "[nav_node] received laser scan by dds" << std::endl;
  sensor_msgs::LaserScan *laser = new sensor_msgs::LaserScan;
  *laser = *(sensor_msgs::LaserScan*)msg;
  laser->header.stamp = ros::Time(0); // TODO Bill
  sensor_msgs::LaserScanConstPtr ptr(laser);
  if (m_bridge->laserScanCallback_) {
    m_bridge->laserScanCallback_(ptr);
  }
}

template<>
void SensorTopicListener<nav_msgs::Odometry>::onTopicReceived(void* msg) {
  // std::cout << "[nav_node] received odometry by dds" << std::endl;
  nav_msgs::Odometry *odometry = new nav_msgs::Odometry;
  *odometry = *(nav_msgs::Odometry*)msg;
  nav_msgs::OdometryConstPtr ptr(odometry);
  if (m_bridge->odomCallback_) {
    m_bridge->odomCallback_(ptr);
  }
}

template<>
void SensorTopicListener<geometry_msgs::Twist>::onTopicReceived(void* msg) {
  geometry_msgs::Twist vel = *(geometry_msgs::Twist*)msg;
#if 1 // for ros enviroment or simulation
  m_bridge->sendVelocity(vel);
#else
  // TODO : Call the drive interface directly
#endif
}

template<>
void SensorTopicListener<nav_msgs::Path>::onTopicReceived(void* msg) {
    nav_msgs::Path p = *(nav_msgs::Path*)msg;
    nav_msgs::Path *path = new nav_msgs::Path;
    *path = p;
    nav_msgs::PathConstPtr ptr(path);
    if (m_topic == kGlobalPlanTopicName) {
        std::cout << "[ros_node] received global path by dds\n";
        if (m_bridge->globalPathCallback_) {
          m_bridge->globalPathCallback_(ptr);
        }
    } else {
        std::cout << "[error] unknown\n";
        assert(false);
    }
}

SensorBridgeClient::SensorBridgeClient(const std::string& localip, const std::string& bridge_ip) {
  std::shared_ptr<RosAdapterContentPubSubType> ros_type(new RosAdapterContentPubSubType);
  std::shared_ptr<NavMapContentPubSubType> map_type(new NavMapContentPubSubType);
  if (bridge_ip.empty()) {
    global_plan_publisher_ = std::make_unique<DDSPublisher>(kGlobalPlanTopicName, map_type);
    dwa_global_plan_publisher_ = std::make_unique<DDSPublisher>(kDWAGlobalPlanTopicName, map_type);
    dwa_local_plan_publisher_ = std::make_unique<DDSPublisher>(kDWALocalPlanTopicName, map_type);
    velocity_publisher_ = std::make_unique<DDSPublisher>(kVelocityTopicName, ros_type);
    footprint_publisher_ = std::make_unique<DDSPublisher>(kPolygonStampedTopicName, ros_type);
    local_costmap_publisher_ = std::make_unique<DDSPublisher>(kLocalMapTopicName, map_type);
  #ifdef USED_NAVIGATION_ONLY
    subscribers_.push_back(std::make_shared<DDSSubscriber>(kNavMapTopicName, map_type,
                           std::make_shared<SensorTopicListener<nav_msgs::OccupancyGrid>>(this, kNavMapTopicName)));
  #else
    subscribers_.push_back(std::make_shared<DDSSubscriber>(kGlobalMapTopicName, map_type,
                           std::make_shared<SensorTopicListener<nav_msgs::OccupancyGrid>>(this, kGlobalMapTopicName)));
  #endif
    subscribers_.push_back(std::make_shared<DDSSubscriber>(kGoalPoseStampedTopicName, ros_type,
                           std::make_shared<SensorTopicListener<geometry_msgs::PoseStamped>>(this, kGoalPoseStampedTopicName)));
    // TODO Bill Temp
    subscribers_.push_back(std::make_shared<DDSSubscriber>(kOdometryTopicName, ros_type,
                           std::make_shared<SensorTopicListener<nav_msgs::Odometry>>(this, kOdometryTopicName)));
    subscribers_.push_back(std::make_shared<DDSSubscriber>(kLaserScanTopicName, ros_type,
                           std::make_shared<SensorTopicListener<sensor_msgs::LaserScan>>(this, kLaserScanTopicName)));
    subscribers_.push_back(std::make_shared<DDSSubscriber>(kTeleopKeyTopicName, ros_type,
                           std::make_shared<SensorTopicListener<geometry_msgs::Twist>>(this, kTeleopKeyTopicName)));
    subscribers_.push_back(std::make_shared<DDSSubscriber>(kCoveragePoseStampedTopicName, ros_type,
                           std::make_shared<SensorTopicListener<geometry_msgs::PoseStamped>>(this, kCoveragePoseStampedTopicName)));
    subscribers_.push_back(std::make_shared<DDSSubscriber>(kGlobalPlanTopicName, map_type,
                           std::make_shared<SensorTopicListener<nav_msgs::Path>>(this, kGlobalPlanTopicName)));
  } else {
    global_plan_publisher_ = std::make_unique<DDSPublisher>(localip, kNavigationTopicPort, kGlobalPlanTopicName, map_type);
    dwa_global_plan_publisher_ = std::make_unique<DDSPublisher>(localip, kNavigationTopicPort, kDWAGlobalPlanTopicName, map_type);
    dwa_local_plan_publisher_ = std::make_unique<DDSPublisher>(localip, kNavigationTopicPort, kDWALocalPlanTopicName, map_type);
    velocity_publisher_ = std::make_unique<DDSPublisher>(localip, kNavigationTopicPort, kVelocityTopicName, ros_type);
    footprint_publisher_ = std::make_unique<DDSPublisher>(localip, kNavigationTopicPort, kPolygonStampedTopicName, ros_type);
    local_costmap_publisher_ = std::make_unique<DDSPublisher>(localip, kNavigationTopicPort, kLocalMapTopicName, map_type);
  #ifdef USED_NAVIGATION_ONLY
    subscribers_.push_back(std::make_shared<DDSSubscriber>(bridge_ip, kRosBridgeTopicPort, kNavMapTopicName, map_type,
                           std::make_shared<SensorTopicListener<nav_msgs::OccupancyGrid>>(this, kNavMapTopicName)));
  #else
    subscribers_.push_back(std::make_shared<DDSSubscriber>(bridge_ip, kRosBridgeTopicPort, kGlobalMapTopicName, map_type,
                           std::make_shared<SensorTopicListener<nav_msgs::OccupancyGrid>>(this, kGlobalMapTopicName)));
  #endif
    subscribers_.push_back(std::make_shared<DDSSubscriber>(bridge_ip, kRosBridgeTopicPort, kGoalPoseStampedTopicName, ros_type,
                           std::make_shared<SensorTopicListener<geometry_msgs::PoseStamped>>(this, kGoalPoseStampedTopicName)));
    subscribers_.push_back(std::make_shared<DDSSubscriber>(bridge_ip, kRosBridgeTopicPort, kLaserScanTopicName, ros_type,
                           std::make_shared<SensorTopicListener<sensor_msgs::LaserScan>>(this, kLaserScanTopicName)));
    subscribers_.push_back(std::make_shared<DDSSubscriber>(bridge_ip, kRosBridgeTopicPort, kOdometryTopicName, ros_type,
                           std::make_shared<SensorTopicListener<nav_msgs::Odometry>>(this, kOdometryTopicName)));
    subscribers_.push_back(std::make_shared<DDSSubscriber>(bridge_ip, kRosBridgeTopicPort, kGlobalPlanTopicName, ros_type,
                           std::make_shared<SensorTopicListener<nav_msgs::Path>>(this, kGlobalPlanTopicName)));
  }
}

SensorBridgeClient::~SensorBridgeClient() {
}

void SensorBridgeClient::registerGridMapCallback(const GridMapCallbackType& cb) {
  gridMapCallback_ = cb;
}

void SensorBridgeClient::registerGoalCallback(const GoalCallbackType& cb) {
  goalCallback_ = cb;
}

void SensorBridgeClient::registerOdomCallback(const OdomCallbackType& cb) {
  odomCallback_ = cb;
}

void SensorBridgeClient::registerLaserScanCallback(const LaserScanCallbackType& cb) {
  laserScanCallback_ = cb;
}

void SensorBridgeClient::registerFullCoverageCallback(const FullCoverageCallbackType& cb) {
  fullCoverageCallback_ = cb;
}

void SensorBridgeClient::registerGlobalPathCallback(const GlobalPathCallbackType& cb) {
  globalPathCallback_ = cb;
}

void SensorBridgeClient::sendGlobalPath(const nav_msgs::Path& path) {
  // std::cout << "send global path by dds" << std::endl;
  global_plan_publisher_->publishMap<nav_msgs::Path>(path);
}

void SensorBridgeClient::sendDWAGlobalPath(const nav_msgs::Path& path) {
  // std::cout << "send DWA global path by dds" << std::endl;
  dwa_global_plan_publisher_->publishMap<nav_msgs::Path>(path);
}

void SensorBridgeClient::sendDWALocalPath(const nav_msgs::Path& path) {
  // std::cout << "send DWA local path by dds" << std::endl;
  dwa_local_plan_publisher_->publishMap<nav_msgs::Path>(path);
}

void SensorBridgeClient::sendVelocity(const geometry_msgs::Twist& vel_cmd) {
  // std::cout << "[nav_node] send velocity by dds" << std::endl;
  velocity_publisher_->publishRos<geometry_msgs::Twist>(vel_cmd);
}

void SensorBridgeClient::sendFootprint(const geometry_msgs::PolygonStamped& footprint) {
  // std::cout << "[nav_node] send footprint by dds" << std::endl;
  footprint_publisher_->publishRos<geometry_msgs::PolygonStamped>(footprint);
}

void SensorBridgeClient::sendLocalCostmap(const nav_msgs::OccupancyGrid& map) {
  local_costmap_publisher_->publishMap<nav_msgs::OccupancyGrid>(map);
}