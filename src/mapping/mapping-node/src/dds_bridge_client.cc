#include <mutex>
#include <condition_variable>
#include "dds_bridge_client.h"

#include "ros/serialization.h"
#include "sensor_data.h"
#include "node_constants.h"
#include "RosAdapterContentPubSubTypes.h"
#include "NodeControlCommandPubSubTypes.h"
using namespace sensor_data;
using namespace mapping_node;
using namespace ros::serialization;

std::mutex kBridgeMutex;
std::condition_variable kBridgeCond;
bool kBridgeReady = false;

//#if USED_MULTI_ECHO_SCAN
#include "ros/time.h"

static int scan_count = 0;
static int imu_count = 0;
static ::ros::Time imu_start_time;
static ::ros::Time imu_end_time;
//#endif
class NodeControlCommandListener : public TopicListener {
public:
    NodeControlCommandListener(DDSBridgeClient* bridge) : TopicListener((void*)&m_data) {
      m_bridge = bridge;
    }
    void onTopicReceived(void* msg) override {
        NodeControlCommand cmd = *(NodeControlCommand*)msg;
        printf("%d, %d, %s\n", cmd.type(), cmd.do_mapping(), cmd.map_path().c_str());
#if USED_MULTI_ECHO_SCAN
        printf("scan_count: %d, imu_count:%d\n", scan_count, imu_count);
        printf("imu_start_time: %lf, imu_end_time:%lf\n",imu_start_time.toSec(),imu_end_time.toSec());
#endif
        std::unique_lock <std::mutex> lck(kBridgeMutex);
        while (!kBridgeReady) {
          kBridgeCond.wait(lck);
        }

        if (cmd.type() == 1) {
         m_bridge->m_controlCommandCallback(cmd.do_mapping(), cmd.map_path());
        }
    };
    NodeControlCommand m_data;
    DDSBridgeClient* m_bridge;
};

template <typename T>
class SensorTopicListener : public TopicListener {
public:
    SensorTopicListener(DDSBridgeClient* bridge) : TopicListener((void*)&m_data) {
      m_bridge = bridge;
    }
    void onTopicReceived(void* msg) override {
        T* map = (T*)msg;
        std::cout << "received data" << std::endl;
    };
    T m_data;
    DDSBridgeClient* m_bridge;
};
#if USED_MULTI_ECHO_SCAN
template<>
void SensorTopicListener<sensor_msgs::MultiEchoLaserScan>::onTopicReceived(void* msg) {
  if (m_bridge->sensorStarted() & 0x1) {
    scan_count++;
    sensor_msgs::MultiEchoLaserScan *laser = new sensor_msgs::MultiEchoLaserScan;
    *laser = *(sensor_msgs::MultiEchoLaserScan*)msg;
    sensor_msgs::MultiEchoLaserScanConstPtr ptr(laser);
    m_bridge->m_laserScanCallback(ptr);
  }
}
#else
template<>
void SensorTopicListener<sensor_msgs::LaserScan>::onTopicReceived(void* msg) {
  if (m_bridge->sensorStarted() & 0x1) {
    scan_count++;
    sensor_msgs::LaserScan *laser = new sensor_msgs::LaserScan;
    *laser = *(sensor_msgs::LaserScan*)msg;
    sensor_msgs::LaserScanConstPtr ptr(laser);
    m_bridge->m_laserScanCallback(ptr);
  }
}
#endif
template<>
void SensorTopicListener<sensor_msgs::Imu>::onTopicReceived(void* msg) {
  if (m_bridge->sensorStarted() & 0x2) {
    sensor_msgs::Imu *imu = new sensor_msgs::Imu;
    *imu = *(sensor_msgs::Imu*)msg;
    sensor_msgs::ImuConstPtr ptr(imu);
    m_bridge->m_imuCallback(ptr);
//#if USED_MULTI_ECHO_SCAN
    if (imu_count == 0) {
      imu_start_time = ::ros::Time::now();
      std::cout << "ptr->header.stamp: " << ptr->header.stamp << std::endl;
    }
    imu_end_time = ::ros::Time::now();
    imu_count++;
//#endif
  }
}
template<>
void SensorTopicListener<nav_msgs::Odometry>::onTopicReceived(void* msg) {
  if (m_bridge->sensorStarted() & 0x4) {
    nav_msgs::Odometry *odom = new nav_msgs::Odometry;
    *odom = *(nav_msgs::Odometry*)msg;
    nav_msgs::OdometryConstPtr ptr(odom);
    //m_bridge->m_odomCallback(ptr);
  }
}

template<>
void SensorTopicListener<sensor_msgs::NavSatFix>::onTopicReceived(void* msg) {
  if (m_bridge->sensorStarted() & 0x8) {
    sensor_msgs::NavSatFix *gnss = new sensor_msgs::NavSatFix;
    *gnss = *(sensor_msgs::NavSatFix*)msg;
    sensor_msgs::NavSatFixConstPtr ptr(gnss);
    m_bridge->m_GnssCallback(ptr);
  }
}

DDSBridgeClient::DDSBridgeClient(const std::string& localip, const std::string& bridge_ip) {
  m_started = false;
  std::shared_ptr<RosAdapterContentPubSubType> ros_type(new RosAdapterContentPubSubType);
  std::shared_ptr<NodeControlCommandPubSubType> dds_type(new NodeControlCommandPubSubType);
  std::shared_ptr<NavMapContentPubSubType> map_type(new NavMapContentPubSubType);
  if (bridge_ip.empty()) {
    m_mapPublisher = std::make_unique<DDSPublisher>(kGlobalMapTopicName, map_type);
    m_posePublisher = std::make_unique<DDSPublisher>(kPoseStampedTopicName, ros_type);
    m_subscribers.push_back(std::make_shared<DDSSubscriber>(kImuTopicName, ros_type, std::make_shared<SensorTopicListener<sensor_msgs::Imu>>(this)));
    //m_subscribers.push_back(std::make_shared<DDSSubscriber>(kOdometryTopicName, ros_type, std::make_shared<SensorTopicListener<nav_msgs::Odometry>>(this)));
#if USED_GNSS_MAPPING
    m_subscribers.push_back(std::make_shared<DDSSubscriber>(kGnssTopicName, ros_type, std::make_shared<SensorTopicListener<sensor_msgs::NavSatFix>>(this)));
#else
#if USED_MULTI_ECHO_SCAN
    m_subscribers.push_back(std::make_shared<DDSSubscriber>(kLaserScanTopicName, ros_type, std::make_shared<SensorTopicListener<sensor_msgs::MultiEchoLaserScan>>(this)));
#else
    m_subscribers.push_back(std::make_shared<DDSSubscriber>(kLaserScanTopicName, ros_type, std::make_shared<SensorTopicListener<sensor_msgs::LaserScan>>(this)));
#endif
#endif
  } else {
    m_subscribers.push_back(std::make_shared<DDSSubscriber>(bridge_ip, kRosBridgeTopicPort, kImuTopicName, ros_type, std::make_shared<SensorTopicListener<sensor_msgs::Imu>>(this)));
    //m_subscribers.push_back(std::make_shared<DDSSubscriber>(bridge_ip, kRosBridgeTopicPort, kOdometryTopicName, ros_type, std::make_shared<SensorTopicListener<nav_msgs::Odometry>>(this)));
#if USED_GNSS_MAPPING
    m_subscribers.push_back(std::make_shared<DDSSubscriber>(bridge_ip, kRosBridgeTopicPort, kGnssTopicName, ros_type, std::make_shared<SensorTopicListener<sensor_msgs::NavSatFix>>(this)));
#else
#if USED_MULTI_ECHO_SCAN
    m_subscribers.push_back(std::make_shared<DDSSubscriber>(bridge_ip, kRosBridgeTopicPort, kLaserScanTopicName, ros_type, std::make_shared<SensorTopicListener<sensor_msgs::MultiEchoLaserScan>>(this)));
#else
    m_subscribers.push_back(std::make_shared<DDSSubscriber>(bridge_ip, kRosBridgeTopicPort, kLaserScanTopicName, ros_type, std::make_shared<SensorTopicListener<sensor_msgs::LaserScan>>(this)));
#endif
#endif
    m_mapPublisher = std::make_unique<DDSPublisher>(localip, kMappingTopicPort, kGlobalMapTopicName, map_type);
    m_posePublisher = std::make_unique<DDSPublisher>(localip, kMappingTopicPort, kPoseStampedTopicName, ros_type);
  }
  m_subscribers.push_back(std::make_shared<DDSSubscriber>(kNodeControlCommandTopicName, dds_type, std::make_shared<NodeControlCommandListener>
  (this)));
}

DDSBridgeClient::~DDSBridgeClient() {}

void DDSBridgeClient::startSensor(unsigned char start) {
  m_started = start;
}

unsigned char DDSBridgeClient::sensorStarted() {
  return m_started;
}
//#if USED_MULTI_ECHO_SCAN
static int map_count = 0;
//#endif
void DDSBridgeClient::postGridMap(const nav_msgs::OccupancyGrid& grid) {
#if 0  // implement by async socket
  sensor_data::SensorData sensorData(grid);
  auto data = sensorData.serialize();
  
  int part = data.size() / kGripMapPartitionSize;
  for(int i = 0; i <= part; i++) {
    std::string segment;
    if (data.size() <= kGripMapPartitionSize) {
      segment = data;
    } else {
      segment = data.substr(0,kGripMapPartitionSize);
      data = data.substr(kGripMapPartitionSize,data.size());
    }
    udpSocket_->Send(segment);
  }
#else
  m_mapPublisher->publishMap<nav_msgs::OccupancyGrid>(grid);
//#if USED_MULTI_ECHO_SCAN
  //std::cout << "map_count: " << map_count++ << "  --> scan_count:" << scan_count << " imu_count:" << imu_count << std::endl;
//#endif
#endif
}

void DDSBridgeClient::postPose(const geometry_msgs::PoseStamped& pose) {
    m_posePublisher->publishRos<geometry_msgs::PoseStamped>(pose);
}

void DDSBridgeClient::registerOdomCallback(const OdomCallbackType& cb) {
  m_odomCallback = cb;
}

void DDSBridgeClient::registerLaserScanCallback(const LaserScanCallbackType& cb) {
  m_laserScanCallback = cb;
}

void DDSBridgeClient::registerImuCallback(const ImuCallbackType& cb) {
  m_imuCallback = cb;
}

void DDSBridgeClient::registerGnssCallback(const GnssCallbackType& cb) {
  m_GnssCallback = cb;
}

void DDSBridgeClient::registerControlCommandCallback(const ControlCommandCallbackType& cb) {
  m_controlCommandCallback = cb;
  std::unique_lock <std::mutex> lck(kBridgeMutex);
  kBridgeReady = true;
  kBridgeCond.notify_all();
}
