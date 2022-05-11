#include "collector_dds_client.h"
#include "RosAdapterContentPubSubTypes.h"

using namespace collector;

template <typename T>
class SensorTopicListener : public TopicListener {
public:
    SensorTopicListener(CollectorDDSClient* bridge) : TopicListener((void*)&m_data) {
      m_bridge = bridge;
    }
    void onTopicReceived(void* msg) override {
        T* map = (T*)msg;
        std::cout << "received data" << std::endl;
    };
    T m_data;
    CollectorDDSClient* m_bridge;
};
template<>
void SensorTopicListener<geometry_msgs::Twist>::onTopicReceived(void* msg) {
    geometry_msgs::Twist velocity = *(geometry_msgs::Twist*)msg;
    m_bridge->m_TwistCallback(velocity);
};

CollectorDDSClient::CollectorDDSClient() {
  m_started = false;
  std::shared_ptr<RosAdapterContentPubSubType> ros_type(new RosAdapterContentPubSubType);

  m_imuPublisher = std::make_unique<DDSPublisher>(kImuTopicName, ros_type);
  m_laserPublisher = std::make_unique<DDSPublisher>(kLaserScanTopicName, ros_type);
  m_odomPublisher = std::make_unique<DDSPublisher>(kOdometryTopicName, ros_type);
  m_gnssPublisher = std::make_unique<DDSPublisher>(kGnssTopicName, ros_type);

  m_twistSubscribers = std::make_shared<DDSSubscriber>(kVelocityTopicName, ros_type, std::make_shared<SensorTopicListener<geometry_msgs::Twist>>(this));
}

CollectorDDSClient::~CollectorDDSClient() {
}

void CollectorDDSClient::startSensor(unsigned char start) {
  m_started = start;
}

unsigned char CollectorDDSClient::sensorStarted() {
  return m_started;
}

void CollectorDDSClient::postImu(const sensor_msgs::Imu& msg) {
  m_imuPublisher->publishRos<sensor_msgs::Imu>(msg);
}

void CollectorDDSClient::postLaser(const sensor_msgs::LaserScan& msg) {
  m_laserPublisher->publishRos<sensor_msgs::LaserScan>(msg);
}

void CollectorDDSClient::postOdom(const nav_msgs::Odometry& msg) {
  m_odomPublisher->publishRos<nav_msgs::Odometry>(msg);
}

void CollectorDDSClient::postGnss(const sensor_msgs::NavSatFix& msg) {
  m_gnssPublisher->publishRos<sensor_msgs::NavSatFix>(msg);
}

