
#include "sensor_bridge_client.h"

#include "sensor_data.h"

#define FRAGMENT_SIZE 0xf000
using namespace sensor_data;

namespace sensor_data {

template <>
struct MessageTraits<nav_msgs::OccupancyGrid> {
  const static SensorType type = SensorType::MAP;
};

template <>
struct MessageTraits<nav_msgs::Path> {
  const static SensorType type = SensorType::PATH;
};

}  // namespace sensor_data

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
SensorBridge& SensorBridge::getInstance(const std::string& ip, const uint16_t port) {
    static SensorBridgeClient sInstance(ip, port);
    return sInstance;
}

SensorBridgeClient::SensorBridgeClient(const std::string& ip,
                                       const uint16_t port)
#ifdef USE_TCP
                                        // socket_
#else
                                      :  socket_(true)
#endif
{
  socket_.Connect(ip, port);
  SensorData data;
  socket_.Send(data.serialize());

#ifdef USE_TCP
  using std::placeholders::_1;
  socket_.onMessageReceived =
  std::bind(&SensorBridgeClient::sensorDataHandler, this, _1);
#else
  socket_.onMessageReceived =
  [this, data](std::string message, std::string ip, std::uint16_t port) mutable {
      if (message.size() <= 0) {
          cerr << "wrong data length" << endl;
          return;
      }
      data.receiveData(message.data(), message.size());
      Host h{ip, port};
      // cout << "receive from " << h << " " << data << endl;
      if (!data.isComplete()) {
          return;
      }

      assert(data.direction == SensorData::Direction::PUSH);
      sensorDataHandler(data.serialize());
  };
#endif
}

SensorBridgeClient::~SensorBridgeClient() { socket_.Close(); }

void SensorBridgeClient::sendData(std::string data) {
  const auto len = data.size();
  int part = len / FRAGMENT_SIZE;
  std::cout << "data size: " << len << " part: " << part << std::endl;
  for(int i = 0; i <= part; i++) {
    std::string segment;
    if (data.size() <= FRAGMENT_SIZE) {
      segment = data;
    } else {
      segment = data.substr(0, FRAGMENT_SIZE);
      data = data.substr(FRAGMENT_SIZE);
    }
    socket_.Send(segment);
  }
}

bool SensorBridgeClient::sendGridMap(const nav_msgs::OccupancyGrid& grid) {
  sensor_data::SensorData sensorData(grid);
  auto data = sensorData.serialize();
  sendData(data);
  return true;
}

bool SensorBridgeClient::sendPose(const geometry_msgs::PoseStamped& pose) {
    return true;
}

void SensorBridgeClient::sendLocalCostmap(const nav_msgs::OccupancyGrid& map) {
}

bool SensorBridgeClient::registerOdomCallback(const OdomCallbackType& cb) {
  odomCallback_ = cb;
  return true;
}

bool SensorBridgeClient::registerLaserScanCallback(
    const LaserScanCallbackType& cb) {
  laserScanCallback_ = cb;
  return true;
}

bool SensorBridgeClient::registerImuCallback(const ImuCallbackType& cb) {
  imuCallback_ = cb;
  return true;
}

bool SensorBridgeClient::registerGridMapCallback(const GridMapCallbackType& cb) {
  gridMapCallback_ = cb;
  return true;
}

bool SensorBridgeClient::registerGoalCallback(const GoalCallbackType& cb) {
  goalCallback_ = cb;
  return true;
}

bool SensorBridgeClient::registerGnssCallback(const GnssCallbackType& cb) {
  gnssCallback_ = cb;
  return true;
}

bool SensorBridgeClient::sendGlobalPath(const nav_msgs::Path& path) {
  sensor_data::SensorData sensorData(path);
  auto data = sensorData.serialize();
  sendData(data);
  return true;
}

void SensorBridgeClient::sendDWAGlobalPath(const nav_msgs::Path& path) {
}

void SensorBridgeClient::sendDWALocalPath(const nav_msgs::Path& path) {
}

void SensorBridgeClient::sendFootPrint(const geometry_msgs::PolygonStamped& footprint) {
}

void SensorBridgeClient::sendVelocity(const geometry_msgs::Twist& velocity) {

}

template<typename M>
void deserializeAndInvokeCallback(const std::string& msg, MessageCallback<M>& cb) {
  if (!cb) return;
  auto m = new M;
  deserializeStringMessage(msg, *m);
  MsgConstPtr<M> ptr(m);
  cb(ptr);
}

using namespace std;
int SensorBridgeClient::sensorDataHandler(const std::string& message) {
  SensorData sensorData(message);
  // cout << "sensorDataHandler, type:" << sensorData.type << endl;
  switch (sensorData.type) {
    case SensorType::SCAN: {
      deserializeAndInvokeCallback(sensorData.content, laserScanCallback_);
    } break;
    case SensorType::IMU: {
      deserializeAndInvokeCallback(sensorData.content, imuCallback_);
    } break;
    case SensorType::ODOM: {
      deserializeAndInvokeCallback(sensorData.content, odomCallback_);
    } break;
    case SensorType::GOAL: {
      deserializeAndInvokeCallback(sensorData.content, goalCallback_);
    } break;
    case SensorType::MAP: {
      deserializeAndInvokeCallback(sensorData.content, gridMapCallback_);
    } break;
    default:
      break;
  }
  return 0;
}
