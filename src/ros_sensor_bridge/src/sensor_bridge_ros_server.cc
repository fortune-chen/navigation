#include <iostream>
#include <set>
#include <cassert>
#include "sensor_bridge.h"
#include "sensor_data.h"

using namespace std;
using namespace ros::serialization;

// #define USE_TCP
#ifdef USE_TCP
#include "tcpserver.hpp"
#else
#include "udpserver.hpp"
#endif

#define SENSOR_SERVICE_PORT 1234
#define FRAGMENT_SIZE 0xf000

#ifdef USE_TCP
TCPServer gServer;
#else
UDPServer gServer;
#endif

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

#ifdef USE_TCP
std::set<TCPSocket*> gReceiverSet;
#else
std::set<Host> gReceiverSet;
#endif

auto onError = [](int errorCode, std::string errorMessage){
    cerr << "errorCode: " << errorCode
        << ", errorMessage: " << errorMessage << endl;
};

void sendFragment(const std::string& frag) {
    for (const auto& receiver: gReceiverSet) {
#ifdef USE_TCP
        sock->Send(frag);
#else
        gServer.SendTo(frag, receiver.ip, receiver.port);
#endif
    }
}

void sendSensorMessage(const sensor_data::SensorData& data) {
    auto s = data.serialize();

    const auto len = s.size();
    int part = len / FRAGMENT_SIZE;
    std::cout << "sending data: " << data << std::endl;
    // std::cout << "data type: " << len << " part: " << part << std::endl;
    for(int i = 0; i <= part; i++) {
        std::string segment;
        if (s.size() <= FRAGMENT_SIZE) {
            segment = s;
        } else {
            segment = s.substr(0, FRAGMENT_SIZE);
            s = s.substr(FRAGMENT_SIZE);
        }
        sendFragment(segment);
    }
}

namespace sensor_data {
    
template<>
struct MessageTraits<nav_msgs::Odometry> {
    const static SensorType type = SensorType::ODOM;
};

template<>
struct MessageTraits<sensor_msgs::LaserScan> {
    const static SensorType type = SensorType::SCAN;
};

template<>
struct MessageTraits<sensor_msgs::Imu> {
    const static SensorType type = SensorType::IMU;
};

template<>
struct MessageTraits<nav_msgs::OccupancyGrid> {
    const static SensorType type = SensorType::MAP;
};

template<>
struct MessageTraits<geometry_msgs::PoseStamped> {
    const static SensorType type = SensorType::GOAL;
};

}   // namespace sensor_data

template<typename M>
void messageCallback(const boost::shared_ptr<const M>& msg) {
    const auto& data = sensor_data::SensorData(*msg);
    cout << "---- messageCallback " << data << endl;
    sendSensorMessage(data);
}

using namespace sensor_data;
int main(int argc, char **argv)
{
    auto& bridge = SensorBridge::getInstance();

    gServer.Bind(SENSOR_SERVICE_PORT, onError);

#ifdef USE_TCP
    gServer.Listen(onError);
    gServer.onNewConnection = [&bridge](TCPSocket *sock) {
        sock->onSocketClosed = [sock](int) {
            gReceiverSet.erase(sock);
        };
        sock->onRawMessageReceived = [sock, &bridge](const char* msg, int len) {
            const auto ip = sock->remoteAddress();
            const auto port = sock->remotePort();
            SensorData data(msg, len);
            cout << "receive " << data
                << " from " << Host{ip, port} << endl;
            if (data.direction == SensorData::Direction::PULL) {
                gReceiverSet.insert(sock);
            } else {
                if (data.type == SensorType::MAP) {
                    const auto map = deserializeStringMessage<nav_msgs::OccupancyGrid>(data.content);
                    bridge.sendGridMap(map);
                }
            }
        };
    };
#else
    SensorData data;
    gServer.onRawMessageReceived =
    [&bridge, data] (const char* msg, int len, std::string ip, std::uint16_t port) mutable {
        if (len <= 0) {
            cerr << "wrong data length" << endl;
            return;
        }
        data.receiveData(msg, len);
        Host h{ip, port};
        cout << "receive from " << h << " " << data << endl;
        if (!data.isComplete()) {
            return;
        }

        if (data.direction == SensorData::Direction::PULL) {
            gReceiverSet.insert(h);
        } else {
            if (data.type == SensorType::MAP) {
                const auto map = deserializeStringMessage<nav_msgs::OccupancyGrid>(data.content);
                bridge.sendGridMap(map);
            } else if (data.type == SensorType::PATH) {
                const auto path = deserializeStringMessage<nav_msgs::Path>(data.content);
                bridge.sendGlobalPath(path);
            }
        }
        data.clear();
    };
#endif

    bool r = false;
    r = bridge.registerOdomCallback(messageCallback<nav_msgs::Odometry>);
    assert(r);
    r = bridge.registerLaserScanCallback(messageCallback<sensor_msgs::LaserScan>);
    assert(r);
    r = bridge.registerImuCallback(messageCallback<sensor_msgs::Imu>);
    assert(r);
    r = bridge.registerGridMapCallback(messageCallback<nav_msgs::OccupancyGrid>);
    assert(r);
    r = bridge.registerGoalCallback(messageCallback<geometry_msgs::PoseStamped>);
    assert(r);

    pause();

    gServer.Close();
    return 0;
}