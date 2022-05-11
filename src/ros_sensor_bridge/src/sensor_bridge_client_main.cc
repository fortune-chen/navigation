#include "sensor_bridge.h"

#define SENSOR_SERVICE_PORT 1234

using namespace std;

int main(int argc, char **argv) {
    auto& bridge = SensorBridge::getInstance(argv[1], SENSOR_SERVICE_PORT);
    bridge.registerOdomCallback([](const nav_msgs::OdometryConstPtr& odom) {
        // cout << "-----odom: " << odom->child_frame_id << endl;
    });
    bridge.registerGridMapCallback([](const MsgConstPtr<nav_msgs::OccupancyGrid>& map) {
        cout << "-----map: " << endl;
    });

    pause();
}