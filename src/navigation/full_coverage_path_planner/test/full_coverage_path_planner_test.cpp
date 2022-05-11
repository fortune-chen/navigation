#include "full_coverage_path_planner/spiral_stc.h"
#include "sensor_bridge/global_sensor_bridge_client.h"

using namespace full_coverage_path_planner;

int main(int argc, char** argv) {
    ros::Time::init(); // TODO
    kBridge.reset(new SensorBridgeClient("127.0.0.1", ""));

    SpiralSTC stc;
    stc.initialize("full_coverage_path_planner", nullptr);
    sleep(15);
    geometry_msgs::PoseStamped start;
    geometry_msgs::PoseStamped goal;
    std::vector<geometry_msgs::PoseStamped> plan;
    stc.makePlan(start, goal, plan);
    while(1);
    return 0;
}