#include <unistd.h>
#include <unordered_map>

#include "debug_helper.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

static std::unordered_map<std::string, std::function<void(void)>> functionsMap;

static const std::string helpMessage = 
    "+----------------------------------------------------------------------------+\n"
    "|                                  Options:                                  |\n"
    "|       Press '1' to set transform                                           |\n"
    "|       Press '2' to register transform listener                             |\n"
    "| Quit:                                                                      |\n"
    "|       Press 'q' to quit the application.                                   |\n"
    "+----------------------------------------------------------------------------+\n";

static std::vector<std::string> kFrameIds = {
    "A",
    "B",
    "C",
    "D",
    "E",
    "F",
    "G",
    "H",
    "I",
    "J",
    "K"
};

void setTransform() {
    std::cout << "setTransform"<< std::endl;
    tf2_ros::TransformBroadcaster broadcaster;
    std::vector<geometry_msgs::TransformStamped> tf_transforms;
    geometry_msgs::TransformStamped tf_transform;

    for (int i = 0; i < kFrameIds.size() - 1; i++) {
        tf_transform.transform.translation.x  = 0.000;
        tf_transform.transform.translation.y  = 0.000;
        tf_transform.transform.translation.z  = 0.000;
        tf_transform.transform.rotation.w     = 1.000;
        tf_transform.transform.rotation.x     = 0.000;
        tf_transform.transform.rotation.y     = 0.000;
        tf_transform.transform.rotation.z     = 0.000;
        tf_transform.header.stamp = ros::Time::now();
        tf_transform.header.frame_id = kFrameIds[i];
        tf_transform.child_frame_id = kFrameIds[i + 1];
        tf_transforms.push_back(tf_transform);
    }

    sleep(1);
    std::cout << "start sendTransform every 500us" << std::endl;
    while (true) {
        usleep(500);
        broadcaster.sendTransform(tf_transforms);
    }
}

void registerTransformListener() {
    tf2_ros::TransformListener tf(tf2_ros::Buffer::getInstance());
    std::cout << "registerTransformListener"<< std::endl;
    pause();
}

int32_t main(int argc, char **argv) {
    utils::DebugHelper::getInstance()->init();
    ros::Time::init(); // TODO
    auto& buffer = tf2_ros::Buffer::getInstance();
    functionsMap.emplace("1", setTransform);
    functionsMap.emplace("2", registerTransformListener);
    std::string cmd;

    while (true) {
        std::cout << helpMessage;
        std::cin >> cmd;

        auto function = functionsMap.find(cmd);
        if (functionsMap.end() != function) {
            function->second();
        }

        if ("q" == cmd) {
            break;
        }
    }
    return 0;
}