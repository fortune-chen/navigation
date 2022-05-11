#include <iostream>
#include <thread>
#include <move_base/move_base.h>
#include <sensor_bridge/global_sensor_bridge_client.h>

#ifndef USED_TF2_BRIDGE
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>

boost::shared_ptr<robot_state_publisher::RobotStatePublisher> kRobotStatePublisher;

bool StartRobot(const std::string& robot_model_filename) {
  urdf::Model model;
  if (!model.initFile(robot_model_filename)) {
    std::cout << "Failed to init modelfile\n";
    return false;
  }
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree)) {
     std::cout << "Failed to extract kdl tree from xml robot description\n";
    return false;
  }
  kRobotStatePublisher.reset(new robot_state_publisher::RobotStatePublisher(tree, model));
  // robot_state_publisher::RobotStatePublisher state_publisher(tree, model);
  // use other thread to send fix transform
  // std::thread t1([](){
  //   while (true) {
  //     kRobotStatePublisher->publishFixedTransforms(false);
  //     usleep(100000);
  //   }
  // });
  // t1.detach();

  std::thread t2([](){
    kRobotStatePublisher->publishFixedTransforms(true);
    sleep(2);
  });

  t2.join();
  std::cout << "finish StartRobot()\n";
  return true;
}

void sendFixedTransforms() {
  kRobotStatePublisher.reset(new robot_state_publisher::RobotStatePublisher());
  std::thread([] {
    while (true) {
      kRobotStatePublisher->publishFixedTransformsWithPathFrame();
      usleep(100000);
    }
  }).detach();
}
#endif

int main(int argc, char** argv){
  std::cout << "start...\n";
  ros::Time::init(); // TODO

  std::string robot_model_filename;
  std::string local_ip;
  std::string bridge_ip;
  std::cout << "start...\n";

  if (argc == 2) {
    robot_model_filename = argv[1];
    local_ip = "127.0.0.1";
  } else if (argc == 3) {
    robot_model_filename = argv[1];
    local_ip = argv[2];
  }

  if (!robot_model_filename.empty()) {
    if (!StartRobot(robot_model_filename)) {
      return -1;
    }
  } else {
    sendFixedTransforms();
  }

  kBridge.reset(new SensorBridgeClient(local_ip, bridge_ip)); // TODO

#ifdef USED_TF2_BRIDGE
  move_base::MoveBase move_base(*tf2_ros::TF2BridgeClient::getInstance());
#else
  tf2_ros::TransformListener tf(tf2_ros::Buffer::getInstance());
  move_base::MoveBase move_base(tf2_ros::Buffer::getInstance());
#endif
  std::cout << "pause...\n";
  pause();

  return(0);
}
