#include <thread>
#include <memory>

#include "urdf/model.h"
#include "kdl_parser/kdl_parser.hpp"
#include "robot_state_publisher/robot_state_publisher.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/tf2_bridge_server.h"

std::shared_ptr<robot_state_publisher::RobotStatePublisher> kRobotStatePublisher;

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
  std::thread t1([](){
    while (true) {
      kRobotStatePublisher->publishFixedTransforms(false);
      usleep(100000);
    }
  });
  t1.detach();
  std::thread t2([](){
    std::cout << "state_publisher.publishFixedTransforms()\n";
    kRobotStatePublisher->publishFixedTransforms(true);
    sleep(2);
  });
  t2.join();
  std::cout << "finish StartRobot()\n";
  return true;
}

int main(int argc, char** argv){
  std::string robot_model_filename;
  std::cout << "start tf2 node\n";
  if (argc > 1) {
    robot_model_filename = argv[1];
  } else {
    std::cout << "[error] need to specify the urdf file";
    return -1;
  }

  ros::Time::init(); // TODO
  
  if (!robot_model_filename.empty()) {
    if (!StartRobot(robot_model_filename)) {
      return -1;
    }
  } else {
    std::cout << "robot_model_filename empty\n";
    return -1;
  }

  auto& buffer = tf2_ros::Buffer::getInstance();
  tf2_ros::TF2BridgeServer tf2BridgeServer(buffer);
  std::cout << "pause...\n";
  pause();

  return(0);
}
