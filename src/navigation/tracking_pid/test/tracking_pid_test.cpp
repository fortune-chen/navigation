#include "ros/rate.h"
#include "tracking_pid/interpolator.h"
#include "tracking_pid/controller_node.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "sensor_bridge/global_sensor_bridge_client.h"
#include "robot_state_publisher/robot_state_publisher.h"

#include <mutex>
#include <Eigen/Eigen>
#include <condition_variable>

std::mutex kMutex;
std::condition_variable kCondVar;
std::shared_ptr<robot_state_publisher::RobotStatePublisher> kRobotStatePublisher;

using namespace tracking_pid;

void print_traj_point(const tracking_pid::TrajectoryPoint& goalPointMsg) {
  printf("goalPointMsg.acceleration.linear.x     : %lf\n", goalPointMsg.acceleration.linear.x);
  printf("goalPointMsg.acceleration.linear.y     : %lf\n", goalPointMsg.acceleration.linear.y);
  printf("goalPointMsg.acceleration.angular.x    : %lf\n", goalPointMsg.acceleration.angular.x);
  printf("goalPointMsg.acceleration.angular.y    : %lf\n", goalPointMsg.acceleration.angular.y);
  printf("goalPointMsg.acceleration.angular.z    : %lf\n", goalPointMsg.acceleration.angular.z);
  printf("goalPointMsg.controller.data           : %d \n", goalPointMsg.controller);
  printf("goalPointMsg.pose.header.frame_id      : %s\n", goalPointMsg.pose.header.frame_id.c_str());
  printf("goalPointMsg.pose.header.seq           : %d\n", goalPointMsg.pose.header.seq);
  printf("goalPointMsg.pose.pose.position.x      : %lf\n", goalPointMsg.pose.pose.position.x);
  printf("goalPointMsg.pose.pose.position.y      : %lf\n", goalPointMsg.pose.pose.position.y);
  printf("goalPointMsg.pose.pose.orientation.x   : %lf\n", goalPointMsg.pose.pose.orientation.x);
  printf("goalPointMsg.pose.pose.orientation.y   : %lf\n", goalPointMsg.pose.pose.orientation.y);
  printf("goalPointMsg.pose.pose.orientation.z   : %lf\n", goalPointMsg.pose.pose.orientation.z);
  printf("goalPointMsg.pose.pose.orientation.w   : %lf\n", goalPointMsg.pose.pose.orientation.w);
  printf("goalPointMsg.time.data                 : %lf\n", goalPointMsg.time);
  printf("goalPointMsg.velocity.linear.x         : %lf\n", goalPointMsg.velocity.linear.x);
  printf("goalPointMsg.velocity.linear.y         : %lf\n", goalPointMsg.velocity.linear.y);
  printf("goalPointMsg.velocity.linear.z         : %lf\n", goalPointMsg.velocity.linear.z);
  printf("goalPointMsg.velocity.angular.x        : %lf\n", goalPointMsg.velocity.angular.x);
  printf("goalPointMsg.velocity.angular.y        : %lf\n", goalPointMsg.velocity.angular.y);
  printf("goalPointMsg.velocity.angular.z        : %lf\n", goalPointMsg.velocity.angular.z);
}

class UserListener : public TrajectoryPointListener {
  public:
    UserListener() {}

    void onTrajectoryPointReceived(const TrajectoryPoint& tp) override {
        goalPoint = tp;
        try {
          // print_traj_point(goalPoint);
          geometry_msgs::PoseStamped out;
          auto transform = tf2_ros::Buffer::getInstance().lookupTransform(map_frame, tf2::getFrameId(goalPoint.pose), ros::Time(0), ros::Duration(0));
          tf2::doTransform(goalPoint.pose, out, transform);
          goalPoint.pose = out;
          waiting_for_setpoint =  false;
        }
        catch (tf2::TransformException ex) {
            ROS_ERROR("[onTrajectoryPointReceived] %s", ex.what());
            return; 
        }
        tfGoalPose = tf::Transform(
                                tf::Quaternion(goalPoint.pose.pose.orientation.x, goalPoint.pose.pose.orientation.y,
                                                goalPoint.pose.pose.orientation.z, goalPoint.pose.pose.orientation.w),
                                tf::Vector3(goalPoint.pose.pose.position.x, goalPoint.pose.pose.position.y, goalPoint.pose.pose.position.z));
        if (goalPoint.controller != controlType) {
            controlType = goalPoint.controller;
            pid_controller.selectMode((tracking_pid::ControllerMode)controlType);
        }
    }
};

void preparePath(nav_msgs::Path& path) {
  geometry_msgs::PoseStamped pose;
  path.header.frame_id = "path_frame";
  path.header.seq = 1;

  pose.header.seq = 0;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);

  pose.header.seq = 1;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = -0.938475;
  pose.pose.orientation.w = 0.345347;
  path.poses.push_back(pose);

  pose.header.seq = 2;
  pose.pose.position.x = -0.175259;
  pose.pose.position.y = -0.149189;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = -0.938475;
  pose.pose.orientation.w = 0.345347;
  path.poses.push_back(pose);

  pose.header.seq = 3;
  pose.pose.position.x = -0.175259;
  pose.pose.position.y = -0.149189;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.707107;
  pose.pose.orientation.w = 0.707107;
  path.poses.push_back(pose);

  pose.header.seq = 4;
  pose.pose.position.x = -0.175259;
  pose.pose.position.y = -0.149189;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.707107;
  pose.pose.orientation.w = 0.707107;
  path.poses.push_back(pose);

  pose.header.seq = 5;
  pose.pose.position.x = -0.175259;
  pose.pose.position.y = 0.250811;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.707107;
  pose.pose.orientation.w = 0.707107;
  path.poses.push_back(pose);

  pose.header.seq = 6;
  pose.pose.position.x = -0.175259;
  pose.pose.position.y = 0.250811;
  pose.pose.orientation.x = -0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 1.0;
  pose.pose.orientation.w = -0.0;
  path.poses.push_back(pose);

  pose.header.seq = 7;
  pose.pose.position.x = -0.575259;
  pose.pose.position.y = 0.250811;
  pose.pose.orientation.x = -0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 1.0;
  pose.pose.orientation.w = -0.0;
  path.poses.push_back(pose);

  pose.header.seq = 8;
  pose.pose.position.x = -0.575259;
  pose.pose.position.y = 0.250811;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.707107;
  pose.pose.orientation.w = 0.707107;
  path.poses.push_back(pose);

  pose.header.seq = 9;
  pose.pose.position.x = -0.575259;
  pose.pose.position.y = 0.650811;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.707107;
  pose.pose.orientation.w = 0.707107;
  path.poses.push_back(pose);

  pose.header.seq = 10;
  pose.pose.position.x = -0.575259;
  pose.pose.position.y = 0.650811;
  pose.pose.orientation.x = -0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 1.0;
  pose.pose.orientation.w = -0.0;
  path.poses.push_back(pose);

  pose.header.seq = 11;
  pose.pose.position.x = -1.375259;
  pose.pose.position.y = 0.650811;
  pose.pose.orientation.x = -0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 1.0;
  pose.pose.orientation.w = -0.0;
  path.poses.push_back(pose);

  pose.header.seq = 12;
  pose.pose.position.x = -1.375259;
  pose.pose.position.y = 0.650811;
  pose.pose.orientation.x = -0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.707107;
  pose.pose.orientation.w = -0.707107;
  path.poses.push_back(pose);

  pose.header.seq = 13;
  pose.pose.position.x = -1.375259;
  pose.pose.position.y = -0.549189;
  pose.pose.orientation.x = -0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.707107;
  pose.pose.orientation.w = -0.707107;
  path.poses.push_back(pose);

  pose.header.seq = 14;
  pose.pose.position.x = -1.375259;
  pose.pose.position.y = -0.549189;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);

  pose.header.seq = 15;
  pose.pose.position.x = 0.224741;
  pose.pose.position.y = -0.549189;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;
  path.poses.push_back(pose);

  pose.header.seq = 16;
  pose.pose.position.x = 0.224741;
  pose.pose.position.y = -0.549189;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.707107;
  pose.pose.orientation.w = 0.707107;
  path.poses.push_back(pose);

  pose.header.seq = 17;
  pose.pose.position.x = 0.224741;
  pose.pose.position.y = 0.650811;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.707107;
  pose.pose.orientation.w = 0.707107;
  path.poses.push_back(pose);

  pose.header.seq = 18;
  pose.pose.position.x = 0.224741;
  pose.pose.position.y = 0.650811;
  pose.pose.orientation.x = -0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 1.0;
  pose.pose.orientation.w = -0.0;
  path.poses.push_back(pose);
}

void poseCallback() {
  try {
    tf2_ros::Buffer::getInstance().canTransform(base_link_frame, map_frame, ros::Time(0), ros::Duration(10.0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("Received an exception trying to transform: %s", ex.what());
  }

  // calculate delta_t
  if (!prev_time.isZero())  // Not first time through the program
  {
    delta_t = ros::Time::now() - prev_time;
    prev_time = ros::Time::now();
    if (0 == delta_t.toSec())
    {
      ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", ros::Time::now().toSec());
      return;
    }
  } else {
    ROS_DEBUG("prev_time is 0, doing nothing");
    prev_time = ros::Time::now();
    return;
  }
  if (!waiting_for_setpoint) {
    geometry_msgs::Twist cmd_vel;
    // tracking_pid::PidDebug pid_debug;
    geometry_msgs::TransformStamped tfCurPoseStamped;
    try {
      tfCurPoseStamped = tf2_ros::Buffer::getInstance().lookupTransform(map_frame, base_link_frame, ros::Time(0));
    } catch (tf2::TransformException ex) {
        ROS_ERROR("[poseCallback] %s", ex.what());
        return; 
    }

    tfCurPose = tfCurPoseStamped.transform;
    cmd_vel = pid_controller.update(tfCurPose, tfGoalPose, delta_t);

    if (controller_enabled) {
      if (cmd_vel.linear.x > 0.22 || cmd_vel.angular.z > 2.0) {
        printf("[Velocity] linear.x(%lf) angular(%lf)\n", cmd_vel.linear.x, cmd_vel.angular.z);
      }
      kBridge->sendVelocity(cmd_vel);
    }
  }
}

void reconfigure_callback(const tracking_pid::PidConfig& config) {
  pid_controller.configure(config);
  controller_debug_enabled = config.controller_debug_enabled;
}

int main() {
    ros::Time::init();
    kBridge.reset(new SensorBridgeClient("127.0.0.1", ""));
    tf2_ros::TransformListener tf_listener(tf2_ros::Buffer::getInstance());
    kRobotStatePublisher.reset(new robot_state_publisher::RobotStatePublisher());
    std::thread([]() {
      while (true) {
        kRobotStatePublisher->publishFixedTransformsWithPathFrame();
        usleep(100000);
      }
    }).detach();
    sleep(1);
    // ----------------------------------------------------------------------------------
    std::thread([]() {
        map_frame = "map";
        base_link_frame = "base_link";
        pid_controller.setHolonomic(false);
        pid_controller.setTrackBaseLink(true);
        controller_enabled = true;
        waiting_for_setpoint = true;

        tracking_pid::PidConfig config;
        reconfigure_callback(config);

        double loop_rate = 20.0;
        ros::Rate rate(loop_rate);
        // Wait for frames to be available before starting node
        while (!tf2_ros::Buffer::getInstance().canTransform(base_link_frame,  map_frame, ros::Time(0), ros::Duration(10.0))) {
          ROS_INFO("Waiting for transform between %s and %s", map_frame.c_str(), base_link_frame.c_str());
        }
        ROS_INFO("Transform between %s and %s found", map_frame.c_str(), base_link_frame.c_str());
        while (true)
        {
            // Get current robot pose and spin controller
            poseCallback();
            rate.sleep();
        }
    }).detach();

#if 0
    Interpolator interpolator;
    nav_msgs::Path path;
    preparePath(path);
    kBridge->sendGlobalPath(path);
    interpolator.setPlan(path);
    std::shared_ptr<TrajectoryPointListener> listener = std::make_shared<UserListener>();
    interpolator.registerTrajectoryPointListener(listener);
    interpolator.startPath();
#else
    Interpolator interpolator;
    kBridge->registerGlobalPathCallback([&](const nav_msgs::PathConstPtr& path) {
      printf("Received fcpp path\n");
      nav_msgs::Path realPath = *path;
      realPath.header.frame_id = "path_frame";

      for (auto& p : realPath.poses) {
        p.header.frame_id = "path_frame";
      }

      interpolator.setPlan(realPath);
      kCondVar.notify_one();
    });


    std::unique_lock<std::mutex> lck(kMutex);
    kCondVar.wait(lck);
    
    std::shared_ptr<TrajectoryPointListener> listener = std::make_shared<UserListener>();
    interpolator.registerTrajectoryPointListener(listener);
    interpolator.startPath();
  #endif

    pause();
    return 0;
}