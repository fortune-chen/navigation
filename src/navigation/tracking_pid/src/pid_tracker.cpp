#include <thread>

#include "tracking_pid/pid_tracker.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "ros/rate.h"
#include "sensor_bridge/global_sensor_bridge_client.h"

using namespace tracking_pid;

static std::string kMapFrame = "map";
static std::string kBaseLinkFrame = "base_link";
bool kControllerEnabled = true;
bool kWaitingForSetpoint = true;
ros::Time prev_time;
ros::Duration delta_t;
tf::Transform tfGoalPose = tf::Transform::getIdentity();


Controller mPidController;

// for test
// void preparePath(nav_msgs::Path& path) {
//   geometry_msgs::PoseStamped pose;
//   path.header.frame_id = "path_frame";
//   path.header.seq = 1;

//   pose.header.seq = 0;
//   pose.pose.position.x = 0.0;
//   pose.pose.position.y = 0.0;
//   pose.pose.orientation.x = 0.0;
//   pose.pose.orientation.y = 0.0;
//   pose.pose.orientation.z = 0.0;
//   pose.pose.orientation.w = 1.0;
//   path.poses.push_back(pose);

//   pose.header.seq = 1;
//   pose.pose.position.x = 0.0;
//   pose.pose.position.y = 0.0;
//   pose.pose.orientation.x = 0.0;
//   pose.pose.orientation.y = 0.0;
//   pose.pose.orientation.z = 0.707107;
//   pose.pose.orientation.w = 0.707107;
//   path.poses.push_back(pose);

//   pose.header.seq = 2;
//   pose.pose.position.x = 0.0;
//   pose.pose.position.y = 1.0;
//   pose.pose.orientation.x = 0.0;
//   pose.pose.orientation.y = 0.0;
//   pose.pose.orientation.z = 0.707107;
//   pose.pose.orientation.w = 0.707107;
//   path.poses.push_back(pose);

//   pose.header.seq = 3;
//   pose.pose.position.x = 0.0;
//   pose.pose.position.y = 1.0;
//   pose.pose.orientation.x = -0.0;
//   pose.pose.orientation.y = 0.0;
//   pose.pose.orientation.z = 1.0;
//   pose.pose.orientation.w = -0.0;
//   path.poses.push_back(pose);

//   pose.header.seq = 4;
//   pose.pose.position.x = -1.5;
//   pose.pose.position.y = 1.0;
//   pose.pose.orientation.x = -0.0;
//   pose.pose.orientation.y = 0.0;
//   pose.pose.orientation.z = 1.0;
//   pose.pose.orientation.w = -0.0;
//   path.poses.push_back(pose);
// }

class PathInterpolationListener : public TrajectoryPointListener {
  public:
    PathInterpolationListener() {}

    void onTrajectoryPointReceived(const TrajectoryPoint& tp) override {
        printf("---------------------- onTrajectoryPointReceived\n");
        auto goalPoint = tp;
        try {
            geometry_msgs::PoseStamped out;
            auto transform = tf2_ros::Buffer::getInstance().lookupTransform(kMapFrame, tf2::getFrameId(goalPoint.pose), ros::Time(0), ros::Duration(0));
            tf2::doTransform(goalPoint.pose, out, transform);
            goalPoint.pose = out;
            kWaitingForSetpoint =  false;
        }
        catch (tf2::TransformException ex) {
            ROS_ERROR("[onTrajectoryPointReceived] %s", ex.what());
            return; 
        }
        tfGoalPose = tf::Transform(tf::Quaternion(goalPoint.pose.pose.orientation.x, goalPoint.pose.pose.orientation.y,
                                                  goalPoint.pose.pose.orientation.z, goalPoint.pose.pose.orientation.w),
                                   tf::Vector3(goalPoint.pose.pose.position.x, goalPoint.pose.pose.position.y, goalPoint.pose.pose.position.z));
            
        static int controlType = 0;
        if (goalPoint.controller != controlType) {
            controlType = goalPoint.controller;
            mPidController.selectMode((tracking_pid::ControllerMode)controlType);
        }
    }
};


PidTracker::PidTracker() {
    mPidController.setHolonomic(false);
    mPidController.setTrackBaseLink(true);
    tracking_pid::PidConfig config;
    mPidController.configure(config);
    std::thread(&PidTracker::updateLoop, this).detach();
    std::thread(&PidTracker::startPath, this).detach();
}

PidTracker::~PidTracker() {

}

void PidTracker::updateLoop() {
    double loop_rate = 20.0;
    ros::Rate rate(loop_rate);
    // Wait for frames to be available before starting node
    while (!tf2_ros::Buffer::getInstance().canTransform(kBaseLinkFrame,  kMapFrame, ros::Time(0), ros::Duration(10.0))) {
        ROS_INFO("Waiting for transform between %s and %s", kMapFrame.c_str(), kBaseLinkFrame.c_str());
    }
    ROS_INFO("Transform between %s and %s found", kMapFrame.c_str(), kBaseLinkFrame.c_str());
    while (true) {
        // Get current robot pose and spin controller
        update();
        rate.sleep();
    }
}

void PidTracker::update() {
    try {
        tf2_ros::Buffer::getInstance().canTransform(kBaseLinkFrame, kMapFrame, ros::Time(0), ros::Duration(10.0));
    } catch (tf2::TransformException ex) {
        ROS_ERROR("Received an exception trying to transform: %s", ex.what());
    }

    // calculate delta_t
    if (!prev_time.isZero())  // Not first time through the program
    {
        delta_t = ros::Time::now() - prev_time;
        prev_time = ros::Time::now();
        if (0 == delta_t.toSec()) {
            ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", ros::Time::now().toSec());
            return;
        }
    } else {
        ROS_DEBUG("prev_time is 0, doing nothing");
        prev_time = ros::Time::now();
        return;
    }
    if (!kWaitingForSetpoint) {
        kWaitingForSetpoint = true;
        geometry_msgs::Twist cmd_vel;
        // tracking_pid::PidDebug pid_debug;
        geometry_msgs::TransformStamped tfCurPoseStamped;
        try {
            tfCurPoseStamped = tf2_ros::Buffer::getInstance().lookupTransform(kMapFrame, kBaseLinkFrame, ros::Time(0));
        } catch (tf2::TransformException ex) {
            ROS_ERROR("[poseCallback] %s", ex.what());
            return; 
        }

        auto tfCurPose = tfCurPoseStamped.transform;
        cmd_vel = mPidController.update(tfCurPose, tfGoalPose, delta_t);

        if (kControllerEnabled) {
            if (cmd_vel.linear.x > 0.22 || cmd_vel.angular.z > 2.0) {
                printf("[Velocity] linear.x(%lf) angular(%lf)\n", cmd_vel.linear.x, cmd_vel.angular.z);
            }
            kBridge->sendVelocity(cmd_vel);
        }
    }
}

void PidTracker::startPath() {
    kBridge->registerGlobalPathCallback([this](const nav_msgs::PathConstPtr& path) {
        printf("Received fcpp path\n");
        nav_msgs::Path realPath = *path;
        realPath.header.frame_id = "path_frame";

        for (auto& p : realPath.poses) {
            p.header.frame_id = "path_frame";
        }

        mPathInterpolator.setPlan(realPath);
        mCondVar.notify_one();
    });

    std::unique_lock<std::mutex> lck(mMutex);
    std::shared_ptr<TrajectoryPointListener> listener = std::make_shared<PathInterpolationListener>();
    mPathInterpolator.registerTrajectoryPointListener(listener);

    while (true) {
        mCondVar.wait(lck);
        printf("startPath\n");
        mPathInterpolator.startPath();
    }
}
