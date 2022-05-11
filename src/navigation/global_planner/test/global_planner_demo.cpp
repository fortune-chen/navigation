#include "global_planner_demo.h"
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <ros/log_transfer.h>
#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <global_planner/planner_core.h>
#include <sensor_bridge/global_sensor_bridge_client.h>

#ifdef USED_TF2_BRIDGE
#include "tf2_bridge/tf2_bridge_client.h"
#endif

#ifdef USED_TF2_BRIDGE
GlobalPlannerDemo::GlobalPlannerDemo(tf2_ros::TF2BridgeClient& tf) :
#else
GlobalPlannerDemo::GlobalPlannerDemo(tf2_ros::Buffer& tf) :
#endif
    tf_(tf),
    planner_costmap_ros_(NULL),
    planner_plan_(NULL),
    runPlanner_(false) {
    ROS_DEBUG("[%s] [%d]\n", __FUNCTION__, __LINE__);
    robot_base_frame_ = "base_footprint";
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    std::string global_planner = "global_planner/GlobalPlanner";

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    planner_.reset(new global_planner::GlobalPlanner());
    planner_->initialize(global_planner, planner_costmap_ros_);

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();

    planner_thread_ = new boost::thread(boost::bind(&GlobalPlannerDemo::planThread, this));

    kBridge->registerGoalCallback([this](const geometry_msgs::PoseStampedConstPtr& goal) {
        goalCB(goal);
        printf("[subscribe] >>>>>>>>>>>>> goal [%lf, %lf]\n", goal->pose.position.x, goal->pose.position.y);
    });
}

void GlobalPlannerDemo::planThread() {
    while(true){
        if(runPlanner_) {
            runPlanner_ = false;
            planner_plan_->clear();
            geometry_msgs::PoseStamped temp_goal = planner_goal_;
            if (!planner_costmap_ros_->isMapReceived()) {
                std::cout << "[warn] Unable make plan without map" << std::endl;
                continue;
            }
            std::cout << ">>> GlobalPlanner makePlan" << std::endl;
            bool gotPlan = makePlan(temp_goal, *planner_plan_);
        }
        
        boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
    }
}

void GlobalPlannerDemo::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal) {
    planner_goal_ = *goal;
    runPlanner_ = true;
}

bool GlobalPlannerDemo::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));
    //make sure to set the plan to be empty initially
    plan.clear();
    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
        std::cout << "Planner costmap ROS is NULL, unable to create global plan\n";
        return false;
    }
    //get the starting pose of the robot
    geometry_msgs::PoseStamped global_pose;
    if(!getRobotPose(global_pose, planner_costmap_ros_)) {
        std::cout << "Unable to get starting pose of robot, unable to create global plan\n";
        return false;
    }

    const geometry_msgs::PoseStamped& start = global_pose;

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty()) {
        printf("Failed to find a  plan to point (%.2f, %.2f)\n", goal.pose.position.x, goal.pose.position.y);
        return false;
    }
    return true;
}

bool GlobalPlannerDemo::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap) {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try
    {
        tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    }
    catch (tf2::LookupException& ex)
    {
        printf("No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ConnectivityException& ex)
    {
        printf("Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf2::ExtrapolationException& ex)
    {
        printf("Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
    {
        printf("Transform timeout for %s. " \
                            "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                            current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
        return false;
    }

    return true;
}
