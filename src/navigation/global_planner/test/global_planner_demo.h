#ifndef NAV_GLOBAL_PLANNER_DEMO_H_
#define NAV_GLOBAL_PLANNER_DEMO_H_

#include <vector>
#include <string>

#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/GetPlan.h>

class GlobalPlannerDemo {
public:
    
#ifdef USED_TF2_BRIDGE
    GlobalPlannerDemo(tf2_ros::TF2BridgeClient& tf);
#else
    GlobalPlannerDemo(tf2_ros::Buffer& tf);
#endif

    virtual ~GlobalPlannerDemo() {};

    bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);

    void planThread();

    bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

private:
#ifdef USED_TF2_BRIDGE
    tf2_ros::TF2BridgeClient& tf_;
#else
    tf2_ros::Buffer& tf_;
#endif
    costmap_2d::Costmap2DROS* planner_costmap_ros_;
    std::vector<geometry_msgs::PoseStamped>* planner_plan_;
    bool runPlanner_;
    std::string robot_base_frame_;
    boost::thread* planner_thread_;
    boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
    geometry_msgs::PoseStamped planner_goal_;
};


#endif