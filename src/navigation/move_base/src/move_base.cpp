/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*         Mike Phillips (put the planner in its own thread)
*********************************************************************/
#include <move_base/move_base.h>
#include <move_base/RecoveryStatus.h>
#include <cmath>

#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <geometry_msgs/Twist.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ADD
#include <global_planner/planner_core.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <clear_costmap_recovery/clear_costmap_recovery.h>
#include <sensor_bridge/global_sensor_bridge_client.h>
#include <ros/log_transfer.h>
#include "ros/rate.h"


namespace move_base {

#ifdef USED_TF2_BRIDGE
MoveBase::MoveBase(tf2_ros::TF2BridgeClient& tf) :
#else
MoveBase::MoveBase(tf2_ros::Buffer& tf) :
#endif
    tf_(tf),
    planner_costmap_ros_(NULL), controller_costmap_ros_(NULL),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    runPlanner_(false), setup_(false), p_freq_change_(false), c_freq_change_(false), new_global_plan_(false) {

    kBridge->registerGoalCallback([this](const geometry_msgs::PoseStampedConstPtr& goal) {
        ROS_INFO("registerGoalCallback");
        executeGoal(goal);
    });

    recovery_trigger_ = PLANNING_R;
    configParameter();

    //set up plan triple buffer
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
    controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //set up the planner's thread
    planner_thread_ = new boost::thread(boost::bind(&MoveBase::planThread, this));

    // TODO-Bill
    //for commanding the base
    // current_goal_pub_ = private_nh.advertise<geometry_msgs::PoseStamped>("current_goal", 0 );

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    planner_.reset(new global_planner::GlobalPlanner());
    planner_->initialize("", planner_costmap_ros_);

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    ROS_INFO("Created global_planner");
    controller_costmap_ros_->pause();

    //create a local planner
    tc_.reset(new dwa_local_planner::DWAPlannerROS());
    ROS_INFO("Created local_planner");
    tc_->initialize("", &tf_, controller_costmap_ros_);

    // Start actively updating costmaps based on sensor data
    planner_costmap_ros_->start();
    controller_costmap_ros_->start();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if (shutdown_costmaps_) {
        ROS_DEBUG("[move_base] Stopping costmaps initially");
        planner_costmap_ros_->stop();
        controller_costmap_ros_->stop();
    }

    // TODO Bill
    //load any user specified recovery behaviors, and if that fails load the defaults
    loadDefaultRecoveryBehaviors();

    //initially, we'll need to make a plan
    state_ = PLANNING;

    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;


#ifdef ENABLE_FULL_COVERAGE_PATH_PLANNER
    coverage_planner_.reset(new full_coverage_path_planner::SpiralSTC());
    coverage_planner_->initialize("full_coverage_path_planner", nullptr);

    kBridge->registerFullCoverageCallback([this](const geometry_msgs::PoseStampedConstPtr& init) {
        ROS_INFO("registerFullCoverageCallback");
        geometry_msgs::PoseStamped start;
        geometry_msgs::PoseStamped goal;
        std::vector<geometry_msgs::PoseStamped> plan;
        start.pose.position = init->pose.position;
        start.pose.orientation = init->pose.orientation;
        coverage_planner_->makePlan(start, goal, plan);
        // std::thread(&MoveBase::executeFcppThread, this, plan).detach(); // TODO : Bill
    });

    mTracker.reset(new tracking_pid::PidTracker());
#endif
}

void MoveBase::clearCostmapWindows(double size_x, double size_y){
    geometry_msgs::PoseStamped global_pose;

    //clear the planner's costmap
    getRobotPose(global_pose, planner_costmap_ros_);

    std::vector<geometry_msgs::Point> clear_poly;
    double x = global_pose.pose.position.x;
    double y = global_pose.pose.position.y;
    geometry_msgs::Point pt;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    planner_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);

    //clear the controller's costmap
    getRobotPose(global_pose, controller_costmap_ros_);

    clear_poly.clear();
    x = global_pose.pose.position.x;
    y = global_pose.pose.position.y;

    pt.x = x - size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y - size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x + size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    pt.x = x - size_x / 2;
    pt.y = y + size_y / 2;
    clear_poly.push_back(pt);

    controller_costmap_ros_->getCostmap()->setConvexPolygonCost(clear_poly, costmap_2d::FREE_SPACE);
}

MoveBase::~MoveBase(){
    recovery_behaviors_.clear();

    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;

    if(controller_costmap_ros_ != NULL)
      delete controller_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

    delete planner_plan_;
    delete latest_plan_;
    delete controller_plan_;

    planner_.reset();
    tc_.reset();
}

bool MoveBase::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if (planner_costmap_ros_ == NULL) {
        ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
        return false;
    }

    //get the starting pose of the robot
    geometry_msgs::PoseStamped global_pose;
    if (!getRobotPose(global_pose, planner_costmap_ros_)) {
        ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
        return false;
    }

    const geometry_msgs::PoseStamped& start = global_pose;

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
        ROS_DEBUG("[move_base] Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
        return false;
    }

    return true;
}

void MoveBase::publishZeroVelocity(){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    kBridge->sendVelocity(cmd_vel);
}

bool MoveBase::isQuaternionValid(const geometry_msgs::Quaternion& q){
    //first we need to check if the quaternion has nan's or infs
    if (!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) {
        ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
        return false;
    }

    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if (tf_q.length2() < 1e-6) {
        ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
        return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf2::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if (fabs(dot - 1) > 1e-3) {
        ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
        return false;
    }

    return true;
}

geometry_msgs::PoseStamped MoveBase::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg) {
    std::string global_frame = planner_costmap_ros_->getGlobalFrameID();
    geometry_msgs::PoseStamped goal_pose, global_pose;
    goal_pose = goal_pose_msg;

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.header.stamp = ros::Time();

    try {
        tf_.transform(goal_pose_msg, global_pose, global_frame);
    } catch(tf2::TransformException& ex) {
        ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
            goal_pose.header.frame_id.c_str(), global_frame.c_str(), ex.what());
        return goal_pose_msg;
    }

    return global_pose;
}

void MoveBase::wakePlanner(const ros::Duration& duration) {
    int duration_ms = duration.toSec() * 1000;
    boost::this_thread::sleep_for(boost::chrono::milliseconds(duration_ms));
    planner_cond_.notify_one();
}


void MoveBase::planThread() {
    ROS_DEBUG("[move_base_plan_thread] Starting planner thread...");
    bool wait_for_wake = false;
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    while (true) {
        //check if we should run the planner (the mutex is locked)
        while (wait_for_wake || !runPlanner_) {
            //if we should not be running the planner then suspend this thread
            ROS_DEBUG("move_base_plan_thread] Planner thread is suspending");
            planner_cond_.wait(lock);
            wait_for_wake = false;
        }
        ros::Time start_time = ros::Time::now();

        //time to plan! get a copy of the goal and unlock the mutex
        geometry_msgs::PoseStamped temp_goal = planner_goal_;
        lock.unlock();
        ROS_DEBUG("[move_base_plan_thread] Planning...");

        //run planner
        planner_plan_->clear();
        bool gotPlan = makePlan(temp_goal, *planner_plan_);

        if (gotPlan) {
            ROS_DEBUG("[move_base_plan_thread] Got Plan with %zu points!", planner_plan_->size());
            //pointer swap the plans under mutex (the controller will pull from latest_plan_)
            std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

            lock.lock();
            planner_plan_ = latest_plan_;
            latest_plan_ = temp_plan;
            last_valid_plan_ = ros::Time::now();
            planning_retries_ = 0;
            new_global_plan_ = true;

            ROS_DEBUG("[move_base_plan_thread] Generated a plan from the base_global_planner");

            //make sure we only start the controller if we still haven't reached the goal
            if (runPlanner_) {
                state_ = CONTROLLING;
            }
            if (planner_frequency_ <= 0) {
                runPlanner_ = false;
            } 
            lock.unlock();
        }
        //if we didn't get a plan and we are in the planning state (the robot isn't moving)
        else if (state_ == PLANNING) {
            ROS_DEBUG("[move_base_plan_thread] No Plan...");
            ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

            //check if we've tried to make a plan for over our time limit or our maximum number of retries
            //issue #496: we stop planning when one of the conditions is true, but if max_planning_retries_
            //is negative (the default), it is just ignored and we have the same behavior as ever
            lock.lock();
            planning_retries_++;
            if (runPlanner_ &&
               (ros::Time::now() > attempt_end || planning_retries_ > uint32_t(max_planning_retries_))) {
                //we'll move into our obstacle clearing mode
                state_ = CLEARING;
                runPlanner_ = false;  // proper solution for issue #523
                publishZeroVelocity();
                recovery_trigger_ = PLANNING_R;
            }

            lock.unlock();
        }

        //take the mutex for the next iteration
        lock.lock();

        //setup sleep interface if needed
        if (planner_frequency_ > 0) {
            ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
            if (sleep_time > ros::Duration(0.0)) {
                wait_for_wake = true;
                // TODO Bill , should use timer
                std::thread([this, sleep_time]() {
                    wakePlanner(sleep_time);
                }).detach();
            }
        }
    }
}

void MoveBase::executeGoal(const geometry_msgs::PoseStamped::ConstPtr& target_goal) {
    if (!isQuaternionValid(target_goal->pose.orientation)) {
        ROS_ERROR("Aborting on goal because it was sent with an invalid quaternion");
        assert(false);
        return;
    }
    if (!planner_costmap_ros_->isMapReceived()) {
        ROS_ERROR("Nothing can be done without a map");
        return;
    }

    geometry_msgs::PoseStamped goal = goalToGlobalFrame(*target_goal);

    publishZeroVelocity();
    //we have a goal so start the planner
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    // current_goal_pub_.publish(goal); // TODO-Bill
    std::vector<geometry_msgs::PoseStamped> global_plan;

    ros::Rate r(controller_frequency_);
    if (shutdown_costmaps_) {
        ROS_DEBUG("[move_base] Starting up costmaps that were shut down previously");
        planner_costmap_ros_->start();
        controller_costmap_ros_->start();
    }

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    while (true) {
        if (c_freq_change_) {
            ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
            // r = ros::Rate(controller_frequency_); // TODO-Bill
            c_freq_change_ = false;
        }

        //we also want to check if we've changed global frames because we need to transform our goal pose
        if (goal.header.frame_id != planner_costmap_ros_->getGlobalFrameID()) {
            goal = goalToGlobalFrame(goal);

            //we want to go back to the planning state for the next execution cycle
            recovery_index_ = 0;
            state_ = PLANNING;

            //we have a new goal so make sure the planner is awake
            lock.lock();
            planner_goal_ = goal;
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();

            //publish the goal point to the visualizer
            ROS_DEBUG("[move_base] The global frame for move_base has changed, new frame: %s, new goal position x: %.2f, y: %.2f", goal.header.frame_id.c_str(), goal.pose.position.x, goal.pose.position.y);
            // current_goal_pub_.publish(goal); // TODO-Bill

            //make sure to reset our timeouts and counters
            last_valid_control_ = ros::Time::now();
            last_valid_plan_ = ros::Time::now();
            last_oscillation_reset_ = ros::Time::now();
            planning_retries_ = 0;
        }

        //for timing that gives real time even in simulation
        ros::WallTime start = ros::WallTime::now();

        //the real work on pursuing a goal is done here
        bool done = executeCycle(goal, global_plan);

        //if we're done, then we'll return from execute
        if (done)
            return;

        //check if execution of the goal has completed in some way

        ros::WallDuration t_diff = ros::WallTime::now() - start;
        ROS_DEBUG("[move_base] Full control cycle time: %.9f", t_diff.toSec());

        r.sleep();
        //make sure to sleep for the remainder of our cycle time
        if (r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
            ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

    //wake up the planner thread so that it can exit cleanly
    lock.lock();
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    //if the node is killed then we'll abort and return
    ROS_ERROR("Aborting on the goal because the node has been killed");
    // assert(move_base_msgs::MoveBaseResult()); // TODO-Bill
    return;
}

double MoveBase::distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
    return hypot(p1.pose.position.x - p2.pose.position.x, p1.pose.position.y - p2.pose.position.y);
}

bool MoveBase::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan){
    boost::recursive_mutex::scoped_lock ecl(configuration_mutex_);
    //we need to be able to publish velocity commands
    geometry_msgs::Twist cmd_vel;

    //update feedback to correspond to our curent position
    geometry_msgs::PoseStamped global_pose;
    getRobotPose(global_pose, planner_costmap_ros_);
    const geometry_msgs::PoseStamped& current_position = global_pose;

    //check to see if we've moved far enough to reset our oscillation timeout
    if (distance(current_position, oscillation_pose_) >= oscillation_distance_) {
        last_oscillation_reset_ = ros::Time::now();
        oscillation_pose_ = current_position;

        //if our last recovery was caused by oscillation, we want to reset the recovery index
        if (recovery_trigger_ == OSCILLATION_R)
            recovery_index_ = 0;
    }

    //check that the observation buffers for the costmap are current, we don't want to drive blind
    if (!controller_costmap_ros_->isCurrent()) {
        ROS_WARN("Sensor data is out of date, we're not going to allow commanding of the base for safety");
        publishZeroVelocity();
        return false;
    }

    //if we have a new plan then grab it and give it to the controller
    if (new_global_plan_) {
        //make sure to set the new plan flag to false
        new_global_plan_ = false;

        ROS_DEBUG("[move_base] Got a new plan...swap pointers");

        //do a pointer swap under mutex
        std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
        controller_plan_ = latest_plan_;
        latest_plan_ = temp_plan;
        lock.unlock();
        ROS_DEBUG("[move_base] pointers swapped!");

        if (!tc_->setPlan(*controller_plan_)) {
            //ABORT and SHUTDOWN COSTMAPS
            ROS_ERROR("Failed to pass global plan to the controller, aborting.");
            resetState();

            //disable the planner thread
            lock.lock();
            runPlanner_ = false;
            lock.unlock();

            // as_->setAborted(move_base_msgs::MoveBaseResult(), "Failed to pass global plan to the controller.");
            ROS_ERROR("Aborting because failed to pass global plan to the controller.");
            assert(false);
            return true;
        }

        //make sure to reset recovery_index_ since we were able to find a valid plan
        if (recovery_trigger_ == PLANNING_R)
            recovery_index_ = 0;
    }

    //the move_base state machine, handles the control logic for navigation
    switch (state_) {
        //if we are in a planning state, then we'll attempt to make a plan
        case PLANNING:
            {
                boost::recursive_mutex::scoped_lock lock(planner_mutex_);
                runPlanner_ = true;
                planner_cond_.notify_one();
            }
            ROS_DEBUG("[move_base] Waiting for plan, in the planning state.");
            break;

        //if we're controlling, we'll attempt to find valid velocity commands
        case CONTROLLING:
            ROS_DEBUG("[move_base] In controlling state.");

            //check to see if we've reached our goal
            if (tc_->isGoalReached()) {
                ROS_DEBUG("[move_base] Goal reached!");
                resetState();

                //disable the planner thread
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                runPlanner_ = false;
                lock.unlock();

                // as_->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached."); // TODO-Bill
                return true;
            }

            //check for an oscillation condition
            if (oscillation_timeout_ > 0.0 &&
                last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now()) {
                publishZeroVelocity();
                state_ = CLEARING;
                recovery_trigger_ = OSCILLATION_R;
            }

            {
                boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_ros_->getCostmap()->getMutex()));

                  if (tc_->computeVelocityCommands(cmd_vel)) {
                      ROS_DEBUG( "[move_base] Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                                        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
                      last_valid_control_ = ros::Time::now();
                      //make sure that we send the velocity command to the base
                      kBridge->sendVelocity(cmd_vel);
                      if (recovery_trigger_ == CONTROLLING_R) {
                          recovery_index_ = 0;
                      }
                }
                else {
                    ROS_DEBUG("[move_base] The local planner could not find a valid plan.");
                    ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

                    //check if we've tried to find a valid control for longer than our time limit
                    if (ros::Time::now() > attempt_end) {
                        //we'll move into our obstacle clearing mode
                        publishZeroVelocity();
                        state_ = CLEARING;
                        recovery_trigger_ = CONTROLLING_R;
                    } else {
                        //otherwise, if we can't find a valid control, we'll go back to planning
                        last_valid_plan_ = ros::Time::now();
                        planning_retries_ = 0;
                        state_ = PLANNING;
                        publishZeroVelocity();

                        //enable the planner thread in case it isn't running on a clock
                        boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                        runPlanner_ = true;
                        planner_cond_.notify_one();
                        lock.unlock();
                    }
                }
            }
            break;

        //we'll try to clear out space with any user-provided recovery behaviors
        case CLEARING:
            ROS_DEBUG("[move_base] In clearing/recovery state");
            //we'll invoke whatever recovery behavior we're currently on if they're enabled
            if (recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size()) {
                ROS_DEBUG("[move_base_recovery] Executing behavior %u of %zu", recovery_index_+1, recovery_behaviors_.size());

                move_base_msgs::RecoveryStatus msg;
                msg.pose_stamped = current_position;
                msg.current_recovery_number = recovery_index_;
                msg.total_number_of_recoveries = recovery_behaviors_.size();
                msg.recovery_behavior_name =  recovery_behavior_names_[recovery_index_];

                // recovery_status_pub_.publish(msg); // TODO-Bill

                recovery_behaviors_[recovery_index_]->runBehavior();

                //we at least want to give the robot some time to stop oscillating after executing the behavior
                last_oscillation_reset_ = ros::Time::now();

                //we'll check if the recovery behavior actually worked
                ROS_DEBUG("[move_base_recovery] Going back to planning state");
                last_valid_plan_ = ros::Time::now();
                planning_retries_ = 0;
                state_ = PLANNING;

                //update the index of the next recovery behavior that we'll try
                recovery_index_++;
            } else {
                ROS_DEBUG("[move_base_recovery] All recovery behaviors have failed, locking the planner and disabling it.");
                //disable the planner thread
                boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
                runPlanner_ = false;
                lock.unlock();

                ROS_DEBUG("[move_base_recovery] Something should abort after this.");

                if (recovery_trigger_ == CONTROLLING_R) {
                    ROS_ERROR("[error] Aborting because a valid control could not be found. Even after executing all recovery behaviors");
                    assert(false);
                } else if(recovery_trigger_ == PLANNING_R){
                    ROS_ERROR("[error] Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
                    assert(false);
                } else if(recovery_trigger_ == OSCILLATION_R){
                    ROS_ERROR("[error] Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
                    assert(false);
                }
                resetState();
                return true;
            }
            break;
        default:
            ROS_ERROR("[error] This case should never be reached, something is wrong, aborting");
            resetState();
            //disable the planner thread
            boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
            runPlanner_ = false;
            lock.unlock();
            ROS_ERROR("[error] Reached a case that should not be hit in move_base. This is a bug, please report it.");
            assert(false);
            return true;
    }

    //we aren't done yet
    return false;
}

//we'll load our default recovery behaviors here
void MoveBase::loadDefaultRecoveryBehaviors(){
    recovery_behaviors_.clear();
    try {
        //first, we'll load a recovery behavior to clear the costmap
        boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(new clear_costmap_recovery::ClearCostmapRecovery());
        cons_clear->initialize("conservative_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("conservative_reset");
        recovery_behaviors_.push_back(cons_clear);

        //next, we'll load a recovery behavior to rotate in place
        boost::shared_ptr<nav_core::RecoveryBehavior> rotate(new clear_costmap_recovery::ClearCostmapRecovery());
        if (clearing_rotation_allowed_) {
            rotate->initialize("rotate_recovery", &tf_, planner_costmap_ros_, controller_costmap_ros_);
            recovery_behavior_names_.push_back("rotate_recovery");
            recovery_behaviors_.push_back(rotate);
        }

        //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
        boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(new clear_costmap_recovery::ClearCostmapRecovery());
        ags_clear->initialize("aggressive_reset", &tf_, planner_costmap_ros_, controller_costmap_ros_);
        recovery_behavior_names_.push_back("aggressive_reset");
        recovery_behaviors_.push_back(ags_clear);

        //we'll rotate in-place one more time
        if (clearing_rotation_allowed_) {
            recovery_behaviors_.push_back(rotate);
            recovery_behavior_names_.push_back("rotate_recovery");
        }
    } catch(...) {
      ROS_FATAL("loadDefaultRecoveryBehaviors() error.");
    }

    return;
}

void MoveBase::resetState() {
    // Disable the planner thread
    boost::unique_lock<boost::recursive_mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    recovery_index_ = 0;
    recovery_trigger_ = PLANNING_R;
    publishZeroVelocity();

    //if we shutdown our costmaps when we're deactivated... we'll do that now
    if (shutdown_costmaps_) {
        ROS_DEBUG("[move_base] Stopping costmaps");
        planner_costmap_ros_->stop();
        controller_costmap_ros_->stop();
    }
}

bool MoveBase::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap) {
    tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
    geometry_msgs::PoseStamped robot_pose;
    tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
    robot_pose.header.frame_id = robot_base_frame_;
    robot_pose.header.stamp = ros::Time(); // latest available
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get robot pose on the given costmap frame
    try {
        tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
    } catch (tf2::LookupException& ex) {
        ROS_ERROR("No Transform available Error looking up robot pose: %s", ex.what());
        return false;
    } catch (tf2::ConnectivityException& ex) {
        ROS_ERROR("Connectivity Error looking up robot pose: %s", ex.what());
        return false;
    } catch (tf2::ExtrapolationException& ex) {
        ROS_ERROR("Extrapolation Error looking up robot pose: %s", ex.what());
        return false;
    }

    // check if global_pose time stamp is within costmap transform tolerance
    if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance()) {
        // TODO Bill Temp
        ROS_WARN("Transform timeout for %s. " \
                          "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                          current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
        return false;
    }
  
    return true;
  }


void MoveBase::configParameter() {
    //get some parameters that will be global to the move base node
    robot_base_frame_ = "base_footprint";
    global_frame_ = "map";
    planner_frequency_ = 5.000000;
    controller_frequency_ = 10.000000;
    planner_patience_ = 5.000000;
    controller_patience_ = 15.000000;
    max_planning_retries_ = -1;
    oscillation_timeout_ = 10.000000;
    oscillation_distance_ = 0.500000;
    make_plan_clear_costmap_ = true;
    make_plan_add_unreachable_goal_ = true;

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    inscribed_radius_ = 0.325000;
    circumscribed_radius_ = 0.460000;
    clearing_radius_ = 0.460000;
    conservative_reset_dist_ = 3.000000;
    shutdown_costmaps_ = false;
    clearing_rotation_allowed_ = true;
    recovery_behavior_enabled_ = true;
}

#ifdef ENABLE_FULL_COVERAGE_PATH_PLANNER
void MoveBase::executeFcppThread(const std::vector<geometry_msgs::PoseStamped> goalList) {
    // boost::unique_lock<boost::recursive_mutex> lock(fcpp_mutex_);
    for (const auto& goal : goalList) {
        geometry_msgs::PoseStamped *pose = new geometry_msgs::PoseStamped;
        *pose = goal;
        pose->header.frame_id = "map";
        geometry_msgs::PoseStampedConstPtr ptr(pose);
        executeGoal(ptr);
        // fcpp_cond_.wait(lock);
    }
}
#endif

};
