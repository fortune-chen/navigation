/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * Author: Bhaskara Marthi
 *         David V. Lu!!
 *********************************************************************/

// 该宏把TF代码包起来
#define ENABLE_TF_CODE

#include "global_planner/planner_core.h"
#include <boost/shared_ptr.hpp>
#include "costmap_2d/costmap_2d_ros.h"
#ifdef ENABLE_TF_CODE
#include <tf2_ros/transform_listener.h>
#endif
namespace cm = costmap_2d;
namespace rm = geometry_msgs;

using std::vector;
using rm::PoseStamped;
using std::string;
using cm::Costmap2D;
using cm::Costmap2DROS;

costmap_2d::Costmap2DROS* planner_costmap_ros_;
boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;

int main(int argc, char** argv) {
#ifdef ENABLE_TF_CODE
    // tf2_ros::Buffer buffer = tf2_ros::Buffer::getInstance();
    tf2_ros::TransformListener tf(tf2_ros::Buffer::getInstance());
#endif
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf2_ros::Buffer::getInstance());;
    planner_costmap_ros_->pause();
    planner_.reset(new global_planner::GlobalPlanner());
    planner_->initialize("global_planner", planner_costmap_ros_);

    planner_costmap_ros_->start();
    const geometry_msgs::PoseStamped start; // TODO
    const geometry_msgs::PoseStamped goal; // TODO
    std::vector<geometry_msgs::PoseStamped> plan;

    planner_->makePlan(start, goal, plan);
    pause();
    return 0;
}

