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
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <cstdio>
#include <string>
#include <algorithm>
#include <vector>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#ifdef USED_TF2_BRIDGE
#include "tf2_bridge/tf2_bridge_client.h"
#endif
// ADD
#include <costmap_2d/static_layer.h>
#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/inflation_layer.h>
#include <ros/rate.h>

// REMOVE: remove in future
#include "ros/log_transfer.h"
#include <sensor_bridge/global_sensor_bridge_client.h>

using namespace std;

namespace costmap_2d
{

#ifdef USED_TF2_BRIDGE
Costmap2DROS::Costmap2DROS(const std::string& name, tf2_ros::TF2BridgeClient& tf) :
#else
Costmap2DROS::Costmap2DROS(const std::string& name, tf2_ros::Buffer& tf) :
#endif
    layered_costmap_(NULL),
    name_(name),
    tf_(tf),
    transform_tolerance_(0.3),
    map_update_thread_shutdown_(false),
    stop_updates_(false),
    initialized_(true),
    stopped_(false),
    robot_stopped_(false),
    map_update_thread_(NULL),
    last_publish_(0),
	  publisher_(NULL),
    footprint_padding_(0.0),
    movement_check_thread_(NULL)
{
  // Initialize old pose with something
  tf2::toMsg(tf2::Transform::getIdentity(), old_pose_.pose);

  if (name_ == "global_costmap") {
    global_frame_ = "map";
    robot_base_frame_ = "base_footprint";
  } else {
    global_frame_ = "odom";
    robot_base_frame_ = "base_footprint";    
  }

  ros::Time last_error = ros::Time::now();
  std::string tf_error;
  double timeout = 5.0;
  // we need to make sure that the transform between the robot base frame and the global frame is available'
  while (!tf_.canTransform(global_frame_, robot_base_frame_, ros::Time(), ros::Duration(0.1), &tf_error))
  {
    if (last_error + ros::Duration(timeout) < ros::Time::now())
    {
      ROS_WARN("transform from %s to %s timed out %lfs in Costmap2DROS\n", robot_base_frame_.c_str(), global_frame_.c_str(), timeout);
      ROS_WARN("Timed out waiting for transform from %s to %s to become available before running costmap, tf error: %s\n",
               robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());
      last_error = ros::Time::now();
    }
    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation.
    tf_error.clear();
  }
  // check if we want a rolling window version of the costmap
  bool rolling_window, track_unknown_space, always_send_full_costmap;
  if (name_ == "global_costmap") {
    rolling_window = false;
    track_unknown_space = false;
    always_send_full_costmap = false;
  } else {
    rolling_window = true;
    track_unknown_space = false;
    always_send_full_costmap = false;
  }

  layered_costmap_ = new LayeredCostmap(global_frame_, rolling_window, track_unknown_space);

  initialLayers(name); // ADD

  // 直接构建 footprint 
  setUnpaddedRobotFootprint(makeFootprint()); // ADD

  publisher_ = new Costmap2DPublisher(layered_costmap_->getCostmap(), global_frame_, "costmap",
                                      always_send_full_costmap);

  // create a thread to handle updating the map
  stop_updates_ = false;
  initialized_ = true;
  stopped_ = false;

  // Create a time r to check if the robot is moving
  robot_stopped_ = false;
  movement_check_thread_ = new boost::thread(boost::bind(&Costmap2DROS::movementCheckLoop, this, 100));

  reconfigure(name); // ADD
}

void Costmap2DROS::setUnpaddedRobotFootprintPolygon(const geometry_msgs::Polygon& footprint)
{
  setUnpaddedRobotFootprint(toPointVector(footprint));
}

Costmap2DROS::~Costmap2DROS()
{
  map_update_thread_shutdown_ = true;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_->join();
    delete map_update_thread_;
  }
  if (publisher_ != NULL)
    delete publisher_;

  delete layered_costmap_;
}

void Costmap2DROS::setUnpaddedRobotFootprint(const std::vector<geometry_msgs::Point>& points)
{
  unpadded_footprint_ = points;
  padded_footprint_ = points;
  padFootprint(padded_footprint_, footprint_padding_);

  layered_costmap_->setFootprint(padded_footprint_);
}

void Costmap2DROS::mapUpdateLoop(double frequency)
{
  // the user might not want to run the loop every cycle
  if (frequency == 0.0)
    return;

  ros::Rate r(frequency);
  while (!map_update_thread_shutdown_)
  {
    #ifdef HAVE_SYS_TIME_H
    struct timeval start, end;
    double start_t, end_t, t_diff;
    gettimeofday(&start, NULL);
    #endif
    
    updateMap();

    #ifdef HAVE_SYS_TIME_H
    gettimeofday(&end, NULL);
    start_t = start.tv_sec + double(start.tv_usec) / 1e6;
    end_t = end.tv_sec + double(end.tv_usec) / 1e6;
    t_diff = end_t - start_t;
    ROS_DEBUG("Map update time: %.9f", t_diff);
    #endif
    
    if (publish_cycle.toSec() > 0 && layered_costmap_->isInitialized())
    {
      unsigned int x0, y0, xn, yn;
      layered_costmap_->getBounds(&x0, &xn, &y0, &yn);
      publisher_->updateBounds(x0, xn, y0, yn);

      ros::Time now = ros::Time::now();
      if (last_publish_ + publish_cycle < now)
      {
        publisher_->publishCostmap();
        last_publish_ = now;
      }
    }
    r.sleep();
    // make sure to sleep for the remainder of our cycle time
    if (r.cycleTime() > ros::Duration(1 / frequency))
      ROS_WARN("Map update loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency,
               r.cycleTime().toSec());
  }
}

void Costmap2DROS::updateMap()
{
  if (!stop_updates_)
  {
    // get global pose
    geometry_msgs::PoseStamped pose;
    if (getRobotPose (pose))
    {
      double x = pose.pose.position.x,
             y = pose.pose.position.y,
             yaw = tf2::getYaw(pose.pose.orientation);

      layered_costmap_->updateMap(x, y, yaw);

      geometry_msgs::PolygonStamped footprint;
      footprint.header.frame_id = global_frame_;
      footprint.header.stamp = ros::Time::now();
      transformFootprint(x, y, yaw, padded_footprint_, footprint);
      // TODO
      // footprint_pub_.publish(footprint);
	    kBridge->sendFootprint(footprint);
      initialized_ = true;
    }
  }
}

void Costmap2DROS::start()
{
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // check if we're stopped or just paused
  if (stopped_)
  {
    // if we're stopped we need to re-subscribe to topics
    for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
        ++plugin)
    {
      (*plugin)->activate();
    }
    stopped_ = false;
  }
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  ros::Rate r(100.0);
  while (!initialized_)
    r.sleep();
}

void Costmap2DROS::stop()
{
  stop_updates_ = true;
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  // unsubscribe from topics
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->deactivate();
  }
  initialized_ = false;
  stopped_ = true;
}

void Costmap2DROS::pause()
{
  stop_updates_ = true;
  initialized_ = false;
}

void Costmap2DROS::resume()
{
  stop_updates_ = false;

  // block until the costmap is re-initialized.. meaning one update cycle has run
  ros::Rate r(100.0);
  while (!initialized_)
    r.sleep();
}


void Costmap2DROS::resetLayers()
{
  Costmap2D* top = layered_costmap_->getCostmap();
  top->resetMap(0, 0, top->getSizeInCellsX(), top->getSizeInCellsY());
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
      ++plugin)
  {
    (*plugin)->reset();
  }
}

bool Costmap2DROS::getRobotPose(geometry_msgs::PoseStamped& global_pose) const
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = ros::Time();
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get the global pose of the robot
  try
  {
    // use current time if possible (makes sure it's not in the future)
    if (tf_.canTransform(global_frame_, robot_base_frame_, current_time))
    {
      geometry_msgs::TransformStamped transform = tf_.lookupTransform(global_frame_, robot_base_frame_, current_time);
      tf2::doTransform(robot_pose, global_pose, transform);
    }
    // use the latest otherwise
    else
    {
      tf_.transform(robot_pose, global_pose, global_frame_);
    }
  }
  catch (tf2::LookupException& ex)
  {
    // TODO Bill
    // ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    ROS_ERROR("No Transform available Error looking up robot pose\n");
    return false;
  }
  catch (tf2::ConnectivityException& ex)
  {
    // TODO Bill
    // ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    ROS_ERROR("Connectivity Error looking up robot pose\n");
    return false;
  }
  catch (tf2::ExtrapolationException& ex)
  {
    // TODO Bill
    // ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    ROS_ERROR("Extrapolation Error looking up robot pose\n");
    return false;
  }
  // check global_pose timeout
  if (current_time.toSec() - global_pose.header.stamp.toSec() > transform_tolerance_)
  {
    // TODO Bill Temp
    // ROS_WARN_THROTTLE(1.0,
    //                   "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
    //                   current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
    ROS_WARN("Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f\n", current_time.toSec(), global_pose.header.stamp.toSec(), transform_tolerance_);
    return false;
  }

  return true;
}

void Costmap2DROS::getOrientedFootprint(std::vector<geometry_msgs::Point>& oriented_footprint) const
{
  geometry_msgs::PoseStamped global_pose;
  if (!getRobotPose(global_pose))
    return;

  double yaw = tf2::getYaw(global_pose.pose.orientation);
  transformFootprint(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
                     padded_footprint_, oriented_footprint);
}

// ADD: 根据ROS Turtlebot3 burger的参数配置，不同机器参数需要调整
void Costmap2DROS::reconfigure(const std::string &costmap_name) {
  Costmap2DConfig config;
  if (costmap_name == "global_costmap") {
    config.transform_tolerance = 0.5;
    config.update_frequency = 10.0;
    config.publish_frequency = 10.0;
    config.width = 10;
    config.height = 10;
    config.resolution = 0.05;
    config.origin_x = 0.0;
    config.origin_y = 0.0;
  } else {
    config.transform_tolerance = 0.5;
    config.update_frequency = 10.0;
    config.publish_frequency = 10.0;
    config.width = 3;
    config.height = 3;
    config.resolution = 0.05;
    config.origin_x = 0.0;
    config.origin_y = 0.0;
  }
  config.footprint_padding = 0.01;

  transform_tolerance_ = config.transform_tolerance;
  if (map_update_thread_ != NULL)
  {
    map_update_thread_shutdown_ = true;
    map_update_thread_->join();
    delete map_update_thread_;
  }
  map_update_thread_shutdown_ = false;
  double map_update_frequency = config.update_frequency;

  double map_publish_frequency = config.publish_frequency;
  if (map_publish_frequency > 0)
    publish_cycle = ros::Duration(1 / map_publish_frequency);
  else
    publish_cycle = ros::Duration(-1);

  // find size parameters
  double map_width_meters = config.width, map_height_meters = config.height, resolution = config.resolution, origin_x =
             config.origin_x,
         origin_y = config.origin_y;

  if (!layered_costmap_->isSizeLocked())
  {
    layered_costmap_->resizeMap((unsigned int)(map_width_meters / resolution),
                                (unsigned int)(map_height_meters / resolution), resolution, origin_x, origin_y);
  }


  // If the padding has changed, call setUnpaddedRobotFootprint() to
  // re-apply the padding.
  if (footprint_padding_ != config.footprint_padding)
  {
    footprint_padding_ = config.footprint_padding;
    setUnpaddedRobotFootprint(unpadded_footprint_);
  }

  map_update_thread_ = new boost::thread(boost::bind(&Costmap2DROS::mapUpdateLoop, this, map_update_frequency));
}


// ADD: 初始化代价地图的各个层
void Costmap2DROS::initialLayers(const std::string &costmap_name) {
  if (costmap_name == "global_costmap") {
    boost::shared_ptr<Layer> static_plugin(new StaticLayer());
    layered_costmap_->addPlugin(static_plugin);
    static_plugin->initialize(layered_costmap_, costmap_name + "/static_layer", &tf_);
  }

  // TODO : Bill
  boost::shared_ptr<Layer> obstacle_plugin(new ObstacleLayer());
  layered_costmap_->addPlugin(obstacle_plugin);
  obstacle_plugin->initialize(layered_costmap_, costmap_name + "/obstacle_layer", &tf_);

  boost::shared_ptr<Layer> inflation_plugin(new InflationLayer());
  layered_costmap_->addPlugin(inflation_plugin);
  inflation_plugin->initialize(layered_costmap_, costmap_name + "/inflation_layer", &tf_);
}

// ADD: 根据预定义好的footprint，返回footprint点的集合
std::vector<geometry_msgs::Point> Costmap2DROS::makeFootprint() {
  std::vector<geometry_msgs::Point> footprint;
  std::string footprintString = "[[-0.105, -0.105], [-0.105, 0.105], [0.041, 0.105], [0.041, -0.105]]";
  makeFootprintFromString(footprintString, footprint);
  return footprint;
}

// ADD: 用于检测机器是否运动的线程函数
void Costmap2DROS::movementCheckLoop(int duration_ms) {
  while (true) {
    geometry_msgs::PoseStamped new_pose;

    if (!getRobotPose(new_pose)) {
      std::cout << "Could not get robot pose\n";
      robot_stopped_ = false;
    }
    // make sure that the robot is not moving
    else {
      old_pose_ = new_pose;
      robot_stopped_ = (tf2::Vector3(old_pose_.pose.position.x, old_pose_.pose.position.y,
                                    old_pose_.pose.position.z).distance(tf2::Vector3(new_pose.pose.position.x,
                                        new_pose.pose.position.y, new_pose.pose.position.z)) < 1e-3) &&
                      (tf2::Quaternion(old_pose_.pose.orientation.x,
                                        old_pose_.pose.orientation.y,
                                        old_pose_.pose.orientation.z,
                                        old_pose_.pose.orientation.w).angle(tf2::Quaternion(new_pose.pose.orientation.x,
                                            new_pose.pose.orientation.y,
                                            new_pose.pose.orientation.z,
                                            new_pose.pose.orientation.w)) < 1e-3);
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(duration_ms));
  }
}

bool Costmap2DROS::isMapReceived() {
  if (!layered_costmap_) {
    return false;
  }
  std::vector < boost::shared_ptr<Layer> > *plugins = layered_costmap_->getPlugins();
  for (vector<boost::shared_ptr<Layer> >::iterator plugin = plugins->begin(); plugin != plugins->end();
        ++plugin)
  {
    if ((*plugin)->getName() == "global_costmap/static_layer") {
      if ((*plugin)->isMapReceived()) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace costmap_2d
