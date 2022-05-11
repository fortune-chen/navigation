#include <thread>
#include <algorithm>
#include "ros/rate.h"
#include "ros/log_transfer.h"
#include "tracking_pid/interpolator.h"

#include "ros/ros_type_print.h"

namespace tracking_pid
{


Interpolator::Interpolator() : 
        _rate(50.0),
        robot_frame("base_link"),
        _target_x_vel(0.1),
        _target_x_acc(0.05),
        _target_yaw_vel(0.1),
        _target_yaw_acc(0.05),
        progress_on_section(0.0) {
    ROS_INFO("Interpolator::Interpolator()");
    _path_poses.clear();
    _sections.clear();
    _current_section = nullptr;

    _paused = false;
}

Interpolator::~Interpolator() {
    
}

void Interpolator::startPath() {
    ROS_INFO("Interpolator::startPath()");
    std::lock_guard<std::mutex> lock(_state_mutex);
    _state = kInterpolating;
    std::thread(&Interpolator::updateThread, this, _rate).detach();
}

void Interpolator::stopPath() {
    std::lock_guard<std::mutex> lock(_state_mutex);
    _state = kIdle;
}

void Interpolator::continuePath(const ros::Time &stamp) {

}

void Interpolator::acceptPathFromTopic(const nav_msgs::Path& path) {

}

void Interpolator::acceptGoal() {

}

void Interpolator::preemptGoal() {

}

void Interpolator::processPath(nav_msgs::Path& path) {

}

void Interpolator::updateTarget(const ros::Time &current_real) {
    if (_path_poses.empty()) {
        ROS_INFO("No path poses set");
    }
    if (_state == kIdle) {
        ROS_INFO("Path_interpolator is idle");
    }

    if (!_current_section || ros::Time::now() > _current_section->mSection.section_end_time) {
        if (_sections.size() == 1) {
            ROS_INFO("No loop requested or remaining, finishing up");
            _sections.clear();
            stopPath();
            return;
        }
        auto section_start = _sections.back();
        _sections.pop_back();
        auto section_end = _sections.back();

        _current_section.reset(
            new SectionInterpolation(section_start, section_end, current_real, _target_x_vel, _target_x_acc, _target_yaw_vel, _target_yaw_acc));        
    }

    auto duration_on_section = current_real - _current_section->mSection.section_start_time;

    if (!_current_section->mSection.duration_for_section.isZero()) {
        progress_on_section = duration_on_section.toNSec() / _current_section->mSection.duration_for_section.toNSec();
    } else {
        progress_on_section = 1.0;
    }

    tp = _current_section->interpolateWithAcceleration(ros::Time::now());
    tp.pose.header.stamp = current_real;
    _latest_subgoal_pose = tp.pose;
    std::lock_guard<std::mutex> lock(_listener_mutex);
#if 1 // if true : Consistent with the tracking_pid about publish rate 
    static int count = 3;
    static int pre_count = 3;
    --count;
    if (count == 0) {
        if (pre_count == 3) {
            count = pre_count = 2;
        } else {
            count = pre_count = 3;
        }
        for (auto& l : mTpListeners) {
            l->onTrajectoryPointReceived(tp);
        }
    }
#else
    for (auto& l : mTpListeners) {
        l->onTrajectoryPointReceived(tp);
    }
#endif
}

void Interpolator::updateThread(double frequency) {
    ros::Rate r(frequency);
    while (true) {
        updateTarget(ros::Time::now());
        r.sleep();
        if (r.cycleTime() > ros::Duration(1 / frequency)) {
            ROS_WARN("updateTarget missed its desired rate of %.4fHz... the loop actually took %.4f seconds", frequency, r.cycleTime().toSec());
        }
        if (_state == kIdle) {
            ROS_INFO("Interpolation State is idle, so leave updateThread()");
            break;
        }
    }
}

// Similar to the processPath()
bool Interpolator::setPlan(const nav_msgs::Path& path) {
    ROS_INFO("Interpolator::setPlan()");
    if (path.poses.empty()) {
        ROS_ERROR("There are no poses in the given path");
        return false;
    }
    if (path.header.frame_id.empty()) {
        ROS_ERROR("Frame_id is empty in the given path");
        return false;
    }

    _latest_path_msg = path;
    _path_poses = path.poses;

    for (auto& pose : _path_poses) {
        if (pose.header.frame_id.empty()) {
            pose.header.frame_id = path.header.frame_id;
        }
    }

    _sections = _path_poses;
    reverse(begin(_sections), end(_sections));

    return true;
}

void Interpolator::registerTrajectoryPointListener(const std::shared_ptr<TrajectoryPointListener>& listener) {
    ROS_INFO("Interpolator::registerTrajectoryPointListener()");
    std::lock_guard<std::mutex> lock(_listener_mutex);
    mTpListeners.push_back(listener);
}

void Interpolator::unregisterTrajectoryPointListener(const std::shared_ptr<TrajectoryPointListener>& listener) {
    std::lock_guard<std::mutex> lock(_listener_mutex);
    for (auto iter = mTpListeners.begin(); iter != mTpListeners.end();) {
        if (listener == *iter) {
            iter = mTpListeners.erase(iter);
        } else {
            ++iter;
        }
    }
}

}