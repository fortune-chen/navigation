#include <cmath>
#include <cstdio>
#include <iostream>
#include "tf/transform_datatypes.h"
#include "tracking_pid/section_interpolation.h"
#include "ros/log_transfer.h"

namespace tracking_pid
{

SectionInterpolation::SectionInterpolation(const geometry_msgs::PoseStamped& from,
                                           const geometry_msgs::PoseStamped& to,
                                           const ros::Time &stamp,
                                           double x_vel,
                                           double x_acc,
                                           double yaw_vel,
                                           double yaw_acc) {
    if (x_vel == 0
        || x_acc == 0
        || yaw_vel == 0
        || yaw_acc== 0) {
        ROS_ERROR("x_vel / x_acc / yaw_vel / yaw_acc, cannot be zero !!!");
        assert(false);
    }

    mSection._x_vel = x_vel;
    mSection._x_acc_decc = x_acc;
    mSection._yaw_vel = yaw_vel;
    mSection._yaw_acc_decc = yaw_acc;

    mSection.duration_on_section = ros::Duration(0);
    mSection.section_start_pose_stamped = from;
    mSection.section_end_pose_stamped = to;

    mSection._start_xyz[0] = from.pose.position.x;
    mSection._start_xyz[1] = from.pose.position.y;
    mSection._start_xyz[2] = from.pose.position.z;

    double end_xyz[3];
    end_xyz[0] = to.pose.position.x;
    end_xyz[1] = to.pose.position.y;
    end_xyz[2] = to.pose.position.z;

    mSection._start_yaw = tf::getYaw(from.pose.orientation);
    double end_yaw = tf::getYaw(to.pose.orientation);

    mSection._delta[0] = end_xyz[0] - mSection._start_xyz[0];
    mSection._delta[1] = end_xyz[1] - mSection._start_xyz[1];
    mSection._delta[2] = end_xyz[2] - mSection._start_xyz[2];

    mSection._delta_yaw = end_yaw - mSection._start_yaw;
    // 加 2 * M_PI 是为了保证 _delta_yaw 不为负
    mSection._delta_yaw = fmod(mSection._delta_yaw + M_PI + 2 * M_PI, 2 * M_PI) - M_PI;

    if (mSection._delta[0] != 0.0) {
        mSection.length_of_section = fabs(mSection._delta[0]);
    } else {
        mSection.length_of_section = fabs(mSection._delta[1]);
    }
    
    mSection.length_of_section_ang = fabs(mSection._delta_yaw);

    // Time during (de) and acceleration phases t = v/a
    mSection.time_x_acc_decc = mSection._x_vel / mSection._x_acc_decc;    
    // Translation during acceleration phase x = 0.5a*t^2
    mSection.length_x_acc_decc = 0.5 * mSection._x_acc_decc * (mSection.time_x_acc_decc * mSection.time_x_acc_decc);
    // Translation during constant velocity phase
    mSection.length_x_vel = mSection.length_of_section - 2 * mSection.length_x_acc_decc;
    // Time during constant velocity phase t = v/a
    mSection.time_x_vel  = mSection.length_x_vel / mSection._x_vel;
    mSection._x_vel_adjusted = mSection._x_vel;

    // No constant acceleration phase. Recompute (de)-acceleration phase
    if (mSection.time_x_vel < 0) {
        mSection.length_x_acc_decc = 0.5 * mSection.length_of_section;
        // x = 0.5a*t^2 -> 2x/a = t^2 -> t = sqrt(2x/a)
        mSection.time_x_acc_decc = sqrt(2 * mSection.length_x_acc_decc / mSection._x_acc_decc);
        mSection._x_vel_adjusted = mSection._x_acc_decc * mSection.time_x_acc_decc;
        mSection.length_x_vel = 0.0;
        mSection.time_x_vel = 0.0;
    }

    // Time during acceleration phase t = v/a
    mSection.time_yaw_acc_decc = mSection._yaw_vel / mSection._yaw_acc_decc;
    // Translation during acceleration phase x = 0.5a*t^2
    mSection.length_yaw_acc_decc = 0.5 * mSection._yaw_acc_decc * (mSection.time_yaw_acc_decc * mSection.time_yaw_acc_decc);
    // Translation during constant velocity phase
    mSection.length_yaw_vel = mSection.length_of_section_ang - 2 * mSection.length_yaw_acc_decc;
    // Time during constant velocity phase t = v/a
    mSection.time_yaw_vel  = mSection.length_yaw_vel / mSection._yaw_vel;
    mSection._yaw_vel_adjusted = mSection._yaw_vel;

    // No constant acceleration phase. Recompute (de)-acceleration phase
    if (mSection.time_yaw_vel < 0) {
        mSection.length_yaw_acc_decc = 0.5 * mSection.length_of_section_ang;
        // x = 0.5a*t^2 -> 2x/a = t^2 -> t = sqrt(2x/a)
        mSection.time_yaw_acc_decc = sqrt(2 * mSection.length_yaw_acc_decc / mSection._yaw_acc_decc);
        mSection._yaw_vel_adjusted = mSection._yaw_acc_decc*mSection.time_yaw_acc_decc;
        mSection.length_yaw_vel = 0.0;
        mSection.time_yaw_vel = 0.0;
    }

    mSection.duration_for_section_x = 2 * mSection.time_x_acc_decc + mSection.time_x_vel;
    mSection.duration_for_section_yaw = 2 * mSection.time_yaw_acc_decc + mSection.time_yaw_vel;
    if (mSection.duration_for_section_x > mSection.duration_for_section_yaw) {
        mSection.duration_for_section = ros::Duration(mSection.duration_for_section_x);
    } else {
        mSection.duration_for_section = ros::Duration(mSection.duration_for_section_yaw);
    }
    
    mSection.section_start_time = stamp; // MARK : Bill
    mSection.x_progress = 0.0;
    mSection.yaw_progress = 0.0;
    mSection.current_x_vel = 0.0;
    mSection.current_yaw_vel = 0.0;
    mSection.section_end_time = mSection.section_start_time + mSection.duration_for_section;

    printf("************************** SectionInterpolation *****************************\n");
    printf("from [%lf, %lf](%lf, %lf, %lf, %lf)\n", from.pose.position.x, from.pose.position.y, from.pose.orientation.x, from.pose.orientation.y, from.pose.orientation.z, from.pose.orientation.w);
    printf("to   [%lf, %lf](%lf, %lf, %lf, %lf)\n", to.pose.position.x, to.pose.position.y, to.pose.orientation.x, to.pose.orientation.y, to.pose.orientation.z, to.pose.orientation.w);
    if (mSection.length_of_section > 0) {
        printf("_x_vel: %f\n", mSection._x_vel);
        printf("time_x_vel: %f\n", mSection.time_x_vel);
        printf("length_x_vel: %f\n", mSection.length_x_vel);
        printf("_x_acc_decc: %f\n", mSection._x_acc_decc);
        printf("time_x_acc_decc: %f\n", mSection.time_x_acc_decc);
        printf("length_x_acc_decc: %f\n", mSection.length_x_acc_decc);
        printf("length_of_section: %f\n", mSection.length_of_section);
        printf("duration_for_section_x: %f\n", mSection.duration_for_section_x);
    }
    if (mSection.length_of_section_ang > 0) {
        printf("_yaw_vel: %f\n", mSection._yaw_vel);
        printf("time_yaw_vel: %f\n", mSection.time_yaw_vel);
        printf("length_yaw_vel: %f\n", mSection.length_yaw_vel);
        printf("_yaw_acc_decc: %f\n", mSection._yaw_acc_decc);
        printf("time_yaw_acc_decc: %f\n", mSection.time_yaw_acc_decc);
        printf("length_yaw_acc_decc: %f\n", mSection.length_yaw_acc_decc);
        printf("length_of_section_ang: %f\n", mSection.length_of_section_ang);
        printf("duration_for_section_yaw: %f\n", mSection.duration_for_section_yaw);
    }
    printf("duration_for_section: %f\n", mSection.duration_for_section.toSec());
    printf("[section_start_time: %lf]-[section_end_time: %lf]\n", mSection.section_start_time.toSec(), mSection.section_end_time.toSec());
    printf("*****************************************************************************\n");
}

SectionInterpolation::~SectionInterpolation() {

}

TrajectoryPoint SectionInterpolation::interpolateWithAcceleration(const ros::Time &stamp) {
    auto current_section_time = stamp - mSection.section_start_time; // TODO : Bill
    auto t = current_section_time.toSec(), tr = 0.0;

    if  (t < mSection.time_x_acc_decc) {
        tr = t;
        mSection.x_progress = 0.5 * mSection._x_acc_decc * tr * tr;
        mSection.current_x_vel = mSection._x_acc_decc*tr;
        if (mSection.x_progress > mSection.length_x_acc_decc) {
            mSection.current_x_vel = mSection._x_vel_adjusted;
            mSection.x_progress = mSection.length_x_acc_decc;
        }
    } else if (t < mSection.time_x_acc_decc + mSection.time_x_vel) {
        tr = t - mSection.time_x_acc_decc;
        mSection.x_progress = mSection.length_x_acc_decc + mSection.current_x_vel*tr;
        mSection.current_x_vel = mSection._x_vel_adjusted;
        if (mSection.x_progress > mSection.length_x_acc_decc + mSection.length_x_vel) {
            mSection.x_progress = mSection.length_x_acc_decc + mSection.length_x_vel;
        }
    } else if (t < mSection.time_x_acc_decc + mSection.time_x_vel + mSection.time_x_acc_decc) {
        tr = t - mSection.time_x_acc_decc - mSection.time_x_vel;
        mSection.x_progress = (mSection.length_x_acc_decc + mSection.length_x_vel) + mSection._x_vel_adjusted*tr - 0.5 * mSection._x_acc_decc * tr * tr;
        mSection.current_x_vel = mSection._x_vel_adjusted - mSection._x_acc_decc * tr;
        if (mSection.x_progress > mSection.length_of_section) {
            mSection.current_x_vel = 0.0;
            mSection.x_progress = mSection.length_of_section;
        }
    } else {
        mSection.current_x_vel = 0.0;
        mSection.x_progress = mSection.length_of_section;
    }

    if (t < mSection.time_yaw_acc_decc) {
        tr = t;
        mSection.yaw_progress = 0.5 * mSection._yaw_acc_decc * tr * tr;
        mSection.current_yaw_vel = mSection._yaw_acc_decc * tr;
        if (mSection.yaw_progress > mSection.length_yaw_acc_decc) {
            mSection.current_yaw_vel = mSection._yaw_vel_adjusted;
            mSection.yaw_progress = mSection.length_yaw_acc_decc;
        }
    } else if (t < mSection.time_yaw_acc_decc + mSection.time_yaw_vel) {
        tr = t - mSection.time_yaw_acc_decc;
        mSection.yaw_progress = mSection.length_yaw_acc_decc + mSection.current_yaw_vel * tr;
        mSection.current_yaw_vel = mSection._yaw_vel_adjusted;
        if (mSection.yaw_progress > (mSection.length_yaw_acc_decc + mSection.length_yaw_vel)) {
            mSection.yaw_progress = mSection.length_yaw_acc_decc + mSection.length_yaw_vel;
        }
    } else if (t < mSection.time_yaw_acc_decc + mSection.time_yaw_vel + mSection.time_yaw_acc_decc) {
        tr = t - mSection.time_yaw_acc_decc - mSection.time_yaw_vel;
        mSection.yaw_progress = (mSection.length_yaw_acc_decc + mSection.length_yaw_vel) + mSection._yaw_vel_adjusted * tr - 0.5 * mSection._yaw_acc_decc * tr * tr;
        mSection.current_yaw_vel = mSection._yaw_vel_adjusted - mSection._yaw_acc_decc * tr;
        if (mSection.yaw_progress > mSection.length_of_section_ang) {
            mSection.current_yaw_vel = 0.0;
            mSection.yaw_progress = mSection.length_of_section_ang;
        }
    } else {
        mSection.current_yaw_vel = 0.0;
        mSection.yaw_progress = mSection.length_of_section_ang;
    }

    double x_progress_ratio = 0.0;
    if (mSection.length_of_section > 0) {
        x_progress_ratio = mSection.x_progress / mSection.length_of_section;
    } else {
        x_progress_ratio = 1.0;
    }

    double yaw_progress_ratio = 0.0;
    if (mSection.length_of_section_ang > 0) {
        yaw_progress_ratio = mSection.yaw_progress / mSection.length_of_section_ang;
    } else {
        yaw_progress_ratio = 1.0;
    }
        
    // target_x = start_x + delta_x * progress_on_section
    double next_xyz[3];
    next_xyz[0] = mSection._start_xyz[0] + mSection._delta[0] * x_progress_ratio;
    next_xyz[1] = mSection._start_xyz[1] + mSection._delta[1] * x_progress_ratio;
    next_xyz[2] = mSection._start_xyz[2] + mSection._delta[2] * x_progress_ratio;
    double next_yaw = mSection._start_yaw + mSection._delta_yaw * yaw_progress_ratio;

    TrajectoryPoint tp;
    // next_pose.header.stamp is to be filled by the caller
    tp.pose.header.frame_id = mSection.section_start_pose_stamped.header.frame_id;
    tp.pose.pose.position.x = next_xyz[0];
    tp.pose.pose.position.y = next_xyz[1];
    tp.pose.pose.position.z = next_xyz[2];
    tp.velocity.linear.x = mSection.current_x_vel;
    // Compute orientation. PID can use it for holonomic robots
    tp.pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_yaw);

    if (mSection._delta_yaw > 0) {
        tp.velocity.angular.z = mSection.current_yaw_vel;
    } else if (mSection._delta_yaw < 0) {
        tp.velocity.angular.z = -mSection.current_yaw_vel;
    } else {
        tp.velocity.angular.z = 0;
    }

    return tp;
}

}