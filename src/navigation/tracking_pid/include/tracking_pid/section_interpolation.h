#ifndef _TRACKING_PID_SECTION_INTERPOLATION_H
#define _TRACKING_PID_SECTION_INTERPOLATION_H

#include "ros/time.h"
#include "ros/duration.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

namespace tracking_pid
{

struct SectionParam {    
    // line
    double _x_vel = 0.0;
    double _x_acc_decc = 0.0;
    double time_x_acc_decc = 0.0;
    double length_x_acc_decc = 0.0;
    double length_x_vel = 0.0;
    double time_x_vel = 0.0;
    double _x_vel_adjusted = 0.0;
    double duration_for_section_x = 0.0;
    double x_progress = 0.0;
    double current_x_vel = 0.0;
    double length_of_section = 0.0;
    double _start_xyz[3];
    double _delta[3];

    // angle
    double _yaw_vel = 0.0;
    double _yaw_acc_decc = 0.0;
    double time_yaw_acc_decc = 0.0;
    double length_yaw_acc_decc = 0.0;
    double length_yaw_vel = 0.0;
    double time_yaw_vel = 0.0;
    double _yaw_vel_adjusted = 0.0;
    double duration_for_section_yaw = 0.0;
    double yaw_progress = 0.0;
    double current_yaw_vel = 0.0;
    double length_of_section_ang = 0.0;
    double _start_yaw = 0.0;
    double _delta_yaw = 0.0;

    ros::Duration duration_for_section;
    ros::Time section_start_time;
    ros::Time section_end_time;

    ros::Duration duration_on_section;
    geometry_msgs::PoseStamped section_start_pose_stamped;
    geometry_msgs::PoseStamped section_end_pose_stamped;
};

struct TrajectoryPoint {
    double time;
    int8_t controller = 0;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Twist velocity;
    geometry_msgs::Twist acceleration;
};

class SectionInterpolation {
  public:
    SectionInterpolation(const geometry_msgs::PoseStamped& from,
                         const geometry_msgs::PoseStamped& to,
                         const ros::Time &stamp,
                         double x_vel,
                         double x_acc,
                         double yaw_vel,
                         double yaw_acc);

    ~SectionInterpolation();

    TrajectoryPoint interpolateWithAcceleration(const ros::Time &stamp);

    SectionParam mSection;
};

}

#endif