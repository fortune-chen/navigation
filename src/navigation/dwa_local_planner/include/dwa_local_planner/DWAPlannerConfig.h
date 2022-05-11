#ifndef __dwa_local_planner__DWAPLANNERCONFIG_H__
#define __dwa_local_planner__DWAPLANNERCONFIG_H__

namespace dwa_local_planner
{

  struct DWAPlannerConfig
  {
    double max_vel_trans = 0.0;
    double min_vel_trans = 0.0;
    double max_vel_x = 0.0;
    double min_vel_x = 0.0;
    double max_vel_y = 0.0;
    double min_vel_y = 0.0;
    double max_vel_theta = 0.0;
    double min_vel_theta = 0.0;
    double acc_lim_x = 0.0;
    double acc_lim_y = 0.0;
    double acc_lim_theta = 0.0;
    double acc_lim_trans = 0.0;
    bool prune_plan = false;
    double xy_goal_tolerance = 0.0;
    double yaw_goal_tolerance = 0.0;
    double trans_stopped_vel = 0.0;
    double theta_stopped_vel = 0.0;
    double sim_time = 0.0;
    double sim_granularity = 0.0;
    double angular_sim_granularity = 0.0;
    double path_distance_bias = 0.0;
    double goal_distance_bias = 0.0;
    double occdist_scale = 0.0;
    double twirling_scale = 0.0;
    double stop_time_buffer = 0.0;
    double oscillation_reset_dist = 0.0;
    double oscillation_reset_angle = 0.0;
    double forward_point_distance = 0.0;
    double scaling_speed = 0.0;
    double max_scaling_factor = 0.0;
    int vx_samples = 0;
    int vy_samples = 0;
    int vth_samples = 0;
    bool use_dwa = false;
    bool restore_defaults = false;

    bool state = false;
    std::string name;
  };

}

#endif