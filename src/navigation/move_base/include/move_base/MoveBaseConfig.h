#ifndef __MOVE_BASE__MOVEBASECONFIG_H__
#define __MOVE_BASE__MOVEBASECONFIG_H__

namespace move_base
{

struct MoveBaseConfig {
    std::string base_global_planner;
    std::string base_local_planner;
    double planner_frequency = 0.0;
    double controller_frequency = 0.0;
    double planner_patience = 0.0;
    double controller_patience = 0.0;
    int max_planning_retries = 0;
    double conservative_reset_dist = 0.0;
    bool recovery_behavior_enabled = false;
    bool clearing_rotation_allowed = false;
    bool shutdown_costmaps = false;
    double oscillation_timeout = 0.0;
    double oscillation_distance = 0.0;
    bool make_plan_clear_costmap = false;
    bool make_plan_add_unreachable_goal = false;
    bool restore_defaults = false;

    bool state = false;
    std::string name;
};


}


#endif