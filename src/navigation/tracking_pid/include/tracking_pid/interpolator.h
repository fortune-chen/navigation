#ifndef _TRACKING_PID_INTERPOLATOR_H
#define _TRACKING_PID_INTERPOLATOR_H

#include <mutex>
#include <vector>
#include <string>
#include <memory>
#include <utility>

#include "nav_msgs/Path.h"
#include "tracking_pid/section_interpolation.h"

namespace tracking_pid
{

enum InterpolationState {
    kInterpolating,
    kIdle
};

class TrajectoryPointListener {
  public:
    virtual ~TrajectoryPointListener() = default;

    virtual void onTrajectoryPointReceived(const TrajectoryPoint& tp) = 0;
};

class Interpolator {
  public:
    Interpolator();

    ~Interpolator();

    void startPath();

    void stopPath();

    void continuePath(const ros::Time &stamp);

    void acceptPathFromTopic(const nav_msgs::Path& path);

    void acceptGoal();

    void preemptGoal();

    void processPath(nav_msgs::Path& path);

    void updateTarget(const ros::Time &current_real);

    void updateThread(double frequency);

    bool setPlan(const nav_msgs::Path& path);

    void registerTrajectoryPointListener(const std::shared_ptr<TrajectoryPointListener>& listener);

    void unregisterTrajectoryPointListener(const std::shared_ptr<TrajectoryPointListener>& listener);

    // typedef std::pair<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> Section;

  private:
    std::vector<geometry_msgs::PoseStamped> _path_poses;
    std::string robot_frame;
    // TransformListener // TODO : Bill
    std::vector<geometry_msgs::PoseStamped> _sections;
    std::shared_ptr<SectionInterpolation> _current_section;
    double _rate;
    // _timer // TODO : Bill
    geometry_msgs::PoseStamped _latest_subgoal_pose;
    bool _paused;
    double _target_x_vel;
    double _target_x_acc;
    double _target_yaw_vel;
    double _target_yaw_acc;
    nav_msgs::Path _latest_path_msg;
    double progress_on_section;
    TrajectoryPoint tp;

    InterpolationState _state;
    std::mutex  _state_mutex;
    std::vector<std::shared_ptr<TrajectoryPointListener>> mTpListeners;
    std::mutex  _listener_mutex;

};

}

#endif
