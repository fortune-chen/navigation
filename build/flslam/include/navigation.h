#ifndef FLSLAM_SRC_APP_SERVER_NAVIGATION_H
#define FLSLAM_SRC_APP_SERVER_NAVIGATION_H

#include <mutex>
#include <memory>
#include <vector>

namespace flslam {

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double orientation = 0.0;
};

enum Direction {
    kUp = 1,
    kDown = 2,
    kLeft = 3,
    kRight = 4,
    kStop = 5
};

class PathListener {
  public:
    virtual ~PathListener() = default;

    virtual void onGlobalPathReceived(const std::vector<Pose>& path) = 0;
};

class Navigation {
  public:
    Navigation();

    ~Navigation();

    void setGoal(const Pose& pose);

    void doFullCoveragePathPlan(const Pose& initialPoint);

    void controlMotion(Direction direction);

    void registerPathListener(const std::shared_ptr<PathListener>& listener);

    void unregisterPathListener(const std::shared_ptr<PathListener>& listener);

    std::mutex mMutex;
    std::vector<std::shared_ptr<PathListener>> mPathListeners;

  private:
    std::mutex mVelocityMutex;
    double mLinearVelocity;
    double mAngularVelocity;
};

}

#endif  // FLSLAM_SRC_APP_SERVER_NAVIGATION_H
