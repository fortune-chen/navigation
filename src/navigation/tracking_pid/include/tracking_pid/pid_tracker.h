#ifndef TRACKING_PID_PID_TRACKER_H
#define TRACKING_PID_PID_TRACKER_H

#include <mutex>
#include <condition_variable>
#include "controller.h"
#include "interpolator.h"

namespace tracking_pid {

class PidTracker {
 public:
  PidTracker();

  ~PidTracker();

  void updateLoop();

  void update();

  void startPath();

 private:
  Interpolator mPathInterpolator;
  std::mutex mMutex;
  std::condition_variable mCondVar;

  bool controllerEnabled;
};

}


#endif