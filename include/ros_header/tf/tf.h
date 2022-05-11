#ifndef TF_TF_H
#define TF_TF_H

#include <ros/duration.h>
#include <ros/time.h>
#include "LinearMath/Matrix3x3.h"
#include "LinearMath/Transform.h"
#include "transform_datatypes.h"

typedef double tfScalar;

namespace tf
{
typedef tf::Vector3 Point;

class Transformer
{
public:
    Transformer(bool interpolating = true,
              ros::Duration cache_time_ = ros::Duration(10)) {}

    virtual ~Transformer(void) {}

    void lookupTransform(const std::string& target_frame, const std::string& source_frame,
                       const ros::Time& time, StampedTransform& transform) {}

    void lookupTransform(const std::string& target_frame, const ros::Time& target_time,
                       const std::string& source_frame, const ros::Time& source_time,
                       const std::string& fixed_frame, StampedTransform& transform) {}




};

}

#endif 