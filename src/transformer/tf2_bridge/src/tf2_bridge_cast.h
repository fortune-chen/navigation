#ifndef TF2_BRIDGE_CAST_H
#define TF2_BRIDGE_CAST_H

#include "ros/time.h"
#include "ros/duration.h"
#include "TF2Communication.h"
#include "TF2CommunicationPubSubTypes.h"

namespace tf2_ros {

TF2RequestTime cast(const ros::Time& in);

TF2RequestDuration cast(const ros::Duration& in);

ros::Time cast(const TF2RequestTime& in);

ros::Duration cast(const TF2RequestDuration& in);

}

#endif