#include "tf2_bridge_cast.h"

namespace tf2_ros {

TF2RequestTime cast(const ros::Time& in) {
    TF2RequestTime out;
    out.time_sec(in.sec);
    out.time_nsec(in.nsec);
    return out;
}

TF2RequestDuration cast(const ros::Duration& in) {
    TF2RequestDuration out;
    out.duration_sec(in.sec);
    out.duration_nsec(in.nsec);
    return out;
}

ros::Time cast(const TF2RequestTime& in) {
    ros::Time out(in.time_sec(), in.time_nsec());
    return out;
}

ros::Duration cast(const TF2RequestDuration& in) {
    ros::Duration out(in.duration_sec(), in.duration_nsec());
    return out;
}

}
