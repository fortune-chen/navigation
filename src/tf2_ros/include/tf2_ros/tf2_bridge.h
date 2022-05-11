#ifndef TF2_BRIDGE_H
#define TF2_BRIDGE_H

#include <string>
#include <assert.h>
#include "ros/time.h"
#include "ros/duration.h"
#include "ros/serialization.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "TF2Communication.h"
#include "TF2CommunicationPubSubTypes.h"

namespace tf2_ros {


template<typename T>
static void deserializeString(const char* in, int in_size, T& out) {
    using namespace ros::serialization;
    IStream strm((uint8_t*)in, in_size);
    deserialize(strm, out);
}
template<typename T>
static void deserializeString(const std::vector<char>& in, T& out) {
    deserializeString(in.data(), in.size(), out);
}
template<>
void deserializeString(const std::vector<char>& in, geometry_msgs::PoseStamped& out) {
    deserializeString(in.data(), in.size(), out);
}
template<>
void deserializeString(const std::vector<char>& in, geometry_msgs::PointStamped& out) {
    deserializeString(in.data(), in.size(), out);
}
template<>
void deserializeString(const std::vector<char>& in, geometry_msgs::TransformStamped& out) {
    deserializeString(in.data(), in.size(), out);
}
template<>
void deserializeString(const std::vector<char>& in, sensor_msgs::PointCloud2& out) {
    deserializeString(in.data(), in.size(), out);
}

// template<typename T>
// static void deserialize(const TF2Transform& result, T& out) {
//     switch (result.type()) {
//         case TF2TransformType::kPoseStamped:
//             deserializeString<geometry_msgs::PoseStamped>(result.serialized(), out);
//             break;
//         case TF2TransformType::kPointStamped:
//             deserializeString<geometry_msgs::PointStamped>(result.serialized(), out);
//             break;
//         case TF2TransformType::kTransformStamped:
//             // deserializeString<geometry_msgs::TransformStamped>(result.serialized(), out);
//             break;
//         case TF2TransformType::kPointCloud2:
//             deserializeString<sensor_msgs::PointCloud2>(result.serialized(), out);
//             break;
//         default:
//             std::cout << "[error] unkown TF2TransformType\n";
//             assert(false);
//             break;
//     }
// }

template<typename T>
static void serializeToString(const T& in, std::string& out) {
    auto m = ros::serialization::serializeMessage(in);
    if (m.num_bytes > 4096) {
        std::cout << "[error] sequence<char, 4096>, 4096 bytes not big enough\n";
        assert(false);
    }
    out.assign(reinterpret_cast<char*>(m.message_start), m.num_bytes);
}

template <typename T>
static void packTransform(const T& x, TF2Request& out) {
    std::cout << "[error] packTransform() need template specialization\n";
    assert(false);
}

template<>
void packTransform(const geometry_msgs::PoseStamped& in, TF2Request& out) {
    std::string str;
    serializeToString(in, str);
    out.transform().type(TF2TransformType::kPoseStamped);
    out.transform().serialized().resize(str.size());
    out.transform().serialized().assign(str.begin(), str.end());
}

template<>
void packTransform(const geometry_msgs::PointStamped& in, TF2Request& out) {
    std::string str;
    serializeToString(in, str);
    out.transform().type(TF2TransformType::kPointStamped);
    out.transform().serialized().resize(str.size());
    out.transform().serialized().assign(str.begin(), str.end());
}

template<>
void packTransform(const geometry_msgs::TransformStamped& in, TF2Request& out) {
    std::string str;
    serializeToString(in, str);
    out.transform().type(TF2TransformType::kTransformStamped);
    out.transform().serialized().resize(str.size());
    out.transform().serialized().assign(str.begin(), str.end());
}

template<>
void packTransform(const sensor_msgs::PointCloud2& in, TF2Request& out) {
    std::string str;
    serializeToString(in, str);
    out.transform().type(TF2TransformType::kPointCloud2);
    out.transform().serialized().resize(str.size());
    out.transform().serialized().assign(str.begin(), str.end());
}

template <typename T>
static void packTransform(const T& x, TF2Response& out) {
    std::cout << "[error] packTransform() need template specialization\n";
    assert(false);
}

template<>
void packTransform(const geometry_msgs::PoseStamped& in, TF2Response& out) {
    std::string str;
    serializeToString(in, str);
    out.transform().type(TF2TransformType::kPoseStamped);
    out.transform().serialized().resize(str.size());
    out.transform().serialized().assign(str.begin(), str.end());
}

template<>
void packTransform(const geometry_msgs::PointStamped& in, TF2Response& out) {
    std::string str;
    serializeToString(in, str);
    out.transform().type(TF2TransformType::kPointStamped);
    out.transform().serialized().resize(str.size());
    out.transform().serialized().assign(str.begin(), str.end());
}

template<>
void packTransform(const geometry_msgs::TransformStamped& in, TF2Response& out) {
    std::string str;
    serializeToString(in, str);
    out.transform().type(TF2TransformType::kTransformStamped);
    out.transform().serialized().resize(str.size());
    out.transform().serialized().assign(str.begin(), str.end());
}

template<>
void packTransform(const sensor_msgs::PointCloud2& in, TF2Response& out) {
    std::string str;
    serializeToString(in, str);
    out.transform().type(TF2TransformType::kPointCloud2);
    out.transform().serialized().resize(str.size());
    out.transform().serialized().assign(str.begin(), str.end());
}

class TF2Bridge {
 public:

  // MAPPING -> BufferCore::canTransform
  virtual bool canTransform(const std::string& target_frame, const std::string& source_frame,
                            const ros::Time& time, std::string* error_msg = NULL) = 0;

  // MAPPING -> Buffer::canTransform
  virtual bool canTransform(const std::string& target_frame, const std::string& source_frame,
                            const ros::Time& time, const ros::Duration timeout, std::string* errstr = NULL) = 0;

  // MAPPING -> BufferCore::canTransform
  virtual bool canTransform(const std::string& target_frame, const ros::Time& target_time,
                            const std::string& source_frame, const ros::Time& source_time,
                            const std::string& fixed_frame, std::string* error_msg = NULL) = 0;

  // MAPPING -> Buffer::canTransform
  virtual bool canTransform(const std::string& target_frame, const ros::Time& target_time,
                            const std::string& source_frame, const ros::Time& source_time,
                            const std::string& fixed_frame, const ros::Duration timeout, std::string* errstr = NULL) = 0;

  // MAPPING -> BufferCore::lookupTransform
  virtual geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                                          const ros::Time& time) = 0;

  // MAPPING -> Buffer::lookupTransform
  virtual geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                                          const ros::Time& time, const ros::Duration timeout) = 0;

  // MAPPING -> BufferCore::lookupTransform
  virtual geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const ros::Time& target_time,
		                                                  const std::string& source_frame, const ros::Time& source_time,
		                                                  const std::string& fixed_frame) = 0;

  // MAPPING -> Buffer::lookupTransform
  virtual geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const ros::Time& target_time,
                                                          const std::string& source_frame, const ros::Time& source_time,
                                                          const std::string& fixed_frame, const ros::Duration timeout) = 0;

  // MAPPING -> TransformBroadcaster::sendTransform
  virtual void sendTransform(const geometry_msgs::TransformStamped & transform) = 0;

  // MAPPING -> TransformBroadcaster::sendTransform
  virtual void sendTransform(const std::vector<geometry_msgs::TransformStamped> & transforms) = 0;

  // MAPPING -> StaticTransformBroadcaster::sendTransform
  virtual void sendStaticTransform(const geometry_msgs::TransformStamped & transform) = 0;

  // MAPPING -> StaticTransformBroadcaster::sendTransform
  virtual void sendStaticTransform(const std::vector<geometry_msgs::TransformStamped> & transforms) = 0;
};


}

#endif
