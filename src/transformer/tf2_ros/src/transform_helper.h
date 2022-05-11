#ifndef TF2_ROS_TRANSFORM_HELPER_H
#define TF2_ROS_TRANSFORM_HELPER_H

#include <mutex>
#include <memory>
#include "dds_publisher.h"
#include "dds_subscriber.h"
#include "TF2Message.h"
#include "TF2MessagePubSubTypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_msgs/TFMessage.h"

#define TF2_SET_TRANSFORM_TOPIC             "TF2-SET-TRANSFORM"

namespace tf2_ros {

class TransformHelper {
 public:

  static TransformHelper* getInstance();

  void publish(const tf2_msgs::TFMessage& message, bool isStatic = false);

  class TF2MessageListener : public TopicListener {
   public:
    TF2MessageListener(tf2::BufferCore& buffer);
    void onTopicReceived(void* msg) override;

    TF2Message mMessage;
    tf2::BufferCore& mBuffer;

  };

 private:
  TransformHelper();
  ~TransformHelper();

 private:
  static std::mutex mInstanceMutex;
  static TransformHelper* mInstance;
  std::unique_ptr<DDSPublisher> mPublisher;
};

}

#endif