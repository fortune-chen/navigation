#ifndef TF2_BRIDGE_SERVER_H
#define TF2_BRIDGE_SERVER_H


#include <map>
#include <mutex>
#include <memory>
#include <functional>

#include "tf2_bridge.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "dds_publisher.h"
#include "dds_subscriber.h"
#include "TF2Communication.h"
#include "TF2CommunicationPubSubTypes.h"

namespace tf2_ros {

using TF2RequestCallback = std::function<void (const TF2Request&)>;

class TF2BridgeServer : public TF2Bridge {
 public:
  
  TF2BridgeServer(Buffer& buffer);

  ~TF2BridgeServer();

  // MAPPING -> BufferCore::canTransform
  bool canTransform(const std::string& target_frame, const std::string& source_frame,
                    const ros::Time& time, std::string* error_msg = NULL) override;

  // MAPPING -> Buffer::canTransform
  bool canTransform(const std::string& target_frame, const std::string& source_frame,
                    const ros::Time& time, const ros::Duration timeout, std::string* errstr = NULL) override;

  // MAPPING -> BufferCore::canTransform
  bool canTransform(const std::string& target_frame, const ros::Time& target_time,
                    const std::string& source_frame, const ros::Time& source_time,
                    const std::string& fixed_frame, std::string* error_msg = NULL) override;

  // MAPPING -> Buffer::canTransform
  bool canTransform(const std::string& target_frame, const ros::Time& target_time,
                    const std::string& source_frame, const ros::Time& source_time,
                    const std::string& fixed_frame, const ros::Duration timeout, std::string* errstr = NULL) override;

  // MAPPING -> BufferInterface::canTransform
  template <class T>
  T& transform(const T& in, T& out, const std::string& target_frame, ros::Duration timeout=ros::Duration(0.0)) {
    mTF.transform(in, out, target_frame, timeout);
    return out;
  }

  // MAPPING -> BufferCore::lookupTransform
  geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                                  const ros::Time& time) override;

  // MAPPING -> Buffer::lookupTransform
  geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                                  const ros::Time& time, const ros::Duration timeout) override;

  // MAPPING -> BufferCore::lookupTransform
  geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const ros::Time& target_time,
		                                          const std::string& source_frame, const ros::Time& source_time,
		                                          const std::string& fixed_frame) override;

  // MAPPING -> Buffer::lookupTransform
  geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const ros::Time& target_time,
                                                  const std::string& source_frame, const ros::Time& source_time,
                                                  const std::string& fixed_frame, const ros::Duration timeout) override;

  // MAPPING -> TransformBroadcaster::sendTransform
  void sendTransform(const geometry_msgs::TransformStamped & transform);

  // MAPPING -> TransformBroadcaster::sendTransform
  void sendTransform(const std::vector<geometry_msgs::TransformStamped> & transforms);

  // MAPPING -> StaticTransformBroadcaster::sendTransform
  void sendStaticTransform(const geometry_msgs::TransformStamped & transform);

  // MAPPING -> StaticTransformBroadcaster::sendTransform
  void sendStaticTransform(const std::vector<geometry_msgs::TransformStamped> & transforms);

  void parseAndExecuteRequest(const TF2Request& request, TF2Response& response);

 private:

  class TF2RequestListener : public TopicListener {
   public:
    TF2RequestListener(TF2BridgeServer* c) : TopicListener((void*)&mRequest) {
        mTF2BridgeServer = c;
    }

    void onTopicReceived(void* msg) override {
        if (!msg) {
            std::cout << "[error] nullptr\n";
            return;
        }
        TF2Request* request = (TF2Request*)msg;
        TF2Response response;
        response.response_id(request->request_id());
        mTF2BridgeServer->parseAndExecuteRequest(*request, response);
        mTF2BridgeServer->mPublisher->publish<TF2Response>(response);
    };

    TF2BridgeServer* mTF2BridgeServer;
    TF2Request mRequest;
  };

 private:
  Buffer& mTF;
  TransformBroadcaster mTFBroadcaster;
  StaticTransformBroadcaster mTFStaticBroadcaster;
  std::unique_ptr<DDSPublisher> mPublisher;
  std::unique_ptr<DDSSubscriber> mSubscriber;
  std::shared_ptr<TF2BridgeServer::TF2RequestListener> mRequestListener;
};

}



#endif
