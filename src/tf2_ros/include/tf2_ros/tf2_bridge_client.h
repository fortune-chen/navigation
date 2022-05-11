#ifndef TF2_BRIDGE_CLIENT_H
#define TF2_BRIDGE_CLIENT_H

#include <map>
#include <mutex>
#include <memory>
#include <functional>

#include "tf2_bridge.h"
#include "dds_publisher.h"
#include "dds_subscriber.h"
#include "TF2Communication.h"
#include "TF2CommunicationPubSubTypes.h"

namespace tf2_ros {

using TF2ResultCallback = std::function<void (const TF2Response&)>;

class TF2BridgeClient : public TF2Bridge {
 public:

  static TF2BridgeClient* getInstance();

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

  // MAPPING -> BufferInterface::transform
  template <class T>
  T& transform(const T& in, T& out, const std::string& target_frame, ros::Duration timeout=ros::Duration(0.0)) {
    TF2Request req;
    packTransform(in, req);
    req.target_frame(target_frame);
    req.path(getTransformCallPath());
    req.duration().duration_sec(timeout.sec);
    req.duration().duration_nsec(timeout.nsec);
    auto response = TF2Call(req);
    deserializeString(response.transform().serialized(), out);
    return out;
  }

  // MAPPING -> BufferCore::lookupTransform
  geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                                  const ros::Time& time) override;

  // MAPPING -> Buffer::lookupTransform
  geometry_msgs::TransformStamped lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                                  const ros::Time& time, const ros::Duration timeout) override;

  // MAPPING -> BufferCore::lookupTransform
  // path : lookupTransform-2
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

 private:
  
  void TF2Call(const TF2Request& req, const TF2ResultCallback& cb);

  TF2Response TF2Call(TF2Request& req);

  void TF2CallWithoutResponse(const TF2Request& req);

  std::string getTransformCallPath();

  class TF2ResponseListener : public TopicListener {
   public:
    TF2ResponseListener() : TopicListener((void*)&mResponse) {}

    void onTopicReceived(void* msg) override {
        TF2Response* response = (TF2Response*)msg;
        std::lock_guard<std::mutex> lock(TF2BridgeClient::getInstance()->mCallbackMapMutex);
        auto iter = TF2BridgeClient::getInstance()->mCallbackMap.find(response->response_id());
        if (iter != TF2BridgeClient::getInstance()->mCallbackMap.end()) {
            iter->second(*response);
            TF2BridgeClient::getInstance()->mCallbackMap.erase(response->response_id());
        }
    };

    TF2Response mResponse;

  };

 private:
  TF2BridgeClient();
  ~TF2BridgeClient();
  static TF2BridgeClient* mInstance;
  static std::mutex mInstanceMutex;

  std::map<std::string, TF2ResultCallback> mCallbackMap;
  std::mutex mCallbackMapMutex;
  std::unique_ptr<DDSPublisher> mPublisher;
  std::unique_ptr<DDSSubscriber> mSubscriber;
  std::shared_ptr<TF2BridgeClient::TF2ResponseListener> mResponseListener;
};

}


#endif