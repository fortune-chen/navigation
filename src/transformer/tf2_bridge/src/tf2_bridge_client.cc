#include <future>
#include <assert.h>
#include "uuid_generation.h"
#include "tf2_bridge_cast.h"
#include "tf2_bridge_constants.h"
#include "tf2_bridge/tf2_bridge_client.h"

using namespace tf2_ros;

static uint32_t kRequestTimeoutMilli = 2000;

TF2BridgeClient* TF2BridgeClient::mInstance = nullptr;
std::mutex TF2BridgeClient::mInstanceMutex;

TF2BridgeClient* TF2BridgeClient::getInstance() {
    std::lock_guard<std::mutex> lock(mInstanceMutex);
    if (mInstance == nullptr) {
        mInstance = new TF2BridgeClient();
    }
    return mInstance;
}

TF2BridgeClient::TF2BridgeClient() {
    std::shared_ptr<TF2RequestPubSubType> req(new TF2RequestPubSubType);
    mPublisher = std::make_unique<DDSPublisher>(kTF2RequestTopic, req);
    std::shared_ptr<TF2ResponsePubSubType> resp(new TF2ResponsePubSubType);
    mResponseListener.reset(new TF2BridgeClient::TF2ResponseListener());
    mSubscriber = std::make_unique<DDSSubscriber>(kTF2ResponseTopic, resp, mResponseListener);
}

TF2BridgeClient::~TF2BridgeClient() {
    std::lock_guard<std::mutex> lock(mInstanceMutex);
    if (mInstance) {
        delete mInstance;
        mInstance = nullptr;
    }
}

bool TF2BridgeClient::canTransform(const std::string& target_frame, const std::string& source_frame,
                                   const ros::Time& time, std::string* error_msg) {
    TF2Request req;
    req.target_frame(target_frame);
    req.source_frame(source_frame);
    req.time(cast(time));
    req.path(CALL_PATH_CAN_TRANSFORM_FOUR_PARAM);
    auto response = TF2Call(req);
    if (error_msg) {
        *error_msg = response.error_msg();
    }
    return response.result();
}

bool TF2BridgeClient::canTransform(const std::string& target_frame, const std::string& source_frame,
                                   const ros::Time& time, const ros::Duration timeout, std::string* errstr) {
    TF2Request req;
    req.target_frame(target_frame);
    req.source_frame(source_frame);
    req.time(cast(time));
    req.duration(cast(timeout));
    req.path(CALL_PATH_CAN_TRANSFORM_FIVE_PARAM);
    auto response = TF2Call(req);
    if (errstr) {
        *errstr = response.error_msg();
    }
    return response.result();
}

bool TF2BridgeClient::canTransform(const std::string& target_frame, const ros::Time& target_time,
                                   const std::string& source_frame, const ros::Time& source_time,
                                   const std::string& fixed_frame, std::string* error_msg) {
    TF2Request req;
    req.target_frame(target_frame);
    req.target_time(cast(target_time));
    req.source_frame(source_frame);
    req.source_time(cast(source_time));
    req.fixed_frame(fixed_frame);
    req.path(CALL_PATH_CAN_TRANSFORM_SIX_PARAM);
    auto response = TF2Call(req);
    if (error_msg) {
        *error_msg = response.error_msg();
    }
    return response.result();
}

bool TF2BridgeClient::canTransform(const std::string& target_frame, const ros::Time& target_time,
                                   const std::string& source_frame, const ros::Time& source_time,
                                   const std::string& fixed_frame, const ros::Duration timeout, std::string* errstr) {
    TF2Request req;
    req.target_frame(target_frame);
    req.target_time(cast(target_time));
    req.source_frame(source_frame);
    req.source_time(cast(source_time));
    req.fixed_frame(fixed_frame);
    req.duration(cast(timeout));
    req.path(CALL_PATH_CAN_TRANSFORM_SEVEN_PARAM);
    auto response = TF2Call(req);
    if (errstr) {
        *errstr = response.error_msg();
    }
    return response.result();
}

geometry_msgs::TransformStamped TF2BridgeClient::lookupTransform(const std::string& target_frame,
                                                                 const std::string& source_frame,
                                                                 const ros::Time& time) {
    TF2Request req;
    req.target_frame(target_frame);
    req.source_frame(source_frame);
    req.path(CALL_PATH_LOOKUP_TRANSFORM_THREE_PARAM);
    auto response = TF2Call(req);
    geometry_msgs::TransformStamped transform;
    deserializeString<geometry_msgs::TransformStamped>(response.transform().serialized(), transform);
    return transform;
}

geometry_msgs::TransformStamped
TF2BridgeClient::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                 const ros::Time& time, const ros::Duration timeout) {
    TF2Request req;
    req.target_frame(target_frame);
    req.source_frame(source_frame);
    req.time(cast(time));
    req.duration(cast(timeout));
    req.path(CALL_PATH_LOOKUP_TRANSFORM_FOUR_PARAM);
    auto response = TF2Call(req);
    geometry_msgs::TransformStamped transform;
    deserializeString<geometry_msgs::TransformStamped>(response.transform().serialized(), transform);
    return transform;
}

geometry_msgs::TransformStamped
TF2BridgeClient::lookupTransform(const std::string& target_frame, const ros::Time& target_time,
		                         const std::string& source_frame, const ros::Time& source_time,
		                         const std::string& fixed_frame) {
    TF2Request req;
    req.target_frame(target_frame);
    req.target_time(cast(target_time));
    req.source_frame(source_frame);
    req.source_time(cast(source_time));
    req.fixed_frame(fixed_frame);
    req.path(CALL_PATH_LOOKUP_TRANSFORM_FIVE_PARAM);
    auto response = TF2Call(req);
    geometry_msgs::TransformStamped transform;
    deserializeString<geometry_msgs::TransformStamped>(response.transform().serialized(), transform);
    return transform;
}

geometry_msgs::TransformStamped
TF2BridgeClient::lookupTransform(const std::string& target_frame, const ros::Time& target_time,
                                 const std::string& source_frame, const ros::Time& source_time,
                                 const std::string& fixed_frame, const ros::Duration timeout) {
    TF2Request req;
    req.target_frame(target_frame);
    req.target_time(cast(target_time));
    req.source_frame(source_frame);
    req.source_time(cast(source_time));
    req.fixed_frame(fixed_frame);
    req.duration(cast(timeout));
    req.path(CALL_PATH_LOOKUP_TRANSFORM_SIX_PARAM);
    auto response = TF2Call(req);
    geometry_msgs::TransformStamped transform;
    deserializeString<geometry_msgs::TransformStamped>(response.transform().serialized(), transform);
    return transform;
}

void TF2BridgeClient::sendTransform(const geometry_msgs::TransformStamped& transform) {
    TF2Request req;
    packTransform(transform, req);
    req.path(CALL_PATH_SEND_TRANSFORM);
    TF2CallWithoutResponse(req);
}

void TF2BridgeClient::sendTransform(const std::vector<geometry_msgs::TransformStamped>& transforms) {
    for (const auto& t : transforms) {
        sendTransform(t);
    }
}

void TF2BridgeClient::sendStaticTransform(const geometry_msgs::TransformStamped& transform) {
    TF2Request req;
    packTransform(transform, req);
    req.path(CALL_PATH_SEND_TRANSFORM_STATIC);
    TF2CallWithoutResponse(req);
}

void TF2BridgeClient::sendStaticTransform(const std::vector<geometry_msgs::TransformStamped>& transforms) {
    for (const auto& t : transforms) {
        sendStaticTransform(t);
    }
}

void TF2BridgeClient::TF2Call(const TF2Request& req, const TF2ResultCallback& cb) {
    {
        std::lock_guard<std::mutex> lock(mCallbackMapMutex);
        mCallbackMap.emplace(req.request_id(), cb);
    }
    // 注意：mPublisher->publish不能放到上面的锁内，否则会阻塞导致锁无法释放
    mPublisher->publish<TF2Request>(req);
}

TF2Response TF2BridgeClient::TF2Call(TF2Request& req) {
    TF2Response resp;
    req.request_id(utils::generateUUID());
    auto prom = std::make_shared<std::promise<TF2Response>>();
    auto fut = prom->get_future();
    TF2Call(req, [prom](const TF2Response& r) {
        prom->set_value(r);
    });

    std::future_status status;
    do {
        status = fut.wait_for(std::chrono::milliseconds(kRequestTimeoutMilli));
        if (status == std::future_status::deferred) {
            std::cout << "future_status: deferred\n";
        } else if (status == std::future_status::timeout) {
            std::cout << "future_status: timeout\n";
            assert(false);
            break;
        } else if (status == std::future_status::ready) {
            return fut.get();
        }
    } while (status != std::future_status::ready);
    return resp;
}

void TF2BridgeClient::TF2CallWithoutResponse(const TF2Request& req) {
    mPublisher->publish<TF2Request>(req);
}

std::string TF2BridgeClient::getTransformCallPath() {
    return CALL_PATH_TRANSFORM_TEMPLATE;
}