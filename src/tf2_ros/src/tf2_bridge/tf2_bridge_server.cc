#include "tf2_bridge_cast.h"
#include "tf2_bridge_constants.h"
#include "tf2_ros/tf2_bridge_server.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace tf2_ros;

// typedef std::uint64_t hash_t;
 
// constexpr hash_t prime = 0x100000001B3ull;
// constexpr hash_t basis = 0xCBF29CE484222325ull;

// hash_t hash_(char const* str)
// {
// 	hash_t ret{basis};
// 	while(*str){
// 		ret ^= *str;
// 		ret *= prime;
// 		str++;
// 	}
// 	return ret;
// }

TF2BridgeServer::TF2BridgeServer(Buffer& buffer) : mTF(buffer) {
    std::shared_ptr<TF2ResponsePubSubType> resp(new TF2ResponsePubSubType);
    mPublisher = std::make_unique<DDSPublisher>(kTF2ResponseTopic, resp);
    std::shared_ptr<TF2RequestPubSubType> req(new TF2RequestPubSubType);
    mRequestListener.reset(new TF2BridgeServer::TF2RequestListener(this));
    mSubscriber = std::make_unique<DDSSubscriber>(kTF2RequestTopic, req, mRequestListener);
}

TF2BridgeServer::~TF2BridgeServer() {

}

bool TF2BridgeServer::canTransform(const std::string& target_frame, const std::string& source_frame, 
                    const ros::Time& time, const ros::Duration timeout, std::string* errstr) {
    return mTF.canTransform(target_frame, source_frame, time, timeout, errstr);
}

bool TF2BridgeServer::canTransform(const std::string& target_frame, const std::string& source_frame,
                const ros::Time& time, std::string* error_msg) {
    return mTF.canTransform(target_frame, source_frame, time, error_msg);
}

bool TF2BridgeServer::canTransform(const std::string& target_frame, const ros::Time& target_time,
                                   const std::string& source_frame, const ros::Time& source_time,
                                   const std::string& fixed_frame, const ros::Duration timeout, std::string* errstr) {
    printf("[function: %s][line: %d] not support\n", __FUNCTION__, __LINE__);
    assert(false);
}

bool TF2BridgeServer::canTransform(const std::string& target_frame, const ros::Time& target_time,
                                   const std::string& source_frame, const ros::Time& source_time,
                                   const std::string& fixed_frame, std::string* error_msg) {
    printf("[function: %s][line: %d] not support\n", __FUNCTION__, __LINE__);
    assert(false);
}

geometry_msgs::TransformStamped
TF2BridgeServer::lookupTransform(const std::string& target_frame,
                                 const std::string& source_frame,
                                 const ros::Time& time) {
    return mTF.lookupTransform(target_frame, source_frame, time);
}

geometry_msgs::TransformStamped
TF2BridgeServer::lookupTransform(const std::string& target_frame, const ros::Time& target_time,
		                         const std::string& source_frame, const ros::Time& source_time,
		                         const std::string& fixed_frame) {
    printf("[function: %s][line: %d] not support\n", __FUNCTION__, __LINE__);
    assert(false);
}

geometry_msgs::TransformStamped
TF2BridgeServer::lookupTransform(const std::string& target_frame, const std::string& source_frame,
                                 const ros::Time& time, const ros::Duration timeout) {
    printf("[function: %s][line: %d] not support\n", __FUNCTION__, __LINE__);
    assert(false);
}

geometry_msgs::TransformStamped
TF2BridgeServer::lookupTransform(const std::string& target_frame, const ros::Time& target_time,
                                 const std::string& source_frame, const ros::Time& source_time,
                                 const std::string& fixed_frame, const ros::Duration timeout) {
    printf("[function: %s][line: %d] not support\n", __FUNCTION__, __LINE__);
    assert(false);
}

// path : sendTransform-1
void TF2BridgeServer::sendTransform(const geometry_msgs::TransformStamped& transform) {
    mTFBroadcaster.sendTransform(transform);
}

void TF2BridgeServer::sendTransform(const std::vector<geometry_msgs::TransformStamped>& transforms) {
    printf("[function: %s][line: %d] not support\n", __FUNCTION__, __LINE__);
    assert(false);
}

// path : sendStaticTransform-1
void TF2BridgeServer::sendStaticTransform(const geometry_msgs::TransformStamped& transform) {
    mTFStaticBroadcaster.sendTransform(transform);
}

void TF2BridgeServer::sendStaticTransform(const std::vector<geometry_msgs::TransformStamped>& transforms) {
    printf("[function: %s][line: %d] not support\n", __FUNCTION__, __LINE__);
    assert(false);
}

void TF2BridgeServer::parseAndExecuteRequest(const TF2Request& request, TF2Response& response) {
    
    if (request.path() == CALL_PATH_CAN_TRANSFORM_FOUR_PARAM) {
        std::string errMsg;
        auto time = cast(request.time());
        auto ret = canTransform(request.target_frame(), request.source_frame(), time, &errMsg);
        response.error_msg(errMsg);
        response.result(ret);
    } else if (request.path() == CALL_PATH_CAN_TRANSFORM_FIVE_PARAM) {
        std::string errMsg;
        auto time = cast(request.time());
        auto duration = cast(request.duration());
        auto ret = canTransform(request.target_frame(), request.source_frame(), time, duration, &errMsg);
        response.error_msg(errMsg);
        response.result(ret);
    } else if (request.path() == CALL_PATH_CAN_TRANSFORM_SIX_PARAM) {
        std::string errMsg;
        auto target_time = cast(request.target_time());
        auto source_time = cast(request.source_time());
        auto ret = canTransform(request.target_frame(), target_time,
                                request.source_frame(), source_time,
                                request.fixed_frame(), &errMsg);
        response.error_msg(errMsg);
        response.result(ret);
    } else if (request.path() == CALL_PATH_CAN_TRANSFORM_SEVEN_PARAM) {
        std::string errMsg;
        auto target_time = cast(request.target_time());
        auto source_time = cast(request.source_time());
        auto duration = cast(request.duration());
        auto ret = canTransform(request.target_frame(), target_time,
                                request.source_frame(), source_time,
                                request.fixed_frame(), duration,
                                &errMsg);
        response.error_msg(errMsg);
        response.result(ret);
    } else if (request.path() == CALL_PATH_TRANSFORM_TEMPLATE) {
        geometry_msgs::PoseStamped pose;
        geometry_msgs::PoseStamped out;
        deserializeString(request.transform().serialized(), pose);
        transform(pose, out, request.target_frame());
        packTransform(out, response);
    } else if (request.path() == CALL_PATH_LOOKUP_TRANSFORM_THREE_PARAM) {
        auto time = cast(request.time());
        auto transform = lookupTransform(request.target_frame(), request.source_frame(), time);
        packTransform(transform, response);
    } else if (request.path() == CALL_PATH_LOOKUP_TRANSFORM_FOUR_PARAM) {
        auto time = cast(request.time());
        auto duration = cast(request.duration());
        auto transform = lookupTransform(request.target_frame(), request.source_frame(), time, duration);
        packTransform(transform, response);
    } else if (request.path() == CALL_PATH_LOOKUP_TRANSFORM_FIVE_PARAM) {
        auto target_time = cast(request.target_time());
        auto source_time = cast(request.source_time());
        auto transform = lookupTransform(request.target_frame(), target_time,
                                         request.source_frame(), source_time,
                                         request.fixed_frame());
        packTransform(transform, response);
    } else if (request.path() == CALL_PATH_LOOKUP_TRANSFORM_SIX_PARAM) {
        auto target_time = cast(request.target_time());
        auto source_time = cast(request.source_time());
        auto duration = cast(request.duration());
        auto transform = lookupTransform(request.target_frame(), target_time,
                                         request.source_frame(), source_time,
                                         request.fixed_frame(), duration);
        packTransform(transform, response);
    } else if (request.path() == CALL_PATH_SEND_TRANSFORM) {
        geometry_msgs::TransformStamped transform;
        deserializeString(request.transform().serialized(), transform);
        sendTransform(transform);
    } else if (request.path() == CALL_PATH_SEND_TRANSFORM_STATIC) {
        geometry_msgs::TransformStamped transform;
        deserializeString(request.transform().serialized(), transform);
        sendStaticTransform(transform);
    } else {
        std::cout << "[error] Unsupport this path\n";
        assert(false);
    }
}