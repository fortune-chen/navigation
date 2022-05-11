#include "transform_helper.h"
#include "ros/serialization.h"
#include "ros/ros_type_print.h"

#define ENABLE_TRACK_TRANSFORM          0

namespace tf2_ros {

std::mutex TransformHelper::mInstanceMutex;
TransformHelper* TransformHelper::mInstance = nullptr;

template<typename T>
static void serializeToString(const T& in, std::string& out) {
    auto m = ros::serialization::serializeMessage(in);
    if (m.num_bytes > 1024) {
        std::cout << "\n";
        printf("[error] TF2Message->sequence<char, 1024> not big enough, need %zu bytes\n", m.num_bytes);
        assert(false);
    }
    out.assign(reinterpret_cast<char*>(m.message_start), m.num_bytes);
}

void deserializeString(const std::vector<char>& in, tf2_msgs::TFMessage& out) {
    using namespace ros::serialization;
    IStream strm((uint8_t*)in.data(), in.size());
    deserialize(strm, out);
}

static void cast(const tf2_msgs::TFMessage& in, bool isStatic, TF2Message& out) {
    std::string str;
    serializeToString(in, str);
    out.serialized().resize(str.size());
    out.serialized().assign(str.begin(), str.end());
    out.isStatic(isStatic);
}

TransformHelper::TransformHelper() {
    std::shared_ptr<TF2MessagePubSubType> tf2Msg(new TF2MessagePubSubType);
    mPublisher = std::make_unique<DDSPublisher>(TF2_SET_TRANSFORM_TOPIC, tf2Msg);
}

TransformHelper::~TransformHelper() {
    
}

TransformHelper* TransformHelper::getInstance() {
    if (mInstance == nullptr) {
        std::lock_guard<std::mutex> lock(mInstanceMutex);
        if (mInstance == nullptr) {
            mInstance = new TransformHelper();
        }
    }
    return mInstance;  
}

void TransformHelper::publish(const tf2_msgs::TFMessage& message, bool isStatic) {
    TF2Message t;
#if ENABLE_TRACK_TRANSFORM
    static uint32_t index = 0;
    index++;
    // std::cout << "------------------------ sending TF2Message-index: " << index << std::endl;
    if (index % 100000 == 0) {
        std::cout << "------------------------ sending TF2Message-index: " << index << std::endl;
    }
    t.index(index);
#endif
    cast(message, isStatic, t);
    mPublisher->publish<TF2Message>(t);
}

TransformHelper::TF2MessageListener::TF2MessageListener(tf2::BufferCore& buffer)
    : TopicListener((void*)&mMessage),
      mBuffer(buffer) {
}

void TransformHelper::TF2MessageListener::onTopicReceived(void* msg) {
    TF2Message* message = (TF2Message*)msg;
    tf2_msgs::TFMessage tfMsg;
    deserializeString(message->serialized(), tfMsg);
#if ENABLE_TRACK_TRANSFORM
    static uint32_t index = 0;
    index++;
    // std::cout << "------------------------ recived TF2Message-index: " << index << std::endl;
    if (index != message->index()) {
        std::cout << "------------------------ missing TF2Message-index from " << index << " to " << message->index() << std::endl;
        index = message->index();
    }
    if (index % 100000 == 0) {
        std::cout << "------------------------ recived TF2Message-index: " << index << std::endl;
    }
#endif
    if (message->isStatic()) {
        for (const auto& transform : tfMsg.transforms) {
            Buffer::getInstance().setTransform(transform, "transform_broadcaster", true);
        }
    } else {
        for (auto& transform : tfMsg.transforms) {
            // TODO Bill Temp
            // PRINT_TRANSFORM_STAMPED("transform", transform);
            // transform.header.stamp = ros::Time::now(); // TODO Bill
            // printf("received transform.header.stamp = %.4f\n", transform.header.stamp.toSec());
            Buffer::getInstance().setTransform(transform, "transform_broadcaster");
        }    
    }
};
    
}