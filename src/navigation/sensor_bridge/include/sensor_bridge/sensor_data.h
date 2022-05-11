#ifndef __SENSOR_DATA__
#define __SENSOR_DATA__

#include <string>
#include <sstream>
#include <cassert>
#include <map>
#include "ros/serialization.h"

namespace sensor_data {

enum class SensorType {
    ODOM = 0,
    SCAN,
    IMU,
    MAP,
    PATH,
    GOAL,
    VELOCITY,
    FOOTPRINT
};

template<typename Os>
Os& operator<<(Os& os, const SensorType& t) {
    const static std::map<SensorType, std::string> map = {
        {SensorType::ODOM, "ODOM"},
        {SensorType::SCAN, "SCAN"},
        {SensorType::IMU, "IMU"},
        {SensorType::MAP, "MAP"},
        {SensorType::PATH, "PATH"},
        {SensorType::GOAL, "GOAL"},
        {SensorType::VELOCITY, "VELOCITY"},
        {SensorType::FOOTPRINT, "FOOTPRINT"},
    };
    return os << map.find(t)->second;
}

template<typename M>
struct MessageTraits {
    // default, no type member
};

// Note: provide specialization for your message type BEFORE using SensorData
// e.g.
// template<>
// struct MessageTraits<Odometry> {
//     const static SensorType type = SensorType::ODOM;
// };

class BigEndianInt32 {
public:
    explicit BigEndianInt32(int32_t i) {
        data[0] = i >> 24;
        data[1] = i >> 16;
        data[2] = i >> 8;
        data[3] = i & ((1 << 8) - 1);
    }
    operator int32_t() const {
        int32_t r = data[0] << 24;
        r |= data[1] << 16;
        r |= data[2] << 8;
        r |= data[3];
        return r;
    }

private:
    uint8_t data[4];
};

struct SensorData {
    enum class Direction {
        PUSH,
        PULL,
    } direction = Direction::PULL;
    SensorType type;
    int32_t content_length = 0;
    std::string content;

    enum class PacketStatus {
        DONE,
        RECEIVING,
    } status = PacketStatus::DONE;

    SensorData() = default;

    bool isComplete() const {
        return status == PacketStatus::DONE;
    }

    void clear() {
        *this = SensorData();
    }

    template<typename M>
    SensorData(const M& msg) {
        using namespace ros::serialization;
        direction = Direction::PUSH;
        type = MessageTraits<M>::type;
        auto m = serializeMessage(msg);
        content.assign(reinterpret_cast<char*>(m.message_start), m.num_bytes);
        content_length = content.length();
    }

    SensorData(const std::string& data): SensorData(data.c_str(), data.size()) {}
    SensorData(const char* data, int length) {
        receiveData(data, length);
    }

    void receiveData(const char* data, int length) {
        assert(length >= 1);
        switch (status) {
        case PacketStatus::DONE: {
            auto index = 0;
            direction = static_cast<Direction>(data[index++]);
            if (direction == Direction::PUSH) {
                type = static_cast<SensorType>(data[index++]);

                content_length = *reinterpret_cast<const BigEndianInt32*>(&data[index]);
                index += sizeof(content_length);
                // assert(index + content_length == length);
                auto remaining = length - index;
                if (remaining < content_length) {
                    status = PacketStatus::RECEIVING;
                }

                content.assign(&data[index], remaining);
            }
        }
        break;
        case PacketStatus::RECEIVING: {
            content += std::string(data, length);
            assert(content.length() <= content_length);
            if (content.length() == content_length) {
                status = PacketStatus::DONE;
            }
        }
        break;
        }
    }

    std::string serialize() const {
        std::ostringstream os;
        os.put(static_cast<char>(direction));
        if (direction == Direction::PUSH) {
            os.put(static_cast<char>(type));
            assert(content_length == content.length());
            BigEndianInt32 i(content_length);
            os.write(reinterpret_cast<char*>(&i), sizeof(i));
            os.write(content.data(), content.size());
        }

        return os.str();
    }
};

template<typename Os>
Os& operator<<(Os& os, const SensorData& data) {
    if (data.direction == SensorData::Direction::PULL) {
        return os << "request";
    } else {
        return os << "data, type:" << data.type
            << ", length:" << data.content.length() 
            << (data.isComplete() ? "" : " to be continued...");
    }
}

// template<typename M>
// void deserializeStringMessage(const char* s, int len, M& msg) {
//     auto p = new char[len];
//     for (int i = 0; i != len; ++i) {
//         p[i] = s[i];
//     }
//     boost::shared_array<uint8_t> a((uint8_t*)p);
//     ros::SerializedMessage m(a, len);

//     ros::serialization::deserializeMessage(m, msg);
// }

template<typename M>
void deserializeStringMessage(const char* s, int len, M& msg) {
    using namespace ros::serialization;
    IStream strm((uint8_t*)s, len);
    deserialize(strm, msg);
}

template<typename M>
M deserializeStringMessage(const char* s, int len) {
    M m;
    deserializeStringMessage(s, len, m);
    return m;
}

template<typename M>
void deserializeStringMessage(const std::string& s, M& msg) {
    deserializeStringMessage(s.data(), s.length(), msg);
}

template<typename M>
M deserializeStringMessage(const std::string& s) {
    return deserializeStringMessage<M>(s.data(), s.length());
}

}   // namespace sensor_data

#endif // __SENSOR_DATA__