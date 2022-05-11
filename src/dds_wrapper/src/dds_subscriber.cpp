#include "dds_subscriber.h"
#include <fastrtps/participant/Participant.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/subscriber/SubscriberListener.h>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/Domain.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>

#include <sys/types.h>
#include <unistd.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>
#include <fastrtps/utils/IPLocator.h>
#include "ros_data.h"
#include "RosAdapterContentPubSubTypes.h"

using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

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

template<typename M>
void deserializeStringMessage(const std::vector<char>& s, M& msg) {
    deserializeStringMessage(s.data(), s.size(), msg);
}

template<typename M>
M deserializeStringMessage(const std::vector<char>& s) {
    return deserializeStringMessage<M>(s.data(), s.size());
}

class SubListener : public SubscriberListener {
  public:
    SubListener(const std::string& topic, std::shared_ptr<TopicListener> listener) {
        m_matched = 0;
        m_topic = topic;
        m_listener = listener;
    };
    ~SubListener() {
    };
    void onSubscriptionMatched(eprosima::fastrtps::Subscriber* sub, eprosima::fastrtps::rtps::MatchingInfo& info) {
        if (info.status == MATCHED_MATCHING) {
            m_matched++;
            std::cout << "Subscriber matched: " << m_matched << " topic: " << m_topic << std::endl;
        } else {
            m_matched--;
            std::cout << "Subscriber unmatched: " << m_matched << " topic: " << m_topic << std::endl;
        }
    };

    void onNewDataMessage(eprosima::fastrtps::Subscriber* sub) {
        if (m_listener && m_listener->m_buffer) {
            if ("RosAdapterContent" == m_type) {
                RosAdapterContent adapter;
                if (sub->takeNextData((void*)(&adapter), &m_info)) {
                        switch (ROSDataType(adapter.type())) {
                        case ROSDataType::SCAN: {
                            #if USED_MULTI_ECHO_SCAN
                            sensor_msgs::MultiEchoLaserScan laser;
                            deserializeStringMessage<sensor_msgs::MultiEchoLaserScan>(adapter.serialized(), laser);
                            #else
                            sensor_msgs::LaserScan laser;
                            deserializeStringMessage<sensor_msgs::LaserScan>(adapter.serialized(), laser);
                            #endif
                            m_listener->onTopicReceived((void*)&laser);
                        } break;
                        case ROSDataType::IMU: {
                            sensor_msgs::Imu imu;
                            deserializeStringMessage<sensor_msgs::Imu>(adapter.serialized(), imu);
                            m_listener->onTopicReceived((void*)&imu);
                        } break;
                        case ROSDataType::ODOM: {
                            nav_msgs::Odometry odom;
                            deserializeStringMessage<nav_msgs::Odometry>(adapter.serialized(), odom);
                            m_listener->onTopicReceived((void*)&odom);
                        } break;
                        case ROSDataType::PATH: {
                            nav_msgs::Path path;
                            deserializeStringMessage<nav_msgs::Path>(adapter.serialized(), path);
                            m_listener->onTopicReceived((void*)&path);
                        } break;
                        case ROSDataType::POSE: {
                            geometry_msgs::PoseStamped pose;
                            deserializeStringMessage<geometry_msgs::PoseStamped>(adapter.serialized(), pose);
                            m_listener->onTopicReceived((void*)&pose);
                        } break;
                        case ROSDataType::POLYGON: {
                            geometry_msgs::PolygonStamped footprint;
                            deserializeStringMessage<geometry_msgs::PolygonStamped>(adapter.serialized(), footprint);
                            m_listener->onTopicReceived((void*)&footprint);
                        } break;
                        case ROSDataType::VELOCITY: {
                            geometry_msgs::Twist velocity;
                            deserializeStringMessage<geometry_msgs::Twist>(adapter.serialized(), velocity);
                            m_listener->onTopicReceived((void*)&velocity);
                        } break;
                        case ROSDataType::GNSS: {
                            sensor_msgs::NavSatFix gnss;
                            deserializeStringMessage<sensor_msgs::NavSatFix>(adapter.serialized(), gnss);
                            m_listener->onTopicReceived((void*)&gnss);
                        } break;
                        default:
                        break;
                    }
                }
            } else if ("NavMapContent" == m_type) {
                NavMapContent navMap;
                if (sub->takeNextData((void*)(&navMap), &m_info)) {
                        switch (ROSDataType(navMap.type())) {
                        case ROSDataType::MAP: {
                            nav_msgs::OccupancyGrid map;
                            deserializeStringMessage<nav_msgs::OccupancyGrid>(navMap.serialized(), map);
                            m_listener->onTopicReceived((void*)&map);
                        } break;
                        case ROSDataType::IMAGE: {
                            sensor_msgs::Image image;
                            deserializeStringMessage<sensor_msgs::Image>(navMap.serialized(), image);
                            m_listener->onTopicReceived((void*)&image);
                        } break;
                        case ROSDataType::PATH: {
                            nav_msgs::Path* path = new nav_msgs::Path;
                            deserializeStringMessage<nav_msgs::Path>(navMap.serialized(), *path);
                            m_listener->onTopicReceived((void*)path);
                        } break;
                        default:
                        break;
                    }
                }
            } else {
                if (sub->takeNextData(m_listener->m_buffer, &m_info)) {
                    m_listener->onTopicReceived(m_listener->m_buffer);
                }
            }
        }
    };
    int m_matched;
    SampleInfo_t m_info;
    std::string m_type;
    std::string m_topic;
    std::shared_ptr<TopicListener> m_listener;
};

static std::map<std::string, std::shared_ptr<SubListener>> kSubListenerMap;
static std::map<unsigned short, Participant*> kSubscriberParticipantMap;
static std::map<unsigned short, std::vector<std::string>> kSubscriberParticipantTypeMap;

DDSSubscriber::DDSSubscriber(const std::string& topic, std::shared_ptr<eprosima::fastrtps::TopicDataType> type, std::shared_ptr<TopicListener> listener) {
    if (!type) {
        std::cout << "subscriber topic data type invalid\n";
        return;
    }
    m_type = type;
    Participant* participant = nullptr;
    auto iter = kSubscriberParticipantMap.find(0);
    if (iter == kSubscriberParticipantMap.end()) {
        ParticipantAttributes PParam;
        PParam.rtps.builtin.discovery_config.discoveryProtocol = DiscoveryProtocol_t::SIMPLE;
        PParam.rtps.builtin.discovery_config.use_SIMPLE_EndpointDiscoveryProtocol = true;
        PParam.rtps.builtin.discovery_config.m_simpleEDP.use_PublicationReaderANDSubscriptionWriter = true;
        PParam.rtps.builtin.discovery_config.m_simpleEDP.use_PublicationWriterANDSubscriptionReader = true;
        PParam.rtps.builtin.discovery_config.leaseDuration = 10;
        PParam.rtps.builtin.discovery_config.leaseDuration_announcementperiod = Duration_t(5, 0);
        std::string participantName = "Participant_";
        participantName.append(std::to_string(getpid()));
        PParam.rtps.setName(participantName.c_str());
        PParam.rtps.useBuiltinTransports = false;
        auto tcp_descriptor = std::make_shared<TCPv4TransportDescriptor>();
        PParam.rtps.userTransports.push_back(tcp_descriptor);
        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
        PParam.rtps.userTransports.push_back(udp_transport);
        participant = Domain::createParticipant(PParam);
        kSubscriberParticipantMap.insert({0, participant});
        Domain::registerType(participant, m_type.get());
        kSubscriberParticipantTypeMap.insert({0,{m_type->getName()}});
    } else {
        participant = iter->second;
        auto type = kSubscriberParticipantTypeMap.find(0)->second;
        if (std::find(type.begin(), type.end(), m_type->getName()) == type.end() ) {
            type.push_back(m_type->getName());
            kSubscriberParticipantTypeMap[0] = type;
            Domain::registerType(participant, m_type.get());
        }
    }

    if (participant == nullptr) {
        std::cout << "participant create failed\n";
        return;
    }

    auto subListener = std::shared_ptr<SubListener>(new SubListener(topic, listener));
    subListener->m_type.assign(m_type->getName());
    kSubListenerMap.insert({topic, subListener});

    SubscriberAttributes Rparam;
    Rparam.topic.topicKind = NO_KEY;
    Rparam.topic.topicDataType = m_type->getName();
    Rparam.topic.topicName = topic;
    Rparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    Rparam.topic.historyQos.depth = 18000;
    Rparam.topic.resourceLimitsQos.max_samples = 20000;
    Rparam.topic.resourceLimitsQos.allocated_samples = 20;
    Rparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    Rparam.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
    Rparam.qos.m_lifespan.duration = 60;
    m_subscriber = Domain::createSubscriber(participant, Rparam, (SubscriberListener*)(subListener.get()));

    if (m_subscriber == nullptr) {
        std::cout << "subscriber create failed\n";
        return;
    }
}

DDSSubscriber::DDSSubscriber(const std::string& ip, unsigned short port, const std::string& topic, std::shared_ptr<eprosima::fastrtps::TopicDataType> type,       std::shared_ptr<TopicListener> listener) {
    if (!type) {
        std::cout << "subscriber topic data type invalid\n";
        return;
    }
    m_type = type;

    Participant* participant = nullptr;
    auto iter = kSubscriberParticipantMap.find(port);
    if (iter == kSubscriberParticipantMap.end()) {
        ParticipantAttributes pparam;
        int32_t kind = LOCATOR_KIND_TCPv4;

        Locator_t initial_peer_locator;
        initial_peer_locator.kind = kind;

        std::shared_ptr<TCPv4TransportDescriptor> descriptor = std::make_shared<TCPv4TransportDescriptor>();

        if (!ip.empty()) {
            IPLocator::setIPv4(initial_peer_locator, ip);
            std::cout << "subscriber " << ip << ":" << port << std::endl;
        } else {
            IPLocator::setIPv4(initial_peer_locator, "127.0.0.1");
        }
        initial_peer_locator.port = port;
        pparam.rtps.builtin.initialPeersList.push_back(initial_peer_locator); // Publisher's meta channel
        pparam.rtps.builtin.discovery_config.leaseDuration = 8;
        pparam.rtps.builtin.discovery_config.leaseDuration_announcementperiod = Duration_t(5, 0);
        std::string participantName = "Participant_";
        participantName.append(std::to_string(getpid()));
        pparam.rtps.setName(participantName.c_str());
        pparam.rtps.useBuiltinTransports = false;
        pparam.rtps.userTransports.push_back(descriptor);
        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
        pparam.rtps.userTransports.push_back(udp_transport);

        participant = Domain::createParticipant(pparam);
        kSubscriberParticipantMap.insert({port, participant});
        Domain::registerType(participant, m_type.get());
        kSubscriberParticipantTypeMap.insert({port,{m_type->getName()}});
    } else {
        participant = iter->second;
        auto type = kSubscriberParticipantTypeMap.find(port)->second;
        if (std::find(type.begin(), type.end(), m_type->getName()) == type.end() ) {
            type.push_back(m_type->getName());
            kSubscriberParticipantTypeMap[port] = type;
            Domain::registerType(participant, m_type.get());
        }
    }

    if (participant == nullptr) {
        std::cout << "tcp participant create failed\n";
        return;
    }

    auto subListener = std::shared_ptr<SubListener>(new SubListener(topic, listener));
    subListener->m_type.assign(m_type->getName());
    kSubListenerMap.insert({topic, subListener});

    SubscriberAttributes rparam;
    rparam.topic.topicKind = NO_KEY;
    rparam.topic.topicDataType = m_type->getName();
    rparam.topic.topicName = topic;
    rparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    rparam.topic.historyQos.depth = 18000;
    rparam.topic.resourceLimitsQos.max_samples = 20000;
    rparam.topic.resourceLimitsQos.allocated_samples = 20;
    rparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    rparam.qos.m_durability.kind = TRANSIENT_LOCAL_DURABILITY_QOS;
    rparam.qos.m_lifespan.duration = 60;
    m_subscriber = Domain::createSubscriber(participant, rparam, (SubscriberListener*)(subListener.get()));
    if (m_subscriber == nullptr) {
        return;
    }
}

DDSSubscriber::~DDSSubscriber() {
    kSubListenerMap.erase(m_subscriber->getAttributes().topic.getTopicName().to_string());
    Domain::removeSubscriber(m_subscriber);
}
