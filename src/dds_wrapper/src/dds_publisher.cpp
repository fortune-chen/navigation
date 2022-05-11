
#include <unistd.h>
#include <sys/types.h>

#include <fastrtps/Domain.h>
#include <fastrtps/utils/IPLocator.h>
#include <fastrtps/publisher/PublisherListener.h>
#include <fastrtps/attributes/PublisherAttributes.h>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/transport/TCPv4TransportDescriptor.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>

#include "dds_publisher.h"
#include "RosAdapterContentPubSubTypes.h"

using namespace ros::serialization;
using namespace eprosima::fastrtps;
using namespace eprosima::fastrtps::rtps;

class PubListener : public PublisherListener {
  public:
    PubListener(const std::string& topic) {
        m_matched = 0;
        m_topic = topic;
    };
    ~PubListener() {
    };
    void onPublicationMatched(Publisher* pub, MatchingInfo& info) override {
        if (info.status == MATCHED_MATCHING) { 
            m_matched++;
            std::cout << "Publisher matched: " << m_matched << " topic: " << m_topic <<std::endl;
        } else {
            m_matched--;
            std::cout << "Publisher unmatched:" << m_matched << " topic: " << m_topic << std::endl;
        }
    };
    int m_matched;
    std::string m_topic;
};

static std::map<std::string, std::shared_ptr<PubListener>> kPubListenerMap;
static std::map<unsigned short, Participant*> kPublisherParticipantMap;
static std::map<unsigned short, std::vector<std::string>> kPublisherParticipantTypeMap;

DDSPublisher::DDSPublisher(const std::string& topic, std::shared_ptr<eprosima::fastrtps::TopicDataType> type) {
    if (!type) {
        std::cout << "publisher topic data type invalid\n";
        return;
    }
    m_type = type;
    Participant* participant = nullptr;
    auto iter = kPublisherParticipantMap.find(0);
    if (iter == kPublisherParticipantMap.end()) {
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
        kPublisherParticipantMap.insert({0, participant});
        Domain::registerType(participant, m_type.get());
        kPublisherParticipantTypeMap.insert({0,{m_type->getName()}});
    } else {
        participant = iter->second;
        auto type = kPublisherParticipantTypeMap.find(0)->second;
        if (std::find(type.begin(), type.end(), m_type->getName()) == type.end() ) {
            type.push_back(m_type->getName());
            kPublisherParticipantTypeMap[0] = type;
            Domain::registerType(participant, m_type.get());
        }
    }

    if (participant == nullptr) {
        std::cout << "participant create failed\n";
        return;
    }

    auto listener = std::shared_ptr<PubListener>(new PubListener(topic));
    kPubListenerMap.insert({topic, listener});

    PublisherAttributes Wparam;
    Wparam.topic.topicKind = NO_KEY;
    Wparam.topic.topicDataType = m_type->getName();
    Wparam.topic.topicName = topic;
    Wparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    Wparam.topic.historyQos.depth = 18000;
    Wparam.topic.resourceLimitsQos.max_samples = 20000;
    Wparam.topic.resourceLimitsQos.allocated_samples = 20;
    Wparam.times.heartbeatPeriod.seconds = 2;
    Wparam.times.heartbeatPeriod.nanosec = 200 * 1000 * 1000;
    Wparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    Wparam.qos.m_lifespan.duration = 60;
    m_publisher = Domain::createPublisher(participant, Wparam, (PublisherListener*)(listener.get()));
    if (m_publisher == nullptr) {
        std::cout << "publisher create failed\n";
        return;
    }
}

DDSPublisher::DDSPublisher(const std::string& ip, unsigned short port, const std::string& topic, std::shared_ptr<eprosima::fastrtps::TopicDataType> type) {
    if (!type) {
        std::cout << "publisher topic data type invalid\n";
        return;
    }
    m_type = type;

    Participant* participant = nullptr;
    auto iter = kPublisherParticipantMap.find(port);
    if (iter == kPublisherParticipantMap.end()) {
        ParticipantAttributes pparam;
        pparam.rtps.builtin.discovery_config.leaseDuration = 8;
        pparam.rtps.builtin.discovery_config.leaseDuration_announcementperiod = Duration_t(5, 0);
        std::string participantName = "Participant_";
        participantName.append(std::to_string(getpid()));
        pparam.rtps.setName(participantName.c_str());

        pparam.rtps.useBuiltinTransports = false;

        std::shared_ptr<TCPv4TransportDescriptor> descriptor = std::make_shared<TCPv4TransportDescriptor>();

        descriptor->sendBufferSize = 0;
        descriptor->receiveBufferSize = 0;
        if (!ip.empty()) {
            descriptor->set_WAN_address(ip);
            std::cout << "publisher " << ip << ":" << port << std::endl;
        }
        descriptor->add_listener_port(port);
        pparam.rtps.userTransports.push_back(descriptor);

        auto udp_transport = std::make_shared<UDPv4TransportDescriptor>();
        pparam.rtps.userTransports.push_back(udp_transport);

        participant = Domain::createParticipant(pparam);
        kPublisherParticipantMap.insert({port, participant});
        Domain::registerType(participant, m_type.get());
        kPublisherParticipantTypeMap.insert({port,{m_type->getName()}});
    } else {
        participant = iter->second;
        auto type = kPublisherParticipantTypeMap.find(port)->second;
        if (std::find(type.begin(), type.end(), m_type->getName()) == type.end() ) {
            type.push_back(m_type->getName());
            kPublisherParticipantTypeMap[port] = type;
            Domain::registerType(participant, m_type.get());
        }
    }
    if (participant == nullptr) {
        std::cout << "tcp participant create failed\n";
        return;
    }

    auto listener = std::shared_ptr<PubListener>(new PubListener(topic));
    kPubListenerMap.insert({topic, listener});

    PublisherAttributes wparam;
    wparam.topic.topicKind = NO_KEY;
    wparam.topic.topicDataType = m_type->getName();
    wparam.topic.topicName = topic;
    wparam.topic.historyQos.kind = KEEP_LAST_HISTORY_QOS;
    wparam.topic.historyQos.depth = 18000;
    wparam.topic.resourceLimitsQos.max_samples = 20000;
    wparam.topic.resourceLimitsQos.allocated_samples = 20;
    wparam.times.heartbeatPeriod.seconds = 2;
    wparam.times.heartbeatPeriod.nanosec = 200 * 1000 * 1000;
    wparam.qos.m_reliability.kind = RELIABLE_RELIABILITY_QOS;
    wparam.qos.m_lifespan.duration = 60;
    m_publisher = Domain::createPublisher(participant, wparam, (PublisherListener*)(listener.get()));
    if (m_publisher == nullptr) {
        std::cout << "publisher tcp create failed\n";
        return;
    }
}
DDSPublisher::~DDSPublisher() {
    kPubListenerMap.erase(m_publisher->getAttributes().topic.getTopicName().to_string());
    Domain::removePublisher(m_publisher);
}
void DDSPublisher::write(void* data) {
    if (m_publisher) {
        m_publisher->write(data);
    }
}

void DDSPublisher::write(char type, std::string& data) {
    RosAdapterContent adapter;
    adapter.type(type);
    adapter.serialized().resize(data.size());
    adapter.serialized().assign(data.begin(),data.end());
    if (m_publisher) {
        m_publisher->write((void*)&adapter);
    }
}

void DDSPublisher::mapWrite(char type, std::string& data) {
    NavMapContent map;
    map.type(type);
    map.serialized().resize(data.size());
    map.serialized().assign(data.begin(),data.end());
    if (m_publisher) {
        m_publisher->write((void*)&map);
    }
}