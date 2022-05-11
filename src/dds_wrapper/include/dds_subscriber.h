
#ifndef DDSSUBSCRIBER_H_
#define DDSSUBSCRIBER_H_

#include <iostream>
#include <typeinfo>
#include <cxxabi.h>
#include <memory>

#include <fastrtps/TopicDataType.h>
#include <fastrtps/subscriber/Subscriber.h>
#include <fastrtps/participant/Participant.h>

class TopicListener {
public:
	TopicListener() = delete;
	TopicListener(void* data) {
		m_buffer = data;
	}
	~TopicListener() {
	};
	virtual void onTopicReceived(void* msg) {
		// please implement at children class
	};
	void* m_buffer;
};

class DDSSubscriber
{
public:
	DDSSubscriber() = delete;
	virtual ~DDSSubscriber();
	DDSSubscriber(const std::string& topic, std::shared_ptr<eprosima::fastrtps::TopicDataType> type, std::shared_ptr<TopicListener> listener);
	DDSSubscriber(const std::string& ip, unsigned short port, const std::string& topic, std::shared_ptr<eprosima::fastrtps::TopicDataType> type, std::shared_ptr<TopicListener> listener);

private:
	std::shared_ptr<eprosima::fastrtps::TopicDataType> m_type;
	eprosima::fastrtps::Subscriber* m_subscriber;
};


#endif /* DDSSUBSCRIBER_H_ */
