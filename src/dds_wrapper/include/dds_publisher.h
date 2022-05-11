
#ifndef DDSPUBLISHER_H_
#define DDSPUBLISHER_H_

#include <iostream>
#include <typeinfo>
#include <cxxabi.h>
#include <memory>
#include <fastrtps/TopicDataType.h>
#include <fastrtps/publisher/Publisher.h>
#include <fastrtps/participant/Participant.h>

#include "ros/serialization.h"

#include "ros_data.h"

class DDSPublisher
{
public:
	DDSPublisher() = delete;
	virtual ~DDSPublisher();
	DDSPublisher(const std::string& topic, std::shared_ptr<eprosima::fastrtps::TopicDataType> type);
	DDSPublisher(const std::string& ip, unsigned short port, const std::string& topic, std::shared_ptr<eprosima::fastrtps::TopicDataType> type);
template<typename T>
	void publish(T e) {
		int status;
		if (*(abi::__cxa_demangle(typeid(e).name(), 0, 0, &status)) != *(m_type->getName())) {
			std::cout << "publish topic not matched\n";
			return;
		}
		write((void*)&e);
	}
template<typename R>
	void publishRos(R e) {
		if (*(m_rosContent.c_str()) == *(m_type->getName())) {
			std::string content;
			char type = (char)MessageTraits<R>::type;
			auto m = ros::serialization::serializeMessage(e);
			// TODO
			if (m.num_bytes > 40960) {
				printf("\n\n[error] dds pulish size limit, %d > 40960\n\n", (int)m.num_bytes);
				assert(false);
			}
			content.assign(reinterpret_cast<char*>(m.message_start), m.num_bytes);
			write(type, content);
			return;
		}
	}
template<typename R>
	void publishMap(R e) {
		if (*(m_mapContent.c_str()) == *(m_type->getName())) {
			std::string content;
			char type = (char)MessageTraits<R>::type;
			auto m = ros::serialization::serializeMessage(e);
			// TODO
			if (m.num_bytes > 16777216) {
				printf("\n\n[error] dds pulish size limit, %d > 16777216\n\n", (int)m.num_bytes);
				assert(false);
			}
			content.assign(reinterpret_cast<char*>(m.message_start), m.num_bytes);
			mapWrite(type, content);
			return;
		}
	}

private:
	void write(void* data);
	void write(char type, std::string& data);
	void mapWrite(char type, std::string& data);
	std::string m_rosContent = "RosAdapterContent";
	std::string m_mapContent = "NavMapContent";
	std::shared_ptr<eprosima::fastrtps::TopicDataType> m_type;
	eprosima::fastrtps::Publisher* m_publisher;
};
#endif /* DDSPUBLISHER_H_ */
